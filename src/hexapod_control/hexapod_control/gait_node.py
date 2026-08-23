#!/usr/bin/env python3
# encoding: utf-8
"""
hexapod_gait_node

Subscribes:
    cmd_vel                    (geometry_msgs/Twist)     -> walking command
    imu/roll_pitch_yaw         (geometry_msgs/Vector3Stamped) -> body leveling feedback

Publishes:
    servo_controller            (hexapod_msgs/ServosPosition) -> to sts3215_driver

Runs a fixed-rate control loop: advances the gait phase, asks the gait
engine for each leg's target foot offset, adds it to that leg's neutral
stance position, applies a small IMU-driven body-leveling rotation,
converts to leg-local coordinates, solves IK, converts to servo ticks via
per-joint calibration, and streams the result out.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from geometry_msgs.msg import Twist, Vector3Stamped

from hexapod_msgs.msg import ServosPosition, ServoPosition

from hexapod_kinematics import geometry as geo
from hexapod_kinematics import gait as gaitlib
from hexapod_kinematics.ik import leg_ik, body_to_local, local_to_body, Unreachable

from hexapod_control.calibration import load_calibration


def rotate_point_rp(p, roll, pitch):
    """Rotate point p=(x,y,z) by roll about x then pitch about y (small-angle body leveling)."""
    x, y, z = p
    # rotate about x (roll)
    y2 = y * math.cos(roll) - z * math.sin(roll)
    z2 = y * math.sin(roll) + z * math.cos(roll)
    x2 = x
    # rotate about y (pitch)
    x3 = x2 * math.cos(pitch) + z2 * math.sin(pitch)
    z3 = -x2 * math.sin(pitch) + z2 * math.cos(pitch)
    y3 = y2
    return x3, y3, z3


class HexapodGaitNode(Node):
    def __init__(self):
        super().__init__('hexapod_gait_node')

        self.declare_parameter('config_path', '')
        self.declare_parameter('gait', 'tripod')
        self.declare_parameter('step_period_s', 1.0)
        self.declare_parameter('lift_height_mm', 30.0)
        self.declare_parameter('max_stride_mm', 60.0)
        self.declare_parameter('max_yaw_stride_rad', 0.35)
        self.declare_parameter('stance_radius_mm', geo.DEFAULT_STANCE_RADIUS)
        self.declare_parameter('stance_height_mm', geo.DEFAULT_STANCE_HEIGHT)
        self.declare_parameter('control_rate_hz', 50.0)
        self.declare_parameter('leveling_enabled', True)
        self.declare_parameter('leveling_gain', 0.6)
        self.declare_parameter('leveling_max_rad', math.radians(15.0))
        self.declare_parameter('duty_factor', 0.5)  # tripod only; lower = more ground-contact overlap
        self.declare_parameter('accel_ramp_s', 0.4)  # time to ramp stride from 0 to full target --
                                                        # raise this for a heavier robot to soften
                                                        # start/stop/turn transitions further

        config_path = self.get_parameter('config_path').value
        if not config_path:
            self.get_logger().error('config_path parameter is required (path to servo_config.yaml)')
            raise SystemExit(1)
        self.calib = load_calibration(config_path)

        self.gait = self.get_parameter('gait').value
        self.period = float(self.get_parameter('step_period_s').value)
        self.lift_height = float(self.get_parameter('lift_height_mm').value)
        self.max_stride = float(self.get_parameter('max_stride_mm').value)
        self.max_yaw_stride = float(self.get_parameter('max_yaw_stride_rad').value)
        self.stance_radius = float(self.get_parameter('stance_radius_mm').value)
        self.stance_height = float(self.get_parameter('stance_height_mm').value)
        self.rate = float(self.get_parameter('control_rate_hz').value)
        self.leveling_enabled = bool(self.get_parameter('leveling_enabled').value)
        self.leveling_gain = float(self.get_parameter('leveling_gain').value)
        self.leveling_max = float(self.get_parameter('leveling_max_rad').value)
        self.duty_factor = float(self.get_parameter('duty_factor').value)
        self.accel_ramp_s = max(0.05, float(self.get_parameter('accel_ramp_s').value))

        # neutral (standing) foot position for each leg, in body frame
        self.neutral_body = {}
        for leg in geo.LEG_NAMES:
            mx, my, mz, myaw = geo.LEG_MOUNTS[leg]
            neutral_local = (self.stance_radius, 0.0, -self.stance_height)
            self.neutral_body[leg] = local_to_body(neutral_local, (mx, my, mz), myaw)

        self.cycle_phase = 0.0
        self.cmd_vx = 0.0   # mm/s equivalent (forward)
        self.cmd_vy = 0.0   # mm/s equivalent (strafe)
        self.cmd_wz = 0.0   # rad/s equivalent (turn)
        self.meas_roll = 0.0
        self.meas_pitch = 0.0

        # current (rate-limited) stride state -- ramps toward whatever cmd_vel
        # requests, rather than snapping instantly, so starts/stops/turns are
        # smooth. Phase advances continuously regardless of whether stride is
        # currently zero, so there's no separate "moving" state to manage.
        self.stride_x = 0.0
        self.stride_y = 0.0
        self.yaw_stride = 0.0

        self.cmd_sub = self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)
        self.imu_sub = self.create_subscription(
            Vector3Stamped, 'imu/roll_pitch_yaw', self.on_rpy, QoSPresetProfiles.SENSOR_DATA.value)

        self.servo_pub = self.create_publisher(ServosPosition, 'servo_controller', 10)

        self.dt = 1.0 / self.rate
        self.timer = self.create_timer(self.dt, self.control_step)

        self.get_logger().info(f'hexapod_gait_node ready (gait={self.gait}, period={self.period}s)')

    def on_cmd_vel(self, msg: Twist):
        self.cmd_vx = msg.linear.x
        self.cmd_vy = msg.linear.y
        self.cmd_wz = msg.angular.z

    def on_rpy(self, msg: Vector3Stamped):
        self.meas_roll = msg.vector.x
        self.meas_pitch = msg.vector.y

    @staticmethod
    def _ramp_toward(current, target, max_delta):
        """Move current toward target by at most max_delta -- clamps exactly
        to target rather than overshooting once within max_delta of it."""
        diff = target - current
        if diff > max_delta:
            return current + max_delta
        if diff < -max_delta:
            return current - max_delta
        return target

    def control_step(self):
        # target stride from the latest cmd_vel, in REAL units: linear.x/y in
        # m/s, angular.z in rad/s. stride length (mm) for one full gait cycle
        # = speed * period, converted m -> mm.
        target_stride_x = max(-self.max_stride, min(self.max_stride, self.cmd_vx * self.period * 1000.0))
        target_stride_y = max(-self.max_stride, min(self.max_stride, self.cmd_vy * self.period * 1000.0))
        target_yaw_stride = max(-self.max_yaw_stride, min(self.max_yaw_stride, self.cmd_wz * self.period))

        # rate-limit toward the target rather than snapping -- this is the
        # main fix for abrupt start/stop/turn jerk on a heavier robot.
        max_delta_linear = (self.max_stride / self.accel_ramp_s) * self.dt
        max_delta_yaw = (self.max_yaw_stride / self.accel_ramp_s) * self.dt
        self.stride_x = self._ramp_toward(self.stride_x, target_stride_x, max_delta_linear)
        self.stride_y = self._ramp_toward(self.stride_y, target_stride_y, max_delta_linear)
        self.yaw_stride = self._ramp_toward(self.yaw_stride, target_yaw_stride, max_delta_yaw)

        # phase always advances, whether or not stride is currently zero --
        # avoids a special-cased "moving" flag, and means a stop mid-stride
        # ramps stride to zero smoothly (via foot_target's own linear scaling
        # of dx/dy by stride) rather than teleporting the leg back to neutral.
        self.cycle_phase = (self.cycle_phase + self.dt / self.period) % 1.0

        # scale lift height by how much stride is actually currently commanded
        # (post-ramp) -- without this, a leg mid-swing when stride reaches
        # zero would still lift and set back down at the same spot ("marching
        # in place") instead of settling flat once actually stopped.
        activity = min(1.0, math.sqrt(
            (self.stride_x / self.max_stride) ** 2
            + (self.stride_y / self.max_stride) ** 2
            + (self.yaw_stride / self.max_yaw_stride) ** 2
        )) if self.max_stride > 0 and self.max_yaw_stride > 0 else 0.0
        effective_lift_height = self.lift_height * activity

        # leveling correction (small-angle, gain-limited)
        if self.leveling_enabled:
            corr_roll = max(-self.leveling_max, min(self.leveling_max, -self.leveling_gain * self.meas_roll))
            corr_pitch = max(-self.leveling_max, min(self.leveling_max, -self.leveling_gain * self.meas_pitch))
        else:
            corr_roll = corr_pitch = 0.0

        servo_positions = []
        for leg in geo.LEG_NAMES:
            mx, my, mz, myaw = geo.LEG_MOUNTS[leg]
            neutral = self.neutral_body[leg]

            dx, dy, up = gaitlib.foot_target(
                self.gait, leg, self.cycle_phase, self.stride_x, self.stride_y, self.yaw_stride,
                effective_lift_height, neutral, duty_factor=self.duty_factor)

            target_body = (neutral[0] + dx, neutral[1] + dy, neutral[2] + up)
            target_body = rotate_point_rp(target_body, corr_roll, corr_pitch)

            x_local, y_local, z_local = body_to_local(target_body, (mx, my, mz), myaw)

            try:
                t1, t2, t3 = leg_ik(x_local, y_local, z_local,
                                     geo.L1_COXA, geo.L2_FEMUR, geo.L3_TIBIA)
            except Unreachable:
                continue

            angles = {'coxa': t1, 'femur': t2, 'tibia': t3}
            for joint_name, angle in angles.items():
                servo_id, calib = self.calib[leg][joint_name]
                ticks = calib.angle_to_ticks(angle)
                sp = ServoPosition()
                sp.id = servo_id
                sp.position = float(ticks)
                servo_positions.append(sp)

        msg = ServosPosition()
        msg.duration = self.dt * 1.5  # slightly > loop period for smooth continuous interpolation
        msg.position = servo_positions
        self.servo_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = HexapodGaitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
