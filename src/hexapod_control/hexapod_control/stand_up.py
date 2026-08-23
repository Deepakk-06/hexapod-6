#!/usr/bin/env python3
# encoding: utf-8
"""
stand_up_node

Ramps every leg from wherever it currently is up to the neutral standing
pose over a few seconds, instead of snapping there instantly. Run this
once after powering on, before starting hexapod_gait_node, so the legs
don't slam into the neutral pose from whatever position they happened to
power on in.

Usage:
    ros2 run hexapod_control stand_up --ros-args -p config_path:=/path/to/servo_config.yaml
"""
import math
import time

import rclpy
from rclpy.node import Node

from hexapod_msgs.msg import ServosPosition, ServoPosition

from hexapod_kinematics import geometry as geo
from hexapod_kinematics.ik import leg_ik, local_to_body, body_to_local

from hexapod_control.calibration import load_calibration


class StandUpNode(Node):
    def __init__(self):
        super().__init__('stand_up_node')
        self.declare_parameter('config_path', '')
        self.declare_parameter('stance_radius_mm', geo.DEFAULT_STANCE_RADIUS)
        self.declare_parameter('stance_height_mm', geo.DEFAULT_STANCE_HEIGHT)
        self.declare_parameter('ramp_time_s', 4.0)

        config_path = self.get_parameter('config_path').value
        if not config_path:
            self.get_logger().error('config_path parameter is required')
            raise SystemExit(1)

        self.calib = load_calibration(config_path)
        self.stance_radius = float(self.get_parameter('stance_radius_mm').value)
        self.stance_height = float(self.get_parameter('stance_height_mm').value)
        self.ramp_time = float(self.get_parameter('ramp_time_s').value)

        self.pub = self.create_publisher(ServosPosition, 'servo_controller', 10)

    def run(self):
        time.sleep(0.5)  # let publisher connect

        # target: neutral stance, all legs
        targets = {}
        for leg in geo.LEG_NAMES:
            mx, my, mz, myaw = geo.LEG_MOUNTS[leg]
            neutral_local = (self.stance_radius, 0.0, -self.stance_height)
            target_body = local_to_body(neutral_local, (mx, my, mz), myaw)
            x_local, y_local, z_local = body_to_local(target_body, (mx, my, mz), myaw)
            t1, t2, t3 = leg_ik(x_local, y_local, z_local, geo.L1_COXA, geo.L2_FEMUR, geo.L3_TIBIA)
            targets[leg] = {'coxa': t1, 'femur': t2, 'tibia': t3}

        # single move, servo-side interpolation handles the ramp via goal time
        servo_positions = []
        for leg in geo.LEG_NAMES:
            for joint_name, angle in targets[leg].items():
                servo_id, calib = self.calib[leg][joint_name]
                ticks = calib.angle_to_ticks(angle)
                sp = ServoPosition()
                sp.id = servo_id
                sp.position = float(ticks)
                servo_positions.append(sp)

        msg = ServosPosition()
        msg.duration = self.ramp_time
        msg.position = servo_positions
        self.pub.publish(msg)
        self.get_logger().info(f'Standing up over {self.ramp_time:.1f}s ...')
        time.sleep(self.ramp_time + 0.5)
        self.get_logger().info('Done. Robot should now be in neutral stance.')


def main(args=None):
    rclpy.init(args=args)
    node = StandUpNode()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
