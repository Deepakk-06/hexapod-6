#!/usr/bin/env python3
# encoding: utf-8
"""
joint_jog

Command a single leg/joint to a specific kinematic angle (in degrees) so
you can calibrate direction/offset one joint at a time instead of trying
to interpret a full stand-up pose where 18 servos move at once.

Usage:
    ros2 run hexapod_control joint_jog <leg> <joint> <angle_deg> [--raw] --ros-args -p config_path:=<path>

    <leg>   : LF, LM, LR, RR, RM, RF
    <joint> : coxa, femur, tibia
    <angle_deg> : kinematic angle in degrees (see convention below)

    --raw : ignore servo_config.yaml calibration entirely and send using
            direction=+1, offset=0, center=2048 -- useful for figuring
            out a joint's true direction/offset from a blank slate.
            Without --raw, uses whatever is currently in the config file
            (so you can iteratively edit the yaml and re-test).

Kinematic angle convention (see hexapod_kinematics/ik.py docstring):
    coxa : 0 = pointing straight outward. Positive = rotates toward the
           leg's local +y (test with -20 then +20 and watch which way
           the leg swings at the shoulder).
    femur: 0 = horizontal. Positive = femur tip rises (leg lifts up).
           Test with -10 then +30 -- higher angle should visibly raise
           the leg.
    tibia: 0 = leg fully extended (straight line through femur+tibia).
           Positive = knee folds. Test with 10 then 60 -- higher angle
           should fold the lower leg up/inward, NOT hyperextend it.

Recommended procedure per joint:
    1. Run with --raw at two angles a good distance apart (e.g. -20 and
       +20 for coxa; -10 and +30 for femur; 10 and 60 for tibia).
    2. Watch which physical direction it moves.
    3. If it moved the WRONG way, set that joint's `direction` to -1 in
       servo_config.yaml (or back to 1 if it was already -1).
    4. Re-run WITHOUT --raw at the same two angles to confirm the config
       now gives the right direction.
    5. If direction is right but the resting position is off (e.g. tibia
       looks slightly folded at what should be angle 0), adjust
       `offset_deg` in small steps and re-test at angle 0.
"""
import sys
import math
import time

import rclpy
from rclpy.node import Node

from hexapod_msgs.msg import ServosPosition, ServoPosition
from hexapod_kinematics import geometry as geo
from hexapod_control.calibration import load_calibration, JointCalibration


class JointJogNode(Node):
    def __init__(self, leg, joint, angle_deg, raw, config_path):
        super().__init__('joint_jog')
        self.pub = self.create_publisher(ServosPosition, 'servo_controller', 10)

        if raw:
            servo_id = geo.SERVO_IDS[leg][joint]
            calib = JointCalibration(direction=1, offset_rad=0.0, center_ticks=2048)
            self.get_logger().info(f'[RAW] {leg}/{joint} (id {servo_id}): direction=1 offset=0 center=2048')
        else:
            if not config_path:
                self.get_logger().error('config_path is required unless --raw is given')
                raise SystemExit(1)
            calib_map = load_calibration(config_path)
            servo_id, calib = calib_map[leg][joint]
            self.get_logger().info(
                f'[CONFIG] {leg}/{joint} (id {servo_id}): '
                f'direction={calib.direction} offset_deg={math.degrees(calib.offset_rad):.1f} '
                f'center={calib.center_ticks}')

        angle_rad = math.radians(angle_deg)
        ticks = calib.angle_to_ticks(angle_rad)

        self.get_logger().info(f'Commanding angle={angle_deg:.1f}deg -> ticks={ticks}')

        sp = ServoPosition()
        sp.id = servo_id
        sp.position = float(ticks)
        msg = ServosPosition()
        msg.duration = 1.0
        msg.position = [sp]
        self.pub.publish(msg)


def main(args=None):
    argv = sys.argv[1:]
    if len(argv) < 3:
        print(__doc__)
        sys.exit(1)

    leg, joint, angle_str = argv[0], argv[1], argv[2]
    raw = '--raw' in argv
    angle_deg = float(angle_str)

    config_path = ''
    for i, a in enumerate(argv):
        if a == '-p' and i + 1 < len(argv) and argv[i + 1].startswith('config_path:='):
            config_path = argv[i + 1].split(':=', 1)[1]

    if leg not in geo.LEG_NAMES:
        print(f'unknown leg {leg!r}, must be one of {geo.LEG_NAMES}')
        sys.exit(1)
    if joint not in ('coxa', 'femur', 'tibia'):
        print(f'unknown joint {joint!r}, must be coxa/femur/tibia')
        sys.exit(1)

    rclpy.init(args=args)
    node = JointJogNode(leg, joint, angle_deg, raw, config_path)
    time.sleep(0.5)  # let publisher connect + message flush
    rclpy.spin_once(node, timeout_sec=0.5)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
