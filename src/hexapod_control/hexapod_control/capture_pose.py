#!/usr/bin/env python3
# encoding: utf-8
"""
capture_pose

Lets you hand-pose the robot (torque off) into your desired standing
stance, reads back the real tick position of every servo, and back-solves
the `offset_deg` calibration value each joint needs so that commanding
the kinematic neutral stance (via stand_up / gait_node) reproduces
EXACTLY the pose you just posed by hand.

This is a direct serial-port tool, NOT a ROS node -- it talks to the bus
itself, so it needs exclusive access to the port. Stop driver_node
first (Ctrl-C it) before running this, then restart driver_node
afterward.

Usage:
    python3 -m hexapod_control.capture_pose /dev/ttyUSB0 \\
        --config install/hexapod_control/share/hexapod_control/config/servo_config.yaml \\
        --stance-radius 150 --stance-height 90 \\
        --out /tmp/servo_config_captured.yaml

Steps it walks you through:
    1. Disables torque on all 18 servos (they go limp / free-spinning).
    2. Waits for you to hand-pose every leg into the stance you want.
    3. Reads back all 18 real tick positions.
    4. Re-enables torque immediately (locks the pose in place, no snap).
    5. Computes the offset_deg each joint needs, and writes a full new
       servo_config.yaml with those offsets filled in (direction, id,
       center_ticks are carried over unchanged from your current config).

Review the output file's offset_deg values before trusting them --
sanity check they're all small-ish (a few tens of degrees at most). A
huge offset on one joint usually means direction is still wrong for
that joint, or that joint's leg wasn't posed at the intended stance.
"""
import sys
import argparse
import math

import yaml

from hexapod_kinematics.ik import leg_ik
from hexapod_kinematics import geometry as geo
from hexapod_control.calibration import load_calibration, TICKS_PER_RAD
from sts3215_driver.scservo_protocol import SCServoBus


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('port')
    parser.add_argument('--config', required=True, help='current servo_config.yaml (for id/direction/center)')
    parser.add_argument('--stance-radius', type=float, default=geo.DEFAULT_STANCE_RADIUS)
    parser.add_argument('--stance-height', type=float, default=geo.DEFAULT_STANCE_HEIGHT)
    parser.add_argument('--out', required=True, help='where to write the new servo_config.yaml with computed offsets')
    args = parser.parse_args()

    calib = load_calibration(args.config)

    # kinematic angle for the intended neutral stance -- same for every leg
    # since it's expressed in each leg's own local frame
    t1, t2, t3 = leg_ik(args.stance_radius, 0.0, -args.stance_height,
                         geo.L1_COXA, geo.L2_FEMUR, geo.L3_TIBIA)
    neutral_angle = {'coxa': t1, 'femur': t2, 'tibia': t3}
    print(f'Intended neutral stance (radius={args.stance_radius}mm, height={args.stance_height}mm):')
    for j, a in neutral_angle.items():
        print(f'  {j}: {math.degrees(a):.1f} deg')

    bus = SCServoBus(port=args.port, baudrate=1000000)

    print('\nDisabling torque on all 18 servos -- they will go limp.')
    for leg in geo.LEG_NAMES:
        for joint in ('coxa', 'femur', 'tibia'):
            sid, _ = calib[leg][joint]
            bus.enable_torque(sid, False)

    input('\nHand-pose the robot into your desired standing stance now.\n'
          'Support its weight yourself -- torque is off, it will not hold itself up.\n'
          'Press Enter when you are happy with the pose ...')

    print('\nReading back real positions ...')
    measured_ticks = {}
    for leg in geo.LEG_NAMES:
        measured_ticks[leg] = {}
        for joint in ('coxa', 'femur', 'tibia'):
            sid, _ = calib[leg][joint]
            pos = bus.read_position(sid)
            measured_ticks[leg][joint] = pos
            print(f'  {leg}/{joint} (id {sid}): {pos}')

    print('\nRe-enabling torque to lock the pose in place ...')
    for leg in geo.LEG_NAMES:
        for joint in ('coxa', 'femur', 'tibia'):
            sid, jcalib = calib[leg][joint]
            ticks = measured_ticks[leg][joint]
            # command the servo to hold exactly the position it's already at
            bus.set_position(sid, ticks, time_ms=200, speed=0)

    bus.close()

    # back-solve offset_deg per joint:
    #   ticks = center + direction * (angle + offset) * TICKS_PER_RAD
    #   => offset = (ticks - center) / (direction * TICKS_PER_RAD) - angle
    out = {'legs': {}}
    print('\nComputed offsets:')
    for leg in geo.LEG_NAMES:
        out['legs'][leg] = {}
        for joint in ('coxa', 'femur', 'tibia'):
            sid, jcalib = calib[leg][joint]
            ticks = measured_ticks[leg][joint]
            angle = neutral_angle[joint]
            offset_rad = (ticks - jcalib.center_ticks) / (jcalib.direction * TICKS_PER_RAD) - angle
            offset_deg = math.degrees(offset_rad)
            print(f'  {leg}/{joint}: offset_deg = {offset_deg:.2f}'
                  + ('   <-- LARGE, double check direction/pose for this joint' if abs(offset_deg) > 40 else ''))
            out['legs'][leg][joint] = {
                'id': sid,
                'direction': jcalib.direction,
                'offset_deg': round(offset_deg, 2),
                'center_ticks': jcalib.center_ticks,
            }

    with open(args.out, 'w') as f:
        yaml.safe_dump(out, f, sort_keys=False, default_flow_style=None)
    print(f'\nWrote {args.out}')
    print('Review it, then copy it over your servo_config.yaml (or pass it directly as config_path)'
          ' and re-run stand_up with the SAME --stance-radius/--stance-height you used here.')


if __name__ == '__main__':
    main()
