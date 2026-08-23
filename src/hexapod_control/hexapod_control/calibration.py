#!/usr/bin/env python3
# encoding: utf-8
"""
Converts kinematic joint angles (as produced by hexapod_kinematics.ik) into
STS3215 servo tick positions (0..4095, center 2048 = 0 rad).

Every servo horn was centered by hand at 2048, so "angle 0 -> tick 2048"
is only true if the horn happened to land exactly on the kinematic zero
for that joint (coxa pointing straight outward, femur horizontal, tibia
fully extended). In practice it won't be exact, so each joint gets a
small `offset_rad` trim plus a `direction` (+1/-1) to fix mirrored legs,
both stored in config/servo_config.yaml and adjustable with the
calibration helper script.
"""
import math
import yaml

TICKS_PER_RAD = 4096.0 / (2.0 * math.pi)
CENTER_TICKS = 2048
MIN_TICKS = 0
MAX_TICKS = 4095


class JointCalibration:
    def __init__(self, direction=1, offset_rad=0.0, center_ticks=CENTER_TICKS):
        self.direction = direction
        self.offset_rad = offset_rad
        self.center_ticks = center_ticks

    def angle_to_ticks(self, angle_rad):
        real_angle = self.direction * (angle_rad + self.offset_rad)
        ticks = self.center_ticks + real_angle * TICKS_PER_RAD
        ticks = int(round(ticks))
        return max(MIN_TICKS, min(MAX_TICKS, ticks))

    def ticks_to_angle(self, ticks):
        real_angle = (ticks - self.center_ticks) / TICKS_PER_RAD
        return self.direction * real_angle - self.offset_rad


def load_calibration(yaml_path):
    """
    Loads config/servo_config.yaml. Expected structure:

        legs:
          LF:
            coxa:  {id: 1, direction: 1,  offset_deg: 0.0}
            femur: {id: 3, direction: 1,  offset_deg: 0.0}
            tibia: {id: 2, direction: 1,  offset_deg: 0.0}
          ...

    Returns: {leg_name: {joint_name: (servo_id, JointCalibration)}}
    """
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)

    result = {}
    for leg_name, joints in data['legs'].items():
        result[leg_name] = {}
        for joint_name, cfg in joints.items():
            calib = JointCalibration(
                direction=int(cfg.get('direction', 1)),
                offset_rad=math.radians(float(cfg.get('offset_deg', 0.0))),
                center_ticks=int(cfg.get('center_ticks', CENTER_TICKS)),
            )
            result[leg_name][joint_name] = (int(cfg['id']), calib)
    return result
