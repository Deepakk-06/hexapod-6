#!/usr/bin/env python3
# encoding: utf-8
"""
Body & leg geometry for the hexapod, all lengths in millimeters.

Leg segment lengths were measured from the official RoSpider URDF
(rospider_description/urdf/base.urdf.xacro link origins) -- since this
build reuses the same outer shell/limbs, these carry over directly:

    coxa (L1):  45.0 mm   (coxa joint axis -> femur joint axis)
    femur (L2): 77.1 mm   (femur joint axis -> tibia joint axis)
    tibia (L3): 115.6 mm  (tibia joint axis -> foot tip)

Leg mount positions/yaw angles (relative to body center, body frame
x-forward / y-left / z-up) were likewise measured from the URDF.
"""

import math

L1_COXA = 45.0
L2_FEMUR = 77.1
L3_TIBIA = 115.6

# Leg naming: LF, LM, LR, RF, RM, RR (Left/Right, Front/Middle/Rear)
LEG_NAMES = ['LF', 'LM', 'LR', 'RR', 'RM', 'RF']

# (mount_x_mm, mount_y_mm, mount_z_mm, mount_yaw_rad) relative to body center.
# yaw = direction the coxa points outward in the body's rest pose (0 = +x/forward).
LEG_MOUNTS = {
    'LF': (104.63,  50.63, 16.55,  math.radians(45.0)),
    'LM': (11.33,   73.54, 16.55,  math.radians(90.0)),
    'LR': (-82.13,  50.98, 16.55,  math.radians(135.0)),
    'RF': (104.63, -50.62, 16.55,  math.radians(-45.0)),
    'RM': (11.33,  -73.53, 16.55,  math.radians(-90.0)),
    'RR': (-82.13, -50.98, 16.55,  math.radians(-135.0)),
}

# Default standing foot position, expressed in each leg's *local* frame
# (x_local = outward along the mount yaw direction, y_local = sideways,
# z_local = down-negative from the coxa joint). This is the neutral stance
# used as the center point that gaits oscillate around.
DEFAULT_STANCE_RADIUS = 130.0   # how far out from the coxa axis the foot sits, mm
DEFAULT_STANCE_HEIGHT = 90.0    # body height above the ground, mm (positive)

# Servo id map you confirmed (coxa = 3n+1, tibia = 3n+2, femur = 3n+3):
SERVO_IDS = {
    'LF': {'coxa': 1,  'femur': 3,  'tibia': 2},
    'LM': {'coxa': 4,  'femur': 6,  'tibia': 5},
    'LR': {'coxa': 7,  'femur': 9,  'tibia': 8},
    'RR': {'coxa': 10, 'femur': 12, 'tibia': 11},
    'RM': {'coxa': 13, 'femur': 15, 'tibia': 14},
    'RF': {'coxa': 16, 'femur': 18, 'tibia': 17},
}
