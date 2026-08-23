#!/usr/bin/env python3
# encoding: utf-8
"""
3-DOF serial leg (coxa-yaw, femur-pitch, tibia-pitch) inverse & forward
kinematics. Standard law-of-cosines planar 2-link solution for the
femur/tibia pair, plus atan2 for the coxa yaw. This is independent,
textbook robotics math (see e.g. Craig, "Introduction to Robotics") -- it
does not depend on and is not derived from the closed-source RoSpider
kinematics.so.

Angle convention (all radians):
    theta1 (coxa):  yaw about the vertical axis. 0 = pointing straight out
                     along the leg's local +x (outward) direction.
    theta2 (femur):  pitch from horizontal. 0 = femur horizontal,
                     positive = femur tip rises above the coxa joint.
    theta3 (tibia):  bend at the knee relative to the femur's direction.
                     0 = leg fully extended (femur+tibia form a straight
                     line), positive = knee folds the tibia upward/inward.

These are *kinematic* angles, not servo ticks. hexapod_control converts
them to ticks using per-joint direction/offset calibration, because the
servo horns were centered by hand and won't all agree with this
convention out of the box.
"""

import math


class Unreachable(Exception):
    pass


def leg_ik(x_local, y_local, z_local, l1, l2, l3, clamp=True):
    """
    Inverse kinematics for one leg.

    :param x_local, y_local, z_local: desired foot position in the leg's
        local frame, origin at the coxa joint, x = outward, y = sideways,
        z = up (so a foot below the body has z_local < 0).
    :param l1, l2, l3: coxa, femur, tibia link lengths (same units as xyz).
    :param clamp: if True, silently clamp unreachable targets to the
        nearest reachable point instead of raising.
    :return: (theta1, theta2, theta3) in radians.
    """
    theta1 = math.atan2(y_local, x_local)

    r_total = math.hypot(x_local, y_local)
    r = r_total - l1          # horizontal distance, femur joint -> foot
    h = z_local                # vertical distance, femur joint -> foot
    d = math.hypot(r, h)        # straight-line distance, femur joint -> foot

    d_min = abs(l2 - l3) + 1e-6
    d_max = l2 + l3 - 1e-6
    if d < d_min or d > d_max:
        if not clamp:
            raise Unreachable(f'distance {d:.1f}mm outside reachable range [{d_min:.1f}, {d_max:.1f}]')
        d = max(d_min, min(d_max, d))

    # interior knee angle (angle between femur and tibia segments)
    cos_knee = (l2 * l2 + l3 * l3 - d * d) / (2.0 * l2 * l3)
    cos_knee = max(-1.0, min(1.0, cos_knee))
    knee_interior = math.acos(cos_knee)
    theta3 = math.pi - knee_interior   # 0 = fully extended, + = folded

    # angle of the femur above horizontal needed to reach distance d
    cos_a = (l2 * l2 + d * d - l3 * l3) / (2.0 * l2 * d)
    cos_a = max(-1.0, min(1.0, cos_a))
    theta2 = math.atan2(h, r) + math.acos(cos_a)

    return theta1, theta2, theta3


def leg_fk(theta1, theta2, theta3, l1, l2, l3):
    """Forward kinematics: joint angles -> foot position in leg-local frame."""
    # femur tip position in the (r, h) plane relative to the femur joint
    r_femur = l2 * math.cos(theta2)
    h_femur = l2 * math.sin(theta2)

    # tibia points at (theta2 - theta3) from horizontal, since positive
    # theta3 folds the knee back relative to the femur's own direction
    tibia_dir = theta2 - theta3
    r_tibia = l3 * math.cos(tibia_dir)
    h_tibia = l3 * math.sin(tibia_dir)

    r = l1 + r_femur + r_tibia
    h = h_femur + h_tibia

    x_local = r * math.cos(theta1)
    y_local = r * math.sin(theta1)
    z_local = h
    return x_local, y_local, z_local


def body_to_local(foot_body_xyz, mount_xyz, mount_yaw):
    """Rotate/translate a foot position from body frame into leg-local frame."""
    fx, fy, fz = foot_body_xyz
    mx, my, mz = mount_xyz
    dx, dy, dz = fx - mx, fy - my, fz - mz
    c, s = math.cos(-mount_yaw), math.sin(-mount_yaw)
    x_local = dx * c - dy * s
    y_local = dx * s + dy * c
    z_local = dz
    return x_local, y_local, z_local


def local_to_body(foot_local_xyz, mount_xyz, mount_yaw):
    """Inverse of body_to_local."""
    x_local, y_local, z_local = foot_local_xyz
    mx, my, mz = mount_xyz
    c, s = math.cos(mount_yaw), math.sin(mount_yaw)
    dx = x_local * c - y_local * s
    dy = x_local * s + y_local * c
    return mx + dx, my + dy, mz + z_local
