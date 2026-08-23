#!/usr/bin/env python3
# encoding: utf-8
"""
Gait engine for a 6-legged robot.

Two gaits, matching what the stock RoSpider firmware exposes conceptually
(tripod / wave), reimplemented from scratch as an open, understandable
phase-based trajectory generator:

  - "tripod": legs split into two groups of 3 ({LF,LR,RM} and {LM,RF,RR}),
    each group alternates swing/stance every half cycle. Fast, less stable.
  - "wave"  : legs step one at a time in sequence around the body
    (LR->LM->LF->RF->RM->RR style offsets), always keeping 5 feet down.
    Slow, very stable.

For each leg the engine produces a target foot position in BODY frame at
the current gait phase. Both the stance and swing trajectories are
shaped with a "smootherstep" easing function (zero velocity AND zero
acceleration at both endpoints), rather than raw linear/half-sine curves.
This matters a lot for a heavier robot: with a naive linear stance +
half-sine swing, the foot's horizontal velocity instantaneously reverses
direction at every touchdown/liftoff -- a velocity discontinuity that
translates directly into an impulsive jerk each step. More weight means
more momentum means a bigger, more destabilizing thump. Easing both
curves so velocity ramps smoothly to zero at each transition removes
that discontinuity entirely -- the foot arrives at touchdown already
slowing to a stop, and leaves liftoff already accelerating from a stop,
rather than snapping between two different speeds instantaneously.

velocity_x / velocity_y are in mm per gait cycle-equivalent (i.e. stride
length along body x / y), angular_z is the yaw the body rotates per full
cycle (radians). This intentionally mirrors the stride/direction/rotation
parameterization used by the original controller.move / step_controller
nodes, just with the actual math implemented here instead of hidden in a
compiled extension.
"""

import math

LEG_NAMES = ['LF', 'LM', 'LR', 'RR', 'RM', 'RF']

TRIPOD_GROUP_A = {'LF', 'LR', 'RM'}
TRIPOD_GROUP_B = {'LM', 'RF', 'RR'}

# Wave gait: each leg's swing window is offset by 1/6 of the cycle, in this order
WAVE_ORDER = ['RR', 'RM', 'RF', 'LF', 'LM', 'LR']


def smootherstep(t):
    """
    Ken Perlin's "smootherstep": 6t^5 - 15t^4 + 10t^3, clamped to [0,1].
    Zero first AND second derivative at both t=0 and t=1 -- used here to
    re-time every phase (both swing and stance) so foot velocity eases
    to zero at every transition instead of snapping discontinuously.
    """
    t = max(0.0, min(1.0, t))
    return t * t * t * (t * (t * 6.0 - 15.0) + 10.0)


def _swing_arc(phase01, lift_height):
    """
    phase01 in [0,1) across the swing portion of the cycle.
    Returns (forward_frac, up) where forward_frac in [-1,1] sweeps the
    foot from the back of the stride to the front, and up in [0, lift_height]
    is how high the foot lifts. Both curves are eased via smootherstep so
    velocity is zero at liftoff (phase01=0) and touchdown (phase01=1).
    """
    eased = smootherstep(phase01)
    forward_frac = -1.0 + 2.0 * eased
    # lift curve: still an arc shape (up then down), but built from the
    # eased parameter so its velocity also eases to zero at both ends,
    # not just at the geometric top of the arc
    up = lift_height * math.sin(math.pi * eased)
    return forward_frac, up


def _stance_frac(phase01):
    """
    phase01 in [0,1) across the stance portion -> forward_frac from 1 down
    to -1, eased so velocity ramps smoothly to zero at both the start
    (just after touchdown) and end (just before liftoff) of stance,
    matching the swing curve's eased endpoints for a continuous-velocity
    transition all the way around the cycle.
    """
    eased = smootherstep(phase01)
    return 1.0 - 2.0 * eased


def leg_phase_offset(gait, leg_name, duty_factor=None):
    """Fraction of a full cycle [0,1) at which this leg's swing begins."""
    if gait == 'tripod':
        return 0.0 if leg_name in TRIPOD_GROUP_A else 0.5
    elif gait == 'wave':
        idx = WAVE_ORDER.index(leg_name)
        return idx / 6.0
    else:
        raise ValueError(f'unknown gait {gait!r}')


def swing_duty_fraction(gait, duty_factor=None):
    """
    Fraction of the cycle each leg spends in swing (airborne). For tripod,
    this is normally 0.5 (evenly split between the two 3-leg groups), but
    can be reduced below 0.5 via duty_factor to shorten the swing portion
    and lengthen stance -- giving a longer period where both tripod groups
    overlap on the ground (more double/triple-support time), which trades
    some speed for a wider stability margin. Useful for a heavier robot
    where extra ground contact time reduces sway between weight transfers.
    """
    if gait == 'tripod':
        return 0.5 if duty_factor is None else max(0.05, min(0.5, duty_factor))
    elif gait == 'wave':
        return 1.0 / 6.0
    else:
        raise ValueError(f'unknown gait {gait!r}')


def foot_target(gait, leg_name, cycle_phase, stride_x, stride_y, yaw_stride,
                 lift_height, neutral_xyz, duty_factor=None):
    """
    Compute the target foot offset (relative to the leg's neutral stance
    point) for a given global cycle_phase in [0,1).

    stride_x, stride_y: total peak-to-peak stride length (mm) along body
        x/y for one full gait cycle.
    yaw_stride: total body yaw rotation (radians) contributed by this leg
        per full gait cycle (rotation is applied as a tangential
        component at the leg's neutral radius, added on top of stride_x/y).
    lift_height: how high the foot lifts during swing, mm.
    neutral_xyz: (x,y,z) neutral foot position in body frame, used to get
        the tangential direction for yaw.
    duty_factor: for tripod only, optionally shrink the swing fraction
        below the default 0.5 for more ground-contact overlap (stability).
        None uses the gait's default duty (0.5 for tripod, 1/6 for wave).
    """
    duty = swing_duty_fraction(gait, duty_factor)
    offset = leg_phase_offset(gait, leg_name)
    local_phase = (cycle_phase - offset) % 1.0

    nx, ny, _nz = neutral_xyz
    r = math.hypot(nx, ny)
    tangent_ang = math.atan2(ny, nx) + math.pi / 2.0
    tan_x, tan_y = math.cos(tangent_ang), math.sin(tangent_ang)

    if local_phase < duty:
        # swing phase
        swing_phase01 = local_phase / duty
        forward_frac, up = _swing_arc(swing_phase01, lift_height)
    else:
        # stance phase
        stance_phase01 = (local_phase - duty) / (1.0 - duty)
        forward_frac = _stance_frac(stance_phase01)
        up = 0.0

    dx = 0.5 * stride_x * forward_frac
    dy = 0.5 * stride_y * forward_frac
    dyaw_tangential = 0.5 * yaw_stride * r * forward_frac

    return (dx + dyaw_tangential * tan_x,
            dy + dyaw_tangential * tan_y,
            up)
