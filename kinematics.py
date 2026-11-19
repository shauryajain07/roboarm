"""Arm geometry and inverse kinematics for the Drive tab's optional gripper drive.

Shoulder, elbow and wrist all turn about parallel horizontal axes, so whatever the base angle,
the gripper tip moves in one vertical plane. In that plane a tip pose is

  reach   m forward from the base's turning axis (horizontal)
  height  m above the bottom of the base
  pitch   deg the gripper points above horizontal (0 = level, -90 = straight down)

and `inverse` turns a pose back into shoulder / elbow / wrist angles in closed form (no
iterations, so it is cheap at 40 Hz on the Pi). The base still turns on its own.

Geometry is the course's arm model: Kelvinchan324/robot-arm-simulation, dist/robot/engg1101.xml
(commit f4fff72). Joint angles are the official arm_settings.py angles (ZERO_TICKS /
DIRECTIONS). The course's Pi code treats those as the simulation's displayed angles, which
put shoulder +90 deg and wrist +90 deg on their upper stops and elbow -90 deg on its lower stop.
OFFSET_DEG converts between them. An arm with uncalibrated zeros still keeps its tilt exactly
(that only depends on the sum of the joint angles), but its straight lines bend a little.
"""
from __future__ import annotations

import math
from typing import NamedTuple

IK_JOINTS = (2, 3, 4)                    # shoulder, elbow, wrist (servo IDs)

# ---- model geometry (metres), copied from engg1101.xml ------------------------------------
_AXIS = (-0.9402479141, -0.3404906166)  # x, y of the shared hinge axis (z = 0: horizontal)
_FWD = (-_AXIS[1], _AXIS[0])              # z x axis: horizontal, in the arm's plane
_BASE_Z = 0.0497                          # yaw platform above the base
_SHOULDER_POS = (-0.001148084157, 0.05207040333, 0.0577)   # in the yaw platform
_UPPER_ARM = (-0.006135781789, 0.01841212569, 0.1189216373)  # shoulder -> elbow
_FOREARM = (0.03923147545, -0.1080420757, 0.1033874809)      # elbow -> wrist
_HAND = (0.005532242792, -0.06300226309, -0.1462409799)      # wrist -> gripper tip
_TIP_QUAT = (0.0339906847, -0.1694717566, 0.9657159589, -0.1936921368)   # w x y z
# model joint ranges (rad) that fix the course's displayed-angle convention
_SHOULDER_HI, _ELBOW_LO, _WRIST_HI = 1.240242507, -1.86479407, 3.607706376
OFFSET_DEG = (90 - math.degrees(_SHOULDER_HI), -90 - math.degrees(_ELBOW_LO),
              90 - math.degrees(_WRIST_HI))   # arm_settings angle - model angle


def _plane(v):
    """3-D vector in the yaw platform -> (forward, up) in the arm's plane."""
    return (v[0] * _FWD[0] + v[1] * _FWD[1], v[2])


def _tip_direction():
    w, x, y, z = _TIP_QUAT
    zaxis = (2 * (x * z + y * w), 2 * (y * z - x * w), 1 - 2 * (x * x + y * y))
    u, v = _plane(zaxis)
    return math.atan2(v, u)


_S = (_plane(_SHOULDER_POS)[0], _plane(_SHOULDER_POS)[1] + _BASE_Z)
_A, _B, _C = _plane(_UPPER_ARM), _plane(_FOREARM), _plane(_HAND)
_LA, _LB = math.hypot(*_A), math.hypot(*_B)
_TA, _TB = math.atan2(_A[1], _A[0]), math.atan2(_B[1], _B[0])
_TIP0 = _tip_direction()                  # gripper direction with every model angle at 0


class Pose(NamedTuple):
    reach: float
    height: float
    pitch: float


def _rot(v, q):
    c, s = math.cos(q), math.sin(q)
    return v[0] * c - v[1] * s, v[0] * s + v[1] * c


def _model(sh, el, wr):
    return tuple(math.radians(a - o) for a, o in zip((sh, el, wr), OFFSET_DEG))


def forward(sh: float, el: float, wr: float) -> Pose:
    """arm_settings angles (deg) of shoulder, elbow, wrist -> gripper tip pose."""
    q1, q2, q3 = _model(sh, el, wr)
    a, b, c = _rot(_A, q1), _rot(_B, q1 + q2), _rot(_C, q1 + q2 + q3)
    return Pose(_S[0] + a[0] + b[0] + c[0], _S[1] + a[1] + b[1] + c[1],
                _near(math.degrees(_TIP0 + q1 + q2 + q3), 0.0))


def _near(a, ref):
    return ref + (a - ref + 180) % 360 - 180


def inverse(pose: Pose, near: tuple[float, float, float]) -> tuple[float, float, float] | None:
    """Gripper tip pose -> (shoulder, elbow, wrist) arm_settings angles, deg.

    Of the two elbow bends that reach a pose, returns the one closest to `near` (normally
    where the arm is now), so the arm never flips over. None = out of reach."""
    total = math.radians(pose.pitch) - _TIP0          # q1 + q2 + q3
    c = _rot(_C, total)
    dx, dy = pose.reach - c[0] - _S[0], pose.height - c[1] - _S[1]   # shoulder -> wrist
    cos_bend = (dx * dx + dy * dy - _LA * _LA - _LB * _LB) / (2 * _LA * _LB)
    if not -1.0 <= cos_bend <= 1.0:
        return None
    best = None
    for bend in {math.acos(cos_bend), -math.acos(cos_bend)}:
        a = math.atan2(dy, dx) - math.atan2(_LB * math.sin(bend), _LA + _LB * math.cos(bend))
        q1 = a - _TA
        q2 = bend + _TA - _TB
        q = (q1, q2, total - q1 - q2)
        ang = tuple(_near(math.degrees(v) + o, r) for v, o, r in zip(q, OFFSET_DEG, near))
        cost = max(abs(x - r) for x, r in zip(ang, near))
        if best is None or cost < best[0]:
            best = (cost, ang)
    return best[1]


class TipJog:
    """Velocity jogging of the gripper tip, for the Drive tab.

    `conv` is official.Conv: angle(id, raw) -> deg and ticks(id, deg) -> unrounded raw.
    start() takes the joints' current raw targets; each step() moves the tip goal by a little
    and returns raw targets for joints 2-4, or None (and `note` says why) if that would leave
    the arm's reach or a joint limit, or need a jump at a nearly straight arm."""

    def __init__(self, conv):
        self.conv = conv
        self.goal: Pose | None = None
        self.q: tuple[float, float, float] | None = None
        self.note = ""

    def reset(self):
        self.goal = self.q = None

    def pose_of(self, raw: dict) -> Pose:
        return forward(*(self.conv.angle(j, raw[j]) for j in IK_JOINTS))

    def start(self, raw: dict):
        self.q = tuple(self.conv.angle(j, raw[j]) for j in IK_JOINTS)
        self.goal = forward(*self.q)
        self.note = ""

    def _solve(self, d_reach, d_height, d_pitch):
        g = self.goal
        cand = Pose(g.reach + d_reach, g.height + d_height, g.pitch + d_pitch)
        q = inverse(cand, self.q)
        if q is None:
            return cand, None, None
        return cand, q, {j: self.conv.ticks(j, a) for j, a in zip(IK_JOINTS, q)}

    def step(self, d_reach: float, d_height: float, d_pitch: float, limits: dict,
             max_step: float) -> dict[int, float] | None:
        now = {j: self.conv.ticks(j, a) for j, a in zip(IK_JOINTS, self.q)}
        cand, q, raw = self._solve(d_reach, d_height, d_pitch)
        if raw is not None:
            big = max(abs(raw[j] - now[j]) for j in IK_JOINTS)
            if big > max_step:                 # nearly straight arm: slow the tip down to fit
                f = max_step / big
                cand, q, raw = self._solve(d_reach * f, d_height * f, d_pitch * f)
                if raw is not None and max(abs(raw[j] - now[j]) for j in IK_JOINTS) > 1.5 * max_step:
                    self.note = "the arm is nearly straight - move back or down a little"
                    return None
        if raw is None:
            self.note = "the gripper can't reach any further that way"
            return None
        for j in IK_JOINTS:
            lo, hi = limits[j]
            if not lo <= raw[j] <= hi:
                self.note = f"the {('shoulder', 'elbow', 'wrist')[j - 2]} is at its limit"
                return None
        self.goal, self.q, self.note = cand, q, ""
        return raw
