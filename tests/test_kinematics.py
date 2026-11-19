"""Gripper drive inverse kinematics - pure maths, no arm or Tk needed."""
import math
import os
import sys
import types
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import kinematics as k  # noqa: E402
import official  # noqa: E402

NAMES = ("base_yaw", "shoulder", "elbow", "wrist", "gripper")


def conv(zero=2048, directions=None):
    s = types.SimpleNamespace(
        JOINT_NAMES=NAMES, SERVO_IDS={n: i + 1 for i, n in enumerate(NAMES)},
        ZERO_TICKS=dict.fromkeys(NAMES, zero),
        DIRECTIONS=directions or {"base_yaw": 1, "shoulder": 1, "elbow": 1, "wrist": 1, "gripper": -1},
        ANGLE_LIMITS={"base_yaw": (-90, 90), "shoulder": (-90, 90), "elbow": (-135, 135),
                      "wrist": (-135, 135), "gripper": (-90, 90)})
    return official.Conv(s)


WIDE = {2: (0, 4095), 3: (0, 4095), 4: (0, 4095)}
POSES = [(41.5, 4.0, -59.0), (-3.2, 8.3, -4.7), (28.8, -51.3, -57.0), (64.6, -67.0, -5.0),
         (-12.0, 5.0, -76.0), (16.4, -14.9, -5.3)]      # recorded SAFE / HIT / GRAB / ... angles


# ---- an independent 3-D chain straight from the model file's numbers ------------------------
def _rodrigues(axis, q):
    x, y, z = axis
    n = math.sqrt(x * x + y * y + z * z)
    x, y, z = x / n, y / n, z / n
    c, s, t = math.cos(q), math.sin(q), 1 - math.cos(q)
    return [[c + x * x * t, x * y * t - z * s, x * z * t + y * s],
            [y * x * t + z * s, c + y * y * t, y * z * t - x * s],
            [z * x * t - y * s, z * y * t + x * s, c + z * z * t]]


def _mul(a, b):
    return [[sum(a[i][m] * b[m][j] for m in range(3)) for j in range(3)] for i in range(3)]


def _app(a, v):
    return [sum(a[i][m] * v[m] for m in range(3)) for i in range(3)]


def chain_3d(base_deg, sh, el, wr):
    """World tip position and pointing direction, MJCF-style (rotate about each joint axis)."""
    axis = (*k._AXIS, 0.0)
    q1, q2, q3 = k._model(sh, el, wr)
    r = _rodrigues((0, 0, -1), math.radians(base_deg))
    p = [0.0, 0.0, k._BASE_Z]
    for pos, q in ((k._SHOULDER_POS, q1), (k._UPPER_ARM, q2), (k._FOREARM, q3)):
        p = [a + b for a, b in zip(p, _app(r, pos))]
        r = _mul(r, _rodrigues(axis, q))
    p = [a + b for a, b in zip(p, _app(r, k._HAND))]
    w, x, y, z = k._TIP_QUAT
    zaxis = [2 * (x * z + y * w), 2 * (y * z - x * w), 1 - 2 * (x * x + y * y)]
    return p, _app(r, zaxis), _app(_rodrigues((0, 0, -1), math.radians(base_deg)), axis)


class Forward(unittest.TestCase):
    def test_matches_full_3d_chain_at_any_base_angle(self):
        for base in (-70, 0, 35):
            for sh, el, wr in POSES:
                p, d, axis = chain_3d(base, sh, el, wr)
                fwd = (-axis[1], axis[0])                       # horizontal, in the arm plane
                pose = k.forward(sh, el, wr)
                self.assertAlmostEqual(pose.height, p[2], places=9)
                self.assertAlmostEqual(pose.reach, p[0] * fwd[0] + p[1] * fwd[1], places=9)
                self.assertAlmostEqual(pose.pitch, math.degrees(
                    math.atan2(d[2], d[0] * fwd[0] + d[1] * fwd[1])), places=6)

    def test_tip_stays_in_the_plane_of_the_base_axis(self):
        # the sideways offsets of the links cancel: reach is the distance from the base axis
        for sh, el, wr in POSES:
            p, _, _ = chain_3d(0, sh, el, wr)
            self.assertAlmostEqual(math.hypot(p[0], p[1]), abs(k.forward(sh, el, wr).reach), places=3)

    def test_tilt_is_the_sum_of_the_joints(self):
        a, b = k.forward(10, 20, 30), k.forward(15, 10, 35)
        self.assertAlmostEqual(a.pitch, b.pitch, places=9)


class Inverse(unittest.TestCase):
    def test_round_trip(self):
        for q in POSES:
            for got, want in zip(k.inverse(k.forward(*q), near=q), q):
                self.assertAlmostEqual(got, want, places=7)

    def test_keeps_the_elbow_bend_it_is_near(self):
        q = (20.0, -40.0, -30.0)
        pose = k.forward(*q)
        found = {}
        for s in range(-180, 181, 20):
            for e in range(-180, 181, 20):
                sol = tuple((a + 180) % 360 - 180 for a in k.inverse(pose, near=(s, e, q[2])))
                found[round(sol[1], 6)] = sol
        self.assertEqual(len(found), 2)                              # elbow up and elbow down
        for sol in found.values():
            for a, b in zip(k.forward(*sol), pose):
                self.assertAlmostEqual(a, b, places=7)              # same tip pose either way
            for got, want in zip(k.inverse(pose, near=sol), sol):
                self.assertAlmostEqual(got, want, places=7)          # stays on the bend it's near

    def test_out_of_reach(self):
        self.assertIsNone(k.inverse(k.Pose(0.60, 0.20, 0.0), near=POSES[0]))


class Jog(unittest.TestCase):
    def setUp(self):
        self.c = conv()
        self.jog = k.TipJog(self.c)
        self.start = {j: self.c.ticks(j, a) for j, a in zip(k.IK_JOINTS, POSES[1])}   # attack HIT
        self.jog.start(self.start)

    def test_forward_is_a_straight_level_line(self):
        p0 = self.jog.goal
        prev = dict(self.start)
        for _ in range(40):                          # 4 cm forward in 1 mm steps
            raw = self.jog.step(0.001, 0, 0, WIDE, max_step=30)
            self.assertIsNotNone(raw, self.jog.note)
            self.assertLess(max(abs(raw[j] - prev[j]) for j in raw), 30)
            prev = raw
            p = self.jog.pose_of(raw)
            self.assertAlmostEqual(p.height, p0.height, places=6)
            self.assertAlmostEqual(p.pitch, p0.pitch, places=4)
        self.assertAlmostEqual(self.jog.goal.reach, p0.reach + 0.04, places=9)

    def test_up_and_tilt(self):
        p0 = self.jog.goal
        raw = self.jog.step(0, 0.01, 5.0, WIDE, max_step=500)
        p = self.jog.pose_of(raw)
        self.assertAlmostEqual(p.reach, p0.reach, places=5)
        self.assertAlmostEqual(p.height, p0.height + 0.01, places=5)
        self.assertAlmostEqual(p.pitch, p0.pitch + 5.0, places=3)

    def test_a_step_too_big_for_the_joints_is_shortened_not_bent(self):
        p0 = self.jog.goal
        raw = self.jog.step(0, 0.01, 5.0, WIDE, max_step=20)
        self.assertIsNotNone(raw, self.jog.note)
        self.assertLess(max(abs(raw[j] - self.start[j]) for j in raw), 30)
        p = self.jog.pose_of(raw)
        up, tilt = p.height - p0.height, p.pitch - p0.pitch
        self.assertTrue(0 < up < 0.01 and 0 < tilt < 5.0)
        self.assertAlmostEqual(tilt / up, 5.0 / 0.01, delta=1.0)     # same direction, just less
        self.assertAlmostEqual(p.reach, p0.reach, places=5)

    def test_edge_of_reach_holds_and_says_why(self):
        p0 = self.jog.goal
        out = None
        for _ in range(400):                         # push forward until the arm is straight
            out = self.jog.step(0.002, 0, 0, WIDE, max_step=30)
            if out is None:
                break
        self.assertIsNone(out)
        self.assertTrue(self.jog.note)
        self.assertGreater(self.jog.goal.reach, p0.reach)
        self.assertIsNone(self.jog.step(0.002, 0, 0, WIDE, max_step=30))   # still held
        self.assertIsNotNone(self.jog.step(-0.002, 0, 0, WIDE, max_step=30))  # back is fine

    def test_joint_limit_refuses_the_step(self):
        lim = dict(WIDE)
        lim[2] = (round(self.start[2]) - 1, round(self.start[2]) + 1)   # shoulder can't move
        goal = self.jog.goal
        self.assertIsNone(self.jog.step(0.01, 0, 0, lim, max_step=200))
        self.assertIn("shoulder", self.jog.note)
        self.assertEqual(self.jog.goal, goal)

    def test_reversed_servo_direction_and_zero(self):
        c = conv(zero=2100, directions={"base_yaw": 1, "shoulder": -1, "elbow": 1, "wrist": -1,
                                        "gripper": -1})
        jog = k.TipJog(c)
        jog.start({j: c.ticks(j, a) for j, a in zip(k.IK_JOINTS, POSES[0])})
        p0 = jog.goal
        raw = jog.step(0, -0.01, 0, WIDE, max_step=500)
        p = jog.pose_of(raw)
        self.assertAlmostEqual(p.height, p0.height - 0.01, places=6)
        self.assertAlmostEqual(p.reach, p0.reach, places=6)

    def test_conv_raw_is_rounded_ticks(self):
        for a in (-33.3, 0.0, 12.71):
            self.assertEqual(self.c.raw(3, a), round(self.c.ticks(3, a)))


if __name__ == "__main__":
    unittest.main()
