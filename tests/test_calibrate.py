"""calibrate.py: gripper fit, arm_settings.py editing, joint check, full run on the mock arm."""
import contextlib
import io
import json
import os
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import calibrate as C  # noqa: E402
import official  # noqa: E402
from servo_bus import mock_bus  # noqa: E402

# same layout as the official repo's arm_settings.py (defaults)
SETTINGS = '''"""Edit these values to match the robot arm and Waveshare controller."""
SERIAL_PORT = "/dev/ttyUSB0"
BAUD_RATE = 115200
JOINT_NAMES = ("base_yaw", "shoulder", "elbow", "wrist", "gripper")
SERVO_IDS = {
    "base_yaw": 1,
    "shoulder": 2,
    "elbow": 3,
    "wrist": 4,
    "gripper": 5,
}
TICKS_PER_REVOLUTION = 4096
ZERO_TICKS = {
    "base_yaw": 2048,
    "shoulder": 2048,
    "elbow": 2048,
    "wrist": 2048,
    "gripper": 2048,
}
DIRECTIONS = {
    "base_yaw": 1,
    "shoulder": 1,
    "elbow": 1,
    "wrist": 1,
    "gripper": -1,
}
# Approximate software safety limits relative to the configured middle position.
ANGLE_LIMITS = {
    "base_yaw": (-90.0, 90.0),
    "shoulder": (-90.0, 90.0),
    "elbow": (-135.0, 135.0),
    "wrist": (-135.0, 135.0),
    "gripper": (-90.0, 90.0),
}
MOVE_SPEED = 3000
'''


def load(text):
    ns = {}
    exec(compile(text, "arm_settings.py", "exec"), ns)  # noqa: S102 - our own fixture
    return ns


def hands(t):
    """ask() that 'moves the jaws by hand' at each calibration prompt, then answers yes."""
    def ask(prompt):
        if prompt.startswith("1/2"):
            t.hand_move({5: 905})
        elif prompt.startswith("2/2"):
            t.hand_move({5: 2380})
        return "y"
    return ask


class Fit(unittest.TestCase):
    def test_course_gripper_reads_course_angles(self):
        cal = C.fit_gripper(1104, 2367)              # a gripper already at the course's 83 / -28 deg
        self.assertEqual((cal["zero"], cal["direction"]), (2048, -1))
        self.assertAlmostEqual(cal["open_deg"], 83.0, delta=0.2)
        self.assertAlmostEqual(cal["closed_deg"], -28.0, delta=0.2)

    def test_open_limit_stays_short_of_the_stop_and_closed_reaches_touch(self):
        cal = C.fit_gripper(905, 2380)
        lo, hi = cal["raw_limits"]
        self.assertGreaterEqual(lo, 905 + 20)        # ~2 deg inside the open hard stop
        self.assertLessEqual(lo, 905 + 25)
        self.assertGreaterEqual(hi, 2380)            # can close until the jaws touch
        self.assertLessEqual(hi, 2382)
        self.assertEqual(cal["widened"], [])

    def test_recorded_values_stay_playable(self):
        cal = C.fit_gripper(905, 2380, [("grip 'attack'", 2400), ("SAFE", 1038)])
        self.assertEqual(cal["widened"], [("grip 'attack'", 2400)])
        self.assertGreaterEqual(cal["raw_limits"][1], 2400)

    def test_mirrored_gripper_keeps_closing_smaller(self):
        cal = C.fit_gripper(3000, 1700)
        self.assertEqual(cal["direction"], 1)
        self.assertLess(cal["closed_deg"], cal["open_deg"])

    def test_jaws_that_did_not_move_are_refused(self):
        with self.assertRaises(ValueError):
            C.fit_gripper(2000, 2050)


class Settings(unittest.TestCase):
    def test_patch_changes_only_the_gripper_lines(self):
        cal = C.fit_gripper(905, 2380)
        new = C.patch_settings(SETTINGS, "gripper", cal)
        ns = load(new)
        self.assertEqual(ns["ZERO_TICKS"]["gripper"], cal["zero"])
        self.assertEqual(ns["DIRECTIONS"]["gripper"], -1)
        self.assertEqual(ns["ANGLE_LIMITS"]["gripper"], cal["limits"])
        changed = [(a, b) for a, b in zip(SETTINGS.splitlines(), new.splitlines()) if a != b]
        self.assertEqual(len(changed), 2)            # zero + limits (direction unchanged)
        self.assertEqual(load(SETTINGS)["ZERO_TICKS"]["shoulder"], ns["ZERO_TICKS"]["shoulder"])

    def test_official_conversion_agrees(self):
        cal = C.fit_gripper(905, 2380)
        with tempfile.TemporaryDirectory() as d:
            Path(d, "arm_settings.py").write_text(C.patch_settings(SETTINGS, "gripper", cal))
            s = official.load_settings(Path(d))
            self.assertEqual(official.Conv(s).raw_limits()["5"], cal["raw_limits"])

    def test_real_official_file_if_present(self):
        repo = official.find_repo()
        if repo is None:
            self.skipTest("official repo not cloned here")
        ns = load(C.patch_settings((repo / "arm_settings.py").read_text(), "gripper", C.fit_gripper(905, 2380)))
        self.assertIn("gripper", ns["ZERO_TICKS"])


class Check(unittest.TestCase):
    def test_pose_on_a_limit_is_flagged(self):
        rec = {1: [("A", 2000)], 2: [("attack_r1c2/FRONT", 3070), ("B", 2500)], 3: [], 4: [], 5: []}
        warn = C.check_joints({"1": [1024, 3072], "2": [1024, 3072]}, rec)
        self.assertEqual(len(warn), 1)
        self.assertIn("2 shoulder", warn[0])
        self.assertIn("3070", warn[0])


class FullRun(unittest.TestCase):
    """main() end to end on the mock arm, with temp copies of every file it touches."""

    def _files(self, d):
        (d / "arm_settings.py").write_text(SETTINGS)
        (d / "arm_config.json").write_text(json.dumps({"joint_limits": {str(j): [0, 4095] for j in range(1, 6)}}))
        (d / "scenario_data.json").write_text(json.dumps({
            "shared": {"SAFE": {"1": 2085, "2": 2520, "3": 2093, "4": 1377, "5": 1038}},
            "grips": {"open": 1024, "cube": 1393, "attack": 2223}, "captures": {}, "notes": {}}))
        return ["--data", str(d / "scenario_data.json"), "--config", str(d / "arm_config.json"),
                "--poses", str(d / "none.json"), "--backups", str(d / "backups")]

    def test_calibrate_and_save(self):
        with tempfile.TemporaryDirectory() as d:
            d = Path(d)
            args = self._files(d)
            bus, t = mock_bus(time_scale=50)
            out = io.StringIO()
            with mock.patch.dict(os.environ, {"ROBOARM_OFFICIAL": str(d)}), contextlib.redirect_stdout(out):
                rc = C.main(args, bus=bus, transport=t, ask=hands(t))
                from motion import load_config
                limits = load_config(str(d / "arm_config.json"))["joint_limits"]["5"]
            self.assertEqual(rc, 0, out.getvalue())
            cal = C.fit_gripper(905, 2380)
            self.assertEqual(load((d / "arm_settings.py").read_text())["ZERO_TICKS"]["gripper"], cal["zero"])
            self.assertNotIn("5", json.loads((d / "arm_config.json").read_text())["joint_limits"])
            self.assertEqual(limits, cal["raw_limits"])              # roboarm now uses the calibration
            self.assertEqual(json.loads((d / "scenario_data.json").read_text())["grips"]["attack"], 2380)
            self.assertEqual(len(list((d / "backups").iterdir())), 3)  # settings, config, data
            self.assertTrue(t.servos[5].torque)                       # gripper powered again

    def test_answering_no_changes_nothing(self):
        with tempfile.TemporaryDirectory() as d:
            d = Path(d)
            args = self._files(d)
            before = {p.name: p.read_text() for p in d.iterdir()}
            bus, t = mock_bus(time_scale=50)
            yes = hands(t)

            def ask(prompt):
                yes(prompt)
                return "n"
            with mock.patch.dict(os.environ, {"ROBOARM_OFFICIAL": str(d)}), \
                    contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(C.main(args, bus=bus, transport=t, ask=ask), 0)
            self.assertEqual({p.name: p.read_text() for p in d.iterdir()}, before)

    def test_check_only_needs_no_arm(self):
        with tempfile.TemporaryDirectory() as d:
            d = Path(d)
            args = self._files(d)
            out = io.StringIO()
            with mock.patch.dict(os.environ, {"ROBOARM_OFFICIAL": str(d)}), contextlib.redirect_stdout(out):
                self.assertEqual(C.main(args + ["--check"]), 0)
            self.assertIn("overrides the limits", out.getvalue())


if __name__ == "__main__":
    unittest.main()
