"""Fluid motion: cleaning, waypoint curves, streaming playback, PREHIT punch, data cleaning."""
import importlib.util
import json
import os
import random
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import clean as cleaner  # noqa: E402
import official  # noqa: E402
import scenarios as sc  # noqa: E402
import trajectory as T  # noqa: E402
from motion import DEFAULT_CONFIG, EStop  # noqa: E402
from poses import GRIPPER, PoseStore  # noqa: E402
from test_all_cases import make_arm  # noqa: E402

START = {1: 2085, 2: 2520, 3: 2093, 4: 1377, 5: 1024}


def fake_recording(seed=3):
    """25 Hz hand demo: idle, move, pause with a grab, move back, idle, + shake + a glitch."""
    rnd = random.Random(seed)
    tr = T.Traj(meta={"source": "recorded"})
    t = 0.0

    def seg(dur, a, b, g0, g1):
        nonlocal t
        n = int(dur * 25)
        for k in range(n):
            u = k / max(1, n - 1)
            p = {j: START[j] + 6 * (rnd.random() - 0.5) for j in (1, 3, 4)}
            p[2] = a + (b - a) * u + 8 * (rnd.random() - 0.5)
            p[GRIPPER] = g0 + (g1 - g0) * min(1, u * 3)
            tr.t.append(t)
            tr.q.append(p)
            t += 0.04
    seg(1.0, 2520, 2520, 1024, 1024)      # idle at start
    seg(2.0, 2520, 2100, 1024, 1024)      # move down
    seg(1.5, 2100, 2100, 1024, 1393)      # pause + grab
    seg(2.0, 2100, 2520, 1393, 1393)      # back up
    seg(1.0, 2520, 2520, 1393, 1393)      # idle at end
    tr.q[60][2] += 400                    # glitchy read
    return tr


class Cleaning(unittest.TestCase):
    def test_clean_trims_smooths_and_snaps(self):
        raw = fake_recording()
        end = {**START, GRIPPER: 1393}
        tr, rep = T.clean(raw, START, end)
        self.assertGreater(rep["trimmed_s"], 1.4, "idle start/end trimmed")
        self.assertLess(tr.duration, raw.duration - 1.4)
        self.assertEqual(tr.rounded(0), START, "starts exactly at SAFE")
        self.assertEqual(tr.rounded(len(tr) - 1), end, "ends exactly at SAFE")
        self.assertLess(max(p[2] for p in tr.q), 2540, "glitch removed")
        jerk = lambda xs: max(abs(xs[i + 1] - 2 * xs[i] + xs[i - 1]) for i in range(1, len(xs) - 1))
        raw_rs = T._resample(raw, 0.02)
        self.assertLess(jerk([p[2] for p in tr.q]), jerk([p[2] for p in raw_rs.q]) / 3, "hand shake smoothed")
        self.assertEqual(rep["grip_events"], 1)

    def test_grab_not_squashed_by_pause_removal(self):
        tr, rep = T.clean(fake_recording(), START, None)
        g = [p[GRIPPER] for p in tr.q]
        i0 = next(i for i, v in enumerate(g) if v > 1030)
        i1 = next(i for i, v in enumerate(g) if v > 1390)
        self.assertGreater((i1 - i0) * 0.02, 0.15, "gripper close keeps its time")

    def test_too_short_rejected(self):
        tr = T.Traj([0, 0.04, 0.08], [dict(START)] * 3)
        with self.assertRaises(ValueError):
            T.clean(tr)


class Waypoints(unittest.TestCase):
    PTS = [{1: 2000, 2: 2500, 3: 2000, 4: 2000, 5: 1024},
           {1: 2000, 2: 2300, 3: 2100, 4: 2000, 5: 1024},
           {1: 2100, 2: 2200, 3: 2200, 4: 2000, 5: 1024},
           {1: 2100, 2: 2150, 3: 2250, 4: 2000, 5: 1393}]

    def test_passes_through_every_point(self):
        tr = T.from_waypoints(self.PTS, [True, False, False, True], [1100] * 4)
        ts = [0.0]
        for s in tr.meta["seg_T"]:
            ts.append(ts[-1] + s)
        for p, t in zip(self.PTS, ts):
            q = tr.at(t)
            for j in p:
                self.assertAlmostEqual(q[j], p[j], delta=1.5)

    def test_glides_through_non_stops(self):
        tr = T.from_waypoints(self.PTS, [True, False, False, True], [1100] * 4)
        t1 = tr.meta["seg_T"][0]
        v = (tr.at(t1 + 0.02)[2] - tr.at(t1 - 0.02)[2]) / 0.04
        self.assertGreater(abs(v), 100, "still moving at a pass-through point (no stop-and-go)")
        tr2 = T.from_waypoints(self.PTS, [True] * 4, [1100] * 4)
        t1b = tr2.meta["seg_T"][0]
        v2 = (tr2.at(t1b + 0.01)[2] - tr2.at(t1b - 0.01)[2]) / 0.02
        self.assertLess(abs(v2), abs(v) / 3, "comes to rest at a stop point")

    def test_peak_speed_respected(self):
        tr = T.from_waypoints(self.PTS, [True, False, False, True], [1100] * 4)
        self.assertLess(T.max_speed(tr), 1100 * 1.3)


class Prehit(unittest.TestCase):
    def test_punch_limited_to_200_ticks(self):
        front = {1: 2083, 2: 3070, 3: 1423, 4: 1568, 5: 1907}   # real attack_r1c2 data
        hit = {1: 2086, 2: 1954, 3: 2150, 4: 1982, 5: 1907}
        ph = sc.prehit(front, hit)
        d = max(abs(ph[j] - hit[j]) for j in (1, 2, 3, 4))
        self.assertLessEqual(d, sc.PUNCH_TICKS)
        self.assertGreater(d, sc.PUNCH_TICKS - 3)
        short = {**hit, 2: hit[2] + 50}
        self.assertEqual(sc.prehit(short, hit), short, "already short -> unchanged")

    def test_attack_template_order(self):
        refs = [r.ref for r in sc.template("attack_r2c2")]
        self.assertEqual(refs[2:5], ["FRONT", "PREHIT", "HIT"])
        self.assertEqual(sc.speeds("attack_r2c2")[4], "max")
        self.assertEqual(sc.captures_needed("attack_r2c2"), ["FRONT", "HIT"])


class Playback(unittest.TestCase):
    def test_streams_to_end_pose(self):
        arm, mock, _ = make_arm(PoseStore(str(Path(tempfile.mkdtemp()) / "p.json")), time_scale=3)
        pts = Waypoints.PTS
        tr = T.from_waypoints(pts, [True, False, False, True], [1100] * 4)
        t0 = time.monotonic()
        T.play(arm, tr, rate=2.0, ignore_gripper=False)
        took = time.monotonic() - t0
        for j in (1, 2, 3, 4):
            self.assertLessEqual(abs(mock.servos[j].pos - pts[-1][j]), 15)
        self.assertLess(took, tr.duration / 2 + 3.0)
        streamed = [p for p in mock.log if p[4] == 0x83]
        self.assertGreater(len(streamed), 10, "many small target updates = fluid")

    def test_estop_interrupts_stream(self):
        arm, mock, _ = make_arm(PoseStore(str(Path(tempfile.mkdtemp()) / "p.json")), time_scale=3)
        tr = T.from_waypoints(Waypoints.PTS, [True] * 4, [300] * 4)
        threading.Timer(0.3, arm.emergency_stop).start()
        with self.assertRaises(EStop):
            T.play(arm, tr)


class Files(unittest.TestCase):
    def test_roundtrip_and_official_04_loader(self):
        tr, rep = T.clean(fake_recording(), START, None)
        path = Path(tempfile.mkdtemp()) / "pick_cube.json"
        T.save(path, tr, "pick_cube", "test", extra={"report": rep})
        back = T.load(path)
        self.assertEqual(len(back), len(tr))
        self.assertEqual(back.rounded(5), tr.rounded(5))
        repo = official.find_repo()
        if repo is None:
            self.skipTest("official repo not available")
        spec = importlib.util.spec_from_file_location("ts04", repo / "scripts" / "04_timestamp_motion.py")
        mod = importlib.util.module_from_spec(spec)
        try:
            spec.loader.exec_module(mod)
        except ImportError:
            self.skipTest("pyserial not installed")
        motion, poses, interval = mod.load_motion(path)       # the OFFICIAL loader
        self.assertEqual(len(motion["steps"]), len(tr))
        self.assertAlmostEqual(interval, 0.02, places=3)


class DataCleaning(unittest.TestCase):
    TEAM = {  # the team's first push (subset) - real values
        "shared": {"STOW": {"1": 2223, "2": 2579, "3": 993, "4": 2289, "5": 1843},
                   "SAFE": {"1": 2085, "2": 2520, "3": 2093, "4": 1377, "5": 1038},
                   "RACKWAIT": {"1": 2043, "2": 2783, "3": 1286, "4": 1991, "5": 2223}},
        "grips": {"open": 643, "cube": 1393, "cylinder": 1387, "trophy": 1384, "attack": 2223},
        "captures": {
            "pick_cube": {"ABOVE": {"1": 1937, "2": 2323, "3": 1597, "4": 1372, "5": 1391},
                          "GRAB": {"1": 2081, "2": 2376, "3": 1464, "4": 1400, "5": 1474}},
            "pick_trophy": {"ABOVE": {"1": 1152, "2": 1910, "3": 2104, "4": 1178, "5": 1020},
                            "GRAB": {"1": 1154, "2": 1911, "3": 2105, "4": 1183, "5": 1310}},
            "attack_r1c3": {"FRONT": {"1": 1843, "2": 3133, "3": 1294, "4": 1640, "5": 1941},
                            "HIT": {"1": 1844, "2": 2028, "3": 2189, "4": 1851, "5": 1941}}}}

    def test_team_data_is_fixed(self):
        p = Path(tempfile.mkdtemp()) / "d.json"
        p.write_text(json.dumps(self.TEAM))
        d = sc.ScenarioData(p)
        cfg = dict(DEFAULT_CONFIG, joint_limits={"1": [0, 4095], "2": [1024, 3072], "3": [512, 3584],
                                                 "4": [512, 3584], "5": [1024, 3072]})
        fixes, warns = cleaner.clean_data(d, cfg)
        self.assertEqual(d.grips["open"], 1024)
        self.assertEqual(d.captures["attack_r1c3"]["FRONT"][2], 3072)
        tro = d.captures["pick_trophy"]
        self.assertGreaterEqual(cleaner.arm_dist(tro["ABOVE"], tro["GRAB"]), cleaner.NO_LIFT_TICKS)
        self.assertEqual(tro["ABOVE"][1], tro["GRAB"][1], "provisional lift goes up, not sideways")
        self.assertIn("pick_trophy/ABOVE", d.notes)
        self.assertTrue(any("pick_cube: base turns" in w for w in warns))
        self.assertTrue(any("attack_r1c3: FRONT->HIT" in w for w in warns))
        d.save()
        self.assertIn("pick_trophy/ABOVE", sc.ScenarioData(p).notes, "notes survive save/load")


if __name__ == "__main__":
    unittest.main()
