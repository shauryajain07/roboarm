"""Scenario templates, data file, and compatibility with the OFFICIAL 05 script."""
import importlib.util
import os
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import official  # noqa: E402
import scenarios as sc  # noqa: E402
from poses import BLOCKS, CELLS, GRIPPER  # noqa: E402

SHARED = {"STOW": {1: 2048, 2: 3000, 3: 1100, 4: 3000, 5: 2048},
          "SAFE": {1: 2048, 2: 2600, 3: 1400, 4: 2100, 5: 2048},
          "RACKWAIT": {1: 2048, 2: 2300, 3: 1800, 4: 2000, 5: 2048}}
GRIPS = {"open": 1100, "cube": 2350, "cylinder": 2300, "trophy": 2250, "attack": 2500}


def full_data(path):
    d = sc.ScenarioData(path)
    d.shared, d.grips = dict(SHARED), dict(GRIPS)
    for n in sc.all_scenarios():
        for k, lbl in enumerate(sc.captures_needed(n)):
            d.captures.setdefault(n, {})[lbl] = {1: 1900 + k, 2: 2100, 3: 1700, 4: 2000, 5: 2048}
    return d


class Templates(unittest.TestCase):
    def test_41_scenarios_cover_every_case(self):
        names = sc.all_scenarios()
        self.assertEqual(len(names), 41)
        for b in BLOCKS:
            self.assertIn(f"pick_{b}", names)
            for c in CELLS:
                self.assertIn(f"place_{b}_{c}", names)
        for c in CELLS:
            self.assertIn(f"attack_{c}", names)

    def test_only_new_poses_need_capturing(self):
        self.assertEqual(sc.captures_needed("place_trophy_r1c3"), ["FRONT", "IN"])
        self.assertEqual(sc.captures_needed("attack_r2c2"), ["FRONT", "HIT"])
        self.assertEqual(sc.captures_needed("pick_cube"), ["ABOVE", "GRAB"])
        self.assertEqual(sc.captures_needed("start"), [])

    def test_place_holds_then_releases(self):
        d = full_data(Path(tempfile.mkdtemp()) / "d.json")
        pts = d.points("place_cylinder_r3c1")
        grips = [p[GRIPPER] for p in pts]
        self.assertEqual(grips, [2300] * 4 + [1100] * 4)
        self.assertEqual(pts[0], {**SHARED["SAFE"], 5: 2300})
        self.assertEqual(pts[-1], {**SHARED["SAFE"], 5: 1100})
        self.assertEqual({k: v for k, v in pts[3].items() if k != 5},
                         {k: v for k, v in pts[4].items() if k != 5}, "release happens in place")

    def test_chained_files_meet_at_safe(self):
        d = full_data(Path(tempfile.mkdtemp()) / "d.json")
        for b in BLOCKS:
            self.assertEqual(d.points(f"pick_{b}")[-1], d.points(f"place_{b}_r1c1")[0])

    def test_missing_reported(self):
        d = sc.ScenarioData(Path(tempfile.mkdtemp()) / "d.json")
        miss = d.missing("place_cube_r1c1")
        self.assertIn("shared SAFE", miss)
        self.assertIn("capture FRONT", miss)
        with self.assertRaises(ValueError):
            d.points("place_cube_r1c1")

    def test_only_attack_hit_is_max_speed(self):
        sp = sc.speeds("attack_r2c3")
        hit = [r.ref for r in sc.template("attack_r2c3")].index("HIT")
        self.assertEqual(sp[hit], "max")
        self.assertEqual(sp.count("max"), 1, "only FRONT -> HIT is fast")
        for n in ("pick_trophy", "place_cube_r1c1", "start", "stow"):
            self.assertNotIn("max", sc.speeds(n))

    def test_speeds_saved_and_reloaded(self):
        folder = Path(tempfile.mkdtemp())
        d = full_data(folder / "d.json")
        path = sc.save_scenario(d, "attack_r1c1", folder)
        self.assertEqual(sc.load_speeds(path), sc.speeds("attack_r1c1"))
        hit = [r.ref for r in sc.template("attack_r1c1")].index("HIT")
        self.assertEqual(sc.load_speeds(path, "attack_r1c1")[hit], "max")

    def test_hit_packet_uses_max_speed_and_acceleration(self):
        from test_all_cases import make_arm
        from motion import DEFAULT_CONFIG
        from poses import PoseStore
        arm, mock, _ = make_arm(PoseStore(str(Path(tempfile.mkdtemp()) / "p.json")))
        arm.move_to({1: 3000, 2: 2048}, speed=DEFAULT_CONFIG["max_speed"], acc=DEFAULT_CONFIG["max_acc"])
        pkt = [p for p in mock.log if p[4] == 0x83][-1]     # last SYNC WRITE
        # FF FF FE len 83 41 7 | id acc posL posH 0 0 spdL spdH | ...
        sid, acc, spd = pkt[7], pkt[8], pkt[13] | (pkt[14] << 8)
        self.assertEqual((sid, acc, spd), (1, 254, 3400))

    def test_data_roundtrip(self):
        p = Path(tempfile.mkdtemp()) / "d.json"
        full_data(p).save()
        d2 = sc.ScenarioData(p)
        self.assertEqual(d2.shared, SHARED)
        self.assertEqual(d2.points("attack_r1c1"), full_data(p).points("attack_r1c1"))


def load_official_05():
    repo = official.find_repo()
    if repo is None:
        return None
    spec = importlib.util.spec_from_file_location("wp05", repo / "scripts" / "05_waypoint_sequence.py")
    mod = importlib.util.module_from_spec(spec)
    try:
        spec.loader.exec_module(mod)
    except ImportError:
        return None
    return mod


class OfficialCompat(unittest.TestCase):
    def test_saved_files_load_in_official_05_script(self):
        wp = load_official_05()
        if wp is None:
            self.skipTest("official repo / pyserial not available")
        folder = Path(tempfile.mkdtemp())
        d = full_data(folder / "d.json")
        s = official.load_settings()
        for name in ("pick_trophy", "place_trophy_r1c3", "attack_r2c2", "start"):
            path = sc.save_scenario(d, name, folder, s)
            seq, pts = wp.load_sequence(path)           # the official loader
            self.assertEqual(seq["name"], name)
            ours = d.points(name)
            for o, p in zip(ours, pts):
                self.assertEqual([o[s.SERVO_IDS[n]] for n in s.JOINT_NAMES], [p[n] for n in s.JOINT_NAMES])
            self.assertEqual(sc.load_points(path), ours)

    def test_reads_official_angle_files(self):
        repo = official.find_repo()
        if repo is None:
            self.skipTest("official repo not available")
        pts = sc.load_points(repo / "motions" / "5.json")
        self.assertEqual(pts[0], {1: 2048, 2: 2048 + 1024, 3: 2048 - 1024, 4: 2048 + 1024,
                                  5: 2048 - 1024})   # 0 90 -90 90 90 with gripper direction -1

    def test_official_settings_drive_config(self):
        s = official.load_settings()
        if s is None:
            self.skipTest("official repo not available")
        o = official.config_overrides(s)
        self.assertEqual(o["baud"], 115200)
        self.assertEqual(o["gripper_close_direction"], 1)   # closing = smaller angle = bigger raw
        self.assertEqual(o["joint_limits"]["2"], [1024, 3072])  # shoulder -90..90


if __name__ == "__main__":
    unittest.main()
