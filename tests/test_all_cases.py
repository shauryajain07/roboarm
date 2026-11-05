"""Run:  python3 -m unittest discover -s tests -v     (from the roboarm folder)"""
import builtins
import os
import random
import sys
import tempfile
import threading
import time
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from motion import DEFAULT_CONFIG, Arm, EStop, LimitError  # noqa: E402
from poses import BLOCKS, CELLS, GRIPPER, JOINT_IDS, PoseStore, checklist  # noqa: E402
from servo_bus import (build_ping_packet, build_sync_write_positions_packet,  # noqa: E402
                       build_write_position_packet, mock_bus, parse_status)
import strategy  # noqa: E402

GRIP_OPEN, GRIP_CLOSED = 1500, 2300


def fake_store(path):
    rnd = random.Random(1)
    s = PoseStore(path)
    for spec in checklist():
        p = {j: rnd.randint(800, 3300) for j in JOINT_IDS}
        p[GRIPPER] = GRIP_OPEN if spec.name == "gripper_open" else GRIP_CLOSED
        s.set(spec.name, p)
    s.save()
    return s


def make_arm(store, time_scale=400):
    bus, mock = mock_bus(time_scale=time_scale)
    cfg = dict(DEFAULT_CONFIG, joint_limits={str(j): [0, 4095] for j in JOINT_IDS})
    visited = []
    arm = Arm(bus, store, cfg, log=lambda m: visited.append(m.strip()))
    arm.torque(True)
    return arm, mock, visited


class Protocol(unittest.TestCase):
    def test_golden_bytes_match_course_web_tool(self):
        # from activelearning-web/tests/scservoProtocol.test.ts
        self.assertEqual(list(build_ping_packet(1)), [0xFF, 0xFF, 0x01, 0x02, 0x01, 0xFB])
        self.assertEqual(list(build_write_position_packet(3, 2050, 300, 20)),
                         [0xFF, 0xFF, 0x03, 0x0A, 0x03, 0x29, 0x14, 0x02, 0x08,
                          0x00, 0x00, 0x2C, 0x01, 0x7B])
        with self.assertRaises(ValueError):
            build_write_position_packet(3, 4096, 300, 20)

    def test_sync_write_layout(self):
        pkt = build_sync_write_positions_packet({1: (2048, 300, 20), 2: (100, 50, 5)})
        self.assertEqual(pkt[2], 0xFE)
        self.assertEqual(pkt[4], 0x83)
        self.assertEqual(pkt[3], (7 + 1) * 2 + 4)  # (L+1)*N + 4
        self.assertEqual(list(pkt[5:7]), [41, 7])

    def test_parser_resyncs_after_garbage_and_bad_checksum(self):
        good = bytes([0xFF, 0xFF, 0x01, 0x04, 0x00, 0x00, 0x08, 0xF2])
        bad = bytes([0xFF, 0xFF, 0x01, 0x04, 0x00, 0x00, 0x08, 0x00])
        pkt, rest = parse_status(bytearray(b"\x12\x34" + bad + good))
        self.assertEqual(pkt["params"], b"\x00\x08")
        self.assertEqual(rest, bytearray())

    def test_bus_read_write_roundtrip_on_mock(self):
        bus, mock = mock_bus()
        self.assertTrue(all(bus.ping(j) for j in JOINT_IDS))
        self.assertFalse(bus.ping(9))
        bus.set_torque(2, True)
        bus.sync_write_positions({2: (3000, 4000, 50)})
        time.sleep(0.05)
        self.assertEqual(bus.read_position(2), 3000)


class Checklist(unittest.TestCase):
    def test_every_case_has_poses(self):
        names = [s.name for s in checklist()]
        self.assertEqual(len(names), len(set(names)), "duplicate pose names")
        self.assertEqual(len(names), 4 + 3 * 2 + 9 * 2 + 3 * 9 * 2)  # 82
        for b in BLOCKS:
            self.assertIn(f"pick/{b}/grip", names)
            for c in CELLS:
                self.assertIn(f"place/{b}/{c}/in", names)
        for c in CELLS:
            self.assertIn(f"attack/{c}/hit", names)

    def test_seed_poses_are_recorded_before_use(self):
        seen = set()
        for s in checklist():
            if s.seed_from:
                self.assertIn(s.seed_from, seen, f"{s.name} seeds from later pose {s.seed_from}")
            seen.add(s.name)


class AllGameCases(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.mkdtemp()
        cls.store = fake_store(os.path.join(cls.tmp, "poses.json"))

    def _check_end(self, mock, pose):
        want = self.store.get(pose)
        for j in (1, 2, 3, 4):
            self.assertLessEqual(abs(mock.servos[j].pos - want[j]), 15)

    def test_place_every_block_in_every_cell(self):
        for b in BLOCKS:
            for c in CELLS:
                with self.subTest(block=b, cell=c):
                    self.assertTrue(self.store.can_place(b, c))
                    arm, mock, visited = make_arm(self.store)
                    mock.gripper_block_at = GRIP_CLOSED - 10   # block in the jaws
                    felt = arm.turn(b, c)
                    self.assertTrue(felt, "gripper should feel the block")
                    expected = ["-> safe", f"-> pick/{b}/above", f"-> pick/{b}/grip",
                                f"-> pick/{b}/above", "-> safe", "-> rack_safe",
                                f"-> place/{b}/{c}/front", f"-> place/{b}/{c}/in",
                                f"-> place/{b}/{c}/front", "-> rack_safe", "-> safe"]
                    self.assertEqual([v for v in visited if v.startswith("->")], expected)
                    self._check_end(mock, "safe")
                    self.assertLessEqual(abs(mock.servos[GRIPPER].pos - GRIP_OPEN), 15,
                                         "block released")

    def test_attack_every_cell(self):
        for c in CELLS:
            with self.subTest(cell=c):
                arm, mock, visited = make_arm(self.store)
                arm.attack(c, authorised=True)
                self.assertEqual([v for v in visited if v.startswith("->")],
                                 ["-> safe", "-> rack_safe", f"-> attack/{c}/front",
                                  f"-> attack/{c}/hit", f"-> attack/{c}/front",
                                  "-> rack_safe", "-> safe"])
                self._check_end(mock, "safe")

    def test_attack_refuses_without_referee(self):
        arm, _, _ = make_arm(self.store)
        with self.assertRaises(PermissionError):
            arm.attack("r1c1", authorised=False)

    def test_missing_grip_is_reported(self):
        arm, mock, _ = make_arm(self.store)
        mock.gripper_block_at = None   # empty jaws
        self.assertFalse(arm.pick("cube"))

    def test_estop_aborts_sequence_and_cuts_torque(self):
        arm, mock, _ = make_arm(self.store, time_scale=2)
        threading.Timer(0.15, arm.emergency_stop).start()
        with self.assertRaises(EStop):
            arm.attack("r2c2", authorised=True)
        self.assertTrue(all(not s.torque for s in mock.servos.values()))
        arm.reset_estop()
        self.assertTrue(all(s.torque for s in mock.servos.values()))

    def test_joint_limits_block_bad_targets(self):
        arm, _, _ = make_arm(self.store)
        arm.cfg["joint_limits"] = dict(arm.cfg["joint_limits"], **{"2": [1000, 3000]})
        with self.assertRaises(LimitError):
            arm.move_to({2: 3500})


class Recording(unittest.TestCase):
    def test_limp_move_capture_saves_pose(self):
        from record import Recorder
        tmp = tempfile.mkdtemp()
        store = PoseStore(os.path.join(tmp, "poses.json"))
        arm, mock, _ = make_arm(store)
        rec = Recorder(arm, store, arm.cfg, mock)
        spec = checklist()[0]                       # 'stow'
        inputs = iter(["l", ""])

        def fake_input(_=""):
            cmd = next(inputs)
            if cmd == "":                          # person moved the arm, then Enter
                mock.hand_move({1: 1111, 2: 2222, 3: 3333, 4: 1234})
            return cmd
        orig = builtins.input
        builtins.input = fake_input
        try:
            self.assertEqual(rec.prompt(spec), 1)
        finally:
            builtins.input = orig
        saved = PoseStore(store.path).get("stow")
        self.assertEqual([saved[j] for j in (1, 2, 3, 4)], [1111, 2222, 3333, 1234])
        self.assertTrue(all(mock.servos[j].torque for j in JOINT_IDS), "arm holds after capture")


class Strategy(unittest.TestCase):
    def test_win_then_block_then_centre(self):
        b = {c: None for c in CELLS}
        self.assertEqual(strategy.suggest_place(b)[0], "r2c2")
        b.update(r1c1=("us", "cube"), r1c2=("us", "cube"), r3c1=("them", "cube"), r3c2=("them", "cube"))
        self.assertEqual(strategy.suggest_place(b)[0], "r1c3")   # win beats block
        b["r1c3"] = ("them", "cube")
        self.assertEqual(strategy.suggest_place(b)[0], "r3c3")   # block their line

    def test_attack_breaks_threat(self):
        b = {c: None for c in CELLS}
        b.update(r2c1=("them", "cube"), r2c2=("them", "cube"), r1c1=("them", "trophy"))
        cell, why = strategy.suggest_attack(b)
        self.assertIn(cell, ("r2c1", "r2c2", "r1c1"))
        self.assertIn("2-in-a-line", why)


if __name__ == "__main__":
    unittest.main()
