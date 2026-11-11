"""Game controller mapping - runs without evdev or a real pad.

The SCUF NOMAD codes below are what Linux's generic HID driver reports for it, worked out from
its HID descriptor and a press-every-button test on the real pad.
"""
import math
import os
import sys
import types
import unittest
from collections import namedtuple
from unittest import mock

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import gamepad as g  # noqa: E402

AbsInfo = namedtuple("AbsInfo", "value min max fuzz flat resolution")
NOMAD = (0x1B1C, 0x3A01)
# Linux codes for each physical NOMAD control
N_A, N_B, N_X, N_Y, N_L1, N_R1, N_L2, N_R2, N_L3, N_R3 = range(304, 314)
N_MENU, N_VIEW, N_HOME = 174, 130, 172


def nomad_state():
    st = g.PadState()
    st.configure(g.ALT_AXES, g.PROFILES[NOMAD])
    st.ranges = {0: (-127, 127), 1: (-127, 127), 2: (-127, 127), 5: (-127, 127)}
    return st


class Nomad(unittest.TestCase):
    def test_face_buttons_by_position(self):
        st = nomad_state()
        for code, name in ((N_A, "a"), (N_B, "b"), (N_X, "x"), (N_Y, "y")):
            self.assertEqual(st.event(g.EV_KEY, code, 1), name)
            self.assertEqual(st.event(g.EV_KEY, code, 0), "-" + name)

    def test_bumpers_drive_the_gripper_and_triggers_do_not(self):
        st = nomad_state()
        st.event(g.EV_KEY, N_L2, 98)          # analog trigger: first value is just "pressed"
        st.event(g.EV_KEY, N_R2, 40)
        self.assertEqual(st.snapshot()[1], set())            # triggers are not L1/R1
        st.event(g.EV_KEY, N_L1, 1)
        st.event(g.EV_KEY, N_R1, 1)
        self.assertEqual(st.snapshot()[1], {"l1", "r1"})

    def test_triggers_are_l2_r2_and_ignore_autorepeat(self):
        st = nomad_state()
        st.event(g.EV_KEY, N_L2, 98)
        self.assertEqual(st.snapshot()[0]["l2"], 1.0)        # held L2 = precision
        st.event(g.EV_KEY, N_L2, 0)
        self.assertEqual(st.snapshot()[0]["l2"], 0.0)
        st.event(g.EV_KEY, N_R2, 2)           # Linux auto-repeat value: no release would follow
        self.assertEqual(st.snapshot()[0]["r2"], 0.0)

    def test_menu_view_home_and_stick_clicks(self):
        st = nomad_state()
        for code, name in ((N_MENU, "menu"), (N_VIEW, "view"), (N_HOME, "home"), (N_L3, "l3"), (N_R3, "r3")):
            self.assertEqual(st.event(g.EV_KEY, code, 1), name)
        self.assertEqual(st.snapshot()[0]["l2"], 0.0)        # L3 used to switch on precision

    def test_dpad_hat_is_ignored(self):
        # Linux reports only one of the NOMAD's four D-pad bits, as a hat stuck on "up";
        # "up" = RESET STOP, so it must never get through.
        st = nomad_state()
        self.assertIsNone(st.event(g.EV_ABS, 17, -1))
        self.assertIsNone(st.event(g.EV_ABS, 16, 1))
        self.assertEqual(st.snapshot()[1], set())

    def test_sticks_on_z_rz(self):
        st = nomad_state()
        for code, v in ((0, 127), (1, -127), (2, -127), (5, 127)):
            st.event(g.EV_ABS, code, v)
        a = st.snapshot()[0]
        self.assertEqual((a["lx"], a["ly"], a["rx"], a["ry"]), (1.0, 1.0, -1.0, -1.0))   # stick up = +1

    def test_input_counter(self):
        st = nomad_state()
        self.assertEqual(st.inputs, 0)
        st.event(g.EV_ABS, 0, 50)
        st.event(g.EV_KEY, N_A, 1)
        st.event(g.EV_KEY, 999, 1)            # unknown code: not counted
        self.assertEqual(st.inputs, 2)
        st.reset()
        self.assertEqual(st.inputs, 0)


class Stick(unittest.TestCase):
    def test_centre_and_dead_zone(self):
        self.assertEqual(g.stick(0, 0), (0.0, 0.0))
        self.assertEqual(g.stick(0.08, 0.05), (0.0, 0.0))

    def test_straight_push_is_one_joint(self):
        self.assertEqual(g.stick(1, 0), (1.0, 0.0))
        self.assertEqual(g.stick(0, -1), (0.0, -1.0))

    def test_near_straight_snaps_to_one_joint(self):
        vx, vy = g.stick(math.cos(math.radians(5)), math.sin(math.radians(5)))
        self.assertEqual(vy, 0.0)
        self.assertAlmostEqual(vx, 1.0, places=3)

    def test_diagonal_moves_both_joints(self):
        vx, vy = g.stick(math.cos(math.radians(45)), math.sin(math.radians(45)))
        self.assertAlmostEqual(vx, vy, places=6)
        self.assertGreater(vx, 0.65)                 # was 0.45 each with per-axis shaping
        vx, vy = g.stick(math.cos(math.radians(30)), -math.sin(math.radians(30)))
        self.assertGreater(-vy, 0.45)                # was 0.19 - "only one joint moves"
        self.assertAlmostEqual(vx / -vy, math.sqrt(3), places=3)   # same direction as the push

    def test_square_gate_corner_is_capped(self):
        vx, vy = g.stick(1, 1)                       # pads that report (1, 1) in the corner
        self.assertAlmostEqual(math.hypot(vx, vy), 1.0, places=6)

    def test_half_push_is_gentle(self):
        vx, vy = g.stick(0.35, 0.35)
        self.assertLess(math.hypot(vx, vy), 0.2)


class DefaultLayout(unittest.TestCase):
    def test_other_pads_unchanged(self):
        st = g.PadState()
        st.configure(g.AXES, None)
        self.assertEqual(st.event(g.EV_KEY, 310, 1), "l1")
        self.assertEqual(st.event(g.EV_KEY, 308, 1), "x")
        self.assertEqual(st.event(g.EV_ABS, 17, 1), "dpad_down")     # hat D-pad still works
        st.event(g.EV_KEY, 312, 1)
        self.assertEqual(st.snapshot()[0]["l2"], 1.0)


class FindDevice(unittest.TestCase):
    def _fake_evdev(self, vendor, product, abs_codes, keys):
        dev = mock.Mock()
        dev.info = types.SimpleNamespace(vendor=vendor, product=product)
        dev.capabilities.return_value = {
            g.EV_ABS: [(c, AbsInfo(0, -127, 127, 0, 0, 0)) for c in abs_codes], g.EV_KEY: list(keys)}

        def absinfo(code):
            if code not in abs_codes:
                raise OSError("no such axis")
            return AbsInfo(0, -127, 127, 0, 0, 0)
        dev.absinfo.side_effect = absinfo
        return types.SimpleNamespace(list_devices=lambda: ["/dev/input/event5"],
                                     InputDevice=lambda path: dev), dev

    def test_nomad_gets_its_profile(self):
        nomad_keys = list(range(304, 314)) + [N_MENU, N_VIEW, N_HOME]
        evdev, dev = self._fake_evdev(*NOMAD, [0, 1, 2, 5, 16, 17], nomad_keys)
        with mock.patch.dict(sys.modules, {"evdev": evdev}):
            d, layout, profile = g._find_device()
        self.assertIs(d, dev)
        self.assertEqual(profile["name"], "SCUF NOMAD")
        st = g.PadState()
        g._setup(st, d, layout, profile)
        self.assertEqual(st.map, g.ALT_AXES)
        self.assertEqual(st.ranges, {0: (-127, 127), 1: (-127, 127), 2: (-127, 127), 5: (-127, 127)})
        self.assertEqual(st.event(g.EV_KEY, N_X, 1), "x")

    def test_unknown_pad_gets_default_map(self):
        evdev, _ = self._fake_evdev(0x045E, 0x02EA, [0, 1, 2, 3, 4, 5, 16, 17], range(304, 319))
        with mock.patch.dict(sys.modules, {"evdev": evdev}):
            _, layout, profile = g._find_device()
        self.assertIsNone(profile)
        self.assertIs(layout, g.AXES)


if __name__ == "__main__":
    unittest.main()
