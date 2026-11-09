"""Game controller (SCUF / PlayStation / Xbox) for the Drive tab - Raspberry Pi, via evdev.

Plug the controller in by USB-C (or pair it over Bluetooth) and it is picked up
automatically; unplug it and every stick reads zero, so the arm stops.

  left stick   ↔ base      ↕ shoulder
  right stick  ↔ wrist     ↕ elbow
  hold R1      close the gripper step by step (let go = stop; won't crush past contact)
  hold L1      open the gripper step by step
  hold L2      precision (¼ speed)
  A B X Y      go to that preset pose          hold View + A/B/X/Y = save the arm here
  D-pad ↓      E-STOP                          D-pad ↑ = RESET STOP
  Menu (≡)     E-STOP too                      tap View = RESET STOP too
  L3 (click)   HOLD here                       R3 (click) = CAPTURE the selected pose (tab 2)
Face buttons are named by POSITION (Xbox layout: A bottom, B right, X left, Y top).

Handles the common Linux layouts: right stick on RX/RY or Z/RZ, triggers on Z/RZ,
GAS/BRAKE or as plain buttons.

Needs:  sudo apt install python3-evdev   (the Pi user must be in the 'input' group - it is by default)
"""
from __future__ import annotations

import threading
import time

# Linux input event codes (numbers, so the mapping can be tested without evdev installed)
EV_KEY, EV_ABS = 1, 3
AXES = {0: "lx", 1: "ly", 3: "rx", 4: "ry", 2: "l2", 5: "r2", 9: "r2", 10: "l2"}   # standard layout
ALT_AXES = {0: "lx", 1: "ly", 2: "rx", 5: "ry", 9: "r2", 10: "l2"}              # right stick on Z/RZ
DIGITAL_TRIGGERS = {312: "l2", 313: "r2"}
INVERT = {"ly", "ry"}                     # stick up = +1
TRIGGERS = {"l2", "r2"}
BUTTONS = {304: "a", 305: "b", 307: "y", 308: "x", 310: "l1", 311: "r1",     # face buttons by position
           314: "view", 315: "menu", 316: "home", 317: "l3", 318: "r3",
           544: "dpad_up", 545: "dpad_down", 546: "dpad_left", 547: "dpad_right"}  # D-pad as buttons
HATS = {16: ("dpad_left", "dpad_right"), 17: ("dpad_up", "dpad_down")}         # D-pad as a hat axis


class PadState:
    """Latest stick / trigger / button state, normalised: sticks -1..1, triggers 0..1."""

    def __init__(self):
        self.lock = threading.Lock()
        self.axes = {a: 0.0 for a in AXES.values()}
        self.held: set[str] = set()
        self.ranges: dict[int, tuple[int, int]] = {}
        self.map = dict(AXES)
        self.hat: dict[int, int] = {}

    def reset(self):
        with self.lock:
            self.axes = {a: 0.0 for a in AXES.values()}
            self.held = set()

    def snapshot(self):
        with self.lock:
            return dict(self.axes), set(self.held)

    def event(self, etype: int, code: int, value: int):
        """Apply one input event. Returns 'name' on a button press, '-name' on a release."""
        if etype == EV_ABS and code in HATS:          # D-pad: -1 / 0 / +1
            prev, self.hat[code] = self.hat.get(code, 0), value
            neg, pos = HATS[code]
            if value and value != prev:
                name = neg if value < 0 else pos
                with self.lock:
                    self.held.add(name)
                return name
            if not value and prev:
                name = neg if prev < 0 else pos
                with self.lock:
                    self.held.discard(name)
                return "-" + name
            return None
        if etype == EV_ABS and code in self.map:
            name = self.map[code]
            lo, hi = self.ranges.get(code, (0, 255))
            if name in TRIGGERS:
                v = (value - lo) / max(1, hi - lo)
            else:
                mid, half = (lo + hi) / 2, max(1, (hi - lo) / 2)
                v = (value - mid) / half
                if name in INVERT:
                    v = -v
            with self.lock:
                self.axes[name] = max(-1.0, min(1.0, v))
        elif etype == EV_KEY and code in DIGITAL_TRIGGERS:
            with self.lock:
                self.axes[DIGITAL_TRIGGERS[code]] = 1.0 if value else 0.0
        elif etype == EV_KEY and code in BUTTONS:
            name = BUTTONS[code]
            with self.lock:
                if value:
                    self.held.add(name)
                else:
                    self.held.discard(name)
            if value == 1:
                return name
            if value == 0:
                return "-" + name
        return None


def _find_device():
    import evdev  # noqa: PLC0415 - optional, Pi only
    for path in evdev.list_devices():
        try:
            d = evdev.InputDevice(path)
        except OSError:
            continue
        caps = d.capabilities()
        abs_codes = {c if isinstance(c, int) else c[0] for c in caps.get(EV_ABS, [])}
        keys = set(caps.get(EV_KEY, []))
        if {0, 1} <= abs_codes and keys & {304, 305} and ({3, 4} <= abs_codes or {2, 5} <= abs_codes):
            return d, (AXES if {3, 4} <= abs_codes else ALT_AXES)
        d.close()
    return None, None


class Gamepad:
    """Background reader. on_button(name) is called from this thread on each press;
    on_status(text) whenever the connection changes."""

    def __init__(self, on_button, on_status, on_release=None):
        self.state = PadState()
        self.on_button, self.on_status = on_button, on_status
        self.on_release = on_release or (lambda name: None)
        self.alive = True
        self.connected = False
        threading.Thread(target=self._run, daemon=True).start()

    def stop(self):
        self.alive = False

    def _run(self):
        try:
            import evdev  # noqa: F401, PLC0415
        except ImportError:
            self.on_status("🎮 controller support off - on the Pi run: sudo apt install python3-evdev")
            return
        last_msg = None
        while self.alive:
            dev = None
            try:
                dev, layout = _find_device()
            except Exception as e:  # noqa: BLE001
                msg = f"🎮 can't read controllers ({e}) - is the user in the 'input' group?"
                if msg != last_msg:
                    self.on_status(msg)
                    last_msg = msg
            if dev is None:
                if last_msg is None or "connected" in last_msg:
                    last_msg = "🎮 no controller - plug it in by USB or pair it over Bluetooth"
                    self.on_status(last_msg)
                time.sleep(2)
                continue
            self.state.map = dict(layout)
            for code in layout:
                try:
                    info = dev.absinfo(code)
                    self.state.ranges[code] = (info.min, info.max)
                except Exception:  # noqa: BLE001
                    pass
            self.state.reset()
            self.connected = True
            last_msg = f"🎮 connected: {dev.name}"
            self.on_status(last_msg)
            try:
                for ev in dev.read_loop():
                    if not self.alive:
                        break
                    b = self.state.event(ev.type, ev.code, ev.value)
                    if b and b.startswith("-"):
                        self.on_release(b[1:])
                    elif b:
                        self.on_button(b)
            except OSError:
                pass                                   # unplugged / out of range
            finally:
                self.connected = False
                self.state.reset()                     # sticks to zero -> the arm stops
                try:
                    dev.close()
                except Exception:  # noqa: BLE001
                    pass
            last_msg = "🎮 controller disconnected - arm stopped. Reconnect it to continue."
            self.on_status(last_msg)
            time.sleep(1)


def _diagnose():
    """python3 gamepad.py  ->  list input devices, then show the controller live (Ctrl+C to quit)."""
    try:
        import evdev  # noqa: PLC0415
    except ImportError:
        raise SystemExit("evdev missing - run: sudo apt install python3-evdev")
    print("input devices:")
    for p in evdev.list_devices():
        try:
            d = evdev.InputDevice(p)
            print(f"  {p}  {d.name}")
        except OSError as e:
            print(f"  {p}  (can't open: {e} - add yourself to the 'input' group and log in again)")
    dev, layout = _find_device()
    if dev is None:
        raise SystemExit("\nNo gamepad found. Is it plugged in / paired? If it isn't listed above at all,\n"
                         "the Pi can't see it (some phone-only controllers never show up on Linux).")
    print(f"\nusing: {dev.name}  ({'standard' if layout is AXES else 'Z/RZ right-stick'} layout)\n"
          "move the sticks / press buttons - Ctrl+C to quit\n")
    st = PadState()
    st.map = dict(layout)
    for code in layout:
        try:
            i = dev.absinfo(code)
            st.ranges[code] = (i.min, i.max)
        except Exception:  # noqa: BLE001
            pass
    for ev in dev.read_loop():
        b = st.event(ev.type, ev.code, ev.value)
        if b and b.startswith("-"):
            b = None
        a, held = st.snapshot()
        line = "  ".join(f"{k}{v:+.2f}" for k, v in a.items()) + "  " + " ".join(sorted(held))
        print(("PRESS " + b + "   " if b else "") + line + " " * 10, end="\r" if not b else "\n", flush=True)


if __name__ == "__main__":
    _diagnose()
