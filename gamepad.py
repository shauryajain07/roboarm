"""Game controller (SCUF / PlayStation / Xbox) for the Drive tab - Raspberry Pi, via evdev.

Plug the controller in by USB-C (or pair it over Bluetooth) and it is picked up
automatically; unplug it and every stick reads zero, so the arm stops.

  left stick   ↔ base      ↕ shoulder
  right stick  ↔ wrist     ↕ elbow
  R2 / L2      close / open the gripper (analog: squeeze harder = faster)
  R1 / L1      grab (this scenario's grip) / open           (same as G / O)
  ○ Circle     E-STOP              Options   RESET STOP
  △ Triangle   HOLD here           ✕ Cross   CAPTURE the selected pose (tab 2)
  □ Square     hold for precise (¼ speed)

Needs:  sudo apt install python3-evdev   (the Pi user must be in the 'input' group - it is by default)
"""
from __future__ import annotations

import threading
import time

# Linux input event codes (numbers, so the mapping can be tested without evdev installed)
EV_KEY, EV_ABS = 1, 3
AXES = {0: "lx", 1: "ly", 3: "rx", 4: "ry", 2: "l2", 5: "r2"}
INVERT = {"ly", "ry"}                     # stick up = +1
TRIGGERS = {"l2", "r2"}
BUTTONS = {304: "cross", 305: "circle", 307: "triangle", 308: "square", 310: "l1", 311: "r1",
           314: "share", 315: "options", 316: "ps", 317: "l3", 318: "r3"}


class PadState:
    """Latest stick / trigger / button state, normalised: sticks -1..1, triggers 0..1."""

    def __init__(self):
        self.lock = threading.Lock()
        self.axes = {a: 0.0 for a in AXES.values()}
        self.held: set[str] = set()
        self.ranges: dict[int, tuple[int, int]] = {}

    def reset(self):
        with self.lock:
            self.axes = {a: 0.0 for a in AXES.values()}
            self.held = set()

    def snapshot(self):
        with self.lock:
            return dict(self.axes), set(self.held)

    def event(self, etype: int, code: int, value: int):
        """Apply one input event. Returns a button name on a button PRESS, else None."""
        if etype == EV_ABS and code in AXES:
            name = AXES[code]
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
        elif etype == EV_KEY and code in BUTTONS:
            name = BUTTONS[code]
            with self.lock:
                if value:
                    self.held.add(name)
                else:
                    self.held.discard(name)
            if value == 1:
                return name
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
        if {0, 1, 3, 4} <= abs_codes and 304 in caps.get(EV_KEY, []):
            return d
        d.close()
    return None


class Gamepad:
    """Background reader. on_button(name) is called from this thread on each press;
    on_status(text) whenever the connection changes."""

    def __init__(self, on_button, on_status):
        self.state = PadState()
        self.on_button, self.on_status = on_button, on_status
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
                dev = _find_device()
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
            for code in AXES:
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
                    pressed = self.state.event(ev.type, ev.code, ev.value)
                    if pressed:
                        self.on_button(pressed)
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
