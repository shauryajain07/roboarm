"""On-screen joystick ("4 Drive" tab) - drive the whole arm like a game controller.

  left stick   ↔ base      ↕ shoulder        keys  A / D   W / S
  right stick  ↔ wrist     ↕ elbow           keys  J / L   I / K
  gripper      hold OPEN / CLOSE             keys  G = grab, O = open
  Shift (or the 'precise' box) = quarter speed for lining up a grab.

Velocity control: how far you push a stick = how fast that joint moves; let go and it stops
where it is. Push diagonally to move both of that stick's joints at once (within ~10 deg of
straight it moves just one). Mouse drives one stick while the keyboard drives the other, so two joints (or
all four) can move together. Everything goes through the slider system, so joint limits,
E-STOP, jam back-off and grip strength still apply.
"""
from __future__ import annotations

import threading
import time
import tkinter as tk

from gamepad import DEAD, Gamepad, curve, stick  # noqa: F401 - curve kept for importers
from poses import GRIPPER, JOINT_NAMES

NEVER = float("-inf")

PAD = 210            # canvas size, px
R = 80               # stick travel radius, px
KNOB = 24
TICK_S = 0.025       # 40 Hz, same as the slider streamer
GRIP_SPEED = 400     # ticks/s while OPEN / CLOSE (or L1 / R1) is held
GRIP_SQUEEZE = 20    # max ticks past the point where the jaws stopped on something
KEYS = {"a": (1, -1), "d": (1, 1), "w": (2, 1), "s": (2, -1),
        "j": (4, -1), "l": (4, 1), "i": (3, 1), "k": (3, -1)}


class Pad:
    """One thumb-stick: drag the knob, it springs back to the centre when released."""

    def __init__(self, parent, title, x_joint, y_joint):
        self.x_joint, self.y_joint = x_joint, y_joint
        self.dx = self.dy = 0.0
        f = self.frame = tk.Frame(parent)
        tk.Label(f, text=title, font=("Arial", 11, "bold")).pack()
        c = self.c = tk.Canvas(f, width=PAD, height=PAD, bg="#1f2937", highlightthickness=0,
                               cursor="fleur")
        c.pack()
        m = PAD / 2
        c.create_oval(m - R - KNOB, m - R - KNOB, m + R + KNOB, m + R + KNOB, outline="#4b5563", width=3)
        c.create_oval(m - DEAD * R, m - DEAD * R, m + DEAD * R, m + DEAD * R, outline="#374151")
        c.create_line(m, 22, m, PAD - 22, fill="#374151")
        c.create_line(22, m, PAD - 22, m, fill="#374151")
        c.create_text(m, 10, text=f"▲ {JOINT_NAMES[y_joint]}", fill="#9ca3af")
        c.create_text(m, PAD - 10, text=f"{JOINT_NAMES[y_joint]} ▼", fill="#9ca3af")
        c.create_text(12, m, text="◀", fill="#9ca3af")
        c.create_text(PAD - 12, m, text="▶", fill="#9ca3af")
        tk.Label(f, text=f"◀ {JOINT_NAMES[x_joint]} ▶", fg="#555").pack()
        self.knob = c.create_oval(m - KNOB, m - KNOB, m + KNOB, m + KNOB, fill="#2563eb",
                                  outline="#93c5fd", width=2)
        c.bind("<Button-1>", self._drag)
        c.bind("<B1-Motion>", self._drag)
        c.bind("<ButtonRelease-1>", self._release)

    def _place(self, dx, dy):
        m = PAD / 2
        x, y = m + dx * R, m - dy * R
        self.c.coords(self.knob, x - KNOB, y - KNOB, x + KNOB, y + KNOB)
        self.c.itemconfig(self.knob, fill="#2563eb" if dx == dy == 0 else "#f59e0b")

    def _drag(self, e):
        m = PAD / 2
        dx, dy = (e.x - m) / R, -(e.y - m) / R
        n = (dx * dx + dy * dy) ** 0.5
        if n > 1:
            dx, dy = dx / n, dy / n
        self.dx, self.dy = dx, dy
        self._place(dx, dy)

    def _release(self, _e=None):
        self.dx = self.dy = 0.0
        self._place(0, 0)


class DrivePanel:
    def __init__(self, parent, app):
        self.app = app
        self.frame = parent
        self.vmax = tk.IntVar(value=500)
        self.precise = tk.BooleanVar(value=False)
        self.flip = {j: tk.BooleanVar(value=j in app.data.joy_flip) for j in (1, 2, 3, 4)}
        self.held: dict[str, str | None] = {}      # key -> pending release timer id
        self.shift = False
        self.grip_dir = 0
        self.active: set[int] = set()               # joints moving last tick
        self.blocked: set[int] = set()              # refused this gesture (E-STOP / busy / recording)
        self.target: dict[int, float] = {}
        self._grip_t0 = 0.0

        self.pad_lbl = tk.Label(parent, text="🎮 looking for a controller…", font=("Arial", 11, "bold"),
                                fg="#1d4ed8")
        self.pad_lbl.pack(anchor="w", pady=(6, 0))
        self.pad_live = tk.Label(parent, text="", font=("Courier", 10), fg="#555", anchor="w",
                                 wraplength=640, justify="left")
        self.pad_live.pack(anchor="w")
        self._ticks = 0
        tk.Label(parent, text=("Controller: sticks as below · hold R1 close · hold L1 open · hold L2 precise · "
                               "A B X Y presets (hold View + button to save) · Menu ≡ or D-pad ↓ E-STOP · "
                               "tap View or D-pad ↑ reset · L3 HOLD · R3 CAPTURE"),
                 fg="#555", wraplength=640, justify="left").pack(anchor="w")
        tk.Label(parent, text=("Push a stick: the further you push, the faster that joint moves. "
                               "Let go: it stops. Mouse = one stick, keyboard = the other."),
                 font=("Arial", 11, "bold"), wraplength=640, justify="left").pack(anchor="w", pady=(6, 2))
        row = tk.Frame(parent)
        row.pack(pady=4)
        left = Pad(row, "LEFT STICK  (W A S D)", 1, 2)
        right = Pad(row, "RIGHT STICK  (I J K L)", 4, 3)
        self.pads = [left, right]
        left.frame.grid(row=0, column=0, padx=8)
        mid = tk.Frame(row)
        mid.grid(row=0, column=1, padx=8, sticky="n")
        right.frame.grid(row=0, column=2, padx=8)

        tk.Label(mid, text="GRIPPER", font=("Arial", 11, "bold")).pack(pady=(30, 4))
        for txt, d, bg in (("CLOSE ▶◀", 1, "#fecaca"), ("OPEN ◀ ▶", -1, "#bbf7d0")):
            b = tk.Button(mid, text=txt, width=10, height=2, bg=bg, font=("Arial", 11, "bold"))
            b.pack(pady=3)
            b.bind("<ButtonPress-1>", lambda e, d=d: setattr(self, "grip_dir", d))
            b.bind("<ButtonRelease-1>", lambda e: setattr(self, "grip_dir", 0))
        tk.Label(mid, text="hold to move\nG = grab   O = open", fg="#555").pack(pady=4)
        tk.Label(mid, text="Space / Esc = E-STOP", fg="#b91c1c", font=("Arial", 10, "bold")).pack(pady=8)

        opts = tk.Frame(parent)
        opts.pack(pady=4)
        tk.Scale(opts, variable=self.vmax, from_=100, to=1500, resolution=50, orient="horizontal",
                 length=240, label="top speed (ticks/s at full push)").pack(side="left", padx=6)
        tk.Checkbutton(opts, text="precise (¼ speed, or hold Shift)", variable=self.precise).pack(side="left")
        ff = tk.Frame(parent)
        ff.pack()
        tk.Label(ff, text="reverse a stick direction:").pack(side="left")
        for j, v in self.flip.items():
            tk.Checkbutton(ff, text=JOINT_NAMES[j], variable=v, command=self._save_flip).pack(side="left")

        pf = tk.LabelFrame(parent, text="Presets - controller A B X Y (or the Go buttons)")
        pf.pack(fill="x", padx=6, pady=6)
        self.preset_lbl = {}
        for r, n in enumerate("abxy"):
            tk.Label(pf, text=n.upper(), width=3, font=("Arial", 12, "bold")).grid(row=r, column=0)
            self.preset_lbl[n] = tk.Label(pf, text="", width=30, anchor="w", font=("Courier", 10))
            self.preset_lbl[n].grid(row=r, column=1)
            tk.Button(pf, text="Save here", command=lambda n=n: app.save_preset(n)).grid(row=r, column=2, padx=2)
            tk.Button(pf, text="Go", width=4, command=lambda n=n: app.go_preset(n)).grid(row=r, column=3, padx=2)
        self.refresh_presets()
        self._view_used = False
        self.pad = Gamepad(on_button=self._pad_button,
                           on_status=lambda t: app.q.put(("pad_status", t)),
                           on_release=lambda n: app.q.put(("pad_rel", n)))
        app.bind_all("<KeyPress>", self._key_down, add="+")
        app.bind_all("<KeyRelease>", self._key_up, add="+")
        app.after(int(TICK_S * 1000), self._tick)

    # ------------------------------------------------------------------ input
    def visible(self):
        try:
            return self.app.nb.select() == str(self.frame)
        except tk.TclError:
            return False

    def _typing(self):
        return isinstance(self.app.focus_get(), (tk.Text, tk.Entry))

    def _key_down(self, e):
        k = e.keysym.lower()
        if k.startswith("shift"):
            self.shift = True
        if k in KEYS and self.visible() and not self._typing():
            t = self.held.get(k)
            if t:
                self.app.after_cancel(t)             # key auto-repeat: still held
            self.held[k] = None

    def _key_up(self, e):
        k = e.keysym.lower()
        if k.startswith("shift"):
            self.shift = False
        if k in self.held:
            # X11 auto-repeat sends release+press pairs: only let go if no press follows
            t = self.held[k]
            if t:
                self.app.after_cancel(t)
            self.held[k] = self.app.after(40, lambda k=k: self.held.pop(k, None))

    def _pad_button(self, name):
        """Controller thread. E-STOP goes straight out - no waiting for the GUI loop."""
        if name in ("dpad_down", "menu"):            # E-STOP: straight out, no GUI wait
            self.app.estop()
        else:
            self.app.q.put(("pad_btn", name))

    def pad_button(self, name):
        """GUI thread: the controller's action buttons."""
        app = self.app
        if name == "view":
            self._view_used = False
            return
        if name in "abxy" and len(name) == 1:
            _, held = self.pad.state.snapshot()
            if "view" in held:                     # View + face button = save here
                self._view_used = True
                return app.save_preset(name)
            return app.go_preset(name)
        act = {"dpad_up": app.reset_stop, "l3": app.hold, "r3": app.capture}.get(name)
        if act:
            app.log(f"🎮 {name.upper()}")
            act()

    def pad_release(self, name):
        if name == "view" and not self._view_used:   # a plain tap of View = reset the stop
            self.app.log("🎮 View -> reset stop")
            self.app.reset_stop()

    def refresh_presets(self):
        for n, l in self.preset_lbl.items():
            p = self.app.data.presets.get(n)
            l.config(text=" ".join(f"{p[j]:4d}" for j in sorted(p)) if p else "(empty)",
                     fg="black" if p else "#9ca3af")

    def _save_flip(self):
        self.app.data.joy_flip = [j for j, v in self.flip.items() if v.get()]
        self.app.data.save()

    # ------------------------------------------------------------------ motion
    def _tick(self):
        try:
            self._step()
            self._ticks += 1
            if self._ticks % 8 == 0:                   # ~5 Hz is plenty for a readout
                self._show_pad()
        finally:
            if self.app.alive:
                self.app.after(int(TICK_S * 1000), self._tick)

    def _show_pad(self):
        """Live line under the controller status: is input arriving, and is anything blocking it?"""
        p = self.pad
        if not p.connected:
            self.pad_live.config(text="")
            return
        if not p.state.inputs:
            self.pad_live.config(fg="#b91c1c", text=(
                "no input from the controller yet - move a stick. If this stays, the Pi isn't receiving "
                "its data: close this app and run  python3 gamepad.py  to see what arrives."))
            return
        axes, held = p.state.snapshot()
        txt = (f"L {axes['lx']:+.2f} {axes['ly']:+.2f}   R {axes['rx']:+.2f} {axes['ry']:+.2f}   "
               f"L2 {axes['l2']:.0f} R2 {axes['r2']:.0f}   {' '.join(sorted(held))}")
        if self.app.arm.estop.is_set():
            txt, fg = txt + "\n⛔ E-STOP is on - sticks do nothing until RESET STOP (or tap View)", "#b91c1c"
        elif self.app.recording:
            txt, fg = txt + "\nfluid recording - only the gripper follows the controller", "#b45309"
        elif self.app.running:
            txt, fg = txt + "\nbusy - a scripted move owns the arm until it finishes", "#b45309"
        else:
            fg = "#555"
        self.pad_live.config(text=txt, fg=fg)

    def _speeds(self) -> dict[int, float]:
        vel: dict[int, float] = {}

        def add(j, v):
            if v:
                vel[j] = vel.get(j, 0.0) + v
        for p in self.pads:                           # diagonal push = both joints, in proportion
            vx, vy = stick(p.dx, p.dy)
            add(p.x_joint, vx)
            add(p.y_joint, vy)
        if self.visible():
            for k in self.held:
                j, s = KEYS[k]
                add(j, s)
        axes, _ = self.pad.state.snapshot()           # controller: same layout as the pads
        for (jx, ax), (jy, ay) in (((1, "lx"), (2, "ly")), ((4, "rx"), (3, "ry"))):
            vx, vy = stick(axes[ax], axes[ay])
            add(jx, vx)
            add(jy, vy)
        out = {}
        for j, v in vel.items():
            v = max(-1.0, min(1.0, v)) * (-1 if self.flip[j].get() else 1)
            if v:
                out[j] = v
        return out

    def _step(self):
        app = self.app
        axes, pad_held = self.pad.state.snapshot()
        slow = self.precise.get() or self.shift or axes["l2"] > 0.4      # hold L2 = precision
        scale = self.vmax.get() * (0.25 if slow else 1.0)
        moving = {j: v * scale for j, v in self._speeds().items()}
        # gripper: hold R1 = close step by step, hold L1 = open; let go = stop
        grip = self.grip_dir or (("r1" in pad_held) - ("l1" in pad_held))
        if grip:
            g = app.data.grips
            closes_up = not (g.get("open") is not None and app.data.closed_values()
                             and max(app.data.closed_values()) < g["open"])
            moving[GRIPPER] = (grip * (1 if closes_up else -1)
                               * GRIP_SPEED * (0.5 if slow else 1.0))
        if app.arm.estop.is_set():
            moving = {}
        self.blocked &= set(moving)                    # a released joint may try again
        for j in list(moving):
            if j in self.blocked:
                del moving[j]
            elif j not in self.active and not app._can_slide(j):   # logs why, once per push
                self.blocked.add(j)
                del moving[j]

        stopped = self.active - set(moving)
        for j, v in moving.items():
            if j in self.target:
                base = self.target[j]
            else:
                recent = time.monotonic() - app.touched.get(j, NEVER) <= 1.5
                base = app.svar[j].get() if recent or j not in app.pos else app.pos[j]
            t = base + v * TICK_S
            if j in app.pos:                           # never run far ahead of the real arm
                # the reading can be ~0.3 s old: allow for how far the arm has moved since,
                # otherwise a fast push gets throttled and feels laggy
                age = min(1.0, time.monotonic() - app.pos_t)
                lead = max(80.0, abs(v) * (0.25 + age))
                t = max(app.pos[j] - lead, min(app.pos[j] + lead, t))
            if j == GRIPPER and j not in self.active:
                self._grip_t0 = time.monotonic()        # this press started now
            if j == GRIPPER and j in app.pos and app.pos_prev.get(j) is not None \
                    and app.pos_prev_t > self._grip_t0 + 0.1 \
                    and abs(app.pos[j] - app.pos_prev[j]) <= 3 and (t - app.pos[j]) * v > GRIP_SQUEEZE:
                # (both readings taken AFTER the press began, so 'not moving' really means blocked)
                # jaws stopped moving while we push on = they are on something: squeeze at most
                # GRIP_SQUEEZE ticks past where they stopped (holding R1 longer won't crush it)
                t = app.pos[j] + GRIP_SQUEEZE * (1 if v > 0 else -1)
            lo, hi = app._limits(j)
            t = max(lo, min(hi, t))
            self.target[j] = t
            app.set_goal(j, t)
        app._joy_speed = scale if moving else 0
        self.active = set(moving)
        last = {j: self.target.pop(j, None) for j in stopped}
        if stopped:
            self._stop_at_actual(stopped, last)

    def _stop_at_actual(self, joints, last=None):
        """Let go = stop where the joint IS (not where the target had run ahead to). The
        gripper keeps up to GRIP_SQUEEZE of its push, so a block it was closing on stays held."""
        bus, q = self.app.arm.bus, self.app.q
        last = last or {}

        def f():
            try:
                at = {j: bus.read_position(j) for j in joints}
                if GRIPPER in at and last.get(GRIPPER) is not None:
                    push = last[GRIPPER] - at[GRIPPER]
                    at[GRIPPER] = int(round(at[GRIPPER] + max(-GRIP_SQUEEZE, min(GRIP_SQUEEZE, push))))
                q.put(("stopat", at))
            except Exception:  # noqa: BLE001
                pass
        threading.Thread(target=f, daemon=True).start()
