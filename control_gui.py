#!/usr/bin/env python3
"""ROBOARM control GUI - record every scenario, then play the game. (Raspberry Pi, Tkinter)

    python3 control_gui.py          # real arm (port/baud from the official arm_settings.py)
    python3 control_gui.py --mock   # simulated arm for practice

Left : a slider per joint (click = go there, drag = follow, wheel / arrows / hold ± = step;
       the servo eases toward the slider at the chosen slider speed and never leaves the
       official joint limits), live readout, LIMP / HOLD, gripper, E-STOP (Space or Esc).
Tabs : 1 Setup poses   - STOW / SAFE / RACKWAIT + gripper values, recorded once
       2 Record        - the 41 scenarios; capture only what's new, test, save
       3 Game          - click a cell -> plays pick_<block> + place_<block>_<cell>
Saved scenarios are official pose_sequence files in the official repo's motions/ folder,
so 05_waypoint_sequence.py can play them too.
"""
from __future__ import annotations

import argparse
import json
import queue
import threading
import time
import tkinter as tk
from pathlib import Path
from tkinter import messagebox, ttk

import official
import scenarios as sc
import strategy
import trajectory
from motion import HERE, Arm, EStop, LimitError, load_config, open_bus
from poses import ARM_JOINTS, BLOCK_POINTS, BLOCKS, CELLS, GRIPPER, JOINT_IDS, JOINT_NAMES, PoseStore
from servo_bus import STRAIN_BITS, ServoError, describe_fault, mock_bus

GAME_SECONDS = 180
MAX_TOKENS = 3
COL = {"us": "#2563eb", "them": "#dc2626", None: "#e5e7eb", "hint": "#facc15"}
ST = {"saved": "✓", "ready": "●", "todo": "○"}
# manual sliders: (ticks/s, acceleration). The servo eases toward the slider, so yanking it
# never snaps the arm - pick "gentle" near the rack, "fast" for big repositioning.
SLIDER_SPEEDS = {"gentle": (200, 15), "normal": (450, 35), "fast": (900, 80)}
SERVO_TOP_SPEED = 3000  # ST3215 real-world top speed, ticks/s
SLIDER_HZ = 20          # max rate slider targets are sent to the servos
TOUCH_HOLD_S = 1.5      # after you touch a slider it stops following the live reading this long


class App(tk.Tk):
    def __init__(self, arm: Arm, data: sc.ScenarioData, folder: Path, settings=None, traj_folder=None):
        super().__init__()
        self.arm, self.data, self.folder, self.settings = arm, data, Path(folder), settings
        self.traj_folder = Path(traj_folder or sc.TRAJ_DIR)
        self.conv = official.Conv(settings) if settings else None
        self.names = ({settings.SERVO_IDS[n]: n for n in settings.JOINT_NAMES} if settings
                      else trajectory.DEFAULT_NAMES)
        self.rate = tk.DoubleVar(value=1.0)
        self.rec_stop: threading.Event | None = None
        self.recording = False
        self.title("ROBOARM control")
        self.q: queue.Queue = queue.Queue()
        arm.log = lambda m: self.q.put(("log", m))
        self.running = False
        self.alive = True
        self.pos: dict[int, int] = {}

        # game state
        self.board = {c: None for c in CELLS}
        self.block = tk.StringVar(value="trophy")
        self.mode = tk.StringVar(value="place")
        self.authorised = tk.BooleanVar(value=False)
        self.tokens = self.tokens_earned = 0
        self.started, self.t0 = False, None

        self.step = tk.IntVar(value=20)
        # manual slider state: pending targets are picked up by the _stream thread
        self.slider_speed = tk.StringVar(value="normal")
        self._spd = SLIDER_SPEEDS["normal"]          # plain copy, read by the worker thread
        self.slider_speed.trace_add("write", lambda *_: setattr(
            self, "_spd", SLIDER_SPEEDS[self.slider_speed.get()]))
        self.goal: dict[int, int] = {}
        self.goal_lock = threading.Lock()
        self.touched: dict[int, float] = {}
        self._grab: dict[int, float | None] = {}
        self._sync_ok: bool | None = None       # SYNC READ supported? (None = not tried yet)
        self.slider_owned: set[int] = set()   # joints whose latest command came from a slider
        self._backed_off: set[int] = set()
        self._build()
        for k in ("<space>", "<Escape>"):
            self.bind_all(k, lambda e: self.estop())
        threading.Thread(target=self._poll, daemon=True).start()
        threading.Thread(target=self._stream, daemon=True).start()
        self.after(60, self._pump)
        self.refresh_all()
        self.protocol("WM_DELETE_WINDOW", self._close)

    # ================================================================ layout
    def _build(self):
        top = tk.Frame(self)
        top.pack(fill="x", padx=6, pady=4)
        self.status = tk.Label(top, text="", anchor="w")
        self.status.pack(side="left")
        tk.Button(top, text="RESET STOP", command=self.reset_stop).pack(side="right", padx=4)
        tk.Button(top, text="E-STOP  (Space)", bg="#b91c1c", fg="white", font=("Arial", 13, "bold"),
                  command=self.estop).pack(side="right")
        sp = tk.Frame(top)
        sp.pack(side="right", padx=12)
        tk.Scale(sp, variable=self.rate, from_=0.5, to=4.0, resolution=0.1, orient="horizontal",
                 length=200, label="playback speed ×").pack(side="left")
        for r in (1, 2, 4):          # one-click presets
            tk.Button(sp, text=f"{r}×", width=3, font=("Arial", 11, "bold"),
                      command=lambda r=r: self.rate.set(r)).pack(side="left", padx=1, anchor="s")

        body = tk.Frame(self)
        body.pack(fill="both", expand=True, padx=6)
        self._build_joints(body)
        nb = ttk.Notebook(body)
        nb.pack(side="left", fill="both", expand=True, padx=(8, 0))
        self.tab_setup, self.tab_rec, self.tab_game = (ttk.Frame(nb) for _ in range(3))
        nb.add(self.tab_setup, text=" 1 Setup poses ")
        nb.add(self.tab_rec, text=" 2 Record scenarios ")
        nb.add(self.tab_game, text=" 3 Game ")
        self._build_setup()
        self._build_record()
        self._build_game()

        self.logbox = tk.Text(self, height=7, font=("Courier", 10))
        self.logbox.pack(fill="x", padx=6, pady=4)

    def _limits(self, j):
        lo, hi = (int(v) for v in self.arm.cfg["joint_limits"].get(str(j), (0, 4095)))
        if j == GRIPPER:
            # never let the slider close past the tightest recorded grip ('attack' = fully
            # closed): beyond it the jaws hit each other and the servo overloads
            opn, closed = self.data.grips.get("open"), self.data.closed_values()
            if opn is not None and closed:
                if max(closed) > opn:
                    hi = min(hi, max(closed))
                else:
                    lo = max(lo, min(closed))
        return lo, hi

    def _build_joints(self, parent):
        f = tk.LabelFrame(parent, text="Arm - drag a slider to move that joint")
        f.pack(side="left", fill="y")
        self.jlbl, self.sliders, self.svar = {}, {}, {}
        for j in JOINT_IDS:
            lo, hi = self._limits(j)
            r = 2 * j
            tk.Label(f, text=f"{j} {JOINT_NAMES[j]}", width=8, anchor="w",
                     font=("Arial", 10, "bold")).grid(row=r, column=0, rowspan=2, sticky="w")
            # hold − / + down to keep moving
            tk.Button(f, text="−", width=2, font=("Arial", 12, "bold"), repeatdelay=350, repeatinterval=90,
                      command=lambda j=j: self.nudge(j, -1)).grid(row=r, column=1, rowspan=2)
            self.svar[j] = tk.IntVar(value=(lo + hi) // 2)
            s = tk.Scale(f, variable=self.svar[j], from_=lo, to=hi, orient="horizontal", length=240,
                         showvalue=False, sliderlength=28, width=20, takefocus=True,
                         highlightthickness=2, highlightcolor="#2563eb")
            s.grid(row=r, column=2, padx=2, pady=(4, 0))
            self._bind_slider(j, s)
            self.sliders[j] = s
            tk.Button(f, text="+", width=2, font=("Arial", 12, "bold"), repeatdelay=350, repeatinterval=90,
                      command=lambda j=j: self.nudge(j, 1)).grid(row=r, column=3, rowspan=2)
            self.jlbl[j] = tk.Label(f, text="----", font=("Courier", 10), anchor="w")
            self.jlbl[j].grid(row=r + 1, column=2, sticky="w")
        sf = tk.Frame(f)
        sf.grid(row=20, columnspan=4, pady=(6, 0))
        tk.Label(sf, text="step (± / wheel / arrows)").pack(side="left")
        for v in (5, 20, 50, 150):
            tk.Radiobutton(sf, text=str(v), variable=self.step, value=v).pack(side="left")
        vf = tk.Frame(f)
        vf.grid(row=21, columnspan=4)
        tk.Label(vf, text="slider speed").pack(side="left")
        for k in SLIDER_SPEEDS:
            tk.Radiobutton(vf, text=k, variable=self.slider_speed, value=k).pack(side="left")
        tk.Label(f, text=("Click anywhere on a slider to send the joint there; drag the knob to follow.\n"
                          "Mouse wheel / arrow keys = one step, Shift = 5 ticks."),
                 fg="#555", justify="left").grid(row=22, columnspan=4, pady=(2, 4))
        tk.Button(f, text="LIMP arm (move by hand)", bg="#fde68a",
                  command=self.limp).grid(row=23, columnspan=4, sticky="ew", pady=2)
        tk.Button(f, text="HOLD here", bg="#bbf7d0", command=self.hold).grid(row=24, columnspan=4, sticky="ew")
        tk.Label(f, text="Gripper", font=("Arial", 10, "bold")).grid(row=25, columnspan=4, pady=(8, 0))
        gf = tk.Frame(f)
        gf.grid(row=26, columnspan=4)
        for g in sc.GRIPS:
            tk.Button(gf, text=g, width=7, command=lambda g=g: self.apply_grip(g)).pack(side="left")
        tk.Label(f, text="LIMP keeps the gripper powered so a held block won't drop.",
                 fg="#555", justify="left").grid(row=27, columnspan=4, pady=6)

    def _bind_slider(self, j, s):
        """Replace Tk's default Scale mouse handling (click on the trough = creep 1 tick at a
        time) with: click = jump there, drag the knob = follow without jumping."""
        s.bind("<Button-1>", lambda e: self._slide_press(j, e))
        s.bind("<B1-Motion>", lambda e: self._slide_drag(j, e))
        s.bind("<ButtonRelease-1>", lambda e: self._slide_release(j))
        s.bind("<MouseWheel>", lambda e: self._step_key(j, 1 if e.delta > 0 else -1))   # mac / windows
        s.bind("<Button-4>", lambda e: self._step_key(j, 1))                            # linux (Pi)
        s.bind("<Button-5>", lambda e: self._step_key(j, -1))
        for key, sign in (("Right", 1), ("Up", 1), ("Left", -1), ("Down", -1)):
            s.bind(f"<{key}>", lambda e, d=sign: self._step_key(j, d))
            s.bind(f"<Shift-{key}>", lambda e, d=sign: self._step_key(j, d, ticks=5))

    def _build_setup(self):
        f = self.tab_setup
        tk.Label(f, text="Record these ONCE (every scenario reuses them). Move the arm there, then 'Set'.",
                 font=("Arial", 11, "bold")).grid(row=0, columnspan=5, sticky="w", pady=6)
        self.sh_lbl = {}
        r = 1
        for p in sc.SHARED:
            tk.Label(f, text=p, width=10, anchor="w", font=("Arial", 10, "bold")).grid(row=r, column=0)
            tk.Label(f, text=sc.SHARED_HELP[p], anchor="w", width=44).grid(row=r, column=1, sticky="w")
            self.sh_lbl[p] = tk.Label(f, text="-", width=26, font=("Courier", 10))
            self.sh_lbl[p].grid(row=r, column=2)
            tk.Button(f, text="Set from arm", command=lambda p=p: self.set_shared(p)).grid(row=r, column=3)
            tk.Button(f, text="Go to", command=lambda p=p: self.goto(self.data.shared.get(p), p)).grid(row=r, column=4)
            r += 1
        tk.Label(f, text="").grid(row=r)
        r += 1
        self.gr_lbl = {}
        for g in sc.GRIPS:
            tk.Label(f, text=f"grip {g}", width=10, anchor="w", font=("Arial", 10, "bold")).grid(row=r, column=0)
            tk.Label(f, text=sc.GRIP_HELP[g], anchor="w", width=44).grid(row=r, column=1, sticky="w")
            self.gr_lbl[g] = tk.Label(f, text="-", width=26, font=("Courier", 10))
            self.gr_lbl[g].grid(row=r, column=2)
            tk.Button(f, text="Set from gripper", command=lambda g=g: self.set_grip(g)).grid(row=r, column=3)
            r += 1
        tk.Button(f, text="Rebuild all saved scenario files (after changing anything here)",
                  command=self.rebuild_all).grid(row=r + 1, columnspan=5, pady=12)

    def _build_record(self):
        f = self.tab_rec
        left = tk.Frame(f)
        left.pack(side="left", fill="y", padx=4, pady=4)
        self.progress = tk.Label(left, text="", font=("Arial", 10, "bold"))
        self.progress.pack(anchor="w")
        self.scen_list = tk.Listbox(left, width=26, height=28, font=("Courier", 10), exportselection=False)
        self.scen_list.pack(fill="y", expand=True)
        self.scen_list.bind("<<ListboxSelect>>", lambda e: self.refresh_rows())
        tk.Label(left, text=f"{ST['saved']} saved  {ST['ready']} ready to save  {ST['todo']} needs poses",
                 fg="#555").pack(anchor="w")

        right = tk.Frame(f)
        right.pack(side="left", fill="both", expand=True, padx=4, pady=4)
        self.scen_title = tk.Label(right, text="", font=("Arial", 13, "bold"))
        self.scen_title.pack(anchor="w")
        cols = ("pose", "gripper", "from", "status")
        self.rows = ttk.Treeview(right, columns=cols, show="headings", height=9, selectmode="browse")
        for c, w in zip(cols, (110, 90, 120, 260)):
            self.rows.heading(c, text=c)
            self.rows.column(c, width=w)
        self.rows.pack(fill="x")
        self.rows.bind("<<TreeviewSelect>>", lambda e: self.show_hint())
        self.hint = tk.Label(right, text="", wraplength=560, justify="left", fg="#92400e",
                             font=("Arial", 11))
        self.hint.pack(anchor="w", pady=6)
        bf = tk.Frame(right)
        bf.pack(anchor="w")
        tk.Button(bf, text="CAPTURE arm → selected pose", bg="#bfdbfe", font=("Arial", 11, "bold"),
                  command=self.capture).pack(side="left", padx=2)
        tk.Button(bf, text="Go to selected", command=self.goto_row).pack(side="left", padx=2)
        tk.Button(bf, text="▶ Test whole scenario", command=self.play_selected).pack(side="left", padx=2)
        tk.Button(bf, text="💾 Save file", bg="#bbf7d0", command=self.save_selected).pack(side="left", padx=2)
        tk.Button(bf, text="Next unfinished →", command=self.next_unfinished).pack(side="left", padx=2)
        tk.Label(right, text=("Workflow: select a pose marked NEED CAPTURE → LIMP → move the arm by hand "
                              "(or jog) → HOLD → CAPTURE. When every row is ✓: Test, then Save."),
                 fg="#555", wraplength=560, justify="left").pack(anchor="w", pady=8)

        ff = tk.LabelFrame(right, text="Fluid motion (optional): record the whole scenario by hand, at your speed")
        ff.pack(fill="x", pady=4)
        self.motion_lbl = tk.Label(ff, text="", font=("Arial", 10, "bold"))
        self.motion_lbl.pack(anchor="w")
        fb = tk.Frame(ff)
        fb.pack(anchor="w", pady=2)
        self.rec_btn = tk.Button(fb, text="● Record fluid", bg="#fecaca", font=("Arial", 11, "bold"),
                                 command=self.toggle_fluid)
        self.rec_btn.pack(side="left", padx=2)
        tk.Button(fb, text="Discard fluid (use waypoints)", command=self.discard_fluid).pack(side="left", padx=2)
        tk.Label(ff, text=("The arm goes to the scenario's start pose and goes LIMP. Move it through the whole "
                           "scenario (use the gripper buttons on the left for grab / release), end at SAFE, "
                           "press ■ Stop. It is saved EXACTLY as you moved it (no smoothing, trimming or "
                           "snapping) and plays back the same way. Without a fluid take, the waypoints "
                           "play as a smooth curve."),
                 fg="#555", wraplength=560, justify="left").pack(anchor="w")

    def _build_game(self):
        f = self.tab_game
        top = tk.Frame(f)
        top.pack(fill="x", pady=4)
        self.start_btn = tk.Button(top, text="START (referee signal)", bg="#16a34a", fg="white",
                                   font=("Arial", 13, "bold"), command=self.start_game)
        self.start_btn.pack(side="left")
        self.timer = tk.Label(top, text="3:00", font=("Arial", 22, "bold"))
        self.timer.pack(side="left", padx=12)
        self.score_lbl = tk.Label(top, text="", font=("Arial", 12))
        self.score_lbl.pack(side="left")
        tk.Button(top, text="▶ start pose", command=lambda: self.play_file("start", game=False)).pack(side="right")
        tk.Button(top, text="▶ stow", command=lambda: self.play_file("stow", game=False)).pack(side="right")

        mid = tk.Frame(f)
        mid.pack()
        bf = tk.Frame(mid)
        bf.grid(row=0, column=0, padx=8)
        self.cells = {}
        for c in CELLS:
            b = tk.Button(bf, width=11, height=4, font=("Arial", 11, "bold"), command=lambda c=c: self.click_cell(c))
            b.grid(row=int(c[1]), column=int(c[3]))
            b.bind("<Button-3>", lambda e, c=c: self.cycle_cell(c))
            self.cells[c] = b
        self.game_hint = tk.Label(bf, text="", fg="#92400e", wraplength=330)
        self.game_hint.grid(row=4, columnspan=4)
        cf = tk.Frame(mid)
        cf.grid(row=0, column=1, sticky="n")
        for m, t in (("place", "PLACE block"), ("attack", "ATTACK"), ("mark", "mark opponent")):
            tk.Radiobutton(cf, text=t, variable=self.mode, value=m, command=self.refresh_game).pack(anchor="w")
        tk.Label(cf, text="\nBlock").pack(anchor="w")
        for b in BLOCKS:
            tk.Radiobutton(cf, text=f"{b} ({BLOCK_POINTS[b]})", variable=self.block, value=b,
                           command=self.refresh_game).pack(anchor="w")
        self.tok_lbl = tk.Label(cf, text="", font=("Arial", 11, "bold"))
        self.tok_lbl.pack(anchor="w", pady=(10, 0))
        tk.Checkbutton(cf, text="Referee authorised attack", variable=self.authorised).pack(anchor="w")
        tk.Label(cf, text="\nRight-click a cell to set it by hand\n(opponent blocks, corrections).",
                 fg="#555", justify="left").pack(anchor="w")

    # ================================================================ plumbing
    def log(self, m):
        self.logbox.insert("end", time.strftime("%H:%M:%S ") + m + "\n")
        self.logbox.see("end")

    def _poll(self):
        while self.alive:
            if not self.running and not self.arm.estop.is_set():
                try:
                    self.q.put(("pos", (self.arm.read_all(), dict(self.arm.bus.faults))))
                except Exception as e:  # noqa: BLE001
                    self.q.put(("conn", str(e)))
            time.sleep(0.3)

    def _pump(self):
        while not self.q.empty():
            kind, val = self.q.get()
            if kind == "log":
                self.log(val)
            elif kind == "pos":
                val, faults = val
                self.pos = val
                now = time.monotonic()
                for j, v in val.items():
                    if self._grab.get(j) is None and now - self.touched.get(j, 0) > TOUCH_HOLD_S:
                        self.svar[j].set(v)          # idle slider follows the real arm
                    self._show(j)
                warn = self._check_faults(faults)
                self.status.config(text=f"connected · {self.arm.cfg['port']} @ {self.arm.cfg['baud']}"
                                        + ("   ⛔ E-STOP" if self.arm.estop.is_set() else "") + warn,
                                   fg="#b91c1c" if warn else "black")
            elif kind == "conn":
                self.status.config(text=f"⚠ servo read failed: {val}")
            elif kind == "rec":
                n, t = val
                self.rec_btn.config(text=f"■ Stop  ({t:4.1f} s, {n} samples)")
            elif kind == "done":
                self.running = False
                if val:
                    val()
                self.refresh_all()
        if self.started and self.t0:
            left = max(0, GAME_SECONDS - int(time.monotonic() - self.t0))
            self.timer.config(text=f"{left // 60}:{left % 60:02d}", fg="red" if left < 30 else "black")
        self.after(80, self._pump)

    def run(self, fn, *a, after=None, game=False):
        if game and not self.started:
            messagebox.showwarning("Not started", "Press START only on the referee's Start Signal.\n"
                                                  "Moving before it is a False Start (rule 3.2.3).")
            return
        if self.running:
            self.log("busy - wait for the current move to finish")
            return
        self.running = True
        self.slider_owned.clear()          # a scripted move now owns the joints

        def work():
            done = None
            try:
                res = fn(*a)
                done = (lambda: after(res)) if after else None
            except EStop:
                self.q.put(("log", "stopped by E-STOP"))
            except Exception as e:  # noqa: BLE001
                self.q.put(("log", f"ERROR: {e}"))
            self.q.put(("done", done))
        threading.Thread(target=work, daemon=True).start()

    def _close(self):
        self.alive = False
        self.destroy()

    # ================================================================ arm controls
    def estop(self):
        threading.Thread(target=self.arm.emergency_stop, daemon=True).start()

    def reset_stop(self):
        try:
            self.arm.reset_estop()
            self.log("stop reset - holding position")
        except ServoError as e:
            self.log(f"reset failed: {e}")

    # ---------------------------------------------------------------- manual sliders
    def _ang(self, j, v):
        return f"{self.conv.angle(j, v):6.1f}°" if self.conv else ""

    def _show(self, j):
        v = self.pos.get(j)
        now = f"{v:4d} {self._ang(j, v)}" if v is not None else "----"
        g = self.svar[j].get()
        tgt = (f"  → {g:4d} {self._ang(j, g).strip()}"
               if time.monotonic() - self.touched.get(j, 0) <= TOUCH_HOLD_S and g != v else "")
        self.jlbl[j].config(text=now + tgt)

    def _can_slide(self, j):
        if self.arm.estop.is_set():
            self.log("E-STOP is latched - press RESET STOP first")
            return False
        if self.recording:
            if j == GRIPPER:
                return True
            self.log("fluid recording - move the arm by hand (only the gripper slider works)")
            return False
        if self.running:
            self.log("busy - wait for the current move to finish")
            return False
        return True

    def set_goal(self, j, v):
        """Point joint j's slider at v (clamped to its limits) and queue the move."""
        lo, hi = self._limits(j)
        v = int(round(max(lo, min(hi, v))))
        self.svar[j].set(v)
        self.touched[j] = time.monotonic()
        with self.goal_lock:
            self.goal[j] = v
        self.slider_owned.add(j)
        self._show(j)

    def _check_faults(self, faults):
        """Servo warning bits from the last poll. If a joint the SLIDER is driving is straining
        (overload / over-current / overheat) it is pushing into something it can't move -
        retarget it to where it actually is so it stops fighting. Returns status-bar text."""
        bad = {j: e for j, e in faults.items() if e}
        for j, e in bad.items():
            if e & STRAIN_BITS and j in self.slider_owned and not self.running and j in self.pos:
                if j not in self._backed_off:
                    self.log(f"⚠ {j} {JOINT_NAMES[j]}: {describe_fault(e)} - it is pushing against something. "
                             f"Slider backed off to where the joint really is ({self.pos[j]}).")
                    self._backed_off.add(j)
                self.set_goal(j, self.pos[j])
        self._backed_off &= set(bad)
        return ("   ⚠ " + "; ".join(f"{j} {JOINT_NAMES[j]}: {describe_fault(e)}" for j, e in sorted(bad.items()))
                if bad else "")

    def _value_at(self, j, e):
        s = self.sliders[j]
        return float(s.tk.call(s._w, "get", e.x, e.y))

    def _slide_press(self, j, e):
        s = self.sliders[j]
        s.focus_set()
        if not self._can_slide(j):
            self._grab[j] = None
            return "break"
        # grabbing the knob keeps its offset (no jump); clicking the trough jumps there
        self._grab[j] = (self._value_at(j, e) - self.svar[j].get()
                         if s.identify(e.x, e.y) == "slider" else 0.0)
        self.set_goal(j, self._value_at(j, e) - self._grab[j])
        return "break"

    def _slide_drag(self, j, e):
        if self._grab.get(j) is not None:
            self.set_goal(j, self._value_at(j, e) - self._grab[j])
        return "break"

    def _slide_release(self, j):
        self._grab[j] = None
        return "break"

    def _step_key(self, j, sign, ticks=None):
        self.nudge(j, sign, ticks)
        return "break"

    def nudge(self, j, sign, ticks=None):
        """± buttons, wheel, arrows: one step from where the joint is heading."""
        if not self._can_slide(j):
            return
        recent = time.monotonic() - self.touched.get(j, 0) <= TOUCH_HOLD_S
        base = self.svar[j].get() if recent or j not in self.pos else self.pos[j]
        self.set_goal(j, base + sign * (ticks or self.step.get()))

    def _stream(self):
        """Send the newest slider target per joint at most SLIDER_HZ times a second, so a
        fast drag never floods the bus and the servo always heads for where the slider is now."""
        while self.alive:
            time.sleep(1 / SLIDER_HZ)
            with self.goal_lock:
                pending, self.goal = self.goal, {}
            if not pending or self.arm.estop.is_set():
                continue
            speed, acc = self._spd
            for j, v in pending.items():
                if self.recording and j != GRIPPER:
                    continue
                if self.running and not self.recording:
                    continue
                try:
                    self.arm.bus.set_torque(j, True)
                    self.arm.bus.write_position(j, v, speed, acc)
                    if j == GRIPPER:
                        self.arm.gripper_goal = v
                except Exception as e:  # noqa: BLE001
                    self.q.put(("log", f"slider move joint {j} failed: {e}"))

    def limp(self):
        self.run(lambda: self.arm.torque(False, ARM_JOINTS))
        self.log("arm joints 1-4 LIMP - support the arm and move it by hand, then HOLD")

    def hold(self):
        def f():
            cur = self.arm.read_all()
            self.arm.bus.sync_write_positions({j: (cur[j], 150, 20) for j in JOINT_IDS})
            self.arm.torque(True)
        self.run(f)

    def apply_grip(self, g):
        if g not in self.data.grips:
            return self.log(f"grip '{g}' not set yet (tab 1)")
        v = self.data.grips[g]
        self.slider_owned.discard(GRIPPER)
        act = lambda: (self.arm.bus.set_torque(GRIPPER, True),
                       self.arm.move_to({GRIPPER: v}, speed=500, ignore=(GRIPPER,)))
        if self.recording:            # grab / release DURING a fluid recording
            self.log(f"  gripper -> {g}")
            threading.Thread(target=act, daemon=True).start()
        else:
            self.run(act)

    def goto(self, pose, label="pose"):
        if not pose:
            return self.log(f"{label} not recorded yet")
        def f():
            self.arm.torque(True)
            self.arm.move_to(pose, speed=self.arm.cfg["slow_speed"],
                             ignore=(GRIPPER,) if pose[GRIPPER] in self.data.closed_values() else ())
        self.log(f"-> {label}")
        self.run(f)

    def play_points(self, pts, label, speeds=None):
        closed = self.data.closed_values()
        speeds = speeds or ["normal"] * len(pts)
        cfg = self.arm.cfg
        self.arm.torque(True)
        for i, (p, sp) in enumerate(zip(pts, speeds), 1):
            fast = sp == "max"
            self.arm.log(f"  {label} point {i}/{len(pts)}" + ("  ⚡ MAX SPEED" if fast else ""))
            self.arm.move_to(p, ignore=(GRIPPER,) if p[GRIPPER] in closed else (),
                             speed=cfg["max_speed"] if fast else None,
                             acc=cfg["max_acc"] if fast else None)
            if i < len(pts) and p[GRIPPER] != pts[i][GRIPPER]:
                time.sleep(0.05)
        return True

    # ================================================================ setup tab
    def _need_pos(self):
        if not self.pos:
            self.log("no arm reading yet")
            return False
        return True

    def _limit_warn(self, pose, what):
        try:
            self.arm.check_limits(pose)
        except LimitError as e:
            self.log(f"⚠ {what}: {e} (outside the official ANGLE_LIMITS - playback will refuse it)")

    def set_shared(self, p):
        if self._need_pos():
            pose = self.arm.read_all()
            self._limit_warn(pose, p)
            self.data.shared[p] = pose
            self.data.save()
            self.log(f"set {p} = {pose}")
            self.refresh_all()

    def set_grip(self, g):
        if self._need_pos():
            self.data.grips[g] = self.arm.bus.read_position(GRIPPER)
            self.data.save()
            self.log(f"set grip {g} = {self.data.grips[g]}")
            lo, hi = self._limits(GRIPPER)
            self.sliders[GRIPPER].config(from_=lo, to=hi)
            self.refresh_all()

    def rebuild_all(self):
        n = 0
        for name in sc.all_scenarios():
            if (self.folder / f"{name}.json").exists() and not self.data.missing(name):
                sc.save_scenario(self.data, name, self.folder, self.settings)
                n += 1
        self.log(f"rebuilt {n} scenario files with the current shared poses")
        self.refresh_all()

    # ================================================================ record tab
    def scen_status(self, name):
        if (self.folder / f"{name}.json").exists() or sc.fluid_path(name, self.traj_folder).exists():
            return "saved"
        return "todo" if self.data.missing(name) else "ready"

    def selected(self):
        sel = self.scen_list.curselection()
        return sc.all_scenarios()[sel[0]] if sel else None

    def refresh_scenarios(self):
        names = sc.all_scenarios()
        keep = self.scen_list.curselection()
        self.scen_list.delete(0, "end")
        n_saved = 0
        for n in names:
            s = self.scen_status(n)
            n_saved += s == "saved"
            self.scen_list.insert("end", f"{ST[s]} {n}")
        if keep:
            self.scen_list.selection_set(keep[0])
        elif names:
            self.scen_list.selection_set(0)
        self.progress.config(text=f"{n_saved}/{len(names)} scenarios saved")

    def refresh_rows(self):
        name = self.selected()
        self.rows.delete(*self.rows.get_children())
        if not name:
            return
        self.scen_title.config(text=f"{name}   [{self.scen_status(name)}]")
        fl = sc.fluid_path(name, self.traj_folder)
        if fl.exists():
            rep = json.loads(fl.read_text()).get("roboarm", {}).get("report", {})
            dur = rep.get("duration_s", rep.get("clean_duration_s", "?"))
            how = (f"exact, {rep['hz']} Hz" if "hz" in rep else "OLD CLEANED take - re-record it")
            self.motion_lbl.config(text=f"plays: ● FLUID TAKE ({dur} s, {how})", fg="#15803d")
        elif not self.data.missing(name):
            self.motion_lbl.config(text="plays: smooth curve through the waypoints", fg="#1d4ed8")
        else:
            self.motion_lbl.config(text="plays: nothing yet (capture the poses or record a fluid take)", fg="#b91c1c")
        if not self.recording:
            self.rec_btn.config(text="● Record fluid")
        for i, r in enumerate(sc.template(name)):
            pose = self.data.row_pose(name, r)
            src = {"shared": "shared (tab 1)", "capture": "CAPTURE here", "repeat": f"= {r.ref} above",
                   "derived": "auto (short punch)"}[r.kind]
            note = self.data.notes.get(f"{name}/{r.ref}")
            if pose is not None and note:
                st = "⚠ " + note
            elif pose is not None:
                st = "✓ " + (self.conv.angles_str(pose) if self.conv else " ".join(map(str, pose.values())))
            elif r.kind == "capture":
                st = "NEED CAPTURE"
            elif r.kind == "shared" and r.ref not in self.data.shared:
                st = f"set {r.ref} in tab 1"
            elif r.grip and r.grip not in self.data.grips:
                st = f"set grip '{r.grip}' in tab 1"
            else:
                st = f"capture {r.ref} first"
            self.rows.insert("", "end", iid=str(i), values=(r.ref, r.grip or "as is", src, st))
        todo = [str(i) for i, r in enumerate(sc.template(name))
                if r.kind == "capture" and (self.data.row_pose(name, r) is None
                                            or f"{name}/{r.ref}" in self.data.notes)]
        first = todo[0] if todo else "0"
        self.rows.selection_set(first)
        self.show_hint()

    def selected_row(self):
        name = self.selected()
        sel = self.rows.selection()
        return (name, sc.template(name)[int(sel[0])]) if name and sel else (name, None)

    def show_hint(self):
        name, r = self.selected_row()
        if r:
            g = f"  Gripper will be set to '{r.grip}' automatically." if r.grip else ""
            self.hint.config(text=f"{r.ref}: {r.hint}.{g}")

    def capture(self):
        name, r = self.selected_row()
        if not r:
            return
        if r.kind != "capture":
            return self.log(f"'{r.ref}' comes from {'tab 1' if r.kind == 'shared' else 'the capture above'} - select a NEED CAPTURE row")
        if not self._need_pos():
            return
        pose = self.arm.read_all()
        self._limit_warn(pose, f"{name}/{r.ref}")
        self.data.captures.setdefault(name, {})[r.ref] = pose
        self.data.notes.pop(f"{name}/{r.ref}", None)       # re-recorded -> no longer provisional
        self.data.save()
        self.log(f"captured {name}/{r.ref}")
        if (self.folder / f"{name}.json").exists():
            self.log("  (file already saved - press Save again to update it)")
        self.refresh_all()

    def goto_row(self):
        name, r = self.selected_row()
        if r:
            self.goto(self.data.row_pose(name, r), f"{name}/{r.ref}")

    def play_scenario(self, name, rate=1.0):
        """Fluid playback: the recorded take if there is one, else the smooth waypoint curve.
        Runs in a worker thread - `rate` is read from the slider on the Tk thread by the caller."""
        tr, src = sc.fluid_for(self.data, name, self.arm.cfg, self.traj_folder)
        if src == "recorded":
            tr = trajectory.load(sc.fluid_path(name, self.traj_folder), self.names)
        self.arm.log(f"  ▶ {name} ({src}, {tr.duration / rate:.1f} s at ×{rate:g})")
        need = trajectory.max_speed(tr) * rate
        if need > SERVO_TOP_SPEED:
            self.arm.log(f"  ⚠ at ×{rate:g} this needs {need:.0f} ticks/s but the servos top out near "
                         f"{SERVO_TOP_SPEED} - the arm will lag behind and cut corners. Try a lower speed.")
        self.arm.torque(True)
        last_g = int(round(tr.q[-1][GRIPPER]))
        trajectory.play(self.arm, tr, rate=rate, hz=self.arm.cfg.get("stream_hz", 50),
                        ignore_gripper=last_g in self.data.closed_values(), log=self.arm.log)
        return True

    def play_selected(self):
        name = self.selected()
        if not sc.playable(self.data, name, self.traj_folder):
            return self.log(f"{name}: still need {', '.join(self.data.missing(name))}")
        self.log(f"TEST {name}" + (" - put the block in the jaws first!" if name.startswith("place") else ""))
        self.run(self.play_scenario, name, float(self.rate.get()))

    # ---------------------------------------------------------------- fluid recording
    def toggle_fluid(self):
        if self.recording and self.rec_stop is not None:
            self.rec_stop.set()
            self.rec_btn.config(text="… saving")
            return
        name = self.selected()
        if not name:
            return
        rows = sc.template(name)
        start, end = self.data.row_pose(name, rows[0]), self.data.row_pose(name, rows[-1])
        if start is None or end is None:
            return self.log("set the shared poses and gripper values in tab 1 first")
        if self.running:
            return self.log("busy - wait for the current move to finish")
        self.running = self.recording = True
        self.rec_stop = threading.Event()
        threading.Thread(target=self._record_fluid, args=(name, start, end), daemon=True).start()

    def _record_fluid(self, name, start, end):
        log = lambda m: self.q.put(("log", m))
        done = None
        try:
            log(f"FLUID RECORD {name}: moving to its start pose")
            self.arm.torque(True)
            self.arm.move_to(start, speed=self.arm.cfg["slow_speed"], ignore=(GRIPPER,))
            self.arm.torque(False, ARM_JOINTS)
            log("  arm LIMP (gripper powered). Recording in 3…")
            for k in (2, 1):
                time.sleep(0.7)
                log(f"  {k}…")
            time.sleep(0.7)
            log("  ● RECORDING - move through the scenario, end at SAFE, press ■ Stop")
            hz = self.arm.cfg.get("record_hz", 50)
            raw = trajectory.record(self._read_fast, self.rec_stop, hz=hz,
                                    on_sample=lambda n, t: n % 10 == 0 and self.q.put(("rec", (n, t))))
            cur = self.arm.read_all()
            self.arm.bus.sync_write_positions({j: (cur[j], 150, 20) for j in JOINT_IDS})
            self.arm.torque(True)
            if len(raw) < 2:
                raise ValueError("recording too short - nothing saved")
            # saved exactly as recorded: no smoothing, trimming, pause-shortening or snapping
            got = round((len(raw) - 1) / raw.duration, 1) if raw.duration else 0
            rep = {"duration_s": round(raw.duration, 2), "samples": len(raw), "hz": got, "cleaned": False}
            trajectory.save_raw(self.traj_folder / "raw" / f"{name}.json", raw)
            trajectory.save(sc.fluid_path(name, self.traj_folder), raw, name,
                            f"roboarm fluid take (exact): {name}", self.names, extra={"report": rep})
            log(f"  ✓ saved exactly as recorded: {rep['duration_s']} s, {len(raw)} samples "
                f"({got} Hz{', bus is the limit' if got < 0.9 * hz else ''}). Press ▶ Test.")
        except EStop:
            log("stopped by E-STOP - recording discarded")
        except Exception as e:  # noqa: BLE001
            log(f"ERROR: {e}")
        finally:
            self.recording = False
            self.rec_stop = None
            self.q.put(("done", done))

    def _read_fast(self):
        """One recording sample: SYNC READ (all 5 joints in one round trip) if the bus supports
        it, else the 5 single reads."""
        if self._sync_ok is not False:
            try:
                r = self.arm.bus.sync_read_positions(JOINT_IDS)
                self._sync_ok = True
                return r
            except ServoError:
                if self._sync_ok:            # worked before: just a dropped sample
                    raise
                self._sync_ok = False
                self.q.put(("log", "  (this bus has no SYNC READ - reading the 5 servos one by one)"))
        return self.arm.read_all()

    def discard_fluid(self):
        name = self.selected()
        p = sc.fluid_path(name, self.traj_folder)
        if p.exists():
            p.unlink()
            self.log(f"discarded the fluid take for {name} - it will play its waypoints as a smooth curve")
        self.refresh_all()

    def save_selected(self):
        name = self.selected()
        try:
            path = sc.save_scenario(self.data, name, self.folder, self.settings)
        except ValueError as e:
            return self.log(str(e))
        self.log(f"saved {path}")
        self.refresh_all()
        self.next_unfinished()

    def next_unfinished(self):
        names = sc.all_scenarios()
        cur = self.selected()
        start = names.index(cur) + 1 if cur in names else 0
        for k in list(range(start, len(names))) + list(range(0, start)):
            if self.scen_status(names[k]) != "saved":
                self.scen_list.selection_clear(0, "end")
                self.scen_list.selection_set(k)
                self.scen_list.see(k)
                self.refresh_rows()
                return
        self.log("ALL 41 SCENARIOS SAVED 🎉")

    # ================================================================ game tab
    def has(self, name):
        return sc.playable(self.data, name, self.traj_folder)

    def play_file(self, name, game=True, after=None):
        names = name if isinstance(name, list) else [name]
        missing = [n for n in names if not self.has(n)]
        if missing:
            return self.log(f"not recorded yet: {', '.join(missing)}")

        rate = float(self.rate.get())       # read on the Tk thread

        def f():
            for n in names:
                self.play_scenario(n, rate)
            return True
        self.run(f, after=after, game=game)

    def start_game(self):
        if not self.started:
            self.started, self.t0 = True, time.monotonic()
            self.start_btn.config(state="disabled", text="GAME RUNNING")
            self.log("START - game period running")

    def click_cell(self, c):
        m = self.mode.get()
        if m == "mark":
            return self.cycle_cell(c)
        if m == "place":
            b = self.block.get()
            if self.board[c]:
                return self.log(f"{c} is occupied")
            self.play_file([f"pick_{b}", f"place_{b}_{c}"], after=lambda _: self._placed(b, c))
        else:
            if self.tokens <= 0:
                return self.log("no attack token")
            if not self.authorised.get():
                return messagebox.showwarning("Attack", 'Call "Attack", wait for the referee, then tick the box.')
            if not self.has(f"attack_{c}"):
                return self.log(f"attack_{c} not recorded")
            self.tokens -= 1                     # consumed when the attempt begins (3.3.4.6)
            self.authorised.set(False)
            self.play_file(f"attack_{c}", after=lambda _: self._attacked(c))

    def _placed(self, b, c):
        self.board[c] = ("us", b)
        if self.tokens_earned < MAX_TOKENS:
            self.tokens_earned += 1
            self.tokens += 1
        self.log(f"placed {b} in {c}")
        if strategy.winner_line(self.board, "us"):
            self.log("*** THREE IN A LINE - call the referee! ***")

    def _attacked(self, c):
        if messagebox.askyesno("Attack result", f"Did the block in {c} fall out?"):
            self.board[c] = None

    def cycle_cell(self, c):
        order = [None] + [("them", b) for b in BLOCKS] + [("us", b) for b in BLOCKS]
        i = order.index(self.board[c]) if self.board[c] in order else 0
        self.board[c] = order[(i + 1) % len(order)]
        self.refresh_game()

    def refresh_game(self):
        b = self.block.get()
        if self.mode.get() == "attack":
            hint, why = strategy.suggest_attack(self.board)
        else:
            ok = [c for c in CELLS if self.has(f"pick_{b}") and self.has(f"place_{b}_{c}")]
            hint, why = strategy.suggest_place(self.board, ok) if self.mode.get() == "place" else (None, "")
        for c, btn in self.cells.items():
            v = self.board[c]
            txt = f"{c}\n" + (f"{'US' if v[0] == 'us' else 'THEM'}\n{v[1]}" if v else "")
            need = (f"place_{b}_{c}" if self.mode.get() == "place" else
                    f"attack_{c}" if self.mode.get() == "attack" else None)
            if need and not self.has(need):
                txt += "\n(not recorded)"
            bg = COL["hint"] if c == hint else COL[v[0] if v else None]
            btn.config(text=txt, bg=bg)
        self.game_hint.config(text=f"Suggest {hint}: {why}" if hint else why)
        self.tok_lbl.config(text=f"Attack tokens: {self.tokens} (earned {self.tokens_earned}/{MAX_TOKENS})")
        self.score_lbl.config(text=f"US {strategy.score(self.board, 'us')}  -  THEM {strategy.score(self.board, 'them')}")

    # ================================================================
    def refresh_all(self):
        for p, l in self.sh_lbl.items():
            v = self.data.shared.get(p)
            l.config(text=" ".join(str(v[j]) for j in JOINT_IDS) if v else "not set", fg="black" if v else "red")
        for g, l in self.gr_lbl.items():
            v = self.data.grips.get(g)
            l.config(text=str(v) if v is not None else "not set", fg="black" if v is not None else "red")
        self.refresh_scenarios()
        self.refresh_rows()
        self.refresh_game()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mock", action="store_true", help="simulated arm, no hardware")
    ap.add_argument("--data", default=str(Path(HERE) / "scenario_data.json"))
    ap.add_argument("--motions", help="where scenario files go (default: official repo's motions/)")
    a = ap.parse_args()

    cfg = load_config()
    settings = official.load_settings()
    if settings is None:
        print("NOTE: official STS3215-5DOF-Robot-Arm repo not found - using roboarm defaults.\n"
              "      Clone it to ~/robot-arm-lab/ (workshop step 1.3.3) so port/limits match.")
    bus = mock_bus(time_scale=5)[0] if a.mock else open_bus(cfg)
    if a.mock:
        cfg["port"] = "MOCK"
    arm = Arm(bus, PoseStore(str(Path(HERE) / "poses.json")), cfg)
    bad = arm.check_servos()
    if bad:
        print(f"WARNING: servos not answering: {bad}")
    folder = Path(a.motions) if a.motions else sc.motions_dir()
    print(f"scenario files -> {folder}")
    App(arm, sc.ScenarioData(a.data), folder, settings).mainloop()


if __name__ == "__main__":
    main()
