#!/usr/bin/env python3
"""ROBOARM control GUI - record every scenario, then play the game. (Raspberry Pi, Tkinter)

    python3 control_gui.py          # real arm (port/baud from the official arm_settings.py)
    python3 control_gui.py --mock   # simulated arm for practice

Left : live joint readout, jog, LIMP / HOLD, gripper, E-STOP (Space or Esc).
Tabs : 1 Setup poses   - STOW / SAFE / RACKWAIT + gripper values, recorded once
       2 Record        - the 41 scenarios; capture only what's new, test, save
       3 Game          - click a cell -> plays pick_<block> + place_<block>_<cell>
Saved scenarios are official pose_sequence files in the official repo's motions/ folder,
so 05_waypoint_sequence.py can play them too.
"""
from __future__ import annotations

import argparse
import queue
import threading
import time
import tkinter as tk
from pathlib import Path
from tkinter import messagebox, ttk

import official
import scenarios as sc
import strategy
from motion import HERE, Arm, EStop, LimitError, load_config, open_bus
from poses import ARM_JOINTS, BLOCK_POINTS, BLOCKS, CELLS, GRIPPER, JOINT_IDS, JOINT_NAMES, PoseStore
from servo_bus import ServoError, mock_bus

GAME_SECONDS = 180
MAX_TOKENS = 3
COL = {"us": "#2563eb", "them": "#dc2626", None: "#e5e7eb", "hint": "#facc15"}
ST = {"saved": "✓", "ready": "●", "todo": "○"}


class App(tk.Tk):
    def __init__(self, arm: Arm, data: sc.ScenarioData, folder: Path, settings=None):
        super().__init__()
        self.arm, self.data, self.folder, self.settings = arm, data, Path(folder), settings
        self.conv = official.Conv(settings) if settings else None
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
        self._build()
        for k in ("<space>", "<Escape>"):
            self.bind_all(k, lambda e: self.estop())
        threading.Thread(target=self._poll, daemon=True).start()
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

    def _build_joints(self, parent):
        f = tk.LabelFrame(parent, text="Arm")
        f.pack(side="left", fill="y")
        self.jlbl = {}
        for j in JOINT_IDS:
            tk.Label(f, text=f"{j} {JOINT_NAMES[j]}", width=9, anchor="w").grid(row=j, column=0)
            tk.Button(f, text="−", width=2, command=lambda j=j: self.jog(j, -1)).grid(row=j, column=1)
            self.jlbl[j] = tk.Label(f, text="----", width=13, font=("Courier", 11))
            self.jlbl[j].grid(row=j, column=2)
            tk.Button(f, text="+", width=2, command=lambda j=j: self.jog(j, 1)).grid(row=j, column=3)
        sf = tk.Frame(f)
        sf.grid(row=6, columnspan=4, pady=4)
        tk.Label(sf, text="step").pack(side="left")
        for v in (5, 20, 50, 150):
            tk.Radiobutton(sf, text=str(v), variable=self.step, value=v).pack(side="left")
        tk.Button(f, text="LIMP arm (move by hand)", bg="#fde68a",
                  command=self.limp).grid(row=7, columnspan=4, sticky="ew", pady=2)
        tk.Button(f, text="HOLD here", bg="#bbf7d0", command=self.hold).grid(row=8, columnspan=4, sticky="ew")
        tk.Label(f, text="Gripper", font=("Arial", 10, "bold")).grid(row=9, columnspan=4, pady=(8, 0))
        gf = tk.Frame(f)
        gf.grid(row=10, columnspan=4)
        for g in sc.GRIPS:
            tk.Button(gf, text=g, width=7, command=lambda g=g: self.apply_grip(g)).pack(side="top")
        tk.Label(f, text="LIMP keeps the gripper\npowered so a held\nblock won't drop.",
                 fg="#555", justify="left").grid(row=11, columnspan=4, pady=6)

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
                    self.q.put(("pos", self.arm.read_all()))
                except Exception as e:  # noqa: BLE001
                    self.q.put(("conn", str(e)))
            time.sleep(0.3)

    def _pump(self):
        while not self.q.empty():
            kind, val = self.q.get()
            if kind == "log":
                self.log(val)
            elif kind == "pos":
                self.pos = val
                for j, v in val.items():
                    ang = f"{self.conv.angle(j, v):6.1f}°" if self.conv else ""
                    self.jlbl[j].config(text=f"{v:4d} {ang}")
                self.status.config(text=f"connected · {self.arm.cfg['port']} @ {self.arm.cfg['baud']}"
                                        + ("   ⛔ E-STOP" if self.arm.estop.is_set() else ""))
            elif kind == "conn":
                self.status.config(text=f"⚠ servo read failed: {val}")
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

    def jog(self, j, sign):
        def f():
            p = self.arm.bus.read_position(j)
            self.arm.bus.set_torque(j, True)
            self.arm.move_to({j: max(0, min(4095, p + sign * self.step.get()))}, speed=300,
                             wait=(j != GRIPPER))
        self.run(f)

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
        self.run(lambda: (self.arm.bus.set_torque(GRIPPER, True),
                          self.arm.move_to({GRIPPER: v}, speed=500, ignore=(GRIPPER,))))

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
        if (self.folder / f"{name}.json").exists():
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
        for i, r in enumerate(sc.template(name)):
            pose = self.data.row_pose(name, r)
            src = {"shared": "shared (tab 1)", "capture": "CAPTURE here", "repeat": f"= {r.ref} above"}[r.kind]
            if pose is not None:
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
                if r.kind == "capture" and self.data.row_pose(name, r) is None]
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
        self.data.save()
        self.log(f"captured {name}/{r.ref}")
        if (self.folder / f"{name}.json").exists():
            self.log("  (file already saved - press Save again to update it)")
        self.refresh_all()

    def goto_row(self):
        name, r = self.selected_row()
        if r:
            self.goto(self.data.row_pose(name, r), f"{name}/{r.ref}")

    def play_selected(self):
        name = self.selected()
        try:
            pts = self.data.points(name)
        except ValueError as e:
            return self.log(str(e))
        self.log(f"TEST {name}" + (" - put the block in the jaws first!" if name.startswith("place") else ""))
        self.run(self.play_points, pts, name, sc.speeds(name))

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
        return (self.folder / f"{name}.json").exists()

    def play_file(self, name, game=True, after=None):
        names = name if isinstance(name, list) else [name]
        missing = [n for n in names if not self.has(n)]
        if missing:
            return self.log(f"not recorded yet: {', '.join(missing)}")

        def f():
            for n in names:
                path = self.folder / f"{n}.json"
                self.play_points(sc.load_points(path), n, sc.load_speeds(path, n))
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
