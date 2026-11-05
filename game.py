#!/usr/bin/env python3
"""ROBOARM game-day operator GUI (Raspberry Pi, Tkinter).

  python3 game.py          # real arm
  python3 game.py --mock   # simulated arm for practice

Flow: connect -> arm sits in STOW -> press START only on the referee's Start Signal
-> pick a block type -> click a cell = robot picks that block and places it there.
Attack: needs a token; call "Attack", wait for referee OK, tick 'Referee authorised',
click the opponent's cell.   SPACE = EMERGENCY STOP (torque off) at any time.
"""
from __future__ import annotations

import argparse
import os
import queue
import threading
import time
import tkinter as tk
from tkinter import messagebox

from motion import HERE, Arm, EStop, load_config, open_bus
from poses import ARM_JOINTS, BLOCK_POINTS, BLOCKS, CELLS, GRIPPER, JOINT_IDS, JOINT_NAMES, PoseStore, cell_words
from servo_bus import ServoError, mock_bus
import strategy

GAME_SECONDS = 180
MAX_TOKENS = 3
COL = {"us": "#2563eb", "them": "#dc2626", None: "#e5e7eb", "hint": "#facc15"}


class Game(tk.Tk):
    def __init__(self, arm: Arm, store: PoseStore):
        super().__init__()
        self.arm, self.store = arm, store
        self.title("ROBOARM - Domination of the Nine Squares")
        self.configure(bg="white")
        self.q: queue.Queue = queue.Queue()
        arm.log = lambda m: self.q.put(("log", m))

        self.board = {c: None for c in CELLS}
        self.stock = {b: tk.IntVar(value=1) for b in BLOCKS}
        self.block = tk.StringVar(value="trophy")
        self.mode = tk.StringVar(value="place")
        self.authorised = tk.BooleanVar(value=False)
        self.tokens_earned = 0
        self.tokens = 0
        self.started = False
        self.t0 = None
        self.running = False

        self._build()
        self.bind_all("<space>", lambda e: self.estop())
        self.bind_all("<Escape>", lambda e: self.estop())
        self.after(50, self._pump)
        self._refresh()

    # ----------------------------------------------------------------- UI
    def _build(self):
        top = tk.Frame(self, bg="white")
        top.pack(fill="x", padx=8, pady=6)
        self.start_btn = tk.Button(top, text="START  (referee signal)", font=("Arial", 14, "bold"),
                                   bg="#16a34a", fg="white", command=self.start)
        self.start_btn.pack(side="left")
        self.timer = tk.Label(top, text="3:00", font=("Arial", 24, "bold"), bg="white")
        self.timer.pack(side="left", padx=16)
        self.score_lbl = tk.Label(top, text="", font=("Arial", 13), bg="white")
        self.score_lbl.pack(side="left", padx=8)
        tk.Button(top, text="RESET STOP", command=self.reset_stop).pack(side="right", padx=4)
        tk.Button(top, text="E-STOP\n(space)", font=("Arial", 16, "bold"), bg="#b91c1c", fg="white",
                  width=8, command=self.estop).pack(side="right", padx=4)

        mid = tk.Frame(self, bg="white")
        mid.pack(padx=8, pady=4)

        # board
        bf = tk.Frame(mid, bg="white")
        bf.grid(row=0, column=0, padx=8)
        tk.Label(bf, text="RACK (our view)", bg="white", font=("Arial", 11, "bold")).grid(columnspan=3)
        self.cells = {}
        for c in CELLS:
            r, k = int(c[1]), int(c[3])
            b = tk.Button(bf, width=11, height=4, font=("Arial", 11, "bold"),
                          command=lambda c=c: self.click(c))
            b.grid(row=r, column=k - 1, padx=2, pady=2)
            b.bind("<Button-3>", lambda e, c=c: self.cycle_manual(c))
            self.cells[c] = b
        self.hint = tk.Label(bf, text="", bg="white", fg="#92400e", wraplength=330, justify="left")
        self.hint.grid(row=4, columnspan=3, pady=4)

        # controls
        cf = tk.Frame(mid, bg="white")
        cf.grid(row=0, column=1, sticky="n", padx=8)
        tk.Label(cf, text="Click a cell to:", bg="white", font=("Arial", 11, "bold")).pack(anchor="w")
        for m, t in (("place", "PLACE selected block"), ("attack", "ATTACK (knock out)"),
                     ("mark", "mark OPPONENT block")):
            tk.Radiobutton(cf, text=t, variable=self.mode, value=m, bg="white",
                           command=self._refresh).pack(anchor="w")
        tk.Label(cf, text="\nBlock  (left = remaining)", bg="white", font=("Arial", 11, "bold")).pack(anchor="w")
        for b in BLOCKS:
            row = tk.Frame(cf, bg="white")
            row.pack(anchor="w")
            tk.Spinbox(row, from_=0, to=9, width=2, textvariable=self.stock[b]).pack(side="left")
            tk.Radiobutton(row, text=f"{b} ({BLOCK_POINTS[b]})", variable=self.block, value=b,
                           bg="white", command=self._refresh).pack(side="left")
        self.tok_lbl = tk.Label(cf, text="", bg="white", font=("Arial", 12, "bold"))
        self.tok_lbl.pack(anchor="w", pady=(10, 0))
        tk.Checkbutton(cf, text="Referee authorised attack", variable=self.authorised,
                       bg="white").pack(anchor="w")
        tk.Button(cf, text="Placement failed (undo)", command=self.undo_last).pack(anchor="w", pady=4)

        tk.Label(cf, text="\nRecovery", bg="white", font=("Arial", 11, "bold")).pack(anchor="w")
        rf = tk.Frame(cf, bg="white")
        rf.pack(anchor="w")
        tk.Button(rf, text="SAFE", command=lambda: self.run(self.arm.go, "safe", keep_gripper=True)).pack(side="left")
        tk.Button(rf, text="STOW", command=lambda: self.run(self.arm.stow)).pack(side="left")
        tk.Button(rf, text="OPEN", command=lambda: self.run(self.arm.open_gripper)).pack(side="left")
        jf = tk.Frame(cf, bg="white")
        jf.pack(anchor="w", pady=2)
        for j in JOINT_IDS:
            tk.Label(jf, text=JOINT_NAMES[j], bg="white", width=8, anchor="w").grid(row=j, column=0)
            tk.Button(jf, text="-", width=2, command=lambda j=j: self.jog(j, -40)).grid(row=j, column=1)
            tk.Button(jf, text="+", width=2, command=lambda j=j: self.jog(j, 40)).grid(row=j, column=2)

        self.logbox = tk.Text(self, height=8, width=90, font=("Courier", 10))
        self.logbox.pack(padx=8, pady=6)
        self.last_action = None

    def log(self, m):
        self.logbox.insert("end", time.strftime("%M:%S ") + m + "\n")
        self.logbox.see("end")

    def _pump(self):
        while not self.q.empty():
            kind, val = self.q.get()
            if kind == "log":
                self.log(val)
            elif kind == "done":
                self.running = False
                if val:
                    val()
                self._refresh()
        if self.started and self.t0:
            left = max(0, GAME_SECONDS - int(time.monotonic() - self.t0))
            self.timer.config(text=f"{left // 60}:{left % 60:02d}", fg="red" if left < 30 else "black")
        self.after(100, self._pump)

    def _refresh(self):
        hint_cell, why = None, ""
        if self.mode.get() == "place":
            ok = [c for c in CELLS if self.store.can_place(self.block.get(), c)]
            hint_cell, why = strategy.suggest_place(self.board, ok)
        elif self.mode.get() == "attack":
            hint_cell, why = strategy.suggest_attack(self.board)
        for c, btn in self.cells.items():
            v = self.board[c]
            txt = f"{c}\n" + (f"{'US' if v[0] == 'us' else 'THEM'}\n{v[1]}" if v else "")
            bg = COL[v[0] if v else None]
            if c == hint_cell:
                bg = COL["hint"] if not v or self.mode.get() == "attack" else bg
            missing = (self.mode.get() == "place" and not self.store.can_place(self.block.get(), c)) or \
                      (self.mode.get() == "attack" and not self.store.can_attack(c))
            btn.config(text=txt + ("\n(not recorded)" if missing else ""), bg=bg,
                       state="disabled" if self.running else "normal")
        self.hint.config(text=f"Suggest: {hint_cell} - {why}" if hint_cell else why)
        self.tok_lbl.config(text=f"Attack tokens: {self.tokens}  (earned {self.tokens_earned}/{MAX_TOKENS})")
        self.score_lbl.config(text=f"US {strategy.score(self.board, 'us')}  -  "
                                   f"THEM {strategy.score(self.board, 'them')}")

    # ------------------------------------------------------------- actions
    def run(self, fn, *a, after=None, **kw):
        if not self.started:
            messagebox.showwarning("Not started", "Wait for the referee's Start Signal, then press START.\n"
                                                  "(Moving before = False Start, rule 3.2.3)")
            return
        if self.running:
            return
        self.running = True
        self._refresh()

        def work():
            done = None
            try:
                res = fn(*a, **kw)
                done = (lambda: after(res)) if after else None
            except EStop:
                self.q.put(("log", "stopped by E-STOP"))
            except Exception as e:  # noqa: BLE001 - report everything to the operator
                self.q.put(("log", f"ERROR: {e}"))
            self.q.put(("done", done))
        threading.Thread(target=work, daemon=True).start()

    def start(self):
        if self.started:
            return
        self.started, self.t0 = True, time.monotonic()
        self.start_btn.config(state="disabled", text="GAME RUNNING")
        self.log("START - game period running")

    def click(self, c):
        m = self.mode.get()
        if m == "mark":
            self.cycle_manual(c)
            return
        if m == "place":
            b = self.block.get()
            if self.board[c]:
                return self.log(f"{c} occupied")
            if self.stock[b].get() <= 0:
                return self.log(f"no {b} left")
            self.run(self.arm.turn, b, c, after=lambda ok: self._placed(b, c, ok))
        elif m == "attack":
            if self.tokens <= 0:
                return self.log("no attack token")
            if not self.authorised.get():
                return messagebox.showwarning("Attack", 'Call "Attack" and wait for the referee, '
                                                        "then tick 'Referee authorised'.")
            self.tokens -= 1           # consumed when the attempt begins (3.3.4.6)
            self.authorised.set(False)
            self.run(self.arm.attack, c, True, after=lambda _: self._attacked(c))

    def _placed(self, b, c, felt):
        self.stock[b].set(self.stock[b].get() - 1)
        self.board[c] = ("us", b)
        self.last_action = c
        if self.tokens_earned < MAX_TOKENS:
            self.tokens_earned += 1
            self.tokens += 1
        self.log(f"placed {b} in {c} ({cell_words(c)})" + ("" if felt else "  [grip not felt!]"))
        if strategy.winner_line(self.board, "us"):
            self.log("*** THREE IN A LINE - call the referee! ***")

    def _attacked(self, c):
        if messagebox.askyesno("Attack result", f"Did the block in {c} fall out?"):
            self.board[c] = None
        self._refresh()

    def undo_last(self):
        c = self.last_action
        if c and self.board[c] and self.board[c][0] == "us":
            b = self.board[c][1]
            self.board[c] = None
            self.stock[b].set(self.stock[b].get() + 1)
            self.log(f"undo: {c} cleared (token count unchanged - fix by hand if referee refused)")
        self._refresh()

    def cycle_manual(self, c):
        order = [None, ("them", "cube"), ("them", "cylinder"), ("them", "trophy"),
                 ("us", "cube"), ("us", "cylinder"), ("us", "trophy")]
        i = order.index(self.board[c]) if self.board[c] in order else 0
        self.board[c] = order[(i + 1) % len(order)]
        self._refresh()

    def jog(self, j, d):
        def f():
            p = self.arm.bus.read_position(j)
            self.arm.move_to({j: max(0, min(4095, p + d))}, speed=200, ignore=(j,) if j == GRIPPER else ())
        self.run(f)

    def estop(self):
        threading.Thread(target=self.arm.emergency_stop, daemon=True).start()

    def reset_stop(self):
        try:
            self.arm.reset_estop()
            self.log("stop reset - holding position")
        except ServoError as e:
            self.log(f"reset failed: {e}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mock", action="store_true")
    ap.add_argument("--poses", default=os.path.join(HERE, "poses.json"))
    a = ap.parse_args()
    cfg = load_config()
    store = PoseStore(a.poses)
    miss = store.missing()
    if miss:
        print(f"WARNING: {len(miss)} poses not recorded (cells needing them are disabled). "
              f"Run: python3 record.py --status")
    bus = mock_bus(time_scale=5)[0] if a.mock else open_bus(cfg)
    arm = Arm(bus, store, cfg)
    bad = arm.check_servos()
    if bad:
        print(f"Servos not answering: {bad}")
    Game(arm, store).mainloop()


if __name__ == "__main__":
    main()
