#!/usr/bin/env python3
"""ROBOARM pose recorder - run this ON THE RASPBERRY PI (SSH is fine).

It walks you through every pose needed for all game cases:
  globals (4) + pick 3 blocks (6) + attack 9 cells (18) + place 3 blocks x 9 cells (54)
and saves after EVERY capture to poses.json, so you can stop and resume any time.

Usage
  python3 record.py                 # record whatever is still missing, in order
  python3 record.py --all           # walk through everything (re-record / check)
  python3 record.py --only place/trophy      # just one group / prefix
  python3 record.py --redo place/cube/r2c3/in  attack/r1c1/hit
  python3 record.py --status        # show what's recorded / missing
  python3 record.py --verify attacks | place:cube | picks | all-slow
  python3 record.py --mock          # practice with a simulated arm (no hardware)

Two-person workflow: one person holds/moves the arm, the other types.
"""
from __future__ import annotations

import argparse
import os
import sys
import time

from motion import HERE, Arm, EStop, load_config, open_bus
from poses import (ARM_JOINTS, BLOCKS, CELLS, DEFAULT_SUPPLY, GRIPPER, JOINT_IDS,
                   JOINT_NAMES, PoseStore, checklist)
from servo_bus import ServoError, mock_bus

HELP = """
  [Enter]/c  CAPTURE current position (then the arm holds there)
  l          LIMP arm joints 1-4 so you can move it by hand (HOLD THE ARM FIRST!)
  h          HOLD: torque back on at the current position
  s          move to the SEED pose (a similar pose already recorded) - then fine tune
  j<N> <d>   nudge joint N by d ticks, e.g.  j2 +30   j1 -10   (4096 ticks = 360 deg)
  o          open gripper          x   close gripper until it feels the block
  g+ / g-    nudge gripper 25 ticks (g+ 60 for a bigger step)
  t          TEST: go to 'safe' and come back to this pose
  go <pose>  move to any recorded pose        p   print positions
  n          skip (next)    b  back    ?  help    q  save & quit
  !          EMERGENCY STOP (torque off)       reset   clear the stop
"""

C = {"b": "\033[1m", "g": "\033[32m", "y": "\033[33m", "r": "\033[31m", "c": "\033[36m", "0": "\033[0m"}


def col(k, s):
    return f"{C[k]}{s}{C['0']}" if sys.stdout.isatty() else s


class Recorder:
    def __init__(self, arm: Arm, store: PoseStore, cfg: dict, mock=None):
        self.arm, self.store, self.cfg, self.mock = arm, store, cfg, mock
        self.held_block: str | None = None

    # ------------------------------------------------------------- helpers
    def hold(self):
        cur = self.arm.read_all()
        self.arm.bus.sync_write_positions({j: (cur[j], 150, 20) for j in JOINT_IDS})
        self.arm.torque(True)
        self.arm.gripper_goal = cur[GRIPPER]
        return cur

    def limp(self):
        self.arm.torque(False, ARM_JOINTS)   # gripper keeps holding the block
        print(col("y", "  arm joints 1-4 LIMP - move it by hand, then press Enter to capture"))

    def show(self, pos):
        print("  " + "  ".join(f"{j}:{JOINT_NAMES[j]}={pos[j]}" for j in JOINT_IDS))

    def nudge(self, j, d):
        cur = self.arm.read_all()
        cur[j] = max(0, min(4095, cur[j] + d))
        self.arm.torque(True, [j])
        self.arm.move_to({j: cur[j]}, speed=200, wait=(j != GRIPPER))
        if j == GRIPPER:          # gripper may stall on a block, so just give it time
            time.sleep(0.4)
        self.show(self.arm.read_all())

    def close_until_block(self):
        d = int(self.cfg.get("gripper_close_direction", 1))
        self.arm.torque(True, [GRIPPER])
        last = self.arm.bus.read_position(GRIPPER)
        for _ in range(60):
            goal = max(0, min(4095, last + d * 20))
            self.arm.move_to({GRIPPER: goal}, speed=300, wait=False)
            time.sleep(0.12)
            pos = self.arm.bus.read_position(GRIPPER)
            load = abs(self.arm.bus.read_load(GRIPPER))
            if load >= self.cfg["grip_load_threshold"] or abs(pos - last) < 5:
                print(col("g", f"  gripper closed on block (pos {pos}, load {load})"))
                return
            last = pos
        print(col("y", "  gripper reached travel end without feeling a block"))

    def ensure_block(self, spec):
        """Place poses must be recorded while actually holding that block."""
        want = spec.needs_block
        if self.held_block == want:
            return True
        if self.held_block and self.held_block != want:
            print(col("y", f"\n  Releasing the {self.held_block} - CATCH IT."))
            input("  Press Enter when your hand is under the gripper...")
            self.arm.go("safe", keep_gripper=True)
            self.arm.open_gripper()
            self.held_block = None
        if want is None:
            return True
        print(col("c", f"\n  Next poses need the {want.upper()} in the gripper."))
        if self.store.can_pick(want):
            print(f"  Put the {want} on {DEFAULT_SUPPLY[want]} exactly where you recorded it.")
            a = input("  Enter = robot picks it (also tests the pick)   m = I'll put it in by hand: ").strip()
            if a != "m":
                ok = self.arm.pick(want)
                print(col("g" if ok else "y", "  pick OK" if ok else "  pick: no block felt - check!"))
                self.held_block = want
                return True
        self.arm.open_gripper()
        input(f"  Place the {want} between the jaws and press Enter to close...")
        self.close_until_block()
        self.held_block = want
        return True

    # --------------------------------------------------------------- main
    def run(self, specs):
        total = len(specs)
        i = 0
        while 0 <= i < total:
            spec = specs[i]
            if spec.needs_block or self.held_block:
                self.ensure_block(spec)
            done = sum(s.name in self.store for s in specs)
            print("\n" + col("b", f"[{i + 1}/{total}]  {spec.name}") + f"   ({done}/{total} recorded)")
            print("  " + spec.instruction)
            if spec.name in self.store:
                print(col("g", "  already recorded: ") + str(self.store.get(spec.name)))
            if spec.seed_from and spec.seed_from in self.store:
                print(col("c", f"  tip: 's' moves to '{spec.seed_from}' first so you only fine-tune"))
            step = self.prompt(spec)
            if step == "quit":
                break
            i += step
        self.store.save()
        miss = self.store.missing()
        print(col("b", f"\nSaved {self.store.path}. ") +
              (col("g", "ALL POSES RECORDED!") if not miss else col("y", f"{len(miss)} still missing.")))

    def prompt(self, spec) -> int:
        while True:
            try:
                cmd = input(col("b", "  > ")).strip()
            except EOFError:
                return "quit"
            try:
                if cmd in ("", "c"):
                    pos = self.hold()
                    self.store.set(spec.name, pos)
                    self.store.save()
                    print(col("g", "  ✓ captured & saved"))
                    self.show(pos)
                    return 1
                if cmd == "l":
                    self.limp()
                elif cmd == "h":
                    self.show(self.hold())
                elif cmd == "s":
                    if spec.seed_from and spec.seed_from in self.store:
                        self.arm.go(spec.seed_from, keep_gripper=True,
                                    speed=self.cfg["slow_speed"])
                    else:
                        print("  no seed pose recorded")
                elif cmd.startswith("j") and len(cmd.split()) == 2:
                    j, d = cmd.split()
                    self.nudge(int(j[1:]), int(d))
                elif cmd.startswith("g+") or cmd.startswith("g-"):
                    n = int(cmd[2:].strip() or 25)
                    self.nudge(GRIPPER, n if cmd[1] == "+" else -n)
                elif cmd == "o":
                    if "gripper_open" in self.store:
                        self.arm.torque(True, [GRIPPER])
                        self.arm.open_gripper()
                    else:
                        print("  gripper_open not recorded yet - use g+ / g-")
                elif cmd == "x":
                    self.close_until_block()
                elif cmd == "t":
                    if spec.name not in self.store:
                        print("  capture it first")
                        continue
                    self.arm.go("safe", keep_gripper=True)
                    self.arm.go(spec.name, keep_gripper=bool(self.held_block),
                                speed=self.cfg["slow_speed"])
                    print("  back at pose - Enter to re-capture, n to accept & move on")
                elif cmd.startswith("go "):
                    self.arm.go(cmd[3:].strip(), keep_gripper=True, speed=self.cfg["slow_speed"])
                elif cmd == "p":
                    self.show(self.arm.read_all())
                elif cmd == "n":
                    return 1
                elif cmd == "b":
                    return -1
                elif cmd == "q":
                    return "quit"
                elif cmd in ("!", "stop", "estop"):
                    self.arm.emergency_stop()
                elif cmd == "reset":
                    self.arm.reset_estop()
                    print("  stop cleared, holding current position")
                elif cmd == "?":
                    print(HELP)
                else:
                    print("  ? unknown - type ? for help")
            except EStop:
                print(col("r", "  stopped (E-STOP latched). type 'reset' to continue"))
            except (ServoError, KeyError, ValueError) as e:
                print(col("r", f"  error: {e}"))


# ============================================================================
def status(store):
    specs = checklist()
    print(col("b", f"poses file: {store.path}"))
    groups = {}
    for s in specs:
        groups.setdefault(s.group, []).append(s.name in store)
    for g, v in groups.items():
        print(f"  {g:10s} {sum(v):3d}/{len(v)}")
    print(col("b", "\ncase coverage:"))
    for b in BLOCKS:
        ok = [c for c in CELLS if store.can_place(b, c)]
        print(f"  place {b:8s} pick={'Y' if store.can_pick(b) else 'n'}  cells {len(ok)}/9  "
              f"{' '.join(ok)}")
    ok = [c for c in CELLS if store.can_attack(c)]
    print(f"  attack          cells {len(ok)}/9  {' '.join(ok)}")
    miss = store.missing(specs)
    print(f"\n{len(specs) - len(miss)}/{len(specs)} recorded" +
          ("" if not miss else f"; next missing: {miss[0]}"))


def verify(arm: Arm, store: PoseStore, what: str):
    """Dry-run real sequences slowly so you SEE every case before game day."""
    arm.cfg["speed"] = min(arm.cfg["speed"], 400)
    if what in ("attacks", "all-slow"):
        for c in CELLS:
            input(f"\nAttack {c}: Enter to run (empty rack!)...")
            arm.attack(c, authorised=True)
    if what in ("picks", "all-slow"):
        for b in BLOCKS:
            input(f"\nPut {b} on {DEFAULT_SUPPLY[b]}. Enter to pick...")
            print("  felt block" if arm.pick(b) else "  NO BLOCK FELT")
            arm.go("safe", keep_gripper=True)
            input("  Catch it - Enter to open")
            arm.open_gripper()
    blocks = BLOCKS if what == "all-slow" else ([what.split(":")[1]] if what.startswith("place:") else [])
    for b in blocks:
        for c in CELLS:
            input(f"\n{b} -> {c}: put {b} on {DEFAULT_SUPPLY[b]}, then Enter...")
            arm.turn(b, c)
            input("  check it sits in the rack, remove it, Enter...")
    arm.stow()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--poses", default=os.path.join(HERE, "poses.json"))
    ap.add_argument("--all", action="store_true", help="go through every pose, not only missing")
    ap.add_argument("--only", help="only poses whose name starts with this (e.g. place/cube)")
    ap.add_argument("--redo", nargs="+", help="re-record these exact pose names")
    ap.add_argument("--status", action="store_true")
    ap.add_argument("--verify", help="attacks | picks | place:<block> | all-slow")
    ap.add_argument("--mock", action="store_true", help="simulated arm, no hardware")
    a = ap.parse_args()

    store = PoseStore(a.poses)
    if a.status:
        return status(store)

    cfg = load_config()
    mock = None
    if a.mock:
        bus, mock = mock_bus(time_scale=5)
    else:
        bus = open_bus(cfg)
    arm = Arm(bus, store, cfg)
    missing = arm.check_servos()
    if missing:
        print(col("r", f"Servos not answering: {missing}. Check power / wiring / baud ({cfg['baud']})."))
        if len(missing) == len(JOINT_IDS):
            sys.exit(1)

    try:
        if a.verify:
            return verify(arm, store, a.verify)
        specs = checklist()
        if a.redo:
            specs = [s for s in specs if s.name in a.redo]
        elif a.only:
            specs = [s for s in specs if s.name.startswith(a.only)]
        if not (a.all or a.redo):
            specs = [s for s in specs if s.name not in store] or specs
        print(col("b", f"ROBOARM recorder - {len(specs)} poses to go. Type ? for help."))
        print(col("y", "Holding the arm in its current position (torque ON)."))
        rec = Recorder(arm, store, cfg, mock)
        rec.hold()
        rec.run(specs)
    except KeyboardInterrupt:
        arm.emergency_stop()
        store.save()
        print("\nCtrl-C: torque off, poses saved.")


if __name__ == "__main__":
    main()
