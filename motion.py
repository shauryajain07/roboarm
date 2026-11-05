"""Motion layer: synchronised moves between recorded poses + the game sequences.

  pick(block)            safe -> above -> grip -> close -> above -> safe
  place(block, cell)     safe -> rack_safe -> front -> in -> OPEN -> front -> rack_safe -> safe
  turn(block, cell)      pick + place
  attack(cell)           safe -> rack_safe -> front -> hit -> front -> rack_safe -> safe
  stow()                 safe -> stow

Every waypoint is a recorded pose, so every one of the 3x9 placements and
9 attacks is covered as soon as its poses exist (see poses.checklist()).
"""
from __future__ import annotations

import json
import os
import threading
import time

from poses import ARM_JOINTS, GRIPPER, JOINT_IDS, PoseStore
from servo_bus import ServoBus, ServoError, ServoTimeout

HERE = os.path.dirname(os.path.abspath(__file__))

DEFAULT_CONFIG = {
    "port": "auto",          # auto -> first of /dev/ttyUSB*, /dev/ttyACM*, /dev/serial0
    "baud": 115200,          # course web tool default; 1000000 if wired straight to the servo bus
    "write_replies": True,   # servos send a status after WRITE (web tool assumes yes)
    "speed": 700,            # ticks/s for the joint that has the furthest to go
    "slow_speed": 250,       # used for the final 'in' / 'grip' approach
    "attack_speed": 1500,    # single fast push
    "acc": 30,
    "arrive_tolerance": 15,  # ticks (~1.3 deg)
    "move_timeout_s": 6.0,
    "grip_squeeze": 40,      # extra ticks past the recorded grip value to hold firmly
    "grip_load_threshold": 120,
    "gripper_close_direction": 1,   # +1 if bigger ticks = jaws closing, else -1 (check with g+)
    "joint_limits": {str(j): [0, 4095] for j in JOINT_IDS},
}


def load_config(path=os.path.join(HERE, "arm_config.json")) -> dict:
    cfg = json.loads(json.dumps(DEFAULT_CONFIG))
    if os.path.exists(path):
        with open(path) as f:
            user = json.load(f)
        cfg.update({k: v for k, v in user.items() if k != "joint_limits"})
        cfg["joint_limits"].update(user.get("joint_limits", {}))
    return cfg


def find_port(cfg) -> str:
    if cfg["port"] != "auto":
        return cfg["port"]
    import glob
    for pat in ("/dev/ttyUSB*", "/dev/ttyACM*", "/dev/tty.usbserial*", "/dev/tty.usbmodem*",
                "/dev/serial0"):
        hits = sorted(glob.glob(pat))
        if hits:
            return hits[0]
    raise SystemExit("No serial port found. Plug in the servo board USB or set 'port' in arm_config.json")


def open_bus(cfg) -> ServoBus:
    return ServoBus(find_port(cfg), cfg["baud"], cfg["write_replies"])


class EStop(Exception):
    """Raised inside a sequence when the emergency stop is pressed."""


class LimitError(Exception):
    pass


class Arm:
    def __init__(self, bus: ServoBus, store: PoseStore, cfg: dict | None = None, log=print):
        self.bus = bus
        self.store = store
        self.cfg = cfg or load_config()
        self.log = log
        self.estop = threading.Event()
        self.gripper_goal: int | None = None
        self.busy = threading.Lock()

    # ------------------------------------------------------------------ basics
    def check_servos(self) -> list[int]:
        return [j for j in JOINT_IDS if not self.bus.ping(j)]

    def read_all(self) -> dict[int, int]:
        return {j: self.bus.read_position(j) for j in JOINT_IDS}

    def torque(self, on: bool, joints=JOINT_IDS):
        for j in joints:
            self.bus.set_torque(j, on)

    def emergency_stop(self):
        """Latch + cut torque on every joint (same idea as the web tool's Space key)."""
        self.estop.set()
        for j in JOINT_IDS:
            for _ in range(2):
                try:
                    self.bus.set_torque(j, False)
                    break
                except ServoError:
                    continue
        self.log("!!! EMERGENCY STOP - torque OFF. Reset before moving again.")

    def reset_estop(self):
        self.estop.clear()
        cur = self.read_all()
        self.gripper_goal = cur[GRIPPER]
        # re-enable holding at the CURRENT position so nothing jumps
        self.bus.sync_write_positions({j: (cur[j], 100, 10) for j in JOINT_IDS})
        self.torque(True)

    def _guard(self):
        if self.estop.is_set():
            raise EStop()

    def _check_limits(self, target):
        for j, v in target.items():
            lo, hi = self.cfg["joint_limits"][str(j)]
            if not lo <= v <= hi:
                raise LimitError(f"joint {j} target {v} outside limit {lo}-{hi}")

    # ------------------------------------------------------------------ moving
    def move_to(self, target: dict[int, int], speed=None, wait=True, ignore=()):
        """Move all joints together so they ARRIVE at the same time."""
        self._guard()
        target = {int(j): int(v) for j, v in target.items()}
        self._check_limits(target)
        speed = speed or self.cfg["speed"]
        try:
            cur = self.read_all()
        except ServoTimeout:
            cur = {j: target[j] for j in target}
        dist = {j: abs(target[j] - cur.get(j, target[j])) for j in target}
        far = max(dist.values()) or 1
        cmd = {j: (target[j], max(30, int(speed * dist[j] / far)), self.cfg["acc"])
               for j in target}
        self.bus.sync_write_positions(cmd)
        if GRIPPER in target:
            self.gripper_goal = target[GRIPPER]
        if wait:
            self._wait(target, ignore)

    def _wait(self, target, ignore=()):
        tol = self.cfg["arrive_tolerance"]
        deadline = time.monotonic() + self.cfg["move_timeout_s"]
        last, still_since = None, None
        while True:
            self._guard()
            pos = {j: self.bus.read_position(j) for j in target if j not in ignore}
            err = {j: abs(pos[j] - target[j]) for j in pos}
            if all(e <= tol for e in err.values()):
                return pos
            # stalled close to the target (e.g. block in the way) -> accept
            if last == pos:
                still_since = still_since or time.monotonic()
                if time.monotonic() - still_since > 0.4 and max(err.values()) < 4 * tol:
                    return pos
            else:
                still_since = None
            last = pos
            if time.monotonic() > deadline:
                bad = {j: (pos[j], target[j]) for j in err if err[j] > tol}
                raise ServoError(f"move timed out, joints not there (pos, goal): {bad}")
            time.sleep(0.03)

    def go(self, name: str, keep_gripper=False, gripper: int | None = None, speed=None):
        target = self.store.get(name)
        if gripper is not None:
            target[GRIPPER] = gripper
        elif keep_gripper and self.gripper_goal is not None:
            target[GRIPPER] = self.gripper_goal
        self.log(f"  -> {name}")
        self.move_to(target, speed=speed, ignore=(GRIPPER,) if keep_gripper else ())

    def open_gripper(self):
        self.move_to({GRIPPER: self.store.gripper_open_value}, speed=800)

    def close_gripper(self, grip_value: int) -> bool:
        """Close to the recorded grip value (+ squeeze). Returns True if it feels a block."""
        opn = self.store.gripper_open_value
        direction = 1 if grip_value > opn else -1
        goal = max(0, min(4095, grip_value + direction * self.cfg["grip_squeeze"]))
        self.move_to({GRIPPER: goal}, speed=500, ignore=(GRIPPER,))
        time.sleep(0.25)
        self._guard()
        try:
            load = abs(self.bus.read_load(GRIPPER))
            pos = self.bus.read_position(GRIPPER)
        except ServoError:
            return True
        stopped_short = abs(pos - goal) > self.cfg["arrive_tolerance"]
        holding = load >= self.cfg["grip_load_threshold"] or stopped_short
        if not holding:
            self.log(f"  ? gripper does not feel a block (load {load}, pos {pos} vs {goal})")
        return holding

    # --------------------------------------------------------------- sequences
    def _sequence(fn):
        def wrapped(self, *a, **kw):
            if not self.busy.acquire(blocking=False):
                raise RuntimeError("arm is already running a sequence")
            try:
                return fn(self, *a, **kw)
            finally:
                self.busy.release()
        wrapped.__name__ = fn.__name__
        return wrapped

    def _pick(self, block) -> bool:
        s = self.store
        opn = s.gripper_open_value
        self.log(f"PICK {block}")
        self.go("safe", gripper=opn)
        self.go(f"pick/{block}/above", gripper=opn)
        self.go(f"pick/{block}/grip", gripper=opn, speed=self.cfg["slow_speed"])
        ok = self.close_gripper(s.get(f"pick/{block}/grip")[GRIPPER])
        self.go(f"pick/{block}/above", keep_gripper=True, speed=self.cfg["slow_speed"])
        self.go("safe", keep_gripper=True)
        return ok

    def _place(self, block, cell):
        self.log(f"PLACE {block} -> {cell}")
        self.go("rack_safe", keep_gripper=True)
        self.go(f"place/{block}/{cell}/front", keep_gripper=True)
        self.go(f"place/{block}/{cell}/in", keep_gripper=True, speed=self.cfg["slow_speed"])
        self.open_gripper()
        time.sleep(0.15)
        self.go(f"place/{block}/{cell}/front", gripper=self.store.gripper_open_value,
                speed=self.cfg["slow_speed"])
        self.go("rack_safe", keep_gripper=True)
        self.go("safe", keep_gripper=True)

    @_sequence
    def pick(self, block) -> bool:
        return self._pick(block)

    @_sequence
    def place(self, block, cell):
        self._place(block, cell)

    @_sequence
    def turn(self, block, cell) -> bool:
        """Full scoring move: pick the block and put it in the cell."""
        ok = self._pick(block)
        if not ok:
            self.log("  ! no block felt in gripper - placing anyway; check it")
        self._place(block, cell)
        return ok

    @_sequence
    def attack(self, cell, authorised: bool):
        """ONE push through `cell`. Only call after the referee authorises (rule 3.3.4.4)."""
        if not authorised:
            raise PermissionError("Attack not authorised by referee")
        self.log(f"ATTACK {cell}")
        self.go("safe")
        self.go("rack_safe")
        self.go(f"attack/{cell}/front")
        self.go(f"attack/{cell}/hit", speed=self.cfg["attack_speed"])
        self.go(f"attack/{cell}/front", speed=self.cfg["slow_speed"])
        self.go("rack_safe")      # fully out of TTTZ -> attack attempt ends (3.3.4.9)
        self.go("safe")

    @_sequence
    def stow(self):
        self.go("safe", keep_gripper=True)
        self.go("stow", keep_gripper=True)
