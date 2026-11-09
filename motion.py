"""Motion layer: every game scenario is a PLAN (list of full 5-joint waypoints),
built from recorded poses. The same plan is
  * executed on the arm here (synchronised moves, grip check, e-stop), and
  * exported as an official pose_sequence file (see export.py) for 05_waypoint_sequence.py.

  pick(block)         safe -> above -> grip(open) -> CLOSE -> above -> safe
  place(block, cell)  rack_safe -> front -> in -> RELEASE -> front -> rack_safe -> safe
  turn(block, cell)   pick + place
  attack(cell)        safe -> rack_safe -> front -> hit -> front -> rack_safe -> safe
"""
from __future__ import annotations

import json
import os
import threading
import time
from dataclasses import dataclass, field

import official
from poses import GRIPPER, JOINT_IDS, PoseStore
from servo_bus import ServoBus, ServoError, ServoTimeout

HERE = os.path.dirname(os.path.abspath(__file__))

DEFAULT_CONFIG = {
    "port": "auto",          # taken from the official arm_settings.py when found
    "baud": 115200,
    "write_replies": True,   # servos send a status after WRITE (web tool assumes yes)
    "speed": 700,            # ticks/s for the joint that has the furthest to go
    "slow_speed": 250,       # final 'in' / 'grip' approach
    "attack_speed": 1500,    # single fast push (legacy record.py/game.py path)
    "max_speed": 3400,       # 'max' steps (attack HIT): above the ST3215's ~3000 ticks/s top speed = flat out
    "max_acc": 254,          # 'max' steps: highest acceleration (same as official MOVE_ACCELERATION)
    "fluid_speed": 1100,     # peak ticks/s of smooth (fluid) playback; GUI slider scales it
    "stream_hz": 50,         # how often fluid playback updates the servo targets
    "record_hz": 50,         # samples per second when recording a fluid take
    "stream_acc": 150,       # servo acceleration while streaming
    "acc": 30,
    "arrive_tolerance": 15,  # ticks (~1.3 deg)
    "move_timeout_s": 6.0,
    "grip_squeeze": 40,      # extra ticks past the recorded grip value to hold firmly
    "grip_load_threshold": 120,
    "gripper_close_direction": 1,
    "open_settle_s": 1.5,
    "joint_limits": {str(j): [0, 4095] for j in JOINT_IDS},
}


def load_config(path=os.path.join(HERE, "arm_config.json"), use_official=True) -> dict:
    """defaults  <  official arm_settings.py  <  our arm_config.json"""
    cfg = json.loads(json.dumps(DEFAULT_CONFIG))
    cfg["official_repo"] = None
    if use_official:
        s = official.load_settings()
        if s is not None:
            cfg.update(official.config_overrides(s))
            cfg["official_repo"] = str(s.REPO)
    if os.path.exists(path):
        with open(path) as f:
            user = json.load(f)
        limits = user.pop("joint_limits", {})
        cfg.update(user)
        cfg["joint_limits"] = dict(cfg["joint_limits"], **limits)
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
    raise SystemExit("No serial port found. Plug in the ESP32 USB cable or set SERIAL_PORT in arm_settings.py")


def open_bus(cfg) -> ServoBus:
    return ServoBus(find_port(cfg), cfg["baud"], cfg["write_replies"],
                    settle_s=cfg.get("open_settle_s", 1.5))


class EStop(Exception):
    """Raised inside a sequence when the emergency stop is pressed."""


class LimitError(Exception):
    pass


@dataclass
class Step:
    label: str                       # pose name (shown as "-> label") or action
    joints: dict = field(default_factory=dict)   # full 5-joint target
    kind: str = "move"               # move | grip | release
    speed: str = "speed"             # config key
    holding: bool = False            # a block is in the jaws -> gripper may stall


# ============================================================================ plans
def gripper_values(store: PoseStore, cfg: dict, block: str) -> tuple[int, int]:
    """(open, closed) gripper ticks for this block. Closed = recorded grip + squeeze."""
    opn = store.gripper_open_value
    grip = store.get(f"pick/{block}/grip")[GRIPPER]
    d = 1 if grip > opn else -1 if grip < opn else int(cfg["gripper_close_direction"])
    return opn, max(0, min(4095, grip + d * cfg["grip_squeeze"]))


def _with(pose: dict, g: int) -> dict:
    p = dict(pose)
    p[GRIPPER] = g
    return p


def plan_pick(store, cfg, block):
    o, c = gripper_values(store, cfg, block)
    g = store.get
    grip = _with(g(f"pick/{block}/grip"), o)
    return [
        Step("safe", _with(g("safe"), o)),
        Step(f"pick/{block}/above", _with(g(f"pick/{block}/above"), o)),
        Step(f"pick/{block}/grip", grip, speed="slow_speed"),
        Step("close", _with(grip, c), kind="grip", holding=True),
        Step(f"pick/{block}/above", _with(g(f"pick/{block}/above"), c), speed="slow_speed", holding=True),
        Step("safe", _with(g("safe"), c), holding=True),
    ]


def plan_place(store, cfg, block, cell):
    o, c = gripper_values(store, cfg, block)
    g = store.get
    inside = g(f"place/{block}/{cell}/in")
    return [
        Step("rack_safe", _with(g("rack_safe"), c), holding=True),
        Step(f"place/{block}/{cell}/front", _with(g(f"place/{block}/{cell}/front"), c), holding=True),
        Step(f"place/{block}/{cell}/in", _with(inside, c), speed="slow_speed", holding=True),
        Step("release", _with(inside, o), kind="release"),
        Step(f"place/{block}/{cell}/front", _with(g(f"place/{block}/{cell}/front"), o), speed="slow_speed"),
        Step("rack_safe", _with(g("rack_safe"), o)),
        Step("safe", _with(g("safe"), o)),
    ]


def plan_turn(store, cfg, block, cell):
    return plan_pick(store, cfg, block) + plan_place(store, cfg, block, cell)


def plan_attack(store, cfg, cell):
    g = store.get
    return [
        Step("safe", g("safe")),
        Step("rack_safe", g("rack_safe")),
        Step(f"attack/{cell}/front", g(f"attack/{cell}/front")),
        Step(f"attack/{cell}/hit", g(f"attack/{cell}/hit"), speed="attack_speed"),
        Step(f"attack/{cell}/front", g(f"attack/{cell}/front"), speed="slow_speed"),
        Step("rack_safe", g("rack_safe")),   # fully out of TTTZ -> attack attempt ends (3.3.4.9)
        Step("safe", g("safe")),
    ]


# ============================================================================ arm
class Arm:
    def __init__(self, bus: ServoBus, store: PoseStore, cfg: dict | None = None, log=print):
        self.bus = bus
        self.store = store
        self.cfg = cfg or load_config()
        self.log = log
        self.estop = threading.Event()
        self.gripper_goal = None
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
        """Latch + cut torque on every joint (same idea as the official Space key)."""
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
        self.bus.sync_write_positions({j: (cur[j], 100, 10) for j in JOINT_IDS})
        self.torque(True)

    def _guard(self):
        if self.estop.is_set():
            raise EStop()

    def check_limits(self, target):
        for j, v in target.items():
            lo, hi = self.cfg["joint_limits"][str(j)]
            if not lo <= v <= hi:
                raise LimitError(f"joint {j} target {v} outside limit {lo}-{hi}")

    # ------------------------------------------------------------------ moving
    def move_to(self, target: dict, speed=None, wait=True, ignore=(), acc=None):
        """Move all joints together so they ARRIVE at the same time."""
        self._guard()
        target = {int(j): int(v) for j, v in target.items()}
        self.check_limits(target)
        speed = speed or self.cfg["speed"]
        acc = self.cfg["acc"] if acc is None else acc
        try:
            cur = self.read_all()
        except ServoTimeout:
            cur = dict(target)
        dist = {j: abs(target[j] - cur.get(j, target[j])) for j in target}
        far = max(dist.values()) or 1
        cmd = {j: (target[j], max(30, int(speed * dist[j] / far)), acc) for j in target}
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
            if last == pos:   # stalled close to the target (block in the way) -> accept
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

    def go(self, name: str, keep_gripper=False, gripper=None, speed=None):
        """Move to one recorded pose (used by the recorder and recovery buttons)."""
        target = self.store.get(name)
        if gripper is not None:
            target[GRIPPER] = gripper
        elif keep_gripper and self.gripper_goal is not None:
            target[GRIPPER] = self.gripper_goal
        self.log(f"  -> {name}")
        self.move_to(target, speed=speed, ignore=(GRIPPER,) if keep_gripper else ())

    def open_gripper(self):
        self.move_to({GRIPPER: self.store.gripper_open_value}, speed=800, ignore=(GRIPPER,))

    def feel_block(self, goal: int) -> bool:
        time.sleep(0.25)
        self._guard()
        try:
            load = abs(self.bus.read_load(GRIPPER))
            pos = self.bus.read_position(GRIPPER)
        except ServoError:
            return True
        holding = load >= self.cfg["grip_load_threshold"] or abs(pos - goal) > self.cfg["arrive_tolerance"]
        if not holding:
            self.log(f"  ? gripper does not feel a block (load {load}, pos {pos} vs {goal})")
        return holding

    def run_plan(self, steps: list[Step]) -> bool:
        """Execute a plan. Returns False if a grip step felt no block."""
        ok = True
        for st in steps:
            if st.kind == "move":
                self.log(f"  -> {st.label}")
                self.move_to(st.joints, speed=self.cfg[st.speed],
                             ignore=(GRIPPER,) if st.holding else ())
            elif st.kind == "grip":
                self.move_to({GRIPPER: st.joints[GRIPPER]}, speed=500, ignore=(GRIPPER,))
                ok = self.feel_block(st.joints[GRIPPER]) and ok
            elif st.kind == "release":
                self.move_to({GRIPPER: st.joints[GRIPPER]}, speed=800, ignore=(GRIPPER,))
                time.sleep(0.15)
        return ok

    # --------------------------------------------------------------- sequences
    def _exclusive(fn):
        def wrapped(self, *a, **kw):
            if not self.busy.acquire(blocking=False):
                raise RuntimeError("arm is already running a sequence")
            try:
                return fn(self, *a, **kw)
            finally:
                self.busy.release()
        wrapped.__name__ = fn.__name__
        return wrapped

    @_exclusive
    def pick(self, block) -> bool:
        self.log(f"PICK {block}")
        return self.run_plan(plan_pick(self.store, self.cfg, block))

    @_exclusive
    def place(self, block, cell):
        self.log(f"PLACE {block} -> {cell}")
        self.run_plan(plan_place(self.store, self.cfg, block, cell))

    @_exclusive
    def turn(self, block, cell) -> bool:
        """Full scoring move: pick the block and put it in the cell."""
        self.log(f"TURN {block} -> {cell}")
        ok = self.run_plan(plan_pick(self.store, self.cfg, block))
        if not ok:
            self.log("  ! no block felt in gripper - placing anyway; check it")
        self.run_plan(plan_place(self.store, self.cfg, block, cell))
        return ok

    @_exclusive
    def attack(self, cell, authorised: bool):
        """ONE push through `cell`. Only call after the referee authorises (rule 3.3.4.4)."""
        if not authorised:
            raise PermissionError("Attack not authorised by referee")
        self.log(f"ATTACK {cell}")
        self.run_plan(plan_attack(self.store, self.cfg, cell))

    @_exclusive
    def stow(self):
        self.go("safe", keep_gripper=True)
        self.go("stow", keep_gripper=True)
