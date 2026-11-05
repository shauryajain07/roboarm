"""Fluid motion: record a hand-guided demo, clean it, and stream it back smoothly.

  record()          sample all 5 joints with timestamps while you move the arm by hand
  clean()           de-glitch, trim idle, smooth hand shake, shorten hesitations,
                    keep settle time around gripper changes, snap ends to SAFE
  from_waypoints()  turn a waypoint scenario into one smooth curve (no stop-and-go):
                    stops only where precision matters, glides through the rest
  play()            stream the curve to the servos at ~40 Hz (E-stop checked every tick)

Files use the OFFICIAL 04_timestamp_motion.py layout (name, sampling_interval_seconds,
initial_position, steps[elapsed_seconds, positions_raw]) plus our own extras.
Pure Python - no numpy needed on the Pi.
"""
from __future__ import annotations

import json
import math
import os
import time
from dataclasses import dataclass, field
from pathlib import Path

from poses import ARM_JOINTS, GRIPPER

DEFAULT_NAMES = {1: "base_yaw", 2: "shoulder", 3: "elbow", 4: "wrist", 5: "gripper"}

CLEAN_DEFAULTS = {
    "dt": 0.02,                 # resample to 50 Hz
    "idle_ticks": 8,            # movement smaller than this = standing still
    "smooth_sigma_s": 0.08,     # gaussian smoothing width (removes hand shake, keeps shape)
    "pause_speed": 20,          # ticks/s: slower than this counts as a pause
    "pause_min_s": 0.5,         # pauses longer than this get shortened ...
    "pause_keep_s": 0.2,        # ... down to this
    "grip_settle_s": 0.35,      # always keep this much stillness around a gripper change
    "grip_change_ticks": 25,    # gripper moving more than this = a grab / release event
    "snap_s": 0.4,              # blend the first/last 0.4 s onto the exact shared pose
}


@dataclass
class Traj:
    t: list = field(default_factory=list)
    q: list = field(default_factory=list)
    meta: dict = field(default_factory=dict)

    @property
    def duration(self) -> float:
        return self.t[-1] - self.t[0] if self.t else 0.0

    def __len__(self):
        return len(self.t)

    def at(self, t: float) -> dict:
        """Linear interpolation at time t (clamped)."""
        ts = self.t
        if t <= ts[0]:
            return dict(self.q[0])
        if t >= ts[-1]:
            return dict(self.q[-1])
        lo, hi = 0, len(ts) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if ts[mid] <= t:
                lo = mid
            else:
                hi = mid
        a = (t - ts[lo]) / (ts[hi] - ts[lo]) if ts[hi] > ts[lo] else 0
        return {j: self.q[lo][j] + a * (self.q[hi][j] - self.q[lo][j]) for j in self.q[lo]}

    def rounded(self, k) -> dict:
        return {j: int(round(v)) for j, v in self.q[k].items()}


# ============================================================================ record
def record(read_all, stop_event, hz: float = 25.0, on_sample=None, max_s: float = 60.0) -> Traj:
    """Sample positions until stop_event is set. read_all() -> {id: ticks}."""
    tr = Traj(meta={"source": "recorded", "hz": hz})
    t0 = time.monotonic()
    period = 1.0 / hz
    while not stop_event.is_set():
        tick = time.monotonic()
        try:
            pose = read_all()
        except Exception:  # noqa: BLE001 - a dropped read just skips one sample
            pose = None
        if pose:
            tr.t.append(tick - t0)
            tr.q.append({int(j): float(v) for j, v in pose.items()})
            if on_sample:
                on_sample(len(tr), tick - t0)
        if tick - t0 > max_s:
            break
        time.sleep(max(0.0, period - (time.monotonic() - tick)))
    return tr


# ============================================================================ clean
def _median3(xs):
    if len(xs) < 3:
        return list(xs)
    return [xs[0]] + [sorted(xs[i - 1:i + 2])[1] for i in range(1, len(xs) - 1)] + [xs[-1]]


def _gauss(xs, sigma_n):
    if sigma_n < 0.5 or len(xs) < 3:
        return list(xs)
    r = int(3 * sigma_n)
    w = [math.exp(-0.5 * (k / sigma_n) ** 2) for k in range(-r, r + 1)]
    out = []
    n = len(xs)
    for i in range(n):
        s = ws = 0.0
        for k in range(-r, r + 1):
            j = min(n - 1, max(0, i + k))     # hold the ends (no shrink)
            s += w[k + r] * xs[j]
            ws += w[k + r]
        out.append(s / ws)
    return out


def _resample(tr: Traj, dt: float) -> Traj:
    if len(tr) < 2:
        return Traj(list(tr.t), [dict(p) for p in tr.q], dict(tr.meta))
    t0 = tr.t[0]
    n = int((tr.t[-1] - t0) / dt + 1e-9) + 1
    out = Traj(meta=dict(tr.meta))
    for k in range(n):
        out.t.append(k * dt)
        out.q.append(tr.at(t0 + k * dt))
    if out.t[-1] < tr.t[-1] - t0 - 1e-9:
        out.t.append(tr.t[-1] - t0)
        out.q.append(dict(tr.q[-1]))
    return out


def clean(raw: Traj, start_pose=None, end_pose=None, opts=None):
    """Return (cleaned trajectory, report dict)."""
    o = dict(CLEAN_DEFAULTS, **(opts or {}))
    if len(raw) < 3:
        raise ValueError("recording too short - move the arm for at least a second")
    rep = {"raw_samples": len(raw), "raw_duration_s": round(raw.duration, 2)}
    joints = sorted(raw.q[0])

    # 1) de-glitch every joint (single bad reads)
    cols = {j: _median3([p[j] for p in raw.q]) for j in joints}
    tr = Traj(list(raw.t), [{j: cols[j][i] for j in joints} for i in range(len(raw))], dict(raw.meta))

    # 2) trim standing-still at the start and end
    def moved(i, ref):
        return any(abs(tr.q[i][j] - ref[j]) > o["idle_ticks"] for j in joints)
    first = next((i for i in range(len(tr)) if moved(i, tr.q[0])), 0)
    last = next((i for i in range(len(tr) - 1, -1, -1) if moved(i, tr.q[-1])), len(tr) - 1)
    pad = 0.15
    t_a = max(tr.t[0], tr.t[first] - pad)
    t_b = min(tr.t[-1], tr.t[last] + pad)
    if t_b - t_a < 0.3:
        raise ValueError("the arm barely moved during the recording")
    keep = [i for i in range(len(tr)) if t_a <= tr.t[i] <= t_b]
    tr = Traj([tr.t[i] for i in keep], [tr.q[i] for i in keep], tr.meta)
    rep["trimmed_s"] = round(raw.duration - tr.duration, 2)

    # 3) uniform 50 Hz, then smooth the ARM joints (gripper stays crisp)
    tr = _resample(tr, o["dt"])
    sig = o["smooth_sigma_s"] / o["dt"]
    for j in joints:
        if j == GRIPPER:
            continue
        col = _gauss([p[j] for p in tr.q], sig)
        for i, p in enumerate(tr.q):
            p[j] = col[i]

    # 4) shorten hesitations (time-warp), but keep settle time around gripper events
    dt = o["dt"]
    arm_j = [j for j in joints if j != GRIPPER]
    speed = [0.0] + [max(abs(tr.q[i][j] - tr.q[i - 1][j]) for j in arm_j) / dt for i in range(1, len(tr))]
    g = [p.get(GRIPPER, 0) for p in tr.q]
    lag = max(1, int(0.3 / dt))
    grip_ev = [i for i in range(1, len(tr)) if abs(g[i] - g[max(0, i - lag)]) > o["grip_change_ticks"]]
    near_grip = set()
    settle_n = int(o["grip_settle_s"] / dt)
    for i in grip_ev:
        near_grip.update(range(max(0, i - settle_n), min(len(tr), i + settle_n)))
    still = [s < o["pause_speed"] and i not in near_grip for i, s in enumerate(speed)]
    new_dt = [dt] * len(tr)
    i, removed = 0, 0.0
    while i < len(tr):
        if still[i]:
            k = i
            while k < len(tr) and still[k]:
                k += 1
            span = (k - i) * dt
            if span > o["pause_min_s"]:
                f = o["pause_keep_s"] / span
                for m in range(i, k):
                    new_dt[m] = dt * f
                removed += span - o["pause_keep_s"]
            i = k
        else:
            i += 1
    t, acc = [], 0.0
    for k in range(len(tr)):
        acc += new_dt[k] if k else 0.0
        t.append(acc)
    tr.t = t
    tr = _resample(tr, dt)
    rep["pauses_removed_s"] = round(removed, 2)
    rep["grip_events"] = len(_cluster(grip_ev, lag))

    # 5) snap the ends onto the exact shared poses so scenarios chain seamlessly
    n_snap = max(1, int(o["snap_s"] / dt))
    for pose, idx in ((start_pose, range(0, min(n_snap, len(tr)))),
                      (end_pose, range(len(tr) - 1, max(-1, len(tr) - 1 - n_snap), -1))):
        if not pose:
            continue
        for rank, i in enumerate(idx):
            w = 1.0 - rank / n_snap
            w = w * w * (3 - 2 * w)                       # smoothstep
            for j in joints:
                tr.q[i][j] = w * pose[j] + (1 - w) * tr.q[i][j]

    tr.meta = dict(raw.meta, cleaned=True)
    rep.update(clean_duration_s=round(tr.duration, 2), samples=len(tr), max_speed=int(max_speed(tr)))
    return tr, rep


def _cluster(idxs, gap):
    groups, last = [], None
    for i in idxs:
        if last is None or i - last > gap:
            groups.append(i)
        last = i
    return groups


def max_speed(tr: Traj) -> float:
    if len(tr) < 2:
        return 0.0
    return max(abs(tr.q[i][j] - tr.q[i - 1][j]) / max(1e-6, tr.t[i] - tr.t[i - 1])
               for i in range(1, len(tr)) for j in tr.q[i] if j != GRIPPER)


# ============================================================================ waypoints -> curve
def from_waypoints(points, stops, speeds, dt: float = 0.02, min_seg_s: float = 0.25) -> Traj:
    """Smooth C1 curve through waypoints.
    stops[i]  : come to rest at point i (ends, grab, in-hole, hit, gripper change)
    speeds[i] : ticks/s for the segment ARRIVING at point i
    Pass-through points get Catmull-Rom tangents -> the arm glides instead of stopping.
    Into/out of stops: cubic Hermite with zero end velocity (smooth ease in/out)."""
    n = len(points)
    if n < 2:
        raise ValueError("need at least 2 points")
    joints = sorted(points[0])
    rest = [bool(stops[i]) or i == 0 or i == n - 1 for i in range(n)]
    seg_T = []
    for i in range(1, n):
        d = max(abs(points[i][j] - points[i - 1][j]) for j in joints if j != GRIPPER)
        dg = abs(points[i][GRIPPER] - points[i - 1][GRIPPER]) if GRIPPER in joints else 0
        # peak speed of an ease-in/out segment is 1.5x its average; gliding segments ~1x
        k = 1.0 + 0.25 * rest[i - 1] + 0.25 * rest[i]
        floor = 0.06 if speeds[i] >= 2000 else min_seg_s     # fast punch keeps its speed
        seg_T.append(max(floor, k * d / max(1.0, speeds[i]), dg / 900.0))
    vel = []
    for i in range(n):
        if stops[i] or i == 0 or i == n - 1:
            vel.append({j: 0.0 for j in joints})
            continue
        Tsum = seg_T[i - 1] + seg_T[i]
        vel.append({j: 0.0 if j == GRIPPER else (points[i + 1][j] - points[i - 1][j]) / Tsum for j in joints})
    tr = Traj(meta={"source": "waypoints"})
    t_base = 0.0
    for s in range(n - 1):
        T = seg_T[s]
        p0, p1, v0, v1 = points[s], points[s + 1], vel[s], vel[s + 1]
        steps = max(1, int(round(T / dt)))
        for k in range(0 if s == 0 else 1, steps + 1):
            u = k / steps
            h00 = 2 * u ** 3 - 3 * u ** 2 + 1
            h10 = u ** 3 - 2 * u ** 2 + u
            h01 = -2 * u ** 3 + 3 * u ** 2
            h11 = u ** 3 - u ** 2
            tr.t.append(t_base + u * T)
            tr.q.append({j: h00 * p0[j] + h10 * T * v0[j] + h01 * p1[j] + h11 * T * v1[j] for j in joints})
        t_base += T
    tr.meta["seg_T"] = [round(x, 3) for x in seg_T]
    return tr


# ============================================================================ play
OVERSHOOT_TICKS = 30   # smooth curves may bulge a little past a limit near the edge


def _clamp_overshoot(target: dict, cfg: dict) -> dict:
    """Clamp small curve overshoots to the joint limit; bigger ones are left for
    check_limits() to refuse (a real problem, not a curve artifact)."""
    out = {}
    for j, v in target.items():
        lo, hi = cfg["joint_limits"][str(j)]
        if lo - OVERSHOOT_TICKS <= v < lo:
            v = lo
        elif hi < v <= hi + OVERSHOOT_TICKS:
            v = hi
        out[j] = v
    return out


def play(arm, tr: Traj, rate: float = 1.0, hz: float = 40.0, ignore_gripper: bool = False, log=None):
    """Stream the trajectory. rate 2.0 = twice as fast. Raises EStop via arm._guard()."""
    cfg = arm.cfg
    tol = cfg["arrive_tolerance"]
    first = tr.rounded(0)
    try:
        cur = arm.read_all()
        if max(abs(cur[j] - first[j]) for j in ARM_JOINTS) > 3 * tol:
            if log:
                log("  moving to the start of the motion")
            arm.move_to(first, speed=cfg["slow_speed"] * 2, ignore=(GRIPPER,))
    except Exception as e:  # noqa: BLE001
        if type(e).__name__ in ("EStop", "LimitError"):
            raise
    period = 1.0 / hz
    t_end = tr.t[-1] / rate
    t0 = time.monotonic()
    prev = tr.at(0)
    while True:
        arm._guard()
        now = time.monotonic() - t0
        tt = min(t_end, now + period)            # aim one tick ahead
        q = tr.at(tt * rate)
        target = _clamp_overshoot({j: int(round(v)) for j, v in q.items()}, cfg)
        arm.check_limits(target)
        cmd = {j: (v, int(min(cfg["max_speed"], abs(q[j] - prev[j]) / period * 1.3 + 40)),
                   cfg.get("stream_acc", 150)) for j, v in target.items()}
        arm.bus.sync_write_positions(cmd)
        arm.gripper_goal = target.get(GRIPPER, arm.gripper_goal)
        prev = q
        if tt >= t_end:
            break
        time.sleep(max(0.0, period - ((time.monotonic() - t0) - now)))
    arm.move_to(tr.rounded(len(tr) - 1), speed=cfg["slow_speed"],
                ignore=(GRIPPER,) if ignore_gripper else ())


# ============================================================================ files
def save(path, tr: Traj, name: str, description: str = "", names=DEFAULT_NAMES, extra=None):
    """Official 04_timestamp_motion layout (+ our extras). name must equal the file stem."""
    path = Path(path)
    to_named = lambda p: {names[j]: int(round(v)) for j, v in sorted(p.items())}
    d = {
        "name": name,
        "description": description,
        "sampling_interval_seconds": round(tr.t[1] - tr.t[0], 4) if len(tr) > 1 else 0.02,
        "initial_position": {"positions_raw": to_named(tr.q[0])},
        "steps": [{"elapsed_seconds": round(t, 3), "positions_raw": to_named(p)} for t, p in zip(tr.t, tr.q)],
        "roboarm": dict(tr.meta, **(extra or {})),
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(".tmp")
    tmp.write_text(json.dumps(d, separators=(",", ":")) + "\n")
    os.replace(tmp, path)


def load(path, names=DEFAULT_NAMES) -> Traj:
    d = json.loads(Path(path).read_text())
    ids = {v: k for k, v in names.items()}
    tr = Traj(meta=d.get("roboarm", {}))
    for s in d["steps"]:
        tr.t.append(float(s["elapsed_seconds"]))
        tr.q.append({ids[n]: float(v) for n, v in s["positions_raw"].items()})
    return tr


def save_raw(path, tr: Traj):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps({"meta": tr.meta, "t": [round(x, 4) for x in tr.t],
                                "q": [{str(j): int(round(v)) for j, v in p.items()} for p in tr.q]},
                               separators=(",", ":")))


def load_raw(path) -> Traj:
    d = json.loads(Path(path).read_text())
    return Traj(list(d["t"]), [{int(j): float(v) for j, v in p.items()} for p in d["q"]], d.get("meta", {}))
