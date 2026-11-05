"""Bridge to the official course repo (Kelvinchan324/STS3215-5DOF-Robot-Arm).

We read ITS arm_settings.py so both programs always agree on port, baud, servo IDs,
zero ticks, joint directions and angle limits, and we write scenarios in ITS
waypoint format so they can be played with scripts/05_waypoint_sequence.py.

Repo location (first match wins):
  $ROBOARM_OFFICIAL, ~/robot-arm-lab/STS3215-5DOF-Robot-Arm, ../STS3215-5DOF-Robot-Arm,
  ../repos/STS3215-5DOF-Robot-Arm
"""
from __future__ import annotations

import importlib.util
import json
import os
from datetime import datetime
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO_NAME = "STS3215-5DOF-Robot-Arm"
TICKS = 4096


def find_repo():
    cands = [os.environ.get("ROBOARM_OFFICIAL"),
             Path.home() / "robot-arm-lab" / REPO_NAME,
             HERE.parent / REPO_NAME,
             HERE.parent / "repos" / REPO_NAME]
    for c in cands:
        if c and (Path(c) / "arm_settings.py").exists():
            return Path(c)
    return None


def load_settings(repo=None):
    """Import the official arm_settings.py as a module (or None if not found)."""
    repo = repo or find_repo()
    if repo is None:
        return None
    spec = importlib.util.spec_from_file_location("official_arm_settings", repo / "arm_settings.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    mod.REPO = Path(repo)
    return mod


class Conv:
    """Raw ticks <-> joint angles exactly like arm_helpers.py, keyed by servo id."""

    def __init__(self, s):
        self.s = s
        self.names = list(s.JOINT_NAMES)
        self.id_of = {n: s.SERVO_IDS[n] for n in self.names}
        self.name_of = {i: n for n, i in self.id_of.items()}

    def angle(self, sid, raw):
        n = self.name_of[sid]
        return (raw - self.s.ZERO_TICKS[n]) * 360.0 / (self.s.DIRECTIONS[n] * TICKS)

    def raw(self, sid, angle):
        n = self.name_of[sid]
        return round(self.s.ZERO_TICKS[n] + self.s.DIRECTIONS[n] * angle * TICKS / 360.0)

    def raw_limits(self):
        """ANGLE_LIMITS converted to raw tick ranges, clipped to 0..4095."""
        out = {}
        for n in self.names:
            lo, hi = self.s.ANGLE_LIMITS[n]
            sid = self.id_of[n]
            a, b = sorted((self.raw(sid, lo), self.raw(sid, hi)))
            out[str(sid)] = [max(0, a), min(4095, b)]
        return out

    def gripper_close_direction(self):
        """Course convention: closing = smaller gripper ANGLE (open 83, closed -28)."""
        return -int(self.s.DIRECTIONS[self.names[-1]])

    def angles_str(self, pose):
        return " ".join(f"{self.angle(i, pose[i]):.1f}" for i in sorted(pose))


def config_overrides(s):
    """Settings our motion.py takes from the official file."""
    c = Conv(s)
    return {
        "port": s.SERIAL_PORT,
        "baud": int(s.BAUD_RATE),
        "joint_limits": c.raw_limits(),
        "gripper_close_direction": c.gripper_close_direction(),
    }


# ----------------------------------------------------------------- export
def write_sequence(path, name, description, points_raw, s=None, extra=None):
    """Write a pose_sequence file that 05_waypoint_sequence.py can load and play."""
    names = list(s.JOINT_NAMES) if s else ["base_yaw", "shoulder", "elbow", "wrist", "gripper"]
    ids = [s.SERVO_IDS[n] for n in names] if s else [1, 2, 3, 4, 5]
    seq = {
        "schema_version": 1,
        "type": "pose_sequence",
        "name": name,
        "description": description,
        "input_type": "raw",
        "joint_order": [{"servo_id": i, "joint": n} for i, n in zip(ids, names)],
        "created_at": datetime.now().astimezone().isoformat(timespec="seconds"),
        **(extra or {}),
        "points": [{"input_values": [str(int(p[i])) for i in ids]} for p in points_raw],
    }
    # same layout as the official save_sequence(): one point per line
    lines = ["{"]
    items = list(seq.items())
    for k, (key, val) in enumerate(items):
        last = k == len(items) - 1
        if key == "points":
            lines.append('  "points": [')
            for j, p in enumerate(val):
                lines.append("    " + json.dumps(p, separators=(", ", ": ")) + ("," if j < len(val) - 1 else ""))
            lines.append("  ]" + ("" if last else ","))
        else:
            lines.append(f"  {json.dumps(key)}: {json.dumps(val, separators=(', ', ': '))}" + ("" if last else ","))
    lines.append("}")
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(".json.tmp")
    tmp.write_text("\n".join(lines) + "\n", encoding="utf-8")
    os.replace(tmp, path)
