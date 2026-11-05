"""The 41 game scenarios as templates of shared poses + poses you capture.

Shared poses (STOW, SAFE, RACKWAIT) and gripper values (open / closed per block /
attack) are recorded ONCE; every scenario reuses them, so for a placement you only
capture FRONT and IN. Finished scenarios are saved as official pose_sequence files
(input_type raw) -> playable with 05_waypoint_sequence.py and by control_gui.py.
"""
from __future__ import annotations

import json
import os
from dataclasses import dataclass
from pathlib import Path

import official
from poses import BLOCKS, CELLS, GRIPPER, cell_words

SHARED = ("STOW", "SAFE", "RACKWAIT")
GRIPS = ("open", "cube", "cylinder", "trophy", "attack")
GRIP_HELP = {"open": "jaws open (wider than the trophy)",
             "cube": "closed on the cube (~10 deg tighter than touching)",
             "cylinder": "closed on the cylinder",
             "trophy": "closed on the trophy",
             "attack": "fully closed, used as the pusher for attacks"}
SHARED_HELP = {"STOW": "start pose, every part inside the Robot Area",
               "SAFE": "high over our side, clear of plates and the green zone",
               "RACKWAIT": "facing the rack, just OUTSIDE the green zone"}


@dataclass(frozen=True)
class Row:
    kind: str          # shared | capture | repeat
    ref: str           # shared pose name, or capture label
    grip: str | None   # gripper override key, None = keep the pose's own gripper
    hint: str


def template(name: str) -> list[Row]:
    S = lambda p, g, h="": Row("shared", p, g, h or SHARED_HELP[p])
    if name == "start":
        return [S("STOW", None), S("SAFE", "open")]
    if name == "stow":
        return [S("SAFE", "open"), S("STOW", None)]
    kind, _, rest = name.partition("_")
    if kind == "pick" and rest in BLOCKS:
        b = rest
        return [S("SAFE", "open"),
                Row("capture", "ABOVE", "open", f"~4 cm above the {b}, centred, jaws open"),
                Row("capture", "GRAB", "open", f"jaws around the {b}, low, still open"),
                Row("repeat", "GRAB", b, f"close on the {b}"),
                Row("repeat", "ABOVE", b, "lift straight up"),
                S("SAFE", b)]
    if kind == "attack" and rest in CELLS:
        c = rest
        return [S("SAFE", "attack"), S("RACKWAIT", "attack"),
                Row("capture", "FRONT", "attack", f"tip lined up with {c} ({cell_words(c)}), ~2 cm in front"),
                Row("capture", "HIT", "attack", "ONE push through the hole (no hammering, rule 3.3.4.13)"),
                Row("repeat", "FRONT", "attack", "pull back out"),
                S("RACKWAIT", "attack"), S("SAFE", "open")]
    if kind == "place":
        b, _, c = rest.partition("_")
        if b in BLOCKS and c in CELLS:
            return [S("SAFE", b), S("RACKWAIT", b),
                    Row("capture", "FRONT", b, f"{b} lined up with {c} ({cell_words(c)}), ~2 cm in front"),
                    Row("capture", "IN", b, f"{b} fully in the hole so it sits stable"),
                    Row("repeat", "IN", "open", "release"),
                    Row("repeat", "FRONT", "open", "back out"),
                    S("RACKWAIT", "open"), S("SAFE", "open")]
    raise KeyError(f"unknown scenario {name}")


def all_scenarios() -> list[str]:
    return (["start", "stow"] + [f"pick_{b}" for b in BLOCKS] + [f"attack_{c}" for c in CELLS]
            + [f"place_{b}_{c}" for b in BLOCKS for c in CELLS])


def captures_needed(name: str) -> list[str]:
    return [r.ref for r in template(name) if r.kind == "capture"]


# ============================================================================ data
class ScenarioData:
    """scenario_data.json: shared poses, gripper values and per-scenario captures."""

    def __init__(self, path):
        self.path = Path(path)
        self.shared: dict[str, dict[int, int]] = {}
        self.grips: dict[str, int] = {}
        self.captures: dict[str, dict[str, dict[int, int]]] = {}
        if self.path.exists():
            d = json.loads(self.path.read_text())
            ints = lambda p: {int(k): int(v) for k, v in p.items()}
            self.shared = {k: ints(v) for k, v in d.get("shared", {}).items()}
            self.grips = {k: int(v) for k, v in d.get("grips", {}).items()}
            self.captures = {s: {l: ints(p) for l, p in c.items()} for s, c in d.get("captures", {}).items()}

    def save(self):
        strs = lambda p: {str(k): v for k, v in sorted(p.items())}
        d = {"shared": {k: strs(v) for k, v in self.shared.items()},
             "grips": self.grips,
             "captures": {s: {l: strs(p) for l, p in c.items()} for s, c in sorted(self.captures.items())}}
        tmp = self.path.with_suffix(".tmp")
        tmp.write_text(json.dumps(d, indent=1))
        os.replace(tmp, self.path)

    # -- building ------------------------------------------------------------
    def row_pose(self, name: str, row: Row):
        """Full 5-joint pose for one template row, or None if something is missing."""
        if row.kind == "shared":
            base = self.shared.get(row.ref)
        else:
            base = self.captures.get(name, {}).get(row.ref)
        if base is None:
            return None
        p = dict(base)
        if row.grip is not None:
            if row.grip not in self.grips:
                return None
            p[GRIPPER] = self.grips[row.grip]
        return p

    def missing(self, name: str) -> list[str]:
        out = []
        caps = self.captures.get(name, {})
        for r in template(name):
            gaps = []
            if r.kind == "shared" and r.ref not in self.shared:
                gaps.append(f"shared {r.ref}")
            if r.kind != "shared" and r.ref not in caps:
                gaps.append(f"capture {r.ref}")
            if r.grip and r.grip not in self.grips:
                gaps.append(f"gripper '{r.grip}'")
            out += [g for g in gaps if g not in out]
        return out

    def points(self, name: str) -> list[dict[int, int]]:
        miss = self.missing(name)
        if miss:
            raise ValueError(f"{name}: still need {', '.join(miss)}")
        return [self.row_pose(name, r) for r in template(name)]

    def closed_values(self) -> set[int]:
        return {v for k, v in self.grips.items() if k != "open"}


def motions_dir() -> Path:
    repo = official.find_repo()
    return (repo / "motions") if repo else Path(__file__).resolve().parent / "motions"


def save_scenario(data: ScenarioData, name: str, folder: Path | None = None, settings=None) -> Path:
    folder = Path(folder or motions_dir())
    path = folder / f"{name}.json"
    rows = template(name)
    official.write_sequence(path, name, f"roboarm scenario: {name}", data.points(name), settings,
                            extra={"made_with": "roboarm control_gui",
                                   "rows": [f"{r.ref}" + (f" ({r.grip})" if r.grip else "") for r in rows]})
    return path


def load_points(path) -> list[dict[int, int]]:
    """Read an official pose_sequence file (raw or angles) into servo-id -> raw ticks."""
    d = json.loads(Path(path).read_text())
    if d.get("type") != "pose_sequence":
        raise ValueError(f"{path} is not a pose_sequence")
    ids = [j["servo_id"] for j in d["joint_order"]]
    s = official.load_settings()
    conv = official.Conv(s) if s else None
    pts = []
    for p in d["points"]:
        vals = p["input_values"]
        if d["input_type"] == "raw":
            pts.append({i: int(v) for i, v in zip(ids, vals)})
        else:
            if conv is None:
                raise ValueError("angle files need the official arm_settings.py")
            pts.append({i: conv.raw(i, float(v)) for i, v in zip(ids, vals)})
    return pts
