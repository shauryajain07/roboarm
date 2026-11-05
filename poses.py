"""Pose store + the complete list of poses the arm needs for EVERY game case.

Cases covered:
  * pick each of the 3 block types (cube / cylinder / trophy) from its supply slot
  * place each block type into each of the 9 rack cells      -> 27 placements
  * attack each of the 9 rack cells                         ->  9 attacks

Cells are named r<row>c<col> as seen by OUR operator looking at the rack:
    r1c1 r1c2 r1c3     (r1 = top row)
    r2c1 r2c2 r2c3
    r3c1 r3c2 r3c3     (r3 = bottom row)
"""
from __future__ import annotations

import json
import os
import shutil
import time
from dataclasses import dataclass

JOINT_IDS = (1, 2, 3, 4, 5)
JOINT_NAMES = {1: "base", 2: "shoulder", 3: "elbow", 4: "wrist", 5: "gripper"}
GRIPPER = 5
ARM_JOINTS = (1, 2, 3, 4)

BLOCKS = ("cube", "cylinder", "trophy")
BLOCK_POINTS = {"cube": 10, "cylinder": 20, "trophy": 40}
CELLS = tuple(f"r{r}c{c}" for r in (1, 2, 3) for c in (1, 2, 3))
LINES = [("r1c1", "r1c2", "r1c3"), ("r2c1", "r2c2", "r2c3"), ("r3c1", "r3c2", "r3c3"),
         ("r1c1", "r2c1", "r3c1"), ("r1c2", "r2c2", "r3c2"), ("r1c3", "r2c3", "r3c3"),
         ("r1c1", "r2c2", "r3c3"), ("r1c3", "r2c2", "r3c1")]

# Where each block type sits in OUR Block Supply Area. We choose this during the
# Preparation Period (rule 3.1.6), so it is fixed and repeatable.
DEFAULT_SUPPLY = {"cube": "Plate B (left)", "cylinder": "Plate A (centre)",
                  "trophy": "Plate B (right)"}

CELL_WORDS = {"r1": "TOP", "r2": "MIDDLE", "r3": "BOTTOM",
              "c1": "LEFT", "c2": "CENTRE", "c3": "RIGHT"}


def cell_words(cell: str) -> str:
    return f"{CELL_WORDS[cell[:2]]}-{CELL_WORDS[cell[2:]]}"


@dataclass(frozen=True)
class PoseSpec:
    name: str
    group: str
    instruction: str
    needs_block: str | None = None   # block that must be held in the gripper
    seed_from: str | None = None     # existing pose to start from (speeds up recording)


def checklist(supply: dict[str, str] = DEFAULT_SUPPLY) -> list[PoseSpec]:
    """Every pose to record, in the order the recorder asks for them."""
    L: list[PoseSpec] = []
    add = L.append

    # 1) global poses -----------------------------------------------------------
    add(PoseSpec("stow", "1-global",
                 "START pose: arm folded so EVERY part (gripper, cables) is inside the "
                 "Robot Area and its vertical projection. This is where we sit at the Start Signal."))
    add(PoseSpec("safe", "1-global",
                 "TRANSIT pose: arm raised high over OUR side, clear of the supply plates "
                 "and the green Tic-Tac-Toe Zone. Every move passes through here."))
    add(PoseSpec("rack_safe", "1-global",
                 "RACK-WAITING pose: gripper facing the rack centre, just OUTSIDE the green "
                 "TTTZ (nothing over the green line). We wait here before entering for "
                 "placement / attack.", seed_from="safe"))
    add(PoseSpec("gripper_open", "1-global",
                 "GRIPPER OPEN: only the gripper matters. Open the jaws wide enough to clear "
                 "the trophy (widest block) with a few mm spare."))

    # 2) pick poses, one pair per block type -------------------------------------
    for b in BLOCKS:
        add(PoseSpec(f"pick/{b}/above", "2-pick",
                     f"ABOVE the {b.upper()} on {supply[b]}: jaws OPEN, ~4 cm above the block, "
                     f"centred over it.", seed_from="safe"))
        add(PoseSpec(f"pick/{b}/grip", "2-pick",
                     f"GRIP the {b.upper()}: lower straight down around it and CLOSE the jaws on "
                     f"it (type x). Grip low enough that it will not tilt.",
                     seed_from=f"pick/{b}/above"))

    # 3) attack poses: one pair per cell (no block in the gripper) ----------------
    for cell in CELLS:
        add(PoseSpec(f"attack/{cell}/front", "3-attack",
                     f"ATTACK {cell} ({cell_words(cell)}) - FRONT: jaws closed, tip lined up with "
                     f"the {cell_words(cell)} hole, ~2 cm in front of the rack face (inside TTTZ).",
                     seed_from="rack_safe"))
        add(PoseSpec(f"attack/{cell}/hit", "3-attack",
                     f"ATTACK {cell} ({cell_words(cell)}) - HIT: push the tip THROUGH the hole far "
                     f"enough to shove an opponent block out the far side. ONE push only (no "
                     f"hammering, rule 3.3.4.13). Do not touch the rack frame.",
                     seed_from=f"attack/{cell}/front"))

    # 4) place poses: every block type into every cell -----------------------------
    for b in BLOCKS:
        for cell in CELLS:
            seed = (f"place/{BLOCKS[BLOCKS.index(b) - 1]}/{cell}/front" if b != BLOCKS[0]
                    else f"attack/{cell}/front")
            add(PoseSpec(f"place/{b}/{cell}/front", "4-place",
                         f"PLACE {b.upper()} in {cell} ({cell_words(cell)}) - FRONT: holding the "
                         f"{b}, line it up with the hole, ~2 cm in front of the rack.",
                         needs_block=b, seed_from=seed))
            add(PoseSpec(f"place/{b}/{cell}/in", "4-place",
                         f"PLACE {b.upper()} in {cell} ({cell_words(cell)}) - IN: slide the {b} "
                         f"fully into the hole so it will sit stable when released. Still gripping.",
                         needs_block=b, seed_from=f"place/{b}/{cell}/front"))
    return L


# ============================================================================
class PoseStore:
    """JSON file of {pose_name: {"1": ticks, ..., "5": ticks}} plus metadata.
    Every save is atomic and keeps a timestamped backup."""

    def __init__(self, path: str):
        self.path = path
        self.poses: dict[str, dict[int, int]] = {}
        self.meta: dict = {}
        if os.path.exists(path):
            self.load()

    def load(self):
        with open(self.path) as f:
            d = json.load(f)
        self.meta = d.get("meta", {})
        self.poses = {k: {int(j): int(v) for j, v in p.items()} for k, p in d["poses"].items()}

    def save(self):
        if os.path.exists(self.path):
            bdir = os.path.join(os.path.dirname(os.path.abspath(self.path)), "backups")
            os.makedirs(bdir, exist_ok=True)
            shutil.copy2(self.path, os.path.join(bdir, time.strftime("poses-%Y%m%d-%H%M%S.json")))
        self.meta["saved_at"] = time.strftime("%Y-%m-%d %H:%M:%S")
        tmp = self.path + ".tmp"
        with open(tmp, "w") as f:
            json.dump({"meta": self.meta,
                       "poses": {k: {str(j): v for j, v in p.items()}
                                 for k, p in sorted(self.poses.items())}}, f, indent=1)
        os.replace(tmp, self.path)

    def __contains__(self, name):
        return name in self.poses

    def get(self, name) -> dict[int, int]:
        if name not in self.poses:
            raise KeyError(f"pose '{name}' not recorded yet - run record.py")
        return dict(self.poses[name])

    def set(self, name, joints: dict[int, int]):
        self.poses[name] = {int(j): int(v) for j, v in joints.items()}

    def missing(self, specs=None) -> list[str]:
        specs = specs or checklist()
        return [s.name for s in specs if s.name not in self.poses]

    @property
    def gripper_open_value(self) -> int:
        return self.get("gripper_open")[GRIPPER]

    def can_pick(self, block) -> bool:
        return all(n in self for n in ("safe", "gripper_open", f"pick/{block}/above",
                                       f"pick/{block}/grip"))

    def can_place(self, block, cell) -> bool:
        return self.can_pick(block) and all(
            n in self for n in ("rack_safe", f"place/{block}/{cell}/front",
                                f"place/{block}/{cell}/in"))

    def can_attack(self, cell) -> bool:
        return all(n in self for n in ("safe", "rack_safe", f"attack/{cell}/front",
                                       f"attack/{cell}/hit"))
