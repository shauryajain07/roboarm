#!/usr/bin/env python3
"""Clean recorded scenario data and report what needs re-recording.

  python3 clean.py            # fix scenario_data.json (backup kept), write CLEANING_REPORT.md
  python3 clean.py --dry-run  # only report

Fixes it applies automatically:
  * gripper values / joints outside the official ANGLE_LIMITS -> clamped to the limit
  * pick ABOVE that doesn't lift (same as GRAB) -> provisional lift copied from a good pick
Things it reports (need a human): above/grab not vertical, huge attack swings (handled by
the automatic PREHIT punch), missing scenarios, and stop-and-go vs fluid timing.
"""
from __future__ import annotations

import argparse
import json
import shutil
import time
from pathlib import Path

import official
import scenarios as sc
from motion import load_config
from poses import ARM_JOINTS, BLOCKS, GRIPPER

HERE = Path(__file__).resolve().parent
NO_LIFT_TICKS = 40         # ABOVE closer than this to GRAB = no real lift
SIDEWAYS_TICKS = 60        # base turning this much between ABOVE and GRAB = not straight down


def arm_dist(a, b):
    return max(abs(a[j] - b[j]) for j in ARM_JOINTS)


def clean_data(data: sc.ScenarioData, cfg: dict, conv=None):
    """Mutates data. Returns (fixes, warnings) lists of strings."""
    fixes, warns = [], []
    lim = {int(j): v for j, v in cfg["joint_limits"].items()}
    deg = (lambda j, v: f" ({conv.angle(j, v):.1f}°)") if conv else (lambda j, v: "")

    def clamp(p, where):
        for j, v in list(p.items()):
            lo, hi = lim[j]
            if not lo <= v <= hi:
                nv = min(hi, max(lo, v))
                fixes.append(f"{where}: joint {j} {v}{deg(j, v)} outside limit -> {nv}{deg(j, nv)}")
                p[j] = nv

    # 1) limits
    for g, v in list(data.grips.items()):
        lo, hi = lim[GRIPPER]
        if not lo <= v <= hi:
            nv = min(hi, max(lo, v))
            fixes.append(f"grip '{g}': {v}{deg(GRIPPER, v)} outside the gripper limit -> {nv}{deg(GRIPPER, nv)}")
            data.grips[g] = nv
    for k, p in data.shared.items():
        clamp(p, f"shared {k}")
    for n, caps in data.captures.items():
        for lbl, p in caps.items():
            clamp(p, f"{n}/{lbl}")

    # 2) picks that don't lift
    good = [b for b in BLOCKS if {"ABOVE", "GRAB"} <= set(data.captures.get(f"pick_{b}", {}))
            and arm_dist(data.captures[f"pick_{b}"]["ABOVE"], data.captures[f"pick_{b}"]["GRAB"]) >= NO_LIFT_TICKS
            and f"pick_{b}/ABOVE" not in data.notes]
    for b in BLOCKS:
        c = data.captures.get(f"pick_{b}", {})
        if not {"ABOVE", "GRAB"} <= set(c):
            continue
        above, grab = c["ABOVE"], c["GRAB"]
        if arm_dist(above, grab) < NO_LIFT_TICKS:
            if good:
                ref = data.captures[f"pick_{good[0]}"]
                lift = {j: ref["ABOVE"][j] - ref["GRAB"][j] for j in ARM_JOINTS}
                lift[1] = 0                                  # lift = up, never sideways
                new = dict(grab)
                for j in ARM_JOINTS:
                    new[j] = grab[j] + lift[j]
                clamp(new, f"pick_{b}/ABOVE (provisional)")
                c["ABOVE"] = new
                data.notes[f"pick_{b}/ABOVE"] = f"PROVISIONAL (lift copied from pick_{good[0]}) - re-record"
                fixes.append(f"pick_{b}: ABOVE was the same pose as GRAB (no lift) -> PROVISIONAL lift "
                              f"copied from pick_{good[0]}. RE-RECORD pick_{b} ABOVE.")
            else:
                warns.append(f"pick_{b}: ABOVE is the same pose as GRAB - the block won't be lifted. RE-RECORD ABOVE.")
        elif abs(above[1] - grab[1]) > SIDEWAYS_TICKS:
            warns.append(f"pick_{b}: base turns {abs(above[1] - grab[1])} ticks between ABOVE and GRAB - "
                         f"the jaws come down sideways and may knock the block. Re-record ABOVE straight above GRAB.")

    # 3) attacks
    for n, c in data.captures.items():
        if n.startswith("attack_") and {"FRONT", "HIT"} <= set(c):
            d = arm_dist(c["FRONT"], c["HIT"])
            if d > sc.PUNCH_TICKS:
                warns.append(f"{n}: FRONT->HIT is {d} ticks ({d * 360 / 4096:.0f}°) - too far to be a push. "
                             f"Auto PREHIT keeps the full-speed part to the last {sc.PUNCH_TICKS} ticks; "
                             f"for a cleaner attack re-record FRONT ~2 cm in front of the hole.")
    return fixes, warns


def timing(data, cfg):
    """Stop-and-go estimate vs smooth fluid duration for every complete scenario."""
    rows = []
    for n in sc.all_scenarios():
        if data.missing(n):
            continue
        pts = data.points(n)
        sp = [cfg["max_speed"] if s == "max" else cfg["speed"] for s in sc.speeds(n)]
        stopgo = sum(arm_dist(pts[i], pts[i - 1]) / sp[i] + 0.35 for i in range(1, len(pts)))
        tr = sc.fluid_from_waypoints(data, n, cfg)
        rows.append((n, len(pts), stopgo, tr.duration))
    return rows


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--data", default=str(HERE / "scenario_data.json"))
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--report", default=str(HERE / "CLEANING_REPORT.md"))
    a = ap.parse_args()

    cfg = load_config()
    s = official.load_settings()
    conv = official.Conv(s) if s else None
    data = sc.ScenarioData(a.data)
    fixes, warns = clean_data(data, cfg, conv)
    names = sc.all_scenarios()
    done = [n for n in names if not data.missing(n)]
    tim = timing(data, cfg)

    L = [f"# Cleaning report — {time.strftime('%Y-%m-%d %H:%M')}", "",
         f"**{len(done)}/41 scenarios complete:** " + ", ".join(f"`{n}`" for n in done), ""]
    L += ["## Fixed automatically", ""] + ([f"- {f}" for f in fixes] or ["- nothing needed"]) + [""]
    L += ["## Needs your attention", ""] + ([f"- {w}" for w in warns] or ["- nothing"]) + [""]
    L += ["## Fluid playback", "", "| Scenario | Waypoints | Stop-and-go (est.) | Fluid |", "|---|---|---|---|"]
    L += [f"| `{n}` | {k} | {sg:.1f} s | {fl:.1f} s |" for n, k, sg, fl in tim]
    todo = [n for n in names if n not in done]
    L += ["", f"## Still to record ({len(todo)})", "", ", ".join(f"`{n}`" for n in todo)]
    report = "\n".join(L) + "\n"
    print(report)
    if not a.dry_run:
        src = Path(a.data)
        if src.exists():
            shutil.copy2(src, src.with_name(f"scenario_data.backup-{time.strftime('%Y%m%d-%H%M%S')}.json"))
        data.save()
        Path(a.report).write_text(report)
        print(f"saved {a.data} (backup kept) and {a.report}")


if __name__ == "__main__":
    main()
