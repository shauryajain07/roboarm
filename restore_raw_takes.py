#!/usr/bin/env python3
"""Undo the old automatic cleaning: rebuild every fluid take from its original recording.

Takes recorded before fluid takes became "exact" were smoothed, trimmed, pause-shortened
and snapped to SAFE. The untouched original of each was kept in trajectories/raw/.
This puts those originals back as the takes that play.

    python3 restore_raw_takes.py            # restore all
    python3 restore_raw_takes.py --dry-run  # only list what would change
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import official
import scenarios as sc
import trajectory


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--folder", default=str(sc.TRAJ_DIR))
    a = ap.parse_args()

    s = official.load_settings()
    names = {s.SERVO_IDS[n]: n for n in s.JOINT_NAMES} if s else trajectory.DEFAULT_NAMES
    folder = Path(a.folder)
    raws = sorted((folder / "raw").glob("*.json"))
    if not raws:
        print(f"no original recordings in {folder / 'raw'} - nothing to restore")
        return
    for rp in raws:
        name = rp.stem
        take = sc.fluid_path(name, folder)
        rep_old = json.loads(take.read_text()).get("roboarm", {}).get("report", {}) if take.exists() else {}
        if rep_old.get("cleaned") is False:
            print(f"  {name}: already exact - skipped")
            continue
        raw = trajectory.load_raw(rp)
        if len(raw) < 2:
            print(f"  {name}: original too short - skipped")
            continue
        t0 = raw.t[0]
        raw.t = [t - t0 for t in raw.t]
        hz = round((len(raw) - 1) / raw.duration, 1) if raw.duration else 0
        rep = {"duration_s": round(raw.duration, 2), "samples": len(raw), "hz": hz, "cleaned": False}
        print(f"  {name}: {rep_old.get('clean_duration_s', '?')} s cleaned -> {rep['duration_s']} s exact "
              f"({len(raw)} samples, {hz} Hz)")
        if not a.dry_run:
            trajectory.save(take, raw, name, f"roboarm fluid take (exact): {name}", names, extra={"report": rep})
    print("dry run - nothing written" if a.dry_run else "done - press ▶ Test in control_gui.py to check each one")


if __name__ == "__main__":
    main()
