#!/usr/bin/env python3
"""Calibrate the gripper and check every joint's limits. Run on the Pi with the GUI closed.

    python3 calibrate.py           # calibrate the gripper (~30 s), then check all joints
    python3 calibrate.py --check   # only check - no arm needed, nothing changes
    python3 calibrate.py --mock    # practice on the simulated arm (nothing is saved)

GRIPPER - only its torque is released; the arm keeps holding. You open the jaws fully by hand,
then close them until they just touch. From those two readings it writes, into the official
arm_settings.py (so the course scripts agree):
  ZERO_TICKS["gripper"]    so open reads ~+83 deg and closed ~-28 deg (course convention)
  ANGLE_LIMITS["gripper"]  the real travel: closed = jaws touching, open = 2 deg short of the
                           hard stop (so "open fully" never pushes into it); widened if needed
                           so every recorded pose still plays
It also removes a joint-5 override from arm_config.json (setup_pi.sh copies in one that switches
the limits off), and offers to move the 'attack' grip - the tightest grip, which caps how far
the slider closes - to the measured closed point. Every file is backed up to backups/ first.

JOINTS 1-4 - their 0 deg is a reference pose set when the servos were centred at assembly, so
there is nothing to measure them against. They are CHECKED instead: a recorded pose sitting on
a software limit means that joint was stopped by the limit, so its zero or limit is off.
"""
from __future__ import annotations

import argparse
import json
import math
import re
import shutil
import statistics
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
TPD = 4096 / 360.0                   # ticks per degree
OPEN_DEG, CLOSED_DEG = 83.0, -28.0   # course convention: closing = smaller gripper angle
OPEN_MARGIN_DEG = 2.0                # stop this far short of the open hard stop
NEAR_TICKS = 12                      # ~1 deg: a recorded value this close to a limit was stopped by it
MIN_TRAVEL = 150                     # ticks; less means the jaws didn't really move
WIDEN_TICKS = 34                     # ~3 deg: recorded values this far past a measured end still count
                                     # (hand measurement); further = from before the zero moved
NAMES = {1: "base", 2: "shoulder", 3: "elbow", 4: "wrist", 5: "gripper"}


# ------------------------------------------------------------------ maths (no hardware)
def fit_gripper(open_raw: int, closed_raw: int, recorded=()) -> dict:
    """Zero, direction and limits from the two measured end points.

    recorded: (label, raw) gripper values already in use. Ones just past a measured end widen
    the limits (playback refuses targets outside them). Ones far past it can't be reached by
    these jaws - they were recorded before the gripper's zero moved - so they are returned as
    'stale' and NOT allowed: driving to them would push the jaws into the hard stop."""
    travel = abs(open_raw - closed_raw)
    if travel < MIN_TRAVEL:
        raise ValueError(f"open {open_raw} and closed {closed_raw} are only {travel} ticks apart - "
                         "the jaws didn't really move. Try again.")
    d = -1 if closed_raw > open_raw else 1          # keep "closing = smaller angle"
    zero = round(((open_raw - d * OPEN_DEG * TPD) + (closed_raw - d * CLOSED_DEG * TPD)) / 2)
    s = 1 if closed_raw > open_raw else -1          # raw direction of closing
    open_lim = open_raw + s * round(OPEN_MARGIN_DEG * TPD)
    lo_raw, hi_raw = sorted((open_lim, closed_raw))
    outside = [(n, v) for n, v in recorded if not lo_raw <= v <= hi_raw]
    widened = [(n, v) for n, v in outside if lo_raw - WIDEN_TICKS <= v <= hi_raw + WIDEN_TICKS]
    stale = [(n, v) for n, v in outside if (n, v) not in widened]
    vals = [lo_raw, hi_raw] + [v for _, v in widened]

    def ang(raw):
        return (raw - zero) / (d * TPD)
    a = sorted((ang(min(vals)), ang(max(vals))))
    limits = (math.floor(a[0] * 10) / 10, math.ceil(a[1] * 10) / 10)   # round outward
    raw_limits = sorted(round(zero + d * x * TPD) for x in limits)
    return {"zero": zero, "direction": d, "limits": limits,
            "raw_limits": [max(0, raw_limits[0]), min(4095, raw_limits[1])],
            "open_deg": round(ang(open_raw), 1), "closed_deg": round(ang(closed_raw), 1),
            "open_raw": open_raw, "closed_raw": closed_raw, "widened": widened, "stale": stale}


def _set_entry(text: str, dict_name: str, key: str, value_src: str) -> str:
    """Replace the value of `key` inside the top-level `dict_name = {...}` block."""
    m = re.search(rf"^{dict_name}\s*=\s*\{{\n(.*?)^\}}", text, re.S | re.M)
    if not m:
        raise ValueError(f"{dict_name} = {{...}} not found in arm_settings.py")
    line = re.compile(rf"^(\s*[\"']{re.escape(key)}[\"']\s*:\s*)(.*?)(,?[ \t]*(?:#.*)?)$", re.M)
    body = m.group(1)
    if len(line.findall(body)) != 1:
        raise ValueError(f"expected exactly one {key!r} entry in {dict_name}")
    body = line.sub(lambda mm: mm.group(1) + value_src + mm.group(3), body, count=1)
    return text[:m.start(1)] + body + text[m.end(1):]


def patch_settings(text: str, name: str, cal: dict) -> str:
    text = _set_entry(text, "ZERO_TICKS", name, str(cal["zero"]))
    text = _set_entry(text, "DIRECTIONS", name, str(cal["direction"]))
    lo, hi = cal["limits"]
    return _set_entry(text, "ANGLE_LIMITS", name, f"({lo:.1f}, {hi:.1f})")


def collect_recorded(data, poses=None) -> dict[int, list[tuple[str, int]]]:
    """Every joint value the app will ever send: shared poses, captures, presets, grips."""
    out: dict[int, list[tuple[str, int]]] = {j: [] for j in NAMES}

    def add(label, pose):
        for j, v in pose.items():
            if int(j) in out:
                out[int(j)].append((label, int(v)))
    for n, p in data.shared.items():
        add(n, p)
    for scen, caps in data.captures.items():
        for lbl, p in caps.items():
            add(f"{scen}/{lbl}", p)
    for n, p in data.presets.items():
        add(f"preset {n.upper()}", p)
    for g, v in data.grips.items():
        out[5].append((f"grip '{g}'", int(v)))
    for n, p in (poses.poses.items() if poses else ()):
        add(f"pose {n}", p)
    return out


def check_joints(limits: dict, recorded: dict) -> list[str]:
    """Warnings for joints whose recorded values sit on (or past) a software limit."""
    out = []
    for j, vals in recorded.items():
        lo, hi = (int(x) for x in limits.get(str(j), (0, 4095)))
        bad = [(n, v) for n, v in vals if v <= lo + NEAR_TICKS or v >= hi - NEAR_TICKS]
        if not bad:
            continue
        worst = sorted(bad, key=lambda nv: min(abs(nv[1] - lo), abs(nv[1] - hi)))
        ex = ", ".join(f"{n} = {v}" for n, v in worst[:3]) + (f" (+{len(bad) - 3} more)" if len(bad) > 3 else "")
        out.append(f"{j} {NAMES[j]}: {len(bad)} recorded value(s) at the limit {lo}..{hi} - {ex}")
    return out


# ------------------------------------------------------------------ files
def backup(path: Path, folder: Path) -> Path:
    folder.mkdir(exist_ok=True)
    dst = folder / f"{path.stem}-{time.strftime('%Y%m%d-%H%M%S')}{path.suffix}"
    shutil.copy2(path, dst)
    return dst


def write_settings(path: Path, name: str, cal: dict, folder: Path) -> Path:
    new = patch_settings(path.read_text(), name, cal)
    b = backup(path, folder)
    path.write_text(new)
    return b


def drop_joint5_override(config: Path, folder: Path) -> Path | None:
    if not config.exists():
        return None
    cfg = json.loads(config.read_text())
    if "5" not in cfg.get("joint_limits", {}):
        return None
    b = backup(config, folder)
    del cfg["joint_limits"]["5"]
    config.write_text(json.dumps(cfg, indent=2) + "\n")
    return b


# ------------------------------------------------------------------ hardware
ARM_PROGRAMS = r"control_gui\.py|record\.py|game\.py|0[1-5]_[a-z_]+\.py"


def other_arm_programs() -> list[str]:
    """Other programs that hold the serial port (Linux); they'd steal the servo replies."""
    import os
    import subprocess
    try:
        out = subprocess.run(["pgrep", "-af", ARM_PROGRAMS], capture_output=True, text=True, timeout=3).stdout
    except (OSError, subprocess.SubprocessError):
        return []
    return [ln for ln in out.splitlines() if ln.strip() and not ln.startswith(f"{os.getpid()} ")
            and "pgrep" not in ln]


def wait_for_servo(bus, sid: int, seconds: float = 4.0) -> bool:
    """Read the servo's position until it answers. The first requests after opening the port
    can be lost while the ESP32 settles; the GUI hides that by polling, so do the same."""
    from servo_bus import ServoError    # noqa: PLC0415
    end = time.monotonic() + seconds
    while True:
        try:
            bus.read_position(sid)
            return True
        except (ServoError, OSError):
            if time.monotonic() > end:
                return False
            time.sleep(0.2)


def read_steady(bus, sid: int) -> int | None:
    vals = []
    for _ in range(7):
        vals.append(bus.read_position(sid))
        time.sleep(0.03)
    return None if max(vals) - min(vals) > 15 else int(statistics.median(vals))


def measure(bus, sid: int, prompt: str, ask, hand=None) -> int:
    while True:
        ask(prompt)
        if hand:
            hand()
        v = read_steady(bus, sid)
        if v is not None:
            print(f"    gripper reads {v}")
            return v
        print("    it moved while reading - hold the jaws still and press Enter again")


def _show_checks(title, limits, recorded):
    warn = check_joints(limits, recorded)
    print(f"\njoint check ({title}):")
    print("\n".join("  ⚠ " + w for w in warn) if warn else "  ✓ no recorded pose is sitting on a limit")


# ------------------------------------------------------------------ main
def main(argv=None, bus=None, transport=None, ask=input) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--check", action="store_true", help="only check every joint; change nothing")
    ap.add_argument("--mock", action="store_true", help="simulated arm; nothing is saved")
    ap.add_argument("--data", default=str(HERE / "scenario_data.json"))
    ap.add_argument("--config", default=str(HERE / "arm_config.json"))
    ap.add_argument("--poses", default=str(HERE / "poses.json"))
    ap.add_argument("--backups", default=str(HERE / "backups"))
    a = ap.parse_args(argv)

    import official                                    # noqa: PLC0415
    from motion import load_config, open_bus           # noqa: PLC0415
    from poses import PoseStore                         # noqa: PLC0415
    from scenarios import ScenarioData                  # noqa: PLC0415

    s = official.load_settings()
    if s is None:
        print("The official STS3215-5DOF-Robot-Arm repo wasn't found - clone it to ~/robot-arm-lab/ first.")
        return 1
    settings_file = Path(s.REPO) / "arm_settings.py"
    name = s.JOINT_NAMES[-1]
    sid = s.SERVO_IDS[name]
    cfg = load_config(a.config)
    data = ScenarioData(a.data)
    poses = PoseStore(a.poses) if Path(a.poses).exists() else None
    recorded = collect_recorded(data, poses)
    off_raw = official.Conv(s).raw_limits()

    print(f"calibration file: {settings_file}")
    print(f"gripper now: zero {s.ZERO_TICKS[name]}, direction {s.DIRECTIONS[name]}, "
          f"limits {tuple(s.ANGLE_LIMITS[name])} deg = raw {off_raw[str(sid)][0]}..{off_raw[str(sid)][1]}")
    diff = [j for j in off_raw if cfg["joint_limits"].get(j) != off_raw[j]]
    if diff:
        print(f"NOTE: {Path(a.config).name} overrides the limits of joint(s) {', '.join(diff)} - "
              "roboarm uses those, not arm_settings.py")
    _show_checks("limits roboarm uses now", cfg["joint_limits"], recorded)
    if diff:
        _show_checks("arm_settings.py limits, without the override", off_raw, recorded)
    if a.check:
        return 0

    port = "mock"
    if bus is None:
        if a.mock:
            from servo_bus import mock_bus                # noqa: PLC0415
            bus, transport = mock_bus(time_scale=50)
        else:
            others = other_arm_programs()
            if others:
                print("\nClose these first - they use the arm's serial port and would steal its replies:")
                print("\n".join("    " + o for o in others))
                print("(close the window, or: pkill -f control_gui.py)")
                return 1
            from motion import find_port                  # noqa: PLC0415
            port = find_port(cfg)
            print(f"\nopening {port} @ {cfg['baud']} ...")
            bus = open_bus(cfg)
    if not wait_for_servo(bus, sid):
        alive = [j for j in NAMES if j != sid and wait_for_servo(bus, j, 0.6)]
        if alive:
            print(f"\nservos {alive} answer on {port} but the gripper (servo {sid}) doesn't - check its cable "
                  "(the last one in the chain) and that its ID is still 5.")
        else:
            print(f"\nno servo answers on {port}. Check: the arm's 12 V power is on, the ESP32 is in "
                  "serial-forwarding mode, the USB cable is in, and no other program has the port open.")
        return 1

    def sim(v):                                         # --mock: pretend a hand moved the jaws
        return (lambda: transport.hand_move({sid: v})) if a.mock and transport else None

    print("\n=== gripper calibration ===  (Ctrl+C at any time = stop, nothing saved)")
    ask("Press Enter to release the GRIPPER's torque (the rest of the arm keeps holding)... ")
    bus.set_torque(sid, False)
    try:
        open_raw = measure(bus, sid, "1/2  Open the jaws as WIDE as they go by hand, hold them there, press Enter ",
                           ask, sim(905))
        closed_raw = measure(bus, sid, "2/2  Close the jaws by hand until they JUST touch, hold, press Enter ",
                             ask, sim(2380))
    finally:
        try:
            cur = bus.read_position(sid)
            bus.write_position(sid, cur, 300, 50)           # hold where it is, no jump
            bus.set_torque(sid, True)
        except Exception as e:  # noqa: BLE001
            print(f"(couldn't re-power the gripper: {e})")
    try:
        cal = fit_gripper(open_raw, closed_raw, recorded[sid])
    except ValueError as e:
        print(f"\n{e}")
        return 1

    s_close = 1 if closed_raw > open_raw else -1           # raw direction of closing
    old_lim, cur_lim = tuple(s.ANGLE_LIMITS[name]), cfg["joint_limits"][str(sid)]
    used_now, used_new = f"{cur_lim[0]}..{cur_lim[1]}", f"{cal['raw_limits'][0]}..{cal['raw_limits'][1]}"
    print(f"\nmeasured: open {open_raw}, closed {closed_raw}  (travel {abs(open_raw - closed_raw)} ticks "
          f"= {abs(open_raw - closed_raw) / TPD:.0f} deg)")
    print(f"  {'':16s}{'before':>16s}{'after':>16s}")
    print(f"  {'ZERO_TICKS':16s}{s.ZERO_TICKS[name]:>16d}{cal['zero']:>16d}")
    print(f"  {'DIRECTIONS':16s}{s.DIRECTIONS[name]:>16d}{cal['direction']:>16d}")
    print(f"  {'ANGLE_LIMITS':16s}{str(old_lim):>16s}{str(cal['limits']):>16s}")
    print(f"  {'raw range used':16s}{used_now:>16s}{used_new:>16s}")
    print(f"  open now reads {cal['open_deg']:+.1f} deg, closed {cal['closed_deg']:+.1f} deg")
    for n, v in cal["widened"]:
        print(f"  (limits widened to keep {n} = {v} playable)")
    if cal["stale"]:
        print(f"\n⚠ {len(cal['stale'])} recorded gripper value(s) are beyond what these jaws can reach - they were "
              "recorded before the gripper's zero moved, so they're NOT allowed (they'd push into the stop):")
        for n, v in cal["stale"][:12]:
            print(f"    {n} = {v}")
        if len(cal["stale"]) > 12:
            print(f"    ... +{len(cal['stale']) - 12} more")
        print("  Re-record the grips on the Setup tab (open, cube, cylinder, trophy, attack). Scenarios that use "
              "any of these values will stop with 'outside limit' until they're re-captured.")

    if a.mock:
        print("\n(mock run - nothing was saved)")
        return 0
    if ask(f"\nWrite this to {settings_file.name}? [y/N] ").strip().lower() != "y":
        print("nothing saved")
        return 0
    folder = Path(a.backups)
    b = write_settings(settings_file, name, cal, folder)
    s2 = official.load_settings(settings_file.parent)
    got = official.Conv(s2).raw_limits()[str(sid)]
    if (s2.ZERO_TICKS[name], s2.DIRECTIONS[name], got) != (cal["zero"], cal["direction"], cal["raw_limits"]):
        shutil.copy2(b, settings_file)
        print("the written file didn't read back correctly - restored the original")
        return 1
    print(f"✓ saved {settings_file}  (old copy: {b})")
    b2 = drop_joint5_override(Path(a.config), folder)
    if b2:
        print(f"✓ removed the gripper limit override from {Path(a.config).name}  (old copy: {b2})")

    closed_grips = {g: v for g, v in data.grips.items() if g != "open"}
    if closed_grips:
        tight = max(closed_grips, key=lambda g: s_close * closed_grips[g])
        short = (closed_raw - closed_grips[tight]) * s_close
        where = f"{short} ticks short of" if short > 0 else f"{-short} ticks past"
        if abs(short) > 2 * NEAR_TICKS and ask(
                f"\ngrip '{tight}' = {closed_grips[tight]} is the tightest grip, which caps how far the slider "
                f"closes - it's {where} fully closed ({closed_raw}). Set it to {closed_raw}? [y/N] "
                ).strip().lower() == "y":
            backup(Path(a.data), folder)
            data.grips[tight] = closed_raw
            data.save()
            print(f"✓ grip '{tight}' = {closed_raw}")
    opn = data.grips.get("open")
    if opn is not None and abs(opn - cur_lim[0 if s_close > 0 else 1]) <= NEAR_TICKS \
            and ("grip 'open'", opn) not in cal["stale"]:
        print(f"\nTIP: grip 'open' = {opn} was recorded while the old limit stopped the jaws. To open wider "
              "when placing, re-record it on the Setup tab with the jaws fully open.")

    _show_checks("new limits", load_config(a.config)["joint_limits"], collect_recorded(data, poses))
    print("\nDone. Restart control_gui.py to use the new calibration.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
