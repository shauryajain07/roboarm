# ROBOARM control for the Raspberry Pi

This is the software for the ENGG1101 ROBOARM Contest 2026/27, "Domination of the Nine Squares". It works **alongside** the official workshop repo, [Kelvinchan324/STS3215-5DOF-Robot-Arm](https://github.com/Kelvinchan324/STS3215-5DOF-Robot-Arm):

- **Settings:** it reads that repo's `arm_settings.py` for the serial port, baud rate, servo IDs, zero positions, joint directions and angle limits.
- **Saved scenarios:** they're official waypoint files in that repo's `motions/` folder, so `05_waypoint_sequence.py` can play them too.

## Main app: `control_gui.py`

| Tab | Purpose |
|---|---|
| **Arm panel** (always visible) | Live position in raw ticks and degrees, jog −/+, LIMP / HOLD, gripper presets, **E-STOP (Space or Esc)** |
| **1 Setup poses** | Record STOW, SAFE, RACKWAIT and the gripper values **once**. Every scenario reuses them. |
| **2 Record scenarios** | All 41 scenarios. Capture only the new poses (2 per scenario), then test and save. |
| **3 Game** | Click a cell to place a block, which plays `pick_<block>` then `place_<block>_<cell>`. Also handles attacks (after the referee says OK), start lock, timer, score, tokens and suggested moves. |

The 41 scenarios are:
- `start` and `stow`
- 3 picks
- 9 attacks
- 27 placements (every block in every cell)

See **[SCENARIO_CHECKLIST.md](SCENARIO_CHECKLIST.md)** for the recording walkthrough.

## Setup on the Pi

This assumes the workshop setup is already done: `~/robot-arm-lab/.venv` exists and the official repo is cloned.

```bash
sudo apt install -y python3-tk
source ~/robot-arm-lab/.venv/bin/activate
cd ~/robot-arm-lab && gh repo clone shauryajain07/roboarm
cd roboarm && python -m unittest discover -s tests     # tests on a simulated arm
python control_gui.py --mock                            # practise with no arm
python control_gui.py                                   # real arm
```

The official repo is found automatically at `~/robot-arm-lab/STS3215-5DOF-Robot-Arm`. If it's somewhere else, set `ROBOARM_OFFICIAL=/path/to/it`. To override a setting just for our code, put it in `arm_config.json`.

### Calibrate the gripper

If the gripper shows fully open or closed but the jaws aren't, its zero isn't calibrated. Close the GUI and run:

```bash
python calibrate.py --check    # what's wrong, changes nothing
python calibrate.py            # open the jaws by hand, close them, done (~30 s)
```

Only the gripper's torque is released. It writes the gripper's `ZERO_TICKS`, `DIRECTIONS` and `ANGLE_LIMITS` into the official `arm_settings.py`, after backing it up to `backups/`. It also checks joints 1–4: a recorded pose sitting on a limit means that joint was stopped by its limit.

### Game controller

Plug in or pair a controller and it shows up on the **4 Drive** tab. Its status line shows live stick values. If they don't change when you move the sticks, run `python gamepad.py` to see what the Pi receives. The SCUF NOMAD has its own button map. Use Menu ≡ for E-STOP and tap View to reset; its D-pad isn't used, because Linux misreads it.

#### Gripper drive (optional)

By default each stick direction turns one joint. Tick **gripper drive (IK)** on the Drive tab, or tap **R2**, and the sticks move the gripper tip instead. Inverse kinematics works out the shoulder, elbow and wrist angles for you.

| Stick | ↕ | ↔ |
|---|---|---|
| Left | forward / back, keeping height and tilt | base (as before) |
| Right | up / down, keeping reach and tilt | tilt the gripper |

The keyboard keys, the on-screen pads, L2/Shift precise mode, the gripper buttons, E-STOP and the joint limits all work the same. A line under the pads shows where the tip is: distance forward, height and tilt (0° = level). At the edge of the arm's reach, or when a joint hits its limit, the tip stops and the log says why. Tap R2 again to go back to single joints.

The geometry comes from the course's arm model (`robot-arm-simulation`), and the joint angles come from the official `arm_settings.py`. If the arm's zero positions are off, straight lines bend slightly, but a held tilt stays exact. Scenario recording and playback don't use it: they still replay your taught poses.

## Files

| File | Contents |
|---|---|
| `control_gui.py` | The GUI described above |
| `trajectory.py` | Fluid motion: records hand-guided takes (saved exactly, ~50 Hz), turns waypoints into smooth curves, and streams them to the servos |
| `restore_raw_takes.py` | One-off: turns old auto-cleaned fluid takes back into the exact original recordings |
| `clean.py` | Cleans pushed data (limits, missing lifts) and writes `CLEANING_REPORT.md` |
| `scenarios.py` | The 41 scenario templates, `scenario_data.json`, and saving and loading official waypoint files |
| `official.py` | Reads the official `arm_settings.py`, converts between raw ticks and angles, and writes waypoint files |
| `calibrate.py` | Gripper calibration (writes the official `arm_settings.py`) and a check of every joint's limits |
| `gamepad.py`, `joystick.py` | Game controller reader with per-pad button maps, and the **4 Drive** tab |
| `kinematics.py` | Arm geometry and inverse kinematics for the Drive tab's optional gripper drive |
| `motion.py` | Synchronised moves (all joints arrive together), joint limits, E-stop and grip detection |
| `servo_bus.py` | The ST3215 serial driver (same bytes as the course tools), plus a simulated arm |
| `strategy.py` | Suggests a placement (win, block, then the best open line) or an attack target |
| `record.py`, `game.py` | Older terminal recorder and game GUI. `control_gui.py` replaces them. |

`scenario_data.json` (poses) and `trajectories/` (fluid takes) hold everything you've recorded. **Commit and push both** so the whole team has the same poses. Run `Rebuild all saved scenario files` on another Pi to regenerate the motion files from it.
