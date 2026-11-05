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
cd roboarm && python -m unittest discover -s tests     # 24 tests on a simulated arm
python control_gui.py --mock                            # practise with no arm
python control_gui.py                                   # real arm
```

The official repo is found automatically at `~/robot-arm-lab/STS3215-5DOF-Robot-Arm`. If it's somewhere else, set `ROBOARM_OFFICIAL=/path/to/it`. To override a setting just for our code, put it in `arm_config.json`.

## Files

| File | Contents |
|---|---|
| `control_gui.py` | The GUI described above |
| `trajectory.py` | Fluid motion: records hand-guided takes, cleans them, turns waypoints into smooth curves, and streams them to the servos |
| `clean.py` | Cleans pushed data (limits, missing lifts) and writes `CLEANING_REPORT.md` |
| `scenarios.py` | The 41 scenario templates, `scenario_data.json`, and saving and loading official waypoint files |
| `official.py` | Reads the official `arm_settings.py`, converts between raw ticks and angles, and writes waypoint files |
| `motion.py` | Synchronised moves (all joints arrive together), joint limits, E-stop and grip detection |
| `servo_bus.py` | The ST3215 serial driver (same bytes as the course tools), plus a simulated arm |
| `strategy.py` | Suggests a placement (win, block, then the best open line) or an attack target |
| `record.py`, `game.py` | Older terminal recorder and game GUI. `control_gui.py` replaces them. |

`scenario_data.json` (poses) and `trajectories/` (fluid takes) hold everything you've recorded. **Commit and push both** so the whole team has the same poses. Run `Rebuild all saved scenario files` on another Pi to regenerate the motion files from it.
