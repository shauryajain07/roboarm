# ROBOARM control for the Raspberry Pi

This is the software for the ENGG1101 ROBOARM Contest 2026/27, "Domination of the Nine Squares". It runs on the team's Raspberry Pi and drives the course robot arm: five ST3215 servos, IDs 1–5. The servo commands are byte-for-byte the same as the course web controller's.

| File | What it does |
|---|---|
| `record.py` | **Pose recorder.** Run it on the Pi. It guides you through every pose and saves each one to `poses.json` as soon as you capture it. |
| `game.py` | **Game-day GUI.** Pick a block type, then click a cell, and the arm picks the block up and places it. It also handles attacks, scoring, the attack tokens and the emergency stop on Space. |
| `motion.py` | Moves the arm between recorded poses, with all joints arriving together, and runs the pick, place and attack sequences. |
| `servo_bus.py` | The ST3215 serial driver. It sends one command for all five joints, and includes a simulated arm for practice. |
| `poses.py` | The list of every pose needed, and the `poses.json` store. |
| `strategy.py` | Suggests the next move: complete a line, block the opponent, then take the best open cell. Also suggests which opponent block to attack. |

## Every case covered

| Case | Count | Poses recorded |
|---|---|---|
| Global: stow, safe, rack_safe, gripper_open | — | 4 |
| Pick the cube, the cylinder and the trophy | 3 | 6 (above + grip for each) |
| Attack each of the 9 cells | 9 | 18 (front + hit for each) |
| Place each block in each of the 9 cells | 27 | 54 (front + in for each) |
| **Total** | | **82 poses** |

Cell names are from **our** side of the rack:

```
r1c1 r1c2 r1c3      top row
r2c1 r2c2 r2c3
r3c1 r3c2 r3c3      bottom row
```

## Pi setup (once)

```bash
git clone <this repo's URL> roboarm && cd roboarm
bash setup_pi.sh          # installs pyserial + tkinter and adds you to the 'dialout' group (reboot after)
python3 record.py --mock  # practise the recorder with a simulated arm
python3 -m unittest discover -s tests   # 15 tests: all 27 placements and 9 attacks on the simulated arm
```

The serial port is detected automatically (`/dev/ttyUSB*`, `/dev/ttyACM*`). Set the port or baud rate in `arm_config.json` if you need to:
- `115200` if the arm plugs into the Pi by USB through the controller board. This is the course web tool's default.
- `1000000` if the Pi's UART is wired straight to the servo bus.

Read [RECORDING_GUIDE.md](RECORDING_GUIDE.md) before you record anything.

## Game day

```bash
python3 record.py --status   # every case should show 9/9
python3 game.py
```

1. **Preparation period:** put the arm in STOW and the blocks on their fixed plates (cube on Plate B left, cylinder on Plate A, trophy on Plate B right).
2. **Start:** press **START** only on the referee's Start Signal. The GUI refuses to move before that.
3. **Placing:** choose a block, then click a cell. The yellow cell is the suggested move. Right-click a cell to mark an opponent's block.
4. **Attacking:** call "Attack" and wait for the referee's OK. Then tick *Referee authorised*, choose ATTACK, and click the cell.
5. **Emergency stop:** **Space** or **Esc** cuts torque immediately. **RESET STOP** makes the arm hold its current position again.

## Sharing poses with the team

`poses.json` is committed to the repo. After a recording session, commit and push it so everyone has the same poses:

```bash
git add poses.json && git commit -m "poses: recorded <what>" && git push
```

Older versions are kept in `backups/`, which is not committed.
