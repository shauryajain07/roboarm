# Scenario recording checklist

You record every scenario by hand in **`control_gui.py`**. It saves each scenario as an official waypoint file in `~/robot-arm-lab/STS3215-5DOF-Robot-Arm/motions/`, so the workshop's `05_waypoint_sequence.py` can play it too.

```bash
source ~/robot-arm-lab/.venv/bin/activate
cd ~/robot-arm-lab/roboarm
python control_gui.py
```

You can **practise first with no arm**: `python control_gui.py --mock`.

## Before you start

- [ ] **Official setup works:** `01_read_positions.py` reads all 5 servos, and the middle pose is close to 0°.
- [ ] **Arm finished:** the final gripper is fitted and the cables are tied down.
- [ ] **Field copy built to the rulebook dimensions:** the Robot Area, the 3 plates and the rack, with the green zone marked.
- [ ] **Base outline taped.** The base **must not move** until every scenario is recorded.
- [ ] **Blocks in fixed, taped spots:** cube on Plate B (left), cylinder on Plate A (centre), trophy on Plate B (right). Put them there the same way on game day.
- [ ] **Two people:** one moves the arm, one clicks.

Cells are named from **our** side of the rack:

```
r1c1 r1c2 r1c3     top
r2c1 r2c2 r2c3     middle
r3c1 r3c2 r3c3     bottom
```

## How to capture any pose

1. Press **LIMP** and support the arm.
2. Move it into position by hand.
3. Press **HOLD**.
4. Fine-tune with **− / +** using a small step.
5. Press **CAPTURE**.

The gripper stays powered while the arm is limp, so a block you're holding won't fall.

## Tab 1 — Setup poses (do this once, about 10 minutes)

- [ ] **STOW:** folded completely inside the Robot Area, including from above.
- [ ] **SAFE:** high over our side, clear of everything. Every move passes through here.
- [ ] **RACKWAIT:** facing the rack, with nothing over the green line.
- [ ] **grip open:** wider than the trophy.
- [ ] **grip cube**, **grip cylinder** and **grip trophy:** closed on each block, about 10° tighter than just touching.
- [ ] **grip attack:** fully closed, so the jaws act as a pusher.

If you change any of these later, press **Rebuild all saved scenario files**.

## Tab 2 — Record (41 scenarios)

For each scenario, select it, capture the rows marked NEED CAPTURE, press **▶ Test**, then **💾 Save**. **Next unfinished →** jumps to the next one.

| Group | Scenarios | What you capture | Block in the gripper? |
|---|---|---|---|
| Start and stow | `start`, `stow` | nothing (they come from tab 1) | no |
| Picks | `pick_cube`, `pick_cylinder`, `pick_trophy` | ABOVE, GRAB | the block sits on its plate |
| Attacks | `attack_r1c1` … `attack_r3c3` (9) | FRONT, HIT (a single push) | no |
| Placements | `place_<block>_<cell>` (27) | FRONT, IN | **yes:** close the gripper on the block with the **grip** buttons |

- [ ] start, stow
- [ ] 3 picks
- [ ] 9 attacks
- [ ] 9 cube placements
- [ ] 9 cylinder placements
- [ ] 9 trophy placements

The GUI tracks progress ("x/41 saved"). **Tell Claude after each group**, and it will check the files.

## Fluid motion

Every scenario now plays **smoothly**:
- **No stop-and-go.** The arm only stops at the precise spots (grab, block in the hole, hit, gripper changes) and glides through everything else.
- **Faster:** about 40% quicker.
- **Speed slider:** "playback speed ×" at the top changes the speed of every playback.

**Want it to move exactly the way you do?** Record a **fluid take** of the scenario.

1. Select the scenario and click **● Record fluid**. The arm goes to the scenario's start pose and goes LIMP after a 3-second countdown.
2. Move the arm through the **whole scenario** by hand, at the speed you want. For a grab or release, click the **gripper buttons** on the left.
3. Finish at SAFE and click **■ Stop**.
4. It's cleaned automatically, and the log shows what changed:
   - hand shake smoothed
   - idle time at the start and end trimmed
   - long pauses shortened, keeping time around grabs
   - both ends snapped exactly to SAFE, so scenarios still chain together
5. Click **▶ Test**. If you don't like it, click **Discard fluid (use waypoints)** or record it again.

Fluid takes are saved in `trajectories/`. **Commit that folder** along with `scenario_data.json`.

**After every push, Claude runs `clean.py` on your data.**
- It fixes values outside the limits and missing lifts.
- It writes `CLEANING_REPORT.md`, which lists anything you need to re-record.

## Tips

- **Do the top corners first** (`r1c1`, `r1c3`). They're at the edge of the arm's reach.
- **For a placement, put the block in the jaws first:** **grip open**, place the block, then **grip cube** (or the matching block).
- **A placement's IN pose** should leave the block **sitting stable on its own** (rule 3.3.2.4).
- **An attack's HIT is one push only, and it plays at maximum speed and acceleration.**
  - Aim it **through the hole**, never at the frame. Excessive force on the rack is a violation (rule 4.4), and so is hammering (rule 3.3.4.13).
  - Before pressing Test, check the HIT pose slowly with **Go to selected**.
- **⚠ outside the official ANGLE_LIMITS** in the log means playback will refuse that pose. Recapture it a little inside the limits.
- **Commit `scenario_data.json`** after each session so the team shares the same poses:
  `git add scenario_data.json && git commit -m "scenarios: <group>" && git push`
