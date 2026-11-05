# Recording guide: when and what to record

## When to record

Record only once **all** of these are true. If you record earlier, you'll have to redo it.

1. **The arm is finished mechanically.** That means the final gripper and any attachments are fitted and tightened, and the cables are tied down where they'll stay. Any change to the gripper or a link moves every pose.
2. **The servos are calibrated.** The middle position has been set (the web tool's calibrate step), and `python3 record.py --mock` works on the Pi.
3. **You're on a copy of the field built to the exact rulebook dimensions** (Figure 7, ±5%). That means:
   - the Robot Area outline,
   - the three supply plates,
   - the rack at the correct distance and height from the base, with the green zone marked.

   You **can't** calibrate on the real game field (rules 3.1.9c and 3.4.7b). Everything is recorded relative to the arm's base, so the base must sit in the same spot on game day. **Mark the base outline with tape** on the copy, and line it up the same way on the real field.
4. **You have all three real blocks:** a cube, a cylinder and a trophy.
5. **There are two of you:** one moves the arm, the other types.

**Re-record** a pose if any of these happen:
- the base is moved or remounted,
- a servo is swapped or recalibrated,
- the gripper is changed,
- the test run (`t`) or `--verify` misses.

Only redo the poses that are affected. For example:

```bash
python3 record.py --redo place/trophy/r1c3/in
```

Total time is about **45–60 minutes** for all 82 poses. The tool saves after every capture, so you can stop (`q`) and carry on later just by running it again.

## Order (the tool asks for poses in this order)

| # | Group | Poses | What you're holding |
|---|---|---|---|
| 1 | Global | `stow`, `safe`, `rack_safe`, `gripper_open` | nothing |
| 2 | Pick | above + grip, for the cube, cylinder and trophy | the block sitting on its plate |
| 3 | Attack | front + hit, for each of the 9 cells | nothing; jaws closed |
| 4 | Place | front + in, for each block in each of the 9 cells | **that block, in the gripper** |

For the place group, the tool **picks the block up itself** using the pick poses you just recorded, so it tests them too. Between block types it tells you to catch the old block.

## Capturing one pose

```
> l        arm joints go LIMP. Hold the arm before pressing Enter!
           (person 1 moves the arm into place by hand)
> [Enter]  captured, saved, and the arm holds there
> t        optional: goes to 'safe' and comes back. Check it lines up.
```

Fine-tune with the joints still powered instead of moving the arm by hand:

```
> j1 +10   base       > j2 -15   shoulder      > j3 +5   elbow     > j4 -8   wrist
> g+ / g-  gripper    > x        close the gripper until it feels the block
> s        jump to a similar pose that's already recorded, then just nudge it
```

(4096 ticks = 360°, so 10 ticks is about 0.9°.)

**Seeding saves a lot of time.** For example:
- Cylinder poses start from the matching cube pose for the same cell.
- Each cube "front" pose starts from the attack "front" pose for that cell.

So after the first block, most poses are just `s`, a couple of nudges, then Enter.

## Tips for each pose

- **stow:** fold the arm completely inside the Robot Area, including from above. The referee checks this.
- **safe:** high and clear of everything. Every move passes through this pose, so a bad `safe` ruins every move.
- **rack_safe:** facing the rack, with **nothing over the green line**. You wait here before calling "Attack".
- **gripper_open:** open just wider than the trophy. Opening wider wastes time.
- **pick grip:** grip low on the block so it doesn't tip over. Use `x` to close the gripper. Check that the block is flat in the jaws.
- **place in:** the block should sit fully in the hole and be **stable when released** (rule 3.3.2.4). Approach straight through `front`. Don't touch the rack frame.
- **attack hit:** a single push only, deep enough to knock out a block on the far side. Repeated hammering is a violation (rule 3.3.4.13).
- **Test the edges:** do the top corners (`r1c1` and `r1c3`) first. They're nearest the arm's reach limit. If you can't reach a cell, tell the team now.

## Checking before game day

```bash
python3 record.py --status            # coverage table: everything should be 9/9
python3 record.py --verify attacks    # runs all 9 attacks slowly (keep the rack empty)
python3 record.py --verify picks
python3 record.py --verify place:cube # 9 real placements; repeat for cylinder and trophy
```

Then commit `poses.json` and push it, so the whole team has the same poses.
