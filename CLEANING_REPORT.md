# Cleaning report — 2026-11-05 16:20

**7/41 scenarios complete:** `start`, `stow`, `pick_cube`, `pick_trophy`, `attack_r1c1`, `attack_r1c2`, `attack_r1c3`

## Fixed automatically

- grip 'open': 643 (123.5°) outside the gripper limit -> 1024 (90.0°)
- attack_r1c3/FRONT: joint 2 3133 (95.4°) outside limit -> 3072 (90.0°)
- pick_trophy/ABOVE: joint 5 1020 (90.4°) outside limit -> 1024 (90.0°)
- pick_trophy: ABOVE was the same pose as GRAB (no lift) -> PROVISIONAL lift copied from pick_cube. RE-RECORD pick_trophy ABOVE.

## Needs your attention

- pick_cube: base turns 144 ticks between ABOVE and GRAB - the jaws come down sideways and may knock the block. Re-record ABOVE straight above GRAB.
- attack_r1c1: FRONT->HIT is 264 ticks (23°) - too far to be a push. Auto PREHIT keeps the full-speed part to the last 200 ticks; for a cleaner attack re-record FRONT ~2 cm in front of the hole.
- attack_r1c2: FRONT->HIT is 1116 ticks (98°) - too far to be a push. Auto PREHIT keeps the full-speed part to the last 200 ticks; for a cleaner attack re-record FRONT ~2 cm in front of the hole.
- attack_r1c3: FRONT->HIT is 1044 ticks (92°) - too far to be a push. Auto PREHIT keeps the full-speed part to the last 200 ticks; for a cleaner attack re-record FRONT ~2 cm in front of the hole.

## Fluid playback

| Scenario | Waypoints | Stop-and-go (est.) | Fluid |
|---|---|---|---|
| `start` | 2 | 1.9 s | 1.5 s |
| `stow` | 2 | 1.9 s | 1.5 s |
| `pick_cube` | 6 | 3.6 s | 2.0 s |
| `pick_trophy` | 6 | 4.8 s | 3.0 s |
| `attack_r1c1` | 8 | 7.0 s | 4.1 s |
| `attack_r1c2` | 8 | 8.9 s | 5.5 s |
| `attack_r1c3` | 8 | 8.5 s | 5.2 s |

## Still to record (34)

`pick_cylinder`, `attack_r2c1`, `attack_r2c2`, `attack_r2c3`, `attack_r3c1`, `attack_r3c2`, `attack_r3c3`, `place_cube_r1c1`, `place_cube_r1c2`, `place_cube_r1c3`, `place_cube_r2c1`, `place_cube_r2c2`, `place_cube_r2c3`, `place_cube_r3c1`, `place_cube_r3c2`, `place_cube_r3c3`, `place_cylinder_r1c1`, `place_cylinder_r1c2`, `place_cylinder_r1c3`, `place_cylinder_r2c1`, `place_cylinder_r2c2`, `place_cylinder_r2c3`, `place_cylinder_r3c1`, `place_cylinder_r3c2`, `place_cylinder_r3c3`, `place_trophy_r1c1`, `place_trophy_r1c2`, `place_trophy_r1c3`, `place_trophy_r2c1`, `place_trophy_r2c2`, `place_trophy_r2c3`, `place_trophy_r3c1`, `place_trophy_r3c2`, `place_trophy_r3c3`
