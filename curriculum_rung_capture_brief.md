# Capture brief: one close-up per curriculum rung (Thesis C report)

**For:** the simulation PC (`~/Go2_RL_workflow`), run by hand or by Claude Code there.
**Purpose:** (1) a close image of **every trained rung** of the four lineage curricula, with
the robot **walking through the field**, so that each ladder can be shown as a labelled
collage in the style of report Figure 35 (`Thesis_C_report/Figures/height_rows_collage.jpg`,
`fig:height_rows_terrain`); untrained rungs are shown without the robot. (2) Images of the
RoboCupRescue competition lanes for future work (Set R, Section 6).
**Deadline:** report due 2026-11-20. Do Sets C and P first, then G, then R.
**Written:** 2026-11-18.

---

## 1. Policy

Use the **final policy (the recreated policy)** for every image:

| | |
|---|---|
| Run | `unitree_go2_omni30_mem6_fwd_direct_usb2/2026-11-15_07-21-20` |
| Checkpoint | `training/logs/rsl_rl/unitree_go2_omni30_mem6_fwd_direct_usb2/2026-11-15_07-21-20/model_999.pt` (exported copy in `results/head_rule/policies/`) |
| Task (`--task`) | `omni30-mem6-fwd-direct-usb2` |

The policy's observations stay its own on every ladder. Only the ground is swapped, with
`--terrain_from` (see `training/scripts/eval_fixed_rung.py`, "BENCHMARK TERRAIN").
Record the policy in the index for every image.

## 2. Target look: match Figure 35

Figure 35 is a 3-column grid of stills. Each still shows **one rung**, with the robot partway
across the field and the field's blocks around it. Reproduce that:

- **Simulator:** Isaac Lab.
- **Camera:** the same camera as Figure 35, kept identical for every rung of every ladder.
  This should be the default tracking camera (`--clip_camera follow`). Before running the
  whole set, film one rung, compare it with `height_rows_collage.jpg`, and switch camera or
  `--clip_down_deg` if the framing does not match. Write the setting used in the index.
- **Robot:** moving, at the 0.3 m/s forward command with heading hold, **in the middle of
  the field** (not on the start border, not past the end). On stair rungs, use a frame on
  the stairs.
- **Look:** OSB wood on turf (`--clip_look`), lane walls off (the default), pushes off
  (`--no_pushes`), D435i model on (the default).
- **Fall rule:** trunk only (the default; do not pass `--head_terminal`).
- **Resolution:** 1920×1080 PNG, with no overlays, timestamps or debug markers.
- **Labels:** none in the raw stills. Labels go on the collage only (Section 5).

## 3. Shot list

Every trained rung, numbered as in the report's appendix tables. **Check the row labels**
that `eval_fixed_rung.py` prints at start-up against these tables before filming. If a
row's printed label does not match, stop and note it in the index; do not renumber.

### Set C: climb-30 ladder + direct rows, rows 0–18 (MUST)

Terrain: the policy's own (`--task omni30-mem6-fwd-direct-usb2`, no `--terrain_from`).
Rows 0–15 are the climb-30 ladder used for the whole lineage; rows 16–18 are the direct-riser
rows of the final fine-tune.

| Row | Terrain | Clip mode |
|---|---|---|
| 0 | Flat | `pass` |
| 1 | Random bumps, 0.5–4 cm | `pass` |
| 2 | 10 cm stairs, climb | `up_over` |
| 3 | 10 cm stairs, descent | `up_over` |
| 4 | 15 cm stairs, climb | `up_over` |
| 5 | 15 cm stairs, descent | `up_over` |
| 6–8 | Single-level fields, 10 / 12.5 / 15 cm | `pass` |
| 9–10 | Height ramps, 0/10/20 and 0/12.5/25 cm | `pass` |
| 11 | Band `staircase_wide` (0→15→30→15→0 cm) | `pass` |
| 12 | Band `staircase_short` | `pass` |
| 13–15 | 0/15/30 cm fields, 5 / 15 / 37.5 % of tops at 30 cm (p.34 rule) | `pass` |
| 16–18 | The same three fields without the p.34 rule (direct risers) | `pass` |

The existing A3 close-ups (`results/clean_repro/curriculum_images/A3_row*.png`) used the
fixed quarter camera, so they do **not** match Figure 35. Re-film all 19 rows for this set.

### Set G: progressive ladder, 18 rungs, rungs 0–13 (MUST)

Terrain: `--terrain_from progressive-lane-fullscan` (the full 18-row ladder).

**Rungs 14–17 (tilted stepfields) were never trained, so capture them WITHOUT the Go2.**
Use terrain-only stills with the same camera pose as the other Set G stills: film the take,
then hide the robot (or render the terrain with no robot spawned), so that the framing still
matches. Name them `G_row14_terrain.png` … `G_row17_terrain.png`. In the index, mark them
"not trained, terrain only".

**Also save the same frame with the Go2 visible** (`G_row14.png` … `G_row17.png`). Take both
images from the same take and the same frame, once with the robot shown and once with it
hidden, so the pair differs only in the robot. Do not retry to get a crossing: these rungs
were never trained, so a stall or fall is expected. Mark it in the index.

| Rung | Terrain | Clip mode |
|---|---|---|
| 0 | Flat | `pass` |
| 1 | Random bumps, 0.5–4 cm | `pass` |
| 2–3 | 10 cm stairs, climb and descent | `up_over` |
| 4–5 | 15 cm stairs, climb and descent | `up_over` |
| 6–8 | Single-level fields, 10 / 12.5 / 15 cm | `pass` |
| 9–10 | Height ramps, 0/10/20 and 0/12.5/25 cm | `pass` |
| 11–12 | 0/15/30 cm fields, 5 and 15 % of raised cells at 30 cm | `pass` |
| 13 | Stepfield, 37.5 % at 30 cm | `pass` |
| 14–17 | Tilted stepfield, 3.75 / 7.5 / 11.25 / 15° (**not trained: terrain only, no Go2**) | — |

Rungs 0–8 have the same geometry as climb-30 rows 0–8, but film them again on this ladder
so that each collage is complete on its own.

**The "progressive ladder with straddle spawns"** (the third lineage curriculum) is this
same terrain with drop-in spawns, so it needs **no separate stills**. Set G covers it.

### Set P: perception ladder, rungs 0–6 (MUST)

Terrain: `--terrain_from percept-teacher`. Training was capped at rung 6
(`PERCEPT_MAX_RUNG = 6`, `training/go2_training/envs/progressive_lane_percept_env_cfg.py`).
Under `--cfg train` the ladder loads with rows 0–6 only. Under `--cfg play` it loads all
11 rows: film only rows 0–6.

| Rung | Terrain | Clip mode |
|---|---|---|
| 0 | Flat | `pass` |
| 1 | 15 cm stairs, climb | `up_over` |
| 2 | 15 cm stairs, descent | `up_over` |
| 3–4 | Single-level fields, 10 and 15 cm | `pass` |
| 5 | Sparse single-level field, 15 cm, 30 % of cells raised (`SPARSE15_RUNG`) | `pass` |
| 6 | Single-level field, 20 cm | `pass` |

## 4. How

Follow the `clips()` / `take()` / `still()` helpers in
`training/scripts/curriculum_capture_chain.sh`. It is simplest to add a new section to that
script (e.g. `want rungs`), or to copy it as `rung_capture_chain.sh`. One
`eval_fixed_rung.py` boot per set:

```bash
CKPT=training/logs/rsl_rl/unitree_go2_omni30_mem6_fwd_direct_usb2/2026-11-15_07-21-20/model_999.pt

# Set C (own terrain). Repeat with --terrain_from progressive-lane-fullscan (Set G)
# and --terrain_from percept-teacher (Set P), with that set's clip list.
python scripts/eval_fixed_rung.py --task omni30-mem6-fwd-direct-usb2 \
  --policies recreated=$CKPT --cfg train --num_envs 1 --seed 42 \
  --no_pushes --speed 0.3 --levels 0 \
  --record_only --record_dir "$VID/.raw" \
  --record_clips 0:4000:pass 1:4000:pass 2:4000:up_over 3:4000:up_over ... 18:4000:pass \
  --clip_camera follow --clip_log --clip_look --episode_s 45 --exit_margin 1.0
```

Then take one still per clip with `scripts/clip_still.py <clip>.mp4 --pick mid` (the frame
where the base is halfway along the course). If `mid` lands on a frame where the robot is
hidden or between blocks, use `--pick top` or `--pick t:<seconds>`, and note the choice in
the index.

If the robot does not cross a rung, `take()` falls back to the `_best` clip. Use that clip,
and mark the row **"best attempt, not crossed"** in the index. Do not re-run with another
seed to get a crossing.

**Output folder:** `results/clean_repro/curriculum_images/rungs/`

**File names:** `C_row{NN}.png`, `G_row{NN}.png`, `P_row{NN}.png` (row number zero-padded).
Keep the clips as `videos/{C,G,P}_row{NN}.mp4`.

## 5. Collages

Make one collage per set, built like Figure 35:

- three columns, rows in reading order;
- every tile the same size and crop;
- a white label box in the bottom-left corner of each tile, in bold sans-serif, giving the
  terrain in the same form as Figure 35 (e.g. "Flat", "Bumps", "10 cm stairs up",
  "0/15 cm", "0/15/30 cm, 37.5 %").

| Set | Tiles | Collage |
|---|---|---|
| C | 19 | `collage_climb30.jpg` |
| G | 14 with the Go2 + 4 terrain only (rungs 14–17, label suffix "(not trained)") | `collage_progressive.jpg` |
| P | 7 | `collage_perception.jpg` |

For Set C, a 3×7 grid leaves two empty tiles. Leave them blank; do not stretch the tiles.
A short Pillow script is enough to build the collages. Keep the raw PNGs as well, so that
labels can be changed later.

## 6. Set R: RoboCupRescue competition lanes (future work)

**Purpose:** images of the RoboCupRescue 2026C arenas, for the report's future-work
section. Most of them are shown empty. The Go2 appears **only** on the stepfield, K-Rail and
pallet lanes.

**Scene:** the baked competition hall (playback route), `docs/competition_lanes.md`:

```bash
ros2 launch go2_bringup sim_rl.launch.py mode:=competition lane:=<key>
```

Switch lanes from the "Go2 Scene" window (lanes are switched by teleport). If the stage is
missing or stale, rebuild it first with `src/go2_sim_isaaclab/scripts/build_competition_usd.sh`.

### R1. Terrain only, no Go2 (all 15 lanes)

For empty shots, hide the robot: make its prim invisible in the Isaac Sim stage tree, or
teleport it out of frame. Record the method used in the index.

**Every R1 image is taken twice: with the Go2 and without it.** Use the same camera pose
and the same moment. First save the image with the robot standing (or walking) on the lane
at its spawn point, then hide the robot and save again. Name them `R1_<key>_overview.png` /
`R1_<key>_overview_nogo2.png` and `R1_<key>_closeup.png` / `R1_<key>_closeup_nogo2.png`.
On lanes outside the R2 list, the robot is only there for scale. Use the recreated policy
standing still at 0 m/s, and do not try to cross.

| Arena | Lane keys |
|---|---|
| Shifty Gravel | `gravel` |
| Diagonal K-Rails | `krails`, `krails_slopes`, `krails_obstacles` |
| K-Rail Square | `square` |
| Half-Cubic Stepfields | `stepfields`, `stepfields_slopes`, `stepfields_obstacles` |
| Pitch/Roll Ramps | `ramps` |
| Center in Alleys | `alleys` |
| Pallets & Pipes | `pallets` |
| Push/Pull Doors | `doors` |
| Avoid Holes/Posts | `avoid` |
| Stair Debris / Pallet Climb | `stairs` (clear) |
| Search & Map Maze | `maze` |

Not needed: `ramps_slopes`, `ramps_obstacles`, `alleys_slopes`, `stair_debris`.

**Two images per lane:**
- **Overview:** a high three-quarter view of the whole lane from the entry end, with the
  same camera pose relative to each lane. → `R1_<key>_overview.png`
- **Close-up:** the lane's key feature filling most of the frame (e.g. the rails, a pallet
  and pipe, the gravel surface, a door, a hole or post, the stair, a maze junction), from
  a lower camera angled down. → `R1_<key>_closeup.png`

### R2. With the Go2 (stepfield, K-Rail and pallet lanes only)

Each lane is walked by the specialist policy for that terrain:

| Lanes | Policy (`policies/`) | Caveat |
|---|---|---|
| `stepfields`, `stepfields_slopes`, `stepfields_obstacles` | Recreated policy (Section 1), exported ONNX from `results/head_rule/policies/` | Trained on the climb-30 ladder, not on the hall lane |
| `krails`, `krails_slopes`, `krails_obstacles` | `unitree_go2_krail_percept_2026-10-28_04-50-38` | Trained on the K-Rail course terrain (`krail-percept`), not on the lane |
| `pallets` | `unitree_go2_pallet_fwd_finetune_usb2_2026-11-05_23-41-48` | Trained on the flush two-layer grid pallet. The hall's pallets are built from the one-layer vendored geometry and have pipes (`docs/grid_pallet_task.md`) |

Select each policy in the policy panel (`rl_policies.json`). If the recreated policy is
missing there, add it from its exported copy. Use the perception mode that the policy's
`deploy.yaml` asks for: the `heightmap_mode:=auto` default handles this.

- **Camera:** the Figure 35 follow-style framing (Section 2), with the robot in the middle
  of the lane, plus the R1 overview pose with the robot in shot.
- **Command:** forward at 0.3 m/s from the entry end.
- **Failures:** if the robot stalls or falls, keep the best frame of it **on** the
  terrain, and mark the index row "did not cross". Do not swap policies or retune to get
  a crossing.
- **Files:** `R2_<key>_follow.png`, `R2_<key>_overview.png`, plus the video
  `videos/R2_<key>.mp4`.

**Output folder:** `results/clean_repro/curriculum_images/competition/`. Use the same index
format as the other sets, with the lane key, policy, camera, robot shown or hidden, and
whether the robot crossed.

No collage is needed for Set R. The report will pick from the raw images.

## 7. When done

1. Check every still: the robot is in the middle of the field, the framing is the same
   across all three sets, and there are no overlays.
2. Add a row for each image to `results/clean_repro/curriculum_images/captures_index.md`.
   Use the existing columns (ID | video | image | timestamp | notes). The notes must give
   the set, row, terrain, policy, camera, fall rule, crossed / best attempt, and the
   command used.
3. Under "Not captured / caveats" at the bottom of the index, list any rung that could not
   be filmed and any row whose printed label did not match the tables above.
4. Sync `results/clean_repro/curriculum_images/rungs/` and `.../competition/` back to the OneDrive copy of the
   repo. `results/` is git-ignored, so use `git add -f` or copy the files.
