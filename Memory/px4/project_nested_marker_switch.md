---
name: project_nested_marker_switch
description: "2026-07-03 perception-phase marker change: single 1m ArUco -> NESTED 0-small_10-big.png (2.0m plane) to fix perception-ON continuity. Big ID10 detectable 7-10m (54px@10m); small ID0 regenerated smaller (108px, ratio ID10/ID0=10.63) so it stays detected to 0.08m (below the 0.20m landed camera height). Layout {0:[0,0,1],10:[0,0,10.63]}. NEW default IMG_MARKER_PRIORITY=big (primary=biggest detected marker, kept while detectable) for flow observability. NEEDS output re-cal. CAVEAT: concentric nested = mutually-exclusive decode -> no corner spread -> flow stays rank-deficient (the graduated board would fix that; user chose nested)."
metadata: 
  node_type: memory
  type: project
  originSessionId: 3c2f4c67-05c1-4e6f-966b-0e62018fc8a7
---

**Marker change (2026-07-03, perception phase), driven by the perception-ON baseline
([[project_perception_on_baseline]]): the single 1m ArUco is lost mid/terminal descent
(overflows FoV at 0.42m; feature stale >1s -> open-loop fallback -> target_lost on all reps).
Switch to the NESTED marker for decode continuity + touchdown visibility.**

## What changed
- **World:** `~/PX4-Autopilot/.../models/arucotag/model.sdf` (OUTSIDE repo, backed up) —
  texture `arucotag.png` -> `0-small_10-big.png`; plane `1.0` -> **`2.0 2.0`**.
- **Marker image** `Images/0-small_10-big.png` (git-tracked; orig backed up
  `.bak_orig7ratio`): REGENERATED with a SMALLER inner marker — kept the big ID10 outer
  pattern, cleared the central black cell (±195px), placed a 108px ID0 (was 167px) with a
  thin white quiet-zone border. Also copied into the model dir.
- **Layout** `Images/aruco_board_layout.npy` (git-tracked; 13-marker board backed up
  `.bak_13marker`): now the nested `{0:[0,0,1.0], 10:[0,0,10.63]}` (concentric; ID10 = big).
- **Selection** `src/img_data.py`: `IMG_MARKER_PRIORITY` (default **"big"**, NEW) — primary =
  the BIGGEST detected marker (max layout size), kept while detectable, falling to the small
  only when the big is gone. Replaces the legacy smallest-ID `argmin` (=**"small"** reverts).
  Concentric -> centroid invariant across the switch; loom already size-normalizes by the
  primary's layout size (img_data:1367) so the scale is continuous.

## Sizing (verified offline, fx=270, 640x480, hfov=1.74; camera mounted +0.20m on x500 ->
## landed camera ~0.20m above marker)
- **Big ID10** (~1.94m, 0.97 of plane): detected 10m (54px) & 7m; overflows ~0.82m.
- **Small ID0** (108px design, ratio 10.63 -> ~0.18m): detected at touchdown crops
  0.20/0.15/0.10/**0.08m** — margin below the 0.20m landed height (the whole point; the old
  1m single marker overflowed at 0.42m -> lost before landing).
- Handover: big 7-10m, small takes over by ~3m, continuous to 0.08m.
- Reference: 1m single marker @7m = 38px (decodes, confirmed); @10m = 27px (marginal — why we
  sized to 2m).

## ⚠ Load-bearing caveats
1. **Concentric nested = MUTUALLY-EXCLUSIVE decode** (one ID per frame — verified; the project's
   own `make_aruco_board.py` docstring + [[feedback_single_marker_rank_deficiency]] say so). So
   the nested fixes DECODE continuity + touchdown visibility, but gives NO corner spread -> the
   6-DOF flow lstsq stays RANK-DEFICIENT (the low-altitude spurious-h / observability root). The
   graduated BOARD (separated markers decoding simultaneously) is the design that fixes BOTH;
   USER chose nested (2026-07-03). Big-priority mitigates (bigger marker = larger single-marker
   spread) but doesn't eliminate the rank deficiency.
2. **OUTPUT (optical-flow/centroid) cal MUST be re-derived** — marker geometry + size both
   changed (was 1m single). INPUT (rate/thrust) cal unaffected (drone dynamics, marker-independent).
   Terminal-commit-off is control-side, cal-irrelevant. GT-FB bypasses the cal entirely.
   Workflow: [[io-calibration skill]] — >=5 phased output-cal runs WITH FLOW_LAT_REDUCED=1 ->
   derive_board_cal.py -> derive_ring_cal.py.

## NEXT
Re-cal (output) -> re-run the perception-ON IC gate with the nested marker to check whether
decode continuity removes the target_lost/open-loop-fallback (and whether IC5's FoV-edge
acquisition improves). Continues [[project_perception_on_baseline]].

## ⛔ REVERTED to ORIGINAL image (2026-07-03, user directive "use original image")
My regenerated smaller-inner PNG is DROPPED. Now using the ORIGINAL user-tested
`0-small_10-big.png` (RGBA, inner ID0 167px). Re-measured actual nested ratio by
progressive downscale (big ID10 only decodes downscaled): **ID10/ID0 = 6.06** (the
`.bak_orig7ratio` filename's "7" was a rough earlier estimate; 6.06 is the direct
detection). **Layout now `{0:[0,0,1], 10:[0,0,6.06]}`** (the 10.63 / 108px-inner numbers
in "What changed" + "Sizing" above are the DEAD regenerated marker — superseded). The
regeneration was both unnecessary (original was already tested-detectable) and the source
of the grayscale bug below.

## ⛔ GOTCHA (2026-07-03, cost ~15 failed SITL attempts): gz marker PNG MUST be RGBA
The regenerated `0-small_10-big.png` was saved 1-channel GRAYSCALE (cv2.IMREAD_GRAYSCALE +
imwrite). gz's PBR `<albedo_map>` loader CANNOT handle a grayscale texture → gz HANGS loading
the model → sim clock stalls → PX4 lockstep DEADLOCK → `is_armable did not go True` (arming
timeout). Symptom: SITL processes in D-state (blocked, not CPU), load spikes ~14, EVERY cal/land
run fails to arm. RED HERRINGS chased first (all wrong): leaked DDS shm (264, cleared - not it),
orphaned setsid process groups (cleared - not it), ros2 daemon pollution (reset - not it),
machine load, even a REBOOT (fresh boot still failed → proved it was a CONFIG/ASSET issue).
DECISIVE test: revert model.sdf to the single arucotag.png (RGBA) → ARMED OK; nested grayscale →
fail. FIX: `cv2.cvtColor(gray, cv2.COLOR_GRAY2BGRA)` → 4-channel → arms fine. Originals
(arucotag.png, the .bak_orig7ratio nested) are all (H,W,4) RGBA. **RULE: any gz albedo_map PNG
must be RGBA; verify `cv2.imread(f, IMREAD_UNCHANGED).shape[2]==4` after generating a marker.**
**META-LESSON: when SITL arming/lockstep fails right after a world/model change, suspect the
CHANGED ASSET before machine state — the timeline (worked pre-change, fails post-change, reboot
doesn't help) pointed at the marker all along.**
