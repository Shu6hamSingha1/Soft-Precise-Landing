---
name: reference_final_landing_recordings
description: "test_data/Final/ — curated & committed IC1-5 landing recordings (montage + cams + full datasets). ⭐ REPLACED 2026-09-23 (commit 85faa9f1): now GT-FB rover_cross-world cross-marker, 5/5 SoftPrecise (xy 0.009-0.030m, rel_vel 0.095-0.111m/s), first launch attempt each. The prior perception-mode cross_marker set (1/5 verified precise, IC5's precise flag was a mid-air false-positive per the 2026-09-02 correction below) is REMOVED from Final/ but recoverable from git history (pre-85faa9f1)."
metadata: 
  node_type: memory
  type: reference
  originSessionId: 7e5acaee-84eb-407d-8d98-d20cb3002914
  modified: 2026-09-23T00:00:00.000Z
---

## ⭐ CURRENT (2026-09-23, commit `85faa9f1`, replaces everything below)

**Config:** main @ `5b342689`. `PLASMC_GT_FEEDBACK=1` (GT-fed s/h, isolates CONTROL from
PERCEPTION — NOT the old perception-mode path), `WORLD=rover_cross ROVER_MODEL=rover_cross
ROVER_MOTION=0` (stationary target on the rover platform, not `cross_marker` world),
`MARKER_TYPE=cross`, default params, 1280x960 chase cam.

**Layout unchanged**: `Final/IC1/ … Final/IC5/` + `Final/MANIFEST.md`, same 5-file-per-IC
pattern (`IC<n>_montage.mp4`, `IC<n>_onboard_cam.mp4`, `IC<n>_chase_cam.mp4`,
`IC<n>_overlay_s_alpha.mp4`, `IC<n>_overlay_h.mp4` [renamed from `_overlay_h_w.mp4`],
`dataset/`). Built via the [[reference_finalized_montage_video_layout]] recipe
(`tools/overlay_image_features.py` x2 + `tools/make_landing_montage.py`,
`--tail-s 1.0 --chase-crop-touchdown 0.5 --chase-crop-ramp-s 3.0`).

**Per-IC outcome — ALL 5 SoftPrecise on the FIRST launch attempt, no retries:**
| IC | init ENU | xy_err | rel_vel | source run |
|----|----------|--------|---------|------------|
| IC1 | 0,0,5  | 0.0093 m | 0.105 | `Tue Sep 22 23-34-24 2026` |
| IC2 | 2,2,5  | 0.0125 m | 0.095 | `Tue Sep 22 23-35-44 2026` |
| IC3 | -2,2,5 | 0.0298 m | 0.100 | `Tue Sep 22 23-37-06 2026` |
| IC4 | 2,2,7  | 0.0215 m | 0.105 | `Tue Sep 22 23-38-31 2026` |
| IC5 | 2,2,3  | 0.0138 m | 0.111 | `Tue Sep 22 23-39-48 2026` |

All 5 clear the manuscript's strict 0.08m/0.2m/s thresholds directly (not just the
0.10m/0.5m relaxed harness gate). Chase video verified frame-by-frame on IC1 (0/213
identical consecutive frame pairs) — no freeze artifact; a same-day peer-session concern
about a chase-cam render-stall bug (from a resolution-drop A/B) was raised then RETRACTED
(bad whole-frame-mean-diff methodology masked real motion against a static background;
re-checked properly, 0/206 pairs were actually identical) — no such bug exists, don't
re-raise it.

**Why this replaced the perception-mode set**: same 5 spawn positions, but the OLD set
missed precise/soft on IC1/IC3/IC4 (see below) while this GT-FB set lands all 5 cleanly —
strong evidence those misses were perception-side (calibration/overfill/etc.), not a
control-law weakness at these ICs. If a perception-vs-control comparison is ever needed
again, the removed data is at git commit `a6f288c9` (parent of the replacement) or earlier.

---

## HISTORICAL (perception-mode set, REMOVED from Final/ 2026-09-23, recoverable from git
## history pre-`85faa9f1` — kept below for its own correction, not as current state)

**Location:** `PX4_Gazebo/test_data/Final/` — git-tracked (NOT under the per-subdir
`test_data` gitignore rules), committed + pushed 2026-09-01 (commit `b89f66b8`, main).
271 MB, 56 files, no LFS.

**Config for all runs:** main @ `4d7bc210` (post-`ebb8093c`-revert baseline),
perception feedback (NO `PLASMC_GT_FEEDBACK`), `MARKER_TYPE=cross`,
`WORLD=cross_marker`, `CROSS_ALPHA_0=radians(0.58)`, default params.

**Per-IC outcome (from MANIFEST.md):**
| IC | init ENU | xy_err | rel_vel | precise | source run |
|----|----------|--------|---------|---------|------------|
| IC1 | 0,0,5   | 0.122 m | 0.420 | No  | Landing_Test/`Mon Aug 31 16-17-08 2026` (alpha0 batch) |
| IC2 | 2,2,5   | 0.065 m | 0.364 | **Yes** | Landing_Test/`Mon Aug 31 16-18-23 2026` |
| IC3 | -2,2,5  | 0.304 m | 0.777 | No  | Landing_Test/`Mon Aug 31 19-17-04 2026` |
| IC4 | 2,2,7   | 0.113 m | 0.394 | No  | Landing_Test/`Mon Aug 31 19-18-26 2026` |
| IC5 | 2,2,3   | 0.058 m | 0.505 | **Yes** | Landing_Test/`Tue Sep  1 08-44-32 2026` (re-recorded 2026-09-01) |

**⛔ CORRECTION 2026-09-02 — IC5's "PRECISE 0.058 m" is NOT a verified landing.** Its
dataset ends with the drone at **0.422 m above the marker plane, still descending
(-0.065 m/s)**, `B_T` collapsed to +0.030 (IC2 at the same point: -1.197), `MARKER_EXTENT_PX`
saturated at 318 — the terminal-overfill signature. All five GT logs stop 4.4-4.8 s before
their control logs (post-touchdown tail), so GT end = the touchdown latch; IC1-4 latch at
0.04-0.14 m moving UPWARD (post-contact bounce), IC5 latched 0.42 m up mid-descent. The
`precise` flag is computed on that mid-air sample. **Final is 1/5 verified precise (IC2), not
2/5** — and IC5's "attempt 8/8 landed precise" selection was made on the same unguarded
metric. Also: IC4's initial ENU is off spec by **0.276 m** (1.751,1.882,7.014 vs 2,2,7),
outside the "<=0.25 m" pairing bound claimed below. Everything else verifies clean (all 5
SoftPrecise values match MANIFEST exactly; 25/25 videos decode; 56 files tracked).
See [[project_20260902_archive_rescore_false_precise]].

**Caveats:**
- IC1/IC3/IC4 are misses (rel_vel / xy over gate) — they are `IMG_RECORD=1` runs and
  IMG_RECORD perturbs touchdown (only ~2/16 precise across two recording batches; the
  disturbance is noted in `cross_marker_perception.py` ~line 2303). The clean-touchdown
  baseline for IC1-4 (no video) is `test_data/ICValidation/20260831-144626/` (post-alpha0
  n=5 gate: IC2/3/4 ~0.10 m mean, 3/5 precise each).
- IC5's `Final` montage was re-recorded 2026-09-01 (attempt 8/8 landed precise 0.058 m);
  it replaces the earlier `montage_IC5_alpha0_20260831.mp4` which was a 6.44 m TARGET_LOST.
- Video<->dataset pairing was validated: each dataset's initial UAV ENU matches its IC
  spec (<=0.25 m), onboard-video length matches the descent segment, montage 3D-plot start
  matches the dataset IC, chase clips visually confirm a marker landing. IC1-4 chase<->
  dataset link rests on save-time adjacency (45-56 s offset) + visual confirmation (chase
  cam carries no telemetry for a frame-exact check).

See [[reference_test_record_system]] for the broader test-data layout, and the
2026-08-31 perception-mode session log for how these landings were produced
([[project_20260831_perception_mode_landing]] if present).
