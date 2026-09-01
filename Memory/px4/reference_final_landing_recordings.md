---
name: reference_final_landing_recordings
description: test_data/Final/ — curated & committed IC1-5 perception-mode cross-marker landing recordings (montage + cams + full datasets)
metadata: 
  node_type: memory
  type: reference
  originSessionId: 7e5acaee-84eb-407d-8d98-d20cb3002914
  modified: 2026-09-01T03:45:33.258Z
---

**Location:** `PX4_Gazebo/test_data/Final/` — git-tracked (NOT under the per-subdir
`test_data` gitignore rules), committed + pushed 2026-09-01 (commit `b89f66b8`, main).
271 MB, 56 files, no LFS.

**Layout:** `Final/IC1/ … Final/IC5/`, plus `Final/MANIFEST.md`. Each `IC<n>/` holds:
- `IC<n>_montage.mp4` — combined view: onboard s/alpha overlay + onboard h/w overlay
  + chase cam + rel-position / rel-velocity / 3D-trajectory plot panel (built by
  `tools/overlay_image_features.py --split --draw-w` then `tools/make_landing_montage.py`).
- `IC<n>_onboard_cam.mp4`, `IC<n>_chase_cam.mp4` — raw source videos.
- `IC<n>_overlay_s_alpha.mp4`, `IC<n>_overlay_h_w.mp4` — the two overlay components.
- `dataset/` — full run: `Control_Data.npy`, `Control_Params.npy`, `Ground_Truth.npy`,
  `Img_Data.npy`, `Img_Params.txt`, `Telemetry_Data.npy`.

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
