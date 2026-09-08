# OverfillCapture_IC1 — 3 fresh real-perception IC1 flights, plain cross_marker world

Recorded 2026-09-08. Purpose: validate the line-width mask-scan loom method (and the
`origin_ratio` double-gate fix) against a fresh real-perception capture of the actual
target world/scenario, after all the earlier validation this session ran on the
`cm_col` color-variant proxy world. See `project_20260908_line_width_loom_investigation`
memory for the full derivation and results.

    HEADLESS=1 WORLD=cross_marker MARKER_TYPE=cross INITIAL_DRONE_ENU=0.0,0.0,5.0 \
        IMG_RECORD=1 LANDING_AUTOSAVE=1 MAX_ATTEMPTS=5 \
        bash scripts/run_aruco_landing_retry.sh

Real perception mode (NOT GT-feedback) — the detector gates the flight, matching the
reference failed rep (`ICValidation/20260831-144626/IC1_rep1`) this whole investigation
traces back to.

Bulk data gitignored (Control/Ground_Truth/Img_Data/Telemetry `.npy` per rep); this
manifest + `run.sh` + per-rep `.log` + `driver.out` are tracked. Regenerate with `run.sh`.
Raw frames (`IMG_RECORD=1`) land separately under
`test_data/Test_Videos/<timestamp>_raw/` (already gitignored) — pair to a rep by capture
order / mtime, not by name (see the memory file's "IC1 confirmation" section for exact
timestamps from this recording).

## Outcomes

| rep | classification | xy_err | rel_vel | min_alt |
|---|---|---|---|---|
| 1 | FAIL | 0.108 m | 0.530 m/s | 0.01 m |
| 2 | PRECISE-only | 0.083 m | 0.434 m/s | 0.06 m |
| 3 | FAIL | 0.267 m | 0.240 m/s | 0.06 m |

None reproduced the exact catastrophic ascent from the reference failed rep — all 3
landed close, missing only on precision/velocity thresholds. All 3 reached genuine deep
overfill (min_alt 0.01-0.06 m), which is what the validation needed.

## Result this validated

Mask-scan width vs. pruned-inlier width, `corr(ln(width), -ln(alt))` on frames with
alt < 1.0 m (offline replay against the raw frames + `Img_Data.npy`'s `Quat`/`Time`):

| rep | old (pruned-inlier) | new (mask-scan) |
|---|---|---|
| 1 | 0.54 | 0.89 |
| 2 | 0.94 | 0.99 |
| 3 | 0.71 | 0.92 |

Mask-scan wins in every rep. Live-wiring (via the actual `process_frame()` entry point,
not the offline replay) reproduced rep1's 0.89 as 0.886 — confirms the production
`width_loom_from_detection()` implementation matches the validated offline method.
