# OverfillCapture_IC2to5 — 1 fresh real-perception flight each, IC2-5, plain cross_marker world

Recorded 2026-09-08, same day/session as `OverfillCapture_IC1` (see its MANIFEST for the
full rationale). Purpose: confirm the IC1 line-width validation generalizes across the
full IC1-5 set, not an IC1-only or lucky-rep effect.

    for ic in IC2 IC3 IC4 IC5; do
      HEADLESS=1 WORLD=cross_marker MARKER_TYPE=cross INITIAL_DRONE_ENU=<ic's ENU> \
          IMG_RECORD=1 LANDING_AUTOSAVE=1 MAX_ATTEMPTS=5 \
          bash scripts/run_aruco_landing_retry.sh
    done
    # ENUs: IC2=2.0,2.0,5.0  IC3=-2.0,2.0,5.0  IC4=2.0,2.0,7.0  IC5=2.0,2.0,3.0

Real perception mode (NOT GT-feedback). Bulk data gitignored; manifest + `run.sh` +
per-IC `.log` + `driver.out` + `raw_dirs.txt` (raw-frame directory paths, needed since
raw frames land under the separately-timestamped, already-gitignored
`test_data/Test_Videos/<timestamp>_raw/`) are tracked. Regenerate with `run.sh`.

## Outcomes

| IC | classification | xy_err | rel_vel | min_alt |
|---|---|---|---|---|
| IC2 | PRECISE-only | 0.087 m | 0.590 m/s | 0.02 m |
| IC3 | FAIL | 0.172 m | 0.518 m/s | 0.15 m |
| IC4 | FAIL | 0.202 m | 0.540 m/s | 0.02 m |
| IC5 | FAIL | 0.326 m | 1.140 m/s | 0.04 m |

All 4 reached genuine deep overfill (min_alt 0.02-0.15 m) without a catastrophic
ascent event, same pattern as `OverfillCapture_IC1`.

## Result this validated

Mask-scan width vs. pruned-inlier width, `corr(ln(width), -ln(alt))` on frames with
alt < 1.0 m — mask-scan wins in every IC, confirming full IC1-5 generalization
(combined with IC1's 3 reps: 7/7 reps, mask-scan beats pruned-inlier every time):

| IC | old (pruned-inlier) | new (mask-scan) |
|---|---|---|
| IC2 | 0.92 | 0.96 |
| IC3 | 0.97 | 1.00 |
| IC4 | 0.58 | 0.90 |
| IC5 | 0.68 | 0.94 |
