# GtfbMulti_col — 5 independent GT-feedback flights on cm_col

Recorded 2026-09-05. Purpose: validate the margin-gated ring+balance rescue
(`a893a77e`) at a much larger, closed-loop-confound-free sample than the single
`RobustnessFrameset/col` recording this thread had relied on until now, and
without the trajectory-divergence confound of the perception-mode SITL A/Bs
(`SceneFix_AB_harness`).

    ~/ws/scripts/env2025/bin/python3 tools/validate_detector_gt.py \
        --set test_data/GtfbMulti_col --variant ens_ring_balance

Bulk data gitignored (raw frames + Img_Data/Ground_Truth/Control_Data npy per
rep, ~145 MB); this manifest is tracked. Regenerate with
`record_col_gtfb_multi.sh` (tracked, same directory).

## How it was recorded

Same two deliberate choices as `RobustnessFrameset` (see its own MANIFEST):
`PLASMC_GT_FEEDBACK=1` (the detector must not gate the flight, so every rep
gets the same reference trajectory and offline scores are directly
comparable) and `CROSS_RING_OVERLAY_DBG=0` (verified clean, no drawn debug
pixels contaminating the frames).

WORLD=cm_col, MARKER_TYPE=cross, IC2 `INITIAL_DRONE_ENU=2.0,2.0,5.0`,
`IMG_RECORD=1` (raw PNGs, not the lossy video path).

| rep | frames |
|---|---|
| 1 | 544 |
| 2 | 574 |
| 3 | 567 |
| 4 | 575 |
| 5 | 555 |

## Result this validated

detOK, `CROSS_RING_BALANCE_RESCUE_MARGIN=1.0` (rescue thresholds unreachable,
approximates the pre-rescue "both stages must independently pass" behavior)
vs the landed default (margin=0.5), per independent flight:

| variant | rep1 | rep2 | rep3 | rep4 | rep5 | mean delta |
|---|---|---|---|---|---|---|
| ring_balance: no-rescue | 56.3 | 52.5 | 51.9 | 55.6 | 55.0 | |
| ring_balance: rescued | 59.7 | 60.9 | 58.4 | 61.6 | 63.2 | **+6.5pt, every rep +** |
| ens_ring_balance: no-rescue | 59.2 | 54.5 | 53.0 | 56.5 | 54.3 | |
| ens_ring_balance: rescued | 62.1 | 61.2 | 57.9 | 61.4 | 58.7 | **+4.8pt, every rep +** |

within-0.15 (accuracy): flat to +1pt mean on both variants, no regression in
any of the 10 rep-comparisons. The rescue improves detection rate on every
single one of 5 independently-recorded flights, for both variant
combinations, with zero accuracy cost — no outliers either direction.

⚠ This validates what the detector REPORTS, not closed-loop landing outcome
in perception mode (see `SceneFix_AB_harness` for that, separately
confounded and only partially conclusive). `ens_ring_balance` overall stays
DEFAULT OFF pending more closed-loop evidence; the rescue itself is landed.
