---
name: aggregate-calibration
description: "aggregate_calibration.py methodology — outlier-rejected median(gt/raw), validity gate, known NaN-gate issue"
metadata: 
  node_type: memory
  type: reference
  originSessionId: 367ac3aa-13f8-41ea-b23e-c929f5e1e50d
---

## Two aggregators exist as of 2026-06-01

| File | Method | Use when |
|---|---|---|
| `aggregate_calibration.py` | Per-run outlier-rejected `median(gt/raw)` → trimmed-mean across runs (drop top/bottom 1) | Mixed/legacy recordings without `gt['Phase']` tag |
| **`aggregate_calibration_phased.py`** | **Phase-segmented**: each cal axis uses ONLY samples tagged with its excitation phase (e.g. flow_z from `phase=='z'` only); std-ratio (σ_GT / σ_raw_filtered) per axis, median across reps | **Default for new runs** (record_output_calibration.py since 2026-05-29 writes `gt['Phase']`). Decouples cross-axis noise. |

**Not** std-ratio across all samples — that's a different ad-hoc method I wrote in standalone diagnostic scripts; ignore those. The two files above are the supported aggregators.

## Methodology

### Per-axis cal (within one run)

```python
def robust_scale(raw, gt_v, threshold):
    mask = np.isfinite(raw) & np.isfinite(gt_v) & (np.abs(gt_v) > threshold) & (np.abs(raw) > 1e-9)
    if mask.sum() < 10: return float('nan')
    return float(np.median(gt_v[mask] / raw[mask]))

# thresholds:
#   flow axes: |gt| > 0.05  rad/s
#   ω axes:    |gt| > 0.02  rad/s
#   centroid:  |gt| > 0.05  (normalized)
```

The threshold filter is the **outlier rejection** layer. Drops samples where GT is near zero (ratio dominated by noise) and samples where raw is essentially zero (likely extrapolation). Median is robust to remaining outliers.

### Per-run validity gate

```python
def is_valid_run(B_x_tu, B_v_tu):
    mid_z = np.median(B_x_tu[n//4 : 3*n//4, 2])  # median altitude during middle half
    return (mid_z > 1.0) and (np.max(np.abs(B_v_tu)) < 10.0)
```

Rejects runs where drone is too low (`mid_z ≤ 1`) or moving unphysically fast (`peak_v ≥ 10`). Whole run dropped, not per-sample.

### Aggregation across runs

**Trimmed-mean** (switched from median on 2026-06-01): drop top and bottom 1 value per axis, mean the rest. Implemented inline in `main()` as `trimmed_mean(arr, trim=1)`. Falls back to plain mean when `n <= 2*trim` (too few runs to trim).

Rationale: at n=6-8, median uses only the middle value while mean is dragged by outliers. Trim-mean uses ~70% of runs and still resists the worst outlier on each side. Empirical comparison on n=6 KLT-on showed mean/median diverged 2× on flow_z (one outlier dominated mean); trim-mean gave a value between the two. Mean still printed in the stats table for diagnostic comparison — if mean ≈ median, no outliers exist and either works.

For very small n (≤ 4): trim=1 leaves ≤ 2 values, falls back to plain mean. Validate via the per-axis stats table that mean and median are similar before trusting the cal.

## NaN gate fix (applied 2026-06-01)

Previously `is_valid_run` used `np.max(np.abs(B_v_tu))` which returns NaN if any sample is NaN, making `NaN < 10` False → run rejected. Cause: duplicate timestamps in some bridge streams cause `np.gradient` divide-by-zero → NaN propagation in B_v_tu. Now uses `np.nanmedian` and `np.nanmax` with `np.isfinite` guards. Verified that 2 of 4 previously-rejected n=6 runs (21-40-20, 21-42-34) recover with the fix; the other 2 (21-36-28, 21-44-54) legitimately have peak velocity > 10 m/s.

Tradeoff: NaN-tolerant aggregators may mask real numerical issues. Always sanity-check the per-axis stats table for crazy values that suggest something is wrong with the underlying GT computation.

## Why median of per-run cal is still noisy

Even with outlier rejection, per-run cal has std/median 0.25-0.70 per axis. Source: the `gt_v / raw` ratio depends on per-run noise floor (`σ_raw = √(σ_signal² + σ_noise²)`). Noisy runs give smaller cal than clean runs. Per-axis bimodality observed: high cal_x runs have low cal_z and vice versa (different motion content per run).

Stabler aggregation alternatives (not implemented):
- **Concatenate all runs** → single std-ratio across all samples. Larger n, more stable.
- **Phase-segmented cal**: read `gt['Phase']`, compute cal_x from `phase=='x'` samples only (signal dominates noise). Output-cal is designed for this; aggregate_calibration.py doesn't use phase info.

## Cal precision is signal-floor limited (2026-06-01 finding)

Concatenating 9 clean post-quality-filter runs (~63k frames) and computing a single robust_scale with bootstrap 95% CIs revealed per-axis precision:

| Axis | CI width | Interpretation |
|---|---|---|
| flow_x, flow_y, ω_x, ω_y | ±2-8% | Stable — well-excited by xy translation and induced pitch/roll |
| ω_z | ±9% (default AMP_YAW=10°) | Borderline — only ±10° yaw amplitude |
| flow_z | ±30% (default AMP_Z=0.6 m) | Poor — z-axis signal small relative to noise |

The wide CIs on flow_z and ω_z are NOT an aggregation problem — they're a signal-floor problem. The drone's z motion at AMP_Z=0.6m gives peak optic-flow ≈ 0.4 rad/s, comparable to the per-frame lstsq noise on that axis. No statistic can extract a higher-precision cal than the signal-to-noise allows.

**To shrink flow_z and ω_z CIs:** increase `CALIB_AMP_Z` and `CALIB_AMP_YAW_DEG` in record_output_calibration.py. Both are env-overridable (`CALIB_AMP_Z`, `CALIB_AMP_YAW_DEG`). Caveat: AMP_Z=1.2m pushes the drone to ~2.2g total thrust and raised lstsq clip-saturation from 0.03% to 0.28% on flow_x/y/ω_x/y — those axes get worse cal when over-driven. Sweet spot likely AMP_Z=0.9, AMP_YAW=20-30°; validate via fresh A/B post-center-fix.

## Sign mismatch on ω_x / ω_y (2026-06-01 finding)

`aggregate_calibration.py:150-151` wraps `robust_scale()` in `np.abs()`. This hides per-axis sign mismatches between raw lstsq output and GT. Empirically discovered when computing un-abs'd cal on KLT-on runs: ω_x consistently signed −0.405, ω_y −0.652. Means raw and GT have opposite signs on those axes — a frame-convention bug in the lstsq's ω columns vs how `compute_gt` derives `B_w_tug`.

abs() keeps the cal numerically usable but the controller sees ω with the WRONG sign on those axes. May or may not matter depending on the controller's use of ω — needs separate audit.

## Historical cal values that crashed

Per `img_data.py:99-117` comments:
- 2026-05-29: multipliers (1.5, 3.2, 95, 2.1, 2.7, 3.5) on flow_xy/z/ω → 5/5 TARGET_LOST, 18 m/s impact crashes
- 2026-05-30: z-axis `0.0257 → 0.9356` (36× jump) → 4/4 TARGET_LOST at xy≈10m

So big z and ω_z multipliers consistently destabilize the loop. Stable working ranges historically (e.g. 2026-05-13b 9-run median): `cal_z ≈ 0.026`, `cal_ω_z ≈ 0.20`. Current values (2026-05-31): `cal_z=2.63, cal_ω_z=10.5` — these are in the "previously crashed" regime; fresh n=8 KLT-on aggregation suggests `cal_z ≈ 0.28, cal_ω_z ≈ 1.11`, much more conservative.
