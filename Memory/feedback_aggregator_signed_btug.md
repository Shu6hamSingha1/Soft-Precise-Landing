---
name: aggregator-signed-btug
description: aggregate_calibration_phased.py was deriving cal vs B_w_ug (UAV body ω) but plotter compares vs B_w_tug (target-rel ω = -B_w_ug). Cal sign was always inverted on w axes.
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 3ffdaf6f-0da4-4c9f-b4df-fb9a91a7eb04
---

`aggregate_calibration_phased.py:compute_gt_signals` returns `B_w_ug` (UAV body ω from quat-diff). The aggregator's per-axis loop used `gt_sig = B_w_ug[:, k-3]` directly for the w (axis 3-5) cal derivation. But the plotter (and the manuscript) compare `w_cal` against `B_w_tug = B_w_tg - B_w_ug = -B_w_ug` (for stationary target). So:

- Aggregator finds `cal × raw ≈ +B_w_ug` (positive correlation)
- Plotter sees `cal × raw vs −B_w_ug` → **negative correlation**
- → User sees sign-flipped w_x / w_y on every recording

Fix: use `gt_sig = -B_w_ug[:, k-3]` for the w axes (axis 3-5), matching the plotter and the manuscript convention.

Additionally, the unsigned `std_ratio = σ(GT)/σ(raw)` lost sign information. Per-sample `median(GT/raw)` was too noisy for weakly-observed axes (LSTSQ recovery of ω_x/ω_y has poor conditioning at image center because L cols 1 & 3 are anti-parallel there). Switched to Pearson correlation sign — robust to noise and only flips when there's actual evidence of an anti-correlated relationship.

## Why this stayed hidden

For all single-axis tests on level drone in shop-flat conditions, the per-axis correlations were either:
- Tiny enough that sign was lost in noise (w_x, w_y on small markers)
- Strongly positive on w_z (yaw is well-observed because col 5 = `[-y, +x]` has spread regardless of marker offset)

The h axes (h_x, h_y, h_z) compare against `B_y_g = B_v_tu/Z` which already IS the target-relative quantity (the centroid construction subtracts UAV from target). So h axes were correctly signed all along.

Only ω flagged because `B_w_tug = -B_w_ug` is more easily confused with `B_w_ug` than `B_v_tu` is with anything else.

## Lesson

When deriving a calibration matrix, the GT used to derive it MUST be in the same frame and sign convention as the GT used to validate it. If the aggregator says "looks great" but the plotter says "looks wrong" — check that both are using the same GT quantity, not just the same Python variable name.
