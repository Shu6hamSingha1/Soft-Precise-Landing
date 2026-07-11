---
name: feedback_klt_fallback_merge_no_separate_cal
description: "Primary ArUco-decode lstsq and KLT-fallback lstsq for corner flow are the SAME calibration category, not separate ones -- both feed the identical _fill_A/lstsq geometry regardless of corner source (decode vs LK-tracked, incl. the 3/4-corner parallelogram-reconstruction case). Difference is corner-position NOISE (tracking drift), already handled by _corner_conf down-weighting in the fusion EKF, not a gain difference a separate cal matrix would fix."
metadata:
  node_type: memory
  type: feedback
  originSessionId: af2bb6fc-90fc-4192-b79a-1590e374c873
---

## Question

Should KLT-fallback corners get their own calibration matrix, separate from clean ArUco-decode
corners, since the calibration derivation is estimator-blind (see
[[feedback_estimator_blind_calibration]])?

## Answer: no — merge into one category

Traced the code path: KLT-fallback (`img_data.py`, `~1427-1489`) feeds its tracked corners
(`_tracked` — either all 4 real-tracked, or 3 tracked + 1 parallelogram-reconstructed via
`MARKER_KLT_RELAX_GATE=1`) into the exact SAME downstream computation as primary decode:
`_scaled_quad_points` → `_fill_A` → lstsq. There is no separate gain/geometry model for
KLT-fallback — it's the identical interaction-matrix computation, just fed corner *positions*
from LK tracking instead of a fresh ArUco decode.

So the difference between primary-decode and KLT-fallback is **corner-position noise/bias**
(tracking drift, plus the parallelogram-completion approximation for the 3/4-tracked case), NOT a
different systematic gain mapping. That noise is already handled, separately from calibration:
`_corner_conf` (`img_data.py:~1960`) ramps down with `_lk_step_count` and down-weights the fusion
EKF's trust in KLT-fallback frames.

Giving KLT-fallback its own calibration matrix would risk fitting *tracking noise* as if it were a
*systematic gain error*, off a dataset that's inherently hard to collect cleanly (KLT-fallback is
rare and brief by design during a calm calibration hover — a calibration maneuver specifically
tries to keep clean ArUco lock).

## How to apply

- `_h_estimator_tag` uses `'lstsq'`/`'lstsq_klt'` as diagnostic-only sub-tags of ONE shared
  calibration bucket (see [[feedback_estimator_blind_calibration]]) — don't split them into
  separate fits.
- The parallelogram-completion case (3-of-4 tracked) is worth a caveat: it's a genuine geometric
  approximation (assumes diagonal midpoints coincide — exact only for an undistorted planar quad
  over a small inter-frame step), so it could in principle introduce a small systematic bias
  distinct from plain tracking noise — but it's a one-frame, small-perspective-change
  approximation, not something a static linear cal matrix could usefully correct for anyway.
