---
name: feedback_estimator_blind_calibration
description: "Output-calibration derive tools (aggregate_calibration_phased.py, derive_board_cal.py) fit against getRawOptFlowAngVel()/getRawImgFeatureParam() with NO record of which estimator produced each sample -- multiple distinct computations (primary ArUco-decode lstsq, KLT-fallback lstsq, centroid-rate observer, board homography, single-marker moment, predict-only coast) silently shared one raw stream. Fixed 2026-07-11: per-frame estimator tags added, logged in recordings, coast samples excluded from the fit, per-estimator coverage reported."
metadata:
  node_type: memory
  type: feedback
  originSessionId: af2bb6fc-90fc-4192-b79a-1590e374c873
---

## The gap

`getRawOptFlowAngVel()`/`getRawImgFeatureParam()` (used by `record_output_calibration.py` to log
raw samples for `derive_board_cal.py`/`aggregate_calibration_phased.py` to fit) just returned
`array[-1]`, whichever computation happened to populate it, with zero record of which. But
multiple structurally distinct estimators feed those same arrays:

- **h:** primary ArUco-decode lstsq, KLT-fallback lstsq (same `_fill_A`/lstsq geometry, different
  corner source — see [[feedback_klt_fallback_merge_no_separate_cal]], merged into one calibration
  category, not split), the centroid-rate observer (overrides h_x/h_y only, or the full 6-vector
  during a total lstsq-miss), and the predict-only "coast" (synthetic extrapolation during a
  marker-loss gap — NOT real data).
- **s:** board homography, single-marker moment fallback, and coast.

A calibration recording flown clean/close (as calibration maneuvers are, by design, to keep ArUco
lock) is very likely ~100% primary-lstsq + board-homography — meaning the derived cal matrix has
never been validated against KLT-fallback or single-marker-moment data, and a calibration run
could silently include fabricated coast-extrapolated frames as if they were real GT-comparable
measurements.

## Fix (2026-07-11)

1. `img_data.py`: every raw-array append site now also appends a matching tag
   (`self._h_estimator_tag` / `self._s_estimator_tag`) — `'lstsq'`/`'lstsq_klt'` (diagnostic
   suffix only, same fit bucket), `'+observer_xy'`, `'observer_full'`, `'coast'` for h;
   `'board_homography'` / `'single_marker_moment'` / `'coast'` for s.
2. New getters `getRawOptFlowEstimatorTag()` / `getRawImgFeatureEstimatorTag()`.
3. `record_output_calibration.py` logs both under `"Opt Flow Estimator Tag"` /
   `"Img Feature Estimator Tag"` in `Ground_Truth.npy`.
4. `aggregate_calibration_phased.py` / `derive_board_cal.py`: print per-estimator sample-count
   coverage, and exclude `'coast'`-tagged samples from the fit mask (a real correctness fix,
   independent of estimator diversity — synthetic extrapolation should never be compared to GT as
   if it were a measurement).
5. Back-compat: older recordings (predate this field) get `''` tags, which are NOT excluded (can't
   tell real from coast, so don't silently drop data) — ran against the existing 5-recording
   2026-07-03 set, output is byte-identical to pre-change (confirms no regression).

## Open

The existing calibration_data/output has no coverage data (predates tagging) and was almost
certainly clean-flown (100% primary/board estimators). A fresh recording will show real coverage
for the first time. If KLT-fallback/single-marker-moment coverage turns out near-zero (likely), a
deliberate off-nominal maneuver segment (partial occlusion / edge-of-frame dwell) would be needed
to validate those estimators against GT — not done yet.
