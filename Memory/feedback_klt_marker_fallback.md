---
name: klt-marker-fallback
description: "KLT-based ArUco re-acquisition in img_data.py — env knob, default cap, calibration vs landing trade-off"
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 367ac3aa-13f8-41ea-b23e-c929f5e1e50d
---

When ArUco detection fails on a frame (drone shadow, marginal contrast, partial occlusion, brief fast motion), `img_data.py:_imgProcess` falls back to LK-tracking the last good ArUco corner positions forward through up to `MARKER_KLT_MAX_STEPS` frames before declaring the marker stale. Bridges short outages without losing the marker. Default `MARKER_KLT_MAX_STEPS=20` (~0.33 s at 60 Hz); set to `0` to disable for A/B testing.

**Why:** [[feedback-marker-detection-stale]] documented that drone-body shadow at low altitude was causing 100-150 ms of `FEATURE_IS_STALE` and frozen optical flow during the final descent. ArUco's threshold rejects marginal frames where the corners are still geometrically visible — LK can keep tracking those corners by their pixel appearance even when the ArUco classifier balks. Validated A/B: KLT-on cuts mean zero-row rate from 2.68% → 0.59% (4.5×) and mean longest-gap from 89 → 22 frames (4×) on calibration sweeps.

**How to apply:**
- Default = on. Disable via `MARKER_KLT_MAX_STEPS=0` for any baseline A/B (e.g. when re-tuning sensor_cal — keep one variable at a time).
- KLT-on landing A/B (n=5 IC1) showed mixed result: rel_vel improved 55% (descent smoother, more SOFT landings) BUT two outliers at 2-3 m xy_err (controller acting on slightly-drifted KLT corners during 20-frame bridges). Cap=20 may be too aggressive for landing; try cap=5 first.
- When ArUco re-acquires, log line prints `"ArUco re-acquired after N KLT-fallback frame(s)"`. Frequent N>10 events suggest the drone is barely keeping the marker in view — that's a controller/altitude problem, not a KLT setting to chase.
- On TRUE marker loss (CHECK_NUM=80 consecutive failures), `_imgProcess` clears the KLT reference (`_prev_aruco_pts = None`, `_prev_img = None`) so we don't try to LK-track from a stale image after a long gap. This is correct behavior — don't remove.

Pairs with [[feedback-v-yaw-source-alpha]] for compass-free + marker-loss-robust pipeline.
