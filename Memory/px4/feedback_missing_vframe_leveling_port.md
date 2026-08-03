---
name: feedback_missing_vframe_leveling_port
description: "cross_marker_perception.py computed h,w,s from raw un-leveled camera pixels with zero attitude compensation, while GT is in a gravity-leveled V-frame -- img_data.py's _getVirtualPts port was silently dropped"
metadata:
  type: feedback
---

When porting/rewriting math from `img_data.py` into a new module, explicitly check
for load-bearing transform steps, not just the final formula shape -- a missing
step can be silent for months if validation never exercises the condition it
matters under.

`cross_marker_perception.py` (built 2026-08-01) computed the image-Jacobian h,w
solve and the centroid `s` directly from raw, un-leveled camera-frame pixel
coordinates, with **zero attitude (roll/pitch) compensation**. `img_data.py`'s
ArUco pipeline never does this -- every point is reprojected through
`_getVirtualPts()` first, onto a gravity-leveled, yaw-aligned "V-frame," using
the live quaternion. `compute_gt_signals` (the shared GT-computation helper used
by both pipelines' calibration derive tools) is *always* in this same V-frame.
So every raw-vs-GT comparison for the cross marker was comparing tilt-
contaminated flow against tilt-compensated ground truth -- a real mismatch, not
noise, that gets worse specifically during maneuvers that induce real tilt
(translation phases, since PX4 tilts to accelerate laterally).

**Why:** `img_data.py`'s own commit history documents a 2026-06-01 sign bug in
this exact transform (body→NED vs NED→body confusion) that was hard-won to find
-- the fix and its rationale lived only as an inline comment in that file, not as
shared/reusable code or a test. When `cross_marker_perception.py` was written as
a deliberately standalone module (avoiding a dependency on `IMG_PROCESSOR`
internals), the V-frame leveling step was dropped along with everything else
that wasn't directly copied, and nothing caught it because the module's original
validation was hover-only (near-zero tilt, near-zero translational velocity --
the exact condition under which a missing tilt-compensation step produces no
visible symptom).

**How to apply:** before trusting a "duplicated math" module's numerical output,
diff its per-transform-stage structure against the original it claims to mirror,
not just spot-check the final formula. If the original has a comment explaining
*why* a step exists (especially one describing a past bug), that step is exactly
the kind that silently vanishes in a fresh rewrite and is worth checking first.
See [[feedback_duplicated_math_diff_check]] (the general version of this lesson)
and [[feedback_dt_staleness_after_detection_dropout]] (a compounding bug found in
the same investigation).
