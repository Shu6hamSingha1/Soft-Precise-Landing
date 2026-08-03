---
name: feedback_cross_marker_radial_spread_ceiling
description: "Cross-marker Hz/Wz/Wx/Wy underperformance vs ArUco traced to tracked-point radial spread, not degeneracy — ROI and altitude both tried and ruled out as fixes"
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 79646ad7-0ee0-45bb-bd3b-73e43c1470bf
  modified: 2026-08-03T06:20:48.334Z
---

The cross+stub marker's flow-Jacobian solve (`_fill_A` in
`PX4_Gazebo/src/cross_marker_perception.py`) needs tracked points spread far
from the image center to observe Hz/Wz (linear columns) and especially Wx/Wy
(quadratic columns: `x*y`, `1+x^2`, `1+y^2`). Measured spread only reaches
~35-50% of the frame's normalized radial half-extent in every phase — this is
the root cause of Hz/Wz lagging ArUco's own board cal and of Wx/Wy being
practically unrecoverable (near-zero, sign-flipping correlation with GT even
under dedicated rollexc/pitchexc excitation).

**Why:** Two candidate fixes were tried and BOTH ruled out as insufficient
(don't retry either in isolation):
1. `_restrict_to_center_roi(roi_frac_y=0.65)` ghost-defense crop — loosened
   to 0.90/1.00 via new `CROSS_ROI_FRAC_Y` env var. Ghost defense held fine
   (`_reject_blobby_components` is the real defense now, ROI is redundant
   backstop), but Wx/Wy correlation only moved to a noisy +0.1..+0.26, and
   x-spread (unaffected by roi_frac_y) barely moved at all — proving x was
   never ROI-limited in the first place.
2. Lowering `CALIB_TAKEOFF_HEIGHT` (2.7m -> 2.4m -> 2.0m) to force more
   angular subtense: 2.4m held ok-rate 100% but didn't move Wx/Wy
   correlation; 2.0m broke detection outright (ok-rate 25%, marker overflows
   frame / line-geometry fit fails) before flow signal could improve.

The real ceiling is the marker's PHYSICAL FOOTPRINT (currently 3.0m plate at
2.7m calibration altitude) — not a recorder or detector parameter. The only
untried lever is a bigger physical marker
(`~/PX4-Autopilot/Tools/simulation/gz/models/cross_marker/model.sdf` size),
so tracked points sit farther from center while the whole shape still fits
in frame.

**How to apply:** Don't re-attempt ROI loosening or altitude lowering alone
expecting a Wx/Wy fix — both were tested cleanly (with a temporary radial-
spread diagnostic, `Radial Diag Log`, left in place in
`cross_marker_perception.py`/wired into `record_cross_marker_calibration.py`)
and neither moved the needle enough to matter. If revisiting this, the
next real lever is marker plate size, not maneuver/detection parameters.
See [[project_cross_marker_pipeline_20260801]] for the full data table.
