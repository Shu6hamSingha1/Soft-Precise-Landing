---
name: feedback_cross_marker_alpha_not_pixel_frame
description: "cross_marker_perception.py's logged alpha(t) is NOT a raw pixel-frame angle -- it's gravity-leveled + mount-axis-swapped (_getVirtualPts) then offset by CROSS_ALPHA_0 (default 90.23deg). Don't plot cos(alpha)/sin(alpha) directly in pixel space when visualizing/debugging."
metadata:
  node_type: memory
  type: feedback
  originSessionId: 4d44a921-8d4d-4924-a38e-243fbd1cb835
  modified: 2026-08-24T11:24:17.833Z
---

Found while building `tools/overlay_image_features.py`'s alpha arrow: the first version drew the
orientation arrow as `(cos(alpha), sin(alpha))` directly in raw pixel coordinates, and it pointed
opposite/rotated relative to the actual visible stub.

**Why:** `cross_marker_perception.py` computes `alpha` from raw stub/arm points via `_getVirtualPts`
(gravity-leveled, quaternion-based V-frame reprojection + a fixed `[y,-x]` mount-axis swap), THEN
subtracts `self._alpha_0` (`CROSS_ALPHA_0` env, default `radians(90.23)`) and wraps. So `alpha` is
calibrated for the yaw CONTROL loop's reference frame, not the camera's raw pixel frame.

**How to apply:** to draw/inspect alpha in pixel space, use the approximate inverse
(`tools/overlay_image_features.py::_alpha_to_pixel_dir`): `theta = alpha + CROSS_ALPHA_0`,
pixel direction `(dx, dy) = (-sin(theta), cos(theta))`. This drops the (small, by-design-decoupled)
gravity-leveling tilt correction, so it's exact for level flight and degrades gracefully under
roll/pitch -- good enough for visualization, not for re-deriving a precision ground truth.

Also relevant: when the stub isn't detected a given frame, `cross_marker_perception.py` HOLDS the
last-good alpha rather than recomputing (`else: # stub not found this frame`) -- a frozen alpha
reading looks identical to a live one unless you check `alpha[i]==alpha[i-1]`. The overlay tool
flags this as `(held)` + draws it gray.

See [[project_20260824_cross_marker_montage_overlay_robustness]] for the fuller investigation
this came out of (drone-shadow-near-touchdown corrupting alpha/s, and the resulting
`_robust_fit_line` bugfix).
