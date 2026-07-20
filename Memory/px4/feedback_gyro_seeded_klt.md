---
name: feedback_gyro_seeded_klt
description: "PlanarFeatureMap's internal KLT tracker now seeds its search with a gyro-rotation-compensated prediction (quat delta) instead of the implicit zero-motion prior, running unconditionally every frame (including through marker-loss stretches) since the FC quaternion doesn't depend on ArUco decode. Added 2026-07-17, not yet SITL-validated."
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 7bc77c5e-027e-4e24-82cb-7e2996f36559
---

`cv2.calcOpticalFlowPyrLK` has no motion model — it seeds its search from the previous
frame's pixel position (implicit zero-motion prior), searching only within
`winSize`/`maxLevel` (21×21, 3 pyramid levels in this codebase). Under large real
inter-frame rotation this fails: the true displacement exceeds what the window covers,
and KLT converges to a locally-similar-but-wrong patch instead of the true correspondence
— this is drift, not a decode/detection problem.

**Evidence (2026-07-17, IC5 SITL trace)**: `PlanarFeatureMap`'s held-out reprojection
error (`err_px`) swung 0.3px → 199px in a chaotic-looking pattern during a 1.5s window —
traced to a genuine drift-accumulate/decode-triggered-snap-correction cycle: `err_px`
grows smoothly across each `used_klt_fallback=True` stretch (KLT coasting without a fresh
ArUco re-anchor), then spikes on the next `fresh_decode=True` frame (revealing the
accumulated drift), then snaps back near-zero once `loop_closure_correct` re-anchors.
This coincided with the drone's real attitude tilt climbing ~4°→24° over the same window
— large real rotation is the root driver, not smooth geometric obliquity (see
[[feedback_planar_map_plausibility_gate]] for the earlier, now-superseded "obliquity
breakdown" hypothesis from a different session's IC5 rep).

**Fix**: `PlanarFeatureMap.update(gray, quat_R=...)` now accepts the current frame's
body→NED DCM (same `Quaternion([w,x,y,z]).to_DCM()` call `_getVirtualPts` already uses —
sign convention stays in ONE validated place, not re-derived). When available (plus a
previous frame's `quat_R` and `self.center`/`self.focal`, now optional constructor
params), it computes `R_delta = quat_R.T @ prev_quat_R` and predicts each tracked point's
expected new pixel position under camera-frame rotation alone
(`d_curr = R_curr.T @ R_prev @ d_prev`, ray-based, reprojected through center/focal),
passing that as KLT's `nextPts` initial guess with `cv2.OPTFLOW_USE_INITIAL_FLOW`. KLT
then only has to resolve the *residual* motion (translation + real target motion), not
the full displacement.

**Why it runs unconditionally, even through marker-loss**: the FC quaternion is available
every frame regardless of whether ArUco decodes anything (user correction, 2026-07-17) —
wired in `img_data.py`'s `self._planar_map.update(_gray0, quat_R=_R0)` call, computed from
`quats[0]` every frame. This matters most during exactly the coast/rescue regime this
whole investigation was about — better KLT tracking through a marker-loss gap reduces how
often the pipeline falls into that regime at all.

**Status**: implemented, compiles clean, **not yet SITL-validated** — the next IC1-5
sweep after this change should specifically check whether `err_px` chaos and the
drift-accumulate pattern are reduced during high-attitude-rate windows (IC5 in
particular). If `quat_R`/`center`/`focal` are unavailable, behavior falls back exactly to
the original zero-motion-prior KLT (no regression risk from missing inputs).

**How to apply:** if a future SITL run shows a similar chaotic-reprojection-error pattern
coincident with a real attitude-rate spike, check whether gyro-seeding is actually wired
end-to-end for that path (both `bootstrap()` and `update()` need `quat_R`) before
re-diagnosing from scratch.
