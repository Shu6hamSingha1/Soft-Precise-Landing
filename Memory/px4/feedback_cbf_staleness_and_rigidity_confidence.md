---
name: feedback_cbf_staleness_and_rigidity_confidence
description: "CBF cone-clamp read frozen _feature_pts unconditionally (no staleness gate); planar_map.py confidence didn't decay on sustained (multi-frame) marker_rigid_ok=False. Two fixes applied 2026-07-17."
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 7bc77c5e-027e-4e24-82cb-7e2996f36559
---

Found via IC2 SITL trace (`ICValidation/20260716-234258/IC2_rep1`): a terminal ~4s corner
collapse (t=41.1-45.4s, `N Flow Corners=0` throughout, never recovers) left
`MARKER_EXTENT_PX` frozen at 245.0px (img_data.py's `not FEATURE_DATA_IS_LOGGED` branch
holds `_feature_pts`/`_virtual_feature_pts` at their last value indefinitely, no decay,
no staleness tag — see img_data.py:2802-2803). Two independent bugs traced from this:

**1. CBF cone-clamp had no staleness awareness** (controller.py, `theta_cone`/`d_min_fov`
computation, ~line 2236). It read `self._img_node._feature_pts[-1][1]` unconditionally
every frame — during the 4s blackout it kept computing the visibility-cone headroom off
the same frozen 245px-extent corners as if they were live. Fixed: gate the read on the
already-existing, already-public `FEATURE_IS_STALE` flag (`STALE_THRESH=3` consecutive
misses) — when stale, `d_min_fov` falls back to 0.0 (no extra tilt headroom, cone reduces
to `theta_current`) instead of trusting a position that may no longer be real.

**2. `PlanarFeatureMap.confidence` didn't decay on SUSTAINED `marker_rigid_ok=False`**
(planar_map.py `update()`). The 2026-07-16 fix ([[feedback_planar_map_plausibility_gate]])
correctly stopped a single momentary partial-corner-drop from zeroing confidence, by
HOLDING `marker_shape_change` rather than forcing it to `inf`. But that hold means
`marker_conf` (derived from `shape_change`) never reflects `rigid_ok` staying False for
MANY consecutive frames either — confirmed live: IC2's terminal window had
`marker_rigid_ok=False` for its entire tail while `confidence` stayed 0.75-0.94 (high).
Fixed: added `_rigid_fail_streak` (consecutive frames `rigid_ok` has been False, reset on
bootstrap and whenever `rigid_ok` goes True), and `marker_conf *= max(0, 1 -
streak/rigid_fail_streak_max)` (`PLANAR_MAP_RIGID_FAIL_STREAK_MAX` env, default 3,
mirrors `STALE_THRESH`'s consecutive-miss convention). A single-frame blip is barely
dented; only persistence collapses `confidence`. `map_confidence` (marker-INDEPENDENT,
used by the RESCUE gate) is deliberately untouched — this fix only affects `confidence`
(marker-AWARE, used by the stricter OVERRIDE gate), which is exactly where it belongs.

**Why both matter together:** the CBF fix stops the cone-clamp from trusting stale
geometry; the confidence fix stops the OVERRIDE gate from trusting a map whose PRIMARY
marker has been visibly non-rigid for seconds. Neither alone would have caught IC2's
specific failure (corners went fully to 0, so override never even got a candidate to
evaluate) — but both close a real gap for the more common case: a marker that's still
*partially* trackable but persistently distorted (drifting KLT, not full loss).

**⛔ CORRECTION (2026-07-17, same session, user pushback):** the FIRST version of the CBF
fix gated on `FEATURE_IS_STALE` — this was ITSELF a band-aid. `FEATURE_IS_STALE` is a
legacy RAW-decode-miss counter (`_consec_misses`) that predates PlanarFeatureMap and has
**zero rescue awareness** — it increments unconditionally inside `not
FEATURE_DATA_IS_LOGGED` regardless of whether `_pm_rescue` succeeded, so it flips True
after just `STALE_THRESH=3` consecutive raw misses even while the map is successfully,
plausibility-checked rescuing every one of them. Gating the CBF on it blinded the
cone-clamp during exactly the scenario PlanarFeatureMap exists to cover. The REAL root
cause: `_feature_pts` (what `MARKER_EXTENT_PX`/the CBF's `d_min_fov` read) was **never
updated with the rescue's geometry** — only `_img_feature_param` (the s-vector) got the
rescued value; the coast branch unconditionally re-appended stale previous corners
regardless of rescue success. Fixed properly: `_feature_pts`/`_virtual_feature_pts` now
carry the plausibility-checked rescued pixel corners when `_pm_rescue` succeeds (paired
`[prev_curr, rescued]`, matching the raw `C_nP` convention); new public property
`FEATURE_PTS_FRESH` (img_data.py) reflects "raw OR rescue succeeded this frame" — the CBF
now gates on THIS, not `FEATURE_IS_STALE`. Validated: IC2 n=1 retest xy_err 3.49m→1.47m,
`target_lost` False (was DRIFT_OFF), with a similar ~320-frame raw-corner dropout still
present — confirms the CBF/consumers are now using rescued geometry through the gap
instead of stale frozen corners, not just suppressing the tilt cap.

**How to apply:** any future controller-side consumer of `_img_node._feature_pts` should
check `FEATURE_PTS_FRESH`, not `FEATURE_IS_STALE` — the latter is RAW-tier-only and
predates the rescue path; using it as a general freshness proxy re-introduces exactly the
band-aid this correction fixes. `RESCUE_ACTIVE` covers the s/alpha/h pipeline;
`FEATURE_PTS_FRESH` covers raw-corner consumers (`_feature_pts` itself, `MARKER_EXTENT_PX`,
the CBF). This codebase's hold-last-known convention (ds/dh outlier-hold, s-extrapolation,
`_feature_pts`) means "latest" and "live" are not the same thing by default — but as of
this fix, `_feature_pts` specifically IS kept live through a successful rescue, not just
held. Not yet validated at n≥5 — pending IC1-5 sweep.
