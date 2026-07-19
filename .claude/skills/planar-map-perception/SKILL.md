---
name: planar-map-perception
description: Debug or extend the PlanarFeatureMap (src/planar_map.py) rescue/override perception path and its CBF integration in PX4_Gazebo — when raw ArUco/KLT feature extraction fails, when the CBF's small-marker preference or overflow/drift-off handling looks wrong, or when investigating wild/implausible centroid or alpha values or a_u/kappa spikes. Use when touching map_confidence/confidence gating, FEATURE_PTS_FRESH, CBF_OVERFLOW/CBF_DRIFT_OFF, gyro-seeded KLT, or the s-vector extrapolation/decay path.
---

# PlanarFeatureMap perception debugging

`PlanarFeatureMap` (src/planar_map.py) is an online KLT+homography scene map, with ArUco
decode as sparse loop-closure — NOT the primary tracker. It tracks up to 2 marker SLOTS
(small+big nested markers), and feeds three distinct downstream consumers, each with its
own gate — **never share a gate across consumers**:

## Three consumers, three gates
- **RESCUE** (img_data.py `not FEATURE_DATA_IS_LOGGED:` branch): raw ArUco/KLT/dense-recovery
  has FAILED this frame — the map is the only source of a position. Gated by
  `_planar_map_gate_on` (marker-INDEPENDENT `map_confidence`), since a marker may be fully
  occluded.
- **OVERRIDE** (img_data.py `if FEATURE_DATA_IS_LOGGED:` branch): raw decode SUCCEEDED, map
  is "polishing" an already-good measurement. Gated by `_planar_map_override_gate_on`
  (marker-AWARE `confidence`, stricter) — replaces good data, needs a higher bar.
- **CBF small-marker preference** (controller.py, `cbf_corners` in the cone-angle
  computation): reads `PlanarFeatureMap.secondary_slot_name()` (the smaller, non-primary
  slot) once `get_slot_confidence()` clears a threshold WITH HYSTERESIS
  (`_cbf_small_slot_on`/`_streak`, 5-frame persistence-on) — independent of which marker
  feeds `h_x`/`h_y` (stays big-priority for flow observability, never changes). A raw
  per-frame threshold check here (no hysteresis) caused a real regression — IC4 hit
  `target_lost=DRIFT_OFF` (`theta_cone<0.05` for 45% of frames) — see
  `feedback_cbf_staleness_and_rigidity_confidence` memory.

Using the rescue gate for override (or vice-versa) is the exact bug that produced IC1's
142m fly-away (2026-07-16) — see `feedback_planar_map_plausibility_gate` memory.

## Never trust a confidence score alone — and know WHICH gap each one has
- `map_confidence`/`confidence` being high says nothing about whether THIS frame's
  projected position/size is geometrically sane. Always additionally check
  `_planarMapPredictionPlausible(pm_px, quat)` (img_data.py, near `_computeFeatureVec`):
  POSITION within `margin*(p10+last-held half-extent)`; SIZE within `[1/ratio,ratio]` of
  `_last_real_extent_px`. REJECT (never clip) — a clipped-but-wrong `s` still gets
  amplified `1/p_10` (~3-7x) by `s_e_n`, enough to trigger κ-ratchet/`a_u`.
- `confidence` (override gate) didn't decay on SUSTAINED `marker_rigid_ok=False` — fixed
  via `_rigid_fail_streak` (persistence, not single-blip-sensitive).
- `map_confidence` (rescue gate) is computed ONLY over RANSAC INLIERS — a marker's own
  corners can be actively drifting/chaotic (held-out reprojection error swinging
  0.3-199px) and get silently excluded as outliers, leaving `map_confidence` deceptively
  high (0.72-0.99). Fixed via a per-slot `inlier_fail_streak` (tracked-but-rejected
  persistence — occlusion-safe, doesn't punish a slot with zero corners this frame).
  See `feedback_cbf_staleness_and_rigidity_confidence`.

## CBF overflow vs. drift-off — computed by the CBF itself, not img_data's heuristic
`CBF_OVERFLOW`/`CBF_DRIFT_OFF` (controller.py properties) classify off the CBF's OWN
`rho_fov_curr` per-corner margin (span = overflow/benign/handover-ready; one-sided =
drift-off/target-visibility-failing) — a DIFFERENT signal from img_data.py's separate
`_last_overflow`/`_last_drifted_off` (different margin, different purpose: ring-flow
routing). Drift-off triggers a pull-back that TIGHTENS `cbf2_filter`'s own `p_10` on the
breaching axis (reusing its already-validated barrier math) rather than injecting a new,
unverified-sign corrective force. `CBF_OVERFLOW` + `SMALL_SLOT_CONFIDENT` together drive
`img_data.py`'s `HANDOVER_LATCHED` via `update_cbf_handover_signal()`, bridged from
`apps/landing_test.py` (controller and img_node stay one-directionally decoupled
otherwise) — a second, independent path alongside the existing loom-M-drop detector.

## Freshness: FEATURE_PTS_FRESH, not FEATURE_IS_STALE
`_feature_pts` (raw corners — what `MARKER_EXTENT_PX`/the CBF read) used to hold its last
value indefinitely during a coast with no staleness signal at all, letting the CBF compute
off frozen, arbitrarily-stale geometry. `FEATURE_IS_STALE` looks like the obvious gate but
is WRONG for this — it's a legacy RAW-decode-miss counter with zero rescue-awareness
(flips True after `STALE_THRESH=3` raw misses even while the map is successfully rescuing
every one of them). Use `FEATURE_PTS_FRESH` instead — reflects "raw OR rescue succeeded
this frame"; `_feature_pts` itself now gets updated with plausibility-checked rescued
geometry (not just held) when a rescue succeeds.

## Extrapolation during a coast: kf_predict, not hand-rolled decay
The old `s` extrapolation (polyfit-trend-fit, decayed toward `[0,0,0,0]` over
`H_EXTRAP_DECAY_FRAMES=10` consecutive misses) was REMOVED 2026-07-17 (user: "the
decay-to-zero behavior... I never approved it. We have kf_predict.") — two bugs traced to
it: (1) the decay multiplied ALL 4 components including `s[2]`, the fixed homogeneous
`1.0` constant, collapsing it to `0.0` and producing a single-frame `a_u` spike to
**610,997**; (2) even after fixing `[2]`, decaying position to exactly `[0,0]` fabricates
a "centered, zero error" reading that can desync from the adaptive gain state
(`kappa` ratcheted then froze reading "converged", then a real-error snap-back produced a
2,976 `a_u` spike). Fixed: `s[0:2]` now reads `self._kf_feat_x[:, 0]` (the feature KF's
own predict-only step, already stepped every coast frame, constant-velocity + growing
uncertainty via its own P matrix) instead of a hand-rolled fit+decay. `s[2]=1.0` stays as
a hard redundant clamp regardless. See `feedback_s2_homogeneous_decay_bug` memory. If you
see `a_u`/`kappa` spike right at a coast-to-live transition, check this mechanism first.

## Gyro-seeded KLT (2026-07-17, added, not yet SITL-validated)
`PlanarFeatureMap.update(gray, quat_R=...)` seeds its internal KLT search with a
rotation-compensated prediction (`R_delta = quat_R.T @ prev_quat_R`, ray-based) instead of
the implicit zero-motion prior, using the SAME `Quaternion(...).to_DCM()` call
`_getVirtualPts` already validates. Runs unconditionally every frame — the FC quaternion
doesn't need a marker decode — including through marker-loss stretches, where it matters
most (targets the chaotic-reprojection-error drift pattern below). Falls back exactly to
original behavior if `quat_R`/`center`/`focal` are unavailable.

## V_aruco_norm[0] vs [1] — don't conflate
- `V_aruco_norm[0]` ← `aruco_pts_0`, the genuine `cv2.ArucoDetector.detectMarkers()`
  output (fresh decode, `imgs[0]`).
- `V_aruco_norm[1]` ← `aruco_pts_1`, `calcOpticalFlowPyrLK`-TRACKED corners propagated
  from `aruco_pts_0` into `imgs[1]` — real image-derived data, but NOT a re-decode.
  This is the one PlanarFeatureMap's override/rescue conditionally replace.
Frame-pair flow (`flow_pts_0/1`, `V_flow_norm` → `h_x`/`h_y`) is built from the SAME raw
`aruco_pts_0`/`aruco_pts_1` and is NEVER touched by PlanarFeatureMap (the map only
predicts single-frame positions, not frame-pairs) — so raw decode/KLT tracking must keep
running every frame regardless of map confidence, both to feed flow and to give the
plausibility gates something real to check against.

## Reading a chaotic `err_px` trace (Planar Map Shadow log)
`err_px` swinging wildly (e.g. 0.3px→100px→0.2px→60px→200px) is NOT random — it's a
drift-accumulate/decode-triggered-snap-correction cycle: during `used_klt_fallback=True`
stretches (no fresh decode), KLT error grows smoothly; the next `fresh_decode=True` frame
reveals the accumulated drift as a spike, then `loop_closure_correct` snaps it back near
zero. Looks chaotic only because raw decode itself is flickering rapidly. Check whether it
coincides with a real attitude-rate spike (quaternion tilt climbing fast) — that's KLT's
zero-motion-prior assumption breaking under real rotation, which gyro-seeded KLT targets.

## Workflow for a new failure report ("centroid/alpha look wrong", or an a_u/kappa spike)
1. Pull `Img_Data.npy`/`Control_Data.npy` for the failing rep; check `map_confidence`/
   `confidence`, `RESCUE_ACTIVE`, `N Flow Corners`, and `a_u`/`kappa` norms around the
   failure frame. A single-frame spike that decays within a few frames (kappa flat or a
   brief ratchet) points at a degenerate INPUT (check `s`, especially `s[2]==1.0`), not a
   runaway control law.
2. Reconstruct the raw signal by hand from logged corners+quat via `_getVirtualPts` —
   don't assume the logged `s`/`alpha` reflects raw decode; it may be map-sourced or
   KF-predicted during a coast.
3. If map-sourced and wrong: check whether `_planarMapPredictionPlausible` should have
   rejected it. If confidence-sourced and wrong: check for the RANSAC-outlier blind spot
   (chaotic `err_px` + high `map_confidence`) or a rigidity-persistence gap.
4. Validate any fix with `HEADLESS=1 N_REPS=1 bash scripts/run_ic_validation.sh` (IC1-5)
   before declaring it fixed — see `feedback_reject_on_single_failure` memory: a single
   failed landing after a perception change is disqualifying, not noise. Given the wall-
   clock cost, consider a single-IC targeted retest first (`IC_LIST=ICn`) before a full
   sweep.
