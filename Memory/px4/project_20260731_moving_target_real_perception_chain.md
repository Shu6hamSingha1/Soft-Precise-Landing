---
name: project_20260731_moving_target_real_perception_chain
description: "Full real-perception (non-GT-feedback) rover diagnostic chain on 2026-07-31 -- traced silent s/h divergence during a moving-target corner-loss coast to the position-KF's coast being decoupled from optical flow, and implemented a flow-coupled-coast fix (img_data.py _kf_feat_update)."
metadata: 
  node_type: memory
  type: project
  originSessionId: e7d8e3d1-20d0-4b05-a155-9404e14de645
  modified: 2026-07-31T15:31:00.628Z
---

Continuation of [[project_rover_baseline_stale_2026-07-30]] / [[project_20260730_five_fixes_ic_validation]]:
after the five 2026-07-30 fixes (self-heal, CBF_CORNERS_STALE, validation-reset,
confidence-weighted correction, kappa/integral freeze) fully solved the rover moving-target
case UNDER GT-FEEDBACK (n=3 Linear: min_alt_xy 0.034-0.167m, matching/beating the historical
0.034-0.143m baseline; n=3 Circular also clean, though at ~4x slower absolute speed than
Linear due to a `speed_mult` scaling mismatch between `rover_trajectory.py`'s Linear
(`s=1.1*mult`) and Circular (`v_tan=0.384*mult`) formulas -- NOT yet a fair trajectory-shape
comparison), the SAME test with `PLASMC_GT_FEEDBACK` off (real image+IMU perception driving
the primary SMC signal, not just the CBF corner path) FAILED badly: n=3, all `TARGET_LOST`,
violent impacts (57-311 m/s^2), xy_err ~1-1.4m at min-alt, rel_vel 2.4-2.7 m/s. This confirmed
the user's original instinct (from early in the 2026-07-30 conversation) that perception
itself, not just the control-law/CBF-authority-starvation chain, needed hardening for a
moving target -- the five fixes made the system robust to perception going ABSENT, not to
perception being systematically wrong/behind.

**Diagnostic chain (offline replay of Ground_Truth.npy poses through gt_feedback.py's own
math, compared against the real Control_Data.npy s(t)/h(t) on 3 real-perception reps):**
1. All 3 reps: real `s` tracks GT well until t~62-63.6s (near-deck transition, consistent
   with every other trace this session), then diverges hard (err_s up to 5-7).
2. `N Flow Corners` = 0 CONTINUOUSLY for ~1.4-1.8s in exactly this window, all 3 reps --
   genuine total corner loss, not intermittent/confidently-wrong tracking (that was my first,
   WRONG hypothesis on rep2 alone; confirming reps 1+3 showed continuous zero-corner streaks,
   not cycling, which revised the diagnosis).
3. `MARKER_LOSS_GRACE` (default 1.0s, `apps/landing_test.py:467`) is working AS DESIGNED --
   `marker_lost_t0` is reset to None only on a fresh `feature_fresh` frame (`:700`), and with
   corners genuinely absent (not falsely "fresh"), the grace clock correctly accumulates and
   the "Marker lost beyond grace" fallback DOES eventually fire in all 3 reps. The problem:
   1.0s of holding the last command is fine for a STATIONARY target (its own design comment:
   tuned for "1-2 frame" dropouts from tilt/motion blur) but lets a target moving at ~0.47 m/s
   drift ~0.47m before the grace even expires -- consistent with the measured error magnitude.
4. User correctly pushed back on "extrapolate using last known target velocity" (would violate
   the project's scale-free/depth-free hard constraint AND repeats the already-rejected
   `PLASMC_TGT_VEL_FF`/lead-pursuit removal, commit 1933367 2026-07-02: "consumed target-pose
   derivatives forbidden by the manuscript Problem Statement"). Correctly redirected to optical
   flow `h`, which IS scale-free/image+IMU-legitimate (h = rel_velocity/depth, already part of
   the normal feature vector).
5. Checked whether `h` stayed accurate during the SAME corner-loss window it should be used to
   extrapolate through: YES -- h_x grew to ~1.0 and stayed there through the whole ~1.4s gap
   while true GT `s` exploded, i.e. h correctly represented the ongoing relative motion.
6. BUT root-caused WHY `s` wasn't using it: `img_data.py:3545` `_kf_step` is a generic 2-state
   (value, rate) constant-velocity KF applied PER-CHANNEL INDEPENDENTLY. The xc/yc position
   channels' "rate" state (used for the predict-only coast extrapolation, `_kf_feat_update`,
   coast call site `:3296`) is a SELF-REFERENTIAL derivative of past xc/yc measurements only --
   entirely disconnected from `h`, which is estimated by a SEPARATE KF/EKF (`_kf_x`/`_ekf_x`).
   Confirmed `h` was ALSO coasting (predict-only) during this window in our default config
   (`PLASMC_CENTROID_RATE`/`FLOW_FUSE_RING` both off) -- so it's not a "fresh ground-truth
   signal," but coasting a velocity-like quantity forward (h roughly constant) is far more
   physically valid than coasting a position channel's own weak, already-decayed-near-zero
   rate state -- which is why h stayed accurate and s's own rate-based coast didn't.

**FIX IMPLEMENTED (2026-07-31, `img_data.py::_kf_feat_update`, backed up
`img_data.py.bak_before_flowcoast_20260731` -- SAME-SESSION, unvalidated pending the n=3
real-perception re-test):** during a predict-only coast (z=None), overwrite the xc/yc rate
slots (`self._kf_feat_x[0:2, 1]`) with the CURRENT calibrated `h_x`/`h_y` (via
`getOptFlowAngVel()[:2]`) before `_kf_step` extrapolates -- first-order `ds/dt ~= h`,
deliberately ignoring the `s*h_z` cross-term the full IBVS interaction matrix carries (a
refinement left for later if this first cut isn't sufficient). Wrapped in try/except so a
flow-read failure can't break the coast path itself.

**VALIDATION RESULT (2026-07-31, same session, n=3 real-perception rover re-test):** partial
fix, confirmed coast-duration-dependent. Rep1 (0.62s longest post-engage coast): min_alt_xy
0.044m -- matches the historical GT-feedback-level baseline (0.034-0.143m), the first
real-perception rover rep to do so. Rep2 (6.09s coast): 1.751m. Rep3 (2.33s coast): 1.122m.
Directly confirms the predicted failure boundary: the fix works when the coast is short
enough that `h`'s own last-fresh value (itself also coasting, no independent corner-free
source in the default config) is still close to true; it can't help once the coast runs
multiple seconds and `h` itself has drifted. **Fix is real and worth keeping, but not
sufficient alone for long outages.**

**How to apply:** the fix is validated-useful, not validated-sufficient. The next lever,
directly indicated by the coast-duration correlation above, is giving `h` an INDEPENDENT,
corner-free data source during exactly these multi-second outages -- `FLOW_FUSE_RING=1`
(ring flow already "computed every frame, survives the marker death" per its own
`img_data.py` comment, `:3171`) or `PLASMC_CENTROID_RATE=1`. Both are currently baked OFF for
stationary-target reasons (ring flow's fixed radii overlap the marker near touchdown) that may
not apply the same way to the moving-target case; not yet re-examined for that context. Also
still fully open: why the corner-loss coast durations themselves are so long (0.6-6.1s) for a
moving target in the first place -- likely the same near-deck marker-overflow geometry as
everywhere else this session, but not separately root-caused for the multi-second tail case.

Also open, not yet acted on: the Linear-vs-Circular rover comparison needs re-running with
matched absolute tangential speed (Circular's `ROVER_CIRCLE_VTAN` set to Linear's ~0.467 m/s
equivalent, not just the same `ROVER_SPEED_MULT`) before drawing any real trajectory-shape
conclusion.

**FURTHER DIAGNOSED (still 2026-07-31 session):** checked the worst rover rep (6.09s coast,
1.75m final error) for WHY corners AND ring flow both hit zero simultaneously. Found: `N Ring
Corners` collapses in lockstep with `N Flow Corners` (48->34->0 over ~0.5s) while `Quat`-derived
tilt climbs STEADILY (not a spike) from ~2deg to 11deg+ over the same window. Conclusion: this
specific severe-failure class is NOT "marker overflows near the deck" -- it's sustained tracking
error -> growing commanded tilt trying to correct -> tilt eventually large enough that the
DOWNWARD CAMERA SWEEPS THE ENTIRE GROUND SCENE OUT OF FRAME (marker AND background both), so
NO vision-based signal (ring flow, centroid-rate observer, anything) has data to work with
regardless of source. This means ring-flow/observer-based fixes (the previously-proposed next
levers) would NOT fix this failure mode -- the real lever is whatever's supposed to bound tilt
generically (the `theta_cone`/CBF visibility constraint) not arresting the growth. NOT YET
INVESTIGATED further (session paused here -- see 2026-08-04 note below).

**SESSION PAUSED 2026-08-04 (user decision):** user has started a parallel thread evaluating a
different (non-ArUco, "cross") marker design in another chat and does not plan to continue
deep ArUco-specific debugging here. Marker-AGNOSTIC findings from this whole investigation that
DO carry over to any future marker choice: (1) the kappa-ratchet/a_u-explosion mechanism and
the staleness-freeze principle (needs an equivalent "no valid measurement" signal for whatever
new detector is used); (2) MARKER_LOSS_GRACE's 1.0s hold being tuned for a stationary target,
not fixed; (3) the flow-coupled-coast fix and the general KF-coast-should-use-velocity-not-
position's-own-rate architecture point; (4) the theta_cone/CBF tilt-bound-not-arresting-growth
question immediately above, still open; (5) the GT-feedback-vs-real-perception independence
(GT-FB never validates whatever the real detector is); (6) the rover_trajectory.py
Linear-vs-Circular speed_mult scaling bug; (7) the practice of measuring effective processing
Hz during the hardest flight phase, not at rest. NOW MOOT (ArUco-specific, drop if switching
markers): PlanarFeatureMap's multi-slot big/small handover and quad-corner overflow math,
confidence-lockup self-heal/validation-reset AS IMPLEMENTED (the detect-map-disagreement
PRINCIPLE could transfer to a new map if one is built), KLT-on-ArUco-corners drift diagnostics.
The mixed-motion-homography critique transfers ONLY if the new marker approach also uses a
shared-scene homography/SLAM-style map; moot if it doesn't.
