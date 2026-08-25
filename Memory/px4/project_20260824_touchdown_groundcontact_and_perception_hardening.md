---
name: project_20260824_touchdown_groundcontact_and_perception_hardening
description: "2026-08-24 session: 3 perception/control fixes for cross-marker near-touchdown robustness (Hough corner-join shadow filter, hw coast+freeze KF ported from ArUco, touchdown-detect rolling-window streak) -- then found the REAL cause of a reported 'won't touch down, loom flips at 0.5m' symptom under GT-feedback: the drone's ground collision geometry was deliberately raised +0.5m (2026-08-09, for camera standoff) -- reverting it to true ground gave a clean SOFT+PRECISE GT-FB landing at min_alt=-0.01m, first try. IC1-5 GT-FB sweep at true ground contact in progress at session end."
metadata:
  node_type: memory
  type: project
  originSessionId: 4d44a921-8d4d-4924-a38e-243fbd1cb835
  modified: 2026-08-24T16:19:19.389Z
---

## Starting symptom

User reported (perception-mode, cross-marker): drone not reaching touchdown, loom-rate
"flipping" around 0.5m altitude, visually confirmed via recorded video. Continues
[[project_20260824_cross_marker_montage_overlay_robustness]] (same session, same shadow
investigation).

## Fix 1: Hough corner-join shadow filter (`cross_marker_detector.py`)

New `_filter_segments_by_corner_join()` stage, runs on raw Hough segments BEFORE
angle-clustering. Every real marker line meets another real marker line near the
marker center at one of exactly two relative angles (90 deg = the two cross arms,
`STUB_REL_ANGLE_DEG`=45 = stub-to-arm). A segment with no such geometrically-joined,
correctly-angled partner (e.g. an edge of the drone's own cast shadow, a separate blob
merged into the same connected component near touchdown) is dropped before it can seed
or pollute a cluster. Falls back to keeping everything if <2 segments would survive
(matches `_best_pair`'s graceful-degradation pattern). Complements, doesn't replace,
the same-session `_robust_fit_line` per-point pruning fix (see
[[project_20260824_cross_marker_montage_overlay_robustness]]).
**Validated**: regenerated the `linear_verify` overlay video with the fix live --
alpha tracked live one frame further into the shadow overlap (held only the last
1-2 frames vs 3+ before), no flip, landing quality unchanged (lat=0.161m vs
original 0.157m). Uncommitted.

## Fix 2: hw coast+freeze KF ported from ArUco (`cross_marker_perception.py`)

**Root cause (user pushed back on "hw=zeros(6) on miss" being an accepted design --
correctly, per below):** `git log --follow --diff-filter=A` showed this has been the
behavior since the file's very first commit (`916aa53`, 2026-08-01), never revisited,
no memory ever validated it as deliberate. The module's own docstring claims "the
feature KF's predict step already provides the smoothing/coasting" but that claim is
FALSE for the actual consumption path: `controller.py:1574-1575` calls
`getOptFlowAngVel()` unconditionally every tick and appends straight into `self._h` --
no gate distinguishing a real solve from the miss-branch's zero-substitute. This is a
reintroduction of [[feedback_kf_frozen_during_marker_loss]] (ArUco, fixed 2026-07-11):
a zeroed/frozen `h_z` during a marker-loss gap makes `_touchdownDetect`'s
`h_z > _td_spike` streak condition structurally unreachable.

**Fix**: ported `img_data.py`'s `_kf_step`/`_kf_update` verbatim (same math, module-level
stateless function + `_kf_update_hw()` method, same env-var names/defaults
`FLOW_KF_Q`/`FLOW_KF_R`/`KF_DT_UNC_MAX`/`PLASMC_KF_COAST_FREEZE_STREAK`). All 3
`self._hw = np.zeros(6)` sites now route through `_kf_update_hw()`: a real observation
corrects the KF, a miss predict-only coasts (state evolves via constant-velocity model,
no correction), and after `_hw_kf_coast_freeze_streak` (default 3) consecutive misses it
FREEZES (holds last state, stops drifting) until a real measurement returns and resets
the streak -- identical semantics to the validated ArUco path. Pre-first-detection
frames still correctly fall back to zeros (nothing to coast from yet -- not a
regression).
**Validated**: fresh SITL rep, `Img_Data.npy` shows all exact-zero `h_z` values on miss
frames are pre-first-detection (idx 0-395); after first lock, a long miss streak
(idx 720-730) holds a frozen non-zero value (-0.0599) instead of the old hard zero.
Landed ON-PLATFORM, lat=0.127m (no regression vs. the 0.12-0.16m range from other reps
this session). Uncommitted.

## Fix 3: touchdown-detect rolling-window streak (`controller.py::_touchdownDetect`)

Secondary safety net, requested alongside Fix 2. `self._td_streak` (strict consecutive
count, resets to 0 on ANY sub-threshold frame) replaced with `self._td_hist =
deque(maxlen=self._td_window)` + `sum(self._td_hist) >= self._td_frames`, i.e. spikes
within a rolling window rather than strictly consecutive. `PLASMC_TD_WINDOW` defaults to
`self._td_frames` (3) -- **behaviorally IDENTICAL to the old consecutive rule by
default**, deliberately not changing the already-validated touchdown timing; widening it
is a one-line env override to sweep later, per the project's validate-before-default
convention. Applies regardless of GT-feedback vs. perception (operates on `self._h[-1]`
whatever its source). Uncommitted.

## The bigger finding: the "0.5m stall" isn't a control bug at all under GT-feedback

User then asked why the drone doesn't touch down even under GT-feedback (which bypasses
perception entirely -- `controller.py:1576-1592` substitutes GT `s`/`h` before
`_touchdownDetect` runs), ruling out Fixes 1-3 as an explanation.

**Traced to `~/PX4-Autopilot/Tools/simulation/gz/models/x500_base/model.sdf` (outside
this git repo).** On 2026-08-09 (see [[project_cross_marker_pipeline_20260801]]'s
"CLOSE-RANGE FIX PROTOTYPED" entry), the landing-gear COLLISION geometry (not the visual
mesh) was deliberately extended `base_link_collision_3`/`_4` Z pose `-0.2195 -> -0.7195`
(+0.5m) -- to give the downward camera more standoff, because the cross-marker's fine
speckle texture visibly blurs below ~0.79m camera-to-marker distance (texel/fx mismatch),
degrading the flow solve exactly where precision matters. Confirmed at the time via
telemetry (body touchdown altitude 0.05m -> 0.487m) and visually by the user in the
Gazebo GUI. **This means the whole "0.487m loom-inversion/kappa-leakage equilibrium"
investigated across [[project_20260817_crossmarker_descent_stall_investigation]] and
[[project_20260823_td_spike_regression]] is downstream of a real, physical, artificially-
raised ground floor** -- the drone was correctly detecting that it had hit the ground,
just at a ground raised 0.5m for an unrelated (perception-texture) reason. The
`TD_SPIKE=0.0` fix in those memories is still correct and needed, but the "0.48m short
of true touchdown, why" framing in `project_20260823_td_spike_regression`'s caveat is
now EXPLAINED, not just flagged open.

**Confirmed experimentally**: backed up the raised model.sdf
(`model.sdf.bak_before_gtfb_groundcontact_test_20260824`), reverted to
`model.sdf.bak_before_legext_20260809` (true `-0.2195`, body alt ~0.05m), ran one
GT-feedback rep on the flat `cross_marker` world (not rover -- rover's own 0.5m platform
height would confound this test). Result: **first try, SOFT+PRECISE, xy_err=0.007m,
rel_vel=0.012m/s, min_alt=-0.01m** (matches the historical median `-0.0088m` across 1340
archived PRECISE landings almost exactly). `TOUCHDOWN-DETECT` fired cleanly, no flip, no
stall. Re-reverted back to the raised (0.487m) model.sdf afterward (needed for
perception-mode work, per the texture-blur reasoning above) -- **then per this session's
final instruction, reverted to TRUE ground contact AGAIN** (`model.sdf.bak_before_legext_20260809`)
to run an IC1-5 GT-feedback sweep and check whether Fixes 1-3 (which don't act on
GT-feedback's own `h_z` beyond the no-op-by-default streak window) still land cleanly
at true ground across all 5 ICs, not just IC1.

**Live model.sdf state at session end: TRUE ground contact (`-0.2195`), NOT the
0.487m-raised version.** Anyone resuming perception-mode (non-GT-feedback) cross-marker
work should check this file's live state before assuming the raised-floor behavior is
still in effect -- it may have been changed again since. Backups of both states exist in
`x500_base/`: `model.sdf.bak_before_legext_20260809` (true ground) and
`model.sdf.bak_before_gtfb_groundcontact_test_20260824` (0.487m-raised, the one
that was live for most of this session's perception-mode fixes above).

## Open item at session end

IC1-5 GT-feedback sweep (`test_data/GTFB_GroundContact_IC1to5/<timestamp>/`, flat
`cross_marker` world, true ground contact) launched but not yet complete/summarized
when this memory was written. Check that directory's `summary.log` for the actual
per-IC results before reporting a conclusion.

## Uncommitted at session end

`PX4_Gazebo/src/cross_marker_detector.py` (Fix 1), `PX4_Gazebo/src/cross_marker_perception.py`
(Fix 2), `PX4_Gazebo/src/controller.py` (Fix 3 + the earlier `theta(t)` logging fix from
[[project_20260817_crossmarker_descent_stall_investigation]]), plus the large pre-existing
backlog of untracked memory files and `.bak_*` files from 2026-08-13 through today (not
touched this session, still sitting uncommitted).
