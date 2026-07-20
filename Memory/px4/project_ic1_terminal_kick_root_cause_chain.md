---
name: project_ic1_terminal_kick_root_cause_chain
description: "Full traced chain for the specific IC1 terminal a_u explosion incident (2026-07-10): ~0.5s KLT-fallback blackout (gyro-confirmed real 2.8-7.5 rad/s attitude rate during the blind window) -> relock switches primary marker small->big (nested ArUco) -> MARKER_EXTENT_PX jumps 51.7->334.1px in one frame -> h_xy explodes (misattributed ROTATION, not translation -- FLOW_LAT_REDUCED=1 drops w_xy columns assuming a level target) -> c-term -> theta (1.76->912) -> kappa ratchet (1.73->3.69) -> a_u blew up to 1400+."
metadata:
  node_type: memory
  type: project
  originSessionId: a8922284-2fe3-4a78-9355-9949c3be5a10
---

## Full causal chain, validated step-by-step against recorded Control_Data/Img_Data/Telemetry

1. **KLT-fallback blackout (~0.5s, t=46.7-47.19 on the traced rep).** `KLT Diag` log shows
   `n_tracked: 0-2, accepted: False` continuously — neither fresh ArUco decode nor KLT-fallback
   succeeds. `MARKER_EXTENT_PX` frozen at 51.7px (small marker, last valid reading).
2. **Real attitude WAS disturbed during the blind window** — gyro (`w(t)`, independent IMU
   reading, not flow-derived) shows 2.8-7.5 rad/s during this exact interval. The controller had
   zero visibility into this (see [[feedback_kf_frozen_during_marker_loss]] — `h_z`/`h_xy` frozen
   the whole time, no predict-only coast existed yet at time of this trace).
3. **Relock (t=47.19-47.20) switches primary marker ID.** Single-marker lock logic
   (`img_data.py`, `_locked_marker_id not in ids` triggers `argmax(_spreads)` re-lock) picks
   whichever nested marker (small ID vs big ID) has the largest pixel spread in the fresh
   detection. After a loss, the code has no bias toward "the one I was tracking before" beyond
   ID continuity — it picked the BIG marker this time.
4. **`MARKER_EXTENT_PX` jumps 51.7→334.1px in ONE frame** — confirmed via `n_flow_corners`
   simultaneously jumping 0→184 (matches `_scaled_quad_points` dense-point generation, ~180pts,
   now applied to the much-larger quad). This is a LEGITIMATE detection (not corrupted/garbage),
   just a discontinuous handover the pipeline doesn't protect against (see below).
5. **`h_xy` explodes: 0.075 → 47.66 over ~0.35s.** ROOT MECHANISM (confirmed via `_fill_A`'s
   interaction-matrix math): `FLOW_LAT_REDUCED=1` (baked default since 2026-06-25) DROPS the
   `w_xy` (roll/pitch rotational flow) columns from the lstsq, assuming `w_xy≈0` for a level
   target. This assumption is about the TARGET's tilt, not the drone's own rotation RATE — and
   the V-frame per-frame leveling (`_getVirtualPts`, independently re-levels each frame using
   its OWN quaternion) SHOULD correctly cancel drone rotation geometrically, confirmed by a full
   audit of every `_getVirtualPts`/`_getRealPtsFromV` call site (all correctly paired,
   frame0↔quats[0], frame1↔quats[1] — the 2026-07-04 frame-pair fix was applied comprehensively).
   Quaternion/image capture-time sync was ALSO ruled out (`gz_subscriber.py`'s `image_callback`
   captures `quat = self._FC.getQuat()` in the SAME callback as the image, no meaningful lag).
   **The actual mechanism: rotational flow contribution scales with corner distance from the
   optical center (`(1+x²)`, `xy`, `(1+y²)` terms in `_fill_A`).** The newly-locked BIG marker's
   corners sit much farther from center than the small marker's did. With `w_xy` dropped from
   the model, ANY real rotation still present gets forcibly misattributed into the translation
   unknowns (`h_xy`) — and that misattribution error scales with `r²`, so the SAME real
   disturbance produces a vastly larger fake `h_xy` reading on the big marker's far corners than
   it did on the small marker's near-center corners. (Two earlier hypotheses — timing lag,
   pure LK-conditioning under-report — were investigated and retracted/refined in favor of this
   one; see the conversation transcript for the falsification trail if revisiting.)
6. **`h_xy`/`dh_d` feed the `c`-term** (controller.py, `c = ... - cterm_loom_scale*(h·e3)*h - dh_d`)
   quadratically and additively — `theta_ctrl = ||Theta||_F` explodes 1.76→912 in lockstep.
7. **`theta` drives the κ-ODE** (`dκ/dt = θ·G·|σ| - N·P·κ`), ratcheting `kappa` 1.73→3.69.
8. **Both feed the switching term of `a_v`** → `a_u` blows from ~1 to 1400+.

## Fixes applied (all baked, see individual memories)

- [[feedback_loom_sign_guard_blocks_touchdown_detect]] — unrelated but co-discovered blocker.
- [[feedback_kf_frozen_during_marker_loss]] — h_z coast fix (prevents step 1-2's blind window
  from being totally opaque to the loom-inversion detector, though doesn't fix the h_xy
  misattribution mechanism itself).
- **Flow-KF reset on primary-marker switch** (img_data.py, mirrors the existing loom/centroid
  reset pattern already present for the 2026-07-04 fix): forces `_kf_initialized=False` on a
  marker-ID switch instead of blending the new marker's measurement into the old marker's
  filter state via a normal Kalman update.

## NOT fixed — the `FLOW_LAT_REDUCED` model-misspecification itself

The `w_xy`-drop assumption breaking under real drone rotation (step 5) is NOT fixed by any of
this session's changes — it's a genuine, confirmed mechanism that would recur any time a large
attitude disturbance coincides with active flow computation, independent of marker switching.
Proper fix needs runtime gating (e.g. fall back to the full 8-column solve, `FLOW_TARGET_LEVEL=0`
equivalent, when `|gyro w_xy|` or `|e_R|` exceeds a threshold) — flagged, not implemented, too
invasive to land without dedicated testing.

## Calibration/filter mismatch (flagged this session, fixed in a PARALLEL session)

Separately found: all sensor-cal matrices (`_sensor_cal_hw`, `_sensor_cal_ring`,
`_sensor_cal_s`) were derived by fitting the RAW unfiltered per-frame signal but applied at
runtime to KF-filtered state — plausible error-amplifier during exactly this kind of transient.
A parallel session re-derived the cal against the correctly-filtered (KF) signal
(`feedback_kf_savgol_cal_mismatch` — not written by this session, check if it exists before
citing further) and this session merged those values in for the final n=5 validation.
