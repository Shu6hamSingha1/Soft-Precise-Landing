---
name: improve-perception
description: Diagnose and fix PERCEPTION-LAYER issues in the PX4/Gazebo PLASMC pipeline — optical-flow (h/w) quality, KF/filter state bugs, marker detection/tracking (ArUco decode, KLT fallback, marker handover), the touchdown detector, ring-flow reliability, and sensor calibration. Use when the failure is in WHAT is measured or WHEN a detection event fires, not in the control gains that consume it. Distinct from `tune-plasmc` (control-parameter tuning) — if you're not sure which applies, ask "is the measured signal wrong/stale/frozen, or is the response to a correct signal wrong?" The former is this skill.
---

# Improve Perception

A playbook for diagnosing and fixing perception-layer bugs in `PX4_Gazebo/src/img_data.py` (image
processing, optical flow, marker tracking) and the perception-adjacent parts of `controller.py`
(the touchdown detector, which consumes perception output but isn't a tuning knob). Distinct from
`tune-plasmc`, which is for control-GAIN tuning on a correctly-measured signal.

**When to use this vs `tune-plasmc`:** if the question is "why is `h(t)`/`s(t)`/`MARKER_EXTENT_PX`
wrong, frozen, stale, or discontinuous," or "why didn't the touchdown detector fire / why did it
fire too early," use THIS skill. If the question is "the signal looks right but the response is
too aggressive/sluggish/oscillatory," use `tune-plasmc`.

## Mental model — the perception pipeline's failure surface

```
Camera image → ArUco decode (or KLT fallback, or dense-recovery) → raw corner positions
  → V-frame de-rotation (_getVirtualPts, per-frame quaternion) → interaction-matrix lstsq (_fill_A)
  → raw flow [h;w] → KF (or savgol) filter → sensor calibration (_sensor_cal_hw/_s/_ring)
  → consumed h(t)/s(t) → controller.py's touchdown detector (_touchdownDetect, checks h_z sign)
```

Every stage can independently corrupt or silently degrade the signal. The 2026-07-10/11
investigation found bugs at FOUR different stages in one incident chain — don't assume a single
root cause; trace the FULL pipeline for a failing rep before concluding.

## Known failure modes (traced 2026-07-10/11, see memory for full detail)

| # | Failure mode | Diagnostic signal | Status |
|---|---|---|---|
| **P1** | **Touchdown detector structurally blocked by a config contradiction** | `h(t)`'s `h_z` never goes positive despite genuine ground contact; no `TOUCHDOWN-DETECT` print; disarm falls through to the accelerometer-impact fallback | **FIXED** 2026-07-10 — `FLOW_LOOM_SIGN_GUARD` (default-on, clamps `h_z≤0`) directly contradicted `TOUCHDOWN_LOOM`'s `h_z>0` requirement; the two features were baked independently and never validated together. Sign-guard default flipped to 0. See `px4/feedback_loom_sign_guard_blocks_touchdown_detect`. |
| **P2** | **KF frozen during any marker-loss gap** | `h_z` (or any KF-filtered channel) pinned at ONE EXACT value for the entire duration of a decode/tracking gap, regardless of real motion | **FIXED** 2026-07-10/11 — `_kf_update()` was only called on real-data frames; the KF's predict step never ran without a fresh measurement. Fixed via a predict-only coast mode (`_kf_step(z=None)`, `_kf_predict_only()`) called every miss-frame, paired with decoupling the state-propagation `dt` (stays capped, numerical stability) from the uncertainty-growth `dt` (now reflects the true elapsed gap, `KF_DT_UNC_MAX`). See `px4/feedback_kf_frozen_during_marker_loss`. **General lesson: any filter state must either predict-forward every tick or explicitly widen its uncertainty on resume — silent freezing during a gap is a bug class, check for it in any new KF/filter path.** |
| **P3** | **Marker-ID handover corrupts flow via a genuine model misspecification** | `MARKER_EXTENT_PX` jumps >5× in ONE frame right after a decode gap ends; `h_xy` explodes over the following ~0.3-0.5s; `theta`/`kappa` ratchet in lockstep | **PARTIALLY FIXED.** `FLOW_LAT_REDUCED=1` (baked default) drops the `w_xy` rotational columns from the flow lstsq, assuming a level target (`w_xy≈0`). This assumption silently breaks the instant a REAL drone attitude-rate disturbance is present (check `w(t)`, the raw IMU gyro reading, independent of flow) — the disturbance gets forcibly misattributed into the translation unknowns (`h_xy`), amplified by `r²` via the newly-locked marker's corner distance from the optical center (bigger marker = corners farther out = bigger misattribution for the SAME real rotation). Flow-KF now resets on a primary-marker switch (was previously blending old/new marker state via a normal Kalman update — same class of bug as the loom/centroid resets already present since 2026-07-04). **The `FLOW_LAT_REDUCED` misspecification itself is NOT fixed** — needs runtime gating (e.g. fall back to the full 8-column solve when `\|gyro w_xy\|` or `\|e_R\|` exceeds a threshold). See `px4/project_ic1_terminal_kick_root_cause_chain`. |
| **P4** | **Touchdown detector has no velocity/magnitude gate** | Landing classified PRECISE-only, not SP: tiny xy_err but rel_vel clustered at/just above the 0.2 m/s threshold | **OPEN, current frontier (2026-07-11).** `_touchdownDetect()` checks loom SIGN + persistence (3 frames) + centering — never checks velocity MAGNITUDE. A geometrically-valid contact/rebound can be detected while vertical speed is still substantial. Needs a magnitude/rate-aware gate (h_z IS meant to encode velocity — use its SIZE, not just its sign). See `px4/project_touchdown_detect_velocity_gate_gap`. |
| **P5** | **Ring-flow LK survival is texture-limited, not geometry-limited** | `RING_FILTER_DBG` shows LK-status survivors low (~15%) even at stable hover, well before any tilt/marker-size effect; arm-mask/MAD/pairing filters compound the loss further | Diagnosed, not fixed as of 2026-07-10 — background/ground texture-poverty in the Gazebo world, confirmed via per-stage survivor counts. A fine-stipple marker texture DOUBLED survival but caused a net landing regression (see P6). A coarse-square alternative exists, untested. |
| **P6** | **Marker texture changes can improve one signal and break another** | Ring-flow LK survival improves, but ArUco's own primary corner/threshold decode gets WORSE (more `TARGET_LOST`) | Fine-stipple texture (`0-small_10-big_textured.png`) FALSIFIED 2026-07-10 — net landing regression despite a real ring-flow win, isolated via a controlled n=5 A/B (same code, texture toggled). **Any marker-appearance change needs its own n=5 A/B before trusting it — a win on one perception signal can cost more on another.** See `px4/feedback_textured_marker_falsified`. |
| **P7** | **Sensor calibration derived on a different filter than runtime consumes** | Cal matrices (`_sensor_cal_hw/_s/_ring`) fit against a raw or differently-filtered signal, applied at runtime to a KF-filtered (or savgol-filtered) signal | Diagnosed 2026-07-10 (this session), fixed in a parallel session (`feedback_kf_savgol_cal_mismatch` — check if that memory exists before citing further) via a KF-refit recalibration, merged into this branch 2026-07-11. **General lesson: whenever `derive_*_cal.py`/`aggregate_calibration*.py` tools are used, verify their signal path (raw/savgol/KF) matches `IMG_FILTER`'s actual runtime default — a static linear cal fit on one filter shape and applied to another is not guaranteed valid.** |

## Diagnostic procedure for a perception-suspected failure

1. **Check `MARKER_EXTENT_PX(t)` and `N Flow Corners`** (Control_Data / Img_Data) for freezes,
   discontinuous jumps, or long zero-streaks. A frozen/discontinuous extent is the first clue —
   trace backward from there (was it a decode gap? a KLT-fallback failure? a marker-ID switch?).
2. **Check the log for `ArUco lost/re-acquired` and `KLT fallback` lines** to bound the gap
   window, then correlate with Control_Data timestamps.
3. **Cross-check against `w(t)` (raw IMU gyro, independent of flow)** — if the drone had a real
   attitude-rate disturbance DURING a gap, the flow computation had no way to see it (pre-P2-fix)
   or may misattribute it (P3). Compare `\|w\|` at the gap boundary against the flow explosion's
   onset.
4. **Validate a claimed KLT-fallback/coast prediction against GT** using the canonical GT-flow
   tool (`tools/gt_optical_flow.py`, importable `compute_gt_flow(rep_dir)` — NOT the buggy
   `_main()` CLI path if `Opt Flow Fused` is 1-D in the recording, e.g. `FLOW_FUSE_RING=0` runs).
   Align GT loom/flow to `img['Time']` via the returned `align(t_abs, y)` helper and compare
   against the logged/consumed `h(t)` during the specific gap window — this is the ONLY reliable
   way to know whether a KLT-predicted or coasted value was actually accurate, vs just plausible.
5. **Audit quaternion/frame pairing** if a de-rotation (`_getVirtualPts`/`_getRealPtsFromV`) is
   suspected — grep every call site and verify frame0 points pair with `quats[0]`, frame1 with
   `quats[1]`. A past bug (fixed 2026-07-04) used the wrong index and produced "residual tilt ∝
   angular rate." Also verify quaternion/image CAPTURE-time sync at the source
   (`gz_subscriber.py`'s `image_callback` — quat should be fetched in the SAME callback as the
   image, not from a separately-timed buffer).
6. **For calibration-suspected issues**, use the `io-calibration` skill's derive/validate
   workflow — but FIRST check P7 above (filter-mismatch) before re-deriving blind.

## Anti-patterns (found this session — don't repeat)

- **Assuming a filter (KF/savgol) that isn't updated during a gap "just holds steady" safely.**
  It can — but only if paired with growing uncertainty AND a downstream consumer that checks
  that uncertainty. A silent freeze with no uncertainty growth is a correctness bug, not a
  reasonable degradation (see P2).
- **Fixing a symptom (sign-guard clamp) without checking what ELSE depends on the unclamped
  behavior.** The sign-guard was a legitimate fix for its own problem (wrong-signed ring loom)
  but nobody checked it against the touchdown detector's requirement before baking both
  default-on (see P1).
- **Trusting a "coarser/lighter" texture experiment without a controlled A/B.** A texture change
  that measurably helps one signal (ring-flow LK survival) can measurably hurt another (ArUco
  decode) — isolate with a same-code, texture-toggled n=5 comparison, not intuition (see P6).
- **Jumping to "this must be a control-law bug" for a terminal explosion without first checking
  whether the INPUT signal was already corrupted.** Several a_u-explosion incidents this session
  traced back entirely to a perception-layer freeze or misattribution, not a control-law flaw.

## Related skills
- **`tune-plasmc`** — control-gain tuning, once perception is confirmed correct.
- **`io-calibration`** — the derive/validate workflow for `_sensor_cal_hw/_s/_ring`. Read P7
  above first if runtime consumes a KF/savgol-filtered signal — a static cal derivation needs
  to match that filter.

## Related memory (px4/ folder)
- `feedback_loom_sign_guard_blocks_touchdown_detect` (P1)
- `feedback_kf_frozen_during_marker_loss` (P2)
- `project_ic1_terminal_kick_root_cause_chain` (P3, full incident trace)
- `project_touchdown_detect_velocity_gate_gap` (P4, current open item)
- `feedback_textured_marker_falsified` (P6)
- `feedback_ring_fusion_marker_overlap` (a DIFFERENT ring issue, fixed-radius overlap near
  touchdown — not the same as P5's texture-limited survival)
