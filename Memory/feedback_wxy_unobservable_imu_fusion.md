---
name: wxy-unobservable-imu-fusion-deferred
description: "CORRECTED 2026-06-07: Image wx/wy (roll/pitch rate) ARE observable — the '2026-06-04 geometrically unobservable' verdict was WRONG. A proper-excitation re-test recovers wx/wy; the old r≈-0.07 was an under-excitation artifact (only ~0.1 rad/s achieved) compounded by analysis in the V-LEVELED frame (which de-rotates r/p out by construction). The real degeneracy is CORNER-CLUSTERING (single marker near image center → x²,y²,xy curvature vanishes), broken by the multi-marker board + ring SPREAD ([[project-landing-target-design]]). IMU↔image fusion + reduced-lstsq remain a moving-target convenience, not a fix for an unobservability that doesn't exist for the spread board."
metadata:
  node_type: memory
  type: feedback
  originSessionId: 7faf44bf-c5f1-4b57-a701-f6d868abfdc1
---

> **⚠️ CORRECTION (2026-06-07) — the 2026-06-04 "geometrically unobservable" verdict below is WRONG.**
> A **proper-excitation re-test** (driving roll/pitch well past the bandwidth-limited ~0.1 rad/s the
> 06-04 test actually achieved) shows image **wx/wy ARE observable**. Two confounds invalidated the
> old null: (1) it measured in the **V-LEVELED frame**, where `_getVirtualPts` de-rotates roll/pitch
> out *by construction* → there was nothing left to observe; (2) PX4 position-control bandwidth filtered
> the 1 Hz excite to only ~0.1 rad/s, so the signal was buried. The true degeneracy is **corner-
> CLUSTERING** (single small marker near the image center → the x²,y²,xy curvature that separates
> tilt-rate from translation vanishes), and it is **broken by the multi-marker board + ring SPREAD**
> (cond 800→60; [[project-landing-target-design]] already flagged "re-enable w_x,w_y now spread is good").
> **Implication:** `W_XY_DEROT='zero'` is now a *modeling choice* (assume level target), NOT forced by
> unobservability. BUT **observable ≠ calibrated**: the runtime solves flow in the gravity-leveled
> **V-frame** (`_getVirtualPts`), where body roll/pitch produce NO rotational flow → the cal
> (`derive_board_cal.py`, fit in V) has no wx/wy signal and `_sensor_cal_hw`'s **Wx/Wy rows are 0**. So
> `W_XY_DEROT='image'` would inject **UNCALIBRATED** (raw, mis-scaled) wx/wy — not usable until a
> **raw/C-frame wx/wy calibration** exists (deferred; the V-frame cal cannot produce it). `'imu'` (gyro
> via `C_R_V.T`) is the **calibration-free** de-rotation path (no image cal needed; assumes level target).
> Net: `'zero'` stays the default; using image-wxy needs new C-frame cal, not just flipping the knob.
> The IMU-fusion/reduced-lstsq below is a moving-target convenience. Everything from here down is the SUPERSEDED 06-04 analysis.

**Finding (2026-06-04, two output-cal recordings, mechanism-confirmed) [SUPERSEDED — see correction above].** The 6-DOF lstsq's
image **wx/wy cannot be recovered** from the downward planar ArUco board — it's a *geometric*
rank deficiency, not a frame bug or a noise/cal issue:
- Image wx/wy vs IMU-in-V: **r ≈ -0.07** even in a dedicated roll/pitch-excite recording
  (`rollexc`/`pitchexc` phases). Image **wz vs IMU-in-V: r = -0.87** (observable; opposite sign,
  ~1.4-1.6× raw scale — varies with `size_factor`).
- **Mechanism (smoking gun):** during roll-excite, the wx-induced flow lands in the **vy** column
  (`corr(image_vy, IMU_wx)=+0.23`); during pitch-excite it lands in **vx** (`corr(image_vx,
  IMU_wy)=-0.29`). The wx interaction column ∥ vy column, wy ∥ vx — so the solve cannot tell
  roll-rate from sideways-slide. Amplitude-independent (stronger excite → bigger *leak*, not
  observability). PX4 position-control bandwidth also filtered the 1 Hz excite cmd → only ~0.1 rad/s
  achieved, but that doesn't change the geometric conclusion.
- **Methodology trap avoided:** must compare image w (V-frame) against the IMU *transformed into V*
  via the SAME `C_R_V.T` that `_getVirtualPts` builds (NOT raw body-FRD — that's the same class of
  error as the [[feedback_imgdata_gt_clock_skew]] mismatch). Raw-frame comparison hid a spurious
  wx r=0.29; the V-matched + excited measurement is the real -0.07.

**Refinement (2026-06-06).** The deeper mechanism is the **V-frame leveling itself**:
`_getVirtualPts` de-rotates each frame to gravity-level, so a body roll/pitch
produces NO V-frame *rotational* flow (the V-frame doesn't tilt). The lstsq is
still **6-DOF** (`_fill_A` 2N×6, solves `w_x,w_y` too) — but with no r/p rotation
in the leveled input, plus the rank-deficient `w_x/w_y` columns (`rcond=1e-3`
truncates them) and `CTRL_ZERO_WXY`, it returns `w_x,w_y≈0`. So "rank deficiency"
and "leveling removes r/p" are two views of the same fact, and the leveling is
**correct, not a bug**. Confirmed by the cell-20 output-cal fix (`86509bb`): the
GT RHS had to zero `V_w_tug[:,:2]` (yaw-only V-rotation) to match the leveled
flow. See [[feedback_vframe_rhs_yaw_only]].

**Decision (user, 2026-06-04): "We will do it later for moving target."** The IMU↔image wx/wy
**fusion** (EKF on ω_target) and the **reduced lstsq** (subtract IMU wx/wy from the flow before a
4-DOF `[vx,vy,vz,wz]` solve → clean lateral `h`) are DEFERRED. They assume a **level target**
(relative wx/wy = drone IMU wx/wy): *exact* for the stationary board + a ground rover (ω_target_wx/wy≡0),
but **invalid for a rolling/pitching target** — and since the image is blind to wx/wy, target roll is
*unobservable* by any sensor in the suite (no method can fully correct it). For a rolling target the
reduced lstsq still strictly beats the rank-deficient full solve (removes the dominant drone-tilt
contamination, leaves only the smaller target-roll residual), but it's a moving-target concern.

**Why:** stationary-board landing is the active scenario; the fusion adds complexity whose only
payoff (separating drone-roll from target-roll) needs a moving target. wx/wy=0 (`CTRL_ZERO_WXY=1`,
default) is fine for the stationary board.

**How to apply:**
- For the **stationary board**: keep `CTRL_ZERO_WXY=1`. Do NOT build the fusion/reduced-lstsq now.
- The **infrastructure is already in place** (uncommitted) for when the moving-target work resumes:
  IMU body-rate plumbing `FC.getAngVelIMU()` → `Image_Node._angvel_deque`/`getAngVels()` →
  `img_data` logs **`'IMU AngVel'`** (FRD) + **`'Quat'`** (FC [w,x,y,z]) synced to the flow; the
  `C_R_V.T @ imu_FRD` transform is validated; `record_output_calibration.py` has `rollexc`/`pitchexc`
  excite phases (env `CALIB_AMP_RP`/`CALIB_FREQ_RP`/`CALIB_RP_S`); scratch analyzers
  `/tmp/plot_imu_v.py`, `/tmp/plot_imu_v_seg.py`, driver `/tmp/rollpitch_outputcal.sh`.
- ~~Don't re-run the wx/wy observability investigation — it's settled.~~ **OVERTURNED 2026-06-07 (see top correction): wx/wy ARE observable under proper excitation + corner spread.** See [[feedback_moment_yaw_canonical]],
  [[feedback_scale_free_depth_free]] (IMU is a body sensor, allowed; the "level target" assumption is a
  model assumption, not a truth-data leak).
