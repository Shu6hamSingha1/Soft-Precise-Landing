---
name: feedback_vframe_rhs_yaw_only
description: Output-cal optic-flow sanity check RHS must zero V-frame roll/pitch (leveling removes it). Fixed 86509bb; fps + attitude-lag were red herrings.
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 7faf44bf-c5f1-4b57-a701-f6d868abfdc1
---

The optical-flow sanity check (`plotter_output_calibration.ipynb` cell 20) built
`RHS = L·[h; V_w_tug]` with the **full** body angular rate transformed to V.
**Wrong:** the V-frame is gravity-leveled, so a body roll/pitch produces NO
V-frame feature motion — the leveled flow `ṡ_V` (the LHS, from `_getVirtualPts`)
carries only **translation + yaw**. The runtime `lstsq` is still **6-DOF**
(`_fill_A` is 2N×6, solving `[h_x,h_y,h_z,w_x,w_y,w_z]`) but `w_x,w_y` come out
structurally **~0**: the leveled input has no roll/pitch rotation, the `w_x/w_y`
columns are rank-deficient (∥ the translation columns; `rcond=1e-3` truncates
them), and `CTRL_ZERO_WXY=1` zeroes them in control. So the RHS over-predicted
by the roll/pitch flow and diverged on tilt-heavy runs.

**Fix (commit `86509bb`, 2026-06-06):** zero `V_w_tug[:, :2]` (V-frame roll/pitch)
before forming `L·[h; w]`.

**Why this matters / red herrings:** this resolved a long mis-diagnosis chain.
The symptom "recent recording fails the flow check, pre-fpsfix matches" was
investigated as (1) the fps fix (stamp vs `/clock`) and (2) EKF-attitude lag in
the leveling — **both were red herrings**:
- fps: both clocks sit at 62.5 Hz (no skew); the **image stamp is the correct
  fps** (Δv spans the capture interval) — the fix was right. Stamp jitter (std
  ~9.8 ms) is real but cancels in `Δv·fps`. (Diagnostic `Image Stamp` log added
  `bba5c33`.)
- attitude: forward-predicting the EKF quat does nothing; my sweep was a no-op
  (one-quat common rotation can't change flow magnitude).
The real issue was **leveled LHS vs non-leveled RHS**. After the fix: the recent
(correctly leveled) recording matches (corr 0.81→0.94, slope 1.58→0.96); the
**pre-fpsfix recording correctly FAILS** (its recorded flow *retained* roll/pitch —
its leveling effectively didn't remove it, pre-dating proper per-frame quat
handling / stale quats). So "pre matches / recent fails" was **backwards**: the
correct recording was failing an incorrect check.

**How to apply:** when validating V-frame optic flow against GT, the RHS rotation
must be **yaw-only** (the V-frame doesn't tilt). This also confirms the runtime
V-frame flow correctly drops roll/pitch — consistent with
[[feedback_wxy_unobservable_imu_fusion]] and `CTRL_ZERO_WXY`. The earlier
"flow-check corr stuck ~0.6" was largely this phantom r/p term, not LK noise.
Consolidates the V-frame model + naming convention from the former `feedback_outputcal_flow_validation_vframe` (folded below).

## V-frame model + naming convention (consolidated from feedback_outputcal_flow_validation_vframe, 2026-06-18)

**Frame model (corrected 2026-06-04, user-confirmed vs the canonical reference notebooks):**
- **Camera frame = body-FRD** — axes aligned, no rotation (SDF mount + cv2 rotation; origin may be offset). Verified: image `s_x`↔body-forward r=+0.89, `s_y`↔body-right r=+0.95.
- **Virtual (V) frame = gravity-LEVELED camera = `rotz(yaw)`** (roll/pitch removed, yaw kept). So `R_{V←body}` is the leveling rotation — **identity ONLY when level, ≠ I under tilt.** `R_V_from_body = I` is WRONG (it conflates camera=body [true] with V=body [false]); added 2026-06-01 (`fce9b84`), corrected 2026-06-04 across output-cal cells 5/6/13/39/40, input-cal 0/24, and the `img_data.py _getVirtualPts` comment. `_getVirtualPts` builds V via cross-product (`x=body_y×gravity`), differing from `rotz(yaw)` by a 2nd-order roll·pitch term (≤1° at 11° tilt; runtime keeps the cross-product frame to match the logged LHS).
- The IBVS flow equation `ṡ=L·[h;w]` is validated in the V frame (runtime-faithful): `img_data` solves `[h;w]` IN the V frame via `lstsq(_fill_A(V_pts), Δ·fps)`, so the check must use V-frame quantities on both sides (`h_z=v/Z` uses `z_V=W_x_tu[2]`, the V-optical-axis altitude, NOT body-z).

**Naming convention (enforced 2026-06-04, commit `ed27641`): frame prefix = ACTUAL frame.** `B_`=body-FRD, `V_`=virtual; image-velocity `y`→`h`. Renames: `B_y_g`→`B_h_g`, `B_y_g_V`→`V_h_g`, `xc_gt/yc_gt`→`V_xc_g/V_yc_g`, `B_w_tug_V`→`V_w_tug`. **Discovered during the rename:** `B_y_g` was FRAME-INCONSISTENT across tools (VIRTUAL in aggregate_calibration_phased/derive_board_cal/find_camera_rotation → `V_h_g`; BODY in validate_pose_transforms/analyze_calibration/aggregate_calibration/tune_savgol → `B_h_g`).

**OPEN SEMANTIC FLAG (not yet fixed — would change cal values):** the cal derivation mixes frames per channel — even the canonical `aggregate_calibration_phased` uses `V_h_g` (virtual) for the h-axes but `B_w_ug` (body) for the w-axis. At low tilt body≈virtual so the cal is ~right; for exactness GT should be V-frame on all 6 channels. Decide separately (behavior change + re-validation). See [[feedback_getvirtualpts_g_sign]], [[feedback_imgdata_gt_clock_skew]] (the `R_V=I` claim is corrected here).
