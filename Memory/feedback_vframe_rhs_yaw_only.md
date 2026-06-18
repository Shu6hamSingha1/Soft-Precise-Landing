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
Refines [[feedback_outputcal_flow_validation_vframe]].
