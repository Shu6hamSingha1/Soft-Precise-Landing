---
name: feedback_savgol_predict_suspect_flyaway
description: "PLASMC_SAVGOL_PREDICT (default-ON since 2026-06-29) is a live CONTROL-AFFECTING spike-reconstruction on self._h, newly suspected as the direct cause of a 23.93m fly-away (densedbg2 rep5) -- NOT confirmed, live debug added (SAVGOL_PREDICT_DBG), investigation in progress."
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 3c2f4c67-05c1-4e6f-966b-0e62018fc8a7
---

**`self._savgol_predict` (controller.py ~567, default-ON, baked 2026-06-29 "model-predict lag comp")
is NOT a logging/diagnostic feature -- it directly overwrites `self._h[-1]`, which feeds the very next
line (`self._h_e.append(self._h[-1] - self._h_d[-1])`) -> zeta -> sigma -> a_v -> a_u (the actual
commanded thrust/rate).** Confirmed via code read, controller.py:1447-1571.

**Mechanism:** when the RAW measurement's rate-of-change (`dh = |h_raw[t]-h_raw[t-1]|/dt`) exceeds
`_dh_limit` on a lateral axis, that axis is treated as an untrustworthy spike and REPLACED with a
model-based prediction (`_predictModel_h`, using the manuscript's `ḣ_z=-β·a_z-h_z²` depth-coupled
dynamics; `_predict_mode="model"` is the active default, NOT the savgol branch despite the flag name).

**Evidence of a bug (2026-07-07, densedbg2 rep5, 23.93m fly-away):** `h_x` jumped 0.016->+1.253 in ONE
control step (~12ms) in `Control_Data.npy`'s `h(t)` field, driving zeta/a_u to explode. BUT: neither
`Img_Data.npy`'s raw `Opt Flow Ang Vel` NOR its `Opt Flow KF` field show ANY corresponding jump in the
same window -- both stay smooth (-0.19, -1.20ish) throughout. The funnel-outlier-containment logic
(controller.py ~1583, fires on `abs(h_e/p)>=1.0`) also did NOT fire (ratio stayed <0.95 the whole
window) -- ruled out as the source too. This means the DISCREPANCY originates specifically in the
`_savgol_predict` spike-reconstruction substitution: either (a) the RATE-OF-CHANGE gate itself
false-triggered on `self._h_raw` (the controller's OWN raw-measurement copy, distinct from img_data's
logged fields -- not yet verified these two actually match), or (b) `_predictModel_h`'s physics-based
prediction is WRONG in this regime (near-touchdown, depth/β likely degenerate -- the same class of
depth-adjacent breakdown seen repeatedly this session at close range).

**PARTIALLY CONFIRMED (2026-07-07 sgpdbg.sh, 4/5 reps landed bounded, no fly-away reproduced).**
`SAVGOL_PREDICT_DBG=1` fired 4-21 times per rep on ORDINARY landings -- NOT rare. Sample firings show
`dh` values of 10-18 (way over any sane per-frame flow-rate) despite `h_raw_prev`/`h_raw_now` being
SMALL and unremarkable (0.005-0.26 magnitude) -- i.e., the spike gate is triggering on NORMAL small
h fluctuations RATE-AMPLIFIED by a near-floor `dt` (the `_dtc = max(dt, 0.006)` floor), not on genuine
large jumps. In these captured cases `h_before`->`h_after` stayed in the same small range (mildly
adjusted, e.g. 0.21->-0.025) -- mostly harmless here, consistent with no fly-away in this batch. This
confirms the mechanism false-triggers FREQUENTLY on ordinary noise via the dt-floor amplification path
-- a real bug in the gate itself (dh should probably use a longer/smoothed dt baseline, not raw
per-step dt near its floor) -- but the SPECIFIC catastrophic case (h jumping to +1.25) was not
reproduced in this batch. `[sgp]` lines are in `run_logs/sgpdbg_*.log`.

**How to apply:** if a future fly-away shows a discontinuous h jump in Control_Data that does NOT
appear in Img_Data's raw/KF fields, and the funnel-outlier-containment ratio stayed <1.0, suspect
`_savgol_predict`/`_predictModel_h` FIRST -- check for `[sgp]` lines at the trigger moment. Candidate
fix (not implemented): validate/bound the model-prediction against the actual raw measurement before
accepting it, or disable the substitution near touchdown where the depth-coupled model is
known-degenerate. Do NOT disable `PLASMC_SAVGOL_PREDICT` wholesale without a validated A/B -- it was
baked for a real lag-compensation reason; the fix should target the failure regime, not remove the
mechanism outright.
