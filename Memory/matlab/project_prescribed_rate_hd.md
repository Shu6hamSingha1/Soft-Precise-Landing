---
name: project_prescribed_rate_hd
description: "Prescribed-rate h_d (user design, 2026-07-02): s_dot_meas -> phi_max.*S_r.*dp_r in h_d, s_ddot-drop removed (deriv 1000x smaller, folded into c-term); 25/25+25/25 gate + A/B breach benefit (engR .76 vs 1.16) — but OPEN: formula incomplete, S_r is NOT constant; awaiting prescribed S_r_dot law"
metadata:
  type: project
---

**User design:** replace the measured centroid rate in the desired flow with the funnel-prescribed
rate — `h_d` xy term: `s_dot_meas` -> `p_10.*S_s.*p_s_dot` = `P.phi_max .* S_r .* dp_r`
(S_r = clipped r_bar_e/p_r from position_funnel, dp_r = funnel contraction rate). Rationale:
s_dot_meas wrongly injected a noisy s_ddot into dh_d (forcing the s_ddot-drop); the prescribed
rate is smooth, and on a funnel breach S_r saturates -> h_d stays bounded -> h_e = h−h_d grows ->
recovery control authority.

**Implemented (uncommitted):** `position_funnel.m` returns 4th output `s_dot_presc`;
`flow_surface.m` uses it in h_d AND folds it into `h_d_all` so its derivative enters
V_dh_d/c-term — s_ddot-drop REMOVED (justified: d(s_dot_presc)/dt RMS .007 vs measured s_ddot
RMS ~7, 1000x smaller, measured on Circular IC2). Wired in `run_simulation.m` +
`Comparison/visualControl_comparison.m` (visualControl_IBVS_adaptive has its own inline controller
— NOT wired). dr_bar_e in position_funnel KEEPS the measured rate (barrier needs true error
velocity; only h_d switched). Backups: `Obsolete/position_funnel_pre_prescrate.m`,
`Obsolete/flow_surface_pre_prescrate.m`.

**Validation:** full gate 25/25 realistic + 25/25 noiseless (baked gains, no override), speed
sweep 20/20 (+-40%, 4 moving traj). A/B vs measured (same seeds): standard stress 1x–7x EQUIVALENT
(5/5 both, engR 0.59 — no breach occurs, benefit can't manifest); BREACHING scenario (Circular 2x
yaw): prescribed engR 0.76 NO BREACH + worstXY 1.12 vs measured engR 1.16 BREACH + 1.56 — the
recovery-authority design works exactly as argued.

**⛔ OPEN — formula incomplete (user 2026-07-02):** `phi_max.*S_r.*dp_r` is d/dt(s_e) along the
funnel ONLY if S_r is constant. Since s_e = phi_max·S_r·p_r, the full derivative is
`phi_max·(S_r_dot·p_r + S_r·dp_r)` — the S_r_dot·p_r term is MISSING. S_r_dot must be a
PRESCRIBED contraction (a measured S_r_dot collapses the expression back to s_dot_meas —
circular). Candidates offered (exponential pull S_r_dot=−k·S_r; barrier-driven); awaiting the
user's chosen S_r_dot law, then re-verify the derivative into the c-term stays smooth.
Current tree holds the incomplete-but-validated form. PX4 side untouched (uses measured s_dot;
port only after the completed form is validated — [[feedback_shared_issue_fix_in_ubuntu]] pattern
reversed: this originated in MATLAB).
