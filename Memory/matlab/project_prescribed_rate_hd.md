---
name: project_prescribed_rate_hd
description: "Prescribed-rate h_d (user design, 2026-07-02): s_dot_meas -> phi_max.*S_r.*dp_r in h_d, s_ddot-drop removed; 25/25+25/25 gate + A/B breach benefit (engR .76 vs 1.16). RESOLVED: formula complete as a DEFINITION (h_e = p_10*zeta_r_dot/g_r — S_r varying IS the signal); NO S_r_dot law; finite-diff dh_d kept per user"
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

**✅ RESOLVED (user 2026-07-02): the formula is COMPLETE as a DEFINITION — no Ṡ_r law.** A
prescribed Ṡ_r contraction was the WRONG approach (it would make the reference the convergence
driver, violating funnel-as-constraint / ASMC-drives-convergence). The correct reading:
`h_d,xy = p_10·S_r·ṗ_r` is the funnel-consistent desired rate, and S_r VARYING is the design
signal, not a flaw — `h_e,xy = ṡ − p_10·S_r·ṗ_r = p_10·(dr̄_e − S_r·ṗ_r) = p_10·ζ̇_r/g_r`,
i.e. h_e IS the (scaled) barrier-coordinate rate: zero when riding the funnel at constant S_r,
grows exactly on adverse S_r motion (incl. the clamp-on-breach recovery case). σ = ζ_h + χ_r·ζ_r
is thus a true PD pair on the barrier coordinate (ζ_r position, ζ_h its rate). dh_d: user chose
KEEP the smooth4 finite-diff of h_d_all as-is (carries a tiny measured Ṡ_r content, RMS .007 vs
old s̈ ~7; gate-validated) over the analytic S_r-frozen alternative. Code as committed (016855e)
is the final form. PX4 side untouched (uses measured s_dot; port is a separate decision).
