---
name: feedback_cbf_qp_slack_validated_px4_unlogged_matlab
description: "Checked whether the Tier-1 visibility-QP's penalised slack (CBF_VIS_RHO / P.cbf_vis_rho) is genuinely needed, on real data. PX4: YES but as graceful-degradation for the tail, not a rescue mechanism -- 352 reps scanned (vis_slack(t)/vis_active(t)): successful landings (n=146) engage the QP 31% of the time but slack stays ~0 (median 0, 2/146 ever >0.1, max 0.19) -- the hard constraint alone almost always suffices when things are going well. Failed/degraded reps (n=206) engage 61% of the time and slack grows large (p90 0.62, max 22.1), but the large values ALL occur at the very end of already-diverging trajectories (checked 5 worst directly: target_lost, xy_err up to 12.3, the moving_cand_IC1 kappa-detonation rep) -- slack is a SYMPTOM of an already-bad state, not what saved the run. MATLAB: CANNOT be validated from existing data -- cbf_visibility.m computes s_star/cbf_ok but run_simulation.m never logs it; no .mat dataset (incl. post-parity-port Comparison results) carries the two-tier QP's diagnostic output at all."
metadata: 
  node_type: memory
  type: feedback
  originSessionId: f78fde4c-2847-4689-96f6-0693b2bc0c15
  modified: 2026-09-15T06:06:42.266Z
---

**CBF-QP slack (Tier-1 visibility QP, `visibility_projection.py` / `cbf_visibility.m`) — validated on
real data whether it's genuinely load-bearing, not just present for numerical safety (2026-09-14).**

Continues [[project_20260909_visibility_projection_wire_in]] (which baked the slack term
`CBF_VIS_RHO=2000` / `P.cbf_vis_rho=2000` "for graceful degradation, never infeasible" but never
checked whether it actually engages).

**PX4: genuinely needed, but only in the tail — confirmed graceful-degradation design intent, not
a rescue mechanism.** Scanned 352 reps (`vis_slack(t)`/`vis_active(t)` logged fields) across
`ICValidation` IC1-5, `VisProjGate`/`VisProjQPGate`, `RoverCBFSweep`, `XirXi2_RoverGTFB`, `Multi_IC`:

| | n | QP active | slack magnitude |
|---|---|---|---|
| successful (precise or soft) | 146 | 31% of reps | median 0, only 2/146 >0.1, max 0.19 |
| failed/degraded | 206 | 61% of reps | median ~0, p90=0.62, **max 22.1** |

Checked the 5 highest-slack reps directly: ALL are failures (`CircularYaw/off` target_lost=True xy=1.6;
`Linear/off` xy=4.0; `moving_cand_IC1` from [[feedback_xir_xi2_handoff_refuted_sitl]]'s detonation rep,
slack=**22.1**, xy=**12.3**, kappa-ratchet blowup). In every case the slack peaks at the VERY END of an
already-diverging trajectory — same touchdown/terminal-window pattern as
[[feedback_rk_compatibility_hypothesis_refuted_sitl]]'s R_k(t) dip. **Slack is a symptom of a state
that's already bad, not evidence it rescued the run.** But the design intent — graceful degradation
instead of hard infeasibility when a rare bad state occurs (IC5 short-runway, rover moving-target,
a bad gain draw) — is genuinely validated: without slack those reps would have hit a hard-infeasible
QP at exactly those moments instead of degrading.

**MATLAB: cannot be validated — the diagnostic is computed but never logged.**
`MATLAB/VDF_ASMC/+blocks/cbf_visibility.m` computes `s_star = max(|c0+L_e·y*|-phi, 0)` and returns
`cbf_ok = all(s_star<=1e-6)` as a function output, but **`run_simulation.m` never captures it** —
grepped, zero hits. Checked the most recent post-parity-port datasets
(`Datasets/Comparison/result_ctrl_1.mat`): only the OLD legacy cone-clamp fields
(`theta_cone_log`/`rho_fov_log`/`d_min_log`) are present, nothing from the two-tier module. Consistent
with `vdf_params.m`'s own flag: "UNVALIDATED in MATLAB — run the IC/50-cell gate before trusting it."

**How to apply:** to check MATLAB, add `cbf_ok`/`s_star` capture in `run_simulation.m`'s output struct
(one-line addition alongside the existing `p_1`/`p_2`/`S_1`/`zeta_1` logs) and re-run a gate — not done
yet (MATLAB sims are user-run, not Claude-run). Don't cite an absence of MATLAB slack evidence as "MATLAB
doesn't need it" — the signal was simply never logged, not checked-and-negative.
