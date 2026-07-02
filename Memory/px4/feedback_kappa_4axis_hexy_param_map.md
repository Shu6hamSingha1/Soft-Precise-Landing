---
name: feedback_kappa_4axis_hexy_param_map
description: "⭐ Kappa is NOT adaptive on the LATERAL axes: N_xy=0.02 -> time constant tau=1/(N*P)=33s >> 7s descent, so kappa_xy is FROZEN at its initial ~0.12 (z: N=0.1 tau=2s adapts; yaw: n_a=1.0 tau=0.5s adapts). + the h_e_xy parameter map (funnel=PERFORMANCE bounds it, SMC=CONVERGENCE drives it). + the velocity-damping lever: flow funnel p2_xy is too loose (9-35) -> ζ_h only 7% of sigma_xy -> velocity UNDAMPED; tighten flow funnel to engage ζ_h + raise N_xy (safe now on clean reduced-solve h_xy) to wake kappa_xy."
metadata:
  node_type: memory
  type: feedback
  originSessionId: 31fd53ca-48b0-48f4-81a8-2e081955028f
---

> ⛔ **2026-06-26 REFINED ([[feedback_kappa0_unfreezes_lateral]]):** the "raise **N_xy** to wake frozen κ_xy" framing here is superseded — the lever is **kappa_0_xy** (0.125→0.5), the BOOTSTRAP gain, which PERSISTS through the descent via the slow τ_xy=6.7s (N_xy alone regresses: woken κ amplifies the terminal). κ_xy IS gain-fixable → holds ~0.40 → s_e_n converges → GT-FB IC4 2/2 SP, IC1 2/2 SP (n=2). The "FROZEN κ_xy = no fix" implication is wrong; κ_0 is the fix.

**KAPPA HEALTH across all 4 axes + the h_e_xy convergence/boundedness parameter map (2026-06-25, user-led).**

**kappa is FROZEN on the lateral, adaptive on z/yaw.** The kappa-ODE convergence time constant is
**tau = 1/(N*P)**: lateral N_xy=0.02,P=1.5 -> **tau=33s >> 7s descent -> kappa_xy never moves from its initial
~0.12** (a de-facto FIXED gain; switching term ~0; the sliding mode is held by REACHING alone, no adaptive
robustness). z: N=0.1,P=5 -> tau=2s (adapts). yaw: n_a=1.0,p_a=2.0 -> tau=0.5s (adapts, decays from kappa_a_0=2).
At the terminal kappa_eq=theta*G*|sigma|/P blows up 59-49708x on ALL axes — but that is a BARRIER ARTIFACT
(theta/G explode as the funnel saturates), a runaway target kappa CORRECTLY does not chase (chasing it commands
the undeliverable authority, [[feedback_terminal_smc_actuator_wall]]). N_xy was kept tiny DELIBERATELY to avoid
NOISE-PUMPING (noisy sigma_xy ratchets kappa up); now SAFE to raise because the reduced-solve cleans h_xy AND
sigma_xy is position-dominated (clean centroid).

**FUNNEL = performance, SMC = convergence (the key distinction — user correction).** The funnel ζ=barrier(e/p)
with p(t) only BOUNDS e within +-p(t) (prescribed transient + steady-state precision) AS LONG AS ζ is finite —
it shapes the envelope, it does NOT drive convergence. The SMC (reaching -Gamma*sigma + adaptive switch) drives
sigma->0 within E = CONVERGENCE. So tightening the funnel will NOT "ensure convergence" — it tightens performance
and ENROLLS the flow into the SMC objective; the SMC then converges it (if it reaches sigma->0; it does NOT at
the terminal). h_e_xy CONVERGES during the descent (mean 0.11, ratio 0.05) only INCIDENTALLY — barely controlled.

**WHY h_e_xy is undamped: the flow funnel is too loose.** sigma_xy = ζ_h + chi_r*ζ_r. p2_xy is 9-35 during the
descent (P2INF_xy=1.5 never reached, XI2_xy=0.6 too slow) -> ζ_h~0.017 = **only 7% of sigma_xy** -> the surface
is 93% POSITION; the velocity-damping ζ_h is meant to provide is DORMANT -> v_lat not arrested -> h_xy=v_lat/Z
explodes at the deck. On the surface ζ_h=-chi_r*ζ_r, so h_e_xy->0 only as the position->0 (coupled via chi_r).

**The h_e_xy parameter map** (values @ 2026-06-25):
- BOUNDEDNESS (funnel): P2_0(~35 init) / **P2INF_xy=1.5** (steady-state precision) / **XI2_xy=0.6** (contraction
  = prescribed transient). FINDING: too loose -> ζ_h inert.
- SURFACE: **chi_r=0.5** (PD balance; LOWER -> more flow/velocity damping, less overshoot) / Omega_xy=0.1
  (integral on barrier) / p_r (PR0=1.2,PRINF=1.0, position funnel).
- CONVERGENCE (SMC): **Gamma_xy=0.44/0.5** (reaching speed) / **E_xy=1.0** (UUB bound; smaller=tighter+chatter)
  / kappa (kappa_0=0.125,max=1e6) / **N_xy=0.02** (adapt rate, FROZEN) / P_xy=1.5 (leak).
- FF: c-term cancels known flow kinematics; h_d = desired flow (rotation/descent in combined mode).

**The velocity-damping lever (the fix that ties it together).** Tighten the flow funnel (P2INF_xy/XI2_xy) to
make ζ_h a meaningful share of sigma_xy -> enrolls v_lat into the SMC objective -> SMC damps it -> v_lat->0 early
-> disturbance stays under the 17 m/s^2 actuator ceiling -> no terminal breach. AND raise N_xy (now safe) to wake
kappa_xy for the robustness. PREREQUISITE: clean h_xy ([[feedback_lateral_flow_reduced_solve]] + recal). h_rd
CONSTANT. Sequence: recal h_xy -> raise N_xy -> tighten flow funnel. Continues
[[feedback_sp_task2_terminal_limit_cycle]], [[feedback_terminal_smc_actuator_wall]].
