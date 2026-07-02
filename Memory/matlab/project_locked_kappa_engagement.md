---
name: project_locked_kappa_engagement
description: "LOCKED config baked into vdf_params (kappa-engaged retune 2026-06-26..07-01): kappa0 .05 / N .10 / Pleak [.5;.5;1.5] / E .5 / Xi_r 0.3 / p_rinf 0.85 / theta_per_axis — 25/25 gate + 7x-stress margin + demonstrable kappa adaptation (z RMS .057->.318 over stress ladder)"
metadata:
  type: project
---

**LOCKED config** (baked in `MATLAB/VDF_ASMC/vdf_params.m`, backup `Obsolete/vdf_params_pre_LOCKED.m`):
`kappa0=[.05;.05;.05]`, `N=diag(.10,.10,.10)`, `Pleak=diag(.5,.5,1.5)`, `E=diag(.5,.5,.5)`,
`Xi_r=diag(.3,.3)`, `p_rinf=[.85;.85]`, `theta_per_axis=true`. Motivation: nominal funnels/kappa
were idle -> no margin for real Gazebo disturbance. kappa-ODE equilibrium k*=theta*G*|sigma|/P
(N cancels at equilibrium; N sets adaptation speed, tau=1/(N*P)).

**Validation** (harnesses `Multi_init_cond/cb_*.m`):
- Full gate 25/25 SP (5 traj x 5 IC realistic) + 25/25 noiseless + speed sweep 40/40 (+-40%)
  + multi-init 50/50; comparison regenerated (Proposed SP all cells).
- Stress margin: 7x disturbance 5/5 on stress cells vs baked-old 3/5 (cb_kappa_reject).
- kappa ADAPTS (not leakage decay): escalation {0,1,3,5,7}x -> kappa_z RMS .057/.073/.137/.220/.318
  (5.6x monotone), peak .063->.457 (7.2x); kappa_x peak 2.7x, kappa_y 1.7x. Data
  `MATLAB/Datasets/MultiInit/kappa_adapt.mat` (krms/kpk mean+std over 10 cells), gen
  `cb_kappa_adapt_data.m`. Known-step: Fx=3N on [2,6]s -> kappa_x +43% while on, leak-decay after
  (KNOWN_DIST hook in init_robustness/run_simulation, default-off; cb_kappa_validate).
- E gates DELIVERY not adaptation (wide E_x=1.0 blocked lateral switching); E_xy<0.5 pumps
  terminal velocity — 0.5 is the floor.
- Funnel: Xi_r .10->.3 + p_rinf 1.0->.85 gives ~15% precision (xy .013->.011) and holds 25/25;
  p_h CANNOT be tightened (funnel overtakes h_e convergence); p_rinf floor ~0.85
  (lower -> engR->1, cb_pr_reduce). NOTE p_rinf=0.85<1 violates proof Standing Condition 1
  (needs manuscript framing — see [[project_prescribed_rate_hd]] status).
- Terminal-balloon mechanism (why old config detonated): G_2 breach + theta inflation
  (theta ~ |V_h|^2, V_h=v_rel/z) -> switching term theta*sat*kappa = 94% of a_u spike;
  Gamma cannot substitute (reaching a_u 47.3 @Gamma=2 vs 10.4 baked via G_2^{-1} amplification).

An adaptation FIGURE (kappa vs stress) was added then REVERTED per user — plasmc_adaptive_gain.pdf
keeps its original 4-panel form (x/y/z vs (Y d_bar)_k + yaw panel); the escalation data stands.

**PX4 cross-port A/B (2026-07-02, cb_px4port.m):** the PX4 GT-FB terminal-approach bakes were
tested on the MATLAB gate and REJECTED — Gamma_xy 0.25: 23/25, worstXY 2.63 (reaching-starved in
the ENGAGED-funnel design; PX4's 0.25 pairs with its wide-funnel PR0=10/S_r~0.05 regime);
Xi_h [0.7,0.7,1.0]: 22/25 + 7x stress 5/5->1/5 (the p_h-overtake failure via contraction rate);
both: 19/25 + 0/5. Already-converged (no action): N=0.10, P2INF_xy=1.0, cbf2-only, no
backstepping h_d. Intentional divergences stand: chi_r 1.5(PX4)/2.0(ML), E_xy 1.0/0.5,
kappa0 big-start/adaptive, p_r0 10/1.2-FoV-locked, Pleak_xy 1.5/0.5. PX4 terminal-commit /
loom-commit / s_e_n-ramp machinery = perception-gated terminal disarm, BANNED in MATLAB (user).
Do NOT re-port without a regime change.

**Status:** COMMITTED+pushed 016855e (2026-07-02) together with the
[[project_yaw_observability_campaign]] + [[project_prescribed_rate_hd]] changes. Datasets under
MATLAB/Datasets/{MultiInit,Comparison} were regenerated at LOCKED but are STALE vs the later
2pi-orientation + prescribed-h_d code changes — regenerate before manuscript number refresh.
