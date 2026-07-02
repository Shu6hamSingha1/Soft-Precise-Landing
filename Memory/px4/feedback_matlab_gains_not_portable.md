---
name: feedback_matlab_gains_not_portable
description: "⭐ NEGATIVE result: the MATLAB vdf_params gains do NOT transfer to PX4 — the 38ms LAG is the binding difference. Porting MATLAB's fresh gains (chi_r=2.0 vs PX4 0.5, N=0.02 vs 0.1, P2INF_xy=1.0 vs 0.5, p_2_0_z=4 vs 10) -> GT-FB 1 sub/2 marg/7 fly (IC1 CENTERED flew 18m, a_u_xy=2,000,000). chi_r=2.0 over-reacts to the lag-induced terminal residual; even DEFEATS the N=0.02 kappa safeguard. You cannot out-gain the lag; PX4 chi_r=0.5 is the survivable ceiling. Cure = remove the residual (velocity damping), not tune the terminal gain."
metadata:
  node_type: memory
  type: feedback
  originSessionId: 31fd53ca-48b0-48f4-81a8-2e081955028f
---

> ⛔ **2026-06-26 STALE ANCHOR ([[feedback_kappa0_unfreezes_lateral]]):** the "chi_r=2.0 catastrophic" verdict was the OLD FROZEN-kappa base (s_e_n never converged -> chi_r weighted a large, un-shrinking zeta_r -> infeasible demand -> IC1 18m). On the kappa_0_xy base (s_e_n converges) the chi_r ceiling is UNTESTED and likely MUCH higher: higher chi_r = stronger chatter mask AND more position-rate (chi_r*dzeta_r) damping of the lateral overshoot. Do NOT cite 2.0 as binding; sweep up empirically (0.85 ~ baseline at n=2 so far).

**THE MATLAB GAINS DO NOT TRANSFER TO PX4 — clean negative, the LAG is the boundary (2026-06-25, user-led).**

**The test.** Applied the fresh MATLAB `VDF_ASMC/vdf_params.m` gains to PX4 via env (bundle 20260625-191808, GT-FB). The real divergences (most already auto-align to VDF; chi_r and N are PX4-specific): **chi_r 0.5->2.0**, **N 0.1->0.02**, **P2INF_xy 0.5->1.0**, **p_2_0_z 10->4**. Env: `PLASMC_CHI_R_X/Y=2.0 PLASMC_N_X/Y/Z=0.02 PLASMC_P2INF_X/Y=1.0 PLASMC_P20_Z=4`.

**Result: 1 sub / 2 marg / 7 fly** (vs PX4-gains 175559 4/2/3, E_z=3 145330 7/0/1). Catastrophic. The tell: **IC1 (the CENTERED, easiest IC) flew to 18 m**; a_u_xy hit **2,000,000 m/s^2** (deliverable = 17). chi_r=2.0 is unusable in PX4.

**Mechanism = the lag-induced TERMINAL RESIDUAL that chi_r=2.0 over-reacts to.** The 38ms lag means PX4 reaches the last metre with s_e_n NOT fully nulled (~0.4m residual; benign by itself). As Z->0, s_e_n=lat/Z lifts and zeta_r (position barrier) pins at its clamp. **chi_r=2.0 weights zeta_r x2 -> 94% of sigma_xy at the terminal** -> a_u_xy explodes -> tilt saturates -> launch -> runaway. **MATLAB is lag-free**, so it nulls s_e_n DURING the descent (no terminal residual) and chi_r=2.0 buys clean precision; **PX4 always has the residual**, so chi_r=2.0 over-reacts catastrophically. The descent is FINE in both (s_e_n 0.01-0.25 to alt 1.5m) — the divergence is purely terminal.

**chi_r=2.0 even DEFEATS the N=0.02 kappa safeguard.** Expected N=0.02 to keep kappa frozen (tau=33s); but kappa_x descent std = **2.9, not 0.01** — because chi_r=2.0 makes sigma_xy so enormous that the kappa-ODE growth theta*N*G*|sigma| blows up even at N=0.02. So BOTH the position and adaptive channels detonate.

**CONCLUSION (the rule).** **You cannot out-gain the lag.** MATLAB tunes chi_r UP for precision (no lag to punish aggression); PX4 must tune it DOWN (lag turns aggression into fly-aways). **PX4 chi_r=0.5 is the survivable ceiling** — do NOT raise it toward MATLAB's 2.0. The gains are non-portable in BOTH directions. The cure is NOT a terminal-gain value — it's **eliminating the terminal residual itself** (arrest v_lat/s_e_n before the deck = velocity damping, [[feedback_flow_funnel_zetah_works]]).

**The PX4<->MATLAB gain map (combined-barrier auto-align).** In combined mode the controller AUTO-ALIGNS most gains to vdf_params, each only if not env-overridden (controller.py ~L265-277): Gamma=[0.4375,0.5,0.75], kappa0=[0.125,0.125,0.25], E=[1,1,0.5], XI2=[0.2,0.2,0.2], P2INF=[0.5,0.5,1.5] all = VDF/MATLAB. The PX4-SPECIFIC divergences (NOT auto-aligned): **chi_r=0.5** (vs MATLAB 2.0; lag), **N=[0.1,0.1,0.1]** (vs 0.02; SITL ~2s descent is kappa-rate-limited + N_xy=0.1 baked 826e655), **P2INF_xy=0.5** (vs MATLAB's fresh 1.0; PX4 auto-align is stale), **p_2_0_z=10** (vs 4), **XI2** when tightened for zeta_h. ⚠ Setting a SINGLE-axis env (e.g. PLASMC_XI2_Z) BYPASSES the auto-align for that param -> X/Y revert to the pa-default ("hot") -> always set all 3 axes. MATLAB fresh bakes this session (git pull): chi_r 1.15->2.0, p_hinf_xy 0.5->1.0, yaw Omega_a/Gamma_a 0.5->0.25, per-axis theta. ⚠ MATLAB code is Windows-only.
