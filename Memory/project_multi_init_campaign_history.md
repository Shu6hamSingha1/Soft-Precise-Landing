---
name: multi-init-campaign-history
description: "consolidated multi-init SEN-funnel tuning campaign history: 50/50 lock, gain combos, per-trajectory ceilings, channel fixes"
type: project
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---

Consolidated history of the MATLAB multi-init (5 trajectories × 5 ICs × {noiseless, realistic}) PLASMC tuning campaign, 2026-04-07 → 2026-04-19. Merges eleven point-in-time memories. Locks, ceilings, and per-channel fixes below are verbatim. (PLASMC/DF-ASMC = same framework, earlier naming epoch; current name VDF-ASMC — see project_naming_decisions.md.) Empirical parameter impact lives in reference_plasmc_parameters.md; gain-source bookkeeping in project_bestparam_location.md.

## Final lock — Combo D, 50/50 precise+soft @ 8 cm (2026-04-19)
Final locked config (2026-04-19): 5 trajectories × 5 ICs × {noiseless, realistic} = 50 runs, **all precise (xy ≤ 0.08 m) and all soft (v_rel ≤ 0.20 m/s)**. Canonical IC table `[0,0,-5; 2,2,-5; 2,-2,-5; 2,2,-7; 2,2,-3]`, seed = 1000+k.

Locked deltas this session (three-file sync):
- `K.zp = diag([9.0, 9.0])` (was 6.0) — **Combo D** outer-PID P-term ×1.5 for lateral precision
- `K.zd = diag([1.4375, 1.4375])` (was 1.15) — **Combo D** D-pair ×1.25 matching the zp bump; recovers the soft margin the P-bump would otherwise cost
- Precision threshold `0.10 → 0.08 m` across `run_simulation.m`, `visualControl_IBVS_adaptive.m`, `Comparison/visualControl_comparison.m`

So **Combo D = zp×1.5, zd×1.25 → zp = diag[9.0, 9.0], zd = diag[1.4375, 1.4375]**.

**Caveat on the 8 cm threshold:** an earlier index hook flagged "tex still cites 0.10 m"; the final-lock record states precision threshold is **0.08 m everywhere — tex sync completed 2026-04-23** across `manuscript.tex`, `results.tex`, `supplemental.tex` ("order of magnitude below" softened to "well inside"; 2.1 cm vs 8 cm is ~3.8×, not order-of-magnitude).

## Combo sweep — why D, not A/B/C/E
After FW=11 lock, full deep-sweep re-run (33 params × 4 mults × 5 trajs × 5 ICs = 3325 sims). Baseline aggT=18.392 s, aggMaxXY=0.0747 m.
- Combo A (FW=11 + p_2inf(3)=0.75 + l_fov=0.125) — cross-cluster; z-tightening acted as a speed knob, not precision
- Combo B (p_2inf(3)=0.75 + Gamma(3)=1.125 + E(3)=0.5) — z-cluster deep; rejected
- Combo C (zp×1.5 + Γ(2)×1.5) — broke soft margin
- **Combo D (zp×1.5 + zd×1.25) — WINNER/LOCKED**: 25/25 land + 25/25 soft, max xy 0.0747 → 0.0561 m (−25%), mean t 18.39 → 17.85 s (−3%)
- Combo E (D + Γ(2)×1.5) — re-broke soft margin; disturbs P+D balance

**P and D must move together, never tune alone.** Outer-PID P (zp) and D (zd) pair in lockstep. Bumping zp alone sharpens position at the cost of v_rel overshoot; zd×1.25 exactly cancels that overshoot. Combos that mix P/D with a cross-cluster knob (Γ) destabilize the paired balance. Cross-cluster Γ(2) at ×1.5 conflicts with Combo D — never re-add.

Superseded prior sweep (2026-04-09): locked Gamma(1,1)=0.1875 + kappa_0=[0.125;0.125;0.25] (−4.2% land time, −8.6% max xy); Gamma later retuned, kappa_0 survived.

Other locked gains (not touched at Combo-D lock): `Gamma = diag[0.4375, 0.5, 0.75]`, `kappa_0 = [0.125, 0.125, 0.25]`, `p_20 = [25, 25, 4]`, `p_2inf = [2.5, 2.5, 1.5]`, `E = diag[1.0, 1.0, 1.0]`, `h_rd = -0.42`, `Omega = diag[0.05, 0.05, 0.025]`, `kR = diag[1.5, 1.5, 0.5]`, `kOmega = diag[0.3, 0.3, 0.1]`. `gamma_1`/`p_1inf` REMOVED by Approach 2.

## Per-trajectory ceilings
**Circular wz ceiling:** On the Circular trajectory (`MATLAB/Common/traj_Gen.m`, r=0.5), the 5-run multi-init sweep has a hard ceiling at **wz=0.3 rad/s — the 50/50 ceiling; wz=0.4 breaks Run 5**. Above 0.3, Run 5 (worst IC) blows up via `I_a_cd` divergence:
- `wz=0.3` → 5/5 noiseless, 5/5 realistic. **Chosen.**
- `wz=0.4` → 5/5 noiseless, 4/5 realistic (Run 5 I_a_cd=114.6 @ t=9.41 s)
- `wz=0.5` → 4/5 noiseless (I_a_cd=121.4 @ t=5.55 s), 4/5 realistic (I_a_cd=173.9 @ t=8.71 s)
Centripetal feedforward scales as `r·wz²`, so 0.3→0.5 is a 2.78× load increase. Stress the sweep via the harder Lissajous (`w1=-0.8, w2=+0.4`, orthogonal, holds 5/5), not via Circular speed. If Circular must go faster, widen `K.p_20` or shrink `r` before touching `wz`.

**Target-speed stress envelope (deterministic, seed=1000+k, canonical 5-IC set, Combo D, `rho_fov_0=[145;105]`, 8 cm threshold; verified on 2 consecutive runs 2026-04-19):**
- **Linear 1.25×** → first failure 1.50× at IC3 [2,-2,-5], FoV breach (max|v|=120.0 px @ t=1.68 s)
- **Sin 1.50×** → first failure 1.75× at IC4 [2,2,-7], I_a_cd divergence @ t=27.5 s
- **Circ 2.50×** (strongest) → first failure 3.00× at IC3 + IC4 (IC3 FoV breach + IC4 I_a_cd)
- **Liss 1.75×** → first failure 2.00× at IC4 [2,2,-7], I_a_cd divergence @ t=25.7 s

Cite as: **Linear 1.25×, Sin 1.50×, Circ 2.50×, Liss 1.75×**. IC pattern: **IC3 [2,-2,-5] is the FoV-limited IC** (Linear/Circular, worst-tilt geometry); **IC4 [2,2,-7] is the funnel-divergence IC** (long descent → more time for S_2 saturation → I_a_cd blow-up). Linear improved 1.00→1.25× and Circular 2.25→2.50× after matching ICs to the canonical set (old IC5 was [-2,-2,-5]). Harness: `MATLAB/Sweeps/sweep_speed.m` (now passes seed via 6th arg — see reference_run_simulation_seed_api.md — and explicit `cfg_override = struct('NOISE',1,'GE',1,'delay',1)`).

## Funnel-asymptote / tight-funnel principle
Deterministic sweep (seeds 1000+k, speed_mult=0.9, R4/R5 ICs at [±1.5,±1.5,-5]) with tightened prescribed-performance bounds:
- `p_1inf = [0.1; 0.1]` (from 0.2)
- `p_2inf = [1.0; 1.0; 1.5]` (from [1.5;1.5;2.0])

Result: **49/50 land, 49/50 precise, 48/50 soft** (noiseless 25/25/25). Recovered three marginal soft misses (Sin R4 0.206→0.197, Liss R4 0.212→0.145, Liss R5 0.205→0.147). Remaining: Sin R5 realistic (phase-dependent), Circ R5 realistic (structural). Principle: terminal v_rel residual on rotating/oscillatory targets is bounded below by the funnel asymptote; tightening `p_2inf(3)` cuts the vertical touchdown envelope and lateral `p_2inf(1:2)` cuts lateral chase residual — does not interact with the cone clamp the way zp/zd stiffening does. **Prefer tightening prescribed-performance asymptotes (`p_1inf`, `p_2inf`) before outer-loop position gains** when closing marginal terminal-v_rel soft misses (theory-consistent, not an envelope cheat).

## Per-channel fixes
**IC4 z-channel hover-fail → Ω(3,3) 0.006 → 0.025 (RESOLVED 2026-04-18).** After Approach 2 (funnel-margin cone clamp) was committed, realistic-mode IC4 hover-failed on Linear/Circular and hard-landed on Sinusoidal. Fix `Omega(3,3): 0.006 → 0.025` restored full landing; subsequent lateral + descent retune (zd=1.15, E(3,3)=1.0, h_rd=-0.42) pushed to 25/25 realistic + 25/25 noiseless all soft. Root cause: `Omega(3,3)=0.006` was tuned under the OLD visibility-funnel architecture where the barrier PID on `zeta_1` supplied extra z-authority; Approach 2's raw-error PID on `V_s_e_n` doesn't replicate that. Hover-fail signature (vs crash-fail): `izeta_2(3)` pinned at max + `I_a_cd(3) ≈ −g`. Diagnostic: `scripts/diag_ic4_z_channel.py`.

**Noiseless Run-5 dither-stabilized limit cycle → raise E_z (E(3,3)).** Sinusoidal/Circular Run 5 (IC=[-2,-2,-5]) fail in noiseless mode (NOISE=GE=delay=0) at t≈4.35 s / 4.41 s while realistic lands 5/5. Verified (step-by-step .mat + FFT, 2026-04-10) as a **dither-stabilized limit cycle, NOT a finite-difference artifact**: noiseless tracks better through t≈3 s then bifurcates; FFT shows a coherent 0.5–1 Hz oscillation with ~5× the energy of the broadband realistic spectrum. Mechanism: funnel `p_2(t)` shrinks exponentially so loop gain `G_2 = (e^ζ+1)²/(2·e^ζ·p_2)` grows monotonically; three hard nonlinearities (cone clamp on `I_a_cd`, `sat()` in `u_sw`, `S_2` hard clamp at ±0.95) sustain the cycle until `zeta_2` saturates, `kappa` runs away, `I_a_cd` explodes. Realistic case avoids it because noise+delay+GE act as broadband dither. Fix (2026-04-14 deck-motion rerun, reappeared on Lissajous noiseless): **K_ctrl.E raised diag([1.0,1.0,0.5]) → diag([1.0,1.0,0.9]) (z only)** in all three gain sources → clean 50/50 multi_Init_Var + 40/40 multi_speed_cond with heave. (Per the final-lock record E(3,3) sits at **1.0**.) Rejected hypotheses: finite-difference noise on V_h_d (would show from t=0; doesn't); LPF on V_h_d (added phase lag, desynced feedforward `c`, broke realistic 25→~15/25, REVERTED).

**ZOH=3 fix (yaw ASMC retune, checkpoint 8d, 2026-04-07).** After yaw PID→adaptive SMC (kappa_a_Solver.m), attitude PID 3×3→2×2 (roll/pitch only), desired attitude using `-V_s(4)`, constants `zf=0.2` (was 0.1), `h_rd=-0.3` (was -1.0). At ZOH=1 (100 Hz) low-altitude (<1 m) optical-flow noise — image scale `f/(z+zf)` amplifies pixel noise ~6× at 0.7 m vs 5 m — causes barrier violations → crash. **ZOH=3 (30 Hz, matching physical camera) holds features 3 steps, reducing noise; essential, don't revert.** Comparison uses ZOH=3 (IBVS controllers 1,4,5 benefit; PBVS 2,3 unaffected). Lesson from that retune: **wider funnels are counterproductive** — tight p_2inf gives aggressive correction and small |h_e| at crash; the ~0.6–0.7 m noise spike overwhelms ANY funnel width.

**Circular wz=0.8 — W-matrix roll-yaw Euler coupling cascade.** On aggressive Circular (r=0.5, wz=0.8 rad/s = 45.8°/s yaw, IC=[0,0,-5], realistic), failure is a 5-stage positive-feedback loop through the Euler-rate W-matrix: `B_w_cd(3) = -sin(roll)·dE2_cd(2) + cos(roll)·cos(pitch)·u_a`. As roll grows under lateral demand, the pitch-PID term `-sin(roll)·dE2_cd(2)` injects into the body-yaw command, saturates `B_w_cd(3)` at w_max=2.0, inertial yaw goes negative, `V_w` explodes (0.2→3.6 rad/s), `cross(V_w, V_s)` blows up V_h_d → more roll → unrecoverable. Aggressive yaw gains (Omega_a=4, Gamma_a=1) made it WORSE (failed t=3.48 vs 3.90). **Fix is structural, not gain tuning: tighten cone clamp `att_cone 35° → 25°` (primary) + slow descent `h_rd −0.8 → −0.5` (secondary).** The W-matrix coupling is a fundamental Euler-angle limitation that gain tuning alone cannot overcome — limit the operating envelope.

## Per-trajectory 5/5 results (universal baseline, early campaign 2026-04-08)
**Linear 5/5:** the two changes that mattered were `zp: 5 → 4` (kills cone-clamp climb-coupling on Run 5) and `Gamma(3): 0.4 → 0.5` (faster vertical convergence). Three manuscript-grade lessons (captured in reference_plasmc_parameters.md): (1) the cone-clamp climb-coupling chain (aggressive xy demand → cone clamp → vertical thrust starves → climb → spiral) is the dominant nonlinear failure mode — cut it at the source (lower zp), not downstream; (2) `yaw_drift` on a failed run is a divergence symptom, not a yaw-loop tuning target; (3) loosening a funnel terminal bound is not always safer — if the funnel was doing useful suppression work, loosening it transfers load elsewhere.

**Circular 5/5 (early r=5, wz=0.11):** locked at 5/5 with r=5.0, wz=0.11 (chase v=0.55 m/s). Lateral chase velocity `v=r·wz` is the dominant constraint, not yaw rate; original r=10/wz=0.1 (v=1.0) saturated the cone clamp on Run 5's opposing-chase geometry. Circular was **rescaled** rather than over-tuned. Yaw-loop knobs trade off: anything adding yaw discipline beyond baseline either breaks Linear Run 5 (yaw-quiet — tighter discipline chases noise) or regresses Circular Run 5 by over-stiffening. `Omega_a=1.0 → 1.5` is the only strict win across all 5 trajectories (adds integral action that has nothing to integrate on yaw-quiet targets). Locked yaw set: `Omega_a=1.5, n_a=0.05, p_a=2, kappa_a_0=0.1, E_a=2.5`. (Note: this early Circular geometry predates the later traj_Gen Circ r=0.5/wz=0.3 ceiling above.) Always run a Linear Run-5 regression check before celebrating any yaw "fix" — Linear Run 5 is the canary for over-stiffened yaw.
