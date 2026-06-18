---
name: project_combined_barrier_matlab
description: "Combined/blended-barrier sliding surface (control_formulation.tex) ported to MATLAB single-run as gated COMBINED_BARRIER mode; s_ddot-drop fix recovers IC5"
metadata:
  node_type: memory
  type: project
  originSessionId: 5ef5a2d7-329a-4539-b101-9f7e6204c84a
---

**Combined-barrier (blended sliding surface) ported to MATLAB single-run (2026-06-18), gated mode.**
visualControl_IBVS_adaptive.m: global COMBINED_BARRIER (default-off = back-mapped SEN funnel). Replaces
the SEN back-map with the position barrier zeta_r entering the sliding surface DIRECTLY:
- lateral sigma_k = zeta_h_k + chi_r*zeta_r_k (PD); descent sigma_3 = zeta_h_3 + chi_z*int(zeta_h_3) (PI, =Omega_z, unchanged).
- zeta_r on r_bar_e = V_s_e/phi_max_xy, phi_max_xy=[res(1);res(2)]/2f (NOT the SEN swapped p_10; matches .tex R/2f + C_nP order).
- h_d = s_dot_meas + transport + h_rd*s, s_dot_meas = smooth4 finite-diff of centroid (NOT V_ds_d, NOT a desired rate) [[feedback_hd_uses_measured_sdot]].
- chi_zeta_aug = [chi_r*dzeta_r; chi_z*zeta_h_3] in Theta/u_eq (the chi*zeta_dot_aug term).
Params: K_ctrl.p_r_0=[1.2;1.2], p_r_inf=[0.4;0.4], xi_r=0.10, chi_r=[1;1], chi_z=Omega_z=0.025.
Test hooks (gated, default-off): CHI_R_OVERRIDE, PR0_OVERRIDE, P20_XY_OVERRIDE, P20_Z_OVERRIDE, CB_DROP_SDDOT.

**KEY FIX — s_ddot-drop (CB_DROP_SDDOT), derivation-driven:** c_h = c̃_h − ḣ_d, and ḣ_d = d/dt(h_d)
of the measured-ṡ h_d CONTAINS s_ddot (centroid acceleration; does NOT cancel — c̃_h has no s_ddot).
s_ddot is mathematically correct BUT 1/z-inflated at terminal → over-aggressive u_eq → IC5 terminal
divergence (lateral re-opens below ~1.5m, fov-fail). FIX: drop s_ddot from c_h (compute ḣ_d from
transport+descent only, h_d_noS = rot+h_d_ff), let adaptive kappa absorb it as d_h. RESULT (noiseless,
corrected c̃_h c_simple=1, chi_r=1): Static IC5 diverge(resid1.42)→LAND xy0.057 vel0.27 resid0.67;
Circ IC5 diverge→LAND xy0.012 vel0.22 resid0.59; Circ IC2 soft+precise either way. r_bar_e now CONVERGES
+ bounded in p_r on IC5 (answers "control authority for position convergence/boundedness"). Residual:
IC5 vel ~0.22-0.27 just over 0.20 soft threshold (minor terminal-velocity tune).

**c̃_h pairing:** combined-barrier needs the CORRECTED ψ̇_b c̃_h (c_simple=1). The .tex's OWN w×s pairing
(c_simple=0, matching control_formulation.tex L47/L102) DIVERGES even at IC2 (r_bar_e_y→−1.04). So
control_formulation.tex's w×s kinematics (L47 ṡ, L102 h_d) is STALE → should be updated to corrected
ψ̇_b throughout (matching manuscript.tex). [[project_kinematics_correction_2026_06_11]]

**Audit (vs control_formulation.tex): implementation STRUCTURALLY CORRECT** — h_d, zeta_r, sigma,
chi*zeta_dot_aug, u_sw/u_eq/kappa all match; phi_max_xy correct; V_s_d=0; c=c̃_h−ḣ_d correct (ḣ_d via
total finite-diff captures all terms incl s_ddot, avoids the algebraic loop an analytical substitution
would create). h_e is NOT exactly (h_z−h_rd)s — retains the velocity residual (ṡ_kinematic−ṡ_measured)
which is the lateral damping (user-corrected; .tex L104 reduction assumes ṡ_meas=ṡ_kinematic).

**FULL 5x5 SUITE (noiseless, CB+s_ddot-drop, corrected c̃_h, chi_r=1): 25/25 LAND, 21/25 soft+precise.**
NO divergence anywhere (s_ddot-drop fixed terminal robustness across the canonical set). The 4 non-SP are
all "Pr" = PRECISE (xy<=0.08) but vel ~0.05 over the 0.20 soft threshold: Static IC5, Sin IC3, Liss IC3,
Circ IC5. Remaining = terminal-velocity tune (descent damping near ground), not a failure.

**MOVING-TARGET terminal phase (s_ddot-drop): RESOLVED.** All 20 moving cells land, r_bar_e bounded
(maxResid 0.40-0.59), descent always soft (~0.08-0.10). The 3 moving Pr misses (Circ IC5, Sin IC3, Liss IC3)
are LATERAL relative velocity (target-chase lag), NOT descent/divergence — because dropping s_ddot also
drops the target-ACCELERATION feedforward.

**FILTERED s_ddot (CB_SDDOT_TAU, LPF on the s_ddot contribution) — user's idea, BETTER than dropping.**
Keeps the smooth target-accel FF (cuts the moving lateral lag) while low-passing the 1/z terminal spike.
tau=0.5 sweep vs drop on the 4 problem cells: Circ IC5 vel 0.224->0.192 (->SP), Sin IC3 lat 0.333->0.050
vel 0.344->0.121 (->SP), Static IC5 ~equal, Liss IC3 ~equal. CAVEAT: only HEAVY filtering (tau>=0.5) is
stable — light tau (0.05-0.2) lets the spike through and REINTRODUCES divergence (Static/Liss IC3 fov-fail).
Implementation: sddot_raw = d/dt(V_h_d − h_d_noS); sddot_filt LPF (a=tau/(tau+dt)); raw_dh_d = d/dt(h_d_noS)
+ sddot_filt. tau=0.5 full-suite validation RUNNING (vs drop's 21/25 SP).

**FILTERED s_ddot tau=0.5 FULL SUITE = 20/25 (WORSE than drop 21/25):** net wash — improved Sin IC3 &
Circ IC5 to SP but regressed Static/Linear/Circ IC3 (SP->Pr) + Liss IC3. The LPF phase-lag adds terminal
lateral velocity to already-converged cells. Filter = right idea for chase-lag but blunt global knob.

**CHASE-LAG LEVER FOUND — p_2inf_xy (optic-flow funnel TERMINAL floor).** Precision thresholds: precise
xy<=0.08m, soft v_rel<=0.20m/s, z_f=0.20m. The Pr cells are PRECISE but fail SOFT (terminal LATERAL
velocity = chase lag). At touchdown s_xy->0 so h_e_xy->velocity-residual, bounded by the flow funnel ->
p_2inf_xy directly sets it. P2INF_XY_OVERRIDE sweep (s_ddot-drop, corrected, chi_r=1), CLEAN MONOTONIC:
lower p_2inf_xy 2.5->0.5 drops terminal vlat across ALL cells. p_2inf=0.5: Static IC5 vel0.267->0.154(SP),
Circ IC5 0.224->0.171(SP), Sin IC3 0.344->0.182(SP), Liss IC3 0.303->0.228(close); control SP cells
(Lin IC5, Circ IC2) IMPROVE not regress; xy stays precise <=0.057. p_2inf_xy is FREE (flow-funnel family,
like p_20 — user: p_20 free, p_s_0 locked). Full-suite p_2inf=0.5 validation RUNNING (vs drop 21/25).

**p_2inf_xy=0.5 FULL SUITE = 24/25 SP, 25/25 land** (vs drop 21/25, filter 20/25). Only Liss IC3 = Pr
(vel0.228). MATCHES the back-mapped form's 24/25 with the cleaner blended-surface architecture. The
combined-barrier winning config = COMBINED_BARRIER + corrected c̃_h (c_simple=1) + chi_r=1 + CB_DROP_SDDOT
+ p_2inf_xy=0.5. (s_ddot-drop fixes terminal divergence; p_2inf_xy tightens terminal velocity tracking =
chase-lag. Both derivation/architecture-driven, NOT the s_ddot filter which was a wash.)

**BREACH CHECK (user directive "ensure s_e_n not breaching p_s"): PASSES.** Combined-barrier r_bar_e
(=s_e_n analog) vs p_r (=p_s analog): worst residency max|r_bar_e/p_r| = 0.674 across all 25 cells with
p_2inf=0.5+s_ddot-drop. NO breach. Holds even at p_2inf_y=0.10 (resid 0.48). Prescribed-performance bound intact.

**Liss IC3 (lone holdout) — vy-dominant chase lag, closed by chi_r (NOT p_2inf).** Per-axis: Liss IC3
vel0.228 = vx-0.02,vy+0.21,vz+0.08 -> the y-axis lateral velocity ([2,-2] IC + Lissajous y-motion). p_2inf
(symmetric or y-only) FLOORS vy at ~0.19 (can't cross 0.20). OTHER-PARAM sweep found the bandwidth levers:
sigma_xy=zeta_h+chi_r*zeta_r, so LOWER chi_r weights velocity-damping zeta_h more -> kills chase-lag vy.
chi_r=0.5: Liss IC3 vel 0.228->0.103 (vy~0!), resid0.47 no breach, ctrl SP. gamma2=0.5: vel->0.154. E=0.5:
->0.170. DIRECTION CRITICAL: higher chi_r/Gamma/gamma2 DIVERGE; lower chi_r / faster gamma2 / smaller E help.
chi_r=0.5 full-suite validation RUNNING (target clean 25/25 + no breach).

**CLEAN 25/25 + 75/75 NOISY (chi_r=0.65 thread).** chi_r=0.5 suite was 24/25 (Circ IC4 lost PRECISION:
lower chi_r de-weights zeta_r; offset is INTERIOR resid0.26, p_r_inf CAN'T bridge). gamma2=0.5 suite 20/25
(wrecks Circular). chi_r=0.65 = the thread (Liss IC3 vy / Circ IC4 vx, DIFFERENT axes): 25/25 SP, no breach
(0.658), both margin cells just under 0.20 (0.196/0.192). NOISY 3-seed x 5x5: 75/75 SP, residency 0.665.

**PROOF RECONCILE + PROOF-GUIDED BETTER MARGIN (pull 74ed0ee COMBINED_SURFACE_PROOF_ADDENDUM, Ubuntu).**
Standing Condition 1: funnel guarantee needs p_r_inf>=1 (FoV-consistent); precision via the manifold
|zeta_r|~|zeta_h|/chi_r (raise chi_r / tighten p_h), NOT sub-FoV p_r. p_r_inf=1.0 -> STILL 25/25, residency
0.615 AND widened margins (Liss IC3 0.196->0.136, Circ IC4 0.192->0.164). Per the manifold, chi_r=0.85 at
p_r_inf=1.0 BEATS 0.65 on BOTH (Liss IC3 0.108, Circ IC4 0.151/xy0.009). The proof turned the thin empirical
thread into a WIDE margin (p_2inf tighter barely helps once p_r_inf=1).

**FINAL WINNING CONFIG:** COMBINED_BARRIER + corrected c̃_h (c_simple=1) + CB_DROP_SDDOT + p_2inf_xy=0.5
+ p_r_inf=1.0 (Standing Cond 1, proof-consistent) + chi_r~0.85. 25/25 SP, 75/75 noisy, NO breach. Beats
back-mapped 24/25. The Ubuntu convergence [[feedback_combined_surface_divergence]] deferred the MATLAB impl
to THIS Windows chat -- done + validated. Test record: MATLAB/test_data combined_barrier_log CB1-CB18.

**BAKED FINAL (2026-06-18):** COMBINED_BARRIER + corrected c̃_h + s̈-drop (default-ON in combined mode) +
p_2inf_xy=0.5 + chi_r=0.85 + p_r_inf=1.0 are now the combined-barrier DEFAULTS (gated; back-mapped default
unchanged). chi_r=1.0 BREACHES (resid 1.282) -> 0.85 optimal. Gated test knobs added: CB_DROP_SDDOT,
CB_SDDOT_TAU (s̈ LPF), CB_SDOT_FILT (Savitzky-Golay s_dot/s̈ filter), P2INF_XY/Z, CHI_R/PR0/PRINF/GAMMA2_XY.

**NOISE (12-seed 5x5x300) + s̈ EXHAUSTION (definitive, do NOT reopen):**
- drop config 12-seed = 205/300 SP (68%), worstResid 5.3 -- LOOKS fragile but isn't: vs BACK-MAPPED on the
  4 hard cells x6 seeds, combined-barrier 16/24 BEATS back-mapped 13/24 with FAR less breach (2.83 vs 19.29,
  the SEN G_s^-1 starvation is worse under noise). The 68% is the hard-cell (Sin/Liss IC3, IC5) noise level,
  SHARED by both architectures -- NOT a combined-barrier regression. Savitzky-Golay s_dot filter helped only
  marginally (16/24 unchanged) -> s_dot is NOT the dominant noise path, the measured FLOW h is.
- KEEPING s̈ is EXHAUSTIVELY irreconcilable: full/hard-cap(DH_D_CAP)/tau-LPF(best tau0.8=5/6)/SG-clean(diverges)/
  symmetric-tuned(23/25)/per-axis-tuned(22/25) ALL worse than drop. ROOT: Circ IC4 needs chi_r_x>=1.1 for
  PRECISION (s̈ over-drives it imprecise) but chi_r_x>=1.1 destabilizes Static/Liss IC3 (X-tilt cross-couples
  to Y -> r_bar_e_y breaches; NO p_2inf/E/chi_r_y rectifies). Irreconcilable chi_r_x conflict; s̈ (1/z-inflated
  centroid accel) IS the root. DROP it (kappa absorbs as d_h) = the clean 25/25. Per-axis method PROVED this.

PENDING: control_formulation.tex w×s->ψ̇_b update (its kinematics is STALE). User exploring s̈-kept at
REDUCED chi_r_x (queued). Backup: Obsolete/Multi_init_cond/MATLAB/visualControl_IBVS_adaptive_v3.m.
See [[project_chtilde_correction_option_b]], [[feedback_combined_surface_divergence]], [[feedback_hd_uses_measured_sdot]].
