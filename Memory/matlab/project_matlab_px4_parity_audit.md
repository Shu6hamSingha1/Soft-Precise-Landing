---
name: project_matlab_px4_parity_audit
description: "Full MATLAB-blocks vs PX4-controller.py parity audit (2026-07-03): control law IDENTICAL layer-by-layer; funnel-ref h_d + DHD_SRC=full + 2pi orientation CONVERGED both sides; divergences enumerated+justified (k_r 0.5 vs 0 flag, CBF corner-vs-centroid, inner loop, terminal machinery, gains regime)"
metadata:
  type: project
---

Layer-by-layer audit of `MATLAB/VDF_ASMC/+blocks/*` vs `PX4_Gazebo/src/controller.py`
(combined-barrier mode, both defaults), 2026-07-03.

**IDENTICAL (verified line-by-line):** position funnel (S_r clip ±0.95, ζ_r log-barrier, g_r,
dζ_r=g_r(dr̄_e−S_r·ṗ_r); r̄_e=s_e/φ_max ≡ s_e/p_10); flow funnel (expm envelope, S-margin 0.05);
σ assembly (σ_xy=ζ_h+χ_r·ζ_r, σ_z=ζ_h3+χ_z∫ζ_h3, PX4 names χ_z as Ω_z; χζ̇_aug identical);
c-term CH_CLEAN=1 (−ψ̇_b(ê3×h)−(h·ê3)h−ḣ_d, same clean-IMU ψ̇_b extraction; PX4 adds cosθ≥0.2
guard + PLASMC_CH_PSIDOT_SIGN toggle default +1); Θ=[−c+S·ṗ−G⁻¹χζ̇_aug, I]; per-axis
θ_k=√(v_k²+1) (THETA_PER_AXIS mirrored); control law a_u=−G⁻¹(u_sw+u_eq) (PX4's a_v composition
expands to exactly MATLAB asmc.m); κ-ODE κ̇=θNG|σ|−NPκ via the same RK5; yaw ASMC form
σ_a=e_a+Ω_a∫e_a.

**CONVERGED THIS WEEK (independent/cross-ported):**
- 2π weighted-corner orientation: PX4 origin (_marker_principal_angle) → MATLAB port 07-01. Both ±π.
- Funnel-ref h_d: PX4 baked PLASMC_HD_FUNNEL_REF=1 on 06-29 (_hd_rate = p_10·S_r·ṗ_r − p_10·k_r·ζ_r/g_r);
  MATLAB s_dot_presc = φ_max·S_r·ṗ_r on 07-02 — SAME base form (the user's p_10·S_s·ṗ_s design).
- dh_d source: PX4 PLASMC_DHD_SRC default "full" (added 07-02 Ubuntu-side) = differentiate the FULL
  h_d incl. the rate term — matches MATLAB's folded dh_d (s̈-drop removed both sides in effect).
- N=diag(0.10) adaptation rate; P2INF_xy=1.0; cbf2-only (no cone/cbf1); no backstepping h_d.

**⚑ DECISION FLAG — k_r:** PX4 bakes HD_KR=0.5 (proportional barrier contraction ζ̇_r,d=−k_r·ζ_r
in h_d); MATLAB has k_r≡0 BY USER DESIGN (2026-07-02: the prescribed-Ṡ_r contraction law is the
WRONG approach — reference must not drive convergence). PX4's k_r=0.5 IS that law. Ubuntu-side
decision pending whether to zero it for consistency.

**⚑ PX4-side open flag (theirs):** controller.py comments mark the CH_CLEAN ψ̇_b transport SIGN as
SITL-unverified (PLASMC_CH_PSIDOT_SIGN escape hatch; "GT-FB w_z is +psidot_b vs manuscript
−psidot_b"). MATLAB's sign is validated (25/25 with the same formula). Do not port MATLAB code
here; flag only.

**INTENTIONALLY DIVERGENT (documented + justified):**
- CBF: MATLAB corner-based w/ 15px inset vs PX4 centroid-based full-edge — parity-doc 06-15
  addendum: PX4's choice safer for its multi-marker board; MATLAB IC5 strip mode is corner-CBF-specific.
- Inner loop: MATLAB geometric SO(3) + adaptive CoG FF (gamma_cog, MATLAB-only by decision) vs PX4
  rate-mode (ships body rates; PX4 owns rate→torque). Architectural, reviewer-defense documented.
- Terminal machinery: PX4 terminal-commit s_e_n ramp / a_u caps / lateral taper / soft-breach
  source-fake / κ_max + containment freeze — SITL perception guards; ALL BANNED in MATLAB (user).
- Filters: PX4 CV-KF (VDS_KF q=1, DHD_KF) with smooth4 as the MATLAB-parity fallback; MATLAB smooth4
  finite-diff. DH_D cap 50 (PX4) vs 20 (MATLAB) — different feature scales, both spike-killers.
- Gains: LOCKED (engaged-funnel: p_r0=1.2 FoV, Xi_r=.3, kappa0=.05, E=.5, Γ_xy .4375/.5) vs GT-FB
  terminal-approach (wide-funnel: PR0=10, XIR=.10, kappa0=.5, E_xy=1.0, Γ_xy .25, chi_r 1.5,
  Pleak_xy 1.5). Cross-port A/B BOTH directions rejected transplants (test-record row P1) — each
  config is correct in its regime (MATLAB mild noise + engaged barrier vs SITL GT-FB noise + wide
  funnel). Same-law-different-operating-point, NOT drift.

**VALIDITY:** MATLAB 50/50 multi-init + 40/40 speed + 7x-stress 5/5 + breach A/B at final code;
PX4 GT-FB IC1-5 19/25 (XIR-revert gate) + rover speed envelope ≤1.1 m/s. Both implement the same
published law; each validated in its own regime. CONTROLLER_PARITY.md §1 (old back-mapped-era
table) is stale on specifics but its addenda remain accurate; this file supersedes it for the
combined-barrier era (PX4_Gazebo/docs is read-only from the Windows side — do not edit there).
