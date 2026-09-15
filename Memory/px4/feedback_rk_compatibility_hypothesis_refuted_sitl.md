---
name: feedback_rk_compatibility_hypothesis_refuted_sitl
description: "NEGATIVE result: computed thm:translational's tight compatibility bound R_k(t)=C_k(t)/|eta_nu_z(t)| directly on PX4 SITL GT-FB logs (test_data/XirXi2_RoverGTFB/) to test whether it explains why Xi_r doesn't port MATLAB->PX4. It does NOT. R_k(t)>1 holds with 1.5-5.6x margin in 22/24 reps (same structural pattern as MATLAB: conservative C_k(t)<1 ~100% of flight, tight R_k(t)>1 throughout); the one dip (moving_base_IC1 rep0, R_x->0.746) is a touchdown-window transient, not a precursor to any breach (no target_lost occurred in any of the 24 runs). The candidate arm (XIR=0.20/XI2=0.20) has LARGER R_k margins than base (3.5-5.6x vs 1.5-2.3x) yet performs WORSE empirically (IC1 detonation at R_x=1.18, still >1) -- margin does not track outcome. The theorem's boundedness condition is not the mechanism behind Xi_r's MATLAB->PX4 non-portability."
metadata: 
  node_type: memory
  type: feedback
  originSessionId: f78fde4c-2847-4689-96f6-0693b2bc0c15
  modified: 2026-09-11T20:58:54.011Z
---

**thm:translational's compatibility-condition margin R_k(t) does NOT explain the Xi_r port failure — tested directly on SITL logs, not inferred (2026-09-12).**

**The hypothesis** (user-proposed, from a MATLAB-side proof audit): Theorem `thm:translational`'s
stated condition `C_k(t) := rho_nu,k/(rho_nu,z*s_bar_k) > 1` is violated 65-86% of the MATLAB
flight (all 5 target-motion cases, IC2) — but that's a conservative upper bound (`|e_nu,z|` replaced
by its worst-case envelope `rho_nu,z`). The tighter, solution-dependent bound
`R_k(t) = C_k(t)/|eta_nu_z(t)|` (`eta_nu_z = e_nu,z/rho_nu,z`) holds with 2.9-4.4x margin in
MATLAB. Hypothesis: PX4's real dynamics (lag, noise, delayed thrust) inflate the realized
`|e_nu,z(t)|`, shrinking/breaking this margin on the moving-target case — a concrete mechanism for
why [[feedback_xir_xi2_handoff_refuted_sitl]] found XIR=0.10 (MATLAB's value) risky on PX4 moving.

**The test.** Computed `R_k(t)` directly from `test_data/XirXi2_RoverGTFB/` GT-FB logs (24 reps, no
rerun). Confirmed field map: `p_r(t)`=rho_p [N,2], `p(t)`=rho_nu [N,3] (z=idx2), `h(t)-h_d(t)`=e_nu
(`controller.py:2486` confirms `h_e = h - h_d`), `p_10`=phi_max, `CP['Des Img Feature Param'][:2]==[0,0]`
confirms `s_d,x=s_d,y=0` EXACTLY (not approximated).

**Result: hypothesis refuted.**
- `min C_k(t)` on PX4 reproduces MATLAB qualitatively: 0.127-0.176, fracC<1 ~100% on base arm —
  the conservative bound IS violated almost the whole flight, same as MATLAB.
- **`min R_k(t)` stays >1 in 22/24 reps**, margin 1.5-5.6x — the tight bound holds in PX4 too, same
  structural story as MATLAB. Moving-target base-arm (the case the hypothesis targeted): minR
  1.52-1.72 (x) / 2.02-2.30 (y), comfortable, 3/3 precise.
- The ONE dip (`moving_base_IC1` rep0, R_x->0.746 for ~1% of trajectory) occurs at t=45.37-45.65s of
  a 10s run — the LAST 0.3s, at touchdown (`p_z` pinned at floor, `h_e_z` growing more negative =
  terminal braking). `s_e_n` had already PEAKED and was declining one sample earlier — not a
  growing-error precursor. No FoV-breach/target_lost occurred in ANY of the 24 runs (GT-FB never
  actually loses the target), so there is no breach timestamp for this dip to precede.
- **Candidate arm (XIR=0.20/XI2=0.20) has LARGER R_k margins than base** (3.5-5.6x vs 1.5-2.3x —
  tighter funnel structurally strengthens the bound) **yet performs WORSE empirically**: moving IC2
  2/4 vs base 3/3 precise; moving IC1 candidate rep0 DETONATED (kappa_xy 0.5->30, a_u_xy 1.3e5) while
  its own R_x sat at 1.18, comfortably >1, throughout the blowup. Margin does not track outcome.

**Conclusion.** `R_k(t)>1` is not the mechanism behind Xi_r's non-portability. It holds fine on both
platforms in the moving-target regime that was supposed to break it, and doesn't discriminate the
arm that actually fails (cand) from the one that doesn't (base). The Xi_r/XI2 non-portability is
real (independently confirmed — cand detonates IC1) but its cause is elsewhere, most likely the
already-documented kappa-leakage/`P_XY`-over-leaky chain
([[project_ic1_kappa_leakage_drift_20260721]]), not an optic-flow/position-funnel boundedness
violation. Reinforces [[feedback_xir_xi2_handoff_refuted_sitl]] and the broader
[[feedback_matlab_gains_not_portable]] pattern: MATLAB mechanisms don't automatically explain PX4
port failures either — test them directly on logs, don't infer from a plausible-sounding proof gap.

**How to apply:** don't re-propose this compatibility-condition margin as the Xi_r-portability
explanation without new evidence. If the IC1-detonation mechanism needs isolating next, look at
kappa_xy/a_u_xy growth directly (already flagged as the leakage-drift chain), not at R_k(t).
