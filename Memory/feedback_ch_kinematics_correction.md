---
name: feedback_ch_kinematics_correction
description: c_h kinematic-coupling term was corrected in the manuscript (2026-06-11 §II review) but NOT yet in code; C_SIMPLE implements the fix
metadata: 
  node_type: memory
  type: feedback
  originSessionId: ce5926b8-f1ea-4718-84c8-9a29e27ac4c6
---

The optic-flow-error feedforward `c_h` (the known kinematic coupling in `ḣ_e`)
was **corrected in the manuscript during the 2026-06-11 §II kinematics review**,
independent of CBF/SEN work. The implementations (MATLAB + PX4) still carry the OLD form.

- **Old (WRONG, still in code):** `c̃_h = ẇ×s + w×(w×s) + 2w×h − [(h+w×s)·ê3]h`
  — Coriolis/centripetal `s̈`-style terms built from the NOISY optic-flow-recovered
  `w` (`V_w`) and its derivative `ẇ` (`V_dw`, the noisiest signal in the PX4 pipeline).
  It is the camera-frame static-target kinematics (Xie/Fink lineage, where `w` is the
  *camera's* angular velocity) mis-transplanted with `w` silently redefined as the
  *relative* rate `w = ω_t − ψ̇ê₃`.
- **New (CORRECT, in manuscript):** `c_h = −ψ̇_b·(ê3×h) − (h·ê3)·h − ḣ_d`
  — clean IMU yaw-rate frame rotation + loom only. No `ẇ`, no `w`, no `s` cross-products.

**Why:** Falsified two independent ways. (1) Analytically — a 4-line transport-theorem
derivation from the paper's own definitions (`s=r_t/z_t`, `h=v_rel/z`, `w=ω_t−ψ̇ê₃`)
yields the new form; in the virtual frame 𝒱 the only camera rotation is yaw `ψ̇ê₃`, so
the transport term collapses to `ψ̇_b ê₃×h`. Target rotation about its own centroid can't
move the centroid; deck roll/pitch unequal-depth cancels to first order for the symmetric
corner pattern. (2) Numerically vs the locked multi-init `.mat` GT — old-form residual was
2.5–4× the signal on deck-motion cases and grew near touchdown; transport form sits at the
~2% noise floor and stays flat. `d_h` also got simpler (spurious `ω_b×v_b/z` dropped →
removes UAV absolute velocity from the model, strengthens own-state-free claim).
I re-derived this from scratch 2026-06-15 and got the identical manuscript form.

**How to apply:** MATLAB fix is implemented as `C_SIMPLE` (default-OFF) in
`visualControl_IBVS_adaptive.m` (≈line 574): `c = -psi_dot_b*cross(e3,V_h) -
dot(V_h,e3)*V_h - V_dh_d` with `psi_dot_b` = ZYX yaw Euler rate from `B_w_c`. Tested via
`csimple_ic5.m` (12 matched seeds, default vs C_SIMPLE).

**EMPIRICAL RESULT 2026-06-15 — C_SIMPLE REJECTED as an IC5 fix; PX4 port CANCELLED.**
IC5 [2,2,-3] noisy, 12 matched seeds: default c = SP 9/12, fail seeds [4 6];
C_SIMPLE = SP **2/12**, fail seeds **[4 6]** (unchanged). Two findings:
(1) **c-term is NOT the IC5 trigger** — seeds 4 & 6 fail IDENTICALLY under both forms (seed4
FoV t≈0.4 startup-tilt; seed6 FoV t≈5.4 CBF-strip, xy 78→98 m). Removing the noisy
`V_w`/`V_dw` channel changes nothing → the runaways are CoG/parametric-driven
([[feedback_ic5_cbf_strip_mechanism]]), my noise-channel hypothesis was WRONG.
(2) **C_SIMPLE regresses nominal SP 9→2** — previously-clean seeds still land but soft/precise-
fail (vel 0.15→0.4-1.3). The full-`w` terms do REAL feedforward work in the closed loop; the
sim's plant ḣ effectively contains them. CAVEAT: only `c` was changed, not `V_h_d` (still
`+w×s`), so the `c_h=c̃_h−ḣ_d` pair is convention-mixed — part of the regression may be that
inconsistency, not the manuscript form itself. Doesn't change the verdict: finding (1) alone
rejects the port (4/6 aren't c-term-driven). `C_SIMPLE` hook kept default-off as a diagnostic.

So the manuscript c_h correction stands as a PAPER fix (analytically + GT-validated), but is a
DEAD-END for the IC5 closed-loop problem and a nominal regression as a partial code port. A
fair paper-alignment test would need the CONSISTENT change (c AND h_d to the psi_dot_b form) —
but that's paper-code alignment, not an IC5 fix. See [[feedback_lateral_kappa_runaway]],
[[project_plasmc_port_status]], [[feedback_reject_on_single_failure]].

**PX4 CONSISTENT-FORM PORT — DONE + A/B'd (2026-06-16, commit 75e8765, `PLASMC_CH_CLEAN` default-off).**
The fair test the caveat called for: changed BOTH `c` AND `h_d` consistently in PX4 controller.py
(c = `−ψ̇_b·(ê3×h)−(h·ê3)h−ḣ_d`, ψ̇_b = ZYX yaw-Euler rate; h_d drops `+w×s`; ψ̇_b sign SITL-unverified,
flip `_r` if it regresses). **IC2 n=5: the consistent form does NOT regress** (xy mean 8.1→5.5, TL
5/5 vs 4/5 noise) → **confirms the MATLAB SP 9→2 regression was the convention-mixing, not the
manuscript form.** Also SMOOTHER (he_std 0.61→0.54, w_u sat 22%→7%). BUT NOT a lateral-wall fix (a_u
32%→41% inward only; baseline a_u already 32% inward → rotation-FF domination of h_d was NOT the
binding limit). Verdict reversed: a paper-aligned, non-regressing, smoother default-off KEEPER
(candidate to default-on after an IC2-5 gate), NOT a wall-breaker. See [[feedback_inner_loop_velocity_thread]].
