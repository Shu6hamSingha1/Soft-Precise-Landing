---
name: feedback_theta_per_axis_decoupling
description: "Per-axis theta (regressor ROW-norm) replaces the shared scalar ||Theta||_F in the optic-flow ASMC — UUB-proven (tight bound, same V), empirically decouples the z over-brake from the lateral zeta_r explosion. BAKED default-ON (PLASMC_THETA_PER_AXIS). n=3 GT-FB: 12/15 sub, 0 stall, z decoupled; the 2/15 fly = un-fixed LATERAL zeta_r root (NOT this change), the next lever."
metadata:
  node_type: memory
  type: feedback
  originSessionId: 31fd53ca-48b0-48f4-81a8-2e081955028f
---

**PER-AXIS THETA — the first PROVEN, non-band-aid terminal fix (2026-06-25).** Implements the lateral-
root finding ([[feedback_terminal_root_lateral_zeta_r]]): the shared scalar switching gain ‖θ‖_F (θ =
regressor matrix [v|I3]) over-bounds EVERY axis by the worst row, so the lateral position-barrier ζ_r
explosion inflates ‖θ‖_F and detonates the z (and cross-lateral) switching term + κ-ODE → the
COLLATERAL z over-brake. Fix = per-axis θ_k = ‖row_k(θ)‖ = sqrt(v_k²+1) (diag(θ_⋆) replaces the scalar
in BOTH the switching term and the κ-ODE).

**CODE:** `PX4_Gazebo/src/controller.py`, env `PLASMC_THETA_PER_AXIS` (default 0). 5 edits: flag in
__init__; theta_ctrl=sqrt(vector²+1) at the Theta block (self._theta still logs the scalar ‖·‖_F for
continuity); theta_ctrl into the kappaSolver call + the a_v switching term; _kappaSolver parenthesized
`theta*(N G |σ|)` so scalar broadcasts identically / a 3-vec multiplies element-wise. θ_k≡‖θ‖_F recovers
the published law exactly (strict generalization). Compiles; flag prints "[PLASMC] PLASMC_THETA_PER_AXIS=1".

**LYAPUNOV PROOF (Soft_Precise_Landing/Drafts/PER_AXIS_THETA_PROOF.md):** UUB PRESERVED — same V, same
φ₁, same bound ϑ. Key: the disturbance already enters σ̇ ROW-WISE, |(θd̄)_k| ≤ θ_k·d̃, so θ_k is the
TIGHT per-axis bound and ‖θ‖_F ≥ θ_k is the conservative special case (worst-row over-bounding = the
coupling pathology). The proof's cross-cancellation survives because θ_k multiplies BOTH the axis-k
switching dissipation AND the axis-k κ-driving term → V̇ collapses line-for-line to the manuscript
inequality. Manuscript edit (when validated): eq (sliding/adaptive law) ‖θ‖→diag(‖e_k^⊤θ‖); Thm 1 proof
insert the per-axis collection; scalar law = conservative corollary.

**EMPIRICAL — BAKED default-ON 2026-06-25 (user call, despite 2/15 fly = lateral root).**
- n=2 (bundle 022346): 9/10 sub, 0 fly, 0 stall. **n=3 CONFIRMATION (bundle 025444, 15 reps): 12/15
  sub-meter, 1 marg, 0 STALLS, 2 FLY** (IC4_rep1 8.0m, IC5_rep3 1.54m). The n=2 "0 fly" was OPTIMISTIC
  — the n=3 stress reps (IC4 from 7m, IC5 from 3m) surfaced the truth: per-axis θ DECOUPLES z + removes
  stalls but does NOT make landing fly-free. BOTH flys = the un-fixed LATERAL zeta_r/s_e_n runaway, NOT
  a θ regression (terminal a_u_z stayed 4.0/3.9 = z decoupled as designed; s_e_n→94/17, a_u_xy→4420/1945).
  Baked anyway = it's the PROVEN z-decoupling/tight-bound contribution; the lateral root is attacked next.
- MECHANISM CONFIRMED: terminal (alt<0.3) peak |a_u_z| stays **3.6-7.8** even when |a_u_xy| explodes to
  **2995** (logged ‖θ‖_F to 2968) — vs shared-θ FLY-AWAYS where the same lateral spike detonated z to
  **92-320** (collateral over-brake). z is structurally decoupled (θ_z driven by the small loom row only).
- CAUSAL CLOSE: the lateral a_u_xy spike STILL occurs (per-axis θ does NOT cure the lateral runaway —
  proof caveat §6), but with z no longer ballooned (no vz-flip), the drone touches down BEFORE the
  terminal lateral spike can throw it → 0 fly-aways.

**Why:** First terminal fix that is PROVEN (UUB) + TIGHT (not conservative) + a genuine control-law
improvement, not a clamp band-aid ([[feedback_kappa_clamp_bandaid]]). The κ caps (z AND xy) now bound a
DECOUPLED, healthy z; the lateral runaway is isolated.

**How to apply.** BAKED default-ON 2026-06-25 (n=3 confirmed; user call despite 2/15 lateral-root fly).
Set `PLASMC_THETA_PER_AXIS=0` for the legacy scalar (parity). Production/perception-on still needs
separate validation. The REMAINING isolated root = the lateral a_u_xy spike (s_e_n 1/Z + ζ_r barrier) —
the ACTIVE next lever (chi_r PD balance / p_r funnel shaping / lateral convergence speed), h_rd CONSTANT.
Continues [[project_gt_feedback_control_tuning]], [[feedback_terminal_root_lateral_zeta_r]].

**WINDOWS TRANSFER (user plan, 2026-06-25): once baked → port to MATLAB + update manuscript on Windows.**
Turnkey spec written: `Soft_Precise_Landing/PER_AXIS_THETA_HANDOFF.md` — (A) MATLAB port = 3 edits in
visualControl_IBVS_adaptive.m (lines 802-819: Theta_perax=sqrt(sum(Theta.^2,2)); u_kappa & u_sw use
theta_ctrl 3-vec) + 1 edit in kappa_Solver.m (X(4:6) per-axis, element-wise theta). Structure is a
faithful mirror of the Python (Theta=[vector|eye(3)], same chi_zeta_aug). Gate behind global
THETA_PER_AXIS (off ≡ scalar path ≡ current main, parity-checkable). (B) manuscript edits =
control_formulation.tex eq (control/adaptive law) ‖θ‖→Θ_⋆=diag(θ_k), Thm 1 proof per-axis collection,
add row-norm bound (★) + tight-vs-conservative remark; lift from Drafts/PER_AXIS_THETA_PROOF.md.
⚠ MATLAB + manuscript edited on WINDOWS by user (don't edit from Ubuntu — active manuscript work there).
Fill the n≥3 number into the handoff + manuscript remark before finalizing. [[project_manuscript_windows_ch_todo]]
