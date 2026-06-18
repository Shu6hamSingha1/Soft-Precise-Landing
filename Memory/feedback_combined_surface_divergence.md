---
name: feedback_combined_surface_divergence
description: "⭐ CONVERGENCE (2026-06-18): both the PX4 AND MATLAB code run the OLD back-mapped formulation (SEN_FUNNEL: s_e_n -> ds_d via G_s^-1 -> h_d includes ds_d; sigma = zeta_h + Omega*int zeta_h), but control_formulation.tex specifies the COMBINED SLIDING SURFACE (sigma = zeta_h + chi_r*zeta_r, h_d = FF-only, NO ds_d, zeta_r direct in sigma). The PX4 lateral wall AND the MATLAB c̃_h option-(b) IC5 block are the SAME root: missing direct position authority. Implementing the combined surface = the unified fix."
metadata:
  node_type: feedback
  type: feedback
  originSessionId: 3553f974-8018-4a1e-ba00-93060aa8d9e4
---

**The manuscript already specifies the structural fix; both code implementations diverge.**
⚠️ THE DESIGN ALREADY EXISTS — see [[project_stacked_barrier_backstepping]] (full design + Lyapunov +
manuscript edits DONE 2026-06-16; draft `Soft_Precise_Landing/Drafts/STACKED_BARRIER_BACKSTEPPING.md`;
**only the CODE is pending**), [[feedback_sliding_surface_relative_degree]] (why naive σ=ζ_h+λζ_s is
rejected but the 3-D combine is well-posed), [[feedback_sen_authority_analysis]] (why the outer PID
can't ensure s_e_n convergence). THIS memory's UNIQUE contribution = the **2026-06-18 CONVERGENCE**:
the MATLAB push d69ed78 (c̃_h option-(b) IC5 block) is the SAME old-form root as the PX4 lateral wall,
so the already-designed combined surface resolves BOTH. Chains off [[feedback_inner_loop_velocity_thread]]
[[feedback_ch_kinematics_correction]].

## The divergence (verified in both codebases)
- **`control_formulation.tex` (manuscript):** dual-funnel COMBINED SLIDING SURFACE. Position barrier
  `ζ_r` (hyperbolic-tan on normalized centroid error) enters the sliding surface DIRECTLY:
  `σ = ζ_h + χ·ζ_aug`, `ζ_aug=[ζ_r, ∫ζ_h3]` → lateral `σ_k = ζ_{h_k}+χ_{r_k}ζ_{r_k}` (PD: flow barrier
  = velocity coord, position barrier = position coord), descent `σ_3=ζ_{h3}+χ_z∫ζ_{h3}` (PI). **`h_d`
  = `w×s + [h_rd−(w×s)·ê3]s` — rotation+descent FEEDFORWARD ONLY, NO `ds_d`.** σ̇ carries the position
  barrier via `G_h⁻¹·χ·ζ̇_aug`; relative-degree-1 preserved. Text l.169: "Rather than back-mapping ζ_r
  into a desired feature rate, the barrier enters the optic-flow sliding surface directly."
- **PX4 `controller.py` + MATLAB `visualControl_IBVS_adaptive.m` (BOTH):** the OLD back-mapped form.
  `SEN_FUNNEL` back-maps `s_e_n → ds_d = G_s⁻¹(−Kp·ζ_s−Ki·izeta−Kd·dzeta)+S_s·dp_s`; `h_d = ds_d +
  cross(w,s) + (h_rd−…)s` (INCLUDES ds_d); `σ = ζ_h + Ω·∫ζ_h` (integral of the FLOW barrier on ALL
  axes — the position barrier `ζ_r` NEVER enters σ). MATLAB l.321/582/586 + PX4 confirmed identical.

## Why this is the unified root cause
In the OLD form the lateral CLOSING AUTHORITY lives in the FEEDFORWARD (the `w`-cross-products in
`c̃_h` / the back-mapped `ds_d`). In the manuscript's combined surface it lives in `ζ_r` DIRECTLY in σ.
So both observed failures are the SAME missing direct-position-authority:
- **MATLAB c̃_h option-(b) BLOCKED at IC5 (d69ed78):** the corrected (clean) `c̃_h` "lacks the closing
  authority the funnel guarantee needs — `s_e_n` breaches `p_s` 46% of steps, →6× p_s, no convergence"
  → because removing the feedforward cross-products removes the closing authority (old form).
- **PX4 lateral wall:** `s_e_n` converges-then-overshoots, `G_s⁻¹` collapse starves the integral
  recovery, `a_u` outward — same missing direct authority, same old form.
MATLAB chat recommended option (c) (present corrected model in paper, keep old-code results,
residual→`d_h`) — a PAPER WORKAROUND that LEAVES the code↔paper divergence. The real fix is the
combined surface (neither chat has implemented it in code).

## THE UNIFIED FIX = implement the combined surface
One change resolves all four: unblocks option (b) (closing authority → `ζ_r`, not the removed FF),
fixes the IC5 deficit, fixes the PX4 lateral wall, and aligns code↔paper (no `d_h` workaround). ⚠️
**MATLAB code is OFF-LIMITS (Windows-owned, user directive 2026-06-18) — do NOT edit it; PX4-side only.**

## PX4 IMPLEMENTATION SPEC — CODE-LEVEL MAP (controller.py @ 2026-06-18; env-gated `PLASMC_COMBINED_SURFACE` default-off)
Worked out by reading the exact blocks (NOT yet implemented — user moved impl to a fresh chat).
**KEY INSIGHT: it is a PURELY LATERAL change.** Descent (z) stays byte-identical because (a) `ds_d`'s
z-component is ALREADY 0 (`raw_ds_d = concat([V_ds_d_xy,[0.0]])` l.679), and (b) descent keeps its PI
form `σ_3=ζ_h3+Ω_z·∫ζ_h3`. So dropping `ds_d` and swapping the σ-augmentation only touch x,y.

DATA ALREADY AVAILABLE (SEN_FUNNEL computes them in `_updateImgFeatureParam`, which runs BEFORE PLASMC):
`ζ_r = self._zeta_s[-1]` (2-vec, l.617) · `ζ̇_r = smooth4(self._dzeta_s_deque)` (2-vec, deque l.426, fed
l.628). Requires `self._sen_funnel` ON (default) — guard the combined path on `self._sen_funnel and len(self._zeta_s)>0`, else old path.

THE CLEAN GENERALIZATION (old form = special case): define two 3-vectors, lateral from ζ_r, descent from Ω_z·ζ_h:
- `zeta_aug = [χ_r·ζ_r_x, χ_r·ζ_r_y, Ω_z·∫ζ_h3]` → `σ = ζ_h + zeta_aug` (replaces l.826 `σ=ζ+Ω·∫ζ`).
- `chi_dzeta_aug = [χ_r·ζ̇_r_x, χ_r·ζ̇_r_y, Ω_z·ζ_h3]` → used in BOTH:
  - Theta `vector` (l.857-859): `-c + S·dp - G⁻¹·chi_dzeta_aug`  (was `- G⁻¹·(Ω·ζ)`).
  - `a_v` (l.885-888): `… - chi_dzeta_aug`  (was `- Ω·ζ`).
  (OLD form is exactly `chi_dzeta_aug=Ω·ζ_h`, `zeta_aug=Ω·∫ζ_h` on all 3 axes — so the descent rows are unchanged.)

`h_d` (`_updateOptFlow` l.724-736) — DROP `ds_d` when COMBINED; composes cleanly with CH_CLEAN:
`ds_d_term = np.zeros(3) if self._COMBINED_SURFACE else self._ds_d[-1]`; then keep the existing
`if self._CH_CLEAN: h_d = ds_d_term + h_ref_eff*s  else: h_d = ds_d_term + cross_ws + (h_ref_eff-dot(cross_ws,e3))*s`.
(ds_d present iff NOT COMBINED; cross_ws present iff NOT CH_CLEAN — orthogonal knobs.)

KNOBS (`__init__` near l.284 + registry near l.218): `self._COMBINED_SURFACE = os.environ.get("PLASMC_COMBINED_SURFACE","0")=="1"`;
`self._chi_r = float(os.environ.get("PLASMC_CHI_R","1.0"))`. **χ_r SIGN: positive is correct** (pos err +x → ζ_r>0
→ σ drives ζ_h<0 → −x closing flow → centers). Keep it SIGNED so SITL can flip if it regresses, like CH_CLEAN's `_r`.
NOTE: the design ([[project_stacked_barrier_backstepping]]) permits χ_r as a 2×2 LATERAL BLOCK (χ=blkdiag(χ_r∈ℝ²ˣ², χ_z));
a scalar χ_r·I₂ (above) is the simplest diagonal instance — start scalar, generalize to per-axis/2×2 only if needed.

WHY REL-DEG-1 IS PRESERVED (matches manuscript): `ζ̇_r ∝ ṡ ∝` flow `h` (a STATE, rel-deg-2 in u) — it's a
MEASURABLE feedforward, exactly cancelled by the `u_eq` term `−G⁻¹·χ·ζ̇_aug`. So `σ̇ = β·G·u + known` — SMC sees u via ζ̇_h only.

VALIDATE: A/B at IC2 + IC1 regression + IC2-5 gate (PX4 SITL stochastic; the deterministic MATLAB-first
validation the convergence calls for is the Windows chat's to run, NOT mine). For MATLAB I may provide a
TEXT spec only, never edit their files. Expected win: `s_e_n` gets DIRECT SMC authority (not routed through
the corrupted/saturating velocity loop) → closes the lateral wall AND removes the c̃_h closing-authority deficit.
