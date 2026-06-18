---
name: project_stacked_barrier_backstepping
description: "New middle-loop design — incorporate position barrier ζ_r into the sliding surface alongside ζ_h (σ=ζ_h+χ_r·ζ_r); replaces the authority-deficient outer PID. Writeup in Drafts, .tex/code pending"
metadata: 
  node_type: memory
  type: project
  originSessionId: ce5926b8-f1ea-4718-84c8-9a29e27ac4c6
---

**New formulation (2026-06-16): incorporate ζ_r into the sliding surface with ζ_h.** Naming:
`_r`=feature (normalized image-point error r̄_e) control, `_h`=optic-flow control. Full writeup:
`Soft_Precise_Landing/Drafts/STACKED_BARRIER_BACKSTEPPING.md`.

**Motivation (user):** the back-mapped PID/PPC on ζ_r (outer feature loop → desired feature rate)
can't ensure convergence of s_e_n=r̄_e — its marginal restoring gain −(rp/2)g'(S_r) peaks at
|S_r|=0.648 then goes ANTI-RESTORING ([[feedback_sen_authority_analysis]]). So move ζ_r OUT of the
weak PID and INTO the sliding surface, where the robust ASMC (Γ, adaptive κ) drives it.

**Surface (3-D COMBINE — well-posed):** `σ = ζ_h + χ ζ_aug`, `χ=diag(χ_r,χ_z)` (blkdiag: χ_r∈ℝ²ˣ²
lateral block, χ_z scalar — NOT diag(χ_r,χ_r,χ_z)), `ζ_aug=[ζ_{r,k}; ∫ζ_{h,z}]` — i.e. replace ∫ζ_h with the honest position barrier ζ_r on the
lateral axes, keep the flow-integral on descent:
```
lateral:  σ_k = ζ_{h,k} + χ_r ζ_{r,k}      descent: σ_z = ζ_{h,z} + χ_z ∫ζ_{h,z}
h_d = w×s + [h_rd−(w×s)·ê₃]s   (rotation/descent feedforward ONLY; NO PPC desired-rate term)
```
Well-posed: ∂σ̇/∂u_h = βG_h (3×3 invertible, rel-deg-1). ζ_r is rel-deg-2 (ζ̇_r has no u_h) but is
MEASURED → χ_r ζ̇_r is a known drift folded into u_eq (like the existing χ_z ζ_h). Position
authority flows through ζ_h. ASMC machinery (u_eq, u_sw, leakage κ) UNCHANGED except the drift
term gains χ_r ζ̇_r. On σ_k=0: ζ_{h,k}=−χ_r ζ_{r,k} → reduced dynamics drive ζ_r→0 (honest centroid).

**REJECTED (rel-degree, for the record) — see [[feedback_sliding_surface_relative_degree]]:**
- Flat σ=ζ_h+λζ_s presented as cascade-replacement with a "PD" claim (my 1st attempt; over-conceded
  as fully wrong — the 3-D combine itself is actually well-posed; the bad parts were the framing).
- 5-D PI surface σ=[ζ_r;ζ_h]+Χ∫[ζ_r;ζ_h]: interaction matrix [0_{2×3};βG_h] rank-3-in-5-D,
  underactuated (3 inputs ≤ 3-D surface). THIS is the genuinely invalid one.
- Backstepping cascade (ζ_r→virtual flow h_d→ζ_h ASMC) was an interim correct option, now
  superseded by putting ζ_r in the surface (PID authority was the whole problem).

**STATUS (2026-06-16): WRITTEN INTO control_formulation.tex** (backup
Drafts/control_formulation_pre_combined_surface_*.tex; verified 0 new LaTeX errors vs backup).
Edits: §III-A1 PID→image-feature funnel ζ_r (new eq `position barrier: equation`) + h_d=rot/descent;
§III-A2 new `sliding surface: equation` σ=ζ_2+χζ_aug (kept FLOW as ζ_2, NOT renamed to ζ_h, to
preserve Theorem-1/supplement/table consistency); θ/u_eq drift Χζ_2→χζ̇_aug; Corollary 1 proof
via ζ_r-in-σ; figure caption + tuning subsecs. NOT committed yet.
**2026-06-16 follow-on edits to control_formulation.tex (all 0-new-LaTeX-error verified):**
- CBF MATERIAL added from manuscript.tex: §III-A3 cone clamp → Target-Visibility CBF (eqs cbf
  barrier/Lw/cbf qp) + Property 1 + LPF; added Theorem 3 (Target-Visibility Forward Invariance,
  eq cbf feasibility) + updated certificate summary/Remark 8 to Theorems 1–3; dual-funnel intro +
  caption + interface + tuning subsec → CBF. "control barrier function" spelled out ONCE (caption
  L141), abbreviated CBF elsewhere (user rule).
- ζ_2→ζ_h GLOBAL RENAME done in control_formulation.tex (flow funnel only; \varphi_2 Lyapunov,
  \mu_{20/02} moments, xie2016_2 cite key correctly UNTOUCHED). Backup Drafts/control_formulation_pre_zeta2h_*.
  SUPPLEMENT still ζ_2 → pending if one doc.
- LYAPUNOV VALIDATION (⭐ real finding): σ̇ reduction + V̇≤−φ₁V+const (σ UUB) CORRECT — new drift
  χζ̇_aug cancels in u_eq (algebra verified). BUT Theorem-1 final step "σ UUB ⇒ flow funnel holds"
  was a GAP for the LATERAL axes: σ_k=ζ_{h,k}+χ_r ζ_{r,k} mixes rel-deg-1 ζ_h + rel-deg-2 ζ_r; σ
  bounded ⇏ ζ_{h,k} bounded, and V=½σ²+… is blind to the joint ζ_h→+∞/ζ_r→−∞ boundary mode;
  circular with Corollary 1. FIX IMPLEMENTED (option a): re-grounded the lateral funnel on the
  CBF — Theorem 3 keeps features in FoV ⇒ r̄_e bounded ⇒ ζ_r bounded INDEPENDENTLY ⇒ ζ_{h,k}=
  σ_k−χ_r ζ_{r,k} bounded ⇒ funnel holds. (Residual subtlety for author: needs p_r FoV-consistent
  so the CBF bound transfers to the funnel barrier; Theorem 1 now forward-refs Theorem 3.)

RECONCILE (author): (1) block-diagram TikZ still shows PID block → regen; (2) supplement ζ_2→ζ_h +
c̃_h + CBF reconcile; (3) ✅ RESOLVED — see below; (4) manuscript.tex has the
intermediate back-mapped-PID ζ_s version. Code NOT yet changed (gated impl + IC validation pending).
[[project_manuscript_windows_ch_todo]]

**LYAPUNOV LATERAL-RECOVERY GAP — VALIDATED + RESOLVED (2026-06-18, full read of control_formulation.tex).**
Verified the σ̇ rearrangement algebra (β-folding absorbs `(β−1)𝒢_h⁻¹χζ̇_aug` into the regressor exactly),
the `u_eq` cancellation, the leakage-ASMC σ-UUB, the descent Hurwitz-PI recovery, and Thm 2/3 — all
RIGOROUS as written. The ONE real gap: lateral `σ_k=ζ_{h_k}+χ_{r_k}ζ_{r_k}` is an algebraic sum of two
independent barriers, V=½σ²+κ-term is BLIND to the joint mode (ζ_h=+L, χ_r ζ_r=−L; σ=0, V small, both
saturated) → σ-bounded ⇏ ζ_h-bounded. The CBF-grounding (Thm 3 → ζ_r bounded) is non-circular but has a
TRANSFER DEFECT: CBF enforces the FoV bound |r̄_e|≤1, ζ_r-finiteness needs the funnel bound |r̄_e|<p_r(t)
— coincide ONLY if p_r≥1. FIX (Resolution A, recommended): **standing condition p_{r∞}⪰1** (funnel bottoms
at the FoV, never sub-FoV) makes the transfer exact (strict CBF interiority ⇒ |r̄_e|<1≤p_r ⇒ ζ_r finite);
**costs NO precision** — precision is delivered by (p_h, χ_r) via the manifold ζ_{h_k}=−χ_{r_k}ζ_{r_k}
(|ζ_r|≈|ζ_h|/χ_r), NOT by shrinking p_r. Resolution B (if sub-FoV p_r wanted): ISS on the reduced
ζ_r-dynamics (more work, plant-dependent). Proof-addendum with LaTeX-ready corrected Thm-1 paragraph +
standing condition: `Soft_Precise_Landing/Drafts/COMBINED_SURFACE_PROOF_ADDENDUM.md`. Implication: p_{r∞}⪰1
is a guarantee-preserving constraint for the param table; χ_r is the lateral-stiffness knob.
[[feedback_combined_surface_divergence]]
