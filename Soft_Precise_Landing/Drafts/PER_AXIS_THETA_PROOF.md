# Per-Axis Regressor-Norm ASMC — Law Change + Lyapunov Proof

**Status:** proposed change to the optic-flow ASMC law (Theorem 1, `control_formulation.tex`).
Implemented behind `PLASMC_THETA_PER_AXIS` in `PX4_Gazebo/src/controller.py` (default off).
Motivation: the GT-FB axis-by-axis dig (`Memory/px4/feedback_terminal_root_lateral_zeta_r.md`)
showed the terminal fly-away is the **lateral position barrier `ζ_r` inflating the SHARED scalar
`‖θ‖`**, which then detonates the switching term + κ-ODE on *every* axis (the z over-brake is
collateral). Decoupling `θ` per axis removes that coupling **without weakening the stability
guarantee** — and is in fact the *tight* bound the shared Frobenius norm over-approximates.

Notation follows `control_formulation.tex`: bold **θ** is the regressor **matrix**
`θ = [ v | I₃ ] ∈ ℝ^{3×4}`, with `v = −c_h + S_h ṗ_h − G_h⁻¹ χ ζ̇_aug` (code `vector`);
`d̄` is the lumped disturbance, `‖d̄‖ ≤ d̃`; `G_h = diag(G_k) ≻ 0`; `κ ∈ ℝ₊³`;
`Γ, E, N, P ≻ 0` diagonal; `β ≥ β_min > 0`.

## 1. The sliding-surface dynamics are already diagonal in the disturbance

From (sigma derivative: equation 2),
```
σ̇ = β G_h [ u_h + c_h − S_h ṗ_h + G_h⁻¹ χ ζ̇_aug ] + β_min G_h ( θ d̄ ).
```
The disturbance enters through the matrix–vector product `θ d̄`. Because `θ = [v | I₃]`, its
**k-th component is the k-th ROW acting on `d̄`**:
```
(θ d̄)_k = v_k δ₀ + d_{h,k},     d̄ = [δ₀, d_h^⊤]^⊤,
|(θ d̄)_k| ≤ ‖e_k^⊤ θ‖ · ‖d̄‖ = θ_k · ‖d̄‖ ≤ θ_k d̃,     θ_k ≜ ‖e_k^⊤ θ‖ = √(v_k² + 1).   (★)
```
So each axis sees a disturbance bounded by its **own row norm** `θ_k`, NOT by the whole-matrix
norm. The shared law bounds every axis by `‖θ‖_F = √(Σ_k θ_k²) ≥ θ_k`, i.e. it over-bounds axis
`k` by the **worst row** — this is exactly the coupling pathology (the lateral row `v_{1,2}`
blowing up inflates the gain on z).

## 2. Modified control law (the change)

Replace the scalar `‖θ‖` by the **diagonal of row-norms** `Θ_⋆ ≜ diag(θ_1, θ_2, θ_3)`:
```
u_sw = −Γ σ − Θ_⋆ sat(E⁻¹ σ) G_h κ(t),                         (switching)
κ̇(t) = Θ_⋆ N G_h |σ| − N P κ(t),     κ(0) ∈ ℝ₊³.               (adaptation)
```
Componentwise: `u_{sw,k} = −γ_k σ_k − θ_k sat(σ_k/ε_k) G_k κ_k`,
`κ̇_k = θ_k N_k G_k |σ_k| − N_k P_k κ_k`. The known-part cancellation (`u_eq`) is unchanged.
Setting `θ_k ≡ ‖θ‖_F ∀k` recovers the published law exactly, so this is a strict generalization.

**Key structural invariant:** `θ_k` multiplies BOTH the axis-k switching dissipation AND the
axis-k κ-driving term — the same symmetry that makes the scalar proof's cross-cancellation work,
now holding per axis.

## 3. Lyapunov proof (UUB preserved, same `V`, same bound)

Use the **identical** candidate (V candidate: equation):
```
V = ½ σ^⊤ σ + (β_min/2) [κ − d̃ 1]^⊤ N⁻¹ [κ − d̃ 1],   1 = [1,1,1]^⊤.
```
Along the closed loop, with the modified law,
```
V̇ = σ^⊤ σ̇ + β_min [κ − d̃1]^⊤ N⁻¹ κ̇
   = −β σ^⊤ [ Γσ + Θ_⋆ sat(E⁻¹σ) G_h κ ]            (switching, from σ̇)
     + β_min σ^⊤ G_h (θ d̄)                            (disturbance, from σ̇)
     + β_min [κ − d̃1]^⊤ N⁻¹ [ Θ_⋆ N G_h |σ| − N P κ ].   (adaptation)
```
Apply the componentwise saturation identity (unchanged from the manuscript)
`σ_k sat(σ_k/ε_k) = |σ_k| − r_{ε_k}`, `0 ≤ r_{ε_k} ≤ ε_k/4`, and `β ≥ β_min`. Collect terms
**per axis** `k` (everything is diagonal: `G_h, Θ_⋆, E, N, P` and `sat` acts componentwise):

*Switching dissipation* contributes `−β_min θ_k G_k κ_k (|σ_k| − r_{ε_k})`.
*Disturbance* is bounded by (★): `β_min σ_k G_k (θ d̄)_k ≤ β_min θ_k G_k |σ_k| ‖d̄‖ ≤ β_min θ_k G_k |σ_k| d̃`.
*Adaptation cross-term* contributes `+β_min θ_k G_k |σ_k| (κ_k − d̃) − β_min P_k κ_k(κ_k − d̃)`
(from `[κ_k − d̃] N_k⁻¹ · θ_k N_k G_k |σ_k|` and the leakage).

The `θ_k G_k |σ_k| κ_k` pieces **cancel exactly** (switching vs adaptation), leaving the
disturbance/`d̃` remainder. Summing over `k`:
```
V̇ ≤ −β_min σ^⊤ Γ σ
    − β_min (d̃ − ‖d̄‖) Σ_k θ_k G_k |σ_k|              (≥ 0 since ‖d̄‖ ≤ d̃)
    − β_min [ κ^⊤ P κ − d̃ 1^⊤ P κ ] + (boundary-layer residual).
```
This is **line-for-line the manuscript inequality** (control_formulation.tex eq. after
"rearranges to"), with the single replacement `‖θ‖ → θ_k` inside the middle term — which is
still `≥ 0`, so it drops. The boundary-layer residual `≤ ¼‖ε‖ λ_max(G_h)‖κ‖` is absorbed into
`d̃` by the same one-time Assumption-1 inflation. Completing the square on `κ^⊤Pκ − d̃ 1^⊤Pκ`
and using `σ^⊤Γσ ≥ λ_min(Γ)‖σ‖²`, `1^⊤P1 ≤ 3λ_max(P)`:
```
V̇ ≤ −φ₁ V + (3/2) β_min λ_max(P) d̃²,     φ₁ = min{2β_min λ_min(Γ), λ_min(NP)} / max{1, λ_max(β_min N⁻¹)},
```
identical to the published `φ₁`. Hence for any `0 < φ₂ < φ₁`, `V̇ ≤ −φ₂V` whenever
`V ≥ 3β_min λ_max(P) d̃² / [2(φ₁−φ₂)]`, and by the GUUB lemma every trajectory reaches and stays
in `{V ≤ Θ}`. Using `V ≥ ½‖σ‖²` gives the **same UUB radius**
```
ϑ = √( 3 β_min λ_max(P) / (φ₁ − φ₂) ) · d̃.                         ∎
```
Funnel recovery (ζ_h, ζ_r → p_h, p_r) and Corollary 1 (visibility) are unchanged: they depend
only on `σ` being UUB and on the CBF forward-invariance (Theorem 3), neither of which uses the
form of the switching gain.

## 4. Why this is strictly better, not just equivalent

- **Tightness.** (★) shows `θ_k = √(v_k²+1)` is the *exact* per-axis disturbance gain; the scalar
  `‖θ‖_F` over-bounds each axis by `√(Σ_j v_j² + 3) ≥ θ_k`. The published proof is the special
  case where every per-axis gain is inflated to the worst row.
- **Decoupling.** When the lateral row `v_{1,2}` blows up near the deck (ζ_r → p_r), the shared
  `‖θ‖` injects that spike into the **z** switching term (`θ·G_z·κ_z`) and the z κ-ODE → the
  collateral z over-brake. With `Θ_⋆`, `θ_z = √(v_z²+1)` is governed by the **loom** row only
  (small, per the dig), so z is isolated from the lateral barrier.
- **No new assumptions.** Same `V`, same `d̃`, same Assumptions 1–2, same `φ₁`, same `ϑ`. The
  adaptive gains `κ_k` still track the (existence-only) `d̃` per axis; the leakage/anti-windup
  guards (Remark 3/5) are untouched.

## 5. Manuscript edits (when validated)
- Eq (sliding/adaptive control law): `‖θ‖ → Θ_⋆ = diag(‖e_k^⊤ θ‖)`; add (★) as the row-norm bound.
- Theorem 1 proof: insert the per-axis collection (Section 3 above); note the scalar law as the
  conservative corollary.
- Remark: per-axis regressor norm = tight bound; decouples cross-axis switching (cite the
  empirical terminal-root finding).

## 6. Caveat (honest)
The proof guarantees **UUB of σ**, not zero terminal error — exactly as before. The per-axis form
removes the *collateral coupling* (z over-brake driven by lateral ζ_r) and restores the tight
gain; it does **not** by itself cure the lateral runaway (s_e_n 1/Z amplification), which is the
separate binding problem. Expect: cleaner/decoupled z terminal, reduced shared-θ detonation; the
lateral ζ_r blow-up still needs its own lever.

## 7. Empirical confirmation (GT-FB IC1–5 gate, bundle 20260625-022346)
CONFIRMED. Terminal (alt<0.3m) peak |a_u_z| stays **3.6–7.8** under per-axis θ even when the lateral
barrier fully explodes (|a_u_xy| up to 2995, logged ‖θ‖_F up to 2968) — vs the shared-θ fly-aways
where the same lateral spike detonated z to **92–320** (the collateral over-brake). So z is
structurally decoupled, exactly as Section 3 predicts. Causal close: the lateral spike still occurs
(caveat §6 holds) but with z no longer ballooned, the drone touches down before the spike throws it.
Outcome: **9/10 sub-meter, 0 fly-aways, 0 stalls** (vs shared-θ baseline 8/10, 1 stall) — best gate
of the campaign, and the stall is removed too. The lateral a_u_xy spike (s_e_n 1/Z + ζ_r) is now the
ISOLATED remaining root. Recommend: confirm at n≥3, then bake `PLASMC_THETA_PER_AXIS=1` (GT-FB; a
proven, tight-bound control-law improvement). Production (perception-on) needs separate validation.
