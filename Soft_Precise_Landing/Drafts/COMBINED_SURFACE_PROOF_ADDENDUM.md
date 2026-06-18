# Proof Addendum — Combined-Surface Lateral Funnel Recovery

**Target:** `control_formulation.tex`, Theorem 1 proof (the funnel-recovery step, ≈ l.328) and
Corollary 1. Addresses the residual gap introduced by the combined sliding surface
`σ = ζ_h + χ ζ_aug` (Eq. `sliding surface: equation`).

**Author-facing status.** This closes the open reconcile item *"verify the p_r/FoV margin in the
Lyapunov fix"* (Drafts/STACKED_BARRIER_BACKSTEPPING.md). The σ-UUB result (Theorem 1 core,
Eqs. `V candidate`–`Vdot bound`), the descent-axis recovery, Corollary's affine bound mechanism,
and Theorems 2–3 are **unaffected and rigorous as written** — verified by expanding the σ̇
rearrangement (Eq. `sigma derivative: equation 2`): the `(β−1)`-weighted known terms, *including the
new* `𝒢_h⁻¹χζ̇_aug`, are absorbed exactly by the regressor `θ` against `d̄`, leaving
`σ̇ = βu_sw + β_min𝒢_h θ d̄`. Relative degree 1 is genuinely preserved. Only the **lateral
funnel-recovery sentence** needs the correction below.

---

## 1. The gap (why the current lateral sentence is incomplete)

On the lateral axes `σ_k = ζ_{h_k} + χ_{r_k} ζ_{r_k}` is an **algebraic sum of two independent
barrier states**, not a stable filter (unlike the descent axis `σ_3 = ζ_{h_3} + χ_z ∫ζ_{h_3}`, a
Hurwitz PI whose integrator state obeys `İ + χ_z I = σ_3` and is therefore bounded by a bounded σ_3).
The Lyapunov function `V = ½σᵀσ + (β_min/2)[κ−d̃𝟙]ᵀ𝒩⁻¹[κ−d̃𝟙]` contains **no `ζ_h` or `ζ_r` term**, so
it is *blind* to the joint boundary mode

```
ζ_{h_k} = +L ,   χ_{r_k} ζ_{r_k} = −L   ⟹   σ_k = 0,  V small,  yet BOTH barriers saturated.
```

Hence **σ bounded ⇏ ζ_{h_k} bounded**; an *independent* bound on `ζ_{r_k}` is required to recover the
optic-flow funnel. The proof supplies it from the target-visibility CBF (Theorem 3) — correctly, and
non-circularly (Theorem 3 needs only bounded drift + feasibility + Assumption 2, not Theorem 1). But
there is a **transfer defect**: the CBF enforces the *field-of-view* bound `|r̄_{e_k}| ≤ 1` (via
`φ_max`), whereas `ζ_{r_k}`-finiteness needs the tighter *funnel* bound `|r̄_{e_k}| < p_{r_k}(t)`.
These coincide only when `p_{r_k}(t) ≥ 1`. For a sub-FoV precision funnel (`p_{r∞} < 1`) the CBF keeps
the centroid in-FoV but *outside* the funnel, so `𝒮_{r_k} = r̄_{e_k}/p_{r_k} → 1` and `ζ_{r_k} → ∞`
while σ stays bounded — the gap reopens **exactly in the precision regime**.

---

## 2. Resolution A (recommended) — FoV-consistent funnel `p_{r∞} ⪰ 1`

Make the position funnel bottom out *at* the FoV edge rather than inside it. This makes the
CBF→funnel transfer exact and **costs no precision**: lateral precision is delivered on the manifold
σ ≈ 0, where `ζ_{h_k} = −χ_{r_k} ζ_{r_k}` gives `|ζ_{r_k}| ≈ |ζ_{h_k}|/χ_{r_k}` — tightness comes from
the optic-flow funnel `p_h` and the surface weight `χ_r`, **not** from shrinking `p_r`. (This is
already the intent of the tuning text: *"larger χ_r sharpens lateral convergence."*)

### 2a. Standing condition — add near Eq. `position barrier: equation`

```latex
\textit{Standing Condition 1 (FoV-consistent image-feature funnel).}
The image-feature funnel half-width is field-of-view consistent,
$\boldsymbol{p}_{r_\infty}\succeq\boldsymbol{1}$, hence $p_{r_k}(t)\ge 1$ for all $t\ge 0$ and
$k\in\{1,2\}$; the funnel shapes the lateral transient but never tightens below the field-of-view
edge $|\bar{r}_{\text{e}_k}|=1$. Lateral precision is delivered by the optic-flow funnel
$\boldsymbol{p}_h$ and the surface weight $\boldsymbol{\chi}_r$ through the manifold relation
$\zeta_{h_k}=-\chi_{r_k}\zeta_{r_k}$ on $\boldsymbol{\sigma}\approx\boldsymbol{0}$, not by tightening
$\boldsymbol{p}_r$.
```

### 2b. Corrected lateral-recovery paragraph — replace the two lateral sentences in the Theorem 1 proof

```latex
On the two lateral axes $\sigma_k=\zeta_{h_k}+\chi_{r_k}\zeta_{r_k}$ algebraically couples the
optic-flow barrier with the relative-degree-two position barrier, so a bounded $\boldsymbol{\sigma}$
does not by itself bound $\zeta_{h_k}$; an independent bound on $\zeta_{r_k}$ is required. Under
Standing Condition~1 ($p_{r_k}(t)\ge 1$) the target-visibility CBF supplies it. Theorem~3 renders the
visibility set forward invariant with strict interior---$|\,^\mathcal{C}\hat r_{i,k}(t)|<\varphi_{\max,k}$
whenever $|\,^\mathcal{C}\hat r_{i,k}(0)|<\varphi_{\max,k}$, since $h_{i,k}(t)\ge h_{i,k}(0)e^{-\alpha_k t}>0$---so
the de-rotated centroid obeys $|\bar r_{\text{e}_k}|<1\le p_{r_k}(t)$. Hence
$\mathcal{S}_{r_k}=\bar r_{\text{e}_k}/p_{r_k}\in(-1,1)$ stays strictly inside the barrier domain and
$\zeta_{r_k}=\ln\frac{1+\mathcal{S}_{r_k}}{1-\mathcal{S}_{r_k}}$ is bounded \emph{independently} of the
optic-flow funnel. Consequently $\zeta_{h_k}=\sigma_k-\chi_{r_k}\zeta_{r_k}$ is bounded by the ultimate
bound on $\boldsymbol{\sigma}$ together with this CBF-supplied bound on $\zeta_{r_k}$.
```

### 2c. Corollary 1

Corollary 1's chain `χ_{r_k}ζ_{r_k} = σ_k − ζ_{h_k}` bounded ⟹ `ζ_r` UUB (affine in `ϑ`) is then valid:
`σ` is UUB by Theorem 1 and `ζ_{h_k}` lies in the optic-flow funnel by §2b, so `|ζ_{r_k}| ≤
(ϑ + ζ_h^{max})/χ_{r_k}`. Add one clause noting it holds **under Standing Condition 1** (so `ζ_r` is
well-defined throughout). No other change.

---

## 3. Resolution B (alternative, if a sub-FoV `p_{r∞} < 1` is wanted)

If the design insists on a position funnel that tightens below the FoV, replace the CBF-transfer with
an **ISS argument on the reduced position-barrier dynamics**. On σ ≈ 0, `ζ_{h_k} = −χ_{r_k}ζ_{r_k}`,
and `ζ̇_r = 𝒢_r(ṙ̄_e − 𝒮_r ṗ_r)` with `ṙ̄_e` the *measured centroid rate* (Eq. `s dot: equation`),
which is the lateral optic flow up to the de-rotation. The surface manifold makes the optic-flow error
a **restoring** function of `ζ_r` (a positive `ζ_r` commands, through σ and the SMC, an inward
corrective flow), so the reduced `ζ_r`-subsystem can be shown ISS with respect to the σ-residual,
yielding `ζ_r` bounded without `p_r ≥ 1`. This captures the actual self-correction but requires
modelling the relative-degree-two map from `ζ_{h_k}` to `ṙ̄_e` (the kinematic optic-flow↔centroid-rate
relation), which is plant-dependent; Resolution A avoids it and is preferred for the manuscript. The
CBF is still required for the hard FoV/visibility guarantee in either case.

---

## 4. Implementation implications (controller.py / MATLAB)

- **`p_{r∞} ⪰ 1` is a guarantee-preserving constraint, not a free knob** — record it in the parameter
  table (Table S1) alongside `p_{r0} ≻ p_{r∞}` and `|r̄_e(0)| < p_{r0}`.
- **Precision tuning routes through `(p_h, χ_r)`, never by shrinking `p_r`** below the FoV. Manifold:
  `|ζ_r| ≈ |ζ_h|/χ_r`, so raise `χ_r` (and/or tighten `p_h`) for tighter lateral hold.
- **Ties to the `c̃_h` option-(b) result:** lateral closing authority now lives in `ζ_r` *inside* σ,
  not in the feedforward `c_h`/`ḣ_d`. So the corrected clean `c̃_h` no longer strips lateral
  authority — option (b) unblocks once the combined surface is implemented. (Cross-ref
  `Memory/feedback_combined_surface_divergence.md`.)
