---
name: Funnel and camera-FoV symbol convention (MDF-ASMC)
description: Image feature funnel uses $\boldsymbol{p}_1(t)$ / $\xi_1$ (pixels); optic-flow funnel uses $\boldsymbol{p}_2(t)$ / $\Xi_2$ (rad/s); camera half-FoV uses $\boldsymbol{\varphi}_\text{max}$. Locked 2026-04-22; renamed 2026-06-10.
type: feedback
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---
Uniform symbol scheme for the two funnels and the camera half-FoV normalization constant (locked 2026-04-22, superseding the mixed `\boldsymbol{\rho}_\text{fov}` / `\ell_\text{fov}` / `\boldsymbol{p}_{1_0}`-as-sensor-halfwidth scheme).

**Image feature funnel (pixel units):**
- Time-varying bound: $\boldsymbol{p}_1(t) \in \mathbb{R}^2_{>0}$ — exponentially shrinking pixel-box (the bound enforced by the funnel-margin cone clamp; the cone clamp is the enforcement mechanism, not the funnel itself).
- Initial value: $\boldsymbol{p}_{1_0} = \boldsymbol{p}_1(0) \preceq \mathcal{R}/2$.
- Terminal value: $\boldsymbol{p}_{1_\infty}$.
- Decay rate: $\xi_1 > 0$ (scalar).
- Scalar-$k$ component: $p_{1_k}(t)$, $k\in\{1,2\}$.

**Optic-flow funnel (rad/s units, 3D):**
- Time-varying bound: $\boldsymbol{p}_2(t) \in \mathbb{R}^3_{>0}$.
- Initial/terminal/decay: $\boldsymbol{p}_{2_0}$, $\boldsymbol{p}_{2_\infty}$, $\Xi_2 = \text{diag}(\xi_{2_k})$.

**Camera half-FoV (feature / tangent units):**
- $\boldsymbol{\varphi}_\text{max} = \mathcal{R}/(2f) \in \mathbb{R}^2_{>0}$ — camera half-FoV expressed in tangent (feature) units.
- Used as the normalization denominator in $\bar{\boldsymbol{r}}_\text{e} = \hat{\boldsymbol{r}}_\text{e}\oslash\boldsymbol{\varphi}_\text{max}$.
- Matrix form in supplemental proof: $\Phi_\text{max} = \text{diag}(\boldsymbol{\varphi}_\text{max})$.
- Do NOT call this `\boldsymbol{p}_{1_0}` — that symbol is now the target-image-funnel initial value (pixels), which is a DIFFERENT quantity from the sensor half-width (feature units).

**Why:** The user requested target-image-funnel / optic-flow-funnel uniformity (`\boldsymbol{p}_1(t)` mirroring `\boldsymbol{p}_2(t)`). This forced a rename of the sensor half-width `\mathcal{R}/(2f)` away from `\boldsymbol{p}_{1_0}`; `\boldsymbol{\varphi}_\text{max}` was chosen because it reads as "half-FoV angle in tangent units", which is physically exactly what the quantity is.

**Don't confuse the units:**
- $\boldsymbol{p}_{1_0}$ is the initial target-image-funnel bound in **pixels** (e.g., $[145,105]^\top$ px).
- $\boldsymbol{\varphi}_\text{max}$ is the camera half-FoV in **feature/tangent** units (= $\mathcal{R}/(2f)$, ≈ angular half-FoV in radians for small FoV).
- They describe the same FoV edge in different units, related by the focal length $f$: $\boldsymbol{p}_{1_0} \preceq \mathcal{R}/2 = f\,\boldsymbol{\varphi}_\text{max}$.

**How to apply:**
- When editing any funnel-related prose or equations, use `\boldsymbol{p}_1(t)` / `\boldsymbol{p}_{1_0}` / `\boldsymbol{p}_{1_\infty}` / `\xi_1` / `p_{1_k}` — never `\rho_\text{fov}` / `\ell_\text{fov}`.
- When writing the PID definition, use `\boldsymbol{\varphi}_\text{max}` for the normalization denominator — never `\boldsymbol{p}_{1_0}` (that collides with the target-image-funnel initial value).
- MATLAB variable names in the codebase (e.g. `rho_fov_0`, `l_fov`) keep their old names for code-compat; only the manuscript/supplement prose adopts this symbol scheme. Parameter-table row labels use the new tex symbols but keep the existing numeric values.

**Subscript-swap decision (locked 2026-04-24):**

After the §III opener was reordered to introduce the optic-flow funnel FIRST and the target image funnel SECOND in prose (matching the §III.A subsubsection development order: PID → optic-flow funnel + ASMC → target image funnel + cone clamp), user asked whether the subscripts should be swapped (`p_2` → `p_1` for optic flow, `p_1` → `p_2` for target image) for "uniformity with the new narrative order".

**Decision: DO NOT SWAP.** The `_1` ↔ image, `_2` ↔ optic-flow assignment is permanent.

**Why:**
1. Subscripts are disambiguating LABELS, not ordinal markers — a reader looks up what `p_1` denotes, doesn't infer ranking from "1".
2. Massive ripple-through: `_2` is baked into $\boldsymbol{\zeta}_2$, $\Xi_2$, $\xi_{2_k}$, $\mathcal{S}_2$, $S_2$, $\mathcal{G}_2$, $g_{2_k}$, $h_{e_k}$, $p_{2_k}$, $\zeta_{2_k}$, all of Theorem 1's statement and proof, supplement §S2-A feasibility remark, Table S1 blocks, MATLAB code variable names, and Python figure scripts. Swap would touch 60-80 sites; high stale-subscript risk.
3. Narrative-order change does not require math-label change — control-theory literature routinely has σ_1, σ_2, V_1, V_2 etc. that don't imply ordinal ranking.

**How to apply:**
- Do not propose subscript swaps based on narrative reordering of any kind.
- If a user asks again about uniformity, point to this rationale.
- Reorder prose freely, but leave the math labels alone.
