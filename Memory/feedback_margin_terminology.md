---
name: "Funnel margin" vs "inset" — two different quantities, two different names
description: Time-varying distance from a feature point to the target image funnel boundary is the "funnel margin" (= $d_\text{min}^\text{fov}$); the fixed 15-px geometric offset $\mathcal{R}/2-\boldsymbol{p}_{1_0}$ is the "inset" — never conflate the two under the generic word "margin"
type: feedback
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---

Locked 2026-04-22. Two semantically distinct quantities in §III.C were both called "margin" (variants: "corner-point margin", "pixel-corner margin", bare "margin"). Unify under two separate names:

- **Funnel margin** — the *time-varying* minimum distance from any target feature point to the target image funnel boundary, formalised as $d_\text{min}^\text{fov}(t) = \max(0,\min_{i,k}(p_{1_k}(t) - |\,^\mathcal{C}n_{i,k}|))$. This is the quantity the funnel-margin cone clamp reads to size $\theta_\text{cone}(t)$. (The clamp is *named after* this scalar — term locked 2026-04-22.)

- **Inset** — the *fixed geometric offset* between the sensor half-widths $\mathcal{R}/2$ and the initial target image funnel $\boldsymbol{p}_{1_0}$; "the initial box satisfies the strict inset $\boldsymbol{p}_{1_0}\prec\mathcal{R}/2$ (15~px per side)". This is a design choice absorbing perspective spread from initial body tilt, NOT a time-varying quantity.

**Forbidden spellings (all superseded):**
- `corner-point margin` / `corner margin` — old phrasing, not aligned with feature-point naming.
- `pixel-corner margin` — old phrasing.
- `feature-point margin` — proposed briefly, rejected in favor of "funnel margin" which emphasizes what the distance is measured *to* (the funnel boundary), not *from*.
- Bare `margin` used for the fixed 15-px inset — use `inset` instead, to avoid collision with "funnel margin".

**Why "funnel margin" and not "feature-point margin":**
- The quantity is fundamentally a distance-to-boundary; naming it after the boundary (funnel) reads semantically cleaner than naming it after the endpoint (feature point).
- Matches the colour pairing of the locked funnel-naming convention: $\boldsymbol{p}_1(t)$ is the target image funnel; its shrinking edge is what the margin is measured against.
- "Funnel margin" generalises cleanly to the optic-flow funnel (if we ever need a dual quantity $\min_k(p_{2_k}(t)-|h_{e_k}|)$, that becomes the "optic-flow funnel margin").

**How to apply:**
- `control_formulation.tex` §III.C L218, L222 (×2), L225 caption context — rename every surviving "corner-point margin" / "pixel-corner margin" / bare "margin"-referring-to-time-varying-distance → "funnel margin".
- `control_formulation.tex` L222 "this margin absorbs the perspective spread" → "this inset absorbs...".
- Symbol `d_\text{min}^\text{fov}` keeps its name (label keys don't render), but prose definitions should read "the worst-case funnel margin $d_\text{min}^\text{fov}$".
- Any figure caption / supplement Table S2 row referring to this quantity: use "funnel margin" consistently.
