---
name: Funnels are named by the variable they bound — target feature funnel + optic-flow funnel
description: In the MDF-ASMC manuscript the two PPC funnels are "target feature funnel" (p_1(t), pixels) and "optic-flow funnel" (p_2(t), rad/s); never "target image funnel", "image feature funnel", "visibility funnel", "inner/outer funnel", or a purpose-based tag. Renamed 2026-06-10 (epoch 3, user-directed).
type: feedback
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---
Locked 2026-04-22: the two PPC funnels are named by the **variable** they bound, not by the downstream effect they produce.

- **Target feature funnel** — PPC envelope $\boldsymbol{p}_1(t)$ on the image feature-point position in the image plane (pixel units).
- **Optic-flow funnel** — PPC envelope $\boldsymbol{p}_2(t)$ on the optic-flow error $\boldsymbol{h}_\text{e}$ (rad/s).

**Forbidden names (all superseded):**
- `target image funnel` — superseded 2026-06-10 (user: "replace target image funnel to target feature funnel"); applied across manuscript.tex, control_formulation.tex, results.tex, supplemental.tex, and scripts/make_plasmc_plots.py titles. Affected figure PDFs still need regeneration (multi-init .mat data lives on Ubuntu).
- `image feature funnel` — brief intermediate name (contribution 4 only, earlier 2026-06-10); unified to `target feature funnel` same day.
- `visibility funnel` / `Visibility Funnel` — the old name for $\boldsymbol{p}_1(t)$. Visibility is a *closed-loop effect* delivered by Corollary 1 (funnel + cone clamp + ISS cascade), not a property of the bound; naming a bound after its downstream effect is aspirational.
- `soft-landing funnel` — rejected for the same reason on the $\boldsymbol{p}_2(t)$ side; soft landing is the effect, not the bound.
- `inner funnel` / `outer funnel` — aliases with "inner loop / outer loop" and readers conflate the two.
- `shrinking pixel-box` alone — OK as a *descriptive phrase* inside one sentence of a subsection intro, not as the authoritative name.
- `optic-flow performance funnel` — redundant (a funnel is already PPC).

**Why (chain of decisions 2026-04-22):**
1. Discussion established that `\boldsymbol{p}_1(t)$ and `\boldsymbol{p}_2(t)` are structurally identical PPC envelopes and should have symmetric names.
2. Purpose-based pair (*visibility funnel* / *soft-landing funnel*) was considered but rejected: both names over-claim, since each property requires more than just the bound (visibility needs the cone clamp; soft landing needs $\boldsymbol{h}_\text{d}$ encoding $h_\text{rd}$). Theorem 1 literally proves invariance of the **bound** $\boldsymbol{p}_2(t)$; calling that bound "soft-landing funnel" makes the theorem title misleading.
3. Variable-based pair (*target image funnel* / *optic-flow funnel*) was adopted: names map unambiguously to the bounded variable, its unit, and the equation label. Theorem 1 reads coherently as "Adaptive Optic-Flow Funnel Invariance" — exactly what's proved.

**How to apply:**
- Scope: `control_formulation.tex`, `supplemental.tex`, `manuscript.tex` (abstract / contributions / conclusion), `results.tex`, `block_diagram.tex`, all figure captions and table headers.
- First-sentence introduction in any rewrite: name both funnels explicitly (*"a target image funnel on $\boldsymbol{p}_1(t)$ and an optic-flow funnel on $\boldsymbol{p}_2(t)$"*).
- Subsection heading "Visibility Funnel: FoV-Adaptive Cone Clamp" → "Target Image Funnel and Funnel-Margin Cone Clamp" (locked 2026-04-22; "FoV-Adaptive Cone Clamp" was an intermediate epoch and has also been superseded). The funnel is the *bound*; the cone clamp is the *enforcement mechanism* — keep them architecturally distinct in headings and prose.
- Captions in `supplemental.tex` Table S2 header: "Target Image Funnel — Funnel-Margin Cone Clamp".
- Figure captions referencing $\pm\boldsymbol{p}_1(t)$: "visibility funnel" → "target image funnel".
- Equation label `rho fov: equation` is a free-rename candidate but low priority — label keys don't render.
- Downstream closed-loop properties (visibility, soft landing) remain the correct language for **effects** — they belong in abstract/contributions/theorem/corollary statements, not on the bound itself.
