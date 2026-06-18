---
name: Rewrite from scratch; no legacy-design comparisons in the manuscript
description: When updating manuscript text after an implementation change, rewrite fully from the current code — do not say "replacing X of earlier designs" or reference the superseded approach
type: feedback
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---
When the implementation has changed and the tex needs to reflect it, the tex must describe the CURRENT controller as-if-designed. Do not write phrases like:

- "replacing the barrier-transformed PPC coordinate of earlier dual-funnel designs"
- "unlike our previous formulation..."
- "superseding the legacy visibility funnel on $\hat{\boldsymbol{r}}_\text{e}$"

**Why:** The manuscript is the public record of the final design, not a changelog. Comparisons to internal prior versions (a) confuse reviewers who never saw the old version, (b) imply the legacy approach was known-wrong when in fact it was a design choice, and (c) clutter the narrative. Only compare against *published* baselines from other authors.

**How to apply:**
- Read the current MATLAB implementation end-to-end before rewriting any control-formulation section.
- Describe the law, the math, and the rationale in forward-looking language: "the outer loop generates $\boldsymbol{h}_\text{d}$ via PID on $\bar{\boldsymbol{r}}_\text{e}$", not "the outer loop now uses PID instead of..."
- If an architectural reason motivates the choice, justify it on its own merits (e.g. "normalization makes the gains sensor-resolution invariant"), not by contrast with a deprecated internal variant.
- Applies to all tex files: control_formulation.tex, supplemental.tex, manuscript.tex, block_diagram.tex, captions, remarks.
