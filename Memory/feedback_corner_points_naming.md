---
name: Tracked targets are "feature points" — drop "corner" and drop "physical" qualifiers
description: In MDF-ASMC tex the four tracked image-plane projections are "feature points" (image plane) or "virtual feature points" (virtual image plane); never "corner points", "physical corners", or "physical feature points"
type: feedback
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---
Locked 2026-04-22. The tracked features projected via the pinhole model into pixel coordinates are **feature points**, matching the supplemental §S1 convention already in place ("a feature point on the image plane is $\,^\mathcal{C}\boldsymbol{\hat{r}} = \ldots$"). The word "corner" is reserved for geometric corners of the target in 3D space (rarely referred to in the manuscript); once projected, the image-plane object is a *feature point*.

**Naming ontology:**
- **3D point** → the real-world target landmark in $\mathbb{R}^3$. Use this phrase when you need to name the 3D source (e.g., `A target 3D point projects onto the tilted image plane as $\,^\mathcal{C}\boldsymbol{\hat{r}}$`).
- **Feature point** → the pinhole projection $\,^\mathcal{C}\boldsymbol{\hat{r}}$ on the image plane (attached to camera frame $\mathcal{C}$). No "physical" qualifier.
- **Virtual feature point** → the projection $\,^\mathcal{V}\boldsymbol{\hat{r}}$ on the virtual image plane (attached to $\mathcal{V}$).

**Frame qualifiers bind to the plane, not to the point:**
- `image plane` (default; attached to $\mathcal{C}$) — no "physical" modifier.
- `virtual image plane` — attached to $\mathcal{V}$.
- `camera frame $\mathcal{C}$` — no "physical" modifier; contrasts with `virtual frame $\mathcal{V}$`.

**Forbidden phrasings:**
- `corner point` / `corner points` / `target image corner points` — superseded.
- `physical corner(s)` / `physical image corner(s)` — ambiguous between 3D corner and image-plane projection.
- `physical feature point(s)` / `physical image feature point(s)` — the "physical" adjective belongs on the **plane**, not on the projected feature.
- `physical camera frame` / `physical image plane` — drop "physical"; the unqualified form IS the physical camera.
- `corner` alone (shortened body usage like "every corner lies inside $\boldsymbol{p}_1(t)$") — use "every feature point" or "every feature" (bare noun acceptable inside a paragraph that has established "feature point" in its first sentence).

**Why:**
- Ambiguity of "physical corner": a reader could read it as either the 3D corner of the target in the real world or the image of that corner on the physical-camera plane.
- "Feature point" matches supplemental §S1, the IBVS literature (Chaumette 2006 etc.), and the block-diagram vocabulary.
- Dropping "physical" everywhere except on a 3D point lets the unqualified term be the default (camera/image-plane/frame), with "virtual" as the explicit derived variant — this matches the manuscript's rotation-out derivation where $\mathcal{V}$ is obtained **from** $\mathcal{C}$.

**How to apply:**
- `control_formulation.tex` ~10 occurrences (L40 figure caption, L160 block-diagram caption, L165 §III intro, L182 PID paragraph, L218 cone-clamp definition and motivation, L222 margin justification, L227/231 shortened body forms, L235 kinematic-invariance line).
- `supplemental.tex` 4 occurrences (L183 ISS cascade, L196 §S3 intro, L478/490 baseline failure discussion).
- `frames_planes.tex` L123 TikZ comment — non-rendering but update for grep-consistency.
- `block_diagram.tex` L50 node label — already "Normalized Virtual\\Feature PID"; unchanged.
- Figure captions generated from Python plots (`scripts/make_multi_init_plots.py`, etc.) — check for "corner" strings; update if present.

**Relation to older memory:** This memory supersedes the pre-2026-04-22 convention which called the tracked features "target image corner points". Older sessions and older drafts in `Drafts/` use the old name and should not be retro-edited.
