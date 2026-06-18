---
name: Frame visibility in physical terms, not corner-point math
description: In the DF-ASMC manuscript prose/abstract/contributions, describe visibility as "target remains visible" — not "corner points stay inside the FoV"
type: feedback
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---
When talking about the visibility guarantee in the manuscript prose, abstract, contribution list, theorem/corollary statements, and figure captions, use **physical / reader-facing language**:

- **Preferred:**
  - "the target remains visible throughout the descent / until precise soft landing is achieved"
  - "the proposed controller ensures that the target is always visible for achieving soft precise landing"
  - "the target image stays inside the FoV until precise soft landing"
- **Avoid in prose:**
  - "the target image corner points stay inside the FoV"
  - "corner points remain in the pixel-box $\boldsymbol{\rho}_\text{fov}(t)$"
  - "visibility of the corner points is preserved"

**Why:** Corner points are the *mathematical handle* we use inside proofs and equations; they are not what a reader intuitively cares about. Readers — especially reviewers scanning the abstract or contribution list — want the physical statement: the camera keeps seeing the target. Leading with corner-point language turns a crisp physical guarantee into a technical footnote.

**How to apply:**
- **Inside equations, set definitions, cone-clamp derivation, and the proof proper:** corner-point language is still correct and necessary (it is the variable being bounded).
- **In theorem/corollary *statements* and their verbal summary lines, abstract, contributions, conclusion, section intros, figure captions:** use "target remains visible" / "target stays inside FoV".
- Pair with the existing corner-point memory (feedback_corner_points_naming.md): "corner point" is the right term *when* talking about the math; in prose-level statements, lift the claim to "target visibility".
