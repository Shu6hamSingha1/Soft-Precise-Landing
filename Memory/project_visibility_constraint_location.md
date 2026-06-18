---
name: Visibility-constraint equation lives in supplement §S1-A (camera-frame pixels)
description: As of 2026-04-23, the FoV visibility constraint equation is defined only in the supplement (label `sup:visibility_constraints`), stated on the camera-frame image feature $\,^\mathcal{C}\hat{\boldsymbol{r}}$ in pixel units; main-paper §II-B no longer contains eq (3)
type: project
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---
The FoV visibility constraint equation now lives only in the **supplement §S1-A** (Virtual-Camera Model and Feature Points), between the pinhole camera model and the virtual-camera-frame introduction.

**Canonical statement (supplement eq, label `sup:visibility_constraints`):**
$$-\mathcal{R}/2 \le \,^\mathcal{C}\hat{\boldsymbol{r}} \le \mathcal{R}/2,\qquad \mathcal{R}=[r_h,r_w]^\top$$
— stated on the **camera-frame** image feature (pixel units), not on $\,^\mathcal{V}\hat{\boldsymbol{r}}$ (virtual frame). A trailing sentence explains that the funnel-margin cone clamp of §III-B.4 tightens the bound through $\boldsymbol{p}_1(t)\preceq\mathcal{R}/2$ and enforces it kinematically.

**Why the frame change:** MATLAB cone clamp operates on `C_nP` (`visualControl_IBVS_adaptive.m` L224, L450–452) — camera-frame physical pixels, NOT the virtual features. The earlier main-paper eq (3) stated the constraint on the virtual frame, which was inconsistent with the Approach 2 architecture. Rewriting it on $\,^\mathcal{C}\hat{\boldsymbol{r}}$ aligns the math with the code: both the cone clamp and the visibility constraint live in the same camera-frame pixel domain, so the target image funnel $\boldsymbol{p}_1(t)$ IS the visibility constraint (in tightened form).

**Consequences for main-paper prose (control_formulation.tex):**
- Eq (3) block is **deleted** from §II-B; the narrative now flows from the normalized-position definition directly to "Differentiating \eqref{normalized_pos: equation} and defining ..."
- Three former `\eqref{visibility_constraints}` references (§II-B problem statement, §III-B.4 end, Corollary 1) are rewritten without the `\eqref` — they now read as plain-text "the visibility constraint" since cross-document refs don't resolve.
- The §III-B.4 inset motivation is now a **safety-margin** argument (15 px between the target image funnel and the image-plane boundary to absorb initial-body-tilt perspective spread) — NOT the old false frame-reconciliation claim ("cone clamp on virtual features, visibility on image-plane features"), which was deleted 2026-04-23.

**How to apply:**
- Any new main-paper prose that needs the pixel FoV equation must cite it as "the visibility constraint (Section~S1-A of the supplement)" — do NOT re-introduce the equation into the main paper.
- Any prose describing the cone clamp / visibility guarantee must stay in the camera-frame pixel domain; never claim the cone clamp operates on virtual features.
- The supplement's follow-on sentence "$\boldsymbol{p}_1(t)\preceq\mathcal{R}/2$ and enforces it kinematically" is the reader-facing link between the FoV math and the target image funnel — keep it intact if you touch §S1-A.
