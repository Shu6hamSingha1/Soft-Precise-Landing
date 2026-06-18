---
name: Image-plane axes vs feature-point components — dual notation convention
description: Image-plane AXES (directions) use $\hat{X}_\text{c}, \hat{Y}_\text{c}$ (capital with camera-frame subscript); feature-point COMPONENTS (values) use $\,^\mathcal{C}\hat{x}, \,^\mathcal{C}\hat{y}$ (lowercase with camera-frame superscript). Never use $(u, v)$.
type: feedback
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---
**The rule (locked 2026-04-24):**

When the prose refers to an **axis / direction** on the image plane, use:
- $\hat{X}_\text{c}$ — image-plane horizontal axis (in camera frame $\mathcal{C}$)
- $\hat{Y}_\text{c}$ — image-plane vertical axis

When the prose refers to the **components / values** of a feature point on the image plane, use:
- $\,^\mathcal{C}\hat{x}$ — horizontal pixel coordinate
- $\,^\mathcal{C}\hat{y}$ — vertical pixel coordinate
- Full feature-point: $\,^\mathcal{C}\boldsymbol{\hat{r}} = [\,^\mathcal{C}\hat{x}, \,^\mathcal{C}\hat{y}]^\top$ (supplement §S1-A, eq `sup:pinhole camera model`)

**NEVER use $(u, v)$ for pixel coordinates or image-plane axes.** The standard computer-vision convention clashes with this paper's $(\hat{x}, \hat{y})$ notation established at `control_formulation.tex:129` ($\,^\mathcal{V}\boldsymbol{\hat{r}}_k = [\,^\mathcal{V}\hat{x}_k, \,^\mathcal{V}\hat{y}_k]^\top$) and supplement §S1-A.

**Why:** User flagged 2026-04-24 that I had written "$|u|_{\max}=134$~px", "$|v|_{\max}=115$~px", and "$u$ and $v$ image-plane axes" in prose I had authored, contradicting the $\hat{x}/\hat{y}$ convention already locked in the definition sections.

**How to apply:**
- Prose referencing worst-case pixel excursions: write $|\,^\mathcal{C}\hat{x}|_{\max}$, $|\,^\mathcal{C}\hat{y}|_{\max}$ — lowercase with camera-frame superscript prefix.
- Prose referencing displacement "along the image-plane axes": write $\hat{X}_\text{c}$, $\hat{Y}_\text{c}$ — capital with camera-frame subscript.
- Even when the surrounding equation block has already established $\boldsymbol{\hat{r}}_i$ without the frame prefix (e.g., `control_formulation.tex:222`, cone-clamp preamble), the **prose descriptions of pixel excursions must still use $\,^\mathcal{C}\hat{x}, \,^\mathcal{C}\hat{y}$ with the camera-frame prefix** for clarity in narrative sentences.
- Axis labels in figure captions (image-plane plots) should also use $\hat{X}_\text{c}, \hat{Y}_\text{c}$ (capital form).
- If writing about virtual-frame quantities (as at lines 129, 148), the prefix flips to $\,^\mathcal{V}$ accordingly: $\,^\mathcal{V}\hat{x}_k, \,^\mathcal{V}\hat{y}_k$. The distinction between $\mathcal{C}$ (physical pixel) and $\mathcal{V}$ (yaw-aligned virtual image) should follow the semantic context.

**Current status (2026-04-24):** The three pixel-axis $(u, v)$ uses in prose I authored have been fixed in `results.tex:98` (centroid worst-case excursions) and `supplemental.tex:389, 395` (per-IC analysis). Any new prose that references pixel excursions or image-plane axes must use the convention above.
