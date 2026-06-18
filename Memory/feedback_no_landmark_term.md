---
name: Don't use "landmark" — use the user's vocabulary
description: The user dislikes the term "landmark" for the four observed target points. Use the established vocabulary as locked 2026-05-04 — "image feature point" / "virtual image feature point" / "image point" / "virtual image point" — with per-feature subscript `_i` everywhere. Locked 2026-04-30; vocabulary updated 2026-05-04.
type: feedback
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---
**Rule.** Do **not** use the words "landmark", "corner-like landmark", "per-landmark", or any close variant when describing the observed target points. The user's preferred vocabulary, locked 2026-05-04, is:

| Quantity | Term to use (long form) | Short form | Symbol |
|---|---|---|---|
| 3-D point on the target | **target point** | — | $\,^\mathcal{V}\boldsymbol{r}_\text{t}$ |
| 2-D individual point in the camera image plane, $i\in\{1,\dots,N\}$ | **image feature point** | — | $\,^\mathcal{C}\boldsymbol{\hat{r}}_i$ |
| 2-D individual point in the virtual image plane, $i\in\{1,\dots,N\}$ | **virtual image feature point** | virtual feature point | $\,^\mathcal{V}\boldsymbol{\hat{r}}_i=[\,^\mathcal{V}\hat{x}_i,\,^\mathcal{V}\hat{y}_i]^\top$ |
| 2-D mean of the camera-plane feature points | **image point** | — | $\,^\mathcal{C}\boldsymbol{\hat{r}}$ |
| 2-D mean of the virtual-plane feature points | **virtual image point** | — | $\,^\mathcal{V}\boldsymbol{\hat{r}}$ |

## Subscript role lock (2026-05-04)

| Role | Letter | Example |
|---|---|---|
| Per-feature index = dummy summation index ($1..N$) | `i` | $\,^\mathcal{V}\boldsymbol{\hat{r}}_i$, $\sum_{i=1}^{N}$ |
| 3-D-vector component index ($1..3$) | `k` | $h_{e_k}$, $k\in\{1,2,3\}$ |
| Moment exponents | `p, q` | $\mu_{pq}$, $(\cdot)^p(\cdot)^q$ |

**Key change from earlier convention:** the per-feature subscript is now `i` (was previously `k` for virtual-frame projections only). Both camera- and virtual-frame per-feature points use `_i`, indexing the same physical point. The letter `k` is reserved for 3-D-vector component indices (e.g., `h_{e_k}` with `k\in\{1,2,3\}`).

## Naming history (vocabulary epochs)

| Epoch | Date | Per-feature term | Centroid term | Subscript |
|---|---|---|---|---|
| 1 | --- | "target image feature point" / "$k$-th feature point" | "target image point" | `_i` (camera) / `_k` (virtual) |
| 2 (locked) | **2026-05-04** | "image feature point" / "virtual image feature point" | "image point" / "virtual image point" | `_i` (uniform) |

**Why epoch 2 supersedes epoch 1:** drops the "target" qualifier (context-redundant since the camera observes only the target); unifies subscript notation to `_i` for both frames; aligns with the parallel "virtual image position / virtual image orientation" parameter-naming epoch locked 2026-05-03.

**Banned words / phrases (replace if seen):**

- "landmark", "the four landmarks" → "the four image feature points $\,^\mathcal{C}\boldsymbol{\hat{r}}_i$"
- "corner-like landmarks", "corner-like points" → "image feature points"
- "per-landmark virtual feature point" → "$i$-th virtual image feature point $\,^\mathcal{V}\boldsymbol{\hat{r}}_i$"
- "$k$-th feature point" (for virtual frame) → "$i$-th virtual image feature point"
- "target image feature points" → "image feature points" (drop "target")
- "target image point" (for the centroid in either frame) → "image point" / "virtual image point"

**Where this came up (2026-04-30):**
A draft of §II.A used "landmark" three times in item (i) ("four target landmarks", "the de-rotation to each landmark", "per-landmark virtual feature point"). The user pointed to their existing canonical sentence in §S1 and asked to stick to that style. All three occurrences in `control_formulation.tex` were replaced.

**Related conventions:**

- Three target image parameters: `feedback_target_image_parameters.md` ($\boldsymbol{s}$, $\alpha$, $\boldsymbol{h}$).
- Image-plane axis vs. component naming: `feedback_image_plane_axis_vs_component.md`.
- Corner-point naming (related but separate convention): `feedback_corner_points_naming.md` --- "corner point" was previously banned in favour of "feature point" / "virtual feature point", which aligns with the present rule.
