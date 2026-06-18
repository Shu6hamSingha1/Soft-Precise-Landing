---
name: Per-feature desired symbol convention — `_{i,\text{d}}` order
description: Locked 2026-05-07. The per-feature desired image feature point uses subscript order `_{i,\text{d}}` (feature index `i` first, then desired qualifier `d`), and is FRAME-AGNOSTIC (no `^\mathcal{C}` or `^\mathcal{V}` prefix), matching the paper's convention for the desired centroid `$\hat{\boldsymbol{r}}_\text{d}$`.
type: feedback
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---
## Rule

Per-feature desired image point: `$\hat{\boldsymbol{r}}_{i,\text{d}}$`

- Subscript order: **`i` (feature index) first, then `\text{d}`** (desired qualifier).
- **No frame prefix** (no `\,^\mathcal{C}` or `\,^\mathcal{V}`).
- Matches paper's existing convention `$\hat{\boldsymbol{r}}_\text{d}=\boldsymbol{0}$` from §III.A.1 (desired centroid is also frame-agnostic).

Wrong forms:
- ❌ `$\,^\mathcal{C}\hat{\boldsymbol{r}}_{i,\text{d}}$` — has frame prefix; desired symbols in this paper are frame-agnostic.
- ❌ `$\hat{\boldsymbol{r}}_{\text{d},i}$` — wrong subscript order (`d` first then `i`).
- ❌ `$\hat{\boldsymbol{r}}_\text{d}$` — that is the centroid, not per-feature.

## Why frame-agnostic for desired

Actual feature points carry a frame prefix because the same physical 3-D point has different pixel coordinates in the C and V frames (de-rotation differs). The desired pose has the centroid on the optical axis with no relative rotation — at the desired pose, $\,^\mathcal{C}\hat{\boldsymbol{r}}_{i,\text{d}} = \,^\mathcal{V}\hat{\boldsymbol{r}}_{i,\text{d}}$ exactly. So the prefix carries no information for the desired symbol and is dropped, mirroring the centroid case.

## Why `i` before `d`

User explicitly chose `_{i,\text{d}}` (not `_{\text{d},i}`) on 2026-05-07. The order puts the feature index closest to the symbol body, with the descriptor (`d` for desired) outside.

## How to apply

- **Figure scripts**: legend label `r"$\hat{\boldsymbol{r}}_{i,\text{d}}$"`.
- **Tex prose**: `$\hat{\boldsymbol{r}}_{i,\text{d}}$` (currently used only in the combined plot legend; if the paper later defines this symbol explicitly, follow this convention).

## Related conventions

- `feedback_image_feature_naming.md` — actual feature points `$\,^\mathcal{C}\hat{\boldsymbol{r}}_i$` (with frame prefix); centroid `$\,^\mathcal{C}\hat{\boldsymbol{r}}$` (no `_i`).
- `feedback_hat_boldsymbol_ordering.md` — `\hat{\boldsymbol{r}}` order (accent outside, boldsymbol inside) for mathtext compatibility.
- `feedback_target_image_parameters.md` — three image parameters convention.
