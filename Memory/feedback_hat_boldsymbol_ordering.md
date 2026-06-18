---
name: Hat-then-boldsymbol ordering convention
description: Locked 2026-05-07. Use `\hat{\boldsymbol{r}}` (hat outside, boldsymbol inside), NOT `\boldsymbol{\hat{r}}`. Both forms render identically in LaTeX, but only the hat-outside form propagates bold through the accent in matplotlib mathtext (used by all 5 plot scripts). Convention applies paper-wide for tex consistency with figure legends.
type: feedback
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---
**Rule.** Whenever a hat-accented bold symbol is needed:
- ✅ `\hat{\boldsymbol{r}}` — hat outside, boldsymbol inside.
- ❌ `\boldsymbol{\hat{r}}` — bold outside, hat inside.

Applies to:
- All `.tex` files in `Soft_Precise_Landing/` (main paper + supplement).
- All matplotlib plot scripts under `scripts/`.

## Why

Both forms render identically when typeset by real LaTeX — they produce a bold $r$ with a hat. But matplotlib's mathtext renderer (which all the figure-generation scripts use, since LaTeX is not on the user's PATH) treats the two orderings differently:

- `\boldsymbol{\hat{r}}` — mathtext renders the hat correctly but **fails to propagate bold through the accent**. The `r` appears non-bold, the hat appears non-bold.
- `\hat{\boldsymbol{r}}` — mathtext renders the bold first, then applies the hat as an accent. Both are bold.

This was discovered 2026-05-07 when the user noticed `\boldsymbol{\hat{r}}_i` legend entries in `plasmc_outer_funnel.pdf` rendered with non-bold `r` even though `\boldsymbol{p}_1(t)` in the same legend was correctly bold.

## What got swapped on 2026-05-07

| File | Replacements |
|---|---|
| `control_formulation.tex` | 13 |
| `results.tex` | 1 |
| `supplemental.tex` | 11 |
| `manuscript.tex` | 0 (already used the correct form) |
| **Total** | **25** |

All instances were `\boldsymbol{\hat{r}}` → `\hat{\boldsymbol{r}}`. Other letter accents (`\bar`, `\dot`, `\tilde`) follow the same convention if they ever appear with `\boldsymbol`.

## How to apply

1. **When writing new tex**: always type `\hat{\boldsymbol{X}}`, never `\boldsymbol{\hat{X}}`.
2. **When reviewing**: any `\boldsymbol{\hat{...}}` that slips in should be flagged and swapped.
3. **In figure script labels**: same rule applies — for the legend to render correctly via mathtext, use `\hat{\boldsymbol{X}}`.
4. **Other accents** (`\dot`, `\bar`, `\tilde`, `\vec`): same ordering — accent outside, boldsymbol inside.

## Related conventions

- `feedback_image_feature_naming.md` — naming of `\hat{\boldsymbol{r}}` etc. (epoch-3, 2026-05-04).
- `feedback_funnel_symbol_convention.md` — symbol conventions for funnels.
- `feedback_image_plane_axis_vs_component.md` — image-plane axis vs component naming.
- `reference_frames_planes_figure.md` — frame conventions; the figure caption uses `\hat{\boldsymbol{r}}` already.

## Helper

A one-shot regex helper was used to do the swap (`scripts/_swap_hat_boldsymbol.py`). It was deleted after use; if you ever need to repeat the audit, the regex is:

```python
PATTERN = re.compile(r"\\boldsymbol\{\\hat\{([^{}]+)\}\}")
REPLACE = r"\\hat{\\boldsymbol{\1}}"
```

This only handles single-level inner content (no nested braces). Should suffice for `\boldsymbol{\hat{r}}` and similar simple patterns; multi-character or accented inner contents may need manual review.
