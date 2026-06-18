---
name: Image-plane error symbol naming (MDF-ASMC)
description: Naming convention for $\hat{r}_e$, $\bar{r}_e$, $\hat{r}_d$ and the outer-loop PID. Vocabulary epoch-2 locked 2026-05-04: drop "feature" qualifier (variables describe centroid error not per-feature), align with "Virtual Image Point" parameter naming.
type: feedback
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---
Naming convention for the outer-loop image-plane error signals and the PID that uses them. The variables describe error in the **virtual image point** $\,^\mathcal{V}\hat{\boldsymbol{r}}$ (centroid in $\mathcal{V}$), not per-feature errors.

| Symbol | Long form | Short form |
|---|---|---|
| $\hat{\boldsymbol{r}}_\text{e}$ | virtual image point error | image point error |
| $\bar{\boldsymbol{r}}_\text{e}$ | normalized virtual image point error | normalized image point error |
| $\hat{\boldsymbol{r}}_\text{d}$ | desired image point | — (no short form) |

**Outer-loop PID short tag:** **"Virtual Image Point PID"**.

## Subsection / control-law titles

- §III.A.1: `\subsubsection{Virtual Image Point Control Design}` — locked 2026-05-04
- §III.A.2: `\subsubsection{Optic Flow Control Design}` — locked 2026-05-04
- §III.B.1: `\subsubsection{Yaw Control Design}` — locked 2026-05-04

All three follow the parallel "[Topic] Control Design" pattern (was previously "Control Law"; renamed 2026-05-04 for uniformity).

## Why this naming

- $\hat{\boldsymbol{r}}_\text{e}=\,^\mathcal{V}\hat{\boldsymbol{r}}-\hat{\boldsymbol{r}}_\text{d}$ is the error in the *centroid* (virtual image point), not a per-feature error. The earlier "virtual image feature error" name was imprecise — "feature" connoted per-feature-point quantity, but $\hat{\boldsymbol{r}}_\text{e}$ is the centroid error.
- "Virtual Image Point" matches the centroid name $\,^\mathcal{V}\hat{\boldsymbol{r}}$ from `feedback_no_landmark_term.md`.
- Parallel with "Virtual Image Position" / "Virtual Image Orientation" parameter titles (§II.B.1 / §II.B.2).

## Naming history

| Epoch | Date | Variable name (long) | PID short tag | Section title |
|---|---|---|---|---|
| 1 (initial) | --- | "image-feature error" | "image-feature error PID" | "Image-Feature Error PID" |
| 2 | 2026-04-22 | "virtual image feature error" / "virtual feature error" (body short) | "Normalized Virtual Feature PID" | "Normalized Virtual Image Feature Control Law" |
| 3 (locked) | **2026-05-04** | **"virtual image point error"** / "image point error" (short) | **"Virtual Image Point PID"** | **"Virtual Image Point Control Design"** |

## How to apply

- Use the long form on first introduction in any section; short form thereafter.
- $\hat{\boldsymbol{r}}_\text{d}$ is always "desired image point" — no short form.
- Normalization denominator for $\bar{\boldsymbol{r}}_\text{e}$ is $\boldsymbol{\varphi}_\text{max}=\mathcal{R}/(2f)$ (camera half-FoV in tangent units), not $\boldsymbol{p}_{1_0}$. The symbol $\boldsymbol{p}_{1_0}$ is reserved for the target image funnel initial value (see `feedback_funnel_symbol_convention.md`).
- Subsection / control-law titles share the "Control Design" suffix: §III.A.1 / §III.A.2 / §III.B.1.
- The block-diagram node label, supplement Table~S1 row label, and Corollary 1 cite all use "Virtual Image Point PID".

## What to avoid

- Do **not** revert to "feature error" / "image-feature error" / "Normalized Virtual Feature PID" — those are epoch-1/-2 forms superseded 2026-05-04.
- Do **not** call $\hat{\boldsymbol{r}}_\text{e}$ a per-feature error — it is the centroid error.
- Do **not** introduce new variants ("virtual feature error PID", "image error PID", etc.). The naming is closed.
- Section titles must use "Control Design" (not "Control Law") for consistency across §III.A.1 / §III.A.2 / §III.B.1.

## Related conventions

- Three image parameters and naming history: `feedback_target_image_parameters.md`
- Per-feature points / centroid vocabulary: `feedback_no_landmark_term.md`
- Funnel symbol convention: `feedback_funnel_symbol_convention.md`
