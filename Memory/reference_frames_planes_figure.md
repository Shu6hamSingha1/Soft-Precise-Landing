---
name: frames_planes.pdf — figure structure and labels
description: Reference for the active 3D frames-and-image-planes figure in the manuscript (`Soft_Precise_Landing/Figures/frames_planes.pdf`). Documents what is drawn, the labels in the figure (which differ from per-feature notation in the prose), and the caption convention. Use when editing the figure caption or when mentioning the figure in prose. Updated 2026-05-05 after a figure regen that added the body frame $\mathcal{B}$ explicitly and reorganised the image-plane vectors. Legacy TikZ source is in `Obsolete/Soft_Precise_Landing/frames_planes.tex`; the PDF replaced TikZ inclusion in `control_formulation.tex` L39.
type: reference
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---
**Active file:** `Soft_Precise_Landing/Figures/frames_planes.pdf` (rendered PDF; ~2.4 MB; portrait orientation, ~1:1.4 aspect). Included in `control_formulation.tex` L39 as `\includegraphics[width=\columnwidth]{Figures/frames_planes.pdf}`. Label `frames planes: figure` (used in §II.B intro and §II.B.1 / §II.B.2 paragraph anchors).

**Legacy TikZ source** (no longer compiled): `Obsolete/Soft_Precise_Landing/frames_planes.tex` (8.4 KB).

## What is drawn (top → bottom)

1. **Quadrotor UAV** at the upper-left, with the **body frame $\mathcal{B}$ explicitly drawn** at the UAV: origin $O_b$, axes $X_b$ (forward), $Y_b$ (right), $Z_b$ (down). Forward-right-down convention.
2. **Dashed curve** from $O_b$ to $O_c$ — represents the rigid camera mount on the body. Visually conveys "$\mathcal{C}$ is body-fixed" without overlapping the two frames.
3. **Camera frame $\mathcal{C}$** at $O_c$: axes $X_c, Y_c, Z_c$ (red). Tilted relative to inertial because of body roll/pitch.
4. **Virtual frame $\mathcal{V}$** at $O_v$, **co-located with $O_c$**: axes $X_v, Y_v, Z_v$ (red). $\mathcal{V}$ shares the camera origin ($O_v \equiv O_c$); the axes radiate from the same point with $\mathcal{V}$'s set axis-aligned with inertial yaw. $X_v$–$Y_v$ plane parallel to inertial $X_i$–$Y_i$. (Earlier 2026-05-05 regen drew them slightly offset for visual clarity; the 2026-05-06 regen aligns them at the shared origin.)
5. **Tilted "Image Plane"** (gray quadrilateral) just below the camera, perpendicular to camera $Z_c$; carries axes $\hat{X}_c, \hat{Y}_c$ (with hats — image-plane axis convention from `feedback_image_plane_axis_vs_component.md`). The plane's axis origin is at the lower-left corner.
6. **Horizontal "Virtual Image Plane"** (gray quadrilateral, parallel to inertial $X_i$–$Y_i$ plane), carries axes $\hat{X}_v, \hat{Y}_v$. Axis origin at the lower-left corner.
7. **Image-plane projections** (yellow polygons): one on the tilted image plane, one on the virtual image plane. Each represents the polygon traced by the four feature points; centroids are labelled.
8. **Inertial frame $\mathcal{I}$** at the floor: NED axes $X_i$ (forward), $Y_i$ (right), $Z_i$ (down). $Z_i$ pointing down per NED convention.
9. **Target** (yellow polygon) on the inertial floor.

## Vectors in the figure

Four vectors are drawn (blue arrows in the rendered PDF):

| Symbol | From | To | Meaning |
|---|---|---|---|
| $\,^\mathcal{C}r$ | $O_c$ | target on floor | Position of target in camera frame |
| $\,^\mathcal{V}r$ | $O_v$ | target on floor | Position of target in virtual frame |
| $\,^\mathcal{C}\hat{r}$ | image-plane axis origin (corner of tilted plane) | centroid of yellow polygon on tilted image plane | Centroid of image feature points in camera frame |
| $\,^\mathcal{V}\hat{r}$ | virtual-image-plane axis origin | centroid of yellow polygon on virtual image plane | Centroid of image feature points in virtual frame |

Drawing $\,^\mathcal{C}\hat{r}, \,^\mathcal{V}\hat{r}$ as vectors from the image-plane axis origin (rather than just labels at the centroid) makes the "image position" interpretation pedagogically explicit — they are 2-D position vectors in the image plane. This is a NEW pedagogical choice in the 2026-05-05 regen.

## Important — figure labels vs prose convention

The PDF labels the **centroids** $\,^\mathcal{C}\hat{r}$ and $\,^\mathcal{V}\hat{r}$ (no subscript $_i$). The polygonal yellow patches imply $N=4$ corner points but the per-feature points $\,^\mathcal{C}\hat{r}_i, \,^\mathcal{V}\hat{r}_i$ are not individually marked.

Consistent with `feedback_target_image_parameters.md`:
- Per-feature: $\,^\mathcal{C}\hat{r}_i, \,^\mathcal{V}\hat{r}_i$ (subscript $_i$, used in math derivations and the §II.B.1 intro of the main paper).
- Centroid: $\,^\mathcal{C}\hat{r}, \,^\mathcal{V}\hat{r}$ (no subscript, as labelled in the figure).

Caption convention (locked 2026-05-05): describe the figure in **centroid terms**, mention $N=4$ as the polygon implication. Do NOT claim per-feature points are labelled in the figure.

## What is NOT in the figure

- **Funnel-margin envelope $\boldsymbol{p}_1(t)$** — earlier draft caption mentioned it acting on the camera image plane, but the rendered PDF does not draw it. The 2026-05-05 caption rewrite dropped that mention. If the figure is regenerated to include $\boldsymbol{p}_1(t)$, restore the caption sentence.
- **Per-feature dots/markers** — only centroids are labelled; the polygon shape implies $N=4$.
- **De-rotation matrix $\,^\mathcal{V}R_\mathcal{C}$** — earlier caption referenced it explicitly; current caption simply says "is de-rotated" since the matrix symbol is introduced in §II.B intro and §II.B.1.

## Current caption (2026-05-06, locked, ~70 words)

> Frames and image planes. Inertial $\mathcal{I}$ locates UAV and target. Body frame $\mathcal{B}$ is fixed to the UAV with camera frame $\mathcal{C}$ rigidly mounted on it. Virtual frame $\mathcal{V}$ shares $\mathcal{C}$'s origin and is de-rotated so its $X_\text{v}$–$Y_\text{v}$ plane stays parallel to the inertial $X_\text{i}$–$Y_\text{i}$ plane. Centroids $\,^\mathcal{C}\boldsymbol{\hat{r}}$ and $\,^\mathcal{V}\boldsymbol{\hat{r}}$ of the $N=4$ feature points project on the tilted image plane (axes $\hat{X}_\text{c},\hat{Y}_\text{c}$) and the virtual image plane (axes $\hat{X}_\text{v},\hat{Y}_\text{v}$), respectively.

**Caption history.**
- 2026-05-05: introduced shorter version (~58 words) replacing the original ~110-word draft caption; centroid framing locked; funnel-margin envelope sentence dropped (not drawn in PDF).
- **2026-05-06: body frame $\mathcal{B}$ added** to caption ("*Body frame $\mathcal{B}$ is fixed to the UAV with camera frame $\mathcal{C}$ rigidly mounted on it*") to anchor the now-explicitly-drawn $\mathcal{B}$ axes. Caption now ~70 words across 4 short sentences (per `feedback_short_sentences_no_colons_no_emdash.md`).

## Where the figure is referenced

- `control_formulation.tex` §II.B intro (L57): `(Fig.~\ref{frames planes: figure})` — anchors the virtual-frame definition.
- `control_formulation.tex` §II.B.1 first sentence (L60): `(Fig.~\ref{frames planes: figure})` — anchors the image-feature-point introduction.
- `control_formulation.tex` §II.B.2 first sentence (L81): `(Fig.~\ref{frames planes: figure})` — anchors the virtual image plane.
- `supplemental.tex` §S1-A (L33): "*(Fig.~1 of the main paper)*" — text reference because supplement compiles separately (per `feedback_supplement_cross_refs.md`).

## Regen history

### 2026-05-05 regen (vs original TikZ)
1. **Body frame $\mathcal{B}$ drawn explicitly** at the UAV — origin $O_b$, axes $X_b, Y_b, Z_b$. Previously the body was implicit (only camera-frame axes appeared at the drone).
2. **Dashed curve from $O_b$ to $O_c$** — represents the rigid camera mount on the body.
3. **In-plane vectors $\,^\mathcal{C}\hat{r}, \,^\mathcal{V}\hat{r}$** drawn as vectors from the image-plane axis origin (corner) to the centroid, rather than just labels at the centroid point. Pedagogically explicit "image position vector" meaning.
4. **$O_v$ adjacent to $O_c$** with slight offset — visually distinguishable axes.

### 2026-05-06 regen (vs 2026-05-05)
1. **$O_c$ and $O_v$ now co-located** — drawn at the same point with $\mathcal{C}$'s axes (tilted) and $\mathcal{V}$'s axes (yaw-aligned with inertial) radiating from the merged origin. Mathematically more accurate ($O_v \equiv O_c$ by construction), and visually emphasises the shared origin.
2. **Caption updated** to mention body frame $\mathcal{B}$ explicitly: "*Body frame $\mathcal{B}$ is fixed to the UAV with camera frame $\mathcal{C}$ rigidly mounted on it*" — anchors the $\mathcal{B}$ axes that the figure now draws.
3. **Dashed mount curve refinement** — cleaner curve from $O_b$ to the merged $O_c \equiv O_v$ origin.

## When to revisit

- If the PDF is regenerated to include $\boldsymbol{p}_1(t)$ visualisation, the funnel-envelope sentence in the caption can be restored.
- If the per-feature points $\,^\mathcal{C}\hat{r}_i$ are individually marked in a future regen, switch caption back to per-feature noun phrasing ("$N$ image feature points $\,^\mathcal{C}\boldsymbol{\hat{r}}_i$ project…").
- The body frame $\mathcal{B}$ is now load-bearing in the caption (added 2026-05-06).
- The legacy TikZ source `Obsolete/Soft_Precise_Landing/frames_planes.tex` is preserved for reference but should not be re-included; if a TikZ-based version is needed, regenerate from the new conventions.

## Related conventions

- `feedback_image_plane_axis_vs_component.md` — axes use $\hat{X}_c, \hat{Y}_c$ (with hats) for image plane; $X_c, Y_c, Z_c$ (no hats) for 3D camera frame.
- `feedback_target_image_parameters.md` — per-feature subscript `_i` vs centroid (no subscript) convention.
- `feedback_no_landmark_term.md` — vocabulary epoch-2 (drop "target" qualifier).
- `feedback_figure_label_sync.md` — when figure scripts/PDFs are updated, axis labels and prose must stay in sync.
- `feedback_supplement_cross_refs.md` — supplement uses text references ("Fig. 1 of the main paper") instead of `\ref{}`.
