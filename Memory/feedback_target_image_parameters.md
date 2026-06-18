---
name: Three image parameters drive MDF-ASMC
description: Locked 2026-04-30. Parameter naming epoch-3 (2026-05-03): virtual image position / virtual image orientation / optic flow. Vocabulary epoch-2 (2026-05-04): per-feature subscript unified to `_i` for both frames; "target" qualifier dropped from "target image parameters" → "image parameters" and from feature-point / centroid names; moment exponents relabelled `p, q`. Section-structure epoch-2 (2026-05-05): §II.B merged from 3 subsubsections (Position / Optic Flow / Orientation) to 2 (Virtual Image Pose / Optic Flow); s and α now share the Pose subsubsection. The three image parameters (s, α, h) survive — merge is structural-only. See `feedback_no_landmark_term.md` for the full naming table.
type: feedback
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---
**Rule.** MDF-ASMC drives its outer loop from **three** image parameters, all computed from the monocular camera measurements:

| # | Image parameter | Symbol | Information type | Consumer block (Section~III of the main paper) |
|---|---|---|---|---|
| (i) | Virtual image position | $\boldsymbol{s}=[\,^\mathcal{V}\boldsymbol{\hat{r}};\,1]^\top$ | position | Normalized virtual feature PID |
| (ii) | Virtual image orientation | $\alpha$ | orientation | Yaw ASMC + virtual-compass integrator |
| (iii) | Optic flow | $\boldsymbol{h}=\,^\mathcal{V}\boldsymbol{v}_{\text{t/b}}/\,^\mathcal{V}z_\text{t}$ | velocity | Optic-flow funnel + adaptive SMC |

Additionally, the **image feature points** $\,^\mathcal{C}\boldsymbol{\hat{r}}_i$ enter the loop *directly* through the **funnel-margin cone clamp** (Section III-A3) — they are *not* a fourth parameter; they are the raw camera observation from which all three parameters are derived.

## The derivation chain (very important)

The three parameters are derived through a chain, not enumerated separately:

```
^C r̂_i  →  ^V r̂_i  →  ^V r̂  →  s
(image     (virtual    (virtual    (homogeneous,
 feature    image       image       scale-free)
 points)    feature     point)
            points)
                                    
                                    s, α, h  →  controller
                                    ^C r̂_i directly  →  cone clamp
```

| Quantity | Definition | Role |
|---|---|---|
| $\,^\mathcal{C}\boldsymbol{\hat{r}}_i$ | **Image feature point**: pinhole projection of the target on the camera image plane, $i\in\{1..N\}$ | Raw camera observation; **also** consumed directly by the funnel-margin cone clamp |
| $\,^\mathcal{V}\boldsymbol{\hat{r}}_i$ | $i$-th **virtual image feature point**: de-rotation $\,^\mathcal{V}R_\mathcal{C}$ applied and re-projected on the virtual image plane | Intermediate quantity |
| $\,^\mathcal{V}\boldsymbol{\hat{r}}$ | $(1/N)\sum_i\,^\mathcal{V}\boldsymbol{\hat{r}}_i$ --- mean of the virtual image feature points | **Virtual image point** — the position component of $\boldsymbol{s}$ |
| $\boldsymbol{s}$ | $[\,^\mathcal{V}\boldsymbol{\hat{r}};\,1]^\top$ | **Virtual image position** — the controller-input parameter |

## Subscript role lock (vocabulary epoch-2, 2026-05-04)

| Role | Letter | Example |
|---|---|---|
| Per-feature index = dummy summation index ($1..N$) | `i` | $\,^\mathcal{V}\boldsymbol{\hat{r}}_i$, $\sum_{i=1}^{N}$ |
| 3-D-vector component index ($1..3$) | `k` | $h_{e_k}$, $k\in\{1,2,3\}$ |
| Moment exponents | `p, q` | $\mu_{pq}$, $(\cdot)^p(\cdot)^q$ |

## Why three, not four

The user pushed back twice (2026-04-30):

1. First push-back: a draft enumerated four parameters (`^C r̂_i`, `^V r̂`, `α`, `h`). The user pointed out that `^V r̂` is computed from `^C r̂_i` via the de-rotation and centroid, so they encode the same position information.
2. Second push-back: a draft used "their virtual-frame counterpart $\,^\mathcal{V}\boldsymbol{\hat{r}}$" for `^C r̂_i`. This is technically imprecise — the 1-to-1 counterparts are $\,^\mathcal{V}\boldsymbol{\hat{r}}_i$ (per-feature), and $\,^\mathcal{V}\boldsymbol{\hat{r}}$ is their *centroid*, not a counterpart. The user then clarified the focus chain: the controller-input parameter is $\boldsymbol{s}$ (virtual image position), derived from $\,^\mathcal{V}\boldsymbol{\hat{r}}$, derived from $\,^\mathcal{C}\boldsymbol{\hat{r}}_i$. So $\boldsymbol{s}$ is the parameter, and $\,^\mathcal{V}\boldsymbol{\hat{r}}$ and $\,^\mathcal{C}\boldsymbol{\hat{r}}_i$ are intermediate / source quantities.

## Naming history

| Parameter naming epoch | Date | §II.B.1 title | §II.B.2 title | Rationale |
|---|---|---|---|---|
| 1 (initial) | --- | — | "Target Virtual Orientation" | initial draft |
| 2 | 2026-05-03 morning | "Normalised Target Position" | "Virtual Target Orientation" | parallelism via `[adjective] + Target [Property]` pattern |
| 3 (locked) | **2026-05-03 evening** | **"Virtual Image Position"** | **"Virtual Image Orientation"** | parallelism + frame-vocabulary consistency + geometric precision |

**Why epoch 3 supersedes epoch 2.** Under the pinhole camera model, $\boldsymbol{s} = [\,^\mathcal{V}\boldsymbol{\hat{r}};\,1]^\top = \,^\mathcal{V}\boldsymbol{r}_\text{t}/\,^\mathcal{V}z_\text{t}$ **is** the image position in normalized homogeneous coordinates. The "normalized" qualifier was implicit in the pinhole projection itself, so dropping it loses no information. "Virtual Image" framing wins because both $\boldsymbol{s}$ and $\alpha$ are *image-plane* (2D + homogeneous) quantities, not 3D target quantities, "virtual image" matches the paper's frame vocabulary ($\mathcal{V}$, virtual image plane, virtual feature points), and the §II.B.1 / §II.B.2 titles share the "Virtual Image [Property]" pattern.

| Vocabulary epoch (feature-point / centroid terminology + subscripts) | Date | Per-feature term | Centroid term | Subscript |
|---|---|---|---|---|
| 1 | --- | "target image feature point" / "$k$-th feature point" | "target image point" | `_i` (camera) / `_k` (virtual) |
| 2 (locked) | **2026-05-04** | **"image feature point" / "virtual image feature point"** | **"image point" / "virtual image point"** | **`_i` (uniform)** |

**Why vocabulary epoch 2 supersedes epoch 1.** Drops the redundant "target" qualifier (camera observes only the target by construction), unifies the per-feature subscript to `_i` for both camera- and virtual-frame projections (same physical point), and frees `_k` for its other established role as 3D-vector component index (e.g., `h_{e_k}`). Moment exponents previously written `^i, ^j` in $\mu_{ij}$ are renamed to `^p, ^q` in $\mu_{pq}$ to avoid collision with the per-feature index — this is also the standard image-moments notation.

## Section-structure history

| Section-structure epoch | Date | §II.B layout | Rationale |
|---|---|---|---|
| 1 (initial) | --- | **3 subsubsections:** §II.B.1 Virtual Image Position, §II.B.2 Optic Flow, §II.B.3 Virtual Image Orientation | One subsubsection per image parameter; matched by-index to §III.A.1/2/3 (later realised the §III mapping was imperfect anyway — orientation is consumed by §III.B.1 Yaw, not §III.A.3 Acceleration Conditioning) |
| 2 (locked) | **2026-05-05** | **2 subsubsections:** §II.B.1 **Virtual Image Pose** (merged Position + Orientation), §II.B.2 Optic Flow | Static (geometric) vs dynamic (velocity) split; Position and Orientation are sister concepts (both algebraic functions of $\,^\mathcal{V}\boldsymbol{\hat{r}}_i$ — centroid for s, second-order moments for α); Optic Flow is the velocity/dynamics side. The §III parallelism argument was flawed — §II.B/§III.A were never 1-to-1. |

**Important: the merge is structural-only.** The three image parameters (`s`, `α`, `h`) survive intact:

- `s` and `α` now share §II.B.1 (Virtual Image Pose) as paragraphs but remain separately defined symbols.
- `h` (and `w`) keeps §II.B.2 (Optic Flow).
- Block diagram (`block_diagram_v3.pdf`) is **unchanged** — its sub-blocks "Virtual Image Position", "Virtual Image Orientation", "Optic Flow" reflect the *parameter-level* decomposition, which is independent of the prose subsection grouping.
- §III consumer subsections are **unchanged** — `s` → §III.A.1 Virtual Image Point Control, `α` → §III.B.1 Yaw Control, `h` → §III.A.2 Optic Flow Control.

**Label changes (2026-05-05).**

| Old label | New label | Notes |
|---|---|---|
| `virtual image position: section` (subsection) | `virtual image pose: section` | Renamed; same anchor |
| `virtual image orientation: section` (subsection) | *(retired)* | The §II.B.3 subsubsection no longer exists; no external `\ref` users were affected |
| `virtual image position: equation` (eq for `s`) | unchanged | Still labels the homogeneous-augmentation equation |
| `virtual image orientation: equation` (eq for `α`) | unchanged | Still labels the second-order-moments equation |

## Block diagram (updated 2026-05-04)

The yellow "Image Parameters" group in `block_diagram_v3.pdf` (regen'd 2026-05-04) shows *five* sub-blocks: "Image Feature Points" (`^C r̂_i`), "Virtual Image Position (2)" (`s`), "Virtual Image Point" (`^V r̂`), "Virtual Image Orientation (9)" (`α`), "Optic Flow (5)" (emits `h` AND `w`). The diagram displays the *computational flow* from the camera measurement to the controller-input parameters; the conceptual parameter count is still **three** (`s`, `α`, `h`) plus the rotational optic flow `w`. The "target" prefix has been dropped from all sub-block labels per epoch-2 vocabulary, and the new "Virtual Image Position (2)" block makes the homogeneous augmentation explicit (was previously implicit). See `reference_block_diagram_structure.md` for the full block-by-block + equation-number map.

## Naming epoch 4 (2026-06-10, user-ruled): "virtual" dropped from parameter names

"Virtual" is reserved for exactly two terms: **virtual image plane** and **virtual camera frame**. Everything else drops it: "image position" $\boldsymbol{s}$, "image orientation" $\alpha$, "image point" $\,^\mathcal{V}\hat{\boldsymbol{r}}$, "Image Point PID", "image point error", "Image Pose" (§II.B.1 title). Disambiguation between $\,^\mathcal{C}\hat{\boldsymbol{r}}_i$ and $\,^\mathcal{V}\hat{\boldsymbol{r}}_i$ is via frame prefixes and "on the camera image plane / on the virtual image plane"; §II.B states once that all three image parameters are computed from image feature points expressed in the virtual image plane. "Virtual-compass" (yaw integrator) kept pending user ruling; "virtual-camera abstraction" kept (derivative of virtual camera frame). Epoch-3 names below are SUPERSEDED for prose (tables/history retained for context). block_diagram_v3.pdf block labels ("Virtual Image Position/Point/Orientation") need regeneration.

## Abstract-level granularity exception (2026-06-10, user-ruled)

The abstract says "**the two image parameters**" / "regulates **both the** image parameters", referring to {regulated image features ≡ virtual image pose (s + α), optic flow (h)}. The user's reasoning: virtual image position and orientation are *computed from* image features, so "regulating image features" is equivalent to regulating both pose parameters — the abstract operates at the two-group granularity that matches §II.B's two subsubsections (Virtual Image Pose / Optic Flow). Verified: no active prose asserts "three image parameters" numerically, so there is no count collision. Do NOT flag "two image parameters" in the abstract as an error. The fine-grained count in §II+ prose remains three (s, α, h).

## How to apply in prose

- In any prose listing what MDF-ASMC observes / closes the loop on, name **three** parameters: $\boldsymbol{s}$, $\alpha$, $\boldsymbol{h}$. Note that $\,^\mathcal{C}\boldsymbol{\hat{r}}_i$ also enters the loop through the cone clamp.
- The §II.B prose now has **2 subsubsections** (Virtual Image Pose + Optic Flow) — but the parameter count is still **three**. Don't conflate "subsection count" with "parameter count" when describing the architecture.
- Use the long form on first introduction in a section and the short form (where defined) thereafter. See `feedback_no_landmark_term.md` for the full vocabulary table.
- Do **not** call $\,^\mathcal{V}\boldsymbol{\hat{r}}$ "the virtual-frame counterpart of $\,^\mathcal{C}\boldsymbol{\hat{r}}_i$" — it's the centroid of the per-feature counterparts, not a counterpart itself. If the per-feature form is needed, write $\,^\mathcal{V}\boldsymbol{\hat{r}}_i$.
- The word "four" is reserved for the *image feature points* ($i\in\{1,\dots,4\}$ in the simulations), not for the parameter count.
- Only the optic flow has dynamics that couple directly to the body force $\,^\mathcal{B}\boldsymbol{F}_u$ of the rigid-body equation; the position $\boldsymbol{s}$ and orientation $\alpha$ have purely kinematic relations.

## Related conventions

- Camera-frame coordinates use $\,^\mathcal{C}\boldsymbol{\hat{r}}_i$, virtual-frame coordinates use $\,^\mathcal{V}\boldsymbol{\hat{r}}$ (`feedback_image_plane_axis_vs_component.md`).
- "Channel" is forbidden when describing actuation or signal flow --- use "actuation" / "body force" / "command" (`feedback_no_channel_word.md`).
- Target image funnel = $\boldsymbol{p}_1$ on $\,^\mathcal{C}\boldsymbol{\hat{r}}_i$; optic-flow funnel = $\boldsymbol{p}_2$ on $\boldsymbol{h}_\text{e}$ (`feedback_funnel_naming.md`, `feedback_funnel_symbol_convention.md`).
- Block-diagram structure: `reference_block_diagram_structure.md`.
