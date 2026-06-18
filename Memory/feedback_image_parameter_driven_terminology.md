---
name: "Image parameter-driven" not "image-driven"
description: Locked 2026-05-08. When describing MDF-ASMC loops being driven by visual feedback in main paper / supplement prose, write "image parameter-driven loops" (or analogous "image parameter-driven outer loop / inner loop"), not "image-driven loops". The framework's defining feature is that the regulated quantities are the SCALE-INDEPENDENT IMAGE PARAMETERS s, α, h — not raw images and not raw image features.
type: feedback
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---
## Rule

When describing MDF-ASMC loops driven by visual feedback, write **"image parameter-driven loops"** (with the noun "image parameter" hyphenated as compound modifier), not the shorter "image-driven loops" or "image-feature-driven loops".

## Why

The framework's defining contribution (Contribution (b) in Table~I) is that the regulated quantities are the *scale-independent image parameters* $\boldsymbol{s}$, $\alpha$, $\boldsymbol{h}$ — not raw images, not raw pixel features. The phrasing "image-driven" loses this specificity and makes the framework sound generic IBVS. "Image parameter-driven" anchors back to the §II.B / §II.C terminology that the paper has spent lock-in effort on (`feedback_target_image_parameters.md`).

## How to apply

1. **Loops are "image parameter-driven"** — outer loop, inner loop, and the overall framework when contrasted with pose-based / classical IBVS.
2. **Pixel-level feedback paths** — keep the existing names: "Virtual Image Point PID", "Optic Flow Control", etc.
3. **In supplement context (§S1-D contrast)**: "the standard pose-based architecture (which drives inertial pose) is replaced with image parameter-driven loops (which drive $\boldsymbol{s}$, $\alpha$, $\boldsymbol{h}$)".
4. **Don't conflate**: "image-feature-point-driven" is wrong — the controller doesn't drive raw $\,^\mathcal{C}\hat{\boldsymbol{r}}_i$ directly, it drives the scale-independent aggregates derived from them.

## Related conventions

- `feedback_target_image_parameters.md` — three image parameters $\boldsymbol{s}, \alpha, \boldsymbol{h}$ drive MDF-ASMC.
- `feedback_image_feature_naming.md` — virtual image point error naming.
- `feedback_no_landmark_term.md` — vocabulary epoch-2.
