---
name: project-ds-zero-holds-for-cross-marker
description: d_s=0 is better justified for the cross marker than for the paper's N=4 symmetric points — the arm-line intersection is exactly tilt-invariant by homography incidence, and the stub never enters s
metadata:
  type: project
---

Re-derived 2026-09-03 (user question: the paper's `d_s = 0` argument assumes `N=4` symmetric
points; does it cover the PX4 Hough arm-line centroid with its asymmetric stub?).

**Answer: yes, and on STRONGER grounds. Do not add a `d_s` term for the cross marker.**

## The derivation

The marker is planar, so plane→image is a **homography** `H`. Homographies map lines to lines and
preserve incidence. The arms are straight segments in the target plane meeting at the physical
centre `C`, so `H(C) = H(L₁) ∩ H(L₂) = ℓ₁ ∩ ℓ₂` — **the intersection of the projected arm lines
IS the projection of the physical centre, exactly, for any camera pose.** A homography absorbs
arbitrary plate tilt, so platform roll/pitch — the exact perturbation the paper's `d_s` argument
worries about — contributes ZERO, not a second-order residual.

The paper's `N=4` centroid is weaker precisely because homographies are **not affine**:
`mean(H(pᵢ)) ≠ H(mean(pᵢ))`. Hence its "cancels to first order" with a second-order residual
pushed into `d_h`. The cross estimator needs no such step.

**Verified numerically** (asymmetric arm sampling — far end of each arm dropped, as in an oblique
view — plus 0.15 px noise). Intersection error is FLAT in tilt (noise floor only); centroid error
grows monotonically:

| plate tilt | line-intersection | 4-pt centroid |
|---|---|---|
| 0° | 0.076 px | 0.000 px |
| 10° | **0.012** | 0.462 |
| 25° | **0.025** | 1.042 |
| 40° | **0.024** | 1.352 |

## Two structural facts worth keeping

- **The stub never enters `s`.** `center = _line_intersection(line_i, line_j)` consumes only the
  two arm point sets; the stub feeds `heading_deg` / `_disambiguate_angle` (π-ambiguity) and
  alpha's moment. The asymmetry is real but disconnected from the centroid.
- **A line fit is UNBIASED under asymmetric sampling along the line** (losing an arm's far end
  inflates variance/conditioning, not the estimate); a centroid is directly biased by it. This
  explains the code's own rejected experiment — replacing the intersection with the arms'
  inlier-point centroid made it WORSE (rover 28.5%→11.0%, "unequal inlier counts / asymmetric
  spans put that mean systematically off the junction", `cross_marker_detector.py` ~line 266).

## What is genuinely not covered — and only one belongs in d_s

1. **Finite stroke width — second order, absorbable.** The fit runs over a BAND, and homographies
   don't preserve midpoints, so the image band's centre-line ≠ `H`(3D centre-line). Order
   (stroke width / depth) × obliqueness. `_fit_arm_centerline_subpix` (intensity-weighted ridge)
   shrinks it. Same order the paper already absorbs — treat it identically.
2. **⛔ Contamination — an OUTLIER regime that must NOT be modelled as a bounded `d_s`.** When a
   platform edge fuses into an arm the fitted line is *wrong* and the centroid error is unbounded
   (`centroid_mismatch` = 79% of rover failures). Modelling a gross detector failure as bounded
   noise would be wrong; the code already handles it correctly by GATING
   (`CROSS_S_JUMP_GATE`, `centroid_mismatch`) rather than absorbing.
   See [[feedback_cross_detector_contrast_not_darkness]],
   [[project_20260901_rover_cross_perception_diagnosis]].

**For the manuscript:** the paper as written is fine (it describes `N=4` symmetric points). If it
ever describes the cross-marker implementation, give the incidence-preservation argument — it is
exact where the current one is first-order. Audit page:
https://claude.ai/code/artifact/342680b7-f7b2-449c-b06b-3c04c7758a3e
