---
name: optical-flow-vs-image-velocity
description: Locked 2026-06-09; updated 2026-06-09. h = v/z is called "optic flow" in §I/abstract to match community convention; ṡ is also optical flow (kinematic sense). Coupling is through c̃h, not image Jacobian.
metadata: 
  node_type: memory
  type: reference
  originSessionId: 1d38ebb9-3a55-4a1a-9447-9b57f89010c8
---

## Rule

In the MDF-ASMC paper and related literature, two distinct quantities must not be conflated:

- **Optical flow** = ṡ = time derivative of image position s = apparent motion of image feature points in the image plane. Herisse 2012 explicitly states: *"The time derivative ṗ is the kinematics of the image point, which is also called **optical-flow equations**."* (p. 79, eq. 7–8)

- **Image velocity** = h = Vvt/b / Vzt = v/z = depth-normalized relative velocity between UAV and target, expressed in the virtual frame. Ho 2018 calls the vertical component D = VZ/Z **"flow divergence"**, not image velocity.

## Relationship

From eq. (3) of the paper:

> ṡ = h − w×s − ((h − w×s)·ê₃)s

Optical flow (ṡ) is **derived from** image velocity h and rotational flow w — they are not equal. The image Jacobian Ls recovers h and w from the stacked pixel-velocity vector ṗ (estimation step, eq. 7); it is NOT the source of coupling in the closed-loop control dynamics.

## Coupling in image-based landing dynamics

The dynamics are coupled because:
1. Image position kinematics (eq. 3): ṡ depends on h → regulating h (soft landing) simultaneously drives image position evolution
2. Image velocity dynamics (eq. 8): ḣ contains kinematic coupling term c̃h = cross products of w, ẇ, s, h → image position s feeds back into image velocity dynamics
3. Shared depth-dependent gain β = 1/Vzt → grows unbounded as altitude decreases

This coupling is **absent in simplified formulations**: Herisse 2012 treats the vertical channel as a decoupled 1D system; Lin 2023 uses rotation-invariant circle features to decouple translation from rotation.

## Terminology convention for writing

In **§I (Introduction) and the Abstract**, use community-standard terms that match what IBVS papers use:
- **"optical flow"** (two words, no hyphen) → h = v/z, the depth-normalized translational velocity (what the controller regulates). Use "optical flow" paper-wide — more generic with respect to literature. Never "optic flow" or "optic-flow".
- **"image features"** or **"image position"** → s, the normalized image centroid.
- The abstract phrase "regulate image features or optical flow" is the canonical template — do not rename either quantity in §I or abstract.
- Hyphenated form "optical-flow" is acceptable only as a compound adjective before a noun (e.g. "optical-flow error", "optical-flow funnel").

In **§II onward** (math sections), the precise technical distinction matters:
- ṡ = optical flow (kinematic sense, Herisse 2012 eq. 7–8): "the time derivative of the image point is called the optical-flow equations"
- h = v/z = depth-normalized translational velocity; "optic flow" in §I is an accepted shorthand for this
- **"flow divergence"** = D = vz/z = vertical scalar component (Ho 2018); do not conflate with h

## Why

User decision 2026-06-09: "In abstract we have used 'regulate optic flow or image features' which is generally what IBVS papers do. For better understanding, let us stick to these widely used terminology in Introduction and Abstract." The earlier instruction to use "image velocity" for h in §I is superseded by this decision.
