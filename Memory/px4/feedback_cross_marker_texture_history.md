---
name: feedback_cross_marker_texture_history
description: "Cross-marker background texture was never fully removed -- the marker's own printed texture is alive (hires speckle, adopted 2026-08-09), tuned deliberately low-contrast; what IS restricted is how much of it the flow sampler draws from (thin dilated-mask band around the lines only, not the whole plate). The world ground_plane was never textured for either marker."
metadata:
  node_type: memory
  type: feedback
  originSessionId: 4d44a921-8d4d-4924-a38e-243fbd1cb835
  modified: 2026-08-24T11:32:04.814Z
---

User asked (2026-08-24) "why did we stop using textured background with cross-marker" -- the
premise doesn't fully hold. Two separate texture threads exist, neither ended in a removal:

**1. The marker's own printed texture (baked into `cross_marker.png`) -- still live.**
Directly inspected the live PNG: visible background speckle grain, confirmed as `tools/
make_cross_marker_hirestex.py`'s "hires" texture (adopted 2026-08-09, commit `e9b1c86`).
History: original background measured ~46px autocorrelation length (comparable to the 15x15 LK
window -> aperture/correspondence-ambiguity risk) -> finetex/finetex2/reftex/reftex2 cross-hatch
grid experiments (2026-08-02, `make_cross_marker_finetexture.py`) -> superseded by hires (finer,
independently-generated speckle at 3x native resolution, DELIBERATELY low-contrast: background
mean~194, std~10). So texture wasn't dropped -- it was refined toward SUBTLER noise, specifically
so it wouldn't itself become spurious Hough-line content (the same failure class fixed in
[[project_20260824_cross_marker_montage_overlay_robustness]] for the drone's shadow).

**2. What IS restricted: the flow SAMPLER's access to that texture, not the texture itself.**
`project_20260812_cross_marker_flow_architecture_investigation` memory: unlike ArUco's nested
marker (textured across its WHOLE plate, flow samples arbitrary background texture freely), the
cross-marker's flow point sampling (`_sample_flow_points`/ring sampling in `cross_marker_
perception.py`) is deliberately confined to a thin `dilated_mask` band immediately around the
drawn cross+stub lines (`CROSS_FLOW_CENTER_EXCLUDE_FRAC`, boundary margins) -- NOT the broader
speckled plate. This is a scope-limiting design choice to avoid diluting/contaminating the
marker-relative flow signal, not a texture removal. Consequence (measured, not hypothesized):
cross-marker's tracked-point correspondence noise is ~2.5x higher than ArUco's (median CV 0.747
vs 0.299 across full datasets) -- attributed to this thinner, noisier sampling region, not to
Wx/Wy ill-conditioning (which is comparable between the two markers).

**3. The world ground_plane (outside the marker) was NEVER textured for either marker.**
Checked both `aruco.sdf` and `cross_marker.sdf` directly: flat gray material
(`ambient/diffuse/specular 0.8 0.8 0.8`, no texture map) in both. Not a regression, never
existed.

**How to apply:** don't assume "add more texture" is free -- it directly trades against
detection robustness (more edge content = more Hough-line contamination risk, same mechanism as
the shadow bug). If revisiting cross-marker flow-noise reduction, the live lever is whether the
newly-fixed `_robust_fit_line` pruning (see [[project_20260824_cross_marker_montage_overlay_robustness]])
changes the calculus for WIDENING the flow sampler's eligible region back toward the fuller
plate texture, since contamination rejection is now stronger than when the narrow-band
restriction was first chosen.
