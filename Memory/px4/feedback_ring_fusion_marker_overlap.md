---
name: feedback_ring_fusion_marker_overlap
description: "ROOT CAUSE of wrong terminal ring loom (2026-07-09) - fixed ring radii (41/79/120/161/199px) physically overlap the marker once MARKER_EXTENT_PX grows past ~160px near touchdown; FLOW_FUSE_RING default flipped OFF (BAKED); the \"ring samples ground / ego-vs-target separation\" design comment was NEVER validated."
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 32b3f473-5219-48ce-b070-323ef9cc309a
---

**Finding (2026-07-09, IC1_rep3 loom-spike trace):** the terminal `h_z` corruption (measured −0.86..−1.9 while GT-implied loom ≈0, flat attitude <1°, marker extent flat) came from the RING loom, fused into the controller-consumed `h(t)` by the then-default-ON `FLOW_FUSE_RING` EKF. Smoking gun: `ring_div=−1.58`/`ring_flow_z=−1.44` were already wrong ONE FRAME BEFORE a marker loss while the corner-only raw flow was still small/correct (+0.028) — ring corruption is independent of decode status.

**Root cause — geometric, quantified (IC1_rep3, extent interpolated onto img_t):** ring stations sit at FIXED frame-relative radii = 17/33/50/67/83% of R_max=240px → 41/79/120/161/199 px. As the marker grows in frame (extent ∝ 1/Z), it physically overlaps the outer ring tiers:

| extent band (px) | mean n_ring | frac \|ring_div\|>0.5 |
|---|---|---|
| 50–120 | 30–31 | 6–9% |
| 120–161 | 26.7 | **53%** |
| 161–199 | **8.9** | **41%** |

Station survival collapses ~70% and the loom goes non-physical exactly when extent crosses the outer radii. Whole radius-tiers (~60 stations each) go bad SIMULTANEOUSLY (threshold event, not isolated outliers) → the MAD median rejection can't save it (bad cohort drags the median). Two mechanisms: (1) depth/surface mixing (stations on marker/platform vs ground violate the uniform-Z coplanarity assumption); (2) ArUco cell texture is aperture-problem-adversarial for LK (locks onto wrong identical-looking cell edges → large discontinuous flow errors).

**Fix BAKED (commit bb0a675): `FLOW_FUSE_RING` default 1→0.** A/B (n=1, same IC1): fused → h_z spike −0.86..−1.9 + endpoint balloon; corner-only → h_z tight −0.29..−0.285, endpoint ≈ min_alt_xy (no balloon). Full baked sweep: IC3 mean xy 12.25→0.248 (the 59m fly-away class gone). Propagation path into lateral: the corrupted h_z entered `a_u_xy` via the loom×flow cross term `−(h·e3)·h` (quadratic in h) in the equivalent/feedforward branch — NOT via zeta_r/reach/switch (those stayed tiny; verified by exact G-diagonal reach/switch/resid decomposition from logged sigma/G/theta/kappa/a_v).

**Why:** this finally acts on [[feedback_ring_loom_hz_terminal_deadend]] ("ring loom at terminal = dead-end, keep OFF") for the FUSION path — that memory only covered ring-COMMIT/loom-RING switching, so the default-ON EKF fusion kept injecting ring loom anyway.

**History correction (user-led):** the code's EKF design comment ("corner=target-relative, ring=ego/ground, ring−corner=target velocity h_tv") is an ASPIRATIONAL design from 6e0b44f (2026-06-06), never empirically validated. The default-ON flip (0ed583d, 2026-06-07) was justified ONLY by "fused h_z == corner h_z (ratio 1.00)" — which on a STATIONARY target (h_tv≡0 by construction) cannot distinguish "ring correctly measures ground" from "ring redundantly samples near/on the target." The ego/ground separation has NO empirical demonstration in the test history. Do NOT treat that comment as fact.

**How to apply:** keep FLOW_FUSE_RING=0. If the ring is ever revived (e.g. for moving-target h_tv), it MUST exclude/shrink radii once MARKER_EXTENT_PX approaches a tier's radius (scale-free: both are pixels), and the ego/ground separation must be validated on a MOVING target where h_tv≠0 is distinguishable. Related: [[feedback_terminal_descent_loom_overreport]] (the older ~4× close-range corner-loom over-report is a separate mechanism).
