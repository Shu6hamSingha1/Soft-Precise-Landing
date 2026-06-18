---
name: cooperative-vs-noncooperative-targets
description: "Locked 2026-05-26. Definitions of cooperative vs non-cooperative targets in autonomous-landing literature, the active/passive spectrum, and where VDF-ASMC sits (cooperative but scale-free)."
metadata: 
  node_type: memory
  type: reference
  originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---

## Cooperative target
The target **helps the UAV detect or localize it**, by design. Two flavors:
- **Active cooperation** — target emits something the UAV senses: RF transponder, AIS beacon (ships), blinking IR LED, GPS broadcasting its own position over a data link.
- **Passive cooperation** — target carries engineered visual features for reliable extraction: AprilTag, ArUco, "H" landing-pad markings, retroreflectors, painted checkerboard, IR LEDs in known geometry.

Defining property: *the target was instrumented for the landing task* — detection is largely solved by design.

## Non-cooperative target
The target **does nothing to aid detection**. The UAV must extract whatever natural features happen to be present — texture, edges, learned object signatures — via visual SLAM, CNN-based detectors, KLT trackers on natural points, etc. Inherits feature-detection failure modes (occlusion, illumination, motion blur) as first-class problems.

Examples: unmarked ship deck, moving truck in natural appearance, debris in a disaster zone, emergency landing site.

## Where each sits in the autonomous-landing literature
- **Almost all** experimental "landing on moving platform" papers (lin2022, cho2022, zhang2026, paris2020, bouaiss2022) use **cooperative** setups — fiducial marker or visual pattern on the platform.
- A small subset (some bouazza2025-style relative-state estimation work, ship-deck papers using horizon + deck edges) push toward **non-cooperative**, usually by trading the marker for a CNN or geometric scene prior.

## VDF-ASMC's position
VDF-ASMC assumes a **cooperative** target — fiducial markers in practical deployment. See [[feedback_fiducial_marker_scope]]: do **not** claim "marker-free" or "non-cooperative" as a paper differentiator.

Acceptable framings for the paper's lane:
- monocular
- scale-free / no metric depth or velocity estimate
- low-SWaP single-camera platform
- GPS-denied
- unknown target depth / motion / dimensions

The contribution is: cooperative-target setup, but with **metric-scale knowledge stripped away**. The target is cooperatively *identifiable*; its *metric* state is not assumed known.

## Related memories
- [[feedback_fiducial_marker_scope]] — forbidden framings ("marker-free", "non-cooperative without markers").
- [[feedback_citation_verification_discipline]] — verify cite claims about target cooperation against actual papers.
- [[feedback_lit_review_category_framing]] — soft / static-soft-precise / moving-soft-precise taxonomy (target cooperation is orthogonal to this axis).
