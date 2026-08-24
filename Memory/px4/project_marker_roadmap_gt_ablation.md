---
name: project_marker_roadmap_gt_ablation
description: "Marker-type roadmap — ArUco is comparison-only, cross-marker + GT-feedback is the active track, next milestone is a GT-ablation test, after which ArUco is no longer needed"
metadata: 
  node_type: memory
  type: project
  originSessionId: db521dfb-a6f1-44ca-b3f6-432e98fc4866
  modified: 2026-08-23T12:41:28.844Z
---

Roadmap (user directive, 2026-08-23):

1. **ArUco marker is comparison-only** going forward — not a target for further perception-pipeline development or debugging. See [[feedback_aruco_perception_scope]].
2. **Active track: cross-marker + GT-feedback** (`MARKER_TYPE=cross WORLD=cross_marker PLASMC_GT_FEEDBACK=1`). Work continues here until it's working well.
3. **Next milestone: a GT-ablation test** — once cross-marker GT-feedback is solid, run the ablation (comparing GT-feedback vs perception-ON cross-marker, or systematically removing GT-feedback components) to characterize what perception costs relative to ground truth.
4. **After the GT-ablation test, ArUco marker won't be needed at all.** Don't invest further effort in ArUco-specific fixes (e.g. the terminal kappa-ratchet-via-detection-flicker bug found 2026-08-23, [[project_20260823_kappa_ratchet_detection_flicker]]) beyond what's already been diagnosed — that fix, if pursued, should target the underlying mechanism (kappa's freshness-gate condition) generically, not be justified by ArUco perception-mode testing specifically.

**How to apply:** when planning test campaigns or deciding where to spend investigation time, default to cross-marker + GT-feedback. Treat any ArUco perception-mode finding as informational/comparison data, not a queue item to fix via more ArUco flights.
