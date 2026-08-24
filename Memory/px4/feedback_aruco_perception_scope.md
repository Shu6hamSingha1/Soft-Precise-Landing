---
name: feedback_aruco_perception_scope
description: "Don't run perception-mode (non-GT-feedback) test flights using the ArUco marker — ArUco is comparison-only now, cross-marker + GT-feedback is the active development track"
metadata: 
  node_type: memory
  type: feedback
  originSessionId: db521dfb-a6f1-44ca-b3f6-432e98fc4866
  modified: 2026-08-23T12:41:17.945Z
---

Stop running perception-based (non-GT-feedback) landing tests using the ArUco marker (`MARKER_TYPE=aruco`, default). ArUco is now **comparison-only** — a reference point to benchmark against, not a target for further perception-pipeline debugging/fixing.

**Why:** user directive, 2026-08-23. The active development track is cross-marker (`MARKER_TYPE=cross WORLD=cross_marker`) with GT-feedback (`PLASMC_GT_FEEDBACK=1`). Once GT-feedback for cross-marker is working well, the next step is a **GT-ablation test** (perception-ON cross-marker vs GT-feedback cross-marker, isolating what the ablation costs). After that, ArUco marker work is no longer needed at all — see [[project_marker_roadmap_gt_ablation]].

**How to apply:** don't launch `run_aruco_landing*.sh` / `run_ic_validation.sh` runs without `MARKER_TYPE=cross WORLD=cross_marker` unless the user explicitly asks for an ArUco comparison run. If mid-investigation work surfaces an ArUco-only perception bug (e.g. the 2026-08-23 kappa-ratchet-via-detection-flicker finding, [[project_20260823_kappa_ratchet_detection_flicker]]), it's fine to have found/diagnosed it, but don't keep iterating perception-mode ArUco test flights to fix it — that's now out of scope. Cross-marker + GT-feedback is where further testing effort goes.
