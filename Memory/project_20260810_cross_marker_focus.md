---
name: project_20260810_cross_marker_focus
description: "Project focus is now the cross-marker pipeline; ArUco is comparison-only, not active work"
metadata: 
  node_type: memory
  type: project
  originSessionId: 3600b91d-f44b-4754-86bc-066d9ec45b18
  modified: 2026-08-09T22:36:45.942Z
---

As of 2026-08-10, active PX4/Gazebo perception work is focused on the
**cross-marker** pipeline (`src/cross_marker_perception.py`,
`src/cross_marker_detector.py`, `calibration_data/output_cross/`,
`validation_data/cross_output_*`) — not the ArUco board pipeline
(`src/img_data.py`, `calibration_data/output/`).

**Why:** user directive, 2026-08-10, stated explicitly during an output-
calibration/validation status review.

**How to apply:** don't propose or spend flight batches on ArUco-side
open items (e.g. its own Hz≈0.48 weakness, never revisited with the
better-powered A/B method used on the cross-marker side today) unless the
user asks. ArUco is still fair game as a **comparison baseline** — e.g.
"does the cross-marker show the same Hz-weaker-than-Hx/Hy pattern as
ArUco" is a valid use, but ArUco is not itself the object of investigation
right now. See [[project_cross_marker_hz_regression_bisection_20260810]]
for the live cross-marker thread this focus applies to.
