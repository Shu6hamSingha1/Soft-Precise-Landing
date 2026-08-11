---
name: feedback_diagnose_against_gt_not_internal_reference
description: Diagnose recorded flight signals against properly-computed ground truth, never against the controller's own internal desired/reference value alone
metadata:
  node_type: memory
  type: feedback
---

**Rule:** when diagnosing why a recorded flight (`Control_Data.npy` /
`Ground_Truth.npy` / `Img_Data.npy`) behaved a certain way, compare the
signal in question against an independently-computed ground-truth value —
never conclude a perception/measurement error just because a signal
diverged from the controller's OWN internal desired/reference value
(`h_d(t)` etc.). A mismatch against an internal reference only shows
tracking wasn't perfect; it doesn't say which side (measurement or
reference) was actually wrong. Also always verify timestamp sync directly
between the arrays being compared (print the actual gap, don't assume the
clocks line up) and use the correct depth-regularization convention for
the altitude range — `tools/aggregate_calibration_phased.py::compute_gt_signals`
has no `Z_REG` and hard-`NaN`s below 1m depth; anything touchdown-adjacent
needs `tools/gt_optical_flow.py` / `_compute_gt_flow_zreg` instead.

**Why:** 2026-08-11, diagnosing the cross-marker's first hard landing test.
Initial diagnosis compared perceived `h_z` only against the controller's
desired `h_d_z` and concluded "the sensor is wrong." User caught this —
redone against proper Z_REG-regularized, time-synced GT, the finding held
up but was revealed to be BIGGER than first thought (a systematic
dynamic-range gap near touchdown, not just a single bad frame) — the wrong
methodology would have pointed at the wrong class of fix (a rate-limiter,
which wouldn't have addressed the real problem). See
[[project_cross_marker_hz_regression_bisection_20260810]] and the
`docs/HANDOVER_cross_marker_headless_flight_testing_20260811.md` root-cause
trace for the full example.

**How to apply:** full procedure now codified as the `diagnose-flight-data`
skill (`PX4_Gazebo/.claude/skills/diagnose-flight-data/SKILL.md`) — invoke
it (or at minimum follow its checklist) before reporting any conclusion
about "the perception was wrong" or "the controller commanded X" from
recorded flight data.
