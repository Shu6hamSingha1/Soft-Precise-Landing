---
name: feedback_whole_flight_fit_before_adopting
description: "Don't adopt a point-sampling/perception change from a narrow single-phase diagnostic - always re-derive the whole-flight calibration fit first"
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 3600b91d-f44b-4754-86bc-066d9ec45b18
  modified: 2026-08-10T16:12:13.897Z
---

Before adopting any change to point-sampling, GFT candidate selection, or
similar perception-pipeline parameters, re-derive the WHOLE-FLIGHT
calibration fit (all excitation phases, e.g. `derive_cross_marker_cal.py`
or `derive_board_cal.py`) and check ALL channels, not just the one channel
the change targeted — even if a narrower single-phase diagnostic (e.g. one
axis's raw correlation) looks like a clear win.

**Why:** 2026-08-10, testing `CROSS_FLOW_CENTER_EXCLUDE_FRAC` 0.35→0.55 on
the cross-marker pipeline. A z-phase-only raw Tz-vs-GT correlation check
showed a real, consistent improvement (mean 0.80→0.84, n=5) — but the
whole-flight joint fit revealed a net regression across nearly every
channel (Hz R² 0.48→0.07, Hx 0.69→0.48, Hy 0.76→0.65): the more aggressive
exclusion starved the point pool overall, and that noise increase in the
far-more-numerous x/y/yaw-phase samples outweighed the narrow z-phase gain
the isolated check was measuring. Recommending the change on the strength
of the narrow diagnostic alone would have shipped a real regression.

**How to apply:** treat a narrow single-phase or single-channel diagnostic
as a HYPOTHESIS TEST, not a decision — the actual gate for adopting a
perception-pipeline change is the full multi-phase, multi-channel fit
(and ideally independent validation data too), because gains in the
targeted dimension can come at a hidden cost elsewhere that the narrow
check structurally cannot see.
