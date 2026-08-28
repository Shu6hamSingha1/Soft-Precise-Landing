---
name: feedback_relative_flatness_needs_absolute_gate
description: "A 'signal has stopped changing relative to its own recent history/minimum' check is never sufficient alone to detect a terminal/settled state (touchdown, convergence, etc.) -- it only proves local stationarity, not proximity to the actual target state. Always pair it with an absolute-value gate. Found via two real bugs in controller.py's GT touchdown detector (2026-08-26, 2026-08-28)."
metadata:
  node_type: memory
  type: feedback
  originSessionId: 8e3cbb0e-2db1-4a43-9a72-a13e19dca7eb
  modified: 2026-08-28T08:00:48.572Z
---

Two consecutive bugs in `controller.py::_touchdownDetect`'s GT-feedback path (see
[[project_20260828_ic_motion_batch_gt_touchdown_and_kappa_ratchet]] for the full trace)
were both instances of the same class of mistake:

1. **First version**: compared current depth to the running MINIMUM depth seen so far.
   Trivially true almost every frame during an ordinary smooth descent (adjacent samples
   of a continuously-decreasing signal are always close together) -- latched at
   depth=4.395m, right after arming.
2. **Second version** (after adding a windowed-rate + progress-since-arm check, genuinely
   fixing bug 1): still had no requirement that depth was actually SMALL. An ordinary
   mid-descent deceleration (e.g. the vehicle transiently slowing its vertical closure
   while correcting an unrelated lateral overshoot) satisfies "flat + progressed since
   arming" at ANY altitude. Fired at depths from 0.3m to 3.3m across an entire 11-run
   batch, undetected because the outer test harness's own acceptance band happened to
   overlap some of those false depths by coincidence.

**Why:** both versions tested LOCAL stationarity (has the signal stopped moving relative to
itself / its recent trend) without ever checking GLOBAL proximity (is the signal actually
near the value a genuine terminal state would have). A signal can plateau for many reasons
unrelated to reaching its target -- correcting an unrelated error axis, a control transient,
noise -- and any such plateau will satisfy a purely-relative flatness test.

**How to apply:** any future "has X settled / stopped / converged" detector -- touchdown,
IC convergence, funnel-residency checks, anything using a runningmin/max + rate-flattened
pattern -- needs BOTH: (1) the relative/rate-based flattening test (this is what actually
tells you it's not still actively changing), AND (2) an absolute-value gate confirming the
current value is near where the true terminal state should physically be. Neither alone is
sufficient. When adding (2), prefer a value already documented/measured elsewhere in the
codebase (e.g. the ~0.1-0.15m real touchdown floor already established in `gt_feedback.py`'s
Z_REG derivation comments) over guessing a new one.
