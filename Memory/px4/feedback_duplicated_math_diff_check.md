---
name: feedback_duplicated_math_diff_check
description: "When a new module deliberately duplicates math/logic from a validated module (rather than importing it), diff the duplicated sections against the original as an early, cheap check before deep independent diagnostics"
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 79646ad7-0ee0-45bb-bd3b-73e43c1470bf
  modified: 2026-08-02T05:15:15.163Z
---

When a module is deliberately written to duplicate math or logic from an existing,
validated module — rather than importing/reusing it, usually to avoid coupling to
the original's internals — **diff the duplicated sections against the original
early**, before reaching for independent statistical/empirical diagnostics.

**Why:** `cross_marker_perception.py` (built 2026-08-01) explicitly duplicated
`img_data.py`'s image-Jacobian math ("duplicated rather than imported so this
module has no dependency on IMG_PROCESSOR's class internals"). It reproduced a
`resolution[::-1]` transpose bug in the image-center computation that `img_data.py`
had already made and reverted back in June 2026 — the fix for that lived only as
an inline comment in `img_data.py`, not as shared/reusable code, a test, or a
memory entry, so it didn't propagate to the fresh reimplementation. The new
module's own docstring even reproduced the same wrong mental model that caused
the original mistake (mislabeling `resolution` as "(h, w), rotated-frame
convention", which is not actually true — see `img_data.py`'s own comment for
why). The bug was symptom-free at build-time validation (hover-only tests have
near-zero translational velocity, so a corrupted flow computation looks the same
as a correct one when there's nothing to measure) and was only found later via a
multi-step diagnostic chain (point count → conditioning → temporal alignment →
per-point fit residual) in a session investigating why a derived calibration
failed to generalize — a direct side-by-side diff against `img_data.py`'s
validated geometry setup would have caught it immediately, before any of that.

**How to apply:** When building or reviewing a module whose own documentation
says it duplicates another module's math (search for words like "duplicated",
"mirrors", "same as X.py"), explicitly diff the duplicated logic against the
original — especially non-obvious conventions (frame rotations, axis order,
index ordering) that needed an explanatory comment in the original, since those
are exactly the parts most likely to be silently gotten wrong in a fresh
reimplementation. Do this BEFORE ruling out other causes via empirical
diagnostics when something in the new module doesn't behave as expected — it's
cheaper and more targeted than working down a list of independent hypotheses.
See [[project_cross_marker_pipeline_20260801]] for the specific incident.
