---
name: project_go_around_progress_gate_20260727
description: "2026-07-27: found and fixed a regression in the 2026-07-26 go-around fix (commit ce513dc) -- unconditional retry burned both attempts on an immediate repeat of the same IC5 DRIFT_OFF, leaving the vehicle stuck near start altitude far more often than pre-fix. Progress-gated retry (commit 7b01231) fixes the waste; IC5's underlying early-marker-loss problem is unfixed and is not a bug."
metadata:
  node_type: memory
  type: project
  originSessionId: 68163648-1a9b-4336-962f-9c4c77471aea
  modified: 2026-07-26T18:44:40.315Z
---

**Regression found.** The unconditional go-around retry ([[project_flowlatreduced_targetlost_gate_20260726]]
item 3) holds the vehicle's CURRENT (already-drifted) lateral position while climbing, then
resumes closed-loop control from that same position. For IC5 -- large lateral offset (2.83m)
at only 3m start altitude, the most oblique/short-runway IC -- this means a retry re-enters
the *exact* geometry that caused the loss, and the marker exits the FoV again almost
immediately, before any descent happens. Measured: pre-go-around IC5 "stuck within 2m of
start altitude" rate was 3/321 reps (0.9%); with unconditional go-around active, 7/10 (70%),
several with WORSE final `xy_err` (up to 13m) than a straight-through descent would have given
-- both attempts spent on a repeat of the same failure, open-loop fallback then starts from
~3m instead of wherever a real (even if imperfect) descent attempt would have reached.

**Fix (commit 7b01231, `landing_test.py`).** A go-around retry is now gated on genuine
descent progress since the previous loss: only spend another attempt if altitude improved by
`LANDING_GO_AROUND_MIN_PROGRESS_M` (default 0.75m) since the last TARGET_LOST event; otherwise
skip straight to the open-loop fallback rather than wasting the second attempt. Validated
live (`IC5_rep5`, `ICValidation/20260726-231227`): go-around attempt 1 climbed to 5m, marker
lost again beyond grace at the same ~3m altitude on re-descent -- gate correctly declined
attempt 2 and fell through to open-loop rather than repeating the failed climb.

**IC5's residual "reason behind target lost" -- confirmed NOT a bug, pre-existing structural
limit.** Post-fix n=8 IC5 run still shows marker loss (`DRIFT_OFF`) firing almost immediately
on closed-loop engagement, at essentially the starting altitude (~3m), before any real descent
progress. Matches the earlier per-onset scan (marker centroid pixel x≈44-56, right at the FoV
edge from frame one). Root cause: IC5's IC asks the controller to close a large lateral error
with very little altitude margin -- the correction itself pushes the marker out of frame
before the vehicle can descend. This is the same "large-offset/short-runway" IC5 limitation
catalogued repeatedly this session (see the IC5 cross-cutting note in
[[project_flowlatreduced_targetlost_gate_20260726]]); today's data shows it strikes at the very
START of the approach, not only terminally. Not resolved by go-around, the leveling fix, or
the progress gate -- all three correctly bound the DAMAGE of a loss but don't address why IC5
loses the marker in the first place. Still open; would need either a wider initial FoV margin,
a slower/more cautious initial correction for large-offset ICs, or accepting this as IC5's
structural difficulty.

**Process note.** Between this and the previous session, an unrelated Pi/Windows-side session
pushed hardware restructuring commits (`f1580db`, `95b0d6b`, disjoint file paths) directly to
`origin/main`. Local push was rejected (diverged history); resolved with a clean `git merge`
(no conflicts, `ort` strategy) before pushing. Worth checking `git fetch`/`git log
HEAD..origin/main` before pushing when a parallel Pi-side session may have been active.
