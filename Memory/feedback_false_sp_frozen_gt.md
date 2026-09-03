---
name: feedback_false_sp_frozen_gt
description: "The SoftPrecise flag can be a FALSE POSITIVE when the GT UAV Pose is degenerate (frozen at IC then reset to origin → xy_err~1e-21 trips 'precise'). 1 found (BootstrapFix rep6 = Landing_Test 'Wed Jun 10 01-22-38', NC48d). It was the ONLY sub-10cm rep in all 101 Jun-9/10 recordings → NO genuine SP in the saved honest-cal (R3) data; the headline NC47e '1/5 SP at 0.03m' is unverified. Always sanity-check an SP against its trajectory."
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 7415f420-9591-41b1-8349-bb9361a8dc82
---

A landing's `SoftPrecise{precise,soft}` flag (in `Ground_Truth.npy`, computed by `landing_test.py`) can be a **false positive** when the GT pose stream is degenerate.

**Mechanism:** `xy_err` = lateral distance from the final/touchdown `UAV Pose` to the target. If the GT `UAV Pose` **freezes at the IC pose** (the drone "never moves" in the log) and then **resets to the origin** `(0,0,~0)` at recording end (model despawn / subscriber glitch), `xy_err`→~1e-21, which trips `precise` (and `soft` if the logged rel_vel < 0.2). The drone never descended; the SP is an artifact, not a landing.

**Found (2026-06-10 audit):** exactly **one** — `BootstrapFix_n21/rep6` = `Landing_Test/Wed Jun 10 01-22-38 2026` (trial NC48d): pose frozen at `(0.153, −0.046, 5.012 m)` for ~25 s, snaps to origin at frame 1517, `xy_err=5.7e-21`, flagged precise+soft. Marked FALSE in the bundle `summary.tsv` (flags flipped 1→0) + `FALSE_SP.md` in both the rep dir and the raw `Landing_Test/` recording dir.

**Implication (important):** it was the **only** sub-10 cm rep across all 101 Jun-9/10 recordings → **there is no genuine SP in the saved honest-cal (R3) data.** The headline **NC47e "1/5 SP at 0.03 m"** (cited in `.ods`, [[feedback_dterm_outer_funnel_analysis]], [[reference_tuning_trajectory]], `docs/PARAMETER_ANALYSIS.md`, the tune-plasmc skill) has **no saved recording** behind it — the gamma_s window's real min is 0.569 m — and is likely the same artifact class. The `.ods` NC47e SP cell is marked `1 (UNVERIFIED)`.

**Why:** SP rate is the project's headline metric; a frozen-GT artifact silently inflates it and can manufacture a "first honest-cal SP" that never happened.

**⬆ SUPERSEDED IN SCOPE 2026-09-02 — this file's "1 found" is one mechanism, not the
population.** An archive-wide re-score of all 4446 saved runs found **159 of 1192 `precise`
verdicts (13.4 %) computed on a MID-AIR sample**, via a SECOND mechanism: `SoftPrecise`
evaluates the pose at **whatever instant the control loop exits** (`landing_test.py:835`;
its "PX4 reports LANDED here" comment is not enforced), with **no floor on altitude above the
landing surface**, so (a) a run that aborts at altitude scores precise because a CENTERED IC starts
directly above the marker (`xy_err`~0 at t=0, 21 runs), and (b) a GT log that stops
mid-descent scores its last airborne sample as a touchdown (138 runs, 112 still descending,
median last altitude 0.486 m). Concentrated in two eras (Jul 19-23, Aug 22-24); the Aug 28-31
headline gates are clean. `test_data/Final/IC5` is the published example — "PRECISE 0.058 m"
scored at 0.422 m altitude. **FIXED `d6610ea7`** — `landing_test.py` now gates precise/soft
on a verified terminal state (lowest altitude above the LANDING SURFACE <= 0.20 m, surface
from `PLASMC_GT_MARKER_DZ`) and records `not_landed_reason`; `tools/rescore_softprecise.py`
re-scores archived runs non-destructively. Full audit, per-dataset impact, the rejected
candidates and what is still open: [[project_20260902_archive_rescore_false_precise]].

**How to apply:** before trusting any SP, sanity-check its trajectory — did the drone actually **descend** (min alt → ~0)? Is `xy_err` a **physical** number (~0.02–0.06 m), not ~1e-21? Is the GT pose **non-frozen** (position std > 0 over the flight) and not origin-reset (`|final xy| > 1e-6`)? A scan over `Ground_Truth.npy` `UAV Pose` flags these.
