---
name: project_20260811_aruco_ic1to5_comparison_baseline
description: "ArUco IC1-5 landing validation (n=2/IC, 2026-08-11), run as a comparison baseline while active work is on the cross-marker pipeline: all 10/10 landed, 0 fly-aways; IC1 excellent (0.021m mean xy, precise); IC2-5 classic off-center pattern (mean xy 0.98-1.39m, no precise, mostly soft) -- somewhat better than the last full gate on record."
metadata:
  node_type: memory
  type: project
---

**ArUco IC1-5 landing validation, 2026-08-11 (n=2/IC, `scripts/run_ic_validation.sh`,
`IC_LIST="IC1 IC2 IC3 IC4 IC5" N_REPS=2 HEADLESS=1`).** Run explicitly as a
**comparison baseline** while active development is on the cross-marker
pipeline (see [[project_20260810_cross_marker_focus]]) -- not itself an
object of investigation, per the user's standing scope directive.

Results: `PX4_Gazebo/test_data/ICValidation/20260811-103308/summary.tsv`.

| IC | n | soft | precise | mean xy | max xy | mean rel-vel |
|---|---|---|---|---|---|---|
| IC1 | 2 | 1 | 1 | 0.021 m | 0.036 m | 0.085 m/s |
| IC2 | 2 | 1 | 0 | 1.391 m | 2.658 m | 0.363 m/s |
| IC3 | 2 | 1 | 0 | 1.246 m | 2.198 m | 0.682 m/s |
| IC4 | 2 | 1 | 0 | 0.984 m | 1.390 m | 1.304 m/s |
| IC5 | 2 | 0 | 0 | 1.362 m | 2.137 m | 1.292 m/s |

All 10/10 landed, 0 fly-aways/target-lost. IC1 excellent (sub-4cm, precise
both reps). IC2-5 show the standard, well-documented IC1-specificity
pattern ([[feedback_ic_validation]]) -- no precise landings off-center,
mean error ~1-1.4m, but stable and mostly soft, not a new regression.

**Somewhat better than the last full IC2-5 gate on record** (2026-06-04:
IC2=2.21m, IC3=2.0m, IC4=0.88m, IC5=2.37m mean xy) -- plausibly reflecting
the accumulated perception/control fixes since then (e.g. the five
2026-07-30 fixes in [[project_20260730_five_fixes_ic_validation]]), though
this run is only n=2/IC so not a rigorous head-to-head comparison (that
gate was n=5-10/IC).

**Why:** establishes a fresh, dated ArUco reference point to compare the
cross-marker pipeline's own future IC1-5 closed-loop results against, once
that pipeline gets its first landing-controller integration (currently
untested end-to-end -- see [[project_20260810_cross_marker_focus]] /
[[project_cross_marker_hz_regression_bisection_20260810]] for the
perception-only calibration/validation work done so far).

**How to apply:** when a future cross-marker IC1-5 result comes in, compare
against THIS table (same script, same N_REPS, same date-adjacent tuning
state) rather than the older 2026-06-04/2026-07-30 numbers, since those
predate whatever controller/perception state is live now.
