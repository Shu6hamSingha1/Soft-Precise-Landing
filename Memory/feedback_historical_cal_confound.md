---
name: historical-cal-confound
description: "CORRECTION (user, 2026-06-03): the historical ~2000 reps' failures were NOT primarily lag — they were run with a 2-13x broken output calibration. All 'lag is the floor / gains can't fix it' conclusions (phases 1-5, coord descent, frontier) are cal-contaminated and must not be cited as lag evidence."
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 7faf44bf-c5f1-4b57-a701-f6d868abfdc1
---

**User correction (2026-06-03): "One major issue was incorrect output calibration"** — applied to my claim
that the historical ~2000 reps attribute the failure floor to loop lag. **The user is right; the historical
attribution is confounded.**

**The numbers:** the entire historical campaign era (2026-05-12 → 06-01: Big Sensitivity, coord descent,
precision-softness frontier, Phases 1-5) ran with the May-12 cal, which under-reported every channel:

| Channel | under-reported by | controller's effective gain |
|---|---|---|
| s (centroid error) | 1.9× | outer PID at **0.53×** design |
| h_x/h_y (lateral flow) | 4.7× / 3.8× | lateral SMC at **0.21×** design |
| h_z (vertical flow) | **13.2×** | vertical SMC at **0.08×** design |
| w (angular flow) | 2.5–3.0× | — |

The historical gain campaigns searched ±50% to ×2 around this mis-scaled operating point — **the cal error
(4.7–13×) was outside every search range**. "No gain change reaches SP" was true only of a controller that
could never reach its design point.

**What survives:** the lag *measurements* (pitch rate-loop 38 ms, roll/pitch 52–61 ms, yaw 287 ms) are from
impulse tests on IMU/telemetry — cal-independent, still valid. The lag is a real bandwidth constraint that
*shapes gain choices* (it is why the validated config uses K_rp=1.4, yaw gains 0.2).

**What does not survive:** "lag is the complete explanation" (phase 4), "the 16× gap IS the lag" (phase 2),
"no per-axis gain change reaches a repeatable SP basin" (coord descent), the precision-softness frontier's
location, and the ~0.4 m IC1 xy floor. All were measured on a 2–13× gain-starved controller.

**The proof:** with the correct (multisine) cal + gains matched to the measured lag, the same plant achieves
28% SP @ 10 cm and 1.8 cm best precision (2026-06-03 campaign) — an order of magnitude past the "floor."

**How to apply:** never cite phases 1-5 / coord-descent / frontier memories as evidence about lag limits
without this caveat. The honest causal statement: historical failures = broken cal (dominant) + lag
(real, secondary, still present) + the gain mismatch both of them produced.
