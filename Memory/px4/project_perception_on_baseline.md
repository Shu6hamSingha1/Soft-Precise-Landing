---
name: project_perception_on_baseline
description: "FIRST perception-ON stationary IC gate with the GT-FB-tuned baked config (2026-07-03, aruco world, GT_FEEDBACK OFF, IC1-5 n=5). Result: config transfers STABLY (0 fly-aways/25, no divergence), lands sub-meter on IC1-4 (xy_med 0.19-0.31, best 0.005), but target_lost=True on ALL 25 (mid-descent feature staleness > 1s grace -> open-loop fallback). IC5 short-runway = systematic 2.9m miss (marker at FoV edge at the off-center 3m start -> never acquired -> descends open-loop onto the uncorrected 2.83m offset). Binding perception gap = FEATURE-FRESHNESS CONTINUITY + off-center low-altitude ACQUISITION, NOT landing precision (which transfers) and NOT stability."
metadata: 
  node_type: memory
  type: project
  originSessionId: 3c2f4c67-05c1-4e6f-966b-0e62018fc8a7
---

**FIRST perception-ON validation of the GT-FB-tuned config (2026-07-03).** Ran the stationary
aruco IC gate with **GT_FEEDBACK OFF** (real ArUco + LK optical-flow pipeline) — the pending
"perception-ON validation" after the whole GT-FB tuning campaign (19/25 GT-FB SP, commit-off
bake, etc.). Bundle `test_data/ICValidation/20260703-163608/`.

## Result (IC1-5 n=5, 25 reps)
| IC | xy_med | xy range | target_lost | fly |
|----|--------|----------|-------------|-----|
| IC1 (0,0,5)   | 0.19 | 0.11–0.53 | 4/5 | 0 |
| IC2 (2,2,5)   | 0.22 | **0.005**–0.56 | 5/5 | 0 |
| IC3 (-2,2,5)  | 0.31 | 0.22–0.92 | 5/5 | 0 |
| IC4 (2,2,7)   | 0.26 | 0.12–0.28 | 5/5 | 0 |
| IC5 (2,2,3)   | **2.93** | 2.78–2.94 | 5/5 | 0 |

## The three load-bearing findings
1. **The config transfers STABLY — 0 fly-aways / 25, no divergence.** The GT-FB tuning
   (commit-off, VDS_KF, W_U_MAX=2.0, etc.) does NOT blow up perception-ON. Big de-risk:
   the Z_REG=0.2 harness fix has no perception analog, yet nothing detonates.
2. **Landing PRECISION transfers — IC1-4 land sub-meter, best 0.005 m** (IC2r2 0.005/0.016,
   IC2r4 0.028/0.015). When the feature is fresh, the closed-loop approach is good.
3. **The binding gap is PERCEPTION CONTINUITY, not control.** `target_lost=True` on ALL 25
   (`terminal_perception_loss=False` — lost MID-descent, not under-airframe): the image feature
   goes STALE > the 1 s MARKER_LOSS_GRACE, tripping landing_test's OPEN-LOOP fallback (zero
   body-rate + constant thrust). Per the project rule, target_lost = failure regardless of
   xy/vel — so 0/25 "SP" strictly, though IC1-4 land fine because the open-loop fallback on a
   STATIONARY target lands where it already was (converged).

## IC5 = the qualitative failure (marker ACQUISITION, not continuity)
IC5 lateral NEVER converges: init_lat 2.86 → stays 2.85/2.88/2.91/2.93 at alt 3/2/1/0.5 m →
lands at the uncorrected **2.83 m initial offset** (=sqrt(2²+2²)). Mechanism: off-center (2,2)
at only **3 m** puts the marker ~2.83 m lateral = at the downward-cam FoV edge (half-width
≈3·tan(0.87)=3.6 m), where ArUco can't decode the steep/edge marker → feature never fresh →
control never closes → open-loop straight down. Short 5.4 s descent gives no runway to recover.
(IC1-4 acquire from the higher/centered starts, converge, then lose it LATE.)

## Perception-phase agenda (set by this baseline)
- **P1 — feature-freshness continuity** (all ICs): keep ArUco decoded / KLT-bridged through the
  descent so control stays closed-loop to touchdown (no open-loop fallback → the sub-meter
  landings become real SP). The KLT bridge (MARKER_KLT_MAX_STEPS=20) is NOT spanning the gap.
  Characterize WHEN/at what altitude the feature goes stale, then decode-robustness + KLT.
- **P2 — off-center low-altitude acquisition** (IC5): the marker at the FoV edge from a low
  off-center start isn't acquired. Either decode-at-edge robustness, or IC5 is a genuine
  FoV-geometry limit (marker near/over the FoV edge) to characterize honestly.
- **NOT the priority:** landing precision (transfers) and stability (no fly-aways).

Relates [[feedback_terminal_descent_loom_overreport]] (terminal loom), the perception findings
in `docs/PERCEPTION_FLOW_FINDINGS.md` (flow honest at altitude, LK dynamic range ~2 m/s,
decode/track AVAILABILITY is the limit — consistent with this continuity finding),
[[feedback_marker_detection_stale]], [[klt-marker-fallback]]. Continues the GT-FB campaign
([[project_why_sp_achieved]]).
