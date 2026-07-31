---
name: project_20260730_five_fixes_ic_validation
description: "IC1-5 stationary validation (n=3, IC5 n=8) of the five perception/control fixes ported from a parallel Windows/hardware session on 2026-07-30 -- no regression found; IC5's one 19.5m outlier is within its known heavy-tailed historical variance (p95-p99), not new."
metadata: 
  node_type: memory
  type: project
  originSessionId: 691885cb-f375-4354-a734-54384181d77a
  modified: 2026-07-31T08:22:08.854Z
---

On 2026-07-30/31, a parallel Windows-side Claude session (working against real Pi hardware
telemetry + the same shared repo) ported/authored five fixes into PX4_Gazebo/src, found while
investigating the same moving-target/CBF-starvation problem this session was independently
chasing on the rover:

1. RANSAC-bounds + confidence-lockup self-heal (`b420d3a`, pre-existing, ported from Pi).
2. `CBF_CORNERS_STALE` property (controller.py) + `feature_fresh` AND-gate (landing_test.py) --
   exposes when cbf_corners (what the visibility CBF reads) has been unavailable 30+ frames,
   independent of the higher-level TARGET_IS_VISIBLE/FEATURE_IS_STALE signals landing_test.py
   already watched (which can report "fine" while the CBF is starved).
3. Held-out validation-triggered map reset (img_data.py) -- cross-checks the map's own
   prediction against a fresh ArUco decode (a signal map_confidence itself can't see, since
   map_confidence is computed from the same possibly-broken homography); forces a full
   `_full_reset()` + re-bootstrap on large disagreement (default PLANAR_MAP_VALIDATION_RESET_PX=30px).
4. Confidence-weighted loop-closure correction (planar_map.py/img_data.py) -- near-edge/
   ill-conditioned decodes now BLEND into the map proportionally (`_markerEdgeMarginScore *
   _quadConditionScore`) instead of being discarded outright.
5. **Kappa/integral freeze on staleness** (controller.py) -- kappa's RK5 integration now
   freezes while `CBF_CORNERS_STALE`; the `is_e_n`/`izeta` trapezoidal integrators freeze on
   any `FEATURE_PTS_FRESH=False` frame. This is the direct fix for the a_u-explosion/kappa-
   ratchet mechanism (dkappa/dt = theta*N*G*|sigma| - N*P*kappa runs away when sigma stays
   saturated because there's no valid measurement, not real tracking error) that this session
   independently traced by hand in a rover rep (kappa 0.3->7.5, a_u exploding to 350 m/s^2).
   Confirmed on 2 of 4 real hardware flights (kappa 0.75->16-18, a_u 59-180) and reproduced in
   Gazebo (kappa 0.75->3.13, a_u 112.6) by the Windows session.

Also: an exit-code bug fix (landing_test.py) -- RuntimeError/Exception during a flight were
always swallowed, so a genuine mid-flight crash always exited 0 ("[retry] SUCCESS"). Doesn't
change how to read outcomes (always use SoftPrecise from Ground_Truth.npy, never trust the
retry wrapper's text), but the retry harness's own auto-retry-on-flake logic will now actually
see real crashes as non-zero exit.

**Validation run same session:** IC1 n=3 (mean xy=0.369, max=0.726, 2/3 soft, 1/3 precise --
consistent with the historical mean=0.21/max=1.85 reference). IC2-4 n=3 each: no crashes,
0 soft/precise but xy_err in a plausible 0.12-1.82m range. IC5 n=8 (3+5): 0/8 precise, one
19.5m outlier (rep4) alongside 0.30-5.18m for the rest.

**IC5 outlier check:** pulled all 447 historical landed=YES IC5 reps
(`grep -h "^IC5" test_data/ICValidation/*/summary.tsv`) -- median=0.78m, p90=3.30m,
p95=5.35m, p99=26.4m, all-time max=231.9m. The 19.5m rep falls at roughly p95-p99 (8/447
historical reps >=19.5m) -- a rare but well-precedented tail event for this specific
historically-fragile IC (heavy right tail, mean 2.58m vs median 0.78m), NOT evidence of a
new regression. The rep4 trace itself also doesn't match the kappa-ratchet signature (kappa
stayed near its 0.5 initial value, s_e_n stayed <0.85, well inside the funnel; the a_u spike
was a first-second reaching-term transient) -- consistent with IC5's own documented character
("only 3m altitude -- fast descent may not give IBVS enough time to converge"), not the
mechanism fix #5 targets.

**Why:** establishes the five 2026-07-30 fixes as validated-safe against the stationary
IC1-5 baseline before trusting them for further rover/moving-target work.

**How to apply:** don't re-litigate IC5's poor precision as a new problem -- it has always
been the weakest IC (0/15 precise in the last pre-fix 07-28 sample too). When citing "is this
outlier normal for IC5", pull the full historical summary.tsv distribution (462+ reps
available) rather than judging off a small n. The five fixes above are safe to build on for
the rover coast-hold/moving-target investigation ([[project_rover_baseline_stale_2026-07-30]]).
