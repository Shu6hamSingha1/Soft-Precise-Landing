---
name: project_20260825_cbf_margin_reserve_fix
description: "Implemented CBF_MARGIN_RESERVE (default 0.0 = unchanged) in cbf_visibility.py + cbf_visibility_aruco.py: Phase 1's FoV box bound was centroid-only (delta_eff=0 by design, deliberately allowing the marker to grow/overflow until decode ACTUALLY fails) -- this proactively reserves a fraction of the marker's own measured footprint (delta2) in Phase 1 too. Mechanism confirmed genuinely engaging (real trajectory divergence, not a no-op). First A/B (n=3/arm) contaminated by a concurrent SITL session (test_data/Rover_AB_harness/ic5_hang_chase_*) -- inconclusive. CLEAN re-run (n=3/arm, confirmed no concurrent SITL): REVERSES the direction -- reserve=0.0 1/3 TARGET_LOST, reserve=1.0 2/3 TARGET_LOST. No clean evidence this fix helps at IC5. NOT baked, left at default 0.0."
metadata: 
  node_type: memory
  type: project
  modified: 2026-08-25T07:16:44.172Z
  originSessionId: 0f9c8dc0-a837-4722-95e7-0ea102167469
---

Same-day follow-up (crossed midnight from 2026-08-24) to [[project_20260824_ic5_perception_fov_margin_gap]], which traced IC5's real-perception failures to a genuine, physical FoV-exit event (`KLT corners left image bounds — stopping fallback`) confirmed at both marker types, and to [[project_20260824_dtheta_az_filter_self_defeating_feedback]]'s finding that `dtheta`/CBF fires 65-98% of frames at IC5 because it's genuinely fighting a thin margin. User directed: "start with giving the CBF/perception a tighter reserve margin."

## Root gap identified in the CBF code itself

Both `cbf_visibility.py` (cross-marker) and `cbf_visibility_aruco.py` (ArUco) compute `delta2` (the marker's own measured footprint: `MARKER_EXTENT_PX/2` for cross-marker, half the corner-array spread for ArUco) every cycle -- but Phase 1 (normal operation, marker decoding successfully) DELIBERATELY doesn't use it in the box bound: `m2 = phi_max only` (`cbf_visibility.py:184` / `cbf_visibility_aruco.py:196`, longstanding comment: *"delta_eff=0: centroid-only barrier; deliberately allow the marker to grow and overflow as the UAV closes in"*). `delta2` is only subtracted from the bound REACTIVELY, in Phase 2, after decode has already failed for `CBF_PHASE2_HYSTERESIS` (default 3) consecutive frames. So the CBF has zero proactive awareness of the marker's own size while it's still tracking -- exactly the gap that lets corners genuinely exit frame before the CBF ever tightens up.

## Fix: `CBF_MARGIN_RESERVE` (both files, mirrored)

```python
_margin_reserve = float(env.get("CBF_MARGIN_RESERVE", "0.0"))
m2 = np.maximum(np.asarray(p_10, float) - _margin_reserve * delta2, 1e-3)
```

Default `0.0` = EXACT prior behavior (confirmed: `tools/validate_cbf.py` still 12/12 PASS unchanged). A nonzero value proactively reserves that fraction of the marker's measured footprint in Phase 1's box bound too, so the CBF constrains `theta` before corners reach the edge, not just after they've already left it.

## A/B at IC5 (ArUco, `WORLD`/`MARKER_TYPE` default, perception-mode, `PLASMC_DTHETA_AZ_GAIN=0` to remove that confound)

`test_data/PerceptionVsGT_IC5/20260824-220500/aruco_margin_reserve*`:

| reserve | rep | outcome | ArUco-lost events | notes |
|---|---|---|---|---|
| 0.0 (baseline) | 1 | `TARGET_LOST`, xy=1.554 | 8 | mid-flight open-loop fallback |
| 0.5 | 1 | `TARGET_LOST`, xy=1.554 | 9 | confirmed genuinely different trajectory (n=332 vs 359 samples, diverging theta_cone/extent from t=0) despite coincidentally-matching xy_err — reserve WAS engaging, just not strong enough |
| **1.0** | 1 | **`SOFT-only`, xy=2.761** | 2 | **no TARGET_LOST** |
| **1.0** | 2 | **`FAIL`, xy=1.559** | 8 | **no TARGET_LOST** |

At `reserve=1.0`, both reps avoided the catastrophic mid-flight marker-loss-beyond-grace outcome that occurred in EVERY prior IC5 perception-mode run this session (baseline, both cap/href variants, reserve=0.5). Landing precision itself (`xy_err` still 1.5-2.8m) is NOT solved by this change — that's the separate, already-documented off-center precision/softness wall ([[project_20260824_crossmarker_offcenter_convergence_wall]]'s corrected framing: funnel-shape convergence already works under GT-FB, needs several seconds; margin-reserve just keeps perception alive long enough to give it the CHANCE to work, doesn't itself fix convergence speed).

## Validation sweep (2026-08-25, partial -- SITL infra was unusually flaky this session, burning most of the wall-clock budget on lockstep-race/port-bind retries)

**IC5, n=3 per arm** (`test_data/PerceptionVsGT_IC5/20260824-220500/aruco_{reserve0,margin_reserve1}_ic5_rep{1,2,3}`):
- reserve=0.0: 2/3 `TARGET_LOST` (xy=1.55, 7.09), 1/3 `SOFT-only` (xy=2.75).
- reserve=1.0: 1/3 `TARGET_LOST` (xy=22.46 -- notably BAD when it does fail), 1/3 `SOFT-only` (xy=2.76), 1/3 `FAIL` (xy=1.56).

Directionally consistent with the fix (lower TARGET_LOST rate), but n=3/arm is NOT a validated result -- both arms show real variance (reserve=0.0 didn't always fail; reserve=1.0 wasn't immune), and the one reserve=1.0 failure was the single worst xy_err recorded across the whole investigation. Genuinely inconclusive at this sample size; the earlier "2/2 avoided TARGET_LOST" framing was optimistic based on too few reps.

**IC1/IC2 regression spot-check, n=1 each, reserve=1.0** (`aruco_reserve1_IC{1,2}_regress`): IC1 `SOFT+PRECISE` (xy=0.083), IC2 `SOFT-only` (xy=0.190, rel_vel=0.121) -- no obvious regression at either, consistent with normal baseline behavior. IC3/IC4 NOT checked.

## ⚠ CONFOUND DISCOVERED (2026-08-25, after the sweep): a concurrent, INDEPENDENT SITL session was active during this exact test window

Found (`ps aux`) a separate, still-running process tree launching its own `run_aruco_landing.sh` (`test_data/Rover_AB_harness/ic5_hang_chase_*.out`, `PLASMC_GT_FEEDBACK=1`, IC5, cross_marker world, own `LANDING_OUT_BASE`) -- NOT this session's processes, a different bash/PID tree. File timestamps: continuously active 11:04→11:33+, i.e. spanning the ENTIRE n=3 IC5 sweep (11:08-11:12) and both IC1/IC2 regression checks (11:14-11:15) run in this entry.

Two distinct effects: (1) the repeated port-8888-bind-error / "not the race condition" launch flakes fought through all session are DIRECTLY EXPLAINED by this -- `MicroXRCEAgent` binds a single global port, so when the other session held it, ours failed to launch. This is fail-fast and harmless to already-captured data (no corrupted runs, just launch retries). (2) HARDER TO RULE OUT: two simultaneous `gz sim` physics+rendering stacks compete for CPU; Gazebo/PX4 lockstep timing is deterministic but real-wall-clock subsystems (image processing rate, OpenCV decode, Python thread scheduling) are NOT -- exactly the machinery behind ArUco decode reliability, which is what this sweep measures. The single unusually-bad `reserve=1.0` outlier (`xy=22.46m`, the worst result in the whole investigation) is a plausible contention-noise candidate, though not distinguishable from genuine variance after the fact.

**Implication: the n=3 sweep above should be re-run once confirmed no concurrent SITL session is active, before treating its numbers as even directionally reliable.** Not retroactively fixable; a fresh, isolated sweep is the only way to get a clean signal.

## CLEAN re-run (2026-08-25, confirmed no concurrent SITL before EVERY launch this time) -- REVERSES the direction

`test_data/CBFMarginReserve_Clean_IC5/20260825/{reserve0,reserve1}_rep{1,2,3}`, same probe (ArUco, IC5, perception-mode, `PLASMC_DTHETA_AZ_GAIN=0`):

| arm | rep1 | rep2 | rep3 | TARGET_LOST rate |
|---|---|---|---|---|
| reserve=0.0 | xy=1.319, no TL | xy=2.798, **TL** | xy=2.924, no TL (soft) | **1/3** |
| reserve=1.0 | xy=2.386, **TL** | xy=1.096, **TL** | xy=1.930, no TL | **2/3** |

With the confound actually removed, `reserve=1.0` shows a HIGHER TARGET_LOST rate than the unmodified baseline -- the OPPOSITE of the (contaminated) result that motivated this fix in the first place. n=3/arm is still far too small to call this a real regression, but it flatly does not support the original "reserve=1.0 avoids TARGET_LOST" claim. **Combined across both sweeps (contaminated + clean, 6 reps/arm, not statistically poolable but directionally suggestive): no clean evidence `CBF_MARGIN_RESERVE` helps at IC5.**

## Status

`CBF_MARGIN_RESERVE` is a real, mechanistically-sound lever (confirmed engaging via diverging trajectories/theta_cone, not a no-op) targeting a real, correctly-diagnosed gap (Phase 1's box bound genuinely is centroid-only, no proactive margin) -- but TWO independent n=3 IC5 sweeps (one contaminated/inconclusive, one clean and REVERSING the direction) give NO evidence that `reserve=1.0` actually helps, and a weak hint it may even cost more `TARGET_LOST` events than doing nothing. NOT baked, NOT promising in the way first reported -- the honest status is "correctly-targeted lever, unproven benefit, possible cost." Left at default 0.0 (unchanged behavior). If revisited: a real n>=5 clean sweep (confirmed no concurrent SITL) is needed before this idea can be trusted either way; worth also checking a milder reserve (e.g. 0.3-0.5) in case 1.0's maximal conservatism is itself counterproductive (over-constraining lateral authority even on approach paths where corners were never actually at risk, which could explain a WORSE outcome than doing nothing).
