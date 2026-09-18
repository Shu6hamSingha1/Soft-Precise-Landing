---
name: project_20260916_curve_qgate_revalidation
description: "⛔ 2026-09-18: ALL ArUco-rover curve results from 2026-09-16/17 (the r=0.8 and r=10 QGATE A/Bs, the cycle-isolation sweep, the d380901c worktree test, the camera-resolution test) were DELETED at user instruction -- ArUco is an obsolete approach for the moving-rover scenario, and those results are misleading now that cross-marker is the only live path. What SURVIVES: (1) ROVER_CIRCLE_R default 0.8m->10m (commit b816fea0, 2026-07-03) is a code-level fact independent of marker -- ROVER_TRAJ=Circular at its default is a gentle arc, NOT a curve; set ROVER_CIRCLE_R=0.8 for the real turning-target case. (2) The cross-marker rover (WORLD=rover_cross MARKER_TYPE=cross) genuinely fails ~1/5 at r=0.8 regardless of AU_LEAD (confirmed real, data kept). (3) Two proposed root causes for that failure are RETRACTED (GT pose-jitter/chase-cam -- physically invalid under lockstep sim time, wrong loop measured; yaw/alpha-sign convention -- already resolved 2026-09-09, baked into current code). (4) NEW LEAD (2026-09-18, from a Windows/MATLAB session's independent same-day work): MATLAB just found PLASMC_YAW_RL_KP=0.3 (the yaw-rate-law's default gain, PX4's default too) destabilizes Circular tracking via an image-position/orientation coupling runaway at elevated target speed; fixed by lowering to 0.02. PX4's cross-marker rover run resolved YAW_RATE_LAW=True, YAW_RL_KP=0.3 -- the exact unstable value -- because the yaw-rate-law is MARKER_TYPE=cross-gated and my 'heading-hold' recipe only zeroed the (inactive) ASMC gains. Untested on PX4 yet -- the next discriminator is a direct A/B with PLASMC_YAW_RATE_LAW=0 forced."
metadata:
  node_type: memory
  type: project
  originSessionId: 878fdadb-dd99-4085-bcf2-e19879f48082
  modified: 2026-09-18T11:18:20.498Z
---

## ⛔⛔ 2026-09-18: ALL ArUco-rover curve test data and findings from this thread REMOVED

**User instruction, verbatim intent: ArUco is an obsolete approach for the moving-rover
scenario; forget findings tied to it, and delete the misleading test data.** Verified from
each run's own recorded `Control_Params.resolved` overrides (not assumption) that the
following directories never set `WORLD`/`MARKER_TYPE`, so they ran on the launcher's ArUco
default (`WORLD=rover`, `ROVER_MODEL=rover_aruco`) rather than the project's live
cross-marker target — **deleted** (`test_data/Rover_Turning/`):
`r08_confirm/`, `qgate_revalidation/`, `qgate_revalidation_r08/`, `cycle_isolation/`,
`worktree_d380901c/`, `camera_640x480/`, `bisect/` (427 MB total, none git-tracked, no repo
history affected).

**Findings retracted along with the data (do not cite these numbers again):**
- The r=0.8 3-arm `AU_LEAD_QGATE` A/B result (baseline 6/6, ungated 6/8 w/ a 20.7 m
  fly-away, gated 7/7 w/ lowest terminal command) — ArUco, not cross-marker.
- The r=10 3-arm A/B ("gate does not neuter the curve", p=0.72) — ArUco AND the wrong
  radius (see below); doubly invalid.
- The 4-arm gain-revert cycle-isolation sweep (`CBF_DRIFT_TAU`/`P_xy`/`P2INF_xy`/`XI2_xy`,
  all null) — ArUco.
- The `d380901c` pre-rewrite-worktree tail-variance comparison — ArUco.
- The 640×480 camera-restoration test — ArUco.
- The `edb546f0` bisect-endpoint test — ArUco.

**What is kept below is only what does NOT depend on that deleted data**: a code-level fact
(the `ROVER_CIRCLE_R` default), and the genuinely cross-marker-run results
(`test_data/Rover_Turning/{qgate_revalidation_r08_cross,cross_gtfb_smoke}/`, still on disk,
verified `WORLD=rover_cross MARKER_TYPE=cross`).

## `ROVER_TRAJ=Circular`'s default is a gentle arc, not a curve (code fact, marker-independent)

Commit **`b816fea0`** (2026-07-03 12:13) changed `ROVER_CIRCLE_R` **0.8 m → 10 m** (own
comment: *"the small-radius circle (0.8 m, ~Ackermann min turn radius) made the TARGET
motion jerky (steering saturated) … only the path curvature drops ~12x"*). Verified via a
Kåsa circle fit on the GT `Target Pose` that the default (r=10) path is a 9-14 m radius,
11-28° swept arc — a straight line in practice, not a curve. **To exercise the turning-target
scenario at all you must set `ROVER_CIRCLE_R=0.8`.** This is a code/config fact independent
of which marker or perception path is in use, so it survives the ArUco-data purge and
applies to all future cross-marker rover testing too.

## Cross-marker rover at r=0.8: genuine ~1/5 failure, independent of AU_LEAD (data kept)

3-arm A/B, `WORLD=rover_cross ROVER_MODEL=rover_cross MARKER_TYPE=cross`,
`ROVER_CIRCLE_R=0.8`, GT-FB, n=5/arm, every rep confirmed on the genuine curve (Kåsa fit
`r_fit` 0.84-0.92 m). Data `test_data/Rover_Turning/qgate_revalidation_r08_cross/`.

| arm | n | landed | lat mean | lat max | terminal peak `|I_a_xy|` |
|---|---|---|---|---|---|
| A base (no lead) | 5 | **1/5** | 0.548 | 1.323 | **6.24** |
| B ungated lead | 5 | **1/5** | 0.625 | 1.590 | 6.69 |
| C gated lead | 5 | **1/5** | 0.627 | 1.246 | 6.44 |

**All three arms land at roughly the same ~1/5 rate, ~6x the terminal command magnitude of
what stationary work normally sees, INCLUDING the arm with AU_LEAD entirely off.** Since arm
A has no lead active and still fails this badly, **the degradation is not in AU_LEAD or the
gate — it is in the shared control/perception path on the moving cross-marker rover.**
`min_alt` on failures is 0.20-0.34 m, well under the 0.5 m platform height — the drone
descends PAST the platform, not just missing laterally.

Confirmed real from the run logs directly (`[controller] TOUCHDOWN-DETECT (GT)` /
`[FC] Impact detected` lines): cross-marker touchdown `|s_e_n|` = **0.67, 2.35, 5.85** — a
genuine, large lateral tracking divergence at touchdown, not a detection artifact. **2 of 5
reps ended in a literal hard IMPACT** (`|a|` 53.2, 61.3 m/s² > the 50 m/s² threshold) rather
than a soft touchdown-detect event.

**Ruled out: marker-mount-height mismatch.** `rover_cross/model.sdf`'s own header comment
says it is a direct geometric port of `rover_aruco` — identical `landing_platform` pose
(0,0,0.30) and `marker_visual` pose (0,0,0.201), same 0.5 m total mount height.
`PLASMC_GT_MARKER_DZ` (launcher auto-exports 0.5) is correct for both.

## Two root-cause candidates tried and RETRACTED — do not re-propose either

**1. "GT `/pose` timestamp jitter, caused by chase-camera render contention."**
Retracted on two independent grounds: (a) Gazebo runs LOCKSTEP sim time with PX4 — render
speed cannot create sim-time gaps, so "heavier render load stutters the pose stream" is not
physically valid; this also matches [[project_20260826_chasecam_resolution_bump]], which
already tested the same shared-render-thread mechanism via a sibling stream across three
resolution bumps with zero effect each time. (b) The `dt<=0` evidence was measured on
`apps/landing_test.py`'s own diagnostic recording loop (`time_node.perf_counter()`,
`src/gz_subscriber.py:319`, genuinely sim-time but polled at wall-clock cadence — a
duplicate row just means the recorder polled faster than the clock ticked), a completely
different loop from the one inside `controller.py`'s `Controller` thread that actually
feeds `gt_feedback.py`. **Chase-cam resolution is not implicated by anything in this
thread.**

**2. "Cross-marker-specific yaw/alpha sign convention bug."** Retracted (user correction):
this bug class was found, flip-flopped across multiple sessions, and finally resolved
2026-09-09 ([[feedback_gtfb_wz_sign_bug]] — see its own superseded block;
[[project_yaw_rate_law_sign_bug_and_validation]]), validated against calibrated perception
`w_iz` correlation, and is baked into the current `gt_feedback.py`
(`w[2] = +_asign * _slope(...)`) — not marker-specific, not open.

**Methodological lesson from both** (kept in
[[feedback_recurring_analysis_mistakes]] §11/§20): verify a mechanism's preconditions
(does the simulator's timing model even permit this? which loop produced the number? has
this exact mechanism already been tested elsewhere?) before presenting something as a root
cause. Both retractions here were caught by the user, not by self-check.

## ⭐⭐⭐ NEW LEAD (2026-09-18): PLASMC_YAW_RL_KP=0.3 — matches a freshly-discovered MATLAB
## instability, and it IS what was active in the cross-marker runs above

A same-day Windows/MATLAB session (independent, unaware of this PX4 thread) found and fixed
a **Circular-target FoV breach**: at elevated target speed, the yaw-rate-law's alignment
error `e_a` blows up (to 83°+) when required spin exceeds the law's validated envelope,
coupling into **unbounded marker-radius growth**. Grid-searched `P.yrl_kp`
(`PLASMC_YAW_RL_KP`) and found the fix runs opposite to intuition — **lowering** kp resolves
it (raising it to 0.6 made it worse), with a clean monotonic trend all the way down: kp=0.20
survives to 1.4x target speed, kp=0.02 survives 1.2x-2.0x (double nominal). Counterintuitively
`max|e_a|` gets LARGER at low kp (up to 160°) — the failure isn't alignment-error magnitude,
it's whether the correction is aggressive enough to excite an image-position/orientation
coupling into runaway growth. Baked `P.yrl_kp = 0.02` (was `0.3`) in `vdf_params.m`
(commits `6a7d7dfb`, `ee7bbd46`). Full mechanism + sweep data: `MATLAB/Multi_init_cond/
cb_yrl_kp_sweep.m`, `vdf_params.m`'s inline comment, `project_ic2_speed_sweep_failure_
2026_09_17` (MATLAB-side; not synced to this machine's memory store — read the commits/code
directly).

**PX4 default is unchanged: `PLASMC_YAW_RL_KP=0.3`** (`src/controller.py:623`) — the exact
value MATLAB just proved unstable. And critically, **this is what was actually driving yaw
in the cross-marker A/B above.** Checked the run's own `Control_Params.resolved`:

```
YAW_RATE_LAW = True   YAW_RL_KP = 0.3
overrides: {..., PLASMC_YAW_GAMMA:'0', PLASMC_YAW_KAPPA0:'0', PLASMC_YAW_N:'0',
                 PLASMC_YAW_OMEGA:'0', WORLD:'rover_cross', MARKER_TYPE:'cross'}
```

The "heading-hold" recipe (inherited from the July `Rover_AB_harness` campaign, predating
`PLASMC_YAW_RATE_LAW`) zeroes the **ASMC** yaw gains (`YAW_GAMMA/KAPPA0/N/OMEGA`) — but
`PLASMC_YAW_RATE_LAW` defaults ON and is gated to `MARKER_TYPE=cross` (falls back to ASMC
on ArUco). So on the cross-marker rover, the zeroed ASMC params did nothing; the *active*
yaw controller was the yaw-rate-law at kp=0.3, chasing the rover's real body-heading
rotation the whole time — **the "heading-hold" tests were never heading-hold on
cross-marker.** The rover's own circling at `ROVER_CIRCLE_R=0.8`, `SPEED_MULT=1.0` demands
a base heading rate `wz = v_tan/r ≈ 0.384/0.8 ≈ 0.48 rad/s`, inside MATLAB's earlier-
validated envelope at *nominal* speed but on a plant (PX4) that additionally carries real
actuation lag MATLAB's torque-level inner loop does not — plausible this pushes the same
marginal coupling into instability at a lower demand than MATLAB needed to trigger it.

**This also cleanly explains the marker-type asymmetry** that the two retracted hypotheses
above were chasing: it isn't that cross-marker perception is worse — it's that
`MARKER_TYPE=cross` is precisely the condition that activates the (now known-unstable-at-
this-gain) yaw-rate-law, while ArUco silently fell back to a correctly-zeroed ASMC (genuine
heading-hold). The failure IS marker-type-correlated, but the mechanism is a yaw-control
gain-path selection, not a perception-quality difference.

**Not yet proven for PX4 — untested.** The discriminator is cheap: rerun the cross-marker
r=0.8 no-lead baseline with `PLASMC_YAW_RATE_LAW=0` forced (true heading-hold via the ASMC
path, correctly zeroed) vs the current default, same recipe, n=3-4 each. If the divergence
disappears with the yaw-rate-law off, this is confirmed as the (or a major) cause. If not,
the remaining untested candidate is the live `gt_feedback.py` call site inside
`controller.py`'s `Controller` thread (never inspected directly).

**How to apply:** before any further cross-marker rover A/B, either force
`PLASMC_YAW_RATE_LAW=0` for a genuine heading-hold baseline, or explicitly set
`PLASMC_YAW_RL_KP` to a low value (MATLAB's `0.02`, unvalidated on PX4) and re-test. Do not
reuse the July `Rover_AB_harness` "heading-hold" recipe unmodified on cross-marker without
accounting for this gate.

## What SURVIVES from the pre-2026-09-16 stationary work (unaffected by any of this)

All stationary IC1-5 work — the AU_LEAD stationary regression, the GT-FB refutation of the
κ-ratchet story, `PLASMC_KAPPA_DZ_*`, and the 2-term `AU_LEAD_QGATE` (18/25 ≈ baseline, IC5
0/5→5/5) — used no rover and is untouched by anything above.
[[feedback_aulead_stationary_regresses]] [[feedback_adaptive_law_noise_behavior]]
