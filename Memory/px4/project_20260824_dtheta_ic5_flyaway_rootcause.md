---
name: project_20260824_dtheta_ic5_flyaway_rootcause
description: "CORRECTS a wrong same-day attribution: IC5's target_lost/ASCENDING failures (in both the pre-v3 dtheta baseline AND the new PLASMC_DTHETA_HREF v3 mechanism) are NOT the off-center kappa-leakage convergence wall (that's a ~2-3m xy_err soft failure, seen cleanly at IC2). They are a genuine catastrophic fly-away: sustained near-continuous (65-98%) dtheta engagement at IC5 injects a growing, uncapped extra-lift correction every single cycle with no bound on cumulative altitude gain, producing a runaway climb (3m->8-10m alt) + 20-28m lateral divergence within ~9s, destabilizing tracking until TARGET_LOST commits to open-loop descent (crash-landing from height). Confirmed present in the PURE v2 (pre-href) baseline -- v3 (today's h_ref/crossfade change) did NOT introduce this; it's a pre-existing defect of the raw dtheta correction's design (no integral/rate cap) that only manifests at ICs with high dtheta duty cycle. IC2 comparison run: dtheta duty cycle only 8-20% there, href_g stays ~1.0, no fly-away, ordinary wall-level xy_err (~2.0-2.3m) in both baseline and v3 -- consistent, not aggravated."
metadata: 
  node_type: memory
  type: project
  modified: 2026-08-24T13:31:58.352Z
  originSessionId: 0f9c8dc0-a837-4722-95e7-0ea102167469
---

> ⛔⛔ **OBSOLETE MECHANISM (stamped 2026-09-03).** `_dtheta_correction` was REMOVED from
> `controller.py` in `e110b8a7` (2026-08-31); the descent-rate/lateral-margin trade now lives
> INSIDE the joint QP as `cbf_visibility.py::CBF_AZ_COST_GAIN` (`controller.py:254`).
> `grep -c _dtheta_correction src/controller.py` == 0. Anything in this file that diagnoses
> `_dtheta_correction` or proposes validating `PLASMC_DTHETA_HREF` is testing a mechanism that
> no longer exists — that env survives (`:247`, default 0) but now gates only the SEPARATE
> upstream `h_ref_eff` shaping. Findings about CAUSALITY (control destabilises first,
> perception second) may still hold; the named fixes and next-steps do not. Current mechanism:
> `CBF_JOINT_QP` (baked default-on) + `CBF_AZ_COST_GAIN`.

Same-day follow-up to [[project_20260824_dtheta_href_continuous_compensation]], answering three user questions: (1) run the baseline-vs-v3 ratio comparison at IC2, (2) identify the real cause of the `target_lost`/`ASCENDING` failures seen in the IC5 smoke test, (3) check whether the new `PLASMC_DTHETA_HREF` v3 mechanism caused/contributed to those failures.

## Correction to the prior turn

The prior response attributed the 4/4 IC5 smoke-test failures to "the known off-center kappa-leakage convergence wall" ([[project_20260824_crossmarker_offcenter_convergence_wall]]). **That was wrong** — that wall is a soft ~2-3m `xy_err` degradation, not what's actually in the data. Checking the actual `UAV Pose` trajectories showed a full-blown fly-away.

## Root cause: uncapped cumulative lift from near-continuous dtheta engagement (NOT the off-center wall)

Trajectory check on all 4 IC5 smoke-test runs (2 pre-v3 baseline from `DthetaGain_IC5`/`DthetaDesired_IC5`, 2 v3 from `DthetaHref_IC5`) shows the SAME pattern: starting at IC5's `(2,2,3)` ENU spawn, altitude climbs from 3m to 7-10m over ~5-6s while lateral position diverges 20-28m, then the vehicle falls/crashes back to the ground from height once `TARGET_LOST` commits to open-loop constant-thrust descent. Example (`v3 gain40_href1`): `z` 3.01 -> 10.08 (t=5.5s) -> crash-lands at t=9.15s from `x=-13.3, y=4.5`. Example (`baseline gain5`, no `href`, predates today's v3 code entirely): `z` 3.00 -> 8.72 (t=5.85s), `x` -22.2 by t=8.57s. Essentially the same magnitude explosion in BOTH.

**Mechanism, confirmed directly from Control_Data**: at IC5, `dtheta` is active (>1e-3) in 91.3%/78.8% of frames within just the FIRST 2 SECONDS of flight (near-continuous from engage, not an occasional spike). The direct `I_a[2] -= gain*dtheta` correction has no memory of how much lift it has already injected — it's a pure per-cycle proportional term with no cap on cumulative altitude gain. Traced `I_a_raw[2]` (pre-correction, ~-9.0 to -9.3, i.e. near-hover baseline) vs `I_a[2]` (post-correction) in `baseline gain5`: the correction magnitude itself GROWS over the first ~2s (0.20 -> 1.5-1.9 m/s²), i.e. 15-20%+ extra sustained upward acceleration on top of the raw command, compounding for as long as `dtheta` stays active — which at IC5 is nearly the whole flight. That sustained excess lift is what drives the multi-meter runaway climb, which then destabilizes lateral tracking (the SMC/CBF geometry was never designed for a 7m mid-flight altitude excursion) until the marker is lost.

This is a DIFFERENT defect from the previously-documented self-defeating attitude-history loop ([[project_20260824_dtheta_az_filter_self_defeating_feedback]]) — that one is about the CBF granting LESS lateral authority over time (a slow degradation). This is about the correction's own OUTPUT being unbounded/uncapped in cumulative effect when its trigger condition (`dtheta>0`) stays true almost continuously — an integral-windup-shaped defect, not a feedback-through-the-CBF defect. Both are real, independent problems with the same `dtheta` mechanism.

## Was this caused by the new v3 (`PLASMC_DTHETA_HREF`) change? NO.

The identical fly-away magnitude and shape is present in `baseline gain5` (`test_data/DthetaDesired_IC5/20260824-163554/gain5`, `PLASMC_DTHETA_HREF` unset — pure pre-existing v2 `dtheta` code, run BEFORE today's h_ref/crossfade implementation existed) as in the two v3 runs. It also appears across gain 10/20/40 in the ORIGINAL `DthetaGain_IC5` sweep from earlier the same session (`test_data/DthetaGain_IC5/20260824-142606/`, also pre-v3). **v3 did not introduce this** — it's a pre-existing defect in the raw `dtheta` direct-correction design that only manifests at ICs where `dtheta`'s duty cycle is high enough for the uncapped accumulation to matter.

## IC2 comparison (requested)

`test_data/DthetaHref_IC2/20260824-185657/{gain10_baseline,gain10_href1}`, gain=10, both landed (no `TARGET_LOST`):

| config | dtheta active frac | mean dtheta | ratio theta_safe/theta_desired | href_g median | zmax | xy_err | rel_vel |
|---|---|---|---|---|---|---|---|
| baseline (no href) | 7.7% | 0.0006 | 0.998 | n/a (off) | 5.02m | 2.290 | 1.788 |
| v3 href=1 | 20.2% | 0.0023 | 0.984 | 1.000 | 5.01m | 2.045 | 2.073 |

At IC2, `dtheta`'s duty cycle is an order of magnitude lower than IC5 (8-20% vs 65-98%), `href_g` stays pinned near 1.0 (the new mechanism barely engages — nothing to compensate for), `zmax` is flat (~5.0m, the IC's own spawn altitude — **no fly-away**), and both configs land with ordinary off-center-wall-level `xy_err` (~2.0-2.3m). This is the actual off-center wall the prior memory documented — correctly seen here, not at IC5.

## Conclusion / next steps

The IC5 catastrophic fly-away is a real, pre-existing bug in the base `dtheta` az-correction (uncapped cumulative lift under near-continuous engagement), independent of and NOT introduced by the v3 continuous-compensation/crossfade work. It's the actual reason IC5 has been an unreliable outcome-evaluation IC for this feature all session — not (only) the off-center wall as previously assumed. Fixing it needs a cap on the correction itself (e.g. clamp cumulative/rate of `I_a[2]` lift injected, or cap `dtheta`'s contribution to a bounded per-flight budget), independent of whatever happens with the `th_curr` self-defeating-loop fix or the v3 `h_ref` compensation. Not yet implemented. n=1-2 per config throughout this investigation — directionally solid (mechanism trace, not outcome-count) but not a validated sweep.
