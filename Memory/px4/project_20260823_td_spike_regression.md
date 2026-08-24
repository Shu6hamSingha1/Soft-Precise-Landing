---
name: project_20260823_td_spike_regression
description: "ROOT-CAUSED and FIXED the GT-feedback SP regression (ArUco AND cross-marker): PLASMC_TD_SPIKE default 0.5 was ~16x larger than the real h_z touchdown signal (peaks ~0.031) and could never latch -- reverted to 0.0 (b2a0194, merged 87b8896). n=5 sweep + wholesale 486f713 revert-and-rerun both confirm. ArUco IC1-5 24/25 SP, ArUco IC2 10/10, cross-marker IC2 5/5 post-fix. Corrects project_20260817's 'kappa-leakage fixed-point, marker-type-agnostic' framing -- that fixed point is real but was never why landings failed."
metadata: 
  node_type: memory
  type: project
  originSessionId: d9217f1e-2435-4b4e-a257-e5841e0b9caa
  modified: 2026-08-23T12:44:30.301Z
---

**Continues / corrects [[project_20260817_crossmarker_descent_stall_investigation]] and
[[project_why_sp_achieved]].** Session 2026-08-20 through 2026-08-23, spanning the ArUco
history-bisection (initially asked to explain "why can't we reproduce the 2026-06-30
19/25-SP GT-FB campaign") and the earlier cross-marker "descent stall" investigation —
**both were the same bug.**

## The bug

`controller.py::_touchdownDetect` latches touchdown when `h_z > PLASMC_TD_SPIKE` for 3
consecutive frames. Its own init comment: `min |h_z| to count (PROVISIONAL, n=2 evidence
-- validate at n>=5)`, default `"0.5"`. **That validation was never done.** Measured `h_z`
over a full 20s GT-FB descent-stall window peaks at only **0.031** (noise-floor scale) —
`0.5` is ~16x too large to ever latch on the real signal. At `486f713` (the 2026-06-30
19/25-SP snapshot) the condition was simply `h_z > 0.0` (no magnitude gate at all), which
is why it worked then.

Consequence: with the touchdown latch permanently dead, the descent runs on past the
point where it should disarm, altitude flatlines at ~0.487m (a real, physical loom-sign-
inversion event near the ground — not noise, not a marker-type artifact), Gazebo's ODE
physics engine eventually auto-disables the near-motionless rigid body (confirmed via an
independent-process `/pose` topic probe: messages kept arriving at 50Hz, `n_poses` stable,
position bit-frozen for 17.5s while `/clock` kept advancing), and the 25s descent-stall
watchdog aborts the flight. **This is marker-type-agnostic** (`_touchdownDetect` reads
`self._h[-1][2]` regardless of `MARKER_TYPE`) — confirmed: ArUco and cross-marker both hit
it, at their own respective altitude/kappa-equilibrium signatures.

## Dead ends ruled out first (in order, each tested and falsified before finding the real cause)

1. **`PLASMC_GT_Z_REG`** (0.2/0.1/0.05 sweep, n=5 each): real double-counted depth-floor
   defect (mount offset applied explicitly in `7830ef6` 2026-07-02, `Z_REG` left at 0.2
   despite the file's own note "left at 0.2 pending a sweep" -- never done), but
   mechanistically wrong direction (inflating Z_eff should REDUCE perceived overspeed,
   not increase it) and empirically inert: 0/15 SP across all three values. Not fixed;
   a real but minor (~22% z-loop-gain erosion) latent issue, separate from this bug.
2. **`PLASMC_VDS_KF_Q`** (1 vs 10, n=5x4 incl. both marker types): affects whether the
   frozen kappa-equilibrium gets broken by a violent detonation before the 25s watchdog
   (a_u ~300-440, 3-7 m/s hard impacts) -- a coincidental side-effect, not a fix. 0 real
   SP either value.
3. **`PLASMC_TERMINAL_COMMIT`** (0 vs 1, n=5x4): same story, ArUco stall rate drops
   80%->40% but 0/40 combined reps across both A/Bs achieved `precise=True` -- completions
   were the same detonation-crash pattern.
4. **`FEATURE_PTS_FRESH` conditional-integration gate** (added 2026-07-30, absent at
   486f713): confirmed IT DOES freeze `izeta_z` exactly (1 distinct value / 1557 samples)
   under GT-feedback, where gating integrators on a PERCEPTION freshness flag is
   measuring the wrong thing. Added `PLASMC_FRESH_GATE_INTEG` bypass knob (default 1 =
   unchanged). Bypassing it (gate0) let `izeta_z` wind up fully to the clamp (5.0) --
   and the drone STILL didn't move (thrust_norm 5.4% below hover, ignored). This is what
   led to discovering the ODE-sleep mechanism directly: gate0's "stall" was NOT the
   controller failing to command descent, it was the vehicle no longer responding to
   commands at all.
5. **Gazebo ODE physics auto-disable** -- real, reproducible (23/56 reps this session
   froze at EXACTLY 0.4860m, std 0.0000), and the IMMEDIATE mechanism of the "stall," but
   a DOWNSTREAM symptom: it only happens because the touchdown latch never fires to
   disarm before velocity decays into the engine's sleep band. Not itself fixable/worth
   fixing -- the real fix is not reaching that state.

## Confirmation (this is the part that makes it certain, not just plausible)

- **n=5 `PLASMC_TD_SPIKE` sweep** (ArUco GT-FB IC2): 0.0 -> 5/5 SOFT+PRECISE (min_alt
  0.4869-0.4871, std 0.0001); 0.1/0.3/0.5 -> 0/15 combined (17 stalls, 3 kappa-detonation
  crashes at 3.8-4.3 m/s). A hard cliff, not a gradient -- consistent with the h_z=0.031
  ceiling measurement.
- **Wholesale revert** of `controller.py`+`cbf_visibility.py`+`img_data.py`+
  `gt_feedback.py` to `486f713` (2 compat shims added for API calls current
  `apps/landing_test.py` makes that didn't exist then: `CBF_CORNERS_STALE_ABORT`
  property->False, `CBF_OVERFLOW` property->False, `IMG_PROCESSOR.
  update_cbf_handover_signal`/`getFailureCause` no-ops), re-run n=5 independently:
  **5/5 SOFT+PRECISE**, min_alt 0.4868-0.4871 (std 0.0001m) -- reproducing the identical
  altitude to sub-millimeter precision. Confirms 486f713's `h_z>0.0` condition IS the
  historical mechanism, not a coincidence.
- **Post-fix full validation**: ArUco GT-FB IC1-5 (n=5/IC) = **24/25 SOFT+PRECISE** (the
  1 miss was an unrelated early-hover stall at 6m, `_touchdownDetect` irrelevant there --
  never got near the ground). ArUco GT-FB IC2 alone (across the sweep + reruns) =
  10/10. Cross-marker GT-FB IC2 (n=5, no marker-specific changes) = **5/5
  SOFT+PRECISE**, min_alt 0.4870-0.4871 -- confirms the fix is marker-type-agnostic, as
  the mechanism predicted.

## Fix

`src/controller.py`: `PLASMC_TD_SPIKE` default `"0.5" -> "0.0"`. Committed `b2a0194`,
merged with 12 diverged remote commits (unrelated Hardware/HW_POS_FEEDBACK work from a
concurrent session) at `87b8896`, pushed. `scripts/run_tdspike_gtfb_ab.sh` is the sweep
harness that produced the evidence.

**Caveat, deliberately left open, not a regression**: `0.0` accepts a real but WEAK loom
inversion ~0.48m above true ground contact as "landed" -- ground truth confirms actual
ground contact is ~0.01m (matches the median `min_alt=-0.0088m` across 1340 archived
historical PRECISE landings elsewhere in this codebase). `0.0` is the historically-
validated, WORKING value; it does not explain why the descent stops ~0.48m short of true
touchdown in the first place. Untouched by this fix.

## Correction to prior memory

[[project_20260817_crossmarker_descent_stall_investigation]]'s framing -- "repeatable
kappa-leakage fixed-point equilibrium, marker-type-agnostic" -- is **not wrong about the
fixed point being real and reproducible** (it is; kappa_z genuinely pins at 0.2516
cross-marker / 0.119 ArUco, confirmed to 4 decimal places across dozens of reps this
session too), but **wrong about it being the cause of the landing failure**. The fixed
point is a real attractor of the kappa-leakage ODE that the descent reaches near
touchdown regardless of TD_SPIKE -- but reaching it was never the problem. The problem
was that nothing was watching for it (or the accompanying loom-sign-inversion) to
declare success. That session's own "next priority: test VDS_KF_Q=1" lead (also
inherited from [[project_why_sp_achieved]]'s enabler list) was a red herring for the
same reason #2 above shows: VDS_KF_Q changes whether a detonation-crash breaks the
stall, not whether a real landing happens.

**Why:** two harness-adjacent defects (7830ef6's Z_REG double-count, 2026-07-30's
FEATURE_PTS_FRESH integrator gate) and one genuine control-law regression (TD_SPIKE)
accumulated in the ~7 weeks since the SP campaign, all downstream-obscured by the SAME
symptom (a stall near 0.49m) that also has a REAL, unrelated ODE-physics mechanism
(auto-sleep) as its literal proximate cause -- five distinct things pointing at the same
altitude made this take multiple sessions and several falsified hypotheses to unwind.
**How to apply:** when a terminal/near-ground failure reproduces at a suspiciously exact,
low-variance altitude across many reps, check the TOUCHDOWN-DETECT / disarm-condition
code FIRST (grep for the abort/latch thresholds), before assuming a control-law/gain
issue -- a control law reaching a real equilibrium is not evidence that equilibrium is
the failure, only that nothing recognized it as success.

## Methodology note for the upcoming GT-ablation work

`GT_ABLATE` (per-channel GT substitution, `controller.py` ~line 1492) was **not used
anywhere this session** -- checked explicitly (grepped every sweep script + every ad-hoc
invocation in the tool-result logs). Every result above used full GT-feedback
(`GT_ABLATE` unset -> `"all"`), meaning `h_z` at the touchdown latch was ALWAYS
ground-truth-sourced, never real perception, in every test that validated this fix. If
GT_ABLATE excludes `'hz'` from the GT channel list, `_touchdownDetect` would read the
REAL perception-computed `h_z` instead -- a materially different config than anything
validated here, worth being explicit about once ablation testing starts.
