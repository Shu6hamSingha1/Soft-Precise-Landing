---
name: reference_cbf_visibility_architecture
description: "Authoritative reference for how the visibility CBF (cbf2_filter) is actually implemented and verified -- what's the REAL certified barrier vs legacy/frozen machinery, what's validated vs not, and three real gaps found 2026-08-24 (missing delta_m margin inset, no ground-truth test coverage for ArUco's Phase-2 rewrite OR cross-marker's radius-based Phase-2 delta term). Read this before reasoning about CBF correctness or proposing to tune rho_fov/funnel params for visibility."
metadata: 
  node_type: memory
  type: reference
  originSessionId: d9217f1e-2435-4b4e-a257-e5841e0b9caa
  modified: 2026-08-24T05:30:22.745Z
---

**Why this file exists**: this session got the CBF architecture wrong TWICE before writing this
down -- once conflating `d_min_fov`/`rho_fov` with the real barrier (already corrected earlier,
[[project_20260817_crossmarker_descent_stall_investigation]]'s standing-correction banner), and
again proposing to tune `rho_fov_0`/`rho_fov_inf` as a fix, which re-opens machinery explicitly
deprecated. User pushback both times. This is the ground-truth reference so it doesn't happen a
third time.

## The REAL certified barrier vs the legacy/frozen machinery — the core distinction

**`cbf2_filter` (`src/cbf_visibility.py` / `src/cbf_visibility_aruco.py`, Phase-1)** is the ONLY
part of this codebase that is a real, proven Control Barrier Function:
- Exact QP: `h_k = phi_max_k - |cr_k| - delta_k`, `phi_max = p_10 = centre/focal` (a FIXED
  camera-geometry constant, `self._p_10` in `controller.py`, NOT depth/altitude-dependent).
- Has a manuscript theorem (`thm:visibility`) with a proven forward-invariance guarantee under a
  feasibility condition.
- `θ_cap` (deliverability cap) is applied AFTER the QP, by the CALLER (`controller.py`), never
  inside `cbf2_filter` itself -- the QP stays a pure visibility constraint; `θ_cap` is a separate
  thrust-deliverability concern. See `PLASMC_THETACAP_DEG` (baked 2026-08-23 to a derived value,
  see [[project_20260823_td_spike_regression]]-adjacent controller.py comment at A_CAP).

**`rho_fov_curr` / `d_min_fov` / `theta_cone` (controller.py, ~line 2946-3020) are NOT part of the
real barrier.** `rho_fov_curr` is frozen at `rho_fov_0` by default (`PLASMC_LFOV=0.0` default,
baked 2026-06-05 -- the funnel-decay-to-`rho_fov_inf` design was ABANDONED, never re-enabled).
`d_min_fov` (built from `rho_fov_curr`) feeds exactly two things, both legacy/secondary:
1. `theta_cone`'s Phase-2 NO-CORNERS FALLBACK cone (not the live QP -- only matters when
   `cbf_corners is None`).
2. `OVERFLOW`/`DRIFT_OFF` classification (line ~2984-2991), which DOES have one real live
   consequence: `DRIFT_OFF` triggers a `p_10_eff` pullback (line ~3158-3162,
   `CBF_DRIFT_PULLBACK_FRAC`) that tightens the REAL barrier's margin on the breaching axis. This
   is the ONLY path by which `rho_fov`/`d_min_fov` touches the certified barrier -- indirectly,
   via classification, not directly as a constraint.

**Practical rule: never propose tuning `rho_fov_0`/`rho_fov_inf`/`PLASMC_LFOV` to fix a
visibility problem.** That machinery is deliberately dormant. If `d_min_fov` reading `0.0` for a
large fraction of a flight looks suspicious, it means the (legacy) margin computed to <=0 (a real
signal that `DRIFT_OFF` is likely firing a lot, worth checking `self._cbf_drift_off` directly,
not inferring from `d_min_fov`), not that corners are unavailable and NOT that the real barrier's
margin is exhausted -- the real barrier's margin (`p_10_eff` vs measured `cr`) is a different,
unlogged-by-default quantity.

## Verification done 2026-08-24 (offline, no SITL needed)

`~/ws/scripts/env2025/bin/python3 tools/validate_cbf.py` -- **12/12 checks pass**. This is a real
offline validator (independent analytic ground truth, not a self-check) covering: parity vs the
pre-refactor inline logic, the barrier holding on BOTH the predicted and TRUE feature position,
minimal-intervention (no spurious clamp when already feasible), no-strangle (inward recovery
stays free -- the property that distinguishes a CBF from a plain cone clamp), and the
`Rz`/tilt-mapping convention round-trips. Re-run this any time `cbf_visibility.py` changes.

**`CBF_LW_ROT` (default `"1"`) is the live, validated 90°-rotation fix** for the lean-vector vs
rotation-axis coupling (`L_w` couples angular RATE to feature flow, but `theta` here is a LEAN
direction -- 90° apart; `=0` reverts to the pre-fix, wrong behavior, kept only for A/B). One test
line (`1b. current code (identity) is WRONG -- 215.6% error`) looks alarming out of context but is
a deliberate NEGATIVE-CONTROL contrast proving the fix matters -- it is NOT flagging a live bug.
Read the full test list before concluding something's broken from one line.

**Phase-1 is byte-identical between `cbf_visibility.py` and `cbf_visibility_aruco.py`** (verified
by line-range extraction + diff, not just trusting the module docstring's claim) -- the ONLY
difference is `delta2`'s derivation: ArUco uses the real 4-corner-array spread
(`0.5*(ct.max(0)-ct.min(0))`), cross-marker uses the closed-form radius-derived version (it has no
corner array to fall back to). This is correct and intentional, not a bug.

## Phase-1 vs Phase-2: `radius`/`delta2` (cross-marker's only marker-size source) does NOT affect
## the live barrier at all during normal operation

Checked 2026-08-24 while verifying cross-marker specifically. **In Phase-1 (marker decoded --
the case that governs essentially all flight time), `m2 = phi_max` only -- `delta2`/`radius` is
deliberately excluded from the bound** (`cbf_visibility.py`'s own comment: "Phase 1... delta_eff
= 0: centroid-only barrier; deliberately allow the marker to grow and overflow as the UAV closes
in"). `radius` is only *tracked* (`state["delta_prev"]`) to compute a loom-rate consumed later --
it never shrinks the Phase-1 bound. So cross-marker's live, normal-operation barrier is exactly as
well-verified as ArUco's (same math, same 12/12 pass) -- the `delta2` derivation difference is
irrelevant while Phase-1 is active, which is nearly always.

`radius`/`delta2` only matters in **Phase-2** (decode-fail fallback): `m2_p2 = phi_max - delta_eff
- tau*ddelta_eff`. This is the ONE place cross-marker's radius-based geometry is actually live.

## Three real gaps found 2026-08-24, NOT yet fixed

1. **The manuscript's `δ_m=15px/f` robustness-margin inset is NOT implemented.** Theorem states
   `φ_max = R/(2f) - δ_m`, explicitly described as absorbing "the one-step linearization residual
   and the attitude-tracking residual." The code's `self._p_10 = center/focal` (controller.py
   line ~304) is the RAW value, used everywhere (`s_e_n` normalization, `p_10_eff`, `cbf2_filter`'s
   own `phi_max = p_10` per its module docstring) with NO margin subtracted anywhere. Either the
   manuscript needs correcting to match the deployed (marginless) barrier, or the margin needs to
   actually be implemented -- the theorem's stated safety buffer does not currently exist in the
   running system. **Not investigated further, not fixed. Flag before trusting the theorem's
   real-world margin claim.**

2. **`cbf_visibility_aruco.py`'s own Phase-2 rewrite (2026-08-13) has ZERO automated test
   coverage.** `tools/validate_cbf.py` only imports/tests `cbf_visibility.py` (cross-marker's
   Phase-2, the OLD logic). The ArUco-specific Phase-2 rewrite -- which is unconditionally active
   for every ArUco run, no working toggle (`CBF_PHASE2_FIX` is a dead switch, comments-only, see
   the theta_cap-era session notes) -- has never been independently validated the way Phase-1 has.
   Building a `validate_cbf_aruco.py` (or parametrizing the existing one over both files) would
   close this gap.

3. **Cross-marker's Phase-2 `radius`-derived margin has never been checked against a true camera
   model -- ATTEMPTED 2026-08-24, harder than it looks, NOT resolved.** `test_barrier_and_end_to_end`
   (checks 2/3, the ONLY checks with an independent ground-truth camera model) always passes real
   `corners` and hardcodes `radius=0.0`, correctly exercising Phase-1 (where radius doesn't matter,
   see above) but never Phase-2. Built a candidate check 9 (`marker_true_extent` -- ray-trace a real
   physical-size marker independent of the code's own model; prime Phase-1 with it; drive
   decode-failures to ramp Phase-2's `alpha`; re-project the SAME true marker at the resulting
   attitude): **passed 13/13 with worst-overshoot 0.000 over 2000 cases -- but a negative control
   (deliberately zeroing the `radius` input, i.e. removing the geometry term entirely) ALSO passed
   with zero overshoot.** The check was not discriminating at all; it gave false confidence, not a
   verification. Root cause: checks 2/3's `dstep<0.10` small-step validity gate (legitimate for
   Phase-1, whose QP anchors near the current tilt and naturally produces small corrective steps)
   does NOT transfer to Phase-2, which is a bare MAGNITUDE clamp on acceleration with no such
   near-current-state anchoring -- applying the same gate to Phase-2 either (a) never binds (my
   first attempt, theta_cone input accidentally dominated via `min(theta_cone, theta_tight)`,
   masking radius entirely) or (b) once the input-cap artifact was fixed, the gate never PASSES at
   all (0 valid cases out of 2000) because Phase-2's output routinely implies a large one-cycle
   tilt change. **The real difficulty: Phase-2's actual guarantee (per the design doc's
   `tau`=drift-look-ahead framing) is a bounded-ACCELERATION-over-a-HORIZON claim, not a
   same-cycle-attitude-teleport claim like Phase-1 -- testing it properly needs a real
   position/velocity rollout under the bounded `a_xy_lim` over `tau`, not an instant-attitude
   substitution.** Reverted the flawed check (`git checkout -- tools/validate_cbf.py`) rather than
   ship something that reports PASS without discriminating power. **This gap is real, confirmed
   non-trivial to close, and still open.** Relevant because cross-marker's IC5 diagnostics
   (2026-08-23/24, [[project_20260817_crossmarker_descent_stall_investigation]]-adjacent) showed
   40%+ detection failure rates -- exactly the condition that pushes Phase-2 (and this untested
   geometry) into frequent use on that IC. Building this properly needs a double-integrator
   position/velocity simulation under `a_xy_lim` over `tau`, not a one-shot attitude-substitution
   trick -- budget it as a real task, not a quick addition.

   **SCOPED as a standalone task 2026-08-24 (not started):**
   - Double-integrator lateral dynamics (position+velocity, world frame), driven each control
     cycle by the ACTUAL `a_xy` `cbf2_filter` returns that cycle -- not a one-shot jump.
   - Attitude assumed to instantaneously realize whatever tilt the commanded `a_xy` implies
     (explicit, flagged simplification -- matches how `theta_cap`/`A_CAP` already treat `a_xy`
     as directly deliverable elsewhere in this codebase; does NOT model real PX4 attitude-
     tracking lag -- note as a limitation, don't silently gloss over it).
   - Camera position must actually translate over the horizon (Phase-1's checks 2/3 fixed the
     camera and only varied attitude -- fine for a one-shot claim, NOT fine for Phase-2's
     multi-cycle horizon where translation is physically significant).
   - Iterate per REAL control cycle (dt~20ms) through a decode-failure streak long enough to
     exercise the full hysteresis+ramp (~8-10 cycles), re-solving `cbf2_filter` each cycle
     against the FROZEN Phase-1 state (`delta_ref`/`cr_ref`/`Lw2_ref`) exactly as the real
     controller does, integrating the TRUE trajectory forward with the actual clamped output
     each step.
   - Check CONTINUOUSLY (every cycle), not just at the end: ray-trace the true, fixed-geometry
     marker (independent of the code's own linear model, same principle as the reverted
     `marker_true_extent`) from the evolving true camera pose each cycle.
   - Sweep realistic ranges: initial lateral velocity INCLUDING NONZERO (a vehicle already
     drifting when decode fails is the realistic case, not starting from rest), decode-failure
     duration spanning both partial-ramp and full-ramp alpha, plus the existing geometry ranges
     (altitude, marker size, off-axis offset) reused from checks 2/3.
   - MANDATORY gate before trusting the result: re-run the SAME negative control that broke the
     reverted attempt (zero/corrupt the `radius` input) and confirm it NOW produces a real,
     clearly-attributable margin violation. Do not consider this closed until that control fails
     as expected -- that's the acceptance criterion, not just "the nominal case passes."

## `DRIFT_OFF` instrumented and confirmed live for cross-marker IC5 (2026-08-24)

Added `self._cbf_overflow_diag_log` (`(t, overflow, drift_off, d_min_fov)` per control step,
exposed as `"CBF Overflow Diag Log"` in `Control_Data.npy`) and re-ran cross-marker GT-FB IC2
IC5 (n=3, `test_data/CrossMarkerGTFB_OverflowDiag_IC5/`). Result: **`OVERFLOW` is 0% in every
rep -- it is genuinely `DRIFT_OFF` firing** (22-26% of control steps), not the benign
marker-filling-frame case. This confirms the `p_10_eff` pullback (the one live path from the
legacy `rho_fov` margin into the real barrier, see above) is actually engaging on IC5, not a
theoretical possibility.

**But `DRIFT_OFF` frequency alone does NOT separate success from failure**: the one SP rep
(25.9% drift_off) and both FAIL reps (22.4%, 24.6% drift_off) are all in the same range --
raw frequency isn't the discriminator.

## ROOT CAUSE of `DRIFT_OFF` on IC5: it's a correct, downstream symptom of the ALREADY-KNOWN
## unresolved lateral-convergence deficiency, not a CBF defect (2026-08-24)

Correlated `drift_off=True`/`False` timestamps against `|s_e_n|` (lateral centering error) and
`MARKER_EXTENT_PX`, same 3 reps. Identical pattern in all three:
- `DRIFT_OFF=True`: `|s_e_n|~0.72` (far off-center -- the FoV edge is 1.0 in these units),
  `MARKER_EXTENT_PX~211-215px`.
- `DRIFT_OFF=False`: `|s_e_n|~0.24-0.25` (well-centered), `MARKER_EXTENT_PX~311-339px`.

`DRIFT_OFF` fires exactly when the drone is badly off-center laterally -- extent is SMALLER
during `DRIFT_OFF` (not larger), ruling out a marker-size/overflow explanation. **The CBF is
correctly detecting a real visibility risk caused by insufficient lateral convergence, not
misclassifying.** Onset timing is also nearly identical across all 3 reps regardless of eventual
outcome (t~27.0s, 26.8s, 28.5s into ~35s flights) -- ruling out onset TIMING as the SP/FAIL
discriminator too; what must differ is whether the vehicle recovers centering before a spurious
loom sign-flip coincides with the still-off-center window, not whether/when it goes off-center.

**This is the SAME lateral-convergence-rate deficiency already documented as unresolved** in
[[project_20260813_cbf_extent_fix_followup]] / [[project_20260817_crossmarker_descent_stall_investigation]]
("ring-sampling's limited effectiveness directly tied to the still-unsolved lateral-convergence-
rate root cause"). IC5 (short 3m runway, fast descent) is that project's own documented "canary"
for exactly this class of failure. `DRIFT_OFF` did not introduce a new problem -- it is a
downstream SYMPTOM surfacing an old, still-open one.

**Not yet done**: confirm the recovery-timing hypothesis directly -- check whether the SP rep's
`|s_e_n|` actually drops back below the drift_off threshold before the loom sign-flip / touchdown
window, while the FAIL reps' doesn't. This would close the causal chain from "known lateral-
convergence deficiency" through "DRIFT_OFF" to "the specific false touchdown-latch event."

**CONCURRENT WORK (2026-08-24, another session)**: implementing a fix that slows descent when
`DRIFT_OFF` is active, using this finding as motivation. Two cautions flagged for that work,
both grounded in this session's own evidence, not speculation:
1. `DRIFT_OFF` fires ~22-26% of the time even in the SUCCESSFUL rep, not just failures -- any
   slow-descent response will engage frequently during normal operation, not just as a rare
   emergency case. Test that it doesn't degrade landing time/precision on the majority of
   flights where `DRIFT_OFF` fires transiently but recovery still happens fine anyway (which,
   per the finding above, is what actually distinguishes SP from FAIL -- not `DRIFT_OFF` itself).
2. **Real risk of reintroducing the Gazebo ODE auto-sleep bug** (see
   [[project_20260823_td_spike_regression]]): sustained near-zero-velocity dwell causes Gazebo's
   physics engine to auto-disable the rigid body. Any "slow/pause descent" response that holds
   low velocity for more than a couple frames should be explicitly tested for that exact
   signature (bit-frozen `Ground_Truth.npy` position across consecutive samples while `/clock`
   keeps advancing) before being trusted.

**Why:** three architecture/methodology misreads in one session (conflating rho_fov with the real
barrier; proposing to tune deprecated funnel params; a Phase-2 ground-truth check that passed
13/13 while being provably non-discriminating) all came from reasoning about the CBF, or about
what a check actually proves, without re-verifying against the actual code + a negative control.
**How to apply:** before any future claim about CBF correctness, margin behavior, or a proposed
visibility fix, (1) re-read this file, (2) run `tools/validate_cbf.py`, (3) grep the actual code
for the quantity in question rather than trusting a prior session's framing -- including this
file's own framing, if code has changed since 2026-08-24. **Before trusting ANY new correctness
check (here or elsewhere), run a negative control** -- deliberately corrupt/remove the thing being
verified and confirm the check actually fails; a check that always passes regardless of the input
is not a check.
