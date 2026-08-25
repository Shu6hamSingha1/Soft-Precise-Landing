---
name: project_20260824_dtheta_az_filter_self_defeating_feedback
description: "PLASMC_DTHETA_AZ_GAIN sweep at IC5 (cross-marker) found the az visibility filter (dtheta = ||theta_desired - theta_safe||, replacing the loom-margin approach) is structurally self-defeating -- higher gain shrinks the CBF's OWN solved lean angle AND the real delivered lateral accel |I_a[:2]| via a multi-cycle attitude-history feedback loop through cbf2_filter's th_curr (NOT a same-cycle a_z-division artifact -- that would be self-canceling; ruled out by ordering + the |I_a[:2]| measurement). Not a gain-tuning problem; A_CAP saturation ruled out as the cause with hard data. CLOSED same day: theta_desired now logged, ratio confirmed shrinking (1.25->0.95, gain 5->40) -- direct smoking gun for QP-side over-constraint."
metadata: 
  node_type: memory
  type: project
  originSessionId: db521dfb-a6f1-44ca-b3f6-432e98fc4866
  modified: 2026-08-24T11:08:46.644Z
---

Investigated 2026-08-24, following up the `dtheta` az-visibility-filter replacement (commit `7a33392`, see [[reference_cbf_visibility_architecture]] for the underlying CBF architecture this builds on) after a real A/B showed it trading better `xy_err` for worse `rel_vel` against the old loom-margin approach (IC2, n=3 each). User asked to investigate whether gain tuning fixes this, at IC5 (the most extreme IC, evaluated mechanistically -- `dtheta` firing rate + `I_a[2]` shift magnitude -- not by SP, since SP is confounded by the separate off-center kappa-leakage wall, see [[project_20260824_crossmarker_offcenter_convergence_wall]]).

## Sweep: `PLASMC_DTHETA_AZ_GAIN` in {5,10,20,40}, IC5 cross-marker, n=3 each

`test_data/DthetaGain_IC5/20260824-142606/`. No stalls in this sweep (all flights 1.1-22.8s -- the Gazebo ODE auto-sleep artifact documented in [[project_20260823_td_spike_regression]] did not confound this data).

**`dtheta` fires far more at IC5 than IC2** -- 65-98% of frames (vs IC2's ~20-23%), i.e. the CBF is constraining lateral authority almost continuously at this IC.

**The correction magnitude scales correctly with gain** (mean `I_a[2]` shift on active frames): gain 5 -> -0.6 to -1.0, gain 10 -> -0.8 to -1.5, gain 20 -> -1.5 to -2.2, gain 40 -> -2.3 to -2.8. Roughly 4x increase, confirming the mechanism itself isn't damped away by the LPF at this IC (unlike the brief/rare IC2 firing pattern, where net effect was tiny).

**But outcome metrics show NO corresponding trend**: `xy_err` stays flat ~2.42-2.77m across all four gains; `rel_vel` bounces 1.0-1.6 mean with no monotonic improvement despite the 4x range in actual correction magnitude applied.

## Root cause: NOT saturation, a genuine self-defeating feedback loop

**Ruled out first, with hard data**: `A_CAP` (deliverable-thrust-magnitude cap, see its top-of-file comment in `controller.py`) saturation is **0.0% at every gain tested**, even gain=40. The earlier concern (that az correction could steal lateral authority via the magnitude-cap rescale) does not explain this data.

**What actually happens**: both `dtheta` itself and `theta_cone` (the CBF's own solved lean-angle magnitude, `||theta_safe||`) **shrink monotonically with gain**:

| gain | mean dtheta (active frames) | mean theta_cone (active frames) |
|---|---|---|
| 5  | 0.164 rad | 12.08 deg |
| 10 | 0.125 rad | 10.53 deg |
| 20 | 0.098 rad | 7.71 deg |
| 40 | 0.070 rad | 6.69 deg |

Mechanism, REFINED 2026-08-24 after user pushback on an imprecise first pass (see below): `cbf2_filter` reads the vehicle's ACTUAL current attitude (`R`, `R33`) each cycle to anchor its FoV-box projection (`th_curr` in the QP, `cbf_visibility.py`/`cbf_visibility_aruco.py`). When the az correction commands extra lift, the vehicle's realized attitude trends more upright over subsequent cycles. That more-upright attitude feeds back into a LATER cycle's `th_curr`, shifting the QP's anchor point -- the solved `theta_safe` comes out smaller. Higher gain -> more lift -> more attitude drift toward upright -> smaller solved `theta_safe` -> less lateral authority actually delivered by the CBF's own QP.

**Correction to the first-pass explanation (do not repeat this mistake)**: initially framed this as the correction "directly cannibalizing a_z in theta_desired = a_xy/a_z" -- WRONG, because the dtheta correction runs AFTER cbf2_filter returns (deliberately, so it can't contaminate that cycle's own QP solve); the a_z used inside theta_desired's formula this cycle is read BEFORE this cycle's correction touches I_a[2]. No same-cycle division effect exists.

**The self-canceling argument this reframing exposes (why "more az to slow descent" should be FREE, not costly)**: physically, more vertical thrust for the SAME lateral force a_xy needs LESS lean angle, not more sacrifice (same conclusion as the much earlier A_CAP-ordering discussion). If the CBF simply passed theta_desired through unchanged (theta_safe~=theta_desired), delivered lateral thrust I_a[:2] = a_z*theta_safe ~= a_z*(a_xy/a_z) = a_xy -- SELF-CANCELING, i.e. growing a_z should be free: smaller angle, same real force, visibility unaffected.

**Why the data rules that optimistic story out**: `|I_a[:2]|` (the REAL delivered lateral acceleration, not just the angle) was directly measured shrinking with gain (1.77->1.68->1.25->1.15, from the A_CAP-saturation check above). Given the self-canceling argument, that shrinkage can ONLY happen if theta_safe comes out SMALLER THAN PROPORTIONAL to a_z's growth -- i.e. the QP is delivering a shrinking FRACTION of an already-shrunk theta_desired ask as gain increases. That points specifically at the multi-cycle th_curr/attitude-history feedback path as the actual driver, not a formula artifact -- the QP genuinely over-constrains more as gain rises, not just "the ask itself got smaller."

**CLOSED 2026-08-24 (same day, follow-up):** logged `theta_desired(t)` to `Control_Data` (norm of `_th_desired`, NaN on the Phase-2 no-projection fallback; `controller.py` ~line 1417/3108/3482). Re-ran 2 confirmation reps at IC5 cross-marker, gain 5 and gain 40 (`test_data/DthetaDesired_IC5/20260824-163554/`, both landed). **Ratio confirmed shrinking**: mean `theta_safe/theta_desired` = 1.2543 (gain 5, n=365 frames) -> 0.9529 (gain 40, n=237 frames), while mean `dtheta` collapsed 0.2264 -> 0.0344 over the same reps. This is the direct smoking gun: the QP delivers a strictly shrinking FRACTION of the (already-shrunk) desired tilt as gain rises, not just a smaller ask -- confirms QP-side over-constraint via the attitude-history loop, no residual formula-artifact doubt remains. Note the >1 ratio at gain 5 is expected/consistent, not a contradiction: `theta_cone` is the POST-`theta_cap`-rescale logged value (a separate deliverability clip applied after `cbf2_filter` returns, see controller.py:3117-3123), and `dtheta` is a vector-difference norm (`||th_desired - th_safe||`), not a scalar-norm difference -- so norm-ratio and vector-distance can disagree in sign at low gain without invalidating the trend at higher gain.

**The mechanism is undermining its own purpose**: it exists to slow descent so the lateral controller has time to converge, but the act of slowing descent changes the attitude history in a way that makes the CBF solve for a SMALLER corrective lean, directly suppressing the lateral correction it was supposed to protect.

## Why this is not a tuning problem

No gain value fixes a mechanism that is structurally fighting itself through this attitude-feedback path -- the negative trend (shrinking `theta_cone`/`dtheta` with increasing gain) is the SAME mechanism at every gain tested, just proportionally stronger. A fix would need to break the loop, e.g.:
- Anchor `cbf2_filter`'s `th_curr` reference against the DESIRED (pre-az-correction) attitude trajectory rather than the realized one, or
- Decouple the az correction's effect on subsequent-cycle attitude from what `cbf2_filter` reads as "current tilt."

Both are real design changes to `cbf_visibility.py`/`cbf_visibility_aruco.py`, not a `controller.py`-side gain adjustment. Not implemented/attempted yet.

## Status

`PLASMC_DTHETA_AZ_GAIN` left at its placeholder default (10.0) -- no gain in the tested range is structurally better than another given this feedback loop. The `dtheta` approach's A/B advantage over the old loom-margin approach (better `xy_err`, worse `rel_vel`, at IC2) should be read in light of this: IC2's much rarer `dtheta` firing (~20% vs IC5's ~65-98%) likely limits how much the feedback loop bites there, which may be WHY IC2 showed a net-positive trade rather than the wash/potential-net-negative seen at IC5. Not yet checked whether the OLD loom-margin approach has an analogous feedback vulnerability (it modifies `I_a[2]` BEFORE `cbf2_filter` runs, using the SAME-cycle `a_z` scale rather than a prior-cycle attitude reference -- structurally different, plausibly immune to this specific loop, but unverified).
