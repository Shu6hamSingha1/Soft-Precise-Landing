---
name: feedback_dont_judge_cbf_by_sp
description: When gating a visibility-CBF change (e.g. CBF_DRIFT_TAU), judge by CBF BEHAVIOUR — trigger correctness + whether the safe set is held — NOT by Soft-Precise / landing outcome. SP is dominated by SITL noise and downstream control/perception, not the CBF.
metadata:
  type: feedback
---

**2026-09-09.** Ran an IC2-5 stationary A/B for `CBF_DRIFT_TAU` 0 vs 0.15
(`DriftTauConfirm/20260909-200537`) and called it a **failure** because
`τ=0.15` had P+S 11/20 vs 14/20 and one IC4 rep touched down at 2.80 m/s.
Reverted the change. User: *"If you are using SP to judge it was a failure, then
you are completely wrong."* Correct.

**Why SP is the wrong lens for a visibility-CBF gate:**
- SP (precise/soft landing) at n=5 in SITL is bimodal noise — flights die anywhere
  2-20 s, IC startup flakes ~50%. Most of the "regressed" reps had `vis_active=0`
  (CBF completely inert that rep) — the CBF had nothing to do with the outcome.
- The CBF's job is a geometric safety guarantee: keep the marker centre inside the
  buffered FoV / on the physical sensor. That is what to measure.
- A single hard touchdown can be terminal-overfill perception corruption
  ([[project_20260901_rover_cross_perception_diagnosis]]) showing through — a
  garbage-in problem, not a CBF-behaviour problem.

**Judge a visibility-CBF change by, per arm:**
1. **Safe set held?** frames with `|c|` past the physical FoV edge (must stay ~0);
   peak `|c|/φ`.
2. **Trigger correctness:** does `vis_active` fire when the predicted `c_next`
   genuinely breaches, and (mostly) not otherwise? TPR on "near edge & moving
   out" frames; note that firing while `|c|` is still small but `d` is real
   drift is *correct early anticipation*, not a false positive.
3. **Safe-input quality:** when Tier 1 acts, is the modification minimal and
   outward, and does `|c|` improve after?
4. **Reaction lag:** frames from `|c|/φ` crossing a threshold until `vis_active`.

Judged this way, `CBF_DRIFT_TAU=0.15` held the stationary safe set exactly as well
as `τ=0` (0 sensor exits both) and closed a real moving-target gap (`τ=0`: 278
centre-off-sensor frames / 24 rover reps) → **baked**. See
[[project_20260909_visibility_projection_wire_in]].

**General principle:** match the metric to what the component guarantees. The
outer landing metric belongs to the whole stack; a barrier function is judged on
its invariant.
