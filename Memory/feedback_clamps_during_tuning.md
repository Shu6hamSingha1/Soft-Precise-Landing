---
name: feedback_clamps_during_tuning
description: "PHILOSOPHY (user, 2026-06-08): clamps/thresholds are band-aids that MASK poor controller performance and HAMPER parameter tuning. DISABLE them while tuning (tune the bare control law), RE-ENGAGE the protective ones only AFTER the gains are tuned. Prefer proper control techniques (e.g. conditional integration) over fixed clamps. Includes the PX4-vs-MATLAB clamp audit (3 buckets)."
metadata: 
  node_type: memory
  type: feedback
  originSessionId: dd8920aa-4635-40a9-ac9d-8409e37243d6
---

**Rule (user, 2026-06-08): "Clamps are just band-aids, which are now masking the poor performance of the controller and hampering parameter tuning. It is better to engage these clamps and thresholds AFTER the parameters have been tuned."** So during a tuning campaign: turn the protective clamps OFF, tune the bare control law until it performs, then re-engage the safety clamps. A clamp that is *biting* during tuning is hiding the signal you need. Prefer a proper control technique (conditional integration, lead comp, gain shaping) over a fixed clamp.

**Clamp audit — PX4 controller.py vs MATLAB visualControl (the clean reference), 2026-06-08:**

- **Bucket A — canonical (IN MATLAB, load-bearing) → KEEP:** SMC boundary layers `sat(σ/E)` & `sat(σ_a/E_a)`; the FoV cone (`rho_fov → theta_cone → a_xy_limit → accel-floor −50`); `izeta` SMC-integral anti-windup; `S_margin`/`R33` asin/acos domain-safety clips (math, not control).
- **Bucket B — PX4-physics-necessary (NOT in MATLAB but model real SITL) → KEEP:** `W_U_MAX` (LK corner tracking breaks >1.7 rad/s — trial 8 proved it); `DH_D_MAX` (1/Z touchdown desired-flow spike); `W_I_MAX` (optic-flow noise); `YAW_TERMINAL_HOLD`/`STALE` (marker fills FoV → alpha unreliable near touchdown). MATLAB uses ideal features + torque output, so never needs these.
- **Bucket C — band-aids for the large-initial-yaw case MATLAB NEVER tests → RECONSIDER/REPLACE:** the **psi_d rate clamp** (`YAW_PSID_RATE` — MATLAB has NONE: `u_a_sat=u_a`; keep only as a `≤W_U_MAX` sanity bound, the 0.7 factor was over-conservative) and the **`_ie_a_clamp`** (yaw-integral, fixed 2.0 — MATLAB has NONE). Both bite ONLY because PX4 starts ~100° yaw and does a long slew (MATLAB spawns square — [[feedback_matlab_yaw_square_start]]).

**Done 2026-06-08:** **`_ie_a_clamp` REMOVED, replaced by CONDITIONAL INTEGRATION** in `_yawCtrl` — freeze `ie_a` while the heading-rate command is saturated (`abs(u_a) > _psid_rate`, flag set at method end, used next step). Proper anti-windup, **introduces no new threshold**. It fixes the windup that was driving the post-slew overshoot (the integral pinned at 2.0 left `σ_a = e_a + Ω_a·ie_a` positive at the zero-crossing → `u_a` kept commanding yaw → 102°→−22° overshoot). See [[feedback_dont_conclude_lag_floor]] (sibling: don't conclude lag-floor when a clamp masks under-tuned control), [[feedback_fix_causes_not_limits]] (fix saturation CAUSES, never widen the limit).
