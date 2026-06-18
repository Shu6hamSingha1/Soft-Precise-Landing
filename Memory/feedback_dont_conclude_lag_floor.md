---
name: feedback_dont_conclude_lag_floor
description: "METHODOLOGY RULE (user, 2026-06-07): when relaxing a SAFETY-NET clamp (CBF cone, etc.) makes a failure WORSE, do NOT conclude the clamp was 'load-bearing' or that the residual is the 'lag floor / architectural ceiling'. The clamp was MASKING an under-tuned controller — relaxing it EXPOSES a control-tuning gap to FIX at the control level. Exhaust control tuning before ever invoking 'lag floor'."
metadata: 
  node_type: memory
  type: feedback
  originSessionId: dd8920aa-4635-40a9-ac9d-8409e37243d6
---

**The mistake (made twice, 2026-06-07).** A safety-net clamp (`FUNNEL_MODE=cbf2` cone) was masking two failures. The pattern each time: relax the clamp (`THETA_FLOOR_DEG=60`) → the failure gets WORSE/visible. The CORRECT reading is **the clamp was hiding an under-tuned controller**; the wrong reading (which I made) is "the clamp is load-bearing" or "the residual is the architectural lag floor, escalate to uXRCE-DDS."

- **κ-runaway:** cbf2 masked it (clamped a_u 51–84%); relaxing exposed it (κ_z 96, a_u 16705). Fixed at the CONTROL level with `E` (boundary layer). ✓ (we got this one right)
- **Lateral drift:** cbf2-relaxed made the lateral WORSE (mean 6→10 m). I WRONGLY concluded "cbf2 containment was load-bearing → lateral drift = lag floor." **User: WRONG — cbf2-relaxed is EXPOSING that the lateral controller needs tuning.** The very `E_X=E_Y=2.5` I'd set to bound κ **softened the lateral SMC** (large boundary layer = weak correction near equilibrium) → the drone can't drive lateral flow to zero → drift. Fix = TUNE the lateral (bring `E_X/Y` down to stiffen, use `P_X/Y` to hold κ, raise `GAMMA_X/Y` to brake harder).

**RULE 1 — never reach for "lag floor / architectural ceiling" while control levers remain.** A masked-then-exposed failure is a TUNING TARGET, not proof of an unfixable floor. The 125 Hz control loop is fast (loop-rate is NOT the limit); the actuation lag (38–61 ms roll / 287 ms yaw) is real but is the LAST resort, not the first explanation. Treat a relaxed safety net like the κ case: it honestly shows what the controller can't yet do — then go tune it.

**RULE 2 — one knob, one job; beware coupling.** `E` (boundary layer) was doing TWO jobs and they conflict: it bounds κ (raise E) AND sets SMC stiffness (raise E → softer → worse tracking). High `E_Z`=2.5 bounded z-κ but killed descent (hover); high `E_X/Y`=2.5 bounded lateral κ but softened lateral hold (drift). The right decomposition: **use `P` (leakage) to bound κ** (κ_eq∝1/P, independent of stiffness — trial 3 had κ bounded at E=1,P=3), and **use `E` only for stiffness/chatter**, tuned PER-AXIS. Don't let a κ-bounding move silently detune tracking.

See [[feedback_newcal_tuning_results]] (the trials), [[feedback_precision_softness_frontier]] (the "lag floor" claim that this rule cautions against over-trusting), [[feedback_fix_causes_not_limits]] (sibling rule: fix saturation CAUSES, never widen the limit).
