---
name: project-q8-yaw-ff-harmful-with-headroom
description: Q8 step 1b — at 0.30 rad/s (real headroom, unlike the 0.48 rad/s ceiling test), PLASMC_YAW_WT_FF is actively HARMFUL, not neutral — turns near-zero e_a into a 60-70deg standing lag. Likely double-counting against the ASMC's own integral tracking of a constant-rate disturbance.
metadata:
  type: project
---

**Supersedes/extends [[project_q8_yaw_ff_dead_sin_ceiling]].** That entry showed perfect
feedforward is USELESS at 0.48 rad/s (the SO(3) `sin(Δψ)` ceiling, ~4% headroom). This entry
tests the natural follow-up — a rate WITH real headroom — and the result is not "no effect", it
is **actively worse**. n=3/arm, GT-feedback, cross-marker.
Data: `test_data/Q8_SpinFF_r30/{off,ff}_rep{1,2,3}`. Harness:
`test_data/Q8_SpinFF/q8_probe_n3.sh` (0.30 rad/s variant of the original probe).

## The result — tight, repeated, opposite-of-hypothesized

| | `e_a` steady-state (last 20%) | `u_a` steady-state |
|---|---|---|
| off (n=3) | +0.5°, +3.3°, +3.1° (SD≈1.3°) | −0.196 |
| **ff (n=3)** | **−69.1°, −64.4°, −59.9°** (SD≈4.6°) | **−1.647** |

`off` ALREADY converges to near-zero steady yaw lag with NO feedforward. `ff` (exact injected
0.30 rad/s fed forward) produces a **60-70° standing error** — worse by nearly two orders of
magnitude, same sign both directions, tight across all 3 reps of each arm (not a flake).

## Why — inference, not fully nailed down, but consistent with every signal

The yaw ASMC's own integral term (`ie_a`, gain `Ω_a`) is a type-1 servo element. Against a
**constant-rate** disturbance (exactly what `PLASMC_GT_SPIN_WZ` injects), a type-1 loop can drive
steady-state error to zero via the internal-model principle — consistent with `off`'s near-zero
lag WITHOUT any feedforward: the loop was already solving this on its own.

Adding a constant feedforward on top of a loop already converging to the right rate is
**double-counting**: `psi_d` races ahead of the true target heading, `alpha`'s error grows in the
OPPOSITE direction, and the ASMC's own adaptive/integral action winds up hard fighting a problem
the feedforward itself created — matching `u_a`'s 8× blow-up (−0.196 → −1.647, same sign) and the
sign-consistent 60-70° lag. Not independently verified by isolating the double-count algebraically
— treat as the leading explanation, not a proven one.

## Combined picture across both tests — NO regime found where FF helps

| rate | headroom | result |
|---|---|---|
| 0.48 (the documented turning case, the ceiling) | ~4% | FF makes no difference — saturated regardless of reference |
| 0.30 (real headroom) | 50% | **FF actively hurts** — induces 60-70° lag where none existed |

## Consequence for the design

⛔ **Do not build a measured-`w_z` feedforward that ADDS to the existing ASMC command.** The
mechanism that would make it help (closing a lag) is the same mechanism that makes it actively
destabilizing wherever the loop is not already saturated — which is most of the flight envelope.
This is stronger than "don't bake `PLASMC_YAW_WT_FF`" (already true,
[[project_q8_yaw_ff_dead_sin_ceiling]]): it argues against the *class* of solution the task spec
proposed as an addition to `_ua_psid`, not just against baking a specific constant.

**The `w_z`-regulation idea is NOT killed, but its justification narrows to exactly one thing:
bypassing the `sin(Δψ)` ceiling at high turn rates**, where a rate-error command structurally
cannot saturate the way an attitude-error command does (`|w_u[2]| = K_R_yaw·sin(Δψ) ≤ 0.5` is the
hard limit; commanding from rate error removes that specific nonlinearity). It is NOT justified as
a general lag-reduction feedforward — the loop doesn't have a lag to reduce in that regime, and
adding one where it does have headroom breaks a loop that was already working.

**Next: prototype the rate-command path scoped to this ceiling case specifically** — replace or
blend the attitude-error yaw command with a rate-error command only where/when the sin-ceiling is
actually binding, rather than as a standing feedforward addition. See
[[project_q8_yaw_ff_dead_sin_ceiling]] for the ceiling mechanism this targets.
