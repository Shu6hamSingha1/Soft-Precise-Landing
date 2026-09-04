---
name: project-yaw-rate-law-sign-bug-and-validation
description: PLASMC_YAW_RATE_LAW (new yaw control, direct integrator on w_z bypassing psi_d/e_R) — a real actuation-chain sign bug was found+fixed via measurement; GT-feedback validation is a clean win INCLUDING beyond the old sin(dpsi) ceiling; real-perception is NOT yet safe (w_z inherits terminal-overfill corruption) — needs a confidence gate next.
metadata:
  type: project
---

**STATE as of 2026-09-05: `PLASMC_YAW_RATE_LAW` exists in `controller.py`, default OFF, GT-feedback
VALIDATED (clean win, including beyond the old ceiling), REAL-PERCEPTION NOT YET SAFE (known,
diagnosed cause). This is the natural pick-up point for a new session.**

## The design (context: [[project_q8_omega_d_ff_fixes_ceiling]], [[project_q8_yaw_ff_harmful_with_headroom]])

User's redesign brief: use `α` and `ω` (specifically `w_z`, the flow-lstsq rotation component,
`self._w_i[-1][2]`) directly, without ever computing `ω_t` as a named quantity. Kinematic identity
(validated in `gt_feedback.py`, `-0.91` correlation with real body yaw rate): `e_a_dot = -w_z`
exactly — `w_z` already IS `e_a`'s derivative, flow-measured, no differentiation of `α` needed.
Direct integrator: `d(w_u[2])/dt = k_p·e_a − w_z` (see the ⚠ SIGN note below — this is the
CORRECTED form). One integrator only (unlike the ASMC's separate `ie_a`), so nothing to
double-count against. Bypasses `psi_d`/`e_R[2]`/`-K_R·sin(Δψ)` for yaw entirely — `psi_d := yaw_c`
is set directly in `_attCtrl` so `R_d`'s heading basis (needed for roll/pitch IK) stays correct.

Implementation: `controller.py` `__init__` (env reads + derivation comment, ~line 581),
`_yawCtrl` (computation, unconditional — runs alongside the ASMC for comparison even when not
driving output), `_attCtrl` (application — REPLACES `w_u[2]` entirely, mutually exclusive with
`OMEGA_D_FF`). Knobs: `PLASMC_YAW_RATE_LAW` (default 0), `PLASMC_YAW_RL_KP` (default 0.3),
`PLASMC_YAW_RL_KI` (default 0.0 — light robustness term, off by default). Logged:
`yaw_rl_cmd(t)`, `yaw_rl_ie(t)`. Recorded in `Control_Params.npy`'s resolved config.

## ⚠⚠ SIGN BUG found and fixed 2026-09-05 — read before touching this law again

**The actuation chain from `w_u[2]` to the ACHIEVED body yaw rate is INVERTED in this codebase.**
Measured directly (GT poses, independent of the law's own state): commanding `w_u[2]=+2.0 rad/s`
(saturated) produced an ACTUAL drone yaw rate of **−2.03 rad/s**. `psi_dot_b_TRUE ≈ -w_u[2]`, not
`+w_u[2]`. Substituting into `w_z = ω_t,z − ψ̇_b` (itself confirmed correct — independently verified
by integrating `-w_z` against measured `e_a` on an unrelated rep, trend matched) gives
`w_z = ω_t,z + w_u[2]` — a POSITIVE gain from command to `w_z`. The FIRST-WRITTEN law
(`Δw_u2 = w_z − k_p·e_a`) put `w_u2` on the RHS of its own update with coefficient **+1**
(`dw_u2/dt = w_u2 + …`), an unstable ODE — exactly the observed runaway to saturation
(first SITL attempt: `e_a` steady-state −39° with `max|a_u|` up to 1663, rate tracking −421%).
**Corrected form: `Δw_u2 = k_p·e_a − w_z`** (both terms flip, not one — re-derived cleanly from
the confirmed relation, not patched ad hoc).

**How this was found — worth the process note.** Two intermediate hypotheses were tried and
discarded before landing on the real cause, each ruled out by direct evidence rather than more
guessing: (1) closed-loop sample correlation between the command and `w_z` — invalid methodology,
confounded (both signals are functions of each other through the law itself); (2) my own
hand-rolled quaternion→yaw formula being wrong — ruled out by comparing against this project's own
`ahrs.Quaternion([w,x,y,z]).to_angles()[2]` convention on the same samples, matched to 1e-15 deg.
The decisive check was measuring the ACTUAL yaw rate from raw GT poses during a saturated window
and comparing directly against the command — cheap, independent, unambiguous.

**Offline verification before every SITL attempt** (the discipline that kept this cheap): a
closed-loop Euler-integration simulator mirroring the exact discrete update, tested against
whatever plant relationship was believed true at each point. It caught nothing the first two times
(both simulator runs "converged" — they were self-consistent with their own wrong assumptions, not
validated against reality) — the lesson: **an offline sim only proves your algebra is internally
consistent; it does not prove the assumed plant relationship is the real one.** Only a real SITL
measurement settled it.

## Results — GT-feedback: a clean, decisive win

n=3/arm (except the smoke tests, n=1), attribution-verified
(`test_data/YawRateLaw/{ceiling048,beyond060}`), cross-marker, `PLASMC_GT_FEEDBACK=1`.

| test | `e_a` steady-state | `max\|a_u\|` | xy_err |
|---|---|---|---|
| ceiling 0.48 rad/s, `off` (baseline ASMC) | −137 to −142° | 6–447 | 0.60–2.74 |
| **ceiling 0.48 rad/s, new law** | **−0.5 to −0.9°** | 1.9–6.2 | 0.03–0.27 |
| **0.60 rad/s — unreachable by ANY prior mechanism** | **−0.6 to −1.0°** | 2.0–2.7 | **0.11–0.12** |

The 0.60 rad/s result is the one that matters most: `OMEGA_D_FF` and every `e_R`-routed command
are hard-capped by `K_R_yaw·|sin Δψ| ≤ 0.5 rad/s` — this is a genuinely new capability, not an
incremental improvement, and it's tight/repeatable (not a lucky single rep).

## ⛔ Real perception: NOT yet safe — diagnosed, not a design flaw

Stationary regression check (`test_data/YawRateLaw/stationary/`, real perception, no GT-FB,
`PLASMC_YAW_RATE_LAW=1`, IC1-5 n=1): IC4 regressed badly — `yaw_rl_cmd` saturated to −2.0,
xy=0.741 (vs the OLD/buggy-sign smoke test's 0.166 on a similar config, i.e. genuinely worse than
even the broken version, because THAT one happened to be locally stable for ω_t=0 specifically).

**Root cause, confirmed via direct correlation with `MARKER_EXTENT_PX` and altitude**: real
(unfiltered) `w_z` tracks marker extent almost exactly — stays small while `extent<~250px`, then
grows monotonically as extent saturates near the frame (318px) and altitude drops below ~1m,
reaching >1.0 rad/s on a target that is not rotating at all. **This is the documented terminal-
overfill mechanism** (already the #1 open blocker for `h_y`, [[project_20260901_rover_cross_perception_diagnosis]])
extended to a NEW channel (`w_z`) that had not previously been load-bearing for anything, so this
corruption mode was never exposed. The OLD ASMC path happened to be incidentally protected — its
damped/adaptive structure reacts more slowly to one bad signal than a pure unfiltered integrator.

**This is not a flaw in the sign fix or the design** — GT-feedback (exact `w_z`, no corruption
possible) validates cleanly. It's that the new law has zero filtering and zero confidence gating
on its one input, inheriting real perception's known failure mode directly.

## Next step (not started)

A confidence gate on `w_z`, using `MARKER_EXTENT_PX` (or a rate-of-change guard on `w_z` itself) to
freeze or blend toward the old ASMC path once overfill is detected — exactly what the original
task spec's own step 3 anticipated ("wire the measured path, gated on decode confidence, fall back
to adaptive rejection when confidence is low"), now with a concrete, measured reason it's needed
rather than a generic precaution. Do this before considering `PLASMC_YAW_RATE_LAW` for any default
flip or further real-perception testing.

## Also still open (unchanged from the Q8 investigation)

- No IC2-5 gate run yet (this is a turning-target mechanism; the existing gate assumes a
  stationary target, needs adapting).
- `k_i` (light integral robustness term) untested — default 0.0, pure P+rate-cancellation only.
- The ASMC/`psi_d`/CV-KF removal question (user asked "is it okay to remove both") — NOT yet acted
  on. Recommendation stands: keep the ASMC running (cheap, useful for comparison/fallback — and
  now, per the gating need above, likely load-bearing as the fallback target) until the confidence
  gate is built and the real-perception path is proven safe; only then reconsider removal.
