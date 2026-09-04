---
name: project-q8-omega-d-ff-fixes-ceiling
description: Q8 step 2 — the EXISTING PLASMC_YAW_OMEGA_D_FF knob (default OFF), re-tested at the 0.48 rad/s turning-rover ceiling, fixes the ramp-windup. Rate tracking 11%->107%, e_a lag -136deg->-19deg, xy 0.844->0.122. Clean n=3/arm, attribution-verified.
metadata:
  type: project
---

**Q8 step 2 result: the existing (default-OFF) `PLASMC_YAW_OMEGA_D_FF` knob fixes the
turning-rover ceiling ramp-windup.** Re-tested rather than newly built — see

⚠ **NAMING CORRECTION (2026-09-04, user question "how are we getting omega_t for
feedforward?"): this is NOT an ω_t feedforward.** `_ua_psid_ff = clip(u_a, ...)` —
`u_a = Γ_a·σ_a + sat(σ_a/E_a)·κ_a + Ω_a·e_a` is a pure function of the MEASURED `e_a`; `ω_t`
never enters this computation. It only enters implicitly as the disturbance `d_α = l_α^T ω_t`
that the feedback loop is already reacting to. The mechanism is: reuse the outer loop's own
already-computed correction (`u_a`) on a second path to the actuator that skips the
`-K_R_yaw·sin(Δψ)` round-trip — NOT better knowledge of the disturbance. No `ω_t`/`w_z`
estimate is needed or used. Contrast with the task spec's original proposal (regulate measured
`w_z` toward `α̇_des`), which WOULD need `ω_t`/`w_z` and has not been built —
`OMEGA_D_FF` sidesteps that need entirely.
[[project_q8_yaw_ff_dead_sin_ceiling]] and [[project_q8_yaw_ff_harmful_with_headroom]] for why the
naive additive-feedforward approaches were ruled out first.

Data: `test_data/Q8_SpinFF_omegaff/{off,on}_rep{1,2,3}`, n=3/arm, GT-feedback, cross-marker,
`PLASMC_GT_SPIN_WZ=0.48` (the documented turning case). Harness:
`test_data/Q8_SpinFF/q8_probe_omegaff.sh` — attribution-verified (see
`SH_REFERENCE.md` pitfall 11; a prior run of this same probe silently attributed 3/6 reps to a
concurrent session before the fix, see commit `ca23a617`). All 6 reps here confirmed genuinely
this run's config before analysis.

## The result — tight, clean, large effect

| | `e_a` steady-state (last 20%) | rate tracking | `u_a` steady-state | xy_err |
|---|---|---|---|---|
| off (n=3) | −136.0° (sd 2.1°) | 11% | −3.03 | 0.844 |
| **on (n=3)** | **−19.0° (sd 0.6°)** | **107%** | **−0.51** | **0.122** |

Both arms replicate tightly (SD 0.6-2.1° across reps) — a real, deterministic effect, not noise.
`on_rep2` lands fully precise (xy=0.044); none of `off` come close.

## Why it works — bypasses the sin(Δψ) ceiling, not more gain

`w_u[2] = -K_R_yaw·e_R,z` is hard-capped at `±K_R_yaw=0.5` because `e_R,z = sin(Δψ)` is bounded by
construction ([[project_q8_yaw_ff_dead_sin_ceiling]]). `PLASMC_YAW_OMEGA_D_FF` adds
`_ua_psid_ff` **AFTER** that term (`controller.py` ~4013, `w_u[2] += _ua_psid_ff`), and the SUM is
clipped only at the much higher `W_U_MAX=2.0` — so the additive term has real room past 0.5.

It also avoids the double-counting failure mode that made the naive `PLASMC_YAW_WT_FF` test
actively harmful: `_ua_psid_ff` is the controller's OWN already-computed rate
(`_ua_psid`, the rate-limited `u_a`), fed to the inner loop through a second path — not an
independently-sourced disturbance estimate stacked on top of what the ASMC already compensates.
`u_a` shrinking 6× (−3.03→−0.51) confirms this: the ASMC is RELAXED with the FF on, not fighting
it — the opposite signature of the harmful 0.30 rad/s case.

## Scope, per the task spec — this is NOT a general feedforward

Its own code comment (`controller.py` ~4005) already scoped it correctly:
*"helped CONSTANT-rate yaw (Circular e_a 180→53 deg) but broke the oscillating Sinusoidal
reference (feeds back the ASMC's own lagged output → circular for fast-varying u_a) → use for
constant-rate rotating targets (the rover), keep OFF elsewhere."* Combined with
[[project_q8_yaw_ff_harmful_with_headroom]] (harmful where the loop has headroom, i.e. NOT
saturated), the justification narrows to exactly: **constant-rate targets, at/near the sin-ceiling
only.** Do not default this on generally — it needs gating to the regime it was shown to help in.

## Not yet done

- Only tested at 0.48 rad/s (the ceiling). Not re-tested at 0.30 (where the naive FF was harmful)
  to confirm this mechanism doesn't share that failure mode there too — worth checking before any
  bake, since the source differs but the "add a rate term" shape is superficially similar.
- Sinusoidal-reference regression (the documented reason it's default-OFF) not re-verified against
  current HEAD — the comment is from an earlier architecture.
- IC2-5 gate not run. This is a turning-target-specific mechanism; existing IC-gate infra assumes
  a stationary target, so the gate methodology itself needs adapting before this can be
  considered for a default flip.
