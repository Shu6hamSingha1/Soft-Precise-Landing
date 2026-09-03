---
name: project-q8-yaw-ff-dead-sin-ceiling
description: Q8 step 1 result — perfect ω_t feedforward does NOT fix turning-rover yaw ramp-windup; the real ceiling is |w_u[2]| = K_R_yaw·sin(Δψ) ≤ 0.5 rad/s, downstream of ψ_d
metadata:
  type: project
---

**Q8 perfect-knowledge test, 2026-09-03. Result: the feedforward line is DEAD. Do not bake it.**
Data: `test_data/Q8_SpinFF/{off,ff}`. Probe knob `PLASMC_YAW_WT_FF` (rad/s, default 0.0 = exact
no-op) adds a target-rate feedforward to `_ua_psid` (the rate `psi_d` integrates at) — deliberately
NOT to `u_a`, so it stays out of `sigma_a`/`kappa_a` (α is relative degree 1 w.r.t. `psi_b_dot`).

## The test

GT-FB, `PLASMC_GT_SPIN_WZ=0.48` (27°/s, the documented turning case). Arm `off` = no FF;
arm `ff` = the **exact injected rate** fed forward. Perfect knowledge, by construction.

| | `e_a` end | `u_a` end | \|last 20%\| mean |
|---|---|---|---|
| off | −159.8° | −3.547 | 2.457 |
| **ff (perfect)** | −162.1° | −3.509 | 2.533 |

**Indistinguishable.** The knob genuinely applied — `Control_Params.npy` records
`PLASMC_YAW_WT_FF: '0.48'` in `ff` and absent in `off` (the resolved-config dump added the same day
is what ruled out a plumbing failure in one command instead of by argument).

⚠ The `ff` arm landed closer (xy 0.310 vs 0.669) — **do not read anything into this.** n=1, and every
yaw signal is unchanged, so there is no mechanism by which a yaw feedforward produced it.

## Why it cannot work — the ceiling is downstream of ψ_d

Measured in `Control_Data`: `e_R(t)[:,2]` ∈ [−0.997, +1.000] (saturated at ±1) and
`w_u(t)[:,2]` ∈ [−0.500, +0.498] = exactly ±`K_R_yaw`·1.0.

`e_R = ½ vee(R_dᵀR − RᵀR_d)`, so for a pure yaw error its z-component **is sin(Δψ)** — bounded by 1
*by construction*. Hence

    |w_u[2]| = K_R_yaw · |sin Δψ|  ≤  K_R_yaw = 0.5 rad/s

a HARD ceiling no amount of `psi_d` slew can exceed. The feedforward acts on `_ua_psid`, i.e.
UPSTREAM of this, so it cannot help. Worse, the ceiling is **non-monotonic in the error**:

| Δψ | max yaw-rate cmd | vs 0.48 demand |
|---|---|---|
| 10° | 0.087 | unreachable |
| 30° | 0.250 | unreachable |
| **74°** | 0.481 | first point it can match |
| 90° | 0.500 | peak |
| 120° | 0.433 | unreachable again |
| 160° | 0.171 | collapsed |

To merely MATCH 0.48 rad/s the loop must first fall **74° behind**; past 90° its authority *decreases
as the error grows*. That is the runaway, and `e_a` ending at −160° sits exactly where sin has
collapsed to a third of peak. Achieved drone yaw rate was only 0.155-0.179 rad/s = **32-37%** of
demand, while `|u_a|` exceeded the `psi_d` clamp (2.0) in 34% of samples — so the clamp is NOT the
binding limit either (it is 4× the demand).

**This is not integral windup.** [[project_rover_turning_open]] mechanism 1 ("yaw-loop ramp windup…
candidate fix: feed the observable target rate as a FEEDFORWARD") is **falsified as stated**. The
task spec's own fallback was right: "more likely an integrator/wrap/saturation issue than a missing
FF" — specifically an SO(3) attitude-error saturation.

## What this does NOT kill, and the actual fix direction

The `w_z`-regulation idea survives, but **for a different reason than feedforward**: it bypasses the
sin() bottleneck. The spec anticipated this — *"if w_z supplies the rate, this
integrate→compare→differentiate round trip becomes optional."* That round trip is precisely what caps
the command at `K_R_yaw·sin(Δψ)`. Commanding `w_u[2]` from a RATE error removes the ceiling.

⛔ Do NOT try to fix this by raising `K_R_yaw`: memory records `K_R_YAW↑ RULED OUT (worse)` —
the yaw rate loop is slow (~287 ms) so stiffening over-drives the lag (`controller.py` ~1148).
The fix is to bypass the nonlinearity, not to gain up against it.

⛔ Do NOT bake `PLASMC_YAW_WT_FF`. Two independent disqualifiers: (1) it is a measured no-op on the
failure it targets; (2) "perfect" `ω_t` only exists because the harness injected it — GT target yaw
rate in these recordings is **+0.000 rad/s**, the target never physically rotated. On a real target
`ω_t` is the unknown the approach was meant to avoid estimating, so there is nothing deployable to
feed forward. The knob stays default-0.0 as a GT-only diagnostic for re-running this class of test.
