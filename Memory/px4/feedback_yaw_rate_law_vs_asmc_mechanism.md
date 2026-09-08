---
name: feedback_yaw_rate_law_vs_asmc_mechanism
description: "WHY PLASMC_YAW_RATE_LAW beats the ASMC yaw loop on turning targets — the ASMC routes its rate command through the SO(3) e_R[2]=sin(dpsi), a bounded + NON-MONOTONIC nonlinearity that caps |w_u[2]| at K_R_yaw=0.5 rad/s and collapses past dpsi=90deg. The new law deletes the psi_d/e_R round-trip (psi_d:=yaw_c => e_R[2]~=0), commands the body yaw rate directly via its own integrator (clamp +-2.0, 4x headroom), and the -w_z term is a free implicit target-rate feedforward (w_z=omega_t,z-psi_dot_b). Measured on the 0.48 rad/s spin gate."
metadata:
  node_type: memory
  type: feedback
  originSessionId: 0131JP4rYAsEd95XXJhArVXy
---

**Measured 2026-09-08, turning-target A/B gate (GT-FB, target spinning 0.48 rad/s, IC1
`test_data/ICValidation/{20260908-232039 ASMC, 20260908-230706 new law}`).** `K_R_yaw=0.5`
(`controller.py:1204`, empirically `-w_u[2]/e_R[2]=0.50`).

| | ASMC | new law |
|---|---|---|
| `e_R[2]` | mean 0.38, SATURATED at ±1.0 in 17% of frames | ≈0 (max 0.089) |
| `w_u[2]` cmd | pinned ±0.500 | −0.53 (clamp ±2.0) |
| ASMC internal `u_a` | wants −1.5 to −3.3 rad/s | n/a |
| ACHIEVED body yaw rate (GT) | **+0.23** (half of the 0.48 needed) | **+0.64** (tracks) |
| `e_a` mid / end | 77° / **−157°** runaway | 6° / +3° |

## The old law's structural ceiling (not a bug)

ASMC yaw command path: `e_a → σ_a → u_a (SMC, ±3 rad/s) → psi_d integrator (±2) → R_d heading
basis → e_R[2] = ½·vee(R_dᵀR − RᵀR_d)_z = sin(psi_d − psi_body) → w_u[2] = −K_R_yaw·e_R[2]`.

`sin(Δψ)` is BOUNDED by 1 AND NON-MONOTONIC: yaw authority peaks at Δψ=90° and DECREASES past it.
On a target rotating faster than `K_R_yaw`=0.5 rad/s: loop falls behind → Δψ>90° → authority drops
→ falls further behind → Δψ→180° → sin→0 → zero authority → `e_a` slides to the ±180° alias.
Positive-feedback collapse. `u_a`/`kappa_a` wind up hard (−3.3) fighting a downstream wall they
can't see. **Can't fix by raising `K_R_yaw`** — yaw rate loop is slow (287 ms), stiffening
over-drives the lag (`K_R_YAW↑` is a recorded dead-end). The fix HAD to be bypassing `e_R`.

## What the new law adds

1. **No saturating nonlinearity between error and command.** `d(w_u[2])/dt = k_p·e_a − w_z`,
   own integrator, clamp ±2.0 (4× the old effective ceiling), applied DIRECTLY to the body-rate
   setpoint (`psi_d := yaw_c` in `_attCtrl` forces `e_R[2] ≈ 0`). `k_p·e_a` gives proportional
   command-rate at any error; the integrator builds until relative motion is zeroed.
2. **`−w_z` is a FREE implicit target-rate feedforward.** `w_z = ω_t,z − ψ̇_b` (flow-measured).
   As the target spins, `w_z` carries `ω_t,z`; `−w_z` drives `w_u[2]` to exactly the rate that
   cancels the RELATIVE yaw motion — no `ω_t` estimate needed. The ASMC has NO target-rate term
   in its core; `PLASMC_YAW_WT_FF` (the probe that bolted one on) was a measured no-op because it
   injected UPSTREAM of the `sin` ceiling ([[project_q8_yaw_ff_dead_sin_ceiling]]).

## How to apply

- On STATIONARY targets Δψ stays small, `sin` ceiling never binds, both laws work — the new law
  is NOT a clear stationary win (residual `e_a` ~15° from the ~3× lstsq-`w_z` magnitude deficit,
  unrelated to this mechanism).
- On a target rotating faster than `K_R_yaw`=0.5 rad/s the ASMC is structurally incapable; the
  new law is the fix. This is the deliverable of the PLASMC_YAW_RATE_LAW thread.
- Its turning-target VALUE is only realised once the separate turning-target LATERAL limit cycle
  is also fixed ([[project_rover_turning_open]], `PLASMC_AU_LEAD`) — perfect yaw alone still lands
  off-center reps ~6 m off via the rotating `cross(w_i,s)` disturbance.
