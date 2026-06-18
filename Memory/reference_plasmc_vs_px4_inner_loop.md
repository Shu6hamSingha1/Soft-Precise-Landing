---
name: PLASMC geometric inner loop vs PX4 attitude controller
description: Architectural comparison between the proposed geometric SO(3) inner loop and PX4's mc_att_control -> mc_rate_control cascade; manuscript reviewer-defense material
type: reference
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---
# PLASMC (Geometric SO(3)) vs PX4 Inner Loop

Reference for the manuscript: why PLASMC uses a direct geometric SO(3) inner loop instead of the PX4 attitude+rate cascade. Use this when a reviewer asks "why not just use PX4's inner loop?"

## Architecture summary

| | Proposed (Geometric SO(3), Lee 2010) | PX4 (Brescianini 2013) |
|---|---|---|
| Stages | Single-stage: attitude -> torque | Cascade: attitude -> rate sp -> rate PID -> torque |
| Representation | Rotation matrices | Quaternions |
| Output | tau (N.m) directly | omega_sp, then PID on rate error |
| Tilt/yaw handling | Coupled via single e_R vector | Decoupled via reduced-attitude + yaw_weight (~0.4) |
| Rate loop | None (kOmega*B_w_c damping only) | Full PID with integral + D-term filter + FF |
| Actuator mapping | Hard-clamp tau_xy, tau_z independently | Prioritized mixer (yaw sacrificed first on saturation) |

## Attitude error formulas

**Proposed:**
```
e_R = 0.5 * vee(R_d' * R - R' * R_d)
tau = -kR * e_R - kOmega * B_w_c + B_w_c x (J * B_w_c)
```

**PX4 (AttitudeControl.cpp, simplified):**
```cpp
Quatf qe = qd.inversed() * q;
Vector3f eq = 2.f * qe.canonical().imag();
eq(2) *= yaw_weight;                        // default 0.4 — the key trick
rates_sp = -kp .* eq + omega_ff;
```

The `yaw_weight` scalar **de-prioritizes yaw** when tilt is large, so aggressive yaw tracking doesn't fight the tilt loop through body-rate coupling. This is PX4's workaround for the Euler-coupling problem that PLASMC v0 hit on Circular wz=2.0. PLASMC solves the same problem by going geometric (no Euler representation anywhere).

## Three reviewer-defense points for the manuscript

1. **No yaw_weight hack needed.** The geometric controller treats all three axes uniformly through a single `e_R` vector. The yaw ASMC produces a smooth `psi_d` reference, so `kR(3,3)` doesn't need to be de-weighted relative to `kR(1,1)`/`kR(2,2)`. Cleaner than PX4's scalar fix, at the cost of relying on a well-behaved ASMC reference.

2. **Single-stage eliminates cascade latency.** Direct torque output removes a PID tuning layer. Trade-off: no integral action on rate error, so persistent rate biases (CG offset, rotor trim) aren't rejected locally — PLASMC absorbs them via the outer-loop adaptive kappa. This is a Lyapunov-consistent trade, not a capability loss.

3. **Stability guarantees flow end-to-end.** Mixing PLASMC's outer loop with PX4's PID cascade breaks the chain of proofs: PLASMC's outer loop assumes `I_a_cd` is realized through an attitude loop whose error dynamics are compatible with the Lyapunov function. PX4's heuristic yaw_weight and cascaded PIDs do not provide such guarantees. Geometric SO(3) does (Lee et al. 2010 — almost-global exponential stability).

## Why it matters for compass-denied visual landing

PX4's inner loop is tuned for general-purpose multicopters with compass-based yaw references. PLASMC operates in a compass-denied regime where the heading reference itself (`psi_d`) comes from integrating the yaw ASMC output, which is driven by the image-moment target orientation `alpha` (Eq. 19 in manuscript). This reference must flow through a Lyapunov-consistent inner loop to preserve the outer-loop guarantees — PX4's heuristic cascade would not.

## Trade-offs we accept

- **No rate integral action**: persistent rate biases rejected by outer-loop adaptation instead of local integral.
- **No prioritized mixer**: simple hard-clamp on torques; fine for simulation, would need attention for flight hardware.
- **No omega_ff from R_d_dot**: we currently set `Omega_d = 0`; if outer loop produces rapidly changing `I_a_cd`, we could add the full `R_c' * R_d * Omega_d` term. Not needed for any trajectory tested so far.
