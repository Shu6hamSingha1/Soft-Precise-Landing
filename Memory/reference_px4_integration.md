---
name: PLASMC PX4 integration guidance
description: How to deploy PLASMC on PX4 — use rate setpoints (not torque); architecture split between sim and deployment variants
type: reference
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---
# PLASMC -> PX4 Integration

Guidance for deploying the proposed geometric-SO(3) PLASMC controller on a real PX4 autopilot. **Rule: send rate setpoints, not torque commands.**

## PX4 entry points for external controllers

| Level | uORB / MAVLink | PX4 still runs | You bypass |
|---|---|---|---|
| Attitude | `vehicle_attitude_setpoint` / `SET_ATTITUDE_TARGET` (q + thrust) | Attitude + Rate + Allocation | Nothing above attitude |
| **Rate (recommended)** | `vehicle_rates_setpoint` / `SET_ATTITUDE_TARGET` (rate mask) | Rate PID + Allocation | Attitude controller |
| Torque (avoid) | `vehicle_torque_setpoint` + `vehicle_thrust_setpoint` | Control allocation only | Attitude + Rate |

## Why rate, not torque

Direct torque loses critical PX4 infrastructure:

1. **Rate integral action** — rejects CG offset, motor trim, battery-dependent motor response. Outer PLASMC adaptation is too slow for this.
2. **D-term low-pass filter** — suppresses gyro noise, motor harmonics, arm-flex vibrations. Geometric kOmega amplifies noise straight to actuators.
3. **Anti-windup + prioritized allocation** — PX4 sacrifices yaw first on saturation. Hard-clamping tau_xy/tau_z independently loses authority unpredictably.
4. **Normalized torque mess** — `VehicleTorqueSetpoint` uses `[-1,1]`, not N.m. Mapping depends on motor KV, battery voltage, prop inertia; fragile per-airframe calibration.
5. **Failsafe** — land/RTL/offboard-loss paths assume rate-level or above control.

Rate commands keep all of this and are the well-tested path used by ETH, MIT FlightGoggles, and most offboard research flights.

## Architecture split

**Sim variant (manuscript)** — direct torque output:
```
e_R = 0.5 * vee(R_d' * R - R' * R_d)
tau = -kR * e_R - kOmega * B_w_c + w x Jw
```

**Deployment variant (PX4)** — rate setpoint output:
```
e_R  = 0.5 * vee(R_d' * R - R' * R_d)
w_sp = -kR_att * e_R                    # attitude P-gain only
```
The `kOmega` damping and `w x Jw` gyroscopic FF both move into PX4's rate PID. `kR_att` now has units of rad/s per unit attitude error.

**95% of code is shared**: outer PLASMC, yaw ASMC, psi_d integration, R_d construction. Only the last 3 lines differ.

## Thrust mapping

- Sim: `T_cd` in Newtons
- PX4: `VehicleThrustSetpoint` as normalized `[0, 1]`
- Conversion: calibrate via `MPC_THR_HOVER` (PX4 already exposes this). Normalized = `T_cd / (m * g / hover_ratio)`.

## Deployment path

1. **Simulation** — keep direct-torque variant for the paper (clean theory, matches comparison controllers).
2. **SITL bridge** — add deployment mode that outputs `w_sp`. Test against PX4 SITL with the X500 model (same Gazebo airframe already simulated).
3. **Hardware** — use uXRCE-DDS (PX4 v1.14+) or MAVROS to publish `VehicleRatesSetpoint` + `VehicleThrustSetpoint` from a companion computer. **Do not modify flight controller firmware.**

## Relevant PX4 PARAMs to calibrate per airframe

- `MC_ROLLRATE_P/I/D`, `MC_PITCHRATE_P/I/D`, `MC_YAWRATE_P/I/D` — rate PID
- `MC_ROLLRATE_K`, etc. — rate loop gain scaling
- `MPC_THR_HOVER` — hover thrust ratio (for thrust normalization)
- `MC_AIRMODE` — keep off for landing (airmode boosts yaw authority at low thrust, can interact oddly with the ASMC)

## Open questions for deployment

- Yaw ASMC integration timestep vs. PX4 offboard rate (nominal 250 Hz for rates): `u_a * dt` integration must happen at the companion-computer rate, not PX4 rate.
- `psi_d` wrap handling at the MAVLink/uORB boundary — rate setpoint is a rate, not angle, so wrapping is only internal to the companion controller.
- Vision latency: PX4 offboard expects rate commands at >= 50 Hz; if visual feature processing is slower, need a rate holdover/extrapolation strategy.
