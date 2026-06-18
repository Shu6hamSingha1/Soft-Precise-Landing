---
name: MC_*RATE_P runtime tuning via MAVSDK is dead in this SITL config
description: Setting PX4 rate-controller gains at runtime via vehicle.param.set_param_float breaks PX4 SITL startup. Worked once in impulse-test, fails systematically in landing_test. Use airframe init or uXRCE-DDS instead.
type: feedback
originSessionId: 3ffdaf6f-0da4-4c9f-b4df-fb9a91a7eb04
---

## Top finding (2026-05-23)

Tuning `MC_ROLLRATE_P/I/D` and `MC_PITCHRATE_P/I/D` at runtime via MAVSDK `vehicle.param.set_param_float()` during `FC.start()` is **incompatible with landing_test.py in this SITL configuration**. Setting non-default values destabilizes PX4's startup such that preflight checks never clear and `landing_test.py` bails with "Unable to get simulation time."

| Test | Scale 1.0 (default) | Scale 1.5 | Scale 2.0 |
|---|---|---|---|
| Impulse-response (one run) | 42.5 ms lag63 | failed | **32.6 ms lag63** (-23%) |
| Landings (N=3-5) | works | 0/2 succeeded | 0/3 succeeded |

The impulse test happened to succeed at scale 2.0 once (a one-off pass) — the landing tests systematically fail.

## Why it fails

Likely interaction:
- `_maybe_apply_rate_gain_scale` runs during `FC.start()` after `connect()` succeeds
- Param writes are sent before PX4 has finished its EKF/lockstep initialization
- Modified rate-controller gains then fail PX4's preflight checks
- Launcher's "Ready for takeoff" wait times out (30s)
- Launcher proceeds anyway; landing_test bails because /clock isn't healthy
- Retry wrapper detects exit code 0 (not 42) and reports success despite no data

The retry wrapper's lockstep-race detection (exit 42) doesn't catch this failure mode because the launcher exits 0 when no exception fired.

## Implementation hook is still in place

`flight_controller.py:_maybe_apply_rate_gain_scale` (with `asyncio.wait_for` timeouts) remains in the code, env-overridable via `PLASMC_PX4_RATE_SCALE` (default 1.0 = no-op). **Leave it at 1.0 in default config.** Reactivate only if a different approach to applying the gains works (e.g., setting them via airframe init at compile time, or via uXRCE-DDS).

## Realistic alternatives to runtime MAVSDK param-set

1. **Airframe init file edit** (compile-time): edit `~/PX4-Autopilot/Tools/simulation/gz/models/x500_mono_cam_down/4014_gz_x500_mono_cam_down` or similar to set `MC_*RATE_P` defaults that take effect before MAVSDK connects. Permanent for that airframe; reversible via the same edit.
2. **uXRCE-DDS migration** for rate setpoint: bypasses MAVSDK entirely for the control path. Doesn't change rate-controller gains, but cuts the 30 ms MAVSDK transport deadtime which was the bigger lever per impulse-response finding.
3. **PX4 `pxh>` shell** at boot: send `param set MC_ROLLRATE_P 0.3` etc. via the PX4 shell before flight. Manual but bypasses MAVSDK timing.

## How to apply

- When the user proposes `MC_*RATE_P` tuning at runtime: cite this finding. The hook is there but it breaks landings.
- The ~10 ms theoretical lag reduction from MC_*RATE_P scale 2.0 isn't recoverable through MAVSDK in this config. To pursue it, use one of the alternatives above.
- The bigger lever (30 ms MAVSDK transport) is still **uXRCE-DDS migration**, which is the next legitimate next step for lag reduction.

## Data

- `~/ws/Test_Data/InterventionsLagScale/20260523-181717/` — 3 failed N=10 runs at scale 2.0
- `~/ws/Test_Data/InterventionsLagScale15/20260523-192303/` — 2 failed N=5 runs at scale 1.5
- `~/ws/Test_Data/RateGainSweep/20260523-175616/` — impulse sweep that did show scale 2.0 reduces pitch lag, but only in the limited impulse-test context
