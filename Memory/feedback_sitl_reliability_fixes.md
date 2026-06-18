---
name: feedback_sitl_reliability_fixes
description: "Two SITL-reliability fixes that unblocked landing runs (2026-06-07): (1) CPU core-pinning + VSCode renice cures the gz_bridge lockstep/is_armable race (it's pure CPU starvation, not a code bug); (2) SLOW the IC yaw servo (IC_YAW_SERVO_DMAX_DEG=0.3) to cure the ±30° alpha limit-cycle that blocked IC convergence."
metadata: 
  node_type: memory
  type: feedback
  originSessionId: dd8920aa-4635-40a9-ac9d-8409e37243d6
---

Two host/rig-level fixes that took a landing run from "0 successful, fails at bringup or IC" to "controller engages and flies" (2026-06-07). Neither is a controller-gain change.

**1. gz_bridge lockstep / `is_armable` race = CPU STARVATION, not a code bug.**
Symptom: `is_armable did not go True within 60s — PX4 lockstep race is not recovering`, intermittent, correlated with host load. Mechanism (confirmed by reading GZBridge.cpp): the clock-sync (`clockCallback`) and `imuCallback` are independent gz-transport callbacks; under CPU contention the clock callback is starved, so IMU is published with an un-synced `CLOCK_MONOTONIC` timestamp → EKF sees time jump backward → never inits → not armable. **The durable code fix is a 2-line GZBridge `imuCallback` timestamp guard + sync MONOTONIC on the first clock msg** (we did NOT apply it — user wanted no PX4 edit). **The no-edit fix that WORKS: free CPU for SITL.** On this 16-core host: `renice -n 15 -p $(pgrep -x code)` (deprioritize VSCode, ~60% CPU) **+ launch the whole run under `taskset -c 6-15`** (px4+gz inherit the pinning). Result: `is_armable` 60s-timeout → **0.2-0.7s**. The DOMINANT load is actually the live `python3` control pipeline (649% CPU = 6.5 cores with fusion EKF + cbf2 + KF), NOT VSCode — the heavy stacked config is largely self-inflicted load; a 6.5-core control loop smells like a spin/inefficiency worth profiling.

**2. IC convergence yaw timeout = IC-yaw servo LIMIT CYCLE (sweeps alpha ±30°).**
Symptom: IC aborts at 60s with `yaw_err≈28-30°` (pos/vel/tilt all fine); same residual for `IC_YAW_TARGET=gt` AND `alpha`. Mechanism: the gate needs `yaw_err≤tol` for 20 consecutive samples, but `_servo_true_yaw` is effectively **bang-bang** — `clip(rad2deg(0.2·alpha), ±dmax)` saturates at `dmax=1°/iter = 50°/s` for any alpha>5°. That 50°/s setpoint exceeds the drone's ~45°/s yaw rate AND the **KF-lagged** alpha feedback (centroid KF became default 2026-06-06) → overshoot → **±30° limit cycle** (the code at landing_test.py:322 even documents the ±30° sweep). Speeding the servo UP (`IC_YAW_SERVO_K=0.5`) made it WORSE (85° divergence) — confirms overshoot. **FIX: SLOW the servo — `IC_YAW_SERVO_DMAX_DEG=0.3`** (15°/s, within drone+sensor bandwidth). Result: `yaw_err 30° → 0.12°`, IC converged at t=4.7s, controller engaged. Do NOT relax `LANDING_IC_YAW_TOL` instead — user requires board-square (alpha→0) at touchdown (`DES_ALPHA_AUTO=0`), so IC must genuinely converge alpha≈0. **Standard run prefix:** `renice 15 $(pgrep -x code)` BEFORE launch (more effective than taskset — `taskset -c 6-15` on the outer bash does NOT reliably pin the PX4 subprocess itself). Always use `run_aruco_landing_retry.sh` (handles IC-timeout + lockstep retries). See [[project_tuning_campaign_newcal_reset]].

**Coordination:** SITL is shared with another chat — only ONE chat runs SITL at a time (shared udp:8888 / px4 -i 0 / gz). Kill ONLY your own PIDs (never blanket `pkill px4/gz` — hits the other chat). The other chat's runs show as transient processes that churn (appear/vanish); genuine orphans keep the same PID with growing age.
