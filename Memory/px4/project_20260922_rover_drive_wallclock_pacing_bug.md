---
name: project_20260922_rover_drive_wallclock_pacing_bug
description: "⭐ FIXED + LIVE-VERIFIED 2026-09-22. apps/rover_drive.py paces its traj_Gen reference time `t` via asyncio.sleep(dt) on WALL-CLOCK, t+=dt unconditionally every loop -- completely decoupled from Gazebo's simulated /clock, which is what Ground_Truth.npy's 'Time' (and every GT-based rover-speed measurement this project has ever made) actually uses. If Gazebo's real-time factor != 1.0 (plausible headless/multi-SITL/shared-machine), the setpoint stream races ahead of true sim time, so MEASURED GT rover speed != the nominal analytic v(t) from ROVER_TRAJ/ROVER_SPEED_MULT. Matches the observed pattern: Sinusoidal measured 0.74 m/s vs 0.58 commanded-equivalent, Lissajous 1.80 vs 0.51, Circular r=0.8 0.58 vs 0.384. Means every rover profile-speed number logged to date (this session and prior) is not a controlled value -- it's whatever RTF happened to be at run time, not just ROVER_SPEED_MULT. Fix applied: rover_drive.py now reads t off Gazebo's /clock (gz_subscriber.Clock_Node, same source as Ground_Truth.npy's Time) instead of accumulating asyncio.sleep() wall-clock ticks. Verified: Sinusoidal measured GT median speed went from 0.74 m/s (pre-fix, 28% over commanded) to 0.579 m/s (post-fix) vs the analytic commanded 0.576 m/s -- matches to noise."
metadata:
  type: project
---

**Trigger:** while comparing MATLAB `traj_Gen.m` profile speeds against PX4 `rover_trajectory.py`'s (2026-09-22), found that Sinusoidal and Lissajous use LITERALLY IDENTICAL constants to MATLAB (A, w0, v0 / A, B, w1, w2), yet this session's GT-measured target speed came out 28-250% higher than what those formulas predict (Sinusoidal 0.74 vs 0.58 m/s median; Lissajous 1.80 vs 0.51 m/s median, measured max 3.03 vs commanded max 0.71). Circular (r=0.8, `ROVER_CIRCLE_VTAN` default 0.384 constant by construction) also measured 0.58 m/s, not 0.384.

## Root cause (confirmed at the code level)

`apps/rover_drive.py`'s main loop (~line 149-161):
```python
t = 0.0
while MAX_T is None or t <= MAX_T:
    s = eval_traj(t, TRAJ, SPEED_MULT, YAW_MODE, prev_yaw=prev_yaw)
    await rover.offboard.set_position_ned(PositionNedYaw(s.x, s.y, 0.0, math.degrees(s.yaw)))
    ...
    await asyncio.sleep(dt)
    t += dt
```
`dt = 1/RATE_HZ` (default 20 Hz) and `t` is advanced by a FIXED WALL-CLOCK increment every loop, paced by `asyncio.sleep` -- a real-world timer, with zero coupling to Gazebo's own simulated clock.

Meanwhile `Ground_Truth.npy`'s `"Time"` field (used for every GT-based speed/yaw-rate measurement in this project, including this session's whole trajectory-profile sweep and the Circular r-sweep before it) comes from `gz_subscriber.py`'s `Clock_Node`, subscribed to `/clock` -- genuine Gazebo SIMULATION time (`src/gz_subscriber.py:316-337`).

If Gazebo's real-time factor (RTF) is not exactly 1.0 -- entirely plausible headless, under load, or (as in this session) with multiple concurrent SITL stacks across peer sessions sharing the machine -- the two clocks diverge: `rover_drive`'s reference position races ahead of (RTF<1, sim slower than real-time) or lags behind (RTF>1) where the SIMULATED world actually is. The rover chases a reference that has already advanced further (or less far) in sim-time-equivalent distance than the nominal `v(t)`/`ROVER_SPEED_MULT` design value predicts, so the GT-measured speed differs from the commanded analytic speed by a factor tied to 1/RTF (not a fixed error).

**Not yet live-confirmed** (no per-run RTF logging exists, and `run_logs/rover_drive.log` is overwritten every run rather than archived per rep, so no historical rover_drive stdout survives to directly check its printed `t` against wall-clock timestamps for a specific past rep). The code-level mechanism is unambiguous; the exact scale factor per run is not directly measured, only inferred from the measured/commanded speed ratio (1.3x-3.5x across profiles this session, consistent in DIRECTION -- always measured > commanded -- but not a single constant, itself consistent with a load-dependent RTF that varied run to run rather than a fixed bug).

## Implications

- Every ROVER-PROFILE SPEED NUMBER this project has ever logged via `rover_drive.py`'s default pacing (this session's r=0.8/1.6/3.2/6.4 Circular sweep, the 5-profile trajectory sweep, and any prior rover work using this launcher) is **not a controlled experimental variable** -- the DELIVERED speed depends on machine load / concurrent-SITL contention at run time, not purely on `ROVER_TRAJ`/`ROVER_SPEED_MULT`/`ROVER_CIRCLE_R` as the harness scripts assume.
- The qualitative conclusion in [[project_20260922_rover_moving_sp_investigation]] ("speed dominates over curve shape") likely still holds directionally, since it's based on the actually-measured GT speed -- which IS the real physical speed the drone had to contend with, regardless of what caused it. But any claim tying a SPECIFIC nominal speed value to a SPECIFIC outcome (e.g. "the ceiling is at 0.5-0.6 m/s") is now suspect -- the true delivered speed on any given rep is unknown without checking its own GT, not assumable from the env vars used to launch it.
- Re-running any of this session's rover speed-dependent conclusions on a QUIET machine (no concurrent peer SITL), or fixing `rover_drive.py` to pace off sim time, would be needed before trusting a specific speed threshold number.

## FIX APPLIED + VERIFIED (2026-09-22, same session)

`apps/rover_drive.py`: added `rclpy.init()` + `gz_subscriber.Clock_Node`/`GZ_Subscriber`
(the same pattern `landing_test.py` already uses for its own sim-time source), read
`time_node.perf_counter()` each loop iteration and set `t = now_sim - t0_sim` (t0_sim
captured at the gate-open moment, or run start if ungated) instead of accumulating
`t += dt` on every `asyncio.sleep(dt)` tick. `asyncio.sleep(dt)` still paces the setpoint
SEND rate (~20 Hz) -- only the VALUE of `t` fed into `eval_traj` changed.

**Live-verified**, single `WORLD=rover_cross ROVER_TRAJ=Sinusoidal` rep, GT-FB:
- Log confirms clock acquisition (`[rover_drive] sim clock acquired.`) and normal
  progression (`t=10.0 p=(+0.49,+5.01) v=0.50 yaw=+97`).
- Landing completed normally (`PRECISE-only`, xy_err=0.016 m) before the harness's own
  teardown killed the rover_drive process — a harmless shutdown-race traceback afterward
  (grpc `Connection refused` while `set_position_ned` raced the launcher's SIGTERM), not
  a defect in the clock logic; the rep had already saved.
- **Measured GT median speed 0.579 m/s vs the analytic commanded 0.576 m/s** (was 0.74 m/s
  pre-fix, 28% over) — matches to noise. Confirms the mechanism and the fix.

**Not yet done:** a full re-run of [[project_20260922_rover_moving_sp_investigation]]'s
Circular r-sweep and 5-profile trajectory sweep under the fixed pacing, to get controlled
speed numbers for those SP/precision conclusions (the qualitative "speed dominates"
finding is expected to hold; the specific speed thresholds quoted there should be
treated as provisional until re-measured post-fix).

## Suggested fix (not applied)

Drive `t` from Gazebo's own `/clock` (subscribe directly in `rover_drive.py`, or accept a sim-time tick piped from `gz_subscriber.py`) instead of `asyncio.sleep`-paced wall-clock accumulation, so the streamed setpoint's implied speed matches the analytic `v(t)` regardless of RTF.

**How to apply:** before citing an exact rover target speed number from ANY existing recording, check that specific rep's own GT-derived speed (savgol-filtered `Target Pose` vs GT `Time`) rather than trusting `ROVER_SPEED_MULT`/the formula. When re-running rover speed-sensitive tests, note concurrent SITL load (other peer sessions) as a confound, same class of trap as [[feedback_recurring_analysis_mistakes]]'s sim-time-vs-wall-clock warning.
