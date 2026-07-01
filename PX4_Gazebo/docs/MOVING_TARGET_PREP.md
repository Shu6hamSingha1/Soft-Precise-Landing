# Moving-target (rover) phase — preparation & deferred work

Status: **PREP staged 2026-06-30, code changes DEFERRED until the stationary
ArUco test is finished** (per user: no pipeline-code changes until then).

Canonical handoff: `Memory/px4/project_moving_target_prep.md`. This file is the
actionable checklist + entry point; the memory note has the reasoning.

## What is already staged (no pipeline code touched)
- `scripts/run_rover_landing.sh` — two-instance launcher (rover 4022 `-i 1` +
  UAV 4014 `-i 0`, `rover` world, `dynamic_pose/info` pose bridge). Forked from
  `run_aruco_landing.sh`, follows the canonical cleanup/`start_bg`/setsid/param-
  reset patterns. **Will not produce a correct landing until item 1 below is done.**

## Infra confirmed present (2026-06-30)
- Airframe `4022_gz_rover_aruco` (Ackermann rover) + `4014_gz_x500_mono_cam_down`
  in `~/PX4-Autopilot/ROMFS/.../airframes/`.
- World `~/PX4-Autopilot/Tools/simulation/gz/worlds/rover.sdf` (empty: ground+sun;
  both vehicles spawned by PX4).
- Model `~/.gazebo/models/rover_aruco/` (`model.sdf` = rover_ackermann + arucotag
  marker 0.5 m above the rover).
- Manual launch reference: `tips.txt:55-82`.

## Deferred code changes (apply AFTER the stationary test)
1. **`src/gz_subscriber.py:84-85` pose indices (DO FIRST — load-bearing).**
   Hard-codes `UAV=poses[2]`, `target=poses[1]` for the stationary `aruco`
   PoseArray. The rover world bridges `/world/rover/dynamic_pose/info`, which has
   different membership/ordering → these indices will silently corrupt BOTH
   perception-NED and GT-feedback (`controller.py:973-975`). First inspect the
   live ordering:
   ```
   gz topic -t /world/rover/dynamic_pose/info -e
   ```
   then set the correct indices (prefer world-conditioned selection over a second
   hard-code, so the same code serves both worlds).
2. **Yaw calibration** (`cal_s[3]`, currently `1.0`). Inert for the stationary
   square marker; ACTIVE once the target translates/turns. Moment-alpha κ_a yaw
   path is already tuned ([[feedback_moment_yaw_canonical]]); only the feature
   cal is pending ([[project_yaw_calibration_pending]]).
3. **Rover motion source.** No trajectory plugin / speed knob exists. Drive the
   rover externally on `-i 1` (QGC mission / MAVLink offboard / manual) and wire
   it into `run_rover_landing.sh` via the `ROVER_DRIVE` env hook (placeholder,
   default OFF). Pick a speed range for the IC/landing test.
4. **Moving-target IC/landing test harness** — analogue of `run_ic_validation.sh`
   with the rover moving. `SoftPrecise` eval already uses RELATIVE xy/rel_vel, so
   no eval change needed.
5. **Yaw caveats for a TURNING rover (only if the rover turns):**
   - `gt_feedback.py:153` `w_z = -alpha_dot` assumes a stationary-yaw target.
   - alpha filters `controller.py:343-364` (YAW_ALPHA_FILT) have a 0.30 rad/s
     max-rate cap that could reject a fast-turning rover as corruption.
   - `CTRL_ZERO_WXY=1` is fine for a TRANSLATING rover; revisit if it rotates.
   Straight-driving rover = all of these are fine as-is.

## Controller readiness (no change needed)
- `gt_feedback.py` already computes RELATIVE pose/vel from both poses
  (`W_x_tu = tpp - up` L100, `W_v_tu` L146) → flow `h` naturally includes target
  motion; driving `h_e→0` makes the UAV track the rover. Moving-target-ready.
- Carried-over baked stationary config (GT-FB, 6/9 SP base): `W_U_MAX=2.0`,
  `VDS_KF_Q=1`, `Z_REG=0.2`. Run this as the rover baseline first.
- `PLASMC_HD_FUNNEL_REF` (`controller.py:884-890`) is the labeled moving-target
  candidate knob (default-OFF); revisit for tracking.

## Expected control challenge
Lateral tracking adds a baseline lateral velocity = rover speed, which STACKS on
the entry-velocity → terminal-cycle sensitivity from the stationary session. The
faster the rover, the harder the terminal cycle / κ-deliverability binds. Start
slow. Keep GT-FB for control-isolation first, then re-introduce perception
(mind the speed-dependent flow under-report 0.79×→0.48× = LK dynamic range, not
recalibratable).
