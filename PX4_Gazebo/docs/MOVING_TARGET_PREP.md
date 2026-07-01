# Moving-target (rover) phase — preparation & deferred work

Status: **Baseline stack RUNS end-to-end (2026-07-01/02).** Two-instance rover
world comes up reliably (retry wrapper + stale-server guard), pose is correct,
GT-FB controller engages. First baseline run diagnosed below.

Canonical handoff: `Memory/px4/project_moving_target_prep.md`. This file is the
actionable checklist + entry point; the memory note has the reasoning.

## FIRST BASELINE RUN (2026-07-02, GT-FB, STATIONARY rover, baked config)
Command: `HEADLESS=1 PLASMC_GT_FEEDBACK=1 ROVER_MOTION=0 MAX_ATTEMPTS=4 \
LANDING_AUTOSAVE=1 bash scripts/run_rover_landing_retry.sh`
Result (test_data/Landing_Test/<ts>): IC converged clean (pos_err 0.199 m).
Drone DESCENDED DEAD-CENTERED to **min alt 0.25 m** (xy still ~(0.5,-0.3)) — an
excellent approach — then a **violent terminal fly-away**: ballooned to 235 m
altitude and 900 m laterally (`descent stall ... alt=207.60 m` abort). Pose
correct throughout (smooth traj; target x=0.000 stationary confirmed).
→ This is the terminal-1/Z kick (memory campaign) but FAR more violent than the
stationary-aruco GT-FB (~1-4 m balloon). NOT a setup bug; the first control
finding to diagnose. Judge by breach%/s_dot_entry, not SP count.

### Lead (a) TESTED — camera/marker mount offsets FIXED, fly-away PERSISTS (2026-07-02)
gt_feedback computed (rover-base − uav-base), ignoring the +0.50 m marker mount
and +0.20 m camera mount → a ~0.30 m depth bias (near-deck TRUE camera-marker
depth ~0.10 m vs old ~0.40 m, a 4× 1/z error). Fixed it (commit: marker − camera
in V-frame, offsets rotated by body attitude). Re-run (GT-FB, stationary):
IC 0.165 m, but STILL a terminal fly-away (alt 243 m). So the mount-offset bias
alone is NOT the cause. ⚠ n=1 each, different IC settle (0.199/0.46° vs
0.165/3.40°) → the min-alt difference (0.25 vs 1.5 m) is within the ±5-7 SITL
noise, not attributable. The correction is still correct/more-faithful (keep it).

### DIAGNOSED via controlled n=3 A/B (2026-07-02) — ROVER-SPECIFIC, root = no platform
Identical GT-FB baked config, only the world differs: **aruco 0/3 fly-aways**
(all land dead-centered 0.08–0.20 m, min-alt at ground) vs **rover 2/3 fly-aways**
(peaks 270–299 m). So the fly-away is rover-specific, NOT a control-law regression.
Ruled out: target jitter (~0 in both), mount-offset (fixed, persists).
ROOT: the rover `arucotag` is a 1 m **visual-only plane at +0.5 m with NO
collision** (highest rover collision = body box ~0.1 m). The elevated marker
puts the terminal high-1/Z danger-zone at ~0.8 m base altitude (open air, no
contact below), so the LATENT terminal-1/Z kick develops and launches the drone;
the ground-level aruco marker coincides with contact that harmlessly arrests it.
**FIX = solid landing PLATFORM at the marker height. ✅ APPLIED + CONFIRMED.**
Added a `landing_platform` pedestal (0.6×0.6, 0.1→0.5 m) fixed to the rover
base_link, marker as a visual on its top face, in the `rover_aruco` model.sdf
(both `~/.gazebo/models/` and `~/PX4-Autopilot/Tools/simulation/gz/models/` —
OUTSIDE the repo). Re-run (GT-FB, stationary): drone lands ON the platform at
min-alt **0.51 m, dead-centered (0.05 m), bounded, no fly-away** (vs 2/3
fly-aways before). Full writeup: [[feedback_rover_flyaway_no_platform]].
Harness: scratchpad/ab_flyaway.sh + ab_analyze.py.

## What is staged
- `scripts/run_rover_landing.sh` — two-instance launcher (rover 4022 `-i 1` +
  UAV 4014 `-i 0`, `rover` world, `dynamic_pose/info` pose bridge). Forked from
  `run_aruco_landing.sh`, follows the canonical cleanup/`start_bg`/setsid/param-
  reset patterns. Exports the rover pose indices (below).

## RESOLVED 2026-07-01 (spawn + pose landmine)
Brought up the two-instance rover stack headless and fixed the blockers to a
clean spawn:
1. **Model not on PX4's gz path.** `rover_aruco` lived only in
   `~/.gazebo/models/rover_aruco/`; PX4 spawns via
   `~/PX4-Autopilot/Tools/simulation/gz/models/` and errored
   "Error finding file .../rover_aruco/model.sdf". **Fix:** installed a copy
   under the PX4 model dir (its includes `rover_ackermann` + `arucotag` already
   live there).
2. **SDF version too old.** `model.sdf`/`model.config` declared
   `<sdf version='1.0'>` → Gazebo Harmonic errored "Unable to convert from SDF
   version 1.0 to 1.11". **Fix:** bumped to `1.9` (matches `arucotag`; `merge`
   includes need ≥1.9) in BOTH `~/.gazebo/models/rover_aruco/` and the PX4 copy.
   Rover now spawns as `rover_aruco_1`.
3. **Pose-index landmine (was item 1).** Echoed `/world/rover/dynamic_pose/info`
   with both vehicles up: top-level model poses are in spawn order → **target =
   `poses[0]`, UAV = `poses[1]`** (vs stationary aruco UAV=`[2]`/target=`[1]`).
   **Fix:** `src/gz_subscriber.py` now reads `POSE_IDX_UAV`/`POSE_IDX_TARGET`
   from env (defaults 2/1 = aruco unchanged); `run_rover_landing.sh` exports
   `POSE_IDX_TARGET=0 POSE_IDX_UAV=1`.

> ⚠ Ordering assumes rover (`-i 1`) spawns before UAV (`-i 0`) — which the
> launcher guarantees. If launch order ever changes, re-verify with the echo.

## Infra confirmed present (2026-06-30)
- Airframe `4022_gz_rover_aruco` (Ackermann rover) + `4014_gz_x500_mono_cam_down`
  in `~/PX4-Autopilot/ROMFS/.../airframes/`.
- World `~/PX4-Autopilot/Tools/simulation/gz/worlds/rover.sdf` (empty: ground+sun;
  both vehicles spawned by PX4).
- Model `~/.gazebo/models/rover_aruco/` (`model.sdf` = rover_ackermann + arucotag
  marker 0.5 m above the rover).
- Manual launch reference: `tips.txt:55-82`.

## Remaining work items
1. ✅ **DONE — pose indices** (see RESOLVED above). `gz_subscriber.py` is now
   env-driven; `run_rover_landing.sh` sets the rover mapping.
2. **Yaw calibration** (`cal_s[3]`, currently `1.0`). Inert for the stationary
   square marker; ACTIVE once the target translates/turns. Moment-alpha κ_a yaw
   path is already tuned ([[feedback_moment_yaw_canonical]]); only the feature
   cal is pending ([[project_yaw_calibration_pending]]).
3. ✅ **DONE — rover motion source** (2026-07-01). Built + live-verified:
   - `src/rover_trajectory.py` — planar (x,y,yaw) port of `traj_Gen.m` (7 types;
     z-heave/roll/pitch ship-deck terms dropped as un-realizable by a ground
     rover). Velocity = d/dt position verified to 1e-10; constants copied verbatim.
   - `apps/rover_drive.py` — MAVSDK offboard POSITION setpoints on `udp://:14541`
     (instance-1 Onboard MAVLink port, confirmed from PX4 console). Env-config:
     `ROVER_TRAJ` / `ROVER_SPEED_MULT` / `ROVER_YAW_MODE` / `ROVER_RATE_HZ` /
     `ROVER_MAX_T`. Live test: connects, arms, offboard starts, rover MOVES
     (origin → path) — offboard accepted by the `rover_ackermann` controller.
   - Launcher hook: `ROVER_MOTION=1` in `run_rover_landing.sh` starts the driver.
   - ⚠ **Ackermann min-turn-radius ≈ 0.56 m** (wheelbase 0.321, 30° steer). The
     MATLAB `Circular` r=0.5 is BELOW this, so `rover_trajectory.py` bumps
     `Circular` to **r=0.8** (speed ≈0.38 m/s at speed_mult=1) to clear it.
     Start slow (`ROVER_SPEED_MULT` low) — rover speed stresses the terminal cycle.
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
