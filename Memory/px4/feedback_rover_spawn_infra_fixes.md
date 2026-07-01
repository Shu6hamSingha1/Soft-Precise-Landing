---
name: feedback_rover_spawn_infra_fixes
description: "Three infra blockers that stop the rover_aruco target from spawning in the rover world, and their fixes (model-path install, SDF version bump, dynamic_pose index ordering). Hit + fixed 2026-07-01 when starting the moving-target phase."
metadata:
  node_type: memory
  type: feedback
---

Bringing up the two-instance rover stack (rover 4022 `-i 1` + UAV 4014 `-i 0`,
`rover` world) for the first time, the `rover_aruco` target silently FAILED to
spawn (only `ground_plane` + later `x500_mono_cam_down_0` in `gz model --list`).
Three sequential blockers, all fixed:

1. **Model not on PX4's gz path.** `rover_aruco` lived ONLY in
   `~/.gazebo/models/rover_aruco/`; PX4 spawns from
   `~/PX4-Autopilot/Tools/simulation/gz/models/` → boot log
   "Error finding file .../rover_aruco/model.sdf". FIX = installed a copy under
   the PX4 model dir (its `merge` includes `rover_ackermann` + `arucotag` already
   live there, so they resolve).
2. **SDF too old.** `model.sdf`/`model.config` declared `<sdf version='1.0'>` →
   Gazebo Harmonic "Error Code 39: Unable to convert from SDF version 1.0 to
   1.11". FIX = bumped to `1.9` (matches `arucotag`; `merge='true'` includes need
   ≥1.9) in BOTH `~/.gazebo/models/rover_aruco/` AND the PX4 copy — gz reads the
   `~/.gazebo` one first, so fixing only the PX4 copy left the error. Then spawns
   as `rover_aruco_1`.
3. **Pose-index landmine.** Rover world bridges `/world/rover/dynamic_pose/info`
   (not `/pose/info`). With both vehicles up the PoseArray lists top-level models
   in SPAWN order → **target=`poses[0]`, UAV=`poses[1]`** (confirmed by position:
   rover at MODEL_POSE 0,0, UAV at 1,0). Stationary aruco was UAV=`[2]`/target=`[1]`.
   FIX = `gz_subscriber.py` now reads `POSE_IDX_UAV`/`POSE_IDX_TARGET` (defaults
   2/1 = aruco unchanged); `run_rover_landing.sh` exports `TARGET=0 UAV=1`.
   Ordering assumes rover (`-i 1`) spawns before UAV (`-i 0`) — the launcher
   guarantees this; re-verify with the echo if launch order changes.

⚠ **Headless PX4 log blowup.** Running PX4 SITL headless (no TTY) spams the
`pxh>` prompt + `\x1b[2K` clear-line escapes to stdout → the log grows ~GB/min
(hit 3 GB). For short inspections redirect PX4 stdout to `/dev/null` and query
`gz topic`/`gz model --list` directly instead of grepping the log. The real
launchers are fine because they kill PX4 after the ~70 s landing.

## Rover MOTION SOURCE (built + verified same session)
`traj_Gen.m` has no rover equivalent, so ported it: `src/rover_trajectory.py`
= planar (x,y,yaw) port of the 7 MATLAB types (z-heave + roll/pitch ship-deck
terms DROPPED — a ground rover can't do them); velocity = d/dt position verified
to 1e-10. `apps/rover_drive.py` = MAVSDK offboard POSITION setpoints on
`udp://:14541` (instance-1 Onboard MAVLink port, from the PX4 console
"mode: Onboard ... remote port 14541"). Live-verified: connects, arms, offboard
starts, rover MOVES from origin along the path → `rover_ackermann` pos controller
accepts offboard. Launcher hook `ROVER_MOTION=1`; env `ROVER_TRAJ` /
`ROVER_SPEED_MULT` / `ROVER_YAW_MODE`.

⚠ **Ackermann min turn radius ≈ 0.56 m** (RA_WHEEL_BASE 0.321, RA_MAX_STR_ANG
30°). The MATLAB `Circular` r=0.5 is BELOW it → `rover_trajectory.py` bumps
`Circular` to **r=0.8** (user-directed; speed ≈0.38 m/s at speed_mult=1).
`Linear`/`Sinusoidal` also track fine. RO_SPEED_LIM=3 m/s caps top speed.

## Baseline run bring-up (2026-07-02) — 3 more fixes + first datapoint
Running the full baseline surfaced:
- **WRONG POSE TOPIC (the real 375 m landmine).** The launcher bridged
  `dynamic_pose/info`, which only carries entities that MOVED that step → its
  PoseArray membership/order VARIES frame-to-frame → fixed indices point at
  random links during flight (IC pos_err 375 m; sometimes a FROZEN 382 m). FIX:
  bridge the FULL `/world/rover/pose/info` (stable order `[ground_plane,
  rover_aruco_1, x500...]` → **target=1, UAV=2, SAME as stationary aruco**;
  ~54 Hz, tracks live motion — verified by driving the rover). pos_err 375→0.199.
  So the earlier "dynamic_pose target=0/UAV=1" fix was the WRONG topic; env
  indices reverted to 2/1 (the gz_subscriber defaults). tips.txt step 6 updated.
- **Headless px4 log blowup** → `px4 -d` (daemon, no pxh shell) in HEADLESS →
  logs 8 KB vs GB/min.
- **Stale gz server poisons the next run.** The standalone gz server (spawned by
  the -i 1 rover px4) repeatedly OUTLIVES the launcher's single `pkill gz sim` →
  the next run/retry attaches to the old world → frozen/garbage pose. FIX:
  pre-launch stale-server GUARD + a verify-and-rekill loop in cleanup; plus a
  `run_rover_landing_retry.sh` (mirrors the aruco retry; PX4 SITL startup is
  ~50% flaky: lockstep is_armable race + IC non-settle both exit 42 → reboot).

**FIRST BASELINE DATAPOINT (GT-FB, STATIONARY rover, baked config):** IC clean
(0.199 m) → descends DEAD-CENTERED to **min alt 0.25 m** → then VIOLENT terminal
fly-away (balloons to 235 m alt / 900 m lateral). Pose correct throughout (target
x=0.000). = the terminal-1/Z kick, but FAR worse than stationary-aruco GT-FB
(~1-4 m). NOT a setup bug — first control finding. Lead: gt_feedback target =
rover_aruco_1 BASE (z≈0.01) but the MARKER is 0.5 m up → possible loom/z bias.

See [[project_moving_target_prep]] + docs/MOVING_TARGET_PREP.md. Next: diagnose
the terminal fly-away; yaw cal (`cal_s[3]`).
