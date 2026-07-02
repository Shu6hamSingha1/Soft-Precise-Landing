---
name: project_moving_rover_landing_works
description: "FIRST MOVING-TARGET LANDINGS WORK (2026-07-02): GT-FB, Linear rover @ 0.47 m/s, n=3 = 3/3 ON the moving platform (rel lateral 0.04-0.28 m, rel speed 0.08-0.23 m/s, 0 fly). Infra: rover motion gated to descent-start (rover holds until CHASE_GATE_FILE), rover_drive on a DEDICATED mavsdk gRPC port 50052 (sharing 50051 with landing_test's FC caused 180 s hangs). Next: speed sweep, Circular (turning) validation, perception-ON."
metadata:
  node_type: memory
  type: project
---

**FIRST MOVING-TARGET LANDINGS (2026-07-02) — the rover phase's core capability works.**
GT-FB, `ROVER_TRAJ=Linear ROVER_SPEED_MULT=0.3` (0.47 m/s), `PLASMC_YAW_ALPHA_FILT=0`
(per [[feedback_rover_yaw_cal_resolved]] — no corruption to reject in GT-FB):

| rep | rover moved | rel lat @min-alt | rel speed | verdict |
|-----|-----------|------------------|-----------|---------|
| 1 | 5.1 m | 0.284 m | 0.08 m/s | ON platform |
| 2 | 5.2 m | 0.044 m | 0.23 m/s | ON platform |
| 3 | 5.6 m | 0.048 m | 0.23 m/s | ON platform |

All min-alt 0.51–0.53 m (platform top), 0 fly-aways. Tracking shape: the drone lags the
rover's instantaneous 0.47 m/s velocity STEP at motion start (rel_lat transient to ~0.74 m
by ~3 s), then closes while descending and touches down centered + velocity-matched — the
IBVS structural strength for moving targets (drive `h_e→0` ⇒ track), exactly as the
[[project_moving_target_prep]] handoff predicted.

**Infra that made it work (commit 111fb4c):**
1. **Rover motion GATED to descent-start.** `rover_drive.py` `ROVER_GATE_FILE`: hold the
   start position (offboard alive, streaming setpoints) until the controller touches the
   descent-start flag (`CHASE_GATE_FILE`, the same gate as the chase recorder), then run
   the trajectory with t=0 at the gate. So the drone's ~60 s arm/takeoff/IC happens over a
   STATIONARY rover — the IC rig never chases a moving target. Launcher sets the gate
   whenever `ROVER_MOTION=1` or `CHASE_CAM=1`.
2. **⚠ MAVSDK gRPC PORT CONFLICT (the 180 s hangs).** `mavsdk.System()` spawns an embedded
   `mavsdk_server` on DEFAULT gRPC port 50051. `rover_drive` (starts first) grabbed it →
   `landing_test`'s FC connected to the ROVER's server (bound to udp 14541) → hung waiting
   for the UAV on 14540. FIX: `rover_drive` uses port 50052 (`ROVER_MAVSDK_PORT`). Any
   future second MAVSDK client must use a distinct port.
3. **New retriable flake: `Unable to get simulation time`** (/clock bridge race) —
   landing_test dies at startup but EXITS 0, fooling the retry wrapper into "SUCCESS".
   Detection added to `run_rover_landing.sh` → exit 42 → stack reboot.

**NEXT:** rover speed sweep (`SPEED_MULT` 0.3→0.5→1.0 — the entry-velocity/terminal-cycle
stress predicted to bind at speed); `Circular` (r=0.8) = the TURNING-rover yaw validation
(alpha cap: keep FILT=0 in GT-FB); perception-ON on the moving rover (flow dynamic range
vs the 0.47–1.5 m/s target speed); montage of a moving landing (CHASE_CAM=1 composes with
the same gate). Eval metric stays RELATIVE (lat, rel-speed at min-alt); platform half-width
0.3 m is the ON-platform threshold.
