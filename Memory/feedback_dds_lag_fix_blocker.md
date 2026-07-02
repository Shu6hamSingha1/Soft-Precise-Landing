---
name: dds-lag-fix-blocker
description: "uXRCE-DDS rate-setpoint lag fix is built + impulse-validated but landing integration is blocked on an rclpy \"context invalid\" — pickup notes"
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 3ffdaf6f-0da4-4c9f-b4df-fb9a91a7eb04
---

**Goal:** cut the ~30 ms MAVSDK transport from the rate loop (impulse: 38 ms = 30 ms MAVSDK + 8 ms PX4, see [[feedback-impulse-response]]) by publishing body-rate setpoints over uXRCE-DDS instead of MAVSDK `offboard.set_attitude_rate`. The documented sanctioned lag fix ([[feedback-mc-rate-p-dead]]).

**Built (committed 82ddff6, env-gated `CMD_TRANSPORT`, default `mavsdk` = unchanged):**
- `src/dds_setpoint.py` `DDSRateSender`: publishes `OffboardControlMode(body_rate=True)` + `VehicleRatesSetpoint` to `/fmu/in/...` (both bridged in PX4 `dds_topics.yaml`). deg/s→rad/s, thrust→`thrust_body[2]=-throttle` — identical physical command to MAVSDK.
- `flight_controller.py`: `send_attitude_rate` branches on `self._cmd_transport`; DDS sender created eagerly in `FC.start()`. Cold path (arm/takeoff/offboard-entry/land) stays MAVSDK.
- `run_aruco_landing.sh`: sources `~/ros2_ws/install/setup.bash` (px4_msgs) under `set +u` (its setup.bash uses COLCON_TRACE unbound).
- px4_msgs is prebuilt at `~/ros2_ws/install` (ROS humble); imports in the env2025 venv after sourcing ros2_ws.

**VALIDATED:** the DDS path flies the drone — `impulse_response.py` with `CMD_TRANSPORT=dds` entered OFFBOARD, PX4 tracked the DDS rate setpoints, all 9 impulses executed. So the transport + offboard interplay (MAVSDK enters offboard, DDS stream takes over) works.

**BLOCKER (landing only):** first in-flight DDS publish in the landing control loop → `RuntimeError: publisher's context is invalid` (rcl publisher.c:389), then the run hangs (launcher kills at 180s → exit 42). It happens at the **warmup→control-loop transition** (after "PID warmup", on the first main-loop `send_attitude_rate`). Tried 3 rclpy integrations, all fail in landing:
- (a) shared default context + lazy create → **deadlock** on the context lock vs the 3 gz_subscriber SingleThreadedExecutors spinning mid-flight.
- (b) isolated private `rclpy.Context()` → works standalone AND in a standalone repro alongside a spinning default-context executor, but races to "context invalid" live.
- (c) default context + own executor thread (exact gz_subscriber pattern) → still "context invalid" at the transition.

**Why impulse works but landing doesn't:** impulse_response has NO gz_subscriber executors (no IMG_PROCESSOR); landing has 3 + the asyncio control loop. Something tears down the rclpy context between warmup and the first rate publish.

**Pickup leads:** (1) grep landing_test for an exception-triggered `finally: rclpy.shutdown()` firing early (the error is on "Main Thread"; a swallowed exception in warmup/startController could run cleanup while a coroutine still publishes). (2) publish() thread-safety: send_attitude_rate is awaited on the asyncio main thread while the DDS node's executor spins in a daemon thread. (3) Add live logging of `rclpy.ok()` / context id around warmup and the first publish. Debug with instrumentation, NOT 4-min batch SITL runs (slow). The lazy-create deadlock (a) confirms node creation contends the shared context lock while executors spin — so the eventual fix likely needs the DDS node fully set up and the publish made thread-safe before the gz executors are hot.

**Default mavsdk => the working 0.675 m board+inner-cluster landing is unaffected** ([[project-landing-target-design]]).

**⛔ CLOSED 2026-07-02 (user): "We will not go ahead with the uXRCE-DDS low-latency rate path. Forget it." Do NOT re-propose the DDS path — for the lag/actuation-phase limits the accepted answer is operational spec (see [[project_rover_turning_open]]).**
