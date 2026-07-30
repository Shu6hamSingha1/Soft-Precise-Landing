#!/usr/bin/env python3
"""
Real-hardware PLASMC landing test, driving the same `Controller` class
(controller.py) used by the Gazebo SITL entry point
(PX4_Gazebo/apps/landing_test.py). Modeled on landing_test.py's control
loop, with the Gazebo-only pieces removed:
  - no rclpy / gz_subscriber / Pose_Node ground truth (hardware has none)
  - no fly-to-ENU-IC step (no ground-truth position to fly to)
  - no MATLAB soft-precise post-hoc classification (needs GT target pose)
Everything safety-relevant from landing_test.py IS kept: warmup before
engaging the controller, marker-loss grace + open-loop fallback descent,
TOUCHDOWN_DETECTED handling, a hover/descent-stall watchdog (using onboard
FC_node.getPosBody() instead of Gazebo truth), and clean shutdown/disarm.

*** THRUST/RATE CALIBRATION: CALIBRATED, see Test_Data/Calibration/Input_Clean/CALIBRATION_RESULT.txt ***
HOVER_THROTTLE_NORM / THRUST_SLOPE_N_PER_UNIT / RATE_CORRECTION defaults below
are the FINAL adaptive-trim, drag-corrected, filtered, r^2-weighted result
from Hardware/scripts/analyze_input_calibration.py (dataset: 47 runs,
Input_Clean/, last updated 2026-07-22 — unchanged since, no new input-cal
recordings). landing_test.py's 0.738 / 42.3 values are SITL X500-specific
(different mass/ESC/prop) and are NOT used here. If new input-cal data is
recorded, rerun analyze_input_calibration.py with best_trim_metrics() (the
adaptive per-run trim search — main() alone does full-run-only and will NOT
reproduce this file's numbers) and update CALIBRATION_RESULT.txt + the
defaults below together; don't hand-derive a "candidate" value against a
different (non-adaptive-trim) run of the script or a stale memory note.
"""

import os
import sys
import asyncio
import time
import numpy as np
from datetime import datetime

sys.path.insert(0, ".")

try:
    from ahrs import RAD2DEG
    from flight_controller import FC
    from controller import Controller
    print("All modules imported successfully")
except ImportError as e:
    print(f"Import failed: {e}")
    sys.exit(1)

# ─── Flight parameters (env-overridable, same convention as landing_test.py) ───
REF_RAD_OPT_FLOW = float(os.environ.get("LANDING_REF_RAD_OPT_FLOW", "-0.30"))
DES_IMG_FEATURE_PARAM = np.array([0.0, 0.0, 1.0,
                                   np.deg2rad(float(os.environ.get("DES_ALPHA_DEG", "0.0")))])
TAKEOFF_HEIGHT = float(os.environ.get("LANDING_TAKEOFF_HEIGHT_M", "3.0"))
SLEEP_TIME = 1 / 200

# Final calibrated values (adaptive-trim, drag-corrected, filtered,
# r^2-weighted; see CALIBRATION_RESULT.txt "2026-07-22 -- Adaptive per-run
# trim" section). Implied mass -1.2% off known 1.204 kg.
HOVER_THROTTLE_NORM = float(os.environ.get("HW_HOVER_THROTTLE_NORM", "0.42"))
THRUST_SLOPE_N_PER_UNIT = float(os.environ.get("HW_THRUST_SLOPE", "31.98"))

# *** Rate-axis command correction, r^2-weighted input-cal cross-check ***
# gain = achieved/commanded from input-cal regression; dividing the intended
# command by gain (== multiplying by these factors) should make the ACHIEVED
# rate match what was originally intended. Adaptive-trim final values
# (n_eff: wx 13.88, wy 15.59, wz 17.56) -- wy remains the least-supported of
# the three but the gap narrowed a lot vs the earlier full-run-only pass.
# See Hardware/Test_Data/Calibration/Input_Clean/CALIBRATION_RESULT.txt for
# the full derivation. Set RATE_CORRECTION_ENABLED=0 to disable and fall
# back to uncorrected commands.
RATE_CORRECTION_ENABLED = os.environ.get("RATE_CORRECTION_ENABLED", "1") != "0"
RATE_CORRECTION = np.array([
    float(os.environ.get("RATE_CORRECTION_WX", "0.758")),
    float(os.environ.get("RATE_CORRECTION_WY", "0.739")),
    float(os.environ.get("RATE_CORRECTION_WZ", "0.665")),
]) if RATE_CORRECTION_ENABLED else np.array([1.0, 1.0, 1.0])

MARKER_LOSS_GRACE = float(os.environ.get("LANDING_MARKER_LOSS_GRACE", "1.0"))
# FALLBACK REDESIGN (2026-07-31, root-cause fix): the old FINAL_DESCENT_THROTTLE path sent
# zero body rates + a fixed throttle -- open-loop, no lateral correction of any kind. Root-
# caused this session (video analysis of the 7 real 3m flights on 2026-07-30): the marker
# only ever transits BRIEFLY through the narrow FOV (~0.8-4s) before drifting out, and once
# lost, the old fallback did nothing to counter that drift -- so the marker essentially never
# came back, every real flight required pilot takeover. PX4 has its own independent position
# estimate (no camera needed) already used for the pre-engage warmup hold (see p0/yaw0_deg
# above) -- reuse it here: hold the last-known XY position via PX4's own EKF-driven position
# controller (send_position_ned) while descending at a fixed rate in Z, instead of ignoring
# drift entirely. This directly counters the residual drift that was sweeping the marker out
# of frame, using a signal (PX4's local position) that doesn't depend on the marker at all.
FALLBACK_DESCENT_RATE_MPS = float(os.environ.get("LANDING_FALLBACK_DESCENT_RATE_MPS", "0.3"))
# send_position_ned is MAVSDK-only (no DDS path) -- see flight_controller.py's
# send_attitude_rate docstring. Per PX4_Gazebo/apps/landing_test.py's 2026-07-26
# TARGET_LOST-leveling comment, switching setpoint TYPE mid-descent would also switch
# TRANSPORT if CMD_TRANSPORT=dds is active (an explicitly untested discontinuity risk that
# fix deliberately avoided). Default CMD_TRANSPORT is "mavsdk" (confirmed: none of today's
# real flights set it to "dds"), so the position-hold fallback below is transport-safe in
# the actually-used configuration -- but stay on the old zero-rate/fixed-throttle fallback
# if dds is ever active, to preserve that same safety property.
FINAL_DESCENT_THROTTLE = float(os.environ.get("LANDING_FINAL_DESCENT_THROTTLE",
                                               str(HOVER_THROTTLE_NORM - 0.07)))
FINAL_DESCENT_TIMEOUT = float(os.environ.get("LANDING_FINAL_DESCENT_TIMEOUT_S", "5.0"))
CONTROL_TIMEOUT_S = float(os.environ.get("LANDING_CONTROL_TIMEOUT_S", "90.0"))
HOVER_STALL_S = float(os.environ.get("LANDING_HOVER_STALL_S", "25.0"))
HOVER_STALL_DZ = float(os.environ.get("LANDING_HOVER_STALL_DZ", "0.3"))


def convert_2_sys_cmd(cmd):
    """[roll_rate, pitch_rate, yaw_rate] rad/s + B_T (N, excess-over-hover) ->
    [roll_rate, pitch_rate, yaw_rate] deg/s + normalized throttle [0,1].
    Same mapping as landing_test.py's convert_2_sys_cmd - PLACEHOLDER thrust
    constants above must be calibrated for this airframe before flight."""
    thrust_norm = float(np.clip(
        HOVER_THROTTLE_NORM - cmd[3] / THRUST_SLOPE_N_PER_UNIT, 0.0, 1.0))
    rates = np.array(cmd[:3], dtype=float) * RATE_CORRECTION
    return np.append(RAD2DEG * rates, thrust_norm)


class HardwareLandingSystem:
    def __init__(self, takeoff_height=TAKEOFF_HEIGHT):
        self.fc = None
        self.controller = None
        self.takeoff_height = takeoff_height
        self.logs = {"time": [], "altitude": [], "control_output": []}

    async def initialize(self):
        print("=" * 60)
        print("Initializing Hardware Landing System")
        print("=" * 60)

        print("\n1. Connecting to flight controller via MAVSDK...")
        self.fc = FC()
        await self.fc.start()
        start_time = time.perf_counter()
        while not self.fc.has_quat():
            if (time.perf_counter() - start_time) > 20:
                raise RuntimeError("Unable to get data from Flight Controller.")
            time.sleep(0.05)
        print("   Flight controller connected, quaternion feed live")

        print("\n2. Starting Controller (owns its own IMG_PROCESSOR/camera)...")
        # Controller.__init__ constructs its own IMG_PROCESSOR internally via
        # `controller=` (the FC instance, for quat/angvel) - do NOT pass an
        # IMG_PROCESSOR here. pose_node=None: no ground-truth on hardware.
        self.controller = Controller(REF_RAD_OPT_FLOW, DES_IMG_FEATURE_PARAM,
                                      time, self.fc, pose_node=None)
        print("   Controller thread started (not yet engaged)")
        print("\n3. System ready for takeoff")

    async def arm_and_takeoff(self):
        print("\nArming and taking off to {:.2f} m...".format(self.takeoff_height))
        await self.fc.arm_and_takeoff(takeoff_hgt=self.takeoff_height)
        print("Takeoff complete")

    async def landing_loop(self):
        print("\n" + "=" * 60)
        print("Landing Control Loop")
        print("=" * 60)

        # Warmup: let the controller's internal buffers (PID integrator,
        # smoothing deques, kappa) settle while we HOLD POSITION (closed-loop),
        # NOT the controller's output. landing_test.py holds a NED setpoint here;
        # an open-loop throttle-only hover would drift/climb/drop, especially
        # with an uncalibrated HOVER_THROTTLE_NORM. Snapshot the current NED pose
        # and current yaw (from the quaternion) and hold them.
        p0 = self.fc.getPosBody()
        q0 = self.fc.getQuat()   # MAVSDK Quaternion (w, x, y, z)
        yaw0_deg = float(np.degrees(np.arctan2(
            2.0 * (q0.w * q0.z + q0.x * q0.y),
            1.0 - 2.0 * (q0.y * q0.y + q0.z * q0.z))))
        print("PID warmup (100 ms - fills deques; holding position)...")
        self.controller.startController()
        for _ in range(5):
            await self.fc.send_position_ned(p0.x_m, p0.y_m, p0.z_m, yaw0_deg)
            await asyncio.sleep(0.02)

        in_final_descent = False
        final_descent_t0 = None
        last_good_sys_cmd = None
        marker_lost_t0 = None
        start_time = time.perf_counter()
        _best_alt = None
        _stall_t0 = None

        while self.controller.is_alive() and not self.fc.LANDED:
            now = time.perf_counter()
            self.logs["time"].append(now - start_time)

            pos = self.fc.getPosBody()
            alt = -pos.z_m if pos else 0.0  # NED z is negative-up -> altitude
            self.logs["altitude"].append(alt)

            # Hover/descent-stall watchdog (onboard altitude, no ground truth needed).
            # alt is +up; descent progress = reaching a NEW LOW. Track the minimum
            # altitude and reset the stall timer whenever we descend >DZ below it.
            # (Mirrors landing_test.py, which tracks ENU min with `alt < best - DZ`.)
            if _best_alt is None or alt < _best_alt - HOVER_STALL_DZ:
                _best_alt = alt
                _stall_t0 = now
            if _stall_t0 is None:
                _stall_t0 = now
            if (now - start_time) > CONTROL_TIMEOUT_S:
                raise RuntimeError(f"control timeout: no landing in {CONTROL_TIMEOUT_S:.0f}s "
                                    f"(alt={alt:.2f} m) - aborting")
            if (now - _stall_t0) > HOVER_STALL_S:
                raise RuntimeError(f"descent stall: no >{HOVER_STALL_DZ:.2f} m descent in "
                                    f"{HOVER_STALL_S:.0f}s (alt={alt:.2f} m) - aborting")

            # CBF_CORNERS_STALE (2026-07-30): a real degraded state was found this
            # session where cbf_corners (what the visibility CBF actually reads)
            # went unavailable for 30+ seconds while TARGET_IS_VISIBLE/
            # FEATURE_IS_STALE (the signals feature_fresh already watched) kept
            # reporting fine -- those reflect a DIFFERENT signal than what gates
            # cbf_corners, so the CBF silently ran in a frozen, near-zero-
            # authority Phase-2 fallback with nothing here noticing or falling
            # back to the grace/open-loop path below. ANDed in (not OR'd) so it
            # can force feature_fresh=False even when every other signal says
            # fine. See PX4_Gazebo/docs/HANDOFF_cbf_lockout_planarmap_2026-07-30.md.
            feature_fresh = (self.controller.TARGET_IS_VISIBLE
                             and not self.controller.FEATURE_IS_STALE
                             and not self.controller.CBF_CORNERS_STALE)

            if feature_fresh and not in_final_descent:
                cmd = self.controller.getControlInput()
                sys_cmd = convert_2_sys_cmd(cmd)
                await self.fc.send_attitude_rate(*sys_cmd)
                self.logs["control_output"].append(list(sys_cmd))
                last_good_sys_cmd = sys_cmd
                marker_lost_t0 = None
            elif (not in_final_descent and last_good_sys_cmd is not None
                  and (marker_lost_t0 is None
                       or (now - marker_lost_t0) < MARKER_LOSS_GRACE)):
                if marker_lost_t0 is None:
                    marker_lost_t0 = now
                await self.fc.send_attitude_rate(*last_good_sys_cmd)
            else:
                _transport_safe_for_switch = getattr(self.fc, "_cmd_transport", "mavsdk") != "dds"
                if not in_final_descent:
                    in_final_descent = True
                    final_descent_t0 = now
                    _fallback_anchor_xy = (pos.x_m, pos.y_m) if pos else (0.0, 0.0)
                    _fallback_anchor_d0 = pos.z_m if pos else 0.0
                    if _transport_safe_for_switch:
                        print("[hardware_landing] Marker lost beyond grace - "
                              f"position-hold fallback descent (rate={FALLBACK_DESCENT_RATE_MPS} "
                              "m/s at last-known XY, PX4 EKF-driven, actively corrects drift).")
                    else:
                        print("[hardware_landing] Marker lost beyond grace - "
                              f"open-loop fallback (throttle={FINAL_DESCENT_THROTTLE}) "
                              "[CMD_TRANSPORT=dds: staying on attitude-rate, no setpoint-type switch].")
                if _transport_safe_for_switch:
                    _fallback_d = _fallback_anchor_d0 + FALLBACK_DESCENT_RATE_MPS * (now - final_descent_t0)
                    await self.fc.send_position_ned(_fallback_anchor_xy[0], _fallback_anchor_xy[1],
                                                     _fallback_d, yaw0_deg)
                else:
                    await self.fc.send_attitude_rate(0.0, 0.0, 0.0, FINAL_DESCENT_THROTTLE)
                if (now - final_descent_t0) > FINAL_DESCENT_TIMEOUT:
                    print("[hardware_landing] Final-descent timeout - PX4 never reported LANDED.")
                    break

            if self.controller.TOUCHDOWN_DETECTED and not self.fc.LANDED:
                print("[hardware_landing] Loom-inversion touchdown (controller) - LANDED")
                self.fc.LANDED = True

            if len(self.logs["time"]) % 100 == 0:
                print(f"  t={now - start_time:.1f}s alt={alt:.2f}m "
                      f"visible={self.controller.TARGET_IS_VISIBLE} "
                      f"stale={self.controller.FEATURE_IS_STALE}")

            await asyncio.sleep(SLEEP_TIME)

        if self.fc.LANDED:
            print("Landed (PX4 LandedState or controller touchdown detect)")
            # Command a safe PX4 LAND here, not an immediate disarm: the
            # loom-inversion touchdown detector (TOUCHDOWN_DETECTED) can latch
            # LANDED on a false positive well above the ground (observed
            # mid-air at ~0.96m) -- an unconditional disarm() at that point
            # cuts motors in flight. PX4 land() is safe either way: it is a
            # controlled descent-and-disarm if genuinely still airborne, and
            # a fast no-op-ish disarm if already on the ground.
            try:
                await self.fc.vehicle.action.land()
                print("Land command issued post-touchdown.")
            except Exception as e:
                print(f"Land command failed (probably already landed/disarmed by PX4): {e}")

    def save_data(self):
        """Persist telemetry/controller logs to disk. Called BEFORE cleanup()
        (matches output_calibration.py / record_input_calibration.py's
        pattern - getLogData() must run before close() tears down state).
        Matches the established Pi convention (Test_Data/Calibration/<ts> for
        output-cal) with a distinctly-named Landing/ subfolder so real flight
        runs are never confused with calibration runs."""
        if not self.fc or not self.controller:
            print("No FC/controller - nothing to save.")
            return
        base = os.environ.get("LANDING_OUT_BASE", "Test_Data/Landing")
        dir_name = os.environ.get(
            "LANDING_OUT_DIR", f"{base}/{time.ctime().replace(':', '-')}")
        os.makedirs(dir_name, exist_ok=True)

        telemetry_data = self.fc.getLogData()
        controller_data = self.controller.getLogData()
        controller_params = self.controller.getParams()
        img_params = self.controller.getImgParams()

        np.save(f"{dir_name}/Telemetry_Data", telemetry_data)
        np.save(f"{dir_name}/Control_Data", controller_data)
        np.save(f"{dir_name}/Control_Params", controller_params)
        np.save(f"{dir_name}/Local_Logs", self.logs)
        with open(f"{dir_name}/Img_Params.txt", "w") as f:
            f.write(str(img_params))
        print(f"\nFlight data saved -> {dir_name}")

    async def cleanup(self):
        print("\nCleaning up...")
        if self.controller and self.controller.is_alive():
            self.controller.close()
            self.controller.join(timeout=3)
        if self.fc:
            await self.fc.close()
        print("Cleanup complete")

    def print_summary(self):
        print("\n" + "=" * 60)
        print("Test Summary")
        print("=" * 60)
        if not self.logs["time"]:
            print("No data collected")
            return
        duration = self.logs["time"][-1]
        frames = len(self.logs["time"])
        altitudes = [a for a in self.logs["altitude"] if a]
        if altitudes:
            print(f"Altitude: min={min(altitudes):.2f}m, max={max(altitudes):.2f}m, "
                  f"avg={np.mean(altitudes):.2f}m")
        print(f"Duration: {duration:.1f}s, iterations: {frames}")


async def main():
    print("\n" + "=" * 60)
    print("Hardware Landing System")
    print("=" * 60)
    print(f"Started: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")

    if HOVER_THROTTLE_NORM == 0.738 and "HW_HOVER_THROTTLE_NORM" not in os.environ:
        print("\n*** WARNING: HOVER_THROTTLE_NORM is the SITL placeholder (0.738), "
              "NOT calibrated for this airframe. Set HW_HOVER_THROTTLE_NORM / "
              "HW_THRUST_SLOPE from this vehicle's own calibration before flying. ***\n")

    system = HardwareLandingSystem(takeoff_height=TAKEOFF_HEIGHT)
    try:
        await system.initialize()
        await system.arm_and_takeoff()
        await system.landing_loop()
    except KeyboardInterrupt:
        print("\nInterrupted by user")
        if system.fc:
            try:
                await system.fc.vehicle.action.land()
            except Exception:
                pass
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
        # On any mid-flight abort (control timeout, descent stall, etc.) command
        # a safe land BEFORE cleanup tears down the setpoint stream — otherwise
        # the drone is left airborne in OFFBOARD relying on PX4's failsafe.
        if system.fc:
            try:
                print("[hardware_landing] Aborting - commanding PX4 land.")
                await system.fc.vehicle.action.land()
            except Exception as le:
                print(f"[hardware_landing] Land command failed: {le}")
    finally:
        try:
            system.save_data()
        except Exception as se:
            print(f"[hardware_landing] Failed to save flight data: {se}")
        await system.cleanup()
        system.print_summary()

    print("\n" + "=" * 60)


if __name__ == "__main__":
    asyncio.run(main())
