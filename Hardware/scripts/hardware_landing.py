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

# CAM_MANUAL_EXPOSURE (2026-08-01): was only ever set by output_calibration.py's own
# hand-sweep recordings (see imgstreamer.py's CAM_MANUAL_EXPOSURE comment for the full
# 2026-07-22 diagnosis -- vigorous motion under auto-exposure blurs the marker's bit
# pattern past decodability). Every real hardware_landing.py flight through 2026-07-31
# ran on auto-exposure/auto-gain instead, since this script never set it -- a real drone
# in flight moves at least as vigorously as a calibration hand-sweep, so the same
# motion-blur risk applies here and was never mitigated. Default ON; must be set BEFORE
# importing controller/img_data/imgstreamer below, since these are read at their MODULE
# IMPORT time, not per-call.
os.environ.setdefault("CAM_MANUAL_EXPOSURE", "1")

# CAPTURE_RATE_HZ=30 (2026-08-01): the real achieved image rate on this hardware is
# ~25-31Hz regardless of what's requested (empirical raw-mode ceiling, see
# img_process_freq_optimization.md + this session's own Img_Data.npy measurement) --
# so requesting the previous default (60Hz) bought nothing but DID cap ExposureTime at
# ~16667us via FrameDurationLimits. Requesting 30Hz explicitly (matching what's actually
# delivered) raises that ceiling to ~33333us at no real throughput cost -- see
# FLIGHT_TEST_ANALYSIS_PROCEDURE.md catalog #12.
os.environ.setdefault("CAPTURE_RATE_HZ", "30")

# CAM_EXPOSURE_US=20000 / CAM_AUTO_GAIN=1 (2026-08-01): validated 2026-08-01 on real
# bench recordings -- the old default (3000us/8.0 gain, auto-exposure's usual ballpark)
# gave 0% ArUco decode in poor indoor lighting (a real sensor noise-floor problem, not a
# levels problem -- proven by digital brightness-correction alone still failing).
# 20000us (still well under the ~33333us ceiling above) took that to 43-65% decode on
# real footage. CAM_AUTO_GAIN=1 closes the loop on GAIN ONLY (exposure stays fixed at
# CAM_EXPOSURE_US, so blur behavior is unaffected by lighting) targeting the empirically
# best brightness band (~73-90) found the same session -- this is what lets ONE
# configuration serve both poor and good lighting without knowing conditions ahead of
# flight time; a single static gain can't (a bright-enough-for-dark gain overexposes a
# lit scene). Real validated result: 73.7% decode across a 60s bench recording spanning
# a genuine lighting change mid-recording, best result of the session. See
# FLIGHT_TEST_ANALYSIS_PROCEDURE.md catalog #12/#13 before changing these.
os.environ.setdefault("CAM_EXPOSURE_US", "20000")
os.environ.setdefault("CAM_AUTO_GAIN", "1")

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
# FALLBACK REDESIGN (2026-07-31): the old path sent zero body rates + a fixed throttle --
# open-loop, freezing whatever tilt the vehicle had at the instant the marker was lost, with
# no correction of any kind. Root-caused this session (video analysis of the 7 real 3m
# flights on 2026-07-30): the marker only ever transits BRIEFLY through the narrow FOV
# (~0.8-4s) before drifting out, and once lost, the old fallback did nothing about it. Went
# through several iterations before landing here: (1) hold last-known XY via PX4's GPS-
# derived position EKF -- REJECTED, this project's controller must never depend on GPS
# (GPS-denied vision-only landing is the whole point; GPS on this airframe is logging-only,
# per user); (2) GPS-free IMU-only active leveling + closed-loop throttle, ported from
# PX4_Gazebo's TARGET_LOST-leveling fix -- worked but was still custom-reinvented flight
# logic; (3) FINAL, per user direction: since this state is a DECLARED FAILURE and no
# attempt is made to regain the marker, stop trying to fly/descend ourselves at all and hand
# off entirely to PX4's own mature RETURN-TO-LAUNCH mode (climb to a safe altitude, fly back
# to the launch point, land there) -- simpler, more robust, and the standard failsafe
# response for a genuine failure, instead of reinventing descent/leveling logic.
# SEARCH-CLIMB REDESIGN (2026-08-04, per explicit user direction, supersedes the
# 2026-07-31 RTL handoff above): root-caused via a real 2026-08-03 "descent stall"
# abort (see project_pi_descent_stall_search_climb_2026_08_04 memory) that PX4's
# native return_to_launch() computed a nonsensical RTL altitude (954m, logged live)
# for this indoor/no-real-GPS-fix test setup -- RTL's altitude planning depends on
# GPS/AMSL semantics that don't hold indoors, making it fundamentally unsound as the
# marker-loss fallback here, not just this one bad run. Replaced with a HOLD +
# CLIMB-SEARCH fallback using our own OFFBOARD position setpoints (same
# send_position_ned() mechanism already used, and already accepted, for takeoff-climb
# and the PID-warmup hold above -- this does NOT reintroduce the previously-rejected
# "GPS position-hold during marker-loss to keep flying ourselves" design (that
# rejection was about substituting GPS-derived position for the vision-based LANDING
# control law itself; this is a bounded, temporary safety hold/search when vision is
# already unavailable, exactly like the takeoff climb already is).
#
# The Controller's own background thread (controller.py::run()) ALREADY auto-
# reinitializes (archives the old getLogData() segment, resets kappa/sigma/all
# adaptive/integrator state to fresh __init__ values) every time
# self._img_node.FEATURE_IS_VISIBLE transitions False->True -- see
# _initialize_controller()'s call site and getLogData()'s own docstring ("locate
# re-inits via kappa resetting to kappa_0"). So no new reset logic is needed here:
# once the search-climb reacquires the marker, simply resuming the normal branch
# below is sufficient -- getControlInput() will already reflect the fresh state.
SEARCH_CLIMB_RATE_M_S = float(os.environ.get("LANDING_SEARCH_CLIMB_RATE_M_S", "0.3"))
SEARCH_CLIMB_MAX_ADD_M = float(os.environ.get("LANDING_SEARCH_CLIMB_MAX_ADD_M", "1.5"))
SEARCH_TIMEOUT_S = float(os.environ.get("LANDING_SEARCH_TIMEOUT_S", "45.0"))
MAX_DESCENT_ATTEMPTS = int(os.environ.get("LANDING_MAX_DESCENT_ATTEMPTS", "3"))
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
        # "attempt"/"state" (2026-08-04, search-climb redesign): every tick is now
        # tagged with which descent attempt it belongs to and whether the app was
        # actively descending or in the hold+search fallback -- needed now that a
        # single continuous flight/recording can contain multiple landing attempts.
        self.logs = {"time": [], "altitude": [], "control_output": [], "attempt": [], "state": []}

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
        # Off by default (video write cost every frame) - env-gated so a real
        # flight can opt in when video/estimator-tag alignment is wanted (see
        # project_pi_izeta_kappa_ratchet_fix_2026_07_31: real landing flights
        # had video but no Img_Data.npy at all, so there was no way to align
        # recorded video frames to "Opt Flow Estimator Tag" after the fact).
        if os.environ.get("LANDING_RECORD_VIDEO", "0") == "1":
            self.controller.enableRecording()
            print("   Video recording ENABLED (LANDING_RECORD_VIDEO=1) -> "
                  "Test_Data/Landing/Test_Videos/<timestamp>.mp4")
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

        # in_search_climb (2026-08-04, replaces the old in_final_descent RTL-handoff
        # flag): True while holding position + climbing to search for the marker
        # after a declared marker-loss-beyond-grace. attempt_idx counts descent
        # attempts (starts at 1 for the initial descent; incremented each time the
        # marker is reacquired after a search) -- logged every tick via
        # self.logs["attempt"] so a single continuous recording spanning multiple
        # attempts can be told apart post-hoc.
        in_search_climb = False
        search_t0 = None
        search_hold_x = search_hold_y = search_base_z = search_hold_yaw = None
        attempt_idx = 1
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
            self.logs["attempt"].append(attempt_idx)
            self.logs["state"].append("search" if in_search_climb else "descent")

            # Hover/descent-stall watchdog (onboard altitude, no ground truth needed).
            # alt is +up; descent progress = reaching a NEW LOW. Track the minimum
            # altitude and reset the stall timer whenever we descend >DZ below it.
            # (Mirrors landing_test.py, which tracks ENU min with `alt < best - DZ`.)
            # SUSPENDED during in_search_climb (2026-08-04): deliberately not
            # descending while holding/searching is expected, not a stall -- reset
            # cleanly (_best_alt=None) the moment a fresh descent attempt resumes,
            # see the search-climb reacquisition handling below.
            if not in_search_climb:
                if _best_alt is None or alt < _best_alt - HOVER_STALL_DZ:
                    _best_alt = alt
                    _stall_t0 = now
                if _stall_t0 is None:
                    _stall_t0 = now
                if (now - _stall_t0) > HOVER_STALL_S:
                    raise RuntimeError(f"descent stall: no >{HOVER_STALL_DZ:.2f} m descent in "
                                        f"{HOVER_STALL_S:.0f}s (alt={alt:.2f} m) - aborting")
            if (now - start_time) > CONTROL_TIMEOUT_S:
                raise RuntimeError(f"control timeout: no landing in {CONTROL_TIMEOUT_S:.0f}s "
                                    f"(alt={alt:.2f} m) - aborting")

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
            #
            # FALSE-ABORT FIX (2026-07-31): CBF_CORNERS_STALE's fast ~30-frame/1s
            # threshold was designed for the LOW-RISK kappa-freeze use inside
            # controller.py (just pauses adaptation). Reusing it HERE for the
            # irreversible mission-abort decision was wrong -- normal ArUco coast
            # bursts on this hardware commonly run 2-327 frames even during ordinary
            # tracking (see project_pi_coast_root_cause, 2026-07-27), so the fast
            # threshold false-tripped on nearly every flight of the 2026-07-31 test
            # session (15/17 aborted to RTL within seconds, none reaching a sustained
            # closed-loop descent). Use CBF_CORNERS_STALE_ABORT instead here -- a
            # separate, longer-fused counter (default 350 frames, override via
            # CBF_CORNERS_STALE_ABORT_FRAMES) sized past the observed normal-coast
            # range, so only a genuinely sustained loss triggers the abort.
            cbf_corners_stale_abort = self.controller.CBF_CORNERS_STALE_ABORT
            feature_fresh = (self.controller.TARGET_IS_VISIBLE
                             and not self.controller.FEATURE_IS_STALE
                             and not cbf_corners_stale_abort)

            if in_search_climb:
                # SEARCH-CLIMB fallback (2026-08-04). Check reacquisition FIRST, before
                # sending another search setpoint this tick -- the Controller's own
                # background thread already auto-reinitialized (fresh kappa/sigma/
                # integrators, archived the prior getLogData() segment) the instant
                # self.controller.TARGET_IS_VISIBLE flipped back True (see
                # controller.py::run()), so simply resuming the normal branch on the
                # NEXT iteration is sufficient -- no manual reset needed here.
                if feature_fresh:
                    print(f"[hardware_landing] {datetime.now().isoformat()} -- Marker "
                          f"reacquired after {now - search_t0:.1f}s search (attempt "
                          f"{attempt_idx} -> {attempt_idx + 1}) - resuming descent "
                          f"(controller auto-reinitialized on reacquisition).")
                    in_search_climb = False
                    attempt_idx += 1
                    marker_lost_t0 = None
                    last_good_sys_cmd = None
                    _best_alt = None   # restart descent-stall tracking fresh for the new attempt
                    _stall_t0 = now
                else:
                    elapsed = now - search_t0
                    # NED z is negative-up: climbing = more NEGATIVE z. Ramp from
                    # search_base_z toward (search_base_z - SEARCH_CLIMB_MAX_ADD_M) at
                    # SEARCH_CLIMB_RATE_M_S, capped at the max additional height.
                    climb_add = min(SEARCH_CLIMB_MAX_ADD_M, SEARCH_CLIMB_RATE_M_S * elapsed)
                    target_z = search_base_z - climb_add
                    await self.fc.send_position_ned(search_hold_x, search_hold_y,
                                                     target_z, search_hold_yaw)
                    if elapsed > SEARCH_TIMEOUT_S:
                        print(f"[hardware_landing] {datetime.now().isoformat()} -- "
                              f"Search-climb timed out after {elapsed:.1f}s (attempt "
                              f"{attempt_idx}) - commanding safe LAND at current position.")
                        try:
                            await self.fc.vehicle.action.land()
                        except Exception as e:
                            print(f"[hardware_landing] LAND command failed: {e}")
                        break
            elif feature_fresh:
                cmd = self.controller.getControlInput()
                sys_cmd = convert_2_sys_cmd(cmd)
                await self.fc.send_attitude_rate(*sys_cmd)
                self.logs["control_output"].append(list(sys_cmd))
                last_good_sys_cmd = sys_cmd
                marker_lost_t0 = None
            elif (last_good_sys_cmd is not None
                  and (marker_lost_t0 is None
                       or (now - marker_lost_t0) < MARKER_LOSS_GRACE)):
                if marker_lost_t0 is None:
                    marker_lost_t0 = now
                await self.fc.send_attitude_rate(*last_good_sys_cmd)
            elif attempt_idx >= MAX_DESCENT_ATTEMPTS:
                # Attempt budget already exhausted (attempt_idx only increments on a
                # SUCCESSFUL reacquisition, so this is the count of completed descent
                # attempts) -- don't start yet another search, go straight to a safe
                # LAND at the current position instead.
                print(f"[hardware_landing] {datetime.now().isoformat()} -- Marker lost "
                      f"beyond grace after {attempt_idx} descent attempt(s) (max "
                      f"{MAX_DESCENT_ATTEMPTS}) - commanding safe LAND at current position.")
                try:
                    await self.fc.vehicle.action.land()
                except Exception as e:
                    print(f"[hardware_landing] LAND command failed: {e}")
                break
            else:
                # HOLD + CLIMB-SEARCH fallback (2026-08-04 -- see the module-level
                # SEARCH_CLIMB_* comment for why this replaced the RTL handoff).
                # Snapshot the current NED pose/yaw exactly like the PID-warmup hold
                # above, and start climbing from here to search for reacquisition.
                p_now = self.fc.getPosBody()
                q_now = self.fc.getQuat()
                search_hold_x, search_hold_y, search_base_z = p_now.x_m, p_now.y_m, p_now.z_m
                search_hold_yaw = float(np.degrees(np.arctan2(
                    2.0 * (q_now.w * q_now.z + q_now.x * q_now.y),
                    1.0 - 2.0 * (q_now.y * q_now.y + q_now.z * q_now.z))))
                in_search_climb = True
                search_t0 = now
                print(f"[hardware_landing] {datetime.now().isoformat()} -- Marker lost "
                      f"beyond grace (attempt {attempt_idx}) - holding position and "
                      f"climbing to search (base_alt={-search_base_z:.2f}m, max +"
                      f"{SEARCH_CLIMB_MAX_ADD_M:.1f}m, timeout {SEARCH_TIMEOUT_S:.0f}s, "
                      f"max {MAX_DESCENT_ATTEMPTS} attempts).")
                await self.fc.send_position_ned(search_hold_x, search_hold_y,
                                                 search_base_z, search_hold_yaw)

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
        # ADDED 2026-07-31: real flights previously saved no per-frame
        # "Opt Flow Estimator Tag"/"Time" at all (only Control_Data.npy's
        # controller-internal h(t)/s(t)), so recorded video (when
        # LANDING_RECORD_VIDEO=1) could never be aligned to which estimator
        # fired each frame - see project_pi_izeta_kappa_ratchet_fix_2026_07_31.
        # getImgData() mirrors output_calibration.py's own image_data save.
        img_data = self.controller.getImgData()

        np.save(f"{dir_name}/Telemetry_Data", telemetry_data)
        np.save(f"{dir_name}/Control_Data", controller_data)
        np.save(f"{dir_name}/Control_Params", controller_params)
        np.save(f"{dir_name}/Img_Data", img_data)
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
