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

# CAM_EXPOSURE_US=200 (2026-08-12, user bake): matches what every real flight from
# 2026-08-11 onward actually launched with (the command line overrode the previous 20000
# default on every single run; 08-05..08-10 used 2000). Making the default match flown
# practice removes the per-launch override and stops the file default drifting away from
# what the vehicle is really configured with.
#
# HISTORY of the previous 20000 default (2026-08-01): validated on real BENCH recordings
# -- 3000us/8.0 gain gave 0% ArUco decode in poor indoor lighting (a sensor noise-floor
# problem, not a levels problem), and 20000us took that to 43-65%, peaking at 73.7% across
# a 60s recording spanning a lighting change. That result was never reproduced in flight:
# the 2026-08-12 campaign review measured 6.4% median decode at 200us vs 5.3% at 2000us
# across 215 flights, i.e. exposure barely moved decode rate in the air, so the bench
# number does not transfer. The binding limiter in flight is marker PIXEL EXTENT (0.28m
# marker at 320x240, f~=515px -> 48px = 8px/ArUco-cell at 3m), not exposure time -- see
# project_pi_cbf_phase2_zeroes_lateral_2026_08_12 and FLIGHT_TEST_ANALYSIS_PROCEDURE.md
# catalog #12/#13. If venue lighting changes, re-pick this with a record_test_feed.py
# bench run in the ACTUAL venue rather than assuming either number transfers.
#
# CAM_AUTO_GAIN=1 closes the loop on GAIN ONLY (exposure stays fixed at CAM_EXPOSURE_US,
# so blur behavior is unaffected by lighting), targeting the empirically best brightness
# band (~73-90) -- this is what lets one configuration serve both poor and good lighting
# without knowing conditions ahead of flight time; a single static gain can't.
os.environ.setdefault("CAM_EXPOSURE_US", "200")
os.environ.setdefault("CAM_AUTO_GAIN", "1")

# CAM_AUTO_EXPOSURE=1 (2026-08-17): was previously OFF here (unset), so exposure sat
# pinned at CAM_EXPOSURE_US=200 for every flight regardless of actual lighting -- correct
# for direct sun, wrong for anything dimmer (cloud cover, overcast, dusk), with no way to
# recover since gain alone can't compensate once its own [1.0, 16.0] range is pinned.
# Real outdoor hand-held bench validation (2026-08-17, two independent runs -- see
# project_pi_outdoor_exposure_2000us_failed memory): sunny settled at 200us (99.8% decode),
# cloudy settled at 500us (100% decode), both via the step-DOWN/step-UP logic in
# imgstreamer.py correctly tracking real conditions -- a single fixed value cannot serve
# both. CAM_EXPOSURE_MAX_US intentionally left at imgstreamer.py's own default (4000, see
# that file's comment) -- outdoor tests never approached that ceiling, so it doesn't
# constrain the outdoor case; it exists as a blur-safety cap if conditions are ever dim
# enough to need to step up.
#
# NOTE: a prior 215-flight campaign review (2026-08-12, see CAM_EXPOSURE_US comment above)
# found exposure barely moved real-flight decode rate historically, attributing the
# dominant limiter to marker PIXEL EXTENT, not exposure time. This change is justified by
# the 2026-08-17 bench evidence but should NOT be assumed to fix decode rate on its own --
# see project_pi_cbf_phase2_zeroes_lateral_2026_08_12 for the still-open Tier 0 work.
os.environ.setdefault("CAM_AUTO_EXPOSURE", "1")

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

# BATTERY-VOLTAGE HOVER CORRECTION (2026-08-21): HOVER_THROTTLE_NORM=0.42 is a
# single fixed value; project_hover_voltage_curve (find_hover_throttle.py bisection
# campaign, 2026-07-09/10) found the REAL hover throttle is voltage-dependent --
# flat ~0.388 across a 22.4-24.0V plateau, but rising above 0.389 below ~22.2V and
# falling below 0.387 above ~24.25V (higher voltage needs less throttle to hover).
# Root-caused as the explanation for a recurring uncommanded-climb-to-8m incident
# (2026-08-20/21): every excess-climb event across 10 battery swaps/5 days of test
# data fell in the first 1-4 runs of a fresh-battery session (highest voltage of
# the session -> true hover point lowest, below the fixed 0.42 assumption) and
# NEVER at session position 5+ (voltage settled into the plateau). This
# correction is COARSE -- built from the existing project_hover_voltage_curve
# clusters (21.68-24.36V, several points still "not yet bisected" per that memory,
# not a clean regression fit) -- not a precise linear model. Deliberately
# conservative at the edges (rounds toward MORE throttle, i.e. less aggressive
# descent, at low voltage; LESS throttle, i.e. less aggressive climb-risk, at high
# voltage) since a wrong correction that undershoots is safer than one that
# overshoots into the opposite failure mode. Set HW_HOVER_VOLTAGE_CORRECTION=0 to
# disable and use the flat HOVER_THROTTLE_NORM unconditionally (e.g. if this
# correction is ever found to hurt more than help). An explicit
# HW_HOVER_THROTTLE_NORM override still takes precedence (skips voltage read
# entirely) -- same override-wins convention as PLASMC_HW_MARKER_NED_XYZ elsewhere
# in this file.
HOVER_VOLTAGE_CORRECTION_ENABLED = (
    os.environ.get("HW_HOVER_VOLTAGE_CORRECTION", "1") != "0"
    and "HW_HOVER_THROTTLE_NORM" not in os.environ
)


def _voltage_corrected_hover_throttle(voltage_v):
    """Coarse piecewise hover-throttle estimate from project_hover_voltage_curve's
    clusters. Returns HOVER_THROTTLE_NORM unchanged if voltage is None or falls
    in the well-supported 22.4-24.0V plateau; nudges up/down outside it."""
    if voltage_v is None:
        return HOVER_THROTTLE_NORM
    if voltage_v <= 22.2:
        return 0.395       # 21.68-22.19V cluster: every point sank even at 0.389
    if voltage_v <= 24.0:
        return 0.388       # 22.4-24.0V plateau: converged across 5 sweeps
    return 0.380            # >24.0V: 24.25/24.36V points all climbed even below 0.388

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

# HW_POS_FEEDBACK VISIBILITY DECOUPLING (2026-08-17, per explicit user direction):
# when PLASMC_HW_POS_FEEDBACK=1, the control loop is driven entirely by the FC's
# own EKF position/attitude (hw_pos_feedback.py), not by ArUco/optical-flow
# perception -- so the marker-visibility-gated grace/search-climb/abort machinery
# below (designed for the perception-only path, where losing the marker means
# losing the control signal) no longer has anything to protect: losing sight of
# the marker does not touch what's driving control in this mode. Read once at
# import time (same convention as every other env flag in this file).
_HW_POS_FEEDBACK_ACTIVE = os.environ.get("PLASMC_HW_POS_FEEDBACK", "0") == "1"

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


# Mutable: HardwareLandingSystem.arm_and_takeoff() overwrites this once, right
# after takeoff, with the voltage-corrected value (see HOVER_VOLTAGE_CORRECTION_ENABLED
# above) -- convert_2_sys_cmd reads it fresh every call rather than closing over
# the module-level constant, so the correction takes effect for the whole flight.
_active_hover_throttle_norm = HOVER_THROTTLE_NORM


def convert_2_sys_cmd(cmd):
    """[roll_rate, pitch_rate, yaw_rate] rad/s + B_T (N, excess-over-hover) ->
    [roll_rate, pitch_rate, yaw_rate] deg/s + normalized throttle [0,1].
    Same mapping as landing_test.py's convert_2_sys_cmd - PLACEHOLDER thrust
    constants above must be calibrated for this airframe before flight."""
    thrust_norm = float(np.clip(
        _active_hover_throttle_norm - cmd[3] / THRUST_SLOPE_N_PER_UNIT, 0.0, 1.0))
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
        # "marker_visible" (2026-08-17): raw perception visibility, logged every
        # tick REGARDLESS of whether PLASMC_HW_POS_FEEDBACK is bypassing it for
        # control decisions -- needed to compare perception vs FC-driven control
        # post-hoc (did the marker actually stay in view while FC data flew the
        # approach, and if not, did FC-driven control still look sane through
        # that stretch?).
        self.logs = {"time": [], "altitude": [], "control_output": [], "attempt": [],
                      "state": [], "marker_visible": []}

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
        # IMG_PROCESSOR here.
        #
        # PLASMC_HW_POS_FEEDBACK=1: pass an HWPoseNode (analytic s/h from the PX4
        # EKF position/attitude + a fixed measured marker NED point, in place of
        # ArUco/optical-flow perception). NOT true ground truth -- see
        # hw_pos_feedback.py's module docstring. Off by default (pose_node=None,
        # perception-only, unchanged behavior).
        hw_pose_node = None
        if os.environ.get("PLASMC_HW_POS_FEEDBACK", "0") == "1":
            from hw_pos_feedback import HWPoseNode
            hw_pose_node = HWPoseNode(self.fc)
            if "PLASMC_HW_MARKER_NED_XYZ" in os.environ:
                print("   PLASMC_HW_POS_FEEDBACK=1 -- analytic EKF-position feedback armed"
                      f" (marker NED override = {os.environ['PLASMC_HW_MARKER_NED_XYZ']})")
            else:
                print("   PLASMC_HW_POS_FEEDBACK=1 -- analytic EKF-position feedback armed"
                      " (marker position will be snapshotted from current pose right before arming)")
        self.hw_pose_node = hw_pose_node   # stashed so arm_and_takeoff() can snapshot it just before arming
        self.controller = Controller(REF_RAD_OPT_FLOW, DES_IMG_FEATURE_PARAM,
                                      time, self.fc, pose_node=hw_pose_node)
        # UNIFIED 2026-08-16: was a separate LANDING_RECORD_VIDEO flag that just
        # called enableRecording() -> self._img_node.RECORD = True -- the exact same
        # effect as IMG_RECORD=1 (checked inside Controller.__init__, controller.py).
        # Two names for one flag was a footgun (easy to set the wrong one and get no
        # video); IMG_RECORD is the name already used everywhere else in the
        # codebase, so this now checks that instead. (Controller.__init__ already
        # honors IMG_RECORD=1 at construction time above, so this block only remains
        # for the printed confirmation -- explicit >>> enableRecording() is otherwise
        # redundant but harmless if IMG_RECORD was also passed as record != 'n'.)
        if os.environ.get("IMG_RECORD", "0") == "1":
            self.controller.enableRecording()
            print("   Video recording ENABLED (IMG_RECORD=1) -> "
                  "Test_Data/Landing/Test_Videos/<timestamp>.mp4")
        print("   Controller thread started (not yet engaged)")
        print("\n3. System ready for takeoff")

    async def arm_and_takeoff(self):
        # Snapshot the marker's NED position from the vehicle's OWN current pose,
        # right here -- the last moment it's guaranteed to still be sitting on the
        # marker (see hw_pos_feedback.py's HWPoseNode docstring for why a hardcoded
        # (0,0,0) is wrong even when physically powered on at the marker: the PX4
        # local origin anchors to an early, still-converging GPS estimate, so
        # getPosBody() drifts away from (0,0,0) over the minutes between power-up
        # and arming even with zero physical movement). No-op if
        # PLASMC_HW_MARKER_NED_XYZ was set explicitly (that override wins).
        if self.hw_pose_node is not None:
            self.hw_pose_node.snapshotMarkerFromCurrentPosition()
        print("\nArming and taking off to {:.2f} m...".format(self.takeoff_height))
        await self.fc.arm_and_takeoff(takeoff_hgt=self.takeoff_height)
        print("Takeoff complete")

        # Voltage-corrected hover throttle (see HOVER_VOLTAGE_CORRECTION_ENABLED
        # above) -- read once here, same MAVSDK call find_hover_throttle.py uses.
        global _active_hover_throttle_norm
        if HOVER_VOLTAGE_CORRECTION_ENABLED:
            try:
                battery = await self.fc.vehicle.telemetry.battery().__aiter__().__anext__()
                v = float(battery.voltage_v)
                _active_hover_throttle_norm = _voltage_corrected_hover_throttle(v)
                print(f"Battery voltage: {v:.2f}V -> hover throttle {_active_hover_throttle_norm:.3f} "
                      f"(base {HOVER_THROTTLE_NORM:.3f})")
            except Exception as e:
                print(f"[WARN] Could not read battery voltage, using base hover throttle "
                      f"{HOVER_THROTTLE_NORM:.3f}: {e}")
                _active_hover_throttle_norm = HOVER_THROTTLE_NORM

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
        # STARTUP-GRACE FIX (2026-08-04, see the grace-hold elif below): snapshot of
        # current pose, taken ONCE the instant we first hit a miss with no
        # last_good_sys_cmd yet -- reused for the duration of that specific grace
        # window (same one-shot-snapshot pattern as search_hold_x/y/z/yaw above, not
        # re-fetched every tick, to avoid any control chatter from constantly
        # re-anchoring to a slightly-drifted "current" position).
        startup_hold_pose = None
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
            # checkTargetVisibility() (-> self._img_node.FEATURE_IS_VISIBLE directly),
            # NOT self.controller.TARGET_IS_VISIBLE: under PLASMC_HW_POS_FEEDBACK (or
            # Gazebo's PLASMC_GT_FEEDBACK) controller.py::run()'s own visibility-latch
            # only clears while `self._img_node.FEATURE_IS_VISIBLE or self._gt_feedback
            # is not None` is False -- and self._gt_feedback is not None is permanently
            # true in this mode, so that branch never runs and TARGET_IS_VISIBLE gets
            # stuck True after its first tick, never reflecting a later marker loss
            # (confirmed in Test_Data/Landing/17082026.txt: "visible=True" printed
            # straight through stretches where the perception layer itself logged
            # "LANDING PAD NOT VISIBLE..."). checkTargetVisibility() reads the flag
            # un-latched, so it stays accurate for this comparison regardless of mode.
            self.logs["marker_visible"].append(bool(self.controller.checkTargetVisibility()))

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
            # checkTargetVisibility(), not TARGET_IS_VISIBLE -- see the
            # self.logs["marker_visible"] comment above for why the latter is
            # unusable once PLASMC_HW_POS_FEEDBACK is active (stuck True after its
            # first tick). Using checkTargetVisibility() here also fixes a latent
            # bug in the perception-only path's own gating: TARGET_IS_VISIBLE was
            # already relying on the same latch machinery, just harmlessly (it
            # tracked correctly there ONLY because self._gt_feedback was always
            # None on hardware before PLASMC_HW_POS_FEEDBACK existed).
            raw_feature_fresh = (self.controller.checkTargetVisibility()
                                 and not self.controller.FEATURE_IS_STALE
                                 and not cbf_corners_stale_abort)
            # PLASMC_HW_POS_FEEDBACK=1: control gating is decoupled from raw
            # perception visibility (see the module-level _HW_POS_FEEDBACK_ACTIVE
            # comment) -- force the gate open so the branches below always take
            # the normal-descent path and never enter grace/search-climb/abort.
            # raw_feature_fresh itself (and self.logs["marker_visible"] above)
            # still reflect the TRUE perception state every tick, unaffected by
            # this override, so a post-hoc comparison of "was the marker actually
            # visible" vs "what did FC-driven control do" stays possible.
            feature_fresh = True if _HW_POS_FEEDBACK_ACTIVE else raw_feature_fresh

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
                    startup_hold_pose = None   # so a later no-command grace-hold gets a fresh snapshot
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
                startup_hold_pose = None   # so a later no-command grace-hold gets a fresh snapshot
            elif (marker_lost_t0 is None
                  or (now - marker_lost_t0) < MARKER_LOSS_GRACE):
                # STARTUP-GRACE FIX (2026-08-04, ported from the identical fix in
                # PX4_Gazebo/apps/landing_test.py -- confirmed via direct SITL
                # instrumentation that a miss on the VERY FIRST control tick after
                # engage, before last_good_sys_cmd is ever set, previously had NO
                # grace window here: this elif's `last_good_sys_cmd is not None`
                # requirement made it skip straight to the attempt-budget/
                # HOLD+CLIMB-SEARCH branches below on that very first miss --
                # launching a spurious search-climb maneuver on nothing more than
                # "haven't seen the marker yet, give it one tick." Fix: enter grace
                # even with no prior command -- hold the CURRENT position (snapshot
                # ONCE, same one-shot pattern as the search-climb branch's own
                # search_hold_x/y/z/yaw) instead of assuming a last_good_sys_cmd
                # exists. An attitude-rate/thrust guess was deliberately avoided here
                # (see the PID-warmup comment above: "an open-loop throttle-only
                # hover would drift/climb/drop, especially with an uncalibrated
                # HOVER_THROTTLE_NORM") -- send_position_ned is what this file
                # already trusts for a safe hold.
                if marker_lost_t0 is None:
                    marker_lost_t0 = now
                if last_good_sys_cmd is not None:
                    await self.fc.send_attitude_rate(*last_good_sys_cmd)
                else:
                    if startup_hold_pose is None:
                        p_h = self.fc.getPosBody()
                        q_h = self.fc.getQuat()
                        yaw_h = float(np.degrees(np.arctan2(
                            2.0 * (q_h.w * q_h.z + q_h.x * q_h.y),
                            1.0 - 2.0 * (q_h.y * q_h.y + q_h.z * q_h.z))))
                        startup_hold_pose = (p_h.x_m, p_h.y_m, p_h.z_m, yaw_h)
                    await self.fc.send_position_ned(*startup_hold_pose)
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
                # `stale=` REMOVED 2026-08-12: it printed controller.FEATURE_IS_STALE,
                # which forwards a flag the Pi's img_data.py never defines (see its
                # FEATURE_PTS_FRESH docstring: "Pi has none currently, unlike Gazebo's
                # legacy FEATURE_IS_STALE"). controller.py's property resolves it via
                # getattr(..., False), so it was hardcoded False on hardware -- verified
                # across every recorded transcript: 0 occurrences of stale=True out of
                # 4896. Meaningful Pi staleness signals are FEATURE_PTS_FRESH and the
                # CBF_CORNERS_STALE / _ABORT coast-streak counters, not this.
                print(f"  t={now - start_time:.1f}s alt={alt:.2f}m "
                      f"visible={self.controller.checkTargetVisibility()}")

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
        # IMG_RECORD=1) could never be aligned to which estimator
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
