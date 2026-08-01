# **************************************************************************
# Changed class and node name used in gz_subscriber
# Added time node for clock synchronization with simulator world
# Used simulator time instead of system time
# **************************************************************************
import os
import asyncio
import time
import traceback
import numpy as np

# A sensor calibration MUST be derived from the true raw sensor output, not
# a control-oriented, spike-rejected version of it. img_data.py's checkposts
# (FLOW_DH_MAX hold-on-spike on h_x/h_y, FLOW_DS_MAX hold-on-spike on
# xc/yc) and the loom override (FLOW_LOOM_DECOUPLE, which REPLACES h_z with
# a moment-based estimator AND has its own nested LOOM_DLNM_MAX checkpost)
# are all live during ANY normal run - including calibration recording,
# since this script runs the full img_data.py pipeline, not a stripped
# capture-only mode. Left enabled, they silently corrupt the logged "raw"
# flow with frozen/held values (confirmed 2026-07-10: 33-47% of consecutive
# h_x/h_y/h_z samples were exact duplicates - stale holds, not fresh
# measurements - which is why an earlier derive_pi_cal.py attempt against
# that data got ~zero correlation with ground truth; alpha and ring flow,
# which pass through neither checkpost, DID show real correlation,
# confirming this was checkpost corruption specifically, not a broken
# frame transform). Setting FLOW_LOOM_DECOUPLE=0 also implicitly disables
# LOOM_DLNM_MAX, since that checkpost lives entirely inside the
# `if self._loom_decouple:` block. This script's only purpose is
# calibration recording, so all of these should ALWAYS be off here.
os.environ.setdefault("FLOW_DH_MAX", "0")
os.environ.setdefault("FLOW_DS_MAX", "0")
os.environ.setdefault("FLOW_LOOM_DECOUPLE", "0")

# Diagnosed 2026-07-22: vigorous hand-sweep calibration runs were losing
# 60-80% of each recording to multi-second ArUco detection dropouts caused
# by motion blur under auto-exposure (marker confirmed centered, not
# off-frame, right before every dropout - see imgstreamer.py's
# CAM_MANUAL_EXPOSURE comment for the full diagnosis). Default ON here since
# this script's only purpose IS vigorous sweeps; override CAM_EXPOSURE_US /
# CAM_ANALOGUE_GAIN per lighting if the default under/over-exposes.
os.environ.setdefault("CAM_MANUAL_EXPOSURE", "1")

# MATCHED TO hardware_landing.py 2026-08-01 (PINNED here 2026-08-01): that
# script switched to CAM_EXPOSURE_US=20000/CAM_AUTO_GAIN=1 for real low-light
# flights (validated: 0% decode at the old 3000us/auto-exposure default in
# poor indoor lighting -> 43-73% at 20000us with gain closing the loop -
# see FLIGHT_TEST_ANALYSIS_PROCEDURE.md catalog #12/#13). A calibration
# recorded at the OLD exposure/rate is a train/run mismatch against flights
# now made at the NEW ones - same class of bug as the ARUCO_ROI_MARGIN_PX
# pin above, just on the camera side instead of the detector side. If
# hardware_landing.py's own defaults are ever retuned again, update these to
# match rather than letting them drift apart silently.
os.environ.setdefault("CAM_EXPOSURE_US", "20000")
os.environ.setdefault("CAM_AUTO_GAIN", "1")

# PINNED 2026-07-31: img_data.py's own ARUCO_ROI_MARGIN_PX default has changed
# TWICE in this project's history (200 -> 80 -> 40, the last a units-only
# rescale, "not a behavior change" per img_data.py's own comment) purely from
# unrelated code edits, with no calibration-specific reasoning behind either
# change. That silent drift is exactly what produced the Jul 28 morning-vs-
# afternoon fit-quality gap this session traced (bias 0.057 -> 0.093,
# corresponding to a ROI_MARGIN_PX 200 -> 80 change mid-day) - it was
# initially mistaken for a mocap lever-arm shift before Img_Params.txt showed
# the real cause. Pinning it explicitly here (same pattern as FLOW_DH_MAX
# etc. above) means calibration recordings stay on ONE value regardless of
# whatever img_data.py's own default becomes for flight use - if that value
# is deliberately changed, update this pin too so calibration keeps testing
# the config that's actually deployed. Value matches img_data.py's current
# default (40) as of this pin - verify against a fresh Img_Params.txt if
# img_data.py's default is ever touched again.
os.environ.setdefault("ARUCO_ROI_MARGIN_PX", "40")

# PlanarFeatureMap: LEFT ON during calibration (2026-07-27). This REVERSES the
# 2026-07-26 decision to force PLANAR_MAP_SHADOW=0 / PLASMC_PLANAR_MAP_PRIMARY=0
# here. Keeping the history because both the original reasoning and why it was
# wrong are load-bearing:
#
# The 2026-07-26 rationale was (a) frame rate - shadow costs ~9-13ms/frame
# ([TIMING] stage "1b_planar_map_shadow") against a ~19Hz-on-battery loop, and
# (b) data integrity - _planar_map_primary is an ACTIVE RESCUE/OVERRIDE that can
# substitute a map-PREDICTED corner for a real ArUco decode ("S Estimator Tag"
# == 'planar_map_rescue'/'lstsq+klt+override'), and "a map-predicted substitute
# is not raw data". (b) sounds right but does not survive the measurements:
#
#   1. The override NEVER touches the flow path. 'rescue'/'override' is 0.0% of
#      the "Opt Flow Estimator Tag" in EVERY recording, including map-ON ones -
#      it only ever fires on the centroid (S) path. So _sensor_cal_hw, the
#      primary cal target, was never at contamination risk.
#   2. The loss is BIASED, not uniform. On the map-ON (Jul 25) runs, map-sourced
#      samples sit at a bbox-to-frame-edge margin of -10..51px (negative = the
#      marker is already clipping) vs 38-92px for pure-ArUco samples, at
#      comparable per-frame motion. The map fires exactly at the FoV limit -
#      i.e. at the FAR ENDS of each excitation sweep, where the calibration
#      signal is LARGEST. Disabling it truncates the excitation tails and leaves
#      the low-excitation middle: the worst possible loss for a regression, and
#      it manufactures the "near-hover takes are noise-dominated" failure as a
#      configuration artifact. Measured: S-path coast 25% (map ON, best run) vs
#      62-95% (map OFF).
#   3. Gazebo already settled this. Its record_output_calibration.py hit the
#      IDENTICAL problem on 2026-07-17 (CONTROLLER_READY not propagated -> the
#      map was never constructed -> "NEVER RAN during output calibration") and
#      named it "a TRAIN/RUN MISMATCH ... the cal was being fit against a
#      map-dead signal the runtime never produces". Gazebo's fix was to make the
#      map RUN during calibration; its recorder never disables it.
#
# The circularity concern is handled at DERIVE time, not RECORD time - which is
# also what Gazebo does (derive_board_cal.py filters only != 'coast', and
# separately derives a map-sourced cal from co-sampled 'Centroid Map Raw').
# Set PLANAR_MAP_SHADOW=0 / PLASMC_PLANAR_MAP_PRIMARY=0 in the environment to
# A/B the frame-rate cost, which remains real and is a SEPARATE tradeoff from
# data integrity - argue it on its own, not by appeal to (b) above.

from flight_controller import FC
from img_geometry import CALIB_CX, CALIB_CY, fx   # for the run-validity framing/marker-size check
import img_data as ID
# from numerical_methods import RK5
# from controller import Controller
from mavsdk.telemetry import LandedState
from mocaptools import QTMWrapper

print(f"[output_calibration] recording TRUE raw flow: "
      f"FLOW_DH_MAX={os.environ['FLOW_DH_MAX']} FLOW_DS_MAX={os.environ['FLOW_DS_MAX']} "
      f"FLOW_LOOM_DECOUPLE={os.environ['FLOW_LOOM_DECOUPLE']} "
      f"(checkpost/loom-override disabled for calibration - see comment above)")

# # Setup variables
QTM_IP = '192.168.0.111'

# Run-validity decode-yield thresholds (2026-07-27). A calibration recording is
# only as good as the fraction of frames where the marker was actually decoded;
# see _print_run_validity for the full rationale and the Gazebo comparison.
# 25%: the 2026-07-26 set ran 62.6-94.8% coast and produced a cal whose
# per-run coefficients varied by ~their own magnitude. Gazebo's runs sit at
# 3.3% (8 of 10 at 0.0%), so 25% is already a generous hardware allowance.
COAST_MAX_PCT = float(os.environ.get("CAL_COAST_MAX_PCT", "25.0"))
# 2.0s: derive_pi_cal.py excludes GT-grid samples interpolated across a >=1.0s
# marker-loss gap, so a blackout past that threshold costs more than itself.
BLACKOUT_MAX_S = float(os.environ.get("CAL_BLACKOUT_MAX_S", "2.0"))

SLEEP_TIME = 1/200   # match hardware_landing.py:45 (was 1/30 - 6.7x slower than the
                      # operational regime this script is calibrating for; same
                      # mismatch PX4_Gazebo's record_output_calibration.py already
                      # fixed for the same reason - see its own SLEEP_TIME comment)

# Companion Computer Details:
# MATCHED TO hardware_landing.py 2026-08-01: this used to be a hardcoded 60,
# passed EXPLICITLY to IMG_PROCESSOR(capRate=...) below - so it silently
# overrode img_data.py's own CAPTURE_RATE_HZ env-var default (60 there too,
# historically) regardless of what that env var said, which is why simply
# setting CAPTURE_RATE_HZ would NOT have picked up hardware_landing.py's
# 2026-08-01 switch to 30Hz (the real achieved rate on this hardware
# regardless of what's requested - see FLIGHT_TEST_ANALYSIS_PROCEDURE.md
# catalog #12). Now reads the same env var so both scripts move together.
CAPTURE_RATE = int(os.environ.get("CAPTURE_RATE_HZ", "30"))
# RESOLUTION = (1280, 960)
RESOLUTION = (640, 480)  # MUST match the calibrated resolution (fx/fy/center in
# img_data.py are ONLY measured at 640x480 - a mismatch silently uses the wrong
# focal length in the core geometry math, since self.focal is a resolution-
# unaware module-level constant, unlike self.center which has a fallback guard).
# RESOLUTION = (320, 240)
REF_RAD_OPT_FLOW = -0.30
DES_IMG_FEATURE_PARAM = np.array([0.0, 0.0, 1.0, 0.0])

# Flags
CONTROLLER_READY = False

# Global Logging variable
image_data = None
telemetry_data = None
controller_data = None
controller_params = None
gt_data = None
img_params = None

async def main(record = 'n'):
    # Global Logging variable
    global image_data, telemetry_data, controller_data, controller_params, gt_data, img_params, CONTROLLER_READY

    # Initialize variables to None to avoid NameError in the `finally` block
    img_node = None
    # EC_node = None
    mocap_node = None
    FC_node = None

    UAV_pose = []
    target_pose = []
    opt_flow_ang_vel = []
    img_feature_param = []
    raw_opt_flow_ang_vel = []      # pre-_sensor_cal_hw, for derive_pi_cal (2026-07-27)
    raw_img_feature_param = []     # pre-_sensor_cal_s
    raw_ring_flow_ang_vel = []     # pre-_sensor_cal_ring
    centroid_map_raw = []          # map centroid, V-frame pre-cal (nan when not fired)
    alpha_map_raw = []             # map alpha, V-frame pre-cal (nan when not fired)
    start_pose = None
    t_c = []
    gt_pose_stamp = []
    img_time_stamp = []
    cmd = []

    try:
        FC_node = FC()
        await FC_node.start()

        mocap_node = QTMWrapper(QTM_IP)

        start_time = time.perf_counter()
        while not FC_node.has_quat():
            if (time.perf_counter() - start_time) > 20:
                raise Exception("Unable to get data from Flight Controller.")
            await asyncio.sleep(0.05)
       
        # Untimed loop hangs forever if the QTM project's rigid-body names don't
        # match the hardcoded body_keys=["UAV","target"] (mocaptools.py) - the
        # mismatched key is silently omitted from _body_index and getPose()["UAV"]
        # stays None indefinitely. Bounded so a naming mismatch fails loudly.
        _pose_wait_t0 = time.perf_counter()
        while start_pose is None:
            start_pose = mocap_node.getPose()["UAV"]
            if (time.perf_counter() - _pose_wait_t0) > 20:
                raise Exception("Unable to get UAV pose from QTM - check that the "
                                 "QTM project's rigid body is named 'UAV' "
                                 "(mocaptools.py body_keys).")
            await asyncio.sleep(0.05)

        img_node = ID.IMG_PROCESSOR(capRate=CAPTURE_RATE, resolution=RESOLUTION, controller=FC_node) # start the thread for onboard camera flow streaming

        if record != 'n':
            img_node.RECORD = True
            print("Starting with recording...")

        else:
            print("Starting without recording...!")

        # await FC_node.arm_and_takeoff()

        # await FC_node.send_position_ned(0.0, 0.0, FC_node.getPosNED().down_m, yaw_deg_0 + 15*0)
        
        # for val in cmd_profile: 
        _last_metrics_print = 0.0
        while img_node.is_alive() and mocap_node.is_alive():
        # while img_node.is_alive() and mocap_node.is_alive() and not FC_node.LANDED:
            if CONTROLLER_READY:
                t_c.append(time.perf_counter() - start_time)

            else:
                start_time = time.perf_counter()
                CONTROLLER_READY = True
                t_c = [0.0]

            # pos_cmd = np.array([val, 0.0, FC_node.getPosNED().down_m, yaw_deg_0 + 15*0])

            UAV_pose.append(mocap_node.getPose()["UAV"])
            target_pose.append(mocap_node.getPose()["target"])
            opt_flow_ang_vel.append(img_node.getOptFlowAngVel())
            img_feature_param.append(img_node.getImgFeatureParam())
            # RAW (pre-_sensor_cal) counterparts, co-sampled on the SAME loop
            # tick as the mocap pose above (2026-07-27). The calibrated arrays
            # cannot be used to FIT a calibration, so derive_pi_cal.py was
            # re-loading Img_Data.npy and np.interp()-ing it onto the GT clock
            # instead. That interpolation measurably hurts: validating GT s
            # against the image gives corr 0.0-0.52 through the interpolated
            # path vs 0.29-0.83 using exactly-paired co-sampled rows on the
            # same recordings. Mirrors what Gazebo already co-samples
            # (getRawOptFlowAngVel/getRawRingFlowAngVel) "so the derive tools
            # can fit". Ring raw is included so ring-cal gets the same
            # zero-interpolation treatment.
            raw_opt_flow_ang_vel.append(img_node.getRawOptFlowAngVel())
            raw_img_feature_param.append(img_node.getRawImgFeatureParam())
            raw_ring_flow_ang_vel.append(img_node.getRawRingFlowAngVel())
            # Map's INDEPENDENT centroid/alpha (V-frame, pre-cal), nan when the
            # map didn't fire. Co-sampled on this same tick so a map cal can be
            # fitted with zero alignment, exactly as Gazebo does with its own
            # 'Centroid Map Raw'/'Alpha Map Raw'. Lets the map path be
            # calibrated and cross-checked on its own terms instead of only
            # reaching the fit as rescue/override-tagged rows in the ordinary
            # feature stream.
            centroid_map_raw.append(img_node.getRawCentroidMapFeature())
            alpha_map_raw.append(img_node.getRawAlphaMapFeature())
            # Real per-iteration capture stamps for the two subsystems this
            # loop is polling, DISTINCT from t_c (this loop's own tick time).
            # t_c only tells you when THIS loop ran - it does not tell you
            # when QTM actually produced the pose above, or when the flow
            # thread actually computed the flow value above, both of which
            # run on their own independent threads/rates and can be stale by
            # up to one of their own inter-sample gaps relative to this poll.
            # Same fix as imgstreamer.py's Capture Stamp on the image side -
            # log every loop's OWN real timestamp rather than assuming one
            # shared relative clock (t_c) is valid for all of them.
            gt_pose_stamp.append(
                (mocap_node.getPoseStamp() - start_time) if mocap_node.getPoseStamp() is not None else None)
            img_time_stamp.append(
                (img_node.getLastTimeStamp() - start_time) if img_node.getLastTimeStamp() is not None else None)
            # cmd.append(pos_cmd)
            # print(f"MoCap | Int_Ctrl = {UAV_pose[-1]} | {FC_node.getPosNED()}")
            # print(f"UAV | Target = {UAV_pose[-1]} | {target_pose[-1]}")

            await asyncio.sleep(SLEEP_TIME)

            if img_node.FEATURE_IS_VISIBLE:
                _now = time.perf_counter()
                if _now - _last_metrics_print >= 1.0:   # throttled to 1 Hz — this loop
                    _last_metrics_print = _now          # runs at 30 Hz and shares the
                    m = img_node.metrics()               # GIL with the flow thread; a
                    print({k: round(v, 2) for k, v in m.items()})  # full-precision-float
                                                          # dict print every tick was
                                                          # unnecessary GIL/stdout cost

            # if time.perf_counter() - start_time > 600.0:
            #     # print(f"Commanded Profile | Position: {pos_cmd} | {UAV_pose[-1]}")
            #     # restart_time = time.perf_counter()
            #     break
        
        # await FC_node.vehicle.action.land()

        img_node.close()
        await FC_node.close()

    except asyncio.CancelledError:
        # await FC_node.vehicle.action.land()
        # Usually a normal Ctrl+C shutdown: asyncio.run() cancels the running
        # task, which raises CancelledError here (not KeyboardInterrupt
        # directly - that only reaches the outer except KeyboardInterrupt
        # around asyncio.run() if the interrupt lands OUTSIDE the running
        # loop). BUT CancelledError can also come from a timeout or an
        # explicit task.cancel() elsewhere - so still log the full traceback
        # (to a file, not the console - it's noise on every normal Ctrl+C)
        # so a non-Ctrl+C cancellation is diagnosable after the fact.
        print("Stopped (Ctrl+C, or a task was cancelled) - see cancelled_error.log for details\n")
        with open("cancelled_error.log", "a") as _f:
            _f.write(f"\n=== {time.ctime()} ===\n")
            _f.write(traceback.format_exc())

    except RuntimeError as e:
        print(f"RuntimeError: Main Thread: {e}\n")

    except SyntaxError:
        print("SyntaxError: Main Thread\n")

    except Exception as e:
        print(f"Unexpected error: Main Thread: {e}\n")

    finally:
        # Save data
        if img_node is not None and mocap_node is not None and FC_node is not None:
            image_data = img_node.getLogData()
            telemetry_data = FC_node.getLogData()
            # controller_data = EC_node.getLogData()
            # controller_params = EC_node.getParams()
            gt_data = {"Start Time": start_time, "Time": t_c, "GT Pose Stamp": gt_pose_stamp, "Img Time Stamp": img_time_stamp, "Start Pose": start_pose, "UAV Pose": UAV_pose, "Target Pose": target_pose, "Opt Flow Ang Vel": opt_flow_ang_vel, "Img Feature Params": img_feature_param, "Raw Opt Flow Ang Vel": raw_opt_flow_ang_vel, "Raw Img Feature Params": raw_img_feature_param, "Raw Ring Flow Ang Vel": raw_ring_flow_ang_vel, "Centroid Map Raw": centroid_map_raw, "Alpha Map Raw": alpha_map_raw, "Command": cmd}
            img_params = img_node.getParams()

        # Close img_node thread
        if img_node is not None and img_node.is_alive():
            img_node.close()
            img_node.join()

        # Close mocap_node thread
        if mocap_node is not None and mocap_node.is_alive():
            mocap_node.close()
            mocap_node.join()

        # Close flight controller
        if FC_node:
            await FC_node.close()

def _print_run_validity(image_data, gt_data):
    """Quick quality gauge for a hand-move output-cal take, printed before the
    save prompt so a low-excitation dud can be discarded before moving on.
    Thresholds mirror the output-cal analysis: near-hover / single-axis takes are
    noise-dominated (raw flow SNR is poor), so each axis must actually be swept."""
    print("\n" + "=" * 52)
    print(" RUN VALIDITY (hand-move output-cal take)")
    print("=" * 52)
    # VISIBILITY CHECK (2026-07-31): print the detection-config env vars this
    # run actually used, pinned near the top of this file specifically so they
    # can't silently drift between calibration sessions the way
    # ARUCO_ROI_MARGIN_PX did (200 -> 80 mid-day on 2026-07-28, initially
    # mistaken for a mocap lever-arm shift - see check_lever_arm.py / this
    # session's history). A mismatch here doesn't necessarily mean anything is
    # wrong (an explicit env override for a deliberate A/B test is fine) but
    # it should never be SILENT.
    _pinned = {"ARUCO_ROI_MARGIN_PX": "40", "CAM_MANUAL_EXPOSURE": "1",
               "FLOW_DH_MAX": "0", "FLOW_DS_MAX": "0", "FLOW_LOOM_DECOUPLE": "0",
               # ADDED 2026-08-01, matched to hardware_landing.py's low-light switch
               "CAM_EXPOSURE_US": "20000", "CAM_AUTO_GAIN": "1",
               "CAPTURE_RATE_HZ": "30"}
    print(" -- detection/pipeline config this run used --")
    for _k, _default in _pinned.items():
        _v = os.environ.get(_k)
        _flag = "" if _v == _default else f"  [DIFFERS FROM PINNED DEFAULT {_default}]"
        print(f" {_k:<22}: {_v}{_flag}")
    try:
        flowN = len(image_data.get("Opt Flow Ang Vel", [])) if image_data else 0
        featN = len(image_data.get("Feature Params", [])) if image_data else 0
    except Exception:
        flowN = featN = 0

    # DECODE YIELD (added 2026-07-27). flowN above is a raw ROW count, and
    # img_data.py logs a row on EVERY call - including 'coast' frames, where no
    # common marker was decoded in both frames of the pair and a literal
    # np.zeros(6) is appended. So flowN says nothing about whether the camera
    # actually SAW anything, and the old gate happily certified a run that was
    # 94.8% coast (67 real samples of 1290) as "GOOD excitation on all axes".
    # Audit vs the Gazebo pipeline: Pi 75.7% coast / 2,128 usable samples across
    # a whole 7-run set, vs Gazebo 3.3% / 94,333 (8 of 10 Gazebo runs are 0.0%).
    # A cal fitted on the remnant cannot be repeatable no matter how the fit is
    # written - see project_pi_output_cal_methodology_audit_2026_07_27.
    realN = coastN = 0
    coast_pct = 0.0
    blackout_s = 0.0
    onset_r = float('nan')     # marker framing at the moment tracking died
    try:
        tags = [str(t) for t in image_data.get("Opt Flow Estimator Tag", [])]
        if tags:
            coastN = sum(1 for t in tags if "coast" in t)
            realN = len(tags) - coastN
            coast_pct = 100.0 * coastN / len(tags)
            # longest CONTIGUOUS blackout, converted to seconds - a single long
            # dropout is far worse than the same count scattered thinly, because
            # derive_pi_cal's >=1s gap guard then discards the interpolated
            # neighbourhood around it too.
            longest = cur = 0
            for t in tags:
                if "coast" in t:
                    cur += 1
                    longest = max(longest, cur)
                else:
                    cur = 0
            _ti = np.asarray(image_data.get("Time", []), dtype=float)
            dt = float(np.median(np.diff(_ti))) if len(_ti) > 2 else 1.0 / 30.0
            blackout_s = longest * dt

            # How much room did the marker have when tracking died?
            #
            # CORRECTED 2026-07-27: an earlier version measured the CENTROID's
            # distance from the principal point and concluded the marker was
            # "well inside the frame". That was the wrong quantity. This marker
            # is ~0.28 m and the camera is narrow (fx=1027, HFOV 34.7 deg), so
            # it spans 30-60% of the frame height at 0.8-2.0 m: the centroid can
            # sit comfortably inside while the marker's own CORNERS are already
            # against the boundary. ArUco needs the whole quad PLUS a quiet
            # zone, so it dies at that point. Measured at blackout onset: bbox
            # margin median 6-10 px (vs 25-88 px while tracking), with 77-92% of
            # onsets under 20 px. Measure the EXTENT, not the centre.
            _P = np.asarray(image_data.get("Image Feature Pts", []), dtype=float)
            if _P.ndim == 4 and len(_P) == len(tags):
                _q = _P[:, 0]
                _W, _H = 2.0 * CALIB_CX, 2.0 * CALIB_CY
                _margin = np.minimum.reduce([
                    _q[:, :, 0].min(1), _W - _q[:, :, 0].max(1),
                    _q[:, :, 1].min(1), _H - _q[:, :, 1].max(1)])
                _c = np.array([t == "coast" for t in tags])
                _on = np.where(_c[1:] & ~_c[:-1])[0]
                if len(_on):
                    onset_r = float(np.median(_margin[_on]))   # px to frame edge
    except Exception:
        pass
    xs = ys = zs = yaws = np.array([])
    tz = np.array([0.0]); dur = 0.0
    try:
        up = [p for p in gt_data.get("UAV Pose", []) if p is not None]
        tp = [p for p in gt_data.get("Target Pose", []) if p is not None]
        xs = np.array([p.x for p in up]); ys = np.array([p.y for p in up]); zs = np.array([p.z for p in up])
        if len(up):
            # Drop NaN (QTM dropouts) BEFORE unwrap - see analyze_output_calibration.py
            # for why: np.unwrap's cumulative correction is corrupted by a
            # single NaN, silently collapsing the reported span to near-zero.
            _raw_yaws = np.array([p.yaw for p in up])
            yaws = np.unwrap(np.deg2rad(_raw_yaws[~np.isnan(_raw_yaws)]))
        if len(tp):
            tz = np.array([p.z for p in tp])
        _t = gt_data.get("Time", [0.0])
        dur = float(_t[-1]) if len(_t) else 0.0
    except Exception:
        pass
    def span(a):
        return float(np.nanmax(a) - np.nanmin(a)) if len(a) else 0.0
    xsp, ysp, zsp = span(xs), span(ys), span(zs)
    yawsp = np.rad2deg(span(yaws)) if len(yaws) else 0.0
    Zlo = float(np.nanmin(zs - np.nanmedian(tz))) if len(zs) else 0.0
    Zhi = float(np.nanmax(zs - np.nanmedian(tz))) if len(zs) else 0.0
    def tag(v, lo):
        return "OK " if v >= lo else "LOW"
    def tag_hi(v, hi):                     # pass when v is at or BELOW hi
        return "OK " if v <= hi else "BAD"
    print(" duration      : %6.1f s" % dur)
    print(" -- decode yield (did the camera actually SEE the marker?) --")
    print(" decoded flow  : %6d   %s (need >=300 REAL, not row count)"
          % (realN, tag(realN, 300)))
    print(" coast (lost)  : %5.1f%%   %s (need <=%.0f%%)"
          % (coast_pct, tag_hi(coast_pct, COAST_MAX_PCT), COAST_MAX_PCT))
    print(" max blackout  : %6.1f s %s (need <=%.1f)"
          % (blackout_s, tag_hi(blackout_s, BLACKOUT_MAX_S), BLACKOUT_MAX_S))
    print(" logged rows   : %6d   (flow) / %6d (feature)  [incl. coast]"
          % (flowN, featN))
    print(" -- excitation (did the vehicle actually MOVE?) --")
    print(" X span        : %6.2f m %s (need >=0.40)" % (xsp, tag(xsp, 0.40)))
    print(" Y span        : %6.2f m %s (need >=0.40)" % (ysp, tag(ysp, 0.40)))
    print(" Z span        : %6.2f m %s (need >=0.40)" % (zsp, tag(zsp, 0.40)))
    print(" yaw span      : %6.1f d %s (need >=20)" % (yawsp, tag(yawsp, 20.0)))
    print(" height range  : %5.2f .. %5.2f m" % (Zlo, Zhi))

    # Marker physical size, measured from THIS run's own tracked frames
    # (side_px = fx * L / z), not assumed. The 2026-07-27 advisory text hardcoded
    # "0.28 m" from one specific marker; that stops being true the moment the
    # marker is swapped (e.g. to fix decode yield) or the nested small marker is
    # what's tracked, and would silently give wrong altitude/margin advice.
    # Only uses the BIG-marker mode (side > 80 px raw), matching the threshold
    # used throughout this session's marker-size/decode-yield analysis - the
    # nested small marker's own apparent size is a different, much smaller L.
    marker_L_m = float('nan')
    try:
        _P2 = np.asarray(image_data.get("Image Feature Pts", []), dtype=float)
        if _P2.ndim == 4 and len(_P2) == len(tags):
            _q2 = _P2[:, 0]
            _side = np.linalg.norm(_q2 - np.roll(_q2, -1, axis=1), axis=2).mean(1)
            _tr = np.array([t != "coast" for t in tags]) & (_side > 80)
            if _tr.sum() > 20 and len(zs):
                # altitude at tracked samples: reuse the same height-range
                # convention as Zlo/Zhi (UAV z minus target z), nearest-index
                # matched onto the flow array's own length.
                _hgt = (zs - np.nanmedian(tz))
                _idx = np.clip(np.linspace(0, len(_hgt) - 1, len(tags)).astype(int),
                               0, len(_hgt) - 1)
                _L = _side[_tr] * np.abs(_hgt[_idx][_tr]) / fx
                _L = _L[np.isfinite(_L) & (_L > 0.02) & (_L < 1.0)]
                if len(_L) > 10:
                    marker_L_m = float(np.median(_L))
    except Exception:
        pass

    weak = [n for n, v, lo in [("X", xsp, 0.40), ("Y", ysp, 0.40),
                               ("Z", zsp, 0.40), ("yaw", yawsp, 20.0)] if v < lo]
    lost = [n for n, v, hi in [("coast", coast_pct, COAST_MAX_PCT),
                               ("blackout", blackout_s, BLACKOUT_MAX_S)] if v > hi]
    if realN < 300:
        lost.append("decoded<300")

    # Reported SEPARATELY and never merged, because the two failures have
    # OPPOSITE fixes. Low excitation -> move more. Low decode yield -> move
    # LESS (slower, smaller, keep the marker framed). The old single "WEAK -
    # low excitation" verdict pushed the operator to sweep harder, which threw
    # the marker out of FoV and made the yield worse; measured corr(hand-speed,
    # coast%) = +0.55. Gazebo hit the same wall and cut its sweep amplitude
    # 0.7 -> 0.35 m for exactly this reason.
    if lost:
        print(" VERDICT: UNUSABLE - poor decode yield: %s" % ", ".join(lost))
        if np.isfinite(onset_r) and onset_r < 25.0:
            print("          Marker bbox was only %.0f px from the frame edge when"
                  % onset_r)
            print("          tracking died -> it ran out of FRAME, not of focus.")
            if np.isfinite(marker_L_m):
                print("          This camera is narrow (HFOV ~35 deg). This run's own"
                      " tracked")
                print("          marker measures ~%.2f m; at 1.0/1.5/2.0 m that leaves"
                      % marker_L_m)
                for _z in (1.0, 1.5, 2.0):
                    _room = max(0.0, (2.0 * CALIB_CY - fx * marker_L_m / _z) * _z / fx / 2.0)
                    print("            z=%.1fm: +/-%.2f m of lateral room" % (_z, _room))
            else:
                print("          This camera is narrow (HFOV ~35 deg): at 1.0/1.5/2.0 m a")
                print("          0.28 m marker leaves only +/-0.10/0.21/0.33 m of lateral")
                print("          room (reference figure - this run's own marker size")
                print("          could not be measured).")
            print("          Raise the altitude, use a smaller marker, or reduce the")
            print("          lateral excursion -- the >=0.40 m span asked for above is")
            print("          NOT geometrically achievable at low altitude with this marker.")
        elif np.isfinite(onset_r):
            print("          Marker still had %.0f px of frame margin when tracking"
                  % onset_r)
            print("          died -> not a framing limit; check the detection path")
            print("          (re-acquisition / ROI recovery / marker scale switching).")
        else:
            print("          The marker was not reliably decoded; check framing first,")
            print("          then the detection path.")
    elif weak:
        print(" VERDICT: WEAK - low excitation in: %s" % ", ".join(weak))
        print("          Each axis still needs a real sweep during its own phase")
        print("          (phased single-axis takes are correct - it is a whole-run")
        print("          near-hover that is noise-dominated).")
    else:
        print(" VERDICT: GOOD - excitation and decode yield both pass")
    print("=" * 52 + "\n")


if __name__ == "__main__":
    # FIXED 2026-07-31: hardcoded to 'n' with the prompt commented out - RECORD
    # was silently NEVER True regardless of intent, which is why zero runs in
    # the entire calibration archive have ever produced a video despite
    # img_data.py's video-write path being correct (confirmed by grepping every
    # existing calibration run: no video timestamp overlaps any Output/ run
    # dir). Restored to match the existing interactive save-prompt below it.
    # Defaults to recording ON a bare Enter (RECORD is diagnostically useful
    # and cheap - see img_data.py:800-804's own reasoning) unless the user
    # explicitly types 'n'.
    res = input("Do you want to record? (Y/n) ").strip().lower()
    if res == '':
        res = 'y'

    try:
        asyncio.run(main(res))
    except KeyboardInterrupt:
        print("Clean exit on Ctrl+C")

    # Save data# Save data
    if CONTROLLER_READY:
        _print_run_validity(image_data, gt_data)
        x = input('Do you want to save the dataset? (y/n)')
        if x != 'n':
            # record timestamp
            timestamp = time.ctime().replace(':', '-')

            # Create a directory named based on timestamp
            # Saves under Output/ (2026-07-26) - matches the repo's own
            # Hardware/Test_Data/Calibration/Output/ convention; previously
            # saved one level up (bare Test_Data/Calibration/<timestamp>),
            # which put every output-cal run alongside Input/Camera and
            # needed a manual sort-and-move into Output/ after the fact.
            dir_name = f"Test_Data/Calibration/Output/{timestamp}"
            os.makedirs(dir_name)

            # Save files inside the folder 
            np.save(f'{dir_name}/Img_Data', image_data)
            np.save(f'{dir_name}/Telemetry_Data', telemetry_data)
            # np.save(f'{dir_name}/Control_Data', controller_data)
            # np.save(f'{dir_name}/Control_Params', controller_params)
            np.save(f'{dir_name}/Ground_Truth', gt_data)
            f = open(f'{dir_name}/Img_Params.txt', 'w')
            f.write(img_params)
            f.close()

            print("Saved to CSV!")

        else:
            print("Closed without saving!")