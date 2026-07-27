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

# Diagnosed 2026-07-26: PlanarFeatureMap SHADOW mode costs ~9-13ms/frame
# (img_data.py's own [TIMING] stage "1b_planar_map_shadow") but on the Pi it
# is PURELY diagnostic - self._planar_map's output only feeds its own
# logging (Planar Map Center/Confidence) and its own loop_closure_correct
# self-update, never getOptFlowAngVel/getImgFeatureParam/the fusion EKF (see
# reference_project_skills-style note: Pi's port never got Gazebo's
# _planar_map_primary ACTIVE rescue/override consumer, only the shadow
# logger). Confirmed this by reading img_data.py directly - no downstream
# consumer of the corner/flow pipeline reads it. Disabling it here changes
# NOTHING about the calibration-relevant Feature Params/Opt Flow Ang Vel
# signals, and reclaims a real chunk of the ~19Hz-on-battery loop budget
# found investigating the mount-rotation check's low sample counts. Only
# disabled for CALIBRATION RECORDING specifically (this script) - left ON
# by default elsewhere (e.g. hardware_landing.py) where the shadow log may
# still be worth collecting during real flight tests.
os.environ.setdefault("PLANAR_MAP_SHADOW", "0")

# CORRECTED 2026-07-26: disabling PLANAR_MAP_SHADOW alone does NOT stop the
# planar-map computation - img_data.py's guard is
# `if (self._planar_map_shadow or self._planar_map_primary) and ...`, and
# self._planar_map_primary (env PLASMC_PLANAR_MAP_PRIMARY) defaults ON
# (added 2026-07-25, mirrors Gazebo's PLASMC_PLANAR_MAP_PRIMARY). This one
# is NOT purely diagnostic like shadow mode - it's an ACTIVE RESCUE/OVERRIDE
# consumer that can substitute a map-PREDICTED corner position for the real
# ArUco decode when detection struggles (see "S Estimator Tag" ==
# 'planar_map_rescue'/'lstsq+klt+override' in recorded Img_Data.npy - already
# confirmed firing on real GOOD-verdict calibration takes, e.g. 19 rescue
# samples in one run). Same violation-of-intent as leaving FLOW_DH_MAX/
# FLOW_DS_MAX/FLOW_LOOM_DECOUPLE on would be: this script's whole purpose is
# recording TRUE RAW flow (see the module's own top-of-file print/comment),
# and a map-predicted substitute is not raw data regardless of how well the
# map itself performs. Force off here for BOTH reasons (frame-rate AND data
# integrity) - do not treat this as merely a performance knob.
os.environ.setdefault("PLASMC_PLANAR_MAP_PRIMARY", "0")

from flight_controller import FC
from img_geometry import CALIB_CX, CALIB_CY   # principal point, for the run-validity framing check
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
CAPTURE_RATE = 60 # Capture Rate = {90, 120, 200}
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
            gt_data = {"Start Time": start_time, "Time": t_c, "GT Pose Stamp": gt_pose_stamp, "Img Time Stamp": img_time_stamp, "Start Pose": start_pose, "UAV Pose": UAV_pose, "Target Pose": target_pose, "Opt Flow Ang Vel": opt_flow_ang_vel, "Img Feature Params": img_feature_param, "Command": cmd}
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
            # is ~0.28 m and the camera is narrow (fx=1020, HFOV 34.8 deg), so
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
            print("          This camera is narrow (HFOV ~35 deg): at 1.0/1.5/2.0 m a")
            print("          0.28 m marker leaves only +/-0.10/0.21/0.33 m of lateral")
            print("          room. Raise the altitude, use a smaller marker, or reduce")
            print("          the lateral excursion -- the >=0.40 m span asked for above")
            print("          is NOT geometrically achievable at low altitude here.")
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
    res = 'n'
    # res = input("Do you want to record? (y/n)")

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