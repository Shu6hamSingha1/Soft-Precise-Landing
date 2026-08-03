# **************************************************************************
# Output-calibration recorder for the cross+stub marker (MARKER_TYPE=cross).
# Copied from record_output_calibration.py's phased-excitation harness (same
# maneuver, same GT capture, same routing conventions) but swaps
# ID.IMG_PROCESSOR (ArUco corner/ring pipeline) for CrossMarkerNode
# (src/cross_marker_perception.py) -- the cross marker computes its own h,w
# via its own image-Jacobian solve, so it has no ring/map/estimator-tag
# subsystems and no pre-existing raw/calibrated split beyond the identity
# cal added in CrossMarkerPerception for this purpose. See
# Memory/px4/project_cross_marker_pipeline_20260801.md for the pipeline's
# architecture and scope.
# **************************************************************************
import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src'))
import os
import asyncio
import time
import numpy as np
import rclpy
from ahrs import RAD2DEG

os.environ.setdefault("MARKER_TYPE", "cross")

from flight_controller import FC
from cross_marker_perception import CrossMarkerNode
from gz_subscriber import GZ_Subscriber, Pose_Node, Clock_Node

# Companion Computer Details -- MUST match cross_marker_hover_sanity.py /
# landing_test.py to keep the calibration regime identical to the
# operational regime.
CAPTURE_RATE = 60
RESOLUTION = (640, 480)
SLEEP_TIME = 1/200

# Calibration sweep amplitudes -- same values/rationale as
# record_output_calibration.py (see that file's comments); kept identical so
# the two markers' cals are derived under the same excitation regime and are
# comparable.
CALIB_AMP_XY = 0.35
CALIB_AMP_Z  = float(os.environ.get("CALIB_AMP_Z", "1.2"))
CALIB_AMP_YAW_DEG = float(os.environ.get("CALIB_AMP_YAW_DEG", "30.0"))
CALIB_AMP_YAW_AGG_DEG = float(os.environ.get("CALIB_AMP_YAW_AGG_DEG", "90.0"))
CALIB_FREQ_YAW_AGG    = float(os.environ.get("CALIB_FREQ_YAW_AGG", "0.5"))
CALIB_YAW_AGG_S       = float(os.environ.get("CALIB_YAW_AGG_S", "12.0"))

CALIB_PHASE_S    = float(os.environ.get("CALIB_PHASE_S", "8.0"))
CALIB_SETTLE_S   = float(os.environ.get("CALIB_SETTLE_S", "1.0"))
CALIB_FREQ_HZ    = float(os.environ.get("CALIB_FREQ_HZ", "0.5"))
CALIB_SEND_TIMEOUT_S = 0.5
CALIB_ALT_DROP_BAIL_M = 2.0

CONTROLLER_READY = False

image_data = None
telemetry_data = None
gt_data = None
img_params = None


def yaw_from_quaternion(q):
    yaw = np.arctan2(
        2.0 * (q.w*q.z + q.x*q.y),
        1.0 - 2.0 * (q.y*q.y + q.z*q.z)
    )
    return RAD2DEG*yaw


async def main(record='n'):
    global image_data, telemetry_data, gt_data, img_params, CONTROLLER_READY

    img_node = None
    pose_subscriber = None
    time_subscriber = None
    FC_node = None

    UAV_pose = []
    target_pose = []
    opt_flow_ang_vel = []      # RAW (uncalibrated) [h;w] -- for derive_cross_marker_cal.py
    img_feature_param = []     # RAW (uncalibrated) [xc,yc,1,alpha]
    phase_log = []
    start_pose = None
    t_c = []
    cmd = []

    zero = lambda tau: 0.0
    sin_x   = lambda tau: CALIB_AMP_XY * np.sin(2*np.pi*CALIB_FREQ_HZ * tau)
    sin_y   = lambda tau: CALIB_AMP_XY * np.sin(2*np.pi*CALIB_FREQ_HZ * tau)
    sin_z   = lambda tau: CALIB_AMP_Z  * np.sin(2*np.pi*CALIB_FREQ_HZ * tau)
    sin_yaw = lambda tau: CALIB_AMP_YAW_DEG * np.sin(2*np.pi*CALIB_FREQ_HZ * tau)
    sin_yaw_agg = lambda tau: CALIB_AMP_YAW_AGG_DEG * np.sin(2*np.pi*CALIB_FREQ_YAW_AGG * tau)

    _selected = os.environ.get("CALIB_PHASES", "yaw,x,y,z,yawagg").split(",")
    _selected = [s.strip() for s in _selected if s.strip()]
    _phase_fns = {
        'yaw':    (zero, zero, zero, sin_yaw),
        'x':      (sin_x, zero, zero, zero),
        'y':      (zero, sin_y, zero, zero),
        'z':      (zero, zero, sin_z, zero),
        'yawagg': (zero, zero, zero, sin_yaw_agg),
    }
    _phase_dur = {'yawagg': CALIB_YAW_AGG_S}
    phase_script = [('settle', zero, zero, zero, zero, CALIB_SETTLE_S)]
    for _ph in _selected:
        if _ph not in _phase_fns:
            print(f"  [warn] unknown phase {_ph!r}; skipping"); continue
        fx_, fy_, fz_, fyaw_ = _phase_fns[_ph]
        phase_script.append((_ph, fx_, fy_, fz_, fyaw_, _phase_dur.get(_ph, CALIB_PHASE_S)))
        phase_script.append(('settle', zero, zero, zero, zero, CALIB_SETTLE_S))
    total_s = sum(p[5] for p in phase_script)
    print(f"[calib-cross] phased excitation: {len(phase_script)} phases, total {total_s:.1f}s")

    try:
        rclpy.init()
        pose_node = Pose_Node()
        pose_subscriber = GZ_Subscriber(pose_node)
        time_node = Clock_Node()
        time_subscriber = GZ_Subscriber(time_node)

        start_time = time.perf_counter()
        while time_node.perf_counter() is None:
            if (time.perf_counter() - start_time) > 20:
                raise Exception("Unable to get simulation time.")

        FC_node = FC(time_node)
        await FC_node.start()

        start_time = time.perf_counter()
        while not FC_node.has_quat():
            if (time.perf_counter() - start_time) > 20:
                raise Exception("Unable to get data from Flight Controller.")
            time.sleep(0.05)

        while start_pose is None:
            start_pose = pose_node.getPose().UAV

        yaw_deg_0 = yaw_from_quaternion(FC_node.getQuat())

        # Diagnostic frame dumps (2026-08-01 point-starvation/centroid-instability
        # investigation) -- opt-in via CROSS_DIAG_SAVE=1, written under this run's
        # own output dir so they travel with the recording. CALIB_OUT_DIR is set by
        # the launcher before this process starts, so it's known here.
        if os.environ.get("CROSS_DIAG_SAVE", "1") == "1":
            _out_dir = os.environ.get("CALIB_OUT_DIR") or os.environ.get(
                "CALIB_OUT_BASE", "/home/shubham/Soft-Precise-Landing/PX4_Gazebo/calibration_data/output_cross")
            os.environ["CROSS_DIAG_SAVE_DIR"] = os.path.join(_out_dir, "diag_frames")

        img_node = CrossMarkerNode(time_keeper=time_node, controller=FC_node)

        _takeoff_h = float(os.environ.get("CALIB_TAKEOFF_HEIGHT", "5.0"))
        await FC_node.arm_and_takeoff(_takeoff_h)

        print("Hovering 5s for PX4 state-machine to stabilize...")
        await asyncio.sleep(5.0)

        print("Establishing offboard with stationary setpoints (2s @ 30 Hz)...")
        for _ in range(60):
            yaw_deg_curr = yaw_from_quaternion(FC_node.getQuat())
            await FC_node.send_position_ned(0.0, 0.0,
                                            FC_node.getPosBody().z_m,
                                            yaw_deg_curr)
            await asyncio.sleep(SLEEP_TIME)

        print("Starting calibration sweep (phased excitation)...")
        takeoff_z = FC_node.getPosBody().z_m

        for ph_name, fn_x, fn_y, fn_z, fn_yaw, duration in phase_script:
            print(f"  [phase] {ph_name:6s}  duration={duration:.1f}s")
            phase_t0 = time_node.perf_counter()
            n_steps = int(duration / SLEEP_TIME)
            for k in range(n_steps):
                if not img_node.is_alive() and FC_node.LANDED:
                    break

                cur_z = FC_node.getPosBody().z_m
                if cur_z - takeoff_z > CALIB_ALT_DROP_BAIL_M:
                    print(f"  [bail] altitude dropped {cur_z - takeoff_z:+.2f} m below takeoff — "
                          f"PX4 likely failsafed; ending sweep early.")
                    break

                if CONTROLLER_READY:
                    t_c.append(time_node.perf_counter() - start_time)
                else:
                    start_time = time_node.perf_counter()
                    phase_t0 = start_time
                    CONTROLLER_READY = True
                    t_c = [0.0]
                    img_node.CONTROLLER_READY = True

                tau = time_node.perf_counter() - phase_t0
                z_cmd = takeoff_z + fn_z(tau)
                pos_cmd = np.array([
                    fn_x(tau),
                    fn_y(tau),
                    z_cmd,
                    yaw_deg_0 + fn_yaw(tau),
                ])

                try:
                    if img_node.FEATURE_IS_VISIBLE:
                        await asyncio.wait_for(
                            FC_node.send_position_ned(*pos_cmd),
                            timeout=CALIB_SEND_TIMEOUT_S)
                    else:
                        yaw_deg = yaw_from_quaternion(FC_node.getQuat())
                        await asyncio.wait_for(
                            FC_node.send_position_ned(0.0, 0.0, takeoff_z, yaw_deg),
                            timeout=CALIB_SEND_TIMEOUT_S)
                except asyncio.TimeoutError:
                    print(f"  [bail] send_position_ned timed out — PX4 likely rejected "
                          f"offboard setpoint; ending sweep early.")
                    break

                UAV_pose.append(pose_node.getPose().UAV)
                target_pose.append(pose_node.getPose().target)
                opt_flow_ang_vel.append(img_node.getRawOptFlowAngVel())
                img_feature_param.append(img_node.getRawImgFeatureParam())
                cmd.append(pos_cmd)
                phase_log.append(ph_name)

                await asyncio.sleep(SLEEP_TIME)
            else:
                continue
            break

        if not FC_node.LANDED:
            await FC_node.vehicle.action.land()

        img_node.close()
        await FC_node.close()

    except KeyboardInterrupt:
        if FC_node:
            await FC_node.vehicle.action.land()
        print("KeyboardInterrupt: Main Thread\n")

    except RuntimeError as e:
        print(f"RuntimeError: Main Thread: {e}\n")

    except SyntaxError:
        print("SyntaxError: Main Thread\n")

    except Exception as e:
        print(f"Unexpected error: Main Thread: {e}\n")

    finally:
        if img_node and pose_subscriber and FC_node:
            image_data = img_node.getLogData()
            telemetry_data = FC_node.getLogData()
            gt_data = {"Start Time": start_time, "Time": t_c, "Start Pose": start_pose,
                       "UAV Pose": UAV_pose, "Target Pose": target_pose,
                       "Opt Flow Ang Vel": opt_flow_ang_vel,
                       "Img Feature Params": img_feature_param,
                       "Command": cmd, "Phase": phase_log,
                       "Diag Log": img_node.get_diag_log(),   # [(t, ok, fail_reason, bbox_area), ...]
                       "Flow Diag Log": img_node.get_flow_diag_log(),   # [(t, n_kept, cond, solved), ...]
                       "Z_V Log": img_node.get_z_v_log()}   # min(z_v) per _getVirtualPts call
            img_params = img_node.getParams()

        if img_node and img_node.is_alive():
            img_node.close()
            img_node.join()

        if pose_subscriber and pose_subscriber.is_alive():
            pose_subscriber.close()
            pose_subscriber.join()

        if time_subscriber and time_subscriber.is_alive():
            time_subscriber.close()
            time_subscriber.join()

        if FC_node:
            await FC_node.close()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    res = 'n'

    try:
        asyncio.run(main(res))
    except KeyboardInterrupt:
        print("Clean exit on Ctrl+C")

    if CONTROLLER_READY:
        # Deliberately SEPARATE from calibration_data/output/ (ArUco cal-of-
        # record) so this never contaminates that scan. derive_cross_marker_cal.py
        # scans calibration_data/output_cross/ by default.
        base = os.environ.get(
            "CALIB_OUT_BASE",
            "/home/shubham/Soft-Precise-Landing/PX4_Gazebo/calibration_data/output_cross")
        dir_name = os.environ.get(
            "CALIB_OUT_DIR", f"{base}/{time.ctime().replace(':', '-')}")
        os.makedirs(dir_name, exist_ok=True)

        np.save(f'{dir_name}/Img_Data', image_data)
        np.save(f'{dir_name}/Telemetry_Data', telemetry_data)
        np.save(f'{dir_name}/Ground_Truth', gt_data)
        with open(f'{dir_name}/Img_Params.txt', 'w') as f:
            f.write(str(img_params))

        print(f"Calibration data saved -> {dir_name}")
