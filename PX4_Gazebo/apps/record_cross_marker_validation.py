# OUTPUT-signal VALIDATION flight for the cross+stub marker.
#
# Independent maneuver (multisine or landing) recorded to validation_data/,
# NEVER calibration_data/output_cross/ -- keeps this data out of the derive
# tool's scan so the calibration and its validation stay independent (same
# two-stage discipline as apps/record_output_validation.py for ArUco; see
# the io-calibration skill).
#
# Adapted from apps/record_output_validation.py: same flight/record
# scaffolding, swapped for CrossMarkerNode (src/cross_marker_perception.py)
# the same way apps/record_cross_marker_calibration.py adapted
# record_output_calibration.py. Records RAW (pre-cal) h,w/s -- the analysis
# tool (tools/validate_cross_marker_flow.py) applies the LIVE
# _sensor_cal_hw/_sensor_cal_s from CrossMarkerPerception at analysis time,
# so re-deriving the cal never requires re-recording this data.
import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src'))
import asyncio, time
import numpy as np
import rclpy
from ahrs import RAD2DEG

os.environ.setdefault("MARKER_TYPE", "cross")

from flight_controller import FC
from cross_marker_perception import CrossMarkerNode
from gz_subscriber import GZ_Subscriber, Pose_Node, Clock_Node

CAPTURE_RATE = 60
RESOLUTION   = (640, 480)
SLEEP_TIME   = 1/200
SEND_TIMEOUT_S = 0.5
TAKEOFF_H = float(os.environ.get("VAL_TAKEOFF_HEIGHT", "5.0"))
PROFILE   = os.environ.get("VALIDATION_PROFILE", "multisine").lower()

CONTROLLER_READY = False
image_data = telemetry_data = gt_data = img_params = None


def yaw_from_quaternion(q):
    return RAD2DEG * np.arctan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))


def build_profile():
    zero = lambda tau: 0.0
    hold = lambda tau: -TAKEOFF_H

    if PROFILE == "multisine":
        Tms   = float(os.environ.get("VAL_MULTISINE_S", "60.0"))
        floor = float(os.environ.get("VAL_FLOOR", "1.0"))
        gate  = float(os.environ.get("VAL_LAT_GATE_ALT", "1.2"))
        A_xy  = float(os.environ.get("VAL_MS_AMP_XY", "0.30"))
        A_yaw = float(os.environ.get("VAL_MS_AMP_YAW_DEG", "15.0"))
        ncyc  = float(os.environ.get("VAL_Z_CYCLES", "2.0"))
        f_x, f_y, f_yaw = 0.40, 0.55, 0.25
        f_z = ncyc / Tms
        cen, amp = 0.5*(TAKEOFF_H + floor), 0.5*(TAKEOFF_H - floor)
        alt = lambda tau: cen + amp*np.cos(2*np.pi*f_z*tau)
        lat = lambda tau: 1.0 if alt(tau) > gate else 0.0
        return "multisine", [
            ("settle",    zero, zero, hold, zero, 2.0),
            ("multisine",
             lambda tau: A_xy * np.sin(2*np.pi*f_x*tau) * lat(tau),
             lambda tau: A_xy * np.sin(2*np.pi*f_y*tau) * lat(tau),
             lambda tau: -alt(tau),
             lambda tau: A_yaw * np.sin(2*np.pi*f_yaw*tau),
             Tms)]

    if PROFILE == "landing":
        Tl    = float(os.environ.get("VAL_LANDING_S", "20.0"))
        floor = float(os.environ.get("VAL_LANDING_FLOOR", "0.0"))
        return "landing", [
            ("settle",  zero, zero, hold, zero, 2.0),
            ("landing", zero, zero,
             lambda tau: -(TAKEOFF_H - (TAKEOFF_H - floor) * min(tau / Tl, 1.0)),
             zero, Tl + 3.0)]

    raise SystemExit(f"unknown VALIDATION_PROFILE={PROFILE!r} (expected multisine|landing)")


async def main():
    global image_data, telemetry_data, gt_data, img_params, CONTROLLER_READY
    img_node = pose_subscriber = time_subscriber = FC_node = None
    UAV_pose = []; target_pose = []; opt_flow = []; feat = []
    cmd = []; phase_log = []; t_c = []; start_pose = None; start_time = 0.0

    label, phases = build_profile()
    print(f"[val:cross] profile={label}, total {sum(p[5] for p in phases):.1f}s")
    try:
        rclpy.init()
        pose_node = Pose_Node(); pose_subscriber = GZ_Subscriber(pose_node)
        time_node = Clock_Node(); time_subscriber = GZ_Subscriber(time_node)

        t0 = time.perf_counter()
        while time_node.perf_counter() is None:
            if time.perf_counter() - t0 > 20: raise Exception("Unable to get simulation time.")

        FC_node = FC(time_node); await FC_node.start()
        t0 = time.perf_counter()
        while not FC_node.has_quat():
            if time.perf_counter() - t0 > 20: raise Exception("Unable to get data from FC.")
            time.sleep(0.05)
        while start_pose is None: start_pose = pose_node.getPose().UAV
        yaw0 = yaw_from_quaternion(FC_node.getQuat())

        img_node = CrossMarkerNode(time_keeper=time_node, controller=FC_node)

        await FC_node.arm_and_takeoff(TAKEOFF_H)
        print("Hovering 5s for the PX4 state-machine to stabilize...")
        await asyncio.sleep(5.0)
        print("Establishing OFFBOARD with stationary setpoints (2s @ 30 Hz)...")
        for _ in range(60):
            await FC_node.send_position_ned(0.0, 0.0, FC_node.getPosBody().z_m,
                                            yaw_from_quaternion(FC_node.getQuat()))
            await asyncio.sleep(SLEEP_TIME)

        start_time = time_node.perf_counter()
        for name, fn_x, fn_y, fn_z, fn_yaw, dur in phases:
            print(f"  [phase] {name:9s} duration={dur:.1f}s")
            phase_t0 = time_node.perf_counter()
            for _k in range(int(dur / SLEEP_TIME)):
                if not img_node.is_alive() and FC_node.LANDED:
                    break
                if CONTROLLER_READY:
                    t_c.append(time_node.perf_counter() - start_time)
                else:
                    start_time = time_node.perf_counter(); phase_t0 = start_time
                    CONTROLLER_READY = True; t_c = [0.0]
                    img_node.CONTROLLER_READY = True

                tau = time_node.perf_counter() - phase_t0
                pos = np.array([fn_x(tau), fn_y(tau), fn_z(tau), yaw0 + fn_yaw(tau)])
                try:
                    if img_node.FEATURE_IS_VISIBLE:
                        await asyncio.wait_for(FC_node.send_position_ned(*pos), timeout=SEND_TIMEOUT_S)
                    else:
                        await asyncio.wait_for(
                            FC_node.send_position_ned(0.0, 0.0, fn_z(tau),
                                                      yaw_from_quaternion(FC_node.getQuat())),
                            timeout=SEND_TIMEOUT_S)
                except asyncio.TimeoutError:
                    print("  [bail] send_position_ned timed out — PX4 likely dropped OFFBOARD.")
                    break

                UAV_pose.append(pose_node.getPose().UAV)
                target_pose.append(pose_node.getPose().target)
                opt_flow.append(img_node.getRawOptFlowAngVel())
                feat.append(img_node.getRawImgFeatureParam())
                cmd.append(pos); phase_log.append(name)
                await asyncio.sleep(SLEEP_TIME)
            else:
                continue
            break

        if not FC_node.LANDED:
            await FC_node.vehicle.action.land()

    except Exception as e:
        print(f"[val:cross] error: {e}")

    finally:
        if img_node and pose_subscriber and FC_node:
            image_data = img_node.getLogData()
            telemetry_data = FC_node.getLogData()
            gt_data = {"Start Time": start_time, "Time": t_c, "Start Pose": start_pose,
                       "UAV Pose": UAV_pose, "Target Pose": target_pose,
                       "Opt Flow Ang Vel": opt_flow, "Img Feature Params": feat,
                       "Command": cmd, "Phase": phase_log,
                       "Diag Log": img_node.get_diag_log(),
                       "Flow Diag Log": img_node.get_flow_diag_log(),
                       "Radial Diag Log": img_node.get_radial_diag_log()}
            img_params = img_node.getParams()
        for node in (img_node, pose_subscriber, time_subscriber):
            if node and node.is_alive():
                node.close(); node.join()
        if FC_node:
            await FC_node.close()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Clean exit on Ctrl+C")

    if CONTROLLER_READY:
        sub = 'cross_output_multisine' if PROFILE == 'multisine' else 'cross_output_landing'
        dir_name = os.environ.get("VAL_OUT_DIR") or os.environ.get("CALIB_OUT_DIR")
        if not dir_name:
            base = os.environ.get(
                "VAL_OUT_BASE",
                f"/home/shubham/Soft-Precise-Landing/PX4_Gazebo/validation_data/{sub}")
            dir_name = f"{base}/{time.ctime().replace(':', '-')}"
        os.makedirs(dir_name, exist_ok=True)
        np.save(f'{dir_name}/Img_Data', image_data)
        np.save(f'{dir_name}/Telemetry_Data', telemetry_data)
        np.save(f'{dir_name}/Ground_Truth', gt_data)
        with open(f'{dir_name}/Img_Params.txt', 'w') as f:
            f.write(str(img_params))
        print(f"[val:cross] {PROFILE} validation data saved -> {dir_name}")
