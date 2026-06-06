# OUTPUT-signal VALIDATION flight.
#
# Drives an INDEPENDENT maneuver and records Img_Data + Ground_Truth for
# notebooks/plotter_output_validation.ipynb (GT-direct validation of the
# calibrated corner + ring optical flow vs Gazebo GT).
#
# Two cmd_profiles, selected by VALIDATION_PROFILE:
#   'multisine' (default) — all axes excited together at distinct, non-harmonic
#       freqs during a slow z-sweep -> exercises the cal's cross-axis coupling
#       (the thing phased single-axis calibration cannot self-check).
#   'landing'             — monotone descent to touchdown -> the operating regime.
#
# Adapted from apps/output_calibration.py: SAME proven flight/record scaffolding
# (node bringup, takeoff, OFFBOARD establishment, send_position_ned loop, the
# same recorded fields). Only the cmd_profile (build_profile) and the
# validation_data routing differ. This is a SEPARATE script so validation
# maneuvers can evolve without touching the calibration app, and so a validation
# run can never be confused with a calibration run.
import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src'))
import asyncio, time
import numpy as np
import rclpy
from ahrs import RAD2DEG

from flight_controller import FC
import img_data as ID
from gz_subscriber import GZ_Subscriber, Pose_Node, Clock_Node

CAPTURE_RATE = 60                              # match landing_test.py / output_calibration.py
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
    """Return (label, phases). phases = [(name, fx, fy, fz, fyaw, dur)] where
    fx/fy = NED north/east position offset (m), fz = ABSOLUTE NED down (m,
    negative = altitude), fyaw = yaw offset (deg) added to the takeoff yaw.
    Each fn takes phase-relative time tau (0..dur)."""
    zero = lambda tau: 0.0
    hold = lambda tau: -TAKEOFF_H                                   # hover at takeoff altitude

    if PROFILE == "multisine":
        Tms   = float(os.environ.get("VAL_MULTISINE_S", "60.0"))
        floor = float(os.environ.get("VAL_FLOOR", "1.0"))
        gate  = float(os.environ.get("VAL_LAT_GATE_ALT", "1.2"))    # gate lateral below this alt
        A_xy  = float(os.environ.get("VAL_MS_AMP_XY", "0.30"))
        A_yaw = float(os.environ.get("VAL_MS_AMP_YAW_DEG", "15.0"))
        ncyc  = float(os.environ.get("VAL_Z_CYCLES", "2.0"))
        f_x, f_y, f_yaw = 0.40, 0.55, 0.25                          # distinct, non-harmonic
        f_z = ncyc / Tms
        cen, amp = 0.5*(TAKEOFF_H + floor), 0.5*(TAKEOFF_H - floor)
        alt = lambda tau: cen + amp*np.cos(2*np.pi*f_z*tau)         # TAKEOFF_H at tau=0
        lat = lambda tau: 1.0 if alt(tau) > gate else 0.0           # lateral stays in FoV
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
        # monotone descent takeoff_h -> floor, marker held centred (x=y=yaw=0).
        return "landing", [
            ("settle",  zero, zero, hold, zero, 2.0),
            ("landing", zero, zero,
             lambda tau: -(TAKEOFF_H - (TAKEOFF_H - floor) * min(tau / Tl, 1.0)),
             zero, Tl + 3.0)]

    raise SystemExit(f"unknown VALIDATION_PROFILE={PROFILE!r} (expected multisine|landing)")


async def main(record='y'):
    global image_data, telemetry_data, gt_data, img_params, CONTROLLER_READY
    img_node = pose_subscriber = time_subscriber = FC_node = None
    UAV_pose = []; target_pose = []; opt_flow = []; ring_flow = []
    feat = []; cmd = []; phase_log = []; t_c = []; start_pose = None; start_time = 0.0

    label, phases = build_profile()
    print(f"[val:output] profile={label}, total {sum(p[5] for p in phases):.1f}s")
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

        img_node = ID.IMG_PROCESSOR(capRate=CAPTURE_RATE, resolution=RESOLUTION,
                                    time_keeper=time_node, controller=FC_node)
        if record != 'n':
            img_node.RECORD = True

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
                opt_flow.append(img_node.getRawOptFlowAngVel())     # RAW (pre _sensor_cal_hw)
                ring_flow.append(img_node.getRawRingFlowAngVel())   # RAW ring (pre _sensor_cal_ring)
                feat.append(img_node.getRawImgFeatureParam())
                cmd.append(pos); phase_log.append(name)
                await asyncio.sleep(SLEEP_TIME)
            else:
                continue   # phase completed without break -> next phase
            break          # inner broke -> stop the maneuver

        if not FC_node.LANDED:
            await FC_node.vehicle.action.land()

    except Exception as e:
        print(f"[val:output] error: {e}")

    finally:
        if img_node and pose_subscriber and FC_node:
            image_data = img_node.getLogData()
            telemetry_data = FC_node.getLogData()
            gt_data = {"Start Time": start_time, "Time": t_c, "Start Pose": start_pose,
                       "UAV Pose": UAV_pose, "Target Pose": target_pose,
                       "Opt Flow Ang Vel": opt_flow, "Ring Opt Flow Ang Vel": ring_flow,
                       "Img Feature Params": feat, "Command": cmd, "Phase": phase_log}
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
        asyncio.run(main('y'))
    except KeyboardInterrupt:
        print("Clean exit on Ctrl+C")

    if CONTROLLER_READY:
        # Route to validation_data/<sub> by default; honor the launcher's
        # CALIB_OUT_DIR (when run via run_output_calibration.sh with CALIB_APP).
        sub = 'multisine' if PROFILE == 'multisine' else 'landing'
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
        print(f"[val:output] {PROFILE} validation data saved -> {dir_name}")
