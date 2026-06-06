# INPUT-signal VALIDATION flight.
#
# Commands body-rate + thrust and records commanded (Ground_Truth['Command'])
# vs achieved (Telemetry angular velocity) for notebooks/plotter_input_
# validation.ipynb, which scores per-axis gain + LAG (the headline metric — a
# gain~1 with a 287 ms yaw lag is still a failing cal).
#
# Two cmd_profiles, selected by VALIDATION_PROFILE:
#   'multisine' (default) — all rate axes commanded TOGETHER at distinct,
#       non-harmonic freqs -> tests gain+lag under cross-axis coupling.
#   'landing'             — THRUST-MAP validation: command thrust_norm as a STAIRCASE
#       from 0.74 (just above hover ~0.738), dropping 0.01 each step (VAL_THRUST_STEP_S),
#       rates held 0. Takeoff 20 m; STOP at 1 m using GAZEBO GT altitude (more accurate
#       than the EKF). Records commanded thrust_norm vs achieved vertical accel across a
#       sweep of thrust levels -> fits the thrust map. NOT position-SP (PX4 would own
#       the thrust, leaving nothing to validate). Env: VAL_THRUST_START/STEP/STEP_S/MIN,
#       VAL_STOP_ALT, VAL_TAKEOFF_HEIGHT, VAL_LANDING_MAX_S.
#
# Adapted from apps/record_input_calibration.py (same convert_2_sys_cmd thrust mapping
# and send_attitude_rate path). OPEN-LOOP body rates: amplitudes are kept modest
# and env-overridable — tune VAL_RATE_AMP / VAL_INPUT_MS_S down if your PX4 build
# tips over or descends during the multisine. No image/controller is needed (the
# input metric reads only Telemetry + Command).
import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src'))
import asyncio, time
import numpy as np
import rclpy
from ahrs import RAD2DEG

from flight_controller import FC
from gz_subscriber import GZ_Subscriber, Pose_Node, Clock_Node

SLEEP_TIME = 1/200
SEND_TIMEOUT_S = 0.5
PROFILE   = os.environ.get("VALIDATION_PROFILE", "multisine").lower()
# landing (thrust-map) starts high (20 m) for the thrust staircase; multisine hovers low.
TAKEOFF_H = float(os.environ.get("VAL_TAKEOFF_HEIGHT", "20.0" if PROFILE == "landing" else "5.0"))
# thrust-staircase (landing): command thrust_norm = THR0, dropping DTHR every STEP_S,
# rates 0; stop at STOP_ALT using Gazebo GT altitude. Sweeps thrust to fit the thrust map.
THR0     = float(os.environ.get("VAL_THRUST_START", "0.74"))   # just above hover (~0.738)
DTHR     = float(os.environ.get("VAL_THRUST_STEP",  "0.01"))   # drop per step
STEP_S   = float(os.environ.get("VAL_THRUST_STEP_S", "1.5"))   # hold each thrust level this long
THR_MIN  = float(os.environ.get("VAL_THRUST_MIN",   "0.55"))   # safety floor on thrust_norm
STOP_ALT = float(os.environ.get("VAL_STOP_ALT",     "1.0"))    # stop at this GT height (m)

CONTROLLER_READY = False
telemetry_data = gt_data = None


def convert_2_sys_cmd(cmd):
    # body rates rad/s -> deg/s; thrust Newton-offset -> normalized.
    # Matches apps/record_input_calibration.py:52 (slope 1/42.3, hover at 0.738).
    return np.append(RAD2DEG * cmd[:3], 0.738 - cmd[3] / 42.3)


def yaw_from_quaternion(q):
    return RAD2DEG * np.arctan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))


def cmd_at(tau):
    """MULTISINE cmd_profile -> [wx, wy, wz, thrust_offset_N] (rad/s rates; thrust 0).
    The 'landing' profile is a thrust_norm STAIRCASE handled in the send loop."""
    A = float(os.environ.get("VAL_RATE_AMP", "0.15"))          # rad/s, modest (open-loop)
    f_x, f_y, f_z = 0.40, 0.55, 0.70                           # distinct, non-harmonic
    return np.array([A*np.sin(2*np.pi*f_x*tau),
                     A*np.sin(2*np.pi*f_y*tau),
                     A*np.sin(2*np.pi*f_z*tau), 0.0])


def thrust_staircase(tau):
    """landing thrust_norm at time tau: THR0, dropping DTHR every STEP_S (floored)."""
    return max(THR_MIN, THR0 - DTHR * int(tau // STEP_S))


async def main():
    global telemetry_data, gt_data, CONTROLLER_READY
    pose_subscriber = time_subscriber = FC_node = None
    UAV_pose = []; target_pose = []; ac_cmd = []; t_c = []; start_pose = None; start_time = 0.0

    DUR = (float(os.environ.get("VAL_INPUT_MS_S", "20.0")) if PROFILE == "multisine"
           else float(os.environ.get("VAL_LANDING_MAX_S", "120.0")))   # GT-alt stop ends it early
    print(f"[val:input] profile={PROFILE}, {DUR:.1f}s")
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

        await FC_node.arm_and_takeoff(TAKEOFF_H)
        yaw = yaw_from_quaternion(FC_node.getQuat())
        await FC_node.send_position_ned(0.0, 0.0, FC_node.getPosBody().z_m, yaw)
        await asyncio.sleep(1.0)

        start_time = time_node.perf_counter()
        for _k in range(int(DUR / SLEEP_TIME)):
            if FC_node.LANDED:
                break
            if CONTROLLER_READY:
                t_c.append(time_node.perf_counter() - start_time)
            else:
                start_time = time_node.perf_counter(); CONTROLLER_READY = True; t_c = [0.0]
            tau = time_node.perf_counter() - start_time
            pose = pose_node.getPose()

            # multisine -> rate-loop excitation (rates via convert_2_sys_cmd);
            # landing  -> thrust_norm STAIRCASE (THR0 dropping DTHR/step, rates 0),
            #             commanded DIRECTLY (not via the 0.738-offset map), so the
            #             commanded thrust_norm is recorded vs the achieved accel.
            try:
                if PROFILE == "landing":
                    thr = thrust_staircase(tau)
                    await asyncio.wait_for(FC_node.send_attitude_rate(0.0, 0.0, 0.0, thr),
                                           timeout=SEND_TIMEOUT_S)
                    cmd = np.array([0.0, 0.0, 0.0, thr])     # record commanded thrust_norm
                else:
                    cmd = cmd_at(tau)
                    await asyncio.wait_for(FC_node.send_attitude_rate(*convert_2_sys_cmd(cmd)),
                                           timeout=SEND_TIMEOUT_S)
            except asyncio.TimeoutError:
                print("  [bail] send_attitude_rate timed out — PX4 likely dropped OFFBOARD.")
                break
            await asyncio.sleep(SLEEP_TIME)
            UAV_pose.append(pose.UAV)
            target_pose.append(pose.target)
            ac_cmd.append(cmd)

            # STOP at STOP_ALT using GAZEBO GT altitude (height above target) — more
            # accurate than the EKF estimate for the terminal cutoff.
            if PROFILE == "landing":
                gt_alt = abs(pose.UAV.position.z - pose.target.position.z)
                if gt_alt <= STOP_ALT:
                    print(f"  [landing] GT alt {gt_alt:.2f} m <= {STOP_ALT} m (thr={thr:.3f}) -> stop.")
                    break

        yaw = yaw_from_quaternion(FC_node.getQuat())
        await FC_node.send_position_ned(0.0, 0.0, FC_node.getPosBody().z_m, yaw)
        await FC_node.vehicle.action.land()

    except Exception as e:
        print(f"[val:input] error: {e}")

    finally:
        if pose_subscriber and FC_node:
            telemetry_data = FC_node.getLogData()
            gt_data = {"Start Time": start_time, "Time": t_c, "Start Pose": start_pose,
                       "UAV Pose": UAV_pose, "Target Pose": target_pose, "Command": ac_cmd}
        for node in (pose_subscriber, time_subscriber):
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
        sub = 'input_multi' if PROFILE == 'multisine' else 'landing'
        dir_name = os.environ.get("VAL_OUT_DIR") or os.environ.get("INPUT_CALIB_OUT_DIR")
        if not dir_name:
            base = os.environ.get(
                "VAL_OUT_BASE",
                f"/home/shubham/Soft-Precise-Landing/PX4_Gazebo/validation_data/{sub}")
            dir_name = f"{base}/{time.ctime().replace(':', '-')}"
        os.makedirs(dir_name, exist_ok=True)
        np.save(f'{dir_name}/Telemetry_Data', telemetry_data)
        np.save(f'{dir_name}/Ground_Truth', gt_data)
        print(f"[val:input] {PROFILE} validation data saved -> {dir_name}")
