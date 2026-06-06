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
#   'landing'             — POSITION-SETPOINT descent (send_position_ned, PX4 holds
#       attitude; NO controller, NO open-loop tip-over) -> records the FC's achieved
#       rate/thrust response through the descent regime to touchdown. (Use the
#       'multisine' profile for the command->response gain+lag validation; the
#       landing profile is a clean-descent characterization, not command-tracking.)
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
TAKEOFF_H = float(os.environ.get("VAL_TAKEOFF_HEIGHT", "5.0"))
PROFILE   = os.environ.get("VALIDATION_PROFILE", "multisine").lower()

CONTROLLER_READY = False
telemetry_data = gt_data = None


def convert_2_sys_cmd(cmd):
    # body rates rad/s -> deg/s; thrust Newton-offset -> normalized.
    # Matches apps/record_input_calibration.py:52 (slope 1/42.3, hover at 0.738).
    return np.append(RAD2DEG * cmd[:3], 0.738 - cmd[3] / 42.3)


def yaw_from_quaternion(q):
    return RAD2DEG * np.arctan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))


def cmd_at(tau):
    """MULTISINE cmd_profile -> [wx, wy, wz, thrust_offset_N] at time tau (rad/s body
    rates; thrust_offset 0). The 'landing' profile is a POSITION-SETPOINT descent
    handled in the send loop (send_position_ned), not here."""
    A = float(os.environ.get("VAL_RATE_AMP", "0.15"))          # rad/s, modest (open-loop)
    f_x, f_y, f_z = 0.40, 0.55, 0.70                           # distinct, non-harmonic
    return np.array([A*np.sin(2*np.pi*f_x*tau),
                     A*np.sin(2*np.pi*f_y*tau),
                     A*np.sin(2*np.pi*f_z*tau), 0.0])


async def main():
    global telemetry_data, gt_data, CONTROLLER_READY
    pose_subscriber = time_subscriber = FC_node = None
    UAV_pose = []; target_pose = []; ac_cmd = []; t_c = []; start_pose = None; start_time = 0.0

    DUR = (float(os.environ.get("VAL_INPUT_MS_S", "20.0")) if PROFILE == "multisine"
           else float(os.environ.get("VAL_LANDING_S", "20.0")) + 3.0)
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

        yaw0  = yaw_from_quaternion(FC_node.getQuat())             # takeoff yaw (held in landing)
        floor = float(os.environ.get("VAL_LANDING_FLOOR", "0.0"))
        Tl    = float(os.environ.get("VAL_LANDING_S", "20.0"))
        start_time = time_node.perf_counter()
        for _k in range(int(DUR / SLEEP_TIME)):
            if FC_node.LANDED:
                break
            if CONTROLLER_READY:
                t_c.append(time_node.perf_counter() - start_time)
            else:
                start_time = time_node.perf_counter(); CONTROLLER_READY = True; t_c = [0.0]
            tau = time_node.perf_counter() - start_time

            try:
                if PROFILE == "landing":
                    # POSITION-SETPOINT descent (PX4 holds attitude — clean, no
                    # open-loop tip-over). Record the z setpoint as 'Command'; the
                    # FC's achieved rate/thrust response is in Telemetry.
                    z = -(TAKEOFF_H - (TAKEOFF_H - floor) * min(tau / Tl, 1.0))
                    await asyncio.wait_for(FC_node.send_position_ned(0.0, 0.0, z, yaw0),
                                           timeout=SEND_TIMEOUT_S)
                    cmd = np.array([0.0, 0.0, z, 0.0])
                else:
                    cmd = cmd_at(tau)
                    await asyncio.wait_for(FC_node.send_attitude_rate(*convert_2_sys_cmd(cmd)),
                                           timeout=SEND_TIMEOUT_S)
            except asyncio.TimeoutError:
                print("  [bail] setpoint send timed out — PX4 likely dropped OFFBOARD.")
                break
            await asyncio.sleep(SLEEP_TIME)
            UAV_pose.append(pose_node.getPose().UAV)
            target_pose.append(pose_node.getPose().target)
            ac_cmd.append(cmd)

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
