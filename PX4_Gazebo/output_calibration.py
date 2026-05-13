# **************************************************************************
# Changed class and node name used in gz_subscriber
# Added time node for clock synchronization with simulator world
# Used simulator time instead of system time
# Sending sinusoidal velocity commands to FC for calibration
# **************************************************************************
import os
import asyncio
import time
import numpy as np
import rclpy # Python library for ROS 2
from ahrs import RAD2DEG

from flight_controller import FC
import img_data as ID
from gz_subscriber import GZ_Subscriber, Pose_Node, Clock_Node
# from numerical_methods import RK5
# from controller import Controller

# Companion Computer Details:
CAPTURE_RATE = 60 # Capture Rate = {90, 120, 200}
# RESOLUTION = (1280, 960)
# RESOLUTION = (640, 480)
RESOLUTION = (320, 240)
SLEEP_TIME = 1/30
REF_RAD_OPT_FLOW = -0.30
DES_IMG_FEATURE_PARAM = np.array([0.0, 0.0, 1.0, 0.0])

# Calibration sweep amplitudes (reduced from 1.0 m / 15 deg to keep PX4
# preflight checks happy. Empirically the previous 1.0 m sinusoid tripped
# "High Accelerometer Bias" / "horizontal velocity unstable" → failsafe →
# OFFBOARD drop → send_position_ned hang.)
CALIB_AMP_XY = 0.7
CALIB_AMP_Z  = 0.3       # mild z oscillation needed to excite the z-axis optical flow
CALIB_AMP_YAW_DEG = 10.0
# Per-call timeout on send_position_ned so a stuck MAVSDK can't deadlock the
# sweep loop. 0.5 s is generous vs the 33 ms loop period.
CALIB_SEND_TIMEOUT_S = 0.5
# Bail if the drone drops more than this many metres below its post-takeoff
# altitude (signals PX4 has entered AUTO_LAND under a failsafe).
CALIB_ALT_DROP_BAIL_M = 2.0

# Flags
CONTROLLER_READY = False

# Global Logging variable
image_data = None
telemetry_data = None
controller_data = None
controller_params = None
gt_data = None
img_params = None

def yaw_from_quaternion(q):
    """
    Compute yaw angle (rad) from quaternion.
    Quaternion format: [w, x, y, z]
    """
    yaw = np.arctan2(
        2.0 * (q.w*q.z + q.x*q.y),
        1.0 - 2.0 * (q.y*q.y + q.z*q.z)
    )
    return RAD2DEG*yaw

async def main(record = 'n'):
    # Global Logging variable
    global image_data, telemetry_data, controller_data, controller_params, gt_data, img_params, CONTROLLER_READY

    # Initialize variables to None to avoid NameError in the `finally` block
    img_node = None
    # EC_node = None
    pose_subscriber = None
    time_subscriber = None
    FC_node = None

    UAV_pose = []
    target_pose = []
    opt_flow_ang_vel = []
    img_feature_param = []
    start_pose = None
    t_c = []
    cmd = []
    # Use the following az_profile for vz_cmd to FC
    cmd_profile = 1.0*np.sin(np.linspace(0, 15*np.pi, 1000))

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

        yaw_deg_0 = yaw_from_quaternion(FC_node.getQuat())*0.0
        
        img_node = ID.IMG_PROCESSOR(capRate=CAPTURE_RATE, resolution=RESOLUTION, time_keeper=time_node, controller=FC_node) # start the thread for onboard camera flow streaming

        if record != 'n':
            img_node.RECORD = True
            print("Starting with recording...")

        else:
            print("Starting without recording...!")

        await FC_node.arm_and_takeoff(8.0)

        # Post-takeoff stabilization: PX4's autonomous TAKEOFF task hands off to
        # HOLD/LOITER but the state-machine transition takes a moment. If we
        # push offboard setpoints into that window, flight_mode_manager logs
        # `Matching flight task was not able to run` and drops into AUTO_LAND
        # ("blind land" failsafe). Sleep + bombard with stationary setpoints
        # so PX4 settles into a clean OFFBOARD mode before the sinusoidal sweep.
        print("Hovering 5s for PX4 state-machine to stabilize...")
        await asyncio.sleep(5.0)

        print("Establishing offboard with stationary setpoints (2s @ 30 Hz)...")
        for _ in range(60):
            yaw_deg_curr = yaw_from_quaternion(FC_node.getQuat())
            await FC_node.send_position_ned(0.0, 0.0,
                                            FC_node.getPosBody().z_m,
                                            yaw_deg_curr)
            await asyncio.sleep(SLEEP_TIME)

        # res = input("Do you want to start? (y/n)")

        print("Starting calibration sweep...")
        # Capture post-takeoff altitude so we can detect failsafe-driven descent.
        takeoff_z = FC_node.getPosBody().z_m
        for val in cmd_profile:
            if not img_node.is_alive() and FC_node.LANDED:
                break

            # Bail if PX4 has dropped offboard and started descending under failsafe.
            # PosBody z is NED-down (negative = above start), so a "drop" means z
            # increases toward zero (or positive).
            cur_z = FC_node.getPosBody().z_m
            if cur_z - takeoff_z > CALIB_ALT_DROP_BAIL_M:
                print(f"  [bail] altitude dropped {cur_z - takeoff_z:+.2f} m below takeoff — "
                      f"PX4 likely failsafed; ending sweep early.")
                break

            if CONTROLLER_READY:
                t_c.append(time_node.perf_counter() - start_time)
            else:
                start_time = time_node.perf_counter()
                CONTROLLER_READY = True
                t_c = [0.0]

            # Reduced-amplitude command: gentler than the original 1.0 m / 15° / ±1 m z
            # sweep, but enough motion on all 4 axes for a clean lstsq solution.
            # z varies smoothly (sin), not the square-wave-of-sign of the original.
            pos_cmd = np.array([CALIB_AMP_XY * val,
                                CALIB_AMP_XY * val,
                                takeoff_z + CALIB_AMP_Z * val,
                                yaw_deg_0 + CALIB_AMP_YAW_DEG * val])

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
            # Log RAW values (pre sensor_cal) so this script can be used to recalibrate.
            opt_flow_ang_vel.append(img_node.getRawOptFlowAngVel())
            img_feature_param.append(img_node.getRawImgFeatureParam())
            cmd.append(pos_cmd)

            await asyncio.sleep(SLEEP_TIME)

            # print(f"Commanded Profile | Position: {pos_cmd} | {UAV_pose[-1]}")
            # print(f"Commanded vz: {vz}")
        
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
        # Save data
        if img_node and pose_subscriber and FC_node:
            image_data = img_node.getLogData()
            telemetry_data = FC_node.getLogData()
            # controller_data = EC_node.getLogData()
            # controller_params = EC_node.getParams()
            gt_data = {"Start Time": start_time, "Time": t_c, "Start Pose": start_pose, "UAV Pose": UAV_pose, "Target Pose": target_pose, "Opt Flow Ang Vel": opt_flow_ang_vel, "Img Feature Params": img_feature_param, "Command": cmd}
            img_params = img_node.getParams()

        # Close img_node thread
        if img_node and img_node.is_alive():
            img_node.close()
            img_node.join()

        # Close pose_subscriber thread
        if pose_subscriber and pose_subscriber.is_alive():
            pose_subscriber.close()
            pose_subscriber.join()

        # Close time_subscriber thread
        if time_subscriber and time_subscriber.is_alive():
            time_subscriber.close()
            time_subscriber.join()

        # Close flight controller
        if FC_node:
            await FC_node.close()

        # Shutdown ROS client library if not already shut down
        if rclpy.ok():  # Check if the context is still active
            rclpy.shutdown()

if __name__ == "__main__":
    res = 'n'

    try:
        asyncio.run(main(res))
    except KeyboardInterrupt:
        print("Clean exit on Ctrl+C")

    # Auto-save (no input prompt — runs headless via run_output_calibration.sh)
    if CONTROLLER_READY:
        # Allow override via env var so the launcher can predict the path.
        dir_name = os.environ.get(
            "CALIB_OUT_DIR",
            f"/home/shubham/Soft-Precise-Landing/PX4_Gazebo/calibration_data/output/"
            f"{time.ctime().replace(':', '-')}"
        )
        os.makedirs(dir_name, exist_ok=True)

        np.save(f'{dir_name}/Img_Data', image_data)
        np.save(f'{dir_name}/Telemetry_Data', telemetry_data)
        np.save(f'{dir_name}/Ground_Truth', gt_data)
        with open(f'{dir_name}/Img_Params.txt', 'w') as f:
            f.write(str(img_params))

        print(f"Calibration data saved -> {dir_name}")

        