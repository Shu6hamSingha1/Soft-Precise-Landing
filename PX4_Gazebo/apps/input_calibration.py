# **************************************************************************
# Based on adaptive_controller.py
# Include initial conditions: non-zero velocity
# **************************************************************************
import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src'))
import os
import asyncio
import time
import numpy as np
import rclpy # Python library for ROS 2
from ahrs import RAD2DEG

from flight_controller import FC
import img_data as ID
from gz_subscriber import GZ_Subscriber, Pose_Node, Clock_Node
from numerical_methods import RK5
from controller import Controller

# Companion Computer Details:
CAPTURE_RATE = 60 # Capture Rate = {90, 120, 200}
RESOLUTION = (1280, 960)
# RESOLUTION = (640, 480)
SLEEP_TIME = 1/100
REF_RAD_OPT_FLOW = -0.30
DES_IMG_FEATURE_PARAM = np.array([0.0, 0.0, 1.0, 0.0])

# Flight Controller Details:
TAKEOFF_HEIGHT = 10.0        # in metres
INITIAL_HEIGHT = 1.5        # in metres
INITIAL_VELOCITY = 0.2      # in m/s
LANDING_HEIGHT = 0.5       # in metres
LANDING_VELOCITY = 0.20     # in m/s
HOME_LOCATION = (13.017442, 77.565477, 955.0) #Lab GPS location and sea level altitude

# Flags
CONTROLLER_READY = False

# Global Logging variable
image_data = None
telemetry_data = None
controller_data = None
controller_params = None
gt_data = None
img_params = None

def convert_2_sys_cmd(cmd):
    return np.append(RAD2DEG*cmd[:3], 0.738 - cmd[3]/45)

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
    EC_node = None
    pose_subscriber = None
    time_subscriber = None
    FC_node = None

    UAV_pose = []
    target_pose = []
    start_pose = None
    t_c = []
    ac_cmd = []
    CONTROLLER_READY = False

    val = 0.05

    # Use the following cmd_profile for cmd to FC
    cmd_profile = np.array([[0.0,0.0,0.0,0.0],[-val,-val,-val,-val*100],[0.0,0.0,0.0,0.0],[val,val,val,val*100]]*5)

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
        
        EC_node = Controller(REF_RAD_OPT_FLOW, DES_IMG_FEATURE_PARAM, time_node, FC_node, record)

        await FC_node.arm_and_takeoff(TAKEOFF_HEIGHT)

        # await FC_node.send_position_ned(0.0, 0.0, FC_node.getPosBody().z_m, 90))
        yaw = yaw_from_quaternion(FC_node.getQuat())         
        await FC_node.send_position_ned(0.0, 0.0, FC_node.getPosBody().z_m, yaw)
        await asyncio.sleep(SLEEP_TIME)

        # main loop
        for cmd in cmd_profile:
            while EC_node.is_alive() and not FC_node.LANDED:
                if CONTROLLER_READY:
                    t_c.append(time_node.perf_counter() - start_time)
                    h = np.diff(t_c[-2:])

                else:
                    start_time = time_node.perf_counter()
                    CONTROLLER_READY = True
                    t_c = [0.0]
                    h = 0.0
                    restart_time = start_time

                # await FC_node.send_velocity_body(*v_cmd, a_cmd[3]))  # FC follows FRD
                # await FC_node.send_velocity_body(*cmd))  # FC follows FRD
                sys_cmd = convert_2_sys_cmd(cmd)
                try:
                    await asyncio.wait_for(
                        FC_node.send_attitude_rate(*sys_cmd),  # FC follows FRD
                        timeout=0.5)
                except asyncio.TimeoutError:
                    print("  [bail] send_attitude_rate timed out — PX4 likely dropped offboard.")
                    break

                await asyncio.sleep(SLEEP_TIME)
                
                UAV_pose.append(pose_node.getPose().UAV)
                target_pose.append(pose_node.getPose().target)
                ac_cmd.append(cmd)

                if time_node.perf_counter() - restart_time > 1.0:
                    print(f"Command | UAV Pose: {cmd} | {UAV_pose[-1].position}")
                    restart_time = time_node.perf_counter()
                    break

        yaw = yaw_from_quaternion(FC_node.getQuat())         
        await FC_node.send_position_ned(0.0, 0.0, FC_node.getPosBody().z_m, yaw)
        
        await FC_node.vehicle.action.land()
        await FC_node.close()

    except KeyboardInterrupt:
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
        if EC_node and pose_subscriber and FC_node:
            image_data = EC_node.getImgData()
            telemetry_data = FC_node.getLogData()
            controller_data = EC_node.getLogData()
            controller_params = EC_node.getParams()
            gt_data = {"Start Time": start_time, "Time": t_c, "Start Pose": start_pose, "UAV Pose": UAV_pose, "Target Pose": target_pose,"Command": ac_cmd}
            img_params = EC_node.getImgParams()

        # Close EC_node thread
        if EC_node and EC_node.is_alive():
            EC_node.close()
            EC_node.join()

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

    # Auto-save (no input prompt — runs headless via run_input_calibration.sh).
    if CONTROLLER_READY:
        dir_name = os.environ.get(
            "INPUT_CALIB_OUT_DIR",
            f"/home/shubham/Soft-Precise-Landing/PX4_Gazebo/calibration_data/input/"
            f"{time.ctime().replace(':', '-')}"
        )
        os.makedirs(dir_name, exist_ok=True)

        np.save(f'{dir_name}/Img_Data', image_data)
        np.save(f'{dir_name}/Telemetry_Data', telemetry_data)
        np.save(f'{dir_name}/Control_Data', controller_data)
        np.save(f'{dir_name}/Control_Params', controller_params)
        np.save(f'{dir_name}/Ground_Truth', gt_data)
        with open(f'{dir_name}/Img_Params.txt', 'w') as f:
            f.write(str(img_params))

        print(f"Input-calibration data saved -> {dir_name}")