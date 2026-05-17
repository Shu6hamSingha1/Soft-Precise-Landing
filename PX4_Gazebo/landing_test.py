# **************************************************************************
# Changed class and node name used in gz_subscriber
# Added time node for clock synchronization with simulator world
# Used simulator time instead of system time
# Added target tracking functionality
# **************************************************************************
import os
import asyncio
import time
import numpy as np
import rclpy # Python library for ROS 2
from ahrs import RAD2DEG


from flight_controller import FC
from gz_subscriber import GZ_Subscriber, Pose_Node, Clock_Node
from numerical_methods import RK5
from controller import Controller

# Setup variables
FENCE = (-5.0, 5.0, -5.0, 5.0, -5.0, 5.0) # xmin, xmax, ymin, ymax, zmin, zmax -- bounding box all values in m

# Companion Computer Details:
CAPTURE_RATE = 60 # Capture Rate = {90, 120, 200}
RESOLUTION = (640, 480)
SLEEP_TIME = 1/200
REF_RAD_OPT_FLOW = -0.42  # MATLAB h_rd (Constants.m); reverted from -0.30 during MATLAB↔PX4 alignment
DES_IMG_FEATURE_PARAM = np.array([0.0, 0.0, 1.0, 0.0])

# Flight Controller Details:
TAKEOFF_HEIGHT = 3.0        # in metres
INITIAL_HEIGHT = 1.5        # in metres
INITIAL_VELOCITY = 0.2      # in m/s
LANDING_HEIGHT = 0.0       # in metres
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
    # Body rates (rad/s -> deg/s) and normalized thrust [0, 1] for MAVSDK.
    #
    # Convention: cmd[3] is the controller's B_T scalar (Newtons of excess
    # thrust beyond gravity compensation). At hover, controller outputs
    # B_T = 0 and PX4 needs ~0.738 throttle to hold the X500 in Gazebo.
    # Slope 1/45 N^-1 was empirically chosen by the original author.
    #
    # PX4 throttle range = [0, 1]; clip to avoid invalid commands.
    thrust_norm = float(np.clip(0.738 - cmd[3] / 45.0, 0.0, 1.0))
    return np.append(RAD2DEG * cmd[:3], thrust_norm)

async def track_target(FC_node, EC_node, pose_node):
    while not EC_node.TARGET_IS_VISIBLE:
        # find target pose
        target_pose = pose_node.getPose().target
        x = target_pose.position.x
        y = target_pose.position.y
        z = FC_node.getPosBody().z_m
        yaw = target_pose.orientation.z
        await FC_node.send_position_ned(x, y, z, yaw)
        await asyncio.sleep(0.01)

async def main(record = 'n'):
    # Global Logging variable
    global image_data, telemetry_data, controller_data, controller_params, gt_data, img_params, CONTROLLER_READY

    # Initialize variables to None to avoid NameError in the `finally` block
    time_subscriber = None
    pose_subscriber = None
    FC_node = None
    EC_node = None

    UAV_pose = []
    target_pose = []
    start_pose = None
    t_c = []
    u_cmd = []

    try:
        rclpy.init()
        pose_node = Pose_Node()
        pose_subscriber = GZ_Subscriber(pose_node)
        time_node = Clock_Node()
        time_subscriber = GZ_Subscriber(time_node)

        start_time = time.perf_counter()
        while time_node.perf_counter() is None:
            if (time.perf_counter() - start_time) > 10:
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

        EC_node = Controller(REF_RAD_OPT_FLOW, DES_IMG_FEATURE_PARAM, time_node, FC_node)

        await FC_node.arm_and_takeoff(TAKEOFF_HEIGHT)

        # Hover-to-origin phase: PX4's built-in takeoff often drifts laterally
        # by several metres, which can put the ArUco marker outside the camera
        # FOV before the controller even starts. Send NED position (0, 0, -alt)
        # for ~4 s to drive the drone over the marker and let position-hold
        # stabilise before IBVS takes over.
        print("[landing_test] Hover-to-origin (4 s) before starting controller…")
        cur_z = FC_node.getPosBody().z_m
        for _ in range(200):                      # 200 * 20 ms = 4 s
            await FC_node.send_position_ned(0.0, 0.0, cur_z, 0.0)
            await asyncio.sleep(0.02)

        # Start controller (its thread begins iterating); send_attitude_rate
        # output is NOT used yet — we keep sending hover setpoints to PX4 while
        # the controller's internal buffers (PID integrator, _ds_d_deque,
        # _izeta, _dh_d_deque, _kappa) settle to steady-state values matching
        # the actual visual feedback. Without this warmup the first PID firing
        # drives dh_d to ~160 m/s² → c-term blow-up → a_u abort.
        start_time = time_node.perf_counter()
        EC_node.startController()
        CONTROLLER_READY = True
        t_c = [0.0]
        # Short warmup — just enough to fill the 4-deep smoothing deques
        # without letting the PID integrator wind up against the (likely
        # nonzero) centroid error while drone is hover-locked.
        print("[landing_test] PID warmup (100 ms — fills deques only)…")
        for _ in range(5):                        # 5 * 20 ms = 100 ms
            await FC_node.send_position_ned(0.0, 0.0, cur_z, 0.0)
            await asyncio.sleep(0.02)

        # Main control loop: controller is now warm.
        while EC_node.is_alive() and not FC_node.LANDED:
            UAV_pose.append(pose_node.getPose().UAV)
            target_pose.append(pose_node.getPose().target)

            t_c.append(time_node.perf_counter() - start_time)
            if EC_node.TARGET_IS_VISIBLE:
                cmd = EC_node.getControlInput()
                sys_cmd = convert_2_sys_cmd(cmd)
                await FC_node.send_attitude_rate(*sys_cmd)  # FC BODY follows FRD
                u_cmd.append(cmd)
            else:
                # Target lost — break the loop and let cleanup save what we have.
                break

            d = pose_node.getPose().UAV.position.z
            # d = - FC_node.getPosBody().z_m
            # Stop the controller if it lands 
            if d <= LANDING_HEIGHT:
            # if d <= LANDING_HEIGHT and abs(vz) < LANDING_VELOCITY:
               print("Landed")
               break

            await asyncio.sleep(SLEEP_TIME)
        
        EC_node.close()
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
        if EC_node and pose_subscriber and FC_node:
            image_data = EC_node.getImgData()
            telemetry_data = FC_node.getLogData()
            controller_data = EC_node.getLogData()
            controller_params = EC_node.getParams()
            gt_data = {"Start Time": start_time, "Time": t_c, "Start Pose": start_pose, "UAV Pose": UAV_pose, "Target Pose": target_pose,"Command": u_cmd}
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
    # res = input("Do you want to record? (y/n)")

    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Clean exit on Ctrl+C")

    # Save data# Save data
    if CONTROLLER_READY:
        if os.environ.get('LANDING_AUTOSAVE') == '1':
            x = 'y'
        else:
            x = input('Do you want to save the dataset? (y/n)')
        if x != 'n':
            # record timestamp
            timestamp = time.ctime().replace(':', '-')

            # Create a directory named based on timestamp
            dir_name = f"/home/shubham/ws/Test_Data/Landing_Test/{timestamp}"
            os.makedirs(dir_name)

            # Save files inside the folder 
            np.save(f'{dir_name}/Img_Data', image_data)
            np.save(f'{dir_name}/Telemetry_Data', telemetry_data)
            np.save(f'{dir_name}/Control_Data', controller_data)
            np.save(f'{dir_name}/Control_Params', controller_params)
            np.save(f'{dir_name}/Ground_Truth', gt_data)
            f = open(f'{dir_name}/Img_Params.txt', 'w')
            f.write(img_params)
            f.close()

            print("Saved to CSV!")

        else:
            print("Closed without saving!")