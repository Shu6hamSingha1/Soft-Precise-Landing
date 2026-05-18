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
# MATLAB-IC match (Multi_init_cond/InitVar.m line 17):
#   I_p_c = [2.0, 2.0, -5.0]   (NED — i.e. 2 east, 2 north, 5 up in ENU)
#   zero velocity, zero attitude rate, identity quaternion
# Drone is positioned to (2, 2, 5) ENU and allowed to settle to zero
# velocity BEFORE the IBVS controller engages — same starting state as
# the MATLAB simulation, so SITL transients no longer depend on the
# random takeoff drift of PX4's AUTO_TAKEOFF.
INITIAL_DRONE_ENU = (0.0, 0.0, 5.0)   # directly above marker at 5 m; isolates the z-channel
TAKEOFF_HEIGHT = INITIAL_DRONE_ENU[2]   # 5.0 m, lift before flying to IC
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

        # Fly to MATLAB initial condition: drone at INITIAL_DRONE_ENU (above the
        # marker, Gazebo world coords). Each iteration recomputes a NED setpoint
        # from the CURRENT Gazebo ENU↔PX4 NED transform — this actively
        # compensates for EKF position drift. An earlier "snapshot the transform
        # once" variant locked the setpoint in PX4 frame and let Gazebo truth
        # drift 1.3 m off the IC over 15 s; recomputing keeps the drone over
        # the marker in world truth regardless of EKF wander.
        #
        # Convention: Gazebo ENU (x=East, y=North, z=Up), PX4 NED (x=North,
        # y=East, z=Down). For DISPLACEMENT vectors: NED_x=ENU_y, NED_y=ENU_x,
        # NED_z=-ENU_z.
        #
        # Convergence: pos_err ≤ 0.5 m AND speed ≤ 0.3 m/s AND |yaw_err| ≤ 2°
        # for 10 consecutive samples (0.2 s). All measured against Gazebo truth,
        # NOT PX4 NED. Tolerances reflect realized PX4 steady-state in SITL
        # (tighter pos/vel values of 0.3 / 0.15 consistently timed out at ~0.4
        # / 0.22). The yaw check was added 2026-05-18 after a 5-run variance
        # sweep showed PX4 sometimes spawns at yaw ≈ -13° and instant position-
        # convergence (0.2 s) doesn't give the yaw setpoint time to rotate the
        # drone to 0°. Engaging the controller from a 13°-misaligned V-frame
        # caused the K_ri ×10 integrator to wind up around the misalignment and
        # produced systematic lateral drift in whatever direction noise nudged.
        # Max wait 15 s; final 1 s is a settle phase regardless of convergence.
        ix, iy, iz = INITIAL_DRONE_ENU
        target_enu_0 = pose_node.getPose().target.position
        print(f"[landing_test] Hover to MATLAB-IC ENU ({ix},{iy},{iz})")
        IC_POS_TOL  = 0.5
        IC_VEL_TOL  = 0.3
        IC_YAW_TOL  = np.deg2rad(2.0)
        STABLE_HITS = 10
        MAX_ITERS   = 750                         # 15 s @ 50 Hz
        prev_enu    = pose_node.getPose().UAV.position
        prev_t      = time.perf_counter()
        stable      = 0
        converged_at = None
        target_n = target_e = target_d = 0.0
        yaw_err = 0.0
        for k in range(MAX_ITERS):
            drone_enu  = pose_node.getPose().UAV.position
            target_enu = pose_node.getPose().target.position
            ned_pos    = FC_node.getPosBody()
            # Re-derive setpoint each iter — chases the Gazebo truth target
            # through PX4 EKF drift.
            de_enu = (target_enu.x + ix - drone_enu.x)
            dn_enu = (target_enu.y + iy - drone_enu.y)
            du_enu = (target_enu.z + iz - drone_enu.z)
            target_n = ned_pos.x_m + dn_enu
            target_e = ned_pos.y_m + de_enu
            target_d = ned_pos.z_m - du_enu
            await FC_node.send_position_ned(target_n, target_e, target_d, 0.0)
            await asyncio.sleep(0.02)
            # Convergence check on Gazebo truth (post-setpoint sample).
            cur_enu = pose_node.getPose().UAV.position
            now = time.perf_counter()
            dt = max(now - prev_t, 1e-3)
            speed = ((cur_enu.x - prev_enu.x)**2 +
                     (cur_enu.y - prev_enu.y)**2 +
                     (cur_enu.z - prev_enu.z)**2) ** 0.5 / dt
            d_e = cur_enu.x - (target_enu.x + ix)
            d_n = cur_enu.y - (target_enu.y + iy)
            d_u = cur_enu.z - (target_enu.z + iz)
            pos_err = (d_e*d_e + d_n*d_n + d_u*d_u) ** 0.5
            # Yaw error from PX4 quaternion: ψ = atan2(2(wz+xy), 1 − 2(y²+z²))
            # Target yaw = 0 rad. We use the smallest signed angular distance.
            q = FC_node.getQuat()
            yaw = np.arctan2(2.0*(q.w*q.z + q.x*q.y),
                             1.0 - 2.0*(q.y*q.y + q.z*q.z))
            yaw_err = abs(np.arctan2(np.sin(yaw), np.cos(yaw)))   # |ψ − 0|, wrapped
            prev_enu, prev_t = cur_enu, now
            if pos_err <= IC_POS_TOL and speed <= IC_VEL_TOL and yaw_err <= IC_YAW_TOL:
                stable += 1
                if stable >= STABLE_HITS:
                    converged_at = (k + 1) * 0.02
                    print(f"[landing_test] IC converged at t={converged_at:.2f}s "
                          f"(pos_err={pos_err:.3f} m, speed={speed:.3f} m/s, "
                          f"yaw_err={np.rad2deg(yaw_err):.2f}°)")
                    break
            else:
                stable = 0
        if converged_at is None:
            print(f"[landing_test] IC convergence TIMEOUT after 15 s "
                  f"(last pos_err={pos_err:.3f} m, speed={speed:.3f} m/s, "
                  f"yaw_err={np.rad2deg(yaw_err):.2f}°)")
            # Hard-abort instead of proceeding with a moving/off-target drone.
            # Empirically: PX4 SITL occasionally fails to settle during the
            # convergence loop (drone oscillates at ±2 m / ±2 m/s for 15s).
            # If we engage the controller from that state, the boosted K_ri ×10
            # integrator winds up around the moving setpoint and turns into a
            # 5+ m runaway. Better to abort and let the retry wrapper try a
            # fresh SITL boot than to fly garbage.
            raise RuntimeError(f"IC convergence timeout (pos_err={pos_err:.2f} m, "
                               f"speed={speed:.2f} m/s) — aborting before controller engage")

        # 1 s of holding the (recomputed) setpoint to let residual velocity damp.
        for _ in range(50):
            drone_enu  = pose_node.getPose().UAV.position
            target_enu = pose_node.getPose().target.position
            ned_pos    = FC_node.getPosBody()
            target_n = ned_pos.x_m + (target_enu.y + iy - drone_enu.y)
            target_e = ned_pos.y_m + (target_enu.x + ix - drone_enu.x)
            target_d = ned_pos.z_m - (target_enu.z + iz - drone_enu.z)
            await FC_node.send_position_ned(target_n, target_e, target_d, 0.0)
            await asyncio.sleep(0.02)
        cur_z = FC_node.getPosBody().z_m       # for the brief PID warmup below

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
        # Short warmup — just enough to fill the 4-deep smoothing deques.
        # Hold the last marker-aligned NED setpoint so drone stays put.
        print("[landing_test] PID warmup (100 ms — fills deques only)…")
        for _ in range(5):                        # 5 * 20 ms = 100 ms
            await FC_node.send_position_ned(target_n, target_e, target_d, 0.0)
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