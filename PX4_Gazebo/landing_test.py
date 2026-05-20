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
# Override via env var INITIAL_DRONE_ENU="x,y,z" (Gazebo ENU). Used by
# run_multi_ic_landing.sh to sweep the MATLAB Multi_init_cond IC list:
#   (0,0,5) (2,2,5) (2,-2,5) (2,2,7) (2,2,3)
_ic_env = os.environ.get("INITIAL_DRONE_ENU", "0.0,0.0,5.0")
INITIAL_DRONE_ENU = tuple(float(v) for v in _ic_env.split(","))
TAKEOFF_HEIGHT = INITIAL_DRONE_ENU[2]   # lift to IC altitude before flying to IC xy
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
    rates = np.array(cmd[:3], dtype=float)
    # Per-axis rate-zero env vars for axis-isolation diagnostics.
    # Each LANDING_NO_W* zeroes the corresponding body rate command.
    # LANDING_VERTICAL_ONLY is kept as shorthand for NO_WX=NO_WY=1.
    if (os.environ.get("LANDING_VERTICAL_ONLY", "0") == "1"
            or os.environ.get("LANDING_NO_WX", "0") == "1"):
        rates[0] = 0.0
    if (os.environ.get("LANDING_VERTICAL_ONLY", "0") == "1"
            or os.environ.get("LANDING_NO_WY", "0") == "1"):
        rates[1] = 0.0
    if os.environ.get("LANDING_NO_WZ", "0") == "1":
        rates[2] = 0.0
    return np.append(RAD2DEG * rates, thrust_norm)

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
    SOFT_PRECISE = {}

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
        print(f"[landing_test] Hover to MATLAB-IC ENU ({ix},{iy},{iz})  (+ tilt check)")
        IC_POS_TOL  = 0.5
        IC_VEL_TOL  = 0.2                          # tightened 0.3 → 0.2
        IC_YAW_TOL  = np.deg2rad(2.0)
        IC_TILT_TOL = np.deg2rad(1.5)              # NEW: |roll|, |pitch| ≤ 1.5°
        STABLE_HITS = 20                           # tightened 10 → 20 (0.4 s of stability)
        MAX_ITERS   = 750                          # 15 s @ 50 Hz
        prev_enu    = pose_node.getPose().UAV.position
        prev_t      = time.perf_counter()
        stable      = 0
        converged_at = None
        target_n = target_e = target_d = 0.0
        yaw_err = 0.0
        tilt_err = 0.0
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
            # Roll, pitch, yaw from PX4 quaternion.
            # Roll  φ = atan2(2(wx + yz), 1 − 2(x² + y²))
            # Pitch θ = asin(2(wy − zx))
            # Yaw   ψ = atan2(2(wz + xy), 1 − 2(y² + z²))
            q = FC_node.getQuat()
            roll = np.arctan2(2.0*(q.w*q.x + q.y*q.z),
                              1.0 - 2.0*(q.x*q.x + q.y*q.y))
            sinp = float(np.clip(2.0*(q.w*q.y - q.z*q.x), -1.0, 1.0))
            pitch = np.arcsin(sinp)
            yaw = np.arctan2(2.0*(q.w*q.z + q.x*q.y),
                             1.0 - 2.0*(q.y*q.y + q.z*q.z))
            yaw_err  = abs(np.arctan2(np.sin(yaw), np.cos(yaw)))
            tilt_err = max(abs(roll), abs(pitch))           # |roll| AND |pitch| both ≤ tol
            prev_enu, prev_t = cur_enu, now
            if (pos_err  <= IC_POS_TOL  and speed   <= IC_VEL_TOL
                    and yaw_err <= IC_YAW_TOL and tilt_err <= IC_TILT_TOL):
                stable += 1
                if stable >= STABLE_HITS:
                    converged_at = (k + 1) * 0.02
                    print(f"[landing_test] IC converged at t={converged_at:.2f}s "
                          f"(pos_err={pos_err:.3f} m, speed={speed:.3f} m/s, "
                          f"yaw_err={np.rad2deg(yaw_err):.2f}°, "
                          f"tilt={np.rad2deg(tilt_err):.2f}°)")
                    break
            else:
                stable = 0
        if converged_at is None:
            print(f"[landing_test] IC convergence TIMEOUT after 15 s "
                  f"(last pos_err={pos_err:.3f} m, speed={speed:.3f} m/s, "
                  f"yaw_err={np.rad2deg(yaw_err):.2f}°, tilt={np.rad2deg(tilt_err):.2f}°)")
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
        # Termination: PX4's LandedState (via FC_node.LANDED, fused onboard
        # from accel/baro/gyro/EKF) — onboard-only, no Gazebo truth.
        # The earlier `d <= LANDING_HEIGHT` check used Gazebo ENU z and only
        # worked in sim; PX4's landed_state is what a real flight would use.
        #
        # ArUco detection drops when the marker corners spill outside the
        # image frame (drone too close, ≈ last 0.2 m). When that happens
        # we switch to an open-loop final descent: fixed gentle thrust below
        # hover, zero body rate setpoints, and wait for PX4 to report
        # LANDED. This is the standard pattern for marker-based landing —
        # close in with vision, last 20 cm with constant descent.
        FINAL_DESCENT_THRUST = 0.65          # below 0.738 hover → mild descent
        FINAL_DESCENT_TIMEOUT = 5.0          # seconds
        # Marker-loss grace: brief dropouts (1-2 frames) are common when the
        # marker briefly leaves FoV due to tilt or motion blur. Without a
        # grace period, a single dropped frame commits us to open-loop final
        # descent — which then drifts on residual lateral momentum. With a
        # grace, IBVS keeps sending the last valid attitude-rate command
        # while waiting for re-detection; only commit to final descent if
        # marker is genuinely gone for MARKER_LOSS_GRACE seconds.
        MARKER_LOSS_GRACE = float(os.environ.get("LANDING_MARKER_LOSS_GRACE", "1.0"))
        in_final_descent = False
        final_descent_t0 = None
        last_good_sys_cmd = None
        marker_lost_t0 = None

        while EC_node.is_alive() and not FC_node.LANDED:
            UAV_pose.append(pose_node.getPose().UAV)
            target_pose.append(pose_node.getPose().target)

            t_c.append(time_node.perf_counter() - start_time)
            if EC_node.TARGET_IS_VISIBLE and not in_final_descent:
                cmd = EC_node.getControlInput()
                sys_cmd = convert_2_sys_cmd(cmd)
                await FC_node.send_attitude_rate(*sys_cmd)  # FC BODY follows FRD
                u_cmd.append(cmd)
                last_good_sys_cmd = sys_cmd
                marker_lost_t0 = None
            elif (not in_final_descent
                  and last_good_sys_cmd is not None
                  and (marker_lost_t0 is None
                       or (time_node.perf_counter() - marker_lost_t0) < MARKER_LOSS_GRACE)):
                # Marker briefly lost — hold last valid command within grace.
                if marker_lost_t0 is None:
                    marker_lost_t0 = time_node.perf_counter()
                await FC_node.send_attitude_rate(*last_good_sys_cmd)
            else:
                # Marker lost beyond grace (or never seen) → final descent.
                # Hold zero body rate, push constant sub-hover thrust until
                # PX4 reports ON_GROUND.
                if not in_final_descent:
                    in_final_descent = True
                    final_descent_t0 = time_node.perf_counter()
                    print(f"[landing_test] Marker lost — final descent "
                          f"(thrust={FINAL_DESCENT_THRUST}) until LANDED.")
                await FC_node.send_attitude_rate(0.0, 0.0, 0.0, FINAL_DESCENT_THRUST)
                if (time_node.perf_counter() - final_descent_t0) > FINAL_DESCENT_TIMEOUT:
                    print(f"[landing_test] Final-descent timeout "
                          f"({FINAL_DESCENT_TIMEOUT}s) — PX4 never reported LANDED.")
                    break

            await asyncio.sleep(SLEEP_TIME)

        if FC_node.LANDED:
            print("Landed (PX4 LandedState or impact spike)")
            # Quick disarm: cut thrust + disarm motors to stop any post-touchdown
            # slide. The drone often has residual lateral velocity at impact;
            # without disarming, motors keep spinning at FINAL_DESCENT_THRUST
            # (or whatever the last attitude_rate cmd's thrust was) and the
            # drone slides on its gear. IC 5 slid 0.9 m east in 3 s of
            # post-touchdown PX4-pre-LANDED window. Disarming kills that.
            try:
                await FC_node.send_attitude_rate(0.0, 0.0, 0.0, 0.0)
                await asyncio.sleep(0.05)
                await FC_node.vehicle.action.disarm()
                print("[landing_test] Disarmed post-touchdown.")
            except Exception as e:
                print(f"[landing_test] Disarm failed (probably already disarmed by PX4): {e}")

        # ─── MATLAB soft-precise landing classification ─────────────────
        # MATLAB criteria (visualControl_IBVS_adaptive.m:337-345):
        #   precise = xy_err  <= 0.08 m     (lateral distance to target)
        #   soft    = rel_vel <= 0.2 m/s    (UAV–target relative speed)
        # Evaluated at touchdown (PX4 reports LANDED here, equivalent to
        # MATLAB's alt_above <= zf=0.2 m check).
        # XY-err uses Gazebo truth (no onboard equivalent without an external
        # ref); rel_vel uses PX4 NED velocity (the drone's onboard estimate).
        SOFT_PRECISE = {}
        try:
            drone_enu  = pose_node.getPose().UAV.position
            target_enu = pose_node.getPose().target.position
            xy_err = ((drone_enu.x - target_enu.x)**2 +
                      (drone_enu.y - target_enu.y)**2) ** 0.5
            v = FC_node.getVelBody()
            rel_vel = (v.x_m_s**2 + v.y_m_s**2 + v.z_m_s**2) ** 0.5
            precise = xy_err  <= 0.08
            soft    = rel_vel <= 0.2
            SOFT_PRECISE = dict(xy_err=xy_err, rel_vel=rel_vel,
                                precise=precise, soft=soft)
            tag = ("SOFT+PRECISE" if (soft and precise)
                   else "PRECISE-only" if precise
                   else "SOFT-only"   if soft
                   else "FAIL")
            print(f"[landing_test] Landing classification: {tag}  "
                  f"(xy_err={xy_err:.3f} m [≤0.08], "
                  f"rel_vel={rel_vel:.3f} m/s [≤0.2])")
        except Exception as e:
            print(f"[landing_test] Could not compute soft-precise metrics: {e}")

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
            gt_data = {"Start Time": start_time, "Time": t_c, "Start Pose": start_pose, "UAV Pose": UAV_pose, "Target Pose": target_pose,"Command": u_cmd, "SoftPrecise": SOFT_PRECISE}
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