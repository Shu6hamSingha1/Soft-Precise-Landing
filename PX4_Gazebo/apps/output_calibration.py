# **************************************************************************
# Changed class and node name used in gz_subscriber
# Added time node for clock synchronization with simulator world
# Used simulator time instead of system time
# Sending sinusoidal velocity commands to FC for calibration
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
# from numerical_methods import RK5
# from controller import Controller

# Companion Computer Details — MUST match landing_test.py to keep the
# calibration regime identical to the operational regime.  Any divergence
# (sleep rate, resolution, filter window, IMG_PROCESSOR config) produces a
# cal that doesn't transfer to landing.
CAPTURE_RATE = 60 # Capture Rate = {90, 120, 200}
RESOLUTION = (640, 480)        # match landing_test.py:25
SLEEP_TIME = 1/200             # match landing_test.py:26 (was 1/30 — 6.7× slower)
REF_RAD_OPT_FLOW = -0.30
DES_IMG_FEATURE_PARAM = np.array([0.0, 0.0, 1.0, 0.0])

# Calibration sweep amplitudes (reduced from 1.0 m / 15 deg to keep PX4
# preflight checks happy. Empirically the previous 1.0 m sinusoid tripped
# "High Accelerometer Bias" / "horizontal velocity unstable" → failsafe →
# OFFBOARD drop → send_position_ned hang.)
CALIB_AMP_XY = 0.35    # 2026-05-30: cut from 0.7 → 0.35. At 0.5 Hz the
                       # peak tilt for 0.7 m amplitude is ~35°, which moves
                       # the marker out of FoV during the x/y phases →
                       # 100% zero-row blackouts mid-phase. 0.35 m → ~10-15°
                       # peak tilt, marker stays detectable.
CALIB_AMP_Z  = float(os.environ.get("CALIB_AMP_Z", "1.2"))
                          # 2026-05-29: bumped from 0.3 to 0.6m to lift z optic-flow
                          # signal out of the noise floor (std had been 0.003).
                          # 2026-06-01: bumped 0.6 → 1.2m. n=9-run cal showed flow_z
                          # 95% CI = ±30% (much wider than other axes' ±4%) because
                          # 0.6m is still small. 1.2m gives peak vert vel ~3.77 m/s
                          # and peak accel ~5.9 m/s² (~0.6g extra thrust) — within
                          # PX4 envelope at h≈3m. Marker stays in FoV across
                          # altitude range 1.8-4.2m. Reduce via env var if PX4
                          # failsafes start tripping on a particular drone build.
CALIB_AMP_YAW_DEG = float(os.environ.get("CALIB_AMP_YAW_DEG", "30.0"))
                          # 2026-06-01: bumped 10° → 30°. n=9-run cal showed ω_z
                          # 95% CI = ±9% (vs ±3-4% for other ω axes) — yaw signal
                          # was small. 30° gives peak yaw rate ~94°/s = 1.64 rad/s
                          # (within typical MAX_YAW_RATE). Marker stays centered.

# PHASED-EXCITATION redesign 2026-05-29. The previous lockstep `pos_cmd = AMP*val`
# on all four axes (x, y, z, yaw) made the four input channels 100% correlated,
# so lstsq could not separate per-axis scale factors — z-axis scale came back
# NaN in 4/5 fresh runs (May 29). Now: each axis is excited ALONE in sequence
# (x then y then z then yaw), with a tagged "phase" log so analyze_calibration
# can derive each cal factor from a clean single-axis segment.
#
# Each phase lasts CALIB_PHASE_S seconds. Sine frequency CALIB_FREQ_HZ=0.5
# → 4 cycles per 8s phase. Setpoint rate now matches landing (200 Hz) — the
# IMG_PROCESSOR / image-callback rate stays at the Gazebo capture rate (~60 Hz)
# regardless of how fast we send setpoints.
CALIB_PHASE_S    = float(os.environ.get("CALIB_PHASE_S", "8.0"))    # seconds per single-axis phase
CALIB_SETTLE_S   = float(os.environ.get("CALIB_SETTLE_S", "1.0"))   # seconds of zero command between phases
CALIB_FREQ_HZ    = float(os.environ.get("CALIB_FREQ_HZ", "0.5"))    # excitation frequency
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
    phase_log = []           # per-sample tag: 'x', 'y', 'z', 'yaw', or 'settle'
    start_pose = None
    t_c = []
    cmd = []
    # Phased excitation: build a script of (phase_name, x_fn, y_fn, z_fn, yaw_fn, duration)
    # Each fn takes phase-relative time `tau` (0..duration) and returns the offset.
    zero = lambda tau: 0.0
    sin_x   = lambda tau: CALIB_AMP_XY * np.sin(2*np.pi*CALIB_FREQ_HZ * tau)
    sin_y   = lambda tau: CALIB_AMP_XY * np.sin(2*np.pi*CALIB_FREQ_HZ * tau)
    sin_z   = lambda tau: CALIB_AMP_Z  * np.sin(2*np.pi*CALIB_FREQ_HZ * tau)
    sin_yaw = lambda tau: CALIB_AMP_YAW_DEG * np.sin(2*np.pi*CALIB_FREQ_HZ * tau)
    # Phase order: settle → yaw → x → y → z. Yaw is FIRST after the initial
    # settle so even if PX4 drops the MAVSDK connection mid-sweep (observed
    # during the more-aggressive z phase), we keep clean ω_z data. z is LAST
    # because it's the most likely to trip PX4 failsafes.
    #
    # CALIB_PHASES env var (comma-separated): select which active phases to
    # include. Default 'yaw,x,y,z' = all. For z-only recovery (when the full
    # sweep dies mid-y leaving z phase unsampled), use CALIB_PHASES=z and the
    # script will skip yaw/x/y entirely — z then runs early, within the ~22 s
    # PX4-MAVSDK uptime budget.
    _selected = os.environ.get("CALIB_PHASES", "yaw,x,y,z").split(",")
    _selected = [s.strip() for s in _selected if s.strip()]
    _phase_fns = {
        'yaw': (zero, zero, zero, sin_yaw),
        'x':   (sin_x, zero, zero, zero),
        'y':   (zero, sin_y, zero, zero),
        'z':   (zero, zero, sin_z, zero),
    }
    phase_script = [('settle', zero, zero, zero, zero, CALIB_SETTLE_S)]
    for _ph in _selected:
        if _ph not in _phase_fns:
            print(f"  [warn] unknown phase {_ph!r}; skipping"); continue
        fx, fy, fz, fyaw = _phase_fns[_ph]
        phase_script.append((_ph, fx, fy, fz, fyaw, CALIB_PHASE_S))
        phase_script.append(('settle', zero, zero, zero, zero, CALIB_SETTLE_S))
    total_s = sum(p[5] for p in phase_script)
    print(f"[calib] phased excitation: {len(phase_script)} phases, total {total_s:.1f}s")

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
        
        img_node = ID.IMG_PROCESSOR(capRate=CAPTURE_RATE, resolution=RESOLUTION, time_keeper=time_node, controller=FC_node) # start the thread for onboard camera flow streaming

        if record != 'n':
            img_node.RECORD = True
            print("Starting with recording...")

        else:
            print("Starting without recording...!")

        # Default 5.0 m matches landing_test.py TAKEOFF_HEIGHT. Env-overridable
        # so we can derive/validate the calibration matrix at several altitudes
        # (test depth-invariance of M — the lateral/tilt axes are only excited
        # at the takeoff height, so multi-height sweeps are required to check it).
        _takeoff_h = float(os.environ.get("CALIB_TAKEOFF_HEIGHT", "5.0"))
        await FC_node.arm_and_takeoff(_takeoff_h)

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

        print("Starting calibration sweep (phased excitation)...")
        # Capture post-takeoff altitude so we can detect failsafe-driven descent.
        takeoff_z = FC_node.getPosBody().z_m

        for ph_name, fn_x, fn_y, fn_z, fn_yaw, duration in phase_script:
            print(f"  [phase] {ph_name:6s}  duration={duration:.1f}s")
            phase_t0 = time_node.perf_counter()
            n_steps = int(duration / SLEEP_TIME)
            for k in range(n_steps):
                if not img_node.is_alive() and FC_node.LANDED:
                    break

                # Bail on failsafe-driven descent.
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

                tau = time_node.perf_counter() - phase_t0   # phase-relative time
                pos_cmd = np.array([
                    fn_x(tau),
                    fn_y(tau),
                    takeoff_z + fn_z(tau),
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
                # Log RAW values (pre sensor_cal) so this script can be used to recalibrate.
                opt_flow_ang_vel.append(img_node.getRawOptFlowAngVel())
                img_feature_param.append(img_node.getRawImgFeatureParam())
                cmd.append(pos_cmd)
                phase_log.append(ph_name)

                await asyncio.sleep(SLEEP_TIME)
            else:
                # inner loop completed without break → next phase
                continue
            break   # outer loop break if inner broke

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
            gt_data = {"Start Time": start_time, "Time": t_c, "Start Pose": start_pose, "UAV Pose": UAV_pose, "Target Pose": target_pose, "Opt Flow Ang Vel": opt_flow_ang_vel, "Img Feature Params": img_feature_param, "Command": cmd, "Phase": phase_log}
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

        