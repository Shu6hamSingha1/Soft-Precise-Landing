# **************************************************************************
# Changed class and node name used in gz_subscriber
# Added time node for clock synchronization with simulator world
# Used simulator time instead of system time
# Added target tracking functionality
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
from gz_subscriber import GZ_Subscriber, Pose_Node, Clock_Node
from numerical_methods import RK5
from controller import Controller

# Setup variables
FENCE = (-5.0, 5.0, -5.0, 5.0, -5.0, 5.0) # xmin, xmax, ymin, ymax, zmin, zmax -- bounding box all values in m

# Companion Computer Details:
CAPTURE_RATE = 60 # Capture Rate = {90, 120, 200}
RESOLUTION = (640, 480)
SLEEP_TIME = 1/200
REF_RAD_OPT_FLOW = float(os.environ.get("LANDING_REF_RAD_OPT_FLOW", "-0.42"))  # BAKED -0.42 2026-06-20: MATLAB VDF-ASMC manuscript value (vdf_params h_rd=-0.42; Table S1 locked) -> parity with the manuscript-combined formulation now baked. -0.42 gives softer touchdown (vel 0.37/land). (Earlier -0.3 was a 2026-06-13 back-mapped-era decision; the lateral-exposure caveat was a back-mapped-wall concern, now superseded by the combined surface.)
# Desired image features [hx, hy, s, alpha]. alpha (s_d[3]) is the DESIRED marker
# orientation and MUST match the board's as-seen alpha at the aligned hover — else the
# controller slews alpha ~85° at engage (the IC1/IC2-5 divergence cause, found
# 2026-06-04). Read the "MEASURED board alpha" print from a run, then set DES_ALPHA_DEG.
DES_IMG_FEATURE_PARAM = np.array([0.0, 0.0, 1.0,
                                  np.deg2rad(float(os.environ.get("DES_ALPHA_DEG", "0.0")))])

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
INITIAL_YAW_DEG = float(os.environ.get("INITIAL_YAW_DEG", "0.0"))
INITIAL_YAW_RAD = np.deg2rad(INITIAL_YAW_DEG)
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
    #
    # Slope tightened 2026-06-01 from 1/45 to 1/42.3 so that 1 N of B_T
    # produces exactly 1/mass m/s² of body-z accel (Newton's law). Empirical
    # verification on n=7 input-cal runs: TEL/(T_u/mass) gain went from
    # 0.937 (old slope) to ~1.00 (new slope). See memory
    # feedback_input_cal_thrust_units.
    #
    # PX4 throttle range = [0, 1]; clip to avoid invalid commands.
    thrust_norm = float(np.clip(0.738 - cmd[3] / 42.3, 0.0, 1.0))
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
        # IC tolerances: 2026-05-21 analysis showed σ0 (initial SMC sliding
        # variable) predicts xy_end at ρ=+0.77 with only 0.4% σ0 variance.
        # The savgol-filtered optic flow at engagement is set by the last ~7
        # samples of pre-engagement motion, so tighter IC settling shrinks
        # σ0 spread. Env-overridable for experimentation.
        IC_POS_TOL  = float(os.environ.get("LANDING_IC_POS_TOL",  "0.5"))
        IC_VEL_TOL  = float(os.environ.get("LANDING_IC_VEL_TOL",  "0.5"))
        IC_YAW_TOL  = np.deg2rad(float(os.environ.get("LANDING_IC_YAW_TOL_DEG", "5.0")))
        IC_TILT_TOL = np.deg2rad(float(os.environ.get("LANDING_IC_TILT_TOL_DEG", "3.0")))
        STABLE_HITS = int(os.environ.get("LANDING_IC_STABLE_HITS", "20"))
        MAX_ITERS   = int(os.environ.get("LANDING_IC_BUDGET_S", "30")) * 50
                                                    # 30 s @ 50 Hz (was 15 s; bumped
                                                    # to absorb slow-settle days)
        prev_enu    = pose_node.getPose().UAV.position
        prev_t      = time.perf_counter()
        stable      = 0
        _spd_hist   = []   # (x,y,z,t) window for jitter-robust speed (single-frame Δpos/dt spikes on GT bridge jitter)
        converged_at = None
        target_n = target_e = target_d = 0.0
        yaw_err = 0.0
        tilt_err = 0.0
        yaw_cmd_deg = float(INITIAL_YAW_DEG)   # servo'd to null TRUE yaw (below)

        def _servo_true_yaw(yc):
            # Drift-compensated yaw for the IC rig: ramp the NED setpoint `yc` so
            # the TRUE (Gazebo) yaw -> INITIAL_YAW, instead of commanding the EKF
            # yaw (which has drifted ~77deg by descent start). The controller never
            # uses compass yaw (it uses image alpha); the rig was holding/gating on
            # the EKF yaw -> drone started the descent badly yawed -> psi_d->180.
            # Truth is allowed for test setup (feedback_scale_free_depth_free).
            # GT_yaw_ENU ≈ -(yaw_cmd_NED) + drift (ENU/NED anti-corr r=-0.99), so
            # d(GT)/d(cmd) = -1: to reduce e>0 you must INCREASE the NED command. The
            # old -0.05 had the WRONG sign (drifted -> 30s timeout at ~83° yaw_err);
            # the +0.5 tried earlier "diverged" only because that gain raced past the
            # drone's ~45°/s yaw-rate. Fix: + sign, moderate gain, per-iter rate-CLAMP
            # so the setpoint stays trackable (~50°/s at 50 Hz). Env-tunable (set K<0
            # if a world's ENU/NED sign differs). MUST be applied in the IC loop AND
            # every post-convergence hold/warmup send, else those un-align the yaw.
            if os.environ.get("IC_YAW_TARGET", "gt") == "alpha":
                # Hold the drone BOARD-SQUARE: drive the (drift-free, marker-relative)
                # perception alpha -> 0. The board is square at alpha 0 (pre-takeoff
                # measured ≈0° across runs); this keeps it square from spawn through
                # engage despite compass drift, so the controller starts at e_a≈0
                # (DES_ALPHA=0). d(alpha)/d(yaw_cmd)<0 (same as GT yaw) -> + sign.
                _fp = EC_node._img_node.getImgFeatureParam()
                if _fp is None or len(_fp) < 4 or not np.isfinite(_fp[3]):
                    return yc          # no marker this frame -> hold last command
                _e = float(_fp[3])     # want alpha -> 0
            else:
                _q = pose_node.getPose().UAV.orientation
                _y = np.arctan2(2.0*(_q.w*_q.z + _q.x*_q.y),
                                1.0 - 2.0*(_q.y*_q.y + _q.z*_q.z))
                _e = np.arctan2(np.sin(_y - INITIAL_YAW_RAD), np.cos(_y - INITIAL_YAW_RAD))
            _k = float(os.environ.get("IC_YAW_SERVO_K", "0.2"))
            _dmax = float(os.environ.get("IC_YAW_SERVO_DMAX_DEG", "0.3"))
            return yc + float(np.clip(np.rad2deg(_k * _e), -_dmax, _dmax))
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
            yaw_cmd_deg = _servo_true_yaw(yaw_cmd_deg)   # drift-compensate the yaw
            await FC_node.send_position_ned(target_n, target_e, target_d, yaw_cmd_deg)
            await asyncio.sleep(0.02)
            # Convergence check on Gazebo truth (post-setpoint sample).
            cur_enu = pose_node.getPose().UAV.position
            now = time.perf_counter()
            dt = max(now - prev_t, 1e-3)
            # Jitter-robust speed: displacement over a ~6-frame window / window-time.
            # Single-frame Δpos/dt spikes to >1 m/s on GT bridge jitter even when the
            # drone is still, which kept resetting the STABLE_HITS counter despite a
            # settled hover (see feedback_gt_noise_uniform_dt).
            _spd_hist.append((cur_enu.x, cur_enu.y, cur_enu.z, now))
            if len(_spd_hist) > 6:
                _spd_hist.pop(0)
            _x0, _y0, _z0, _t0s = _spd_hist[0]
            _win_dt = max(now - _t0s, 1e-3)
            speed = ((cur_enu.x - _x0)**2 + (cur_enu.y - _y0)**2
                     + (cur_enu.z - _z0)**2) ** 0.5 / _win_dt
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
            # Gate yaw on TRUTH (Gazebo), not the drifted EKF yaw (q). roll/pitch
            # stay on the EKF — they don't drift and feed the same V-frame leveling.
            if os.environ.get("IC_YAW_TARGET", "gt") == "alpha":
                # Gate on board-square (|alpha|≈0), matching the alpha-hold servo. GT
                # yaw is NOT 0 here (the drone faces the board, not North).
                _fpg = EC_node._img_node.getImgFeatureParam()
                yaw_err = (abs(float(_fpg[3]))
                           if (_fpg is not None and len(_fpg) > 3 and np.isfinite(_fpg[3]))
                           else np.pi)
            else:
                _gtq2 = pose_node.getPose().UAV.orientation
                _gt_yaw2 = np.arctan2(2.0*(_gtq2.w*_gtq2.z + _gtq2.x*_gtq2.y),
                                      1.0 - 2.0*(_gtq2.y*_gtq2.y + _gtq2.z*_gtq2.z))
                yaw_err  = abs(np.arctan2(np.sin(_gt_yaw2 - INITIAL_YAW_RAD),
                                          np.cos(_gt_yaw2 - INITIAL_YAW_RAD)))
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
            print(f"[landing_test] IC convergence TIMEOUT after {MAX_ITERS*0.02:.0f} s "
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

        # 1 s hold to damp residual velocity (yaw-servo active), then 0.5 s of FROZEN
        # yaw so the drone settles and the board alpha stabilizes (the servo's
        # drift-correction rotation otherwise sweeps alpha ±30°). Then AUTO-SET the
        # desired alpha s_d[3] to that settled value, so the controller descends
        # HOLDING the current marker-relative heading instead of slewing ~60-85° at
        # engage (the IC1/IC2-5 divergence cause, 2026-06-04). The marker sits ~60-85°
        # off the drone's North-aligned heading and that offset varies per rep with the
        # compass drift, so a fixed DES_ALPHA can't match it — auto-set does. Env
        # DES_ALPHA_AUTO=0 disables (then the fixed DES_ALPHA_DEG is used).
        _alpha_meas = []
        for _k in range(75):
            drone_enu  = pose_node.getPose().UAV.position
            target_enu = pose_node.getPose().target.position
            ned_pos    = FC_node.getPosBody()
            target_n = ned_pos.x_m + (target_enu.y + iy - drone_enu.y)
            target_e = ned_pos.y_m + (target_enu.x + ix - drone_enu.x)
            target_d = ned_pos.z_m - (target_enu.z + iz - drone_enu.z)
            yaw_cmd_deg = _servo_true_yaw(yaw_cmd_deg)   # hold board-square (alpha->0) through engage
            await FC_node.send_position_ned(target_n, target_e, target_d, yaw_cmd_deg)
            await asyncio.sleep(0.02)
            if _k >= 50:
                _fp = EC_node._img_node.getImgFeatureParam()
                if _fp is not None and len(_fp) > 3 and np.isfinite(_fp[3]):
                    _alpha_meas.append(float(_fp[3]))
        if _alpha_meas:
            _med = float(np.arctan2(np.median(np.sin(_alpha_meas)),
                                    np.median(np.cos(_alpha_meas))))   # circular median
            _std = float(np.rad2deg(np.std(_alpha_meas)))
            if os.environ.get("DES_ALPHA_AUTO", "0") == "1":
                # NOTE: default OFF. Holding the current heading does NOT land with the
                # board square to the camera (user requirement). Desired alpha must be
                # 0 (board at 0° to camera at touchdown); the drone aligns to it.
                EC_node._s_d[3] = _med
                print(f"[landing_test] AUTO-SET desired alpha s_d[3] = {np.rad2deg(_med):.1f}° "
                      f"(settled, n={len(_alpha_meas)}, std={_std:.1f}°) — hold heading, no slew")
            else:
                print(f"[landing_test] board alpha (settled) = {np.rad2deg(_med):.1f}° std={_std:.1f}°; "
                      f"using fixed DES_ALPHA_DEG s_d[3]={np.rad2deg(DES_IMG_FEATURE_PARAM[3]):.1f}°")
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
            yaw_cmd_deg = _servo_true_yaw(yaw_cmd_deg)   # keep TRUE yaw aligned
            await FC_node.send_position_ned(target_n, target_e, target_d, yaw_cmd_deg)
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
        # NOTE (2026-06-03): an "IBVS handoff altitude" knob briefly existed here,
        # based on misreading MATLAB's zf=0.2m as a controller validity envelope.
        # zf is the LANDING GEAR HEIGHT — MATLAB's termination at 0.2m IS gear
        # contact (same event as PX4's LandedState). The controller is designed
        # to control through touchdown; it must not hand off early. REVERTED.
        #
        # Stale-streak commitment (env-gated, 2026-06-03, SCALE-FREE).
        # Failure mode it addresses (5/24 reps, all catastrophic): detection
        # breaks down at gear contact, the KLT fallback feeds extrapolated
        # corners, the controller chases a phantom error, climbs off a
        # near-perfect landing, and crashes. KLT-cap tuning cannot fix this
        # without destroying routine dropout bridging (proven: caps 3/10/20).
        # The commitment decouples the two: bridging stays (cap 20), but when
        # staleness PERSISTS (>> any routine dropout) while the last FRESH
        # marker extent indicated touchdown proximity (marker filling the
        # image — an image quantity, no depth), commit to the open-loop
        # touchdown instead of chasing. Tagged terminal_perception_loss,
        # NOT target_lost (losing the marker under the airframe at touchdown
        # is expected physics, not a tracking failure).
        #   LANDING_STALE_COMMIT_EXTENT  px threshold; 0 = feature OFF (default)
        #   LANDING_STALE_COMMIT_TIME    persistence required (s) after stale fires
        STALE_COMMIT_EXTENT = float(os.environ.get("LANDING_STALE_COMMIT_EXTENT", "0.0"))
        STALE_COMMIT_TIME = float(os.environ.get("LANDING_STALE_COMMIT_TIME", "0.15"))
        # ── Proximity commit (2026-06-11): clean-touchdown fix ──
        # At deck height (alt ≲ 0.1 m) 1/Z amplifies the residual lateral error into violent
        # tilt+thrust bursts (a_u_xy 22-71, B_T −13..−16) → the drone HOPS off the ground
        # (0.05→0.26 m) instead of settling: the controller image-regulates into ground contact.
        # Fix: when the marker extent (image-only, scale-free proximity proxy) exceeds the
        # threshold on consecutive FRESH frames, commit to the open-loop vertical settle
        # (zero rates + sub-hover thrust → PX4 LANDED → disarm) instead. Data (2 runs):
        # mid-descent extent ≤72-103 px (alt>0.4 m) vs 142-166 px at the deck → threshold ~130.
        # SUCCESS path (not target_lost).
        #   LANDING_COMMIT_EXTENT  px threshold; 0 = OFF (default)
        #   LANDING_COMMIT_FRAMES  consecutive fresh frames required (default 3)
        #   LANDING_COMMIT_SEN     max |s_e_n| (centered gate): commit only when close
        #                          AND centered — an extent threshold alone fired at
        #                          ~0.5 m while 0.72 m off-center (open-loop from there
        #                          → precision loss). Image-only, scale-free.
        COMMIT_EXTENT = float(os.environ.get("LANDING_COMMIT_EXTENT", "0.0"))
        COMMIT_FRAMES = int(os.environ.get("LANDING_COMMIT_FRAMES", "3"))
        COMMIT_SEN = float(os.environ.get("LANDING_COMMIT_SEN", "0.35"))
        commit_streak = 0
        in_final_descent = False
        final_descent_t0 = None
        last_good_sys_cmd = None
        marker_lost_t0 = None
        terminal_perception_loss = False
        stale_streak_t0 = None
        last_fresh_extent = 0.0
        # 2026-05-21: per user direction, marker loss beyond grace counts as
        # a TARGET_LOST landing failure rather than a graceful soft-precise
        # success via fallback. The fallback (zero-rate + fixed thrust) still
        # exists to bring the drone safely to ground, but the landing is
        # tagged as a failure regardless of touchdown xy/vel.
        target_lost = False

        # ── Hover / descent-stall watchdog (added 2026-06-07) ──────────────────
        # The control loop below terminates ONLY on FC_node.LANDED or the controller
        # thread dying — it has NO time bound. A non-descending drone (e.g. a
        # kappa-runaway drives a_v upward -> hover) therefore sits airborne until
        # run_aruco_landing.sh's external `timeout 180` SIGKILLs it, yielding no
        # result and burning ~3 min. This watchdog aborts a stuck/hovering descent
        # fast. Scale-free: Gazebo-truth altitude is used for the TEST abort only,
        # never in the control law. Env-tunable; set CONTROL_TIMEOUT_S huge to disable.
        CONTROL_TIMEOUT_S = float(os.environ.get("LANDING_CONTROL_TIMEOUT_S", "90.0"))
        HOVER_STALL_S     = float(os.environ.get("LANDING_HOVER_STALL_S", "25.0"))
        HOVER_STALL_DZ    = float(os.environ.get("LANDING_HOVER_STALL_DZ", "0.3"))
        _ctrl_t0    = time_node.perf_counter()
        _best_alt   = None     # lowest ENU altitude reached so far (descent progress)
        _stall_t0   = None

        while EC_node.is_alive() and not FC_node.LANDED:
            UAV_pose.append(pose_node.getPose().UAV)
            target_pose.append(pose_node.getPose().target)

            t_c.append(time_node.perf_counter() - start_time)

            # ── hover / descent-stall watchdog (state set above the loop) ──
            _now_w = time_node.perf_counter()
            _alt_w = UAV_pose[-1].position.z          # ENU up; smaller = lower (descending)
            if _best_alt is None or _alt_w < _best_alt - HOVER_STALL_DZ:
                _best_alt = _alt_w                    # fresh >DZ of descent progress
                _stall_t0 = _now_w
            if _stall_t0 is None:
                _stall_t0 = _now_w
            if (_now_w - _ctrl_t0) > CONTROL_TIMEOUT_S:
                raise RuntimeError(
                    f"control timeout: no landing in {CONTROL_TIMEOUT_S:.0f}s "
                    f"(alt={_alt_w:.2f} m) — drone stuck/hovering, aborting")
            if (_now_w - _stall_t0) > HOVER_STALL_S:
                raise RuntimeError(
                    f"descent stall: no >{HOVER_STALL_DZ:.2f} m descent in "
                    f"{HOVER_STALL_S:.0f}s (alt={_alt_w:.2f} m) — hovering, aborting")

            # Intervention 3 (2026-05-22): treat FEATURE_IS_STALE the same as
            # marker-loss — if img_data has been extrapolating for STALE_THRESH+
            # consecutive frames, route to the grace-hold path instead of
            # letting the controller act on stale/extrapolated feature data.
            feature_fresh = (EC_node.TARGET_IS_VISIBLE
                             and not EC_node.FEATURE_IS_STALE)
            # ── Stale-streak commitment (see env knobs above; inert when EXTENT=0) ──
            if STALE_COMMIT_EXTENT > 0.0 and not in_final_descent:
                if feature_fresh:
                    last_fresh_extent = EC_node.MARKER_EXTENT_PX
                    stale_streak_t0 = None
                else:
                    now_t = time_node.perf_counter()
                    if stale_streak_t0 is None:
                        stale_streak_t0 = now_t
                    elif ((now_t - stale_streak_t0) >= STALE_COMMIT_TIME
                          and last_fresh_extent >= STALE_COMMIT_EXTENT):
                        in_final_descent = True
                        terminal_perception_loss = True
                        final_descent_t0 = now_t
                        print(f"[landing_test] Stale-streak commitment: detection lost "
                              f"{now_t - stale_streak_t0:.2f}s with last fresh marker extent "
                              f"{last_fresh_extent:.0f}px >= {STALE_COMMIT_EXTENT:.0f}px "
                              f"(touchdown proximity) -> open-loop touchdown "
                              f"[terminal perception loss, not a tracking failure]")
            # ── Proximity commitment (clean touchdown; inert when COMMIT_EXTENT=0) ──
            if COMMIT_EXTENT > 0.0 and not in_final_descent:
                if (feature_fresh and EC_node.MARKER_EXTENT_PX >= COMMIT_EXTENT
                        and EC_node.LATERAL_ERR_N <= COMMIT_SEN):
                    commit_streak += 1
                    if commit_streak >= COMMIT_FRAMES:
                        in_final_descent = True
                        final_descent_t0 = time_node.perf_counter()
                        print(f"[landing_test] Proximity commitment: marker extent "
                              f"{EC_node.MARKER_EXTENT_PX:.0f}px >= {COMMIT_EXTENT:.0f}px "
                              f"AND centered (|s_e_n|={EC_node.LATERAL_ERR_N:.2f} <= "
                              f"{COMMIT_SEN:.2f}) for {commit_streak} fresh frames -> "
                              f"open-loop vertical settle [clean touchdown, SUCCESS path]")
                else:
                    commit_streak = 0
            if feature_fresh and not in_final_descent:
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
                # Marker briefly lost OR feature stale — hold last valid command
                # within grace, give the detector a chance to re-acquire.
                if marker_lost_t0 is None:
                    marker_lost_t0 = time_node.perf_counter()
                await FC_node.send_attitude_rate(*last_good_sys_cmd)
            else:
                # Marker lost beyond grace (or never seen) → final descent.
                # Hold zero body rate, push constant sub-hover thrust until
                # PX4 reports ON_GROUND.
                if not in_final_descent:
                    in_final_descent = True
                    target_lost = True
                    final_descent_t0 = time_node.perf_counter()
                    print(f"[landing_test] Marker lost beyond grace — TARGET_LOST. "
                          f"Open-loop fallback (thrust={FINAL_DESCENT_THRUST}) "
                          f"to bring drone down safely; landing is tagged as failure.")
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
            # If marker was lost beyond grace, landing is a failure regardless
            # of touchdown xy/vel (per user direction 2026-05-21). The xy/vel
            # numbers are still recorded for diagnostics.
            # Precision tolerance raised 0.08 -> 0.10 m per user (2026-06-03).
            PRECISE_TOL = float(os.environ.get("LANDING_PRECISE_TOL", "0.10"))
            if target_lost:
                precise = False
                soft    = False
                tag = "TARGET_LOST"
            else:
                precise = xy_err  <= PRECISE_TOL
                soft    = rel_vel <= 0.2
                tag = ("SOFT+PRECISE" if (soft and precise)
                       else "PRECISE-only" if precise
                       else "SOFT-only"   if soft
                       else "FAIL")
                if terminal_perception_loss:
                    tag += " [terminal perception loss]"
            SOFT_PRECISE = dict(xy_err=xy_err, rel_vel=rel_vel,
                                precise=precise, soft=soft,
                                target_lost=target_lost,
                                terminal_perception_loss=terminal_perception_loss)
            print(f"[landing_test] Landing classification: {tag}  "
                  f"(xy_err={xy_err:.3f} m [≤{PRECISE_TOL}], "
                  f"rel_vel={rel_vel:.3f} m/s [≤0.2]"
                  f"{', target lost mid-flight' if target_lost else ''})")
        except Exception as e:
            print(f"[landing_test] Could not compute soft-precise metrics: {e}")

        # Post-touchdown video tail: with IMG_RECORD=1 the img-thread keeps writing frames
        # while the pipeline is alive, so delaying teardown captures the settle AFTER the
        # touchdown. Placed AFTER the soft-precise classification so the 5 s of settled
        # hover can't alter the eval metrics. IMG_RECORD_TAIL_S to tune; only when recording.
        if os.environ.get("IMG_RECORD", "0") == "1":
            _tail = float(os.environ.get("IMG_RECORD_TAIL_S", "5.0"))
            print(f"[landing_test] IMG_RECORD: capturing {_tail:.0f} s post-touchdown video tail…")
            await asyncio.sleep(_tail)

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

            # Create a directory named based on timestamp.
            # LANDING_OUT_BASE routes a VALIDATION landing elsewhere (e.g.
            # validation_data/output_landing) without polluting the default test set.
            _land_base = os.environ.get(
                "LANDING_OUT_BASE",
                "/home/shubham/Soft-Precise-Landing/PX4_Gazebo/test_data/Landing_Test")
            dir_name = f"{_land_base}/{timestamp}"
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