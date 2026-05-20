# **************************************************************************
# PLASMC: Performance-constrained Leakage-type Adaptive Sliding-Mode Control
#
# Aligned to MATLAB reference:
#   MATLAB/Multi_init_cond/visualControl_IBVS_adaptive.m
#
# Scope of this port:
#   - Outer position loop (raw normalized pixel error -> PID -> V_ds_d)
#   - Middle adaptive-SMC loop on optical flow (kappa ODE, leakage form)
#   - Yaw adaptive SMC (kappa_a ODE) -> output u_a as yaw-rate setpoint to PX4
#   - Inner attitude/rate loop is delegated to PX4 (we ship body-rate + thrust)
#
# Notes on Gazebo vs MATLAB:
#   - Camera intrinsics: 640x480 @ fx=fy=270 (Gazebo SDF, hfov=1.74 rad).
#     MATLAB uses 320x240 @ f=135 (Constants.m). SAME hfov, so normalized
#     image coordinates and PLASMC gains are invariant. Only pixel-space
#     quantities scale 2x (rho_fov, etc.).
#     1280x960 was tested and rejected on 2026-05-12: Gazebo's native render
#     rate dropped to ~21 Hz with 90 ms ROS-bridge outliers.
#   - img_data._sensor_cal_s = diag(1/6, 1/6, 1, 1) is kept (user choice) pending
#     a recalibration pass via output_calibration.py.
# **************************************************************************
import os
import time
import numpy as np
from scipy.linalg import expm
from threading import Thread
from collections import deque

from numerical_methods import RK5, smooth4
import img_data as ID
from ahrs import Quaternion

SLEEP_TIME = 1/200
N_DIM = 3
e3 = np.eye(N_DIM)[:, 2]

mass = 2.114  # kg, Holybro X500 (matches MATLAB Constants.m: m=2.114)
g = 9.80      # m/s^2 (matches Gazebo aruco.sdf <gravity>0 0 -9.8</gravity>;
              # MATLAB Constants.m uses 9.81 internally — sub-0.1% difference,
              # the 9.80 value here is correct for SITL physics)

# Clamp |S| < 1 - S_MARGIN to keep log-barrier finite (MATLAB uses 0.05 margin)
S_MARGIN = 0.05


class Controller(Thread):
    def __init__(self, ref_rad_opt_flow, des_img_feature, time_keeper=time, controller=None, record='n'):
        Thread.__init__(self)
        self._h_ref = ref_rad_opt_flow
        # NOTE: previously had a soft-engagement ramp (PLASMC_HRD_RAMP_S) and
        # a lateral-error gate (PLASMC_HRD_GATE_ALPHA) that modulated h_ref
        # during the descent. Both removed 2026-05-18: ramp gave essentially
        # zero improvement (3.07 s vs 3.29 s, within noise) and the gate
        # caused catastrophic IC 5 failure (12 m runaway) — both because
        # varying h_ref makes h_d[z] non-steady, generates dh_d transients
        # that feed the SMC c-term destructively, and shrinking h_rd also
        # shrinks the (h_rd − dot(cross(w,s), e3))·s cross-coupling on x/y
        # which alters SMC stability. Direct MATLAB-style use of h_ref
        # is the cleanest.

        self._CONTROLLER_READY = False
        self._warmup_remaining = 0           # set by startController()
        self._STAY_OPEN = True
        self.TARGET_IS_VISIBLE = False

        self._FC = controller
        self._img_node = ID.IMG_PROCESSOR(time_keeper=time_keeper, controller=controller)
        if record != 'n':
            self._img_node.RECORD = True
            print("Starting with recording...")
        else:
            print("Starting without recording...!")

        self._time = time_keeper
        self._s_d = des_img_feature

        # ---------------- MATLAB-aligned gains ----------------
        # Normalized pixel-error half-range (MATLAB: K_ctrl.p_10 = [res(2)/2/f; res(1)/2/f])
        # For Gazebo 1280x960 @ f=540: ~[1.185, 0.889]
        self._p_10 = self._img_node.center / self._img_node.focal  # (2,)

        # Outer-loop PID on V_s_e_n (raw normalized pixel error)
        # MATLAB:  rp = diag(9.0, 9.0), ri = diag(0.1, 0.1), rd = diag(1.4375, 1.4375)
        # SITL:    K_ri boosted 10× (0.1 → 1.0). Diagnosis: the lateral PID
        # is conditionally unstable on SITL's noisier LK centroid — initial
        # noise perturbations grow into 1+ m drift in any direction (run-to-
        # run varies). MATLAB's tiny K_ri = 0.1 can't correct steady drift.
        # Boosting K_ri 10× lets the integral term explicitly null persistent
        # offsets; the existing 5.0 anti-windup clamp on |is_e_n| limits runaway.
        # Env-var scalers stay in place so further tuning doesn't need a code
        # edit:
        #   PLASMC_KP_SCALE  (default 1.0)
        #   PLASMC_KD_SCALE  (default 1.0)
        #   PLASMC_KI_SCALE  (default 1.0)
        #   PLASMC_PID_SCALE (default 1.0) is the legacy uniform scaler,
        #     applied on top of the per-term scales.
        kp_scale  = float(os.environ.get("PLASMC_KP_SCALE",  "1.0"))
        kd_scale  = float(os.environ.get("PLASMC_KD_SCALE",  "1.0"))
        ki_scale  = float(os.environ.get("PLASMC_KI_SCALE",  "1.0"))
        pid_scale = float(os.environ.get("PLASMC_PID_SCALE", "1.0"))
        # Per-axis PID scalers (default 1.0). PID is 2D (x,y of feature
        # error in image plane); _X applies to axis 0, _Y to axis 1.
        # Note: image axis 0 (V-frame x) → PITCH, image axis 1 → ROLL.
        kp_x = float(os.environ.get("PLASMC_KP_X_SCALE", "1.0"))
        kp_y = float(os.environ.get("PLASMC_KP_Y_SCALE", "1.0"))
        ki_x = float(os.environ.get("PLASMC_KI_X_SCALE", "1.0"))
        ki_y = float(os.environ.get("PLASMC_KI_Y_SCALE", "1.0"))
        kd_x = float(os.environ.get("PLASMC_KD_X_SCALE", "1.0"))
        kd_y = float(os.environ.get("PLASMC_KD_Y_SCALE", "1.0"))
        self._K_rp = pid_scale * np.diag([kp_scale * kp_x * 9.0,
                                          kp_scale * kp_y * 9.0])
        self._K_ri = pid_scale * np.diag([ki_scale * ki_x * 1.0,
                                          ki_scale * ki_y * 1.0])
        self._K_rd = pid_scale * np.diag([kd_scale * kd_x * 1.4375,
                                          kd_scale * kd_y * 1.4375])
        if pid_scale != 1.0 or kp_scale != 1.0 or kd_scale != 1.0 or ki_scale != 1.0:
            print(f"[PLASMC] PID scales: P={kp_scale}, I={ki_scale}, D={kd_scale}, uniform={pid_scale}")

        # Per-axis env-var scalers. Each param has a uniform scalar
        # (PLASMC_<KEY>_SCALE, applied to all 3 axes) AND per-axis scalars
        # (PLASMC_<KEY>_{X,Y,Z}_SCALE, applied on top of the uniform). All
        # default to 1.0 except KAPPA0 uniform=1.25 (best from IC 1 sweep).
        def s(key, default="1.0"):
            return float(os.environ.get(f"PLASMC_{key}_SCALE", default))
        def per_axis(key, base, default="1.0"):
            """Return diag([sX*sU*base[0], sY*sU*base[1], sZ*sU*base[2]])"""
            sU = s(key, default)
            sX = s(f"{key}_X")
            sY = s(f"{key}_Y")
            sZ = s(f"{key}_Z")
            return np.array([sX*sU*base[0], sY*sU*base[1], sZ*sU*base[2]])

        # Optic-flow funnel (LOAD-BEARING: p_2_0, p_2_∞)
        self._gamma = np.diag(per_axis("XI2",   [0.2, 0.2, 0.2]))
        self._p_0   =          per_axis("P20",   [25.0, 25.0, 4.0])
        self._p_inf =          per_axis("P2INF", [2.5, 2.5, 1.5])
        # Optic-flow SMC (LOAD-BEARING: Omega)
        self._Omega = np.diag(per_axis("OMEGA", [0.05, 0.05, 0.025]))
        self._Gma   = np.diag(per_axis("GAMMA", [0.4375, 0.5, 0.75]))
        self._E     = np.diag(per_axis("E",     [1.0, 1.0, 1.0]))
        # Adaptive-gain ODE. PLASMC_N_Z is the legacy *absolute* scalar
        # for N[z] (default 0.02 — slowed from MATLAB's 0.05 for SITL).
        # Per-axis scalers (PLASMC_N_X_SCALE etc) multiply on top of this.
        N_z_abs = float(os.environ.get("PLASMC_N_Z", "0.02"))
        n_diag = per_axis("N", [0.02, 0.02, 0.02])
        n_diag[2] = N_z_abs * s("N") * s("N_Z")     # legacy override path
        self._N = np.diag(n_diag)
        self._P = np.diag(per_axis("P",      [1.5, 1.5, 5.0]))
        # KAPPA0 default 1.25 (best single-axis tune from IC 1 sweep)
        self._kappa_0 =        per_axis("KAPPA0", [0.125, 0.125, 0.25], "1.25")
        # Print any non-default scales (uniform + per-axis).
        _print_lines = []
        _keys = ["XI2","P20","P2INF","OMEGA","GAMMA","E","N","P","KAPPA0"]
        _defaults = {"KAPPA0": "1.25"}
        for k in _keys:
            v = s(k, _defaults.get(k, "1.0"))
            if v != 1.0 and k != "KAPPA0":           # KAPPA0=1.25 is now the default; only print if changed
                _print_lines.append(f"  {k:<8} = {v}")
            if k == "KAPPA0" and v != 1.25:
                _print_lines.append(f"  {k:<8} = {v}")
            for ax in ("X","Y","Z"):
                vx = s(f"{k}_{ax}")
                if vx != 1.0:
                    _print_lines.append(f"  {k}_{ax}{' '*(7-len(k))}= {vx}")
        if N_z_abs != 0.02: _print_lines.append(f"  N_Z(abs) = {N_z_abs}")
        if _print_lines:
            print(f"[PLASMC] tunable middle-loop scales:")
            for line in _print_lines: print(line)

        # Yaw adaptive SMC (ROBUST per supplement S3-A — sweep tunable but low impact)
        self._Omega_a    = s("YAW_OMEGA")  * 0.5
        self._Gma_a      = s("YAW_GAMMA")  * 0.5
        self._n_a        = s("YAW_N")      * 1.0
        self._p_a        = s("YAW_P")      * 2.0
        self._kappa_a_0  = s("YAW_KAPPA0") * 2.0
        self._E_a        = s("YAW_E")      * 3.0

        # FoV-margin cone clamp (acceleration conditioning)
        self._rho_fov_0   = s("RHOFOV0")   * np.array([290.0, 210.0])
        self._rho_fov_inf = s("RHOFOVINF") * np.array([80.0, 80.0])
        self._l_fov       = s("LFOV")      * 0.1
        self._theta_cap   = np.deg2rad(s("THETACAP") * 60.0)

        # Low-pass filter on inertial accel (MATLAB: tau_ia = 0.08 s)
        self._tau_ia = 0.08

        # Anti-windup clamps (MATLAB izeta_2 clamped to ±5; others heuristic)
        self._iV_s_e_n_clamp = 5.0
        # Per-component sliding-surface integral anti-windup cap. Matches
        # Supplement S2-D item 2 (corrected): |∫ζ_{2k}dτ| ≤ 5, chosen so the
        # integral contribution X·∫ζ to σ stays ≤ λ_max(X)·5 ≤ 0.25, within
        # the boundary-layer thickness ε_k = 1 used in the simulations.
        self._izeta_clamp = 5.0
        self._ie_a_clamp = 2.0

        # Inner-attitude PD (rate-side; PX4's rate ctrl does the actuator work)
        # K_ei/K_ed default to zero — angle error already feeds the rate
        # setpoint, integration left to PX4 if needed.
        self._K_ep = np.diag([5.0, 5.0, 5.0])
        self._K_ei = np.diag([0.0, 0.0, 0.0])
        self._K_ed = np.diag([0.0, 0.0, 0.0])

        # Virtual-compass + geometric SO(3) attitude path
        # (manuscript Section III-B1/B2: psi_d evolves from image-based alpha_e
        # via leakage ASMC; SO(3) tracker closes attitude through R_d built
        # from -I_a direction + a_h = [cos psi_d, sin psi_d, 0].) Default off
        # for A/B comparison; enable via LANDING_VIRTUAL_COMPASS=1.
        self._virtual_compass = (os.environ.get("LANDING_VIRTUAL_COMPASS", "0") == "1")
        self._K_R = np.diag([5.0, 5.0, 5.0])     # SO(3) attitude-error → body-rate gain
        self._psi_d = None                        # set lazily on first _attCtrl call

        # Reference depth-rate (MATLAB h_rd was -0.42; user has chosen -0.30 historically via REF_RAD_OPT_FLOW)
        # We keep ref_rad_opt_flow from the constructor; do not override.

        self._initialize_controller()
        self.start()

    def __del__(self):
        print("Controller thread is deleted...")

    def close(self):
        self._STAY_OPEN = False

    def _initialize_controller(self):
        # Attitude state
        self._quat = []
        self._w_i = []     # camera-frame angular velocity from img_data
        self._w = []       # body angular velocity (full 3-axis, FRD->body-NED)
        self._dw = []
        self._dw_deque = deque([np.zeros(N_DIM)] * 4)

        # Optical flow / middle-loop state
        self._h = []
        self._h_d = []
        self._h_e = []
        self._dh_d = []
        self._dh_d_deque = deque([np.zeros(N_DIM)] * 4)

        # Image features
        self._s = []
        self._s_e = []        # raw error (s - s_d)
        self._s_e_n = []      # normalized error (s_e[:2] / p_10)
        self._is_e_n = []     # integral of s_e_n
        self._ds_e_n_deque = deque([np.zeros(2)] * 4)
        self._ds_d = []       # desired feature-derivative output of outer PID

        # Middle-loop barrier (optical flow)
        self._p = []
        self._dp = []
        self._S = []
        self._zeta = []
        self._izeta = []
        self._G = []
        self._theta = []      # ||Theta||_F
        self._sigma = []
        self._kappa = [self._kappa_0.copy()]

        # Yaw SMC state
        self._e_a = []
        self._ie_a = []
        self._sigma_a = []
        self._kappa_a = [np.array(self._kappa_a_0)]
        self._u_a = []        # commanded yaw rate (rad/s)

        # Attitude reference / output
        self._euler_d = []
        self._euler_e = []
        self._ieuler_e = []
        self._deuler_e_deque = deque([np.zeros(N_DIM)] * 4)
        self._deuler_ed = []
        self._a_v = []
        self._a_u = []
        self._I_a_raw = []    # pre-LPF, pre-clamp inertial accel command
        self._I_a = []        # post-LPF, post-clamp inertial accel command
        # FoV-cone diagnostics
        self._rho_fov_log = []
        self._d_min_fov_log = []
        self._theta_cone_log = []
        self._theta_current_log = []
        self._w_u = []
        self._B_T = []
        self._u = []

        # Time
        self._t = []
        self._dt = []
        self._t0 = self._time.perf_counter()

    def run(self):
        while self._img_node.is_alive() and self._STAY_OPEN:
            if self._img_node.FEATURE_IS_VISIBLE:

                if self._CONTROLLER_READY:
                    self._updateTime()
                    self._updatePerfFunc()

                    quat = self._FC.getQuat()
                    self._quat.append(np.array([quat.w, quat.x, quat.y, quat.z]))

                    # FRD body rate -> body-NED:
                    #   forward (FRD x)  ->  +x_body (roll rate)
                    #   right   (FRD y)  ->  +y_body (pitch rate)
                    #   down    (FRD z)  ->  +z_body (yaw rate) — sign flip for NED convention
                    w = self._FC.getAngVelIMU()
                    self._w.append(np.array([w.forward_rad_s, w.right_rad_s, -w.down_rad_s]))

                    feature_param = self._img_node.getImgFeatureParam()
                    opt_flow_ang_vel = self._img_node.getOptFlowAngVel()
                    self._updateImgFeatureParam(feature_param)
                    # Append _w_i BEFORE _updateOptFlow — the latter now uses
                    # self._w_i[-1] (MATLAB V_w) and would IndexError on the
                    # first iteration if appended after.
                    #
                    # Cap |w_i| at ±5 rad/s per axis. The raw lstsq + KF + the
                    # 1.98× sensor_cal_hw amplification on ω_z can produce
                    # transient |w_i| > 10 rad/s from image noise. That feeds
                    # cross(w_i, s) in _updateOptFlow / PLASMC, blowing up
                    # h_d → h_e → zeta → kappa → a_u → drone tumbles → more
                    # optic-flow noise → positive feedback. ±5 rad/s
                    # (~285 deg/s) is the X500's physical body-rate limit;
                    # values beyond this aren't physically meaningful for
                    # the marker's apparent relative angular velocity.
                    W_I_MAX = 5.0
                    self._w_i.append(np.clip(opt_flow_ang_vel[3:], -W_I_MAX, W_I_MAX))
                    self._updateOptFlow(opt_flow_ang_vel[:3])

                    self.PLASMC()
                    self._yawCtrl()
                    self._attCtrl()

                if not self.TARGET_IS_VISIBLE:
                    self._initialize_controller()
                    self.TARGET_IS_VISIBLE = True

            elif self.TARGET_IS_VISIBLE:
                self.TARGET_IS_VISIBLE = False

            time.sleep(SLEEP_TIME)

        if self._img_node.is_alive():
            self._img_node.close()
        self._img_node.join()

    def _updateTime(self):
        self._t.append(self._time.perf_counter())
        if len(self._t) > 1:
            while self._t[-1] == self._t[-2]:
                time.sleep(SLEEP_TIME)
                self._t[-1] = self._time.perf_counter()
            self._dt.append(self._t[-1] - self._t[-2])

    def _updatePerfFunc(self):
        """Middle-loop optical-flow performance envelope (MATLAB-style)."""
        t = self._t[-1] - self._t0
        decay = expm(-t * self._gamma)
        self._p.append(decay @ (self._p_0 - self._p_inf) + self._p_inf)
        self._dp.append(-self._gamma @ decay @ (self._p_0 - self._p_inf))

    def _updateImgFeatureParam(self, s):
        """Outer loop: raw normalized pixel error -> PID -> desired feature derivative ds_d.

        MATLAB:
            V_s_e_n = (V_s - V_s_d) ./ p_10
            V_ds_d_xy = -rp*V_s_e_n - ri*iV_s_e_n - rd*dV_s_e_n
            V_ds_d = [V_ds_d_xy; 0]
        """
        self._s.append(s)
        self._s_e.append(self._s[-1] - self._s_d)

        # Normalized pixel error (sensor-half normalization)
        s_e_n = self._s_e[-1][:2] / self._p_10
        self._s_e_n.append(s_e_n)

        # Trapezoidal integration of normalized error + anti-windup
        if len(self._is_e_n) == 0:
            self._is_e_n.append(np.zeros(2))
        else:
            new_int = (self._is_e_n[-1]
                       + self._dt[-1] * 0.5 * (self._s_e_n[-1] + self._s_e_n[-2]))
            n = np.linalg.norm(new_int)
            if n > self._iV_s_e_n_clamp:
                new_int = new_int * (self._iV_s_e_n_clamp / n)
            self._is_e_n.append(new_int)

        # Smoothed derivative of normalized error
        if len(self._s_e_n) > 1:
            self._ds_e_n_deque.append((self._s_e_n[-1] - self._s_e_n[-2]) / self._dt[-1])
            self._ds_e_n_deque.popleft()
        ds_e_n = smooth4(self._ds_e_n_deque)

        # PID -> desired feature-time-derivative
        V_ds_d_xy = (- self._K_rp @ self._s_e_n[-1]
                     - self._K_ri @ self._is_e_n[-1]
                     - self._K_rd @ ds_e_n)
        self._ds_d.append(np.concatenate([V_ds_d_xy, [0.0]]))

    def _updateOptFlow(self, h):
        """Middle-loop: barrier-transform optical flow error, prep zeta / sigma inputs."""
        self._h.append(h)

        # Desired optical flow — MATLAB form (visualControl_IBVS_adaptive.m:369-370).
        # MATLAB uses V_w (optical-flow-derived target-relative ang vel in V frame),
        # NOT the UAV body rate. The corresponding Python variable is self._w_i
        # (= getOptFlowAngVel()[3:] = KF-smoothed lstsq output, V-frame). Using
        # self._w (body IMU) here was a parity bug — frames don't match (V vs body)
        # and physical meaning differs (target-relative vs absolute).
        w = self._w_i[-1]
        # MATLAB visualControl_IBVS_adaptive.m:369-370 EXACTLY:
        #   V_h_d = V_ds_d + cross(V_w, V_s(1:3))
        #         + (h_rd - dot(cross(V_w, V_s(1:3)), e3)) * V_s(1:3)
        # The earlier Python had BOTH coupling-term signs flipped (−cross and
        # +dot). On z that cancels to h_rd at s≈[0,0,1] so descent worked, but
        # x/y picked up doubled cross-coupling — V_h_d[0,1] excursions hit ±20
        # vs MATLAB's ±4, over-driving the SMC and giving 8× MATLAB descent rate.
        # Direct h_ref (MATLAB-equivalent). Previously had a soft-engage ramp
        # and a lateral-error gate here — both removed (see __init__ note).
        h_ref_eff = self._h_ref
        cross_ws = np.cross(w, self._s[-1][:3])
        self._h_d.append(
            self._ds_d[-1]
            + cross_ws
            + (h_ref_eff - np.dot(cross_ws, e3)) * self._s[-1][:3]
        )
        self._h_e.append(self._h[-1] - self._h_d[-1])

        # Barrier transform on h_e — MATLAB visualControl_IBVS_adaptive.m:380-385.
        # Only the RATIO is clamped (for log finiteness); the stored h is left
        # untouched so the downstream c-term still sees the actual measurement.
        S = np.eye(N_DIM)
        zeta = np.zeros(N_DIM)
        G = np.eye(N_DIM)
        for idx in range(N_DIM):
            ratio = self._h_e[-1][idx] / self._p[-1][idx]
            ratio = float(np.clip(ratio, -1.0 + S_MARGIN, 1.0 - S_MARGIN))
            S[idx, idx] = ratio
            zeta[idx] = np.log((1 + ratio) / (1 - ratio))
            G[idx, idx] = (np.exp(zeta[idx]) + 1) ** 2 / (2 * np.exp(zeta[idx]) * self._p[-1][idx])
        self._S.append(S)
        self._zeta.append(zeta)
        self._G.append(G)

        # Smoothed derivative of desired optical flow, with physical cap.
        # Without the cap, the PID's first non-zero firing produces a step
        # in V_h_d that gives raw dh_d ≈ (Δh_d)/dt ≈ 60-160 m/s² — feeds the
        # c-term, blows up |a_u|, drone flies away. Real-flight |dh_d| is
        # well under 5 m/s² even during aggressive maneuvers; ±20 is a
        # conservative bound that passes normal flight but blocks the
        # unphysical startup spike. (MATLAB doesn't need this because it
        # starts from a clean steady-state IC where V_h_d[0] is already
        # near the reference; SITL has takeoff/drift before controller
        # engagement, so V_h_d[0] is far from the equilibrium.)
        DH_D_MAX = float(os.environ.get("PLASMC_DH_D_MAX", "50.0"))
        if len(self._h_d) > 1:
            self._dh_d_deque.append((self._h_d[-1] - self._h_d[-2]) / self._dt[-1])
            self._dh_d_deque.popleft()
        self._dh_d.append(np.clip(smooth4(self._dh_d_deque), -DH_D_MAX, DH_D_MAX))

    def PLASMC(self):
        """Middle-loop adaptive SMC -> body-frame acceleration command a_u."""
        t = self._t[-1] - self._t0

        # Integral of zeta (trapezoidal) with anti-windup
        if len(self._izeta) == 0:
            self._izeta.append(np.zeros(N_DIM))
        else:
            new_int = (self._izeta[-1]
                       + self._dt[-1] * 0.5 * (self._zeta[-1] + self._zeta[-2]))
            # Anti-windup: per-COMPONENT clamp (matches MATLAB visualControl_IBVS
            # _adaptive.m:393-394). Was norm-clamp, which saturated ~30% earlier
            # on 3-vectors at the limit (norm=√3·5 vs per-axis 5 each).
            new_int = np.clip(new_int, -self._izeta_clamp, self._izeta_clamp)
            self._izeta.append(new_int)

        # Angular acceleration: smoothed derivative of V_w. MATLAB derives V_dw
        # from V_w_i (visualControl_IBVS_adaptive.m:295-299); we mirror that by
        # differentiating self._w_i (NOT self._w body rate).
        if len(self._w_i) > 1:
            self._dw_deque.append((self._w_i[-1] - self._w_i[-2]) / self._dt[-1])
            self._dw_deque.popleft()
        self._dw.append(smooth4(self._dw_deque))

        # Sliding surface (MATLAB: sigma = zeta + Omega * integral(zeta))
        self._sigma.append(self._zeta[-1] + self._Omega @ self._izeta[-1])

        # Known dynamics term c — MATLAB visualControl_IBVS_adaptive.m:407-408.
        # All cross products use V-frame target-relative ω (self._w_i), matching
        # MATLAB's V_w. Self._w (body IMU rate) is logged but not used here.
        w = self._w_i[-1]
        c = (np.cross(self._dw[-1], self._s[-1][:3])
             + np.cross(w, np.cross(w, self._s[-1][:3]))
             + 2 * np.cross(w, self._h[-1])
             - (np.dot(self._h[-1] + np.cross(w, self._s[-1][:3]), e3)) * self._h[-1]
             - self._dh_d[-1])

        # Theta matrix and its Frobenius norm
        # MATLAB: Theta = [-c + S*dp - G\(Omega*zeta), eye(3)]
        vector = (- c
                  + self._S[-1] @ self._dp[-1]
                  - np.linalg.solve(self._G[-1], self._Omega @ self._zeta[-1]))
        Theta = np.hstack([vector.reshape(-1, 1), np.eye(N_DIM)])
        self._theta.append(np.linalg.norm(Theta, ord='fro'))

        # Adaptive-gain (translational) update via RK5
        # MATLAB: dkappa/dt = Theta_norm * N * G * |sigma| - N * P * kappa
        if len(self._dt) > 0:
            self._kappa.append(
                RK5(self._kappaSolver, t, self._kappa[-1],
                    [self._sigma[-1], self._theta[-1]], self._dt[-1])
            )
        else:
            self._kappa.append(self._kappa[-1])

        # Switching + equivalent control (in barrier-transformed coords)
        # MATLAB:
        #   u_sw + u_eq when summed -> V_a_cd = -G\(u_sw + u_eq)
        sat_sigma = np.clip(self._sigma[-1] / np.diag(self._E), -1.0, 1.0)
        a_v = (- self._Gma @ self._sigma[-1]
               - self._theta[-1] * np.diag(sat_sigma) @ self._G[-1] @ self._kappa[-1]
               + self._G[-1] @ (- c + self._S[-1] @ self._dp[-1])
               - self._Omega @ self._zeta[-1])
        self._a_v.append(a_v)

        a_u = - np.linalg.solve(self._G[-1], a_v)
        # NOTE: the legacy |a_u|>100 abort was removed. PX4 saturates attitude-
        # rate setpoints internally to physical limits (~±220 deg/s); an
        # over-large a_u from a noisy startup PID firing just produces a
        # brief wobble, not a crash. Keeping the controller alive lets the
        # SMC adapt and recover instead of killing the run permanently.
        # _warmup_remaining is retained as a state variable for future use
        # but no longer gates anything here.
        self._a_u.append(a_u)

    def _yawCtrl(self):
        """Yaw adaptive SMC (MATLAB kappa_a) -> body yaw-rate setpoint u_a."""
        # Yaw feature error wrapped to [-pi/2, pi/2] (ellipse symmetry):
        # Wrap signed-angular error to [-π/2, π/2] via factor-of-2 trick.
        # alpha is π-period (2nd-moment formula is 180°-symmetric on
        # square corners), so this is the natural wrap.
        # MATLAB visualControl_IBVS_adaptive.m:483.
        e_a_raw = self._s[-1][3] - self._s_d[3]
        e_a = np.arctan2(np.sin(2 * e_a_raw), np.cos(2 * e_a_raw)) / 2.0
        self._e_a.append(e_a)

        # Trapezoidal integral with anti-windup
        if len(self._ie_a) == 0:
            self._ie_a.append(0.0)
        else:
            new_int = self._ie_a[-1] + self._dt[-1] * 0.5 * (self._e_a[-1] + self._e_a[-2])
            new_int = float(np.clip(new_int, -self._ie_a_clamp, self._ie_a_clamp))
            self._ie_a.append(new_int)

        # Sliding surface
        sigma_a = self._e_a[-1] + self._Omega_a * self._ie_a[-1]
        self._sigma_a.append(sigma_a)

        # Adaptive gain via RK5 (scalar)
        if len(self._dt) > 0:
            new_kappa_a = RK5(self._kappa_a_solver, self._t[-1] - self._t0,
                              self._kappa_a[-1], [sigma_a], self._dt[-1])
            self._kappa_a.append(new_kappa_a)
        else:
            self._kappa_a.append(self._kappa_a[-1])

        # Yaw command (MATLAB: u_a = Gamma_a*sigma_a + sat(sigma_a/E_a)*kappa_a + Omega_a*e_a)
        sat_term = float(np.clip(sigma_a / self._E_a, -1.0, 1.0))
        u_a = (self._Gma_a * sigma_a
               + sat_term * float(self._kappa_a[-1])
               + self._Omega_a * e_a)
        # Sign: MATLAB integrates u_a into desired heading psi_d. Body yaw rate
        # to drive heading toward target = same sign as u_a's effect on psi_d
        # but expressed in body frame. NED yaw rate convention matches.
        self._u_a.append(float(u_a))

        # Virtual-compass integrator (manuscript Eq. `psi d integrator`):
        #   psi_d(t+dt) = wrap[psi_d(t) + u_a * dt]
        # No external heading reference enters — psi_d evolves purely from
        # the image-based alpha error via the leakage ASMC. Only used in
        # the SO(3) path; legacy rate-mode path still consumes self._u_a.
        if self._virtual_compass and self._psi_d is not None and len(self._dt) > 0:
            self._psi_d = float(
                np.arctan2(np.sin(self._psi_d + u_a * self._dt[-1]),
                           np.cos(self._psi_d + u_a * self._dt[-1]))
            )

    def _kappaSolver(self, _, kappa, X):
        sigma = X[0]
        theta_norm = X[1]
        return theta_norm * self._N @ self._G[-1] @ np.abs(sigma) - self._N @ self._P @ kappa

    def _kappa_a_solver(self, _, kappa_a, X):
        sigma_a = X[0]
        # MATLAB: dkappa_a/dt = n_a * |sigma_a| - n_a * p_a * kappa_a
        return self._n_a * abs(sigma_a) - self._n_a * self._p_a * kappa_a

    def _attCtrl(self):
        """Convert body-frame accel a_u + yaw rate u_a -> [body rates; thrust] for PX4."""
        R = Quaternion(self._quat[-1]).to_DCM()
        euler = Quaternion(self._quat[-1]).to_angles()  # [roll, pitch, yaw]

        # Raw inertial accel (net of gravity)
        I_a_raw = R @ self._a_u[-1] - np.array([0.0, 0.0, g])
        self._I_a_raw.append(I_a_raw.copy())

        # ---- Full MATLAB FoV-margin cone (visualControl_IBVS_adaptive.m:443-469) ----
        I_a = I_a_raw.copy()

        # 1) Current tilt angle from body-z direction (R[2,2] is body-z's inertial-z component)
        R33 = float(np.clip(R[2, 2], -1.0, 1.0))
        theta_current = np.arccos(R33)

        # 2) Pixel-margin envelope (per axis, exponentially decaying)
        t_elapsed = self._t[-1] - self._t0
        rho_fov_curr = ((self._rho_fov_0 - self._rho_fov_inf)
                        * np.exp(-self._l_fov * t_elapsed)
                        + self._rho_fov_inf)   # (2,)

        # 3) Per-corner pixel margins — get latest 4 corners from img_node
        # _feature_pts[-1] is a [prev, curr] frame pair; [-1][1] is current 4 corners.
        # Raw corners are in OpenCV top-left coords; convert to image-centered.
        d_min_fov = 0.0
        try:
            fp_list = self._img_node._feature_pts
            if len(fp_list) > 0:
                raw_corners = np.asarray(fp_list[-1][1])   # (4, 2) — (u, v) top-left
                cx, cy = self._img_node.center
                u_centered = raw_corners[:, 0] - cx
                v_centered = raw_corners[:, 1] - cy
                d_corner_x = rho_fov_curr[0] - np.abs(u_centered)   # (4,)
                d_corner_y = rho_fov_curr[1] - np.abs(v_centered)   # (4,)
                d_min_fov = max(float(np.min(np.concatenate([d_corner_x, d_corner_y]))), 0.0)
        except (IndexError, AttributeError, ValueError, TypeError):
            d_min_fov = 0.0   # fall back to "no extra tilt allowed"

        # 4) Cone angle: current tilt + how-much-more-we-can-tilt-before-edge, capped
        focal_px = float(self._img_node.focal[0])
        theta_cone = float(min(theta_current + np.arctan(d_min_fov / focal_px),
                               self._theta_cap))

        # 5) Apply cone to inertial accel (NED; z=down, gravity subtracted).
        # MATLAB-equivalent safety: I_a represents required thrust acceleration
        # (thrust force / m). For sane upright drone with thrust opposing
        # gravity, I_a[2] should be NEGATIVE (thrust accel up in NED). If the
        # SMC ever commands I_a[2] >= 0, that asks for non-upward thrust →
        # drone would have to flip. Force a moderate negative value (~-3 m/s²
        # of thrust accel, i.e. mild lift but well below hover) to keep the
        # drone upright; the controller can still descend by reducing thrust
        # below gravity.
        if I_a[2] >= 0:
            I_a[2] = -3.0
        a_xy_lim = abs(I_a[2]) * np.tan(theta_cone)
        a_xy_n = np.linalg.norm(I_a[:2])
        if a_xy_n > a_xy_lim and a_xy_n > 1e-9:
            I_a[:2] = a_xy_lim * I_a[:2] / a_xy_n
        I_a[2] = max(I_a[2], -50.0)

        # log FoV diagnostics
        self._rho_fov_log.append(rho_fov_curr.copy())
        self._d_min_fov_log.append(d_min_fov)
        self._theta_cone_log.append(theta_cone)
        self._theta_current_log.append(theta_current)

        # ---- LPF (MATLAB tau_ia = 0.08 s) ----
        if len(self._I_a) == 0:
            self._I_a.append(I_a.copy())
        else:
            alpha = self._tau_ia / (self._tau_ia + self._dt[-1])
            self._I_a.append(alpha * self._I_a[-1] + (1.0 - alpha) * I_a)

        # ---- Inverse kinematics: desired roll/pitch from I_a (use current yaw) ----
        I_a_use = self._I_a[-1]
        yaw_c = euler[2]
        cy, sy = np.cos(yaw_c), np.sin(yaw_c)

        if abs(cy * I_a_use[0] + sy * I_a_use[1]) < 1e-6:
            theta_d = 0.0
        else:
            theta_d = np.arctan2(-cy * I_a_use[0] - sy * I_a_use[1], -I_a_use[2])

        if abs(sy * I_a_use[0] - cy * I_a_use[1]) < 1e-6:
            phi_d = 0.0
        else:
            phi_d = np.arctan2(-sy * I_a_use[0] + cy * I_a_use[1],
                               -I_a_use[2] / np.cos(theta_d))

        # Clamp away from singularity (replaces print-only warning)
        ang_lim = np.deg2rad(85.0)
        phi_d = float(np.clip(phi_d, -ang_lim, ang_lim))
        theta_d = float(np.clip(theta_d, -ang_lim, ang_lim))

        self._euler_d.append(np.array([phi_d, theta_d, 0.0]))

        # Euler error (yaw channel zeroed here; yaw is handled by _yawCtrl -> _u_a)
        self._euler_e.append(np.array([euler[0] - phi_d,
                                       euler[1] - theta_d,
                                       0.0]))

        # Integral / derivative of euler error
        if len(self._ieuler_e) == 0:
            self._ieuler_e.append(np.zeros(N_DIM))
        else:
            self._ieuler_e.append(self._ieuler_e[-1]
                                  + self._dt[-1] * 0.5 * (self._euler_e[-1] + self._euler_e[-2]))

        if len(self._euler_e) > 1:
            self._deuler_e_deque.append((self._euler_e[-1] - self._euler_e[-2]) / self._dt[-1])
            self._deuler_e_deque.popleft()
        deuler_e = smooth4(self._deuler_e_deque)

        self._deuler_ed.append(- self._K_ep @ self._euler_e[-1]
                               - self._K_ei @ self._ieuler_e[-1]
                               - self._K_ed @ deuler_e)

        if self._virtual_compass:
            # ===== Manuscript-faithful path =====
            # Virtual-compass integrator (psi_d) + R_d from thrust direction +
            # heading vector + SO(3) attitude error → body-rate setpoint.
            # See manuscript control_formulation.tex:219-251.

            # Lazy-init psi_d on first call (MATLAB: psi_d = yaw_init, line 127)
            if self._psi_d is None:
                self._psi_d = float(yaw_c)

            # R_d construction (Eq. R_d construction: equation, lines 238-242)
            #   rd3 = -I_a / ||I_a||      (desired body-z opposes net force in NED)
            #   a_h = [cos psi_d, sin psi_d, 0]
            #   rd2 = (rd3 × a_h) / ||rd3 × a_h||
            #   rd1 = rd2 × rd3
            f_mag = float(np.linalg.norm(I_a_use))
            if f_mag < 1e-6:
                R_d = np.eye(3)
            else:
                rd3 = -I_a_use / f_mag
                a_h = np.array([np.cos(self._psi_d), np.sin(self._psi_d), 0.0])
                rd2_raw = np.cross(rd3, a_h)
                n2 = float(np.linalg.norm(rd2_raw))
                if n2 < 1e-6:
                    # degeneracy guard (manuscript: rd2 = inertial East)
                    rd2 = np.array([0.0, 1.0, 0.0])
                else:
                    rd2 = rd2_raw / n2
                rd1 = np.cross(rd2, rd3)
                R_d = np.column_stack([rd1, rd2, rd3])

            # SO(3) attitude error (Eq. so3 errors: equation, line 244-245)
            #   e_R = 0.5 * vee(R_d^T R - R^T R_d)
            eR_mat = 0.5 * (R_d.T @ R - R.T @ R_d)
            e_R = np.array([eR_mat[2, 1], eR_mat[0, 2], eR_mat[1, 0]])

            # Rate-mode mapping: w_u = -K_R · e_R   (proportional part of SO(3)
            # tracker; kΩ·e_Ω damping and ω×Jω feedforward are handled by PX4's
            # rate loop since we ship body rates, not torques).
            w_u = -self._K_R @ e_R
        else:
            # ===== Legacy Euler-PD path (yaw routed via u_a directly) =====
            # Map Euler-rate command to body rates using DESIRED attitude (W_d)
            W_d = np.array([
                [1, 0, -np.sin(theta_d)],
                [0, np.cos(phi_d),  np.sin(phi_d) * np.cos(theta_d)],
                [0, -np.sin(phi_d), np.cos(phi_d) * np.cos(theta_d)],
            ])
            w_u = W_d @ self._deuler_ed[-1]

            # Override yaw rate with the yaw SMC output u_a
            if len(self._u_a) > 0:
                w_u[2] = self._u_a[-1]

        self._w_u.append(w_u)

        # Thrust scalar (Newtons) for landing_test.convert_2_sys_cmd, which
        # expects B_T as "thrust DEFICIT below hover": thrust_norm = 0.738 -
        # B_T/45. So B_T = 0 at hover, B_T > 0 → less thrust (descend),
        # B_T < 0 → more thrust (climb).
        # Our I_a[2] is in NED-with-gravity-subtracted (so I_a[2] = -g at hover,
        # not 0). Need to add g back to align: B_T = mass·(I_a[2] + g)/cos·cos.
        # Without the +g, B_T = -20.7 N at hover → thrust_norm clips to 1.0 →
        # full throttle → drone climbs instead of hovering.
        self._B_T.append(mass * (self._I_a[-1][2] + g)
                         / max(np.cos(euler[0]), 1e-6)
                         / max(np.cos(euler[1]), 1e-6))

        self._u.append(np.concatenate((self._w_u[-1], [self._B_T[-1]])))

    # ---------------- Public API ----------------
    def startController(self, warmup_steps=100):
        self._t0 = self._time.perf_counter()
        self._CONTROLLER_READY = True
        # During warmup, suppress the |a_u|>100 abort so the PID/SMC deques
        # can fill without triggering the safety. Caller (landing_test) should
        # NOT use the controller output during this phase — keep sending hover
        # setpoints instead. Set to 0 to disable warmup.
        self._warmup_remaining = warmup_steps

    def checkTargetVisibility(self):
        # Backward-compat: outer barrier removed, so visibility is delegated to img_node
        return bool(self._img_node.FEATURE_IS_VISIBLE)

    def getControlInput(self):
        return self._u[-1] if len(self._u) > 0 else np.zeros(N_DIM + 1)

    def getParams(self):
        return {
            "Des Img Feature Param": self._s_d,
            # Outer PID
            "p_10": self._p_10,
            "K_rp": self._K_rp,
            "K_ri": self._K_ri,
            "K_rd": self._K_rd,
            # Middle-loop envelope
            "gamma": self._gamma,
            "p_0": self._p_0,
            "p_inf": self._p_inf,
            # Middle-loop SMC
            "Omega": self._Omega,
            "Gamma": self._Gma,
            "E": self._E,
            # Adaptive gains
            "N": self._N,
            "P": self._P,
            "kappa_0": self._kappa_0,
            # Yaw SMC
            "Omega_a": self._Omega_a,
            "Gamma_a": self._Gma_a,
            "n_a": self._n_a,
            "p_a": self._p_a,
            "kappa_a_0": self._kappa_a_0,
            "E_a": self._E_a,
            # FoV / LPF
            "theta_cap_deg": np.rad2deg(self._theta_cap),
            "rho_fov_0": self._rho_fov_0,
            "rho_fov_inf": self._rho_fov_inf,
            "l_fov": self._l_fov,
            "tau_ia": self._tau_ia,
            # Inner-attitude PD (kept for logging; PX4 rate ctrl does the heavy lifting)
            "K_ep": self._K_ep,
        }

    def getLogData(self):
        return {
            "t": self._t,
            "w_i(t)": self._w_i,
            "w(t)": self._w,
            "h(t)": self._h,
            "h_d(t)": self._h_d,
            "dh_d(t)": self._dh_d,
            "s(t)": self._s,
            "s_e(t)": self._s_e,
            "s_e_n(t)": self._s_e_n,
            "is_e_n(t)": self._is_e_n,
            "ds_d(t)": self._ds_d,
            # Middle-loop barrier
            "p(t)": self._p,
            "dp(t)": self._dp,
            "S(t)": self._S,
            "zeta(t)": self._zeta,
            "izeta(t)": self._izeta,
            "G(t)": self._G,
            "theta(t)": self._theta,
            "sigma(t)": self._sigma,
            "kappa(t)": self._kappa,
            # Yaw SMC
            "e_a(t)": self._e_a,
            "ie_a(t)": self._ie_a,
            "sigma_a(t)": self._sigma_a,
            "kappa_a(t)": self._kappa_a,
            "u_a(t)": self._u_a,
            # Attitude / output
            "a_v(t)": self._a_v,
            "a_u(t)": self._a_u,
            "I_a_raw(t)": self._I_a_raw,
            "I_a(t)": self._I_a,
            "w_u(t)": self._w_u,
            "B_T(t)": self._B_T,
            "EA_d(t)": self._euler_d,
            # FoV-margin cone diagnostics
            "rho_fov(t)": self._rho_fov_log,
            "d_min_fov(t)": self._d_min_fov_log,
            "theta_cone(t)": self._theta_cone_log,
            "theta_current(t)": self._theta_current_log,
        }

    def getImgData(self):
        return self._img_node.getLogData()

    def getImgParams(self):
        return self._img_node.getParams()

    def enableRecording(self):
        self._img_node.RECORD = True
