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
#   - Camera intrinsics (fx=fy=540, 1280x960) come from x500_mono_cam_down SDF.
#   - img_data._sensor_cal_s = diag(1/6, 1/6, 1, 1) is kept (user choice).
#     This attenuates the centroid by 6x before reaching the controller, so
#     MATLAB gains transplanted here will have a weaker effective response on
#     the centroid channel. Expect a Gazebo re-tuning pass.
# **************************************************************************
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

mass = 2.114  # kg, Holybro X500
g = 9.80      # m/s^2

# Clamp |S| < 1 - S_MARGIN to keep log-barrier finite (MATLAB uses 0.05 margin)
S_MARGIN = 0.05


class Controller(Thread):
    def __init__(self, ref_rad_opt_flow, des_img_feature, time_keeper=time, controller=None, record='n'):
        Thread.__init__(self)
        self._h_ref = ref_rad_opt_flow

        self._CONTROLLER_READY = False
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
        # MATLAB: rp = diag(9.0, 9.0), ri = diag(0.1, 0.1), rd = diag(1.4375, 1.4375)
        self._K_rp = np.diag([9.0, 9.0])
        self._K_ri = np.diag([0.1, 0.1])
        self._K_rd = np.diag([1.4375, 1.4375])

        # Middle-loop performance envelope (optical flow)
        # MATLAB: gamma_2 = [0.2, 0.2, 0.2], p_20 = [25, 25, 4], p_2inf = [2.5, 2.5, 1.5]
        self._gamma = np.diag([0.2, 0.2, 0.2])
        self._p_0 = np.array([25.0, 25.0, 4.0])
        self._p_inf = np.array([2.5, 2.5, 1.5])

        # Middle-loop SMC
        # MATLAB: Omega = diag(0.05, 0.05, 0.025), Gamma = diag(0.4375, 0.5, 0.75), E = diag(1, 1, 1)
        self._Omega = np.diag([0.05, 0.05, 0.025])
        self._Gma = np.diag([0.4375, 0.5, 0.75])
        self._E = np.diag([1.0, 1.0, 1.0])

        # Adaptive-gain (translational) ODE
        # MATLAB: N = diag(0.02, 0.02, 0.05), P = diag(1.5, 1.5, 5.0), kappa_0 = [0.125, 0.125, 0.25]
        self._N = np.diag([0.02, 0.02, 0.05])
        self._P = np.diag([1.5, 1.5, 5.0])
        self._kappa_0 = np.array([0.125, 0.125, 0.25])

        # Yaw adaptive SMC
        # MATLAB: Omega_a = 0.5, Gamma_a = 0.5, n_a = 1.0, p_a = 2, kappa_a_0 = 2.0, E_a = 3.0
        self._Omega_a = 0.5
        self._Gma_a = 0.5
        self._n_a = 1.0
        self._p_a = 2.0
        self._kappa_a_0 = 2.0
        self._E_a = 3.0

        # FoV-cone tilt clamp (simplified port; MATLAB uses per-corner pixel margin)
        self._theta_cap = np.deg2rad(60.0)

        # Low-pass filter on inertial accel (MATLAB: tau_ia = 0.08 s)
        self._tau_ia = 0.08

        # Anti-windup clamps (MATLAB izeta_2 clamped to ±5; others heuristic)
        self._iV_s_e_n_clamp = 5.0
        self._izeta_clamp = 5.0
        self._ie_a_clamp = 2.0

        # Inner-attitude PD (rate-side; PX4's rate ctrl does the actuator work)
        # K_ei/K_ed default to zero — angle error already feeds the rate
        # setpoint, integration left to PX4 if needed.
        self._K_ep = np.diag([5.0, 5.0, 5.0])
        self._K_ei = np.diag([0.0, 0.0, 0.0])
        self._K_ed = np.diag([0.0, 0.0, 0.0])

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
                    self._updateOptFlow(opt_flow_ang_vel[:3])
                    self._w_i.append(opt_flow_ang_vel[3:])

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

        # Desired optical flow (MATLAB form preserved)
        self._h_d.append(
            self._ds_d[-1]
            - np.cross(self._w[-1], self._s[-1][:3])
            + (self._h_ref + np.dot(np.cross(self._w[-1], self._s[-1][:3]), e3))
              * self._s[-1][:3]
        )
        self._h_e.append(self._h[-1] - self._h_d[-1])

        # Barrier transform on h_e
        S = np.eye(N_DIM)
        zeta = np.zeros(N_DIM)
        G = np.eye(N_DIM)
        for idx in range(N_DIM):
            ratio = self._h_e[-1][idx] / self._p[-1][idx]
            # Clamp to [-1+margin, 1-margin] for log finiteness
            ratio = float(np.clip(ratio, -1.0 + S_MARGIN, 1.0 - S_MARGIN))
            S[idx, idx] = ratio
            # If actually out of envelope, also clamp the stored h to barrier boundary
            if abs(self._h_e[-1][idx]) >= self._p[-1][idx]:
                self._h[-1][idx] = ratio * self._p[-1][idx] + self._h_d[-1][idx]
            zeta[idx] = np.log((1 + ratio) / (1 - ratio))
            G[idx, idx] = (np.exp(zeta[idx]) + 1) ** 2 / (2 * np.exp(zeta[idx]) * self._p[-1][idx])
        self._S.append(S)
        self._zeta.append(zeta)
        self._G.append(G)

        # Smoothed derivative of desired optical flow
        if len(self._h_d) > 1:
            self._dh_d_deque.append((self._h_d[-1] - self._h_d[-2]) / self._dt[-1])
            self._dh_d_deque.popleft()
        self._dh_d.append(smooth4(self._dh_d_deque))

    def PLASMC(self):
        """Middle-loop adaptive SMC -> body-frame acceleration command a_u."""
        t = self._t[-1] - self._t0

        # Integral of zeta (trapezoidal) with anti-windup
        if len(self._izeta) == 0:
            self._izeta.append(np.zeros(N_DIM))
        else:
            new_int = (self._izeta[-1]
                       + self._dt[-1] * 0.5 * (self._zeta[-1] + self._zeta[-2]))
            n = np.linalg.norm(new_int)
            if n > self._izeta_clamp:
                new_int = new_int * (self._izeta_clamp / n)
            self._izeta.append(new_int)

        # Body angular acceleration (smoothed finite difference of w)
        if len(self._w) > 1:
            self._dw_deque.append((self._w[-1] - self._w[-2]) / self._dt[-1])
            self._dw_deque.popleft()
        self._dw.append(smooth4(self._dw_deque))

        # Sliding surface (MATLAB: sigma = zeta + Omega * integral(zeta))
        self._sigma.append(self._zeta[-1] + self._Omega @ self._izeta[-1])

        # Known dynamics term c
        c = (np.cross(self._dw[-1], self._s[-1][:3])
             + np.cross(self._w[-1], np.cross(self._w[-1], self._s[-1][:3]))
             + 2 * np.cross(self._w[-1], self._h[-1])
             - (np.dot(self._h[-1] + np.cross(self._w[-1], self._s[-1][:3]), e3)) * self._h[-1]
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
        # Sanity: if PLASMC blew up (G ill-conditioned or zeta singular), abort.
        if np.any(np.abs(a_u) > 100):
            print(f"[PLASMC] a_u blew up: {a_u}\nG={self._G[-1]}\nc={c}")
            self._STAY_OPEN = False
        self._a_u.append(a_u)

    def _yawCtrl(self):
        """Yaw adaptive SMC (MATLAB kappa_a) -> body yaw-rate setpoint u_a."""
        # Yaw feature error wrapped to [-pi/2, pi/2] (ellipse symmetry):
        # e_a = atan2(sin(2(s4 - s_d4)), cos(2(s4 - s_d4))) / 2
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

        # ---- Tilt-cone clamp (simplified port of MATLAB's FoV cone) ----
        I_a = I_a_raw.copy()
        # Ensure enough downward thrust acceleration to retain tilt authority.
        # In NED, I_a[2] >= 0 would mean net "downward" net acceleration, which
        # with insufficient thrust authority destabilizes the lateral cone.
        if I_a[2] >= 0:
            I_a[2] = -3.0
        a_xy_lim = abs(I_a[2]) * np.tan(self._theta_cap)
        a_xy_n = np.linalg.norm(I_a[:2])
        if a_xy_n > a_xy_lim and a_xy_n > 1e-9:
            I_a[:2] = a_xy_lim * I_a[:2] / a_xy_n
        # Floor on downward (climb) accel magnitude
        I_a[2] = max(I_a[2], -50.0)

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

        # ---- Map Euler-rate command to body rates using DESIRED attitude (W_d) ----
        # MATLAB cascades attitude PID -> body torques, but for PX4 we send rates.
        # Use W_d so the kinematic mapping is consistent with the desired attitude.
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

        # Thrust scalar (Newtons) — same convention as before:
        # B_T is the EXCESS thrust beyond gravity compensation, mapped to PX4 throttle
        # in landing_test.convert_2_sys_cmd. At hover, I_a[2]≈0 -> B_T≈0.
        self._B_T.append(mass * self._I_a[-1][2]
                         / max(np.cos(euler[0]), 1e-6)
                         / max(np.cos(euler[1]), 1e-6))

        self._u.append(np.concatenate((self._w_u[-1], [self._B_T[-1]])))

    # ---------------- Public API ----------------
    def startController(self):
        self._t0 = self._time.perf_counter()
        self._CONTROLLER_READY = True

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
        }

    def getImgData(self):
        return self._img_node.getLogData()

    def getImgParams(self):
        return self._img_node.getParams()

    def enableRecording(self):
        self._img_node.RECORD = True
