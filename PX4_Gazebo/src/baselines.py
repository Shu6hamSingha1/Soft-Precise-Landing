"""Python ports of the four MATLAB comparison baselines (MATLAB/Comparison/ctrl_*.m).

    lin2022   Lin et al., TII 2022      PBVS + prescribed-performance funnels
    zhang2026 Zhang & Wu, TIE 2026      PBVS + AEDO backstepping
    lin2023   Lin et al., TASE 2023     IBVS circle-moment features + funnels
    cho2022   Cho et al., AST 2022      feed-forward point-feature IBVS

Each baseline maps its sensed inputs to the commanded specific-force acceleration
``I_a_cd = F/m`` (NED, gravity included: [0,0,-g] at hover) -- exactly what the MATLAB
harness feeds to the SHARED SO(3) tracker (``shared_so3 = true``, the default since
2026-09-21). ``accel_to_rate_thrust`` is the PX4 counterpart of that tracker and uses the
same convention as ``Controller._attCtrl`` (w_u = -K_R e_R, thrust "deficit" B_T), so every
controller -- PLASMC included -- reaches PX4 through the same attitude path.

Gains are copied from MATLAB/Comparison/InitGains_Comparison.m (2026-09-21 retune). Read
that file for provenance; do not retune here without updating it.

Coordinate frames: NED inertial, FRD body. All vectors are numpy arrays.
"""
import numpy as np

G = 9.81
G_VEC = np.array([0.0, 0.0, G])


# --------------------------------------------------------------------------- helpers
def _rotz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _funnel_terms(e, rho):
    """Log-barrier transform shared by both funnel baselines: returns (eps, q)."""
    xi = np.clip(e / rho, -0.999, 0.999)
    eps = 0.5 * np.log((1.0 + xi) / (1.0 - xi))
    q = 1.0 / ((1.0 + xi) * (1.0 - xi))
    return eps, q


def _funnel_rho(rho0, rho_inf, l, t):
    return (rho0 - rho_inf) * np.exp(-l * t) + rho_inf


# B_T SATURATION (2026-09-23, fixing the near-gimbal-lock blow-up found while
# investigating cho2022's Ls-near-touchdown instability -- see
# Memory/px4/feedback_cho2022_never_lands_rootcause.md). The old code only guarded
# against literal division-by-zero (max(cos(.), 1e-6)), which still lets B_T reach
# +-millions when the REALIZED attitude tips toward gimbal-lock -- a real event this
# baseline's growing-gain-as-z-shrinks IBVS law can trigger near touchdown. That huge
# B_T was always harmless to the ACTUATOR (apps/landing_test.py's convert_2_sys_cmd
# already clips thrust_norm to [0,1]), but it's a meaningless, chattering intermediate
# signal that swings the effective command between full-throttle and zero-throttle on
# noise, and pollutes B_T(t) logs with physically absurd values. Saturating HERE, at
# the source, keeps B_T meaningful and matches exactly what thrust_norm's downstream
# [0,1] clip can express -- B_T outside this range was ALREADY being clipped
# effectively (via thrust_norm), just silently and without B_T itself reflecting it.
# Derived from apps/landing_test.py:84's thrust_norm = clip(0.738 - B_T/42.3, 0, 1):
#   thrust_norm=1 (max climb)     -> B_T = (0.738 - 1) * 42.3   = -11.08 N
#   thrust_norm=0 (min throttle)  -> B_T =  0.738      * 42.3   = +31.22 N
# Hardcoded (not imported) because baselines.py is deliberately standalone (numpy
# only) and landing_test.py's 0.738/42.3 are themselves empirical calibration
# constants, not physical ones -- update BOTH places together if that mapping is
# ever recalibrated.
B_T_MIN, B_T_MAX = -11.08, 31.22


def accel_to_rate_thrust(I_a_cd, R, psi_des, K_R, mass):
    """Shared attitude stage: desired specific force -> (body-rate cmd [3], thrust deficit B_T).

    Same maths as MATLAB blocks.so3_tracker / Controller._attCtrl: rd3 = -F/|F|, heading
    vector [cos psi, sin psi, 0], e_R = 0.5 vee(Rd'R - R'Rd), w_u = -K_R e_R.
    B_T = mass*(I_a[2]+g)/(cos roll cos pitch) is the thrust BELOW hover, as consumed by
    landing_test.convert_2_sys_cmd (0 at hover, >0 descends). SATURATED to [B_T_MIN,
    B_T_MAX] -- see the comment above; matches what the downstream thrust_norm clip can
    actually express, so this is a saturation, not a behavior change under normal flight.
    """
    f = float(np.linalg.norm(I_a_cd))
    if f < 1e-6:
        Rd = np.eye(3)
    else:
        rd3 = -I_a_cd / f
        rd2 = np.cross(rd3, np.array([np.cos(psi_des), np.sin(psi_des), 0.0]))
        n2 = float(np.linalg.norm(rd2))
        rd2 = np.array([0.0, 1.0, 0.0]) if n2 < 1e-6 else rd2 / n2
        Rd = np.column_stack([np.cross(rd2, rd3), rd2, rd3])
    E = 0.5 * (Rd.T @ R - R.T @ Rd)
    e_R = np.array([E[2, 1], E[0, 2], E[1, 0]])
    w_u = -np.asarray(K_R) @ e_R
    roll = np.arctan2(R[2, 1], R[2, 2])
    pitch = -np.arcsin(np.clip(R[2, 0], -1.0, 1.0))
    B_T = mass * (I_a_cd[2] + G) / max(np.cos(roll), 1e-6) / max(np.cos(pitch), 1e-6)
    B_T = float(np.clip(B_T, B_T_MIN, B_T_MAX))
    return w_u, B_T


def marker_key_points(marker_scale=26.0):
    """5 key points of the SITL cross in the marker frame (4 arm tips + stub tip), 3x5 [m].
    Column order is a contract with the MATLAB InitVar (stub last)."""
    s = marker_scale / 250.0
    r = 15.0 / np.sqrt(2.0)
    return np.array([[r, -r, -r, r, 22.0],
                     [r, -r, r, -r, 0.0],
                     [0.0, 0.0, 0.0, 0.0, 0.0]]) * s


def project_marker_v_frame(p_cam, R_cam, p_mkr, R_mkr, yaw, f, zf, key_pts):
    """Pixel coords of the marker key points de-rotated onto the virtual (yaw-only) frame.
    Mirrors visualControl_comparison.m: exact per-point pinhole with the +zf depth offset,
    then V_R_C = rotz(yaw)' R_c (image_features.m). Returns (2xN px, camera-frame marker centre)."""
    P = R_mkr @ key_pts + p_mkr[:, None]
    C = R_cam.T @ (P - p_cam[:, None])
    px = f * C[:2] / np.maximum(C[2] + zf, 1e-3)
    rays = np.vstack([px, f * np.ones(px.shape[1])])
    vr = (_rotz(yaw).T @ R_cam) @ rays
    return f * vr[:2] / vr[2], R_cam.T @ (p_mkr - p_cam)


# --------------------------------------------------------------------------- gains
K_LIN2022 = dict(
    k1=np.array([1.5, 1.5, 0.6]), k2=4.0,
    rho_inf_p=np.array([0.10, 0.10, 0.15]), rho_inf_v=np.array([0.30, 0.30, 0.15]),
    l_p=np.array([0.10, 0.10, 0.10]), l_v=np.array([0.10, 0.10, 0.10]),
    rho_p0_margin=1.5, rho_v0_margin=1.5, r_pt_des=np.zeros(3))

K_ZHANG2026 = dict(
    Kc1=np.diag([0.25, 0.25, 0.03]), Kc2=np.diag([2.0, 2.0, 0.05]), Kc3=np.diag([2.5, 2.5, 0.30]),
    lAF1=1.0, lAF2=0.5, omega_AFm=1.0, PNF_poly=[0.0, 0.0, 150.0], omega_AF0=1.0)

K_LIN2023 = dict(
    k1=np.array([1.2, 1.2, 0.40]), k2=4.0,
    rho_inf_t=np.array([0.10, 0.10, 0.03]), rho_inf_v=np.array([0.30, 0.30, 0.15]),
    l_t=np.array([0.05, 0.05, 0.10]), l_v=np.array([0.03, 0.03, 0.10]),
    rho_t0_margin=np.array([1.5, 1.5, 5.0]), rho_v0_margin=1.5)

K_CHO2022 = dict(
    lambda_ibvs=np.array([-0.8, -0.8, -2.0, 0.0, 0.0, 0.0]),
    v_sat=np.array([0.5, 0.5, 0.7, 0.2]), k_sigmoid=0.0005, use_sq_comp=True,
    Kv=np.diag([1.8, 1.8, 2.0]))


# --------------------------------------------------------------------------- baselines
class Lin2022:
    """PBVS + prescribed-performance funnels (ctrl_Lin2022.m, Eqs. 9-18)."""
    name = "lin2022"
    needs_features = False

    def __init__(self, mass, K=K_LIN2022):
        self.K, self.m = K, mass
        self._rho0 = None

    def reset(self):
        self._rho0 = None

    def step(self, s):
        K = self.K
        e_p = (s["p_c"] - s["p_t"]) - K["r_pt_des"]
        if self._rho0 is None:                       # adaptive rho(0) = |e(0)| + margin
            rho_p0 = np.abs(e_p) + K["rho_p0_margin"]
            eps, q = _funnel_terms(e_p, rho_p0)
            vhat0 = -K["k1"] * (q * eps)
            self._rho0 = (rho_p0, np.abs(s["v_c"] - vhat0) + K["rho_v0_margin"])
        rho_p0, rho_v0 = self._rho0
        t = s["t"]
        rho_p = _funnel_rho(rho_p0, K["rho_inf_p"], K["l_p"], t)
        rho_v = _funnel_rho(rho_v0, K["rho_inf_v"], K["l_v"], t)
        eps_p, q_p = _funnel_terms(e_p, rho_p)
        vhat = -K["k1"] * (q_p * eps_p)                                  # Eq. 15
        eps_v, q_v = _funnel_terms(s["v_c"] - vhat, rho_v)               # Eq. 16
        F = -K["k2"] * (q_v * eps_v) - self.m * G_VEC                    # Eq. 18 (NED)
        return F / self.m


class Zhang2026:
    """PBVS + AEDO backstepping (ctrl_Zhang2026.m, Eqs. 11-16, 39)."""
    name = "zhang2026"
    needs_features = False

    def __init__(self, mass, K=K_ZHANG2026):
        self.K, self.m = K, mass
        self.reset()

    def reset(self):
        self.xhat = np.zeros(6)
        self.w = self.K["omega_AF0"]
        self.F_prev = np.array([0.0, 0.0, -self.m * G])
        self.vm_prev = None

    def step(self, s):
        K, m, dt = self.K, self.m, max(s["dt"], 1e-3)
        vm = s["v_c"]                                   # measured (noisy) velocity
        if self.vm_prev is None:
            self.vm_prev = vm.copy()
        I3, O3 = np.eye(3), np.zeros((3, 3))
        w = self.w
        L = np.vstack([K["lAF1"] * w * I3, K["lAF2"] * w ** 2 * I3])
        Fdm = m * ((s["v_c"] - self.vm_prev) / dt) - self.F_prev
        A = np.block([[O3, I3], [O3, O3]])
        innov = Fdm - self.xhat[:3]
        self.xhat = self.xhat + dt * (A @ self.xhat + L @ innov)
        Fd_hat = self.xhat[:3]
        zpq = abs(s["p_c"][2] - s["p_t"][2])
        PNF = max(np.polyval(K["PNF_poly"], zpq), 1e-6)
        self.w = max((max(1.0, w ** 2) * float(innov @ innov) / PNF) ** 0.25, K["omega_AFm"])

        re, re_dot = s["p_c"] - s["p_t"], s["v_c"] - s["v_t"]
        Fc = (-(K["Kc1"] @ K["Kc3"] + K["Kc2"]) @ re
              - (m * K["Kc1"] + K["Kc3"]) @ re_dot - m * G_VEC - Fd_hat)   # Eq. 39
        I_a = Fc / m
        self.vm_prev = vm.copy()
        self.F_prev = m * (I_a + G_VEC)
        return I_a


class Lin2023:
    """IBVS circle-moment features + funnels (ctrl_Lin2023.m, Eqs. 7-19)."""
    name = "lin2023"
    needs_features = True

    def __init__(self, mass, K=K_LIN2023):
        self.K, self.m = K, mass
        self._rho0 = None

    def reset(self):
        self._rho0 = None

    @staticmethod
    def _poly_area(P):
        c = P.mean(axis=1, keepdims=True)
        o = np.argsort(np.arctan2(P[1] - c[1], P[0] - c[0]))          # polar-sort: cross is self-intersecting
        x, y = P[0, o], P[1, o]
        return 0.5 * abs(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))

    def step(self, s):
        K, f = self.K, s["f"]
        Pi, Pd = s["px"], s["px_d"]
        an = np.sqrt(max(self._poly_area(Pd), 1e-9) / max(self._poly_area(Pi), 1e-9))
        s_t = np.array([an * Pi[0].mean() / f, an * Pi[1].mean() / f, an])            # Eq. 7
        s_d = np.array([Pd[0].mean() / f, Pd[1].mean() / f, 1.0])
        e_t = s_t - s_d
        R_V = _rotz(s["yaw"])
        if self._rho0 is None:
            rho_t0 = np.abs(e_t) + K["rho_t0_margin"]
            eps, q = _funnel_terms(e_t, rho_t0)
            vhat_I0 = R_V @ (K["k1"] * (q * eps))
            self._rho0 = (rho_t0, np.abs(s["v_c"] - vhat_I0) + K["rho_v0_margin"])
        rho_t0, rho_v0 = self._rho0
        t = s["t"]
        rho_t = _funnel_rho(rho_t0, K["rho_inf_t"], K["l_t"], t)
        rho_v = _funnel_rho(rho_v0, K["rho_inf_v"], K["l_v"], t)
        eps_t, q_t = _funnel_terms(e_t, rho_t)
        vhat_I = R_V @ (K["k1"] * (q_t * eps_t))          # +k1: image dynamics invert the sign (see .m)
        eps_v, q_v = _funnel_terms(s["v_c"] - vhat_I, rho_v)
        F = -K["k2"] * (q_v * eps_v) - self.m * G_VEC     # Eq. 19
        return F / self.m


class Cho2022:
    """Feed-forward point-feature IBVS (ctrl_Cho2022.m)."""
    name = "cho2022"
    needs_features = True

    def __init__(self, mass, K=K_CHO2022):
        self.K, self.m = K, mass

    def reset(self):
        pass

    @staticmethod
    def _square_compensate(P):
        cx, cy = P[0].mean(), P[1].mean()
        half = (np.linalg.norm(P[:, 0] - P[:, 3]) + np.linalg.norm(P[:, 0] - P[:, 1])) / 4.0
        return np.array([[cx - half, cx + half, cx + half, cx - half],
                         [cy + half, cy + half, cy - half, cy - half]])

    def step(self, s):
        K, f = self.K, s["f"]
        Pi, Pd = s["px"], s["px_d"]
        N = Pi.shape[1]
        if K["use_sq_comp"] and N == 4:
            Pi = self._square_compensate(Pi)
        e = (Pd - Pi).reshape(-1, order="F")                               # column-major = MATLAB reshape
        z = s["C_s_tc"][2]
        z = 0.01 if abs(z) < 0.01 else z
        Ls = np.zeros((2 * N, 6))
        for i in range(N):
            xp, yp = Pi[0, i], Pi[1, i]
            Ls[2 * i] = [-f / z, 0, xp / z, xp * yp / f, -(f ** 2 + xp ** 2) / f, yp]
            Ls[2 * i + 1] = [0, -f / z, yp / z, (f ** 2 + yp ** 2) / f, -xp * yp / f, -xp]
        vd = -np.diag(K["lambda_ibvs"]) @ np.linalg.pinv(Ls) @ e
        c = np.linalg.norm(Pi.mean(axis=1))
        ad_z = 1.0 - 1.0 / (1.0 + np.exp(-K["k_sigmoid"] * c))              # adaptive altitude gain
        vd[2] *= ad_z
        vd[:3] = np.clip(vd[:3], -K["v_sat"][:3], K["v_sat"][:3])
        vd[5] = np.clip(vd[5], -K["v_sat"][3], K["v_sat"][3])
        R_V = _rotz(s["yaw"])
        I_v_des = R_V @ (vd[:3] + R_V.T @ s["v_t"])                          # FF of target velocity
        I_F = self.m * K["Kv"] @ (I_v_des - s["v_c"]) - self.m * G_VEC
        return I_F / self.m


BASELINES = {c.name: c for c in (Lin2022, Zhang2026, Lin2023, Cho2022)}


def make_baseline(name, mass):
    name = name.strip().lower()
    if name not in BASELINES:
        raise ValueError(f"unknown baseline '{name}'; choose from {sorted(BASELINES)}")
    return BASELINES[name](mass)
