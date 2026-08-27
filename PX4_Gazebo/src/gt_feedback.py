#!/usr/bin/env python3
"""ONLINE ground-truth V-frame feature/flow generator for PLASMC_GT_FEEDBACK.

Substitutes the perception-derived centroid `s` and optical flow `h` with their
EXACT ground-truth values computed from the Gazebo GT poses (UAV + target). This
isolates the CONTROL problem (can the law drive s_e_n->0 and h_e->0?) from the
PERCEPTION ceiling (LK saturation / decode dropout / loom under-report).

Convention is IDENTITY-matched to the perception path the controller consumes
(verified offline in tools/validate_gt_feedback.py on a clean rep: GT V_s_g/V_h_g
correlate +0.83..+0.95 with the recorded s(t)/h(t) under the identity map; the
perception slope<1 is the under-report this scaffold removes). So the GT features
are fed RAW (slope 1), no rotation/flip.

The V-frame and NED conventions mirror tools/gt_optical_flow.py::compute_gt_flow
and img_data._getVirtualPts exactly (gravity-leveled rotz(yaw); body-FRD->NED via
NED_FROM_ENU @ Rfu @ FRD_2_FLU). Velocity (for h) is a smoothed online slope over
a short time-stamped window (offline uses a non-causal savgol; online a causal
least-squares line over WIN samples).

Outputs match the getters they replace:
  s    (4-vec) = [V_s_x, V_s_y, 1.0, alpha]     <- getImgFeatureParam()
  flow (6-vec) = [h_x, h_y, h_z, w_x, w_y, w_z] <- getOptFlowAngVel()
(alpha = GT relative yaw uav-target; w_z = d(alpha)/dt; w_x,w_y are typically
 zeroed downstream by W_XY_DEROT='zero', but provided as the V-frame body rate.)
"""
import os
from collections import deque
import numpy as np
from ahrs import Quaternion

NED_FROM_ENU = np.array([[0., 1., 0.], [1., 0., 0.], [0., 0., -1.]])   # self-inverse
FRD_2_FLU    = np.diag([1., -1., -1.])                                  # DCM(x=180°), self-inverse

# Rigid mount offsets (from the Gazebo model SDFs), expressed in each body's
# FLU/model frame (z up). The pose feedback gives the base_link / target-model
# ORIGIN poses, but the camera and (on some targets) the marker are mounted off
# those, so the controller's target = MARKER as seen by the CAMERA needs the
# relative vector (marker - camera), not (target_origin - uav_base).
#   - CAMERA: x500_mono_cam_down mono_cam at <pose>0 0 .15 ...> -> +0.15 m above
#     base_link. UNIVERSAL (same drone in every world) -> default 0.15. (Was 0.20 until
#     the 2026-08-05 camera-mount Z reduction (x500_mono_cam_down/model.sdf, .20->.18->.15
#     -- keeps the landing legs out of the downward FoV); this default silently went
#     stale for 3 days since nothing re-checks it against the live SDF. Re-verify here
#     whenever the SDF's camera pose changes again -- see CLAUDE.md's camera section.)
#   - MARKER: WORLD-SPECIFIC. The stationary `aruco` world's arucotag is FLAT on
#     the ground (target origin AT the marker plane, z~0) -> offset 0. The rover
#     mounts the marker +0.50 m up (rover_aruco SDF) -> the rover launcher sets
#     PLASMC_GT_MARKER_DZ=0.5. So the DEFAULT is 0.0 (flat-marker worlds); do NOT
#     hardcode 0.5 or it biases every non-rover target's depth.
# FLU here; converted to FRD (FRD_2_FLU is self-inverse) before rotating by the
# body->NED DCM.
_CAM_OFF_FLU    = np.array([0., 0., float(os.environ.get("PLASMC_GT_CAM_DZ",    "0.15"))])
_MARKER_OFF_FLU = np.array([0., 0., float(os.environ.get("PLASMC_GT_MARKER_DZ", "0.00"))])


def _v_frame(R):
    """body-FRD -> V (gravity-leveled rotz(yaw)). R = body->NED DCM.
    Identical to gt_optical_flow._v_frame / img_data._getVirtualPts."""
    g = R.T @ np.array([0., 0., 1.])
    z = g / np.linalg.norm(g)
    x = np.cross([0., 1., 0.], z); x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return np.column_stack([x, y, z]).T


def _yaw_of(q):
    """NED yaw (rad) from a geometry_msgs quaternion (Gazebo ENU body)."""
    return Quaternion([q.w, q.x, q.y, q.z]).to_angles()[2]


def _slope(ts, ys):
    """Least-squares slope dy/dt of each column of ys over the window (causal)."""
    t = np.asarray(ts, float); y = np.asarray(ys, float)
    t = t - t.mean()
    denom = float(t @ t)
    if denom < 1e-9:
        return np.zeros(y.shape[1] if y.ndim > 1 else 1)
    if y.ndim == 1:
        return float((t @ (y - y.mean())) / denom)
    return np.array([t @ (y[:, k] - y[:, k].mean()) / denom for k in range(y.shape[1])])


class GTFeedback:
    """Online GT V-frame s/h generator. Call update() once per control iteration."""

    def __init__(self):
        # Velocity = LS-slope over a TIME window (rate-independent, robust to /pose
        # jitter and repeated control-rate reads). PLASMC_GT_FB_TAU (s) sets the
        # window; longer = smoother loom (vz/Z) at small lag (descent is slow), the
        # binding need for the z-axis. PLASMC_GT_FB_WIN kept as a fixed-count fallback
        # (TAU=0 selects it).
        self._tau = float(os.environ.get("PLASMC_GT_FB_TAU", "0.12"))
        self._win = int(os.environ.get("PLASMC_GT_FB_WIN", "7"))
        _maxlen = 64
        self._t = deque(maxlen=_maxlen)
        self._wx = deque(maxlen=_maxlen)        # W_x history (NED, marker - camera)
        self._ry = deque(maxlen=_maxlen)        # relative-yaw history (rad, unwrapped)
        self._last_ry = None
        self.last_rel_alt = None                # most recent GT camera-marker relative
                                                  # altitude (m, >=0), for GT-only touchdown
                                                  # detection -- see controller.py's
                                                  # _touchdownDetect GT-FEEDBACK path
        # (The 2026-07-02 target-acceleration FF / lead-pursuit estimator was
        # REMOVED per user: the "curved-translation lag" it targeted turned out
        # to be a self-sustained lateral LIMIT CYCLE, not a lag — and the FF
        # consumed target-pose derivatives forbidden by the manuscript Problem
        # Statement anyway. See [[project_rover_turning_open]] for the oracle-
        # bound results (steady 0.9-1.6 -> 0.31-0.51 m) retained as an ablation.)

        # SYNTHETIC TARGET SPIN (PLASMC_GT_SPIN_WZ, rad/s, default 0 = off):
        # add wz_spin*(t-t0) to the target yaw, i.e. a target rotating IN PLACE
        # with NO translation. Pure-rotation isolation experiment for the
        # turning-rover problem ([[project_rover_turning_open]]): the Ackermann
        # rover physically cannot spin in place (min turn radius ~0.56 m), and a
        # circular drive convolves rotation with the translation-lag geometry.
        # alpha AND w_z stay self-consistent automatically (both flow from ry;
        # the slope estimator picks the spin up exactly like a real rotation).
        self._spin_wz = float(os.environ.get("PLASMC_GT_SPIN_WZ", "0.0"))
        self._spin_t0 = None

    def _vel_window(self):
        """Return (ts, wx_arr, ry_arr) restricted to the velocity window: last _tau
        seconds (>=3 samples), or the last _win samples if _tau<=0."""
        ts = np.asarray(self._t, float)
        if self._tau > 0 and len(ts) >= 3:
            keep = ts >= (ts[-1] - self._tau)
            if keep.sum() < 3:
                keep[-3:] = True
            return ts[keep], np.asarray(self._wx)[keep], np.asarray(self._ry)[keep]
        n = min(self._win, len(ts))
        return ts[-n:], np.asarray(self._wx)[-n:], np.asarray(self._ry)[-n:]

    def update(self, uav_pose, target_pose, t):
        """uav_pose, target_pose: geometry_msgs Pose (Gazebo ENU). t: sim time (s).
        Returns (s_4vec, flow_6vec) in the controller's consumed convention."""
        qu, qt = uav_pose.orientation, target_pose.orientation
        Rfu = Quaternion([qu.w, qu.x, qu.y, qu.z]).to_DCM()
        Ru  = NED_FROM_ENU @ Rfu @ FRD_2_FLU                      # UAV body-FRD -> NED
        Rft = Quaternion([qt.w, qt.x, qt.y, qt.z]).to_DCM()
        Rt  = NED_FROM_ENU @ Rft @ FRD_2_FLU                      # target body-FRD -> NED
        up  = NED_FROM_ENU @ np.array([uav_pose.position.x, uav_pose.position.y, uav_pose.position.z])
        tpp = NED_FROM_ENU @ np.array([target_pose.position.x, target_pose.position.y, target_pose.position.z])
        # Apply the rigid mount offsets so the relative vector is MARKER w.r.t.
        # CAMERA (not rover-base w.r.t. uav-base). Each offset is rotated by its
        # body attitude, so camera/marker translate correctly as the drone tilts
        # or the rover pitches (FLU offset -> FRD via the self-inverse FRD_2_FLU).
        cam_ned    = up  + Ru @ (FRD_2_FLU @ _CAM_OFF_FLU)         # camera position, NED
        marker_ned = tpp + Rt @ (FRD_2_FLU @ _MARKER_OFF_FLU)      # marker position, NED

        W_x_tu = marker_ned - cam_ned                             # marker - camera, NED

        # relative yaw (uav - target), unwrapped across calls for a clean rate
        ry = _yaw_of(qu) - _yaw_of(qt)
        if self._spin_wz != 0.0:
            # synthetic in-place target spin (see __init__): target yaw advances
            # at wz_spin, so the RELATIVE yaw loses wz_spin*(t-t0).
            if self._spin_t0 is None:
                self._spin_t0 = t
            ry -= self._spin_wz * (t - self._spin_t0)
        if self._last_ry is not None:
            ry = self._last_ry + np.arctan2(np.sin(ry - self._last_ry), np.cos(ry - self._last_ry))
        self._last_ry = ry

        self._t.append(t); self._wx.append(W_x_tu); self._ry.append(ry)

        # Depth-scale regularization: the quadrotor's LANDING GEAR keeps the true
        # relative depth z bounded away from 0 (z >= z_gear), so beta=1/z is physically
        # bounded (Lyapunov Assumption 1 holds). Compute EVERY 1/z-dependent feature with
        # 1/(z+Z_REG) so the COMPUTED beta honors that bound and never spikes from a
        # transiently-small depth (Z_REG=0.01 is well below z_gear, so it only floors the
        # numerics; the gear sets the actual bound). Bearing and loom use the SAME floored
        # depth for consistency. (V_x[2] = gravity-leveled depth ≈ zB = NED rel altitude.)
        # Z_REG = the PHYSICAL floor on relative depth z (the gear/touchdown floor). The descent
        # bottoms at z>=z_floor, so the true beta=1/z is bounded (Lyapunov Assumption 1 holds); it's
        # ALSO ~where image perception saturates (marker fills the FoV). So 1/(z+Z_REG) makes the
        # GT-synthesized features FAITHFUL to real perception, NOT a lie. BAKED 0.01->0.1 (2026-06-30):
        # the old Z_REG=0.01 let the COMPUTED z fall to 0.01 m -> a non-physical 1/z->100 -> the terminal
        # kappa_eq explosion / limit cycle was a GT-FB ARTIFACT. EMPIRICAL: across 27 reps the min relative
        # z = |UAV_z-Target_z| medians 0.096 m (TOUCHDOWN_LOOM fires z~0.11-0.14) -> the real floor is
        # ~0.1 m (NOT the 0.2 m gear-height first guessed; that over-clamped). Z_REG=0.1 caps 1/z at 10
        # (matches the measured floor) -> bounded disturbance -> leakage-ASMC in design envelope -> SP.
        # (Additive 1/(z+0.1) under-reads ~9% at z=1 m; 1/max(z,0.1) is exact if the altitude bias bites.)
        # NOTE (2026-07-02): the camera/marker mount offsets are now applied
        # explicitly above (W_x_tu = marker - camera), so Z_REG is a pure depth
        # floor on the TRUE camera-to-marker depth, no longer a fudge that also
        # absorbs the mount offset. Measured min first-descent camera-marker
        # depth ~0.10 m, so Z_REG may warrant revisiting (0.2 -> ~0.1); left at
        # 0.2 pending a sweep. (Pre-fix rationale "0.2 sits between base_link 0.1
        # and camera 0.3" is superseded — that offset is no longer double-counted.)
        Z_REG = float(os.environ.get("PLASMC_GT_Z_REG", "0.2"))

        # Alpha sign convention (PLASMC_GT_ALPHA_SIGN, default +1 = historical):
        # GT-FB feeds alpha = SIGN * ry, ry = yaw(uav) - yaw(target). The perception
        # moment-alpha the yaw chain was tuned on is ANTI-correlated with drone yaw
        # (BODY_YAW_ALPHA_K = -0.949), i.e. the perception convention is ~ -ry.
        # With +ry the alpha-servo cascade is still self-consistent at e_a~0 (the
        # stationary/straight-rover landings converge), but a persistently ROTATING
        # target drives the loop through the inverted disturbance path -> yaw
        # runaway to the +-180 antipode (Circular rover, 2026-07-02). SIGN=-1
        # matches the perception convention (cousin of the 2026-06-25 w_z sign
        # fix). w_z below keeps the VALIDATED relation w_z = -d(alpha)/dt in
        # either convention.
        _asign = float(os.environ.get("PLASMC_GT_ALPHA_SIGN", "1"))

        # --- centroid bearing s (V-frame) ---
        B_x = Ru.T @ W_x_tu
        V_x = _v_frame(Ru) @ B_x
        # Depth must be NON-NEGATIVE: the UAV is physically above the target (gear keeps
        # z>0), but a transient/post-touchdown GT glitch could give z<0, which would make
        # (z+Z_REG) flip sign -> wrong-signed bearing/loom. Clamp z>=0 so the regularized
        # depth stays in [Z_REG, inf) (strictly positive) and the feature sign is correct.
        _zb = max(float(V_x[2]), 0.0) + Z_REG                      # regularized, non-negative V-frame depth
        s_xy = np.array([V_x[0] / _zb, V_x[1] / _zb])
        alpha = float(np.arctan2(np.sin(_asign * ry), np.cos(_asign * ry)))   # wrapped rel-yaw feature (sign per convention above)
        s4 = np.array([s_xy[0], s_xy[1], 1.0, alpha])

        # --- flow h (V-frame rel velocity / depth) + w_z (rel yaw rate) ---
        zB = max(float(W_x_tu[2]), 0.0)                            # rel altitude, non-negative
        self.last_rel_alt = zB
        h = np.zeros(3); w = np.zeros(3)
        if len(self._t) >= 3:                                      # no altitude gate: 1/(z+Z_REG) stays bounded to the deck
            ts, wx, ry_arr = self._vel_window()
            W_v_tu = _slope(ts, wx)                                 # NED rel velocity (time-windowed LS)
            B_v = Ru.T @ W_v_tu
            V_v = _v_frame(Ru) @ B_v
            h = V_v / (zB + Z_REG)
            # SIGN FIX (2026-06-25): the rotational optic flow is w_z = -alpha_dot, NOT +alpha_dot.
            # Validated vs IMU: the perception lstsq w_z correlates -0.91 with the body yaw rate
            # (w_z = -psi_dot_b, per manuscript w = ^Vw_t - psi_dot_b*e3); alpha_dot = +psi_dot_b for a
            # stationary target, so alpha_dot = -w_z. The old +d(alpha)/dt fed w_z with the OPPOSITE sign
            # to the perception path, flipping the h_d rotation FF (cross(w,s)) and the old c-term's
            # w-cross-products (omega_dot x s, 2 w x h) -> spurious anti-restoring feedforward.
            w[2] = -_asign * _slope(ts, ry_arr)                     # w_z = -d(alpha)/dt (alpha = _asign*ry) — validated relation kept in either sign convention
        flow6 = np.array([h[0], h[1], h[2], w[0], w[1], w[2]])
        return s4, flow6
