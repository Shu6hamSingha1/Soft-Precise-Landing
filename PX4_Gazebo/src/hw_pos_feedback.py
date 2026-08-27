#!/usr/bin/env python3
"""ONLINE analytic V-frame feature/flow generator for PLASMC_HW_POS_FEEDBACK,
the HARDWARE analog of gt_feedback.py's Gazebo GT-FEEDBACK scaffold.

Substitutes the perception-derived centroid `s` and optical flow `h` with values
computed analytically from (1) the PX4 EKF's own local position/attitude estimate
(FC.getPosBody()/getQuat(), already NED / body-FRD->NED -- MAVSDK odometry, no ENU
conversion needed unlike the Gazebo GT path) and (2) a FIXED, once-measured marker
NED position (the marker is static on the ground -- no tracked target pose exists
on hardware). Same purpose as gt_feedback.py: isolate the CONTROL problem from the
PERCEPTION ceiling -- but NOTE this is NOT true ground truth like Gazebo's /pose:
it is only as good as the PX4 EKF position estimate, which has its own known
failure modes (see reference_pi_chronic_gps_pdop_drift in project memory). Kept as
a SEPARATE env var (PLASMC_HW_POS_FEEDBACK, not PLASMC_GT_FEEDBACK) so the two
tests are never conflated.

Math mirrors gt_feedback.py's GTFeedback exactly (V-frame construction, Z_REG
depth floor, alpha-sign convention, w_z sign, LS-slope velocity window) --  see
that file's inline comments for the rationale behind each choice. The only
difference is the input frame: PX4 MAVSDK odometry delivers position_body already
in NED and q already as the body(FRD)->NED DCM (verified against img_data.py's
identical `Quaternion([q.w,q.x,q.y,q.z]).to_DCM()` usage for V-frame leveling), so
there is no NED_FROM_ENU / FRD_2_FLU step here. The target has no tracked
orientation (static marker) -- its relative-yaw contribution is 0 by construction.

Outputs match the getters they replace:
  s    (4-vec) = [V_s_x, V_s_y, 1.0, alpha]     <- getImgFeatureParam()
  flow (6-vec) = [h_x, h_y, h_z, w_x, w_y, w_z] <- getOptFlowAngVel()
"""
import os
from collections import deque
import numpy as np
from ahrs import Quaternion

# Camera mount offset, expressed directly in body-FRD (down = +z) since the
# input pose is already native FRD -- no FLU intermediate like gt_feedback.py.
# Same physical mount as Gazebo (x500_mono_cam_down, +0.15 m below base_link
# along body-down) -- see CLAUDE.md's camera section if the mount ever changes.
_CAM_OFF_FRD = np.array([0., 0., float(os.environ.get("PLASMC_GT_CAM_DZ", "0.15"))])

# Marker mount offset above the fixed NED point, also FRD-native. Default 0
# (marker flat on the ground, its measured NED point IS the marker plane).
_MARKER_OFF_FRD = np.array([0., 0., float(os.environ.get("PLASMC_GT_MARKER_DZ", "0.00"))])


def _v_frame(R):
    """body-FRD -> V (gravity-leveled rotz(yaw)). R = body->NED DCM.
    Identical to gt_feedback._v_frame / img_data._getVirtualPts."""
    g = R.T @ np.array([0., 0., 1.])
    z = g / np.linalg.norm(g)
    x = np.cross([0., 1., 0.], z); x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return np.column_stack([x, y, z]).T


def _yaw_from_R(R):
    """NED yaw (rad) directly from a body->NED DCM (ZYX-Euler convention),
    frame-agnostic -- unlike gt_feedback._yaw_of this does NOT assume a Gazebo-
    ENU quaternion input, since R here is already the PX4 body(FRD)->NED DCM."""
    return float(np.arctan2(R[1, 0], R[0, 0]))


def _slope(ts, ys):
    """Least-squares slope dy/dt of each column of ys over the window (causal).
    Identical to gt_feedback._slope."""
    t = np.asarray(ts, float); y = np.asarray(ys, float)
    t = t - t.mean()
    denom = float(t @ t)
    if denom < 1e-9:
        return np.zeros(y.shape[1] if y.ndim > 1 else 1)
    if y.ndim == 1:
        return float((t @ (y - y.mean())) / denom)
    return np.array([t @ (y[:, k] - y[:, k].mean()) / denom for k in range(y.shape[1])])


class HWPosFeedback:
    """Online analytic (EKF-position-based) V-frame s/h generator for hardware.
    Call update() once per control iteration."""

    def __init__(self):
        # Same velocity-window knobs as gt_feedback.GTFeedback (see its __init__
        # comment) -- kept as the SAME env vars so tuning transfers 1:1.
        self._tau = float(os.environ.get("PLASMC_GT_FB_TAU", "0.12"))
        self._win = int(os.environ.get("PLASMC_GT_FB_WIN", "7"))
        _maxlen = 64
        self._t = deque(maxlen=_maxlen)
        self._wx = deque(maxlen=_maxlen)        # W_x history (NED, marker - camera)
        self._ry = deque(maxlen=_maxlen)        # relative-yaw history (rad, unwrapped)
        self._last_ry = None

        # POSITION SLEW-RATE LIMITER (2026-08-17): unlike the perception path
        # (which has outlier rejection + KF smoothing on its raw signal), the raw
        # PX4 EKF position from FC.getPosBody() feeds directly into the LS-slope
        # velocity window below with NO filtering -- a single glitched EKF sample
        # corrupts h(t)'s velocity estimate for the full _tau window. Root-caused
        # 2026-08-17 (Test_Data/Landing/2026-08-17, run "12-26-42"): a discrete
        # ~0.15 m step in the EKF's OWN z-position estimate (not a perception
        # artifact -- confirmed via raw Telemetry_Data, x/y/attitude stayed
        # smooth through the same instant) produced an implied ~150 m/s vertical
        # rate, which an already-inflated adaptive kappa (~25, vs KAPPA0=0.5)
        # amplified into an a_u spike of ~-5964 (should be O(10)). The step was
        # PERSISTENT (didn't revert next tick), so a reject-and-hold-last-good
        # filter would get stuck forever after any genuine EKF correction --
        # slew-RATE limiting instead: clamp the per-tick position change to a
        # physically plausible max speed, so a step (transient OR persistent) is
        # smoothly absorbed over a few ticks rather than injected as an
        # instantaneous velocity spike, while still eventually tracking the true
        # value (never permanently stuck).
        # 5.0 m/s default: real flight telemetry across all 2026-08-17 flights showed a
        # max speed of 3.87 m/s (99.9th pct ~3.5 m/s) -- 5.0 gives margin above the
        # observed envelope while still meaningfully clamping a glitch (the 2026-08-17
        # spike implied ~150 m/s). 10.0 (the first cut of this fix) was too generous --
        # the slew-limiter's own catch-up ramp toward a stepped value implies a
        # "velocity" of up to max_speed itself, so an over-generous limit barely
        # suppresses the very spike it's meant to catch.
        self._max_speed = float(os.environ.get("PLASMC_HW_POS_MAX_SPEED_MPS", "5.0"))
        self._filt_up = None
        self._filt_up_t = None
        self.last_rel_alt = None                # mirrors gt_feedback.GTFeedback.last_rel_alt

    def _vel_window(self):
        """Identical to gt_feedback.GTFeedback._vel_window."""
        ts = np.asarray(self._t, float)
        if self._tau > 0 and len(ts) >= 3:
            keep = ts >= (ts[-1] - self._tau)
            if keep.sum() < 3:
                keep[-3:] = True
            return ts[keep], np.asarray(self._wx)[keep], np.asarray(self._ry)[keep]
        n = min(self._win, len(ts))
        return ts[-n:], np.asarray(self._wx)[-n:], np.asarray(self._ry)[-n:]

    def update(self, uav_pose, target_pose, t):
        """uav_pose, target_pose: objects with .position(.x/.y/.z, NED) and
        .orientation(.w/.x/.y/.z, body-FRD->NED) -- i.e. FC.getPosBody()/getQuat()
        and the fixed marker point, both wrapped into _Vec3/_Quat by HWPoseNode
        below (NOT a raw MAVSDK PositionBody, which uses .x_m/.y_m/.z_m). target_pose
        is the SOLE source of the marker position (HWPoseNode owns parsing
        PLASMC_HW_MARKER_NED_XYZ) -- this class holds no marker state of its own, so
        there is only one place the marker offset is read from.
        t: monotonic loop time (s). Returns (s_4vec, flow_6vec)."""
        qu = uav_pose.orientation
        Ru = Quaternion([qu.w, qu.x, qu.y, qu.z]).to_DCM()   # body-FRD -> NED (already native)
        pos = uav_pose.position
        up_raw = np.array([pos.x, pos.y, pos.z], dtype=float)
        # Slew-rate limit the raw EKF position (see __init__'s comment) before it
        # touches anything -- both this tick's bearing AND the velocity window.
        if self._filt_up is None:
            up = up_raw
        else:
            dt = max(t - self._filt_up_t, 1e-6)
            max_step = self._max_speed * dt
            up = self._filt_up + np.clip(up_raw - self._filt_up, -max_step, max_step)
        self._filt_up = up
        self._filt_up_t = t
        tpp = np.array([target_pose.position.x, target_pose.position.y, target_pose.position.z], dtype=float)

        cam_ned = up + Ru @ _CAM_OFF_FRD                       # camera position, NED
        marker_ned = tpp + _MARKER_OFF_FRD                     # marker position, NED (static, unrotated)

        W_x_tu = marker_ned - cam_ned                          # marker - camera, NED

        # relative yaw (uav - target); target has no tracked orientation -> 0
        ry = _yaw_from_R(Ru)
        if self._last_ry is not None:
            ry = self._last_ry + np.arctan2(np.sin(ry - self._last_ry), np.cos(ry - self._last_ry))
        self._last_ry = ry

        self._t.append(t); self._wx.append(W_x_tu); self._ry.append(ry)

        # Depth-scale regularization -- identical rationale/value to gt_feedback.py
        # (see that file's Z_REG comment block for the full derivation).
        Z_REG = float(os.environ.get("PLASMC_GT_Z_REG", "0.2"))

        # Alpha sign convention -- same env var/semantics as gt_feedback.py.
        _asign = float(os.environ.get("PLASMC_GT_ALPHA_SIGN", "1"))

        # --- centroid bearing s (V-frame) ---
        B_x = Ru.T @ W_x_tu
        V_x = _v_frame(Ru) @ B_x
        _zb = max(float(V_x[2]), 0.0) + Z_REG
        s_xy = np.array([V_x[0] / _zb, V_x[1] / _zb])
        # BEARING CLAMP (2026-08-19): near the ground, true depth (V_x[2]) can dip
        # toward/through the Z_REG floor for a large fraction of low-altitude flight
        # time (not just at touchdown) -- when it does, s_xy's magnitude can spike
        # 2x+ in a single control tick (verified against 2026-08-19 hardware flight
        # data: s_e_n climbing from ~10 to ~20+ in one sample right as altitude
        # crossed ~0.49m), which the kappa-ODE reacts to as if it were a genuine
        # large tracking error -- kappa detonates and pins high for 10-28s, starving
        # the vertical channel of command authority (a_u_xy climbed into the
        # hundreds while a_u_z stayed near-flat) and stalling descent at 0.3-0.5m.
        # This is the SAME class of fix as the position slew-rate limiter above
        # (bound a physically-implausible instantaneous jump at its source) rather
        # than raising Z_REG globally, which would also soften genuine far-range
        # bearings. Default 8.0: observed max |s| in flights that did NOT detonate
        # kappa stayed under ~3; 8.0 gives headroom for real large-offset ICs while
        # still cutting off the 10-87 magnitude spikes seen in the stalled runs.
        _s_max = float(os.environ.get("PLASMC_HW_S_MAX", "8.0"))
        _s_mag = float(np.linalg.norm(s_xy))
        if _s_mag > _s_max:
            s_xy = s_xy * (_s_max / _s_mag)
        alpha = float(np.arctan2(np.sin(_asign * ry), np.cos(_asign * ry)))
        s4 = np.array([s_xy[0], s_xy[1], 1.0, alpha])

        # --- flow h (V-frame rel velocity / depth) + w_z (rel yaw rate) ---
        zB = max(float(W_x_tu[2]), 0.0)
        self.last_rel_alt = zB
        h = np.zeros(3); w = np.zeros(3)
        if len(self._t) >= 3:
            ts, wx, ry_arr = self._vel_window()
            W_v_tu = _slope(ts, wx)
            B_v = Ru.T @ W_v_tu
            V_v = _v_frame(Ru) @ B_v
            h = V_v / (zB + Z_REG)
            w[2] = -_asign * _slope(ts, ry_arr)   # w_z = -d(alpha)/dt, same sign relation as gt_feedback.py
        flow6 = np.array([h[0], h[1], h[2], w[0], w[1], w[2]])
        return s4, flow6


class _Vec3:
    __slots__ = ("x", "y", "z")
    def __init__(self, x, y, z):
        self.x, self.y, self.z = x, y, z


class _Quat:
    __slots__ = ("w", "x", "y", "z")
    def __init__(self, w, x, y, z):
        self.w, self.x, self.y, self.z = w, x, y, z


class _Pose:
    __slots__ = ("position", "orientation")
    def __init__(self, position, orientation):
        self.position, self.orientation = position, orientation


class _PoseData:
    __slots__ = ("UAV", "target")
    def __init__(self, UAV, target):
        self.UAV, self.target = UAV, target


class HWPoseNode:
    """Drop-in analog of gz_subscriber.Pose_Node for hardware: getPose().UAV is
    built live from the FC's own EKF odometry, getPose().target is the marker
    point (identity orientation -- static, non-rotating). Lets controller.py's
    existing `pose_node.getPose().UAV/.target` call sites (used by the
    GT-FEEDBACK integration in startController()) work unchanged.

    MARKER POSITION (2026-08-17, replaces the old "just trust
    PLASMC_HW_MARKER_NED_XYZ=0,0,0" default): a hardcoded (0,0,0) assumes the
    vehicle's PX4 LOCAL NED ORIGIN coincides with the marker -- false in
    practice even when the vehicle is physically powered on and sitting
    exactly on the marker. PX4's local origin anchors to the EKF's GLOBAL
    position estimate at the moment the origin is first set (first usable GPS
    fix), which carries ordinary GPS accuracy error (commonly 2-5 m); the
    estimate then keeps refining for the next several minutes as satellite
    geometry improves, drifting getPosBody() away from that now-stale origin
    even with ZERO physical movement (same root cause as
    project_pi_chronic_gps_pdop_drift / the "wait ~5min after power-up"
    lesson from project_pi_aug10_crash_offboard_rc_race). Confirmed
    empirically 2026-08-17: every one of 12 flights that day showed a
    1.3-4.1 m mismatch between assumed (0,0,0) and the vehicle's own
    position at flight start, matching ordinary GPS error magnitude.

    FIX: since the vehicle genuinely does sit on the marker from power-on
    through arming, snapshotMarkerFromCurrentPosition() (call this ONCE,
    immediately before arming) reads getPosBody() AT THAT MOMENT and uses it
    as the marker's NED position for the whole flight -- self-relative, so
    correct regardless of how far the origin itself has drifted since boot.
    PLASMC_HW_MARKER_NED_XYZ remains available as an explicit override (set
    it if the marker is NOT where the vehicle takes off from) -- if the env
    var is present at construction time, the snapshot call becomes a no-op.

    RANGEFINDER Z-ANCHORING -- TRIED AND REVERTED (2026-08-17): a
    PLASMC_HW_USE_RANGEFINDER_Z=1 mode was added and validated offline against
    Test_Data/FlightLogs/2026-08-17/12_45_54.ulg by replaying both paths
    through this exact class. Verdict: WORSE than the EKF+slew-limiter path,
    not better -- this airframe's distance_sensor topic only publishes at
    ~1 Hz, so holding z constant between updates injects a fresh step into the
    LS-slope velocity window on every update cycle (h_z std 0.837 vs 0.550,
    max|h_z| 7.04 vs 3.64 for the EKF path). Don't re-add this without first
    confirming a materially higher rangefinder publish rate is achievable."""

    def __init__(self, fc):
        self._fc = fc
        self._explicit_override = "PLASMC_HW_MARKER_NED_XYZ" in os.environ
        _mx, _my, _mz = (float(v) for v in
                          os.environ.get("PLASMC_HW_MARKER_NED_XYZ", "0,0,0").split(","))
        self._target = _Pose(_Vec3(_mx, _my, _mz), _Quat(1., 0., 0., 0.))

    def snapshotMarkerFromCurrentPosition(self):
        """Call ONCE, immediately before arming, while the vehicle still sits
        on the marker. No-op if PLASMC_HW_MARKER_NED_XYZ was explicitly set
        (that override takes precedence over the live snapshot)."""
        if self._explicit_override:
            return
        pb = self._fc.getPosBody()
        self._target = _Pose(_Vec3(pb.x_m, pb.y_m, pb.z_m), _Quat(1., 0., 0., 0.))
        print(f"[hw_pos_feedback] marker position snapshotted from current pose: "
              f"({pb.x_m:.3f}, {pb.y_m:.3f}, {pb.z_m:.3f}) NED")

    def getPose(self):
        pb = self._fc.getPosBody()
        q = self._fc.getQuat()
        uav = _Pose(_Vec3(pb.x_m, pb.y_m, pb.z_m), _Quat(q.w, q.x, q.y, q.z))
        return _PoseData(UAV=uav, target=self._target)
