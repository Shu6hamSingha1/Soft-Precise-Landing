#!/usr/bin/env python3
"""Derive the Pi hardware sensor-calibration matrix from output_calibration.py
mocap recordings. Pi-adapted port of PX4_Gazebo/tools/gt_optical_flow.py's
canonical GT-flow method + derive_board_cal.py's lstsq fit, for QTM mocap
instead of Gazebo's simulated ROS Pose/quaternion GT.

CONFIRMED FRAME TRANSFORM (user, 2026-07-10, "match drone FRD with mocap
XYZ" calibration run): QTM reports X=Forward, Y=Left, Z=Up (right-handed
FLU, standard optical-mocap convention). The drone/controller body frame is
FRD (X=Forward, Y=Right, Z=Down). FLU -> FRD is a 180 degree rotation about
the shared X (forward) axis:
    pos_FRD = diag(1, -1, -1) @ pos_QTM
    yaw_FRD = -yaw_QTM   (rotation about the flipped Z axis also flips sign)
    R_FRD (body->world) = T @ R_FLU(euler) @ T,  T = diag(1,-1,-1)
(T is orthogonal and self-inverse, so this is a valid similarity transform
applying the SAME axis flip to both the world and body sides of the DCM -
correct because mocaptools reports body roll/pitch/yaw in QTM's own FLU
convention, not already in FRD.)

Gravity ("down") is +Z in this FRD-consistent frame, matching Gazebo's NED
convention exactly, so the rest of the V-frame math (_v_frame, gravity-
leveled bearing/flow) is a direct port with no further changes needed.

*** PROVISIONAL / FIRST-PASS: the calibration below has NOT been validated
against a second independent recording, R^2 has not been reviewed, and it
must not be pasted into img_data.py without that review (see PX4_Gazebo's
own workflow: derive -> inspect R^2/inter-run stability -> THEN apply). ***

FILTER: KF, not Savgol (fixed 2026-07-11, mirroring a bug PX4_Gazebo found
and fixed the same day, feedback_kf_savgol_cal_mismatch). img_data.py's
runtime getOptFlowAngVel() defaults to IMG_FILTER='kf' - fitting a
calibration against a Savgol-filtered raw signal and applying it to a
differently-shaped KF-filtered signal at runtime is not guaranteed valid.
kf_filter_causal() below is a direct port of aggregate_calibration_phased.py's
version, which is itself kept in lockstep with img_data.py::_kf_step - same
process/measurement model, using the Pi's own FLOW_KF_Q/R and
IMG_FEAT_KF_Q/R defaults (5.0/0.1 and 5.0/0.004).

Usage:
    python3 derive_pi_cal.py [<run_dir> ...]     # defaults to all GOOD runs
                                                   # under Test_Data/Calibration
"""
import os
import sys
import glob
import numpy as np
from scipy.signal import savgol_filter as sgf   # GT position->velocity smoothing only
from img_geometry import EGO_MOTION_ROT_SLOPE   # -1.0, validated 2026-07-23 - see its own comment
from img_geometry import vframe_w   # body-FRD gyro -> V-frame, for check_mount_rotation's fixed comparison

CAL_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "Test_Data", "Calibration")

T_QTM_TO_FRD = np.diag([1.0, -1.0, -1.0])   # FLU -> FRD, 180deg about shared X

# ---------------------------------------------------------------------------
# LEVER ARMS (user-measured 2026-07-27, given in MOCAP FLU: X=fwd, Y=left, Z=up)
#
#   camera optical centre -> UAV mocap origin : X +0.08, Z +0.16  m
#   ArUco centre          -> target mocap origin: X -0.012, Y -0.034 m (RE-MEASURED
#     2026-07-28, superseding the original -0.04/-0.01 - prompted by 3 independent
#     lever-arm-certification checks all preferring mrk_y more negative than -0.01
#     and mrk_x closer to zero than +0.04; see project_pi_gt_lever_arm_2026_07_27)
#
# Those are stated as "A -> B" = the vector FROM A TO B. What the geometry
# needs is the opposite: origin -> feature, so each is NEGATED below before
# the FLU->FRD flip.
#
#   camera:  origin -> cam    = -(+0.08, 0, +0.16) FLU = (-0.08, 0, -0.16) FLU
#                             -> T @ ... = (-0.08, 0, +0.16) FRD
#            i.e. the camera sits 16 cm BELOW (+z is down in FRD) and 8 cm AFT
#            of the UAV mocap origin - physically consistent with a
#            downward-facing camera slung under the airframe.
#   marker:  origin -> aruco  = -(-0.012, -0.034, 0) FLU = (+0.012, +0.034, 0) FLU
#                             -> T @ ... = (+0.012, -0.034, 0) FRD
#
# Sign convention verified empirically, not just asserted: applying these
# collapses the GT-vs-logged-corner residual (see the CHECK in __main__ /
# project_pi_gt_lever_arm_2026_07_27). If a future re-measure flips a sign the
# residual will BLOW UP rather than fail quietly, so re-run that check.
#
# cam_z DROPPED (2026-07-28), despite being user-measured at +0.16 m. It is
# STRUCTURALLY DEGENERATE with the bearing's own scale: it enters
# W = (target + ...) - (up + Ru@cam), and its z-component shifts the
# effective depth Vz the SAME way an additive epsilon in 1/(Vz+eps) would -
# which is exactly the arbitrary "+0.2" term just removed on principle
# elsewhere in this file (see the 1/Vz commit). Keeping a value here that the
# fit can neither confirm nor refute (a profile scan over it rails to
# whatever bound is given, never converging - see
# project_pi_gt_lever_arm_2026_07_27) would be inconsistent with having just
# thrown out the epsilon for the same reason. Measured directly: dropping it
# is not just "unconfirmable", it is marginally BETTER (mean residual against
# Virtual Feature Pts, 6 runs: 0.109 with cam_z=0.16 -> 0.104 with cam_z=0).
# cam_x is KEPT - its SIGN is independently confirmed (robust across both the
# flawed and corrected comparand, see project_pi_gt_lever_arm_2026_07_27) and
# unlike z it is not scale-degenerate, so it is a real (if imprecisely
# measured) correction rather than a disguised epsilon.
# Env-overridable in metres, FRD, as "x,y,z".
def _lever(env, default_frd):
    v = os.environ.get(env)
    return np.array([float(x) for x in v.split(",")], float) if v else np.array(default_frd, float)

R_CAM_FRD    = _lever("CAL_CAM_LEVER_FRD",    [-0.08, 0.0, 0.0])     # UAV origin -> camera (z omitted, see above)
R_MARKER_FRD = _lever("CAL_MARKER_LEVER_FRD", [+0.012, -0.034, 0.0])   # target origin -> ArUco centre (re-measured 2026-07-28)
LAB = ['Hx', 'Hy', 'Hz', 'Wx', 'Wy', 'Wz']
RL = ['h0', 'h1', 'h2', 'w0', 'w1', 'w2']
FLOW_KF_Q = float(os.environ.get("FLOW_KF_Q", "5.0"))       # matches img_data.py default
FLOW_KF_R = float(os.environ.get("FLOW_KF_R", "0.1"))
FEAT_KF_Q = float(os.environ.get("IMG_FEAT_KF_Q", "5.0"))
FEAT_KF_R = float(os.environ.get("IMG_FEAT_KF_R", "0.004"))
# Confirmed 2026-07-11 (check_loop_staleness.py): every recording has 7-10
# marker-loss gaps of 1-18s (KLT fallback is capped at MARKER_KLT_MAX_STEPS
# and can't cover these). The flow sample immediately after such a gap is
# unreliable for TWO independent reasons: (1) it may reflect a large
# spurious pixel jump from marker reacquisition, not real target motion, and
# (2) kf_filter_causal's causal KF state was built on data from before the
# gap - see GAP_RESET_S below. Excluded from the FIT, not from the array
# (the KF still needs to run over every sample in order).
GAP_EXCLUDE_S = float(os.environ.get("CAL_GAP_EXCLUDE_S", "1.0"))
# kf_filter_causal used to silently CAP dt at 0.1s regardless of the true
# gap (max(min(ti-prev_t, 0.1), 1e-3)) - so after an 18s marker-loss gap it
# still fed the filter dt=0.1, barely inflating uncertainty and blending a
# stale pre-gap velocity estimate into the post-gap output. Gaps at or above
# this threshold now RESET the filter (re-initialize, like the very first
# sample) instead of predicting across them.
GAP_RESET_S = float(os.environ.get("CAL_GAP_RESET_S", "1.0"))
# Number of real-tagged S samples immediately after a coast run to exclude as
# a reacquisition transient (see the settle-guard comment in derive_one()).
SETTLE_N = int(os.environ.get("CAL_SETTLE_N", "3"))


def kf_filter_causal(raw, t, q, r):
    """Causal constant-velocity 2-state KF per channel, run over a full
    (N, C) raw array + matching timestamps t (N,). Ported verbatim from
    PX4_Gazebo/tools/aggregate_calibration_phased.py::kf_filter_causal,
    which itself mirrors img_data.py::_kf_step exactly - if that function's
    model ever changes, mirror the change here too. Returns (N, C) filtered."""
    raw = np.asarray(raw, dtype=float)
    t = np.asarray(t, dtype=float)
    n, c = raw.shape
    out = np.zeros_like(raw)
    x = np.zeros((c, 2)); P = np.tile(np.eye(2), (c, 1, 1))
    initialized = False
    prev_t = 0.0
    for i in range(n):
        z = raw[i]; ti = t[i]
        # A gap this long means whatever the filter believed before is
        # stale (marker was lost, not just a slow frame) - re-initialize
        # rather than predicting across it with a capped/fake dt.
        if initialized and (ti - prev_t) >= GAP_RESET_S:
            initialized = False
        if not initialized:
            x[:, 0] = z; x[:, 1] = 0.0
            P = np.tile(np.eye(2) * 1.0, (c, 1, 1))
            prev_t = ti; initialized = True
            out[i] = z
            continue
        dt = max(min(ti - prev_t, 0.1), 1e-3)
        prev_t = ti
        F = np.array([[1.0, dt], [0.0, 1.0]])
        Q = q * np.array([[dt**4 / 4.0, dt**3 / 2.0],
                           [dt**3 / 2.0, dt**2]])
        x_pred = x @ F.T
        P_pred = F @ P @ F.T + Q
        y = z - x_pred[:, 0]
        S = P_pred[:, 0, 0] + r
        K = P_pred[:, :, 0] / S[:, None]
        x = x_pred + K * y[:, None]
        P = P_pred - K[:, :, None] * P_pred[:, 0:1, :]
        out[i] = x[:, 0]
    return out


def reject_outliers(raw, window=7, k=5.0, label=""):
    """Hampel-filter sanity guard for the offline derivation, separate from
    the runtime checkposts (which are intentionally disabled during
    calibration recording, per the recording-must-be-truly-raw fix - so
    nothing filters a genuine numerical spike either, e.g. an
    ill-conditioned lstsq solve right at a marker reacquisition).

    Per-channel: for each sample, compute the median and MAD (median
    absolute deviation) over a `window`-sample local neighborhood centered
    on it, and flag the sample an outlier if it deviates from that LOCAL
    median by more than k * 1.4826 * MAD (1.4826 scales MAD to be
    consistent with a Gaussian std, the standard Hampel-filter constant).
    This is more robust than a fixed absolute threshold: it adapts to each
    channel's own local noise level and signal trend, rather than assuming
    one global cutoff is meaningful across every channel and every
    recording's excitation level - and median/MAD are themselves outlier-
    resistant (unlike mean/std, which the outlier being detected would
    itself skew). Flagged samples are replaced by linear interpolation from
    the nearest valid (non-outlier) neighbors, so a single bad frame can't
    bleed into the causal KF's subsequent samples via a garbage
    measurement. Confirmed need 2026-07-11: one row (h_z=-21.7, everything
    else in a sane ~[-2,2] range) found via a data-integrity check on a
    real recording."""
    raw = np.array(raw, dtype=float, copy=True)
    n, c = raw.shape
    idx = np.arange(n)
    half = window // 2
    total_rejected = 0
    for ch in range(c):
        x = raw[:, ch]
        med = np.empty(n)
        mad = np.empty(n)
        for i in range(n):
            lo, hi = max(0, i - half), min(n, i + half + 1)
            w = x[lo:hi]
            med[i] = np.median(w)
            mad[i] = np.median(np.abs(w - med[i]))
        scale = 1.4826 * mad
        # A locally-flat/quiet window gives scale~0, which would flag any
        # tiny wobble as an outlier - fall back to this channel's overall
        # typical scale rather than an arbitrarily tight zero-tolerance band.
        _valid_scale = scale[scale > 1e-6]
        _fallback = float(np.median(_valid_scale)) if len(_valid_scale) else 1e-3
        scale = np.where(scale < 1e-6, _fallback, scale)
        bad = np.abs(x - med) > k * scale
        nbad = int(bad.sum())
        if nbad == 0:
            continue
        total_rejected += nbad
        good = ~bad
        if good.sum() == 0:
            continue   # entire channel is garbage - nothing to interpolate from
        raw[bad, ch] = np.interp(idx[bad], idx[good], raw[good, ch])
    if total_rejected:
        print(f"  outlier guard{(' ('+label+')') if label else ''}: "
              f"{total_rejected} sample(s) rejected (Hampel filter, window={window}, k={k})")
    return raw


def _euler_to_dcm_flu(roll_deg, pitch_deg, yaw_deg):
    """Standard aerospace ZYX Euler -> body->world DCM, in QTM's own FLU
    convention (roll about X-forward, pitch about Y-left, yaw about Z-up)."""
    r, p, y = np.deg2rad([roll_deg, pitch_deg, yaw_deg])
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx   # body -> world (FLU)


def _pose_to_frd(pose):
    """mocaptools.Pose (QTM FLU, degrees) -> (pos_FRD(3,), R_FRD body->world(3,3))."""
    pos_qtm = np.array([pose.x, pose.y, pose.z])
    pos_frd = T_QTM_TO_FRD @ pos_qtm
    R_flu = _euler_to_dcm_flu(pose.roll, pose.pitch, pose.yaw)
    R_frd = T_QTM_TO_FRD @ R_flu @ T_QTM_TO_FRD
    return pos_frd, R_frd


def _v_frame(R):
    """Gravity-leveled V-frame rotation (body-FRD -> V). R = body->world DCM,
    +Z = down (gravity direction) - identical convention to Gazebo's
    gt_optical_flow._v_frame, now that the QTM data is FRD-consistent."""
    g = R.T @ np.array([0., 0., 1.])
    z = g / np.linalg.norm(g)
    x = np.cross([0., 1., 0.], z); x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return np.column_stack([x, y, z]).T


def _robust_vel(x, t):
    """d/dt via uniform-dt interp -> savgol -> gradient -> interp back
    (ports gt_optical_flow._robust_vel - don't differentiate raw jittery
    mocap position directly, it amplifies noise into velocity)."""
    n = len(t)
    if n < 6:
        return np.zeros_like(x)
    tu = np.linspace(t[0], t[-1], n)
    xu = np.column_stack([np.interp(tu, t, x[:, k]) for k in range(x.shape[1])])
    W = max(5, int(round(0.225 / max(np.median(np.diff(tu)), 1e-6))) | 1)
    if W >= n:
        W = n - 1 if (n - 1) % 2 == 0 else n - 2
    if W >= 5:
        xu = sgf(xu, W, 2, axis=0)
    vu = np.gradient(xu, tu, axis=0)
    return np.column_stack([np.interp(t, tu, vu[:, k]) for k in range(x.shape[1])])


# Phase labels: 0=X, 1=Y, 2=Z, 3=yaw, -1=ambiguous/settle.
PHASE_LAB = ['X', 'Y', 'Z', 'yaw']
PHASE_WIN_S = float(os.environ.get("CAL_PHASE_WIN_S", "0.5"))
PHASE_DOM_MIN = float(os.environ.get("CAL_PHASE_DOM_MIN", "0.45"))
# 1 rad/s of yaw is treated as comparable "excitation effort" to this many m/s
# of translation, so the dominance comparison is not unit-biased toward yaw.
PHASE_YAW_SCALE = float(os.environ.get("CAL_PHASE_YAW_SCALE", "0.30"))


def phase_labels(g, win_s=PHASE_WIN_S, dom_min=PHASE_DOM_MIN):
    """Recover per-sample excitation-phase labels from mocap GT alone.

    ADDED 2026-07-27. These recordings ARE phased (operator sweeps one axis at
    a time -- verified from mocap: 0.21 axis-switches/window, 3-4s contiguous
    single-axis blocks, 0.73-0.80 dominance across the 2026-07-26 set), but
    output_calibration.py logs NO phase tag, unlike Gazebo's
    record_output_calibration.py which writes gt['Phase'] per sample and whose
    derive_board_cal.py then fits the centroid scale from the CLEAN
    single-axis segment (`std_ratio(..., phase=='x')`).

    Without tags derive_one() had to fit the centroid scale with one global
    polyfit over all phases mixed. Measured cost of that on the 7-run
    2026-07-26 set: 2/7 runs returned a PHYSICALLY IMPOSSIBLE negative scale
    (-0.0097, -0.0320 -- moving right cannot swing the GT bearing left), and
    the aggregate produced a spurious 3.1x sx/sy asymmetry (0.061 vs 0.189)
    on what is a near-symmetric camera. Phase-selecting removes every negative
    and brings sx/sy to within 7% (0.270 vs 0.288), halving sx inter-run CV
    (0.88 -> 0.42).

    Method: window the run, normalise each axis by its OWN rms (so dominance
    is relative to that axis's excitation scale, not absolute m/s), and label
    a window only when one axis carries > dom_min of the normalised energy.
    Windows below that threshold stay -1 (settle/transition) and are excluded,
    which is the tagged-'settle' equivalent of Gazebo's phase script.

    FRAME NOTE (checked 2026-07-27 - do NOT "fix" this by swapping axes).
    There IS a non-identity 90deg rotation between camera and body
    (img_geometry.R_CAM_TO_BODY = [[0,1,0],[-1,0,0],[0,0,1]]) and another
    between mocap and body (T_QTM_TO_FRD = diag(1,-1,-1), FLU->FRD). Neither
    needs applying here, because both are already consumed UPSTREAM of the
    arrays this function and derive_one() consume:
      - "Feature Params" is built from get_virtual_pts(), which applies
        R_CAM_TO_BODY and then the gravity-levelling R_inv (img_geometry.py
        ~line 122) -> it is already V-frame / body-aligned, not raw camera.
      - g["B_h_g"] comes from compute_gt_flow(), which applies T_QTM_TO_FRD.
    So the labels below (0=X,1=Y in body-FRD) pair DIRECTLY with
    Feature Params columns 0/1. Verified empirically on 4 runs: the body-Y
    phase drives centroid column 1 (std 0.101/0.152/0.133) far more than
    column 0 (0.026/0.045/0.029). Applying R_CAM_TO_BODY again here would
    introduce the very 90deg error it looks like it is preventing.

    Returns an int array on g['t_g'], same length as the GT grid.
    """
    t = np.asarray(g["t_g"], float)
    alt = np.asarray(g["alt"], float)
    # translational velocity (m/s): h = v/z  ->  v = h*z
    v = np.asarray(g["B_h_g"], float)[:, :3] * alt[:, None]
    yaw_rate = _robust_vel(np.unwrap(g["alpha"])[:, None], t)[:, 0]
    A = np.column_stack([v, yaw_rate * PHASE_YAW_SCALE])

    rms = np.sqrt(np.nanmean(A ** 2, 0)) + 1e-9
    An = np.abs(A) / rms

    lab = np.full(len(t), -1, dtype=int)
    if len(t) < 4 or not np.isfinite(t).all():
        return lab
    edges = np.arange(t[0], t[-1], win_s)
    for a, b in zip(edges[:-1], edges[1:]):
        w = (t >= a) & (t < b)
        if w.sum() < 3:
            continue
        e = np.nanmean(An[w] ** 2, 0)
        tot = np.nansum(e)
        if not np.isfinite(tot) or tot <= 0:
            continue
        f = e / tot
        k = int(np.nanargmax(f))
        if f[k] > dom_min:
            lab[w] = k
    return lab


def compute_gt_flow(run_dir):
    """Pi-adapted port of gt_optical_flow.compute_gt_flow. Returns a dict
    with GT reference signals on the GT's own time axis (0-based, seconds
    since Start Time), plus align(t_abs, y) to resample a measured signal."""
    gt = np.load(os.path.join(run_dir, "Ground_Truth.npy"), allow_pickle=True).item()
    St = float(gt["Start Time"])
    tg = np.asarray(gt["Time"], float)
    u_all, tp_all = gt["UAV Pose"], gt["Target Pose"]
    n = min(len(tg), len(u_all), len(tp_all))
    tg, u_all, tp_all = tg[:n], u_all[:n], tp_all[:n]

    # Drop samples with a missing (None) pose OR NaN position/orientation
    # (QTM tracking dropouts - see the yaw-span bug fixed 2026-07-10 for why
    # these MUST be filtered before any differencing/unwrap-style math).
    def _valid(p, t):
        if p is None or t is None:
            return False
        vals = [p.x, p.y, p.z, p.roll, p.pitch, p.yaw, t.x, t.y, t.z]
        return all(np.isfinite(v) for v in vals)
    keep = np.array([_valid(u_all[i], tp_all[i]) for i in range(n)])
    tg = tg[keep]; u = [u_all[i] for i in range(n) if keep[i]]
    tp = [tp_all[i] for i in range(n) if keep[i]]
    n = len(tg)
    if n < 20:
        raise ValueError(f"too few valid (non-NaN) samples after filtering: {n}")

    # de-dup non-increasing timestamps (jitter)
    order = np.hstack(([True], np.diff(tg) > 1e-6))
    tg = tg[order]; u = [u[i] for i in range(len(u)) if order[i]]
    tp = [tp[i] for i in range(len(tp)) if order[i]]
    n = len(tg)

    W_x_tu = np.zeros((n, 3)); Ru = np.zeros((n, 3, 3)); yaw = np.zeros(n)
    roll = np.zeros(n); pitch = np.zeros(n)
    for i in range(n):
        p, t = u[i], tp[i]
        up, Ru[i] = _pose_to_frd(p)
        tpp, Rt = _pose_to_frd(t)
        # ROLL/PITCH GT (2026-08-01, for Wx/Wy columns - previously always zero,
        # since only yaw/alpha had a GT reference). UAV's OWN absolute attitude
        # (not relative-to-target, unlike yaw/alpha - Wx/Wy represent body
        # angular velocity, the same quantity a gyro measures, not a marker's
        # apparent in-image rotation, so there is no "target roll/pitch"
        # analogue to subtract).
        #
        # SIGN FIXED same day: originally derived pitch_FRD=-pitch_QTM by pure
        # analogy to yaw's T=diag(1,-1,-1) conjugation (R_x(180) flips rotations
        # about Y/Z, leaves X unchanged) - mathematically valid IN GENERAL, but
        # WRONG here because it assumes mocaptools.Pose.pitch directly
        # parameterizes a pure-math R_y(theta), which its own sign convention
        # apparently does not. Caught empirically while chasing the sx mystery:
        # cross-checked mocap pitch against the FC's own independently-measured
        # pitch (MAVSDK Odometry quaternion, Telemetry_Data.npy, standard
        # aerospace arcsin(2(wy-zx)) extraction) on "Sat Aug 01 21-51-24 2026" -
        # UNFLIPPED mocap pitch matches the FC to corr=+0.99, mean=0.69deg,
        # std=0.57deg (same quality as roll, which was never flipped); the
        # flipped convention gave corr=-0.99 (near-perfect ANTI-correlation -
        # the tell that a sign was wrong, not a genuine multi-degree sensor
        # disagreement as first suspected before this check). Roll stays
        # unflipped (already agreed with the FC).
        roll[i] = np.deg2rad(p.roll)
        pitch[i] = np.deg2rad(p.pitch)
        # LEVER ARMS (2026-07-27, user-measured). The mocap rigid-body origins
        # are NOT the optical centre / marker centre, and the difference is a
        # real ~0.19-0.23 m systematic error in the GT bearing s (see
        # project_pi_gt_lever_arm_2026_07_27: GT-predicted marker pixel missed
        # the logged corners by 156px in x, 70px in y before this).
        # The camera arm is BODY-FIXED so it rotates with the UAV (Ru @ ...);
        # the marker arm is fixed in the TARGET body (Rt @ ...), which for this
        # static, ~0-yaw target is effectively world-fixed.
        # Flow is unaffected either way - a constant offset differentiates
        # away - so this corrects the bearing/centroid chain only.
        W_x_tu[i] = (tpp + Rt @ R_MARKER_FRD) - (up + Ru[i] @ R_CAM_FRD)
        # FIXED 2026-07-28: was -deg2rad(p.yaw) - the UAV's own ABSOLUTE yaw,
        # completely dropping the target's yaw (t.yaw, only used above for the
        # marker lever-arm rotation, never for orientation). That mirrors the
        # position bug this exact lever-arm fix already corrected: s/h are
        # built from a RELATIVE vector (target - UAV, both lever-arm-corrected)
        # while alpha was left absolute. It only looked fine because the
        # current rig's target is static ("effectively world-fixed", see
        # comment above) - a constant target yaw offset silently absorbs into
        # the fit rather than showing up as error. It breaks the moment the
        # target has ANY nonzero or time-varying yaw (the moving-target/rover
        # phase). Relative yaw = UAV yaw - target yaw (target rotating the
        # marker contributes DIRECTLY to how it looks rotated in-frame; UAV
        # rotating the camera contributes OPPOSITELY - hence the differing
        # sign before negation), then the same QTM->FRD sign flip as before.
        yaw[i] = -np.deg2rad(p.yaw - t.yaw)

    W_v_tu = _robust_vel(W_x_tu, tg)
    B_h_g = np.full((n, 3), np.nan); V_h_g = np.full((n, 3), np.nan)
    V_s_g = np.full((n, 2), np.nan)
    # DEPTH DENOMINATOR: pure 1/Vz (2026-07-28), matching Gazebo's
    # aggregate_calibration_phased.compute_gt_signals (Vx/Vz, Vy/Vz, no
    # epsilon). The prior "+0.2" epsilon was tested and did NOT explain the
    # GT-vs-image mismatch this whole thread chased (removing it made the
    # match worse under the flawed raw-pixel comparand, see
    # project_pi_gt_lever_arm_2026_07_27) - it was pure softening, not a
    # physically-motivated term, so there is no reason to keep diverging from
    # the validated Gazebo formula. V_x[2] and zB are numerically identical
    # (confirmed |zB - Vz| = 0.0000 even at 13.5 deg tilt, since _v_frame is
    # gravity-levelled) so one guard covers both s and h.
    #
    # A hard guard is now REQUIRED where the epsilon used to only soften: s
    # previously had NO guard at all (only the +0.2 kept it finite), which was
    # a latent bug independent of this change - a near-zero/negative Vz could
    # already blow up silently. abs(zB) >= 0.1 matches the guard h already
    # had; s now gets the same one instead of none.
    for i in range(n):
        zB = W_x_tu[i, 2]
        if abs(zB) < 0.1:
            continue
        B_x = Ru[i].T @ W_x_tu[i]
        V_x = _v_frame(Ru[i]) @ B_x
        V_s_g[i] = [V_x[0] / zB, V_x[1] / zB]
        B_v = Ru[i].T @ W_v_tu[i]
        V_v = _v_frame(Ru[i]) @ B_v
        B_h_g[i] = B_v / zB
        V_h_g[i] = V_v / zB

    # Wx/Wy GT rates (2026-08-01): same _robust_vel treatment as yaw_rate below
    # (unwrap first - raw differentiation of jittery mocap attitude amplifies
    # noise, per _robust_vel's own docstring warning, and roll/pitch are
    # arctan2-wrapped same as yaw).
    roll_rate = _robust_vel(np.unwrap(roll)[:, None], tg)[:, 0]
    pitch_rate = _robust_vel(np.unwrap(pitch)[:, None], tg)[:, 0]

    out = dict(t_g=tg, start_time=St, alt=W_x_tu[:, 2],
               B_h_g=B_h_g, V_h_g=V_h_g, loom=V_h_g[:, 2], alpha=yaw, V_s_g=V_s_g,
               roll_rate=roll_rate, pitch_rate=pitch_rate)

    def align(t_abs, y):
        ti = np.asarray(t_abs, float) - St
        y = np.asarray(y)
        if y.ndim == 1:
            return np.interp(tg, ti, y)
        return np.column_stack([np.interp(tg, ti, y[:, k]) for k in range(y.shape[1])])
    out["align"] = align

    def gap_mask(t_abs, gap_max):
        """True at each tg[j] that is safely bracketed by two real samples
        of t_abs no more than gap_max apart (and within t_abs's own span).
        align() LINEARLY INTERPOLATES across whatever gap exists between
        real samples - for a normal ~30Hz flow this is a harmless few-ms
        fill, but for a multi-second marker-loss gap (confirmed common on
        this dataset, see check_loop_staleness.py) it silently fabricates
        potentially hundreds of GT-grid points along a straight line that
        has no reason to track GT's actual (non-linear) motion during the
        gap - injecting mismatched, deceptively dense-looking noise into
        the fit rather than merely losing coverage. Use this to mask those
        fabricated points out wherever align() is used for fitting."""
        ti = np.asarray(t_abs, float) - St
        if len(ti) < 2:
            return np.zeros(len(tg), dtype=bool)
        idx = np.clip(np.searchsorted(ti, tg), 1, len(ti) - 1)
        bracket_w = ti[idx] - ti[idx - 1]
        inside = (tg >= ti[0]) & (tg <= ti[-1])
        return inside & (bracket_w < gap_max)
    out["gap_mask"] = gap_mask
    return out


def std_ratio(gt, raw, mask, k_mad_sample=3.0):
    """Signed scale factor with SAMPLE-LEVEL outlier rejection on matched pairs.
    Ported verbatim from PX4_Gazebo/tools/aggregate_calibration_phased.py (2026-08-02,
    while chasing the negative-sx mystery - see project_pi_output_recal_2026_08_01).

    Unlike a plain np.polyfit slope (this file's OLD _phase_slope method - a raw OLS
    fit with no defense against a handful of contaminated/transient samples flipping
    its sign), this:
      1. Derives SIGN from correlation and MAGNITUDE from the std ratio, not a raw
         least-squares slope.
      2. Does its own MAD-based outlier rejection on residuals before the final fit.
      3. Returns NaN cleanly (rather than a confident wrong number) if the signal is
         below the noise floor (|corr| < 0.05) either before or after cleaning.
    Gazebo's own tooling has NO "drop non-positive slope" safeguard anywhere - this
    method is very likely why: it's architecturally hardened against exactly the
    failure mode that safeguard exists to catch on the Pi side.

    Method: initial robust fit beta_init = sign(corr) * std(gt)/std(raw) on the
    masked, finite pairs; compute residuals of raw vs raw_hat=gt/beta_init; drop
    pairs with |resid - median(resid)| > k_mad_sample * MAD(resid); re-fit the same
    sign/std-ratio on the cleaned set. Returns raw ≈ GT/beta, i.e. GT = beta*raw -
    same convention this file's callers already use."""
    g = gt[mask]; r = raw[mask]
    finite = np.isfinite(g) & np.isfinite(r)
    if finite.sum() < 30:
        return float('nan')
    gf = g[finite]; rf = r[finite]

    sg0 = float(np.std(gf))
    sr0 = float(np.std(rf))
    if sr0 < 1e-9 or sg0 < 1e-9:
        return float('nan')

    co0 = float(np.corrcoef(gf, rf)[0, 1])
    if not np.isfinite(co0) or abs(co0) < 0.05:
        return float('nan')
    sign0 = +1.0 if co0 > 0 else -1.0
    beta_init = sign0 * sg0 / sr0

    raw_hat = gf / beta_init
    resid = rf - raw_hat
    med = float(np.median(resid))
    mad = float(np.median(np.abs(resid - med)))
    if mad < 1e-12:
        return beta_init

    keep = np.abs(resid - med) <= k_mad_sample * mad
    if keep.sum() < 30:
        return beta_init
    gc = gf[keep]; rc = rf[keep]

    sg = float(np.std(gc))
    sr = float(np.std(rc))
    if sr < 1e-9 or sg < 1e-9:
        return float('nan')
    co = float(np.corrcoef(gc, rc)[0, 1])
    if not np.isfinite(co) or abs(co) < 0.05:
        return float('nan')
    sign = +1.0 if co > 0 else -1.0
    return sign * sg / sr


def tls_fit(R, G):
    """Multivariate total-least-squares (orthogonal regression) fit of G = R @ Msol,
    for comparison against the OLS np.linalg.lstsq fit used elsewhere in this file.

    Standard Golub-Van Loan TLS: unlike OLS (which assumes only G is noisy and
    minimizes ||G - R@Msol||, biasing the fitted slope toward zero whenever R
    itself is noisy - exactly derive_one()'s raw_flow regressor, raw std ~1.4
    vs true signal ~0.05), TLS assumes noise on BOTH R and G and minimizes the
    orthogonal (perpendicular) distance to the fit instead of the vertical one.
    This is the textbook correction for errors-in-variables / regression-
    attenuation bias - see project_pi_izeta_kappa_ratchet_fix_2026_07_31.

    R: (n, m) regressor (raw flow). G: (n, k) target (GT). Returns Msol (m, k)
    such that G ~= R @ Msol, i.e. same convention/shape as np.linalg.lstsq(R, G).
    """
    m = R.shape[1]
    C = np.hstack([R, G])
    _, _, Vt = np.linalg.svd(C, full_matrices=True)
    V = Vt.T
    k = G.shape[1]
    Vxy = V[:, m:]          # (m+k, k) - columns for the k smallest singular values
    V12 = Vxy[:m, :]        # (m, k)
    V22 = Vxy[m:, :]        # (k, k)
    return -V12 @ np.linalg.inv(V22)


CORNER_FLOW_TAGS = frozenset({'lstsq', 'lstsq+klt'})
# FIXED 2026-08-02: was missing the '+override' variants. img_data.py's
# _s_estimator_tag is built as _decode_tag + ('+override' if _ov_fired else '')
# (_decode_tag itself always 'lstsq' or 'lstsq+klt') - so the full REAL
# (non-coast, non-rescue) tag space is these 4 values, not 2. Confirmed on
# "Sat Aug 01 22-35-29 2026": 494/812 samples (61%) were 'lstsq+override' -
# a real decode with a plausibility-check override applied, not coast or
# rescue - silently excluded from every centroid fit before this fix, which
# is why that run only ever saw 20 "real" S samples instead of hundreds.
CORNER_S_TAGS = frozenset({'lstsq', 'lstsq+klt', 'lstsq+override', 'lstsq+klt+override'})
MAP_FLOW_TAGS = frozenset({'map_flow'})
MAP_S_TAGS = frozenset({'planar_map_rescue'})


def derive_one(run_dir, fit_method="ols", flow_tags=CORNER_FLOW_TAGS, s_tags=CORNER_S_TAGS):
    # Prefer the mount-rotation-corrected recompute (recompute_raw_flow.py)
    # when present - it reflects img_data.py's CURRENT _getVirtualPts math
    # (R_CAM_TO_BODY fix, 2026-07-11), re-derived offline from this same
    # run's own logged pixel corners/quaternion/gyro. The original
    # Img_Data.npy's "Opt Flow Ang Vel" was computed by the OLD, unfixed
    # code at capture time and can't reflect a fix that changed the
    # ray-projection math itself.
    recomputed_path = os.path.join(run_dir, "Img_Data_Recomputed.npy")
    if os.path.exists(recomputed_path):
        img = np.load(recomputed_path, allow_pickle=True).item()
        print("  using Img_Data_Recomputed.npy (mount-rotation-corrected)")
    else:
        img = np.load(os.path.join(run_dir, "Img_Data.npy"), allow_pickle=True).item()
    g = compute_gt_flow(run_dir)

    t_img_abs = np.asarray(img["Time"], float)
    raw_flow = np.asarray(img["Opt Flow Ang Vel"], float)     # pre-cal, [h;w]
    raw_feat = np.asarray(img["Feature Params"], float)       # [xc, yc, 1, alpha]
    tag = np.asarray(img.get("Opt Flow Estimator Tag", [''] * len(raw_flow)))
    n = min(len(t_img_abs), len(raw_flow), len(raw_feat), len(tag))
    t_img_abs, raw_flow, raw_feat, tag = t_img_abs[:n], raw_flow[:n], raw_feat[:n], tag[:n]

    if np.any(tag != ''):
        _u, _c = np.unique(tag, return_counts=True)
        print("  h estimator coverage: " + ", ".join(f"{u}={c}" for u, c in zip(_u, _c)))

    # COAST-FRAME GUARD (2026-07-26, root-caused a session-long "why is R^2 always
    # weak" question): 'coast' means NO common marker decoded in both frames of the
    # pair this call AND no map rescue fired either - img_data.py APPENDS A LITERAL
    # np.zeros(6) for that sample (img_data.py:1520-1526), not a hold/extrapolation.
    # On real runs this is 60-90% of ALL samples (see the coverage print above).
    # g["align"]'s np.interp() uses EVERY row of t_img_abs/raw_flow as an
    # interpolation knot - a zero at a coast timestamp drags the interpolated
    # GT-grid curve toward 0 not just AT that instant but across its neighboring
    # gap too, contaminating nearby genuine samples' interpolated neighborhood as
    # well. This is a strictly worse corruption than the existing >=1s gap_mask
    # catches (that only guards SUSTAINED marker-loss; most coast runs here are
    # brief, frame-to-frame flicker). Fix: drop coast rows from the FLOW's own
    # (t, value) pair entirely before any interpolation/filtering, so align() only
    # ever bridges two REAL measurements across whatever true gap exists between
    # them. Feature Params is NOT corrupted the same way (rescued/held during
    # coast, never zeroed - see img_data.py:1499-1516), so raw_feat keeps its own
    # unfiltered t_img_abs.
    #
    # SEPARATE CALIBRATION PER ESTIMATION TECHNIQUE (2026-08-01): 'map_flow'
    # (PlanarFeatureMap rescue, img_data.py's _flowMap ~line 1901-1905) is a
    # genuine, plausibility-gated geometric estimate - NOT a literal zero like
    # true 'coast' - but it is a DIFFERENT estimation technique from raw
    # lstsq corner flow, with its own noise/lag/scale characteristics (its own
    # homography-tracked-point geometry, not direct corner-pixel differencing).
    # Briefly tried pooling it into ONE shared corner-cal fit (same day, see
    # git history around this comment): R^2 got WORSE on 2 of 4 axes and
    # produced a physically-suspicious |Wz gain|>1 with a flipped sign -
    # evidence the two techniques don't share one linear transfer function.
    # So each technique now gets its OWN calibration matrix instead (this
    # function is called once per technique from main(), with `flow_tags`/
    # `s_tags` selecting which estimator tags count as "real" input for THAT
    # fit) - same pattern the ring flow cal already used relative to corner.
    # Genuinely-zeroed 'coast' rows (no technique produced anything) are
    # dropped from every technique's fit, same as before.
    _flow_real = (np.isin(tag, list(flow_tags))
                  if np.any(tag != '') else np.ones(len(tag), dtype=bool))
    t_flow_abs = t_img_abs[_flow_real]
    raw_flow = raw_flow[_flow_real]
    if np.any(~_flow_real):
        print(f"  flow tag filter ({sorted(flow_tags)}): kept {int(_flow_real.sum())}/{len(_flow_real)} "
              f"sample(s) (rest dropped - coast or a different technique)")

    # Per-frame raw flow is noise-dominated (raw std ~1.4 vs true signal
    # ~0.05, per project convention) - MUST filter on its own native time
    # axis BEFORE resampling/fitting. Uses the SAME KF the runtime
    # getOptFlowAngVel() actually applies (IMG_FILTER='kf' default) - NOT
    # Savgol, which was the original (2026-07-10) approach and matched a
    # filter-mismatch bug PX4_Gazebo independently found and fixed
    # 2026-07-11 (feedback_kf_savgol_cal_mismatch): fitting against one
    # filter shape and applying at runtime through a differently-shaped
    # filter is not guaranteed valid, and moved every diagonal entry on
    # SITL's dataset (loom +127%, w_z +64%).
    if len(raw_flow) > 1:
        raw_flow = reject_outliers(raw_flow, label="flow")
        raw_flow = kf_filter_causal(raw_flow, t_flow_abs, FLOW_KF_Q, FLOW_KF_R)

    # FEAT NaN-GAP GUARD (2026-08-02): Img_Data_Recomputed.npy (recompute_raw_flow.py)
    # leaves NaN for rows it can't recompute (no predecessor quaternion, degenerate
    # corners) rather than dropping them - unlike raw_flow above, raw_feat had NO
    # pre-interpolation real-only filter (the coast-frame guard only ever applied to
    # flow), so align()'s linear interpolation touches those scattered NaNs directly.
    # A linear interpolation between ANY NaN-adjacent samples is itself NaN, so
    # scattered ~50%-NaN input poisons nearly the ENTIRE interpolated output - this
    # silently zeroed out sx/sy's sample count after the _rp_basis fix enabled using
    # Img_Data_Recomputed.npy (confirmed: "phase coverage" showed real per-phase
    # counts, but std_ratio saw 0 REAL samples - the interpolation NaN-poisoning
    # between accounting for it in phase_labels(, computed from GT alone) and using
    # it in _phase_slope (computed from raw_feat_g, NaN-poisoned) was the gap).
    # Same fix pattern as the flow coast-guard: drop non-finite rows BEFORE
    # outlier-reject/KF/interpolation, not after.
    feat_finite = np.all(np.isfinite(raw_feat), axis=1)
    t_feat_abs = t_img_abs[feat_finite]
    raw_feat = raw_feat[feat_finite]
    if np.any(~feat_finite):
        print(f"  feat NaN-gap guard: {int((~feat_finite).sum())}/{len(feat_finite)} "
              f"non-finite 'Feature Params' sample(s) dropped before KF/interp")
    if len(raw_feat) > 1:
        raw_feat = reject_outliers(raw_feat, label="feature")
        # ALPHA-WRAP KF FIX (2026-08-02, mirrors img_data.py::_kf_feat_update):
        # column 3 (alpha) is a WRAPPED ANGLE - passing it to kf_filter_causal
        # directly like an ordinary linear channel corrupts on any +-pi branch
        # crossing (see _kf_step's own docstring rule, and the Pi/Gazebo
        # comparison that found the Pi's runtime had the same bug). Filter
        # columns 0-2 (xc, yc, scale) normally; filter alpha via a separate
        # [sin(alpha), cos(alpha)] pair through the SAME kf_filter_causal, then
        # resync column 3 to atan2(sin, cos) - matching the runtime fix exactly
        # so this offline derivation reflects what the fixed runtime produces.
        raw_feat_pos = kf_filter_causal(raw_feat[:, :3], t_feat_abs, FEAT_KF_Q, FEAT_KF_R)
        raw_feat_sc = kf_filter_causal(
            np.column_stack([np.sin(raw_feat[:, 3]), np.cos(raw_feat[:, 3])]),
            t_feat_abs, FEAT_KF_Q, FEAT_KF_R)
        raw_feat = np.column_stack([raw_feat_pos, np.arctan2(raw_feat_sc[:, 0], raw_feat_sc[:, 1])])

    raw_flow_g = g["align"](t_flow_abs, raw_flow)     # raw flow resampled onto GT clock (coast-free)
    # WRAP-SAFE ALIGNMENT (2026-08-02): g["align"]'s plain np.interp is wrong for
    # column 3 (alpha, wrapped to (-pi,pi]) whenever a real +-pi crossing falls
    # inside an interpolation window - linearly interpolating e.g. +3.10 and
    # -3.10 gives ~0 (the wrong side of the circle) instead of the true ~+-pi
    # crossing point. Align columns 0-2 normally; align alpha via sin/cos.
    raw_feat_g = np.column_stack([
        g["align"](t_feat_abs, raw_feat[:, :3]),
        np.arctan2(g["align"](t_feat_abs, np.sin(raw_feat[:, 3])),
                   g["align"](t_feat_abs, np.cos(raw_feat[:, 3])))])

    # Marker-loss gap guard (see GAP_EXCLUDE_S / g["gap_mask"]): align()
    # linearly interpolates raw_flow/raw_feat across whatever real-sample
    # gaps exist in t_img_abs - for a multi-second marker-loss gap that
    # fabricates many mismatched GT-grid points (see gap_mask docstring).
    # Excluded here, on the ALIGNED (GT-grid) index space, not on
    # t_img_abs's own sparse index - a mask built on the flow's native
    # samples would not line up with raw_flow_g/raw_feat_g's indexing.
    # Uses t_flow_abs (post coast-guard) for the FLOW gap check, since that is
    # the array raw_flow_g was actually interpolated from.
    gok = g["gap_mask"](t_flow_abs, GAP_EXCLUDE_S)
    n_excluded = int((~gok).sum())
    if n_excluded:
        print(f"  gap guard: {n_excluded}/{len(gok)} GT-grid sample(s) excluded from fit "
              f"(interpolated across a >= {GAP_EXCLUDE_S:.1f}s marker-loss gap)")

    # GT rhs: manuscript w = -alpha_dot-derived yaw rate is folded into
    # V_h_g already having only translational h; the GT yaw-rate (w_z) is
    # d(alpha)/dt, matching img_data's B_v[5].
    #
    # FIXED 2026-07-11: this used to be a bare np.gradient(alpha, t_g) -
    # raw differentiation of jittery QTM yaw with NO smoothing, despite the
    # comment above claiming "the same _robust_vel style" (never actually
    # implemented - _robust_vel is used for V_h_g/W_v_tu above but not here).
    # Two compounding problems: (1) _robust_vel's own docstring warns against
    # differentiating raw jittery mocap data directly, exactly what this did;
    # (2) alpha is arctan2-wrapped to (-pi, pi] and was never unwrapped, so
    # ANY yaw crossing that boundary during the recording produced a huge
    # spurious np.gradient spike right at the wrap (same bug class already
    # found in the yaw-span calc, see the NaN-before-unwrap fix elsewhere).
    # Both would inject severe, GT-uncorrelated noise specifically into the
    # Wz fit target - consistent with Wz being the worst-behaved axis in
    # every derived calibration so far, independent of the other fixes.
    alpha_unwrapped = np.unwrap(g["alpha"])
    yaw_rate = _robust_vel(alpha_unwrapped[:, None], g["t_g"])[:, 0]
    # Wx/Wy ADDED 2026-08-01 (were np.zeros((n,2)) - Wx/Wy were never fitted at
    # all, only Wz had a GT reference). g["roll_rate"]/g["pitch_rate"] are the
    # UAV's own absolute body angular velocity (compute_gt_flow's roll/pitch
    # GT block) - real targets for the raw w0/w1 columns instead of a
    # structurally-zero column that made those rows unfittable by construction.
    G = np.column_stack([g["V_h_g"], g["roll_rate"], g["pitch_rate"], yaw_rate])  # [h(3); w(3)] = 6 cols

    m = np.all(np.isfinite(G), 1) & np.all(np.isfinite(raw_flow_g), 1) & gok
    Gm, Rm = G[m], raw_flow_g[m]
    if len(Rm) < 200:
        raise ValueError(f"too few aligned finite samples: {len(Rm)}")

    # Diagnostic: raw per-axis Pearson correlation BEFORE the full 6x6 fit -
    # a sanity check independent of fit quality. If diagonal-ish entries
    # (Hx-h0, Hy-h1, Hz-h2, Wz-w2) aren't the strongest in their row/col,
    # something upstream (frame transform, alignment, units) is likely wrong
    # rather than the fit itself being the problem.
    corr = np.corrcoef(np.hstack([Gm, Rm]).T)[:6, 6:]
    print("  raw corrcoef(GT, raw) [rows=GT Hx/Hy/Hz/Wx/Wy/Wz, cols=raw h0/h1/h2/w0/w1/w2]:")
    print(" ", np.array2string(corr, precision=3, suppress_small=True).replace("\n", "\n   "))

    if fit_method == "tls":
        # G's Wx/Wy columns (3,4) are STRUCTURALLY zero (np.zeros above, not just
        # noisy - Wx/Wy are never GT-estimated) - feeding a rank-deficient target
        # into the joint-SVD TLS solve makes V22 near-singular and its inverse
        # blow up (empirically confirmed: naive full-6-column TLS gave R^2 in the
        # -20 to -120 range and wildly unstable per-run diagonals, worse than OLS
        # in every respect). Fit TLS only over the 4 real columns and leave the
        # other two at zero, matching what OLS already effectively does for them.
        real_cols = [0, 1, 2, 5]
        Msol = np.zeros((Rm.shape[1], Gm.shape[1]))
        Msol[:, real_cols] = tls_fit(Rm, Gm[:, real_cols])
    else:
        Msol, _, _, _ = np.linalg.lstsq(Rm, Gm, rcond=None)   # G = R @ Msol
    cal = Msol.T                                          # GT = cal @ raw
    pred = Rm @ Msol
    r2 = 1 - np.sum((Gm - pred) ** 2, 0) / np.sum((Gm - Gm.mean(0)) ** 2, 0)

    # centroid scale: GT bearing vs raw feature xc/yc, per-axis regression
    # slope, fitted on that axis's OWN excitation phase.
    #
    # FIXED 2026-07-27 (see phase_labels docstring): this used to be one global
    # polyfit over ALL phases mixed, because output_calibration.py logs no
    # phase tag. Cross-axis motion during the x-phase then leaks into the sy
    # fit and vice versa, which on the 2026-07-26 set produced 2/7 physically
    # impossible NEGATIVE scales and a spurious 3.1x sx/sy asymmetry. Phases
    # are now recovered from mocap (no re-recording needed) and each axis is
    # fitted from its own clean segment, matching what Gazebo's
    # derive_board_cal.py does with its logged gt['Phase'] tags.
    # Held-during-coast centroids are excluded too. The 2026-07-26 coast fix
    # deliberately left "Feature Params" alone because it is rescued/held (not
    # zeroed) during coast -- but a HELD centroid is still a stale constant
    # sitting against a GT that keeps moving, which biases the slope just as
    # surely as a zero does. Measured on the 7-run set: excluding them takes
    # the sy inter-run CV from 0.74 to 0.10.
    gs = g["V_s_g"]
    s_tag = np.array([str(x) for x in img.get("S Estimator Tag", [])])
    if len(s_tag) == len(t_img_abs):
        s_is_real = np.isin(s_tag, list(s_tags))
        # REACQUISITION SETTLE GUARD (2026-08-01, root-caused a "why does sx come
        # out negative on some runs" investigation): a sample's OWN tag being
        # real (e.g. 'lstsq') doesn't mean it's a clean measurement - checked the
        # actual coast->lstsq transitions on a run that produced a strongly
        # wrong-signed sx and found the raw xc JUMPS by 0.15-0.29 at the very
        # first post-coast sample while GT (mocap) stays smooth/continuous
        # through the same instant - the first reacquisition sample(s) are a
        # transient (either genuinely catching up to real motion that happened
        # blind during the loss, or a reacquisition detection glitch), not a
        # clean instantaneous reading. Exclude the first SETTLE_N real-tagged
        # samples after any coast run, same spirit as the existing coast guard
        # but for the settle TRANSIENT rather than coast itself.
        _edge = np.diff(s_is_real.astype(int), prepend=0) == 1   # False->True transitions
        _settle = np.zeros(len(s_is_real), dtype=bool)
        for _i in np.where(_edge)[0]:
            _settle[_i:_i + SETTLE_N] = True
        _n_settled = int((_settle & s_is_real).sum())
        if _n_settled:
            print(f"  reacquisition settle guard (S): {_n_settled} post-coast "
                  f"sample(s) excluded (first {SETTLE_N} after each coast run)")
        s_is_real = s_is_real & ~_settle
        s_real_g = g["align"](t_img_abs, s_is_real.astype(float)[:, None])[:, 0] > 0.99
    else:                                          # older recording, no tag
        s_real_g = np.ones(len(gs), dtype=bool)
        print("  [warn] no usable 'S Estimator Tag' - centroid fit cannot "
              "exclude held-during-coast samples; treat cal_s as provisional")

    ms = (np.all(np.isfinite(gs), 1) & np.all(np.isfinite(raw_feat_g[:, :2]), 1)
          & gok & s_real_g)
    lab = phase_labels(g)
    if len(lab) != len(ms):                       # defensive: keep masks aligned
        lab = np.resize(lab, len(ms))

    cov = ", ".join(f"{PHASE_LAB[k]}={int(((lab == k) & s_real_g).sum())}"
                    for k in range(4))
    print(f"  phase coverage (REAL, non-coast GT-grid samples): {cov}, "
          f"settle/ambiguous={int((lab < 0).sum())}")

    def _phase_slope(col, axis_k, name):
        """Slope from axis_k's own phase ONLY, real samples. Cross-check /
        fallback, not the primary fit as of 2026-08-02 (see _mixed_motion_slope
        below for why) - still useful as a stricter sanity check when it does
        return a number, and as the fallback when mixed-motion itself is NaN.

        FIT METHOD SWAPPED 2026-08-02 (see std_ratio's own docstring): was a
        plain np.polyfit OLS slope, no defense against a handful of
        contaminated/transient samples (e.g. post-coast reacquisition jumps)
        flipping its sign - exactly the negative-sx failure mode this file's
        _pos_median aggregate guard exists to catch downstream. std_ratio
        (ported from Gazebo's aggregate_calibration_phased.py) derives sign
        from correlation + does its own outlier rejection instead.
        """
        sel = ms & (lab == axis_k)
        val = std_ratio(gs[:, col], raw_feat_g[:, col], sel)
        return val, int(sel.sum())

    def _mixed_motion_slope(col, name):
        """PRIMARY fit as of 2026-08-02: std_ratio over ALL real samples,
        regardless of which axis dominates that instant - no phase restriction.

        The phase-purity requirement (_phase_slope above) existed specifically
        to guard against tilt-dependent CROSS-AXIS contamination from the
        img_geometry.py::_rp_basis gravity-vector sign bug (see
        project_pi_output_recal_2026_08_01 - g was R@[0,0,1] instead of
        R.T@[0,0,1], amplifying rather than cancelling tilt-induced error
        during real excitation, exactly the kind of thing that leaks between
        axes during MIXED motion). With that bug fixed, there is no longer a
        structural reason mixed-motion samples should corrupt this fit -
        confirmed empirically (2026-08-02): pooling all-real-sample data
        recovered 5/6 runs with clean positive sx/sy (vs 2/6 phase-restricted),
        pooled corr 0.76/0.90, consistent with the phase-restricted result but
        from ~3x the sample count. std_ratio's own outlier rejection still
        provides defense against ordinary contamination (reacquisition
        transients not caught by the settle guard, detection glitches, etc.)
        - this isn't a return to the pre-2026-07-27 "one global polyfit over
        everything" approach that produced impossible negative scales; that
        one had neither std_ratio's sign-from-correlation robustness nor its
        outlier rejection.
        """
        val = std_ratio(gs[:, col], raw_feat_g[:, col], ms)
        return val, int(ms.sum())

    def _slope(col, axis_k, name):
        mixed, n_mixed = _mixed_motion_slope(col, name)
        phased, n_phased = _phase_slope(col, axis_k, name)
        mixed_str = f"{mixed:+.4f}" if np.isfinite(mixed) else "NaN"
        phased_str = f"{phased:+.4f}" if np.isfinite(phased) else "NaN"
        print(f"  {name}: mixed-motion={mixed_str} (n={n_mixed})  "
              f"phase-restricted={phased_str} (n={n_phased})")
        val, n_used = (mixed, n_mixed) if np.isfinite(mixed) else (phased, n_phased)
        if np.isfinite(val):
            return val, n_used
        print(f"  [warn] {name}: NaN from BOTH mixed-motion and phase-restricted fits "
              f"({n_mixed} total real sample(s), {n_phased} in the {PHASE_LAB[axis_k]} "
              f"phase) - too few real samples or signal below the noise floor; "
              f"re-record rather than trusting a substitute fit")
        return np.nan, n_used

    sx, nx = _slope(0, 0, "sx")
    sy, ny = _slope(1, 1, "sy")
    for nm, val in (("sx", sx), ("sy", sy)):
        if np.isfinite(val) and val <= 0:
            print(f"  [warn] {nm}={val:+.4f} is NON-POSITIVE -- physically "
                  f"impossible for a centroid scale; this run's centroid fit "
                  f"is contaminated, exclude it rather than averaging it in")

    # ALPHA CAL (2026-08-02): _sensor_cal_s[3] has always been hardcoded 1.0
    # (identity - never derived from data), on the assumption alpha needs no
    # correction (true on Gazebo, "cal_s[3]=1.0 is CORRECT, alpha tracks GT
    # r=1.00" per that project's own memory - but never independently checked
    # on the Pi). Checked directly: alpha_raw is NOT a 1:1 match to GT alpha -
    # testing both sign conventions with wrap-safe circular statistics, the
    # SIGN-FLIPPED hypothesis (alpha_raw = -1*alpha_GT + alpha_0) fits an
    # order of magnitude tighter than the identity one. alpha_0 is a genuine,
    # separate, still-open TODO already flagged in get_img_features's own
    # docstring ("alpha_0 = 0 here... recalibrate the equilibrium offset once
    # good mocap data exists") - never actually derived until now. Two
    # DIFFERENT knobs, NOT interchangeable: alpha_0 is SUBTRACTED inside
    # marker_principal_angle (img_geometry.py) BEFORE the 2pi-disambiguation,
    # while cal_s[3] is a MULTIPLICATIVE sign applied AFTER, downstream in
    # img_data.py - a diagonal cal_s cannot represent an additive offset on a
    # wrapped angle by itself, so alpha_0 must be applied separately, not
    # folded into cal_s.
    def _alpha_cal():
        sel = ms & np.isfinite(raw_feat_g[:, 3]) & np.isfinite(gs_alpha)
        if sel.sum() < 30:
            return np.nan, np.nan, np.nan, int(sel.sum())
        ar = raw_feat_g[sel, 3]; ag = gs_alpha[sel]
        best = None
        for sign in (+1.0, -1.0):
            resid = np.arctan2(np.sin(ar - sign * ag), np.cos(ar - sign * ag))
            c = np.arctan2(np.median(np.sin(resid)), np.median(np.cos(resid)))
            spread = float(np.median(np.abs(np.arctan2(np.sin(resid - c), np.cos(resid - c)))))
            if best is None or spread < best[2]:
                best = (sign, float(c), spread)
        sign, offset, spread = best
        return sign, offset, spread, int(sel.sum())

    gs_alpha = g["alpha"]
    a_sign, a_offset, a_spread, a_n = _alpha_cal()
    if np.isfinite(a_sign):
        print(f"  alpha cal: sign={a_sign:+.0f}  alpha_0={np.degrees(a_offset):+.1f}deg  "
              f"residual spread={np.degrees(a_spread):.1f}deg  (n={a_n})")
    else:
        print(f"  alpha cal: too few real samples ({a_n}) for a fit")

    return cal, r2, (sx, sy), len(Rm), (a_sign, a_offset, a_spread, a_n)


def derive_ring_one(run_dir):
    """Ring-flow analogue of derive_one() - fits _sensor_cal_ring against the
    SAME GT this run's corner cal uses, but on the ring's OWN per-frame log
    ("Ring Opt Flow Ang Vel" + "Ring Time"), not "Time"/"Opt Flow Ang Vel".
    Ported alongside the img_data.py change (2026-07-22) that made ring
    logging unconditional (independent of corner-marker detection success,
    mirroring PX4_Gazebo) - the ring array is no longer index-aligned with
    the corner flow/feature arrays, so it needs its own alignment pass here
    rather than reusing derive_one()'s t_img_abs. Requires a recording made
    AFTER that img_data.py change (the 7 runs recorded 2026-07-10/11 predate
    it and have no "Ring Time" key - raises ValueError, same as any other
    run this tool considers unusable)."""
    img = np.load(os.path.join(run_dir, "Img_Data.npy"), allow_pickle=True).item()
    if "Ring Time" not in img:
        raise ValueError("no 'Ring Time' - this run predates unconditional ring logging")
    g = compute_gt_flow(run_dir)

    t_ring_abs = np.asarray(img["Ring Time"], float)
    raw_ring = np.asarray(img["Ring Opt Flow Ang Vel"], float)   # pre-cal, [h;w]
    n = min(len(t_ring_abs), len(raw_ring))
    t_ring_abs, raw_ring = t_ring_abs[:n], raw_ring[:n]

    raw_ring = reject_outliers(raw_ring, label="ring flow")
    raw_ring = kf_filter_causal(raw_ring, t_ring_abs, FLOW_KF_Q, FLOW_KF_R)
    raw_ring_g = g["align"](t_ring_abs, raw_ring)

    # Ring's own gap guard: the ring is designed to survive a CORNER dropout,
    # but it has its own (much rarer) failure mode - texture too sparse / all
    # stations off-frame - which also shows up as a real time gap in its
    # native "Ring Time" log. Same GAP_EXCLUDE_S convention as the corner fit.
    gok = g["gap_mask"](t_ring_abs, GAP_EXCLUDE_S)
    n_excluded = int((~gok).sum())
    if n_excluded:
        print(f"  ring gap guard: {n_excluded}/{len(gok)} GT-grid sample(s) excluded from fit")

    alpha_unwrapped = np.unwrap(g["alpha"])
    yaw_rate = _robust_vel(alpha_unwrapped[:, None], g["t_g"])[:, 0]
    G = np.column_stack([g["V_h_g"], g["roll_rate"], g["pitch_rate"], yaw_rate])

    m = np.all(np.isfinite(G), 1) & np.all(np.isfinite(raw_ring_g), 1) & gok
    Gm, Rm = G[m], raw_ring_g[m]
    if len(Rm) < 200:
        raise ValueError(f"too few aligned finite ring samples: {len(Rm)}")

    corr = np.corrcoef(np.hstack([Gm, Rm]).T)[:6, 6:]
    print("  ring raw corrcoef(GT, raw) [rows=GT Hx/Hy/Hz/Wx/Wy/Wz, cols=raw h0/h1/h2/w0/w1/w2]:")
    print(" ", np.array2string(corr, precision=3, suppress_small=True).replace("\n", "\n   "))

    Msol, _, _, _ = np.linalg.lstsq(Rm, Gm, rcond=None)
    cal = Msol.T
    pred = Rm @ Msol
    r2 = 1 - np.sum((Gm - pred) ** 2, 0) / np.sum((Gm - Gm.mean(0)) ** 2, 0)
    return cal, r2, len(Rm)


def check_mount_rotation(run_dir):
    """FC<->camera mount-rotation diagnostic (2026-07-23) - identifies/validates
    img_geometry.R_CAM_TO_BODY from EGO-MOTION, NOT mocap and NOT FC yaw/heading.
    FC yaw is magnetometer-derived and confirmed unreliable indoors this session
    (mag_test_ratio/heading-innovation crossed PX4's fail threshold repeatedly in
    real flight logs on this rig - see project memory). Gyro RATES and body
    VELOCITY are pure IMU/INS, magnetometer-independent, so this compares:

    ROTATION axis: d(alpha)/dt (marker's own image-plane angular rate, straight
    from raw pixel corners via Feature Params - no assumed mount rotation
    applied) vs the FC's OWN gyro yaw rate (Telemetry_Data's "Angular Velocity
    FRD" down_rad_s) - the same physical quantity two different ways. A static
    marker viewed by a rotating camera must show |slope|~1 REGARDLESS of the
    mount's fixed offset angle (a constant relative rotation between two rigid
    bodies never changes their shared angular RATE - only a genuine axis-flip
    in the mount would break the +-1 relationship) - so this is a structural
    check (pure-yaw mount? axis flipped?), not a numeric calibration.

    TRANSLATION axes: d(centroid)/dt (image xc,yc velocity) vs FC body velocity
    (Telemetry_Data's "Velocity Body" x_m_s/y_m_s) - full 2x2 cross-correlation
    to identify which image axis maps to which body axis and with what sign.
    NOTE (found 2026-07-23 on all 7 existing runs): this part needs a sweep
    with an ISOLATED translation phase to get a clean read - these runs are
    yaw-dominated (130-300 deg/s yaw-rate spans) and/or FC's GPS-denied INS
    velocity may be too drifty; expect weak/inconsistent correlations until a
    better-excited recording exists. Report it anyway (harmless when weak) so
    a future well-excited run's improvement is visible without code changes.

    AXIS MAPPING IS NOW VALIDATED (2026-07-27) - but by a DIFFERENT route than
    this function's translation block, which stays weak for the reason above.
    Rather than FC INS velocity (drifty, GPS-denied), the check used mocap:
    phase_labels() recovers the excitation phase, then the V-frame centroid
    response is measured per phase. body-Y excitation drives centroid column 1
    by 3-5x over column 0 (std 0.101/0.152/0.133 vs 0.026/0.045/0.029) on the
    2026-07-26 runs; a wrong or missing 90 deg in R_CAM_TO_BODY would have
    shown the opposite. See the AXIS MAPPING VALIDATED block in img_geometry.py.
    So do NOT read this function's weak translation correlations as evidence
    against the mount - they reflect the INS velocity reference, not the mount.
    What remains genuinely unvalidated is the mount TRANSLATION (lever arm),
    which is a different quantity - see project_pi_gt_lever_arm_2026_07_27.

    Runs entirely on already-recorded Img_Data.npy + Telemetry_Data.npy - no
    mocap Ground_Truth.npy needed, so this works even on non-mocap hardware
    recordings (e.g. any hardware_landing.py flight log pair).

    FIXED 2026-07-25: dalpha/dt is computed from get_img_features() output,
    which runs on V_nP_norm - ALREADY roll/pitch-leveled (gravity-leveled)
    virtual-frame points (see img_data.py _optFlowAngVel: `self._getImgFeatures
    (V_nP_norm[1])`). Comparing that directly against the RAW BODY-FRD gyro
    z-component (Angular Velocity FRD's down_rad_s) is a frame mismatch - body
    w_z and V-frame w_z only coincide when roll/pitch are exactly zero
    (standard Euler-rate cross-coupling); during real hand-held or flight
    motion they diverge. img_geometry.vframe_w() exists for exactly this -
    it's what the RUNTIME gyro-compensation path already uses to convert body
    gyro into the V-frame before comparing against anything V-framed. This
    fix rotates the full 3-axis body gyro through vframe_w() (using the
    nearest-in-time attitude quaternion, from Odometry Timestamp/Quaternion -
    NOT IMU Timestamp, which has no attitude of its own) before taking its
    z-component, so the comparison is now V-frame-to-V-frame throughout.
    Empirically (2026-07-25, 6-run GOOD dataset) this correction changed
    correlations by <0.02 in every run - roll/pitch tilt during these
    recordings was small enough that the frame mismatch wasn't the dominant
    noise source (see [[project_pi_output_cal_2026_07_25_session]] - the
    actual dominant cause was traced to marker_principal_angle's 2pi-
    disambiguation flickering on corner-detection jitter) - but the fix is
    still correct in principle and matters more on a more aggressively-
    tilted recording.

    FIXED 2026-07-26: this used to differentiate the RAW per-sample
    Feature Params directly (np.gradient on unfiltered alpha/xc/yc) - a
    self-inflicted unfair test, not a real geometry check. Raw image
    signals are inherently noisy (that's WHY the runtime pipeline runs them
    through reject_outliers + a causal KF before ANY consumer - including
    derive_one()'s own flow/centroid fit above - ever differentiates or
    fits them); comparing an unsmoothed raw derivative against gyro will
    look uncorrelated even when the underlying geometry is exactly right,
    for the same reason two people timing the same stopwatch with shaky
    hands will disagree - the disagreement is about hand-shake, not the
    stopwatch. "Raw, no assumed mount rotation applied" in this function's
    original intent meant not baking in the mount-offset GEOMETRY being
    tested - it was never meant to also skip ordinary noise filtering,
    those are orthogonal. Now runs alpha/xc/yc through the SAME
    reject_outliers + kf_filter_causal treatment (same FEAT_KF_Q/R
    constants) derive_one() already applies to the centroid/flow fit,
    before differentiating - alpha is unwrapped first so the periodic
    wrap doesn't look like a spurious Hampel/KF outlier."""
    img = np.load(os.path.join(run_dir, "Img_Data.npy"), allow_pickle=True).item()
    tel_path = os.path.join(run_dir, "Telemetry_Data.npy")
    if not os.path.exists(tel_path):
        raise ValueError("no Telemetry_Data.npy in this run")
    tel = np.load(tel_path, allow_pickle=True).item()

    t_img = np.asarray(img["Time"], float)
    feat = np.asarray(img["Feature Params"], float)   # [xc, yc, 1, alpha]
    n = min(len(t_img), len(feat))
    t_img, feat = t_img[:n], feat[:n]

    t_imu = np.asarray(tel["IMU Timestamp"], float)
    av = tel["Angular Velocity FRD"]
    w_body_full = np.array([[a.forward_rad_s, a.right_rad_s, a.down_rad_s] if a is not None
                             else [np.nan, np.nan, np.nan] for a in av])
    t_odo = np.asarray(tel["Odometry Timestamp"], float)
    quats = tel["Quaternion"]
    vb = tel["Velocity Body"]
    vx_full = np.array([v.x_m_s if v is not None else np.nan for v in vb])
    vy_full = np.array([v.y_m_s if v is not None else np.nan for v in vb])
    if len(t_imu) < 2 or len(t_odo) < 2:
        raise ValueError("insufficient telemetry (IMU/odometry) samples")

    w_body = np.column_stack([np.interp(t_img, t_imu, w_body_full[:, k]) for k in range(3)])
    # Nearest-in-time attitude for each image sample - IMU Timestamp has no
    # attitude of its own; Odometry Timestamp is the quaternion's own clock.
    qi = np.clip(np.searchsorted(t_odo, t_img), 0, len(quats) - 1)
    gyro_yaw = np.array([vframe_w(w_body[i], quats[qi[i]])[2] for i in range(len(t_img))])
    vx = np.interp(t_img, t_odo, vx_full)
    vy = np.interp(t_img, t_odo, vy_full)

    # Filter BEFORE differentiating - same treatment derive_one() already
    # gives the centroid/flow signal, applied here to the [xc, yc, alpha]
    # this function itself differentiates. Unwrap alpha first so the
    # periodic wrap isn't mistaken for an outlier by the Hampel/KF pass.
    feat_for_filt = np.column_stack([feat[:, 0], feat[:, 1], np.unwrap(feat[:, 3])])
    feat_for_filt = reject_outliers(feat_for_filt, label="feature (mount-rotation check)")
    feat_filt = kf_filter_causal(feat_for_filt, t_img, FEAT_KF_Q, FEAT_KF_R)
    alpha = feat_filt[:, 2]
    dt = np.gradient(t_img)
    dalpha_dt = np.gradient(alpha, t_img)
    dcx_dt = np.gradient(feat_filt[:, 0], t_img)
    dcy_dt = np.gradient(feat_filt[:, 1], t_img)

    m = (np.isfinite(gyro_yaw) & np.isfinite(vx) & np.isfinite(vy)
         & np.isfinite(dalpha_dt) & (dt > 1e-4) & (dt < 0.5))
    if m.sum() < 15:
        raise ValueError(f"too few aligned samples: {m.sum()}")

    A = np.column_stack([gyro_yaw[m], np.ones(m.sum())])
    (slope, intercept), *_ = np.linalg.lstsq(A, dalpha_dt[m], rcond=None)
    rot_corr = (float(np.corrcoef(dalpha_dt[m], gyro_yaw[m])[0, 1])
                if np.std(gyro_yaw[m]) > 1e-9 else np.nan)
    yaw_rate_span = float(np.ptp(gyro_yaw[m]))

    Gt = np.column_stack([dcx_dt[m], dcy_dt[m]])
    Gb = np.column_stack([vx[m], vy[m]])
    if all(np.std(x) > 1e-9 for x in (Gt[:, 0], Gt[:, 1], Gb[:, 0], Gb[:, 1])):
        trans_corr = np.corrcoef(np.hstack([Gt, Gb]).T)[:2, 2:]
    else:
        trans_corr = np.full((2, 2), np.nan)

    return dict(n=int(m.sum()), yaw_rate_span_deg=float(np.degrees(yaw_rate_span)),
                rot_slope=float(slope), rot_intercept=float(intercept), rot_corr=rot_corr,
                trans_corr=trans_corr)


def main():
    if len(sys.argv) > 1:
        runs = sys.argv[1:]
    else:
        runs = sorted(d for d in glob.glob(os.path.join(CAL_DIR, "*")) if os.path.isdir(d))

    np.set_printoptions(precision=4, suppress=True, linewidth=130)
    cals, r2s, calS, alphaS = [], [], [], []
    for run_dir in runs:
        name = os.path.basename(run_dir)
        try:
            cal, r2, (sx, sy), nfit, (a_sign, a_offset, a_spread, a_n) = derive_one(run_dir)
        except Exception as e:
            print(f"  skip {name}: {e}")
            continue
        print(f"\n=== {name} (n={nfit} fit samples) ===")
        print("per-axis R^2: " + "  ".join(f"{LAB[k]}={r2[k]:.2f}" for k in range(6)))
        print(cal)
        print(f"centroid slope: sx={sx:.4f}  sy={sy:.4f}")
        cals.append(cal); r2s.append(r2); calS.append([sx, sy])
        if np.isfinite(a_sign):
            alphaS.append([a_sign, a_offset, a_spread])

    if not cals:
        print("\nNo usable runs.")
        return

    cals = np.array(cals); r2s = np.array(r2s); calS = np.array(calS)
    M = cals.mean(0); Mstd = cals.std(0) if len(cals) > 1 else np.zeros_like(M)

    # Centroid scale must be POSITIVE (a bigger raw pixel offset means a bigger
    # GT bearing, never a smaller one). A non-positive per-run slope is a
    # contaminated fit, not a datum -- median-ing it in would drag the applied
    # cal toward a value no camera can have. Dropped explicitly here rather
    # than trusted to the median's outlier tolerance (2026-07-27).
    def _pos_median(col, name):
        v = calS[:, col]
        good = v[np.isfinite(v) & (v > 0)]
        n_bad = int((np.isfinite(v) & (v <= 0)).sum())
        if n_bad:
            print(f"  [warn] {name}: dropped {n_bad} non-positive per-run "
                  f"slope(s) from the aggregate (physically impossible)")
        return float(np.median(good)) if len(good) else np.nan

    sx = _pos_median(0, "sx"); sy = _pos_median(1, "sy")

    # ALPHA CAL AGGREGATE (2026-08-02): sign must be UNANIMOUS across runs (a
    # geometric convention, not a noisy magnitude - if runs disagree on sign
    # something is wrong, don't average +1 and -1 into a meaningless ~0).
    # alpha_0 is circularly averaged (via sin/cos) across runs that agree on
    # sign. cal_s[3] gets the derived sign; alpha_0 is reported separately -
    # it belongs in img_geometry.py::marker_principal_angle's alpha_0
    # constant (or an env-configurable equivalent), not in cal_s (a diagonal
    # matrix can't represent an additive offset on a wrapped angle).
    a_sign_cal = 1.0
    alpha_0_deg = None
    if alphaS:
        alphaS = np.array(alphaS)
        signs = alphaS[:, 0]
        if len(set(signs.tolist())) > 1:
            print(f"  [warn] alpha cal: sign DISAGREES across runs ({signs.tolist()}) - "
                  f"not applying an alpha cal, investigate before trusting either sign")
        else:
            a_sign_cal = float(signs[0])
            sin_m = np.median(np.sin(alphaS[:, 1])); cos_m = np.median(np.cos(alphaS[:, 1]))
            alpha_0 = float(np.arctan2(sin_m, cos_m))
            alpha_0_deg = np.degrees(alpha_0)
            print(f"\nalpha cal: sign={a_sign_cal:+.0f} (unanimous across {len(alphaS)} run(s))  "
                  f"alpha_0={alpha_0_deg:+.1f}deg  (per-run residual spread "
                  f"{np.degrees(alphaS[:,2]).mean():.1f}deg avg)")

    print(f"\n\n=== AGGREGATE across {len(cals)} run(s) — PROVISIONAL, review before use ===")
    print("per-axis R^2 (mean):  " + "  ".join(f"{LAB[k]}={r2s[:,k].mean():.2f}" for k in range(6)))
    if len(cals) > 1:
        print("\ninter-run STD of M (small = robust):")
        print("       " + "  ".join(f"{r:>6}" for r in RL))
        for i in range(6):
            print(f"  {LAB[i]:>2} " + "  ".join(f"{Mstd[i,j]:6.3f}" for j in range(6)))
    print(f"\ncentroid cal_s:  sx={sx:.4f}  sy={sy:.4f}")

    print("\n--- candidate paste for img_data.py (DO NOT apply without reviewing R^2 above) ---")
    rows = ",\n            ".join(
        "[" + ", ".join(f"{M[i,j]:+.4f}" for j in range(6)) + "]" for i in range(6))
    print(f"        self._sensor_cal_hw = np.array([\n            {rows}])")
    print(f"        self._sensor_cal_s  = np.diag([{sx:.4f}, {sy:.4f}, 1.0, {a_sign_cal:+.1f}])")
    if alpha_0_deg is not None:
        print(f"        # ALSO apply alpha_0={alpha_0_deg:+.1f}deg ({np.radians(alpha_0_deg):+.4f} rad) "
              f"in img_geometry.py::marker_principal_angle's alpha_0 constant (NOT in cal_s - a "
              f"diagonal matrix can't represent this additive offset)")

    # MAP cal (2026-08-01): PlanarFeatureMap rescue is its own estimation
    # technique (img_data.py's _flowMap for flow, 'planar_map_rescue' for the
    # centroid/S path) with its own noise/lag/scale characteristics - fit
    # SEPARATELY from the corner cal above rather than pooled into it (pooling
    # was tried and made 2/4 flow axes worse with a physically-suspicious
    # |Wz gain|>1 - see derive_one's flow-tag-filter comment). Most existing
    # runs will have zero 'map_flow'/'planar_map_rescue' samples (map was
    # off/rare during calibration recording until 2026-08-01) and skip here -
    # that's expected, not a bug.
    print("\n\n=== MAP CAL (map_flow/planar_map_rescue technique, independent of corner fit) ===")
    map_cals, map_r2s, map_calS = [], [], []
    for run_dir in runs:
        name = os.path.basename(run_dir)
        try:
            cal, r2, (sx, sy), nfit, _alpha_unused = derive_one(run_dir, flow_tags=MAP_FLOW_TAGS, s_tags=MAP_S_TAGS)
        except Exception as e:
            print(f"  skip {name}: {e}")
            continue
        print(f"\n=== {name} (n={nfit} fit samples) ===")
        print("per-axis R^2: " + "  ".join(f"{LAB[k]}={r2[k]:.2f}" for k in range(6)))
        print(cal)
        print(f"centroid slope: sx={sx:.4f}  sy={sy:.4f}")
        map_cals.append(cal); map_r2s.append(r2); map_calS.append([sx, sy])

    if not map_cals:
        print("\nNo usable map-technique runs.")
    else:
        map_cals = np.array(map_cals); map_r2s = np.array(map_r2s); map_calS = np.array(map_calS)
        Mm = map_cals.mean(0); Mmstd = map_cals.std(0) if len(map_cals) > 1 else np.zeros_like(Mm)

        def _pos_median_map(col, name):
            v = map_calS[:, col]
            good = v[np.isfinite(v) & (v > 0)]
            n_bad = int((np.isfinite(v) & (v <= 0)).sum())
            if n_bad:
                print(f"  [warn] {name}: dropped {n_bad} non-positive per-run "
                      f"slope(s) from the aggregate (physically impossible)")
            return float(np.median(good)) if len(good) else np.nan

        sx_m = _pos_median_map(0, "sx"); sy_m = _pos_median_map(1, "sy")
        print(f"\n\n=== MAP AGGREGATE across {len(map_cals)} run(s) — PROVISIONAL, review before use ===")
        print("per-axis R^2 (mean):  " + "  ".join(f"{LAB[k]}={map_r2s[:,k].mean():.2f}" for k in range(6)))
        if len(map_cals) > 1:
            print("\ninter-run STD of M (small = robust):")
            print("       " + "  ".join(f"{r:>6}" for r in RL))
            for i in range(6):
                print(f"  {LAB[i]:>2} " + "  ".join(f"{Mmstd[i,j]:6.3f}" for j in range(6)))
        print(f"\ncentroid cal_s:  sx={sx_m:.4f}  sy={sy_m:.4f}")
        print("\n--- candidate paste for img_data.py (DO NOT apply without reviewing R^2 above, "
              "AND without wiring a map-technique consumer to actually use it) ---")
        rows_m = ",\n            ".join(
            "[" + ", ".join(f"{Mm[i,j]:+.4f}" for j in range(6)) + "]" for i in range(6))
        print(f"        self._sensor_cal_map = np.array([\n            {rows_m}])")
        print(f"        self._sensor_cal_s_map  = np.diag([{sx_m:.4f}, {sy_m:.4f}, 1.0, 1.0])")

    # Ring cal, from the SAME runs' "Ring Opt Flow Ang Vel"/"Ring Time" logs
    # (see derive_ring_one docstring - requires a recording made after the
    # 2026-07-22 img_data.py change; the 7 runs recorded 2026-07-10/11 will
    # all skip here with "no 'Ring Time'").
    print("\n\n=== RING CAL (independent of the corner fit above) ===")
    ring_cals, ring_r2s = [], []
    for run_dir in runs:
        name = os.path.basename(run_dir)
        try:
            cal, r2, nfit = derive_ring_one(run_dir)
        except Exception as e:
            print(f"  skip {name}: {e}")
            continue
        print(f"\n=== {name} (n={nfit} ring fit samples) ===")
        print("per-axis R^2: " + "  ".join(f"{LAB[k]}={r2[k]:.2f}" for k in range(6)))
        print(cal)
        ring_cals.append(cal); ring_r2s.append(r2)

    # Mount-rotation diagnostic (see check_mount_rotation docstring) - runs off
    # Img_Data.npy + Telemetry_Data.npy only, no mocap Ground_Truth.npy needed,
    # so it's independent of whether the corner/ring fits above found usable
    # data this run - placed BEFORE ring's own early-return below so it still
    # runs even when every run lacks "Ring Time" (all 7 pre-2026-07-22 runs do).
    print("\n\n=== MOUNT-ROTATION CHECK (ego-motion based, magnetometer-free) ===")
    rot_slopes, rot_corrs = [], []
    for run_dir in runs:
        name = os.path.basename(run_dir)
        try:
            r = check_mount_rotation(run_dir)
        except Exception as e:
            print(f"  skip {name}: {e}")
            continue
        tc = r["trans_corr"]
        print(f"  {name}: n={r['n']}  yaw_rate_span={r['yaw_rate_span_deg']:.1f}deg/s")
        print(f"    ROTATION: dalpha/dt = {r['rot_slope']:+.3f}*gyro_yaw {r['rot_intercept']:+.3f}   "
              f"corr={r['rot_corr']:.3f}")
        print(f"    TRANSLATION corr [rows=dcx,dcy cols=vx,vy]: "
              f"[[{tc[0,0]:+.3f} {tc[0,1]:+.3f}] [{tc[1,0]:+.3f} {tc[1,1]:+.3f}]]")
        if r["yaw_rate_span_deg"] > 10 and np.isfinite(r["rot_corr"]):
            rot_slopes.append(r["rot_slope"]); rot_corrs.append(r["rot_corr"])
    if rot_slopes:
        best = int(np.argmax(np.abs(rot_corrs)))
        best_slope = rot_slopes[best]
        dev = abs(best_slope - EGO_MOTION_ROT_SLOPE)
        verdict = "PASS" if dev < 0.15 else ("MARGINAL" if dev < 0.35 else "FAIL")
        print(f"\n  best-excited run: slope={best_slope:+.3f} corr={rot_corrs[best]:.3f} "
              f"(median slope across {len(rot_slopes)} run(s): {np.median(rot_slopes):+.3f})")
        print(f"  vs EGO_MOTION_ROT_SLOPE={EGO_MOTION_ROT_SLOPE:+.1f} (img_geometry.py, validated "
              f"2026-07-23): deviation={dev:.3f}  [{verdict}]")
        print("  a mismatch here would mean the mount's pure-yaw/non-mirrored assumption "
              "(R_CAM_TO_BODY's structure) needs revisiting, independent of its fixed offset angle.")
    else:
        print("  no run had enough yaw-rate excitation for a reliable rotation-axis read.")

    if not ring_cals:
        print("\nNo usable ring runs.")
        return

    ring_cals = np.array(ring_cals); ring_r2s = np.array(ring_r2s)
    Mr = ring_cals.mean(0)
    Mrstd = ring_cals.std(0) if len(ring_cals) > 1 else np.zeros_like(Mr)
    print(f"\n\n=== RING AGGREGATE across {len(ring_cals)} run(s) — PROVISIONAL, review before use ===")
    print("per-axis R^2 (mean):  " + "  ".join(f"{LAB[k]}={ring_r2s[:,k].mean():.2f}" for k in range(6)))
    if len(ring_cals) > 1:
        print("\ninter-run STD of ring M (small = robust):")
        print("       " + "  ".join(f"{r:>6}" for r in RL))
        for i in range(6):
            print(f"  {LAB[i]:>2} " + "  ".join(f"{Mrstd[i,j]:6.3f}" for j in range(6)))
    rows_r = ",\n            ".join(
        "[" + ", ".join(f"{Mr[i,j]:+.4f}" for j in range(6)) + "]" for i in range(6))
    print("\n--- candidate paste for img_data.py (DO NOT apply without reviewing R^2 above) ---")
    print(f"        self._sensor_cal_ring = np.array([\n            {rows_r}])")


if __name__ == "__main__":
    main()
