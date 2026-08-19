"""Pure geometry/math for the Pi vision pipeline - camera intrinsics, the
camera-to-FC mount rotation, and the virtual (gravity-leveled) frame math
(_getVirtualPts/_getRealPtsFromV/_vframe_w/_fill_A/_scaled_quad_points/
_getImgFeatures/_marker_principal_angle in img_data.py).

NO hardware dependencies (no picamera2, no qtm, no cv2) - only numpy + ahrs
(both plain pip packages, work anywhere). img_data.py imports THIS module and
wraps these as instance methods; this file is the single source of truth for
the math, so offline analysis tools (recompute_raw_flow.py, derive_pi_cal.py,
notebooks, ...) can import just the geometry they need without pulling in
img_data.py's full runtime (which imports imgstreamer -> picamera2, Pi-only).

DO NOT duplicate this math elsewhere - if it changes here, both the runtime
(img_data.py) and every offline tool automatically stay in sync.
"""
import os
import numpy as np
from ahrs import Quaternion

# Camera intrinsics.
# MEASURED via checkerboard calibration (compute_camera_calib.py, 2026-07,
# 6x9-square/8x5-inner-corner board), FINAL result on the ACTUAL negotiated
# 640x480 raw sensor mode. History: several attempts with the principal
# point FREE all gave unstable fx (1108-1347) with cx/cy landing OUTSIDE the
# image bounds - a persistent under-constrained-fit symptom that survived
# even a recapture with deliberate vertical board coverage, so it's a
# limitation of hand-held single-camera calibration on this rig, not a
# capture-technique fix. Cross-checked against the official IMX219 datasheet
# (focal length 3.04mm, pixel size 1.12um -> fx=2714px at the FULL 3280-wide
# sensor with 62.2 deg hfov): 640x480 is a documented distinct "640x480p90"
# sensor mode, not a simple downscale of the full-FOV binned modes, so a
# narrower FOV / higher fx-in-pixels than the full-sensor spec is physically
# expected, not a bug.
#
# RE-DERIVED 2026-07-28, supersedes the fx=1020.37 fit below. The prior
# "123-image set" this comment described was actually only 78 UNIQUE captures
# (25 flat-pose + 20 fresh + 33 vertical-coverage, 25+20+33=78) - the on-disk
# folder had 123 FILES because the 45-image midway "combined" checkpoint
# (itself just the 25+20 union) got copied into the final folder a second
# time under new names, so those 45 of 78 images carried double weight in the
# least-squares fit. Also hit a real cv2 5.x compat bug while re-deriving:
# findChessboardCorners' legacy fallback returns (N,2) on this opencv-python
# build, not the (N,1,2) the script assumed - crashed outright rather than
# silently miscalibrating, see compute_camera_calib.py's find_board() fix.
# Re-ran compute_camera_calib.py on the 78 DEDUPLICATED images only (still
# principal-point-fixed, still fx=fy constrained): reprojection err 0.36px
# (same as before - the duplication didn't corrupt the ANSWER, just its
# uncertainty), 78/78 images used, ZERO outliers rejected. Auto-detected
# board size 8x5 inner corners, independently confirmed against the actual
# board (user, 2026-07-28): a calib.io 6x9-square board has 5x8 inner
# corners - same count, transposed - strong evidence the fit used the real
# board geometry rather than a spurious sub-pattern match. New value is
# within 0.6% of the prior fx=1020.37 (1020.37 -> 1026.95) - the duplication
# bug was a methodology flaw, not a source of significant error; DID NOT
# turn out to explain the separately-suspected GT-vs-image scale mismatch
# (see project_pi_gt_lever_arm_2026_07_27) - that remains open.
# Full audit trail: Test_Data/Calibration/Camera/calib_images_dedup78/
# (images + cameraMatrix.txt/cameraDistortion.txt/calib_log.txt).
#
# Cross-validated (prior fit): independent fixed-center fits on the
# 45-image and 33-image subsets gave fx=1011/1074 respectively - all within
# ~6% of the final value, good convergence. Measured hfov ~34 deg (vs the
# IMX219's full-sensor 62 deg). The original fx=512/fy=384 was a linear
# scale-down of an UNVERIFIED spec-sheet guess (768/576 @ 960x720, itself
# never calibrated) and assumed a full-FOV bin - wrong assumption for this
# crop mode.
CALIB_CX = 319.50  # measured principal point (~= geometric center 320.0)
CALIB_CY = 239.50  # measured principal point (~= geometric center 240.0)
fx = 1026.95
fy = 1026.95
f = fx  # focal length in pixels
# NOTE: this hfov (~35 deg) is NOT the same as the Gazebo sim camera (1.74 rad
# / ~99.7 deg @640x480) - matching resolution alone never reproduced the sim's
# normalized-pixel geometry; PLASMC gains tuned against the sim FoV are not
# guaranteed to carry over as-is, now confirmed by real measurement.

# Camera-to-FC (body-FRD) mount rotation, confirmed 2026-07-11: the camera is
# downward-facing so its optical axis (ray z) already aligns with FC z (down)
# - no tilt/roll offset - but the board itself is mounted rotated -90 deg
# about that shared z axis. In camera-ray convention (x=pixel-right,
# y=pixel-down, z=optical/forward-along-boresight), body-FRD (x=forward,
# y=right, z=down): v_body = R_CAM_TO_BODY @ v_cam, i.e. camera "pixel-down"
# (+y_cam) is body-forward (+x_body), and camera "pixel-right" (+x_cam) is
# body-left (-y_body).
#
# VALIDATED 2026-07-23 (derive_pi_cal.py::check_mount_rotation, EGO_MOTION_
# ROT_SLOPE below): the structural assumption here - a PURE single-axis yaw
# mount, no roll/pitch cross-coupling, no axis flip - was independently
# confirmed from real ego-motion data (gyro yaw-rate vs the raw image-plane
# marker angle's own rate of change), NOT mocap and NOT FC yaw/heading (which
# is magnetometer-derived and confirmed unreliable indoors on this rig - see
# feedback_magnetometer_indoor_unreliable/project memory). All 7 existing
# output-cal recordings gave the SAME sign, slope clustering near -1.0 (best-
# excited run: -1.025, corr=-0.91) - exactly the physical expectation for a
# static marker viewed by a rotating camera (a fixed relative rotation
# between two rigid bodies never changes their SHARED angular rate; only a
# genuine axis flip would break the +-1 relationship), which is what a
# correctly-signed, non-mirrored, pure-yaw mount predicts. Does not
# independently pin down the mount's fixed OFFSET angle (the -90 deg above) -
# only that the rotation-rate coupling is right. A fixed relative rotation
# between two rigid bodies never changes their SHARED angular rate, so that
# check is structurally BLIND to the offset angle; it cannot be fixed by more
# yaw data, and needs a different observable entirely.
#
# AXIS MAPPING VALIDATED 2026-07-27 (supersedes this comment's former claim
# that "the translation/axis-mapping half of the matrix remains unvalidated").
# Supplied by exactly that different observable - TRANSLATION, phase-resolved.
# Phase labels were recovered from mocap (derive_pi_cal.phase_labels) and the
# V-frame centroid response measured per phase on the 2026-07-26 recordings:
#
#     body-X phase -> centroid col 0 std 0.088 / 0.063 / 0.042
#     body-Y phase -> centroid col 1 std 0.101 / 0.152 / 0.133
#                     (vs col 0 only 0.026 / 0.045 / 0.029 in the same phase)
#
# i.e. body-Y excitation drives V-frame column 1 by 3-5x over column 0. Since
# "Feature Params" is built by get_virtual_pts() BELOW - which applies this
# very matrix and then the gravity-levelling R_inv - a wrong or missing 90 deg
# here would have shown up as body-Y driving column 0 instead. It does not.
# The Y separation is clean; the X separation is real but weaker (1.1-1.7x),
# limited by that phase's much lower usable-sample count, not by ambiguity in
# the mapping.
#
# STILL UNVALIDATED: the mount TRANSLATION (lever arm) between the camera's
# optical centre and both the FC/mocap body origin and the target origin. That
# is a separate quantity from this rotation, is known non-zero (user-confirmed
# 2026-07-27), and shows up as a ~0.19-0.23 m systematic error in GT bearing s
# - see project_pi_gt_lever_arm_2026_07_27. It needs a physical measurement;
# least-squares on existing data absorbs it into nonsense (a -1.53 m camera
# z-offset). It does NOT affect optical FLOW, which differentiates a constant
# offset away - only the bearing/centroid chain.
R_CAM_TO_BODY = np.array([
    [0.0, 1.0, 0.0],
    [-1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0],
])

# Ego-motion-validated expected rotation-rate coupling (see the VALIDATED
# 2026-07-23 comment above): d(alpha)/dt / gyro_yaw_rate, measured -1.025 on
# the best-excited real recording (corr=-0.91), rounded to the physically-
# exact expectation for a non-mirrored pure-yaw mount. Single source of
# truth for derive_pi_cal.py::check_mount_rotation's pass/fail comparison -
# import this, don't re-hardcode -1.0 there.
EGO_MOTION_ROT_SLOPE = -1.0


def _quat_to_dcm(quat):
    return Quaternion([quat.w, quat.x, quat.y, quat.z]).to_DCM() if quat is not None else np.eye(3)


def _rp_basis(quat):
    """Roll/pitch-only (yaw-removed) rotation basis columns [x,y,z], shared
    by get_virtual_pts / get_real_pts_from_v / vframe_w."""
    R = _quat_to_dcm(quat)
    g = R @ np.array([0.0, 0.0, 1.0])
    z_axis = g / np.linalg.norm(g)
    x_axis = np.cross([0.0, 1.0, 0.0], z_axis)
    x_axis /= np.linalg.norm(x_axis)
    y_axis = np.cross(z_axis, x_axis)
    return np.column_stack([x_axis, y_axis, z_axis])


def get_virtual_pts(pts, quat, center=(CALIB_CX, CALIB_CY)):
    """Pixel points -> normalized, gravity-leveled virtual-frame points.
    quat: object with .w/.x/.y/.z (mavsdk-style), or None -> assume level."""
    R_rp = _rp_basis(quat)
    R_inv = R_rp.T

    cx, cy = center
    x = (pts[:, 0] - cx) / f
    y = (pts[:, 1] - cy) / f

    rays = np.column_stack([x, y, np.ones_like(x)])       # (N, 3), camera frame
    rays_body = rays @ R_CAM_TO_BODY.T                     # (N, 3), body-FRD
    vr = rays_body @ R_inv.T                               # (N, 3)

    z = vr[:, 2]
    vx = vr[:, 0] / z
    vy = vr[:, 1] / z
    return np.column_stack([vx, vy])


def get_real_pts_from_v(V_pts, quat, center=(CALIB_CX, CALIB_CY)):
    """INVERSE of get_virtual_pts: V-frame normalized points -> real pixels
    at the current tilt (used to seed the ring-flow patch each frame)."""
    R_rp = _rp_basis(quat)
    V_rays = np.column_stack([V_pts[:, 0], V_pts[:, 1], np.ones(len(V_pts))])
    C_rays_body = V_rays @ R_rp.T                          # body-FRD
    C_rays = C_rays_body @ R_CAM_TO_BODY                    # camera frame (inverse mount rotation)
    cx, cy = center
    xr = C_rays[:, 0] / C_rays[:, 2]
    yr = C_rays[:, 1] / C_rays[:, 2]
    return np.column_stack([xr * f + cx, yr * f + cy]).astype(np.float32)


def vframe_w(w_body, quat):
    """Rotate a body-FRD angular velocity into the virtual (gravity-leveled)
    frame, using the SAME basis as get_virtual_pts. w_body is the IMU's own
    body-FRD rate directly - no R_CAM_TO_BODY needed even though there IS a
    camera mount rotation, because only w_z (yaw rate) is ever read from the
    output, and a pure z-axis mount rotation leaves the z-component of any
    vector unchanged (shared axis)."""
    R_rp = _rp_basis(quat)
    return R_rp.T @ np.asarray(w_body, float)


def fill_A(centered_pts):
    """centered_pts: (N,2), N>=4 virtual-frame points. Returns the (2N,6)
    IBVS interaction matrix, evaluated per point (depth Z normalized to 1;
    the 1/Z scaling folds into the sensor calibration)."""
    x = centered_pts[:, 0]
    y = centered_pts[:, 1]
    N = len(x)
    A = np.zeros((2 * N, 6))

    A[0::2, 0] = 1
    A[1::2, 1] = 1

    A[0::2, 2] = -x
    A[1::2, 2] = -y

    A[0::2, 3] = -x * y
    A[1::2, 3] = -(1 + y**2)

    A[0::2, 4] = 1 + x**2
    A[1::2, 4] = x * y

    A[0::2, 5] = -y
    A[1::2, 5] = x

    return A


def scaled_quad_points(corners, scales=(1.0, 2.0 / 3.0, 1.0 / 3.0), per_side=15):
    """Dense flow points: 3 concentric quadrilaterals (scaled toward the
    centroid) with `per_side` points per side (~180 pts). Deterministic and
    texture-independent; reduces the single-marker flow noise via sqrt(N)
    averaging through the ill-conditioned interaction matrix. Returns (M,2)
    float32 in the SAME image frame as `corners` (cv2.aruco corner order
    preserved, so both frames' point sets correspond index-wise)."""
    c = np.asarray(corners, np.float32).reshape(-1, 2)
    ctr = c.mean(axis=0)
    pts = []
    for s in scales:
        q = ctr + s * (c - ctr)
        for i in range(4):
            A_, B_ = q[i], q[(i + 1) % 4]
            tv = np.linspace(0.0, 1.0, per_side)
            pts.append(np.outer(1.0 - tv, A_) + np.outer(tv, B_))
    return np.vstack(pts).astype(np.float32)


def marker_principal_angle(pts, prev_angle=None):
    """2pi marker orientation in the level V-plane (raw, no offset).

    The weighted 2nd-moment principal axis 0.5*arctan2(2 mu11, mu20-mu02)
    gives only an AXIS (pi-period) - invariant under 180deg. It must be
    disambiguated to a full 2pi DIRECTION.

    FIXED 2026-07-26 (found via derive_pi_cal.py::check_mount_rotation on
    real Pi recordings, 2026-07-25 session): the original disambiguation
    used the weighted-vs-geometric centroid displacement (the [4,3,2,1]
    corner weights pull the centroid toward the high-weight TL/TR corners, a
    1st-moment vector that should rotate 1:1 with the marker) - but that
    vector is SMALL by construction (~0.1-0.3x half-size), so ordinary
    corner-detection jitter can rotate its own angle across the +-90deg
    decision boundary even when the true marker orientation barely moved.
    Confirmed on real data: dalpha/dt between consecutive real-detection
    frames (~30-50ms apart) showed spurious +-50 rad/s spikes (literal ~pi
    flip-and-back within ~100ms) while the FC's own gyro read near-zero real
    rotation at that instant - a measurement artifact, not physical motion,
    that swamped every attempt to correlate image yaw-rate against ground
    truth. FIX: prefer TEMPORAL CONTINUITY - real rotation between two
    frames at normal camera rates is always << pi, so pick whichever of the
    two pi-periodic branches {a, a+pi} is closer to the PREVIOUS frame's
    disambiguated angle, when one is available. The old geometric heuristic
    is kept ONLY as the bootstrap for when there's no previous angle yet
    (first frame after a marker lock/re-lock, where continuity can't apply).
    Weights are tied to cv2.aruco's intrinsic corner order [TL,TR,BR,BL];
    without them a perfectly square marker has mu11==0 for all yaw (yaw blind).

    prev_angle: previous frame's disambiguated alpha (radians), or None to
    bootstrap via the one-shot geometric heuristic instead."""
    x, y = pts[:, 0], pts[:, 1]
    if len(x) == 4:
        w = np.array([4.0, 3.0, 2.0, 1.0])   # TL, TR, BR, BL
    else:
        w = np.ones(len(x))
    W = w.sum()
    xc = float(np.sum(w * x) / W)
    yc = float(np.sum(w * y) / W)
    Xc, Yc = x - xc, y - yc
    mu20 = float(np.sum(w * Xc * Xc))
    mu02 = float(np.sum(w * Yc * Yc))
    mu11 = float(np.sum(w * Xc * Yc))
    a = 0.0 if abs(mu11) < 1e-9 else 0.5 * np.arctan2(2 * mu11, mu20 - mu02)
    if prev_angle is not None:
        cand = (a, a + np.pi)
        dist = [abs(np.arctan2(np.sin(c - prev_angle), np.cos(c - prev_angle))) for c in cand]
        a = cand[0] if dist[0] <= dist[1] else cand[1]
    else:
        dx = xc - float(np.mean(x))
        dy = yc - float(np.mean(y))
        if dx * dx + dy * dy > 1e-18:
            d = a - np.arctan2(dy, dx)
            if abs(np.arctan2(np.sin(d), np.cos(d))) > np.pi / 2:
                a += np.pi
    return float(np.arctan2(np.sin(a), np.cos(a)))   # wrap to (-pi, pi]


def get_img_features(pts, prev_angle=None):
    """pts: virtual feature points in normalized frame, shape (N,2), already
    (x,y)=(u-cx)/f. For ArUco the 4 corners are in cv2.aruco order
    [TL,TR,BR,BL]. Returns the feature vector [xc, yc, area(=1), alpha].

    Centroid = GEOMETRIC (unweighted) mean - the clean lateral feature,
    decoupled from yaw. (A [4,3,2,1]-weighted centroid, as in Gazebo, is
    offset ~0.4*half-size in the MARKER frame, so it rotates with yaw and
    injects a yaw-coupled lateral bias + degrades the linear centroid cal.)

    Yaw alpha = 2pi-disambiguated WEIGHTED-moment orientation - the weights
    are required to make a square marker yaw-observable. alpha_0 = 0 here
    (Pi-specific; recalibrate the equilibrium offset once good mocap data
    exists, mirroring Gazebo's _moment_alpha_0).

    prev_angle: passed straight through to marker_principal_angle - see its
    docstring (2026-07-26 fix) for why continuity beats the single-frame
    geometric heuristic once a previous estimate exists."""
    x = pts[:, 0]
    y = pts[:, 1]

    xc = float(np.mean(x))
    yc = float(np.mean(y))

    alpha_0 = 0.0
    raw = marker_principal_angle(pts, prev_angle=prev_angle)
    alpha = float(np.arctan2(np.sin(raw - alpha_0), np.cos(raw - alpha_0)))

    return np.array([xc, yc, 1.0, alpha])


def quad_ill_conditioned(pts, min_angle_deg=None, max_elong=None):
    """Ported 2026-07-26 from PX4_Gazebo/src/img_data.py::_quadIllConditioned
    (2026-07-19 there) - True iff a 4-corner marker quad is geometrically
    DEGENERATE: a foreshortened sliver that cannot localize a centre or
    well-condition a homography. Signature at a tilt-grazing/near-edge
    terminal: the two diagonals go near-PARALLEL (angle ~ a few deg vs ~90
    when square) and/or the quad ELONGATES (max/min side ratio far above 1).
    Either makes the diagonal-intersection centre swing wildly (~1/sin(angle)
    amplification) and the homography under-determined.

    Used to gate PlanarFeatureMap.loop_closure_correct(): a real ArUco decode
    should always re-anchor the map (SLAM loop-closure principle) EXCEPT when
    the decode's own quad is this degenerate - correcting the map from a
    near-singular observation would poison its learned geometry rather than
    improve it. Ordered ArUco corners (TL,TR,BR,BL): diagonals are 0-2 & 1-3.
    Corner-order-agnostic elongation via all 4 side lengths.

    Thresholds default to the same env vars Gazebo uses (MAP_ILLCOND_ANGLE_DEG=20,
    MAP_ILLCOND_ELONG=5) so the two ports stay tunable the same way; pass
    explicit values to override without touching the environment."""
    if min_angle_deg is None:
        min_angle_deg = float(os.environ.get("MAP_ILLCOND_ANGLE_DEG", "20"))
    if max_elong is None:
        max_elong = float(os.environ.get("MAP_ILLCOND_ELONG", "5"))
    try:
        c = np.asarray(pts, dtype=float).reshape(-1, 2)
        if c.shape[0] != 4 or not np.all(np.isfinite(c)):
            return True   # malformed = not usable for correction
        d1 = c[2] - c[0]; d2 = c[3] - c[1]
        n1 = np.linalg.norm(d1); n2 = np.linalg.norm(d2)
        if n1 < 1e-6 or n2 < 1e-6:
            return True
        ang = np.degrees(np.arccos(np.clip(abs(np.dot(d1, d2) / (n1 * n2)), 0.0, 1.0)))
        sides = [np.linalg.norm(c[(i + 1) % 4] - c[i]) for i in range(4)]
        elong = max(sides) / max(min(sides), 1e-6)
        return bool(ang < min_angle_deg or elong > max_elong)
    except Exception:
        return True


def marker_near_fov_edge(pts, resolution_hw, margin_px):
    """Ported 2026-07-26 from PX4_Gazebo/src/img_data.py's inline near-edge
    check (same call site as _quadIllConditioned, MAP_REJECT_OVERFLOW_CORRECT
    gate) - True iff any corner sits within margin_px of the frame boundary.
    Corner precision degrades near the frame edge (partial occlusion by the
    boundary itself, more aggressive foreshortening) - a near-edge decode's
    own corner positions are less trustworthy, so loop-closure correction
    should not re-anchor the map from one even if the quad shape itself still
    passes quad_ill_conditioned. resolution_hw: (height, width) of the image
    space pts are expressed in (main-stream on the Pi, matching
    _last_locked_corners_main - NOT the raw sensor resolution)."""
    c = np.asarray(pts, dtype=float).reshape(-1, 2)
    h, w = resolution_hw
    m = margin_px
    return bool(c[:, 0].min() < m or c[:, 0].max() > w - m
                or c[:, 1].min() < m or c[:, 1].max() > h - m)
