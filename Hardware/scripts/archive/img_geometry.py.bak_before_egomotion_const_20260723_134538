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
# expected, not a bug. FINAL FIT: principal point fixed at the geometric
# center (CALIB_FIX_PRINCIPAL_POINT) on the full combined 123-image set (all
# sessions: original 25 flat-pose + 20 fresh + 33 with deliberate vertical
# coverage) - reprojection err 0.36px, small well-behaved distortion (vs
# wild +/-10-30 coefficients in the unstable free-principal-point runs).
# Cross-validated: independent fixed-center fits on the 45-image and
# 33-image subsets gave fx=1011/1074 respectively - all three within ~6% of
# this final value, good convergence. Measured hfov ~34 deg (vs the IMX219's
# full-sensor 62 deg). The original fx=512/fy=384 was a linear scale-down of
# an UNVERIFIED spec-sheet guess (768/576 @ 960x720, itself never
# calibrated) and assumed a full-FOV bin - wrong assumption for this crop mode.
CALIB_CX = 319.50  # measured principal point (~= geometric center 320.0)
CALIB_CY = 239.50  # measured principal point (~= geometric center 240.0)
fx = 1020.37
fy = 1020.37
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
R_CAM_TO_BODY = np.array([
    [0.0, 1.0, 0.0],
    [-1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0],
])


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


def marker_principal_angle(pts):
    """2pi marker orientation in the level V-plane (raw, no offset).

    The weighted 2nd-moment principal axis 0.5*arctan2(2 mu11, mu20-mu02)
    gives only an AXIS (pi-period) - invariant under 180deg. It is
    disambiguated to a full 2pi DIRECTION via the weighted-vs-geometric
    centroid displacement: the [4,3,2,1] corner weights pull the centroid
    toward the high-weight (TL/TR) corners, a 1st-moment vector that rotates
    1:1 with the marker. Flip the axis to whichever 180deg end aligns with it.
    Weights are tied to cv2.aruco's intrinsic corner order [TL,TR,BR,BL];
    without them a perfectly square marker has mu11==0 for all yaw (yaw blind).
    """
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
    dx = xc - float(np.mean(x))
    dy = yc - float(np.mean(y))
    if dx * dx + dy * dy > 1e-18:
        d = a - np.arctan2(dy, dx)
        if abs(np.arctan2(np.sin(d), np.cos(d))) > np.pi / 2:
            a += np.pi
    return float(np.arctan2(np.sin(a), np.cos(a)))   # wrap to (-pi, pi]


def get_img_features(pts):
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
    exists, mirroring Gazebo's _moment_alpha_0)."""
    x = pts[:, 0]
    y = pts[:, 1]

    xc = float(np.mean(x))
    yc = float(np.mean(y))

    alpha_0 = 0.0
    raw = marker_principal_angle(pts)
    alpha = float(np.arctan2(np.sin(raw - alpha_0), np.cos(raw - alpha_0)))

    return np.array([xc, yc, 1.0, alpha])
