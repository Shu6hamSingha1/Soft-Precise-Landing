"""Standalone perception pipeline for the cross+stub fiducial -- NOT routed
through IMG_PROCESSOR/img_data.py's ArUco-shaped machinery (no synthetic
4-corner packaging, no PlanarFeatureMap rescue, no marker handover). Those
subsystems exist to patch specific ArUco weaknesses (decode-or-nothing
dropouts; multi-marker nested boards needed for decode range) that the cross
marker's line-geometry-based, single-marker, partial-credit-native detection
doesn't have -- see docs/cross_marker.pdf and the design discussion history.

Computes the manuscript's actual image-feature quantities (manuscript.tex,
Sec. "Image Features & Optical Flow"), NOT the project's copied
ArUco-specific conventions:
  s     = homogeneous image position [xc, yc, 1] -- the line-intersection
          point directly (cross_marker_detector.detect()), no corner
          averaging detour.
  alpha = unweighted 2nd-moment principal angle over the REAL detected
          arm/stub pixels (no ArUco-style artificial corner weighting --
          the stub's asymmetry is real, not synthesized). Hold-last-good
          when the stub cluster isn't found this frame (no separate
          observability-metric gate -- yaw control tolerates interruption).
  h, w  = optical flow (image velocity / image angular velocity), recovered
          via the SAME image-Jacobian pseudo-inverse solve used for ArUco
          corners (img_data.py's _fill_A, mirrored here), but fed
          Shi-Tomasi points sampled across the WHOLE color/shape-gated
          marker plate (drawn cross+stub AND the surrounding texture) --
          not just the 4 corners of a synthetic quad. This fixes the two
          documented ArUco flow problems instead of patching around them:
          point starvation (a flat ArUco marker has ~4-8 resolvable
          corners; GFT regressed there -- see img_data.py:2018-2021) and
          Jacobian ill-conditioning at small (x,y) (img_data.py:1944-1947)
          -- the textured plate gives many REAL, well-spread trackable
          points instead of 180 synthetic points interpolated from 4 real
          ones.

No KLT tracking of the intersection point itself (see design discussion --
the feature KF's predict step already provides the smoothing/coasting a
dedicated tracker would duplicate; a near-identical mechanism was built and
then removed as redundant for ArUco, 2026-07-17).
"""
import os
import time
from threading import Thread

import cv2
import numpy as np

import cross_marker_detector as cmd
from img_data import fx, fy   # reuse the project's calibrated intrinsics, don't refork them
from gz_subscriber import GZ_Subscriber, Image_Node   # Image_Node is decode-agnostic raw capture --
                                                        # no ArUco logic in it, safe to reuse as-is

GFT_MAX_CORNERS = int(os.environ.get("CROSS_GFT_MAX_CORNERS", "60"))
GFT_QUALITY = float(os.environ.get("CROSS_GFT_QUALITY", "0.02"))
GFT_MIN_DIST = float(os.environ.get("CROSS_GFT_MIN_DIST", "6"))
LK_WIN = (15, 15)
LK_MAX_LEVEL = 2

# Diagnosed 2026-08-01: at 5m altitude the isolated marker mask is only ~76x76px,
# which has a physical ceiling of ~7 GFT corners regardless of quality/minDistance
# tuning (verified directly -- retuning those knobs changed nothing). With a single
# MIN_FLOW_POINTS cliff at 6, one lost point (LK failure or mask-membership reject)
# was forcing a full resample + a zero-output frame on the very next sample. Two
# separate thresholds replace that cliff:
MIN_FLOW_POINTS_SOLVE = 4     # attempt the lstsq with whatever survived, down to this floor
                               # (2*4=8 >= 6 unknowns -- still solvable, just less overdetermined)
RESAMPLE_TRIGGER = 10          # proactively top up the point pool below this count, but don't
                               # discard current tracking to do it (see _compute_hw)
MASK_DILATE_PX = 4             # dilation radius for BOTH GFT sampling bounds and the post-LK
                               # mask-membership retention check -- a fresh per-frame recomputed
                               # mask jitters by a few px frame-to-frame; a tracked point that's
                               # still physically on the marker but just outside this frame's
                               # exact mask boundary was being discarded as "off-target," which
                               # was avoidable attrition, not a real off-target rejection.


def _unweighted_principal_angle(pts):
    """Manuscript alpha (Sec. Image Features, eq. ~197): plain 2nd-moment
    principal angle over N>=3 non-collinear points, pi-disambiguated via the
    (here: unweighted vs a coarse asymmetric sub-split) centroid displacement.
    Unlike img_data.py's _marker_principal_angle, NO [4,3,2,1] weighting --
    that hack exists only because a symmetric ArUco square has mu_11==0
    under uniform weights. The cross marker's stub is a REAL asymmetry, so
    the plain formula is directly usable when the stub cluster is present.
    `pts` must already include the stub points for disambiguation to work;
    callers are responsible for holding-last when the stub isn't detected.
    """
    pts = np.asarray(pts, dtype=np.float64)
    x, y = pts[:, 0], pts[:, 1]
    xc, yc = float(np.mean(x)), float(np.mean(y))
    Xc, Yc = x - xc, y - yc
    mu20 = float(np.sum(Xc * Xc))
    mu02 = float(np.sum(Yc * Yc))
    mu11 = float(np.sum(Xc * Yc))
    a = 0.0 if abs(mu11) < 1e-9 else 0.5 * np.arctan2(2 * mu11, mu20 - mu02)
    return a, (xc, yc)


def _disambiguate_angle(a, asym_vec):
    """pi -> 2pi disambiguation using a real asymmetry vector (e.g. stub
    centroid minus arm-only centroid) instead of ArUco's weighted-vs-geometric
    centroid trick -- same idea, applied to the cross marker's own asymmetry."""
    dx, dy = asym_vec
    if dx * dx + dy * dy < 1e-12:
        return a
    d = a - np.arctan2(dy, dx)
    if abs(np.arctan2(np.sin(d), np.cos(d))) > np.pi / 2:
        a += np.pi
    return float(np.arctan2(np.sin(a), np.cos(a)))


def _fill_A(centered_pts):
    """Image Jacobian for a set of normalized (x,y) points, depth-normalized
    (Z=1 folded into calibration, matching img_data.py's convention). Mirrors
    img_data.py's _fill_A EXACTLY (same 6-DOF [h1,h2,h3,w1,w2,w3] ordering) --
    duplicated rather than imported so this module has no dependency on
    IMG_PROCESSOR's class internals, only the shared math."""
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


class CrossMarkerPerception:
    """Stateful, ROS-independent core -- call process_frame(img, t) once per
    captured frame. Kept separate from any ROS/threading wrapper so it can be
    unit-tested offline against saved frames (this project's established
    validation pattern), same as cross_marker_detector.detect() was."""

    def __init__(self, resolution=(480, 640)):
        self._resolution = resolution   # (h, w), rotated-frame convention (see img_data.py:69-85)
        self.center = np.array(resolution[::-1]) / 2.0   # (cx, cy)
        self.focal = np.array([fx, fy])

        self._prev_gray = None
        self._prev_flow_pts = None   # (N,2) pixel coords tracked for h,w
        self._last_alpha = 0.0
        self._alpha_valid_once = False

        # last computed outputs, for the getter interface
        self._s = np.array([0.0, 0.0, 1.0])
        self._center_px = None   # (2,) raw pixel [cx, cy] of the last successful detection,
                                  # for the visibility CBF (cbf_visibility.cbf2_filter takes
                                  # raw pixels and normalizes internally -- self._s is already
                                  # normalized, wrong units for that call)
        self._center_fresh = False   # True only on frames where det.ok this exact frame
        self._alpha = 0.0
        self._hw = np.zeros(6)
        self._ok = False
        self._last_t = None

        # Output calibration (GT = cal @ raw), same convention as img_data.py's
        # _sensor_cal_hw / _sensor_cal_s -- the cross marker's h,w come from its
        # own image-Jacobian solve (not img_data.py's), so it needs its own
        # empirical correction, not a reuse of the ArUco board's cal.
        # Derived 2026-08-02 via apps/record_cross_marker_calibration.py +
        # tools/derive_cross_marker_cal.py, from 4 phased-excitation runs (a 5th
        # was gated out at 69.5% detection ok-rate; CROSS_CAL_MIN_OKRATE=0.85) --
        # see Memory/px4/project_cross_marker_pipeline_20260801.md for the full
        # investigation (Gazebo camera-render ghost artifact root-caused +
        # layered shape/position rejection fixes in cross_marker_detector.py).
        # R^2: Hx=0.40 Hy=0.49 Hz=0.34 Wz=0.36 (Wx/Wy forced 0, same level-target
        # convention as the ArUco board cal). Centroid scale still has ~2.4x
        # inter-run spread -- usable starting point, not as tight as the ArUco
        # cal; re-derive with more runs if precision landing needs tightening.
        self._sensor_cal_hw = np.array([
            [+0.1651, +0.0040, +0.0535, +0.0062, +0.1837, -0.0485],
            [-0.0090, +0.2024, -0.0558, -0.2010, -0.0180, -0.0703],
            [-0.0274, -0.0369, +0.5992, +0.3072, +0.1833, +0.0229],
            [+0.0000, +0.0000, +0.0000, +0.0000, +0.0000, +0.0000],
            [+0.0000, +0.0000, +0.0000, +0.0000, +0.0000, +0.0000],
            [-0.1273, +3.3435, -0.8369, -2.2182, -0.1183, -0.6168]])
        self._sensor_cal_s = np.diag([0.3217, 0.2862, 1.0, 1.0])

        # Diagnostic instrumentation (2026-08-01, point-starvation/centroid-instability
        # investigation): per-frame (t, ok, fail_reason, bbox_area) log, always cheap
        # to keep. Frame dumps around ok-state transitions are opt-in via
        # CROSS_DIAG_SAVE_DIR (off by default -- full-res frame writes are not free).
        self._diag_log = []
        self._diag_save_dir = os.environ.get("CROSS_DIAG_SAVE_DIR", "")
        self._diag_frame_idx = 0
        self._diag_lost_streak = 0
        if self._diag_save_dir:
            os.makedirs(self._diag_save_dir, exist_ok=True)

    @staticmethod
    def _dilate_mask(mask):
        kernel = np.ones((2 * MASK_DILATE_PX + 1, 2 * MASK_DILATE_PX + 1), np.uint8)
        return cv2.dilate(mask, kernel)

    def _sample_flow_points(self, gray, dilated_mask):
        pts = cv2.goodFeaturesToTrack(
            gray, maxCorners=GFT_MAX_CORNERS, qualityLevel=GFT_QUALITY,
            minDistance=GFT_MIN_DIST, mask=dilated_mask)
        if pts is None:
            return np.zeros((0, 2), dtype=np.float32)
        return pts.reshape(-1, 2).astype(np.float32)

    def _solve_jacobian(self, prev_pts, curr_pts, dt):
        prev_n = (prev_pts - self.center) / self.focal
        curr_n = (curr_pts - self.center) / self.focal
        vel = (curr_n - prev_n) / dt   # (N,2) per-point normalized velocity
        A = _fill_A(prev_n)
        b = vel.reshape(-1)
        sol, *_ = np.linalg.lstsq(A, b, rcond=None)
        return sol

    def _compute_hw(self, gray, mask, dt):
        """LK-track flow points from the previous frame, solve the image
        Jacobian pseudo-inverse for [h1,h2,h3,w1,w2,w3]. Two-threshold
        hysteresis (see MIN_FLOW_POINTS_SOLVE/RESAMPLE_TRIGGER comments):
        attempt the solve with however many points survived tracking, down to
        a hard floor; only DISCARD current tracking for a full fresh-sample
        when the count drops below that floor. A fresh-sample frame still
        can't report a velocity (no correspondence yet), but that no longer
        also happens on every minor, recoverable point-count dip."""
        dilated_mask = self._dilate_mask(mask)

        if self._prev_gray is None or self._prev_flow_pts is None or len(self._prev_flow_pts) == 0:
            self._prev_flow_pts = self._sample_flow_points(gray, dilated_mask)
            self._prev_gray = gray
            return np.zeros(6), False

        tracked, status, _ = cv2.calcOpticalFlowPyrLK(
            self._prev_gray, gray, self._prev_flow_pts, None,
            winSize=LK_WIN, maxLevel=LK_MAX_LEVEL)
        status = status.flatten().astype(bool)
        # keep only points that (a) tracked successfully and (b) are still within
        # the DILATED marker mask -- on-target guarantee, but tolerant of a few
        # px of frame-to-frame mask-boundary jitter (see MASK_DILATE_PX)
        in_mask = np.zeros(len(tracked), dtype=bool)
        h, w = dilated_mask.shape
        for k, p in enumerate(tracked):
            xi, yi = int(round(p[0])), int(round(p[1]))
            if 0 <= xi < w and 0 <= yi < h:
                in_mask[k] = dilated_mask[yi, xi] > 0
        keep = status & in_mask

        prev_pts = self._prev_flow_pts[keep]
        curr_pts = tracked[keep]
        n_kept = len(prev_pts)

        if n_kept < RESAMPLE_TRIGGER:
            # top up the pool with fresh candidates rather than discarding what's
            # still tracking -- only replace outright if the fresh yield actually
            # beats what survived (never make the pool WORSE)
            fresh = self._sample_flow_points(gray, dilated_mask)
            self._prev_flow_pts = fresh if len(fresh) > n_kept else curr_pts
        else:
            self._prev_flow_pts = curr_pts
        self._prev_gray = gray

        if n_kept < MIN_FLOW_POINTS_SOLVE or dt <= 0:
            return np.zeros(6), False

        sol = self._solve_jacobian(prev_pts, curr_pts, dt)
        return sol, True   # [h1,h2,h3,w1,w2,w3]

    def process_frame(self, img_bgr, t):
        dt = 0.0 if self._last_t is None else max(t - self._last_t, 1e-6)
        self._last_t = t

        det = cmd.detect(img_bgr)
        gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
        bbox_area = (det.mask_bbox[2] * det.mask_bbox[3]) if det.mask_bbox else 0
        self._diag_log.append((t, bool(det.ok), det.fail_reason, bbox_area))
        self._diag_frame_idx += 1

        if not det.ok:
            self._ok = False
            self._center_fresh = False   # explicit: this frame did NOT confirm the center is
                                          # in-frame -- the CBF must not treat a stale self._center_px
                                          # as a live in-FoV measurement (see get_center_px())
            # hold last s/alpha; hw defaults to zero (no motion info without a fix)
            self._hw = np.zeros(6)
            self._diag_lost_streak += 1
            # dump the frame at the START of a loss streak (streak==1) and every 30
            # frames thereafter, so a long freeze is sampled across its duration
            # instead of just once -- lets us see whether the cause is static (same
            # failure mode throughout) or drifts (e.g. lighting/tilt changing).
            if self._diag_save_dir and (self._diag_lost_streak == 1 or self._diag_lost_streak % 30 == 0):
                fn = os.path.join(self._diag_save_dir,
                                   f"lost_{self._diag_frame_idx:06d}_streak{self._diag_lost_streak}_{det.fail_reason}.png")
                cv2.imwrite(fn, img_bgr)
            return self.get_output()

        if self._diag_save_dir and self._diag_lost_streak >= 5:
            # recovery frame right after a real (>=5-frame) loss streak
            fn = os.path.join(self._diag_save_dir,
                               f"recovered_{self._diag_frame_idx:06d}_afterstreak{self._diag_lost_streak}.png")
            cv2.imwrite(fn, img_bgr)
        self._diag_lost_streak = 0

        cx, cy = det.center
        self._center_px = np.array([cx, cy])
        self._center_fresh = True
        s_xy = (np.array([cx, cy]) - self.center) / self.focal
        self._s = np.array([s_xy[0], s_xy[1], 1.0])

        # --- alpha: unweighted moment over REAL detected arm/stub pixels ---
        pts_for_alpha = list(det.line_points_i) + list(det.line_points_j)
        if det.stub_points is not None and len(det.stub_points) >= 2:
            stub_pts = np.asarray(det.stub_points, dtype=np.float64)
            arm_pts = np.asarray(pts_for_alpha, dtype=np.float64)
            all_pts = np.vstack([arm_pts, stub_pts])
            a_raw, _ = _unweighted_principal_angle(all_pts)
            asym = (float(stub_pts[:, 0].mean() - arm_pts[:, 0].mean()),
                    float(stub_pts[:, 1].mean() - arm_pts[:, 1].mean()))
            a = _disambiguate_angle(a_raw, asym)
            self._last_alpha = a
            self._alpha_valid_once = True
        # else: stub not found this frame -- hold last-good alpha (no fresh compute)
        self._alpha = self._last_alpha if self._alpha_valid_once else 0.0

        # --- h, w: image Jacobian solve over Shi-Tomasi points on the marker plate ---
        mask = det.isolated_mask if det.isolated_mask is not None else np.zeros(gray.shape, np.uint8)
        hw, hw_ok = self._compute_hw(gray, mask, dt)
        self._hw = hw if hw_ok else np.zeros(6)

        self._ok = True
        return self.get_output()

    def get_output(self):
        return dict(ok=self._ok, s=self._s.copy(), alpha=self._alpha, hw=self._hw.copy())

    # ---- getters mirroring IMG_PROCESSOR's public interface, for controller.py ----
    def getRawImgFeatureParam(self):
        """Uncalibrated [xc, yc, 1, alpha] -- what the calibration recorder logs."""
        return np.array([self._s[0], self._s[1], self._s[2], self._alpha])

    def getRawOptFlowAngVel(self):
        """Uncalibrated [h1,h2,h3,w1,w2,w3] -- what the calibration recorder logs."""
        return self._hw.copy()

    def getImgFeatureParam(self):
        return self._sensor_cal_s @ self.getRawImgFeatureParam()

    def getOptFlowAngVel(self):
        return self._sensor_cal_hw @ self.getRawOptFlowAngVel()

    def get_diag_log(self):
        """List of (t, ok, fail_reason, bbox_area) -- one entry per process_frame call."""
        return list(self._diag_log)

    def get_center_px(self):
        """(2,) raw pixel [cx, cy] for the visibility CBF, or None if this frame
        did not confirm the center is in-frame (see _center_fresh in
        process_frame) -- the CBF must see None on a genuine miss, not a
        silently-stale pixel value, so its own Phase-2 fallback engages
        correctly rather than being fed a frozen "live" measurement."""
        if not self._center_fresh:
            return None
        return self._center_px.copy()

    @property
    def FEATURE_IS_VISIBLE(self):
        return self._ok


class CrossMarkerNode(Thread):
    """Thread wrapper around CrossMarkerPerception, exposing the minimal subset
    of IMG_PROCESSOR's public interface that controller.py actually calls
    (catalogued directly from controller.py's `self._img_node.*` usages,
    2026-08-01) -- NOT a full IMG_PROCESSOR replacement. Attributes tied to
    ArUco-specific subsystems this design deliberately drops (ring-loom
    fusion, PlanarFeatureMap rescue, marker handover, terminal-kick gating)
    are NOT implemented; if controller.py code paths that touch those are
    exercised under MARKER_TYPE=cross (e.g. a real descent/terminal-kick,
    vs. the hover-only sanity path this was validated against), expect
    AttributeErrors there -- that is a known, currently out-of-scope gap,
    not a silent failure.

    Reuses gz_subscriber.Image_Node verbatim for capture -- it is decode-
    agnostic (raw BGR frame + the existing 90deg-CW rotation convention),
    no ArUco logic lives in it.
    """

    def __init__(self, time_keeper=time, controller=None):
        Thread.__init__(self)
        self._time = time_keeper
        self._FC = controller
        self._image_node = Image_Node(time_keeper=time_keeper, controller=controller)
        self._image_sub = GZ_Subscriber(self._image_node)

        self._resolution = self._image_node.getImgResolution()   # blocks until first frame
        self._perception = CrossMarkerPerception(resolution=self._resolution)
        self.center = self._perception.center
        self.focal = self._perception.focal

        # plain-attribute stand-ins for controller.py's other self._img_node.* touches
        self.RECORD = False
        self.CONTROLLER_READY = False
        self._feature_pts = []
        self._ring_loom_source = "n/a"

        self._running = True
        self.start()

    def run(self):
        last_stamp = None
        while self._running:
            try:
                stamp = self._image_node.getStamp()
                if stamp is None or stamp == last_stamp:
                    time.sleep(0.002)
                    continue
                last_stamp = stamp
                imgs = self._image_node.getImages()
                if imgs[-1] is None:
                    time.sleep(0.002)
                    continue
                self._perception.process_frame(imgs[-1], self._time.perf_counter())
            except Exception as e:
                print(f"[CrossMarkerNode] frame processing error: {e}")
                time.sleep(0.01)

    def getImgFeatureParam(self):
        return self._perception.getImgFeatureParam()

    def getOptFlowAngVel(self):
        return self._perception.getOptFlowAngVel()

    def getRawImgFeatureParam(self):
        return self._perception.getRawImgFeatureParam()

    def getRawOptFlowAngVel(self):
        return self._perception.getRawOptFlowAngVel()

    def get_diag_log(self):
        return self._perception.get_diag_log()

    def get_center_px(self):
        return self._perception.get_center_px()

    @property
    def FEATURE_IS_VISIBLE(self):
        return self._perception.FEATURE_IS_VISIBLE

    def getLogData(self):
        return dict(s=self._perception._s.copy(), alpha=self._perception._alpha,
                     hw=self._perception._hw.copy())

    def getParams(self):
        return dict(center=self.center.copy(), focal=self.focal.copy())

    def close(self):
        self._running = False
        try:
            self._image_sub.close()
        except Exception:
            pass

    def __del__(self):
        self._running = False
