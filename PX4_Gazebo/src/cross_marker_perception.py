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
from ahrs import Quaternion

import cross_marker_detector as cmd
from img_data import fx, fy   # reuse the project's calibrated intrinsics, don't refork them
from gz_subscriber import GZ_Subscriber, Image_Node   # Image_Node is decode-agnostic raw capture --
                                                        # no ArUco logic in it, safe to reuse as-is

GFT_MAX_CORNERS = int(os.environ.get("CROSS_GFT_MAX_CORNERS", "60"))
GFT_QUALITY = float(os.environ.get("CROSS_GFT_QUALITY", "0.02"))
GFT_MIN_DIST = float(os.environ.get("CROSS_GFT_MIN_DIST", "6"))
# 2026-08-02 (Hx/Hy investigation): the marker's speckle texture measured at
# ~13-18px/blob across the useful altitude range (see
# feedback_duplicated_math_diff_check's sibling investigation), comparable to
# the default 15x15 LK window -- a window that size can straddle multiple
# repeating blobs, an aperture/correspondence-ambiguity risk. Env-overridable
# to test smaller windows without a code edit each time.
_lk_win = int(os.environ.get("CROSS_LK_WIN", "15"))
LK_WIN = (_lk_win, _lk_win)
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
MASK_DILATE_PX = int(os.environ.get("CROSS_MASK_DILATE_PX", "4"))
                               # dilation radius for BOTH GFT sampling bounds and the post-LK
                               # mask-membership retention check -- a fresh per-frame recomputed
                               # mask jitters by a few px frame-to-frame; a tracked point that's
                               # still physically on the marker but just outside this frame's
                               # exact mask boundary was being discarded as "off-target," which
                               # was avoidable attrition, not a real off-target rejection.
                               # 2026-08-02 (Hx/Hy texture investigation): the color gate
                               # (V<20) only sees the black cross+stub, not the surrounding
                               # textured background -- so this dilation radius is ALSO what
                               # decides how much of that background texture GFT actually gets
                               # to sample. Default 4px was a thin margin; env-overridable to
                               # test a much wider band so a retextured background can matter
                               # for optical flow at all.


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
        # resolution is `_image_node.getImgResolution()` = (msg.height, msg.width)
        # of the ORIGINAL pre-rotation image, i.e. (480, 640) for this camera --
        # NOT the rotated frame's own (h, w). Because cv2.ROTATE_90_CW swaps the
        # two dimensions, `resolution[0]` (480) happens to equal the ROTATED
        # frame's width and `resolution[1]` (640) its height, so
        # np.array(resolution)/2 = (240, 320) is ALREADY (cx, cy) with no
        # reversal needed. A `[::-1]` here silently transposes cx/cy, corrupting
        # every point normalization downstream (the h,w Jacobian solve AND the
        # `s` centroid). See img_data.py:69-91's identical convention -- that
        # file's own comment documents someone adding this exact `[::-1]` on
        # 2026-06-01, believing it fixed a bug, then reverting it after
        # empirical verification showed the original was correct. This module
        # repeated that same mistake fresh on 2026-08-01; fixed 2026-08-02 after
        # a flow-accuracy investigation (near-zero raw-vs-GT correlation despite
        # healthy point counts/conditioning/in-sample fit residual) traced back
        # to this transpose. Do NOT re-add `[::-1]` without re-deriving why.
        self._resolution = resolution
        self.center = np.array(resolution) / 2.0   # (cx, cy) = (240, 320)
        self.focal = np.array([fx, fy])

        self._prev_gray = None
        self._prev_flow_pts = None   # (N,2) pixel coords tracked for h,w
        self._prev_quat = None       # quat AT prev_flow_pts' own frame (2026-08-02 V-frame fix)
        self._prev_frame_t = None    # timestamp AT prev_flow_pts' own frame (2026-08-03 dt-staleness
                                      # fix -- see _compute_hw's docstring)
        self._z_v_log = []           # min(z_v) per _getVirtualPts call -- degeneracy diagnostic
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
        # RE-DERIVED 2026-08-03 (supersedes the 2026-08-02 version) via
        # apps/record_cross_marker_calibration.py + tools/derive_cross_marker_cal.py,
        # from 5 clean phased-excitation runs (all passed the 95% detection
        # ok-rate gate), AFTER root-causing and fixing two real bugs found during
        # this session's Hx/Hy flow-accuracy investigation (see
        # Memory/px4/project_cross_marker_pipeline_20260801.md and
        # feedback_missing_vframe_leveling_port / feedback_dt_staleness_after_
        # detection_dropout for the full writeups):
        #   1. Missing V-frame gravity-leveling (_getVirtualPts) -- this module
        #      computed h,w,s directly from raw, un-leveled camera-frame pixels
        #      with zero attitude compensation, while GT (compute_gt_signals) is
        #      in the tilt-compensated V-frame img_data.py's ArUco pipeline
        #      already reprojects through. Ported _getVirtualPts in, wired the
        #      quaternion through CrossMarkerNode via Image_Node.getQuaternions()
        #      (tightly paired with each frame at capture time, NOT a separately-
        #      polled FC.getQuat() -- that first attempt regressed Hx instead of
        #      fixing it).
        #   2. dt/staleness mismatch on detection-dropout recovery: dt was built
        #      from the outer polling clock (advances every call) while the LK
        #      "previous frame" state only advances on successful detections --
        #      after any dropout (common, every run had some), the next good
        #      frame divided a multi-frame-accumulated displacement by a
        #      single-frame dt, spiking all six solved parameters. Fixed by
        #      tracking the timestamp actually paired with self._prev_gray.
        # Effect: raw Hx/Hy-vs-GT correlation went from ~0.01-0.10 (every test
        # before these fixes) to a consistent 0.74-0.87 across independent
        # flights. R^2 (this fit): Hx=0.70 Hy=0.71 Hz=0.42 Wz=0.52 (Wx/Wy forced
        # 0, same level-target convention as the ArUco board cal) -- Hx/Hy are
        # now within 0.05 of ArUco's own 13-run board cal (0.75/0.75); Hz/Wz
        # remain the honest gap (ArUco: 0.79/0.71). Centroid scale sx/sy inter-
        # run spread dropped to ~7-10% (was ~2.4x before these fixes) -- TIGHTER
        # than ArUco's own current live cal (h_x 1.091+-0.266 = ~24%, h_y
        # 1.063+-0.198 = ~19%, per img_data.py's own provenance comments).
        self._sensor_cal_hw = np.array([
            [+0.9108, +0.0771, -0.0143, -0.0965, +0.9116, -0.0023],
            [+0.0518, +1.0905, -0.0532, -1.1048, +0.0552, -0.0049],
            [-0.0468, +0.1164, +0.4899, -0.1512, -0.1164, -0.0079],
            [+0.0000, +0.0000, +0.0000, +0.0000, +0.0000, +0.0000],
            [+0.0000, +0.0000, +0.0000, +0.0000, +0.0000, +0.0000],
            [+1.0417, +3.0649, -0.0620, -2.8697, +1.0600, +0.6217]])
        self._sensor_cal_s = np.diag([1.0647, 1.1124, 1.0, 1.0])

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

        # Point-count/conditioning/correspondence diagnostic (2026-08-02, flow-accuracy
        # investigation): per-flow-solve (t, n_kept, cond(A), solved, rel_resid,
        # px_disp_median, px_disp_std) log. n_kept/cond(A) ruled out point starvation and
        # ill-conditioning as the cause of near-zero raw-vs-GT correlation on clean
        # (det.ok=True) frames. rel_resid (||A@sol-b||/||b||, the LSTSQ fit's own
        # in-sample residual) now checks whether the tracked points at least agree with
        # EACH OTHER on a single global flow (low resid -> systematic bug elsewhere, e.g.
        # scale/sign/frame convention) or disagree with each other too (high resid ->
        # real per-point LK correspondence noise, e.g. from the marker's self-similar
        # speckle texture). px_disp_median/std are the raw per-point pixel displacements
        # feeding that fit, for a sanity check against plausible motion magnitude.
        self._flow_diag_log = []
        self._radial_diag_log = []   # TEMP DIAG 2026-08-03, see _solve_jacobian
        self._radial_diag = (np.nan, np.nan, np.nan, np.nan)

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

    def _getVirtualPts(self, pts, quat):
        """Reproject camera-frame pixels onto the virtual image plane V: a
        LEVEL frame (gravity-aligned z) that preserves the UAV's yaw heading.
        Direct port of img_data.py's _getVirtualPts (same V-frame convention
        compute_gt_signals uses for GT, so raw and GT are finally in the SAME
        frame) -- adapted to this module's self.center/self.focal instead of
        img_data.py's module-level cx,cy/fx,fy.

        MISSING FROM THIS MODULE UNTIL 2026-08-02: the raw h,w computation
        previously used un-leveled camera-frame coordinates directly, with NO
        attitude compensation at all -- comparing that against V-frame GT
        (which removes roll/pitch by construction) is comparing different
        physical quantities whenever the drone tilts, which happens
        specifically during the x/y translation phases that excite Hx/Hy.
        Root-caused as the likely dominant remaining cause of the Hx/Hy
        weak-correlation investigation -- see
        Memory/px4/project_cross_marker_pipeline_20260801.md. quat=None
        (not yet available, e.g. before FC connects) falls back to the
        un-leveled normalization so callers degrade gracefully rather than
        crash.
        """
        if quat is None:
            cx, cy = self.center
            fxx, fyy = self.focal
            return np.column_stack([(pts[:, 0] - cx) / fxx, (pts[:, 1] - cy) / fyy])

        R = Quaternion([quat.w, quat.x, quat.y, quat.z]).to_DCM()
        g = R.T @ np.array([0, 0, 1])   # world-down in body/camera frame (camera=body-FRD aligned)

        z_axis = g / np.linalg.norm(g)
        x_axis = np.cross([0, 1, 0], z_axis)
        x_axis /= np.linalg.norm(x_axis)
        y_axis = np.cross(z_axis, x_axis)
        C_R_V = np.column_stack([x_axis, y_axis, z_axis])

        cx, cy = self.center
        fxx, fyy = self.focal
        x = (pts[:, 0] - cx) / fxx
        y = (pts[:, 1] - cy) / fyy
        rays = np.column_stack([x, y, np.ones_like(x)])
        V_rays = rays @ C_R_V
        z_v = V_rays[:, 2]
        # Diagnostic (2026-08-02, V-frame regression debug): img_data.py tracks this
        # exact quantity for the exact same reason (its own comment: "z_v -> 0 or
        # negative means this corner's ray... is not representable in the
        # gravity-leveled V-frame -- grazing/behind-camera obliqueness"). A near-zero
        # z_v blows up the perspective divide into a huge or sign-flipped point,
        # exactly the kind of thing that could turn a real translational signal into
        # noise/garbage without tripping any of the existing n_kept/cond/rel_resid
        # diagnostics (those check the LSTSQ fit, not the per-point projection that
        # feeds it).
        self._z_v_log.append(float(np.min(z_v)) if len(z_v) else np.nan)
        return np.column_stack([V_rays[:, 0] / z_v, V_rays[:, 1] / z_v])

    def _solve_jacobian(self, prev_pts, curr_pts, dt, prev_quat=None, curr_quat=None):
        # Level EACH frame's points with THAT frame's own quaternion -- img_data.py's
        # comment on this exact point ("aruco_pts_0 belongs to frame-0 -> level with
        # quats[0], not quats[1]") warns that using the wrong quat leaves a residual
        # tilt proportional to angular rate, a source of yaw/rate leakage.
        prev_n = self._getVirtualPts(prev_pts, prev_quat)
        curr_n = self._getVirtualPts(curr_pts, curr_quat)
        vel = (curr_n - prev_n) / dt   # (N,2) per-point normalized velocity
        A = _fill_A(prev_n)
        b = vel.reshape(-1)
        sol, *_ = np.linalg.lstsq(A, b, rcond=None)
        cond = np.linalg.cond(A)
        # In-sample fit quality (2026-08-02, LK-correspondence-noise investigation):
        # relative residual = ||A@sol - b|| / ||b||. LOW means the tracked points
        # broadly AGREE with each other on a single global rigid/affine flow (so if
        # GT correlation is still poor, the bug is a systematic scale/sign/frame
        # issue elsewhere, not per-point tracking noise). HIGH means the points
        # themselves are inconsistent -- direct evidence of bad LK correspondences
        # (e.g. the speckle texture's self-similarity causing mismatches), which no
        # amount of recalibration downstream could fix.
        b_norm = np.linalg.norm(b)
        rel_resid = np.linalg.norm(A @ sol - b) / b_norm if b_norm > 1e-12 else np.nan
        px_disp = np.linalg.norm(curr_pts - prev_pts, axis=1)   # raw pixel displacement per point
        # TEMP DIAG (2026-08-03, Wx/Wy investigation): radial extent of the
        # normalized points actually fed into A -- max/mean |x|,|y| in prev_n.
        # w1/w2 (cols 3,4 of A) are QUADRATIC in x,y (x*y, x^2, y^2), unlike h3/w3
        # (cols 2,5) which are linear -- so w1/w2 need much larger radial spread
        # for equivalent leverage/conditioning. Logging this to check if the
        # cross-marker's actual tracked-point spread is the limiting factor.
        self._radial_diag = (float(np.max(np.abs(prev_n[:, 0]))),
                              float(np.max(np.abs(prev_n[:, 1]))),
                              float(np.mean(np.abs(prev_n[:, 0]))),
                              float(np.mean(np.abs(prev_n[:, 1]))))
        return sol, cond, rel_resid, float(np.median(px_disp)), float(np.std(px_disp))

    def _compute_hw(self, gray, mask, t, quat=None):
        """LK-track flow points from the previous frame, solve the image
        Jacobian pseudo-inverse for [h1,h2,h3,w1,w2,w3]. Two-threshold
        hysteresis (see MIN_FLOW_POINTS_SOLVE/RESAMPLE_TRIGGER comments):
        attempt the solve with however many points survived tracking, down to
        a hard floor; only DISCARD current tracking for a full fresh-sample
        when the count drops below that floor. A fresh-sample frame still
        can't report a velocity (no correspondence yet), but that no longer
        also happens on every minor, recoverable point-count dip.

        `quat` is THIS frame's attitude, used to level curr_pts (self._prev_quat
        holds the attitude that was current when self._prev_flow_pts was last
        observed, i.e. the PREVIOUS frame's quat) -- see _getVirtualPts.

        `t` is THIS frame's own timestamp (was `dt` from the caller's outer
        polling clock until 2026-08-03). BUG: process_frame's `self._last_t`
        (source of the old `dt`) advances on EVERY call, det.ok or not, but
        `self._prev_gray`/`_prev_flow_pts` only advance on SUCCESSFUL frames --
        held frozen across a detection dropout. After a gap, the old `dt` was
        just ONE poll interval while LK was tracking across the FULL dropout
        duration, inflating the computed velocity by roughly the gap length --
        a spike hitting all six solved parameters on every recovery-from-
        dropout frame. Fixed by tracking `self._prev_frame_t` (the timestamp
        actually paired with `self._prev_gray`) and computing dt from THAT,
        not the outer poll clock."""
        dilated_mask = self._dilate_mask(mask)
        dt = 0.0 if self._prev_frame_t is None else t - self._prev_frame_t

        if self._prev_gray is None or self._prev_flow_pts is None or len(self._prev_flow_pts) == 0:
            self._prev_flow_pts = self._sample_flow_points(gray, dilated_mask)
            self._prev_gray = gray
            self._prev_quat = quat
            self._prev_frame_t = t
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
        prev_quat = self._prev_quat   # the attitude prev_pts were actually observed at

        if n_kept < RESAMPLE_TRIGGER:
            # top up the pool with fresh candidates rather than discarding what's
            # still tracking -- only replace outright if the fresh yield actually
            # beats what survived (never make the pool WORSE)
            fresh = self._sample_flow_points(gray, dilated_mask)
            self._prev_flow_pts = fresh if len(fresh) > n_kept else curr_pts
        else:
            self._prev_flow_pts = curr_pts
        self._prev_gray = gray
        self._prev_quat = quat   # whatever survives to be "prev" next call was observed at THIS quat
        self._prev_frame_t = t   # ...and at THIS timestamp (2026-08-03 dt-staleness fix)

        if n_kept < MIN_FLOW_POINTS_SOLVE or dt <= 0:
            self._flow_diag_log.append((self._last_t, n_kept, np.nan, False, np.nan, np.nan, np.nan))
            return np.zeros(6), False

        sol, cond, rel_resid, px_disp_med, px_disp_std = self._solve_jacobian(
            prev_pts, curr_pts, dt, prev_quat=prev_quat, curr_quat=quat)
        self._flow_diag_log.append((self._last_t, n_kept, cond, True, rel_resid, px_disp_med, px_disp_std))
        self._radial_diag_log.append((self._last_t,) + self._radial_diag)
        return sol, True   # [h1,h2,h3,w1,w2,w3]

    def process_frame(self, img_bgr, t, quat=None):
        # self._last_t is a general "last frame seen" timestamp (used by
        # _flow_diag_log entries below); the h,w Jacobian's own dt is now
        # computed inside _compute_hw from self._prev_frame_t instead (was
        # computed here and passed down -- see _compute_hw's docstring for
        # the staleness bug that caused).
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
        # Level the centroid into the SAME V-frame compute_gt_signals' GT is in
        # (2026-08-02 fix -- was raw camera-frame, tilt-uncompensated, same gap as h,w).
        s_xy = self._getVirtualPts(np.array([[cx, cy]]), quat)[0]
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
        hw, hw_ok = self._compute_hw(gray, mask, t, quat=quat)
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

    def get_flow_diag_log(self):
        """List of (t, n_kept, cond(A), solved, rel_resid, px_disp_median, px_disp_std)
        -- one entry per flow-solve attempt (only logged on det.ok=True frames, since
        _compute_hw only runs then)."""
        return list(self._flow_diag_log)

    def get_radial_diag_log(self):
        """TEMP DIAG 2026-08-03: (t, max|x|, max|y|, mean|x|, mean|y|) of the
        normalized points fed into A, per successful solve."""
        return list(self._radial_diag_log)

    def get_z_v_log(self):
        """List of min(z_v) per _getVirtualPts call (2 per flow solve: prev_pts,
        curr_pts, plus 1 for the centroid) -- degeneracy diagnostic."""
        return list(self._z_v_log)

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
                # Use the frame's OWN capture stamp (msg.header.stamp, sim time,
                # already fetched above via getStamp()) for dt, not perf_counter()
                # at the polling callback -- img_data.py's own comment documents
                # why: perf_counter() is quantized to the ~250 Hz sim clock and
                # jitters the apparent frame interval (62/83/125 Hz instead of a
                # clean ~60), which directly corrupts vel = (curr_n-prev_n)/dt.
                # Found 2026-08-02 during the Hx/Hy flow-accuracy investigation --
                # this module had reproduced the jittered pattern fresh instead of
                # the stamp-based fix, same class of mistake as the resolution
                # transpose bug (see feedback_duplicated_math_diff_check).
                #
                # quat: live attitude for the V-frame gravity-leveling transform
                # (_getVirtualPts) -- img_data.py's IMG_PROCESSOR does this for
                # every point; this module never did until this same 2026-08-02
                # investigation traced the missing step as the likely dominant
                # remaining cause of Hx/Hy weakness (raw camera-frame flow vs
                # V-frame GT is comparing different physical quantities under
                # any real tilt).
                #
                # FIRST ATTEMPT (regressed Hx instead of fixing it) called
                # self._FC.getQuat() separately here -- measured lag against the
                # frame's own stamp was small (<=28ms) so that wasn't the whole
                # story. Root cause: img_data.py NEVER calls FC.getQuat() from its
                # own polling loop at all -- Image_Node.image_callback samples
                # quat = self._FC.getQuat() SYNCHRONOUSLY inside the same callback
                # that captures the frame, storing both into paired deques
                # (self._img_deque / self._quat_deque, appended back-to-back) --
                # see gz_subscriber.py's image_callback. getQuaternions() returns
                # that ALREADY-paired quat, tightly synced to imgs[-1] by
                # construction, not independently re-polled later from a separate
                # thread with its own scheduling jitter. Use that instead.
                quats = self._image_node.getQuaternions()
                quat = quats[-1] if quats else None
                self._perception.process_frame(imgs[-1], stamp, quat=quat)
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

    def get_flow_diag_log(self):
        return self._perception.get_flow_diag_log()

    def get_radial_diag_log(self):
        return self._perception.get_radial_diag_log()

    def get_z_v_log(self):
        return self._perception.get_z_v_log()

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
