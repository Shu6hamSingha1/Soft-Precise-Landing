# **************************************************************************
# Used virtual image feature points for optical flow calculation
# Detect nested (concentric) Aruco markers; lock one marker (largest corner spread)
# and re-lock only when it leaves both frames. h_z from the decoupled moment loom.
# **************************************************************************

"""
Code to compute real-time optical flow
    - https://ieeexplore.ieee.org/abstract/document/8753669/
    - https://docs.opencv.org/3.4/d4/dee/tutorial_optical_flow.html
"""
import os
import numpy as np
import cv2 # OpenCV library
from collections import deque
from threading import Thread 
import time
from scipy.signal import savgol_filter as sgf
import traceback
from imgstreamer import imgstream

CHECK_NUM = 80
# Camera intrinsics + camera-to-FC mount rotation: single source of truth is
# img_geometry.py (a pure-math module with NO hardware deps, so offline tools
# can import it without pulling in picamera2/qtm via imgstreamer/mocaptools).
# See img_geometry.py for full provenance comments (checkerboard calibration
# history, R_CAM_TO_BODY mount-rotation derivation).
from img_geometry import (CALIB_CX, CALIB_CY, fx, fy, f, R_CAM_TO_BODY,
                           get_virtual_pts, get_real_pts_from_v, vframe_w,
                           fill_A, scaled_quad_points, marker_principal_angle,
                           get_img_features, _quat_to_dcm,
                           quad_ill_conditioned, marker_near_fov_edge)
from planar_map import PlanarFeatureMap
from numerical_methods import extrapolate
FILTER_WIN = int(os.environ.get("IMG_FILTER_WIN", "13"))       # savgol FALLBACK window (Gazebo-aligned 13,1;
FILTER_POLYORDER = int(os.environ.get("IMG_FILTER_POLY", "1"))  # the KF is the default runtime filter)

VIDEO = False


def debayer_bayer_to_bgr(frame, display_gain=4.0):
    """Convert raw Bayer to BGR for DISPLAY only. Input: uint8 2D. Output: uint8 3-channel BGR.
    Raw-stream capture bypasses the ISP digital gain/tone-mapping stage, so the
    debayered image is genuinely dark (~20/255 mean indoors); display_gain
    brightens it for human viewing. Detection/flow code uses the un-boosted
    raw frame directly - do not apply this gain outside of display paths."""
    if frame is None or frame.ndim != 2:
        return frame
    bgr = cv2.cvtColor(frame, cv2.COLOR_BAYER_BG2BGR)
    return cv2.convertScaleAbs(bgr, alpha=display_gain, beta=0)

def build_aruco_detector():
    """Builds the (dict, params, detector) tuple shared by IMG_PROCESSOR and
    any standalone diagnostic script (e.g. live_preview.py) that wants the
    SAME detection behavior instead of a silently-drifting hardcoded copy.
    See IMG_PROCESSOR.__init__'s original inline comments (git history) for
    the full per-parameter rationale (perf profiling, perimeter-rate bounds,
    ArUco3 2026-07-23 regression) - kept there historically, not duplicated
    here to avoid a second copy going stale."""
    arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    arucoParams = cv2.aruco.DetectorParameters()
    arucoParams.adaptiveThreshWinSizeMin = int(os.environ.get("ARUCO_THRESH_WIN_MIN", "15"))
    arucoParams.adaptiveThreshWinSizeMax = int(os.environ.get("ARUCO_THRESH_WIN_MAX", "15"))
    arucoParams.adaptiveThreshWinSizeStep = int(os.environ.get("ARUCO_THRESH_WIN_STEP", "10"))
    arucoParams.minMarkerPerimeterRate = float(os.environ.get("ARUCO_MIN_PERIMETER_RATE", "0.02"))
    arucoParams.maxMarkerPerimeterRate = float(os.environ.get("ARUCO_MAX_PERIMETER_RATE", "4.0"))
    # DEFAULT FLIPPED 2026-07-26 (found via marker_principal_angle's
    # temporal-continuity fix the same session - see img_geometry.py):
    # fixing the branch-selection flip cut the raw alpha-rate noise
    # roughly in half, but a large noise floor remained (rate_std still
    # 10-100x the true gyro-yaw signal on every real recording) - the
    # underlying 2nd-moment orientation estimate itself is noisy at
    # CORNER_REFINE_NONE's achievable corner-localization precision,
    # independent of the disambiguation bug. Sub-pixel refinement directly
    # attacks that noise at its source (vs. only smoothing it downstream,
    # which the runtime feature-KF already does for CONTROL but which
    # offline/diagnostic consumers reading the raw signal - e.g.
    # derive_pi_cal.py::check_mount_rotation - don't benefit from).
    # NOT YET EMPIRICALLY VALIDATED against a live recording (existing
    # recordings didn't save raw frames, so corner detection can't be
    # retroactively re-run on them) - re-check per-frame timing budget
    # and alpha noise on the next real recording; ARUCO_CORNER_REFINE=none
    # reverts to the old fast path if this regresses frame rate too much.
    arucoParams.cornerRefinementMethod = (
        cv2.aruco.CORNER_REFINE_NONE
        if os.environ.get("ARUCO_CORNER_REFINE", "subpix") == "none"
        else cv2.aruco.CORNER_REFINE_SUBPIX)
    if hasattr(arucoParams, "useAruco3Detection"):
        try:
            arucoParams.useAruco3Detection = os.environ.get("ARUCO_USE_ARUCO3", "0") == "1"
            arucoParams.minSideLengthCanonicalImg = int(os.environ.get("ARUCO_ARUCO3_CANON_SIDE", "32"))
        except Exception:
            pass
    detector = cv2.aruco.ArucoDetector(arucoDict, arucoParams)
    return arucoDict, arucoParams, detector


class IMG_PROCESSOR(Thread):
    def __init__(self, resolution = (640, 480), capRate = 60, time_keeper=time, controller=None):
        Thread.__init__(self)
        self.daemon = True  # never block process exit if run() hangs
        self.RECORD = False

        # Image streaming setup
        self._image_node = imgstream(resolution = resolution, capRate = capRate)

        # Flight-controller handle for real quaternion/angvel telemetry
        # (imgstreamer.py is camera-only on hardware; the pose feed comes
        # from here instead of the sim's ROS /pose topic).
        self._controller = controller

        self._time = time_keeper
    
        # Image processing parameters
        self._calc_time = 1e-06
        self._fps = 0.0
        self._capRate = capRate
        # imgstream may have silently negotiated a different sensor mode than
        # requested (IMX219 only has fixed native raw modes) - use the ACTUAL
        # resolution it reports, not the constructor argument, so self.center
        # matches the real frame content.
        self._resolution = self._image_node.getResolution()
        self.focal = f
        # fx/fy/CALIB_CX/CALIB_CY were measured at 640x480 specifically - only
        # valid at that resolution. Fall back to the geometric-center
        # assumption (uncalibrated) at any other resolution and warn loudly,
        # rather than silently applying a wrong-resolution principal point.
        if tuple(self._resolution) == (640, 480):
            self.center = np.array([CALIB_CX, CALIB_CY])
        else:
            print(f"WARNING: IMG_PROCESSOR resolution={self._resolution} != the "
                  f"calibrated (640,480) - fx/fy/center are UNCALIBRATED guesses "
                  f"at this resolution.")
            self.center = np.array(self._resolution)/2     # Here radius is considered zero for the center
        self._sensor_cal_hw = np.diag([1/6, 1/6, 1/6, 1, 1, 1]) # Sensor calibration matrix
        self._sensor_cal_s = np.diag([1/12, 1/12, 1, 1]) # Sensor calibration matrix

        # ArUco detection runs on the smaller, ISP-scaled "main" stream
        # (genuinely fewer pixels than the raw stream, which is locked to the
        # sensor's native modes - see imgstreamer.py module docstring). The
        # detectMarkers adaptive-threshold/contour search is the dominant
        # per-frame cost (Hardware/docs/*timing* breakdown); shrinking the
        # search image directly cuts it. Corners are found in main-stream
        # pixel space, then scaled back up to the calibrated raw-resolution
        # pixel space (fx/fy/center above are calibrated at raw resolution)
        # before any downstream geometry touches them.
        self._main_resolution = self._image_node.getMainResolution()
        self._aruco_scale = np.array([
            self._resolution[0] / self._main_resolution[0],
            self._resolution[1] / self._main_resolution[1],
        ], dtype=np.float32)

        # PlanarFeatureMap SHADOW mode (2026-07-23, Phase 2 of the port scoped
        # earlier this session - Phase 0 feasibility benchmark + Phase 1 verbatim
        # file port both already done, see planar_map.py). Mirrors PX4_Gazebo's
        # PLANAR_MAP_SHADOW=1 default: runs alongside the existing corner/ring
        # pipeline, logs its own predictions to SEPARATE arrays, and touches
        # NOTHING getOptFlowAngVel()/getImgFeatureParam() actually return -
        # strictly observational until validated against real Pi data, same as
        # Gazebo's own staged rollout. Tracks on the 320x240 main stream (NOT
        # raw) - the Phase-0 benchmark measured ~13.6-18.4ms/frame there vs
        # ~94.6ms/frame at 640x480, and 320x240 is now the validated final
        # detection resolution (see this session's ARUCO_USE_ARUCO3 fix).
        # center/focal must be in MAIN-STREAM pixel space (CALIB_CX/CY/fx/fy
        # are calibrated at RAW resolution) - divide by _aruco_scale, the same
        # conversion already used everywhere else corner geometry crosses
        # between the two streams.
        self._planar_map_shadow = os.environ.get("PLANAR_MAP_SHADOW", "1") == "1"
        # Ported 2026-07-26 from PX4_Gazebo's MAP_REJECT_OVERFLOW_CORRECT/
        # PLASMC_MARKER_FOV_MARGIN (see img_geometry.quad_ill_conditioned /
        # marker_near_fov_edge docstrings) - gates loop_closure_correct so a
        # degenerate or near-edge decode doesn't poison the map's learned
        # geometry, even though it's a "real" decode.
        self._map_reject_overflow_correct = os.environ.get("MAP_REJECT_OVERFLOW_CORRECT", "1") == "1"
        self._marker_fov_margin = float(os.environ.get("PLASMC_MARKER_FOV_MARGIN", "40"))
        # RESCUE infrastructure (2026-07-25 port from PX4_Gazebo/src/img_data.py,
        # NOT yet wired to any consumer -- see build_aruco_detector-style
        # dedup note: ported piece-by-piece to avoid the blind-port bugs Gazebo
        # already hit and fixed (IC1 142m fly-away from a missing plausibility
        # check, a_u spike to 610997 from a hand-rolled decay, IC4 drift-off
        # from a raw per-frame threshold with no hysteresis - see
        # feedback_cbf_staleness_and_rigidity_confidence /
        # feedback_planar_map_plausibility_gate / feedback_s2_homogeneous_decay_bug
        # memories). _planarMapPredictionPlausible below is additive-only until
        # a RESCUE consumer branch is wired into the frame-pair loss path.
        # _planar_map_primary (2026-07-25, matches Gazebo's PLASMC_PLANAR_MAP_PRIMARY):
        # master switch for the RESCUE/OVERRIDE consumers, separate from _planar_map_shadow
        # (which only logs/self-corrects, never feeds anything downstream). Default ON to
        # match Gazebo's production default.
        self._planar_map_primary = os.environ.get("PLASMC_PLANAR_MAP_PRIMARY", "1") == "1"
        self._planar_map_conf_floor = float(os.environ.get("PLANAR_MAP_CONF_FLOOR", "0.5"))
        self._planar_map_rescue_fov_margin = float(os.environ.get("PLANAR_MAP_RESCUE_FOV_MARGIN", "1.5"))
        self._planar_map_rescue_size_ratio = float(os.environ.get("PLANAR_MAP_RESCUE_SIZE_RATIO", "2.0"))
        self._last_real_extent_px = None   # last GENUINE raw-decode marker span (px); never set from a rescue/extrapolated frame
        self._planar_map_gate_on = False
        self._planar_map_override_gate_on = False   # BUG FIX 2026-07-25 (same class as _planar_map_primary_pred_px): was only ever assigned inside the per-frame gated block, never initialized
        self._planar_map_gate_streak = 0
        self._planar_map_gate_on_frames = int(os.environ.get("PLANAR_MAP_GATE_ON_FRAMES", "5"))
        self._planar_map_override_gate_streak = 0
        # BUG FIX (2026-07-25, caught live via check_loop_freq.py crashing the
        # background processing thread with AttributeError): this was only ever
        # assigned inside the per-frame gated block below, never initialized here -
        # if _rescueOrCoastFeatureLog ran before the gate turned on even once,
        # reading it raised AttributeError and silently killed the capture thread.
        self._planar_map_primary_pred_px = None
        self._planar_map_rescue_active = False   # True iff THIS frame's s/alpha came from RESCUE, not raw decode - see RESCUE_ACTIVE property
        self._planar_map = None
        self._planar_map_center = (CALIB_CX / self._aruco_scale[0], CALIB_CY / self._aruco_scale[1])
        self._planar_map_focal = (fx / self._aruco_scale[0], fy / self._aruco_scale[1])
        self._planar_map_center_log = []    # get_marker_center() each frame, or None
        self._planar_map_conf_log = []      # map_confidence each frame (marker-independent, see planar_map.py)
        self._planar_map_time_log = []      # own timestamp - unconditional every frame, like ring flow
        self._cmap_raw_log = []             # (2,) map centroid in V-frame, pre-_sensor_cal_s, or (nan,nan)
        self._amap_raw_log = []             # map alpha (rad) in V-frame, pre-cal, or nan

        # ArUco marker detection setup. detectMarkers() was measured as 76% of
        # per-frame cost (Hardware/docs/power_undervoltage_investigation.md,
        # 2026-07-10: 91.9ms/2 frames synthetic, ring-LK only 24%) - this is
        # the dominant lever for img_process_freq, not the ring-flow fallback.
        # Untuned cv2.aruco.DetectorParameters() defaults sweep 3 adaptive-
        # threshold window sizes (3,13,23 px, step=10) and search the WHOLE
        # marker-perimeter range (0.03-4.0x image perimeter) every full-frame
        # call - most of that range is wasted work for a single known marker
        # at a roughly known apparent size. All knobs below are env-var
        # overridable (same convention as the rest of __init__) so a bad
        # bound can be widened without a code change if it starts missing
        # detections at an untested altitude/distance.
        self._arucoDict, self._arucoParams, self._detector = build_aruco_detector()

        # Flags and counters
        self._STAY_OPEN = True
        self.FEATURE_IS_VISIBLE = False
        self._count_check_img_feature = CHECK_NUM
        self._count_check_opt_flow = CHECK_NUM
        self._no_common_marker_warned = False

        # Per-stage timing instrumentation (diagnostic only, zero-cost unless
        # IMG_TIMING_DBG=1): accumulates wall time per named stage inside
        # _optFlowAngVel, prints a summary every _timing_report_every frames.
        self._timing_dbg = os.environ.get("IMG_TIMING_DBG", "0") == "1"
        self._timing_accum = {}
        self._timing_n = 0
        self._timing_report_every = int(os.environ.get("IMG_TIMING_EVERY", "30"))

        # ArUco ROI-crop fast path: once locked, the marker barely moves
        # frame-to-frame, so search a small crop around its last known
        # location instead of blind-scanning the full frame every call —
        # detectMarkers' adaptive-threshold/contour search (the dominant
        # per-frame cost, see Hardware/docs/*timing* breakdown) scales with
        # pixel area. Falls back to full-frame search when unlocked or after
        # too many consecutive ROI misses (fast motion / occlusion).
        # REVERTED 2026-07-28 back to the 2026-07-10-validated default of 80
        # (raw px, ~40 main-stream px). The 2026-07-24 bump to 200 (see git
        # history) was a first-pass reaction to a loop slowdown (30-46Hz ->
        # 12.7-20.3Hz) that its OWN comment predicted might not work ("if
        # roi_hits stays 0 even at this margin, the fix needs to scale with
        # measured inter-call dt instead of a fixed constant") - and indeed,
        # a fresh IMG_TIMING_DBG+ARUCO_ROI_DEBUG session at margin=200 still
        # showed roi_hits near-zero in most windows. Per the ORIGINAL
        # 2026-07-10 finding (img_process_freq_optimization.md, "Fix 2"),
        # widening the ROI margin costs more per search attempt (bigger crop)
        # without cutting the miss rate enough to compensate - so the 200px
        # bump was very likely making the underlying slowdown WORSE, not
        # better, by adding search cost on top of whatever caused the
        # original regression. That original regression (something between
        # 07-10 and 07-24 that dropped the camera-fps-bound baseline) is
        # still unexplained and worth chasing separately - this revert only
        # undoes a band-aid that never worked by its own stated criterion.
        self._roi_margin_px = int(os.environ.get("ARUCO_ROI_MARGIN_PX", "80"))
        self._roi_max_misses = int(os.environ.get("ARUCO_ROI_MAX_MISSES", "5"))
        self._roi_miss_count = 0
        self._last_locked_corners = None   # (4,2) full-image px, most recent lock
        self._roi_hits = 0                 # diagnostic counters
        self._roi_misses = 0
        self._fullframe_searches = 0
        self._roi_debug = os.environ.get("ARUCO_ROI_DEBUG", "0") == "1"   # see ROI_DEBUG comment below
        self._roi_debug_n = 0

        # KLT corner-tracking fallback (ported from PX4_Gazebo/src/img_data.py
        # 2026-07-09, MARKER_KLT_RELAX_GATE): when detectMarkers can't decode
        # the locked marker for a frame (motion blur, partial occlusion, brief
        # ID-decode glitch), track its last-known 4 corners via LK optical
        # flow instead of dropping the frame outright. SITL A/B-tested the
        # alternative (hold-last-value / drop) and it regressed badly (mean
        # xy error 0.49->1.52m, max 0.77->4.83m across 5 reps) - this is a
        # validated approach, not a guess. Gated on >= _klt_min_tracked
        # corners actually tracked (default 3/4, "relaxed gate"); a 3/4
        # result is completed via parallelogram reconstruction (ArUco's
        # fixed corner order [TL,TR,BR,BL]: opposite corners share a
        # diagonal midpoint, so missing = other-two-sum minus diagonal
        # partner). Capped at _max_lk_steps consecutive fallback frames
        # before forcing a fresh full-frame re-acquisition.
        self._klt_relax_gate = os.environ.get("MARKER_KLT_RELAX_GATE", "1") == "1"
        self._klt_min_tracked = 3 if self._klt_relax_gate else 4
        self._max_lk_steps = int(os.environ.get("MARKER_KLT_MAX_STEPS", "10"))
        self._lk_step_count = 0
        self._last_good_main_img = None        # main-stream gray image, last successful decode
        self._last_locked_corners_main = None  # (4,2) main-stream px, same frame as above
        self._klt_lk_params = dict(
            winSize=(int(os.environ.get("MARKER_KLT_LK_WIN", "21")),) * 2,
            maxLevel=int(os.environ.get("MARKER_KLT_LK_LVL", "3")),
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01),
        )
        self._klt_hits = 0   # diagnostic counters
        self._klt_misses = 0
        self._frame_used_klt = False   # set by _klt_fallback_fill, read/reset per _optFlowAngVel call
        self._frame_used_dense = False   # set by _klt_fallback_fill when dense-homography recovery filled a corner (2026-07-28)

        # DENSE-HOMOGRAPHY RECOVERY (2026-07-28 port from PX4_Gazebo, closes the
        # "third fallback tier" gap identified comparing the two pipelines' fallback
        # order: ArUco decode -> KLT corner-fallback -> dense-homography recovery ->
        # planar-map rescue / h_extrap. KLT tracks only the 4 primary corners and
        # aborts outright the moment any one of them leaves the image; dense-recovery
        # tracks MANY candidate points (RANSAC-filtered) across the same on-marker
        # region, so it survives partial/near-edge occlusion that would kill KLT's
        # strict all-4-in-bounds gate, and degrades gracefully as survivors thin out
        # instead of hard-stopping. Default-off in Gazebo too (PLASMC_DENSE_RECOVER=0,
        # unused in any Gazebo launcher/A-B script as of the port date) - ported
        # as an available, A/B-testable option, not baked-on behavior.
        self._dense_recover = os.environ.get("PLASMC_DENSE_RECOVER", "0") == "1"
        self._dense_recover_min_pts = int(os.environ.get("DENSE_RECOVER_MIN_PTS", "12"))
        self._dense_recover_ransac_px = float(os.environ.get("DENSE_RECOVER_RANSAC_PX", "3.0"))
        self._dense_recover_max_frames = int(os.environ.get("DENSE_RECOVER_MAX_FRAMES", "60"))
        self._dense_recover_min_inlier_frac = float(os.environ.get("DENSE_RECOVER_MIN_INLIER_FRAC", "0.5"))
        self._dense_pts_per_side = int(os.environ.get("IMG_DENSE_PER_SIDE", "15"))
        self._dense_soft_anchor_max_steps = int(os.environ.get("DENSE_SOFT_ANCHOR_MAX_STEPS", "2"))
        self._dense_canon_quad = None      # (4,2) main-stream px, the anchor's raw corner quad
        self._dense_canon_pts = None       # (M,2) canonical dense point layout, index-aligned with _dense_track_pts
        self._dense_track_pts = None       # (M,2) CURRENT LK-tracked positions of the canonical points
        self._dense_ref_img = None         # main-stream gray image the tracked points are relative to
        self._dense_recover_active = False # True when the last frame's recovered quad came from this path (diagnostic)
        self._dense_frames_since_anchor = 0

        # Nested-marker single-marker LOCK (Gazebo-aligned): lock one concentric
        # marker, re-lock only when it leaves both frames -> no per-frame min/max
        # ID flicker (the loom-spike root). On re-lock pick the LARGEST-corner-
        # spread common marker (biggest apparent = best-conditioned, longest-lived).
        self._locked_marker_id = None
        # Decoupled MOMENT loom for h_z (Gazebo FLOW_LOOM_DECOUPLE): the lstsq
        # divergence row is the sigma_min (weak) mode -> noise-dominated (measured
        # h_z GT-corr ~0.01). Replace it with -0.5*d(lnM)/dt from the primary
        # marker's V-frame scale M = mu20+mu02, via a causal short-window fit.
        self._loom_decouple = os.environ.get("FLOW_LOOM_DECOUPLE", "1") == "1"
        self._loom_gain = float(os.environ.get("FLOW_LOOM_GAIN", "1.0"))
        self._mtrace_hist = deque(maxlen=int(os.environ.get("FLOW_LOOM_WIN", "9")))  # (t, ln M)

        # GYRO COMPENSATION of the flow (uses the synced IMU body rate). The V-frame
        # de-rotation removes roll/pitch rotational flow, leaving only YAW; we subtract
        # that yaw rotational flow using the measured gyro (V-frame yaw) and solve
        # translation-only, so h_xy stops being the ill-conditioned sigma_min mode
        # degenerate with w_xy. FLOW_GYRO_COMP=0 reverts to the joint 6-DOF solve.
        # (For a ROTATING target the ego gyro-yaw != relative yaw; correct for a
        # translating target. A target-yaw/alpha-rate refinement is a future hook.)
        self._gyro_comp = os.environ.get("FLOW_GYRO_COMP", "1") == "1"
        # ATTITUDE-RATE GATE (2026-07-26, ported gap-fix -- Gazebo's FLOW_LAT_REDUCED
        # class of reasoning, no direct Gazebo counterpart to port verbatim since Gazebo's
        # V-frame de-rotation + reduced solve don't share this exact assumption). The
        # comp above ASSUMES V-frame de-rotation leaves ONLY yaw rotational flow --
        # true only insofar as the de-rotation is itself exact (perfect quat sync,
        # zero-lag). Residual roll/pitch rate in the de-rotated frame breaks that
        # assumption exactly when the vehicle is moving fastest (the same regime the
        # mount-rotation co-excitation recording needs) -- gate on the residual
        # roll/pitch magnitude and fall back to the joint 6-DOF solve (already the
        # gyro-unavailable branch) rather than silently trusting a violated assumption.
        self._gyro_comp_wxy_max = float(os.environ.get("GYRO_COMP_WXY_MAX", "0.2"))

        # REDUCED LATERAL SOLVE (FLOW_LAT_REDUCED, ported from Gazebo 2026-07-26, see
        # its __init__ comment ~2026-06-25 there). Drops the w_x,w_y columns (indices
        # 3,4 of _fill_A's 6-col output) from the lstsq when the gyro-comp path above
        # isn't usable this frame (gyro unavailable, or the attitude-rate gate rejected
        # it) -- a level target has near-zero w_xy (V-frame is gravity-leveled), so
        # dropping those columns turns h_xy from the ill-conditioned sigma_min mode into
        # the largest-sigma mode (Gazebo: cond 14->2). Gated on FLOW_TARGET_LEVEL
        # (tilting-target/ship-deck scenarios need the full solve instead).
        # DEFAULT OFF ON PI (unlike Gazebo's default-ON): Gazebo's own docs are explicit
        # this REQUIRES a PAIRED sensor-cal recal first -- _sensor_cal_hw's h_x/h_y rows
        # are a degeneracy-recombination tuned for the FULL 6-DOF solve; turning this on
        # without re-deriving cal against the reduced solve mis-scales h_x/h_y ~3x. Set
        # FLOW_LAT_REDUCED=1 ONLY after re-running output-cal with it enabled and
        # re-deriving _sensor_cal_hw via derive_pi_cal.py against that recording.
        self._lat_reduced = os.environ.get("FLOW_LAT_REDUCED", "0") == "1"
        self._target_level = os.environ.get("FLOW_TARGET_LEVEL", "1") == "1"

        # GYRO-COMPENSATED CENTROID-RATE OBSERVER (PLASMC_CENTROID_RATE, ported from
        # Gazebo 2026-07-26, default-ON there since 2026-07-03 for exactly this rig's
        # class of problem: a nested/small marker's LK corner-flow sits at the noise
        # floor or saturates the ~1 rad/s LK ceiling under fast motion -- see
        # project_pi_mount_rotation_validated_2026_07_26 / the centroid-rate
        # investigation for why ring-flow does NOT substitute (same LK/lstsq basis,
        # same saturation exposure). This differentiates the DECODED marker centroid
        # directly (no LK correspondence needed) through a constant-velocity Kalman
        # filter, giving an alternative h_x,h_y source that survives LK failure/
        # saturation. h_z (loom) and w_z (gyro) are reused from the existing
        # loom-decouple/gyro-comp pipeline above -- this only replaces the LATERAL
        # flow, matching Gazebo's h_x=ṡ_x+x0·h_z+y0·w_z / h_y=ṡ_y+y0·h_z-x0·w_z.
        self._centroid_rate = os.environ.get("PLASMC_CENTROID_RATE", "1") == "1"
        self._centroid_hist = deque(maxlen=int(os.environ.get("CENTROID_RATE_WIN", "9")))  # (t, x0, y0)
        self._obs_kf_x = None; self._obs_kf_y = None      # [pos, vel] states
        self._obs_kf_Px = None; self._obs_kf_Py = None    # 2x2 covariances
        self._obs_kf_t = None                             # last update stamp (for dt)
        self._obs_kf_q = float(os.environ.get("CENTROID_RATE_KF_Q", "1e-3"))   # process noise
        self._obs_kf_r = float(os.environ.get("CENTROID_RATE_KF_R", "1e-3"))   # measurement noise
        self._observer_valid = False   # reset every frame; True when the observer produced flow this frame

        # MAP-DERIVED FLOW (2026-07-28 port from PX4_Gazebo's _flowMap/_loomMapM_slot,
        # closes project_map_flow_rescue_port_pending): during a marker-loss rescue frame,
        # h previously only ever came from h_extrap (temporal extrapolation of the last
        # REAL h values) - the map's own rich tracked-point history was never turned into
        # a flow estimate, even though it already drives s/alpha rescue via the same
        # primary-slot geometry. h_x/h_y: the SAME validated 2-state CV-Kalman rate
        # estimator _obs_vel_kf uses, but with ITS OWN state (never touches
        # _obs_kf_x/_obs_kf_y) - fed the map's primary-slot V-frame centre instead of a
        # decoded centroid. h_z: the same causal d(ln M)/dt loom fit used for a real
        # decode, but on the map's primary-slot corners and ITS OWN history (never
        # touches _mtrace_hist). Gated by self._map_flow (env, default ON alongside the
        # map itself) - a rescue frame still falls back to h_extrap if this is
        # unavailable (map not ready / too few tracked corners / non-finite result).
        self._map_flow = os.environ.get("PLASMC_MAP_FLOW", "1") == "1"
        self._flowmap_kf_x = None; self._flowmap_kf_y = None
        self._flowmap_kf_Px = None; self._flowmap_kf_Py = None
        self._flowmap_kf_t = None
        self._flowmap_lnM_hist = deque(maxlen=int(os.environ.get("FLOW_LOOM_WIN", "9")))

        # Perception CHECKPOSTS (Gazebo 97bd801): per-instant outlier rejection on
        # the lateral flow, centroid, and loom. Detection reference = last RAW
        # (advances every frame -> per-instant check, NEVER latches); substitution =
        # last ACCEPTED (updated only on accept -> a spike is fully dropped, no leak).
        self._flow_dh_max = float(os.environ.get("FLOW_DH_MAX", "0.15"))     # lateral |Δh| bound (0=off)
        self._flow_prev = None; self._flow_hold = None
        self._s_ds_max = float(os.environ.get("FLOW_DS_MAX", "0.15"))        # centroid |Δs| bound (0=off)
        self._s_prev = None; self._s_hold = None
        self._loom_dlnM_max = float(os.environ.get("LOOM_DLNM_MAX", "0.04")) # loom |Δln M| bound (0=off)
        self._loom_lnM_prev = None; self._loom_hold = 0.0
        # marker_principal_angle's temporal-continuity disambiguation state
        # (2026-07-26 fix) - previous frame's disambiguated alpha, reset
        # alongside the other detection-continuity state (_flow_prev etc.)
        # whenever the lock is dropped or switches, so a stale prior can't
        # wrongly bias the first re-acquired frame's branch choice.
        self._prev_alpha = None
        # CONDITION-AWARE OUTLIER REJECTION (ported from PX4_Gazebo img_data.py,
        # 2026-06-15 there): the fusion EKF's "noise" is mostly garbage spikes
        # from the lstsq going ill-conditioned at low corner/point counts (raw
        # |h| spikes far above the typical level), which the other checkposts
        # don't catch (they bound a per-frame JUMP, not absolute solve quality).
        # Down-weights the corner measurement's EKF confidence by its lstsq
        # conditioning (cond(A) computed at solve time) instead of blanket
        # smoothing, which would also attenuate genuine fast flow. 0 = off
        # (matches Gazebo's default; _ekf_fuse_step already accepted a
        # corner_conf param, see class docstring history, but _fuse_step's
        # caller here always passed a hardcoded 1.0 - this wires it up).
        self._flow_cond_reject = float(os.environ.get("FLOW_COND_REJECT", "0"))

        # Per-axis 2-state constant-velocity Kalman filter (Gazebo-aligned), the
        # DEFAULT runtime filter replacing savgol(51). Low-lag (<~1 sample) vs
        # savgol's win/2 group delay, which at the ~6-22 Hz flow rate was seconds.
        # Flow KF on the 6 [h;w] channels; feature KF on the 4 [xc,yc,1,alpha].
        # IMG_FILTER / IMG_FEATURE_FILTER = 'savgol' restore the legacy filter.
        self._kf_q = float(os.environ.get("FLOW_KF_Q", "5.0"))   # process-noise PSD
        self._kf_r = float(os.environ.get("FLOW_KF_R", "0.1"))   # measurement noise variance
        # Ported from PX4_Gazebo 2026-07-11 (feedback_kf_frozen_during_marker_loss):
        # during a marker-loss gap, _kf_update(None,...)/_kf_feat_update(None,...)
        # now coast (predict-only, no correction) instead of not being called at
        # all (the Pi's prior behavior - the KF state simply froze until the next
        # detection). dt_unc_max uses the TRUE elapsed gap (capped here) instead
        # of the state-transition dt (capped at 0.1s), so P correctly reflects
        # staleness across a multi-frame gap and a fresh relock measurement gets
        # full Bayesian trust instead of a multi-frame catch-up ramp.
        self._kf_dt_unc_max = float(os.environ.get("KF_DT_UNC_MAX", "2.0"))
        self._kf_x = np.zeros((6, 2))                            # [value, rate] per channel
        self._kf_P = np.tile(np.eye(2) * 1.0, (6, 1, 1))
        self._kf_prev_t = None
        self._kf_initialized = False
        # Centroid-feature KF — same model, DECOUPLED (q, r): the order-1 centroid
        # needs a much smaller r than the flow's rad/s ang-vel (else over-smooth).
        self._kf_feat_q = float(os.environ.get("IMG_FEAT_KF_Q", "5.0"))
        self._kf_feat_r = float(os.environ.get("IMG_FEAT_KF_R", "0.004"))
        self._kf_feat_x = np.zeros((4, 2))
        self._kf_feat_P = np.tile(np.eye(2) * 1.0, (4, 1, 1))
        self._kf_feat_prev_t = None
        self._kf_feat_initialized = False
        self._kf_feat_last_n = 0

        # ---- Texture-free RING flow (Gazebo-aligned safety net) ----
        # Fixed V-frame ring stations (concentric about the nadir), re-projected
        # into the real image each frame and LK-tracked -> divergence/flow that
        # SURVIVES marker death (no ArUco decode needed). Radii = fractions of
        # R_max = min(W,H)/2. Fed through the SAME _getVirtualPts+_fill_A+lstsq
        # chain as the corner flow. Control consumes the CORNER flow; the ring is
        # the safety net for the terminal descent when the marker overflows the FoV.
        _Rmax = float(min(self._resolution)) / 2.0
        # 5 rings x 10 pts = 50 total LK-tracked points (was 4x25=100,
        # briefly; 3x20=60 before that; 5x30=150 originally). 2026-07-11:
        # tuned down further from the 100-point config after confirming
        # >=30Hz headroom at that setting (median 33.5Hz) - 50 gives more
        # margin below 30Hz while keeping the original 5-radius coverage
        # (0.17-0.83) for a well-conditioned divergence fit.
        _fracs = [float(x) for x in
                  os.environ.get("FLOW_RING_FRACS", "0.17,0.33,0.50,0.67,0.83").split(",")]
        _ring_radii = [fr * _Rmax for fr in _fracs]
        _ring_npts = int(os.environ.get("FLOW_RING_NPTS", "40"))   # 40 x 5 radii = 200 total ring points
        _ring_pts_V = []
        for _rr in _ring_radii:
            _ra = 2.0 * np.pi * np.arange(_ring_npts) / _ring_npts
            _ring_pts_V.append(np.c_[(_rr / f) * np.cos(_ra), (_rr / f) * np.sin(_ra)])
        self._ring_pts0_V = np.vstack(_ring_pts_V).astype(np.float32)
        # 20 (was 25, then 41 originally): LK per-point cost scales with
        # window AREA, so this cuts further off the ring-flow tracking
        # cost, at reduced tolerance for large inter-frame ring-point
        # motion - now below SITL's own marker-corner KLT fallback window
        # (MARKER_KLT_LK_WIN=21), so faster motion is more likely to lose
        # ring points than marker corners at this setting.
        self._ring_lk_params = dict(
            winSize=(int(os.environ.get("FLOW_RING_LK_WIN", "20")),) * 2,
            maxLevel=int(os.environ.get("FLOW_RING_LK_LVL", "3")),
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01),
            minEigThreshold=float(os.environ.get("FLOW_RING_LK_EIG", "1e-2")),
        )
        self._ring_on = os.environ.get("FLOW_RINGS", "1") == "1"
        # calcOpticalFlowPyrLK builds its image pyramid over the WHOLE input
        # frame every call, regardless of point count - point count has
        # already been cut 300->50 and window 41->20px (see comments above),
        # but the pyramid-build cost is image-AREA bound, not point-count
        # bound, and was never moved off the full 640x480 raw frame the way
        # ArUco detection was moved to the smaller 320x240 'main' stream
        # (self._aruco_scale). Tracking on 'main' instead is a 4x smaller
        # pyramid for the same reason that helped ArUco. Seed points are
        # generated in raw-pixel space (calibrated fx/fy/center are raw-
        # resolution only - img_geometry.py), so they're scaled DOWN by
        # self._aruco_scale before tracking and the tracked delta scaled
        # BACK UP before any _getVirtualPts/_fill_A call, mirroring
        # _detect_markers' own main->raw scale-up pattern. Env-gated so a
        # tracking-quality regression (main stream is coarser, so a real
        # sub-pixel corner shift there is a smaller sub-pixel shift in
        # absolute terms) can be reverted without a code change.
        self._ring_main_stream = os.environ.get("FLOW_RING_MAIN_STREAM", "1") == "1"
        # Ring cal: placeholder identity — derive like the corner cal once recorded.
        self._sensor_cal_ring = np.eye(6)
        # Ring KF (same _kf_step model, separate state) + latest per-frame products.
        self._kf_x_ring = np.zeros((6, 2))
        self._kf_P_ring = np.tile(np.eye(2) * 1.0, (6, 1, 1))
        self._kf_ring_prev_t = None
        self._kf_ring_initialized = False
        self._ring_v_raw = np.zeros(6)
        self._ring_pure_div = 0.0
        self._ring_moment = np.nan
        self._ring_ok = False
        self._ring_n = 0
        # Independent per-frame ring log, own timestamp - NOT aligned with the
        # corner flow/feature arrays below. Ported from PX4_Gazebo's img_data.py
        # (~line 2931, "computed EVERY frame (survives the marker death), logged"):
        # ring flow is a texture-free method independent of ArUco corner decode,
        # so it must be logged on its OWN per-frame cadence to be individually
        # calibratable during a corner dropout - gating it behind corner success
        # (the old Pi behavior) discarded it exactly when it's most needed.
        # Time/quats/fps/flow/feature stay corner-gated as before (several tools
        # positionally index them assuming equal length - do not add ring here).
        self._ring_opt_flow_raw = []   # ring V-frame [h;w], one entry per _optFlowAngVel call
        self._ring_time_log = []       # perf_counter() paired 1:1 with the row above

        # ---- Corner+ring FUSION EKF (Gazebo-aligned; moving-target capable) ----
        # State [h_tr(3), h_tv(3), w(3)] = target-relative flow, target velocity,
        # angular velocity. Corner measures [h_tr; w] (tracks the marker = target);
        # ring measures [h_tr+h_tv; w] (tracks ground) -> their difference is the
        # target velocity h_tv. Reconstructs h_tr through corner dropout (h_tv
        # persists via low process noise). Control consumes the fused [h_tr; w];
        # h_tv is a feedforward. FLOW_FUSE_RING=0 reverts to corner-only.
        self._fuse_ring = os.environ.get("FLOW_FUSE_RING", "1") == "1"
        self._ekf_x = np.zeros(9)
        self._ekf_P = np.eye(9) * 1.0
        self._ekf_prev_t = None
        self._ekf_init = False
        _I3 = np.eye(3); _Z3 = np.zeros((3, 3))
        self._H_corner = np.block([[_I3, _Z3, _Z3], [_Z3, _Z3, _I3]])    # [h_tr; w]
        self._H_ring = np.block([[_I3, _I3, _Z3], [_Z3, _Z3, _I3]])      # [h_tr+h_tv; w]
        self._H_ring_loom = np.array([[0., 0., 1., 0., 0., 1., 0., 0., 0.]])
        self._R_ring_loom = np.array([[float(os.environ.get("FLOW_R_RING_DIV", "0.5"))]])
        self._ring_div_loom_on = int(os.environ.get("RING_LOOM_PUREDIV", "1"))
        self._ring_div_cal = self._sensor_cal_ring[2, 2]                 # pure_div -> loom units
        self._H_htv_z = np.array([[0., 0., 0., 0., 0., 1., 0., 0., 0.]])
        self._R_htv_z = np.array([[float(os.environ.get("FLOW_R_HTVZ", "0.3"))]])
        self._htv_z_prior_on = int(os.environ.get("FLOW_HTVZ_PRIOR", "1"))
        self._loom_stale = 0
        self._loom_stale_max = int(os.environ.get("FLOW_LOOM_STALE_MAX", "6"))
        self._loom_decay = float(os.environ.get("FLOW_LOOM_DECAY", "0.85"))
        _rc = float(os.environ.get("FLOW_R_CORNER", "0.05"))
        _rr = float(os.environ.get("FLOW_R_RING_H", "0.5"))
        self._R_corner = np.diag([_rc] * 6)
        self._ring_loom_thresh = int(os.environ.get("RING_LOOM_NCORN", "3"))
        self._R_ring = np.diag([_rr, _rr, _rr, 1e6, 1e6, 1e6])           # ring h ok; w garbage
        _qtr = float(os.environ.get("FLOW_Q_HTR", "5.0"))
        _qtv = float(os.environ.get("FLOW_Q_HTV", "0.2"))               # rover vel ~constant
        _qw = float(os.environ.get("FLOW_Q_W", "5.0"))
        self._ekf_Q = np.diag([_qtr] * 3 + [_qtv] * 3 + [_qw] * 3)
        # Sign guard clamps the fused ego-loom <=0 (a descent can't have +ego-loom).
        # DEFAULT-OFF on the Pi: hand-cal moves the vehicle UP too (legit +loom).
        # Set FLOW_LOOM_SIGN_GUARD=1 for actual landing (descent-only).
        self._loom_sign_guard = os.environ.get("FLOW_LOOM_SIGN_GUARD", "0") == "1"
        self._target_vel_log = []       # estimated target velocity h_tv (aligned with flow)

        # Data storage
        self._time_log = []
        # Hardware capture timestamp, index-aligned with self._time_log -
        # see imgstreamer.py's getCaptureStamps() comment for why (clock-
        # source mismatch investigation, ported from PX4_Gazebo bba5c33).
        self._cap_stamp_log = []
        self._fps_log = []
        self._feature_pts = []
        self._virtual_feature_pts = []
        self._A = np.zeros((8, 6))  # 4 points → 8 rows
        self._quats = []
        self._img_feature_param = []
        self._opt_flow_ang_vel_raw = []
        # Per-frame ESTIMATOR TAG (ported from PX4_Gazebo, 2026-07-11,
        # feedback_estimator_blind_calibration): the Pi has TWO real ways to
        # produce a corner-flow sample - direct ArUco decode, or the
        # KLT-fallback-tracked corners (2026-07-11 port) - both feeding the
        # SAME _fill_A/lstsq geometry regardless of corner source (matches
        # SITL's own design choice not to need a separate cal for KLT).
        # Without this tag a calibration recording can't verify it actually
        # exercised both paths, or (in the future, if a coast/extrapolation
        # mechanism is ever added) exclude synthetic samples from the fit.
        self._opt_flow_estimator_tag = []   # 'lstsq'|'lstsq+klt'|'lstsq+dense'|'map_flow'|'coast', index-aligned with the above
        # Per-frame estimator tags for s/alpha (2026-07-25, user requirement: every
        # computational path feeding s/alpha/h must be individually identifiable so
        # calibration can be derived/validated per-path, not blindly mixed). Mirrors
        # the existing _opt_flow_estimator_tag convention. Values: 'lstsq'|'lstsq+klt'
        # (raw decode, matches _opt_flow_estimator_tag exactly since both come from
        # the same C_nP/getImgFeatures call), 'planar_map_rescue' (RESCUE path fired
        # and passed plausibility), 'coast' (no raw, no rescue - feature KF predict-
        # only). Extended _opt_flow_estimator_tag itself to also fire every loss frame
        # ('map_flow' when the map-derived flow fired this loss frame, else 'coast'
        # even though the VALUE may still be h_extrap-filled when enabled - see
        # __init__'s _h_extrap/_map_flow blocks below; ring flow already logs
        # separately/unconditionally in its own _ring_time_log) so BOTH tag arrays
        # stay dense/index-aligned across every frame, not just successful ones.
        self._s_estimator_tag = []
        self._feature_pts_fresh = False   # this frame's _feature_pts is LIVE geometry (raw or plausibility-checked rescue), not a held-over value

        # h_extrap (2026-07-28 port from PX4_Gazebo, closes project_map_flow_rescue_port_pending):
        # during marker loss the Pi zeroed h/w outright ("no h_extrap/observer port" above).
        # That's a real gap, not just an intentional-parity note - the centroid/alpha DOES get
        # rescued by the planar map (or held via kf_feat predict-only), but h/w had no equivalent,
        # so every marker-loss frame fed zero flow to the corner+ring fusion EKF and to any
        # calibration fit against those frames. Port the SAME decayed deg-1 real-only-history
        # extrapolation Gazebo already runs in production (img_data.py ~line 3178), gated by
        # PLASMC_H_EXTRAP so it can be disabled and fall back to the old hard-zero behavior.
        self._h_extrap = os.environ.get("PLASMC_H_EXTRAP", "1") == "1"
        self._h_extrap_decay_frames = int(os.environ.get("H_EXTRAP_DECAY_FRAMES", "10"))
        self._h_extrap_max = float(os.environ.get("H_EXTRAP_MAX", "5.0"))
        self._h_extrap_clip_bound = float(os.environ.get("H_EXTRAP_CLIP_BOUND", "10.0"))
        self._h_real_t = []   # REAL-only history (raw decode frames), never receives extrapolated output back
        self._h_real_v = []
        self._h_consec_misses = 0   # frames since the last raw h decode (Pi has no shared _consec_misses like Gazebo's stale-tracker)
        self._imu_angvel_raw = []   # IMU FRD body rate [fwd,right,down], synced to the flow log
        self._quat_log = []         # FC quaternion [w,x,y,z], synced to the flow log

        # Video recording
        self._video = None
        
        # start the thread
        self.start()

    # Calling destructor
    def __del__(self):
        print("Flow Streamer thread is deleted...")
    
    def close(self):
        self._STAY_OPEN = False

    def run(self):
        # The run() replaces the run() from the Thread class 
        # Since this is an inherited instance of the 
        # Thread class, you define your main process here 
        '''This function will calculate optical flow'''
        try:
            AVAILABLE = True
            self._wait_for_images()
                
            while self._STAY_OPEN:
                # tp = 10
                timer_flag = self._time.perf_counter()
                images = self._image_node.getImages()
                main_images = self._image_node.getMainImages()
                cap_stamps = self._image_node.getCaptureStamps()

                # Hardware: quaternions/angvels from flight controller
                n_imgs = len(list(images))
                if self._controller is not None and self._controller.has_quat():
                    quat = self._controller.getQuat()
                    ang_vel = self._controller.getAngVelIMU()
                    quaternions = [quat] * n_imgs
                    angvels = [ang_vel] * n_imgs
                else:
                    quaternions = [None] * n_imgs
                    angvels = [None] * n_imgs
                self._fps = self._image_node.getFPS()   
                # print(f"Image FPS: {self._fps}")                 
                
                # Check if at least 2 frames of images have been received
                if (images[0] is not None and images[1] is not None
                        and main_images[0] is not None and main_images[1] is not None):
                    if VIDEO:
                        # Resize display image
                        resized_img = cv2.resize(images[1], None, fx=1.0, fy=1.0, interpolation=cv2.INTER_AREA)
                        resized_img = debayer_bayer_to_bgr(resized_img)
                        cv2.imshow('Image Streamer', resized_img)
                        if cv2.waitKey(1) == 27:
                            self.close()

                    # MOVED 2026-07-23 (from further down, after the AVAILABLE
                    # branching below): every failure path there (`elif AVAILABLE:`
                    # and the `if not AVAILABLE:` early-exit) hits a `continue`
                    # before ever reaching that code, so RECORD only ever wrote a
                    # frame on a SUCCESSFUL corner decode - exactly the same
                    # architectural bug as the ring-flow logging fixed earlier
                    # this session (img_data.py's ring log, see __init__ comment
                    # on _ring_opt_flow_raw/_ring_time_log), just for the debug
                    # video instead. Confirmed live: a run with 0 successful
                    # decodes across 80s produced ZERO video frames - the one
                    # tool meant to help diagnose WHY detection is failing
                    # didn't run WHILE detection was failing. VIDEO's live
                    # cv2.imshow preview just above was never affected (it's
                    # already unconditional) - only the record-to-file path was.
                    if self.RECORD:
                        if self._video is None:
                            # Fixed path mismatch: this pointed at Test_Data/Test_Videos
                            # (never existed - cv2.VideoWriter does not auto-create dirs,
                            # so RECORD=True silently produced no video). The real videos
                            # dir is Test_Data/Calibration/Test_Videos.
                            self.timestamp = time.ctime().replace(':', '-')
                            os.makedirs('Test_Data/Calibration/Test_Videos', exist_ok=True)
                            self._video = cv2.VideoWriter(f'Test_Data/Calibration/Test_Videos/{self.timestamp}.mp4',
                                    cv2.VideoWriter_fourcc(*'mp4v'),
                                    self._capRate, self._resolution)
                        self._video.write(debayer_bayer_to_bgr(images[1]))

                    # Calculate the radial optical flow if it is AVAILABLE. Else the loop is restarted.
                    _opt_flow_result = self._optFlowAngVel(images, quaternions, angvels, showVideo = VIDEO, main_imgs = main_images, cap_stamps = cap_stamps)

                    # TRUE per-call loop time, captured unconditionally right after
                    # _optFlowAngVel returns - BEFORE any of the branch-specific
                    # `continue`s below. Previously this line only ran on the
                    # corner-decode-SUCCESS path (every miss `continue`d past it),
                    # so metrics()['img_process_freq'] was silently a corner-
                    # success-gated rate, not the true loop rate - it matched
                    # check_loop_freq.py's "Time (corner-success-gated)" summary
                    # stat instead of its "Ring Time (TRUE processing-loop rate,
                    # every frame)" stat, even though ring_flow/planar_map_shadow/
                    # aruco_detect all already run unconditionally every call (see
                    # the miss-path comment below). Moved here so the live metric
                    # reflects the true loop rate check_loop_freq.py's Ring Time
                    # measures.
                    self._calc_time = self._time.perf_counter() - timer_flag

                    if _opt_flow_result is AVAILABLE:
                        if not AVAILABLE:
                            time.sleep(1/100) # 100 Hz
                            continue
                        if self._count_check_opt_flow > 0:
                            self._count_check_opt_flow = 0

                    elif AVAILABLE:
                        self._count_check_opt_flow += 1
                        if self._count_check_opt_flow > CHECK_NUM:
                            print("OPTIC FLOW UNAVAILABLE...")
                            AVAILABLE = False
                            self._count_check_opt_flow = 0
                        # REMOVED 2026-07-23 (same finding/rationale as the
                        # success-path sleep removed above): this fires on EVERY
                        # corner-decode MISS, not just genuine idle waiting - by
                        # this point ring_flow (~7ms) + PlanarFeatureMap shadow
                        # (~9ms) + ArUco detect (~5ms) have already run for this
                        # frame (they execute unconditionally near the top of
                        # _optFlowAngVel, before this branch), so a miss already
                        # costs ~21ms of real work; this then added another 10ms
                        # of pure sleep on top for no benefit, same as the
                        # removed success-path sleep. Real work paces this loop
                        # on its own; a stale 100Hz cap sleep isn't needed here
                        # either.
                        continue

                    else:
                        print("OPTIC FLOW AVAILABLE NOW...")
                        AVAILABLE = True
                        self._count_check_opt_flow = 0

                    # REMOVED 2026-07-23 (found via IMG_TIMING_DBG profiling): this
                    # was an unconditional 10ms sleep on EVERY successful frame,
                    # originally intended to cap the loop at 100Hz ("# 100 Hz"). By
                    # the time this session profiled it, real per-frame work
                    # (ring flow + PlanarFeatureMap shadow + ArUco detect + dense-
                    # point flow solve, ~32ms combined) already ran well under
                    # 100Hz on its own - so this sleep was no longer capping
                    # anything, just unconditionally subtracting ~24% of the total
                    # frame budget (10ms out of a measured 42.1ms median) for no
                    # benefit. The other three time.sleep(1/100) calls in this
                    # method (on the AVAILABLE-flip-flop retry/wait paths, not this
                    # success path) are left as-is - a brief yield during a genuine
                    # wait-for-new-frame retry still serves a real purpose there.

                else:
                    print("Waiting to receive at least 2 frames")

        except KeyboardInterrupt:
            print("KeyboardInterrupt: Flow Streamer Thread\n")
        
        except RuntimeError as e:
            print(f"RuntimeError: Flow Streamer Thread: {e}\n")
            traceback.print_exc()

        except SyntaxError:
            print("SyntaxError: Flow Streamer Thread\n")

        except Exception as e:
            print(f"Unexpected error: Flow Streamer Thread: {e}\n")
        
        finally:            
            # Comment below line if image_subscriber no longer uses Python thread
            self._image_node.close()

            # When everything done, release the video write object
            if self._video:
                self._video.release()

            cv2.destroyAllWindows()
            print("Video recording stopped...")

    def _wait_for_images(self):
        """Wait until at least two frames are available."""
        print("Waiting for image streaming")
        start_time = self._time.perf_counter()
        while (any(image is None for image in list(self._image_node.getImages()))
               or any(image is None for image in list(self._image_node.getMainImages()))):
            time.sleep(1/100)
            if (self._time.perf_counter() - start_time) > 20:
                raise Exception("Unable to get image data.")
    
    def metrics(self):
        return {
            'fps': self._fps, 'img_process_freq':1/self._calc_time
        }

    def _tstage(self, t_prev, name):
        """Diagnostic-only stage timer. Returns a fresh perf_counter() and
        accumulates (now - t_prev) AND a per-stage call count under `name` —
        each stage is averaged over ITS OWN call count, not a shared
        success-only frame counter (stages before an early return, e.g.
        ring_flow/aruco_detect which run on every call including failed
        common-marker frames, must not be divided by the success-only n, or
        their per-frame cost is wildly overstated). No-op when
        _timing_dbg is False (single bool check)."""
        if not self._timing_dbg:
            return t_prev
        _now = self._time.perf_counter()
        _sum, _cnt = self._timing_accum.get(name, (0.0, 0))
        self._timing_accum[name] = (_sum + (_now - t_prev), _cnt + 1)
        return _now

    def _tmark_frame_end(self):
        """Call once per _optFlowAngVel invocation (success or fail) to drive
        the report cadence — independent of which stages a given call reached."""
        if not self._timing_dbg:
            return
        self._timing_n += 1
        if self._timing_n >= self._timing_report_every:
            _parts = ", ".join(
                f"{k}={1000*_s/_c:.2f}ms(n={_c})" for k, (_s, _c) in self._timing_accum.items())
            print(f"[TIMING] calls={self._timing_n} roi_hits={self._roi_hits} "
                  f"roi_misses={self._roi_misses} fullframe={self._fullframe_searches} "
                  f"klt_hits={self._klt_hits} klt_misses={self._klt_misses} | {_parts}")
            self._roi_hits = 0; self._roi_misses = 0; self._fullframe_searches = 0
            self._klt_hits = 0; self._klt_misses = 0
            self._timing_accum = {}
            self._timing_n = 0

    def _klt_track_corners(self, main_img_now):
        """Track self._last_locked_corners_main forward into main_img_now via
        LK optical flow (see __init__ comment for the ported-from-SITL
        rationale). Returns (4,2) main-stream px corners on success, None
        otherwise. Does NOT reset/increment _lk_step_count on its own for
        the "no attempt possible" early-outs — only on an actual tracked
        attempt — so the cap only counts real fallback usage.
        _max_lk_steps is now PURELY the confidence-decay reference (see
        _corner_conf below) -- no hard attempt ceiling here; TARGET_LOST /
        MARKER_LOSS_GRACE upstream is the real backstop (matches Gazebo
        d3dc8b0/3e96a78)."""
        try:
            seed = self._last_locked_corners_main.reshape(-1, 1, 2).astype(np.float32)
            lk_out = cv2.calcOpticalFlowPyrLK(
                self._last_good_main_img, main_img_now, seed, None, **self._klt_lk_params)
            if lk_out is None or lk_out[0] is None or lk_out[1] is None:
                self._klt_misses += 1
                return None
            lk_pts, lk_status = lk_out[0], lk_out[1]
            status_ok = np.asarray(lk_status).flatten() == 1
            n_tracked = int(np.sum(status_ok))
            if n_tracked < self._klt_min_tracked or len(status_ok) != 4:
                self._klt_misses += 1
                return None
            tracked = lk_pts.reshape(-1, 2).astype(np.float32)
            if n_tracked == 3:
                # PARALLELOGRAM COMPLETION: ArUco's fixed corner order is
                # [TL,TR,BR,BL] - opposite corners share a diagonal midpoint
                # (c0+c2 == c1+c3), so the missing corner = sum of the other
                # two minus its diagonal partner. Good approximation over one
                # small inter-frame step; the in-bounds check below is the
                # safety net if it's wrong.
                miss = int(np.where(~status_ok)[0][0])
                partner = (miss + 2) % 4
                others = [k for k in range(4) if k not in (miss, partner)]
                tracked[miss] = tracked[others[0]] + tracked[others[1]] - tracked[partner]
            mh, mw = main_img_now.shape[:2]
            in_bounds = (np.all(tracked[:, 0] >= 0) and np.all(tracked[:, 0] < mw)
                         and np.all(tracked[:, 1] >= 0) and np.all(tracked[:, 1] < mh))
            if not in_bounds:
                # Marker has fully left the frame - stop chaining, force a
                # fresh full-frame re-acquisition next time it reappears.
                self._klt_misses += 1
                self._lk_step_count = 0
                self._last_good_main_img = None
                self._last_locked_corners_main = None
                return None
            self._lk_step_count += 1
            self._klt_hits += 1
            return tracked
        except Exception:
            self._klt_misses += 1
            return None

    def _dense_recover_anchor(self, corners_main, img_main):
        """Re-anchor the dense-homography-recovery canonical state on a CLEAN
        decode of the locked marker (main-stream px, same space as
        _last_locked_corners_main/_last_good_main_img): canonical dense layout +
        canonical quad = THIS frame's geometry (so canonical==tracked right now),
        ref image = img_main. Ported from PX4_Gazebo's _dense_recover_anchor,
        adapted to the Pi's main-stream/locked-marker convention (Gazebo anchors
        once per call on frame0; the Pi anchors wherever it updates its own KLT
        seed, i.e. every clean raw decode of the locked marker)."""
        if not self._dense_recover:
            return
        self._dense_canon_quad = np.asarray(corners_main, np.float32).reshape(-1, 2).copy()
        self._dense_canon_pts = self._scaled_quad_points(self._dense_canon_quad,
                                                           per_side=self._dense_pts_per_side)
        self._dense_track_pts = self._dense_canon_pts.copy()
        self._dense_ref_img = img_main.copy()
        self._dense_recover_active = False
        self._dense_frames_since_anchor = 0

    def _dense_recover_step(self, img_main):
        """Advance the dense-homography-recovery tracking one step (_dense_ref_img
        -> img_main), independent of decode/KLT status - called every frame so it
        survives partial dropouts the strict corner gate alone would fail. Drops
        out-of-bounds/lost survivors; canonical<->tracked correspondence stays
        index-aligned (both shrink together). Ported from PX4_Gazebo's
        _dense_recover_step."""
        if not self._dense_recover or self._dense_track_pts is None or self._dense_ref_img is None:
            return
        try:
            g0 = self._dense_ref_img if self._dense_ref_img.ndim == 2 else cv2.cvtColor(self._dense_ref_img, cv2.COLOR_BGR2GRAY)
            g1 = img_main if img_main.ndim == 2 else cv2.cvtColor(img_main, cv2.COLOR_BGR2GRAY)
            p1, st, _ = cv2.calcOpticalFlowPyrLK(g0, g1, self._dense_track_pts, None, **self._klt_lk_params)
            st = np.asarray(st).flatten().astype(bool)
            _ih, _iw = img_main.shape[:2]
            _p1f = p1.reshape(-1, 2)
            _in = ((_p1f[:, 0] >= 0) & (_p1f[:, 0] < _iw) & (_p1f[:, 1] >= 0) & (_p1f[:, 1] < _ih))
            keep = st & _in
            self._dense_canon_pts = self._dense_canon_pts[keep]
            self._dense_track_pts = _p1f[keep].astype(np.float32)
            self._dense_ref_img = img_main.copy()
            self._dense_frames_since_anchor += 1
        except Exception:
            self._dense_canon_pts = None
            self._dense_track_pts = None
            self._dense_ref_img = None

    def _dense_recover_quad(self):
        """Recover the FULL primary-corner quad (main-stream px) via a RANSAC
        homography fit from the surviving canonical dense points to their tracked
        positions, then map the canonical quad's 4 corners through it. Returns
        (4,2) float32 recovered corners, or None if too few survivors / a
        degenerate fit / too stale since the last real anchor. No depth/scale
        needed - pure 2D pixel homography. Ported from PX4_Gazebo's
        _dense_recover_quad."""
        if (not self._dense_recover or self._dense_track_pts is None
                or len(self._dense_track_pts) < self._dense_recover_min_pts
                or self._dense_canon_quad is None
                or self._dense_frames_since_anchor > self._dense_recover_max_frames):
            return None
        try:
            Hmat, mask = cv2.findHomography(self._dense_canon_pts, self._dense_track_pts,
                                             cv2.RANSAC, self._dense_recover_ransac_px)
            if Hmat is None or mask is None or int(mask.sum()) < self._dense_recover_min_pts:
                return None
            _inlier_frac = float(mask.sum()) / max(1, len(self._dense_track_pts))
            if _inlier_frac < self._dense_recover_min_inlier_frac:
                return None
            recovered = cv2.perspectiveTransform(
                self._dense_canon_quad.reshape(-1, 1, 2), Hmat).reshape(-1, 2).astype(np.float32)
            _ih, _iw = self._dense_ref_img.shape[:2]
            if not (np.all(recovered[:, 0] >= -_iw) and np.all(recovered[:, 0] < 2 * _iw)
                    and np.all(recovered[:, 1] >= -_ih) and np.all(recovered[:, 1] < 2 * _ih)):
                return None
            self._dense_recover_active = True
            return recovered
        except Exception:
            return None

    def _klt_fallback_fill(self, main_imgs, results):
        """For each image where the currently-locked marker wasn't decoded
        by ArUco, try to recover its corners via _klt_track_corners and
        splice a synthetic detection entry into that image's results (same
        shape a real cv2.aruco.detectMarkers hit would produce), so the
        existing common-marker/lock logic downstream picks it up with no
        other changes needed. Resets _lk_step_count once neither image
        needed the fallback (a clean direct-decode frame).

        DENSE-HOMOGRAPHY RECOVERY (2026-07-28 port from PX4_Gazebo) runs as a
        THIRD tier inside this same per-image loop, after KLT: it advances every
        call regardless of outcome (_dense_recover_step) and is only asked for a
        recovered quad when KLT itself couldn't fill this image. Default-off
        (self._dense_recover)."""
        if self._dense_recover:
            for img in main_imgs:
                self._dense_recover_step(img)
        if self._locked_marker_id is None:
            return results
        used_klt = False
        used_dense = False
        new_results = list(results)
        for i, img in enumerate(main_imgs):
            corners, ids, rejected = new_results[i]
            ids_flat = np.asarray(ids).flatten() if ids is not None and len(ids) else np.array([])
            if self._locked_marker_id in ids_flat:
                continue
            tracked = None
            if self._last_good_main_img is not None and self._last_locked_corners_main is not None:
                tracked = self._klt_track_corners(img)
            if tracked is not None:
                used_klt = True
            else:
                tracked = self._dense_recover_quad()
                if tracked is not None:
                    used_dense = True
                    if os.environ.get("DENSE_RECOVER_DBG", "0") == "1":
                        print("[dr] t%.3f RECOVERED quad from %d dense survivors" % (
                            self._time.perf_counter(), len(self._dense_track_pts)), flush=True)
            if tracked is None:
                continue
            new_corner = tracked.reshape(1, 4, 2).astype(np.float32)
            new_id = np.array([[self._locked_marker_id]], dtype=np.int32)
            if ids is not None and len(ids_flat):
                merged_corners = tuple(corners) + (new_corner,)
                merged_ids = np.vstack([np.asarray(ids), new_id])
            else:
                merged_corners = (new_corner,)
                merged_ids = new_id
            new_results[i] = (merged_corners, merged_ids, rejected)
        if used_klt:
            self._frame_used_klt = True
        else:
            self._lk_step_count = 0
        self._frame_used_dense = used_dense
        self._dense_recover_active = used_dense
        return new_results

    def _detect_markers(self, main_imgs):
        """ArUco detection on the smaller 'main' stream (see __init__
        comment: genuinely fewer pixels than the raw stream, cutting the
        dominant per-frame cost), with an ROI-crop fast path around the
        last locked marker location. Returns the same
        [(corners, ids, rejected), ...] structure as a direct detectMarkers()
        call, with corners scaled back up to the calibrated RAW-resolution
        pixel space regardless of which path (ROI or full-frame) ran, so
        every downstream consumer is unaffected by which stream this used."""
        _mw, _mh = self._main_resolution
        main_results = None   # (corners, ids, rejected) x2, in MAIN-stream px space
        if self._last_locked_corners is not None:
            # last_locked_corners is stored in RAW pixel space (what every
            # other consumer expects) - convert to main-stream space for the
            # crop. _roi_margin_px is specified in RAW pixels for an
            # intuitive env-var meaning; convert to main-stream pixels too.
            _c_main = self._last_locked_corners / self._aruco_scale
            _margin_main = max(1, int(self._roi_margin_px / self._aruco_scale.mean()))
            x0 = max(0, int(_c_main[:, 0].min()) - _margin_main)
            y0 = max(0, int(_c_main[:, 1].min()) - _margin_main)
            x1 = min(_mw, int(_c_main[:, 0].max()) + _margin_main)
            y1 = min(_mh, int(_c_main[:, 1].max()) + _margin_main)
            if x1 > x0 and y1 > y0:
                crops = [img[y0:y1, x0:x1] for img in main_imgs]
                if self._detector is not None:
                    roi_results = [self._detector.detectMarkers(c) for c in crops]
                else:
                    roi_results = [cv2.aruco.detectMarkers(c, self._arucoDict, parameters=self._arucoParams)
                                    for c in crops]
                if all(r[0] for r in roi_results):
                    _off = np.array([x0, y0], dtype=np.float32)
                    main_results = [(tuple(corner + _off for corner in corners), ids, rejected)
                                     for corners, ids, rejected in roi_results]
                    self._roi_miss_count = 0
                    self._roi_hits += 1
                else:
                    self._roi_miss_count += 1
                    self._roi_misses += 1
                    # ONE-OFF DIAGNOSTIC 2026-07-24 (ARUCO_ROI_DEBUG=1): margin
                    # bump 80->200 raw px did not move roi_hits off 0 despite
                    # full-frame decoding ~94% of calls - investigating whether
                    # this is minMarkerPerimeterRate/maxMarkerPerimeterRate
                    # rejecting the marker inside its own tight crop (those
                    # rates are relative to the INPUT image's max dimension,
                    # so a crop sized close to the marker's own bbox makes the
                    # marker look "too large" relative to the crop even though
                    # it's normal-sized relative to the full frame) vs. the
                    # crop containing nothing decodable at all. Capped print
                    # count so a long recording doesn't spam.
                    if self._roi_debug and self._roi_debug_n < 20:
                        self._roi_debug_n += 1
                        for _i, (c, (corners, ids, rejected)) in enumerate(zip(crops, roi_results)):
                            _ch, _cw = c.shape[:2]
                            _n_rej = len(rejected) if rejected is not None else 0
                            print(f"[ROI_DEBUG #{self._roi_debug_n}] img{_i}: crop={_cw}x{_ch} "
                                  f"(main-space), corners_found={corners is not None and len(corners) > 0}, "
                                  f"rejected_candidates={_n_rej}, "
                                  f"maxPerimRate={self._arucoParams.maxMarkerPerimeterRate}, "
                                  f"minPerimRate={self._arucoParams.minMarkerPerimeterRate}")
                    if self._roi_miss_count >= self._roi_max_misses:
                        self._last_locked_corners = None   # force re-acquire via full-frame
        if main_results is None:
            # Full-frame search on the main stream: first lock, ROI miss this
            # call, or too many consecutive ROI misses (cleared above).
            self._fullframe_searches += 1
            if self._detector is not None:
                main_results = [self._detector.detectMarkers(img) for img in list(main_imgs)]
            else:
                main_results = [cv2.aruco.detectMarkers(img, self._arucoDict, parameters=self._arucoParams)
                                 for img in main_imgs]
        # KLT corner-tracking fallback (main-stream space, matching where
        # _last_locked_corners_main lives) - fills in the locked marker for
        # any image ArUco couldn't decode it in, before the final scale-up.
        main_results = self._klt_fallback_fill(main_imgs, main_results)
        return [(tuple(corner * self._aruco_scale for corner in corners), ids, rejected)
                for corners, ids, rejected in main_results]

    def _optFlowAngVel(self, imgs, quats, angvels = None, showVideo = False, main_imgs = None, cap_stamps = None):
        # This function will return True if the optical flow is AVAILABLE and calculate the optical flow. Else, it will return False.
        # Return type is a Boolean
        # imgs/main_imgs are the imgstreamer's live deques (maxlen=2), still
        # being appended to by the background capture thread - snapshot to
        # stable lists ONCE here so nothing downstream (multiple iteration
        # sites in _detect_markers/_klt_fallback_fill) races the capture
        # thread (confirmed 2026-07-11: "RuntimeError: deque mutated during
        # iteration" from iterating main_imgs directly mid-capture).
        imgs = list(imgs)
        main_imgs = list(main_imgs) if main_imgs is not None else None
        _tt = self._time.perf_counter()
        self._frame_used_klt = False   # reset; set by _klt_fallback_fill (via _detect_markers) below
        self._frame_used_dense = False   # reset; set by _klt_fallback_fill (via _detect_markers) below
        # Texture-free RING flow (safety net) — runs EVERY frame, independent of
        # ArUco, so it survives marker death. Steps its own KF; latest raw stored.
        if self._ring_on:
            _vvr, _pdiv, _nr, _rmom = self._compute_ring_flow(imgs, quats, main_imgs)
            self._ring_v_raw = _vvr; self._ring_pure_div = _pdiv; self._ring_moment = _rmom
            self._ring_ok = bool(_nr > 0 and np.all(np.isfinite(_vvr)) and np.any(_vvr != 0))
            self._ring_n = int(_nr)
            (self._kf_x_ring, self._kf_P_ring, self._kf_ring_prev_t,
             self._kf_ring_initialized) = self._kf_step(
                self._kf_x_ring, self._kf_P_ring, self._kf_ring_prev_t,
                self._kf_ring_initialized, _vvr, self._time.perf_counter(),
                self._kf_q, self._kf_r, dt_unc_max=self._kf_dt_unc_max)
            # Log EVERY frame here (own timestamp), independent of whether the
            # corner marker decodes below - see __init__ comment on
            # _ring_opt_flow_raw/_ring_time_log for why this moved out of the
            # old corner-success-gated append.
            self._ring_opt_flow_raw.append(_vvr.copy())
            self._ring_time_log.append(self._time.perf_counter())
        _tt = self._tstage(_tt, "1_ring_flow")

        # PlanarFeatureMap SHADOW update - runs EVERY frame, independent of
        # corner decode below (see __init__ comment). Wrapped defensively:
        # this is strictly observational, so any internal exception here must
        # never propagate and disturb the real corner/ring pipeline above.
        if (self._planar_map_shadow or self._planar_map_primary) and main_imgs is not None and main_imgs[1] is not None:
            try:
                _pm_img = main_imgs[1] if main_imgs[1].ndim == 2 else cv2.cvtColor(main_imgs[1], cv2.COLOR_BGR2GRAY)
                _pm_quat = quats[1] if (quats is not None and len(quats) > 1) else None
                _pm_quat_R = _quat_to_dcm(_pm_quat) if _pm_quat is not None else None
                if self._planar_map is None:
                    self._planar_map = PlanarFeatureMap(center=self._planar_map_center, focal=self._planar_map_focal)
                    self._planar_map.bootstrap(_pm_img, quat_R=_pm_quat_R)
                else:
                    self._planar_map.update(_pm_img, quat_R=_pm_quat_R)
                    # RESCUE/OVERRIDE gate-streak tracking (2026-07-25 port from Gazebo,
                    # see feedback_cbf_staleness_and_rigidity_confidence memory). Runs on
                    # EVERY frame (map confidence is a property of the map itself, not
                    # gated on THIS frame's decode outcome) - not inside a decode-success
                    # branch. Marker-aware self.confidence (not map_confidence) for BOTH
                    # gates by default (RESCUE_GATE_MARKER_AWARE=1, matches Gazebo): using
                    # the marker-independent map_confidence for rescue alone was found
                    # structurally blind to the marker DEFORMING (quad foreshortening at
                    # a tilt-grazing terminal approach) rather than being occluded -
                    # confidence collapses via rigidity, map_confidence does not.
                    # primary_zero_corners carve-out (ported from Gazebo 2a5c21f): when the
                    # PRIMARY slot dropped all 4 corners THIS frame there is literally no shape
                    # information to distrust -- fall back to map_confidence instead of letting
                    # marker-aware confidence (which zeroes on a bare decode failure) stall the
                    # rescue-gate streak. A deforming-but-present marker (1-4 corners) still gates
                    # on the marker-aware confidence, preserving the original 2026-07-19 fix.
                    _mkr_aware = os.environ.get("RESCUE_GATE_MARKER_AWARE", "1") == "1"
                    _rescue_conf = (self._planar_map.map_confidence
                                     if (_mkr_aware and self._planar_map.primary_zero_corners)
                                     else self._planar_map.confidence if _mkr_aware
                                     else self._planar_map.map_confidence)
                    _good = _rescue_conf >= self._planar_map_conf_floor
                    if _good:
                        self._planar_map_gate_streak += 1
                    else:
                        self._planar_map_gate_streak = 0
                        self._planar_map_gate_on = False
                    if not self._planar_map_gate_on and self._planar_map_gate_streak >= self._planar_map_gate_on_frames:
                        self._planar_map_gate_on = True
                    # OVERRIDE gate: separate, stricter gate for replacing an already-
                    # successful raw decode - always marker-aware self.confidence.
                    _good_override = self._planar_map.confidence >= self._planar_map_conf_floor
                    if _good_override:
                        self._planar_map_override_gate_streak += 1
                    else:
                        self._planar_map_override_gate_streak = 0
                        self._planar_map_override_gate_on = False
                    if (not self._planar_map_override_gate_on
                            and self._planar_map_override_gate_streak >= self._planar_map_gate_on_frames):
                        self._planar_map_override_gate_on = True
                    if os.environ.get("PLANAR_MAP_DBG", "0") == "1" and int(getattr(self, "_pm_dbg_ctr", 0)) % 30 == 0:
                        print(f"[planar_map gate] confidence={self._planar_map.confidence:.3f} "
                              f"map_confidence={self._planar_map.map_confidence:.3f} "
                              f"rigid_ok={self._planar_map.marker_rigid_ok} "
                              f"gate_streak={self._planar_map_gate_streak} gate_on={self._planar_map_gate_on} "
                              f"override_streak={self._planar_map_override_gate_streak} "
                              f"override_on={self._planar_map_override_gate_on} "
                              f"pred_cached={self._planar_map_primary_pred_px is not None}", flush=True)
                    self._pm_dbg_ctr = int(getattr(self, "_pm_dbg_ctr", 0)) + 1
                    if self._planar_map_primary and self._planar_map_gate_on:
                        # SPACE FIX (2026-07-25): PlanarFeatureMap operates in MAIN-STREAM
                        # pixel space (_planar_map_center/_focal = raw/_aruco_scale, per the
                        # __init__ comment), but _planarMapPredictionPlausible/_getVirtualPts
                        # both expect RAW-resolution space (self.center/self.focal) - same
                        # convention _detect_markers uses when scaling corners back up
                        # ("corners scaled back up to the calibrated RAW-resolution pixel
                        # space"). Without this, every rescue attempt was silently comparing
                        # a main-stream-scaled prediction against a raw-space extent - a
                        # systematic ~_aruco_scale-factor size mismatch that rejected nearly
                        # every rescue as "implausible size" regardless of the map's actual
                        # estimate quality (caught live via PLANAR_MAP_DBG=1 -- every single
                        # attempt in a test session was rejected on size until this fix).
                        _pm_pred_main = self._planar_map.get_marker_frame_pts()
                        self._planar_map_primary_pred_px = (
                            _pm_pred_main * self._aruco_scale if _pm_pred_main is not None else None)
                _pm_center = self._planar_map.get_marker_center()
                self._planar_map_center_log.append(_pm_center.copy() if _pm_center is not None else None)
                self._planar_map_conf_log.append(self._planar_map.map_confidence)
                self._planar_map_time_log.append(self._time.perf_counter())

                # MAP-SOURCED centroid/alpha in V-FRAME, pre-_sensor_cal_s
                # (2026-07-27, Pi counterpart of Gazebo's 'Centroid Map Raw' /
                # 'Alpha Map Raw', see feedback_map_cal_validation_gap).
                #
                # ARCHITECTURE NOTE: this is NOT a line-by-line port. Gazebo has
                # dedicated _centroidMap()/_alphaMap() consumers that OVERRIDE
                # _img_feature_param[0..1]/[3] and whose raw values it logs. The Pi
                # has no such consumer -- its map feeds the pipeline by substituting
                # CORNERS (_planar_map_primary_pred_px), and centroid/alpha are then
                # computed downstream by the normal path. So the equivalent
                # INDEPENDENT map observable has to be built here from the map's own
                # geometry rather than read off an override.
                #
                # Space chain matches the rescue path exactly: the map works in
                # MAIN-STREAM pixels, so scale by _aruco_scale to RAW pixels before
                # _getVirtualPts (the same SPACE FIX as the prediction above -- getting
                # this wrong silently rescales everything by _aruco_scale).
                # get_marker_center_native() is preferred over get_marker_center():
                # per its docstring it is the projective diagonal-intersection (a true
                # projective invariant) rather than the corner mean, which perspective
                # shifts off the real centre -- and it survives marker overflow, which
                # is exactly when the map is worth having.
                _cm_raw, _am_raw = (np.nan, np.nan), np.nan
                try:
                    _q1 = quats[1] if (quats is not None and len(quats) > 1
                                       and quats[1] is not None) else None
                    if _q1 is not None:
                        _c_main = self._planar_map.get_marker_center_native()
                        if _c_main is None:
                            _c_main = _pm_center
                        if _c_main is not None:
                            _c_v = self._getVirtualPts(
                                np.asarray([_c_main], np.float32) * self._aruco_scale, _q1)
                            _cm_raw = (float(_c_v[0][0]), float(_c_v[0][1]))
                        _p_main = self._planar_map.get_marker_frame_pts()
                        if _p_main is not None and len(_p_main) >= 4:
                            _p_v = self._getVirtualPts(
                                np.asarray(_p_main, np.float32) * self._aruco_scale, _q1)
                            _am_raw = float(marker_principal_angle(_p_v))
                except Exception:
                    pass       # never let the map cross-check touch the real pipeline
                self._cmap_raw_log.append(_cm_raw)
                self._amap_raw_log.append(_am_raw)
            except Exception:
                pass   # shadow mode: never let an internal error touch the real pipeline
        _tt = self._tstage(_tt, "1b_planar_map_shadow")

        # Detect markers on the smaller 'main' stream (ROI-crop fast path
        # when locked). main_imgs is required - _detect_markers always
        # scales its output up from main-stream to raw-resolution pixel
        # space, so silently falling back to `imgs` (already raw-space)
        # here would double up the scale factor and corrupt every corner.
        if main_imgs is None:
            raise ValueError("_optFlowAngVel requires main_imgs (the ISP-scaled "
                              "'main' stream) for ArUco detection - see imgstreamer.getMainImages().")
        results = self._detect_markers(main_imgs)
        _tt = self._tstage(_tt, "2_aruco_detect")

        # Check if both detections were successful
        if all(r[0] for r in results):# Ensure that a common marker ID is detected in both the frame
            # SINGLE-MARKER LOCK (Gazebo-aligned nested-marker handling): a flow
            # sample needs the SAME marker in both frames. Keep the locked marker
            # while it is visible in both; else re-lock onto the largest-corner-
            # spread common marker. Concentric markers share a centre, so a switch
            # never moves the centroid; the loom history is cleared on re-lock so
            # the -0.5 d(lnM)/dt slope never spans the big<->small size step.
            ids0 = np.asarray(results[0][1]).flatten()
            ids1 = np.asarray(results[1][1]).flatten()
            common = np.intersect1d(ids0, ids1)
            if len(common) == 0:
                if not self._no_common_marker_warned:
                    print("No common marker in both frames. Skipping optical flow calculation.")
                    self._no_common_marker_warned = True
                self._rescueOrCoastFeatureLog(quats, angvels, cap_stamps)
                self._kf_update(None, self._time.perf_counter())
                self._fuse_step(None, False, 0, self._time.perf_counter())   # ring-only
                self._tmark_frame_end()
                return False
            self._no_common_marker_warned = False
            if self._locked_marker_id is None or self._locked_marker_id not in common:
                spreads = [float(np.std(results[0][0][int(np.where(ids0 == m)[0][0])].reshape(-1, 2)))
                           for m in common]
                new_lock = int(common[int(np.argmax(spreads))])
                if new_lock != self._locked_marker_id:
                    self._mtrace_hist.clear()   # size step at a switch -> don't fit across it
                    # Corner-flow + centroid-feature KF reset on a primary-
                    # marker-ID switch (ported from PX4_Gazebo, 2026-07-11):
                    # without this, a nested big<->small marker handover hands
                    # both KFs a discontinuous geometry step (the state/rate
                    # estimate from the OLD marker's corner scale/position is
                    # not valid for the NEW one) that they'd otherwise try to
                    # smoothly track through instead of re-bootstrapping.
                    # Directly relevant on the Pi given the nested-marker setup.
                    self._kf_initialized = False
                    self._kf_feat_initialized = False
                    self._prev_alpha = None   # discontinuous geometry step - re-bootstrap disambiguation
                    self._centroid_hist.clear()   # centroid-rate observer: same discontinuity, don't diff across a marker-ID switch
                    self._obs_kf_x = None; self._obs_kf_t = None   # force _obs_vel_kf to re-bootstrap
                self._locked_marker_id = new_lock
            mid = self._locked_marker_id
            i0 = int(np.where(ids0 == mid)[0][0])
            i1 = int(np.where(ids1 == mid)[0][0])

            # Locked-marker corners (4,2) in each frame.
            C_nP = [results[0][0][i0].reshape(-1, 2), results[1][0][i1].reshape(-1, 2)]
            # Last-known REAL marker span (px) - for the RESCUE plausibility check's
            # size test (_planarMapPredictionPlausible). Updated ONLY on genuine raw
            # decode, never from a rescued/extrapolated frame (ported from Gazebo).
            self._last_real_extent_px = float(
                max(C_nP[0][:, 0].max() - C_nP[0][:, 0].min(),
                    C_nP[0][:, 1].max() - C_nP[0][:, 1].min()))
            self._last_locked_corners = np.asarray(C_nP[1], dtype=np.float32)   # for next call's ROI
            self._last_good_main_img = main_imgs[1].copy()   # for next call's KLT fallback
            self._last_locked_corners_main = self._last_locked_corners / self._aruco_scale
            if self._dense_recover:
                # Re-anchor on a genuine fresh decode of the locked marker (never on a
                # KLT-filled or dense-recovered synthetic corner - that would compound
                # drift into the canonical layout). Also SOFT-anchor after a short KLT
                # chain (still close enough to trustworthy) - ported from Gazebo's
                # _dense_soft_anchor_max_steps logic.
                if not self._frame_used_klt and not self._dense_recover_active:
                    self._dense_recover_anchor(self._last_locked_corners_main, main_imgs[1])
                elif self._frame_used_klt and self._lk_step_count <= self._dense_soft_anchor_max_steps:
                    self._dense_recover_anchor(self._last_locked_corners_main, main_imgs[1])
            # FIXED 2026-07-26 (user-identified bug): loop-closure correction used
            # to be gated on self._planar_map_shadow alone - meaning disabling
            # shadow mode while self._planar_map_primary stayed on (its default)
            # left the map PREDICTING/RESCUING corner positions but NEVER
            # CORRECTED against real decodes - a drifting, uncorrected map still
            # feeding rescue substitutions, worse than either flag being fully
            # consistent. A real ArUco decode should ALWAYS re-anchor the map when
            # available (SLAM loop-closure principle), independent of which
            # consumer (shadow logging or primary rescue) is currently active -
            # this is what keeps ANY future consumer's predictions trustworthy.
            # Same defensive wrap as the unconditional update block.
            # ILL-CONDITIONED/NEAR-EDGE REJECTION (ported 2026-07-26 from
            # PX4_Gazebo's MAP_REJECT_OVERFLOW_CORRECT gate): don't loop-close
            # from a decode whose quad is a degenerate sliver (tilt-grazing/
            # overflow) or sits near the frame boundary - that near-singular or
            # edge-degraded observation would poison the map's gauge/homography
            # rather than improve it, even though ArUco itself reported success.
            if self._planar_map is not None:
                _reject_correct = False
                if self._map_reject_overflow_correct:
                    _mh, _mw = main_imgs[1].shape[:2]
                    _near_edge = marker_near_fov_edge(self._last_locked_corners_main,
                                                       (_mh, _mw), self._marker_fov_margin)
                    _reject_correct = _near_edge or quad_ill_conditioned(self._last_locked_corners_main)
                if not _reject_correct:
                    try:
                        self._planar_map.loop_closure_correct(self._last_locked_corners_main, marker_id=mid)
                    except Exception:
                        pass
            _tt = self._tstage(_tt, "3_lock_and_extract")

            V_nP_norm = [self._getVirtualPts(p, q) for p, q in zip(C_nP, quats)]

            # CENTROID-RATE OBSERVER: track the frame-0 decoded centroid (V-frame,
            # already roll/pitch-leveled) through the CV-Kalman filter every frame a
            # real decode exists, independent of whether LK/lstsq succeeds below --
            # see __init__ comment for why this is worth having on this rig.
            self._observer_valid = False
            if self._centroid_rate:
                _x0 = float(V_nP_norm[0][:, 0].mean()); _y0 = float(V_nP_norm[0][:, 1].mean())
                _to = self._time.perf_counter()
                self._centroid_hist.append((_to, _x0, _y0))
                if len(self._centroid_hist) >= 3:
                    _ta = np.array([c[0] for c in self._centroid_hist])
                    if (_ta.max() - _ta.min()) > 1e-4:
                        self._obs_sdx, self._obs_sdy = self._obs_vel_kf(_x0, _y0, _to)
                        self._obs_x0, self._obs_y0 = _x0, _y0
                        self._observer_valid = True
            # PLANAR-MAP OVERRIDE (2026-07-25 port from Gazebo, PLASMC_PLANAR_MAP_PRIMARY
            # default-on): override V_nP_norm[1] -- the SINGLE shared source both the
            # centroid (s/alpha, via _getImgFeatures below) and the moment-loom (_vp,
            # below) read from -- with PlanarFeatureMap's primary-slot prediction,
            # whenever the map is confident+rigid this frame. Overriding HERE (once)
            # rather than at each consumer keeps centroid and loom internally consistent
            # (both always agree on which corner source produced this frame's estimate)
            # and fixes small<->big decode-flicker at its root: the raw per-frame ArUco
            # decode this array would otherwise carry jumps instantaneously on a flicker;
            # the map's KLT+homography-tracked slot does not, since it only gets
            # LOOP-CLOSURE corrected (not overwritten) by whichever marker decodes this
            # frame. Does NOT touch flow_pts_0/1 or the frame-pair dense-flow lstsq
            # (V_dense, below) -- PlanarFeatureMap predicts a single frame's position,
            # not a frame-pair. Falls back to the untouched raw V_nP_norm[1] whenever the
            # map isn't ready/confident (identical behavior to before this port).
            #
            # STRICTER gate than RESCUE: this REPLACES an already-successful raw decode,
            # so requires _planar_map_override_gate_on (marker-aware self.confidence), not
            # just _planar_map_gate_on (map_confidence, which only gates whether
            # _planar_map_primary_pred_px was computed at all). Using the permissive
            # rescue gate here would let the override fire on perfectly healthy
            # raw-decode frames too, replacing accurate fresh corners with a
            # possibly-drifted map estimate.
            #
            # PLAUSIBILITY CHECK: reuses the SAME _planarMapPredictionPlausible check the
            # rescue path uses (position AND size, not just "the map claims confidence")
            # -- without it, a drifted map projection could silently replace a genuinely
            # good raw decode.
            _ov_fired = False
            if (self._planar_map_primary and self._planar_map_override_gate_on
                    and self._planar_map_primary_pred_px is not None):
                _ov_ok, _pm_v1, _ov_why = self._planarMapPredictionPlausible(
                    self._planar_map_primary_pred_px, quats[1])
                if _ov_ok and len(_pm_v1) == len(V_nP_norm[1]):
                    V_nP_norm[1] = _pm_v1
                    _ov_fired = True
                elif os.environ.get("PLANAR_MAP_DBG", "0") == "1":
                    print(f"[planar_map override] REJECTED implausible {_ov_why} "
                          f"-- keeping raw ArUco V_nP_norm[1]", flush=True)
            _tt = self._tstage(_tt, "4_getVirtualPts_corners")

            # Shows image with optical flow
            if showVideo:
                self._showOptFlow(imgs[1], C_nP, V_nP_norm)

            # Compute optical flow on DENSE scaled-quadrilateral points (~180/frame)
            # for a well-conditioned lstsq. The bare 4-corner solve is noise-
            # dominated (per-frame raw std ~1.4 vs true signal ~0.05); the dense
            # spread + sqrt(N) averaging cut that. Points are generated identically
            # from each frame's corners, so they correspond index-wise; de-rotate
            # both sets to the gravity-leveled V-frame before the solve.
            dense_px = [self._scaled_quad_points(p) for p in C_nP]
            V_dense  = [self._getVirtualPts(dp, q) for dp, q in zip(dense_px, quats)]
            _tt = self._tstage(_tt, "5_dense_pts_and_getVirtualPts")

            A = self._fill_A(V_dense[1])
            Y = np.reshape(V_dense[1] - V_dense[0], (-1,)) * self._fps
            _tt = self._tstage(_tt, "6_fill_A")

            # GYRO COMPENSATION: subtract the yaw rotational flow (the only rotation
            # left after V-frame de-rotation) using the measured gyro, then solve
            # translation-only (well-conditioned h_xy). w_z from the gyro (direct).
            # Falls back to the joint 6-DOF solve when the gyro is unavailable.
            _wz = None
            if self._gyro_comp and angvels is not None and len(angvels) > 1 and angvels[1] is not None:
                try:
                    _av = angvels[1]
                    _wv = self._vframe_w([_av.forward_rad_s, _av.right_rad_s, _av.down_rad_s], quats[1])
                    # Residual roll/pitch rate after V-frame de-rotation should be ~0 if
                    # the yaw-only assumption holds; if it doesn't (fast/co-excited
                    # motion), don't trust the yaw-only compensation -- fall back to the
                    # joint 6-DOF solve below instead of injecting an unmodeled roll/pitch
                    # rotational-flow error into the lateral translation estimate.
                    if max(abs(float(_wv[0])), abs(float(_wv[1]))) <= self._gyro_comp_wxy_max:
                        _wz = float(_wv[2])
                except Exception:
                    _wz = None
            if _wz is not None:
                _Yc = Y - A[:, 5] * _wz                       # remove yaw rotational flow
                _Asolve = A[:, 0:3]
                _h = np.linalg.lstsq(_Asolve, _Yc, rcond=1e-3)[0]
                B_v = np.array([_h[0], _h[1], _h[2], 0.0, 0.0, _wz])
            elif self._lat_reduced and self._target_level:
                # REDUCED 4-DOF fallback (gyro-comp above wasn't usable this frame):
                # drop w_x,w_y (cols 3,4), keep [h_x,h_y,h_z,w_z] -- see __init__
                # comment. w_xy set to 0 in B_v (level-target assumption).
                _Asolve = np.delete(A, [3, 4], axis=1)
                _h = np.linalg.lstsq(_Asolve, Y, rcond=1e-3)[0]
                B_v = np.array([_h[0], _h[1], _h[2], 0.0, 0.0, _h[3]])
            else:
                _Asolve = A
                B_v = np.linalg.lstsq(_Asolve, Y, rcond=1e-3)[0]
            _tt = self._tstage(_tt, "7_gyro_comp_and_lstsq")

            # Corner EKF confidence: KLT-fallback depth (corners degrade the
            # deeper the LK track runs, per _max_lk_steps) AND solve
            # conditioning (see FLOW_COND_REJECT above). 1.0 = full trust
            # (both terms are no-ops at their defaults, matching Gazebo).
            _corner_conf = 1.0
            if self._frame_used_klt:
                _corner_conf = max(0.05, 1.0 - self._lk_step_count / max(self._max_lk_steps, 1))
            elif self._frame_used_dense:
                # Dense-homography recovery has no per-attempt step counter (RANSAC
                # inlier fraction is its own within-frame quality gate) - decay on
                # frames-since-anchor instead, same floor as the KLT case.
                _corner_conf = max(0.05, 1.0 - self._dense_frames_since_anchor / max(self._dense_recover_max_frames, 1))
            if self._flow_cond_reject > 0:
                try:
                    _cond = float(np.linalg.cond(_Asolve))
                    if np.isfinite(_cond) and _cond > 0:
                        _corner_conf *= min(1.0, self._flow_cond_reject / _cond)
                except np.linalg.LinAlgError:
                    pass

            # DECOUPLED MOMENT loom: override the weak-mode lstsq h_z with
            # -0.5*d(ln M)/dt from the primary marker's V-frame scale
            # M = mu20+mu02 (trace of the de-rotated corner scatter, proportional
            # to 1/Z^2), via a causal short-window linear fit. Scale-free; no size
            # normalization needed (concentric markers -> history cleared on re-lock).
            if self._loom_decouple:
                _vp = V_nP_norm[1]
                _ctr = _vp.mean(axis=0)
                _M = float(np.mean(np.sum((_vp - _ctr) ** 2, axis=1)))
                if _M > 1e-12 and np.isfinite(_M):
                    _lnM = np.log(_M)
                    # d(lnM)/dt checkpost: a residual switch/marker-leaving transient
                    # steps ln M -> HOLD the loom, don't fit across it.
                    _dlnM = abs(_lnM - self._loom_lnM_prev) if self._loom_lnM_prev is not None else 0.0
                    self._loom_lnM_prev = _lnM
                    if self._loom_dlnM_max > 0 and _dlnM > self._loom_dlnM_max:
                        self._mtrace_hist.clear()
                        B_v[2] = float(self._loom_hold)          # reject: hold last-good
                    else:
                        self._mtrace_hist.append((self._time.perf_counter(), _lnM))
                        if len(self._mtrace_hist) >= 3:
                            _ta = np.array([h[0] for h in self._mtrace_hist])
                            _la = np.array([h[1] for h in self._mtrace_hist])
                            if (_ta.max() - _ta.min()) > 1e-4:
                                _slope = np.polyfit(_ta - _ta[0], _la, 1)[0]
                                B_v[2] = float(np.clip(-0.5 * _slope * self._loom_gain, -10.0, 10.0))
                                self._loom_hold = B_v[2]         # remember last-good
                        else:
                            B_v[2] = float(self._loom_hold)      # during rebuild, hold last-good

            # CENTROID-RATE OBSERVER injection: replace the lateral flow (B_v[0],
            # B_v[1]) with the KF-filtered centroid rate + gyro/loom coupling, when
            # available. h_z (B_v[2], just finalized above) and w_z (B_v[5], from the
            # gyro-comp block) are REUSED, not re-derived -- this only supplies the
            # lateral term, matching Gazebo's h_x=sdx+x0*h_z+y0*wz / h_y=sdy+y0*h_z-x0*wz.
            if self._centroid_rate and self._observer_valid:
                _obs_hz = float(B_v[2]); _obs_wz = float(B_v[5])
                B_v[0] = self._obs_sdx + self._obs_x0 * _obs_hz + self._obs_y0 * _obs_wz
                B_v[1] = self._obs_sdy + self._obs_y0 * _obs_hz - self._obs_x0 * _obs_wz
            _tt = self._tstage(_tt, "8_loom_decouple")

            # Lateral-flow checkpost: reject a per-frame |Δh| spike (σ_min LK garbage),
            # emit last-good. Rate-based -> scale/depth-free.
            if self._flow_dh_max > 0:
                _v_raw = np.array([B_v[0], B_v[1]], dtype=float)   # RAW, before any reject
                if self._flow_hold is None:
                    self._flow_hold = _v_raw.copy()
                for _i in (0, 1):
                    if self._flow_prev is not None and abs(_v_raw[_i] - self._flow_prev[_i]) > self._flow_dh_max:
                        B_v[_i] = float(self._flow_hold[_i])       # reject: last accepted
                    else:
                        self._flow_hold[_i] = _v_raw[_i]           # accept: advance
                self._flow_prev = _v_raw                           # detection ref = RAW (no latch)
            _tt = self._tstage(_tt, "9_flow_checkpost")

            self._opt_flow_ang_vel_raw.append(B_v)
            _decode_tag = ('lstsq+klt' if self._frame_used_klt
                           else 'lstsq+dense' if self._frame_used_dense else 'lstsq')
            self._opt_flow_estimator_tag.append(_decode_tag)
            if self._h_extrap:
                self._h_real_t.append(self._time.perf_counter())
                self._h_real_v.append(B_v.copy())
                if len(self._h_real_t) > 32:
                    self._h_real_t = self._h_real_t[-32:]
                    self._h_real_v = self._h_real_v[-32:]
            self._h_consec_misses = 0
            # 2-state constant-velocity KF on the raw [h;w] (low-lag; getOptFlowAngVel
            # reads _kf_x). Stepped once per fresh frame.
            self._kf_update(B_v, self._time.perf_counter())
            _tt = self._tstage(_tt, "10_kf_update")

            # Feature params (centroid + yaw alpha) from the 4 primary corners.
            self._getImgFeatures(V_nP_norm[1])
            self._s_estimator_tag.append(
                _decode_tag + ('+override' if _ov_fired else ''))
            self._feature_pts_fresh = True
            self._planar_map_rescue_active = False   # raw tier succeeded this frame, not a rescue
            _tt = self._tstage(_tt, "11_getImgFeatures")

            # Centroid checkpost: reject a per-frame |Δs| jump (detect/LK glitch), hold last-good.
            if self._s_ds_max > 0 and self._img_feature_param:
                _s = self._img_feature_param[-1]
                _s_raw = np.array([_s[0], _s[1]], dtype=float)     # RAW, before any reject
                if self._s_hold is None:
                    self._s_hold = _s_raw.copy()
                for _j in (0, 1):
                    if self._s_prev is not None and abs(_s_raw[_j] - self._s_prev[_j]) > self._s_ds_max:
                        _s[_j] = float(self._s_hold[_j])           # reject: last accepted
                    else:
                        self._s_hold[_j] = _s_raw[_j]              # accept: advance
                self._s_prev = _s_raw                              # detection ref = RAW (no latch)
            _tt = self._tstage(_tt, "12_centroid_checkpost")

            if not self.FEATURE_IS_VISIBLE:
                print("LANDING PAD VISIBLE NOW...")
                self.FEATURE_IS_VISIBLE  = True 
            if self._count_check_img_feature > 0:
                self._count_check_img_feature = 0

            # Store the feature points
            self._time_log.append(self._time.perf_counter())
            self._cap_stamp_log.append(
                cap_stamps[1] if cap_stamps is not None and len(cap_stamps) > 1 else None)
            self._feature_pts.append(C_nP)
            self._virtual_feature_pts.append(V_nP_norm)
            self._quats.append(quats)
            self._fps_log.append(self._fps)
            # Ring flow is now logged unconditionally where it's computed
            # (own _ring_time_log, see __init__ comment) - no longer appended
            # here to avoid double-logging every corner-success frame.

            # Synced attitude + gyro (aligned with the flow) for gyro compensation.
            _av = angvels[1] if (angvels is not None and len(angvels) > 1 and angvels[1] is not None) else None
            self._imu_angvel_raw.append(
                np.array([_av.forward_rad_s, _av.right_rad_s, _av.down_rad_s])
                if _av is not None else np.full(3, np.nan))
            _q1 = quats[1] if (quats is not None and len(quats) > 1 and quats[1] is not None) else None
            self._quat_log.append(
                np.array([_q1.w, _q1.x, _q1.y, _q1.z]) if _q1 is not None else np.full(4, np.nan))

            # Corner+ring fusion EKF — corner measurement this frame (marker decoded).
            _corner_cal = self._sensor_cal_hw @ B_v
            if self._loom_decouple:
                _corner_cal[2] = B_v[2]        # moment loom already physical, bypass cal
            self._fuse_step(_corner_cal, True, 4, self._time.perf_counter(), corner_conf=_corner_conf)
            self._target_vel_log.append(
                self._ekf_x[3:6].copy() if self._ekf_init else np.zeros(3))
            _tt = self._tstage(_tt, "13_logging_and_fuse_step")
            self._tmark_frame_end()

            return True

        elif self.FEATURE_IS_VISIBLE:
            self._count_check_img_feature +=1
            if self._count_check_img_feature > CHECK_NUM:
                print("LANDING PAD NOT VISIBLE...")
                self._count_check_img_feature = 0
                # Swap flag value to initiate necessary action to get the image feature in the field of view of the camera.
                self.FEATURE_IS_VISIBLE  = False
                # Marker truly lost: drop the lock + loom history so re-acquisition
                # re-locks by spread and the loom slope never spans the gap. Reset the
                # checkpost detection refs so the first re-acquired frame isn't rejected.
                self._locked_marker_id = None
                self._last_locked_corners = None   # drop ROI too, force full-frame re-acquire
                self._last_good_main_img = None    # drop KLT anchor too
                self._last_locked_corners_main = None
                self._lk_step_count = 0
                self._mtrace_hist.clear()
                self._flow_prev = None; self._s_prev = None; self._loom_lnM_prev = None
                self._prev_alpha = None
            # KF coast (predict-only, no correction) rather than not calling the
            # KF at all — the prior behavior left the state frozen until the
            # next real detection. See __init__ comment / feedback_kf_frozen_
            # during_marker_loss (ported from PX4_Gazebo, 2026-07-11).
            self._kf_update(None, self._time.perf_counter())
            self._rescueOrCoastFeatureLog(quats, angvels, cap_stamps)
            self._fuse_step(None, False, 0, self._time.perf_counter())   # ring-only (dropout)
            self._tmark_frame_end()
            return False

        # No marker ever locked, or lock fully dropped after the CHECK_NUM
        # debounce above - keep coasting the KFs too (harmless/no-op if they
        # were never initialized; _kf_step's z=None early-out handles that).
        self._kf_update(None, self._time.perf_counter())
        self._rescueOrCoastFeatureLog(quats, angvels, cap_stamps)
        self._fuse_step(None, False, 0, self._time.perf_counter())       # ring-only (no marker)
        self._tmark_frame_end()
        return False
    
    def _rescueOrCoastFeatureLog(self, quats, angvels, cap_stamps):
        """Ported from PX4_Gazebo/src/img_data.py's RESCUE branch (2026-07-25), called
        from every marker-loss site (no common marker, debounced truly-lost, never-locked
        tail) instead of leaving s/alpha/h/_feature_pts sparse (only appended on raw
        decode success, per user requirement that every frame's computation path be
        individually taggable for calibration). This method does NOT touch the corner+ring
        fusion EKF or _kf_update(h) - callers still do that themselves exactly as before
        (matching Gazebo: the rescue only grounds s/alpha/_feature_pts, never h_z/target
        velocity, which stay on ring-flow-only during a raw-decode dropout).

        Precedence per frame: PlanarFeatureMap RESCUE (if gated-on, available, and
        plausibility-checked) > feature-KF predict-only coast (self._kf_feat_x[:,0],
        matches Gazebo's kf_predict fix - never a hand-rolled decay, see
        feedback_s2_homogeneous_decay_bug memory)."""
        _t = self._time.perf_counter()
        self._kf_feat_update(None, _t)   # predict-only step; RESCUE below may follow with a correct-step
        _s_coast = self._kf_feat_x[:, 0].copy()
        _s_coast = np.nan_to_num(np.asarray(_s_coast), nan=0.0, posinf=5.0, neginf=-5.0)
        _s_coast = np.clip(_s_coast, -5.0, 5.0)
        _s_coast[2] = 1.0   # homogeneous constant - never a tracked/decaying KF channel, see feedback_s2_homogeneous_decay_bug
        if self._img_feature_param:
            _hold_a = float(self._img_feature_param[-1][3])
            _s_coast[3] = float(np.arctan2(np.sin(_hold_a), np.cos(_hold_a)))   # alpha is WRAPPED - hold, don't let KF predict push it past +-pi

        _pm_rescue = (self._planar_map_primary and self._planar_map_gate_on
                      and self._planar_map_primary_pred_px is not None)
        _s_final = _s_coast
        _pm_v1_checked = None
        if _pm_rescue:
            _ok, _pm_v1_checked, _why = self._planarMapPredictionPlausible(
                self._planar_map_primary_pred_px, quats[1])
            if _ok:
                _s_final = get_img_features(_pm_v1_checked)
            else:
                _pm_rescue = False
                if os.environ.get("PLANAR_MAP_DBG", "0") == "1":
                    print(f"[planar_map rescue] REJECTED implausible {_why} -- falling back to coast", flush=True)

        self._img_feature_param.append(_s_final)
        self._s_estimator_tag.append('planar_map_rescue' if _pm_rescue else 'coast')
        self._planar_map_rescue_active = _pm_rescue
        if _pm_rescue:
            self._kf_feat_update(_s_final, _t)   # genuine correct-step against the rescue's fresh geometric estimate

        # h: MAP-DERIVED flow preferred (2026-07-28 port from PX4_Gazebo's _flowMap,
        # closes project_map_flow_rescue_port_pending) - a fresh geometric estimate from
        # the map's own tracked points, rather than a temporal extrapolation of stale
        # real values. Falls back to the h_extrap decayed deg-1 extrapolation (against
        # REAL-ONLY history, never self-referencing) when the map isn't available/ready,
        # then to hard zero. Ring flow still runs unconditionally (own _ring_time_log)
        # and is unaffected either way.
        self._h_consec_misses += 1
        _q1 = quats[1] if (quats is not None and len(quats) > 1 and quats[1] is not None) else None
        _map_h = self._flowMap(_q1, self._time.perf_counter()) if self._map_flow else None
        if _map_h is not None:
            _h_out = _map_h
            self._opt_flow_estimator_tag.append('map_flow')
        else:
            if self._h_extrap and len(self._h_real_t) >= 5:
                _rt = np.asarray(self._h_real_t[-8:])
                _rv = np.asarray(self._h_real_v[-8:])
                _valid = np.all(np.abs(_rv) < self._h_extrap_clip_bound - 1e-3, axis=1)
                if int(_valid.sum()) >= 5:
                    _rt, _rv = _rt[_valid], _rv[_valid]
                    _fit = extrapolate(_rt, _rv, n=min(4, len(_rt) - 1), deg=1, default_shape=6)
                    _fit = np.nan_to_num(np.asarray(_fit), nan=0.0, posinf=self._h_extrap_max,
                                          neginf=-self._h_extrap_max)
                else:
                    _fit = self._h_real_v[-1].copy() if self._h_real_v else np.zeros(6)
                _decay = max(0.0, 1.0 - self._h_consec_misses / max(1, self._h_extrap_decay_frames))
                _h_out = np.clip(_fit * _decay, -self._h_extrap_max, self._h_extrap_max)
            else:
                _h_out = np.zeros(6)
            self._opt_flow_estimator_tag.append('coast')
        self._opt_flow_ang_vel_raw.append(_h_out)

        # _feature_pts/_virtual_feature_pts: rescued geometry when available (so
        # MARKER_EXTENT_PX/CBF see LIVE map-grounded geometry, not frozen corners),
        # else hold last value. FEATURE_PTS_FRESH distinguishes the two for consumers.
        self._feature_pts_fresh = bool(_pm_rescue)
        if _pm_rescue:
            _prev_curr = (self._feature_pts[-1][1] if self._feature_pts
                          else np.asarray(self._planar_map_primary_pred_px, np.float32))
            self._feature_pts.append(np.array(
                [_prev_curr, np.asarray(self._planar_map_primary_pred_px, np.float32)]))
            self._virtual_feature_pts.append(np.array(
                [self._virtual_feature_pts[-1][1] if self._virtual_feature_pts
                 else _pm_v1_checked, _pm_v1_checked]))
        else:
            self._feature_pts.append(self._feature_pts[-1] if self._feature_pts else np.zeros((2, 4, 2)))
            self._virtual_feature_pts.append(self._virtual_feature_pts[-1] if self._virtual_feature_pts else np.zeros((2, 4, 2)))

        # General per-frame bookkeeping, kept dense/index-aligned with the success branch.
        self._time_log.append(_t)
        self._cap_stamp_log.append(cap_stamps[1] if cap_stamps is not None and len(cap_stamps) > 1 else None)
        self._quats.append(quats)
        self._fps_log.append(self._fps)
        self._imu_angvel_raw.append(np.full(3, np.nan))   # marker lost: no synced IMU pairing
        _qL = quats[1] if (quats is not None and len(quats) > 1 and quats[1] is not None) else None
        self._quat_log.append(np.array([_qL.w, _qL.x, _qL.y, _qL.z]) if _qL is not None else np.full(4, np.nan))
        return _pm_rescue

    def _fill_A(self, centered_pts):
        """See img_geometry.fill_A (single source of truth)."""
        return fill_A(centered_pts)

    def _obs_vel_kf(self, x0, y0, t):
        """Constant-velocity Kalman filter on the decoded centroid -> smoothed lateral
        velocity (sdx, sdy). Ported verbatim from Gazebo img_data.py (pure math, no
        calibration data) -- replaces raw polyfit differentiation (jitter-amplifying).
        Resets on init or a stale/large time gap (won't differentiate across a
        marker-loss gap)."""
        if self._obs_kf_x is None or self._obs_kf_t is None:
            self._obs_kf_x = np.array([x0, 0.0]); self._obs_kf_y = np.array([y0, 0.0])
            self._obs_kf_Px = np.eye(2); self._obs_kf_Py = np.eye(2); self._obs_kf_t = t
            return 0.0, 0.0
        dt = t - self._obs_kf_t
        if dt <= 1e-4 or dt > 0.5:                 # bad/large gap -> reset, don't diff across it
            self._obs_kf_x = np.array([x0, 0.0]); self._obs_kf_y = np.array([y0, 0.0])
            self._obs_kf_Px = np.eye(2); self._obs_kf_Py = np.eye(2); self._obs_kf_t = t
            return 0.0, 0.0
        F = np.array([[1.0, dt], [0.0, 1.0]])
        Q = self._obs_kf_q * np.array([[dt ** 3 / 3, dt ** 2 / 2], [dt ** 2 / 2, dt]])
        r = self._obs_kf_r
        out = []
        for st, P, z in ((self._obs_kf_x, self._obs_kf_Px, x0), (self._obs_kf_y, self._obs_kf_Py, y0)):
            st[:] = F @ st
            P[:] = F @ P @ F.T + Q
            S = P[0, 0] + r; K = P[:, 0] / S
            st[:] = st + K * (z - st[0])
            P[:] = P - np.outer(K, P[0, :])
            out.append(float(st[1]))
        self._obs_kf_t = t
        return out[0], out[1]

    def _loomMapM_primary(self, quat):
        """Map-driven h_z helper (2026-07-28 port from PX4_Gazebo's _loomMapM_slot,
        adapted to the Pi's single-primary-rescue design - no small/big HANDOVER_LATCHED
        slot selection here, always the primary slot, matching how s/alpha rescue already
        reads it). Same causal d(ln M)/dt fit as the real-decode loom (_mtrace_hist), on
        its OWN separate history (self._flowmap_lnM_hist) so it never interferes with
        the primary loom's own state."""
        if self._planar_map is None or not getattr(self._planar_map, 'initialized', False) or quat is None:
            return None
        try:
            px = self._planar_map.get_marker_frame_pts()   # slot=None -> primary
            if px is None or len(px) != 4:
                return None
            Vp = self._getVirtualPts(np.asarray(px, np.float32) * self._aruco_scale, quat)
            if Vp is None or len(Vp) != 4:
                return None
            ctr = Vp[:, :2].mean(axis=0)
            M = float(np.mean(np.sum((Vp[:, :2] - ctr) ** 2, axis=1)))
            if not (M > 1e-12 and np.isfinite(M)):
                return None
            t = self._time.perf_counter()
            lnM = float(np.log(M))
            self._flowmap_lnM_hist.append((t, lnM))
            if len(self._flowmap_lnM_hist) < 3:
                return 0.0
            ts = np.array([c[0] for c in self._flowmap_lnM_hist])
            if ts.max() - ts.min() < 1e-4:
                return 0.0
            t0 = ts - ts[0]
            slope = np.polyfit(t0, [c[1] for c in self._flowmap_lnM_hist], 1)[0]
            return float(np.clip(-0.5 * slope * self._loom_gain, -10.0, 10.0))
        except Exception:
            return None

    def _flowMap(self, quat, t):
        """Map-driven flow [h_x, h_y, h_z] (2026-07-28 port from PX4_Gazebo's _flowMap),
        for use during a marker-loss RESCUE frame - preferred over h_extrap when
        available, since it's a fresh geometric estimate from the map's own tracked
        points rather than a temporal extrapolation of stale real values. h_x/h_y: the
        SAME 2-state CV-Kalman rate estimator _obs_vel_kf uses, own state
        (_flowmap_kf_*), fed the map's primary-slot V-frame centre. h_z: _loomMapM_primary.
        None -> caller falls back to h_extrap (or zero) as before."""
        pm = self._planar_map
        if pm is None or not getattr(pm, 'initialized', False) or quat is None:
            return None
        try:
            px = pm.get_marker_center_native()   # slot=None -> primary
            if px is None:
                return None
            Vp = self._getVirtualPts(np.asarray([px], np.float32) * self._aruco_scale, quat)
            if Vp is None or len(Vp) != 1:
                return None
            x0, y0 = float(Vp[0, 0]), float(Vp[0, 1])
            if self._flowmap_kf_x is None or self._flowmap_kf_t is None:
                self._flowmap_kf_x = np.array([x0, 0.0]); self._flowmap_kf_y = np.array([y0, 0.0])
                self._flowmap_kf_Px = np.eye(2); self._flowmap_kf_Py = np.eye(2); self._flowmap_kf_t = t
                hx, hy = 0.0, 0.0
            else:
                dt = t - self._flowmap_kf_t
                if dt <= 1e-4 or dt > 0.5:
                    self._flowmap_kf_x = np.array([x0, 0.0]); self._flowmap_kf_y = np.array([y0, 0.0])
                    self._flowmap_kf_Px = np.eye(2); self._flowmap_kf_Py = np.eye(2); self._flowmap_kf_t = t
                    hx, hy = 0.0, 0.0
                else:
                    F = np.array([[1.0, dt], [0.0, 1.0]])
                    Q = self._obs_kf_q * np.array([[dt ** 3 / 3, dt ** 2 / 2], [dt ** 2 / 2, dt]])
                    r = self._obs_kf_r
                    out = []
                    for st, P, z in ((self._flowmap_kf_x, self._flowmap_kf_Px, x0),
                                     (self._flowmap_kf_y, self._flowmap_kf_Py, y0)):
                        st[:] = F @ st
                        P[:] = F @ P @ F.T + Q
                        S = P[0, 0] + r; K = P[:, 0] / S
                        st[:] = st + K * (z - st[0])
                        P[:] = P - np.outer(K, P[0, :])
                        out.append(float(st[1]))
                    hx, hy = out[0], out[1]
                    self._flowmap_kf_t = t
            hz = self._loomMapM_primary(quat)
            if hz is None:
                return None
            if not all(np.isfinite(v) for v in (hx, hy, hz)):
                return None
            return np.array([hx, hy, hz, 0.0, 0.0, 0.0], dtype=float)   # w_x/w_y/w_z: not map-derived, zero (matches CTRL_ZERO_WXY level-target convention)
        except Exception:
            return None

    def _scaled_quad_points(self, corners, scales=(1.0, 2.0/3.0, 1.0/3.0), per_side=15):
        """See img_geometry.scaled_quad_points (single source of truth)."""
        return scaled_quad_points(corners, scales=scales, per_side=per_side)

    def _marker_principal_angle(self, pts):
        """See img_geometry.marker_principal_angle (single source of truth)."""
        return marker_principal_angle(pts)

    def _getImgFeatures(self, pts):
        """See img_geometry.get_img_features (single source of truth) - this
        wrapper preserves the append-to-self._img_feature_param side effect
        callers rely on (see e.g. the centroid checkpost, which reads
        self._img_feature_param[-1] right after calling this), and also
        tracks self._prev_alpha across calls for marker_principal_angle's
        temporal-continuity disambiguation (2026-07-26 fix) - reset alongside
        the lock-switch/marker-loss state elsewhere in this class."""
        feat = get_img_features(pts, prev_angle=self._prev_alpha)
        self._prev_alpha = float(feat[3])
        self._img_feature_param.append(feat)

    def _getVirtualPts(self, pts, quat):
        """See img_geometry.get_virtual_pts (single source of truth)."""
        return get_virtual_pts(pts, quat, center=self.center)

    def _planarMapPredictionPlausible(self, pm_px, quat):
        """Ported from PX4_Gazebo/src/img_data.py (2026-07-25) - PHYSICAL-PLAUSIBILITY
        check for a PlanarFeatureMap camera-pixel corner prediction. Shared by any
        future RESCUE/OVERRIDE consumer - never duplicate this logic (Gazebo's override
        path once shipped with no check at all and let a drifted map projection replace
        a good raw decode undetected).

        Two INDEPENDENT failure modes, both checked:
        (1) POSITION: map confidence says nothing about whether THIS SPECIFIC projected
            position is geometrically sane - a confident-but-drifted map can project a
            point far outside the camera's actual visible frame.
        (2) SIZE: an in-frame position is NOT sufficient - the homography can ALSO drift
            in scale while position still looks plausible.

        Returns (ok: bool, V-frame corners or None, reason: str). Callers must fall back
        to their pre-existing behavior on ok=False - NEVER clip/clamp an implausible
        value (s_e_n amplifies residual error ~3-7x for this camera, turning even a
        moderately-wrong clamped value into a kappa-ratchet/a_u blowup - this is exactly
        how Gazebo's IC1 75m/138m fly-away happened)."""
        pm_v1 = self._getVirtualPts(np.asarray(pm_px, np.float32), quat)
        if pm_v1 is None or len(pm_v1) != 4:
            return False, None, "shape"

        p10 = self.center / self.focal
        try:
            _held = np.asarray(self._feature_pts[-1])
            _held = _held[1] if _held.ndim == 3 else _held
            delta = 0.5 * (_held.max(0) - _held.min(0)) / self.focal
        except Exception:
            delta = np.zeros(2)
        bound = self._planar_map_rescue_fov_margin * (p10 + delta)
        ctr = pm_v1.mean(axis=0)
        if not np.all(np.abs(ctr) <= bound):
            if os.environ.get("PLANAR_MAP_DBG", "0") == "1":
                print(f"[planar_map rescue DEBUG] position reject: ctr={ctr} bound={bound}", flush=True)
            return False, pm_v1, "position"

        if self._last_real_extent_px is not None and self._last_real_extent_px > 1e-6:
            pm_px_arr = np.asarray(pm_px, dtype=float)
            pm_extent = float(max(pm_px_arr[:, 0].max() - pm_px_arr[:, 0].min(),
                                   pm_px_arr[:, 1].max() - pm_px_arr[:, 1].min()))
            ratio = pm_extent / self._last_real_extent_px
            if not ((1.0 / self._planar_map_rescue_size_ratio) <= ratio <= self._planar_map_rescue_size_ratio):
                if os.environ.get("PLANAR_MAP_DBG", "0") == "1":
                    print(f"[planar_map rescue DEBUG] size reject: pm_extent={pm_extent:.1f}px "
                          f"last_real_extent={self._last_real_extent_px:.1f}px ratio={ratio:.2f} "
                          f"pm_px_bbox=({pm_px_arr[:,0].min():.0f}-{pm_px_arr[:,0].max():.0f}, "
                          f"{pm_px_arr[:,1].min():.0f}-{pm_px_arr[:,1].max():.0f})", flush=True)
                return False, pm_v1, "size"

        return True, pm_v1, "ok"

    def _showOptFlow(self, img, C_pts, V_nP_norm):
        # 1. Ensure pixel coordinates for both frames are int32
        C_pts = [p.astype(np.int32) for p in C_pts]

        # 2. Convert normalized → pixel coordinates
        V_pts = [(pts * f + self.center).astype(np.int32) for pts in V_nP_norm]

        # 3. Draw real flow (RED)
        for old, new in zip(C_pts[0], C_pts[1]):
            img = cv2.arrowedLine(img, (int(old[0]), int(old[1])), (int(new[0]), int(new[1])), (0,0,255), 2)

        # 4. Draw real marker polygon (GREEN)
        cv2.polylines(img, [C_pts[1].reshape(-1, 1, 2)], isClosed=True, color=(0,255,0), thickness=2)

        # 5. Draw virtual flow (GREEN)
        for old, new in zip(V_pts[0], V_pts[1]):
            img = cv2.arrowedLine(img, (int(old[0]), int(old[1])), (int(new[0]), int(new[1])), (0,255,0), 2)

        # 6. Draw virtual marker polygon (RED)
        cv2.polylines(img, [V_pts[1].reshape(-1, 1, 2)], isClosed=True, color=(0,0,255), thickness=2)

        # Resize display image
        resized_img = cv2.resize(img, None, fx=1.0, fy=1.0, interpolation=cv2.INTER_AREA)
        cv2.imshow('Image Streamer', resized_img)
        if cv2.waitKey(1) == 27:
            self.close()

    def _kf_step(self, x, P, prev_t, initialized, z, t, q, r, dt_unc_max=None):
        """Generic per-channel 2-state (value, rate) constant-velocity KF step
        (ported from PX4_Gazebo/src/img_data.py, 2026-07-11 generalization).
        Operates on the passed state (no self.* writes) and returns the
        updated (x, P, prev_t, initialized) so the SAME filter can run on any
        channel count / noise params (corner flow, ring flow, centroid
        feature all share this). z: (C,) measurement, or None for a
        PREDICT-ONLY step (coast on the last estimated rate, skip the
        correction — used during a marker-loss gap so the state neither
        freezes (no step at all, the Pi's prior behavior) nor gets corrected
        against a synthetic/extrapolated value as if it were real data).
        t: timestamp. q, r: this channel's process/measurement noise.
        dt_unc_max: if set, Q's dt uses the TRUE elapsed gap (capped at
        dt_unc_max) instead of the state-transition dt (capped at 0.1s for
        numerical stability) — so P correctly reflects staleness across a
        multi-frame gap, and a fresh measurement at relock gets full
        Bayesian trust instead of a multi-frame catch-up ramp."""
        if not initialized:
            if z is None:
                return x, P, prev_t, initialized   # nothing to coast from yet
            z = np.asarray(z, dtype=float)
            x = np.zeros((len(z), 2)); x[:, 0] = z     # value=z, rate=0
            P = np.tile(np.eye(2) * 1.0, (len(z), 1, 1))   # moderate prior
            return x, P, t, True

        dt = max(min(t - prev_t, 0.1), 1e-3)
        dt_q = max(min(t - prev_t, dt_unc_max), 1e-3) if dt_unc_max is not None else dt
        F = np.array([[1.0, dt], [0.0, 1.0]])
        Q = q * np.array([[dt_q**4 / 4.0, dt_q**3 / 2.0],
                          [dt_q**3 / 2.0, dt_q**2]])
        x_pred = x @ F.T
        P_pred = F @ P @ F.T + Q
        if z is None:
            return x_pred, P_pred, t, True         # PREDICT-ONLY: coast, no correction
        z = np.asarray(z, dtype=float)
        y = z - x_pred[:, 0]
        S = P_pred[:, 0, 0] + r
        K = P_pred[:, :, 0] / S[:, None]
        x = x_pred + K * y[:, None]
        P = P_pred - K[:, :, None] * P_pred[:, 0:1, :]
        return x, P, t, True

    def _kf_update(self, z, t):
        """Corner-flow KF — thin wrapper around _kf_step.
        z=None -> predict-only coast (marker-loss gap, no correction)."""
        self._kf_x, self._kf_P, self._kf_prev_t, self._kf_initialized = self._kf_step(
            self._kf_x, self._kf_P, self._kf_prev_t, self._kf_initialized, z, t,
            self._kf_q, self._kf_r, dt_unc_max=self._kf_dt_unc_max)

    def _kf_feat_update(self, z, t):
        """4-channel 2-state KF for the centroid feature (xc, yc, 1, alpha) —
        thin wrapper around the shared _kf_step, with its OWN (q, r)
        (self._kf_feat_q/_r — the flow KF's q/r are mis-scaled for the
        order-1 centroid). z: (4,) raw feature, or None for a PREDICT-ONLY
        coast (marker-loss gap, no correction — mirrors _kf_update)."""
        self._kf_feat_x, self._kf_feat_P, self._kf_feat_prev_t, self._kf_feat_initialized = \
            self._kf_step(self._kf_feat_x, self._kf_feat_P, self._kf_feat_prev_t,
                          self._kf_feat_initialized, z, t,
                          self._kf_feat_q, self._kf_feat_r, dt_unc_max=self._kf_dt_unc_max)

    def _compute_savgol_output(self):
        """Latest savgol(FILTER_WIN, FILTER_POLYORDER) sample of the raw [h;w]
        (PRE-calibration 6-vec). Legacy fallback for IMG_FILTER=savgol."""
        if len(self._opt_flow_ang_vel_raw) == 0:
            return np.zeros(6)
        if len(self._opt_flow_ang_vel_raw) >= FILTER_WIN:
            sgf_buf = sgf(self._opt_flow_ang_vel_raw[-FILTER_WIN:],
                          FILTER_WIN, FILTER_POLYORDER, axis=0)
            return sgf_buf[int(FILTER_WIN / 2 + 1)]
        return np.mean(self._opt_flow_ang_vel_raw, axis=0)

    def _getRealPtsFromV(self, V_pts, quat):
        """See img_geometry.get_real_pts_from_v (single source of truth)."""
        return get_real_pts_from_v(V_pts, quat, center=self.center)

    def _vframe_w(self, w_body, quat):
        """See img_geometry.vframe_w (single source of truth)."""
        return vframe_w(w_body, quat)

    def _compute_ring_flow(self, imgs, quats, main_imgs=None):
        """Texture-free V-frame flow from the fixed ring stations: LK-track the
        nadir-seeded ring on the real image, de-rotate to V, then the SAME
        _fill_A+lstsq as the corner flow. Returns (V_v_ring[6], pure_div,
        n_stations, ring_moment). Runs every frame; survives ArUco loss."""
        zero = (np.zeros(6), 0.0, 0, np.nan)
        if (imgs is None or imgs[0] is None or imgs[1] is None
                or quats is None or len(quats) < 2 or quats[0] is None or quats[1] is None):
            return zero
        # Track on the smaller 'main' stream when available (see
        # self._ring_main_stream comment in __init__) - smaller pyramid,
        # same tracking math, just scaled. Falls back to the full raw frame
        # if main_imgs wasn't passed (e.g. a caller that predates this) or
        # is itself unavailable this frame, or the env toggle is off.
        _use_main = (self._ring_main_stream and main_imgs is not None
                     and main_imgs[0] is not None and main_imgs[1] is not None)
        try:
            if _use_main:
                g0 = main_imgs[0] if main_imgs[0].ndim == 2 else cv2.cvtColor(main_imgs[0], cv2.COLOR_BGR2GRAY)
                g1 = main_imgs[1] if main_imgs[1].ndim == 2 else cv2.cvtColor(main_imgs[1], cv2.COLOR_BGR2GRAY)
            else:
                g0 = imgs[0] if imgs[0].ndim == 2 else cv2.cvtColor(imgs[0], cv2.COLOR_BGR2GRAY)
                g1 = imgs[1] if imgs[1].ndim == 2 else cv2.cvtColor(imgs[1], cv2.COLOR_BGR2GRAY)
            _seed = self._getRealPtsFromV(self._ring_pts0_V, quats[0])   # raw-pixel space (fx/fy/center are raw-only)
            _seed_track = (_seed / self._aruco_scale) if _use_main else _seed
            p1, st, _ = cv2.calcOpticalFlowPyrLK(g0, g1, _seed_track.astype(np.float32), None, **self._ring_lk_params)
            st = np.asarray(st).flatten().astype(bool)
            if int(st.sum()) < 6:
                return (np.zeros(6), 0.0, int(st.sum()), np.nan)
            p1_raw = (p1 * self._aruco_scale) if _use_main else p1
            r0 = _seed[st]; r1 = p1_raw.reshape(-1, 2)[st]
            # robustify: drop per-station flow-magnitude outliers (MAD)
            fm = np.linalg.norm(r1 - r0, axis=1)
            med = np.median(fm); mad = np.median(np.abs(fm - med)) + 1e-6
            keep = fm < med + 3.0 * 1.4826 * mad
            if int(keep.sum()) >= 6:
                r0, r1 = r0[keep], r1[keep]
            V0 = self._getVirtualPts(r0, quats[0])
            V1 = self._getVirtualPts(r1, quats[1])
            # pure divergence (radial-mean loom) about the V-frame nadir
            rvecV = -V0
            radial = np.einsum('ij,ij->i', (V1 - V0), rvecV) / (np.sum(rvecV ** 2, axis=1) + 1e-6)
            pure_div = float(np.median(radial)) * self._fps
            # ring MOMENT loom: -0.5 d(lnM)/dt of the tracked stations
            ring_moment = np.nan
            _c0 = V0.mean(0); _c1 = V1.mean(0)
            _M0 = float(np.mean(np.sum((V0 - _c0) ** 2, axis=1)))
            _M1 = float(np.mean(np.sum((V1 - _c1) ** 2, axis=1)))
            if _M0 > 1e-12 and _M1 > 1e-12:
                ring_moment = float(np.clip(-0.5 * (np.log(_M1) - np.log(_M0)) * self._fps, -10.0, 10.0))
            A = self._fill_A(V1)
            Y = np.reshape(V1 - V0, (-1,)) * self._fps
            V_v_ring, _, rank, sv = np.linalg.lstsq(A, Y, rcond=1e-3)
            cond = (sv[0] / sv[-1]) if (len(sv) > 0 and sv[-1] > 0) else np.inf
            if (rank < 6 or cond > 1e4 or not np.all(np.isfinite(V_v_ring))
                    or np.max(np.abs(V_v_ring)) > 50.0):
                V_v_ring = np.zeros(6)
            return (np.clip(V_v_ring, -10.0, 10.0), pure_div, int(len(r0)), ring_moment)
        except Exception:
            return zero

    def _ekf_fuse_step(self, corner_cal, corner_ok, corner_conf, ring_cal, ring_ok, t,
                       n_corn=999, ring_loom=0.0, ring_loom_ok=False):
        """Augmented-state EKF fusing corner (target-relative) + ring (ego) flow.
        State [h_tr(3), h_tv(3), w(3)]. Linear measurement models (constant H), so
        the update is the standard KF update. Works for stationary (h_tv->0) and
        moving (h_tv = ring-corner) targets; reconstructs h_tr through corner
        dropout. corner_conf in (0,1] scales the corner R by 1/conf."""
        if not self._ekf_init:
            if corner_ok:
                self._ekf_x[0:3] = corner_cal[0:3]
                self._ekf_x[6:9] = corner_cal[3:6]
                if ring_ok:
                    self._ekf_x[3:6] = ring_cal[0:3] - corner_cal[0:3]
            elif ring_ok:
                self._ekf_x[0:3] = ring_cal[0:3]; self._ekf_x[6:9] = ring_cal[3:6]
            else:
                return
            self._ekf_P = np.eye(9) * 1.0
            self._ekf_prev_t = t; self._ekf_init = True
            return
        dt = max(min(t - self._ekf_prev_t, 0.1), 1e-3)
        self._ekf_prev_t = t
        x = self._ekf_x.copy()
        P = self._ekf_P + self._ekf_Q * dt                  # predict (random walk F=I)

        def _update(x, P, z, H, R):
            S = H @ P @ H.T + R
            K = P @ H.T @ np.linalg.inv(S)
            x = x + K @ (z - H @ x)
            P = (np.eye(9) - K @ H) @ P
            return x, P

        loom_updated = False
        if corner_ok:
            R_c = self._R_corner / max(corner_conf, 0.02)
            if self._ring_loom_thresh > 0 and n_corn <= self._ring_loom_thresh:
                R_c = R_c.copy(); R_c[2, 2] = 1e6            # degraded -> ring carries the loom
            else:
                loom_updated = True
            x, P = _update(x, P, corner_cal, self._H_corner, R_c)
        if ring_ok:
            x, P = _update(x, P, ring_cal, self._H_ring, self._R_ring)
            loom_updated = True
        elif ring_loom_ok:
            x, P = _update(x, P, np.array([ring_loom]), self._H_ring_loom, self._R_ring_loom)
            loom_updated = True
        if self._htv_z_prior_on:
            x, P = _update(x, P, np.array([0.0]), self._H_htv_z, self._R_htv_z)
        if loom_updated:
            self._loom_stale = 0
        else:
            self._loom_stale += 1
            if self._loom_stale >= self._loom_stale_max:
                x[2] *= self._loom_decay
                x[5] *= self._loom_decay
        if self._loom_sign_guard:
            x[2] = min(x[2], 0.0)
        self._ekf_x, self._ekf_P = x, P

    def _fuse_step(self, corner_cal, corner_ok, n_corn, t, corner_conf=1.0):
        """Step the fusion EKF once for this frame (corner + ring). Inert unless
        FLOW_FUSE_RING=1; uses the ring products computed at the top of the frame.
        corner_conf in (0,1]: EKF down-weights the corner measurement by
        1/corner_conf (see _ekf_fuse_step) - ported from Gazebo's
        condition-aware rejection, see FLOW_COND_REJECT in __init__."""
        if not self._fuse_ring:
            return
        if self._ring_on and self._ring_ok:
            _ring_cal = self._sensor_cal_ring @ self._ring_v_raw
            _ring_ok = True
        else:
            _ring_cal = np.zeros(6); _ring_ok = False
        _ring_loom = self._ring_div_cal * self._ring_pure_div if self._ring_on else 0.0
        _ring_loom_ok = bool(self._ring_on and self._ring_div_loom_on
                             and self._ring_n >= 6 and np.isfinite(self._ring_pure_div))
        self._ekf_fuse_step(corner_cal, corner_ok, corner_conf, _ring_cal, _ring_ok, t,
                            n_corn=n_corn, ring_loom=_ring_loom, ring_loom_ok=_ring_loom_ok)

    def getTargetVel(self):
        """Estimated target/rover velocity h_tv (flow units) from the fusion EKF;
        ~0 for a stationary target. Zeros unless FLOW_FUSE_RING=1."""
        return self._ekf_x[3:6].copy() if self._ekf_init else np.zeros(3)

    def getLogData(self):
        return {
            "Time": self._time_log,
            "Capture Stamp": self._cap_stamp_log,   # hardware capture time, index-aligned with Time
            "Image Feature Pts": self._feature_pts,
            "Virtual Feature Pts": self._virtual_feature_pts,
            "Feature Params": self._img_feature_param,
            "Opt Flow Ang Vel": self._opt_flow_ang_vel_raw,
            "Opt Flow Estimator Tag": self._opt_flow_estimator_tag,   # 'lstsq' | 'lstsq+klt', index-aligned above
            "S Estimator Tag": self._s_estimator_tag,   # 'lstsq'|'lstsq+klt'|'planar_map_rescue'|'coast', index-aligned with Feature Params/Time
            # Ring flow now logs every frame independent of corner-marker
            # success (ported from PX4_Gazebo, see __init__ comment) - it is
            # NOT index-aligned with "Time" above; use "Ring Time" instead,
            # which is 1:1 with this array specifically.
            "Ring Opt Flow Ang Vel": self._ring_opt_flow_raw,
            "Ring Time": self._ring_time_log,
            "Target Velocity": self._target_vel_log,
            "Angular Velocity": self._imu_angvel_raw,
            "Quaternion": self._quat_log,
            "FPS": self._fps_log,
            # PlanarFeatureMap SHADOW mode (2026-07-23, Phase 2) - strictly
            # observational, does not affect any control-consumed field above.
            # Own timestamp ("Planar Map Time"), unconditional every frame,
            # same convention as "Ring Time". "Planar Map Center" entries are
            # None on frames where the map had no prediction yet (e.g. before
            # bootstrap's first marker slot is seeded via a decode).
            "Planar Map Center": self._planar_map_center_log,
            "Planar Map Confidence": self._planar_map_conf_log,
            "Planar Map Time": self._planar_map_time_log,
            "Centroid Map Raw": self._cmap_raw_log,   # (2,) V-frame, pre-_sensor_cal_s, nan when the map didn't fire
            "Alpha Map Raw": self._amap_raw_log,      # rad, V-frame, pre-cal, nan when the map didn't fire
        }
    
    def getParams(self):
        """Config snapshot written to Img_Params.txt at save time.

        EXTENDED 2026-07-27. Was `{'Capture Rate':.., 'resolution':..}` only,
        which meant a recording carried NO record of the signal-chain
        configuration it was made under. That matters because
        derive_pi_cal.py's default (no-argument) invocation aggregates EVERY
        run under Test_Data/Calibration/ -- and the 2026-07-26 session alone
        produced runs with PLASMC_CENTROID_RATE alternating 1/0, with and
        without FLOW_LAT_REDUCED, and with the planar map on and off. Pooling
        those fits signals from different chains into one cal, and nothing in
        the recording let you detect it: you had to consult the shell history.
        Same class of error as the savgol/KF train-run mismatch Gazebo fixed,
        just across runs instead of across stages.

        Emitted as a literal dict so a tool can ast.literal_eval() it and
        group/reject runs by configuration. Values are the RESOLVED runtime
        attributes wherever they exist (not os.environ lookups), so an unset
        env var reports the default actually in force rather than absent.
        The live sensor cal matrices are included too -- a recording is only
        interpretable against the cal that was applied while it was made.
        """
        def _f(x):
            try:
                return np.asarray(x).tolist()
            except Exception:
                return None
        cfg = {
            # --- identity / geometry ---
            'Capture Rate': self._capRate,
            'resolution': self._resolution,
            'main_resolution': getattr(self, '_main_resolution', None),
            'aruco_scale': _f(getattr(self, '_aruco_scale', None)),
            # --- filters (MUST match what derive_pi_cal.py replicates) ---
            'IMG_FILTER': os.environ.get('IMG_FILTER', 'kf'),
            'IMG_FEATURE_FILTER': os.environ.get('IMG_FEATURE_FILTER', 'kf'),
            'FILTER_WIN': FILTER_WIN,
            'FILTER_POLYORDER': FILTER_POLYORDER,
            'FLOW_KF_Q': self._kf_q, 'FLOW_KF_R': self._kf_r,
            'IMG_FEAT_KF_Q': self._kf_feat_q, 'IMG_FEAT_KF_R': self._kf_feat_r,
            'CENTROID_RATE_KF_Q': getattr(self, '_obs_kf_q', None),
            'CENTROID_RATE_KF_R': getattr(self, '_obs_kf_r', None),
            # --- signal-chain switches (change the SHAPE of the logged signal) ---
            'PLASMC_CENTROID_RATE': int(bool(getattr(self, '_centroid_rate', False))),
            'FLOW_LAT_REDUCED': int(bool(getattr(self, '_lat_reduced', False))),
            'FLOW_LOOM_DECOUPLE': int(bool(getattr(self, '_loom_decouple', False))),
            'FLOW_FUSE_RING': int(bool(getattr(self, '_fuse_ring', False))),
            'FLOW_DH_MAX': getattr(self, '_flow_dh_max', None),
            'PLASMC_H_EXTRAP': int(bool(getattr(self, '_h_extrap', False))),
            'PLASMC_DENSE_RECOVER': int(bool(getattr(self, '_dense_recover', False))),
            'PLASMC_MAP_FLOW': int(bool(getattr(self, '_map_flow', False))),
            'GYRO_COMP_WXY_MAX': getattr(self, '_gyro_comp_wxy_max', None),
            # --- perception path ---
            'PLANAR_MAP_SHADOW': int(bool(getattr(self, '_planar_map_shadow', False))),
            'PLASMC_PLANAR_MAP_PRIMARY': int(bool(getattr(self, '_planar_map_primary', False))),
            'MARKER_KLT_MAX_STEPS': getattr(self, '_max_lk_steps', None),
            'ARUCO_ROI_MARGIN_PX': getattr(self, '_roi_margin_px', None),
            'ARUCO_ROI_MAX_MISSES': getattr(self, '_roi_max_misses', None),
            'ARUCO_CORNER_REFINE': os.environ.get('ARUCO_CORNER_REFINE', 'subpix'),
            'ARUCO_MIN_PERIMETER_RATE': os.environ.get('ARUCO_MIN_PERIMETER_RATE', '0.02'),
            'ARUCO_MAX_PERIMETER_RATE': os.environ.get('ARUCO_MAX_PERIMETER_RATE', '4.0'),
            # --- camera ---
            'CAM_MANUAL_EXPOSURE': os.environ.get('CAM_MANUAL_EXPOSURE', '0'),
            'CAM_EXPOSURE_US': os.environ.get('CAM_EXPOSURE_US', None),
            'CAM_ANALOGUE_GAIN': os.environ.get('CAM_ANALOGUE_GAIN', None),
            'MAIN_STREAM_SIZE': os.environ.get('MAIN_STREAM_SIZE', '320,240'),
            # --- cal APPLIED while recording (a recording is only
            #     interpretable against the cal that was live) ---
            'sensor_cal_hw': _f(self._sensor_cal_hw),
            'sensor_cal_s': _f(self._sensor_cal_s),
            'sensor_cal_ring': _f(getattr(self, '_sensor_cal_ring', None)),
        }
        return repr(cfg)
    
    def getImgFeatureParam(self):
        """Calibrated, KF-smoothed feature vector [xc, yc, 1, alpha] (Gazebo-aligned).
        IMG_FEATURE_FILTER=savgol restores the legacy filter. The centroid KF is
        stepped once per fresh raw sample (this getter may be called faster)."""
        if len(self._img_feature_param) == 0:
            return np.zeros(4)
        if os.environ.get('IMG_FEATURE_FILTER', 'kf') != 'savgol':
            n = len(self._img_feature_param)
            if n != self._kf_feat_last_n:
                self._kf_feat_last_n = n
                self._kf_feat_update(self._img_feature_param[-1],
                                     self._time.perf_counter())
            if self._kf_feat_initialized:
                return self._sensor_cal_s @ self._kf_feat_x[:, 0]
        if len(self._img_feature_param) >= FILTER_WIN:
            sgf_buf = sgf(self._img_feature_param[-FILTER_WIN:], FILTER_WIN, FILTER_POLYORDER, axis=0)
            return self._sensor_cal_s @ sgf_buf[int(FILTER_WIN / 2 + 1)]
        return self._sensor_cal_s @ np.mean(self._img_feature_param, axis=0)

    @property
    def RESCUE_ACTIVE(self):
        """Ported from PX4_Gazebo/src/img_data.py (2026-07-25). True iff THIS frame's
        s/alpha came from the PlanarFeatureMap RESCUE rather than a raw ArUco/KLT decode.
        Distinct from FEATURE_PTS_FRESH, which is True for EITHER a genuine raw decode
        OR a plausibility-checked rescue (this property isolates just the rescue case)."""
        return self._planar_map_rescue_active

    @property
    def FEATURE_PTS_FRESH(self):
        """Ported from PX4_Gazebo/src/img_data.py (2026-07-25). True iff _feature_pts
        (raw pixel corners) was updated with LIVE geometry this frame (raw decode/KLT
        OR a plausibility-checked PlanarFeatureMap rescue), vs held at its last value
        (a genuine total coast - neither raw nor rescue available). Consumers that read
        _feature_pts directly should gate on this, not a raw-miss-only staleness counter
        (Pi has none currently, unlike Gazebo's legacy FEATURE_IS_STALE - if one is added
        later, it must NOT be used for this purpose, see Gazebo's own history of that
        exact mistake)."""
        return self._feature_pts_fresh

    def getOptFlowAngVel(self):
        """Calibrated, KF-smoothed optical flow + angular velocity (6-vec).
        IMG_FILTER=savgol restores the legacy filter. The decoupled moment loom
        (row 2) is already physical vz/Z, so it BYPASSES the cal row (as in Gazebo).
        FLOW_FUSE_RING=1 overrides both: the fusion EKF returns the TARGET-relative
        flow [h_tr; w] (works for stationary AND moving targets); already calibrated."""
        if self._fuse_ring and self._ekf_init:
            return np.concatenate([self._ekf_x[0:3], self._ekf_x[6:9]])
        if len(self._opt_flow_ang_vel_raw) == 0:
            return np.zeros(6)
        if os.environ.get('IMG_FILTER', 'kf') == 'savgol':
            _raw = self._compute_savgol_output()
            _out = self._sensor_cal_hw @ _raw
            if self._loom_decouple:
                _out[2] = _raw[2]
            return _out
        _out = self._sensor_cal_hw @ self._kf_x[:, 0]
        if self._loom_decouple:
            _out[2] = self._kf_x[2, 0]
        return _out

    def getLastTimeStamp(self):
        """Real perf_counter() timestamp of the most recent successful flow-
        loop iteration (self._time_log[-1]) - lets a caller polling
        getOptFlowAngVel() at its own rate (e.g. output_calibration.py's
        30 Hz GT loop) tell how stale the value it just read is, instead of
        assuming its own poll instant equals the flow computation's actual
        time. None if no successful iteration has completed yet."""
        return self._time_log[-1] if self._time_log else None

    def getRawOptFlowAngVel(self):
        """Latest KF-smoothed corner [h;w] BEFORE _sensor_cal_hw (6-vec).

        ADDED 2026-07-27. The Pi already co-samples getOptFlowAngVel() /
        getImgFeatureParam() into Ground_Truth.npy alongside each mocap pose,
        but those are CALIBRATED, so derive_pi_cal.py could not use them to
        FIT a calibration and was forced to re-load Img_Data.npy and
        np.interp() it onto the GT clock instead. That interpolation is a real
        error source: validating GT s against the image gave corr 0.0-0.52 via
        the interpolated path vs 0.29-0.83 using the exactly-paired co-sampled
        rows on the same recordings.

        Gazebo avoids this by co-sampling its RAW getters
        (getRawOptFlowAngVel/getRawRingFlowAngVel) into the GT dict
        specifically "so the derive tools can fit" - this is the missing Pi
        counterpart.

        DELIBERATELY does NOT mirror getOptFlowAngVel()'s fusion branch
        (BUG FIX 2026-07-27, same day it was introduced). getOptFlowAngVel()
        returns the fusion-EKF state when FLOW_FUSE_RING=1 and the EKF has
        initialised - but that state is POST-cal and ring-fused: the EKF's
        corner measurement is `_corner_cal = _sensor_cal_hw @ B_v` (see the
        _fuse_step call site), so its output already has _sensor_cal_hw baked
        in and is blended with the ring. Emitting it from a getter named "Raw"
        would co-sample calibrated, fused data into "Raw Opt Flow Ang Vel",
        and any cal fitted against it would be circular - fitting a cal
        against a signal that already contains that cal.

        This is not a corner case: on the Pi FLOW_FUSE_RING defaults to 1, and
        _ekf_init flips True on the first frame with EITHER corner or ring
        valid (ring logs unconditionally every frame), so the fusion branch is
        taken essentially from the start of every recording. Always return the
        PRE-cal filtered corner state instead.
        """
        if len(self._opt_flow_ang_vel_raw) == 0:
            return np.zeros(6)
        if os.environ.get('IMG_FILTER', 'kf') == 'savgol':
            return self._compute_savgol_output()
        return self._kf_x[:, 0].copy()

    def getRawImgFeatureParam(self):
        """Latest KF-smoothed [xc, yc, 1, alpha] BEFORE _sensor_cal_s.
        Raw counterpart of getImgFeatureParam() - see getRawOptFlowAngVel()
        for why the co-sampled log needs the PRE-cal value."""
        if len(self._img_feature_param) == 0:
            return np.zeros(4)
        if os.environ.get('IMG_FEATURE_FILTER', 'kf') != 'savgol':
            n = len(self._img_feature_param)
            if n != self._kf_feat_last_n:
                self._kf_feat_last_n = n
                self._kf_feat_update(self._img_feature_param[-1],
                                     self._time.perf_counter())
            if self._kf_feat_initialized:
                return self._kf_feat_x[:, 0].copy()
        if len(self._img_feature_param) >= FILTER_WIN:
            sgf_buf = sgf(self._img_feature_param[-FILTER_WIN:], FILTER_WIN,
                          FILTER_POLYORDER, axis=0)
            return sgf_buf[int(FILTER_WIN / 2 + 1)]
        return np.mean(self._img_feature_param, axis=0)

    def getRawCentroidMapFeature(self):
        """Latest raw (2,) V-frame centroid from the PlanarFeatureMap, BEFORE
        _sensor_cal_s -- (nan, nan) if the map didn't produce one this frame
        (map off, not yet bootstrapped, or no primary slot).

        Co-sampled by output_calibration.py on the same tick as
        getRawImgFeatureParam(), so deriving a MAP cal against it needs no
        separate GT/img alignment. Pi counterpart of Gazebo's
        getRawCentroidMapFeature (see feedback_map_cal_validation_gap); the
        underlying value is built from the map's own geometry rather than read
        off a _centroidMap override, because the Pi's map feeds CORNERS rather
        than a dedicated centroid consumer -- see the logging site for the
        space-chain and get_marker_center_native rationale.

        Why it is worth having: the map path currently reaches the cal only
        indirectly, as 'planar_map_rescue'/'*override*'-tagged rows inside the
        ordinary feature stream. Logging the map's INDEPENDENT estimate lets
        the map path be calibrated and cross-checked on its own, instead of
        being either excluded wholesale or blindly trusted.
        """
        if len(self._cmap_raw_log) == 0:
            return np.array([np.nan, np.nan])
        return np.array(self._cmap_raw_log[-1], dtype=float)

    def getRawAlphaMapFeature(self):
        """Latest raw map alpha (rad, V-frame) BEFORE _sensor_cal_s -- nan if the
        map didn't produce one this frame. Same co-sampling convention as
        getRawCentroidMapFeature()."""
        if len(self._amap_raw_log) == 0:
            return np.nan
        return float(self._amap_raw_log[-1])

    def getRawRingFlowAngVel(self):
        """Latest raw ring [h;w] BEFORE _sensor_cal_ring (for ring-cal derivation)."""
        return self._ring_v_raw

    def getRingFlowAngVel(self):
        """Calibrated, KF-smoothed ring flow (safety net; control consumes the
        corner flow). _sensor_cal_ring is identity until derived like the corner cal."""
        return self._sensor_cal_ring @ self._kf_x_ring[:, 0]