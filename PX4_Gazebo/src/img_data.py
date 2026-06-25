# **************************************************************************
# Changed class and node name used in gz_subscriber
# Used simulator time instead of system time
# Used virtual image feature points for optical flow calculation
# Detect nested Aruco markers of different IDs and select marker with smallest ID.
# Used Lucas-Kanade method for optical flow calculation.
# **************************************************************************

"""
Code to compute real-time optical flow
    - https://ieeexplore.ieee.org/abstract/document/8753669/
    - https://docs.opencv.org/3.4/d4/dee/tutorial_optical_flow.html
"""
import os
import numpy as np
import cv2 # OpenCV library
from threading import Thread
from collections import deque
import time
from scipy.signal import savgol_filter as sgf
from ahrs import Quaternion
from numerical_methods import extrapolate

from gz_subscriber import GZ_Subscriber, Image_Node

CHECK_NUM = 80
# Camera intrinsics for x500_mono_cam_down at 640x480, hfov=1.74 rad.
# fx = (W/2) / tan(hfov/2) = 320 / tan(0.87) ≈ 270.
# MATLAB Constants.m uses f=135 at 320x240; same hfov, so normalized image
# coordinates and PLASMC gains are identical. Only pixel-space FoV margins
# (rho_fov in controller.py) are scaled 2x vs MATLAB.
fx = 270
fy = 270

FILTER_WIN = int(os.environ.get("IMG_FILTER_WIN", "13"))
                      # sliding-window length for savgol on raw image-side measurements
                      # Retuned 2026-05-12 via tune_savgol.py across 5 calibration
                      # recordings × 8 channels. Best runtime mean|corr| was (13, 1),
                      # vs legacy (51, 2) which actually HURT runtime correlation
                      # because ~25-sample lag pulled the centroid out of phase
                      # with ground truth. Env-overridable for tuning the
                      # delay-vs-noise tradeoff (smaller window = less lag).
FILTER_POLYORDER = int(os.environ.get("IMG_FILTER_POLY", "1"))
VIDEO = False

class IMG_PROCESSOR(Thread):
    def __init__(self, resolution = (1280, 960), capRate = 60, time_keeper=time, controller=None):
        Thread.__init__(self)
        self.RECORD = os.environ.get("IMG_RECORD", "0") == "1"   # IMG_RECORD=1 saves the descent video
        # IMG_RECORD_RAW=1 → ALSO dump LOSSLESS PNG frames (+ per-frame stamp) for offline LK/GFT
        # tuning. The mp4 (mp4v ~50:1) destroys the sub-pixel inter-frame flow the loom is made of;
        # raw frames let the offline tuner see exactly what the live pipeline sees. Quaternion for the
        # V-frame is recovered offline from Ground_Truth by stamp. 2026-06-13.
        self.RECORD_RAW = os.environ.get("IMG_RECORD_RAW", "0") == "1"
        self._raw_dir = None; self._raw_i = 0; self._raw_stamps = []
        self.CONTROLLER_READY = False

        # Image streaming setup
        self._image_node = Image_Node(time_keeper=time_keeper, controller=controller)
        self._image_subscriber = GZ_Subscriber(self._image_node)
        self._time = time_keeper
        # time.sleep(0.5) # Wait for the image subscriber to start
    
        # Image processing parameters
        self._calc_time = 1e-06
        self._capRate = capRate
        self._resolution = self._image_node.getImgResolution()
        self.focal = np.array([fx, fy])
        # `_resolution` is set in gz_subscriber.py as `(msg.height, msg.width)`
        # for the ORIGINAL (pre-cv2-rotation) image. For a 640×480 sensor
        # that cv2.ROTATE_90_CW makes into a 480-wide × 640-tall image:
        #   msg.height = 480, msg.width = 640    → `_resolution = (480, 640)`
        #   post-rotation: W = 480, H = 640
        #   correct cx = W/2 = 240, cy = H/2 = 320
        # So `np.array(self._resolution)/2 = (240, 320) = (cx, cy)` is
        # already correct — `_resolution[0]` happens to equal post-rot W
        # and `_resolution[1]` happens to equal post-rot H because of how
        # the cv2 90° rotation swaps the two dimensions. A previous
        # 2026-06-01 commit transposed this with `[::-1]` thinking it was
        # a bug; that change was REVERTED 2026-06-01 after empirical
        # check showed the original was correct.
        # The actual bug (which IS real, fixed separately) was in
        # `plotter_output_calibration.ipynb` cell 38, which used
        # `center = (320, 240)` — that one was transposed and is now (240, 320).
        self.center = np.array(self._resolution) / 2.0   # (cx, cy) = (240, 320)

        # Sensor calibration — PHASED single-axis M (refreshed 2026-06-07),
        # derived over ALL 13 phased runs (calibration_data/output/, 8 original
        # Jun 5-6 + 5 new fused Jun 6) via tools/derive_board_cal.py.
        # FULL 6x6: calibrated [h;w]=M@raw.
        #
        # Standard phased excitation (apps/record_output_calibration.py): each axis
        # driven ALONE in sequence (x -> y -> z -> yaw, settles between), so the
        # GT axes are decorrelated by construction (cleaner than the prior
        # freq-multiplexed multisine, which retained cross-axis correlation that
        # biased the h-block low). Result: the h-block comes out NEAR-IDENTITY
        # (Hx +1.01, Hy +0.93, Hz +0.99) — the board-homography corner flow is
        # already well-scaled; the geometric h<->w coupling ((h0,w1)/(h1,w0)
        # off-diagonals) supplies the rest and holds for moving targets too.
        # (Prior provenance — all in git history, superseded by the all-13 refresh
        # below: the 4-multisine cal, then the 9-run, then the 8-run honest cal
        # 2026-06-06. Order in [h;w]: [h_x, h_y, h_z, w_x, w_y, w_z]. CAVEAT
        # carried forward: Hz (divergence) is the least-certain row — poorly
        # observed, high inter-run STD; re-validate the descent (vertical) gain.)
        # Corner sensor cal — re-derived 2026-06-07 from ALL 13 phased runs
        # (8 original + 5 new fused) via tools/derive_board_cal.py. R^2 Hx 0.75
        # Hy 0.75 Hz 0.79 Wz 0.71 (most-robust pool; supersedes the 8-run cal).
        # Wx/Wy rows zeroed = the gravity-leveled V-frame de-rotates roll/pitch out
        # by construction + a level-target modeling choice (CTRL_ZERO_WXY=1). This is
        # NOT because wx/wy are "geometrically unobservable from image flow" — that
        # 2026-06-04 claim was OVERTURNED 2026-06-07: in the raw/body frame wx/wy ARE
        # observable with the multi-marker board's corner SPREAD + adequate excitation
        # (see memory wxy-unobservable-imu-fusion-deferred). Zeroing here is a choice.
        # SINGLE-MARKER cal (2026-06-23): re-derived in the single ~1m-ArUco world with
        # PLASMC_SINGLE_MARKER=1 (derive_board_cal, 5 phased runs). Large-marker lateral R^2
        # Hx 0.63 Hy 0.62 — vs the nested-board single-marker collapse 0.07 (the large marker
        # escapes the rank-deficiency). The BOARD cal (multi-marker world) is preserved in
        # src/img_data.py.pre_singlemarker_cal_bak + git history.
        # h_x/h_y rows (0,1): DIAGONAL recal for FLOW_LAT_REDUCED — RECAL DONE 2026-06-25 (5
        # reduced-solve output-cal recordings, std-ratio beta=sigma_GT/sigma_raw, median; w_xy=0
        # confirmed). The reduced solve separates h_xy from w_xy at the SOLVE stage (drops the w_xy
        # cols), so the OLD DEGENERACY-RECOMBINATION rows (h_x=.86*h_x_raw+.87*w_y_raw /
        # h_y=.79*h_y_raw-.78*w_x_raw, which folded the split-off raw w_xy back into h_xy for the
        # FULL solve) are REPLACED by a pure per-axis scale: beta_x=0.73, beta_y=0.59 (the reduced
        # raw slightly OVER-reads GT). ⚠ paired with FLOW_LAT_REDUCED=1: set FLOW_LAT_REDUCED=0
        # (full solve) -> restore the recombination rows from git (pre-commit a081af9).
        self._sensor_cal_hw = np.array([
            [+0.7300, +0.0000, +0.0000, +0.0000, +0.0000, +0.0000],   # h_x = beta_x * h_x_raw (reduced-solve diagonal recal)
            [+0.0000, +0.5900, +0.0000, +0.0000, +0.0000, +0.0000],   # h_y = beta_y * h_y_raw
            [-0.1031, -0.0147, +0.5514, +0.0136, -0.1013, +0.0399],   # loom row; control uses the MOMENT loom (FLOW_LOOM_DECOUPLE) so the lstsq h_z cal is secondary
            [+0.0000, +0.0000, +0.0000, +0.0000, +0.0000, +0.0000],
            [+0.0000, +0.0000, +0.0000, +0.0000, +0.0000, +0.0000],
            [+0.0000, +0.0000, +0.0000, +0.0000, +0.0000, +0.9773]])   # w_z decoupled to w_z_raw only (board practice; avoids close-range h_y/w_x contamination of yaw)
        self._sensor_cal_hw[2, 2] = float(os.environ.get("PLASMC_LOOM_CAL", str(self._sensor_cal_hw[2, 2])))  # A/B knob: default = baked 1.0744; PLASMC_LOOM_CAL=1.2988 re-applies the recal
        if os.environ.get("PLASMC_WZ_CROSS", "0") == "1":   # restore the full old w_z cross-coupling (A/B)
            self._sensor_cal_hw[5, 0:5] = [+0.0526, +1.0862, -0.0096, -0.7395, +0.0161]
        self._sensor_cal_s  = np.diag([1.0273, 1.0669, 1.0, 1.0])

        # Texture-free RING flow calibration M_ring (calibrated [h;w]=M_ring@ring_raw).
        # The ring lstsq is NOT depth-mixed (board coplanar + V-frame leveling ->
        # uniform depth Z=altitude; ring h_z ~ corner h_z r~0.95), so a FIXED M_ring
        # generalizes. Re-derived 2026-06-07 (keyed to the all-13 corner M) via
        # tools/derive_ring_cal.py mode=transfer:
        # the ring is calibrated against the ALREADY-CALIBRATED CORNER as a transfer
        # standard (M_ring@ring ~= M_corner@corner) over 13 SAME-CLOCK recordings
        # (RingFlow landings + output runs) — avoids the Img/GT cross-clock smear of
        # the GT-direct retro path, and makes ring_cal ~= corner_cal so the
        # marker->ring handoff is continuous. Per-axis R^2 (keyed to all-13 corner M):
        # Hx 0.67 Hy 0.85 Hz 0.77 Wz 0.89 (approaching corner). Wx/Wy rows EXACTLY 0
        # (inherited from the corner standard; CTRL_ZERO_WXY=1). CAVEAT: ring-yaw Wz
        # gain ~3 with high inter-run STD — the ring sees yaw weakly (concentric, low
        # texture); trust the h-block, treat ring-yaw as coarse. PROVISIONAL: keyed
        # to the current corner M (re-derive if that changes).
        # GT-DIRECT (RING_CAL_MODE=gt) was TRIED and is WORSE: it can only use the 5
        # co-sampled fused runs (noisy, stressed SITL session: R^2 Hx 0.34 Hz 0.57
        # Wz 0.56) — so transfer (13 recordings) wins.
        # Ring is the FUSION input (control consumes EKF; FLOW_FUSE_RING default ON).
        # SINGLE-MARKER ring cal (2026-06-23, transfer mode keyed to the new corner cal).
        self._sensor_cal_ring = np.array([
            [+1.0803, -0.3005, -0.4927, +0.4782, +0.8557, +0.5029],
            [-0.1462, +0.8626, +0.5851, -0.6551, -0.3149, +0.5336],
            [-0.4810, -0.0463, +1.6539, +0.0359, -0.4477, -0.3487],
            [+0.0000, +0.0000, +0.0000, +0.0000, +0.0000, +0.0000],
            [+0.0000, +0.0000, +0.0000, +0.0000, +0.0000, +0.0000],
            [-0.9229, +0.6559, +0.4264, -0.7377, -0.8550, +0.7238]])

        # ArUco marker detection setup, with sub-pixel corner refinement
        # (added 2026-05-13). Default cornerRefinementMethod is CORNER_REFINE_NONE
        # which returns integer-rounded pixel corners. SUBPIX runs the standard
        # cv2.cornerSubPix algorithm internally on each detected corner, giving
        # sub-pixel precision before LK tracks them. Cheap and improves the
        # downstream lstsq input quality.
        #
        # 2026-05-22 — INTERVENTION 1: tuned for low-contrast detection so we
        # can pick up the small marker through drone-body shadow at low alt
        # (the failure mode where Image_Feature_Pts freezes in the last 100-
        # 150 ms of descent).  Env knobs let us A/B without code edits.
        _arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        _arucoParams = cv2.aruco.DetectorParameters()
        _arucoParams.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        _arucoParams.cornerRefinementWinSize = 5       # 11x11 actual window
        _arucoParams.cornerRefinementMaxIterations = 30
        _arucoParams.cornerRefinementMinAccuracy = 0.01
        # Low-contrast / shadow-robustness tuning.  Defaults shown in comments.
        _arucoParams.adaptiveThreshConstant = float(
            os.environ.get("ARUCO_ADAPT_THRESH_C", "5.0"))         # default 7.0
        _arucoParams.errorCorrectionRate = float(
            os.environ.get("ARUCO_ERR_CORRECT", "0.8"))             # default 0.6
        _arucoParams.minMarkerPerimeterRate = float(
            os.environ.get("ARUCO_MIN_PERIM_RATE", "0.02"))         # default 0.03
        _arucoParams.minOtsuStdDev = float(
            os.environ.get("ARUCO_MIN_OTSU_STD", "3.0"))            # default 5.0
        self._detector = cv2.aruco.ArucoDetector(_arucoDict, _arucoParams)

        # Parameters for Lucas Kanade algorithm. Retuned 2026-05-13 for 640x480 frames
        # with focus on REJECTING low-confidence corners (was producing lstsq garbage
        # when corners drifted near the image edge during the calibration sweep).
        #   winSize (21,21)    — odd-sized window centered on each corner; ~22% of
        #                        the typical marker width at 640x480 hover altitude
        #   maxLevel 3         — pyramid levels (frame downsampled to 80x60 at L3)
        #   criteria 30 / 0.01 — tighter than before (was 20 / 0.05); convergence
        #                        within 0.01 px or 30 iterations
        #   minEigThreshold 1e-3 — reject corners with min spatial eigenvalue below
        #                        this; cuts off poorly-tracked points before they
        #                        feed the lstsq solve
        # Env-tunable (2026-06-13) for offline LK tuning to improve the corner loom h_z
        # (the descent loom under-reports GT ~2x + corners die at close range — LK front-end limits).
        self._lk_params = dict(
            winSize=(int(os.environ.get("FLOW_LK_WIN", "21")),) * 2,
            maxLevel=int(os.environ.get("FLOW_LK_LVL", "3")),
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT,
                      int(os.environ.get("FLOW_LK_ITER", "30")), float(os.environ.get("FLOW_LK_EPS", "0.01"))),
            minEigThreshold=float(os.environ.get("FLOW_LK_EIG", "1e-3")),
        )

        # Ring stations for the TEXTURE-FREE V-frame optic flow (Singhal ring sampler):
        # fixed concentric-ring points about the image centre, LK-tracked every frame and
        # fed through the SAME _getVirtualPts + _fill_A + lstsq as the corners. Survives the
        # marker death (tracks pattern/ground, no decode needed); logged for the to-touchdown
        # output calibration. Control still consumes the corner flow until validated.
        self._ring_log_on = os.environ.get("FLOW_RINGS_LOG", "1") == "1"
        # Ring radii are RESOLUTION-ADAPTIVE: fractions of R_max = min(W,H)/2 (the largest ring that
        # fits the frame). Spanning ~17-83% of R_max (NOT clustered) both spreads the L+ lstsq AND
        # lowers divergence noise (noise ~ LK_px / r): on this 480x640 frame it cut temporal noise
        # 3.58->1.59 (-56%) vs the old fixed [40..100] (inner 42% only). Tuned 2026-06-05.
        # FLOW_RING_RADII (explicit px list) overrides the fractions if set.
        _Rmax = float(min(self._resolution)) / 2.0
        _expl = os.environ.get("FLOW_RING_RADII", "")
        if _expl:
            _ring_radii = [float(r) for r in _expl.split(",")]
        else:
            _fracs = [float(x) for x in
                      os.environ.get("FLOW_RING_FRACS", "0.17,0.33,0.50,0.67,0.83").split(",")]
            _ring_radii = [f * _Rmax for f in _fracs]
        _ring_npts = int(os.environ.get("FLOW_RING_NPTS", "60"))
        # Ring stations are formed on the VIRTUAL (gravity-leveled) image plane in NORMALIZED coords,
        # concentric about the V-frame nadir, then re-projected (POINTS, not an image warp) into the
        # real image at the current tilt via _getRealPtsFromV -> LK flow on the REAL image -> tracked
        # points projected BACK to virtual (_getVirtualPts) -> h_i,w_i from pinv(L_s). So the ring
        # ALWAYS tracks the NADIR patch (uniform depth) regardless of tilt = tilt-invariant flow.
        # (The legacy FIXED real-image ring sampled OFF-nadir under tilt -> ring-loom GT-correlation
        # collapsed near the deck; it was geometrically WRONG and was REMOVED 2026-06-24. The V-frame
        # ring is now the only ring.)
        _ring_pts_V = []
        for _rr in _ring_radii:
            _ra = 2.0 * np.pi * np.arange(_ring_npts) / _ring_npts
            _ring_pts_V.append(np.c_[(_rr / fx) * np.cos(_ra), (_rr / fy) * np.sin(_ra)])
        self._ring_pts0_V = np.vstack(_ring_pts_V).astype(np.float32)
        # Ring LK params (separate from the corner LK): larger winSize + higher
        # minEigThreshold reject ill-textured ring stations (the board's white cells lack
        # texture). Tuned offline 2026-06-05 (win 21->41, eig 1e-3->1e-2): ring spread
        # 1.28->1.20, temporal noise -9%, tracked count +58%. PARTIAL mitigation only —
        # texturing the board is the deeper fix (the noise stays high without it).
        self._ring_lk_params = dict(
            winSize=(int(os.environ.get("FLOW_RING_LK_WIN", "41")),) * 2,
            maxLevel=int(os.environ.get("FLOW_RING_LK_LVL", "3")),
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01),
            minEigThreshold=float(os.environ.get("FLOW_RING_LK_EIG", "1e-2")),
        )

        # NOTE: a V_YAW_SOURCE=alpha mode was REMOVED 2026-06-04. Rotating V to be
        # marker-aligned forces the yaw feature s[3]->0 by construction, which zeros
        # the yaw-SMC error (e_a = s[3]-s_d[3]) -> open-loop yaw -> the board
        # orientation at touchdown is uncontrolled. The V-frame is gravity-leveled and
        # body-relative (no yaw NUMBER enters _getVirtualPts); yaw is the alpha OUTPUT,
        # not an input. Compass-free belongs on the CONTROL side (BODY_YAW_SOURCE=alpha).

        # 2026-06-02 — MULTI-MARKER BOARD layout.
        # The landing pad carries several ArUco markers at KNOWN, non-
        # overlapping board offsets (Images/aruco_board_layout.npy:
        # id -> (x, y, size), DIMENSIONLESS (normalised to marker-0 size), board
        # centre at origin; no metres). The OPTICAL FLOW
        # [h;w] uses all corners identity-free (rigid-body twist); the POSITION
        # feature s=(xc,yc) and yaw alpha need a stable reference, so they are
        # reconstructed from this layout via a board->V-frame homography fit to
        # whatever markers are visible (see _board_feature). This keeps the
        # centroid continuous/unbiased as markers enter/leave the FoV — a plain
        # mean-of-visible-corners would jump when the visible subset changes.
        #
        # If the layout file is absent (single-marker pad, old recordings) the
        # pipeline falls back to the single-marker _getImgFeatures path.
        self._board_layout = None
        _layout_path = os.environ.get(
            "ARUCO_BOARD_LAYOUT",
            os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         '..', 'Images', 'aruco_board_layout.npy'))
        try:
            if os.path.exists(_layout_path):
                self._board_layout = np.load(_layout_path, allow_pickle=True).item()
                print(f"[img_data] board layout loaded: {len(self._board_layout)} "
                      f"markers from {_layout_path}")
        except Exception as _e:
            print(f"[img_data] board layout load failed ({_e}); single-marker mode")
            self._board_layout = None

        # Equilibrium offset for the 2pi-disambiguated moment yaw (see
        # _marker_principal_angle). It is the steady alpha (in V) when hovering
        # aligned over the board at PX4 yaw~0, so that alpha-_moment_alpha_0 == 0
        # at the desired heading. RECALIBRATED 2026-06-04 for the multi-marker
        # board + 2pi convention: -2.533 rad (-145.1 deg), measured from the
        # settle phase of an output-cal run. (The legacy single-marker pi-axis
        # value was -0.9379 and is WRONG for this geometry -> equilibrium sat at
        # ~88 deg.) Env-overridable; re-measure if the board/camera geometry
        # changes.
        self._moment_alpha_0 = float(os.environ.get("MOMENT_ALPHA0", "-2.533"))

        # 2026-06-02 — FEATURE SOURCE for the 6-DOF flow lstsq:
        #   'dense' (default, approach A): textured nested marker. The lstsq is
        #     fed DENSE interior points (3 nested quads × side points) generated
        #     from the primary marker's corners; the marker texture makes them
        #     LK-trackable. Centroid/yaw from the primary marker moments
        #     (_getImgFeatures). Multi-scale conditioning that improves at
        #     touchdown. See _get_all_feature_points.
        #   'board': multi-marker ArUco board — all decoded markers' corners
        #     feed the lstsq; centroid/yaw via board homography.
        # Default 'board': discrete ArUco corners are the resolution-correct
        # feature at 640x480 (R^2 0.55-0.90). 'dense' (textured single-marker)
        # was tested in two forms (fine-stipple geometric pts, coarse on-marker
        # GFT) and BOTH underperformed the board here (R^2 0.1-0.5) — the fine
        # features don't track at fx=270; it would only win at 1280x960 (ruled
        # out by FPS). Kept env-selectable for a future high-res/real-camera run.
        self._feature_source = os.environ.get("IMG_FEATURE_SOURCE", "board")
        self._dense_pts_per_side = int(os.environ.get("IMG_DENSE_PER_SIDE", "15"))
        # SINGLE-MARKER mode (2026-06-22, default-off). Kills the nested-ArUco primary-marker
        # SWITCHING that spikes the terminal loom -> vertical launch. Lock the loom/centroid to
        # ONE marker (re-lock only when the locked one disappears -> no per-frame min-ID flicker)
        # and feed the flow with baseline-style scaled-quad DENSE points (ported from
        # ~/ws/.../img_data_LK.py — deterministic, texture-independent; ~sqrt(N) loom-noise
        # reduction through the ill-conditioned single-marker L_s; does NOT fix cond, which is
        # spread-set). When the single marker is NOT VISIBLE (decode-fail AND KLT out-of-bounds
        # = left FoV, ~Z<0.42m for a 1m marker), hand the loom to the RING (visibility-triggered,
        # not the n_corn<=3 threshold). The detection-vs-visibility classifier IS the existing
        # KLT _in_bounds check. A/B before baking.
        self._single_marker = os.environ.get("PLASMC_SINGLE_MARKER", "1") == "1"   # default-on 2026-06-23 (single ~1m-ArUco world + matched cal). PROVISIONAL — escapes the nested-board rank-deficiency (R²0.62) but FAILED the IC2-5 gate (2/12 sub, 6/12 fly): off-center ICs trigger the off-center spurious corner-flow spike (position-term confounding). NOT git-committed; needs the off-center fix (ring-carries-lateral when s_e_n breaches). Set =0 for the board (also restore board cal+SDF).
        self._locked_marker_id = None   # the locked single marker (None = unacquired)
        # Visibility-by-MARGIN: when ANY marker corner comes within this many px of the FoV edge
        # (the marker is LEAVING/overflowing the frame), the corner is DROPPED (aruco_pts_0=None) ->
        # corner_ok stays False -> the fusion EKF lets the RING carry ALL flow components. One
        # mechanism (no separate visible-flag): the margin switches to rings a few frames BEFORE the
        # marker fully leaves, off cleaner (non-edge) corners. 0 = strict (switch only on full exit).
        self._marker_fov_margin = float(os.environ.get("PLASMC_MARKER_FOV_MARGIN", "40"))

        # 2026-05-31 — KLT fallback for marker re-acquisition.
        # When ArUco detection fails on a frame (drone shadow, low contrast,
        # partial occlusion, fast motion), we track the last good corner
        # positions forward with Lucas-Kanade. Bridges short detection
        # outages (~0.3-1 s at 60 Hz) without losing the marker. After
        # _max_lk_steps consecutive LK-only frames, we declare the marker
        # truly stale to prevent unbounded drift.
        #
        # Set MARKER_KLT_MAX_STEPS=0 to disable the fallback (legacy behaviour).
        self._max_lk_steps = int(os.environ.get("MARKER_KLT_MAX_STEPS", "20"))
        self._lk_step_count = 0
        # Guard #1 RESTORED (2026-06-11): clip the marker-LOST centroid EXTRAPOLATION to the FoV.
        # It was removed in the remove-all-guards cleanup, and the RingLoomFix n=5 fly-aways showed
        # exactly why it existed: on marker-LOST the deg-1 extrapolation ran s OFF-SCREEN (|s|=6.65 /
        # 4.63 vs FoV edge ~1.2) → wrong h_d/θ_norm → κ ratchet (1.99/4.61) → 29.5/27.3 m fly-away.
        # Conditional — only bounds the PHANTOM extrapolation, never a genuine detection (the global
        # clamp variant #2 regressed and stays dead). PLASMC_FEAT_FOV_CLIP=0 to disable.
        self._feat_fov_clip = os.environ.get("PLASMC_FEAT_FOV_CLIP", "1") == "1"
        self._prev_aruco_pts = None       # most recent good primary corners (ArUco or KLT)
        self._prev_extra_pts = None       # most recent good extra (on-marker) corners
        self._prev_img = None             # frame those corners were measured in
        # KLT corner-track PERSISTENCE (2026-06-19, thread project_decode_availability_thread).
        # The availability dropout at close range is 100% ArUco decode-fail (corners still
        # track). DEFAULT-OFF env gate. When ON: the KLT fallback carries the extra on-marker
        # corners (not just the primary 4) AND uses a PER-CORNER in-bounds gate — surviving
        # in-bounds corners drive the FLOW lstsq even when <4 primary corners remain; the
        # CENTROID/yaw stay gated on all-4-primary (else the centroid extrapolation path runs).
        # Offline-validated (tools/sim_klt_persistence.py): +6-12pp close-range flow availability,
        # ratio ~1 (no extrapolation drift). Phantom-clip preserved: off-screen corners DROPPED,
        # never extrapolated (the κ-runaway guard, feedback_lateral_kappa_runaway).
        self._klt_persist = os.environ.get("PLASMC_KLT_PERSIST", "0") == "1"
        self._klt_min_corn = int(os.environ.get("PLASMC_KLT_MIN_CORN", "4"))   # min combined corners for flow
        # PARTIAL-DECODE persistence (2026-06-21, terminal-fly-away fix). When the board
        # only PARTIALLY decodes near touchdown (e.g. only the primary marker -> N Flow
        # Corners collapses to 4), the 8x6 flow lstsq goes ill-conditioned -> spurious
        # lateral flow h_x -> terminal kick -> marker leaves FoV -> fly-away (traced on
        # the E_z=0.5 IC2 residual: decode-fail-dominated reps, corners still IN-FoV).
        # When PLASMC_KLT_PERSIST=1, carry the PREVIOUS frame's extra corners forward by
        # 1-frame LK (the primary re-decodes every frame, so _prev_img is last frame =
        # minimal drift) and merge in-FoV survivors NOT already covered by a freshly
        # decoded corner (dedup radius below). Keeps Nfc high -> conditioned lstsq.
        # Self-limiting: on a FULL decode the carried pts dedup out (already covered).
        self._klt_persist_dedup = float(os.environ.get("PLASMC_KLT_PERSIST_DEDUP_PX", "8.0"))
        # Edge margin (px): carried corners within this distance of any image edge are
        # DROPPED. Backs persistence off when the marker is leaving the FoV (geometric
        # fly-away flavor) so it doesn't feed near-edge/drifting corners; 0 = no margin
        # (the original full-frame gate). REVERTED to 0 default (2026-06-22): the 40px margin
        # A/B REGRESSED (KltMargin) -- it dropped near-edge corners that ARE the close-range
        # conditioning (marker fills FoV at terminal) so the within-session sub-meter lift fell
        # +4->+1 vs no-margin, AND did NOT kill the geometric tail (103m rep) because that's a
        # geometry/leaving problem needing the descent-gate/commit, not a perception gate.
        # Knob kept for experiments; default 0 = the better no-margin persist.
        self._klt_persist_margin = float(os.environ.get("PLASMC_KLT_PERSIST_MARGIN_PX", "0.0"))
        # Corner-flow lstsq regularization (2026-06-19). rcond = RELATIVE singular-value
        # cutoff (numpy semantics: σ < rcond·σmax are truncated). Default 1e-3 = unchanged.
        # Raising it regularizes the rank-deficient LOOM direction (σmin column): on a
        # CENTERED descent this halved close-range loom RMSE (0.78→0.42 at rcond=3e-2) by
        # killing the phantom-loom spikes (feedback_terminal_descent_loom_overreport) — but
        # it ALSO attenuates loom magnitude (ratio 0.90→0.66) and barely moves correlation,
        # so it's a spike/magnitude tradeoff = SITL-validate before baking. NB the MATLAB
        # pinv(L_s, tol=0.01) does NOT map here: that tol is ABSOLUTE and below σmin~0.077
        # (feedback_pinv_tol_loom_scaling); the PX4 analog is this RELATIVE rcond≈3e-2.
        self._flow_lstsq_rcond = float(os.environ.get("FLOW_LSTSQ_RCOND", "1e-3"))
        # DECOUPLED LOOM (2026-06-19, feedback_pinv_tol_loom_scaling). The loom V_v[2]=vz/z is the
        # σ_min (weak depth-observability) mode of the joint pinv(L_s) flow solve → coupled to the
        # lateral/rotational columns + at the mercy of the SVD tolerance (bias/variance dial, no
        # value wins). ESCAPE: estimate it DECOUPLED from the primary marker's apparent-scale rate
        # M = μ20+μ02 (trace of the V-frame corner scatter, ∝ s² ∝ 1/Z²): loom = -½·d(ln M)/dt.
        # Scale-free (no Z), well-conditioned (strongest/most-averaged signal, no inversion).
        # Offline-verified vs GT loom on a centered descent: corr 0.16→0.85, rmse 0.88→0.06.
        # DEFAULT-OFF. When ON, overrides ONLY V_v[2] (lateral h_x/h_y + ω stay from the lstsq).
        self._loom_decouple = os.environ.get("FLOW_LOOM_DECOUPLE", "1") == "1"   # BAKED default-on 2026-06-23 with single-marker: corner MOMENT loom (−½d(lnM)/dt) sidesteps the loom rank-deficiency (no pinv). Set FLOW_LOOM_DECOUPLE=0 for the pinv loom.
        # 8×5 REDUCED-PINV (FLOW_LSTSQ_DROP_LOOM_COL): when the loom is decoupled, drop the loom
        # column (v_z = [-x;-y], the σ_min weak mode) from L_s so the lstsq solves only the
        # well-conditioned lateral/rotational [h_x,h_y,w_x,w_y,w_z]. The loom comes from the moment
        # override. MATLAB's "reduced-pinv": well-calibrated (slope 1.02=pinv) but no closed-loop
        # gain over full 8×6 (lateral is strong either way) → cleanliness/conditioning, not a win.
        # Only acts with FLOW_LOOM_DECOUPLE=1. Default-off.
        self._loom_drop_col = os.environ.get("FLOW_LSTSQ_DROP_LOOM_COL", "0") == "1"
        # REDUCED LATERAL SOLVE (FLOW_LAT_REDUCED, default-off; 2026-06-25). The lateral
        # translation h_xy and the tilt w_xy occupy the SAME pixel-motion subspace — the
        # principal angle between A's [h_x,h_y] and [w_x,w_y] column spans is only ~0.4° — so in
        # the full 8×6 lstsq h_xy is the σ_min mode and per-corner LK jitter is amplified into it
        # (the corr-vs-GT 0.1-0.66 noise floor; SVD: cond 14, lateral noise gain ~146). Since the
        # V-frame is gravity-leveled, a LEVEL target has rotational flow w_xy≈0 (drone tilt
        # leveled out, no target tilt) → DROP the w_xy columns: h_xy becomes the LARGEST-σ mode
        # (cond 14→2, lateral noise gain 146→0.71, ~200× cleaner). Gated on FLOW_TARGET_LEVEL
        # (default-on): a TILTING target (ship deck) has real w_xy that is NOT zero and NOT in the
        # IMU, and dropping it mis-reads target tilt as phantom h_xy → set FLOW_TARGET_LEVEL=0 for
        # the full-solve fallback. Target yaw/translation are fine either way (w_z is orthogonal,
        # 90°; h_xy is the relative velocity). h_z still from the moment loom (FLOW_LOOM_DECOUPLE).
        # BAKED default-ON 2026-06-25 (offline-validated: lateral corr-vs-GT 0.2-0.3 -> 0.5-0.65).
        # ⚠ HARD PREREQUISITE FOR PRODUCTION: the sensor_cal_hw h_x/h_y rows below are a
        # DEGENERACY-RECOMBINATION (h_x_cal = 0.86*h_x_raw + 0.87*w_y_raw ; h_y likewise with w_x)
        # tuned for the FULL solve — they fold the split-off raw w_xy back into h_xy. The reduced
        # solve ZEROS w_xy, so those cross-terms go inert and the calibrated h_xy MIS-SCALES
        # (~3x under-read). So perception-on REQUIRES a PAIRED RECAL first: re-run output-cal with
        # FLOW_LAT_REDUCED=1 -> the h_x/h_y cal rows become ~DIAGONAL (no w_xy cross-terms).
        # GT-FB is UNAFFECTED (controller consumes GT, not perception). Set FLOW_LAT_REDUCED=0 for
        # the old full solve. (feedback_lateral_flow_reduced_solve + the cal-coupling note.)
        self._lat_reduced  = os.environ.get("FLOW_LAT_REDUCED", "1") == "1"
        self._target_level = os.environ.get("FLOW_TARGET_LEVEL", "1") == "1"
        # FLOW_LOOM_GAIN=1.0 (2026-06-19): offline RMSE-fit gave 1.15, but the MATLAB
        # CLOSED-LOOP suite ([[project_moment_loom]]) shows gain>1.0 HURTS (1.0→95, 1.1→91,
        # 1.2→88) — the ~0.82 under-read is BENIGN (controller gains are tuned around the
        # filter attenuation; lag-compensating over-drives). Keep 1.0; offline RMSE 1.15 is a red herring.
        self._loom_gain = float(os.environ.get("FLOW_LOOM_GAIN", "1.0"))
        self._primary_id = None   # smallest-ID decoded marker; for size-normalizing the loom scale M
        # FLOW_LOOM_WIN = causal linear-fit window (frames). Offline on a centered descent the
        # CAUSAL deque-fit vs GT loom: WIN=5 corr 0.69, WIN=9 corr 0.93, WIN=13 corr 0.97 (rmse
        # ~0.06; joint-lstsq was 0.16/0.88). 9 balances accuracy vs lag (~0.15s @ 60fps).
        self._mtrace_hist = deque(maxlen=int(os.environ.get("FLOW_LOOM_WIN", "9")))  # (t, ln M)

        # GYRO-COMPENSATED CENTROID-RATE OBSERVER (PLASMC_CENTROID_RATE, default-off; single-marker).
        # At altitude the small single marker's LK fails (Nfc=0) → the lstsq lateral flow is
        # unavailable. The DECODED corners survive (100% decode), so derive the lateral flow from the
        # decoded-corner centroid RATE instead: ṡ = L_v·v + L_w·w → h_x = ṡ_x + x0·h_z + (L_w·w)_x, etc.,
        # with w from the IMU GYRO (rotated to V), NOT the ill-conditioned corner lstsq. Works at Nfc=0
        # AND avoids the off-center spurious spikes. h_z stays the moment loom. A/B before baking.
        # ⛔ DEAD-END (2026-06-23): the centroid-rate observer REGRESSED (IC2 1/5 vs baseline 3/5).
        # Differentiating the DECODED centroid = NOISE (sub-pixel decode jitter amplified -> alt corr
        # 0.05, |h_lat| 5-6x inflated vs GT) — fundamentally noisier than LK flow. AND the premise was
        # wrong: the baseline altitude flow is NOT starved (alt corr 0.39-0.59; KF + flow-by-3m bridge
        # the top Nfc=0). The IC2 fly-aways are STOCHASTIC/terminal (1-2/5 run-to-run), the same ceiling
        # as the multi-marker board — NOT altitude velocity starvation. Keep default-off.
        self._centroid_rate = os.environ.get("PLASMC_CENTROID_RATE", "0") == "1"
        self._centroid_hist = deque(maxlen=int(os.environ.get("CENTROID_RATE_WIN", "9")))  # (t, x0, y0, ln M)
        self._observer_flow = np.zeros(6)   # [h_x, h_y, h_z, 0, 0, w_z] from the observer
        self._observer_valid = False        # reset per-frame; True when the observer produced flow

        # Flags and counters
        self._STAY_OPEN = True
        self.FEATURE_IS_VISIBLE = False
        self._count_check_img_feature = CHECK_NUM
        self._count_check_opt_flow = CHECK_NUM

        # 2026-05-22 — INTERVENTION 2: explicit stale-feature detection.
        # Tracks consecutive frames where FEATURE_DATA_IS_LOGGED was False
        # (either ArUco didn't detect or LK lost corners).  When the streak
        # exceeds STALE_THRESH, FEATURE_IS_STALE flips True so downstream
        # consumers (controller, landing_test) can refuse to act on the
        # extrapolated feature data.  Resets to 0 on every successful frame.
        #
        # Default threshold: 3 frames ≈ 35 ms at 86 Hz — short enough to
        # catch the failure mode (100-150 ms freezes in failed PX4 reps)
        # before the controller commits to stale-derivative drift, long
        # enough to absorb the 1-frame dropouts that the existing
        # extrapolation handles cleanly (per 2026-05-20 finding).
        self._consec_misses = 0
        self.FEATURE_IS_STALE = False
        self.STALE_THRESH = int(os.environ.get("IMG_STALE_THRESH", "3"))
        # Duty-cycle staleness (2026-06-11): consecutive-miss-only staleness was DEFEATED by
        # intermittent 1-frame re-detections during the terminal slide — a 76%-phantom stream
        # stayed "fresh" (each glimpse reset the counter AND cleared stale) → landing_test
        # closed-looped on garbage for ~6 s → 7–8.6 m slide + 688 m/s² impact, target_lost
        # never set (Guard1Restored_IC1 reps 2/5). Stale now ALSO latches when the miss
        # FRACTION over a sliding window is high, and CLEARING requires IMG_STALE_CLEAR
        # consecutive genuine detections (hysteresis) so single glimpses can't reset an
        # effectively-lost stream.
        # PROXIMITY-AWARE staleness (2026-06-11): the strict duty-cycle rules apply ONLY when the
        # last GENUINE marker extent says we're near the deck (image-only, scale-free; data:
        # mid-descent extent ≤72–103 px, deck 135–166 px → threshold 120). Near the deck a
        # glimpse-punctuated phantom stream is fatal (closed-loop slide → 688 m/s² impact);
        # at altitude a dropout is benign (extrapolation bridges it) and an abort is expensive
        # (the GLOBAL-strict config ballistic-drifted 21–23 m from altitude — DutyStale_IC1).
        # IMG_STALE_PROX_EXTENT=0 disables (pure legacy everywhere).
        self._stale_prox_extent = float(os.environ.get("IMG_STALE_PROX_EXTENT", "120"))  # px
        self._stale_prox_frac   = float(os.environ.get("IMG_STALE_PROX_FRAC", "0.5"))
        self._stale_prox_clear  = int(os.environ.get("IMG_STALE_PROX_CLEAR", "5"))
        self._last_extent_px    = 0.0     # span of the most recent GENUINE detection
        self._stale_win   = int(os.environ.get("IMG_STALE_WIN", "40"))     # frames (~1 s @ 42 Hz)
        # ⛔ DEFAULTS = legacy behavior (FRAC=1.0 disables the miss-frac latch; CLEAR=1 restores the
        # 1-hit clear). The duty-cycle config (FRAC=0.5, CLEAR=5) REGRESSED at n=5 (DutyStale_IC1:
        # mean 10.9, 3 TL, worst 23 m): it correctly latched + tagged the terminal losses, but the
        # slow CLEAR turned benign mid-flight blips into permanent final-descent aborts from
        # altitude → ballistic 21-23 m drifts. The 1-frame clear is load-bearing mid-flight; the
        # terminal false-fresh fix needs PROXIMITY-AWARE staleness (strict only near the deck).
        self._stale_frac  = float(os.environ.get("IMG_STALE_FRAC", "1.0")) # miss fraction to latch (1.0=off)
        self._stale_clear = int(os.environ.get("IMG_STALE_CLEAR", "1"))    # consec hits to clear (1=legacy)
        self._hit_hist    = deque(maxlen=self._stale_win)
        self._consec_hits = 0

        # Data storage
        self._time_log = []
        self._fps_log = []
        self._stamp_log = []   # image CAPTURE stamp per frame (clock-diag vs perf_counter Time)
        self._feature_pts = []
        self._virtual_feature_pts = []
        # _fill_A allocates A fresh each frame now (variable N with hybrid flow)
        self._quats = []
        self._img_feature_param = []
        self._opt_flow_ang_vel_raw = []
        self._imu_angvel_raw = []   # IMU body rate (FRD) [fwd,right,down], synced to the flow log
        self._quat_log = []         # FC quat [w,x,y,z], synced to the flow log (for IMU->V transform)
        self._n_flow_corners = []   # # corners fed to the lstsq per frame (board diag)
        self._ring_opt_flow_log = []   # texture-free ring V-frame flow [h;w] per frame (V_v_ring)
        self._ring_div_log = []        # pure depth-independent divergence (loom) — safety-net vertical
        self._ring_moment_log = []     # ring MOMENT loom (area-rate) — live A/B vs the divergence
        self._ring_opt_flow_kf_log = []  # ring V-frame flow through the SAME KF as corner flow
        self._n_ring_corners = []      # # ring stations fed to the ring lstsq per frame
        # Online post-filter logs (both filters computed every frame, regardless
        # of which one IMG_FILTER selects for the controller). Saved as
        # "Opt Flow KF" / "Opt Flow Savgol" inside getLogData for offline A/B.
        self._opt_flow_kf_log = []
        self._opt_flow_savgol_log = []

        # Per-axis 2-state constant-velocity Kalman filter on the 6 optic-flow /
        # ang-vel channels. Replaces the prior Savgol(13, 1) buffer.
        #
        # State per channel: [value, rate-of-change]
        # Model: x_{k+1} = F·x_k + w,   z_k = H·x_k + v,
        #        F = [[1, dt], [0, 1]],  H = [1, 0]
        #        Q = q · [[dt^4/4, dt^3/2], [dt^3/2, dt^2]]   (white-noise on ẍ)
        #        R = r                                          (measurement variance)
        #
        # Why over Savgol(13,1)? Savgol delivered ~92% HF reduction but at ~100 ms
        # group delay (legacy 51-window was outright worse due to lag). A 2-state
        # KF adapts: when motion is steady, the innovation is small and the gain
        # tightens (more smoothing); during transients, innovation grows and gain
        # opens (faster response). Steady-state HF reduction comparable to Savgol;
        # transient lag dominated by 1/(KF gain), typically <30 ms.
        #
        # Tuning: q (process noise) / r (measurement noise) ratio sets bandwidth.
        # Defaults below tuned for ~10 Hz steady-state cutoff at 60 Hz sampling
        # (matches Savgol13's effective bandwidth).
        # LEVER 3 (ceiling): q/r are env-overridable so the flow KF can be made more
        # responsive to fast transients (raise FLOW_KF_Q and/or lower FLOW_KF_R -> higher
        # bandwidth -> less attenuation of genuine >1 rad/s flow). NOTE: our raw-vs-filtered
        # A/B showed raw p95 ≈ filtered p95, so the filter is NOT the dominant ceiling (it
        # only clips noise spikes) -> expected MARGINAL; defaults preserve current behavior.
        self._kf_q = float(os.environ.get("FLOW_KF_Q", "5.0"))   # process-noise PSD (rad/s² per √s)²
        self._kf_r = float(os.environ.get("FLOW_KF_R", "0.1"))   # measurement noise variance
        self._kf_x = np.zeros((6, 2))       # [value, rate] per channel
        self._kf_P = np.tile(np.eye(2) * 1.0, (6, 1, 1))
        self._kf_prev_t = None
        self._kf_initialized = False
        # Ring-flow KF — the SAME _kf_step filter, separate state (V_v_ring through it)
        self._kf_x_ring = np.zeros((6, 2))
        self._kf_P_ring = np.tile(np.eye(2) * 1.0, (6, 1, 1))
        self._kf_ring_prev_t = None
        self._kf_ring_initialized = False

        # FUSED corner+ring optical flow — augmented-state EKF. Env FLOW_FUSE_RING=1
        # (default OFF — co-tuning + IC2-5 gate before defaulting). Works for BOTH
        # stationary AND MOVING targets by separating the two sources' physical
        # meaning instead of blending them:
        #   corner = TARGET-relative  (corners are ON the target)        -> h_tr
        #   ring   = EGO/ground motion (rings sample the static ground)  -> h_ego = h_tr + h_tv
        #   ring - corner = TARGET velocity in flow units (h_tv); == 0 for a stationary target.
        # State x(9) = [h_tr(3) target-rel flow (the CONTROL signal),
        #               h_tv(3) target/rover velocity (0 if stationary),
        #               w(3)    angular velocity (common to both sources)].
        # Measurement models are LINEAR (constant Jacobians -> EKF reduces to a KF):
        #   corner z=[h_tr; w]      H_corner=[[I,0,0],[0,0,I]]   low R
        #   ring   z=[h_tr+h_tv; w] H_ring  =[[I,I,0],[0,0,I]]   moderate R on h, HUGE on w
        # During corner DROPOUT only the ring fires, but h_tv persists (low process
        # noise) so the EKF RECONSTRUCTS h_tr = ring - h_tv -> keeps tracking the
        # moving target through the gap instead of reverting to ego-motion. (The
        # transfer-derived M_ring puts the ring on the corner SCALE; the EKF supplies
        # the FRAME decomposition.) Control consumes [h_tr; w]; h_tv is feedforward.
        self._ekf_x = np.zeros(9)
        self._ekf_P = np.eye(9) * 1.0
        self._ekf_init = False
        self._ekf_prev_t = None
        # DEFAULT ON (2026-06-07). The EKF fused [h_tr;w] is the best estimator by
        # signal (R^2 vs GT across cal/multisine/landing, wins h_z/w_z) AND feeds
        # the controller corner-EQUIVALENT h_z (fused h_z == corner-KF h_z, ratio
        # 1.00) so the descent input is unchanged from corner-only. The earlier
        # 1-rep no-descent was UNATTRIBUTED (loom stayed 0 despite commanded
        # descent) and is NOT an EKF signal change; a fused-vs-corner descent
        # comparison is the check (if corner also fails to descend at that IC it
        # was a fluke). Set FLOW_FUSE_RING=0 to revert to corner-only.
        self._fuse_ring = os.environ.get("FLOW_FUSE_RING", "1") == "1"
        _I3 = np.eye(3); _Z3 = np.zeros((3, 3))
        self._H_corner = np.block([[_I3, _Z3, _Z3], [_Z3, _Z3, _I3]])        # measures [h_tr; w]
        self._H_ring   = np.block([[_I3, _I3, _Z3], [_Z3, _Z3, _I3]])        # measures [h_tr+h_tv; w]
        # Loom-only safety net (2026-06-13). When the ring 6-DOF lstsq is REJECTED (ill-conditioned
        # under a lateral fly-away — exactly when RING_LOOM_NCORN hands it the loom), the robust
        # radial divergence (pure_div) still survives. Feed it as a SCALAR ego-loom measurement
        # (h_tr[2]+h_tv[2], = the 3rd row of H_ring) so the loom keeps updating. Without this, the
        # design's premise ("ring carries the vertical") was never wired in — the EKF's ring input
        # is the lstsq V_v_ring, which gets zeroed when rejected, so the loom got NO update and the
        # random-walk predictor FROZE on a stale value (a +0.6 balloon apex while plummeting → the
        # z-SMC saw the wrong sign and commanded MORE descent). RING_LOOM_PUREDIV=0 disables.
        self._H_ring_loom = np.array([[0., 0., 1., 0., 0., 1., 0., 0., 0.]])
        self._R_ring_loom = np.array([[float(os.environ.get("FLOW_R_RING_DIV", "0.5"))]])
        self._ring_div_loom_on = int(os.environ.get("RING_LOOM_PUREDIV", "1"))
        self._ring_div_cal = self._sensor_cal_ring[2, 2]   # raw pure_div -> calibrated loom units
        # GROUND-target vertical-velocity prior (2026-06-13). The ring measures EGO loom
        # (h_tr[2]+h_tv[2]); when corners die that scalar is under-determined and the EKF parked the
        # descent in h_tv[2], FREEZING h_tr[2] (the loom the controller reads) at its last corner
        # value (h_tr=-1.06, h_tv=+1.06, sum=0). A ground target — stationary OR a moving rover —
        # does NOT change its own altitude, so the LOOM component of its velocity h_tv[2] ~ 0 (only
        # the HORIZONTAL h_tv[0:2] is nonzero for a moving target, and that stays free). Anchoring
        # h_tv[2]->0 makes the ego-loom resolve to h_tr[2] for BOTH stationary and moving targets.
        self._H_htv_z = np.array([[0., 0., 0., 0., 0., 1., 0., 0., 0.]])
        self._R_htv_z = np.array([[float(os.environ.get("FLOW_R_HTVZ", "0.3"))]])
        self._htv_z_prior_on = int(os.environ.get("FLOW_HTVZ_PRIOR", "1"))
        # Anti-freeze: if NO source updated the loom for this many frames, decay h_tr/h_tv loom
        # toward 0 (the random-walk predictor would otherwise hold a stale value forever).
        self._loom_stale = 0
        self._loom_stale_max = int(os.environ.get("FLOW_LOOM_STALE_MAX", "6"))
        self._loom_decay     = float(os.environ.get("FLOW_LOOM_DECAY", "0.85"))
        _rc = float(os.environ.get("FLOW_R_CORNER", "0.05"))
        _rr = float(os.environ.get("FLOW_R_RING_H", "0.5"))
        self._R_corner = np.diag([_rc] * 6)                                  # trust corner (h AND w)
        # z-axis ring-weighted loom (threshold lowered 12→4→3, 2026-06-13 — user decision).
        # When n_flow_corners <= this, ignore the corner LOOM (h_tr z) in the fusion so the ring carries
        # the vertical descent; the corner keeps LATERAL (ring has ~0 lateral).
        # WHY 3 NOT 4/12: GT recheck (2026-06-13) showed the threshold was DISCARDING a live, GT-tracking
        # corner loom. At n_corn=4 the corner loom is still good (IC5 alt 0.78 m: corner −4.26 tracked GT
        # −6.57) but thresh=4 (n_corn≤4) handed it to the ring, which reads ~0 → fused collapsed to ~0.
        # =3 keeps the corner loom whenever n_corn≥4 (where it tracks GT) and only hands to the ring at
        # the TRUE terminal (n_corn≤3). (thresh=12 was worse still — fired the handoff from 4 m altitude,
        # 16× loom under-report.) Set =0 to disable entirely (trust the corner loom at all n_corn≥1).
        self._ring_loom_thresh = int(os.environ.get("RING_LOOM_NCORN", "3"))
        self._R_ring   = np.diag([_rr, _rr, _rr, 1e6, 1e6, 1e6])             # ring h ok; w garbage
        _qtr = float(os.environ.get("FLOW_Q_HTR", "5.0"))                    # target-rel responsive
        _qtv = float(os.environ.get("FLOW_Q_HTV", "0.2"))                    # rover vel ~constant (persists)
        _qw  = float(os.environ.get("FLOW_Q_W",  "5.0"))
        self._ekf_Q = np.diag([_qtr] * 3 + [_qtv] * 3 + [_qw] * 3)
        # POSITIVE-LOOM SIGN GUARD (2026-06-24, default-on). A descending landing can never have a
        # positive ego-loom: h_z = vz/Z < 0 the whole approach (vz<0 toward the deck, Z>0). When ArUco
        # decode is lost near the deck (single ~1m marker OVERFLOWS the FoV at Z≈0.4m → corners off-frame
        # → N flow corners→0), the fusion EKF falls back to the RING loom, which is noisy at that geometry
        # and swings WRONG-SIGNED (+); the z-SMC reads that as "marker receding" → commands UP thrust →
        # balloon → s_e_n breach → fly-away. Traced on IC3_rep1 (bundle 20260624-132414): loom −0.8→+1.8
        # right as N flow corners 89→0; GT-grounded audit (bundle 20260624-150335) confirmed the wrong-sign
        # frames are 86/93 at aruco=0 (ring takeover). The existing anti-stale decay only catches a FROZEN
        # loom, not an actively-updated wrong-signed one — so clamp the consumed loom ≤ 0 directly. This is
        # the physical-impossibility guard (NOT a control fix): the off-center CONTROL divergence the GT-FB
        # gate exposed (IC3/IC4 fail on perfect loom) is a separate, control-side problem. FLOW_LOOM_SIGN_GUARD=0 reverts.
        self._loom_sign_guard = os.environ.get("FLOW_LOOM_SIGN_GUARD", "1") == "1"
        self._opt_flow_fused_log = []   # fused target-relative [h_tr; w] per frame (A/B)
        self._target_vel_log = []       # estimated target velocity h_tv (flow units)

        # Centroid-feature KF (4 channels: xc, yc, scale, alpha) — same 2-state
        # constant-velocity model as the flow KF. DEFAULT since 2026-06-06
        # (IMG_FEATURE_FILTER=savgol restores the legacy filter).
        # Cuts the ~110 ms group delay of savgol(13) on the OUTER-loop centroid
        # input (savgol lags the flow KF by ~7 samples + is ~2x noisier). That
        # lag is exactly where off-center convergence stalls: KP=9 commands the
        # correction but the outer PID reacts to a ~110 ms-stale centroid — the
        # whole image path (flow + ring + now centroid) is on the KF.
        self._kf_feat_x = np.zeros((4, 2))
        self._kf_feat_P = np.tile(np.eye(2) * 1.0, (4, 1, 1))
        self._kf_feat_prev_t = None
        self._kf_feat_initialized = False
        self._kf_feat_last_n = 0
        # Centroid-KF (q, r) are DECOUPLED from the flow KF: the flow's q=5.0/r=0.1
        # are sized for optic-flow ang-vel (rad/s); the centroid (xc, yc, scale,
        # alpha) is order-1 with measured per-frame noise std ~0.04 (variance
        # ~0.0016, pooled n=6), so the flow's r=0.1 is ~10-60x too large and would
        # over-smooth. Default r=0.004 (std 0.063; between the savgol-residual floor
        # and the raw level) keeps q/r ratio responsive. Env-tunable for the A/B.
        self._kf_feat_q = float(os.environ.get("IMG_FEAT_KF_Q", "5.0"))
        self._kf_feat_r = float(os.environ.get("IMG_FEAT_KF_R", "0.004"))

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
                timer_flag = time.perf_counter()
                images = self._image_node.getImages()
                quaternions = self._image_node.getQuaternions()
                angvels = self._image_node.getAngVels()   # IMU body rate (FRD), paired w/ flow
                self._fps = self._image_node.getFPS()
                self._stamp = self._image_node.getStamp()   # image capture stamp, same frame as fps
                # print(f"Image FPS: {self._fps}")

                # Check if at least 2 frames of images have been received
                if images[0] is not None and images[1] is not None:              
                    if VIDEO:
                        # Resize display image
                        resized_img = cv2.resize(images[0], None, fx=4, fy=4, interpolation=cv2.INTER_AREA)
                        cv2.imshow('Image Streamer', resized_img)
                        if cv2.waitKey(1) == 27:
                            self.close()

                    # IMG_RECORD=1 → save the descent video.
                    if self.RECORD and self.CONTROLLER_READY:
                        if self._video is None:
                            self.timestamp = time.ctime().replace(':', '-')
                            _frame = images[1]
                            # cv2.VideoWriter frameSize is (WIDTH, HEIGHT); _resolution is (H, W) —
                            # use the actual frame dims so write() doesn't silently no-op. isColor must
                            # match the frame (down-cam is mono → isColor=False), and use a valid FPS.
                            _h, _w = _frame.shape[0], _frame.shape[1]
                            _fps = self._capRate if (isinstance(self._capRate, (int, float)) and self._capRate > 1) else 30.0
                            self._video = cv2.VideoWriter(
                                f'/home/shubham/Soft-Precise-Landing/PX4_Gazebo/test_data/Test_Videos/{self.timestamp}.mp4',
                                cv2.VideoWriter_fourcc(*'mp4v'),
                                _fps, (_w, _h), isColor=(_frame.ndim == 3))

                        self._video.write(images[1])

                    # IMG_RECORD_RAW=1 → dump LOSSLESS PNG frames + stamps (offline LK tuning).
                    if self.RECORD_RAW and self.CONTROLLER_READY:
                        if self._raw_dir is None:
                            self.timestamp = getattr(self, 'timestamp', None) or time.ctime().replace(':', '-')
                            self._raw_dir = f'/home/shubham/Soft-Precise-Landing/PX4_Gazebo/test_data/Test_Videos/{self.timestamp}_raw'
                            os.makedirs(self._raw_dir, exist_ok=True)
                        _rg = images[1] if images[1].ndim == 2 else cv2.cvtColor(images[1], cv2.COLOR_BGR2GRAY)
                        cv2.imwrite(f'{self._raw_dir}/f{self._raw_i:05d}.png', _rg,
                                    [cv2.IMWRITE_PNG_COMPRESSION, 1])   # lossless, fast
                        self._raw_stamps.append(self._stamp)
                        self._raw_i += 1

                    # Calculate the radial optical flow if it is AVAILABLE. Else the loop is restarted.
                    if self._imgProcess(images, quaternions, angvels, showVideo = VIDEO) is AVAILABLE:
                        if not AVAILABLE:
                            time.sleep(1/300) # ~300 Hz polling — camera arrives at ~62 Hz, this is just a CPU yield
                            continue                        
                        if self._count_check_opt_flow > 0:
                            self._count_check_opt_flow = 0

                    elif AVAILABLE:
                        self._count_check_opt_flow += 1
                        if self._count_check_opt_flow > CHECK_NUM:
                            print("OPTIC FLOW UNAVAILABLE...")
                            AVAILABLE = False
                            self._count_check_opt_flow = 0
                        time.sleep(1/300) # ~300 Hz polling — camera arrives at ~62 Hz, this is just a CPU yield
                        continue

                    else:
                        print("OPTIC FLOW AVAILABLE NOW...")
                        AVAILABLE = True
                        self._count_check_opt_flow = 0

                else:
                    print("Waiting to receive at least 2 frames")

                self._calc_time = time.perf_counter() - timer_flag

                time.sleep(1/300) # ~300 Hz polling — camera arrives at ~62 Hz, this is just a CPU yield

        except KeyboardInterrupt:
            print("KeyboardInterrupt: Flow Streamer Thread\n")
        
        except RuntimeError:
            print("RuntimeError: Flow Streamer Thread\n")

        except SyntaxError:
            print("SyntaxError: Flow Streamer Thread\n")

        except Exception as e:
            print(f"Unexpected error: Flow Streamer Thread: {e}\n")
        
        finally:            
            # Comment below line if image_subscriber no longer uses Python thread
            if self._image_subscriber.is_alive():
                self._image_subscriber.close()
            self._image_subscriber.join()

            # When everything done, release the video write object
            if self._video:
                self._video.release()
            if self.RECORD_RAW and self._raw_dir is not None:
                np.save(f'{self._raw_dir}/stamps.npy', np.asarray(self._raw_stamps, dtype=float))
                print(f"IMG_RECORD_RAW: saved {self._raw_i} lossless PNG frames + stamps to {self._raw_dir}")

            cv2.destroyAllWindows()
            print("Video recording stopped...")

    def _wait_for_images(self):
        """Wait until at least two frames are available."""
        print("Waiting for image streaming")
        start_time = self._time.perf_counter()
        while any(image is None for image in self._image_node.getImages()):
            time.sleep(1/300)
            if (self._time.perf_counter() - start_time) > 20:
                raise Exception("Unable to get image data.")
    
    def metrics(self):
        return {
            'fps': self._fps, 'img_process_freq':1/self._calc_time
        }

    def _compute_ring_flow(self, imgs, quats):
        """Texture-free V-frame optic flow from fixed ring stations (Singhal sampler) fed
        through the SAME _getVirtualPts + _fill_A + lstsq as the corner flow. Returns
        (V_v_ring [6], pure_div, n_stations), where pure_div is the depth-INDEPENDENT radial-mean
        divergence (Singhal loom) — the robust safety-net VERTICAL signal. The lstsq V_v_ring is
        NOT depth-mixed: the board is coplanar with the ground, so in the gravity-leveled V-frame
        every station shares one perpendicular depth Z=altitude (ring h_z tracks corner h_z at
        r~0.95 across the whole descent, verified 2026-06-06). A FIXED M_ring therefore generalizes
        and is derived like the corner cal (tools/derive_ring_cal.py). The loom is preferred for
        ROBUSTNESS (texture-free median, survives marker death), NOT for depth-invariance. See
        docs/FUNNEL_CBF_DESIGN.md. Runs every frame independent of ArUco
        detection, so it survives the marker death. The PRIMARY goal stays REDUCING perception
        death; this is the safety net."""
        zero = (np.zeros(6), 0.0, 0, np.nan)
        if (imgs is None or imgs[0] is None or imgs[1] is None
                or quats is None or len(quats) < 2 or quats[0] is None or quats[1] is None):
            return zero
        try:
            g0 = imgs[0] if imgs[0].ndim == 2 else cv2.cvtColor(imgs[0], cv2.COLOR_BGR2GRAY)
            g1 = imgs[1] if imgs[1].ndim == 2 else cv2.cvtColor(imgs[1], cv2.COLOR_BGR2GRAY)
            # Re-project the leveled (V-frame) ring into the real image at the frame-0 tilt so LK
            # tracks the NADIR patch (uniform depth, tilt-invariant). The legacy fixed real-image ring
            # was removed 2026-06-24 — it sampled off-nadir under tilt (geometrically wrong).
            _seed = self._getRealPtsFromV(self._ring_pts0_V, quats[0])
            p1, st, _ = cv2.calcOpticalFlowPyrLK(g0, g1, _seed, None, **self._ring_lk_params)
            st = np.asarray(st).flatten().astype(bool)
            if int(st.sum()) < 6:
                return (np.zeros(6), 0.0, int(st.sum()), np.nan)
            r0 = _seed[st]
            r1 = p1.reshape(-1, 2)[st]
            # robustify: drop per-station flow-magnitude outliers (the raw mean is noisy)
            fm = np.linalg.norm(r1 - r0, axis=1)
            med = np.median(fm); mad = np.median(np.abs(fm - med)) + 1e-6
            keep = fm < med + 3.0 * 1.4826 * mad
            if int(keep.sum()) >= 6:
                r0, r1 = r0[keep], r1[keep]
            # identical V-frame chain to the corner flow (gravity-leveled, same L+)
            V0 = self._getVirtualPts(r0, quats[0])
            V1 = self._getVirtualPts(r1, quats[1])
            # pure divergence (Singhal radial mean / loom): the robust safety-net VERTICAL signal.
            # FIXED 2026-06-22 to compute in the VIRTUAL plane (was real-image-plane: rvec about the
            # REAL centre on real-px flow -> GT-uncorrelated, corr~0). Now radial divergence of the
            # de-rotated flow (V1-V0) about the V-frame ORIGIN (nadir) -> matches the moment loom /
            # V_v_ring / MATLAB (everything on the virtual points). Robust median over stations.
            rvecV = -V0
            radial = np.einsum('ij,ij->i', (V1 - V0), rvecV) / (np.sum(rvecV**2, axis=1) + 1e-6)
            pure_div = float(np.median(radial)) * self._fps
            # RING MOMENT loom (the ring analog of the corner moment): area-rate of the
            # tracked stations, M=μ20+μ02 = trace of the V-frame scatter ∝ 1/Z². loom =
            # -½·d(lnM)/dt = -½·(ln M1 - ln M0)·fps (one-frame, uses the SAME MAD-rejected
            # stations as pure_div). Logged for the live ring-moment vs ring-divergence A/B.
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

    def _persist_extras(self, extra_pts_0, current_pts, img0):
        """Partial-decode persistence: 1-frame LK-carry of the previous frame's extra
        corners through a partial board decode, appending in-FoV survivors that aren't
        already covered by a freshly-decoded corner. Keeps the flow lstsq conditioned
        (high N Flow Corners) when only the primary marker decodes near touchdown.
        Default-off (gated on self._klt_persist by the caller). Identity-free: the flow
        [h;w] is a rigid-body twist, so extra corners need no marker ID. Self-limiting:
        on a full decode every carried point dedups out (already covered)."""
        try:
            if self._prev_extra_pts is None or len(self._prev_extra_pts) == 0 \
                    or self._prev_img is None:
                return extra_pts_0
            g1 = img0 if img0.ndim == 2 else cv2.cvtColor(img0, cv2.COLOR_BGR2GRAY)
            g0 = (self._prev_img if self._prev_img.ndim == 2
                  else cv2.cvtColor(self._prev_img, cv2.COLOR_BGR2GRAY))
            out = cv2.calcOpticalFlowPyrLK(g0, g1, self._prev_extra_pts, None, **self._lk_params)
            if out is None or len(out) < 2 or out[0] is None or out[1] is None:
                return extra_pts_0
            tr = out[0].reshape(-1, 2).astype(np.float32)
            ok = out[1].flatten() == 1
            h, w = g1.shape[:2]
            # Edge-MARGIN gate (2026-06-22): only carry corners safely INSIDE the FoV
            # (>= _klt_persist_margin px from every edge). On a partial decode where the
            # marker is genuinely LEAVING the frame (geometric flavor), the carried
            # corners are themselves near-edge/drifting -> feeding them keeps the loop
            # reacting to bad flow longer -> a BIGGER launch (the 74 m tail in the first
            # A/B). The margin backs persistence off as the marker leaves, so the
            # geometric flavor falls to the descent-gate/commit instead, while the
            # decode-fail flavor (corners central) still gets the conditioning benefit.
            m = self._klt_persist_margin
            inb = ((tr[:, 0] >= m) & (tr[:, 0] < w - m)
                   & (tr[:, 1] >= m) & (tr[:, 1] < h - m))
            cand = tr[ok & inb]
            if len(cand) == 0:
                return extra_pts_0
            present = (np.vstack([current_pts, extra_pts_0])
                       if len(extra_pts_0) else np.asarray(current_pts, np.float32))
            keep = []
            for p in cand:
                if len(present) == 0 or np.min(np.hypot(*(present - p).T)) > self._klt_persist_dedup:
                    keep.append(p)
                    present = np.vstack([present, p[None]])
            if keep:
                extra_pts_0 = (np.vstack([extra_pts_0, np.array(keep, np.float32)])
                               if len(extra_pts_0) else np.array(keep, np.float32)).astype(np.float32)
        except Exception:
            pass
        return extra_pts_0

    def _imgProcess(self, imgs, quats, angvels=None, showVideo = False):
        # This function will return True if the optical flow is AVAILABLE and calculate the optical flow. Else, it will return False.
        # Return type is a Boolean
        # Detect markers for both images
        size_factor = 1.0
        results = self._detector.detectMarkers(imgs[0])

        FEATURE_DATA_IS_LOGGED = False
        self._observer_valid = False   # centroid-rate observer: reset per-frame (no stale leak)
        # per-frame corner measurement + RELIABILITY for the fusion EKF. conf in
        # (0,1]: 1.0 = clean ArUco decode; ramps toward 0 as the KLT fallback
        # deepens (corners detected-but-degrading -> below ~0.5 m near touchdown).
        # The EKF scales corner R by 1/conf, so low conf hands the flow to the ring.
        # Image-based only (no altitude) -> scale-free compliant.
        _corner_ok = False; _corner_cal = None; _corner_conf = 1.0
        _n_corn = 0   # fresh-corner count this frame (0 on dropout/extrapolated frames)

        # === Marker corner acquisition (ArUco primary, KLT fallback) ===
        # When ArUco detection fails on the current frame, fall back to LK
        # tracking from the previous frame's good corners (whether they came
        # from ArUco or from a previous LK step). This bridges short detection
        # outages (drone-body shadow, marginal contrast, partial occlusion)
        # without losing the marker. Drift is capped via _max_lk_steps; after
        # that we declare the marker stale and let the normal extrapolation
        # path take over.
        # MULTI-MARKER (nested ArUco): the landing pad carries several
        # concentric ArUco markers of different scales/IDs. ALL decoded
        # markers' corners are target-rigid, so we feed the full corner set
        # into the 6-DOF lstsq — the spread across scales breaks the rank
        # deficiency that cripples a single near-axial marker (cols 0/4 and
        # 1/3 of L become parallel at small (x,y); spread separates them).
        #
        # The PRIMARY marker (smallest ID = innermost/smallest, most reliable
        # near touchdown) still defines the centroid s and yaw alpha (identity
        # needed). Because the markers are CONCENTRIC, every marker's centre is
        # the board centre, so the primary's centroid IS the board centroid —
        # invariant to which other markers happen to be in view.
        #
        # This is structurally the old "hybrid flow" (primary + extra corners)
        # but the extras are now on-target marker corners, NOT off-marker
        # Shi-Tomasi — so it is moving-target-safe (all corners share the
        # target's rigid motion; no camera-rel-ground bias).
        aruco_pts_0 = None
        extra_pts_0 = np.zeros((0, 2), dtype=np.float32)   # other markers' corners
        marker_ids = None        # per-marker IDs in all_pts_0 order (None in KLT fallback)
        used_klt_fallback = False
        if results[0]:
            ids = np.asarray(results[1]).flatten()
            M = len(ids)
            if self._single_marker:
                # SINGLE-MARKER LOCK: re-lock only when the locked marker is gone -> no per-frame
                # min-ID flicker (the loom-spike root). On (re)lock pick the LARGEST-spread marker
                # (best-conditioned, visible longest).
                if self._locked_marker_id is None or self._locked_marker_id not in ids:
                    _spreads = [float(np.std(results[0][i][0].reshape(-1, 2))) for i in range(M)]
                    primary_i = int(np.argmax(_spreads))
                    self._locked_marker_id = int(ids[primary_i])
                else:
                    primary_i = int(np.where(ids == self._locked_marker_id)[0][0])
            else:
                primary_i = int(np.argmin(ids))
            self._primary_id = int(ids[primary_i])   # for the decoupled-loom marker-size normalization
            # Primary first, then the rest — so marker k occupies corners
            # [4k:4k+4] of all_pts_0 and marker_ids[k] is its ID. Primary stays
            # first for KLT-fallback continuity + display + the strict gate.
            order = [primary_i] + [i for i in range(M) if i != primary_i]
            marker_corners_0 = [results[0][i][0].reshape(-1, 2).astype(np.float32)
                                for i in order]
            aruco_pts_0 = marker_corners_0[0]
            if self._single_marker:
                # Single-marker flow: baseline scaled-quad dense points on the ONE locked marker
                # (no other markers). 2026-06-22: GFT was tried here and REGRESSED (A/B: lat_noise
                # 0.126->0.173, vz>3 5/5) — a clean ArUco marker has only ~4-8 resolvable cell
                # corners, so GFT STARVES the point count -> less sqrt(N) averaging -> MORE noise
                # than the 180 scaled-quad points (point-count beats the aperture-problem gain).
                marker_ids = None
                extra_pts_0 = self._scaled_quad_points(aruco_pts_0, per_side=self._dense_pts_per_side)
            elif self._feature_source == 'dense':
                # Approach A (resolution-correct): extra LSTSQ points = REAL
                # corners found INSIDE the primary marker via goodFeaturesToTrack,
                # masked to the marker polygon. At 640x480 the only on-marker
                # features that resolve at altitude are the ArUco cells' own
                # corners (~18px at 5m); sub-cell texture aliases. GFT picks
                # exactly those (+ the coarse-texture squares). Masked to the
                # marker -> target-rigid -> moving-target-safe. marker_ids stays
                # None so centroid/yaw take the single-marker moment path.
                marker_ids = None
                _g0 = imgs[0] if imgs[0].ndim == 2 else cv2.cvtColor(imgs[0], cv2.COLOR_BGR2GRAY)
                _mask = np.zeros(_g0.shape[:2], np.uint8)
                cv2.fillConvexPoly(_mask, aruco_pts_0.astype(np.int32), 255)
                _gf = cv2.goodFeaturesToTrack(
                    _g0, maxCorners=int(os.environ.get("IMG_GFT_MAX", "200")),
                    qualityLevel=float(os.environ.get("IMG_GFT_QUALITY", "0.01")),
                    minDistance=int(os.environ.get("IMG_GFT_MINDIST", "5")),
                    mask=_mask)
                if _gf is not None and len(_gf) > 0:
                    extra_pts_0 = _gf.reshape(-1, 2).astype(np.float32)
                else:
                    extra_pts_0 = self._get_all_feature_points(
                        aruco_pts_0, self._dense_pts_per_side)
            else:
                # Board mode: extra points = the OTHER markers' corners.
                marker_ids = [int(ids[i]) for i in order]
                if len(marker_corners_0) > 1:
                    extra_pts_0 = np.vstack(marker_corners_0[1:]).astype(np.float32)
                # Partial-decode persistence: carry last frame's extras through a
                # partial board decode so the flow lstsq stays conditioned (Nfc high).
                # marker_ids is left as the decoded IDs (centroid/yaw use the primary
                # moment path); carried corners only feed the rigid-twist flow lstsq.
                if self._klt_persist:
                    extra_pts_0 = self._persist_extras(extra_pts_0, aruco_pts_0, imgs[0])
            if self._lk_step_count > 0:
                print(f"ArUco re-acquired after {self._lk_step_count} KLT-fallback frame(s)")
            self._lk_step_count = 0
            if ids.min() == 0:
                size_factor = 1/1.0
        elif (self._max_lk_steps > 0
              and self._prev_aruco_pts is not None
              and self._prev_img is not None
              and self._lk_step_count < self._max_lk_steps):
            try:
                _img0_gray = imgs[0] if imgs[0].ndim == 2 else cv2.cvtColor(imgs[0], cv2.COLOR_BGR2GRAY)
                _prev_gray = (self._prev_img if self._prev_img.ndim == 2
                              else cv2.cvtColor(self._prev_img, cv2.COLOR_BGR2GRAY))
                lk_out = cv2.calcOpticalFlowPyrLK(
                    _prev_gray, _img0_gray, self._prev_aruco_pts, None, **self._lk_params)
                if lk_out is not None and len(lk_out) >= 2:
                    lk_pts, lk_status = lk_out[0], lk_out[1]
                    if (lk_pts is not None and lk_status is not None
                            and int(np.sum(np.asarray(lk_status).flatten() == 1)) == 4):
                        _tracked = lk_pts.reshape(-1, 2).astype(np.float32)
                        # Abort KLT if any corner has left the image — the marker is
                        # gone and continuing to extrapolate produces off-screen centroids
                        # (s[0] up to 3× beyond image boundary) that blow up cross(dw,s)
                        # in θ_norm → κ runaway. Reset so next frame starts fresh.
                        _img_h, _img_w = imgs[0].shape[:2]
                        _in_bounds = (np.all(_tracked[:, 0] >= 0) and
                                      np.all(_tracked[:, 0] < _img_w) and
                                      np.all(_tracked[:, 1] >= 0) and
                                      np.all(_tracked[:, 1] < _img_h))
                        if _in_bounds:
                            aruco_pts_0 = _tracked
                            used_klt_fallback = True
                            self._lk_step_count += 1
                            if self._lk_step_count == 1:
                                print(f"ArUco lost — KLT fallback active (cap {self._max_lk_steps} frames)")
                            # SINGLE-MARKER: decode-fail but still IN FoV (KLT in-bounds). Keep the
                            # dense scaled-quad flow on the tracked corners (centroid uses them too).
                            if self._single_marker:
                                extra_pts_0 = self._scaled_quad_points(_tracked, per_side=self._dense_pts_per_side)
                            # PERSISTENCE (default-off): also carry the extra on-marker
                            # corners through the decode gap so the FLOW lstsq keeps its
                            # spread/conditioning while ArUco can't decode. Per-corner
                            # in-bounds gate — off-screen extras DROPPED (phantom-clip),
                            # never extrapolated. Centroid/yaw stay on the strict-4-primary
                            # path below. (The <4-primary flow path is a deferred follow-up.)
                            if (self._klt_persist and self._prev_extra_pts is not None
                                    and len(self._prev_extra_pts) > 0):
                                _ex_out = cv2.calcOpticalFlowPyrLK(
                                    _prev_gray, _img0_gray, self._prev_extra_pts, None,
                                    **self._lk_params)
                                if _ex_out is not None and len(_ex_out) >= 2 \
                                        and _ex_out[0] is not None and _ex_out[1] is not None:
                                    _ex_tr = _ex_out[0].reshape(-1, 2).astype(np.float32)
                                    _ex_ok = _ex_out[1].flatten() == 1
                                    _ex_in = ((_ex_tr[:, 0] >= 0) & (_ex_tr[:, 0] < _img_w)
                                              & (_ex_tr[:, 1] >= 0) & (_ex_tr[:, 1] < _img_h))
                                    extra_pts_0 = _ex_tr[_ex_ok & _ex_in]
                        else:
                            print(f"KLT corners left image bounds — stopping fallback")
                            self._lk_step_count = 0
                            self._prev_aruco_pts = None
                            # SINGLE-MARKER: KLT out-of-bounds = the marker fully LEFT the FoV. Drop
                            # the lock (allow re-lock if a marker reappears). With no corners,
                            # corner_ok stays False -> ring carries the flow; centroid extrapolates.
                            if self._single_marker:
                                self._locked_marker_id = None
            except Exception as _e:
                # Defensive: if anything in the KLT fallback path errors, fall
                # through to the normal stale path rather than killing the thread.
                aruco_pts_0 = None

        if aruco_pts_0 is not None:
            # Update KLT-fallback reference state for next call. ALWAYS copy
            # the image — imgs[0] is a reference into a rolling buffer that
            # gz_subscriber overwrites with new frames; storing the reference
            # would leave _prev_img pointing at stale (or freed) memory by the
            # next iteration.
            self._prev_aruco_pts = aruco_pts_0.copy()
            self._prev_img = imgs[0].copy()
            if self._klt_persist:
                # Carry the current extra on-marker corners forward for the next
                # KLT-fallback frame (empty array if none this frame).
                self._prev_extra_pts = (extra_pts_0.copy() if extra_pts_0 is not None
                                        and len(extra_pts_0) > 0 else None)

        # SINGLE-MARKER visibility-by-MARGIN: the marker is LEAVING when any corner comes within
        # _marker_fov_margin px of the FoV edge (near-edge/overflowing). When leaving, route the
        # FLOW to the ring (corner_ok forced False below) but KEEP the corners so the CENTROID still
        # tracks the marker via KLT (the 1st moment is robust to near-edge corners; the flow, a
        # derivative, is not). Only the FLOW switches to rings, not the position.
        _marker_leaving = False
        if (self._single_marker and aruco_pts_0 is not None and self._marker_fov_margin > 0):
            _m = self._marker_fov_margin; _ih, _iw = imgs[0].shape[:2]
            _cc = np.asarray(aruco_pts_0, float)
            _marker_leaving = bool(_cc[:, 0].min() < _m or _cc[:, 0].max() > _iw - _m
                                   or _cc[:, 1].min() < _m or _cc[:, 1].max() > _ih - _m)

        # GYRO-COMPENSATED CENTROID-RATE OBSERVER (default-off). Computed from the DECODED corners
        # (aruco_pts_0) — NOT the LK-tracked V_aruco_norm — so it runs even when LK fails (Nfc=0) at
        # altitude. Provides the lateral flow h_x,h_y from ṡ + loom + gyro-rotation compensation:
        #   h_x = ṡ_x + x0·h_z + y0·wz   (V-frame: roll/pitch leveled out, yaw preserved -> wz only)
        #   h_y = ṡ_y + y0·h_z − x0·wz
        # h_z from the moment loom; w from the IMU gyro rotated into the V-frame (clean, not the
        # off-center-ill-conditioned lstsq). Stored for injection at the flow-output sites below.
        if (self._single_marker and self._centroid_rate and aruco_pts_0 is not None
                and quats is not None and len(quats) > 1 and quats[1] is not None):
            try:
                _Vdec = self._getVirtualPts(np.asarray(aruco_pts_0, np.float32), quats[1])
                _x0 = float(_Vdec[:, 0].mean()); _y0 = float(_Vdec[:, 1].mean())
                _Mo = float(np.mean(np.sum((_Vdec - np.array([_x0, _y0])) ** 2, axis=1)))
                _to = float(getattr(self, '_stamp', 0.0))
                if _Mo > 1e-12 and np.isfinite(_Mo):
                    self._centroid_hist.append((_to, _x0, _y0, np.log(_Mo)))
                if len(self._centroid_hist) >= 3:
                    _ta = np.array([c[0] for c in self._centroid_hist])
                    if (_ta.max() - _ta.min()) > 1e-4:
                        _t0 = _ta - _ta[0]
                        _sdx = float(np.polyfit(_t0, [c[1] for c in self._centroid_hist], 1)[0])
                        _sdy = float(np.polyfit(_t0, [c[2] for c in self._centroid_hist], 1)[0])
                        _loom_dec = float(-0.5 * np.polyfit(_t0, [c[3] for c in self._centroid_hist], 1)[0]
                                          * self._loom_gain)   # sz=1 single-marker (size-norm inert)
                        _avo = angvels[1] if (angvels is not None and len(angvels) > 1
                                              and angvels[1] is not None) else None
                        if _avo is not None:
                            _wv = self._vframe_w([_avo.forward_rad_s, _avo.right_rad_s, _avo.down_rad_s], quats[1])
                        else:
                            _wv = np.zeros(3)
                        _alpha_rate = 0.0   # TODO moving-target: w_target_z = d(alpha)/dt; 0 for stationary
                        _wv[2] -= _alpha_rate                  # relative yaw = camera (gyro) − target
                        _hz = _loom_dec
                        # FRAME FIX (2026-06-23): ṡ is the VIRTUAL (de-rotated) centroid rate, and
                        # _getVirtualPts ALREADY levels out roll/pitch (z→world-down) while preserving
                        # yaw (rotz(yaw)). So w_x,w_y rotation flow is already removed by the de-rotation
                        # — subtracting L_w with the full gyro DOUBLE-COUNTS it (the tilt terms inflated
                        # h 6.2× off-center: meas|h_lat| 1.20 vs GT 0.19). Keep ONLY the yaw term (w_z,
                        # preserved) + the loom term. Signs from _fill_A (wz col = [−y, x]).
                        _hx = _sdx + _x0 * _hz + _y0 * _wv[2]
                        _hy = _sdy + _y0 * _hz - _x0 * _wv[2]
                        self._observer_flow = np.clip(
                            np.array([_hx, _hy, _hz, 0.0, 0.0, _wv[2]]), -10.0, 10.0)
                        self._observer_valid = True
            except Exception:
                self._observer_valid = False

        # Check if feature detection was successful
        if aruco_pts_0 is not None:
            n_aruco = len(aruco_pts_0)
            all_pts_0 = np.vstack([aruco_pts_0, extra_pts_0])

            # For successful run of cv2.calcOpticalFlowPyrLK() function, here's what you need to know.
            # 1. make sure the images are grayscale.
            # 2. your coordinate parameter that is i_old_pts should be single precision float meaning float32.
            # 3. the coordinate parameter i_old_pts(from your program) should be a numpy array with the dimension (n,1,2) where n represents the number of points.
            # Link: https://stackoverflow.com/questions/34540181/opencv-optical-flow-assertion
            all_pts_1, status, _ = cv2.calcOpticalFlowPyrLK(
                imgs[0], imgs[1], all_pts_0, None, **self._lk_params
            )

            status = status.flatten()
            aruco_status = status[:n_aruco]
            extra_status = status[n_aruco:]

            # PRIMARY gate stays strict: need all 4 primary corners tracked
            # for centroid/alpha. Extra (other-marker) corners are kept
            # individually — any that pass LK add spread to the lstsq.
            board_markers_px1 = None     # [(id, frame1 corners 4x2)] for homography
            if int(np.sum(aruco_status == 1)) == n_aruco:
                FEATURE_DATA_IS_LOGGED = True
                aruco_pts_1 = all_pts_1[:n_aruco].reshape(-1, 2)
                extra_good = (extra_status == 1)
                extra_pts_0_kept = extra_pts_0[extra_good]
                extra_pts_1_kept = all_pts_1[n_aruco:].reshape(-1, 2)[extra_good]
                flow_pts_0 = np.vstack([aruco_pts_0, extra_pts_0_kept])
                flow_pts_1 = np.vstack([aruco_pts_1, extra_pts_1_kept])
                # Primary-only pair preserved for centroid / alpha / display.
                C_nP = [aruco_pts_0, aruco_pts_1]

                # Collect per-marker frame-1 corners (only markers whose all-4
                # corners survived LK) + IDs, for the board homography. Marker k
                # occupies corners [4k:4k+4] of all_pts_0/all_pts_1.
                if marker_ids is not None and self._board_layout is not None:
                    all_pts_1_2d = all_pts_1.reshape(-1, 2)
                    grp = []
                    for k, mid in enumerate(marker_ids):
                        sl = slice(4 * k, 4 * k + 4)
                        if np.all(status[sl] == 1) and mid in self._board_layout:
                            grp.append((mid, all_pts_1_2d[sl].astype(np.float32)))
                    if grp:
                        board_markers_px1 = grp

            # NOTE: size_factor adjustment based on min(results[1]) == 0 lives
            # only in the ArUco-detection branch up top. On the KLT-fallback
            # path results[1] is None, so we'd crash with "'NoneType' object is
            # not iterable" if we re-ran the check here. Already handled above.

            if FEATURE_DATA_IS_LOGGED:
                # Primary-marker virtual points → fallback centroid s and alpha.
                V_aruco_norm = [self._getVirtualPts(p, a) for p, a in zip(C_nP, quats)]
                # Full (all-marker) virtual points → 6-DOF lstsq spread.
                V_flow_norm = [self._getVirtualPts(flow_pts_0, quats[0]),
                               self._getVirtualPts(flow_pts_1, quats[1])]
                # Per-marker frame-1 V-corners → board homography (centroid/yaw).
                board_markers_V = None
                if board_markers_px1 is not None:
                    board_markers_V = [(mid, self._getVirtualPts(c, quats[1]))
                                       for mid, c in board_markers_px1]

                # (V_YAW_SOURCE=alpha marker-alignment removed 2026-06-04 — it zeroed
                # the yaw feature s[3]; V stays gravity-leveled/body-relative. See the
                # note in __init__.)

                # Shows image with optical flow (primary marker only)
                if showVideo:
                    self._showOptFlow(imgs[1], C_nP, V_aruco_norm)

                # Compute optical flow via 6-DOF image-Jacobian lstsq on the
                # full multi-marker corner set ((2N)x6, N>=4). With one marker
                # this is the original 8x6; with several decoded the system is
                # strongly over-determined AND spatially spread, which is what
                # restores rank on the lateral/tilt columns. Notes from
                # 2026-05-13 experiments (kept lstsq, rejected robust-regression
                # alternatives for this specific algebra):
                #   - IRLS/Huber on residuals: NO-OP. Residual nullspace is 2D
                #     with equal projection onto every corner pair, so per-corner
                #     residual norms are structurally equal regardless of which
                #     corner is bad.
                #   - LOO by held-out residual norm: same symmetry, NO-OP.
                #   - LK-`err` row weighting: HURT (cap-hit% rose 0.5→1.9%);
                #     downweighting pushes the system toward 6x6 exactly
                #     determined, which is more sensitive to remaining noise.
                # Noise reduction is handled temporally (KF in getOptFlowAngVel),
                # not via the lstsq solve itself.
                # Stability gates retained: rcond=1e-3, rank, condition, ±10 clip.
                A = self._fill_A(V_flow_norm[1])
                Y = np.reshape(V_flow_norm[1] - V_flow_norm[0], (-1,)) * self._fps

                # V_v: corner-flow V-frame 6-DOF velocity [h; w] (renamed from B_v — it is the
                # VIRTUAL-frame velocity, not body-frame). V_v_ring is its texture-free ring twin.
                if self._lat_reduced and self._target_level:
                    # REDUCED 4-DOF lateral solve: drop the w_xy columns (3,4). h_xy is the σ_min
                    # mode of the full 8×6 (degenerate with tilt w_xy, 0.4°) → noise-amplified;
                    # dropping w_xy makes it the LARGEST-σ mode (cond 14→2). w_xy set 0 (V-frame
                    # leveled + level target; controller zeros it anyway), h_z overridden by the
                    # moment loom below. Tilting-target fallback = FLOW_TARGET_LEVEL=0 (full solve).
                    A4 = np.delete(A, [3, 4], axis=1)             # keep [h_x, h_y, h_z, w_z]
                    V4, residuals, rank, sv = np.linalg.lstsq(A4, Y, rcond=self._flow_lstsq_rcond)
                    V_v = np.array([V4[0], V4[1], V4[2], 0.0, 0.0, V4[3]])
                    _min_rank = 4
                elif self._loom_decouple and self._loom_drop_col:
                    # 8×5 reduced-pinv: drop the loom column (idx 2); loom from the moment override.
                    A5 = np.delete(A, 2, axis=1)
                    V5, residuals, rank, sv = np.linalg.lstsq(A5, Y, rcond=self._flow_lstsq_rcond)
                    V_v = np.array([V5[0], V5[1], 0.0, V5[2], V5[3], V5[4]])
                    _min_rank = 5
                else:
                    V_v, residuals, rank, sv = np.linalg.lstsq(A, Y, rcond=self._flow_lstsq_rcond)
                    _min_rank = 6
                cond = (sv[0] / sv[-1]) if (len(sv) > 0 and sv[-1] > 0) else np.inf
                bad = (
                    rank < _min_rank
                    or cond > 1e4
                    or not np.all(np.isfinite(V_v))
                    or np.max(np.abs(V_v)) > 50.0
                )
                if bad:
                    V_v = np.zeros(6)
                V_v = np.clip(V_v, -10.0, 10.0)

                # DECOUPLED LOOM override (default-off): replace the lstsq loom row V_v[2]
                # with -½·d(ln M)/dt from the primary marker's V-frame scale M=μ20+μ02
                # (trace of the de-rotated corner scatter ∝ 1/Z²). A short-window linear-fit
                # derivative (causal); lateral/rotational rows of V_v are untouched.
                if self._loom_decouple:
                    _vp = V_aruco_norm[1]                          # de-rotated primary corners (4×2)
                    _ctr = _vp.mean(axis=0)
                    _M = float(np.mean(np.sum((_vp - _ctr) ** 2, axis=1)))   # μ20+μ02 ∝ (sz·f/Z)²
                    # SIZE-NORMALIZE by the primary marker's physical size² so the scale is
                    # continuous across primary-ID SWITCHES (the board's markers span ~7× in
                    # size; min-ID flickers as markers enter/leave decode → d(lnM)/dt jumps).
                    # M/sz² = (f/Z)², marker-independent. Without this the loom is GARBAGE
                    # (offline corr 0.41 vs 0.65 with norm). Fallback sz=1 (single-marker world).
                    _sz = 1.0
                    if self._board_layout is not None and self._primary_id in self._board_layout:
                        _sz = float(self._board_layout[self._primary_id][2]) or 1.0
                    _M = _M / (_sz * _sz)
                    _t = float(getattr(self, '_stamp', 0.0))
                    if _M > 1e-12 and np.isfinite(_M):
                        self._mtrace_hist.append((_t, np.log(_M)))
                        if len(self._mtrace_hist) >= 3:
                            _ta = np.array([h[0] for h in self._mtrace_hist])
                            _la = np.array([h[1] for h in self._mtrace_hist])
                            if (_ta.max() - _ta.min()) > 1e-4:
                                _slope = np.polyfit(_ta - _ta[0], _la, 1)[0]   # d(ln M)/dt
                                V_v[2] = float(np.clip(-0.5 * _slope * self._loom_gain, -10.0, 10.0))

                # CENTROID-RATE OBSERVER injection (a): when the LK flow IS available, still prefer
                # the gyro-compensated centroid-rate lateral (robust to the off-center spurious spikes
                # the lstsq produces). h_z stays the moment loom (V_v[2]). Default-off.
                if self._single_marker and self._centroid_rate and self._observer_valid:
                    V_v[0] = float(self._observer_flow[0])
                    V_v[1] = float(self._observer_flow[1])

                V_v_scaled = size_factor * V_v
                self._opt_flow_ang_vel_raw.append(V_v_scaled)
                _av = angvels[1] if (angvels is not None and len(angvels) > 1 and angvels[1] is not None) else None
                self._imu_angvel_raw.append(
                    np.array([_av.forward_rad_s, _av.right_rad_s, _av.down_rad_s])
                    if _av is not None else np.full(3, np.nan))
                _q1 = quats[1] if (quats is not None and len(quats) > 1 and quats[1] is not None) else None
                self._quat_log.append(
                    np.array([_q1.w, _q1.x, _q1.y, _q1.z]) if _q1 is not None else np.full(4, np.nan))
                self._n_flow_corners.append(int(len(flow_pts_1)))
                # 2-state KF update — only on a fresh raw measurement.
                self._kf_update(V_v_scaled, self._time.perf_counter())
                # Calibrated corner measurement this frame, for the fused KF (the raw
                # per-frame value, NOT the KF output — avoids double filtering).
                # corner FLOW is rejected when the marker is leaving (ring carries the flow);
                # the centroid below still uses these corners (computed in this same block).
                _corner_ok = not _marker_leaving
                _n_corn = int(len(flow_pts_1))
                _corner_cal = self._sensor_cal_hw @ V_v_scaled
                if self._loom_decouple:
                    # CAL BYPASS for the moment loom: V_v[2] is already the calibrated vz/Z
                    # (scale-free -½·d(lnM)/dt). _sensor_cal_hw row 2 = the raw-pinv→cal map
                    # (×1.0744 + cross-terms from h/w) would OVER-scale it ~7% AND re-inject the
                    # lateral/rotational coupling the decoupling removed. So take the loom straight
                    # from V_v_scaled[2] (no cal). This is ALSO what feeds the fusion EKF as the
                    # corner ego-loom → corner-MOMENT + ring-DIVERGENCE dual decoupled-loom hybrid.
                    _corner_cal[2] = V_v_scaled[2]
                # Corner RELIABILITY: clean ArUco -> 1.0; KLT-fallback corners are
                # less trustworthy the deeper the LK track (corners degrading toward
                # touchdown), so conf ramps down with the KLT step depth -> the EKF
                # gives the ring the flow below ~0.5 m as the decode becomes unreliable.
                if used_klt_fallback:
                    _corner_conf = max(0.05, 1.0 - self._lk_step_count / max(self._max_lk_steps, 1))
                else:
                    _corner_conf = 1.0
                # CONDITION-AWARE OUTLIER REJECTION (2026-06-15): the fused-EKF "noise" is
                # mostly GARBAGE SPIKES from the lstsq going ill-conditioned at low n_corners
                # (raw |h| spikes to 6 vs p95 0.9), which the rank/cond=1e4 gate lets through.
                # Down-weight the corner measurement by its conditioning so the EKF rejects
                # those frames (R_c = R_corner/conf -> high R) — smooths WITHOUT lowering the
                # flow bandwidth (unlike blanket Q/R smoothing, which attenuates genuine fast
                # flow). FLOW_COND_REJECT = cond threshold (0 = OFF, default); conf scales as
                # min(1, thresh/cond) so frames with cond > thresh are progressively rejected.
                _cond_ref = float(os.environ.get("FLOW_COND_REJECT", "0"))
                if _cond_ref > 0 and np.isfinite(cond) and cond > 0:
                    _corner_conf *= min(1.0, _cond_ref / cond)
                # Log both filters' calibrated outputs every frame for A/B.
                self._opt_flow_kf_log.append(self._sensor_cal_hw @ self._kf_x[:, 0])
                self._opt_flow_savgol_log.append(self._sensor_cal_hw @ self._compute_savgol_output())

                # Centroid s and yaw alpha. Preferred path: board homography
                # (stable board centre + true yaw from the known marker layout,
                # occlusion-robust). Fallback: single-marker weighted moments
                # on the primary marker (no layout, or homography degenerate).
                board_s = None
                if board_markers_V is not None:
                    board_s = self._board_feature(board_markers_V, size_factor)
                if board_s is not None:
                    self._img_feature_param.append(board_s)
                else:
                    self._getImgFeatures(size_factor * V_aruco_norm[1])

                if not self.FEATURE_IS_VISIBLE:
                    print("LANDING PAD VISIBLE NOW...")
                    self.FEATURE_IS_VISIBLE  = True
                if self._count_check_img_feature > 0:
                    self._count_check_img_feature = 0
                # Intervention 2: detection succeeded → reset stale counter
                self._consec_misses = 0
                self._hit_hist.append(1)
                self._consec_hits += 1
                if self.FEATURE_IS_STALE:
                    # Hysteresis: a single glimpse must NOT clear stale — require
                    # IMG_STALE_CLEAR consecutive detections AND a healthy window.
                    _mf = 1.0 - (sum(self._hit_hist) / max(len(self._hit_hist), 1))
                    if self._consec_hits >= self._stale_clear and _mf <= self._stale_frac:
                        print("LANDING PAD RE-ACQUIRED (stale → fresh)")
                        self.FEATURE_IS_STALE = False

                # Log the ArUco-only feature pts (shape (2,4,2)) — consumers
                # (offline plotters, _getImgFeatures replay) expect 4 corners.
                self._feature_pts.append(C_nP)
                self._virtual_feature_pts.append(V_aruco_norm)

        elif self.FEATURE_IS_VISIBLE:
            self._count_check_img_feature +=1
            if self._count_check_img_feature > CHECK_NUM:
                print("LANDING PAD NOT VISIBLE...")
                self._count_check_img_feature = 0
                # Swap flag value to initiate necessary action to get the image feature in the field of view of the camera.
                self.FEATURE_IS_VISIBLE  = False
                # Marker truly lost — also drop the KLT-fallback reference so we
                # don't try to LK-track from a stale image after the gap closes
                # (would propagate huge motion and fail anyway).
                self._prev_aruco_pts = None
                self._prev_extra_pts = None
                self._prev_img = None
                self._lk_step_count = 0
                self._mtrace_hist.clear()    # drop scale history so the decoupled-loom slope never spans a gap

        # Log the recorded data
        self._time_log.append(self._time.perf_counter())
        self._quats.append(quats)
        self._fps_log.append(self._fps)
        self._stamp_log.append(getattr(self, '_stamp', 0.0))   # capture stamp vs perf_counter Time (clock diag)
        # CENTROID-RATE OBSERVER injection (b): when LK FAILED (Nfc=0, FEATURE_DATA_IS_LOGGED False) but
        # the marker is decoded, the observer (decoded-corner centroid rate + gyro) supplies the corner
        # flow that the LK lstsq couldn't — so the controller gets VELOCITY at altitude instead of 0.
        # Feed it as the corner measurement for the fusion EKF (default-on path the controller consumes)
        # AND update the corner KF (IMG_FILTER=kf path). n_corn=4 > RING_LOOM_NCORN(3) keeps the
        # observer's moment loom (else the ring would override h_z). size_factor=1.0 for the id-0 marker.
        if (self._single_marker and self._centroid_rate and self._observer_valid
                and not FEATURE_DATA_IS_LOGGED):
            _obs_scaled = size_factor * self._observer_flow
            _corner_cal = self._sensor_cal_hw @ _obs_scaled
            _corner_ok = not _marker_leaving
            _corner_conf = 1.0
            _n_corn = 4
            self._kf_update(_obs_scaled, self._time.perf_counter())
        # Texture-free ring flow — computed EVERY frame (survives the marker death), logged
        # aligned with _time_log for the to-touchdown calibration. SAFETY NET only; control
        # still consumes the corner flow. PRIMARY goal remains reducing perception death.
        if self._ring_log_on:
            _vvr, _pdiv, _nr, _rmom = self._compute_ring_flow(imgs, quats)
            self._ring_opt_flow_log.append(_vvr)
            self._ring_div_log.append(_pdiv)
            self._ring_moment_log.append(_rmom)
            self._n_ring_corners.append(_nr)
            # run V_v_ring through the SAME KF the corner flow uses (separate state)
            (self._kf_x_ring, self._kf_P_ring, self._kf_ring_prev_t,
             self._kf_ring_initialized) = self._kf_step(
                self._kf_x_ring, self._kf_P_ring, self._kf_ring_prev_t,
                self._kf_ring_initialized, _vvr, self._time.perf_counter())
            self._ring_opt_flow_kf_log.append(self._kf_x_ring[:, 0].copy())

            # FUSION EKF: corner (this frame, if detected) + ring, decomposed into
            # target-relative flow + target velocity. Runs every frame so the ring
            # carries h_tr through corner dropouts (reconstructed via h_tv). Inert
            # unless FLOW_FUSE_RING=1.
            if self._fuse_ring:
                _ring_ok = (_nr > 0 and np.all(np.isfinite(_vvr)) and np.any(_vvr != 0))
                _ring_cal = self._sensor_cal_ring @ _vvr
                # robust radial-divergence loom (survives the lstsq rejection): scaled to calibrated
                # loom units, fed only when the 6-DOF lstsq is rejected (see _ekf_fuse_step).
                _ring_loom = self._ring_div_cal * _pdiv
                _ring_loom_ok = bool(self._ring_div_loom_on and _nr >= 6 and np.isfinite(_pdiv))
                self._ekf_fuse_step(_corner_cal, _corner_ok, _corner_conf,
                                    _ring_cal, _ring_ok, self._time.perf_counter(),
                                    n_corn=_n_corn, ring_loom=_ring_loom, ring_loom_ok=_ring_loom_ok)
                self._opt_flow_fused_log.append(
                    np.concatenate([self._ekf_x[0:3], self._ekf_x[6:9]]) if self._ekf_init else np.zeros(6))
                self._target_vel_log.append(self._ekf_x[3:6].copy() if self._ekf_init else np.zeros(3))

        if not FEATURE_DATA_IS_LOGGED:
            # Intervention 2: increment the stale streak and flip the flag
            # if we've been extrapolating for too many consecutive frames.
            self._consec_misses += 1
            self._hit_hist.append(0)
            self._consec_hits = 0
            _mf = 1.0 - (sum(self._hit_hist) / max(len(self._hit_hist), 1))
            if not self.FEATURE_IS_STALE and (
                    self._consec_misses >= self.STALE_THRESH
                    or (len(self._hit_hist) >= self._stale_win // 2
                        and _mf > self._stale_frac)):
                print(f"FEATURE_IS_STALE = True  ({self._consec_misses} consec misses, "
                      f"miss-frac {_mf:.2f} > {self._stale_frac})" if _mf > self._stale_frac
                      else f"FEATURE_IS_STALE = True  ({self._consec_misses} consec misses)")
                self.FEATURE_IS_STALE = True
            # No new feature data this frame (LK failed or marker not seen).
            # 2026-05-13: do NOT extrapolate the optical-flow / angular-velocity vector.
            # The previous deg=1 polynomial extrapolation cascaded: if past values
            # were noisy or clipped, the extrapolation amplified into 10^5+ outliers
            # OR saturated at the clip ceiling and propagated forever. Appending zero
            # is the safer "no information" default; the controller already does its
            # own smoothing/freezing if it sees zeros.
            #
            # For the image-feature centroid we DO extrapolate (clipped) because
            # holding the last marker position is a reasonable assumption while LK
            # briefly loses tracking — the marker hasn't moved much in a frame or two.
            # 2026-05-20: tried hold-last-value as a "safer" alternative — REGRESSED
            # badly (mean xy 0.49→1.52, max 0.77→4.83 across 5 reps). During descent
            # the marker moves in image; extrapolation predicts the trend, hold-last
            # freezes at a stale position. Single-frame dropouts can then cause
            # catastrophic lateral excursions. Keep the polyfit-deg-1 extrapolate.
            extrapolated_opt_flow_ang_vel_raw = np.zeros(6)

            extrapolated_img_feature_param = extrapolate(
                self._time_log, self._img_feature_param, n=4, deg=1, default_shape=4)
            extrapolated_img_feature_param = np.nan_to_num(
                np.asarray(extrapolated_img_feature_param), nan=0.0, posinf=5.0, neginf=-5.0)
            extrapolated_img_feature_param = np.clip(extrapolated_img_feature_param, -5.0, 5.0)
            if self._feat_fov_clip:
                # Guard #1: clip the CENTROID [0:2] to ±(p_10 + δ) — FoV edge + last-good marker
                # half-extent. Keeps the short-dropout trend (within the FoV) but bounds the
                # long-dropout off-screen run that fabricates h_d and ratchets κ into a fly-away.
                p10 = self.center / self.focal
                try:
                    _held = np.asarray(self._feature_pts[-1])
                    _held = _held[1] if _held.ndim == 3 else _held
                    delta = 0.5 * (_held.max(0) - _held.min(0)) / self.focal
                except Exception:
                    delta = np.zeros(2)
                bound = p10 + delta
                extrapolated_img_feature_param[0:2] = np.clip(
                    extrapolated_img_feature_param[0:2], -bound, bound)
            # Alpha (index 3) is a WRAPPED angle — the deg-1 extrapolation + the ±5 rad
            # clip push it PAST ±π during marker loss (observed at 286°≈5 rad), handing
            # the controller a garbage phantom yaw error -> max yaw cmd -> spin-out +
            # TARGET_LOST (the dominant divergence mode, found 2026-06-04). Linear-
            # extrapolating a wrapped angle is broken near ±180°, so HOLD the last alpha
            # (marker orientation barely changes over a short dropout) and wrap it.
            if self._img_feature_param:
                _hold_a = float(self._img_feature_param[-1][3])
                extrapolated_img_feature_param[3] = float(
                    np.arctan2(np.sin(_hold_a), np.cos(_hold_a)))

            # CENTROID-RATE OBSERVER injection: log the observer flow (decoded-corner centroid rate)
            # instead of the zero "no information" default when it produced a value this frame.
            _of = (size_factor * self._observer_flow
                   if (self._single_marker and self._centroid_rate and self._observer_valid)
                   else extrapolated_opt_flow_ang_vel_raw)
            self._opt_flow_ang_vel_raw.append(_of)
            self._imu_angvel_raw.append(np.full(3, np.nan))   # marker lost: no synced IMU pairing
            # Guard #1 companion: keep the quat VALID through marker-LOST — the FC attitude is
            # genuine (use-genuine-data directive). Only nan if the FC quat itself is missing.
            _qL = quats[1] if (quats is not None and len(quats) > 1 and quats[1] is not None) else None
            self._quat_log.append(np.array([_qL.w, _qL.x, _qL.y, _qL.z]) if _qL is not None else np.full(4, np.nan))
            self._img_feature_param.append(extrapolated_img_feature_param)
            self._n_flow_corners.append(0)   # extrapolated frame: no fresh corners

            # Log the previous data
            self._feature_pts.append(self._feature_pts[-1] if self._feature_pts else np.zeros((2,4,2)))
            self._virtual_feature_pts.append(self._virtual_feature_pts[-1] if self._virtual_feature_pts else np.zeros((2,4,2)))

        return FEATURE_DATA_IS_LOGGED

    # ── Dense interior feature points (textured nested marker, approach A) ──
    # Ported from ~/ws/scripts/soft_precise_landing/img_data_LK.py. Instead of
    # only the 4 ArUco corners, synthesize a dense set of points spread over the
    # marker interior at multiple scales. On a TEXTURED marker (stipple fill)
    # every such point lands on real 2D texture, so LK tracks it reliably (no
    # aperture problem). Benefits: (1) heavy over-determination of the 6-DOF
    # lstsq → low noise; (2) multi-radius sampling → better conditioning that
    # IMPROVES at close range (touchdown), where the marker is large-in-image.
    # Points are interior-only (built from the marker's own corners) → they ride
    # the target's rigid motion → moving-target-safe (unlike background texture).
    def _get_scaled_quadrilaterals(self, pts):
        """3 nested quads from the marker corners: 1.0, 2/3, 1/3 scale about
        the centroid. pts: (4,2). Returns list of 3 (4,2) arrays."""
        centroid = np.mean(pts, axis=0)
        return [centroid + s * (pts - centroid) for s in (1.0, 2.0/3.0, 1.0/3.0)]

    def _get_side_points(self, quad, n_per_side=15):
        """Divide each side of a quad into n_per_side points. quad: (4,2).
        Returns (4*n_per_side, 2)."""
        pts = []
        t = np.linspace(0.0, 1.0, n_per_side)
        for i in range(4):
            A = quad[i]; B = quad[(i + 1) % 4]
            pts.append(np.outer(1 - t, A) + np.outer(t, B))
        return np.vstack(pts)

    def _scaled_quad_points(self, corners, scales=(1.0, 2.0/3.0, 1.0/3.0), per_side=15):
        """Baseline (img_data_LK.py) DENSE flow points: 3 concentric scaled quadrilaterals
        (scaled toward the centroid) with `per_side` points along each side (~180 pts).
        Deterministic + texture-independent (unlike goodFeaturesToTrack). Reduces single-marker
        loom NOISE (~sqrt(N) averaging through the ill-conditioned L_s); does NOT change cond
        (spread-set). Returns (M,2) float32 in the same image frame as `corners`."""
        c = np.asarray(corners, np.float32).reshape(-1, 2)
        ctr = c.mean(axis=0)
        pts = []
        for s in scales:
            q = ctr + s * (c - ctr)
            for i in range(4):
                A, B = q[i], q[(i + 1) % 4]
                tv = np.linspace(0.0, 1.0, per_side)
                pts.append(np.outer(1.0 - tv, A) + np.outer(tv, B))
        return np.vstack(pts).astype(np.float32)

    def _get_all_feature_points(self, pts, n_per_side=15):
        """All dense interior points (3 scaled quads × side points) for the
        marker with corners `pts` (4,2). Returns (M,2) float32."""
        allp = [self._get_side_points(q, n_per_side)
                for q in self._get_scaled_quadrilaterals(pts)]
        return np.vstack(allp).astype(np.float32)

    def _fill_A(self, centered_pts):
        """
        centered_pts: shape (N, 2), N>=4 — all decoded markers' corner
        positions in the V-frame (4 per marker). Returns A of shape (2N, 6),
        the IBVS interaction matrix evaluated per corner (depth Z normalized
        to 1; the 1/Z scaling is folded into _sensor_cal_hw).
        """
        x = centered_pts[:, 0]
        y = centered_pts[:, 1]
        N = len(x)
        A = np.zeros((2 * N, 6))

        # Row indices
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

    def _kf_step(self, x, P, prev_t, initialized, z, t):
        """Generic per-channel 2-state (value, rate) constant-velocity KF step.
        Operates on the passed state (no self.* writes) and returns the updated
        (x, P, prev_t, initialized) so the SAME filter can run on multiple flow
        sources (corner flow + ring flow). z: (6,) measurement; t: timestamp."""
        if not initialized:
            x = np.zeros((6, 2)); x[:, 0] = z          # value=z, rate=0
            P = np.tile(np.eye(2) * 1.0, (6, 1, 1))    # moderate prior
            return x, P, t, True

        dt = max(min(t - prev_t, 0.1), 1e-3)
        F = np.array([[1.0, dt], [0.0, 1.0]])
        # Discrete-white-noise-on-acceleration process model
        Q = self._kf_q * np.array([
            [dt**4 / 4.0, dt**3 / 2.0],
            [dt**3 / 2.0, dt**2],
        ])
        R = self._kf_r
        # Predict: x ← Fx, P ← FPF^T + Q  (vectorized over the 6 channels)
        x_pred = x @ F.T                                   # (6, 2)
        P_pred = F @ P @ F.T + Q                           # (6, 2, 2)
        # Innovation y = z - Hx_pred (H = [1, 0]), scalar per channel
        y = z - x_pred[:, 0]                               # (6,)
        S = P_pred[:, 0, 0] + R                            # (6,)
        K = P_pred[:, :, 0] / S[:, None]                   # (6, 2)
        # Update: x ← x_pred + K·y, P ← (I - K H) P_pred
        x = x_pred + K * y[:, None]
        P = P_pred - K[:, :, None] * P_pred[:, 0:1, :]
        return x, P, t, True

    def _kf_update(self, z, t):
        """Corner-flow KF — thin wrapper around _kf_step on the corner state."""
        self._kf_x, self._kf_P, self._kf_prev_t, self._kf_initialized = self._kf_step(
            self._kf_x, self._kf_P, self._kf_prev_t, self._kf_initialized, z, t)

    def _ekf_fuse_step(self, corner_cal, corner_ok, corner_conf, ring_cal, ring_ok, t, n_corn=999,
                       ring_loom=0.0, ring_loom_ok=False):
        """Augmented-state EKF fusing corner (target-relative) + ring (ego) flow.
        State [h_tr(3), h_tv(3), w(3)]. Measurement models are linear (constant
        Jacobians H_corner/H_ring), so the EKF update is the standard KF update;
        kept in EKF form so a nonlinear term (e.g. ring deck-takeover near
        touchdown) can be added later. Works for stationary (h_tv->0) and moving
        (h_tv = ring-corner) targets; reconstructs h_tr through corner dropout.
        corner_conf in (0,1] scales the corner R by 1/conf: as corners become
        unreliable (deep KLT-fallback near touchdown) the corner is down-weighted
        and the ring carries the flow — image-based, no altitude gate."""
        if not self._ekf_init:
            if corner_ok:
                self._ekf_x[0:3] = corner_cal[0:3]      # h_tr <- corner h
                self._ekf_x[6:9] = corner_cal[3:6]      # w    <- corner w
                if ring_ok:
                    self._ekf_x[3:6] = ring_cal[0:3] - corner_cal[0:3]   # h_tv <- ring-corner
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
            R_c = self._R_corner / max(corner_conf, 0.02)   # low conf -> high R -> ring carries
            # z-axis ring-weighted loom: when ill-conditioned at low alt, ignore the corner LOOM (index 2)
            # so the ring divergence carries the vertical descent; keep the corner for lateral.
            if self._ring_loom_thresh > 0 and n_corn <= self._ring_loom_thresh:
                R_c = R_c.copy(); R_c[2, 2] = 1e6            # degraded corners present -> ring carries LOOM only
            else:
                loom_updated = True                          # corner supplied the loom this frame
            x, P = _update(x, P, corner_cal, self._H_corner, R_c)
        if ring_ok:
            x, P = _update(x, P, ring_cal, self._H_ring, self._R_ring)
            loom_updated = True                              # ring 6-DOF lstsq supplied the loom
        elif ring_loom_ok:
            # lstsq V_v_ring was REJECTED but the robust radial divergence survives: feed pure_div
            # as a scalar ego-loom measurement so the ring still carries the vertical. THE safety
            # net the RING_LOOM_NCORN design assumed (see __init__). 2026-06-13.
            x, P = _update(x, P, np.array([ring_loom]), self._H_ring_loom, self._R_ring_loom)
            loom_updated = True
        # GROUND-target prior: anchor the vertical target-velocity loom h_tv[2]->0 (a ground target
        # — stationary or a moving rover — doesn't change its own altitude). This makes the ring's
        # ego-loom (h_tr[2]+h_tv[2]) resolve to h_tr[2] (what the controller reads) instead of being
        # absorbed into h_tv[2] and freezing h_tr. h_tv[0:2] (horizontal target motion) stays free,
        # so moving-rover lateral estimation is untouched. 2026-06-13.
        if self._htv_z_prior_on:
            x, P = _update(x, P, np.array([0.0]), self._H_htv_z, self._R_htv_z)
        # Anti-FREEZE: no source updated the loom -> the F=I predictor would HOLD the last value
        # (a stale balloon-apex +0.6 while plummeting -> z-SMC sees the wrong sign). Decay toward 0
        # after a short stale streak so the controller stops trusting a frozen estimate. 2026-06-13.
        if loom_updated:
            self._loom_stale = 0
        else:
            self._loom_stale += 1
            if self._loom_stale >= self._loom_stale_max:
                x[2] *= self._loom_decay                     # h_tr loom -> 0
                x[5] *= self._loom_decay                     # h_tv loom -> 0
        # Positive-loom sign guard (see __init__): the consumed ego-loom h_tr[2] is clamped <= 0 so a
        # wrong-signed ring-takeover loom (ArUco lost near the deck) can never command an ascent. Catches
        # the actively-updated wrong sign the anti-stale decay above misses (it only handles a frozen loom).
        if self._loom_sign_guard:
            x[2] = min(x[2], 0.0)
        self._ekf_x, self._ekf_P = x, P

    def getTargetVel(self):
        """Estimated target/rover velocity in flow units (h_tv) from the fusion EKF;
        ~0 for a stationary target. Zeros unless FLOW_FUSE_RING=1. Control feedforward."""
        return self._ekf_x[3:6].copy() if self._ekf_init else np.zeros(3)

    def _kf_feat_update(self, z, t):
        """4-channel 2-state KF for the centroid feature (xc, yc, scale, alpha).

        Same constant-velocity model as _kf_update but with its OWN (q, r)
        (self._kf_feat_q/_r) — the flow KF's q=5/r=0.1 are mis-scaled for the
        order-1 centroid (see __init__). Low-lag alternative to savgol(13) for the
        OUTER-loop centroid input. z : (4,) raw feature; t : perf_counter.
        """
        z = np.asarray(z, dtype=float)
        if not self._kf_feat_initialized:
            self._kf_feat_x[:, 0] = z
            self._kf_feat_x[:, 1] = 0.0
            self._kf_feat_P = np.tile(np.eye(2) * 1.0, (4, 1, 1))
            self._kf_feat_prev_t = t
            self._kf_feat_initialized = True
            return
        dt = max(min(t - self._kf_feat_prev_t, 0.1), 1e-3)
        self._kf_feat_prev_t = t
        F = np.array([[1.0, dt], [0.0, 1.0]])
        Q = self._kf_feat_q * np.array([[dt**4 / 4.0, dt**3 / 2.0],
                                        [dt**3 / 2.0, dt**2]])
        R = self._kf_feat_r
        x_pred = self._kf_feat_x @ F.T                       # (4, 2)
        P_pred = F @ self._kf_feat_P @ F.T + Q               # (4, 2, 2)
        y = z - x_pred[:, 0]                                 # (4,)
        S = P_pred[:, 0, 0] + R                              # (4,)
        K = P_pred[:, :, 0] / S[:, None]                     # (4, 2)
        self._kf_feat_x = x_pred + K * y[:, None]
        self._kf_feat_P = P_pred - K[:, :, None] * P_pred[:, 0:1, :]

    def _marker_principal_angle(self, pts):
        """2pi-periodic marker orientation in the (level) V plane (raw, no offset).

        The weighted 2nd-moment principal axis 0.5*arctan2(2 mu11, mu20-mu02)
        gives only an AXIS (pi-period) — invariant under 180deg. We disambiguate
        it into a full 2pi DIRECTION using the weighted-centroid DISPLACEMENT: the
        [4,3,2,1] corner weights pull the centroid toward the high-weight (TL/TR)
        corners, so (weighted_centroid - geometric_centroid) is a 1st-moment
        vector that rotates 1:1 with the marker over a full turn. We flip the
        principal axis to whichever 180deg end aligns with that vector -> no
        pi-flip (verified to sweep a clean 360deg). Returns the raw angle (before
        the _moment_alpha_0 equilibrium offset) so callers apply their own offset.
        Used by _board_feature (per-marker, averaged) and the single-marker
        _getImgFeatures fallback.
        """
        x, y = pts[:, 0], pts[:, 1]
        if len(x) == 4:
            w = np.array([4.0, 3.0, 2.0, 1.0])
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
        # Disambiguate the pi-axis to a 2pi direction via the weighted-centroid
        # displacement (geometric centre -> weighted centre).
        dx = xc - float(np.mean(x))
        dy = yc - float(np.mean(y))
        if dx * dx + dy * dy > 1e-18:
            d = a - np.arctan2(dy, dx)
            if abs(np.arctan2(np.sin(d), np.cos(d))) > np.pi / 2:
                a += np.pi
        return float(np.arctan2(np.sin(a), np.cos(a)))   # wrap to (-pi, pi]

    def _board_corners(self, mid):
        """Board-plane coords (DIMENSIONLESS — normalised to marker-0's size,
        board centre = origin) of marker `mid`'s 4 corners, in cv2.aruco order
        [TL, TR, BR, BL]. Board +x = texture column (right), +y = texture row
        (down) — matching make_aruco_board.py. The layout carries NO metres: the
        findHomography fit that consumes these is scale-invariant, so the centroid
        is identical to any global unit; normalising to marker-0 = 1 makes the
        scale-freeness manifest (no marker physical size anywhere in the pipeline).
        """
        cx, cy, sz = self._board_layout[mid]
        h = sz / 2.0
        return np.array([[cx - h, cy - h],   # TL
                         [cx + h, cy - h],   # TR
                         [cx + h, cy + h],   # BR
                         [cx - h, cy + h]],  # BL
                        dtype=np.float64)

    def _board_feature(self, markers_V, size_factor=1.0):
        """Board centroid + yaw s=(xc,yc,1,alpha).

        markers_V : list of (id, 4x2 V-frame-normalized corners) for the markers
                    whose all-4 corners survived LK this frame.

        Centroid (xc,yc): the board centre (0,0) mapped through a board->V-frame
        homography fit from the known layout (Images/aruco_board_layout.npy) —
        stable and occlusion-robust as markers enter/leave the FoV.

        Yaw (alpha): the MOMENT yaw — the convention the yaw SMC is designed for
        (same as the single-marker _getImgFeatures fallback) — computed PER MARKER
        via _marker_principal_angle and AVERAGED. Every board marker is axis-
        aligned, so each marker's weighted 2nd-moment principal axis gives the
        same orientation (confirmed: <=1.6 deg spread across the 13 markers);
        averaging denoises it. Because board path and fallback now share the
        moment convention + offset, the board<->fallback switch no longer makes
        alpha jump. This REPLACES the previous GEOMETRIC board-+x yaw (arctan2 of
        the homography-mapped +x axis), a 2pi-period convention inconsistent with
        the moment yaw the control law expects — see git 886809d (introduced) /
        0008ba1 (geom-unify revert).

        Returns None if the homography fit is degenerate (caller falls back to
        single-marker _getImgFeatures). A single marker (4 pts) is the minimum
        for a homography (8 DOF); >=2 markers over-determine it.
        """
        board_pts, img_pts = [], []
        for mid, cV in markers_V:
            board_pts.append(self._board_corners(mid))
            img_pts.append(np.asarray(cV, dtype=np.float64) * size_factor)
        board_pts = np.vstack(board_pts)
        img_pts = np.vstack(img_pts)
        if len(board_pts) < 4:
            return None
        try:
            H, _ = cv2.findHomography(board_pts, img_pts, method=0)
        except Exception:
            H = None
        if H is None or not np.all(np.isfinite(H)):
            return None

        # Occlusion-robust centroid: board centre (0,0) -> V through H.
        v = H @ np.array([0.0, 0.0, 1.0])
        if abs(v[2]) < 1e-12:
            return None
        center = v[:2] / v[2]

        # Moment yaw: per-marker 2pi orientation (_marker_principal_angle),
        # circular-averaged. These are full 2pi DIRECTIONS (disambiguated), so a
        # NORMAL circular mean (not the doubled-angle axis mean). Same convention
        # + offset as the _getImgFeatures fallback, so the two alpha paths agree.
        raw = np.array([self._marker_principal_angle(np.asarray(cV, dtype=np.float64))
                        for _mid, cV in markers_V])
        avg_raw = np.arctan2(float(np.mean(np.sin(raw))), float(np.mean(np.cos(raw))))
        alpha_0 = self._moment_alpha_0
        alpha = float(np.arctan2(np.sin(avg_raw - alpha_0), np.cos(avg_raw - alpha_0)))
        return np.array([float(center[0]), float(center[1]), 1.0, alpha])

    def _getImgFeatures(self, pts):
        """
        pts : virtual feature points in normalized frame
            shape (N,2), already (x,y) = (u-cx)/fx, (v-cy)/fy.
            For ArUco markers, pts MUST be in cv2.aruco's intrinsic
            corner order [TL, TR, BR, BL] of the marker frame.

        Weighted-moment alpha (MATLAB image_feature.m, modified for SITL):

          MATLAB uses an asymmetric corner geometry (T_nP3 — one corner at
          (-20,+20) vs others (±15,±15)) so the moment-based alpha is yaw-
          observable. SITL uses a perfectly square ArUco; with uniform
          weights mu_11 ≡ 0 ∀ yaw → alpha undefined → yaw SMC blind.

          We synthesize the asymmetry via per-corner weights [4,3,2,1]
          tied to cv2.aruco's intrinsic corner labeling (TL=4, TR=3, BR=2,
          BL=1). The weights rotate WITH the marker frame, so alpha
          encodes drone-relative-to-marker yaw. Bias offset alpha_0
          recenters so alpha=0 corresponds to a stable equilibrium yaw.

          alpha_0 = -0.9379 was calibrated empirically from controller-
          start samples at PX4 yaw≈0. Equilibrium happens to be at PX4_yaw
          ≈ 0 in our world — this makes IC convergence's yaw=0 target
          coincide with the SMC equilibrium so no yaw rotation is needed
          during descent (minimizes lateral leak via PX4 mixer coupling).

          The raw 2nd-moment AXIS is π-period (mu_11, mu_20, mu_02 all
          invariant under 180° rotation), but _marker_principal_angle
          disambiguates it into a full 2π DIRECTION via the weighted-
          centroid displacement (see its docstring), so the alpha returned
          here sweeps a clean 360° with a SINGLE equilibrium at alpha=0 —
          the controller wraps the yaw error over the full 2π. (An
          asymmetric ArUco geometry, as in MATLAB, would make even the raw
          axis directional.)

          Falls back to uniform weighting (MATLAB-equivalent) when N != 4.
        """
        x = pts[:, 0]
        y = pts[:, 1]
        N = len(x)

        if N == 4:
            w = np.array([4.0, 3.0, 2.0, 1.0])
            # Board equilibrium offset (shared with the board path _board_feature so
            # the board<->single-marker fallback never jumps).
            alpha_0 = self._moment_alpha_0
        else:
            w = np.ones(N)
            alpha_0 = 0.0
        W = w.sum()

        # Weighted centroid (TL-biased) — the established SITL lateral feature.
        xc = float(np.sum(w * x) / W)
        yc = float(np.sum(w * y) / W)

        # Yaw: 2pi-disambiguated moment orientation (same convention/offset as the
        # board path _board_feature, so a board<->fallback switch never jumps).
        raw = self._marker_principal_angle(pts)
        alpha = float(np.arctan2(np.sin(raw - alpha_0), np.cos(raw - alpha_0)))

        # ---- 4. Feature vector (unnormalized) ----
        s = np.array([xc, yc, 1.0, alpha])

        self._img_feature_param.append(s)

    def _getVirtualPts(self, pts, quat):
        """Reproject camera-frame pixels onto the virtual image plane V.

        V is the MATLAB `I_R_V = rotz(yaw)` frame: a LEVEL frame
        (gravity-aligned z) that preserves the UAV's yaw heading. Roll and
        pitch are removed by aligning V's z-axis with world-down; yaw is
        carried over via the camera-y axis used to construct V's x-axis.

        2026-06-01: fixed sign-of-rotation bug — `g` was previously
        `R @ [0,0,1]`, but AHRS' `Quaternion.to_DCM()` returns the
        body→NED DCM, so `R @ [0,0,1]` gives `body_z_in_NED` (a NED
        vector), NOT `world_down_in_body`. For level drone both are
        `(0,0,1)`, hiding the bug. For tilted drone the V-frame then
        amplified the tilt-induced apparent marker offset instead of
        cancelling it — visible during yaw phase (drone tilts ±10° to
        track position-hold while yawing) as low-correlation oscillation
        in cal_s vs GT_s. Correct form: `R.T @ [0,0,1]` = NED→body @
        NED-down = world-down in body, which is what the V-frame
        construction needs.
        """
        R = Quaternion([quat.w, quat.x, quat.y, quat.z]).to_DCM()
        # World-down in the camera frame. Camera axes = body-FRD (aligned), so this
        # is R.T @ [0,0,1] = world-down in body. NOTE: the *virtual* frame V is the
        # LEVELED camera frame, so R_V_from_body is the roll/pitch leveling rotation
        # built below — NOT identity. ("camera = body" is the axis alignment, which
        # is what justifies using the body quaternion here.)
        g = R.T @ np.array([0, 0, 1])

        # Build V's basis in camera coords: z_V along gravity (no roll/pitch),
        # x_V from camera-y × z_V so the heading is set by the camera-y axis
        # (which is rigidly aligned with the UAV body — hence V inherits the
        # UAV's yaw).
        z_axis = g / np.linalg.norm(g)
        x_axis = np.cross([0, 1, 0], z_axis)
        x_axis /= np.linalg.norm(x_axis)
        y_axis = np.cross(z_axis, x_axis)

        # Rotation matrix from V to camera (columns = V-axes in camera coords).
        C_R_V = np.column_stack([x_axis, y_axis, z_axis])

        # Normalize pixel coords to camera-frame rays (z = 1 plane).
        cx, cy = self.center
        x = (pts[:, 0] - cx) / fx
        y = (pts[:, 1] - cy) / fy
        rays = np.column_stack([x, y, np.ones_like(x)])              # (N × 3)

        # Rotate rays into V: V_ray = (V_R_C) · C_ray = C_R_V.T · C_ray.
        # Equivalent np-broadcast form: rays @ C_R_V.
        V_rays = rays @ C_R_V                                        # (N × 3)

        # Reproject onto V's image plane (perspective divide by depth-in-V).
        z_v = V_rays[:, 2]
        return np.column_stack([V_rays[:, 0] / z_v, V_rays[:, 1] / z_v])

    def _vframe_w(self, w_body, quat):
        """Rotate a body-FRD angular velocity into the virtual (gravity-leveled) frame, using the
        SAME C_R_V basis as _getVirtualPts. w_V = V_R_C·w_body = C_R_V.T·w_body. For the
        centroid-rate observer's L_w·w rotation compensation (gyro -> V-frame)."""
        R = Quaternion([quat.w, quat.x, quat.y, quat.z]).to_DCM()
        g = R.T @ np.array([0, 0, 1.0])
        z_axis = g / np.linalg.norm(g)
        x_axis = np.cross([0, 1, 0], z_axis); x_axis /= np.linalg.norm(x_axis)
        y_axis = np.cross(z_axis, x_axis)
        C_R_V = np.column_stack([x_axis, y_axis, z_axis])
        return C_R_V.T @ np.asarray(w_body, float)

    def _getRealPtsFromV(self, V_pts, quat):
        """INVERSE of _getVirtualPts: project V-frame (gravity-leveled) NORMALIZED image points
        back to REAL camera pixels for the current tilt. Used to seed the virtual-plane ring into
        the real image each frame so LK tracks the NADIR patch regardless of tilt. Round-trip with
        _getVirtualPts is exact to machine precision (verified)."""
        R = Quaternion([quat.w, quat.x, quat.y, quat.z]).to_DCM()
        g = R.T @ np.array([0, 0, 1])
        z_axis = g / np.linalg.norm(g)
        x_axis = np.cross([0, 1, 0], z_axis); x_axis /= np.linalg.norm(x_axis)
        y_axis = np.cross(z_axis, x_axis)
        C_R_V = np.column_stack([x_axis, y_axis, z_axis])
        V_rays = np.column_stack([V_pts[:, 0], V_pts[:, 1], np.ones(len(V_pts))])
        C_rays = V_rays @ C_R_V.T                                            # rotate V rays -> camera rays
        cx, cy = self.center
        xc = C_rays[:, 0] / C_rays[:, 2]; yc = C_rays[:, 1] / C_rays[:, 2]
        return np.column_stack([xc * fx + cx, yc * fy + cy]).astype(np.float32)
    
    def _showOptFlow(self, img, C_pts, V_nP_norm):
        # 1. Ensure pixel coordinates for both frames are int32
        C_pts = [p.astype(np.int32) for p in C_pts]

        # 2. Convert normalized → pixel coordinates
        V_pts = [(pts * self.focal + self.center).astype(np.int32) for pts in V_nP_norm]

        # 3. Draw real flow (RED)
        for old, new in zip(C_pts[0], C_pts[1]):
            img = cv2.arrowedLine(img, old, new, (0,0,255), 2)

        # 4. Draw real marker polygon (GREEN)
        cv2.polylines(img, [C_pts[1]], isClosed=True, color=(0,255,0), thickness=2)

        # 5. Draw virtual flow (GREEN)
        for old, new in zip(V_pts[0], V_pts[1]):
            img = cv2.arrowedLine(img, old, new, (0,255,0), 2)

        # 6. Draw virtual marker polygon (RED)
        cv2.polylines(img, [V_pts[1]], isClosed=True, color=(0,0,255), thickness=2)


    def getLogData(self):
        return {
            "Time": self._time_log,
            "Image Feature Pts": self._feature_pts,
            "Virtual Feature Pts": self._virtual_feature_pts,
            "Feature Params": self._img_feature_param,
            "Opt Flow Ang Vel": self._opt_flow_ang_vel_raw,
            "IMU AngVel": self._imu_angvel_raw,
            "Quat": self._quat_log,
            "N Flow Corners": self._n_flow_corners,
            "Ring Opt Flow Ang Vel": self._ring_opt_flow_log,   # texture-free ring V-frame flow (V_v_ring), per frame
            "Ring Divergence": self._ring_div_log,              # pure depth-independent loom (safety-net vertical)
            "Ring Moment": self._ring_moment_log,               # ring area-rate loom (live A/B vs divergence)
            "Ring Opt Flow KF": self._ring_opt_flow_kf_log,     # ring flow through the corner-flow KF
            "N Ring Corners": self._n_ring_corners,
            "Opt Flow KF": self._opt_flow_kf_log,
            "Opt Flow Savgol": self._opt_flow_savgol_log,
            "Opt Flow Fused": self._opt_flow_fused_log,   # corner+ring EKF target-rel [h_tr;w] (FLOW_FUSE_RING=1)
            "Target Vel": self._target_vel_log,           # EKF target/rover velocity h_tv (flow units)
            "FPS": self._fps_log,
            "Image Stamp": self._stamp_log
        }
    
    def getParams(self):
        parameter = f"{{'Capture Rate':{self._capRate}, 'resolution':{self._resolution}}}"
        return parameter
    
    def getRawImgFeatureParam(self):
        """Image feature vector BEFORE _sensor_cal_s. Used by output_calibration."""
        if len(self._img_feature_param) == 0:
            return np.zeros(4)
        return np.array(self._img_feature_param[-1])

    def getRawOptFlowAngVel(self):
        """Raw 6-vector LSTSQ output `[h; w]` BEFORE `_sensor_cal_hw`.

        Legacy field name kept for back-compat with existing recordings. In
        the PLASMC manuscript terminology this is **NOT** optical flow:

            ṡ = L(s) · [h; w]              ← optical flow ṡ comes from L·[h;w]
            s = virtual image position     (image corner coords)
            ṡ = optical flow               (= r̂̇)
            h = virtual image velocity     (translation part of V_input)
            w = virtual image angular vel  (rotation part of V_input)

        The 6-vector this method returns is `[h_raw; w_raw]` — the LSTSQ
        solution that, fed through L at each corner, would reproduce the
        measured per-corner pixel velocity (= the actual optical flow).
        Used by `record_output_calibration.py` to derive `_sensor_cal_hw`.
        """
        if len(self._opt_flow_ang_vel_raw) == 0:
            return np.zeros(6)
        return np.array(self._opt_flow_ang_vel_raw[-1])

    def getRawRingFlowAngVel(self):
        """Raw 6-vector `[h; w]` from the TEXTURE-FREE ring lstsq, BEFORE any
        calibration (the runtime applies none to the ring). Mirrors
        getRawOptFlowAngVel for the ring sampler so record_output_calibration.py can
        co-sample it into the GT dict and derive_ring_cal.py can fit M_ring
        (GT[h;w] = M_ring @ ring_raw), exactly as the corner cal is derived.
        Returns the latest per-frame V_v_ring (the same value logged to
        'Ring Opt Flow Ang Vel'). Zeros if ring logging is off / no sample yet.
        """
        if len(self._ring_opt_flow_log) == 0:
            return np.zeros(6)
        return np.array(self._ring_opt_flow_log[-1])

    def getRingFlowAngVel(self):
        """Calibrated texture-free ring flow `[h; w]` (6-vec): the ring analogue
        of getOptFlowAngVel — `_sensor_cal_ring @ ring-KF state`. SAFETY-NET
        signal (control consumes the corner flow); `_sensor_cal_ring` is identity
        until the co-sampled M_ring is derived (tools/derive_ring_cal.py).
        """
        return self._sensor_cal_ring @ self._kf_x_ring[:, 0]

    def _compute_savgol_output(self):
        """Latest Savgol(FILTER_WIN, FILTER_POLYORDER) sample, matching the
        legacy non-causal sliding-window pattern (returns sgf_buf[win/2 + 1]).
        Returns the PRE-calibration 6-vec; getOptFlowAngVel / log buffers
        apply _sensor_cal_hw.
        """
        if len(self._opt_flow_ang_vel_raw) == 0:
            return np.zeros(6)
        if len(self._opt_flow_ang_vel_raw) >= FILTER_WIN:
            sgf_buf = sgf(self._opt_flow_ang_vel_raw[-FILTER_WIN:],
                          FILTER_WIN, FILTER_POLYORDER, axis=0)
            return sgf_buf[int(FILTER_WIN / 2 + 1)]
        return np.mean(self._opt_flow_ang_vel_raw, axis=0)

    def getOptFlowAngVel(self):
        """Calibrated, smoothed optical flow + angular velocity (6-vec).

        Both filters' outputs are computed and logged every frame; this
        getter just selects which one the controller sees via IMG_FILTER:
          'kf'     (default) → 2-state constant-velocity Kalman.
          'savgol'           → legacy Savgol(13, 1).
        FLOW_FUSE_RING=1 overrides both: the corner+ring fusion EKF returns the
        TARGET-relative flow [h_tr; w] (works for stationary AND moving targets).
        Its state is already calibrated, so return as-is.
        """
        if self._fuse_ring and self._ekf_init:
            return np.concatenate([self._ekf_x[0:3], self._ekf_x[6:9]])   # corner-moment loom fed via _corner_cal[2]
        if os.environ.get('IMG_FILTER', 'kf') == 'savgol':
            _out = self._sensor_cal_hw @ self._compute_savgol_output()
            if self._loom_decouple:
                _out[2] = self._compute_savgol_output()[2]   # loom = raw vz/Z, bypass cal
            if self._loom_sign_guard:
                _out[2] = min(_out[2], 0.0)                  # positive-loom sign guard (non-fuse path)
            return _out
        _out = self._sensor_cal_hw @ self._kf_x[:, 0]
        if self._loom_decouple:
            _out[2] = self._kf_x[2, 0]                       # loom already vz/Z, bypass cal row 2
        if self._loom_sign_guard:
            _out[2] = min(_out[2], 0.0)                      # positive-loom sign guard (non-fuse path)
        return _out

    def getImgFeatureParam(self):
        """Calibrated, KF-smoothed image-feature vector (4-vec).

        IMG_FEATURE_FILTER selects the OUTER-loop centroid filter:
          'kf'     (default since 2026-06-06) → 2-state constant-velocity Kalman.
          'savgol'                            → legacy Savgol(13, 1).
        Switched to KF by default: savgol(13) adds ~110 ms group delay on the
        centroid and is ~2x noisier than the KF; the KF cuts both. (Both outputs
        are logged every frame for A/B.)
        """
        if len(self._img_feature_param) == 0:
            return np.zeros(4)
        # KF path (default; IMG_FEATURE_FILTER=savgol selects the legacy filter).
        # Step the centroid KF once per fresh raw sample (the controller calls
        # this getter every control iteration, possibly faster than the camera).
        if os.environ.get('IMG_FEATURE_FILTER', 'kf') != 'savgol':
            n = len(self._img_feature_param)
            if n != self._kf_feat_last_n:
                self._kf_feat_last_n = n
                self._kf_feat_update(self._img_feature_param[-1],
                                     self._time.perf_counter())
            if self._kf_feat_initialized:
                return self._sensor_cal_s @ self._kf_feat_x[:, 0]
        if len(self._img_feature_param) >= FILTER_WIN:
            sgf_buf = sgf(self._img_feature_param[-FILTER_WIN:],
                          FILTER_WIN, FILTER_POLYORDER, axis=0)
            return self._sensor_cal_s @ sgf_buf[int(FILTER_WIN / 2 + 1)]
        return self._sensor_cal_s @ np.mean(self._img_feature_param, axis=0)