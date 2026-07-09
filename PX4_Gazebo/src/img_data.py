# **************************************************************************
# Changed class and node name used in gz_subscriber
# Used simulator time instead of system time
# Used virtual image feature points for optical flow calculation
# Detect nested Aruco markers; PRIMARY = the BIGGEST detected marker (IMG_MARKER_PRIORITY=big, 2026-07-03) for observability; falls to the small marker only when the big is gone.
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
        # OBSERVER cal (2026-07-03): 1m NESTED marker + centroid-rate observer (PLASMC_CENTROID_RATE=1).
        # h_x/h_y = reduced-solve std-ratio on the OBSERVER lateral flow (median of 5 runs) — recovers
        # the lateral channel the σ_min corner-flow left at the noise floor (old NaN/unstable betas).
        self._sensor_cal_hw = np.array([
            [+1.4272, +0.0000, +0.0000, +0.0000, +0.0000, +0.0000],   # h_x = observer beta_x
            [+0.0000, +1.0253, +0.0000, +0.0000, +0.0000, +0.0000],   # h_y = observer beta_y
            [+0.0535, -0.0044, +0.4973, +0.0000, +0.0000, +0.0000],   # loom row (board-fit; control uses MOMENT loom, secondary)
            [+0.0000, +0.0000, +0.0000, +0.0000, +0.0000, +0.0000],
            [+0.0000, +0.0000, +0.0000, +0.0000, +0.0000, +0.0000],
            [+0.0000, +0.0000, +0.0000, +0.0000, +0.0000, +0.8439]])   # w_z decoupled to w_z_raw only
        self._sensor_cal_hw[2, 2] = float(os.environ.get("PLASMC_LOOM_CAL", str(self._sensor_cal_hw[2, 2])))  # A/B knob: default = baked 1.0744; PLASMC_LOOM_CAL=1.2988 re-applies the recal
        if os.environ.get("PLASMC_WZ_CROSS", "0") == "1":   # restore the full old w_z cross-coupling (A/B)
            self._sensor_cal_hw[5, 0:5] = [+0.0526, +1.0862, -0.0096, -0.7395, +0.0161]
        self._sensor_cal_s  = np.diag([1.1391, 1.1437, 1.0, 1.0])   # observer/1m-nested centroid (2026-07-03)

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
        # OBSERVER ring cal (2026-07-03, 5 runs): h-block R² 0.72/0.82/0.66 (up from 0.34). The ring-yaw
        # (Wz) row is ZEROED — its derived gain was a 9.6 runaway (ring sees yaw weakly); the corner
        # marker provides yaw. Re-derive Wz if a board/turning case needs ring yaw.
        self._sensor_cal_ring = np.array([
            [+0.6523, +0.0317, +0.1372, -0.0371, +0.6014, -0.1252],
            [-0.1414, +0.7969, -0.0414, -0.8080, -0.1262, +0.3377],
            [+0.1154, +0.0131, +2.0499, -0.1148, +0.1176, +0.4799],
            [+0.0000, +0.0000, +0.0000, +0.0000, +0.0000, +0.0000],
            [+0.0000, +0.0000, +0.0000, +0.0000, +0.0000, +0.0000],
            [+0.0000, +0.0000, +0.0000, +0.0000, +0.0000, +0.0000]])

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
        # DECODE-CORRESPONDENCE flow fallback (2026-07-04, user): LK is sub-pixel-accurate at LOW
        # speed but loses the marker corners at HIGH speed (it iterates from the old position and
        # can't reach the new one). ArUco decode re-finds the corners GLOBALLY in the next frame in
        # canonical order -> valid correspondence with no basin. So when LK DROPS marker corners,
        # fill them from decode: decode's ~1px corner noise is negligible vs the large high-speed
        # displacement (signal >> noise), exactly the regime LK can't handle. LK stays primary where
        # it succeeds (low speed). The extra detectMarkers only runs when LK actually failed.
        self._decode_flow = os.environ.get("FLOW_DECODE_CORR", "1") == "1"   # BAKED default-on 2026-07-04 (user); FLOW_DECODE_CORR=0 to disable
        # Decode-flow scale cal: the decode displacement over-reports ~1.4x vs GT/LK at high speed
        # (decode corner = marker edge, slightly wider than the LK feature point). Scale the decode
        # DISPLACEMENT so the reconstructed flow matches truth. Measured from perc_diag high-vel bin.
        self._decode_scale = float(os.environ.get("FLOW_DECODE_SCALE", "1.0"))
        # LK-STUCK trigger: LK reports status=1 even when it converges to a wrong nearby minimum
        # (tiny displacement) at high speed, so status==0 never catches it. Fire the decode fallback
        # when decode sees materially MORE motion than LK reports (px). Handles the silent-stuck case.
        self._decode_trig = float(os.environ.get("FLOW_DECODE_TRIG", "1.5"))
        self._decode_fills = 0

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
        # ARM MASK (2026-07-07, user): the body-mounted down-camera sees the drone's OWN arms/props
        # as fixed STATIC bands (top y<~155, bottom y>~445 in the 480x640 raw image; measured from
        # video temporal-std). Ring stations landing there have ~0 optic flow -> they drag pure_div
        # toward zero + inject noise. Drop them by a FIXED raw-y band (robust to the ring's tilt
        # re-projection, since the arms are camera-fixed, not scene-fixed). BAKED default-ON 2026-07-07:
        # validated ~3x terminal-band noise cut (pure_div std 2.27->0.67, moment 1.03->0.60) and
        # sign-flip cut (51%->21% / 45%->11% positive), no observed downside (ring flow itself is only
        # consumed when ring-commit/loom-ring are explicitly on, both default OFF — see
        # [[feedback_ring_loom_hz_terminal_deadend]]). Y-limits tunable.
        self._ring_arm_mask = os.environ.get("PLASMC_RING_ARM_MASK", "1") == "1"
        self._arm_y_top = float(os.environ.get("RING_ARM_Y_TOP", "155"))
        self._arm_y_bot = float(os.environ.get("RING_ARM_Y_BOT", "445"))
        # PAIRED-STATION SYMMETRY (2026-07-07, user): global index of each station's 180deg-opposite
        # partner (same radius block, angle i <-> i+npts/2). pure_div/moment cancel lateral translation
        # ONLY across complete opposite pairs -> keep a station iff its partner also survives.
        self._ring_paired = os.environ.get("PLASMC_RING_PAIRED", "1") == "1"
        _opp = np.concatenate([_b * _ring_npts + (np.arange(_ring_npts) + _ring_npts // 2) % _ring_npts
                               for _b in range(len(_ring_radii))]).astype(int)
        self._ring_opp_idx = _opp
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

        # ONLINE BOARD SELF-CALIBRATION (2026-07-03, user directive: no hardcoded layout/IDs/sizes).
        # Learn each marker's relative (cx,cy,sz) from co-visible frames instead of the
        # aruco_board_layout.npy file. Feasible for a BOARD (markers co-visible) — unlike concentric
        # nested (mutually exclusive). Marker IDs are used only as runtime correspondence KEYS, not
        # hardcoded. The first multi-marker frame bootstraps the layout (scale-free: gauge = median
        # marker size, origin = marker centroid); later frames are aligned to it via a similarity
        # (Umeyama) on shared markers and running-averaged, accepting only low-residual (well-leveled,
        # un-occluded) frames. Until ready (>= min_frames) the board feature falls back to the file
        # (if present) else single-marker. Default ON; the file becomes an optional prior.
        self._board_selfcal = os.environ.get("BOARD_SELFCAL", "1") == "1"
        self._selfcal_layout = {}     # {id: (cx, cy, sz)} learned, scale-free (marker-arrangement gauge)
        self._selfcal_counts = {}     # {id: n} running-average weights
        self._selfcal_frames = 0      # reliable frames accumulated
        self._selfcal_min_frames = int(os.environ.get("BOARD_SELFCAL_MIN_FRAMES", "5"))
        self._selfcal_res_max = float(os.environ.get("BOARD_SELFCAL_RES_MAX", "0.15"))  # similarity RMS gate (frac of scale)

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
        self._last_drifted_off = False   # last-known: marker left the FoV OFF-CENTER (drift), vs SPANNING (overflow, still over target)
        self._last_overflow = False      # companion: marker SPANNED (overflow) rather than drifted off one side
        self._drift_off_hist = []        # per-frame log of _last_drifted_off (2026-07-07: failure-cause tagging)
        self._overflow_hist = []         # per-frame log of _last_overflow

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
        # RELAXED KLT-FALLBACK GATE (2026-07-09, default-OFF pending n>=5 validation): the
        # strict all-4-corners-tracked requirement discards an otherwise-good 3/4-tracked frame
        # outright. KLT Diag logging (added same day) showed a "momentary flicker" regime
        # (single corner losing LK lock for 1-2 frames during otherwise-clean tracking) where
        # 100% of gate4-rejections were exactly 3/4 tracked -- but that's n=1, and this changes
        # CONTROL-AFFECTING behavior (which corners feed s/flow), so it stays env-gated until
        # validated at n>=5 rather than baked on this single run. MARKER_KLT_RELAX_GATE=1 to test.
        self._klt_relax_gate = os.environ.get("MARKER_KLT_RELAX_GATE", "0") == "1"
        self._klt_min_tracked = 3 if self._klt_relax_gate else 4
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
        # DENSE-HOMOGRAPHY RECOVERY (2026-07-07, user): when the strict all-4-primary-corner gate
        # fails (partial visibility/occlusion), recover the FULL quad via a RANSAC homography fit
        # from the marker's dense CANONICAL point layout (scaled-quad, deterministic, ~180 pts) to
        # their CURRENTLY LK-tracked positions, then map the canonical 4 corners through that
        # homography. Unlike averaging surviving points (biased toward whatever fraction is visible),
        # a homography recovers the FULL marker geometry from a partial view, because every dense
        # point has a KNOWN canonical position relative to the whole quad (planarity is the only
        # assumption -- true for a flat ArUco marker; needs no depth/scale -- a homography is a pure
        # 2D pixel-to-pixel projective fit). Re-anchored on every clean 4-corner detection (never
        # goes far stale); tracked independently of the strict gate so it survives partial dropouts
        # the strict gate alone would kill. Falls through to the (self-reinforcing, buggy)
        # deg-1 s-extrapolation only if even this dense set can't sustain enough survivors.
        self._dense_recover = os.environ.get("PLASMC_DENSE_RECOVER", "0") == "1"
        self._dense_recover_min_pts = int(os.environ.get("DENSE_RECOVER_MIN_PTS", "12"))
        self._dense_recover_ransac_px = float(os.environ.get("DENSE_RECOVER_RANSAC_PX", "3.0"))
        self._dense_canon_pts = None       # (M,2) canonical dense-point layout, re-anchored per clean detection
        self._dense_canon_quad = None      # (4,2) canonical primary-corner layout (same anchor frame)
        self._dense_track_pts = None       # (M,2) CURRENT LK-tracked positions of the canonical points
        self._dense_ref_img = None         # frame the dense tracking was last stepped from
        self._dense_recover_active = False # True when the last frame's aruco_pts_0 came from recovery (diagnostic)
        # UNIFIED STALENESS GATE (2026-07-07, user): the corner-only KLT fallback (elif branch,
        # MARKER_KLT_MAX_STEPS=20) and dense-recovery were two DISJOINT, uncoordinated LK-chain
        # trackers with inconsistent trust bounds (20-frame hard cap vs unbounded) that never
        # informed each other -- structurally wrong (same underlying drift-accumulation risk,
        # arbitrarily different tolerances). Fix: (1) the corner-fallback's SHORT successes
        # (still fresh, low accumulated drift) now ALSO soft-re-anchor the dense canonical state
        # (not just a full fresh decode), so dense-recovery gets refreshed far more often; (2)
        # dense-recovery itself gets a PRINCIPLED staleness cap -- frames-since-last-anchor AND a
        # minimum RANSAC inlier ratio -- replacing its previous unbounded runtime (which let it
        # chase a 30+ second drift in the densedbg batch).
        self._dense_frames_since_anchor = 0
        self._dense_recover_max_frames = int(os.environ.get("DENSE_RECOVER_MAX_FRAMES", "60"))
        self._dense_recover_min_inlier_frac = float(os.environ.get("DENSE_RECOVER_MIN_INLIER_FRAC", "0.5"))
        self._dense_soft_anchor_max_steps = int(os.environ.get("DENSE_SOFT_ANCHOR_MAX_STEPS", "2"))
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
        self._primary_id = None   # primary decoded marker; for size-normalizing the loom scale M
        # TERMINAL RING-COMMIT (2026-07-04, project_terminal_velocity_handover_design). HANDOVER_LATCHED
        # fires (one-way) on a large NEGATIVE d(ln RAW_M) — the RAW apparent-size² M drops ~ratio² (~113x)
        # at the big->small switch. ID/SIZE-FREE + image-space (NO marker-ID, NO physical size, NO
        # board_layout): unmistakable vs the ~0.04 normal-descent loom. RING_LOOM = the marker-less ring
        # loom (calibrated pure_div * scale) the controller swaps h_z onto once its gate (handover +
        # centered + settled) fires. Both consumed by controller.py PLASMC_TERMINAL_RING_COMMIT.
        self.HANDOVER_LATCHED = False
        self.OVER_TARGET = False                                             # nadir (V-frame origin) inside the primary marker quad = we are over the target
        self.RING_LOOM = 0.0
        self.RING_N_STATIONS = 0; self.RING_DIV_RAW = float('nan'); self.RING_MOMENT_RAW = float('nan')
        self._ring_loom_scale = float(os.environ.get("RING_LOOM_SCALE", "1.0"))   # deck over-report correction (~0.75; re-pin on nested-marker GT-FB)
        self._ring_loom_source = os.environ.get("RING_LOOM_SOURCE", "div")         # "div" (pure_div) or "moment" (arm-masked ring MOMENT — cleaner at the deck: std 0.60 vs 0.67, 11% vs 21% sign-flip)
        self._raw_lnM_prev = None
        self._handover_ln = float(os.environ.get("HANDOVER_DLOGM", "1.0"))   # |d(ln raw_M)| latch threshold (≫0.04 loom, ≪4.7 handover)
        self._handover_cand = 0                                              # frames the small-primary has PERSISTED since a big->small drop
        self._handover_persist = int(os.environ.get("HANDOVER_PERSIST_FRAMES", "5"))  # require the drop SUSTAINED (secondary filter)
        # OVERFLOW SIGNATURE (2026-07-05, corrected): gate on the SMALL (post-drop) marker subtending a
        # large-enough ANGLE = we are CLOSE (real terminal). At the terminal the surviving small marker
        # fills a good fraction of the FoV (normalized-bearing scatter M~0.08); at altitude the big
        # tilt-fails while the small is TINY (M~0.0001) -> ~800x separation, far cleaner than the big's
        # pre-drop M (0.15 vs 0.4, which overlapped). Image-space, scale-free w.r.t. physical size (fires
        # at a fixed angular subtense regardless of marker size/Z). Rejects the altitude tilt-failures.
        self._handover_min_m = float(os.environ.get("HANDOVER_MIN_SMALL_M", "0.01"))
        self._handover_min_ln = float(np.log(max(self._handover_min_m, 1e-9)))
        self._handover_dbg = os.environ.get("HANDOVER_DBG", "0") == "1"
        # Marker priority (2026-07-03, user): which marker is the PRIMARY when >1 nested marker
        # decodes. 'big' = the LARGEST (max layout size) — keep using the big marker (ID10) as long
        # as it's detectable, fall to the small (ID0) only when the big is gone. The big marker has
        # larger corner spread -> better flow OBSERVABILITY. 'small' = legacy smallest-ID (argmin),
        # which preferred the tiny inner marker -> rank-deficient flow / the observability issues.
        # Concentric markers share the board centre, so switching the primary never moves the centroid.
        self._marker_priority = os.environ.get("IMG_MARKER_PRIORITY", "big").strip().lower()
        # FLOW_LOOM_WIN = causal linear-fit window (frames). Offline on a centered descent the
        # CAUSAL deque-fit vs GT loom: WIN=5 corr 0.69, WIN=9 corr 0.93, WIN=13 corr 0.97 (rmse
        # ~0.06; joint-lstsq was 0.16/0.88). 9 balances accuracy vs lag (~0.15s @ 60fps).
        self._mtrace_hist = deque(maxlen=int(os.environ.get("FLOW_LOOM_WIN", "9")))  # (t, ln M)
        # d(lnM)/dt OUTLIER-HOLD on the loom (2026-07-04, user: parallel to the ds/dh checks). The
        # big<->small handover drops the tracked apparent size M abnormally fast (~7%/frame vs ~1.5%
        # for a real descent), corrupting the loom = -½·d(lnM)/dt into a spurious WRONG-SIGN spike
        # (+2.06 vs GT -0.24, GT-FB-confirmed). If the per-frame |Δln M| exceeds a bound it's a
        # handover/marker-leaving transient -> HOLD the loom at last-good + drop the M-history so the
        # resumed slope never spans the size transition.
        self._loom_lnM_prev = None
        self._loom_hold = 0.0
        self._loom_dlnM_max = float(os.environ.get("LOOM_DLNM_MAX", "0.04"))   # loom CHECKPOST: reject a per-frame |Δln M| spike for that instant only (0=off)
        # BOTH-VISIBLE ratio-normalization (2026-07-04, user): when 2+ markers decode in the SAME
        # frame they share Z, so sz_i/sz_j is the physical-size ratio (Z-independent) — learn it
        # online (anchor the primary at 1.0, propagate via co-visibility, running-avg). Normalizing
        # the loom size M by sz_ratio² makes M CONTINUOUS across the big<->small handover (both →
        # (f/Z)²), so the loom stays LIVE and accurate through the switch (vs the d(lnM)/dt hold
        # which only freezes it). Marker-agnostic — no layout/ID/size prior. Falls back to the hold
        # when only one marker is visible (marker-leaving) or the ratio isn't learned yet.
        self._sz_ratio = {}                                                   # mid -> physical-size ratio vs anchor
        self._sz_ratio_a = float(os.environ.get("LOOM_SZ_RATIO_ALPHA", "0.1"))  # running-avg rate
        self._loom_sz_ratio_on = os.environ.get("LOOM_SZ_RATIO", "1") == "1"   # BAKED default-on 2026-07-04 (user); LOOM_SZ_RATIO=0 to disable

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
        # REVIVED default-ON 2026-07-03 for the 1m NESTED marker: the BIG marker's σ_min corner-flow
        # sits at the LK noise floor (recal std-ratio betas came out NaN/unstable → 15-24m fly-aways).
        # The centroid-rate observer recovers a derivable lateral signal; with the MATCHED observer cal
        # (h_x/h_y = 1.43/1.03) IC1 fly-away 15-24m → 0.41m (n=1). The 2026-06-23 dead-end above was the
        # OLD ~1m single marker where lateral was NOT starved — different regime. Set =0 to disable.
        self._centroid_rate = os.environ.get("PLASMC_CENTROID_RATE", "1") == "1"
        self._centroid_hist = deque(maxlen=int(os.environ.get("CENTROID_RATE_WIN", "9")))  # (t, x0, y0, ln M)
        self._observer_flow = np.zeros(6)   # [h_x, h_y, h_z, 0, 0, w_z] from the observer
        self._observer_valid = False        # reset per-frame; True when the observer produced flow
        # Constant-velocity KALMAN FILTER on the decoded centroid -> smoothed lateral velocity
        # (2026-07-03). Replaces the raw polyfit differentiation, which amplified sub-pixel centroid
        # jitter into flow noise ~30x the true lateral flow (corr with GT ~0 -> residual drift).
        # Offline vs GT: tracks real motion (|corr| .13->.67) AND cuts noise 3x. q,r tuned offline.
        self._obs_kf_x = None; self._obs_kf_y = None      # [pos, vel] states
        self._obs_kf_Px = None; self._obs_kf_Py = None    # 2x2 covariances
        self._obs_kf_t = None                             # last update stamp (for dt)
        self._obs_kf_q = float(os.environ.get("CENTROID_RATE_KF_Q", "1e-3"))   # process noise. BAKED 1e-4->1e-3 (2026-07-04): the old 1e-4 over-smoothed the centroid-rate -> low-velocity attenuation (h_x ratio 0.66@mid). q=1e-3 lifts it to 0.90 (offline+live A/B) with no landing regression -- the low q was tuned for the PRE-FIX noisy observer; the corrected observer (frame/w_z fixes) affords more gain.
        self._obs_kf_r = float(os.environ.get("CENTROID_RATE_KF_R", "1e-3"))   # measurement noise
        # ds/dh OUTLIER GATE on the lateral flow (2026-07-04). The σ_min LK corner-flow (used when
        # the marker isn't freshly decoded) intermittently ramps to physically-impossible values
        # (|h|~1.6 = 6 m/s-equivalent at 3.8m) -> SMC divergence (the catastrophic fly-away tail).
        # The median flow is fine; only the fat tail is garbage -> reject per-frame OUTLIERS whose
        # frame-to-frame change exceeds a bound, HOLD last-good. RATE-based -> scale/depth-free
        # (genuine v/Z growth near touchdown is slow, ~0.02/frame; spikes step ~0.2/frame).
        self._flow_prev = None                                                 # detection reference = last RAW [h_x,h_y] (advances every frame)
        self._flow_hold = None                                                 # substitution = last ACCEPTED [h_x,h_y] (updated only on accept; no spike leak)
        self._flow_dh_max = float(os.environ.get("FLOW_DH_MAX", "0.15"))       # flow CHECKPOST: reject a per-frame |Δh| spike for that instant only (0=off)
        # ds OUTLIER-HOLD on the RAW centroid s (2026-07-04, user plan): the centroid is the ACCURATE
        # signal, so a frame whose centroid JUMPS beyond a physical rate (|Δs|>thresh) is a detection/
        # LK glitch -> reject it, HOLD the previous value. Cleans s AT THE SOURCE, so both the position
        # loop and the observer's centroid-rate get a clean input. Companion to the flow (dh) hold above.
        self._s_prev = None                                                    # detection reference = last RAW centroid [xc,yc] (advances)
        self._s_hold = None                                                    # substitution = last ACCEPTED centroid (updated only on accept)
        self._s_ds_max = float(os.environ.get("FLOW_DS_MAX", "0.15"))        # centroid CHECKPOST: reject a per-frame |Δs| spike for that instant only (0=off)

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
        self._z_v_log = []          # virtual-plane z-coordinate per frame (diagnose z_v→0 phantom-s)
        self._z_v_min_log = []      # min z_v per frame (track when reprojection risks singularity)
        # H-EXTRAPOLATION (2026-07-07, user; BAKED default-ON 2026-07-07): re-derived alternative to
        # the 2026-05-13 hard-zero policy. That decision was correct GIVEN its implementation (deg-1
        # fit self-referencing its own extrapolated output -> cascaded to 10^5+ outliers or pinned at
        # the clip ceiling forever) but the FIX is to correct the implementation, not abandon
        # extrapolation: (1) fit ONLY against REAL (non-extrapolated) samples via a separate history
        # buffer -- never self-reference; (2) DECAY the extrapolated velocity toward zero over
        # H_EXTRAP_DECAY_FRAMES consecutive misses, so a long gap converges to "no known motion"
        # (matching hard-zero's safety) instead of committing to an indefinitely-continued trend; (3)
        # exclude already-clipped real samples from the fit basis (a saturated sample's slope is
        # untrustworthy); (4) bound the result at H_EXTRAP_MAX (tighter than the general lstsq ±10
        # clamp). VALIDATED (A/B, IC2 x5 each, observer+DESCENT_GATE): baseline hard-zero had 1/5 bad
        # misses (GT xy 2.703m); h-extrap had 0/3 valid reps miss (GT xy 0.075-0.097m all); zero
        # fly-aways either leg. PLASMC_H_EXTRAP=0 to revert to hard-zero.
        self._h_extrap = os.environ.get("PLASMC_H_EXTRAP", "1") == "1"
        self._h_extrap_decay_frames = int(os.environ.get("H_EXTRAP_DECAY_FRAMES", "10"))
        self._h_extrap_max = float(os.environ.get("H_EXTRAP_MAX", "5.0"))
        self._h_extrap_clip_bound = float(os.environ.get("H_EXTRAP_CLIP_BOUND", "10.0"))  # matches the lstsq's own ±10 clamp -- samples AT this bound are excluded from the fit
        self._h_real_t = []      # timestamps of REAL (non-extrapolated) h samples only
        self._h_real_v = []      # the REAL h values themselves (never mixed with extrapolated output)
        self._img_feature_param_real_t = []  # timestamps of REAL (non-extrapolated) s samples only
        self._img_feature_param_real = []    # the REAL s values themselves (never mixed with extrapolated output)
        self._imu_angvel_raw = []   # IMU body rate (FRD) [fwd,right,down], synced to the flow log
        self._quat_log = []         # FC quat [w,x,y,z], synced to the flow log (for IMU->V transform)
        self._n_flow_corners = []   # # corners fed to the lstsq per frame (board diag)
        # KLT-fallback diagnostic (2026-07-09): validates the hypothesis that the strict
        # `n_tracked == 4` gate (img_data.py, KLT-fallback elif branch) is the dominant cause
        # of momentary (1-2 frame) decode gaps -- a single corner losing LK lock for one step
        # currently discards an otherwise-good 3/4-tracked frame entirely. One entry per frame
        # where ArUco decode failed AND a KLT-fallback attempt was actually made (cv2.calcOpticalFlowPyrLK
        # ran) -- (n_tracked, gate4_passed, in_bounds_if_gate_passed, accepted). Empty entries
        # (no attempt this frame -- e.g. cap exhausted, no prior corners) are NOT logged here, only
        # actual attempts, so len() != total frame count -- correlate by "t".
        self._klt_diag_log = []
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
        zero = (np.zeros(6), np.nan, 0, np.nan)   # FIX 2026-07-07: pure_div was 0.0 (a FAKE "stationary" reading
        # the controller's isfinite(pdiv) check then treated as real data) -> NaN so degenerate/too-few-station
        # frames are correctly rejected downstream instead of injecting a fabricated zero-descent step.
        if (imgs is None or imgs[0] is None or imgs[1] is None
                or quats is None or len(quats) < 2 or quats[0] is None or quats[1] is None):
            return zero
        try:
            g0 = imgs[0] if imgs[0].ndim == 2 else cv2.cvtColor(imgs[0], cv2.COLOR_BGR2GRAY)
            g1 = imgs[1] if imgs[1].ndim == 2 else cv2.cvtColor(imgs[1], cv2.COLOR_BGR2GRAY)
            # Re-project the leveled (V-frame) ring into the real image at the frame-0 tilt so LK
            # tracks the NADIR patch (uniform depth, tilt-invariant). The legacy fixed real-image ring
            # was removed 2026-06-24 — it sampled off-nadir under tilt (geometrically wrong).
            _seed = self._getRealPtsFromV(self._ring_pts0_V, quats[0])   # FULL ring (indices align with _ring_opp_idx)
            p1, st, _ = cv2.calcOpticalFlowPyrLK(g0, g1, _seed, None, **self._ring_lk_params)
            st = np.asarray(st).flatten().astype(bool)
            _sf = _seed.reshape(len(_seed), -1)          # (N,2) raw frame-0 station positions
            _p1f = p1.reshape(-1, 2)                      # (N,2) tracked frame-1 positions
            valid = st.copy()
            if self._ring_arm_mask:                      # drop the drone's own arm/prop bands (static -> 0 flow)
                valid &= (_sf[:, 1] > self._arm_y_top) & (_sf[:, 1] < self._arm_y_bot)
            fm = np.linalg.norm(_p1f - _sf, axis=1)       # per-station flow magnitude (ALL stations)
            if int(valid.sum()) >= 6:                     # MAD flow-magnitude outlier rejection (over valid)
                _m = np.median(fm[valid]); _d = np.median(np.abs(fm[valid] - _m)) + 1e-6
                valid &= fm < _m + 3.0 * 1.4826 * _d
            # PAIRED-STATION SYMMETRY: keep a station iff its 180deg-opposite also survived, so the
            # uniform-translation radial component cancels pairwise in pure_div/moment (else it leaks
            # -> loom sign-flip noise, worst during lateral drift = the terminal).
            if self._ring_paired:
                valid &= valid[self._ring_opp_idx]
            if int(valid.sum()) < 6:
                return (np.zeros(6), np.nan, int(valid.sum()), np.nan)   # NaN, not 0.0 (see `zero` sentinel above)
            r0 = _sf[valid].astype(np.float32)
            r1 = _p1f[valid].astype(np.float32)
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

    def _dense_recover_anchor(self, corners, img0):
        """Re-anchor the dense-homography-recovery canonical state on a CLEAN 4-corner
        detection: canonical dense layout + canonical quad = THIS frame's geometry (so
        canonical==tracked right now), ref image = img0. Called every clean detection so
        the canonical anchor never goes far stale."""
        if not self._dense_recover:
            return
        self._dense_canon_quad = np.asarray(corners, np.float32).reshape(-1, 2).copy()
        self._dense_canon_pts = self._scaled_quad_points(self._dense_canon_quad,
                                                           per_side=self._dense_pts_per_side)
        self._dense_track_pts = self._dense_canon_pts.copy()
        self._dense_ref_img = img0.copy()
        self._dense_recover_active = False
        self._dense_frames_since_anchor = 0

    def _dense_recover_step(self, img0):
        """Advance the dense-homography-recovery tracking one frame (_dense_ref_img -> img0),
        independent of the strict corner gate -- called EVERY frame so it survives partial
        dropouts the strict all-4-primary gate alone would fail. Drops out-of-bounds/lost
        survivors; the canonical<->tracked correspondence stays index-aligned (both shrink together)."""
        if not self._dense_recover or self._dense_track_pts is None or self._dense_ref_img is None:
            return
        try:
            g0 = self._dense_ref_img if self._dense_ref_img.ndim == 2 else cv2.cvtColor(self._dense_ref_img, cv2.COLOR_BGR2GRAY)
            g1 = img0 if img0.ndim == 2 else cv2.cvtColor(img0, cv2.COLOR_BGR2GRAY)
            p1, st, _ = cv2.calcOpticalFlowPyrLK(g0, g1, self._dense_track_pts, None, **self._lk_params)
            st = np.asarray(st).flatten().astype(bool)
            _ih, _iw = img0.shape[:2]
            _p1f = p1.reshape(-1, 2)
            _in = ((_p1f[:, 0] >= 0) & (_p1f[:, 0] < _iw) & (_p1f[:, 1] >= 0) & (_p1f[:, 1] < _ih))
            keep = st & _in
            self._dense_canon_pts = self._dense_canon_pts[keep]
            self._dense_track_pts = _p1f[keep].astype(np.float32)
            self._dense_ref_img = img0.copy()
            self._dense_frames_since_anchor += 1
        except Exception:
            self._dense_canon_pts = None
            self._dense_track_pts = None
            self._dense_ref_img = None

    def _dense_recover_quad(self):
        """Recover the FULL primary-corner quad via a RANSAC homography fit from the surviving
        canonical dense points to their tracked positions, then map the canonical quad's 4
        corners through it. Returns (4,2) float32 recovered corners, or None if too few
        survivors / a degenerate fit. No depth/scale needed -- pure 2D pixel homography."""
        if (not self._dense_recover or self._dense_track_pts is None
                or len(self._dense_track_pts) < self._dense_recover_min_pts
                or self._dense_canon_quad is None
                or self._dense_frames_since_anchor > self._dense_recover_max_frames):
            return None   # too stale since the last real anchor -- don't trust an ever-aging chain
        try:
            Hmat, mask = cv2.findHomography(self._dense_canon_pts, self._dense_track_pts,
                                             cv2.RANSAC, self._dense_recover_ransac_px)
            if Hmat is None or mask is None or int(mask.sum()) < self._dense_recover_min_pts:
                return None
            _inlier_frac = float(mask.sum()) / max(1, len(self._dense_track_pts))
            if _inlier_frac < self._dense_recover_min_inlier_frac:
                return None   # too many outliers vs the canonical fit -- the tracked cloud has drifted
            recovered = cv2.perspectiveTransform(
                self._dense_canon_quad.reshape(-1, 1, 2), Hmat).reshape(-1, 2).astype(np.float32)
            _ih, _iw = self._dense_ref_img.shape[:2]
            if not (np.all(recovered[:, 0] >= -_iw) and np.all(recovered[:, 0] < 2 * _iw)
                    and np.all(recovered[:, 1] >= -_ih) and np.all(recovered[:, 1] < 2 * _ih)):
                return None   # degenerate/runaway homography guard
            self._dense_recover_active = True
            return recovered
        except Exception:
            return None

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
        self._dense_recover_step(imgs[0])   # advance dense-recovery tracking EVERY frame, independent of decode/gate status
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
            elif self._marker_priority == 'small':
                primary_i = int(np.argmin(ids))      # legacy: smallest ID (innermost/smallest marker)
            else:
                # BIGGER-marker priority (default): pick the largest marker by layout size, so the
                # big ID10 is used while detectable, then ID0 takes over when the big is gone.
                if self._board_layout is not None:
                    _sz = [float(self._board_layout.get(int(m), (0.0, 0.0, 0.0))[2]) for m in ids]
                    primary_i = int(np.argmax(_sz))
                else:
                    primary_i = int(np.argmax(ids))  # nested convention: larger ID = bigger marker
            _prev_primary = self._primary_id
            self._primary_id = int(ids[primary_i])
            if _prev_primary is not None and self._primary_id != _prev_primary:
                self._mtrace_hist.clear()   # loom d(lnM)/dt must not span a marker switch (sz step)
                # The observer must NOT differentiate the centroid JUMP at the big<->small handover:
                # a fast switch (no >0.5s gap) slipped past the KF's gap-reset and produced a ~7x flow
                # spike -> SMC divergence (the 0.9-19m catastrophic variance). Reset its history + KF.
                self._centroid_hist.clear()
                self._obs_kf_x = None; self._obs_kf_y = None
                self._obs_kf_Px = None; self._obs_kf_Py = None; self._obs_kf_t = None
            # Primary first, then the rest — so marker k occupies corners
            # [4k:4k+4] of all_pts_0 and marker_ids[k] is its ID. Primary stays
            # first for KLT-fallback continuity + display + the strict gate.
            order = [primary_i] + [i for i in range(M) if i != primary_i]
            marker_corners_0 = [results[0][i][0].reshape(-1, 2).astype(np.float32)
                                for i in order]
            aruco_pts_0 = marker_corners_0[0]
            self._dense_recover_anchor(aruco_pts_0, imgs[0])   # clean detection: re-anchor the dense-recovery canonical layout
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
                    _status_ok = (np.asarray(lk_status).flatten() == 1) if lk_status is not None else np.zeros(0, bool)
                    _n_tracked = int(np.sum(_status_ok))
                    _klt_diag = {"t": float(self._time.perf_counter()), "n_tracked": _n_tracked,
                                 "gate4_passed": bool(_n_tracked == 4), "gate_accept_passed": bool(_n_tracked >= self._klt_min_tracked),
                                 "reconstructed_idx": None, "in_bounds": None, "accepted": False}
                    if lk_pts is not None and lk_status is not None and _n_tracked >= self._klt_min_tracked and len(_status_ok) == 4:
                        _tracked = lk_pts.reshape(-1, 2).astype(np.float32)
                        if _n_tracked == 3:
                            # RELAXED GATE (2026-07-09, validated via KLT Diag n=1 run: momentary
                            # 1-2 frame decode gaps are dominated by exactly-3/4-tracked frames --
                            # a single corner losing LK lock for one step should not discard an
                            # otherwise-good frame). Reconstruct the missing corner via PARALLELOGRAM
                            # COMPLETION: for a planar quad in ArUco's fixed corner order
                            # [TL, TR, BR, BL], opposite corners share a diagonal midpoint
                            # (c0+c2 == c1+c3), so the missing corner = sum of the OTHER TWO minus
                            # its diagonal partner. Good approximation over one small inter-frame
                            # step (near-fronto-parallel or small perspective change); the existing
                            # downstream in-bounds check is the safety net if it's wrong.
                            _miss = int(np.where(~_status_ok)[0][0])
                            _partner = (_miss + 2) % 4
                            _others = [i for i in range(4) if i not in (_miss, _partner)]
                            _tracked[_miss] = _tracked[_others[0]] + _tracked[_others[1]] - _tracked[_partner]
                            _klt_diag["reconstructed_idx"] = _miss
                        # Abort KLT if any corner has left the image — the marker is
                        # gone and continuing to extrapolate produces off-screen centroids
                        # (s[0] up to 3× beyond image boundary) that blow up cross(dw,s)
                        # in θ_norm → κ runaway. Reset so next frame starts fresh.
                        _img_h, _img_w = imgs[0].shape[:2]
                        _in_bounds = (np.all(_tracked[:, 0] >= 0) and
                                      np.all(_tracked[:, 0] < _img_w) and
                                      np.all(_tracked[:, 1] >= 0) and
                                      np.all(_tracked[:, 1] < _img_h))
                        _klt_diag["in_bounds"] = bool(_in_bounds)
                        if _in_bounds:
                            _klt_diag["accepted"] = True
                            aruco_pts_0 = _tracked
                            used_klt_fallback = True
                            self._lk_step_count += 1
                            if self._lk_step_count == 1:
                                print(f"ArUco lost — KLT fallback active (cap {self._max_lk_steps} frames)")
                            # UNIFIED STALENESS GATE (2026-07-07): a SHORT corner-fallback success
                            # (still barely diverged from the last real decode) is trustworthy enough
                            # to ALSO soft-re-anchor the dense-recovery canonical state -- extends its
                            # freshness far more often than requiring a full fresh decode every time,
                            # without trusting a long-chained (>DENSE_SOFT_ANCHOR_MAX_STEPS) KLT drift.
                            if self._lk_step_count <= self._dense_soft_anchor_max_steps:
                                self._dense_recover_anchor(aruco_pts_0, imgs[0])
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
                    self._klt_diag_log.append(_klt_diag)
            except Exception as _e:
                # Defensive: if anything in the KLT fallback path errors, fall
                # through to the normal stale path rather than killing the thread.
                aruco_pts_0 = None

        # THIRD FALLBACK TIER (2026-07-07): fresh decode and KLT corner-fallback both failed (or
        # KLT's strict all-4-in-bounds requirement wasn't met) -- try recovering the FULL quad via
        # the dense-homography fit before giving up to the (self-reinforcing) s-extrapolation path.
        # More robust than the KLT corner-fallback: many candidate points (RANSAC-filtered) instead
        # of 4 fragile ones, and no MARKER_KLT_MAX_STEPS cap (the homography degrades gracefully as
        # survivors thin out, rather than hard-stopping at a fixed frame count).
        if aruco_pts_0 is None and self._dense_recover:
            _recovered = self._dense_recover_quad()
            if _recovered is not None:
                aruco_pts_0 = _recovered
                if self._single_marker:
                    extra_pts_0 = self._scaled_quad_points(aruco_pts_0, per_side=self._dense_pts_per_side)
                if os.environ.get("DENSE_RECOVER_DBG", "0") == "1":
                    print("[dr] t%.3f RECOVERED quad from %d dense survivors" % (
                        float(getattr(self, '_stamp', 0.0)), len(self._dense_track_pts)), flush=True)

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
            # OVERFLOW vs DRIFT-OFF (same corner-bounds logic): OVERFLOW SPANS the frame (corners past
            # OPPOSITE edges = marker filling/centered = still over the target); DRIFT-OFF leaves ONE
            # side (marker slid off-center). Only drift-off invalidates the ring (nadir no longer over
            # the target). Latched last-known so it survives the Ncorn->0 frame (decode already gone).
            _span = ((_cc[:, 0].min() < _m and _cc[:, 0].max() > _iw - _m) or
                     (_cc[:, 1].min() < _m and _cc[:, 1].max() > _ih - _m))
            self._last_drifted_off = bool(_marker_leaving and not _span)
            self._last_overflow = bool(_marker_leaving and _span)   # companion flag (2026-07-07): SPANNING case, latched the same way

        # GYRO-COMPENSATED CENTROID-RATE OBSERVER (default-off). Computed from the DECODED corners
        # (aruco_pts_0) — NOT the LK-tracked V_aruco_norm — so it runs even when LK fails (Nfc=0) at
        # altitude. Provides the lateral flow h_x,h_y from ṡ + loom + gyro-rotation compensation:
        #   h_x = ṡ_x + x0·h_z + y0·wz   (V-frame: roll/pitch leveled out, yaw preserved -> wz only)
        #   h_y = ṡ_y + y0·h_z − x0·wz
        # h_z from the moment loom; w from the IMU gyro rotated into the V-frame (clean, not the
        # off-center-ill-conditioned lstsq). Stored for injection at the flow-output sites below.
        if (self._single_marker and self._centroid_rate and aruco_pts_0 is not None
                and quats is not None and len(quats) > 0 and quats[0] is not None):
            try:
                _Vdec = self._getVirtualPts(np.asarray(aruco_pts_0, np.float32), quats[0])   # FRAME-PAIR FIX 2026-07-04: aruco_pts_0 belongs to frame-0 -> level with quats[0], not quats[1] (matches V_aruco_norm/V_flow_norm convention; the quats[1] mismatch left a residual tilt ∝ angular rate = a source of the off-center yaw leak)
                _x0 = float(_Vdec[:, 0].mean()); _y0 = float(_Vdec[:, 1].mean())
                _Mo = float(np.mean(np.sum((_Vdec - np.array([_x0, _y0])) ** 2, axis=1)))
                _to = float(getattr(self, '_stamp', 0.0))
                if _Mo > 1e-12 and np.isfinite(_Mo):
                    self._centroid_hist.append((_to, _x0, _y0, np.log(_Mo)))
                if len(self._centroid_hist) >= 3:
                    _ta = np.array([c[0] for c in self._centroid_hist])
                    if (_ta.max() - _ta.min()) > 1e-4:
                        _t0 = _ta - _ta[0]
                        # CV-Kalman filter (not raw polyfit) — suppresses centroid-jitter noise
                        # that drove the residual lateral drift (offline: corr .13->.67, noise -3x).
                        _sdx, _sdy = self._obs_vel_kf(_x0, _y0, _to)
                        _loom_dec = float(-0.5 * np.polyfit(_t0, [c[3] for c in self._centroid_hist], 1)[0]
                                          * self._loom_gain)   # sz=1 single-marker (size-norm inert)
                        _avo = angvels[0] if (angvels is not None and len(angvels) > 0
                                              and angvels[0] is not None) else None
                        if _avo is not None:
                            _wv = self._vframe_w([_avo.forward_rad_s, _avo.right_rad_s, _avo.down_rad_s], quats[0])   # FRAME-PAIR FIX 2026-07-04: match the frame-0 centroid (was quats[1]/angvels[1])
                        else:
                            _wv = np.zeros(3)
                        _alpha_rate = 0.0   # TODO moving-target: w_target_z = d(alpha)/dt; 0 for stationary
                        _wv[2] -= _alpha_rate                  # relative yaw = camera (gyro) − target
                        # W_Z SIGN FIX (2026-07-04): _fill_A's ω_z is the INTERACTION-matrix z-rate =
                        # −body_yaw_rate (validated 2026-06-25: lstsq w_z corr −0.91 w/ body yaw). But
                        # _vframe_w gives _wv[2] = +body_yaw_rate. So the interaction ω_z = −_wv[2].
                        # The observer had used +_wv[2] → sign-flipped yaw coupling: harmless on h_x
                        # (small y0·w_z when drifting in x) but ANTI-correlated h_y (large x0·w_z).
                        _oz = -_wv[2]
                        _hz = _loom_dec
                        # FRAME FIX (2026-06-23): ṡ is the VIRTUAL (de-rotated) centroid rate, and
                        # _getVirtualPts ALREADY levels out roll/pitch (z→world-down) while preserving
                        # yaw (rotz(yaw)). So w_x,w_y rotation flow is already removed by the de-rotation
                        # — subtracting L_w with the full gyro DOUBLE-COUNTS it (the tilt terms inflated
                        # h 6.2× off-center: meas|h_lat| 1.20 vs GT 0.19). Keep ONLY the yaw term (w_z,
                        # preserved) + the loom term. Signs from _fill_A (wz col = [−y, x]).
                        _hx = _sdx + _x0 * _hz + _y0 * _oz
                        _hy = _sdy + _y0 * _hz - _x0 * _oz
                        self._observer_flow = np.clip(
                            np.array([_hx, _hy, _hz, 0.0, 0.0, _oz]), -10.0, 10.0)
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

            if self._decode_flow and os.environ.get("DECODE_DBG") == "1":
                self._dbg_f = getattr(self, '_dbg_f', 0) + 1
                _lkd = float(np.mean(np.abs(all_pts_1[:n_aruco].reshape(-1, 2) - aruco_pts_0)))
                print(f"[dd] t{float(getattr(self, '_stamp', 0.0)):.3f} lkd{_lkd:.2f} lost{int(np.sum(status[:n_aruco] == 0))}", flush=True)

            # DECODE-CORRESPONDENCE fallback with LK-STUCK detection. LK re-detects the marker
            # GLOBALLY in the next frame (canonical corner order → corner k(t)↔k(t+1), no basin).
            # Use it per marker when LK is unreliable: either it dropped a corner (status==0) OR it
            # reports materially LESS motion than decode sees (it silently converged to a wrong
            # nearby min — the high-speed failure that status==0 misses). LK stays primary when it
            # agrees with decode (low speed, sub-pixel accurate). Match on ID (nested-marker safe).
            if self._decode_flow and marker_ids is not None:
                _res1 = self._detector.detectMarkers(imgs[1])
                if _res1[0]:
                    _ids1 = np.asarray(_res1[1]).flatten()
                    _c1 = {int(_ids1[q]): np.asarray(_res1[0][q]).reshape(-1, 2).astype(np.float32)
                           for q in range(len(_ids1))}
                    for k, mid in enumerate(marker_ids):
                        mid = int(mid); sl = slice(4 * k, 4 * k + 4)
                        if mid not in _c1:
                            continue
                        _d0 = all_pts_0[sl].reshape(-1, 2)                    # frame-0 corners
                        _d1_dec = _c1[mid]                                    # decode frame-1 corners
                        _disp_lk = float(np.mean(np.abs(all_pts_1[sl].reshape(-1, 2) - _d0)))
                        _disp_dec = float(np.mean(np.abs(_d1_dec - _d0)))
                        if np.any(status[sl] == 0) or _disp_dec > _disp_lk + self._decode_trig:
                            all_pts_1[sl] = (_d0 + (_d1_dec - _d0) * self._decode_scale).reshape(-1, 1, 2)
                            status[sl] = 1
                            self._decode_fills += 1

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
                if marker_ids is not None and (self._board_layout is not None or self._board_selfcal):
                    all_pts_1_2d = all_pts_1.reshape(-1, 2)
                    grp = []
                    for k, mid in enumerate(marker_ids):
                        sl = slice(4 * k, 4 * k + 4)
                        # self-cal: track ALL decoded markers (to learn them); file mode: only layout markers
                        if np.all(status[sl] == 1) and (self._board_selfcal
                                or (self._board_layout is not None and mid in self._board_layout)):
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
                    self._update_sz_ratio(board_markers_V)   # learn size ratios from co-visible markers

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
                # CONSOLIDATION (2026-07-04, user): when the centroid-rate observer is active, its
                # V-frame centroid-rate defines h_xy (overrides below), the moment loom defines h_z,
                # and the gyro defines w_z — so the σ_min corner lstsq is fully redundant (only w_z
                # ever survived it, and the observer's gyro w_z is cleaner). SKIP the second LK's
                # degenerate 6-DOF solve. One KLT → V-frame centroid → position + rate drives the flow.
                _obs_active = (self._single_marker and self._centroid_rate and self._observer_valid)
                if _obs_active:
                    V_v = np.zeros(6)
                    V_v[5] = float(self._observer_flow[5])       # gyro yaw rate (replaces lstsq w_z)
                elif self._lat_reduced and self._target_level:
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
                if not _obs_active:
                    cond = (sv[0] / sv[-1]) if (len(sv) > 0 and sv[-1] > 0) else np.inf
                    bad = (
                        rank < _min_rank
                        or cond > 1e4
                        or not np.all(np.isfinite(V_v))
                        or np.max(np.abs(V_v)) > 50.0
                    )
                    # DEBUG (2026-07-07, user): expose the RAW pre-clamp solve whenever it's borderline
                    # (bad OR would-be-clipped) -- the clamp/zero-fallback can mask a real correspondence
                    # break (e.g. marker re-lock mixing stale+fresh corners) as a plausible, HELD value.
                    if os.environ.get("FLOW_CLAMP_DBG", "0") == "1" and (bad or np.max(np.abs(V_v)) > 10.0):
                        print("[flowclamp] t%.3f RAW V_v=%s bad=%s rank=%d/%d cond=%.1f Ncorn=%d" % (
                            float(getattr(self, '_stamp', 0.0)), np.round(V_v, 2), bad, rank, _min_rank,
                            cond, int(len(A) // 2)), flush=True)
                    if os.environ.get("FLOW_CLAMP_OFF", "0") == "1":
                        # bypass the safety net entirely (diagnostic only): NaN/inf guard kept minimal
                        # so downstream doesn't crash, but the magnitude/zero-fallback gates are OFF.
                        if not np.all(np.isfinite(V_v)):
                            V_v = np.zeros(6)
                    else:
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
                    # OVER_TARGET (2026-07-05, user): is the NADIR (V-frame origin) inside the primary
                    # marker's V-frame quad? Depth-free "over the target" (tilt-removed), replaces the
                    # ambiguous s_e_n gate for the ring-commit. Self-scaling: at altitude the small marker
                    # is tiny -> nadir rarely inside -> rejects premature fire; at the terminal it's large
                    # -> easy when centered. Convex point-in-quad via CCW edge cross-products.
                    _q = np.asarray(_vp[:4], float)
                    if _q.shape[0] >= 4 and np.isfinite(_q).all():
                        _qc = _q.mean(0)
                        _ord = _q[np.argsort(np.arctan2(_q[:, 1] - _qc[1], _q[:, 0] - _qc[0]))]
                        _ins = True
                        for _e in range(4):
                            _a = _ord[_e]; _b = _ord[(_e + 1) % 4]
                            if (_b[0]-_a[0])*(0.0-_a[1]) - (_b[1]-_a[1])*(0.0-_a[0]) < 0.0:
                                _ins = False; break
                        self.OVER_TARGET = bool(_ins)
                    # HANDOVER LATCH (ID/size-FREE, image-space): the RAW apparent-size² M drops abruptly
                    # (~ratio², big->small switch) -> large NEGATIVE d(ln RAW_M), unmistakable vs the
                    # ~0.04 normal loom. Uses the RAW M here (BEFORE the LOOM_SZ_RATIO normalization below,
                    # which deliberately makes the NORMALIZED d(lnM) continuous). One-way; only big->small
                    # (M drop, negative) latches — small->big flicker-back (positive) is ignored.
                    if _M > 1e-12 and np.isfinite(_M):
                        _rlnM = float(np.log(_M))
                        if self._raw_lnM_prev is not None and not self.HANDOVER_LATCHED:
                            _dlm = _rlnM - self._raw_lnM_prev
                            # OVERFLOW SIGNATURE: big->small drop AND the SMALL (current) marker is large
                            # enough (_rlnM = ln(small M) > min) = we are close = real terminal overflow.
                            _drop = _dlm < -self._handover_ln
                            if _drop and _rlnM > self._handover_min_ln:   # overflow: small marker is large (close)
                                self._handover_cand = 1
                            elif _drop:                          # drop but small is TINY = altitude tilt-failure -> REJECT
                                self._handover_cand = 0
                                if self._handover_dbg:
                                    print(f"[HANDOVER] rejected t={float(getattr(self,'_stamp',0.0)):.2f} "
                                          f"(small M={_M:.4f} < {self._handover_min_m} = far/tilt-fail, not overflow)")
                            elif _dlm > self._handover_ln:       # small->big re-decode (flicker back): CANCEL
                                self._handover_cand = 0
                            elif self._handover_cand > 0:        # small primary persisting (big has NOT re-decoded)
                                self._handover_cand += 1
                                if self._handover_cand >= self._handover_persist:
                                    self.HANDOVER_LATCHED = True
                                    if self._handover_dbg:
                                        print(f"[HANDOVER] latched t={float(getattr(self,'_stamp',0.0)):.2f} "
                                              f"(overflow: small M={_M:.4f}>{self._handover_min_m}, sustained {self._handover_persist}f)")
                        self._raw_lnM_prev = _rlnM
                    # SIZE-NORMALIZE by the primary marker's physical size² so the scale is
                    # continuous across primary-ID SWITCHES (the board's markers span ~7× in
                    # size; min-ID flickers as markers enter/leave decode → d(lnM)/dt jumps).
                    # M/sz² = (f/Z)², marker-independent. Without this the loom is GARBAGE
                    # (offline corr 0.41 vs 0.65 with norm). Fallback sz=1 (single-marker world).
                    # No size normalization needed (2026-07-03): within one marker sz² cancels in
                    # d(lnM)/dt (the absolute M level is never used downstream), and the primary-
                    # SWITCH step is killed by _mtrace_hist.clear() on switch (see selection) and on
                    # marker loss. So the loom is marker-agnostic — zero layout/ID/ratio dependence.
                    # (Replaced the board_layout[primary_id][2] lookup, which cancelled anyway.)
                    # BOTH-VISIBLE normalization: divide M by the primary's physical-size ratio² so
                    # M ∝ (f/Z)² is CONTINUOUS across the big<->small handover (no size step to spike
                    # the loom). Ratio learned online from co-visible frames; 1.0 until learned.
                    _szr = self._sz_ratio.get(self._primary_id, 1.0) if self._loom_sz_ratio_on else 1.0
                    _M = _M / (_szr * _szr)
                    _t = float(getattr(self, '_stamp', 0.0))
                    if _M > 1e-12 and np.isfinite(_M):
                        _lnM = np.log(_M)
                        # d(lnM)/dt outlier-hold (parallel to the ds/dh checks): a residual handover /
                        # marker-leaving transient steps ln M -> HOLD the loom, don't span it.
                        _dlnM = abs(_lnM - self._loom_lnM_prev) if self._loom_lnM_prev is not None else 0.0
                        self._loom_lnM_prev = _lnM
                        if self._loom_dlnM_max > 0 and _dlnM > self._loom_dlnM_max:
                            self._mtrace_hist.clear()           # don't span the size transition
                            V_v[2] = float(self._loom_hold)     # HOLD the loom through the handover
                        else:
                            self._mtrace_hist.append((_t, _lnM))
                            if len(self._mtrace_hist) >= 3:
                                _ta = np.array([h[0] for h in self._mtrace_hist])
                                _la = np.array([h[1] for h in self._mtrace_hist])
                                if (_ta.max() - _ta.min()) > 1e-4:
                                    _slope = np.polyfit(_ta - _ta[0], _la, 1)[0]   # d(ln M)/dt
                                    V_v[2] = float(np.clip(-0.5 * _slope * self._loom_gain, -10.0, 10.0))
                                self._loom_hold = V_v[2]        # remember last-good loom
                            else:
                                V_v[2] = float(self._loom_hold) # during the 3-frame rebuild, hold last-good

                # CENTROID-RATE OBSERVER injection (a): when the LK flow IS available, still prefer
                # the gyro-compensated centroid-rate lateral (robust to the off-center spurious spikes
                # the lstsq produces). h_z stays the moment loom (V_v[2]). Default-off.
                if self._single_marker and self._centroid_rate and self._observer_valid:
                    V_v[0] = float(self._observer_flow[0])
                    V_v[1] = float(self._observer_flow[1])

                # ds/dh OUTLIER GATE (2026-07-04): reject a per-frame lateral-flow spike (|Δh| beyond
                # a physical rate) and HOLD last-good — kills the σ_min LK garbage ramp (→ fly-away)
                # while passing the good median + genuine slow v/Z growth. Applies to whichever source
                # produced V_v[0:2] (corner-flow OR observer). Rate-based → scale/depth-free.
                if self._flow_dh_max > 0:
                    _v_raw = np.array([V_v[0], V_v[1]], dtype=float)   # RAW, BEFORE any reject
                    if self._flow_hold is None:
                        self._flow_hold = _v_raw.copy()
                    for _i in (0, 1):
                        if self._flow_prev is not None and abs(_v_raw[_i] - self._flow_prev[_i]) > self._flow_dh_max:
                            V_v[_i] = float(self._flow_hold[_i])         # reject: emit last ACCEPTED (no spike leak)
                        else:
                            self._flow_hold[_i] = _v_raw[_i]             # accept: advance last-good
                    self._flow_prev = _v_raw                            # detection ref = RAW (advances, no latch)

                V_v_scaled = size_factor * V_v
                self._opt_flow_ang_vel_raw.append(V_v_scaled)
                if self._h_extrap:
                    # REAL-only history for the extrapolation fit -- never receives the extrapolated
                    # output back (the self-reinforcement bug that broke deg-1 s-extrapolation).
                    self._h_real_t.append(self._time.perf_counter())
                    self._h_real_v.append(V_v_scaled.copy())
                    if len(self._h_real_t) > 32:      # bounded history (only the last few are ever fit)
                        self._h_real_t = self._h_real_t[-32:]
                        self._h_real_v = self._h_real_v[-32:]
                _av = angvels[1] if (angvels is not None and len(angvels) > 1 and angvels[1] is not None) else None
                self._imu_angvel_raw.append(
                    np.array([_av.forward_rad_s, _av.right_rad_s, _av.down_rad_s])
                    if _av is not None else np.full(3, np.nan))
                _q1 = quats[1] if (quats is not None and len(quats) > 1 and quats[1] is not None) else None
                self._quat_log.append(
                    np.array([_q1.w, _q1.x, _q1.y, _q1.z]) if _q1 is not None else np.full(4, np.nan))
                self._n_flow_corners.append(int(len(flow_pts_1)))
                self._drift_off_hist.append(self._last_drifted_off)
                self._overflow_hist.append(self._last_overflow)
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

                # ds OUTLIER-HOLD on the raw centroid: reject a per-frame centroid JUMP (detection/LK
                # glitch) and hold last-good — keeps s clean for the position loop AND the observer.
                if self._s_ds_max > 0 and self._img_feature_param:
                    _s = self._img_feature_param[-1]
                    _s_raw = np.array([_s[0], _s[1]], dtype=float)   # RAW value, BEFORE any reject
                    if self._s_hold is None:
                        self._s_hold = _s_raw.copy()
                    for _j in (0, 1):
                        if self._s_prev is not None and abs(_s_raw[_j] - self._s_prev[_j]) > self._s_ds_max:
                            _s[_j] = float(self._s_hold[_j])         # reject: emit last ACCEPTED (no spike leak)
                        else:
                            self._s_hold[_j] = _s_raw[_j]            # accept: advance last-good
                    # detection ref = RAW (advances) → per-instant check, no latch; substitution =
                    # last-accepted (_s_hold) → spike fully dropped. (Matches the loom d(lnM)/dt gate.)
                    self._s_prev = _s_raw

                # REAL-SAMPLE BUFFER for s-extrapolation (2026-07-09 fix): must be captured AFTER the
                # ds outlier-hold above, not before. Bug found this session: an isolated single-frame
                # detection spike (e.g. -0.44 sandwiched between ~0 values, n_corners>0 so genuinely
                # "detected" but still a glitch) passes the ds_max>0.15 guard and gets corrected in
                # self._img_feature_param[-1] IN PLACE -- but if this buffer were populated from the
                # PRE-guard board_s/s value (as an earlier version of this fix did), the extrapolation
                # fit's "real-only" history would still contain the un-rejected outlier, corrupting the
                # deg-1 fit's slope. Confirmed via reconstruction: this exact sequence (a real-but-spiky
                # sample at t-0.3s) produced a terminal centroid extrapolation jump (0.12->1.03 the
                # instant the marker was lost) that fed s_e_n/zeta_r and traced directly into the
                # terminal-kick breach. Read self._img_feature_param[-1] HERE (post-guard) instead.
                if self._img_feature_param:
                    self._img_feature_param_real_t.append(self._time.perf_counter())
                    self._img_feature_param_real.append(np.asarray(self._img_feature_param[-1]).copy())
                    if len(self._img_feature_param_real_t) > 32:
                        self._img_feature_param_real_t = self._img_feature_param_real_t[-32:]
                        self._img_feature_param_real = self._img_feature_param_real[-32:]

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
                self._flow_prev = None       # ds/dh gate: re-init after a marker-loss gap (don't hold stale)

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
            self.RING_N_STATIONS = int(_nr)   # debug: paired-station survivor count this frame
            self.RING_DIV_RAW = float(_pdiv) if np.isfinite(_pdiv) else float('nan')
            self.RING_MOMENT_RAW = float(_rmom) if np.isfinite(_rmom) else float('nan')
            # Expose the marker-less ring loom for the terminal ring-commit (calibrated pure_div; the
            # A/B-preferred, accurate-to-0.2m divergence). TODO: swap to the ring MOMENT near the deck
            # (milder over-report) on a DEPTH-FREE trigger (Nring drop), not altitude.
            if self._ring_loom_source == "moment" and np.isfinite(_rmom):
                # ring MOMENT is already in loom units (-½ d(lnM)/dt), no div-cal needed
                self.RING_LOOM = float(np.clip(_rmom * self._ring_loom_scale, -10.0, 10.0))
            elif np.isfinite(_pdiv):
                self.RING_LOOM = float(np.clip(_pdiv * self._ring_div_cal * self._ring_loom_scale, -10.0, 10.0))
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
            if self._h_extrap and len(self._h_real_t) >= 5:
                # DECAYED deg-1 fit against REAL-ONLY samples (never self-referencing). Reject
                # samples already at the clip boundary (their slope is untrustworthy, not signal).
                # extrapolate() needs len(t)>=n+1 for a genuine trend fit (else it falls back to a
                # mean) -- window the last 8 real samples, n=4, so a fit is actually attempted.
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
                # DECAY toward zero over consecutive misses -- a long gap converges to "no known
                # motion" (matching hard-zero's safety net) instead of an indefinitely-held trend.
                _decay = max(0.0, 1.0 - self._consec_misses / max(1, self._h_extrap_decay_frames))
                extrapolated_opt_flow_ang_vel_raw = np.clip(
                    _fit * _decay, -self._h_extrap_max, self._h_extrap_max)
            else:
                extrapolated_opt_flow_ang_vel_raw = np.zeros(6)

            # Centroid s: DECAYED deg-1 fit against REAL-ONLY samples (never self-referencing).
            # Similar to h-extrapolation: avoid cascading extrapolated values back into future
            # extrapolations. Use bounded history (max 32 samples) and clipped-sample rejection.
            if len(self._img_feature_param_real_t) >= 5:
                _rt = np.asarray(self._img_feature_param_real_t[-8:])
                _rv = np.asarray(self._img_feature_param_real[-8:])
                # Reject s[0:2] samples near FoV boundary (extrapolation unreliable at edge)
                _valid = np.all(np.abs(_rv[:, 0:2]) < 5.0 - 1e-3, axis=1)  # ±5 rad ~ FoV clip bound
                if int(_valid.sum()) >= 5:
                    _rt, _rv = _rt[_valid], _rv[_valid]
                    _fit = extrapolate(_rt, _rv, n=min(4, len(_rt) - 1), deg=1, default_shape=4)
                    _fit = np.nan_to_num(np.asarray(_fit), nan=0.0, posinf=5.0, neginf=-5.0)
                else:
                    _fit = self._img_feature_param_real[-1].copy() if self._img_feature_param_real else np.zeros(4)
                # DECAY toward zero over consecutive misses (long gaps -> no known position)
                _decay = max(0.0, 1.0 - self._consec_misses / max(1, self._h_extrap_decay_frames))
                extrapolated_img_feature_param = np.clip(_fit * _decay, -5.0, 5.0)
                extrapolated_img_feature_param = np.nan_to_num(
                    np.asarray(extrapolated_img_feature_param), nan=0.0, posinf=5.0, neginf=-5.0)
            else:
                # Not enough real data yet; hold last value or zero
                extrapolated_img_feature_param = (self._img_feature_param_real[-1].copy()
                                                  if self._img_feature_param_real else np.zeros(4))
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
            self._drift_off_hist.append(self._last_drifted_off)   # latched value (2026-07-07 failure-cause tagging)
            self._overflow_hist.append(self._last_overflow)

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

    def _obs_vel_kf(self, x0, y0, t):
        """Constant-velocity Kalman filter on the decoded centroid -> smoothed lateral velocity
        (ṡx, ṡy). Replaces raw polyfit differentiation (jitter-amplifying). Resets on init or a
        stale/large time gap (won't differentiate across a marker-loss gap)."""
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

    @staticmethod
    def _marker_center_size(cV):
        """V-frame center (mean corner) + size (RMS corner distance from center) of a marker."""
        c = np.asarray(cV, dtype=float).reshape(-1, 2)
        ctr = c.mean(axis=0)
        sz = float(np.sqrt(np.mean(np.sum((c - ctr) ** 2, axis=1))))
        return ctr, sz

    def _update_sz_ratio(self, markers_V):
        """Learn each marker's physical-size RATIO online from co-visible frames. In one frame all
        markers share Z, so sz_i/sz_j is Z-independent = the physical-size ratio. Anchor the current
        PRIMARY at 1.0 (so its loom M is unchanged and nothing jumps when the ratio is first learned),
        propagate to co-visible markers, running-average. Enables switch-continuous loom M/sz²
        normalization with zero layout/ID/size prior."""
        if not self._loom_sz_ratio_on or markers_V is None or len(markers_V) < 2:
            return
        sizes = {int(mid): self._marker_center_size(cV)[1] for mid, cV in markers_V}
        sizes = {m: s for m, s in sizes.items() if s > 1e-6}
        if len(sizes) < 2:
            return
        known = [m for m in sizes if m in self._sz_ratio]
        if not known:                                   # anchor the primary (or smallest ID) at 1.0
            anchor = self._primary_id if self._primary_id in sizes else min(sizes)
            self._sz_ratio[anchor] = 1.0
            known = [anchor]
        ref = known[0]; ref_r = self._sz_ratio[ref]; ref_s = sizes[ref]
        for m, s in sizes.items():
            r_meas = ref_r * s / ref_s                   # this marker's ratio from the co-visible pair
            if m in self._sz_ratio:
                self._sz_ratio[m] = (1 - self._sz_ratio_a) * self._sz_ratio[m] + self._sz_ratio_a * r_meas
            else:
                self._sz_ratio[m] = r_meas

    @staticmethod
    def _fit_similarity(src, dst):
        """Least-squares similarity (scale s, rotation R 2x2, translation t) mapping src->dst
        (Umeyama). Returns (s, R, t, rms) or None. rms is the fit residual in dst units."""
        src = np.asarray(src, float); dst = np.asarray(dst, float)
        if len(src) < 2:
            return None
        mu_s, mu_d = src.mean(0), dst.mean(0)
        S, D = src - mu_s, dst - mu_d
        var_s = float(np.mean(np.sum(S ** 2, axis=1)))
        if var_s < 1e-18:
            return None
        C = (D.T @ S) / len(src)
        U, sig, Vt = np.linalg.svd(C)
        R = U @ Vt
        if np.linalg.det(R) < 0:
            U = U.copy(); U[:, -1] *= -1; R = U @ Vt
        s = float(np.sum(sig) / var_s)
        t = mu_d - s * (R @ mu_s)
        pred = (s * (R @ src.T)).T + t
        rms = float(np.sqrt(np.mean(np.sum((pred - dst) ** 2, axis=1))))
        return s, R, t, rms

    def _selfcal_ready(self):
        return self._board_selfcal and self._selfcal_frames >= self._selfcal_min_frames

    def _update_board_selfcal(self, markers_V):
        """Self-calibrate the board layout online from co-visible markers (no file/IDs/sizes).
        The V-frame marker centers/sizes are the board layout up to a global similarity; the first
        multi-marker frame bootstraps it (scale-free gauge), later frames are similarity-aligned to
        the running layout on shared markers and running-averaged. Low-residual gate rejects
        badly-leveled/occluded frames. IDs are correspondence keys only."""
        if not self._board_selfcal or len(markers_V) < 2:
            return
        obs = {int(mid): self._marker_center_size(cV) for mid, cV in markers_V}
        if not self._selfcal_layout:
            sizes = np.array([s for _, s in obs.values()])
            sref = float(np.median(sizes)) or 1.0
            origin = np.array([c for c, _ in obs.values()]).mean(axis=0)
            for mid, (c, s) in obs.items():
                self._selfcal_layout[mid] = (float((c[0] - origin[0]) / sref),
                                             float((c[1] - origin[1]) / sref), float(s / sref))
                self._selfcal_counts[mid] = 1
            self._selfcal_frames = 1
            return
        shared = [m for m in obs if m in self._selfcal_layout]
        if len(shared) < 2:
            return
        sim = self._fit_similarity(np.array([obs[m][0] for m in shared]),
                                   np.array([self._selfcal_layout[m][:2] for m in shared]))
        if sim is None:
            return
        s, R, t, rms = sim
        if rms > self._selfcal_res_max:          # poorly-leveled / occluded frame — skip
            return
        for mid, (c, sz) in obs.items():
            lc = s * (R @ c) + t                 # center in the layout gauge
            lsz = s * sz                         # size in layout units
            if mid in self._selfcal_layout:
                n = self._selfcal_counts[mid]
                ox, oy, osz = self._selfcal_layout[mid]
                self._selfcal_layout[mid] = ((ox * n + lc[0]) / (n + 1),
                                             (oy * n + lc[1]) / (n + 1),
                                             (osz * n + lsz) / (n + 1))
                self._selfcal_counts[mid] = n + 1
            else:
                self._selfcal_layout[mid] = (float(lc[0]), float(lc[1]), float(lsz))
                self._selfcal_counts[mid] = 1
        self._selfcal_frames += 1

    def _board_corners(self, mid):
        """Board-plane coords (DIMENSIONLESS — normalised to marker-0's size,
        board centre = origin) of marker `mid`'s 4 corners, in cv2.aruco order
        [TL, TR, BR, BL]. Board +x = texture column (right), +y = texture row
        (down) — matching make_aruco_board.py. The layout carries NO metres: the
        findHomography fit that consumes these is scale-invariant, so the centroid
        is identical to any global unit; normalising to marker-0 = 1 makes the
        scale-freeness manifest (no marker physical size anywhere in the pipeline).
        """
        if self._selfcal_ready() and mid in self._selfcal_layout:
            cx, cy, sz = self._selfcal_layout[mid]     # online self-calibrated (no file/IDs/sizes)
        else:
            cx, cy, sz = self._board_layout[mid]       # file prior (fallback until self-cal ready)
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
        # Online board self-calibration: accumulate this frame's co-visible markers into the
        # learned layout (no file/IDs/sizes). Once ready it supplies _board_corners; until then
        # fall back to the file prior (if present), else return None -> single-marker fallback.
        self._update_board_selfcal(markers_V)
        use_selfcal = self._selfcal_ready()
        board_pts, img_pts = [], []
        for mid, cV in markers_V:
            if use_selfcal:
                if mid not in self._selfcal_layout:
                    continue
            elif self._board_layout is None or mid not in self._board_layout:
                continue
            board_pts.append(self._board_corners(mid))
            img_pts.append(np.asarray(cV, dtype=np.float64) * size_factor)
        if len(board_pts) < 1:
            return None
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
        # NOTE: the REAL-sample buffer (_img_feature_param_real/_real_t, used by the s-extrapolation
        # fit) is populated by the CALLER after this returns, once the ds outlier-hold guard has run
        # on self._img_feature_param[-1] -- see the call site (single-marker branch). Populating it
        # here would capture the PRE-guard (possibly outlier) value. Do not re-add here.

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
        # Diagnostic only (2026-07-09): track z_v for phantom-s analysis. z_v->0 or negative means
        # this corner's ray, at the current tilt, is not representable in the gravity-leveled V-frame
        # (grazing/behind-camera obliqueness -- see feedback_lateral_kappa_runaway). NOT clamped here:
        # an earlier clamp-to-0.01 was tried and removed -- it only bounded the magnitude of an
        # already-fabricated point rather than addressing it, matching the project's documented
        # anti-pattern (feedback_clamps_during_tuning: clamps mask symptoms, don't fix causes).
        # Post-hoc reconstruction (2026-07-09) found z_v only goes low/negative during frames that
        # are ALREADY part of a diverging tilt event (correlates with the terminal 1/Z tilt-command
        # amplification, not an independent trigger) -- so treating a bad z_v frame as informative
        # (log it) rather than silently patching it is the right call until a dedicated reject-as-lost
        # gate is built and validated.
        self._z_v_min_log.append(float(np.min(z_v)) if len(z_v) > 0 else np.nan)
        if len(self._z_v_min_log) > 5000:
            self._z_v_min_log = self._z_v_min_log[-5000:]
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
            "KLT Diag": self._klt_diag_log,   # (2026-07-09) per-attempt KLT-fallback status: validates the ==4 gate hypothesis for momentary decode gaps
            "Drift Off": self._drift_off_hist,      # per-frame: marker left FoV OFF-CENTER (2026-07-07 failure-cause tagging)
            "Overflow": self._overflow_hist,        # per-frame: marker SPANNED the frame (overflow, still over target)
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

    def getFailureCause(self):
        """Live snapshot for failure-cause tagging (2026-07-07, user): distinguishes DRIFT_OFF
        (marker left the FoV off-center -> the accumulated-position-error precursor, see
        feedback_terminal_overflow_deck_flyaway and the a_u drift investigation) from OVERFLOW
        (marker spanned the frame, still over target -> the close-range deck failure) at the
        moment a caller (landing_test.py) checks it -- e.g. right when TARGET_LOST/terminal
        perception loss triggers, to auto-tag root cause instead of requiring a manual trace."""
        if self._last_drifted_off:
            return "DRIFT_OFF"
        if self._last_overflow:
            return "OVERFLOW"
        return "UNKNOWN"

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