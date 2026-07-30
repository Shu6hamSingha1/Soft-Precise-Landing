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
from planar_map import PlanarFeatureMap

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
        self._controller_was_ready = False   # edge-detects the False->True transition (see run())

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
        # KF-REFIT (2026-07-11): the SAME 5 2026-07-03 recordings, re-derived with
        # tools/aggregate_calibration_phased.py after fixing a filter mismatch —
        # derive_board_cal.py/derive_ring_cal.py/aggregate_calibration_phased.py were
        # Savgol(13,1)-filtering the raw signal to fit the cal, while runtime has
        # defaulted to KF filtering (IMG_FILTER=kf) since 2026-06-06 (commit 6e0b44f).
        # A static linear cal fit on a Savgol-shaped signal and applied to a
        # differently-shaped KF signal is not guaranteed valid; on this dataset it
        # moved every diagonal entry, most sharply the loom row (0.4973->1.1279,
        # +127%) and w_z (0.8439->1.3879, +64%) — h_x/h_y/s_x/s_y moved ~4-10%.
        # See feedback_kf_savgol_cal_mismatch. Off-diagonal loom cross-terms
        # (row 2, cols 0-1) are an older board-fit and NOT re-derived by this
        # tool (diagonal-only); kept as-is pending a future full-matrix KF refit.
        # ⭐ OBSERVER-REBUILD RECAL (2026-07-17) — the rows below are re-derived from a FRESH
        # 5-recording set (calibration_data/output_obskf_q1e3, all 5 runs valid) taken with the
        # CURRENT binary, superseding the 2026-07-03 set. WHY: the 07-03 recordings predate
        # commit f790245 (2026-07-04), which rebuilt the observer path in THREE ways at once --
        # (a) the CV-Kalman filter replaced the polyfit centroid-rate (attenuates -> needs MORE
        # gain), (b) W_Z SIGN FIX (_oz = -_wv[2]), (c) FRAME-PAIR FIX (quats[0] not quats[1]).
        # The observer's rate estimator runs INSIDE img_data UPSTREAM of the logged "raw"
        # (_obs_vel_kf ~:2027 -> _observer_flow ~:2054), so its smoothing is BAKED INTO the
        # recording and the aggregator CANNOT re-filter it offline -- hence new recordings were
        # required (unlike the outer KF, which tools/aggregate_calibration_phased.py re-applies).
        # The 07-11 KF-refit did NOT catch this: it only fixed the OUTER Savgol-vs-KF mismatch.
        # Old (Jul-3) -> new (Jul-17), per-run n=5, broken Jul-3 run excluded (loom -0.009 /
        # w_z -0.027 = failed z/yaw phases; MAD-rejected by the tool too):
        #   h_y  1.063+-0.198 -> 1.617+-0.283  (+52%, Welch t=+3.59, ranges DISJOINT)  [established]
        #   w_z  1.388+-0.123 -> 1.098+-0.025  (-21%, t=-4.65, DISJOINT)               [established]
        #   h_x  1.091+-0.266 -> 1.572+-0.331  (+44%, t=+2.54, ranges OVERLAP)         [direction
        #        consistent w/ h_y + mechanistically expected, but MAGNITUDE not nailed at n=5]
        #   loom 1.128+-0.036 -> 1.127+-0.018  (-0.1%, t=-0.06)  <- CONTROL: the moment loom is
        #        polyfit-based and was NOT touched by f790245; it reproduces to 0.1% across two
        #        independent sets 2 weeks apart => the two campaigns agree where the code didn't
        #        change and diverge ONLY on the 3 observer channels => the shift is the observer
        #        rebuild, not drift/noise//a global recording difference. Loom kept at 1.1279.
        # Consequence of the stale rows: lateral flow h=v_lat/Z was UNDER-read ~1/3 and yaw rate
        # OVER-read ~21% from 07-04 to 07-17 -- h_x/h_y feed the terminal sigma-breach signal.
        # Derived at the BAKED CENTROID_RATE_KF_Q=1e-3; a q sweep needs its OWN recording set per
        # q (not offline-refittable, same reason as above). See feedback_observer_rebuild_recal.
        self._sensor_cal_hw = np.array([
            [+1.5724, +0.0000, +0.0000, +0.0000, +0.0000, +0.0000],   # h_x = observer beta_x (obs-rebuild recal 2026-07-17; was 1.2833 = pre-CV-KF polyfit-era fit)
            [+0.0000, +1.5889, +0.0000, +0.0000, +0.0000, +0.0000],   # h_y = observer beta_y (obs-rebuild recal 2026-07-17; was 1.0630)
            [+0.0535, -0.0044, +1.1279, +0.0000, +0.0000, +0.0000],   # loom row (UNCHANGED — new fit 1.1266 confirms the baked 1.1279 to 0.1%; cross-terms board-fit; control uses the MOMENT loom via the row-2 cal BYPASS, see the _loom_decouple sites)
            [+0.0000, +0.0000, +0.0000, +0.0000, +0.0000, +0.0000],
            [+0.0000, +0.0000, +0.0000, +0.0000, +0.0000, +0.0000],
            [+0.0000, +0.0000, +0.0000, +0.0000, +0.0000, +1.0879]])   # w_z decoupled to w_z_raw only (obs-rebuild recal 2026-07-17; was 1.3879 = pre-sign/frame-fix fit)
        self._sensor_cal_hw[2, 2] = float(os.environ.get("PLASMC_LOOM_CAL", str(self._sensor_cal_hw[2, 2])))  # A/B knob: default = baked 1.1279 (KF-refit, 2026-07-11); PLASMC_LOOM_CAL=0.4973 reverts to the pre-refit (Savgol-derived) value
        if os.environ.get("PLASMC_WZ_CROSS", "0") == "1":   # restore the full old w_z cross-coupling (A/B)
            self._sensor_cal_hw[5, 0:5] = [+0.0526, +1.0862, -0.0096, -0.7395, +0.0161]
        self._sensor_cal_s  = np.diag([1.0491, 1.0108, 1.0, 1.0])   # single-marker-moment centroid (obs-rebuild recal 2026-07-17, same 5-run set as _sensor_cal_hw above; was 1.0215/1.0494 from the 07-11 KF-refit of the 07-03 set). s moved only +2.7%/-3.7% (it is NOT observer-sourced — raw moments -> outer feature KF, which the aggregator re-applies offline; re-derived here only to keep one coherent per-set fit). s[2]=homogeneous const, s[3]=alpha: BOTH identity BY DESIGN, not placeholders — the yaw chain is built on RAW moment-alpha (offset MOMENT_ALPHA0 upstream + gain BODY_YAW_ALPHA_K=-0.949 downstream at controller.py:2219); setting s[3]!=1 double-counts there, silently retunes the yaw SMC loop gain, rescales YAW_ALPHA_MAX_RATE/YAW_HOLD_ALPHA_RATE/YAW_KF_R, and breaks the GT-FB ablation (GT substitutes POST-cal). See project_yaw_calibration_pending (RESOLVED).
        # MAP-SOURCED centroid gain (2026-07-20, see feedback_map_cal_validation_gap /
        # feedback_map_override_kf_ordering_bug): derive_board_cal.py's new 'Centroid Map Raw'
        # fit (5 runs, 47025 fired samples, POST the ordering-bug fix so the override is
        # actually live) measured mx=1.1235/my=1.0946 -- DIVERGES from decode's sx=1.0491/
        # sy=1.0108 above by ~7-8%. The map's diagonal-intersection center is a structurally
        # different computation from decode's corner-mean (see _centroidMap docstring), so
        # reusing decode's gain under-corrected it. cal_s is applied ONCE at readout to the
        # KF's already-time-blended state (see getImgFeatureParam), so an exact per-sample
        # source split isn't possible there -- _effectiveCalSxy() below blends decode's/map's
        # gain by THIS FRAME's centroid-map trust (self._cmap_trust_log[-1]) as the best
        # available approximation, consistent with the same "cal assumes a slowly-varying
        # signal" approximation the KF-vs-raw cal fit already makes elsewhere.
        self._sensor_cal_s_map_xy = np.array([1.1235, 1.0946])

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
        # Ring cal below is still used when FLOW_FUSE_RING=1 (default OFF as of 2026-07-09 --
        # see the _fuse_ring assignment for why -- ring loom is unreliable near terminal/contact).
        # SINGLE-MARKER ring cal (2026-06-23, transfer mode keyed to the new corner cal).
        # OBSERVER ring cal (2026-07-03, 5 runs): h-block R² 0.72/0.82/0.66 (up from 0.34). The ring-yaw
        # (Wz) row is ZEROED — its derived gain was a 9.6 runaway (ring sees yaw weakly); the corner
        # marker provides yaw. Re-derive Wz if a board/turning case needs ring yaw.
        # KF-REFIT (2026-07-11): same 5 recordings, re-derived via derive_ring_cal.py
        # (transfer mode, keyed to the KF-refit corner M above) after the same
        # Savgol->KF filter-mismatch fix (see feedback_kf_savgol_cal_mismatch).
        # h-block R^2 0.81/0.88/0.79 (up from 0.72/0.82/0.66); magnitudes moved
        # substantially (Hx 0.65->1.23, Hy 0.80->1.54, loom 2.05->4.49). Wz row
        # is STILL a runaway under the refit (inter-run STD up to ~8, entries to
        # ±16) — zeroed again, same precedent as before; this channel is inert
        # unless FLOW_FUSE_RING=1 regardless.
        self._sensor_cal_ring = np.array([
            [+1.2283, -0.0165, +0.1649, +0.0111, +1.1254, -0.2802],
            [-0.2799, +1.5367, -0.0634, -1.5646, -0.2271, +0.6139],
            [+0.3906, -0.2532, +4.4900, +0.0764, +0.3203, +0.0865],
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
        # NOT auto-loaded from a hardcoded default path (was: Images/aruco_board_layout.npy,
        # always loaded if present). That file hardcodes THIS deployment's marker IDs (currently
        # [0, 10]) -- a future deployment with different IDs would silently load a WRONG,
        # non-corresponding prior during the ~5-frame self-cal warm-up window (BOARD_SELFCAL is
        # ID/size-free and default-ON; the single-marker fallback bridges that window harmlessly
        # with no prior at all). Require an EXPLICIT ARUCO_BOARD_LAYOUT path if a known-correct
        # prior for the CURRENT deployment's IDs is actually wanted; default is pure self-cal.
        # 2026-07-11, see feedback_board_layout_file_deployment_risk.
        _layout_path = os.environ.get("ARUCO_BOARD_LAYOUT", "")
        if _layout_path:
            try:
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
        # REVERTED to default-off (2026-07-28): the 2026-06-23 default-on flip was for a
        # DIFFERENT, uncommitted single-marker world+cal experiment that itself FAILED the
        # IC2-5 gate (2/12 sub, 6/12 fly) -- see the git-committed comment history on this
        # line. Nothing in the standard launch path (run_aruco_landing.sh/run_ic_validation.sh)
        # ever set PLASMC_SINGLE_MARKER=0, so this default has been SILENTLY ACTIVE against the
        # nested board for over a month: every standard IC1-5 gate/A-B since 2026-06-24 ran
        # reactive single-marker LOCK (switch only once the current marker fully vanishes)
        # instead of the board's intended PROACTIVE bigger-marker-priority handoff (the branch
        # below, itself labeled "(default)" -- was unreachable dead code this whole time). The
        # CBF small-marker preference, HANDOVER_LATCHED terminal commit, and marker-switch
        # KF-reset machinery are all designed around the proactive handoff, not lock mode.
        # See project_single_marker_default_mismatch_20260728 memory for the A/B.
        self._single_marker = os.environ.get("PLASMC_SINGLE_MARKER", "0") == "1"
        self._locked_marker_id = None   # the locked single marker (None = unacquired)
        # PlanarFeatureMap SHADOW mode (2026-07-15, default-on, PLANAR_MAP_SHADOW env-gated).
        # Runs the online KLT+homography scene map ALONGSIDE the existing ArUco/KLT-fallback/
        # dense-recovery tiers with ZERO effect on aruco_pts_0/extra_pts_0/marker_ids or any
        # downstream control decision -- pure side-by-side logging, per the user's explicit
        # build-order choice (validate on LIVE SITL data before any switch-over, offline
        # tools/validate_planar_map.py isn't enough on its own). See src/planar_map.py.
        #
        # NOT constructed here. A 2026-07-15 SITL run found 65-110px held-out prediction
        # errors confined entirely to the PRE-ENGAGE phase (hover/takeoff climb, well before
        # CONTROLLER_READY) -- fast/large camera motion during climb stresses the RANSAC
        # homography fit into an occasionally-bad-but-locally-self-consistent solution
        # (rigid_ok/confidence didn't catch it). The controlled descent itself was clean
        # (max 3.61px). Since deployment has no way to know when the target first becomes
        # visible except "somewhere during the descent" anyway, there's no reason to run
        # (or accumulate potentially-corrupted state from) the pre-engage phase at all --
        # the map is (re)constructed fresh in _reset_stateful_trackers(), exactly on the
        # CONTROLLER_READY False->True transition, same as every other control-decision
        # stateful tracker there.
        self._planar_map_shadow = os.environ.get("PLANAR_MAP_SHADOW", "1") == "1"
        # PlanarFeatureMap PRIMARY BAKE (2026-07-15, widened from the initial loom-only bake
        # same day): promotes the map from pure shadow logging to the actual SINGLE-FRAME
        # corner-position source for BOTH consumers that only need one frame's position --
        # the centroid (s/alpha, via V_aruco_norm[1] -> _getImgFeatures) AND the moment-loom
        # (_loom_decouple's `_vp`/`_M`) -- overridden together at ONE point (V_aruco_norm[1]
        # itself, right after it's computed) so the two consumers can never disagree about
        # which corner source produced this frame's estimate.
        #
        # Root bug this fixes: a 2026-07-15 SITL trace showed the raw per-frame ArUco decode
        # jumping ~50-80px instantaneously whenever the small<->big nested-marker decode
        # flickers (the two markers are marginally co-decodable near their handover altitude
        # band). Specifically for the loom, this fed the Δln(M) outlier-hold guard
        # (LOOM_DLNM_MAX) -- a one-shot-transient design that gets stuck holding a stale/
        # spurious value under a SUSTAINED flicker (every frame looks like a fresh "outlier"
        # vs the last) -- producing a sustained, bit-frozen, sign-wrong h_z through ordinary
        # mid-descent (independently confirmed against ring loom, which stayed smoothly
        # negative throughout). PlanarFeatureMap's primary-slot corner prediction is
        # CONTINUOUS across a flicker (both physical markers are tracked every frame via
        # KLT+homography regardless of which one decodes; a fresh ArUco decode only LOOP-
        # CLOSURE-corrects its own slot, in place, never resetting the other) -- so sourcing
        # V_aruco_norm[1] from it instead of the raw per-frame decode removes the jump at the
        # root, for the centroid too (which read the exact same flicker-prone array).
        #
        # Held-out validation (tools/validate_planar_map.py, offline): 1.3-1.7px mean / <4px
        # max error even across 70+-frame decode gaps and a live flicker window. A 2026-07-15
        # SITL shadow run found large (65-110px) errors, but CONFINED to pre-engage hover/climb
        # (fast camera motion breaks the RANSAC homography fit) -- the controlled descent itself
        # was clean (max 3.61px). Gated to start only post-CONTROLLER_READY (map is (re)built in
        # _reset_stateful_trackers, same as every other control-decision stateful tracker) and
        # requires self._planar_map.map_confidence (marker-independent, see planar_map.py)
        # before trusting the prediction -- falls back to the untouched raw ArUco decode
        # otherwise (identical to pre-bake).
        #
        # NOT (yet) touched: aruco_pts_0/extra_pts_0/flow_pts_0/1/V_flow_norm -- the frame-PAIR
        # flow lstsq (lateral h_xy + rotational w) stays on the raw multi-tier corner
        # correspondence. PlanarFeatureMap's public API predicts a single frame's position
        # (get_marker_frame_pts()), not a frame-pair; extending the override to the flow lstsq
        # would need new map-side machinery (a two-frame query) not yet built -- attempting it
        # by reusing the single-frame API would desync the lstsq's corner correspondence, a
        # correctness risk with no held-out validation behind it. Also NOT touched: marker
        # locking (_locked_marker_id selection stays on raw decode identity+spread; the map's
        # slot routing is independent and geometric, so it doesn't need the lock to agree).
        self._planar_map_primary = (os.environ.get("PLASMC_PLANAR_MAP_PRIMARY",
                                     os.environ.get("PLASMC_PLANAR_MAP_LOOM", "1")) == "1")
        self._planar_map_conf_floor = float(os.environ.get("PLANAR_MAP_CONF_FLOOR", "0.5"))
        # SOFT rigidity/confidence gate for the OVERRIDE consumers (_centroidMap/_alphaMap,
        # 2026-07-19 — see feedback_soft_rigidity_gate). A hard reject at _planar_map_conf_floor
        # (0.5) fixed the corrupted-geometry blowup class (IC4: err_px=1281/confidence=0.0
        # frame feeding a bad value through) but a validation rerun then showed the opposite
        # failure: IC3 lost the marker for an extended real stretch where map confidence sat
        # in the 0.0-0.5 band the whole time, and hard-rejecting ALL of it left the override
        # contributing nothing for 4+ seconds -- pure KF coast, 13m miss. Below
        # PLANAR_MAP_REJECT_FLOOR (default near-zero -- exactly where the diagnosed garbage
        # frames sat, confidence=0.0) still hard-rejects (genuinely nothing to trust). Between
        # the two floors, BLEND the map value with decode proportional to confidence instead
        # of an all-or-nothing swap -- a partially-degraded slot still carries some signal,
        # just not full trust.
        self._planar_map_reject_floor = float(os.environ.get("PLANAR_MAP_REJECT_FLOOR", "0.05"))
        # WINDOWED-RATE plausibility gate for _centroidMap/_alphaMap (2026-07-21, see
        # feedback_map_rate_plausibility_gate): confidence measures internal RIGIDITY of the
        # tracked point cluster, not whether it's still anchored to the real target -- a
        # coherently-drifting-but-rigid cluster can hold confidence=1.0 while genuinely
        # diverging (diagnosed: IC3's 158m fly-away and a 13m IC1 miss both showed the map's
        # OWN raw centroid accelerating to ~0.8-1.1/s in its final accepted frames while trust
        # stayed pegged at 1.0 throughout). This is a SEPARATE axis from confidence, checked
        # via a short rolling-window slope fit (same pattern as the loom's d(lnM)/dt gate,
        # _mtrace_hist) on the map's own accepted history -- independent of what the map
        # itself reports about its confidence. Two tiers, mirroring the reject/full-trust
        # confidence floors: below MAP_RATE_SOFT, full trust; between SOFT and REJECT,
        # tapered trust (folded into the SAME _cmap_last_trust/_amap_last_trust the
        # confidence-scaled KF r already uses -- the value itself is NOT touched, map stays
        # authoritative for what's reported); above MAP_RATE_REJECT, hard reject (None), same
        # tier as a position/size plausibility failure. Centroid defaults derived from
        # tonight's normal-vs-failure separation (normal fired stretches: ~0.02-0.04/s; both
        # centroid failures: ~0.8-1.1/s in their final frames -- a 20-50x gap).
        self._map_rate_soft = float(os.environ.get("MAP_RATE_SOFT", "0.3"))
        self._map_rate_reject = float(os.environ.get("MAP_RATE_REJECT", "0.8"))
        # ALPHA gets its OWN thresholds + a LONGER averaging window (2026-07-23, see
        # feedback_alpha_rate_gate_separate_thresholds) -- reusing the centroid numbers/window
        # length here was a mistake, found live: an IC3 rep's alpha drifted at a sustained
        # ~0.26 rad/s for ~26 deg of uncorrected yaw error (root cause of a real miss), well
        # UNDER MAP_RATE_SOFT=0.3 (never tapered) using the short 3-5 sample window shared
        # with centroid. Checked why: decode alpha's own INSTANTANEOUS per-sample jitter
        # (median|rate| 0.24/s, p95 0.83/s on a normal, non-drifting stretch of the SAME
        # flight) already spans/exceeds the centroid-derived thresholds -- a short window is
        # too noise-sensitive for alpha to separate real drift from ordinary jitter. Over a
        # LONGER window (~0.35s, MAP_ALPHA_RATE_HIST=15 samples vs centroid's 5) the net
        # (not instantaneous) drift rate DOES separate cleanly: normal stretch net-drift-rate
        # median 0.10/s, max 0.22/s (bounded -- genuine jitter cancels out over the longer
        # window); the diagnosed failure's sustained net rate was 0.61/s -- ~3x the normal
        # ceiling. Soft/reject set to bracket that gap (worse separation margin than
        # centroid's 20-50x, still provisional -- validate at n>=5 before trusting further).
        self._map_alpha_rate_soft = float(os.environ.get("MAP_ALPHA_RATE_SOFT", "0.25"))
        self._map_alpha_rate_reject = float(os.environ.get("MAP_ALPHA_RATE_REJECT", "0.5"))
        self._map_alpha_rate_hist_len = int(os.environ.get("MAP_ALPHA_RATE_HIST", "15"))
        self._cmap_rate_hist = []   # [(t, x, y), ...] last few ACCEPTED centroid-map samples
        self._amap_rate_hist = []   # [(t, sin(alpha), cos(alpha)), ...] -- circular-safe, LONGER window (see above)
        # Physical-plausibility rejection margin for the rescue (2026-07-16, see rescue
        # site comment): a rescue-projected centroid is only trusted if it falls within
        # this multiple of the true FoV-edge bound (self.center/focal + last-held marker
        # half-extent) -- 1.5x allows some legitimate near-edge slack without accepting a
        # position that's geometrically nonsensical for this camera.
        self._planar_map_rescue_fov_margin = float(os.environ.get("PLANAR_MAP_RESCUE_FOV_MARGIN", "1.5"))
        # SIZE-plausibility check (2026-07-16, user correction): the position-only rejection
        # gate above missed a genuinely distinct failure mode -- the map's homography can
        # drift in SCALE while still projecting a position that lands within the FoV bound
        # (nothing in map_confidence validates scale, only tracking density + residual).
        # Traced live (IC5): during a corner-loss window the rescue implied a marker extent
        # of ~96.6px, frozen for ~300ms with h_z drifting smoothly upward (0.37->0.44) the
        # whole time -- then the INSTANT raw tracking recovered, the true extent was only
        # ~15px, a 6x mismatch. That phantom size feeds the SAME loom (_loom_decouple)
        # moment-size trend the earlier flicker fix was built to protect, just corrupted by
        # a wrong scale instead of a wrong marker identity. self._last_real_extent_px (set
        # only on genuine raw decode, below) is the reference; a rescue whose implied
        # extent deviates beyond this ratio (either direction) is rejected, same as an
        # implausible position.
        self._planar_map_rescue_size_ratio = float(os.environ.get("PLANAR_MAP_RESCUE_SIZE_RATIO", "2.0"))
        self._last_real_extent_px = None
        # HYSTERESIS (2026-07-15, found via IC1-5 n=5 SITL validation of the primary bake):
        # a raw confidence>=floor check has NO persistence, so ordinary per-frame confidence
        # noise (marker_rigid_ok toggling True/False on small KLT shape-signature jitter,
        # NOT a real tracking event -- confirmed in Planar Map Shadow logs oscillating
        # 0.78-0.91 <-> 0.0 every few frames on an otherwise-healthy, fully-corner-tracked
        # rep) makes the gate flip the V_aruco_norm[1] SOURCE between map-predicted and
        # raw-ArUco every few frames -- each flip is a small discontinuity, exactly the SAME
        # CLASS of bug (repeated source-switch flicker) as the small<->big marker-decode
        # flicker this whole effort was built to fix, this time self-inflicted by the
        # gate's own threshold noise.
        # CORRECTION (2026-07-15, later same session): the s_e_n=[0,0]-then-jump event
        # originally cited here as live confirmation (IC3_rep4, ICValidation/20260715-192025)
        # was RE-TRACED and found to occur 0.028s after ctrl_time[0] -- i.e. the very first
        # logged control-loop frame right after CONTROLLER_READY engagement (logging starts
        # there, not at motor-arm), not a genuine mid-flight event. s_e_n=[0,0] before the
        # first real measurement lands is an EXPECTED engagement transient, not this gate
        # firing -- that specific trace does NOT constitute confirmed live evidence of the
        # bug. The confidence-oscillation mechanism itself (marker_rigid_ok jitter -> gate
        # flip -> source-switch discontinuity) is still a real, demonstrable code property
        # independent of when in a flight it triggers, so the fix below is kept as a sound,
        # low-risk improvement on code-reasoning grounds -- but treat it as UNVALIDATED by a
        # confirmed live reproduction until re-traced away from an engagement window.
        # Fix: distrust the map IMMEDIATELY on any bad frame (safe default, falls back to
        # the proven raw path), but require PLANAR_MAP_GATE_ON_FRAMES consecutive good
        # frames before trusting it again (mirrors the existing FEATURE_IS_STALE clear
        # hysteresis pattern elsewhere in this file) -- damps rapid oscillation around the
        # confidence floor into a single, non-repeating transition.
        self._planar_map_gate_on = False
        self._planar_map_gate_streak = 0
        self._planar_map_gate_on_frames = int(os.environ.get("PLANAR_MAP_GATE_ON_FRAMES", "5"))
        # TWO SEPARATE GATES (2026-07-16, found via IC1/IC4 SITL regression after the
        # map_confidence split above): _planar_map_gate_on (map_confidence, marker-
        # INDEPENDENT) is correct for the RESCUE path (not FEATURE_DATA_IS_LOGGED -- no
        # raw reading exists to protect, any grounded estimate beats blind extrapolation).
        # But the SAME flag was ALSO gating the V_aruco_norm[1] OVERRIDE inside
        # `if FEATURE_DATA_IS_LOGGED:` -- i.e. REPLACING an already-successful, fresh raw
        # ArUco decode with the map's prediction. That's a fundamentally different
        # decision: only worth doing if the map's MARKER-SPECIFIC tracking is ALSO
        # verified trustworthy (self.confidence, marker-aware), not just the general
        # scene's global tracking health. Using the permissive map_confidence there meant
        # the override could now fire on frames where the raw decode was perfectly fine,
        # replacing accurate fresh corners with a possibly slightly-drifted map estimate --
        # confirmed live: IC1/IC4 both showed a smooth, GROWING divergence (0.64->1.5m /
        # 0.30->3.7m) while n_corners stayed nonzero throughout (no corner-loss event at
        # all), consistent with a subtly-wrong position being fed in on otherwise-healthy
        # frames, not a perception dropout. _planar_map_override_gate_on is the separate,
        # STRICTER gate for that use case -- back to the marker-aware self.confidence.
        self._planar_map_override_gate_on = False
        self._planar_map_override_gate_streak = 0
        self._planar_map = None
        self._planar_map_primary_pred_px = None   # current-frame camera-pixel primary-slot prediction, refreshed each frame the map runs
        self._planar_map_log = []   # per-frame dicts: t, err_px (vs whatever tier won this frame), decode_calls, confidence
        self._planar_map_decode_calls = 0
        # Default 0 (pre-engage): the savgol paths' post-engage clip is a no-op until the
        # first CONTROLLER_READY transition actually sets these in _reset_stateful_trackers.
        self._engage_idx_flow = 0
        self._engage_idx_feat = 0
        # Visibility-by-MARGIN: when ANY marker corner comes within this many px of the FoV edge
        # (the marker is LEAVING/overflowing the frame), the corner is DROPPED (aruco_pts_0=None) ->
        # corner_ok stays False -> the fusion EKF lets the RING carry ALL flow components. One
        # mechanism (no separate visible-flag): the margin switches to rings a few frames BEFORE the
        # marker fully leaves, off cleaner (non-edge) corners. 0 = strict (switch only on full exit).
        self._marker_fov_margin = float(os.environ.get("PLASMC_MARKER_FOV_MARGIN", "40"))
        # REJECT-OVERFLOW-FOR-MAP-CORRECTION (2026-07-19, user). As the drone tilts, the marker
        # slides to the FoV edge and partially overflows -> its de-rotated quad degenerates to a
        # near-collinear sliver (diag angle 1.3deg, elongation up to 70x -- verified) whose
        # geometry no longer localizes the centre. Feeding such a degenerate DECODE frame into the
        # map's loop_closure_correct POISONS the map's learned geometry (frame_to_map/gauge) with a
        # near-singular observation. The map is PRIMARY: it should PREDICT through the overflow on
        # its CLEAN last-good state, not CORRECT from the degenerate view. So when the decoded
        # corners are at/over the FoV edge (overflow), SKIP the correction -- keep the map's geometry
        # uncorrupted so it can bridge the terminal. Decode stays the cross-check ONLY on
        # well-conditioned frames. Default off; MAP_REJECT_OVERFLOW_CORRECT=1 to enable.
        self._map_reject_overflow_correct = os.environ.get("MAP_REJECT_OVERFLOW_CORRECT", "1") == "1"
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
        # NO hard step-count ceiling on the KLT-fallback ATTEMPT itself (2026-07-23, see
        # feedback_klt_soft_cap -- REMOVED the ceiling after a deliberate analysis, not an
        # oversight). _max_lk_steps above is now PURELY the confidence-decay reference
        # (_decode_conf in _kf_feat_update floors at 0.05 once lk_step_count reaches it, never
        # zero). It used to ALSO hard-stop the KLT attempt entirely at that same count, which
        # produced a total, sustained blackout (confirmed live: 700ms with zero corners, frozen
        # _prev_aruco_pts/_prev_img, because KLT was NEVER RE-ATTEMPTED past step 20 even though
        # it was still tracking cleanly, in-bounds, 4/4, right up to the cutoff). A first fix
        # added a separate, much larger hard ceiling (MARKER_KLT_HARD_MAX_STEPS=200) -- but that
        # number was arbitrary, and the analysis that justified it doesn't hold up: the thing a
        # step-count ceiling used to guard against (a FULLY-TRUSTED KLT chain drifting
        # indefinitely with no fresh anchor) can no longer happen once confidence decays with
        # streak length -- by the time drift could matter, trust is already at its 0.05 floor.
        # The REMAINING risk (an unboundedly long, floor-trust chain still contributing a small
        # nonzero correction every frame, forever) is real but much smaller/slower than what the
        # original cap guarded against, and there is already a SEPARATE, principled backstop for
        # genuine total marker loss: apps/landing_test.py's LANDING_MARKER_LOSS_GRACE=1.0s
        # (TARGET_LOST abort). That mechanism handles "nothing to track"; this one is specifically
        # for "still tracking something, just decreasingly trusted" -- the two are not redundant,
        # but stacking an arbitrary second ceiling on top of a working confidence decay added risk
        # (an early, unjustified blackout) without a clearly justified benefit. User directive
        # (2026-07-23): remove it. KLT's own in-bounds/reconstruction-quality checks remain the
        # real per-frame safety net (see the elif's in_bounds abort just below).
        self._lk_step_count = 0
        # RELAXED KLT-FALLBACK GATE (2026-07-09; BAKED default-ON 2026-07-09 for the n>=5
        # IC1-5 controller-tuning validation pass): the strict all-4-corners-tracked requirement
        # discarded an otherwise-good 3/4-tracked frame outright. KLT Diag logging (added same
        # day) showed a "momentary flicker" regime (single corner losing LK lock for 1-2 frames
        # during otherwise-clean tracking) where 100% of gate4-rejections were exactly 3/4
        # tracked. Set MARKER_KLT_RELAX_GATE=0 to revert to the strict 4-corner requirement.
        self._klt_relax_gate = os.environ.get("MARKER_KLT_RELAX_GATE", "1") == "1"
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
        # BAKED default-ON (2026-07-28): isolated n=25/24 A/B (dense-recover OFF vs ON, IC1-5)
        # showed a consistent net improvement -- TARGET_LOST rate 60%->46%, mean xy 1.11->0.92m,
        # 0 DESCENT_ANOMALY vs 1, fewer UNKNOWN failure causes (11->6). Per-IC: improved on
        # IC1-4; IC5 alone got a slightly worse TARGET_LOST rate (4/5->5/5, but mean xy still
        # improved 2.94->2.13) -- consistent with IC5's already-catalogued structural issue
        # (marker leaves the FoV entirely, not partial occlusion) being outside what a
        # partial-view homography recovery can fix. See project_dense_recover_ab_20260728
        # memory; closes out the previously "mixed, don't bake" status from 2026-07-07.
        self._dense_recover = os.environ.get("PLASMC_DENSE_RECOVER", "1") == "1"
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
        # RUNTIME ATTITUDE-RATE GATE (2026-07-26, see the flow-solve site's comment for the
        # full derivation): threshold on the real V-frame roll/pitch RATE (from the FC gyro,
        # rad/s) above which the reduced solve's w_xy≈0 premise is considered violated for
        # THIS frame, falling through to the full 6-DOF solve regardless of FLOW_TARGET_LEVEL.
        # 0.3 rad/s ≈ 17 deg/s -- comfortably above ordinary descent attitude noise/settling
        # (per this project's own KLT/LK breakage threshold references, ~1-2 rad/s is already
        # "aggressive maneuvering"; a real ignited tumble measured this session climbed well
        # past 1 rad/s within under a second) but well below that, so it should catch the
        # ignition EARLY rather than only once already deep into a tumble. Provisional --
        # validate at n>=5 before treating this value as settled.
        self._lat_reduced_wmax = float(os.environ.get("FLOW_LAT_REDUCED_WMAX_RADS", "0.3"))
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
        # CBF-DRIVEN HANDOVER PATH (2026-07-17, user design): a SECOND, independent way to
        # latch HANDOVER_LATCHED, alongside the loom-M-drop detector above. The CBF's own
        # per-corner FoV-margin overflow classification (CBF_OVERFLOW, controller.py) +
        # PlanarFeatureMap's small-slot confidence is a geometrically-direct "big has
        # overflowed AND small is trustworthy" signal -- driven from landing_test.py each
        # step via update_cbf_handover_signal() (apps/landing_test.py owns both the
        # controller and this img_node, so it's the natural place to bridge the two
        # modules; img_data.py and controller.py stay one-directionally decoupled).
        # Deliberately does NOT touch _primary_id/primary_i (the raw-decode marker
        # selection) -- that must stay big-priority for h_x/h_y flow observability (see
        # IMG_MARKER_PRIORITY doc above); this only affects the terminal-phase
        # HANDOVER_LATCHED flag that _ringCommitStep gates on.
        self._handover_cbf_cand = 0
        self._cbf_small_conf_min = float(os.environ.get("CBF_SMALL_SLOT_CONF_MIN", "0.5"))   # SAME env/default as controller.py -- keep the two decisions consistent
        # HANDOVER via SMALL-SLOT CONFIDENCE ALONE (2026-07-17, user directive). The two
        # existing latch paths both require a DECODE event that didn't happen in the textured
        # IC1-5 gate (ICValidation/20260717-185746, handover=0 in every rep): (1) the loom-M-drop
        # path needs the small marker to stay primary 5 frames without the big RE-DECODING, but
        # the nested marker FLICKERS big<->small (3 down + 3 up d(lnM) crossings), so each
        # candidate is cancelled before persisting; (2) the CBF path ANDs small-slot-confident
        # with CBF_OVERFLOW, which fired 0 times. The map, unlike raw decode, tracks the small
        # slot CONTINUOUSLY through the flicker (KLT+homography) -- so its per-slot confidence is
        # the flicker-immune "we can rely on the small marker now" signal. When
        # HANDOVER_REQUIRE_OVERFLOW=0, latch on SMALL_SLOT_CONFIDENT sustained alone (no
        # CBF_OVERFLOW gate). Still one-way + HANDOVER_PERSIST-frame hysteresis (the skill's
        # IC4 DRIFT_OFF regression came from a RAW per-frame slot-confidence read -- keep the
        # persistence). Self-gating vs firing at altitude: get_slot_confidence folds in
        # track_conf (n_tracked corners), and the tiny inner marker isn't well-tracked until
        # close, so confidence naturally stays low until the terminal. Downstream, ring-commit
        # STILL requires OVER_TARGET + settled, so an early latch cannot itself commit.
        self._handover_require_overflow = os.environ.get("HANDOVER_REQUIRE_OVERFLOW", "0") == "1"
        # MAP-DRIVEN LOOM M (2026-07-18, user directive: "replace all decode-driven measurements
        # with map-driven"). STAGE 1: the loom scale M=μ20+μ02 is computed from the PlanarFeatureMap's
        # continuously-tracked slot corners (get_marker_frame_pts) instead of the raw-decode corners.
        # WHY: the decode primary flickers big<->small, and the raw M then jumps (~ratio²) -> spurious
        # positive h_z mid-descent (the touchdown-detection root: h_z isn't cleanly negative through
        # the descent, so a descent-arrest detector can't work). The map tracks each slot through the
        # flicker, so its M is continuous. SLOT = big (primary) before HANDOVER_LATCHED, small
        # (secondary) after -- the user's big-before/small-after selection; the two slots share one
        # map with the online-learned relative size, so the scale is continuous across the single
        # latched switch. Isolated to the LOOM's M only (centroid _x0/_y0, flow, alpha stay on decode)
        # so this stage validates alone. Default off; LOOM_M_FROM_MAP=1 to enable.
        self._loom_m_from_map = os.environ.get("LOOM_M_FROM_MAP", "1") == "1"
        # MAP-DRIVEN CENTROID s[0:2] (2026-07-18, decode->map migration STAGE 2). The position
        # feature's centroid is taken from the map's continuously-tracked slot (get_marker_center)
        # instead of the raw-decode corner mean. The nested markers are CONCENTRIC (big/small share
        # a centre), so this is not a position shift -- it is CONTINUITY: the map tracks through the
        # decode dropouts/flicker that make the raw centroid jump/stale. Handover-gated slot (big
        # before HANDOVER_LATCHED, small after), same as the loom. Isolated to s[0:2]; alpha (s[3])
        # stays on decode until stage 3. Default off; CENTROID_FROM_MAP=1 to enable.
        self._centroid_from_map = os.environ.get("CENTROID_FROM_MAP", "1") == "1"
        # MAP-DRIVEN ALPHA s[3] (2026-07-19, decode->map migration STAGE 3). Yaw feature from
        # get_marker_points()'s handover-gated corner+dense-interior (position, weight) set via
        # the SAME _marker_principal_angle moment math (weights param added for this, no
        # duplicated logic) and the SAME _moment_alpha_0 offset, so a decode<->map fallback never
        # jumps convention. cal_s[3]=1.0 stays load-bearing (see project_yaw_calibration_pending)
        # regardless of source -- this changes WHERE alpha is measured FROM, not its calibration.
        # BAKED 2026-07-20 (n=1 IC1-5, post soft-gate fix -- see feedback_soft_rigidity_gate;
        # NOTE: below this project's usual n>=5 bar, baked on explicit user direction).
        # ALPHA_FROM_MAP=0 to disable.
        self._alpha_from_map = os.environ.get("ALPHA_FROM_MAP", "1") == "1"
        # MAP-DRIVEN SMALL-MARKER FLOW (2026-07-19, decode->map migration STAGE 4, user
        # directive). Flow (h_xy) is a VELOCITY DERIVATIVE -- needs corner-spread observability,
        # so it works best on a LARGE marker, but we can't just wait for the big marker to fill
        # the FoV (it degrades/overflows before that). So: switch to the SMALL marker once it is
        # confidently mapped AND its apparent size falls in the empirically-characterized
        # reliable BAND -- SEPARATE from and typically EARLIER than HANDOVER_LATCHED (which
        # gates loom/centroid/alpha). Characterized this session (textured IC1-5, decode flow vs
        # V-framed GT flow, time-synced): median error ~0.49 at ~8px extent, 0.06-0.07 in
        # 15-35px, back up to 0.17-0.18 past ~40px (edge/overflow degradation, same mechanism as
        # the loom/centroid). Defaults are THAT characterization; re-derive if camera/marker
        # geometry changes. BAKED 2026-07-20 (n=1 IC1-5 -- see feedback_soft_rigidity_gate;
        # NOTE: below this project's usual n>=5 bar, baked on explicit user direction).
        # FLOW_FROM_MAP=0 to disable.
        self._flow_from_map = os.environ.get("FLOW_FROM_MAP", "1") == "1"
        self._flow_map_min_ext = float(os.environ.get("MAP_FLOW_MIN_EXT_PX", "15"))
        self._flow_map_max_ext = float(os.environ.get("MAP_FLOW_MAX_EXT_PX", "40"))
        self._flowmap_kf_x = None; self._flowmap_kf_y = None   # own KF state, isolated from the observer's
        self._flowmap_kf_Px = None; self._flowmap_kf_Py = None; self._flowmap_kf_t = None
        # DURATION CAP (2026-07-30, ported from the Hardware/ fix for the same gap): _flowMap/
        # _loomMapM_slot only ever checked `pm.initialized` (true indefinitely) -- a map whose own
        # tracking has gone stale still lets a pure KF-prediction assert a confident nonzero
        # velocity forever. Real Pi hardware flights showed this dead-reckon a linearly-growing
        # s_e_n for ~400 frames, blowing up zeta/sigma/kappa even with the PID integral's
        # anti-windup clamp (a magnitude clamp, not a source fix). Reuse PlanarFeatureMap's own
        # staleness clock (map_confidence, decays to 0 over decode_staleness_max_seconds) instead
        # of adding a new counter -- below this floor, _flowMap/_loomMapM_slot refuse to answer.
        self._flowmap_min_confidence = float(os.environ.get("PLANAR_MAP_FLOW_MIN_CONFIDENCE", "0.1"))
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
        # RESCUE_ACTIVE (2026-07-16): whether THIS frame's s/alpha (and, transitively, the
        # loom h_z) came from the PlanarFeatureMap rescue rather than a raw ArUco/KLT/
        # dense-recovery decode. FEATURE_IS_STALE (above) counts consecutive RAW-tier
        # misses only -- it has zero visibility into whether the rescue is providing a
        # trustworthy substitute, so apps/landing_test.py's feature_fresh check (which
        # gates whether closed-loop control even runs) was treating "raw tiers failed but
        # the map rescue has a grounded estimate" identically to "genuinely no information
        # at all" -- confirmed live (IC5): the rescue kept producing a smooth, plausible
        # s(t) through a 1.5s corner-loss window, but FEATURE_IS_STALE flagged stale after
        # just 3 frames (~0.1s) and landing_test.py abandoned closed-loop control (grace-
        # hold, then TARGET_LOST) long before the rescue's estimate could ever be used.
        # This flag is the thread-through: apps/landing_test.py's feature_fresh now also
        # accepts RESCUE_ACTIVE, so a rescue-covered dropout keeps running getControlInput()
        # (which already consumes the rescued getImgFeatureParam()/getOptFlowAngVel()
        # output) instead of freezing on the last command or giving up entirely.
        self._planar_map_rescue_active = False
        # FEATURE_PTS_FRESH (2026-07-17): was _feature_pts (what MARKER_EXTENT_PX / the CBF's
        # d_min_fov read) updated with LIVE geometry this frame (raw decode/KLT OR a
        # plausibility-checked map rescue), vs held at its last value (genuine total coast,
        # neither raw nor rescue available)? Distinct from FEATURE_IS_STALE, which is a
        # RAW-miss-only counter with no rescue awareness -- gating on FEATURE_IS_STALE alone
        # blinds consumers during a run of frames the map IS successfully rescuing. See
        # feedback_cbf_staleness_and_rigidity_confidence memory.
        self._feature_pts_fresh = True
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
        # Per-frame ESTIMATOR TAG, lockstep-appended with the arrays above. Multiple distinct
        # computations feed the same raw h/s arrays (primary ArUco-decode lstsq, KLT-fallback
        # lstsq, the centroid-rate observer, board homography, single-marker moment, and the
        # predict-only "coast" during a marker-loss gap) -- getRawOptFlowAngVel/
        # getRawImgFeatureParam previously returned whichever fired with NO record of which, so
        # a calibration recording couldn't tell you which estimator(s) it actually exercised, or
        # exclude synthetic coast samples from the fit. See feedback_estimator_blind_calibration.
        # Tags — h: 'lstsq' (primary decode + KLT-fallback merged: same _fill_A/lstsq geometry
        # regardless of corner source, see feedback_klt_fallback_merge_no_separate_cal) |
        # 'lstsq+observer_xy' (h_z/w lstsq, h_x/h_y observer-overridden) | 'observer_full' (all
        # channels from the centroid-rate observer, no lstsq data this frame) | 'coast'
        # (predict-only extrapolation, NOT real data — exclude from any cal fit).
        # Tags — s: 'board_homography' | 'single_marker_moment' | 'coast' (exclude from fit).
        self._h_estimator_tag = []
        self._s_estimator_tag = []
        # RAW (PRE-CAL) map-sourced logs (2026-07-20 -- see feedback_map_cal_validation_gap):
        # _centroidMap/_alphaMap/_flowMap's OWN return value, logged every frame regardless of
        # whether the soft gate ultimately blended it in, BEFORE _sensor_cal_s/_sensor_cal_hw are
        # applied -- neither existed before this. _s_estimator_tag/_h_estimator_tag are set
        # BEFORE the map override/blend runs and never updated after, so they can't tell you
        # whether/how much a logged s/h was map-influenced; the *_map_dbg fields are whole-flight
        # tallies, not per-frame. Needed to validate whether _sensor_cal_s/_sensor_cal_hw (fit
        # against the DECODE path only) are still correct for these structurally different
        # computations (map diagonal-intersection center vs decode corner-mean; map's
        # corner+dense-interior-weighted angle vs decode's 4-corner-weighted angle; map's
        # single-point-velocity KF flow vs decode's corner-spread LSTSQ flow). NaN/0 when that
        # frame's map function didn't fire (gate rejected, unavailable, or the *_FROM_MAP flag
        # is off) -- lockstep length with self._s_estimator_tag/_h_estimator_tag.
        self._cmap_raw_log = []     # (2,) raw V-frame centroid from _centroidMap, or (nan, nan)
        self._cmap_trust_log = []   # soft-gate trust in [0, 1] used for this frame's blend, or 0.0
        self._amap_raw_log = []     # raw alpha (rad) from _alphaMap, or nan
        self._amap_trust_log = []   # soft-gate trust in [0, 1], or 0.0
        self._fmap_raw_log = []     # (3,) raw [hx, hy, hz] from _flowMap, or (nan, nan, nan)
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
        # Predict-only coast during a marker-loss gap (see _kf_step docstring): Q's dt uses the
        # TRUE elapsed gap (capped here), decoupled from the 0.1s state-transition dt cap, so P
        # correctly reflects staleness and a relock gets one correct Bayesian update instead of a
        # multi-frame catch-up ramp. Shared by the corner-flow and centroid-feature KFs.
        self._kf_dt_unc_max = float(os.environ.get("KF_DT_UNC_MAX", "2.0"))
        self._kf_x = np.zeros((6, 2))       # [value, rate] per channel
        self._kf_P = np.tile(np.eye(2) * 1.0, (6, 1, 1))
        self._kf_prev_t = None
        self._kf_initialized = False
        # Ring-flow KF — the SAME _kf_step filter, separate state (V_v_ring through it)
        self._kf_x_ring = np.zeros((6, 2))
        self._kf_P_ring = np.tile(np.eye(2) * 1.0, (6, 1, 1))
        self._kf_ring_prev_t = None
        self._kf_ring_initialized = False

        # FUSED corner+ring optical flow — augmented-state EKF (FLOW_FUSE_RING, default OFF
        # since 2026-07-09 — see the _fuse_ring assignment below).
        # ⚠ 2026-07-09 (user-led history dig): the ego/ground-vs-target separation described
        # below is the ORIGINAL DESIGN ASPIRATION (6e0b44f), NEVER empirically validated — all
        # closed-loop evidence is stationary-target, where h_tv≡0 makes "ring measures ground"
        # indistinguishable from "ring redundantly samples near/on the target" (the fused==corner
        # ratio-1.00 finding that justified the old default-ON is consistent with BOTH). In
        # practice the ring's fixed radii overlap the grown marker near touchdown (see
        # _compute_ring_flow / memory feedback_ring_fusion_marker_overlap), so the separation
        # premise fails exactly when it matters. Treat the model below as unverified design
        # intent, not fact:
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
        # ⛔ DEFAULT FLIPPED TO OFF (2026-07-09, BAKED for the n>=5 IC1-5 controller-tuning
        # validation pass). Was DEFAULT ON since 2026-06-07 on the strength of R^2-vs-GT bench
        # comparisons (cal/multisine/landing) -- but this session found the ring loom estimator
        # produces spurious large values (|h_z| up to ~1.9, vs a GT-implied true value near zero)
        # SPECIFICALLY in the terminal/close-range regime, independent of marker-decode status
        # (confirmed: ring_div/ring_flow_z were already wrong ONE FRAME BEFORE a marker loss, while
        # the corner-only signal was still small/correct) -- and since this is the DEFAULT fusion
        # input the controller consumes, the bad ring value propagates into h(t) and, via the
        # loom-flow cross term, into a_u_xy, producing a terminal lateral kick. Live A/B (n=1,
        # same IC1 scenario): FLOW_FUSE_RING=1 -> h_z spiked to -0.86..-1.9 near touchdown, endpoint
        # ballooned well past min-alt precision; FLOW_FUSE_RING=0 -> h_z stayed in a tight, GT-
        # consistent band (-0.29..-0.285) through the same window, endpoint matched min-alt
        # precision almost exactly (no balloon). Matches the ALREADY-DOCUMENTED terminal ring-loom
        # dead-end (feedback_ring_loom_hz_terminal_deadend memory: "ring loom for h_z at terminal...
        # RECONFIRMED dead-end... keep ring-commit/loom-ring OFF") -- this bake finally acts on that
        # finding for the FUSION path specifically (the memory addressed ring-COMMIT/loom-RING
        # switching, not this default-on EKF fusion input). Set FLOW_FUSE_RING=1 to revert to fusion.
        self._fuse_ring = os.environ.get("FLOW_FUSE_RING", "0") == "1"
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
        # POSITIVE-LOOM SIGN GUARD -- REMOVED 2026-07-11 (was: clamp consumed h_z<=0, added
        # 2026-06-24 to band-aid a wrong-signed RING-loom fallback near the deck; see
        # feedback_loom_sign_guard_blocks_touchdown_detect). Removed rather than left default-off
        # because it clamped h_z UNCONDITIONALLY, including the genuine positive inversion at real
        # ground-contact that PLASMC_TOUCHDOWN_LOOM needs to ever fire -- structurally incompatible
        # with that detector. The ORIGINAL ring-loom wrong-sign problem this guarded against is
        # UNFIXED and now re-exposed -- see that memory's "ROOT CAUSE is still unfinished" section
        # for the two unconfirmed hypotheses (ring LK texture-starvation vs. marker-handover
        # attitude disturbance) if a real fix is needed.
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
        # ALPHA SIN/COS-PAIR SUB-FILTER (2026-07-19, replaces an earlier wrap-patch — see
        # feedback_alpha_kf_wrap_bug). Channel 3 of self._kf_feat_x (alpha) is a WRAPPED
        # ANGLE, not a plain linear quantity — a linear KF's innovation (z - x_pred) is
        # wrong across the +-pi branch cut, and just re-wrapping the innovation/state
        # (the earlier patch) still leaves the RATE state free to be corrupted by a
        # near-cut measurement. Proper fix: filter [sin(alpha), cos(alpha)] as two
        # ordinary LINEAR channels through the SAME generic _kf_step (no periodicity, no
        # branch cut, no asymptote — unlike tan(alpha), which is pi-periodic and would
        # silently reintroduce the 2pi/2-fold ambiguity _marker_principal_angle's corner
        # weighting was built to remove, see project docstring). self._kf_feat_x[3, :] is
        # kept in sync (channel 3 = atan2(sin_state, cos_state)) purely so every existing
        # consumer (getImgFeatureParam extrapolation, the sensor-cal readout, etc.) keeps
        # reading a plain alpha scalar from the same slot — the sin/cos state itself is
        # private to _kf_feat_update, below.
        self._kf_feat_sc_x = np.zeros((2, 2))            # rows: [sin(alpha), cos(alpha)]
        self._kf_feat_sc_P = np.tile(np.eye(2) * 1.0, (2, 1, 1))
        self._kf_feat_sc_prev_t = None
        self._kf_feat_sc_initialized = False
        # BUGFIX (2026-07-15, found via IC1-5 SITL validation): _kf_feat_initialized gets
        # cleared on every primary-marker-ID switch (small<->big decode flicker, below) so
        # the NEXT correct-step re-seeds state=z instead of blending a spurious velocity
        # across the discontinuous size/frame jump (matches _kf_x's identical pattern,
        # 2026-07-11). But getImgFeatureParam() previously GATED its entire return on this
        # SAME flag -- so any call between the reset and that next real correct-step fell
        # through to a DIFFERENT signal path (savgol/mean-of-recent-history), producing a
        # one-to-two-frame value fully disconnected from the KF's perfectly good (merely
        # un-"initialized"-flagged) last state.
        # CORRECTION (2026-07-15, later same session): the trace originally cited here as
        # live confirmation (IC3_rep4, s(t) hitting exactly [0,0,0,0] at t=34.144,
        # ICValidation/20260715-202110) was RE-TRACED and found to occur 0.028s after
        # ctrl_time[0] -- the very first logged control-loop frame after CONTROLLER_READY
        # engagement (logging starts there, not at motor-arm), not a genuine mid-flight
        # marker-switch event. s(t)=[0,0,0,0] before the first real sample lands is an
        # EXPECTED engagement transient, not this bug firing -- that trace does NOT
        # constitute confirmed live evidence. The code-level vulnerability itself (the
        # gate falling through to a different signal path on ANY _kf_feat_initialized=False
        # reset, including a genuine marker-switch one) is still real and demonstrable by
        # reading the code, independent of this mistraced example, so the fix is kept on
        # code-reasoning grounds -- but treat it as UNVALIDATED by a confirmed live
        # reproduction until re-traced against an actual mid-flight marker-ID switch.
        # getOptFlowAngVel() (the analogous corner-flow getter) does NOT have this bug --
        # it reads self._kf_x[:,0] unconditionally, so a marker-switch reset there just
        # naturally holds the last value (the array itself is never cleared, only the
        # flag) until the next correct-step updates it. This flag replicates that: TRUE
        # once ever (first real sample of the flight), never cleared by a marker-switch
        # reset (only by a genuine full _reset_stateful_trackers), so
        # getImgFeatureParam() can gate on "has this flight ever had a real sample" instead
        # of "was there a real sample since the LAST marker switch" -- same hold-last-good
        # principle used pervasively elsewhere in this file (ds/dh outlier-hold, s-extrap
        # real-buffer), not a new one.
        self._kf_feat_ever_initialized = False
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

    def _reset_stateful_trackers(self):
        """Reset every CONTROL-DECISION-relevant stateful tracker. Called once, exactly on the
        CONTROLLER_READY False->True transition (see run()) -- NOT every frame, and NOT a full
        re-init. This node runs continuously from Controller() construction (well before
        CONTROLLER_READY, through the whole hover/IC-convergence settle -- ~15-20s observed),
        so by engage time these trackers may already hold state accumulated during a completely
        different flight regime (takeoff climb, hover settle) that has nothing to do with the
        controlled descent about to begin. 2026-07-11: found via a real investigation where an
        offline-reconstructed "positive loom" event turned out to be ~15s BEFORE the descent even
        started -- harmless in isolation, but exactly the kind of pre-engage state that COULD
        carry forward into a KF/lock/arm decision post-engage undetected. Mirrors (and extends)
        the existing marker-switch reset block (~line 1391-1402) -- same lesson, applied at the
        other place a discontinuous regime change happens without an explicit code trigger.
        Board self-cal / layout learning is deliberately NOT reset here -- it's geometry/scale
        learning, not a control-decision state, and losing it costs a few frames of re-warmup for
        no correctness benefit."""
        self._locked_marker_id = None
        self._primary_id = None
        self._mtrace_hist.clear()
        self._centroid_hist.clear()
        self._obs_kf_x = None; self._obs_kf_y = None
        self._obs_kf_Px = None; self._obs_kf_Py = None; self._obs_kf_t = None
        self._flow_prev = None; self._flow_hold = None
        self._s_prev = None; self._s_hold = None
        self._consec_misses = 0
        self._consec_hits = 0
        self.FEATURE_IS_STALE = False
        self._planar_map_rescue_active = False
        self._last_real_extent_px = None
        self._hit_hist.clear()
        self._h_real_t = []; self._h_real_v = []
        self._kf_initialized = False
        self._kf_ring_initialized = False
        self._kf_feat_initialized = False
        self._kf_feat_sc_initialized = False
        self._kf_feat_ever_initialized = False   # genuine full reset -- unlike the marker-switch reset below
        self._ekf_init = False
        self._loom_stale = 0
        # Savgol pre-engage guard (2026-07-15, user directive: audit ALL pre-engage state
        # leaks, not just planar_map). _opt_flow_ang_vel_raw / _img_feature_param are
        # append-only FULL-RUN logs (also used by getLogData() for post-hoc analysis of
        # the whole flight incl. pre-engage, so they must NOT be cleared/truncated here).
        # But the legacy IMG_FILTER=savgol / IMG_FEATURE_FILTER=savgol path slices the last
        # FILTER_WIN samples directly off these logs -- for the first ~FILTER_WIN/2 frames
        # after engage, that window would silently span the CONTROLLER_READY boundary and
        # blend pre-engage samples into the "post-engage" filtered output. (The DEFAULT
        # 'kf' path does NOT have this bug: _kf_initialized/_kf_feat_initialized are reset
        # below, and _kf_step's own re-seed-on-next-sample logic means the KF value itself
        # starts clean post-engage -- verified, no fix needed there.) These indices mark
        # "first sample index that's valid to feed the savgol window" -- _compute_savgol_
        # output / getImgFeatureParam's savgol branch must never read before them.
        self._engage_idx_flow = len(self._opt_flow_ang_vel_raw)
        self._engage_idx_feat = len(self._img_feature_param)
        # PlanarFeatureMap: (re)constructed fresh here, NOT at node construction -- see
        # __init__ comment. Deployment has no way to know when the target first becomes
        # visible except "sometime during the descent", and pre-engage hover/climb motion
        # was found to occasionally corrupt the homography fit (65-110px, see __init__
        # comment) with nothing useful gained from tracking that phase at all.
        self._planar_map = (PlanarFeatureMap(center=self.center, focal=self.focal)
                            if (self._planar_map_shadow or self._planar_map_primary) else None)   # center/focal enable gyro-seeded KLT, see planar_map.py __init__ comment
        self._planar_map_primary_pred_px = None
        self._planar_map_gate_on = False
        self._planar_map_gate_streak = 0
        self._planar_map_override_gate_on = False
        self._planar_map_override_gate_streak = 0
        self._planar_map_decode_calls = 0
        print("[img_data] CONTROLLER_READY -> True: reset all control-decision stateful trackers "
              "(KF/lock/EKF/stale/hold-guard state) so pre-engage hover/climb history can't carry "
              "into the descent.")

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

                    if self.CONTROLLER_READY and not self._controller_was_ready:
                        self._reset_stateful_trackers()
                        self._controller_was_ready = True

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
                # 2026-07-11: the corner-flow and centroid-feature KFs (_kf_x, _kf_feat_x) had NO
                # reset here -- a marker handover (small<->big) is a discontinuous geometry step
                # (size/frame jump) that a constant-velocity KF has no way to distinguish from
                # genuine fast motion, so it would blend a spurious "velocity" across the switch
                # instead of accepting the new geometry as a fresh sample. _kf_step's own
                # `if not initialized:` branch re-seeds state=z, rate=0 on the NEXT real sample, so
                # just clearing the flag is enough (same lazy-reinit pattern as _obs_kf_x above).
                # See feedback_kf_frozen_during_marker_loss ("Related, same-class bug — STILL OPEN").
                self._kf_initialized = False
                self._kf_feat_initialized = False
                self._kf_feat_sc_initialized = False
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
              and self._prev_img is not None):
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
                                print(f"ArUco lost — KLT fallback active (confidence decays past {self._max_lk_steps} frames, no attempt ceiling -- see TARGET_LOST/MARKER_LOSS_GRACE for the real backstop)")
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

        # --- PlanarFeatureMap: shadow logging + PRIMARY prediction capture -------------
        # Tracks every frame, loop-closure-corrects on a fresh ArUco decode, and (unlike
        # the original pure-shadow design) now ALSO caches the primary-slot prediction in
        # self._planar_map_primary_pred_px for the single-frame V_aruco_norm[1] override
        # below (centroid + loom both consume it -- see the PLASMC_PLANAR_MAP_PRIMARY
        # __init__ comment) -- still never touches aruco_pts_0/extra_pts_0/marker_ids or
        # the frame-pair flow-lstsq path. self._planar_map is None until the
        # CONTROLLER_READY transition (see _reset_stateful_trackers) -- pre-engage
        # hover/climb is deliberately never fed in.
        self._planar_map_primary_pred_px = None
        if (self._planar_map_shadow or self._planar_map_primary) and self._planar_map is not None:
            try:
                _gray0 = imgs[0] if imgs[0].ndim == 2 else cv2.cvtColor(imgs[0], cv2.COLOR_BGR2GRAY)
                # GYRO-SEEDED KLT (2026-07-17, user design): the FC quaternion is available
                # every frame REGARDLESS of whether ArUco decodes anything this frame -- so
                # this runs unconditionally, including through marker-loss stretches (where
                # it matters most, see planar_map.py's update() docstring). quats[0] pairs
                # with imgs[0]/_gray0 (frame-0 convention, matches _getVirtualPts elsewhere
                # in this method); same Quaternion(...).to_DCM() call, no new conversion.
                _q0 = quats[0] if (quats is not None and len(quats) > 0 and quats[0] is not None) else None
                _R0 = (Quaternion([_q0.w, _q0.x, _q0.y, _q0.z]).to_DCM() if _q0 is not None else None)
                if not self._planar_map.initialized:
                    self._planar_map.bootstrap(
                        _gray0,
                        marker_px_corners=(aruco_pts_0 if results[0] else None),
                        marker_id=(self._primary_id if results[0] else None),
                        quat_R=_R0)
                    self._planar_map_decode_calls += 1
                else:
                    self._planar_map.update(_gray0, quat_R=_R0)
                    # Cache the primary (largest) slot's current-frame prediction for the
                    # shared V_aruco_norm[1] override downstream (centroid + loom + the
                    # PARTIAL/TOTAL-occlusion RESCUE below). Gated on map_confidence WITH
                    # HYSTERESIS (see __init__ comment).
                    #
                    # 2026-07-16 (user correction): gate on map_confidence (track_conf *
                    # resid_conf, marker-INDEPENDENT -- see its definition in planar_map.py),
                    # NOT the combined self.confidence / marker_rigid_ok. The combined metric
                    # requires the marker's OWN corners to currently survive KLT to stay
                    # nonzero -- exactly the condition the rescue exists to survive THROUGH.
                    # Gating on it meant the rescue could bridge at most a frame or two into
                    # any marker occlusion (partial or total) before reverting to blind
                    # extrapolation, defeating the module's actual purpose: infer the
                    # marker's position from the GLOBAL homography fit (built from whatever
                    # OTHER scene features are currently tracked) even when the marker itself
                    # is partially visible or has vanished entirely.
                    # MARKER-AWARE RESCUE GATE (2026-07-19, user). Was map_confidence
                    # (marker-INDEPENDENT = track_conf*resid_conf only), chosen so a fully-occluded
                    # marker wouldn't block the rescue. But that makes the gate STRUCTURALLY BLIND to
                    # the marker DEFORMING: at the tilt-grazing terminal the quad foreshortens to a
                    # sliver, map_confidence stayed ~0.575 (verified) and the rescue happily fired on
                    # geometry that can no longer localize a centre -> confident-wrong extrapolation.
                    # self.confidence DOES see it (marker_conf collapses via shape_change) -- verified:
                    # 39/39 zero-confidence frames were rigid_ok=False at exactly those frames.
                    #
                    # CORRECTION (2026-07-25, see feedback_rescue_gate_zero_corner): the claim below
                    # ("marker_conf survives a genuine occlusion") is only true for a SHORT occlusion.
                    # _rigid_fail_streak's persistence decay (planar_map.py, added 2026-07-17 to catch
                    # a SUSTAINED rigid_ok=False run the held shape_change alone couldn't see) also
                    # fires on a sustained ZERO-corner run, collapsing self.confidence to a hard 0 after
                    # rigid_fail_streak_max (default 3) consecutive zero-corner frames -- and it STAYS
                    # there for the rest of the outage, since marker_rigid_ok can't recover with no
                    # corners at all. Traced live (IC1_rep3/IC3_rep2/IC4_rep2, ICValidation/20260724-
                    # 172603): a >1s total zero-corner window (attitude flat, no tumble -- a genuine
                    # decode/tracking dropout, not a deforming marker) pinned self.confidence at exactly
                    # 0.0 the entire time while map_confidence stayed healthy (0.54-0.58, well above the
                    # 0.5 floor) -- the rescue gate's streak requirement never got a chance to
                    # accumulate, RESCUE_ACTIVE never fired, and the outage outlasted MARKER_LOSS_GRACE
                    # -> TARGET_LOST despite the map's underlying track being fine. Use
                    # planar_map.primary_zero_corners (set precisely on this frame's own zero-corner
                    # case, not the sustained streak) to fall back to map_confidence ONLY when there is
                    # LITERALLY NO shape information to distrust this frame -- a deforming-but-present
                    # marker (1-4 corners) still gates on the marker-aware self.confidence, preserving
                    # the 2026-07-19 fix for its actual target. RESCUE_GATE_MARKER_AWARE=0 still fully
                    # reverts (ignores this carve-out too).
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
                    # OVERRIDE gate (2026-07-16, see __init__ comment): separate, STRICTER
                    # gate for replacing an already-successful raw decode -- marker-aware
                    # self.confidence (still includes marker_conf), not map_confidence.
                    _good_override = self._planar_map.confidence >= self._planar_map_conf_floor
                    if _good_override:
                        self._planar_map_override_gate_streak += 1
                    else:
                        self._planar_map_override_gate_streak = 0
                        self._planar_map_override_gate_on = False
                    if (not self._planar_map_override_gate_on
                            and self._planar_map_override_gate_streak >= self._planar_map_gate_on_frames):
                        self._planar_map_override_gate_on = True
                    if self._planar_map_primary and self._planar_map_gate_on:
                        self._planar_map_primary_pred_px = self._planar_map.get_marker_frame_pts()
                    # HELD-OUT PREDICTION FIRST, before loop_closure_correct touches the
                    # map -- comparing AFTER correction would be circular on a fresh-decode
                    # frame (map_pts gets set FROM aruco_pts_0 this same frame via
                    # frame_to_map, then get_marker_frame_pts() reads it back through the
                    # SAME frame_to_map -> an exact round-trip, err~0 by construction, not a
                    # real accuracy check). This mirrors tools/validate_planar_map.py's
                    # held-out methodology exactly.
                    #
                    # SLOT MATTERS (2026-07-15 bug fix): get_marker_frame_pts(slot=None)
                    # returns the map's own "primary" (largest) slot, which is independent
                    # of -- and can disagree with -- whichever physical marker img_data.py's
                    # OWN separate _locked_marker_id/_primary_id logic actually decoded this
                    # frame (aruco_pts_0). Comparing against the wrong slot produced spurious
                    # ~100px "errors" during n=6 IC1 validation that were a bug in THIS
                    # comparison, not in PlanarFeatureMap. identify_slot() finds which slot
                    # aruco_pts_0 itself geometrically belongs to (None if it doesn't match
                    # any yet -- correctly excluded, not compared against the wrong one).
                    _pred = None
                    if aruco_pts_0 is not None:
                        _slot_guess = self._planar_map.identify_slot(aruco_pts_0)
                        if _slot_guess is not None:
                            _pred = self._planar_map.get_marker_frame_pts(slot=_slot_guess)
                    # REJECT MAP CROSS-CORRECTION ON ILL-CONDITIONED QUAD (2026-07-19, user): don't
                    # loop-close from a decode whose quad is a degenerate SLIVER (tilt-grazing /
                    # overflow) -- that near-singular observation poisons the map's gauge/homography.
                    # The map is PRIMARY: predict through it on clean state; decode cross-checks ONLY
                    # on WELL-CONDITIONED frames. Detected on the raw decode quad's own geometry
                    # (diagonals near-parallel / quad elongated), NOT edge-proximity -- a quad can be
                    # degenerate from grazing foreshortening before it reaches the edge.
                    # (a) DECODE correction gate: reject NEAR-EDGE decodes (corner precision degrades
                    # at the frame boundary -> noisy correction poisons the map; verified: edge-reject
                    # dropped map err_px p90 3.8->1.4/6.6->2.0). OR-in the ill-cond check as a harmless
                    # backstop -- verified INERT on the decode path (ArUco self-gates conditioning:
                    # decode-success quads have min diag angle 27deg, never a sliver), but future-proof.
                    # CONFIDENCE-WEIGHTED LOOP-CLOSURE CORRECTION (2026-07-30, ported from
                    # Hardware/scripts/img_data.py): previously a binary gate --
                    # _reject_correct=True skipped loop_closure_correct entirely, throwing
                    # away a near-edge/marginal decode's information completely even though
                    # it still carries SOME real signal. Replaced with a continuous
                    # confidence in [0,1] (_markerEdgeMarginScore * _quadConditionScore,
                    # same underlying thresholds the old boolean gate used) passed into
                    # loop_closure_correct, which now BLENDS the map's per-corner position
                    # toward the decode by this amount instead of a hard snap-or-skip.
                    _corr_confidence = 1.0
                    if self._map_reject_overflow_correct and aruco_pts_0 is not None:
                        _ih, _iw = imgs[0].shape[:2]; _m = self._marker_fov_margin
                        _corr_confidence = (self._markerEdgeMarginScore(aruco_pts_0, (_ih, _iw), _m)
                                             * self._quadConditionScore(aruco_pts_0))
                        if os.environ.get("PLANAR_MAP_DBG", "0") == "1" and _corr_confidence < 0.999:
                            print(f"[planar_map] loop_closure_correct confidence="
                                  f"{_corr_confidence:.3f} (near-edge/marginal decode) "
                                  f"t={float(getattr(self,'_stamp',0.0)):.2f}", flush=True)

                    # VALIDATION-TRIGGERED RESET (2026-07-30, see HANDOFF_cbf_lockout_
                    # planarmap_2026-07-30.md): _err below is a HELD-OUT ground-truth
                    # cross-check (map's own prediction, from BEFORE this frame's
                    # correction, vs the actual fresh decode) -- previously computed
                    # AFTER loop_closure_correct already ran and only ever logged
                    # ("[planar_map shadow]"), never acted on. Confirmed live (2026-07-30
                    # Gazebo crash, t~64.0s): map_confidence stayed 0.27-0.9 (healthy-
                    # looking) for the ENTIRE window the held-out err spiked to
                    # 645-1483px, because map_confidence is track_conf*resid_conf*...
                    # computed from the SAME (already-broken) homography the error is
                    # measuring against -- it can't see its own failure. This ground-
                    # truth check is the one signal that CAN see it: it's compared
                    # against an actual fresh decode, not the map's own internal state.
                    # MUST be computed BEFORE loop_closure_correct touches the map (see
                    # "HELD-OUT PREDICTION FIRST" comment above) -- computing it after
                    # would be circular. On a large disagreement, a per-corner snap
                    # (loop_closure_correct) isn't enough -- the underlying homography
                    # itself is wrong -- so discard all map state and re-bootstrap fresh
                    # from this decode instead, exactly like the existing
                    # resid_px-non-finite SELF-HEAL path in planar_map.py:update(), just
                    # triggered by a different (ground-truth, not internal) signal.
                    _err = None
                    if aruco_pts_0 is not None and _pred is not None and len(_pred) == len(aruco_pts_0):
                        _err = float(np.mean(np.linalg.norm(
                            _pred - np.asarray(aruco_pts_0, dtype=np.float64), axis=1)))
                    _validation_reset_px = float(os.environ.get("PLANAR_MAP_VALIDATION_RESET_PX", "30.0"))
                    _validation_failed = (_err is not None and _err > _validation_reset_px)

                    if _validation_failed:
                        if os.environ.get("PLANAR_MAP_DBG", "0") == "1":
                            print(f"[planar_map] VALIDATION-TRIGGERED RESET: held-out err="
                                  f"{_err:.1f}px exceeds {_validation_reset_px:.1f}px -- "
                                  f"discarding poisoned map state and re-bootstrapping "
                                  f"fresh from this decode t={float(getattr(self,'_stamp',0.0)):.2f}",
                                  flush=True)
                        self._planar_map._full_reset()
                        self._planar_map.bootstrap(_gray0, marker_px_corners=aruco_pts_0,
                                                    marker_id=self._primary_id, quat_R=_R0)
                        self._planar_map_decode_calls += 1
                    elif results[0]:
                        self._planar_map.loop_closure_correct(
                            aruco_pts_0, marker_id=self._primary_id, confidence=_corr_confidence)
                        self._planar_map_decode_calls += 1

                    if _err is not None:
                        self._planar_map_log.append({
                            "t": float(getattr(self, '_stamp', 0.0)), "err_px": _err,
                            "fresh_decode": bool(results[0]), "used_klt_fallback": used_klt_fallback,
                            "confidence": self._planar_map.confidence,
                            "map_confidence": self._planar_map.map_confidence,
                            "marker_rigid_ok": self._planar_map.marker_rigid_ok,
                            "primary_zero_corners": self._planar_map.primary_zero_corners,
                            "decode_calls": self._planar_map_decode_calls,
                            "validation_reset": _validation_failed})
                        if os.environ.get("PLANAR_MAP_DBG", "0") == "1":
                            print(f"[planar_map shadow] t={self._planar_map_log[-1]['t']:.3f} "
                                  f"err={_err:.2f}px conf={self._planar_map.confidence:.2f} "
                                  f"n_tracked={self._planar_map.n_tracked} "
                                  f"rigid_ok={self._planar_map.marker_rigid_ok}")
            except Exception as _e:
                # Never let a map-internal failure take down the real acquisition path or
                # poison the loom source -- pred already defaulted to None above this try.
                if os.environ.get("PLANAR_MAP_DBG", "0") == "1":
                    print(f"[planar_map shadow] exception (non-fatal): {_e}")

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

        # PLAUSIBILITY GATE (2026-07-24, see project_ic2_observer_plausibility memory): this
        # observer reads `aruco_pts_0` -- a raw per-frame ArUco decode with NO LK correspondence
        # requirement at all (that's the whole point, per the docstring below -- it must survive
        # nfc=0) -- and therefore had NONE of the sanity checks the LK-correspondence path got
        # (neither the original _planarMapPredictionPlausible map-path check nor the 2026-07-23
        # raw-decode extension added for aruco_pts_1, see the "PLAUSIBILITY GATE ON RAW DECODE"
        # comment ~line 2384). Traced live (IC2_rep3, ICValidation/20260723-191943, t~45-46.3s):
        # a single-frame decode anomaly (MARKER_EXTENT_PX 70->315px in one frame -- a marker-size-
        # switch/handover) triggered a real, escalating attitude tumble (confirmed via Quat); during
        # the ensuing ~1.3s TOTAL raw-decode blackout (nfc=0 throughout, the LK-gated path correctly
        # froze `_feature_pts`/MARKER_EXTENT_PX) this OBSERVER kept running every frame (only needs
        # `aruco_pts_0 is not None`, not LK success) on spurious/false decodes as the tumbling camera
        # swept across background clutter, feeding wildly different `_x0,_y0` positions into
        # `_obs_vel_kf` each frame -- theta exploded 2->3080 and `h_x` alternated roughly -161/+130
        # frame-to-frame, feeding the SAME flow KF getOptFlowAngVel() returns. Reuse the same
        # position+size sanity check (against the pre-blackout known-good state) rather than trusting
        # any `aruco_pts_0` decode unconditionally.
        _obs_gate_ok = False
        if (self._single_marker and self._centroid_rate and aruco_pts_0 is not None
                and quats is not None and len(quats) > 0 and quats[0] is not None):
            _obs_gate_ok, _, _obs_why = self._planarMapPredictionPlausible(aruco_pts_0, quats[0])
            if not _obs_gate_ok and os.environ.get("DECODE_PLAUS_DBG", "0") == "1":
                print(f"[decode plausibility] REJECTED observer aruco_pts_0 ({_obs_why}) "
                      f"-- skipping centroid-rate observer this frame", flush=True)
        # GYRO-COMPENSATED CENTROID-RATE OBSERVER (PLASMC_CENTROID_RATE, default-ON since 2026-07-03
        # -- see the fuller history at its __init__ comment ~line 985). Computed from the DECODED corners
        # (aruco_pts_0) — NOT the LK-tracked V_aruco_norm — so it runs even when LK fails (Nfc=0) at
        # altitude. Provides the lateral flow h_x,h_y from ṡ + loom + gyro-rotation compensation:
        #   h_x = ṡ_x + x0·h_z + y0·wz   (V-frame: roll/pitch leveled out, yaw preserved -> wz only)
        #   h_y = ṡ_y + y0·h_z − x0·wz
        # h_z from the moment loom; w from the IMU gyro rotated into the V-frame (clean, not the
        # off-center-ill-conditioned lstsq). Stored for injection at the flow-output sites below.
        if _obs_gate_ok:
            try:
                _Vdec = self._getVirtualPts(np.asarray(aruco_pts_0, np.float32), quats[0])   # FRAME-PAIR FIX 2026-07-04: aruco_pts_0 belongs to frame-0 -> level with quats[0], not quats[1] (matches V_aruco_norm/V_flow_norm convention; the quats[1] mismatch left a residual tilt ∝ angular rate = a source of the off-center yaw leak)
                _x0 = float(_Vdec[:, 0].mean()); _y0 = float(_Vdec[:, 1].mean())
                # Loom M: map-driven (stage 1) when enabled + available, else decode. Centroid
                # (_x0/_y0) stays on DECODE -- this stage migrates the loom scale ONLY.
                _Mo = None
                if self._loom_m_from_map:
                    _Mo = self._loomMapM(quats[0])
                if _Mo is None:
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
                            # all_pts_1 is (N,2) here (all_pts_0 was vstacked as (N,2), not
                            # reshaped to (N,1,2) before the LK call) -- a stale (-1,1,2)
                            # reshape here throws "could not broadcast (4,1,2) into (4,2)".
                            # Found 2026-07-28: this line is only reachable when marker_ids is
                            # not None (board/multi-marker mode), which was dead code for over
                            # a month while PLASMC_SINGLE_MARKER defaulted on by accident -- see
                            # project_single_marker_default_mismatch_20260728 memory.
                            all_pts_1[sl] = (_d0 + (_d1_dec - _d0) * self._decode_scale).reshape(-1, 2)
                            status[sl] = 1
                            self._decode_fills += 1

            aruco_status = status[:n_aruco]
            extra_status = status[n_aruco:]

            # PRIMARY gate stays strict: need all 4 primary corners tracked
            # for centroid/alpha. Extra (other-marker) corners are kept
            # individually — any that pass LK add spread to the lstsq.
            board_markers_px1 = None     # [(id, frame1 corners 4x2)] for homography
            if int(np.sum(aruco_status == 1)) == n_aruco:
                aruco_pts_1 = all_pts_1[:n_aruco].reshape(-1, 2)
                # PLAUSIBILITY GATE ON RAW DECODE (2026-07-23, see
                # project_ic2_wrong_marker_decode memory): a raw ArUco decode that tracks
                # cleanly (all 4 corners, LK status==1) was previously trusted
                # UNCONDITIONALLY -- no sanity check at all, unlike the map override/rescue
                # paths (_planarMapPredictionPlausible). Traced live (IC2_rep2,
                # ICValidation/20260723-191943): a single-frame marker-loss gap (nfc=0) was
                # immediately followed by a DIFFERENT, spurious 4-corner detection (~9px
                # marker in a corner of the frame vs the true ~150px marker) that decoded
                # and LK-tracked CLEANLY for several subsequent frames (self-consistent
                # frame-to-frame, so the ds outlier-hold's single-frame delta check couldn't
                # catch it past the first frame) -- fed straight into s/sigma/kappa and
                # detonated a_u. Reuses the SAME position+size sanity check the map path
                # already trusts, against the OLD _last_real_extent_px/_feature_pts (checked
                # BEFORE either is overwritten below). An implausible raw decode is treated
                # like a decode miss this frame (falls through to the existing rescue/hold
                # path via FEATURE_DATA_IS_LOGGED staying False), not clipped/clamped.
                _raw_ok, _, _raw_why = self._planarMapPredictionPlausible(aruco_pts_1, quats[1])
                if not _raw_ok:
                    if os.environ.get("DECODE_PLAUS_DBG", "0") == "1":
                        print(f"[decode plausibility] REJECTED raw decode ({_raw_why}) "
                              f"-- treating as a miss this frame", flush=True)
                else:
                    FEATURE_DATA_IS_LOGGED = True
                    extra_good = (extra_status == 1)
                    extra_pts_0_kept = extra_pts_0[extra_good]
                    extra_pts_1_kept = all_pts_1[n_aruco:].reshape(-1, 2)[extra_good]
                    flow_pts_0 = np.vstack([aruco_pts_0, extra_pts_0_kept])
                    flow_pts_1 = np.vstack([aruco_pts_1, extra_pts_1_kept])
                    # Primary-only pair preserved for centroid / alpha / display.
                    C_nP = [aruco_pts_0, aruco_pts_1]
                    # Last-known REAL marker span (px, camera-pixel space) -- for the rescue's
                    # size-plausibility check below. Updated ONLY on genuine raw decode (never
                    # from a rescue/extrapolated frame), same "real-only" discipline as
                    # _h_real_v elsewhere in this file.
                    self._last_real_extent_px = float(
                        max(aruco_pts_0[:, 0].max() - aruco_pts_0[:, 0].min(),
                            aruco_pts_0[:, 1].max() - aruco_pts_0[:, 1].min()))

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
                # NOTE (2026-07-16, clarified per user correction): V_aruco_norm[0] is built
                # from aruco_pts_0 -- the actual cv2 ArucoDetector.detectMarkers() output on
                # imgs[0] (line ~1584), i.e. the genuine fresh decode. V_aruco_norm[1] is built
                # from aruco_pts_1 -- calcOpticalFlowPyrLK-TRACKED corners propagated from
                # aruco_pts_0 into imgs[1], NOT a re-decode. Both are real image-derived data,
                # but only [0] is "raw decode"; [1] (the one PlanarFeatureMap's override/rescue
                # conditionally replace below) is one KLT hop removed from it.
                V_aruco_norm = [self._getVirtualPts(p, a) for p, a in zip(C_nP, quats)]
                # PLANAR-MAP PRIMARY (2026-07-15, PLASMC_PLANAR_MAP_PRIMARY, default-on):
                # override V_aruco_norm[1] -- the SINGLE shared source both the centroid
                # (s/alpha, below) and the moment-loom (_loom_decouple's `_vp`, below) read
                # from -- with PlanarFeatureMap's primary-slot prediction, whenever the map
                # is confident+rigid this frame. Overriding HERE (once) rather than at each
                # consumer keeps centroid and loom internally consistent (both always agree
                # on which corner source produced this frame's estimate) and is what
                # actually fixes the small<->big decode-flicker bug at its root: the raw
                # per-frame ArUco decode this array would otherwise carry jumps ~50-80px
                # instantaneously on a flicker (see feedback_loom_flicker_hold_stall /
                # __init__ comment); the map's KLT+homography-tracked slot does not, since
                # it only gets LOOP-CLOSURE corrected (not overwritten) by whichever marker
                # happens to decode this frame. Does NOT touch aruco_pts_0/extra_pts_0/
                # flow_pts_0/1 or V_flow_norm -- the frame-PAIR flow lstsq (h_xy/w, below)
                # stays on the raw multi-tier corner correspondence; PlanarFeatureMap's
                # public API predicts a single frame's position, not a frame-pair, so
                # extending the override there would need new map-side machinery, not yet
                # built. Falls back to the untouched raw V_aruco_norm[1] whenever the map
                # isn't ready/confident (identical behavior to before this bake).
                # STRICTER gate here (2026-07-16): this REPLACES an already-successful raw
                # decode, so requires _planar_map_override_gate_on (marker-aware
                # self.confidence), not just _planar_map_gate_on (map_confidence, which
                # only gates whether _planar_map_primary_pred_px was computed at all this
                # frame). See __init__ comment -- using the permissive rescue gate here let
                # the override fire on perfectly healthy raw-decode frames too, replacing
                # accurate fresh corners with a possibly-drifted map estimate (confirmed
                # live: IC1/IC4 smooth growing divergence with corners never lost).
                #
                # PLAUSIBILITY CHECK (2026-07-16, user correction): this override had NO
                # sanity check at all until now -- confirmed live (IC2/IC3 terminal
                # phase) it let a drifted map projection silently replace a genuinely
                # good raw ArUco decode, producing physically-impossible V-frame
                # centroids like [-23,24] fed straight into centroid/alpha/loom, even
                # though the raw decode itself (reconstructed independently) was fine.
                # Reuses the SAME _planarMapPredictionPlausible check the rescue path
                # uses -- position AND size, not just "the map claims confidence".
                if (self._planar_map_primary and self._planar_map_override_gate_on
                        and self._planar_map_primary_pred_px is not None):
                    _ov_ok, _pm_v1, _ov_why = self._planarMapPredictionPlausible(
                        self._planar_map_primary_pred_px, quats[1])
                    if _ov_ok and len(_pm_v1) == len(V_aruco_norm[1]):
                        V_aruco_norm[1] = _pm_v1
                    elif os.environ.get("PLANAR_MAP_DBG", "0") == "1":
                        print(f"[planar_map override] REJECTED implausible {_ov_why} "
                              f"-- keeping raw ArUco V_aruco_norm[1]", flush=True)
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
                # RUNTIME ATTITUDE-RATE GATE for FLOW_LAT_REDUCED (2026-07-26, see
                # project_ic1rep2_ic2rep5_flyaway_traces_20260725 / project_ic1_terminal_kick_root_cause_chain):
                # the reduced 4-DOF solve's w_xy≈0 premise is about the CAMERA's own V-frame
                # tilt-RATE being negligible (roll/pitch RATE, not static tilt -- de-rotating
                # POSITIONS into the level V-frame does not itself zero out a genuinely nonzero
                # rotational-flow contribution when the camera is actively rolling/pitching).
                # FLOW_TARGET_LEVEL only guards the OTHER half of the premise (is the TARGET's
                # geometry level, e.g. ship-deck tilt) -- it says nothing about the CAMERA's
                # instantaneous motion, so a real, fast roll/pitch rate (e.g. during a marker-
                # switch/handover-triggered erroneous command igniting a genuine tumble --
                # traced live twice this session, IC1_rep2 and IC2_rep3) still gets silently
                # misattributed into h_xy by the reduced solve, amplified by r^2 via far
                # corners, feeding a spurious velocity command straight into the controller --
                # exactly the "needs runtime gating on real attitude rate" fix flagged (not
                # implemented) back in the 2026-07-10/11 session. Use the FC gyro (angvels),
                # available every frame independent of decode/tracking health, rotated into
                # the SAME V-frame the flow-Jacobian columns are defined in (_vframe_w, the
                # identical transform the centroid-rate observer already uses) -- if the real
                # |w_x|,|w_y| exceeds FLOW_LAT_REDUCED_WMAX_RADS, the premise is violated THIS
                # FRAME regardless of FLOW_TARGET_LEVEL, so fall through to the full 6-DOF
                # solve (which can properly attribute the rotation to w_xy instead of h_xy).
                _lat_reduced_safe = self._target_level
                if self._lat_reduced and _lat_reduced_safe:
                    _avo1 = angvels[1] if (angvels is not None and len(angvels) > 1
                                           and angvels[1] is not None) else None
                    if _avo1 is not None and quats is not None and len(quats) > 1 and quats[1] is not None:
                        try:
                            _wv1 = self._vframe_w(
                                [_avo1.forward_rad_s, _avo1.right_rad_s, _avo1.down_rad_s], quats[1])
                            if max(abs(_wv1[0]), abs(_wv1[1])) > self._lat_reduced_wmax:
                                _lat_reduced_safe = False
                                if os.environ.get("FLOW_LAT_REDUCED_DBG", "0") == "1":
                                    print(f"[flow_lat_reduced] t{float(getattr(self, '_stamp', 0.0)):.3f} "
                                          f"GATE FIRED: |w_xy|=({abs(_wv1[0]):.2f},{abs(_wv1[1]):.2f}) "
                                          f"rad/s > {self._lat_reduced_wmax} -- falling through to full "
                                          f"6-DOF solve this frame", flush=True)
                        except Exception:
                            pass   # never let a gyro-read hiccup take down the flow solve
                _obs_active = (self._single_marker and self._centroid_rate and self._observer_valid)
                if _obs_active:
                    V_v = np.zeros(6)
                    V_v[5] = float(self._observer_flow[5])       # gyro yaw rate (replaces lstsq w_z)
                elif self._lat_reduced and _lat_reduced_safe:
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
                    # V_aruco_norm[1] is already PlanarFeatureMap-sourced when the map is
                    # confident this frame (single override point, above) -- automatically
                    # continuous across a small<->big decode flicker, no loom-specific
                    # branch needed here anymore (see PLASMC_PLANAR_MAP_PRIMARY comment).
                    _vp = V_aruco_norm[1]                          # de-rotated primary corners (4×2); kept for OVER_TARGET quad below
                    _ctr = _vp.mean(axis=0)
                    # Loom M: map-driven (stage 1) when enabled + available, else decode scatter.
                    # _vp itself stays decode-sourced (OVER_TARGET point-in-quad needs the live
                    # primary-marker quad, a separate concern from the loom scale).
                    _M = self._loomMapM(quats[0]) if self._loom_m_from_map else None
                    if _M is None:
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
                _hxy_observer = self._single_marker and self._centroid_rate and self._observer_valid
                if _hxy_observer:
                    V_v[0] = float(self._observer_flow[0])
                    V_v[1] = float(self._observer_flow[1])

                # MAP-DRIVEN SMALL-MARKER FLOW (stage 4): takes PRIORITY over both the lstsq and
                # the observer once the small marker is confidently mapped AND in its reliable
                # size band (see _smallSlotFlowReady docstring) -- independent of/typically
                # earlier than HANDOVER_LATCHED. Falls through untouched if not ready/unavailable.
                _fm_raw = (np.nan, np.nan, np.nan)   # logged regardless of fire (see __init__)
                if self._flow_from_map:
                    _q1 = quats[1] if (quats is not None and len(quats) > 1 and quats[1] is not None) else None
                    _fm_dbg = os.environ.get("FLOW_MAP_DBG", "0") == "1"
                    if self._smallSlotFlowReady(_q1):
                        _fm = self._flowMap(_q1, float(getattr(self, '_stamp', 0.0)))
                        if _fm is not None:
                            _fm_raw = (float(_fm[0]), float(_fm[1]), float(_fm[2]))
                            V_v[0], V_v[1], V_v[2] = float(_fm[0]), float(_fm[1]), float(_fm[2])
                            if _fm_dbg:
                                self._fmap_dbg = getattr(self, '_fmap_dbg', {})
                                self._fmap_dbg['fired'] = self._fmap_dbg.get('fired', 0) + 1
                        elif _fm_dbg:
                            self._fmap_dbg = getattr(self, '_fmap_dbg', {})
                            self._fmap_dbg['ready_but_none'] = self._fmap_dbg.get('ready_but_none', 0) + 1
                    elif _fm_dbg:
                        self._fmap_dbg = getattr(self, '_fmap_dbg', {})
                        self._fmap_dbg['not_ready'] = self._fmap_dbg.get('not_ready', 0) + 1
                self._fmap_raw_log.append(_fm_raw)

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
                # 'lstsq'/'lstsq_klt' share ONE calibration category (same _fill_A/lstsq geometry
                # regardless of corner source — see feedback_klt_fallback_merge_no_separate_cal);
                # the _klt suffix is coverage/diagnostic ONLY, not a separate fit bucket.
                _tag = 'lstsq_klt' if used_klt_fallback else 'lstsq'
                self._h_estimator_tag.append(_tag + '+observer_xy' if _hxy_observer else _tag)
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
                self._planar_map_rescue_active = False   # raw tier succeeded this frame, not a rescue
                if board_markers_V is not None:
                    board_s = self._board_feature(board_markers_V, size_factor)
                if board_s is not None:
                    self._img_feature_param.append(board_s)
                    self._s_estimator_tag.append('board_homography')
                else:
                    self._getImgFeatures(size_factor * V_aruco_norm[1])
                    self._s_estimator_tag.append('single_marker_moment')
                # ORDERING FIX (2026-07-20, see feedback_map_override_kf_ordering_bug): the KF
                # correct-step used to run HERE, against the pre-override decode value -- but
                # getImgFeatureParam() (default IMG_FEATURE_FILTER=kf) returns
                # self._sensor_cal_s @ self._kf_feat_x[:, 0], NOT self._img_feature_param[-1]
                # directly. Since nothing re-ran the KF update after the centroid/alpha map
                # override below, CENTROID_FROM_MAP/ALPHA_FROM_MAP's blend never reached the
                # controller under the default filter -- it only mutated a value the default
                # read path doesn't consume. Moved to AFTER both override blocks (below) so the
                # KF is corrected against the FINAL (post-blend) feature vector.

                # MAP-DRIVEN CENTROID (stage 2): override s[0:2] with the map's continuously-tracked
                # slot centre (handover-gated slot), keeping alpha (s[3]) on decode for stage isolation.
                # quats[0] matches the map's frame-0 tracking (same convention the loom uses). Falls
                # through to the decode centroid untouched if the map is unavailable. Placed BEFORE the
                # ds-outlier-hold so the map centroid still gets the per-frame-jump guard.
                # MAP IS AUTHORITATIVE (2026-07-20, user directive: "decode is just for
                # cross-correction of map", corrected from an earlier "decode is a fallback"
                # framing -- decode is NOT a peer/backup value source; its role is the
                # cross-check reference the map's OWN validity gates are judged against
                # (_planarMapPredictionPlausible's position/size sanity, the rigidity/
                # confidence floor inside _centroidMap). Whenever _centroidMap ACCEPTS this
                # frame (returns non-None -- a genuinely-garbage frame is still rejected by
                # those gates), use its value FULLY, not blended down toward decode by
                # degree of confidence -- confidence already decided ACCEPT/REJECT; it
                # doesn't get a second, weaker vote via a value taper. When the map has
                # nothing this frame (_mc is None), s[0:2] mechanically remains whatever
                # decode's own _getImgFeatures/_board_feature already computed above (there
                # is no third value to substitute) -- decode isn't "supplying" that value
                # as a designed fallback, it's what cross-checking requires having anyway.
                # _cmap_last_trust is still computed (for logging / _effectiveCalSxy's gain
                # selection below) but no longer tapers the VALUE.
                _mc_raw, _mc_trust = (np.nan, np.nan), 0.0   # logged regardless of fire (see __init__)
                if self._centroid_from_map and self._img_feature_param:
                    _q1 = quats[1] if (quats is not None and len(quats) > 1 and quats[1] is not None) else None
                    _mc = self._centroidMap(_q1, size_factor)
                    if _mc is not None:
                        _mc_raw = (float(_mc[0]), float(_mc[1]))
                        _mc_trust = self._cmap_last_trust
                        self._img_feature_param[-1][0] = float(_mc[0])
                        self._img_feature_param[-1][1] = float(_mc[1])
                self._cmap_raw_log.append(_mc_raw)
                self._cmap_trust_log.append(_mc_trust)

                # MAP-DRIVEN ALPHA (stage 3): override s[3] with the map's handover-gated slot
                # orientation (corner+dense moment, lower variance than 4 decode corners alone).
                # Same quats[1] convention as the centroid. Falls through to decode alpha untouched
                # if the map/plausibility gate rejects this frame.
                # MAP IS AUTHORITATIVE (same directive as centroid above): _alphaMap's value
                # is used FULLY whenever accepted, not tapered toward decode by confidence.
                _ma_raw, _ma_trust = np.nan, 0.0   # logged regardless of fire (see __init__)
                if self._alpha_from_map and self._img_feature_param:
                    _q1 = quats[1] if (quats is not None and len(quats) > 1 and quats[1] is not None) else None
                    _ma = self._alphaMap(_q1)
                    if _ma is not None:
                        _ma_raw = float(_ma)
                        _ma_trust = self._amap_last_trust
                        self._img_feature_param[-1][3] = float(_ma)
                self._amap_raw_log.append(_ma_raw)
                self._amap_trust_log.append(_ma_trust)

                # ds OUTLIER-HOLD on the raw centroid: reject a per-frame centroid JUMP (detection/LK
                # glitch) and hold last-good — keeps s clean for the position loop AND the observer.
                # MOVED BEFORE the KF correct-step (2026-07-20, see feedback_ds_guard_kf_ordering_bug):
                # this guard used to run AFTER _kf_feat_update, so it only ever corrected
                # self._img_feature_param[-1] -- a value the KF had ALREADY consumed that frame. The
                # KF's actual state (self._kf_feat_x, what getImgFeatureParam() reads by default) was
                # never protected by this guard, before OR after the 2026-07-20 override-ordering fix
                # (c654557) -- that fix moved the KF update to run after the centroid/alpha map
                # override so CENTROID_FROM_MAP/ALPHA_FROM_MAP would reach the controller, but left
                # this guard downstream of it, so a spike this guard SHOULD catch still fed the KF at
                # full strength. Now runs first, so the KF is corrected against the GUARDED value.
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

                # KF correct-step -- corrects against the FINAL feature vector, after any
                # centroid/alpha map blend AND the ds outlier-hold above, so
                # CENTROID_FROM_MAP/ALPHA_FROM_MAP reach the controller under the default
                # IMG_FEATURE_FILTER=kf path (the original 2026-07-20 fix, c654557) AND a
                # single-frame spike the ds guard catches never reaches the KF either.
                self._kf_feat_update(self._img_feature_param[-1], self._time.perf_counter())

                # (2026-07-09 fix, historical: a REAL-sample s buffer used to be captured here,
                # post-ds-outlier-hold, feeding a polyfit-based s-extrapolation during marker-loss
                # coasts. That extrapolation was REPLACED 2026-07-17 by the feature KF's own
                # predict-only step (self._kf_feat_x, stepped every real frame via
                # self._kf_feat_update(self._img_feature_param[-1], ...) a few lines above) -- the KF already
                # tracks a live, correctly-scaled position+velocity estimate, making a separate
                # bounded-history trend-fit buffer redundant. Removed; if a future need for raw
                # real-only s history resurfaces, re-add it capturing self._img_feature_param[-1]
                # HERE (post-guard), not the pre-guard raw value -- see feedback_s2_homogeneous_decay_bug
                # memory for why the guard-vs-pre-guard distinction matters.)

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
                self._feature_pts_fresh = True   # genuine raw decode/KLT this frame
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
                # BUGFIX (2026-07-23, see project_ic1_ds_guard_gap_reset memory): the comment above
                # says "ds/dh gate" but only _flow_prev was ever reset here -- _s_prev (the ds
                # outlier-hold's detection reference, ~line 2872) was NOT, so after a marker-loss
                # gap the centroid guard compared the just-reacquired raw position against a
                # STALE pre-gap reference. Real motion during the gap (plus continued motion while
                # reacquiring) routinely exceeds FLOW_DS_MAX=0.15 every frame -> the guard rejected
                # for MULTIPLE consecutive control cycles (s frozen while the real error kept
                # growing unseen by sigma/kappa) -> then the missed motion landed as one compounded
                # jump the instant a delta finally cleared. Traced live in IC1_rep1
                # (ICValidation/20260723-185307): s_e_n frozen at [0.033,-0.732] for 3 ctrl cycles
                # (t=47.864-47.94) despite healthy 184-corner decode throughout, then jumped to
                # [-0.878,-2.334] in one step -- immediately followed by kappa/a_u detonation.
                self._s_prev = None          # (fix) re-init the centroid guard on the same gap

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
            if self._loom_decouple:
                # CAL BYPASS for the moment loom -- MIRRORS the identical bypass on the
                # LK-succeeded branch (~:2450); added 2026-07-17 to fix an ASYMMETRY. The
                # observer assembles its row 2 as `_hz = _loom_dec` (~:2044), i.e. the SAME
                # scale-free moment loom (−½·d(lnM)/dt) that the other branch protects, NOT a
                # raw pinv loom. So _sensor_cal_hw row 2 (the raw-pinv→cal map) over-scales it
                # ~7% AND re-injects the lateral/rotational coupling the decoupling removed --
                # exactly the reasoning already documented at the other site. Without this, the
                # loom's scale+coupling silently CHANGED depending on whether LK survived this
                # frame, and this branch is the default-on path the controller consumes (it
                # feeds the fusion EKF), so the discontinuity reached control on every
                # LK-dropout-at-altitude. Same quantity -> same treatment.
                _corner_cal[2] = _obs_scaled[2]
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
                self._kf_ring_initialized, _vvr, self._time.perf_counter(),
                self._kf_q, self._kf_r)
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
            # For the image-feature centroid, holding the last marker position outright
            # is NOT a safe default either — a genuinely-moving marker (descent) needs
            # some form of forward propagation, not a freeze. 2026-05-20: tried
            # hold-last-value as a "safer" alternative — REGRESSED badly (mean xy
            # 0.49→1.52, max 0.77→4.83 across 5 reps); a frozen position under real
            # motion causes catastrophic lateral excursions on the next single-frame
            # dropout. Originally solved with a polyfit-deg-1 extrapolate (below is that
            # SAME rationale applied to h/optical-flow, which still uses it) — the
            # CENTROID path itself was changed 2026-07-17 to the feature KF's own
            # predict-only step (self._kf_feat_x) instead of a hand-rolled polyfit; see
            # that block further down (past the h-extrapolation code immediately below)
            # for why (feedback_s2_homogeneous_decay_bug memory).
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

            # Centroid s (2026-07-17, user correction: "the decay-to-zero behavior for
            # position is intentional design, but I never approved it. We have kf_predict.")
            # REPLACED the polyfit-then-decay-to-zero extrapolation with the ALREADY-EXISTING
            # feature KF's own predict-only step. The decay-to-zero design (position ->
            # exactly [0,0] after H_EXTRAP_DECAY_FRAMES=10 consecutive misses) was found to
            # cause a REAL problem, not just a stylistic one: it fabricates a "we are
            # centered, zero error" reading regardless of the drone's actual (unmeasured)
            # position, discarding the KF's own velocity/momentum state -- when real tracking
            # resumes, s snaps from the fabricated zero back to a genuine nonzero value, and
            # if the adaptive gain (kappa) had frozen/ratcheted during the fabricated-zero
            # window (reading "converged"), the resulting mismatch between an already-elevated
            # frozen gain and a suddenly-real error produces an amplified corrective a_u spike
            # -- confirmed live (IC1 SITL trace, 2026-07-17): kappa ratcheted 3.57->4.18 then
            # FROZE exactly as a coast began, and a_u spiked to 2976 right as the coast likely
            # ended. Fix: step the feature KF's PREDICT-ONLY branch (self._kf_feat_update(None,
            # t) -- already exists, already runs every coast frame at the later call site
            # below, just moved earlier so THIS frame's s can read its result) and use
            # self._kf_feat_x[:, 0] (constant-velocity-propagated estimate, uncertainty-
            # growing via the KF's own P matrix, NOT a hand-rolled decay) as the position.
            # The later call site is now SKIPPED when not rescuing (this predict already ran)
            # to avoid double-stepping the KF's dt accounting in one frame.
            self._kf_feat_update(None, self._time.perf_counter())
            extrapolated_img_feature_param = self._kf_feat_x[:, 0].copy()
            extrapolated_img_feature_param = np.nan_to_num(
                np.asarray(extrapolated_img_feature_param), nan=0.0, posinf=5.0, neginf=-5.0)
            extrapolated_img_feature_param = np.clip(extrapolated_img_feature_param, -5.0, 5.0)
            # HOMOGENEOUS-CONSTANT FIX (2026-07-17, found via IC5 SITL trace, still applied as
            # a hard safety redundant clamp after the 2026-07-17 KF-predict switch above): s[2]
            # is the fixed homogeneous "1.0" component of s=[xc,yc,1.0,alpha] -- a structural
            # constant, never a tracked/decaying KF channel. The KF's own scale-channel state
            # should already track ~1.0 (every real correction feeds it z[2]=1.0), but this
            # stays as a hard floor regardless -- originally found when the OLD polyfit+decay
            # extrapolation (now removed) collapsed s[2] to 0.0 after a long coast, producing
            # s=[0,0,0,-3.068] and a single-frame a_u spike to 610,997.
            extrapolated_img_feature_param[2] = 1.0
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

            # PLANARFEATUREMAP RESCUE (2026-07-16): this is the map's ACTUAL intended use --
            # user correction, same day. The earlier V_aruco_norm[1] override (single-frame
            # bake, 2026-07-15) only ever fired INSIDE `if FEATURE_DATA_IS_LOGGED:`, i.e. only
            # on frames where the raw ArUco/KLT-fallback/dense-recovery tiers had ALREADY
            # succeeded -- it could polish an already-good frame but could NEVER rescue one
            # where the marker is partially visible or not visible at all, which is exactly
            # the scenario the module's own docstring describes ("the marker's position
            # becomes one entry in this map, inferable even when the marker itself is briefly
            # untrackable, as long as enough other mapped features remain visible"). THIS is
            # that path: self._planar_map_primary_pred_px is computed unconditionally each
            # frame (in the shadow/prediction-capture block above, gated by the hysteresis
            # gate, independent of whether ArUco/KLT/dense-recovery decoded anything this
            # frame) -- so when it's available, it grounds this frame's s/alpha in the map's
            # OTHER currently-tracked scene features, a strictly better-informed estimate
            # than a pure polynomial extrapolation of past trend with zero current-frame
            # visual grounding at all. Overrides the extrapolation wholesale (not blended)
            # when available; falls through to the extrapolation above untouched otherwise.
            _pm_rescue = (self._planar_map_primary and self._planar_map_gate_on
                          and self._planar_map_primary_pred_px is not None)
            if _pm_rescue:
                _pm_v1 = self._getVirtualPts(
                    np.asarray(self._planar_map_primary_pred_px, np.float32), quats[1])
                if _pm_v1 is not None and len(_pm_v1) == 4:
                    # PHYSICAL-PLAUSIBILITY REJECTION (2026-07-16, user correction): a
                    # blanket np.clip(..., -5, 5) here was WRONG, not just loose -- s[0:2]
                    # are normalized image coordinates ((u-cx)/fx), so a point at the true
                    # edge of THIS camera's visible frame is ~1.2, not 5. That +-5.0 bound
                    # was inherited unexamined from the OLD polynomial-extrapolation guard
                    # (a genuinely different failure mode: an unbounded FITTED TREND, where
                    # 5.0 was a reasonable runaway cap) -- never reconsidered for a
                    # GEOMETRIC map projection, which should almost always land at or near
                    # the real visible frame. map_confidence says nothing about whether
                    # THIS SPECIFIC projected position is geometrically sane -- a confident
                    # map can still project a point far outside the camera's FoV if its
                    # homography has drifted (e.g. from accumulated error during the same
                    # extended dropout the rescue exists to bridge). Clamping such a value
                    # to a bound (any bound) still hands the controller a fabricated
                    # position -- s_e_n = s_e/p_10 amplifies it by 1/p_10 (~3-7x for this
                    # camera), so even a moderately-wrong clamped s can read as a
                    # double-digit normalized error, triggering the same kappa-ratchet/a_u
                    # explosion this project has repeatedly traced to a bad position
                    # signal. Confirmed live: exactly this chain produced IC1's 75m/138m
                    # fly-away (ICValidation/20260716-211434). Fix: REJECT (not clip) a
                    # rescue whose projected centroid/size falls outside the same bounds
                    # used to guard the override path (shared helper, see
                    # _planarMapPredictionPlausible) -- fall through to hold/decay instead
                    # of feeding a physically-impossible position to the controller.
                    _ok, _pm_v1_checked, _why = self._planarMapPredictionPlausible(
                        self._planar_map_primary_pred_px, quats[1])
                    if _ok:
                        _pm_s = self._computeFeatureVec(size_factor * _pm_v1_checked)
                        extrapolated_img_feature_param = _pm_s
                    else:
                        _pm_rescue = False
                        if os.environ.get("PLANAR_MAP_DBG", "0") == "1":
                            print(f"[planar_map rescue] REJECTED implausible {_why} "
                                  f"-- falling back to hold/decay", flush=True)
                else:
                    _pm_rescue = False
            self._planar_map_rescue_active = _pm_rescue   # exposed via RESCUE_ACTIVE, see __init__ comment

            # CENTROID-RATE OBSERVER injection: log the observer flow (decoded-corner centroid rate)
            # instead of the zero "no information" default when it produced a value this frame.
            _observer_fresh = (self._single_marker and self._centroid_rate and self._observer_valid)
            _of = (size_factor * self._observer_flow if _observer_fresh
                   else extrapolated_opt_flow_ang_vel_raw)
            self._opt_flow_ang_vel_raw.append(_of)
            self._h_estimator_tag.append('observer_full' if _observer_fresh else 'coast')
            # Corner-flow KF: the observer branch is genuine fresh data (real correct-step);
            # otherwise this frame has nothing but a synthetic decayed extrapolation — coast the
            # KF (predict-only, no correction) rather than freezing it (no step at all) or
            # correcting it against a fabricated value. See feedback_kf_frozen_during_marker_loss.
            self._kf_update(_of if _observer_fresh else None, self._time.perf_counter())
            self._imu_angvel_raw.append(np.full(3, np.nan))   # marker lost: no synced IMU pairing
            # Guard #1 companion: keep the quat VALID through marker-LOST — the FC attitude is
            # genuine (use-genuine-data directive). Only nan if the FC quat itself is missing.
            _qL = quats[1] if (quats is not None and len(quats) > 1 and quats[1] is not None) else None
            self._quat_log.append(np.array([_qL.w, _qL.x, _qL.y, _qL.z]) if _qL is not None else np.full(4, np.nan))
            self._img_feature_param.append(extrapolated_img_feature_param)
            self._s_estimator_tag.append('planar_map_rescue' if _pm_rescue else 'coast')
            # _centroidMap/_alphaMap/_flowMap (the OVERRIDE consumers) never run in this branch
            # (raw decode failed this frame) -- keep the raw-log arrays lockstep-aligned with
            # _s_estimator_tag/_h_estimator_tag regardless (see __init__).
            self._cmap_raw_log.append((np.nan, np.nan)); self._cmap_trust_log.append(0.0)
            self._amap_raw_log.append(np.nan); self._amap_trust_log.append(0.0)
            self._fmap_raw_log.append((np.nan, np.nan, np.nan))
            # Centroid KF: the map-rescue branch is a genuine current-frame geometric estimate
            # (grounded in this frame's OTHER tracked scene features, not a blind trend fit) --
            # real correct-step, same treatment as the observer's h-rescue above. The
            # predict-only coast step (the `else None` case) was MOVED EARLIER (2026-07-17,
            # see the "Centroid s" comment above) so this frame's logged s could read its
            # result -- calling it again here with None would double-step the KF's dt
            # accounting within one frame, so only call when rescuing (a genuine correct-step
            # against the rescue's fresh geometric estimate, which wasn't available yet at the
            # earlier predict call). See feedback_kf_frozen_during_marker_loss.
            if _pm_rescue:
                self._kf_feat_update(extrapolated_img_feature_param, self._time.perf_counter())
            self._n_flow_corners.append(0)   # extrapolated frame: no fresh corners
            self._drift_off_hist.append(self._last_drifted_off)   # latched value (2026-07-07 failure-cause tagging)
            self._overflow_hist.append(self._last_overflow)

            # RESCUED CORNER GEOMETRY (2026-07-17, user correction — the FEATURE_IS_STALE gate
            # I'd put on the CBF was a band-aid: that flag is a legacy RAW-decode-miss counter
            # that predates PlanarFeatureMap and has zero awareness of a successful rescue, so
            # it fires after just STALE_THRESH=3 raw misses even while the map is rescuing
            # EVERY one of those frames with a plausibility-checked estimate. The actual root
            # cause: _feature_pts (what MARKER_EXTENT_PX / the CBF's d_min_fov read) was NEVER
            # updated with the rescue's geometry -- only _img_feature_param (the s vector) got
            # it; this line unconditionally re-appended the stale previous corners regardless
            # of whether a fresh rescue succeeded. Fix: when _pm_rescue succeeded this frame,
            # carry the plausibility-checked rescued pixel corners into _feature_pts (paired
            # with the previous frame's current-corners as [0], matching the raw C_nP=[prev,
            # curr] convention) so MARKER_EXTENT_PX/CBF see LIVE (map-grounded) geometry
            # instead of frozen raw corners. Only a genuine TOTAL coast (no raw, no rescue)
            # still holds the last value -- see FEATURE_PTS_FRESH below for consumers.
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

    def _kf_step(self, x, P, prev_t, initialized, z, t, q, r, dt_unc_max=None):
        """Generic per-channel 2-state (value, rate) constant-velocity KF step.
        Operates on the passed state (no self.* writes) and returns the updated
        (x, P, prev_t, initialized) so the SAME filter can run on any channel
        count / noise params (corner flow, ring flow, centroid feature all share
        this). z: (C,) measurement, or None for a PREDICT-ONLY step (coast on the
        last estimated rate, skip the correction — used during a marker-loss gap
        so the state neither freezes (no step at all) nor gets corrected against
        a synthetic/extrapolated value as if it were real data). t: timestamp.
        q, r: this channel's process/measurement noise. dt_unc_max: if set, Q's
        dt uses the TRUE elapsed gap (capped at dt_unc_max) instead of the
        state-transition dt (capped at 0.1s for numerical stability) — so P
        correctly reflects staleness across a multi-frame gap, and a fresh
        measurement at relock gets full Bayesian trust instead of a multi-frame
        catch-up ramp (see feedback_kf_frozen_during_marker_loss).

        PURELY LINEAR — every channel here must be a plain unwrapped scalar. A
        WRAPPED ANGLE (e.g. alpha) must NEVER be passed to this directly: the
        innovation y=z-x_pred is a raw subtraction, so a measurement landing
        near/across the +-pi branch cut produces a spurious ~2*pi innovation
        that corrupts the rate state and then runs away unbounded during a
        predict-only coast (see feedback_alpha_kf_wrap_bug — this is why
        _kf_feat_update filters [sin(alpha), cos(alpha)] as two ordinary linear
        channels through this same function, rather than alpha itself)."""
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
        # Discrete-white-noise-on-acceleration process model
        Q = q * np.array([
            [dt_q**4 / 4.0, dt_q**3 / 2.0],
            [dt_q**3 / 2.0, dt_q**2],
        ])
        # Predict: x ← Fx, P ← FPF^T + Q  (vectorized over channels)
        x_pred = x @ F.T
        P_pred = F @ P @ F.T + Q
        if z is None:
            return x_pred, P_pred, t, True         # PREDICT-ONLY: coast, no correction
        z = np.asarray(z, dtype=float)
        # Innovation y = z - Hx_pred (H = [1, 0]), scalar per channel
        y = z - x_pred[:, 0]
        S = P_pred[:, 0, 0] + r
        K = P_pred[:, :, 0] / S[:, None]
        # Update: x ← x_pred + K·y, P ← (I - K H) P_pred
        x = x_pred + K * y[:, None]
        P = P_pred - K[:, :, None] * P_pred[:, 0:1, :]
        return x, P, t, True

    def _kf_update(self, z, t):
        """Corner-flow KF — thin wrapper around _kf_step on the corner state.
        z=None -> predict-only coast (marker-loss gap, no correction)."""
        self._kf_x, self._kf_P, self._kf_prev_t, self._kf_initialized = self._kf_step(
            self._kf_x, self._kf_P, self._kf_prev_t, self._kf_initialized, z, t,
            self._kf_q, self._kf_r, dt_unc_max=self._kf_dt_unc_max)

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
        self._ekf_x, self._ekf_P = x, P

    def getTargetVel(self):
        """Estimated target/rover velocity in flow units (h_tv) from the fusion EKF;
        ~0 for a stationary target. Zeros unless FLOW_FUSE_RING=1. Control feedforward."""
        return self._ekf_x[3:6].copy() if self._ekf_init else np.zeros(3)

    def _kf_feat_update(self, z, t):
        """4-channel feature KF for the centroid+alpha (xc, yc, scale, alpha) — thin wrapper
        around the shared _kf_step, with its OWN (q, r) (self._kf_feat_q/_r) — the flow KF's
        q=5/r=0.1 are mis-scaled for the order-1 centroid (see __init__). Low-lag alternative
        to savgol(13) for the OUTER-loop centroid input. z : (4,) raw feature, or None for a
        PREDICT-ONLY coast (marker-loss gap, no correction — mirrors _kf_update; see
        feedback_kf_frozen_during_marker_loss). t : perf_counter.

        CHANNELS 0-2 (xc, yc, scale) run through _kf_step directly — plain linear
        quantities. CHANNEL 3 (alpha) is a WRAPPED ANGLE and is instead tracked via a
        separate sin/cos-pair sub-filter (self._kf_feat_sc_x, 2026-07-19 — see
        feedback_alpha_kf_wrap_bug): [sin(alpha), cos(alpha)] run through the SAME
        _kf_step as two ordinary linear channels (no branch cut, no asymptote — unlike
        tan(alpha), which is pi-periodic and would reintroduce the 2-fold ambiguity
        _marker_principal_angle's corner weighting exists to remove). self._kf_feat_x[3, 0]
        is kept synced to atan2(sin_state, cos_state) purely so every existing consumer
        (extrapolation, sensor-cal readout) keeps reading a plain alpha scalar from the
        same slot; self._kf_feat_x[3, 1] (rate) is unused externally and is not meaningful
        in this representation, left at 0.

        CONFIDENCE-SCALED r (2026-07-20, see feedback_confidence_scaled_kf_r): "map is
        authoritative" (2026-07-20) means an accepted map sample's VALUE is used fully
        regardless of where its confidence sits between the reject and full-trust floors
        -- but self._kf_feat_r=0.004 tells this KF EVERY correction is near-ground-truth
        (K~=1, confirmed empirically: post-correction state tracks raw*gain almost
        exactly), so a frame accepted near the reject floor -- e.g. a slot whose geometry
        is actively degrading toward rejection, which was exactly the diagnosed IC3
        158m fly-away mechanism -- gets treated as fully trustworthy and can dominate the
        RATE state in one step. The value taper this design replaced (trust*map +
        (1-trust)*decode) accidentally solved this by shrinking the numeric step size for
        a low-confidence sample before it ever reached the KF; scaling r here is the
        textbook equivalent for a Kalman filter (tell it a sample is noisier, so it barely
        moves K) WITHOUT touching the reported value at all -- map stays authoritative,
        but the state a future coast extrapolates from stays protected. Applies to
        map-sourced frames (trust>0) via confidence; DECODE-sourced frames (trust==0, map
        didn't fire) are ALSO scaled now (2026-07-21, see feedback_decode_klt_confidence_
        scaled_r below) by a KLT-fallback-streak confidence, so decode isn't the
        unconditionally-trusted assumption every other fix this session was protecting
        against."""
        z_pos = None if z is None else np.asarray(z, dtype=float)[:3]
        r_pos = self._kf_feat_r
        if z is not None:
            _map_tr = float(self._cmap_trust_log[-1]) if self._cmap_trust_log else 0.0
            if _map_tr > 0.0:
                r_pos = np.array([self._kf_feat_r / max(_map_tr, 0.05),
                                   self._kf_feat_r / max(_map_tr, 0.05),
                                   self._kf_feat_r])
            else:
                # DECODE-SOURCED this frame (map didn't fire) -- confidence-scale r by the
                # SAME KLT-fallback-streak-based confidence the flow channel's _corner_conf
                # already uses (2026-07-21, see feedback_decode_klt_confidence_scaled_r):
                # every fix this session treated decode as unconditionally trustworthy, the
                # thing the map's confidence/rate gates protect AGAINST -- but decode's OWN
                # KLT-fallback corners degrade too, the deeper into a fallback streak with no
                # fresh ArUco re-anchor. Diagnosed IC4_rep5's terminal excursion: the map
                # correctly abstained (tapered trust to 0, then stopped firing) during a
                # genuine drift-off event, while decode's KLT-fallback corners -- left as the
                # sole source once the map abstained -- kept feeding the KF at full strength
                # through the SAME degrading stretch, producing the actual blowup. Same
                # formula as _corner_conf (max(0.05, 1 - lk_step_count/max_lk_steps)) so a
                # fresh decode (lk_step_count==0) is unaffected; only a deep fallback streak
                # softens the KF's trust here, exactly mirroring the map-side fix.
                _decode_conf = (1.0 if self._lk_step_count <= 0 else
                                 max(0.05, 1.0 - self._lk_step_count / max(self._max_lk_steps, 1)))
                if _decode_conf < 1.0:
                    r_pos = np.array([self._kf_feat_r / max(_decode_conf, 0.05),
                                       self._kf_feat_r / max(_decode_conf, 0.05),
                                       self._kf_feat_r])
        x_pos, P_pos, self._kf_feat_prev_t, self._kf_feat_initialized = \
            self._kf_step(self._kf_feat_x[:3], self._kf_feat_P[:3], self._kf_feat_prev_t,
                          self._kf_feat_initialized, z_pos, t,
                          self._kf_feat_q, r_pos, dt_unc_max=self._kf_dt_unc_max)
        self._kf_feat_x[:3] = x_pos
        self._kf_feat_P[:3] = P_pos

        z_sc = None if z is None else np.array([np.sin(float(z[3])), np.cos(float(z[3]))])
        r_sc = self._kf_feat_r
        if z is not None:
            _amap_tr = float(self._amap_trust_log[-1]) if self._amap_trust_log else 0.0
            if _amap_tr > 0.0:
                r_sc = self._kf_feat_r / max(_amap_tr, 0.05)
            else:
                # DECODE-sourced alpha this frame -- same KLT-fallback-streak confidence
                # as the position channels above (see that comment for the full
                # rationale); decode's alpha comes from the same degrading corners.
                _decode_conf_a = (1.0 if self._lk_step_count <= 0 else
                                   max(0.05, 1.0 - self._lk_step_count / max(self._max_lk_steps, 1)))
                if _decode_conf_a < 1.0:
                    r_sc = self._kf_feat_r / max(_decode_conf_a, 0.05)
        self._kf_feat_sc_x, self._kf_feat_sc_P, self._kf_feat_sc_prev_t, self._kf_feat_sc_initialized = \
            self._kf_step(self._kf_feat_sc_x, self._kf_feat_sc_P, self._kf_feat_sc_prev_t,
                          self._kf_feat_sc_initialized, z_sc, t,
                          self._kf_feat_q, r_sc, dt_unc_max=self._kf_dt_unc_max)
        if self._kf_feat_sc_initialized:
            self._kf_feat_x[3, 0] = np.arctan2(self._kf_feat_sc_x[0, 0], self._kf_feat_sc_x[1, 0])
            self._kf_feat_x[3, 1] = 0.0

        if self._kf_feat_initialized:
            self._kf_feat_ever_initialized = True   # sticky -- see __init__ comment

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

    def _marker_principal_angle(self, pts, weights=None):
        """2pi-periodic marker orientation in the (level) V plane (raw, no offset).

        weights (2026-07-19, stage 3): optional explicit per-point weight array, same
        length as pts. None (default) preserves the original behavior EXACTLY -- [4,3,2,1]
        for a 4-corner quad, uniform otherwise. Lets the MAP-DRIVEN path (get_marker_points'
        corner+dense set, richer than 4 corners alone) reuse this SAME moment+disambiguation
        math instead of duplicating it -- never re-derive this logic, see _computeFeatureVec's
        docstring for why (a board<->map<->fallback switch must never jump conventions).

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
        if weights is not None:
            w = np.asarray(weights, dtype=float)
        elif len(x) == 4:
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
        s = self._computeFeatureVec(pts)
        self._img_feature_param.append(s)
        # NOTE: the CALLER runs the feature KF's correct-step (_kf_feat_update) AFTER this
        # returns, once the ds outlier-hold guard has run on self._img_feature_param[-1] --
        # see the call site (single-marker branch). Correcting the KF here would feed it the
        # PRE-guard (possibly outlier) value; do not add a KF update here.

    def _computeFeatureVec(self, pts):
        """Pure computation of the feature vector [xc, yc, 1.0, alpha] from a set of V-frame
        points (4 ArUco corners normally; also used directly by the PlanarFeatureMap RESCUE
        path below, on the map's reprojected corners, since it's the same geometric
        computation regardless of source). Extracted from _getImgFeatures (2026-07-16) so
        both the raw-decode path and the map-rescue path share ONE implementation -- never
        duplicate this math."""
        x = pts[:, 0]
        y = pts[:, 1]
        N = len(x)

        if N == 4:
            # Board equilibrium offset (shared with the board path _board_feature so
            # the board<->single-marker fallback never jumps).
            alpha_0 = self._moment_alpha_0
        else:
            alpha_0 = 0.0
        # Unweighted (geometric) centroid — the actual marker position. The [4,3,2,1]
        # corner weights (applied only inside _marker_principal_angle below) exist
        # ONLY to make yaw observable (see docstring); using them for position too
        # silently biased xc,yc toward the TL corner by an amount that ROTATES WITH
        # THE MARKER's yaw (the weights are corner-label-tied), corrupting the
        # position signal the outer loop and calibration both consume. Corrected
        # 2026-07-11 (was a real train/run mismatch vs PLASMC_GT_FEEDBACK, which
        # substitutes the true unbiased GT centroid).
        xc = float(np.mean(x))
        yc = float(np.mean(y))

        # Yaw: 2pi-disambiguated moment orientation (same convention/offset as the
        # board path _board_feature, so a board<->fallback switch never jumps). Uses
        # its OWN internal weighted centroid (_marker_principal_angle) purely for
        # direction disambiguation -- never returned as position, so this is unaffected
        # by the xc,yc fix above.
        raw = self._marker_principal_angle(pts)
        alpha = float(np.arctan2(np.sin(raw - alpha_0), np.cos(raw - alpha_0)))

        # ---- 4. Feature vector (unnormalized) ----
        return np.array([xc, yc, 1.0, alpha])

    def _mapRatePlausible(self, hist, t, vals, soft_bound, reject_bound, max_hist=5):
        """WINDOWED-RATE plausibility check shared by _centroidMap/_alphaMap (2026-07-21 --
        see the MAP_RATE_SOFT/MAP_RATE_REJECT __init__ comment for full rationale). This is
        a THIRD, INDEPENDENT axis alongside _planarMapPredictionPlausible's position/size
        checks -- confidence (and position/size plausibility) can't see a cluster that's
        coherently drifting while staying internally rigid and in-frame; this catches that
        by looking at the ACCEPTED value's own recent trend, the same pattern the loom's
        d(lnM)/dt gate (_mtrace_hist) already uses for an analogous "smooth drift, not a
        discontinuous glitch" problem in a different channel.

        hist: the caller's rolling list (self._cmap_rate_hist or self._amap_rate_hist),
        mutated in place (append + trim to max_hist) -- caller owns the list identity.
        t: timestamp for this sample. vals: tuple of linear-safe values to rate-check
        (x, y for centroid; sin(alpha), cos(alpha) for alpha -- NEVER alpha itself, which
        is a wrapped angle and would spuriously spike at the +-pi branch cut exactly like
        the linear-KF bug this session already fixed once).

        soft_bound/reject_bound/max_hist: CALLER-SUPPLIED, not shared constants (2026-07-23
        -- see feedback_alpha_rate_gate_separate_thresholds). Centroid and alpha have
        different noise characteristics (alpha's raw per-sample jitter alone can exceed
        centroid's thresholds) and need their own bounds/window length -- reusing one set
        for both let a real, diagnosed alpha drift (~0.26 rad/s sustained, ~26 deg of
        uncorrected yaw error) pass through un-tapered because it sat under the
        centroid-derived MAP_RATE_SOFT=0.3 despite being ~2.5x the alpha channel's own
        normal-noise ceiling (~0.1-0.22 rad/s over a comparably long window).

        Returns (ok: bool, rate_trust: float in [0,1]). ok=False -> caller should REJECT
        this sample entirely (same weight as a position/size plausibility failure).
        rate_trust (when ok) is meant to be combined (via min()) with confidence-derived
        trust -- it tapers how much the KF trusts this sample (via the confidence-scaled r
        mechanism), NOT the reported value itself; map stays authoritative for what's
        shown this frame regardless."""
        hist.append((t,) + tuple(vals))
        if len(hist) > max_hist:
            del hist[0]
        if len(hist) < 3:
            return True, 1.0   # not enough history to judge yet -- don't block early samples
        tt = np.array([h[0] for h in hist])
        if tt.max() - tt.min() < 1e-3:
            return True, 1.0   # degenerate window (near-duplicate timestamps)
        rate_sq = 0.0
        for k in range(1, len(vals) + 1):
            vv = np.array([h[k] for h in hist])
            slope = np.polyfit(tt - tt[0], vv, 1)[0]
            rate_sq += slope ** 2
        rate = float(np.sqrt(rate_sq))
        if rate > reject_bound:
            return False, 0.0
        if rate > soft_bound:
            return True, float(np.clip(
                1.0 - (rate - soft_bound) / max(reject_bound - soft_bound, 1e-6),
                0.0, 1.0))
        return True, 1.0

    def _planarMapPredictionPlausible(self, pm_px, quat):
        """PHYSICAL-PLAUSIBILITY check for a PlanarFeatureMap camera-pixel corner
        prediction (2026-07-16, extracted so the RESCUE and OVERRIDE consumers share
        ONE check -- never duplicate this logic, which is exactly how the override
        path shipped with NO check at all and let a drifted map projection replace a
        perfectly good raw ArUco decode undetected, confirmed live on IC2/IC3's
        terminal phase: wild V-frame centroids like [-23,24] fed straight through).

        Two INDEPENDENT failure modes, both checked:
        (1) POSITION: map_confidence (or the marker-aware confidence gating the
            override) says nothing about whether THIS SPECIFIC projected position is
            geometrically sane -- the map's homography can drift arbitrarily far while
            still reporting high internal confidence (confidence is about tracking
            density + residual, not about ground truth it has no access to). A
            confident-but-drifted map can project a point far outside the camera's
            actual visible frame.
        (2) SIZE: a position that happens to land within the FoV bound is NOT
            sufficient -- the homography can ALSO drift in SCALE while the position
            still looks plausible (confirmed live, IC5: implied extent 96.6px held for
            300ms vs true 15px on recovery -- a 6x mismatch with a perfectly
            in-frame centroid throughout).

        Returns (ok: bool, V-frame corners or None, reason: str). Callers fall back to
        their pre-existing behavior (raw decode for the override, hold/decay for the
        rescue) on ok=False -- NEVER clip/clamp an implausible value, since s_e_n =
        s_e/p_10 amplifies any residual error by ~3-7x for this camera, turning even a
        moderately-wrong clamped value into a double-digit normalized error and the
        same kappa-ratchet/a_u explosion this project has repeatedly traced to a bad
        position signal (confirmed live: IC1's 75m/138m fly-away,
        ICValidation/20260716-211434)."""
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
            return False, pm_v1, "position"

        if self._last_real_extent_px is not None and self._last_real_extent_px > 1e-6:
            pm_px_arr = np.asarray(pm_px, dtype=float)
            pm_extent = float(max(pm_px_arr[:, 0].max() - pm_px_arr[:, 0].min(),
                                   pm_px_arr[:, 1].max() - pm_px_arr[:, 1].min()))
            ratio = pm_extent / self._last_real_extent_px
            if not ((1.0 / self._planar_map_rescue_size_ratio) <= ratio <= self._planar_map_rescue_size_ratio):
                return False, pm_v1, "size"

        return True, pm_v1, "ok"

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
            "Image Stamp": self._stamp_log,
            "Centroid Map Dbg": getattr(self, '_cmap_dbg', {}),   # (2026-07-18) _centroidMap fire/none breakdown (CENTROID_MAP_DBG=1)
            "Flow Map Dbg": getattr(self, '_fmap_dbg', {}),   # (2026-07-19) stage-4 small-marker-flow fire/ready breakdown (FLOW_MAP_DBG=1)
            "Planar Map Shadow": self._planar_map_log,   # (2026-07-15) shadow-mode PlanarFeatureMap vs live-winning tier, see src/planar_map.py
            # RAW (PRE-CAL) map-sourced logs (2026-07-20, see __init__ + feedback_map_cal_validation_gap):
            # per-frame, lockstep with Feature Params/Opt Flow Ang Vel -- (nan/0) when that frame's
            # map function didn't fire. Needed to check whether _sensor_cal_s/_sensor_cal_hw (fit
            # against the decode path only) are still correct for CENTROID_FROM_MAP/ALPHA_FROM_MAP/
            # FLOW_FROM_MAP's structurally different raw computations.
            "Centroid Map Raw": self._cmap_raw_log,     # (2,) raw V-frame centroid, pre-_sensor_cal_s
            "Centroid Map Trust": self._cmap_trust_log, # soft-gate blend weight used this frame [0,1]
            "Alpha Map Raw": self._amap_raw_log,        # raw alpha (rad), pre-_sensor_cal_s
            "Alpha Map Trust": self._amap_trust_log,    # soft-gate blend weight used this frame [0,1]
            "Flow Map Raw": self._fmap_raw_log,         # (3,) raw [hx,hy,hz], pre-_sensor_cal_hw
        }
    
    def getParams(self):
        parameter = f"{{'Capture Rate':{self._capRate}, 'resolution':{self._resolution}}}"
        return parameter
    
    def _effectiveCalSxy(self):
        """(gx, gy) centroid gain for THIS readout. MAP IS AUTHORITATIVE (2026-07-20, user
        directive: "decode is just for cross-correction") -- whenever _centroidMap accepted
        this frame (self._cmap_trust_log[-1] > 0; the override already used the map value
        FULLY, not blended, see the call site), use the map-sourced gain FULLY too, not
        blended by degree of confidence -- a partial gain blend against a FULL value swap
        was internally inconsistent, and produced a real step artifact at every
        decode<->map transition (gain jumped even though it wasn't tapering anything).
        Uses decode's cal_s[0:2] exactly when the map didn't fire this frame (CENTROID_FROM_MAP
        off, or the gate rejected/never fired) -- not because decode is a backup value source,
        but because that's what s[0:2] mechanically holds then (decode's cross-check
        computation, always run first -- see the override call site). Decode's cal_s is never
        blended with the map's gain; the map, once accepted, is authoritative."""
        fired = bool(self._cmap_trust_log) and self._cmap_trust_log[-1] > 0.0
        if fired:
            return float(self._sensor_cal_s_map_xy[0]), float(self._sensor_cal_s_map_xy[1])
        return float(self._sensor_cal_s[0, 0]), float(self._sensor_cal_s[1, 1])

    def getRawImgFeatureParam(self):
        """Image feature vector BEFORE _sensor_cal_s. Used by output_calibration."""
        if len(self._img_feature_param) == 0:
            return np.zeros(4)
        return np.array(self._img_feature_param[-1])

    def getRawImgFeatureEstimatorTag(self):
        """Which estimator produced the latest getRawImgFeatureParam() sample:
        'board_homography' | 'single_marker_moment' | 'coast' (synthetic
        extrapolation — exclude from any calibration fit). '' if no sample yet
        (back-compat: older recordings predate this field entirely)."""
        if len(self._s_estimator_tag) == 0:
            return ''
        return self._s_estimator_tag[-1]

    def getRawCentroidMapFeature(self):
        """Latest raw (2,) V-frame centroid from _centroidMap, BEFORE _sensor_cal_s and BEFORE
        the soft-gate blend — (nan, nan) if it didn't fire this frame (map unavailable, gate
        rejected, or CENTROID_FROM_MAP=0). Co-sampled by record_output_calibration.py, same
        clock as getRawImgFeatureParam(), so this needs NO separate GT/img alignment — see
        feedback_map_cal_validation_gap."""
        if len(self._cmap_raw_log) == 0:
            return np.array([np.nan, np.nan])
        return np.array(self._cmap_raw_log[-1], dtype=float)

    def getRawAlphaMapFeature(self):
        """Latest raw alpha (rad) from _alphaMap, BEFORE _sensor_cal_s and BEFORE the soft-gate
        blend — nan if it didn't fire this frame. Same co-sampling convention as
        getRawCentroidMapFeature()."""
        if len(self._amap_raw_log) == 0:
            return np.nan
        return float(self._amap_raw_log[-1])

    def getRawFlowMapFeature(self):
        """Latest raw (3,) [hx, hy, hz] from _flowMap, BEFORE _sensor_cal_hw — (nan, nan, nan) if
        it didn't fire this frame (no secondary/small marker slot, size band not met, or
        FLOW_FROM_MAP=0). Same co-sampling convention as getRawCentroidMapFeature()."""
        if len(self._fmap_raw_log) == 0:
            return np.array([np.nan, np.nan, np.nan])
        return np.array(self._fmap_raw_log[-1], dtype=float)

    def getRawOptFlowEstimatorTag(self):
        """Which estimator produced the latest getRawOptFlowAngVel() sample:
        'lstsq'/'lstsq_klt' (merged calibration category — see
        feedback_klt_fallback_merge_no_separate_cal) | '+observer_xy' suffix
        (h_x/h_y observer-overridden, h_z/w still lstsq) | 'observer_full' (all
        channels from the centroid-rate observer) | 'coast' (synthetic
        predict-only extrapolation — exclude from any calibration fit). '' if no
        sample yet."""
        if len(self._h_estimator_tag) == 0:
            return ''
        return self._h_estimator_tag[-1]

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
        # Never let the window read before the CONTROLLER_READY engage point (see
        # _engage_idx_flow comment in _reset_stateful_trackers) -- clip the available
        # history to post-engage samples only, so the first few post-engage frames don't
        # silently blend in pre-engage measurements.
        _post_engage = self._opt_flow_ang_vel_raw[self._engage_idx_flow:]
        if len(_post_engage) == 0:
            return np.zeros(6)
        if len(_post_engage) >= FILTER_WIN:
            sgf_buf = sgf(_post_engage[-FILTER_WIN:],
                          FILTER_WIN, FILTER_POLYORDER, axis=0)
            return sgf_buf[int(FILTER_WIN / 2 + 1)]
        return np.mean(_post_engage, axis=0)

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
            return _out
        _out = self._sensor_cal_hw @ self._kf_x[:, 0]
        if self._loom_decouple:
            _out[2] = self._kf_x[2, 0]                       # loom already vz/Z, bypass cal row 2
        return _out

    @property
    def RESCUE_ACTIVE(self):
        """True iff THIS frame's s/alpha/h_z came from the PlanarFeatureMap rescue rather
        than a raw ArUco/KLT/dense-recovery decode -- see __init__ comment. apps/
        landing_test.py's feature_fresh check reads this alongside FEATURE_IS_STALE so a
        rescue-covered dropout can keep running closed-loop control instead of falling
        into the grace-hold/TARGET_LOST path meant for genuinely-no-information frames."""
        return self._planar_map_rescue_active

    @property
    def FEATURE_PTS_FRESH(self):
        """True iff _feature_pts (raw pixel corners -- what MARKER_EXTENT_PX and the CBF's
        d_min_fov consume) was updated with LIVE geometry this frame (raw decode/KLT OR a
        plausibility-checked PlanarFeatureMap rescue), vs held at its last value (a genuine
        total coast -- neither raw nor rescue available this frame). Distinct from
        FEATURE_IS_STALE (a legacy RAW-miss-only counter with STALE_THRESH persistence and
        no rescue awareness -- it flips True after 3 consecutive raw misses even while the
        map is successfully rescuing every one of them). Consumers that read _feature_pts
        directly (not the s/alpha/h pipeline, which RESCUE_ACTIVE already covers) should
        gate on this, not FEATURE_IS_STALE."""
        return self._feature_pts_fresh

    @property
    def SMALL_SLOT_CONFIDENT(self):
        """True iff PlanarFeatureMap's secondary (smaller, by construction) slot is
        confidently mapped right now -- same threshold + source the CBF itself uses for
        its own small-marker preference (CBF_SMALL_SLOT_CONF_MIN), read here too so the
        CBF's corner choice and the CBF-driven HANDOVER_LATCHED path (see
        update_cbf_handover_signal) stay consistent with each other."""
        if self._planar_map is None or not getattr(self._planar_map, 'initialized', False):
            return False
        sec = self._planar_map.secondary_slot_name()
        if sec is None:
            return False
        return self._planar_map.get_slot_confidence(sec) >= self._cbf_small_conf_min

    def _mapSlotVpts(self, quat):
        """De-rotated (V-frame) corners of the map's HANDOVER-GATED slot -- PRIMARY (big) before
        HANDOVER_LATCHED, SECONDARY (small) after (the user's big-before/small-after selection).
        Shared source for the decode->map migration (loom M stage 1, centroid stage 2): the map
        tracks each slot continuously (KLT+homography), so these corners don't flicker across a
        big<->small decode blink, and both slots share one map with the online-learned relative
        size (continuous scale across the single latched switch). Uses the SAME _getVirtualPts
        de-rotation as the decode path -> directly substitutable. None if unavailable."""
        pm = self._planar_map
        if pm is None or not getattr(pm, 'initialized', False) or quat is None:
            return None
        slot = pm.secondary_slot_name() if self.HANDOVER_LATCHED else None  # None -> primary (big)
        try:
            px = pm.get_marker_frame_pts(slot=slot)   # continuous per-slot corners (pixel frame)
            if px is None or len(px) != 4:
                return None
            Vp = self._getVirtualPts(np.asarray(px, np.float32), quat)
            if Vp is None or len(Vp) != 4:
                return None
            return Vp
        except Exception:
            return None

    def _loomMapM(self, quat):
        """Map-driven loom scale M = μ20+μ02 (de-rotated corner scatter ∝ (sz·f/Z)²), STAGE 1
        of the decode->map migration. Returns None if unavailable (caller falls back to decode M)."""
        Vp = self._mapSlotVpts(quat)
        if Vp is None:
            return None
        ctr = Vp.mean(axis=0)
        M = float(np.mean(np.sum((Vp[:, :2] - ctr[:2]) ** 2, axis=1)))
        return M if (M > 1e-12 and np.isfinite(M)) else None

    def _quadIllConditioned(self, pts):
        """True iff a 4-corner marker quad is geometrically DEGENERATE -- a foreshortened sliver
        that cannot localize a centre or well-condition a homography (2026-07-19, user). Verified
        signature at the tilt-grazing terminal: the two diagonals go near-PARALLEL (angle ~1.3deg
        vs ~90 when square) and/or the quad ELONGATES (max/min side up to 70x vs ~1 square). Either
        makes the diagonal-intersection centre swing wildly (~1/sin(angle) amplification) and the
        homography under-determined. Ordered ArUco corners (TL,TR,BR,BL): diagonals are 0-2 & 1-3.
        Thresholds env-tunable. Corner-order-agnostic elongation via all 4 side lengths."""
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
            _min_ang = float(os.environ.get("MAP_ILLCOND_ANGLE_DEG", "20"))
            _max_elong = float(os.environ.get("MAP_ILLCOND_ELONG", "5"))
            return bool(ang < _min_ang or elong > _max_elong)
        except Exception:
            return True

    def _quadConditionScore(self, pts):
        """CONTINUOUS companion to _quadIllConditioned (2026-07-30, for confidence-weighted
        loop_closure_correct, ported from Hardware/scripts/img_geometry.py's
        quad_condition_score): 1.0 = well-conditioned (near-square), 0.0 = at or beyond the
        same ill-conditioned thresholds _quadIllConditioned rejects on. Same geometry, just
        returned as a smooth [0,1] score so a marginal decode can be BLENDED instead of
        binary accept/reject."""
        try:
            c = np.asarray(pts, dtype=float).reshape(-1, 2)
            if c.shape[0] != 4 or not np.all(np.isfinite(c)):
                return 0.0
            d1 = c[2] - c[0]; d2 = c[3] - c[1]
            n1 = np.linalg.norm(d1); n2 = np.linalg.norm(d2)
            if n1 < 1e-6 or n2 < 1e-6:
                return 0.0
            ang = np.degrees(np.arccos(np.clip(abs(np.dot(d1, d2) / (n1 * n2)), 0.0, 1.0)))
            sides = [np.linalg.norm(c[(i + 1) % 4] - c[i]) for i in range(4)]
            elong = max(sides) / max(min(sides), 1e-6)
            _min_ang = float(os.environ.get("MAP_ILLCOND_ANGLE_DEG", "20"))
            _max_elong = float(os.environ.get("MAP_ILLCOND_ELONG", "5"))
            w_ang = np.clip((ang - _min_ang) / max(90.0 - _min_ang, 1e-6), 0.0, 1.0)
            w_elong = np.clip((_max_elong - elong) / max(_max_elong - 1.0, 1e-6), 0.0, 1.0)
            return float(w_ang * w_elong)
        except Exception:
            return 0.0

    def _markerEdgeMarginScore(self, pts, resolution_hw, margin_px):
        """CONTINUOUS companion to the inline near-edge check at the loop_closure_correct
        call site (2026-07-30, ported from Hardware/scripts/img_geometry.py's
        marker_edge_margin_score): 1.0 = every corner well clear of the frame boundary,
        0.0 = at or inside margin_px of the edge. Ramps linearly in between."""
        try:
            c = np.asarray(pts, dtype=float).reshape(-1, 2)
            h, w = resolution_hw
            min_dist = min(c[:, 0].min(), w - c[:, 0].max(), c[:, 1].min(), h - c[:, 1].max())
            return float(np.clip(min_dist / max(margin_px, 1e-6), 0.0, 1.0))
        except Exception:
            return 0.0

    def _centroidMap(self, quat, size_factor=1.0):
        """Map-driven V-frame centroid [xc, yc], STAGE 2. Uses the map's NATIVE (projective-
        invariant diagonal-intersection) center reprojected to the frame, NOT the corner mean:
        the true center is robust to a partially-visible/overflowing marker (fixed gauge center
        + all-feature homography), and the corner-mean was shown to shift off-center under
        perspective (user, 2026-07-18). de-rotate that single pixel via the SAME _getVirtualPts
        the decode path uses. quat = frame-1 (quats[1]), matching the validated map override/
        rescue convention (controller.py-consumed V_aruco_norm[1] is frame-1). size_factor
        matches the decode feature (1.0 for the single id-0 marker). None -> caller keeps decode.

        RIGIDITY/CONFIDENCE gate (2026-07-19, SOFT as of the same day — see
        feedback_soft_rigidity_gate): the position+size plausibility gate below catches an
        off-FoV/off-scale reprojection, but NOT a slot whose tracked geometry is
        in-place-but-corrupted (marker_rigid_ok=False, confidence collapsed) — that geometry
        can still look plausible on POSITION+SIZE while its internal corner agreement is
        garbage. Diagnosed IC4 case: a single frame with err_px=1281 and confidence=0.0/
        marker_rigid_ok=False fed a bad centroid through, contributing to a 4.5m miss. A
        first pass hard-rejected anything below _planar_map_conf_floor (0.5) — this then
        regressed IC3 (13m miss), which spent 4+ real seconds with confidence genuinely
        stuck in the 0.0-0.5 band, so hard-rejecting all of it left the override
        contributing nothing for the whole stretch (pure, uncorrected KF coast). Fixed:
        confidence below _planar_map_reject_floor (~0.05, matching where the diagnosed
        garbage frames actually sat) still hard-rejects; between the two floors, this
        method still computes and returns the map value, but self._cmap_last_trust (read
        by the caller) is a confidence-proportional blend weight instead of the implicit
        1.0 a return always used to carry — the caller blends toward decode rather than
        swapping wholesale, so a partially-degraded slot contributes SOME signal instead
        of none."""
        _dbg = os.environ.get("CENTROID_MAP_DBG", "0") == "1"
        def _tally(k):
            if _dbg:
                self._cmap_dbg = getattr(self, '_cmap_dbg', {})
                self._cmap_dbg[k] = self._cmap_dbg.get(k, 0) + 1
        self._cmap_last_trust = 1.0
        pm = self._planar_map
        if pm is None:
            _tally('none_pm_is_None'); return None
        if not getattr(pm, 'initialized', False):
            _tally('none_not_initialized'); return None
        if quat is None:
            _tally('none_quat_None'); return None
        slot = pm.secondary_slot_name() if self.HANDOVER_LATCHED else None   # big before handover, small after
        _conf = pm.get_slot_confidence(slot) if slot is not None else pm.confidence
        if _conf < self._planar_map_reject_floor:
            _tally('none_low_confidence'); return None
        self._cmap_last_trust = float(np.clip(
            (_conf - self._planar_map_reject_floor)
            / max(self._planar_map_conf_floor - self._planar_map_reject_floor, 1e-6),
            0.0, 1.0))
        try:
            # PLAUSIBILITY GATE (2026-07-18) -- the OVERFLOW FIX. This override shipped WITHOUT the
            # check the rescue/override paths use (_planarMapPredictionPlausible), which is exactly
            # the bug that method's docstring warns about ("the override path shipped with NO check
            # at all and let a drifted map projection replace a perfectly good raw ArUco decode").
            # Diagnosed source of the map centroid's terminal spikes: when the marker LEAVES the FoV
            # (IC5: |s| up to 3.4, 18% of frames past the 1.2 FoV bound), the homography reprojects
            # an OFF-SCREEN EXTRAPOLATED center that decode would flag as garbage but the map emits
            # as a plausible-looking 1-3.4 value. Run the SAME position+size plausibility check on
            # the slot's corner prediction; if it fails, REJECT (fall back to decode) rather than
            # feed an off-FoV extrapolation the controller amplifies by 1/p_10 (~3-7x).
            corners_px = pm.get_marker_frame_pts(slot=slot)
            if corners_px is None or len(corners_px) != 4:
                _tally('none_native'); return None
            _ok, _v, _why = self._planarMapPredictionPlausible(corners_px, quat)
            if not _ok:
                _tally('none_implausible'); return None
            px = pm.get_marker_center_native(slot=slot)      # single true-center pixel
            if px is None:
                _tally('none_native'); return None
            Vp = self._getVirtualPts(np.asarray([px], np.float32), quat)      # de-rotate the one point
            if Vp is None or len(Vp) != 1:
                _tally('none_vpts'); return None
            c = size_factor * Vp[0, :2]
            if not np.all(np.isfinite(c)):
                _tally('none_nonfinite'); return None
            # WINDOWED-RATE plausibility (2026-07-21, see _mapRatePlausible docstring +
            # MAP_RATE_SOFT/MAP_RATE_REJECT __init__ comment) -- a THIRD axis alongside
            # position/size, catching a coherently-drifting-but-rigid cluster the
            # confidence gate above can't see.
            _rate_ok, _rate_trust = self._mapRatePlausible(
                self._cmap_rate_hist, self._time.perf_counter(), (float(c[0]), float(c[1])),
                self._map_rate_soft, self._map_rate_reject)
            if not _rate_ok:
                _tally('none_rate_implausible'); return None
            self._cmap_last_trust = min(self._cmap_last_trust, _rate_trust)
            _tally('fired'); return c
        except Exception:
            _tally('none_exc'); return None

    def _alphaMap(self, quat):
        """Map-driven yaw feature alpha, STAGE 3 (2026-07-19). Reuses _marker_principal_angle
        UNCHANGED -- only the point SOURCE differs: get_marker_points() (handover-gated slot)
        supplies the corner+dense-interior (position, weight) set instead of just the 4 decode
        corners, the richer/lower-variance input the map's own class docstring names this for.
        Same PLAUSIBILITY gate as the centroid (corner prediction position+size sanity) before
        trusting the slot at all -- a degenerate/off-FoV slot has no reliable orientation either.
        RIGIDITY/CONFIDENCE gate (2026-07-19, SOFT as of the same day -- see
        feedback_soft_rigidity_gate, same rationale/shape as _centroidMap): unlike the
        centroid override, orientation is far more sensitive to a slot whose KLT corner
        tracking has degraded but not yet failed the (coarser) position/size plausibility
        check -- a non-rigid/low-confidence point set can still sit "in the right place"
        while its INTERNAL geometry (and therefore the principal angle) drifts smoothly and
        unboundedly. Diagnosed: both observed IC1/IC5 blowups occurred with
        marker_rigid_ok=False, confidence=0.0 in the shadow log immediately prior -- a hard
        reject at _planar_map_conf_floor (0.5) fixed those, but (per _centroidMap's
        docstring) a hard floor also starves the override during an extended
        genuinely-degraded-but-not-garbage stretch. Below _planar_map_reject_floor (~0.05,
        where the diagnosed blowup frames actually sat): still hard reject. Between the two
        floors: self._amap_last_trust (read by the caller) is a confidence-proportional
        blend weight, and the caller circular-blends toward decode instead of swapping
        wholesale. Applies the SAME _moment_alpha_0 offset as the decode path so a
        decode<->map fallback never jumps convention. None -> caller keeps decode alpha
        untouched."""
        self._amap_last_trust = 1.0
        pm = self._planar_map
        if pm is None or not getattr(pm, 'initialized', False) or quat is None:
            return None
        slot = pm.secondary_slot_name() if self.HANDOVER_LATCHED else None
        _conf = pm.get_slot_confidence(slot) if slot is not None else pm.confidence
        if _conf < self._planar_map_reject_floor:
            return None
        self._amap_last_trust = float(np.clip(
            (_conf - self._planar_map_reject_floor)
            / max(self._planar_map_conf_floor - self._planar_map_reject_floor, 1e-6),
            0.0, 1.0))
        try:
            corners_px = pm.get_marker_frame_pts(slot=slot)
            if corners_px is None or len(corners_px) != 4:
                return None
            _ok, _v, _why = self._planarMapPredictionPlausible(corners_px, quat)
            if not _ok:
                return None
            pts_px, wts = pm.get_marker_points(slot=slot)
            if pts_px is None or wts is None or len(pts_px) < 3:
                return None
            Vp = self._getVirtualPts(np.asarray(pts_px, np.float32), quat)
            if Vp is None or len(Vp) != len(pts_px):
                return None
            raw = self._marker_principal_angle(Vp, weights=wts)
            a = float(np.arctan2(np.sin(raw - self._moment_alpha_0), np.cos(raw - self._moment_alpha_0)))
            if not np.isfinite(a):
                return None
            # WINDOWED-RATE plausibility (2026-07-21, see _mapRatePlausible docstring) --
            # rate-checked on [sin(a), cos(a)], NEVER on the wrapped angle `a` itself (a
            # linear slope fit across the +-pi branch cut would spuriously spike, exactly
            # the class of bug the sin/cos-pair KF fix already addressed once this session).
            # ALPHA-SPECIFIC bounds + longer window (2026-07-23, see __init__ comment /
            # feedback_alpha_rate_gate_separate_thresholds) -- NOT the centroid constants.
            _rate_ok, _rate_trust = self._mapRatePlausible(
                self._amap_rate_hist, self._time.perf_counter(), (np.sin(a), np.cos(a)),
                self._map_alpha_rate_soft, self._map_alpha_rate_reject,
                max_hist=self._map_alpha_rate_hist_len)
            if not _rate_ok:
                return None
            self._amap_last_trust = min(self._amap_last_trust, _rate_trust)
            return a
        except Exception:
            return None

    def _smallSlotExtentPx(self, quat):
        """Current apparent size (px, V-frame RMS corner-to-centre distance * focal) of the
        map's SECONDARY (smaller) slot -- STAGE 4 (2026-07-19, user). None if unavailable."""
        pm = self._planar_map
        if pm is None or not getattr(pm, 'initialized', False) or quat is None:
            return None
        sec = pm.secondary_slot_name()
        if sec is None:
            return None
        try:
            px = pm.get_marker_frame_pts(slot=sec)
            if px is None or len(px) != 4:
                return None
            Vp = self._getVirtualPts(np.asarray(px, np.float32), quat)
            if Vp is None or len(Vp) != 4:
                return None
            ctr = Vp[:, :2].mean(axis=0)
            ext_v = float(np.sqrt(np.mean(np.sum((Vp[:, :2] - ctr) ** 2, axis=1))))
            fx = self.focal[0] if self.focal is not None else 270.0
            return ext_v * fx
        except Exception:
            return None

    def _smallSlotFlowReady(self, quat):
        """Is the small (secondary) marker BOTH confidently mapped AND in the SIZE BAND where
        flow (a velocity DERIVATIVE, needs corner-spread observability) is reliable -- STAGE 4
        (2026-07-19, user directive + characterization). Two-sided gate, NOT just "big enough":
        - too SMALL (< MAP_FLOW_MIN_EXT_PX): corner spread too tight, flow noise dominates
          (measured: median error 0.49 vs GT V-frame flow at ~8px extent, vs 0.06-0.07 in the
          15-35px band -- a ~7x jump).
        - too LARGE (> MAP_FLOW_MAX_EXT_PX): approaching overflow/edge-foreshortening, flow
          degrades AGAIN (measured: median error back up to 0.17-0.18 past ~40px, matching the
          same tilt-grazing/overflow degradation traced for the loom/centroid). This is the
          "can't wait for the big marker to fill the FoV" half of the directive -- the reliable
          window is a BAND, not a threshold, and it closes again well before overflow.
        Defaults (15, 40 px) are this session's IC1-5 characterization; MAY need re-deriving if
        camera intrinsics/marker size change. Independent of HANDOVER_LATCHED (loom/centroid/
        alpha's gate) -- this is a SEPARATE, typically EARLIER, size-gated readiness."""
        if not self.SMALL_SLOT_CONFIDENT:
            return False
        ext = self._smallSlotExtentPx(quat)
        if ext is None:
            return False
        return self._flow_map_min_ext <= ext <= self._flow_map_max_ext

    def _flowMap(self, quat, t):
        """Map-driven small-marker flow [h_x, h_y, h_z], STAGE 4. h_x/h_y: the SAME validated
        2-state CV-Kalman rate estimator the observer uses (_obs_vel_kf), fed the small slot's
        map-derived V-frame centre instead of a decoded centroid -- own KF state so this never
        interferes with the observer's. h_z: _loomMapM on the secondary slot explicitly
        (independent of HANDOVER_LATCHED, matching this stage's own earlier size gate).
        None -> caller keeps whichever source (lstsq/observer) was already active this frame."""
        pm = self._planar_map
        if pm is None or not getattr(pm, 'initialized', False) or quat is None:
            return None
        # DURATION CAP: see _flowmap_min_confidence comment at __init__ (2026-07-30).
        if float(getattr(pm, 'map_confidence', 1.0)) < self._flowmap_min_confidence:
            return None
        sec = pm.secondary_slot_name()
        if sec is None:
            return None
        try:
            px = pm.get_marker_center_native(slot=sec)
            if px is None:
                return None
            Vp = self._getVirtualPts(np.asarray([px], np.float32), quat)
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
            hz = self._loomMapM_slot(sec, quat)
            if hz is None:
                return None
            if not all(np.isfinite(v) for v in (hx, hy, hz)):
                return None
            return np.array([hx, hy, hz], dtype=float)
        except Exception:
            return None

    def _loomMapM_slot(self, slot, quat):
        """Loom scale->rate for an EXPLICIT slot (STAGE 4 helper): same μ20+μ02 + causal
        linear-fit d(lnM)/dt as _loomMapM, but bypassing its HANDOVER_LATCHED slot selection --
        this stage gates on SIZE, independently and typically earlier than the handover latch."""
        pm = self._planar_map
        if pm is None or not getattr(pm, 'initialized', False) or quat is None:
            return None
        try:
            px = pm.get_marker_frame_pts(slot=slot)
            if px is None or len(px) != 4:
                return None
            Vp = self._getVirtualPts(np.asarray(px, np.float32), quat)
            if Vp is None or len(Vp) != 4:
                return None
            ctr = Vp[:, :2].mean(axis=0)
            M = float(np.mean(np.sum((Vp[:, :2] - ctr) ** 2, axis=1)))
            if not (M > 1e-12 and np.isfinite(M)):
                return None
            t = float(getattr(self, '_stamp', 0.0))
            lnM = float(np.log(M))
            self._flowmap_lnM_hist = getattr(self, '_flowmap_lnM_hist',
                                             deque(maxlen=int(os.environ.get("FLOW_LOOM_WIN", "9"))))
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

    def update_cbf_handover_signal(self, cbf_overflow):
        """CBF-driven alternative path to HANDOVER_LATCHED (2026-07-17, user design),
        called once per step from apps/landing_test.py (which owns both the controller and
        this img_node). Complements — does NOT replace — the existing loom-M-drop detector
        above: two independent signals for "big marker has overflowed AND small is
        trustworthy", either can latch. Same one-way latch, same persistence discipline
        (HANDOVER_PERSIST_FRAMES) rejecting a single-frame blip. cbf_overflow is the
        controller's CBF_OVERFLOW (span-classified, per-corner FoV margin); small-slot
        confidence is checked here via SMALL_SLOT_CONFIDENT so both conditions are
        evaluated against the SAME live map state."""
        if self.HANDOVER_LATCHED:
            return
        # HANDOVER_REQUIRE_OVERFLOW=1 (default): legacy AND -- CBF_OVERFLOW *and* small-slot.
        # =0 (user directive 2026-07-17): small-slot confidence ALONE, flicker-immune (the map
        # tracks the small slot through the decode flicker that starves both legacy paths).
        _cond = (self.SMALL_SLOT_CONFIDENT if not self._handover_require_overflow
                 else (bool(cbf_overflow) and self.SMALL_SLOT_CONFIDENT))
        if _cond:
            self._handover_cbf_cand += 1
            if self._handover_cbf_cand >= self._handover_persist:
                self.HANDOVER_LATCHED = True
                if self._handover_dbg:
                    _via = ("small-slot-confident alone" if not self._handover_require_overflow
                            else "CBF overflow+small-slot-confident")
                    print(f"[HANDOVER] latched via {_via} "
                          f"t={float(getattr(self, '_stamp', 0.0)):.2f}", flush=True)
        else:
            self._handover_cbf_cand = 0

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
        # Pure read — the KF is STEPPED in the capture loop itself (real correct-step
        # on a fresh sample, predict-only coast on a marker-loss gap; see the
        # _kf_feat_update call sites), exactly once per camera frame regardless of how
        # often the controller polls this getter. Do not step it here (2026-07-11: the
        # old lazy step-on-array-growth here couldn't distinguish real from
        # extrapolated samples, so a gap always got a full correct-step against a
        # synthetic value instead of a coast — see feedback_kf_frozen_during_marker_loss).
        if os.environ.get('IMG_FEATURE_FILTER', 'kf') != 'savgol':
            # Gate on EVER-initialized (sticky across a marker-switch reset of the plain
            # _kf_feat_initialized flag), not the plain flag itself -- see __init__
            # comment (2026-07-15 bugfix: the plain-flag gate fell through to the
            # unrelated savgol/mean path below for one-two frames on every marker-ID
            # switch, producing a value fully disconnected from the KF's still-valid last
            # state; _kf_feat_x itself is never cleared by that reset, only the flag).
            if self._kf_feat_ever_initialized:
                gx, gy = self._effectiveCalSxy()   # source-aware centroid gain, see __init__
                out = self._sensor_cal_s @ self._kf_feat_x[:, 0]
                out[0] = gx * self._kf_feat_x[0, 0]
                out[1] = gy * self._kf_feat_x[1, 0]
                return out
        # Same post-engage clipping as _compute_savgol_output -- see _engage_idx_feat
        # comment in _reset_stateful_trackers.
        _post_engage = self._img_feature_param[self._engage_idx_feat:]
        if len(_post_engage) == 0:
            return np.zeros(4)
        gx, gy = self._effectiveCalSxy()   # source-aware centroid gain, see __init__
        if len(_post_engage) >= FILTER_WIN:
            sgf_buf = sgf(_post_engage[-FILTER_WIN:],
                          FILTER_WIN, FILTER_POLYORDER, axis=0)
            _feat = sgf_buf[int(FILTER_WIN / 2 + 1)]
        else:
            _feat = np.mean(_post_engage, axis=0)
        out = self._sensor_cal_s @ _feat
        out[0] = gx * _feat[0]
        out[1] = gy * _feat[1]
        return out