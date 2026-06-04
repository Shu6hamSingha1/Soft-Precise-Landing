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
        self.RECORD = False
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

        # Sensor calibration — MULTISINE height-validated M (2026-06-02 night),
        # CONFIRMED over 4 descent runs. FULL 6x6: calibrated [h;w]=M@raw.
        #
        # Derived via CALIB_MODE=multisine (apps/output_calibration.py): freq-
        # multiplexed excitation (x@0.40,y@0.55,yaw@0.25Hz — distinct freqs ->
        # GT axes decorrelated -> GT=M@raw lstsq separates them) during a slow
        # 2-cycle z-sweep 5<->0.3m, lateral gated >1.2m, yaw throughout. Fit
        # binned by altitude shows M is HEIGHT-STABLE (diag ~const, R^2 0.8+
        # from 0.5-5.5m) — overturns the earlier "no single M spans descent"
        # (that was noise in 2-run discrete sweeps). Pooled over 4 runs / 49k
        # samples: R^2 Hx 0.81 Hy 0.83 Hz 0.92 Wx 0.49 Wy 0.50 Wz 0.78.
        # Reproducible: diag Hx 0.749±4%, Hy 0.683±2%, Hz 0.864±4%, Wz 0.610±6%
        # (tilt Wx/Wy noisy 16-27%). The h<->w coupling (off-diagonals in the
        # (h0,w1)/(h1,w0) pairs) is geometric -> holds for moving targets too.
        #
        # NOTE: this GT-accurate M landed WORSE than the prior board M in n<=3
        # IC1 tests (1.088/0.889-crash/1.565 vs 0.675m) — the controller is
        # co-tuned to the older cal's magnitudes and the residual is LAG-limited,
        # not cal-limited. Kept per user decision (correct cal of record);
        # closing the landing gap is the lag fix / controller retune, not cal.
        # Prior 5m board M recoverable: Obsolete/src/img_data_5mM_20260602.py.
        # Order in [h;w]: [h_x, h_y, h_z, w_x, w_y, w_z].
        self._sensor_cal_hw = np.array([
            [+0.7126, -0.0190, +0.0115, -0.0268, +0.4315, +0.0042],
            [-0.0291, +0.6817, +0.0072, -0.3727, +0.0732, -0.0099],
            [+0.0005, +0.0010, +0.8583, -0.0545, -0.0106, +0.0023],
            [+0.0050, -0.7570, -0.0153, +0.6147, -0.0552, -0.0101],
            [+0.7930, +0.0092, +0.0106, -0.0370, +0.6207, +0.0014],
            [+0.0052, +0.0364, +0.0238, +0.0171, -0.1310, +0.6088]])
        self._sensor_cal_s  = np.diag([1.0986, 1.0562, 1.0, 1.0])

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
        self._lk_params = dict(
            winSize=(21, 21),
            maxLevel=3,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01),
            minEigThreshold=1e-3,
        )

        # 2026-05-31 — V_YAW_SOURCE: pick where the virtual frame's yaw
        # comes from. 'compass' (default, legacy) uses the drone's full
        # quaternion (so V is yaw-locked to the drone's heading, which
        # is in turn compass-derived). 'alpha' uses the marker's
        # principal-axis angle from this frame's ArUco corners, so V
        # is marker-aligned and the whole flow/centroid pipeline becomes
        # compass-independent. See _imgProcess for the rotation; the
        # controller-side alpha-offset is also disabled below.
        self._v_yaw_source = os.environ.get("V_YAW_SOURCE", "compass")

        # 2026-06-02 — MULTI-MARKER BOARD layout.
        # The landing pad carries several ArUco markers at KNOWN, non-
        # overlapping board offsets (Images/aruco_board_layout.npy:
        # id -> (x_m, y_m, size_m), board centre at origin). The OPTICAL FLOW
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
        # Yaw-reference offset for the board (analogue of the single-marker
        # alpha_0). Board axes are explicit, so default 0; re-calibrate if the
        # board's +x is not aligned with the desired equilibrium heading.
        self._board_alpha_0 = float(os.environ.get("BOARD_ALPHA0", "0.0"))

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
        self._prev_aruco_pts = None       # most recent good corners (ArUco or KLT)
        self._prev_img = None             # frame those corners were measured in

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

        # Data storage
        self._time_log = []
        self._fps_log = []
        self._feature_pts = []
        self._virtual_feature_pts = []
        # _fill_A allocates A fresh each frame now (variable N with hybrid flow)
        self._quats = []
        self._img_feature_param = []
        self._opt_flow_ang_vel_raw = []
        self._n_flow_corners = []   # # corners fed to the lstsq per frame (board diag)
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
        self._kf_q = 5.0                    # process-noise PSD (rad/s² per √s)²
        self._kf_r = 0.1                    # measurement noise variance
        self._kf_x = np.zeros((6, 2))       # [value, rate] per channel
        self._kf_P = np.tile(np.eye(2) * 1.0, (6, 1, 1))
        self._kf_prev_t = None
        self._kf_initialized = False

        # Centroid-feature KF (4 channels: xc, yc, scale, alpha) — same 2-state
        # constant-velocity model as the flow KF, env IMG_FEATURE_FILTER=kf.
        # Cuts the ~110 ms group delay of savgol(13) on the OUTER-loop centroid
        # input (savgol lags the flow KF by ~7 samples + is ~2x noisier). That
        # lag is exactly where off-center convergence stalls: KP=9 commands the
        # correction but the outer PID reacts to a ~110 ms-stale centroid.
        # Default 'savgol' until A/B-validated.
        self._kf_feat_x = np.zeros((4, 2))
        self._kf_feat_P = np.tile(np.eye(2) * 1.0, (4, 1, 1))
        self._kf_feat_prev_t = None
        self._kf_feat_initialized = False
        self._kf_feat_last_n = 0

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
                self._fps = self._image_node.getFPS()   
                # print(f"Image FPS: {self._fps}")

                # Check if at least 2 frames of images have been received
                if images[0] is not None and images[1] is not None:              
                    if VIDEO:
                        # Resize display image
                        resized_img = cv2.resize(images[0], None, fx=4, fy=4, interpolation=cv2.INTER_AREA)
                        cv2.imshow('Image Streamer', resized_img)
                        if cv2.waitKey(1) == 27:
                            self.close()

                    # Uncomment the following code to record video/images
                    if self.RECORD and self.CONTROLLER_READY:
                        if self._video is None:
                            # Below VideoWriter object will store video in 'timestamp.avi' file. 
                            self.timestamp = time.ctime().replace(':', '-')
                            self._video = cv2.VideoWriter(f'/home/shubham/Soft-Precise-Landing/PX4_Gazebo/test_data/Test_Videos/{self.timestamp}.mp4',  
                                    cv2.VideoWriter_fourcc(*'mp4v'), 
                                    self._capRate, self._resolution)

                        self._video.write(images[1])

                    # Calculate the radial optical flow if it is AVAILABLE. Else the loop is restarted.
                    if self._imgProcess(images, quaternions, showVideo = VIDEO) is AVAILABLE:
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

    def _imgProcess(self, imgs, quats, showVideo = False):
        # This function will return True if the optical flow is AVAILABLE and calculate the optical flow. Else, it will return False.
        # Return type is a Boolean
        # Detect markers for both images
        size_factor = 1.0
        results = self._detector.detectMarkers(imgs[0])

        FEATURE_DATA_IS_LOGGED = False

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
            primary_i = int(np.argmin(ids))
            # Primary first, then the rest — so marker k occupies corners
            # [4k:4k+4] of all_pts_0 and marker_ids[k] is its ID. Primary stays
            # first for KLT-fallback continuity + display + the strict gate.
            order = [primary_i] + [i for i in range(M) if i != primary_i]
            marker_corners_0 = [results[0][i][0].reshape(-1, 2).astype(np.float32)
                                for i in order]
            aruco_pts_0 = marker_corners_0[0]
            if self._feature_source == 'dense':
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
                        aruco_pts_0 = lk_pts.reshape(-1, 2).astype(np.float32)
                        used_klt_fallback = True
                        self._lk_step_count += 1
                        if self._lk_step_count == 1:
                            print(f"ArUco lost — KLT fallback active (cap {self._max_lk_steps} frames)")
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

                # V_YAW_SOURCE=alpha: replace the compass-derived yaw of the V
                # frame with the marker's principal-axis angle (alpha). V is
                # then marker-aligned rather than NED-aligned; flow and
                # centroid are measured in the marker's frame, removing the
                # dependency on PX4's compass. The gravity (roll/pitch) part
                # of V is unchanged — it still comes from the accelerometer-
                # derived quaternion component.
                #
                # Implementation: compute alpha from the current V_aruco
                # (which is in compass-V frame) using the same weighted-moment
                # formula as _getImgFeatures, then rotate every V point by
                # -alpha around z. Resulting V is marker-aligned.
                if self._v_yaw_source == 'alpha':
                    alpha_cv = self._marker_principal_angle(V_aruco_norm[1])
                    c_a, s_a = float(np.cos(alpha_cv)), float(np.sin(alpha_cv))
                    R2d_T = np.array([[c_a, -s_a], [s_a, c_a]])    # rotate pts by -alpha
                    V_aruco_norm = [pts @ R2d_T for pts in V_aruco_norm]
                    V_flow_norm  = [pts @ R2d_T for pts in V_flow_norm]
                    if board_markers_V is not None:
                        board_markers_V = [(mid, c @ R2d_T) for mid, c in board_markers_V]

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

                B_v, residuals, rank, sv = np.linalg.lstsq(A, Y, rcond=1e-3)
                cond = (sv[0] / sv[-1]) if (len(sv) > 0 and sv[-1] > 0) else np.inf
                bad = (
                    rank < 6
                    or cond > 1e4
                    or not np.all(np.isfinite(B_v))
                    or np.max(np.abs(B_v)) > 50.0
                )
                if bad:
                    B_v = np.zeros(6)
                B_v = np.clip(B_v, -10.0, 10.0)

                B_v_scaled = size_factor * B_v
                self._opt_flow_ang_vel_raw.append(B_v_scaled)
                self._n_flow_corners.append(int(len(flow_pts_1)))
                # 2-state KF update — only on a fresh raw measurement.
                self._kf_update(B_v_scaled, self._time.perf_counter())
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
                if self.FEATURE_IS_STALE:
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
                self._prev_img = None
                self._lk_step_count = 0

        # Log the recorded data
        self._time_log.append(self._time.perf_counter())
        self._quats.append(quats)
        self._fps_log.append(self._fps)

        if not FEATURE_DATA_IS_LOGGED:
            # Intervention 2: increment the stale streak and flip the flag
            # if we've been extrapolating for too many consecutive frames.
            self._consec_misses += 1
            if self._consec_misses >= self.STALE_THRESH and not self.FEATURE_IS_STALE:
                print(f"FEATURE_IS_STALE = True  ({self._consec_misses} consec misses)")
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

            self._opt_flow_ang_vel_raw.append(extrapolated_opt_flow_ang_vel_raw)
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

    def _kf_update(self, z, t):
        """Per-channel 2-state KF (constant-velocity) update.

        z : (6,) measurement vector (raw lstsq output, post-clip).
        t : current timestamp in seconds (monotonic perf_counter).

        On the first call, the state is initialized to z with zero rate; the
        full covariance is reset to a moderate prior so the next few updates
        adapt quickly. Subsequent calls run the standard Kalman predict +
        update with dt computed from the previous call's time.
        """
        if not self._kf_initialized:
            self._kf_x[:, 0] = z
            self._kf_x[:, 1] = 0.0
            # Diagonal prior covariance — value uncertainty ~ |z|, rate ~|z|/dt
            self._kf_P = np.tile(np.eye(2) * 1.0, (6, 1, 1))
            self._kf_prev_t = t
            self._kf_initialized = True
            return

        dt = max(min(t - self._kf_prev_t, 0.1), 1e-3)
        self._kf_prev_t = t

        F = np.array([[1.0, dt], [0.0, 1.0]])
        # Discrete-white-noise-on-acceleration process model
        Q = self._kf_q * np.array([
            [dt**4 / 4.0, dt**3 / 2.0],
            [dt**3 / 2.0, dt**2],
        ])
        R = self._kf_r

        # Vectorize across the 6 channels: each has its own (2,) state and (2,2) P.
        # Predict: x ← Fx, P ← FPF^T + Q
        x_pred = self._kf_x @ F.T                          # (6, 2)
        P_pred = F @ self._kf_P @ F.T + Q                  # (6, 2, 2)

        # Innovation y = z - Hx_pred (H = [1, 0]), scalar per channel
        y = z - x_pred[:, 0]                               # (6,)
        S = P_pred[:, 0, 0] + R                            # (6,) innovation variance
        K = P_pred[:, :, 0] / S[:, None]                   # (6, 2) Kalman gain

        # Update: x ← x_pred + K·y, P ← (I - K H) P_pred
        self._kf_x = x_pred + K * y[:, None]
        # (I - K H) P_pred: subtract K_i · P_pred[i, 0, :] from row i of P_pred
        self._kf_P = P_pred - K[:, :, None] * P_pred[:, 0:1, :]

    def _kf_feat_update(self, z, t):
        """4-channel 2-state KF for the centroid feature (xc, yc, scale, alpha).

        Identical constant-velocity model + (q, r) tuning as _kf_update, but on
        its own (4,2) state — the low-lag alternative to savgol(13) for the
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
        Q = self._kf_q * np.array([[dt**4 / 4.0, dt**3 / 2.0],
                                   [dt**3 / 2.0, dt**2]])
        R = self._kf_r
        x_pred = self._kf_feat_x @ F.T                       # (4, 2)
        P_pred = F @ self._kf_feat_P @ F.T + Q               # (4, 2, 2)
        y = z - x_pred[:, 0]                                 # (4,)
        S = P_pred[:, 0, 0] + R                              # (4,)
        K = P_pred[:, :, 0] / S[:, None]                     # (4, 2)
        self._kf_feat_x = x_pred + K * y[:, None]
        self._kf_feat_P = P_pred - K[:, :, None] * P_pred[:, 0:1, :]

    def _marker_principal_angle(self, pts):
        """Marker principal-axis angle in the (level) V plane.

        Same weighted-moment formula as _getImgFeatures, but returns just the
        raw angle (before alpha_0 offset) so we can use it to ROTATE V into
        marker-alignment. Used when V_YAW_SOURCE='alpha'.
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
        if abs(mu11) < 1e-9:
            return 0.0
        return 0.5 * np.arctan2(2 * mu11, mu20 - mu02)

    def _board_corners(self, mid):
        """Board-plane coords (metres, board centre = origin) of marker `mid`'s
        4 corners, in cv2.aruco order [TL, TR, BR, BL]. Board +x = texture
        column (right), +y = texture row (down) — matching make_aruco_board.py.
        """
        cx, cy, sz = self._board_layout[mid]
        h = sz / 2.0
        return np.array([[cx - h, cy - h],   # TL
                         [cx + h, cy - h],   # TR
                         [cx + h, cy + h],   # BR
                         [cx - h, cy + h]],  # BL
                        dtype=np.float64)

    def _board_feature(self, markers_V, size_factor=1.0):
        """Board centroid s=(xc,yc,1,alpha) from a board->V-frame homography.

        markers_V : list of (id, 4x2 V-frame-normalized corners) for the markers
                    whose all-4 corners survived LK this frame.

        Fits a planar homography H from board-plane coords (known offsets) to
        the V-frame image coords, then maps the board centre (0,0) -> V to get
        a stable, occlusion-robust centroid, and the board +x direction -> V to
        get a true yaw. Returns None if the fit is degenerate (caller falls
        back to single-marker moments).

        Each marker contributes 4 correspondences, so a single marker (4 pts)
        is the minimum for a homography (8 DOF); >=2 markers over-determine it.
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

        def apply(p):
            v = H @ np.array([p[0], p[1], 1.0])
            if abs(v[2]) < 1e-12:
                return None
            return v[:2] / v[2]

        center = apply((0.0, 0.0))
        xdir = apply((0.05, 0.0))      # small step along board +x near centre
        if center is None or xdir is None:
            return None
        d = xdir - center
        if np.hypot(d[0], d[1]) < 1e-9:
            return None
        alpha = float(np.arctan2(d[1], d[0])) - self._board_alpha_0
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

          Fundamental limitation: 2nd-moment alpha has π-period symmetry
          (mu_11, mu_20, mu_02 all invariant under 180° rotation), so
          alpha=0 has two equilibria 180° apart. For a symmetric ArUco
          this is harmless (orientation doesn't affect landing precision);
          for directional landings, an asymmetric marker geometry would
          be needed.

          Falls back to uniform weighting (MATLAB-equivalent) when N != 4.
        """
        x = pts[:, 0]
        y = pts[:, 1]
        N = len(x)

        if N == 4:
            w = np.array([4.0, 3.0, 2.0, 1.0])
            # When V_YAW_SOURCE='alpha', V is already rotated by -alpha_raw in
            # _imgProcess, so the marker principal axis sits along V.x and the
            # measured alpha here is ~0. The compass-V bias offset (-0.9379)
            # would push the controller's input to a constant non-zero value,
            # creating a permanent yaw-error signal. Use 0 in alpha-mode.
            alpha_0 = 0.0 if self._v_yaw_source == 'alpha' else -0.9379
        else:
            w = np.ones(N)
            alpha_0 = 0.0
        W = w.sum()

        xc = float(np.sum(w * x) / W)
        yc = float(np.sum(w * y) / W)

        Xc = x - xc
        Yc = y - yc
        mu20 = float(np.sum(w * Xc * Xc))
        mu02 = float(np.sum(w * Yc * Yc))
        mu11 = float(np.sum(w * Xc * Yc))

        if abs(mu11) < 1e-6:
            alpha = -alpha_0
        else:
            alpha = 0.5 * np.arctan2(2 * mu11, (mu20 - mu02)) - alpha_0

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
        # World-down expressed in the camera (= body, since R_V_from_body = I) frame.
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
            "N Flow Corners": self._n_flow_corners,
            "Opt Flow KF": self._opt_flow_kf_log,
            "Opt Flow Savgol": self._opt_flow_savgol_log,
            "FPS": self._fps_log
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
        Used by `output_calibration.py` to derive `_sensor_cal_hw`.
        """
        if len(self._opt_flow_ang_vel_raw) == 0:
            return np.zeros(6)
        return np.array(self._opt_flow_ang_vel_raw[-1])

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
        """
        if os.environ.get('IMG_FILTER', 'kf') == 'savgol':
            return self._sensor_cal_hw @ self._compute_savgol_output()
        return self._sensor_cal_hw @ self._kf_x[:, 0]

    def getImgFeatureParam(self):
        """Calibrated, savgol-smoothed image-feature vector (4-vec).

        Same pattern as getOptFlowAngVel — sliding-window savgol on raw centroid /
        scale / alpha. Until buffer fills, fall back to mean.
        """
        if len(self._img_feature_param) == 0:
            return np.zeros(4)
        # KF path (env IMG_FEATURE_FILTER=kf): low-lag alternative to savgol.
        # Step the centroid KF once per fresh raw sample (the controller calls
        # this getter every control iteration, possibly faster than the camera).
        if os.environ.get('IMG_FEATURE_FILTER', 'savgol') == 'kf':
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