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
from ahrs import Quaternion

CHECK_NUM = 80
# f = 269.96816635131836 # Focal length in pixels
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
f = fx # Focal length in pixels
# NOTE: this hfov (~35 deg) is NOT the same as the Gazebo sim camera (1.74 rad
# / ~99.7 deg @640x480) - matching resolution alone never reproduced the sim's
# normalized-pixel geometry; PLASMC gains tuned against the sim FoV are not
# guaranteed to carry over as-is, now confirmed by real measurement.
# f = 1 # Focal length in pixels
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

        # ArUco marker detection setup
        self._arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self._arucoParams = cv2.aruco.DetectorParameters()
        self._detector = cv2.aruco.ArucoDetector(self._arucoDict, self._arucoParams) 

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
        self._roi_margin_px = int(os.environ.get("ARUCO_ROI_MARGIN_PX", "80"))
        self._roi_max_misses = int(os.environ.get("ARUCO_ROI_MAX_MISSES", "5"))
        self._roi_miss_count = 0
        self._last_locked_corners = None   # (4,2) full-image px, most recent lock
        self._roi_hits = 0                 # diagnostic counters
        self._roi_misses = 0
        self._fullframe_searches = 0

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

        # Per-axis 2-state constant-velocity Kalman filter (Gazebo-aligned), the
        # DEFAULT runtime filter replacing savgol(51). Low-lag (<~1 sample) vs
        # savgol's win/2 group delay, which at the ~6-22 Hz flow rate was seconds.
        # Flow KF on the 6 [h;w] channels; feature KF on the 4 [xc,yc,1,alpha].
        # IMG_FILTER / IMG_FEATURE_FILTER = 'savgol' restore the legacy filter.
        self._kf_q = float(os.environ.get("FLOW_KF_Q", "5.0"))   # process-noise PSD
        self._kf_r = float(os.environ.get("FLOW_KF_R", "0.1"))   # measurement noise variance
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
        _fracs = [float(x) for x in
                  os.environ.get("FLOW_RING_FRACS", "0.17,0.33,0.50,0.67,0.83").split(",")]
        _ring_radii = [fr * _Rmax for fr in _fracs]
        # 30/radius (was 60): halves the LK-tracked point count (5 radii x
        # 30 = 150, was 300), directly cutting calcOpticalFlowPyrLK cost -
        # the dominant CPU-bound stage once ArUco was fixed (see
        # img_process_freq_optimization.md). Azimuthal sampling density is
        # still well above what a 6-DOF lstsq divergence/flow fit needs.
        _ring_npts = int(os.environ.get("FLOW_RING_NPTS", "30"))
        _ring_pts_V = []
        for _rr in _ring_radii:
            _ra = 2.0 * np.pi * np.arange(_ring_npts) / _ring_npts
            _ring_pts_V.append(np.c_[(_rr / f) * np.cos(_ra), (_rr / f) * np.sin(_ra)])
        self._ring_pts0_V = np.vstack(_ring_pts_V).astype(np.float32)
        self._ring_lk_params = dict(
            winSize=(int(os.environ.get("FLOW_RING_LK_WIN", "41")),) * 2,
            maxLevel=int(os.environ.get("FLOW_RING_LK_LVL", "3")),
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01),
            minEigThreshold=float(os.environ.get("FLOW_RING_LK_EIG", "1e-2")),
        )
        self._ring_on = os.environ.get("FLOW_RINGS", "1") == "1"
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
        self._ring_opt_flow_raw = []   # logged (aligned with corner flow) for ring cal

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
        self._fps_log = []
        self._feature_pts = []
        self._virtual_feature_pts = []
        self._A = np.zeros((8, 6))  # 4 points → 8 rows
        self._quats = []
        self._img_feature_param = []
        self._opt_flow_ang_vel_raw = []
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

                    # Calculate the radial optical flow if it is AVAILABLE. Else the loop is restarted.
                    if self._optFlowAngVel(images, quaternions, angvels, showVideo = VIDEO, main_imgs = main_images) is AVAILABLE:
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
                        time.sleep(1/100) # 100 Hz
                        continue

                    else:
                        print("OPTIC FLOW AVAILABLE NOW...")
                        AVAILABLE = True
                        self._count_check_opt_flow = 0

                    self._calc_time = self._time.perf_counter() - timer_flag

                    time.sleep(1/100) # 100 Hz

                    # Uncomment the following code to record video/images
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

                        frame_to_write = debayer_bayer_to_bgr(images[1])
                        self._video.write(frame_to_write)

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
        while (any(image is None for image in self._image_node.getImages())
               or any(image is None for image in self._image_node.getMainImages())):
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
                  f"roi_misses={self._roi_misses} fullframe={self._fullframe_searches} | {_parts}")
            self._roi_hits = 0; self._roi_misses = 0; self._fullframe_searches = 0
            self._timing_accum = {}
            self._timing_n = 0

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
                    scaled_results = [
                        (tuple((corner + _off) * self._aruco_scale for corner in corners), ids, rejected)
                        for corners, ids, rejected in roi_results
                    ]
                    self._roi_miss_count = 0
                    self._roi_hits += 1
                    return scaled_results
                self._roi_miss_count += 1
                self._roi_misses += 1
                if self._roi_miss_count >= self._roi_max_misses:
                    self._last_locked_corners = None   # force re-acquire via full-frame
        # Full-frame search on the main stream: first lock, ROI miss this
        # call, or too many consecutive ROI misses (cleared above).
        self._fullframe_searches += 1
        if self._detector is not None:
            full_results = [self._detector.detectMarkers(img) for img in list(main_imgs)]
        else:
            full_results = [cv2.aruco.detectMarkers(img, self._arucoDict, parameters=self._arucoParams)
                             for img in main_imgs]
        return [(tuple(corner * self._aruco_scale for corner in corners), ids, rejected)
                for corners, ids, rejected in full_results]

    def _optFlowAngVel(self, imgs, quats, angvels = None, showVideo = False, main_imgs = None):
        # This function will return True if the optical flow is AVAILABLE and calculate the optical flow. Else, it will return False.
        # Return type is a Boolean
        _tt = self._time.perf_counter()
        # Texture-free RING flow (safety net) — runs EVERY frame, independent of
        # ArUco, so it survives marker death. Steps its own KF; latest raw stored.
        if self._ring_on:
            _vvr, _pdiv, _nr, _rmom = self._compute_ring_flow(imgs, quats)
            self._ring_v_raw = _vvr; self._ring_pure_div = _pdiv; self._ring_moment = _rmom
            self._ring_ok = bool(_nr > 0 and np.all(np.isfinite(_vvr)) and np.any(_vvr != 0))
            self._ring_n = int(_nr)
            (self._kf_x_ring, self._kf_P_ring, self._kf_ring_prev_t,
             self._kf_ring_initialized) = self._kf_step(
                self._kf_x_ring, self._kf_P_ring, self._kf_ring_prev_t,
                self._kf_ring_initialized, _vvr, self._time.perf_counter())
        _tt = self._tstage(_tt, "1_ring_flow")

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
                self._locked_marker_id = new_lock
            mid = self._locked_marker_id
            i0 = int(np.where(ids0 == mid)[0][0])
            i1 = int(np.where(ids1 == mid)[0][0])

            # Locked-marker corners (4,2) in each frame.
            C_nP = [results[0][0][i0].reshape(-1, 2), results[1][0][i1].reshape(-1, 2)]
            self._last_locked_corners = np.asarray(C_nP[1], dtype=np.float32)   # for next call's ROI
            _tt = self._tstage(_tt, "3_lock_and_extract")

            V_nP_norm = [self._getVirtualPts(p, q) for p, q in zip(C_nP, quats)]
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
                    _wz = float(_wv[2])
                except Exception:
                    _wz = None
            if _wz is not None:
                _Yc = Y - A[:, 5] * _wz                       # remove yaw rotational flow
                _h = np.linalg.lstsq(A[:, 0:3], _Yc, rcond=1e-3)[0]
                B_v = np.array([_h[0], _h[1], _h[2], 0.0, 0.0, _wz])
            else:
                B_v = np.linalg.lstsq(A, Y, rcond=1e-3)[0]
            _tt = self._tstage(_tt, "7_gyro_comp_and_lstsq")

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
            # 2-state constant-velocity KF on the raw [h;w] (low-lag; getOptFlowAngVel
            # reads _kf_x). Stepped once per fresh frame.
            self._kf_update(B_v, self._time.perf_counter())
            _tt = self._tstage(_tt, "10_kf_update")

            # Feature params (centroid + yaw alpha) from the 4 primary corners.
            self._getImgFeatures(V_nP_norm[1])
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
            self._feature_pts.append(C_nP)
            self._virtual_feature_pts.append(V_nP_norm)
            self._quats.append(quats)
            self._fps_log.append(self._fps)
            self._ring_opt_flow_raw.append(self._ring_v_raw.copy())   # aligned with corner flow

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
            self._fuse_step(_corner_cal, True, 4, self._time.perf_counter())
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
                self._mtrace_hist.clear()
                self._flow_prev = None; self._s_prev = None; self._loom_lnM_prev = None
            self._fuse_step(None, False, 0, self._time.perf_counter())   # ring-only (dropout)
            self._tmark_frame_end()
            return False

        self._fuse_step(None, False, 0, self._time.perf_counter())       # ring-only (no marker)
        self._tmark_frame_end()
        return False
    
    def _fill_A(self, centered_pts):
        """centered_pts: (N,2), N>=4 virtual-frame points. Returns the (2N,6)
        IBVS interaction matrix, evaluated per point (depth Z normalized to 1;
        the 1/Z scaling folds into the sensor calibration)."""
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

    def _scaled_quad_points(self, corners, scales=(1.0, 2.0/3.0, 1.0/3.0), per_side=15):
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

    
    def _marker_principal_angle(self, pts):
        """2pi marker orientation in the level V-plane (raw, no offset).

        The weighted 2nd-moment principal axis 0.5*arctan2(2 mu11, mu20-mu02)
        gives only an AXIS (pi-period) — invariant under 180deg. It is
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

    def _getImgFeatures(self, pts):
        """pts : virtual feature points in normalized frame, shape (N,2), already
        (x,y)=(u-cx)/f. For ArUco the 4 corners are in cv2.aruco order
        [TL,TR,BR,BL].

        Centroid = GEOMETRIC (unweighted) mean — the clean lateral feature,
        decoupled from yaw. (A [4,3,2,1]-weighted centroid, as in Gazebo, is
        offset ~0.4*half-size in the MARKER frame, so it rotates with yaw and
        injects a yaw-coupled lateral bias + degrades the linear centroid cal.)

        Yaw alpha = 2pi-disambiguated WEIGHTED-moment orientation — the weights
        are required to make a square marker yaw-observable. alpha_0 = 0 here
        (Pi-specific; recalibrate the equilibrium offset once good mocap data
        exists, mirroring Gazebo's _moment_alpha_0).
        """
        x = pts[:, 0]
        y = pts[:, 1]

        # ---- Geometric centroid (yaw-decoupled lateral feature) ----
        xc = float(np.mean(x))
        yc = float(np.mean(y))

        # ---- Yaw orientation (weighted moment, 2pi) ----
        alpha_0 = 0.0
        raw = self._marker_principal_angle(pts)
        alpha = float(np.arctan2(np.sin(raw - alpha_0), np.cos(raw - alpha_0)))

        # ---- Feature vector [xc, yc, area(=1), alpha] ----
        s = np.array([xc, yc, 1.0, alpha])

        self._img_feature_param.append(s)

    def _getVirtualPts(self, pts, quat):
        # Extract roll, pitch (yaw does not affect virtual camera)
        # No controller/quaternion available (e.g. camera-only display test) ->
        # assume level (identity DCM). Real flight always has a controller, so
        # this fallback only applies to display-only diagnostics, never flight.
        R = Quaternion([quat.w, quat.x, quat.y, quat.z]).to_DCM() if quat is not None else np.eye(3)
        # ---- Remove yaw (keep roll & pitch only) ----
        # Gravity direction in camera frame
        g = R @ np.array([0, 0, 1])

        # Construct roll–pitch-only rotation
        z_axis = g / np.linalg.norm(g)
        x_axis = np.cross([0, 1, 0], z_axis)
        x_axis /= np.linalg.norm(x_axis)
        y_axis = np.cross(z_axis, x_axis)

        R_rp = np.column_stack([x_axis, y_axis, z_axis])

        R_inv = R_rp.T   # inverse rotation
        
        # Normalize image points (pixel → normalized camera coords)
        cx, cy = self.center
        x = (pts[:,0] - cx) / f
        y = (pts[:,1] - cy) / f

        # Build rays and rotate
        rays = np.column_stack([x, y, np.ones_like(x)])          # (N × 3)
        vr = rays @ R_inv.T                                      # (N × 3)

        # Reproject normalized coordinates onto virtual image plane
        z = vr[:,2]
        vx = vr[:,0] / z
        vy = vr[:,1] / z

        return np.column_stack([vx, vy])
    
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

    def _kf_step(self, x, P, prev_t, initialized, z, t):
        """Generic per-channel 2-state (value, rate) constant-velocity KF step.
        img = debayer_bayer_to_bgr(img)
        Operates on the passed state (no self.* writes) and returns the updated
        (x, P, prev_t, initialized). z: (6,) measurement; t: timestamp."""
        if not initialized:
            x = np.zeros((6, 2)); x[:, 0] = z          # value=z, rate=0
            P = np.tile(np.eye(2) * 1.0, (6, 1, 1))
            return x, P, t, True
        dt = max(min(t - prev_t, 0.1), 1e-3)
        F = np.array([[1.0, dt], [0.0, 1.0]])
        Q = self._kf_q * np.array([[dt**4 / 4.0, dt**3 / 2.0],
                                   [dt**3 / 2.0, dt**2]])
        R = self._kf_r
        x_pred = x @ F.T                                # (6, 2)
        P_pred = F @ P @ F.T + Q                        # (6, 2, 2)
        y = z - x_pred[:, 0]                            # (6,)
        S = P_pred[:, 0, 0] + R                         # (6,)
        K = P_pred[:, :, 0] / S[:, None]                # (6, 2)
        x = x_pred + K * y[:, None]
        P = P_pred - K[:, :, None] * P_pred[:, 0:1, :]
        return x, P, t, True

    def _kf_update(self, z, t):
        """Step the corner-flow KF on a fresh raw [h;w] measurement."""
        self._kf_x, self._kf_P, self._kf_prev_t, self._kf_initialized = self._kf_step(
            self._kf_x, self._kf_P, self._kf_prev_t, self._kf_initialized,
            np.asarray(z, float), t)

    def _kf_feat_update(self, z, t):
        """4-channel 2-state KF for the feature (xc, yc, 1, alpha) with its OWN
        (q, r). Low-lag alternative to savgol on the OUTER-loop centroid input."""
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
        x_pred = self._kf_feat_x @ F.T                  # (4, 2)
        P_pred = F @ self._kf_feat_P @ F.T + Q          # (4, 2, 2)
        y = z - x_pred[:, 0]                            # (4,)
        S = P_pred[:, 0, 0] + R                         # (4,)
        K = P_pred[:, :, 0] / S[:, None]                # (4, 2)
        self._kf_feat_x = x_pred + K * y[:, None]
        self._kf_feat_P = P_pred - K[:, :, None] * P_pred[:, 0:1, :]

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
        """INVERSE of _getVirtualPts (Pi convention): project V-frame (gravity-
        leveled) normalized image points back to REAL camera pixels at the current
        tilt, so the fixed ring seeds onto the nadir patch each frame (tilt-
        invariant). Forward maps rays->rays@R_rp; inverse is V_rays@R_rp.T."""
        R = Quaternion([quat.w, quat.x, quat.y, quat.z]).to_DCM()
        g = R @ np.array([0, 0, 1])
        z_axis = g / np.linalg.norm(g)
        x_axis = np.cross([0, 1, 0], z_axis); x_axis /= np.linalg.norm(x_axis)
        y_axis = np.cross(z_axis, x_axis)
        R_rp = np.column_stack([x_axis, y_axis, z_axis])
        V_rays = np.column_stack([V_pts[:, 0], V_pts[:, 1], np.ones(len(V_pts))])
        C_rays = V_rays @ R_rp.T
        cx, cy = self.center
        xr = C_rays[:, 0] / C_rays[:, 2]; yr = C_rays[:, 1] / C_rays[:, 2]
        return np.column_stack([xr * f + cx, yr * f + cy]).astype(np.float32)

    def _vframe_w(self, w_body, quat):
        """Rotate a body-FRD angular velocity into the virtual (gravity-leveled)
        frame, using the SAME basis as _getVirtualPts (Pi convention g=R@[0,0,1]).
        The camera is body-FRD (no mount rotation), so w_body = the IMU FRD rate."""
        R = Quaternion([quat.w, quat.x, quat.y, quat.z]).to_DCM()
        g = R @ np.array([0, 0, 1.0])
        z_axis = g / np.linalg.norm(g)
        x_axis = np.cross([0, 1, 0], z_axis); x_axis /= np.linalg.norm(x_axis)
        y_axis = np.cross(z_axis, x_axis)
        R_rp = np.column_stack([x_axis, y_axis, z_axis])
        return R_rp.T @ np.asarray(w_body, float)

    def _compute_ring_flow(self, imgs, quats):
        """Texture-free V-frame flow from the fixed ring stations: LK-track the
        nadir-seeded ring on the real image, de-rotate to V, then the SAME
        _fill_A+lstsq as the corner flow. Returns (V_v_ring[6], pure_div,
        n_stations, ring_moment). Runs every frame; survives ArUco loss."""
        zero = (np.zeros(6), 0.0, 0, np.nan)
        if (imgs is None or imgs[0] is None or imgs[1] is None
                or quats is None or len(quats) < 2 or quats[0] is None or quats[1] is None):
            return zero
        try:
            g0 = imgs[0] if imgs[0].ndim == 2 else cv2.cvtColor(imgs[0], cv2.COLOR_BGR2GRAY)
            g1 = imgs[1] if imgs[1].ndim == 2 else cv2.cvtColor(imgs[1], cv2.COLOR_BGR2GRAY)
            _seed = self._getRealPtsFromV(self._ring_pts0_V, quats[0])
            p1, st, _ = cv2.calcOpticalFlowPyrLK(g0, g1, _seed, None, **self._ring_lk_params)
            st = np.asarray(st).flatten().astype(bool)
            if int(st.sum()) < 6:
                return (np.zeros(6), 0.0, int(st.sum()), np.nan)
            r0 = _seed[st]; r1 = p1.reshape(-1, 2)[st]
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

    def _fuse_step(self, corner_cal, corner_ok, n_corn, t):
        """Step the fusion EKF once for this frame (corner + ring). Inert unless
        FLOW_FUSE_RING=1; uses the ring products computed at the top of the frame."""
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
        self._ekf_fuse_step(corner_cal, corner_ok, 1.0, _ring_cal, _ring_ok, t,
                            n_corn=n_corn, ring_loom=_ring_loom, ring_loom_ok=_ring_loom_ok)

    def getTargetVel(self):
        """Estimated target/rover velocity h_tv (flow units) from the fusion EKF;
        ~0 for a stationary target. Zeros unless FLOW_FUSE_RING=1."""
        return self._ekf_x[3:6].copy() if self._ekf_init else np.zeros(3)

    def getLogData(self):
        return {
            "Time": self._time_log,
            "Image Feature Pts": self._feature_pts,
            "Virtual Feature Pts": self._virtual_feature_pts,
            "Feature Params": self._img_feature_param,
            "Opt Flow Ang Vel": self._opt_flow_ang_vel_raw,
            "Ring Opt Flow Ang Vel": self._ring_opt_flow_raw,
            "Target Velocity": self._target_vel_log,
            "Angular Velocity": self._imu_angvel_raw,
            "Quaternion": self._quat_log,
            "FPS": self._fps_log
        }
    
    def getParams(self):
        parameter = f"{{'Capture Rate':{self._capRate}, 'resolution':{self._resolution}}}"
        return parameter
    
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

    def getRawRingFlowAngVel(self):
        """Latest raw ring [h;w] BEFORE _sensor_cal_ring (for ring-cal derivation)."""
        return self._ring_v_raw

    def getRingFlowAngVel(self):
        """Calibrated, KF-smoothed ring flow (safety net; control consumes the
        corner flow). _sensor_cal_ring is identity until derived like the corner cal."""
        return self._sensor_cal_ring @ self._kf_x_ring[:, 0]