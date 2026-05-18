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

FILTER_WIN = 13       # sliding-window length for savgol on raw image-side measurements
                      # Retuned 2026-05-12 via tune_savgol.py across 5 calibration
                      # recordings × 8 channels. Best runtime mean|corr| was (13, 1),
                      # vs legacy (51, 2) which actually HURT runtime correlation
                      # because ~25-sample lag pulled the centroid out of phase
                      # with ground truth.
FILTER_POLYORDER = 1
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
        self.center = np.array(self._resolution)/2     # Here radius is considered zero for the center

        # Sensor calibration matrices — RECALIBRATED 2026-05-13 (3rd iteration).
        # Method: std-ratio (RMA / geometric-mean regression) across 9 valid
        # post-fix runs:  α = σ(GT) / σ(filtered_raw)
        #
        # Why std-ratio over median(gt/raw) and over LS-optimal?
        #   - median(gt/raw) is unstable per-run because raw magnitudes vary 10×
        #     between runs (lstsq SNR is low). 9-run inter-run CV was 50-150%.
        #   - LS-optimal α = E[raw·gt]/E[raw²] is biased toward zero when corr<1
        #     (raw has noise that gt doesn't), shrinking calibrated output below
        #     GT amplitude. Verified empirically: LS-derived α gave RMSE = 95% of
        #     GT RMS (no better than predicting zero).
        #   - Std-ratio matches amplitudes: σ(α·filt) = σ(GT) by construction, so
        #     calibrated and GT plots overlay in envelope. Per-sample correlation
        #     (0.2-0.5 per axis) bounds how tightly individual peaks align — that
        #     ceiling is set by raw signal SNR, not the cal factor.
        #
        # Previous iterations (kept for reference):
        #   2026-05-12 median (4 runs):
        #     _sensor_cal_hw = np.diag([0.1518, 0.1777, 0.0651, 0.2083, 0.2209, 0.2435])
        #   2026-05-13a 4-run nanmedian:
        #     _sensor_cal_hw = np.diag([0.1498, 0.1694, 0.0877, 0.2188, 0.2114, 0.4236])
        #   2026-05-13b 9-run median(gt/raw) (NEW/OLD RMSE within ±3%):
        #     _sensor_cal_hw = np.diag([0.1972, 0.1764, 0.0257, 0.1801, 0.2139, 0.1998])
        # All 6 axes use the 9-run median(gt/raw) cal (2026-05-13b). The earlier
        # HYBRID variant amplified ω_z by 1.9788× to match the deliberate-yaw
        # output_calibration-sweep amplitude. But in the landing scenario the
        # actual yaw rate is near zero (PX4 truth |ω_yaw| < 0.43 rad/s during
        # the 5m → 0m descent), while the lstsq optic-flow solve produces a
        # consistent +0.8 rad/s bias on ω_z (probably leakage from the marker-
        # corner divergence pattern as the drone approaches). Amplifying that
        # bias by 1.98× gave w_i[z] median +1.56 rad/s, which doubly poisoned
        # the SMC via the V_h_d cross-coupling term and the c-term — partially
        # responsible for the 8× faster-than-MATLAB descent rate. Reverting to
        # 0.1998 reduces the bias contamination 10× and matches the policy used
        # on the other 5 channels. Centroid s uses median(gt/raw) (stable, 0.58).
        self._sensor_cal_hw = np.diag([0.1972, 0.1764, 0.0257, 0.1801, 0.2139, 0.1998])
        self._sensor_cal_s  = np.diag([0.5830, 0.6104, 1.0000, 1.0000])

        # ArUco marker detection setup, with sub-pixel corner refinement
        # (added 2026-05-13). Default cornerRefinementMethod is CORNER_REFINE_NONE
        # which returns integer-rounded pixel corners. SUBPIX runs the standard
        # cv2.cornerSubPix algorithm internally on each detected corner, giving
        # sub-pixel precision before LK tracks them. Cheap and improves the
        # downstream lstsq input quality.
        _arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        _arucoParams = cv2.aruco.DetectorParameters()
        _arucoParams.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        _arucoParams.cornerRefinementWinSize = 5       # 11x11 actual window
        _arucoParams.cornerRefinementMaxIterations = 30
        _arucoParams.cornerRefinementMinAccuracy = 0.01
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

        # Flags and counters
        self._STAY_OPEN = True
        self.FEATURE_IS_VISIBLE = False
        self._count_check_img_feature = CHECK_NUM
        self._count_check_opt_flow = CHECK_NUM

        # Data storage
        self._time_log = []
        self._fps_log = []
        self._feature_pts = []
        self._virtual_feature_pts = []
        self._A = np.zeros((8, 6))  # 4 points → 8 rows
        self._quats = []
        self._img_feature_param = []
        self._opt_flow_ang_vel_raw = []
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
                            self._video = cv2.VideoWriter(f'/home/shubham/ws/Test_Data/Test_Videos/{self.timestamp}.mp4',  
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

        # Check if feature detection was successful
        if results[0]:# Ensure that at least one marker ID is detected in the first frame
            corner_pts_0 = results[0][np.argmin(results[1])][0]  # Select marker with smallest ID

            # For successful run of cv2.calcOpticalFlowPyrLK() function, here's what you need to know.
            # 1. make sure the images are grayscale.
            # 2. your coordinate parameter that is i_old_pts should be single precision float meaning float32.
            # 3. the coordinate parameter i_old_pts(from your program) should be a numpy array with the dimension (n,1,2) where n represents the number of points.
            # Link: https://stackoverflow.com/questions/34540181/opencv-optical-flow-assertion
            corner_pts_1, status, _ = cv2.calcOpticalFlowPyrLK(
                imgs[0], imgs[1], np.array(corner_pts_0), None, **self._lk_params
            )

            if len(status[status==1]) == len(corner_pts_0):
                FEATURE_DATA_IS_LOGGED = True
                C_nP = [pts for pts in [corner_pts_0, corner_pts_1]]

            if min(results[1]) == 0:
                size_factor = 1/1.0

            if FEATURE_DATA_IS_LOGGED:
                V_nP_norm = [self._getVirtualPts(p, a) for p, a in zip(C_nP, quats)]

                # Shows image with optical flow
                if showVideo:
                    self._showOptFlow(imgs[1], C_nP, V_nP_norm)
                
                # Compute optical flow via 6-DOF image-Jacobian lstsq.
                # Notes from 2026-05-13 experiments (kept lstsq, rejected
                # robust-regression alternatives for this specific 8x6 algebra):
                #   - IRLS/Huber on residuals: NO-OP. Residual nullspace is 2D
                #     with equal projection onto every corner pair, so per-corner
                #     residual norms are structurally equal regardless of which
                #     corner is bad.
                #   - LOO by held-out residual norm: same symmetry, NO-OP.
                #   - LK-`err` row weighting: HURT (cap-hit% rose 0.5→1.9%);
                #     downweighting pushes the system toward 6x6 exactly
                #     determined, which is more sensitive to remaining noise.
                # Noise reduction is now handled temporally (KF in getOptFlowAngVel),
                # not via the lstsq solve itself.
                # Stability gates retained: rcond=1e-3, rank, condition, ±10 clip.
                A = self._fill_A(V_nP_norm[1])
                Y = np.reshape(V_nP_norm[1] - V_nP_norm[0], (-1,)) * self._fps

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
                # 2-state KF update — only on a fresh raw measurement.
                self._kf_update(B_v_scaled, self._time.perf_counter())
                # Log both filters' calibrated outputs every frame for A/B.
                self._opt_flow_kf_log.append(self._sensor_cal_hw @ self._kf_x[:, 0])
                self._opt_flow_savgol_log.append(self._sensor_cal_hw @ self._compute_savgol_output())

                self._getImgFeatures(size_factor * V_nP_norm[1])

                if not self.FEATURE_IS_VISIBLE:
                    print("LANDING PAD VISIBLE NOW...")
                    self.FEATURE_IS_VISIBLE  = True 
                if self._count_check_img_feature > 0:
                    self._count_check_img_feature = 0

                # Log the updated data
                self._feature_pts.append(C_nP)
                self._virtual_feature_pts.append(V_nP_norm)

        elif self.FEATURE_IS_VISIBLE:
            self._count_check_img_feature +=1
            if self._count_check_img_feature > CHECK_NUM:
                print("LANDING PAD NOT VISIBLE...")
                self._count_check_img_feature = 0
                # Swap flag value to initiate necessary action to get the image feature in the field of view of the camera.
                self.FEATURE_IS_VISIBLE  = False

        # Log the recorded data
        self._time_log.append(self._time.perf_counter())
        self._quats.append(quats)
        self._fps_log.append(self._fps)

        if not FEATURE_DATA_IS_LOGGED:
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
            extrapolated_opt_flow_ang_vel_raw = np.zeros(6)

            extrapolated_img_feature_param = extrapolate(
                self._time_log, self._img_feature_param, n=4, deg=1, default_shape=4)
            extrapolated_img_feature_param = np.nan_to_num(
                np.asarray(extrapolated_img_feature_param), nan=0.0, posinf=5.0, neginf=-5.0)
            extrapolated_img_feature_param = np.clip(extrapolated_img_feature_param, -5.0, 5.0)

            self._opt_flow_ang_vel_raw.append(extrapolated_opt_flow_ang_vel_raw)
            self._img_feature_param.append(extrapolated_img_feature_param)

            # Log the previous data
            self._feature_pts.append(self._feature_pts[-1] if self._feature_pts else np.zeros((2,4,2)))
            self._virtual_feature_pts.append(self._virtual_feature_pts[-1] if self._virtual_feature_pts else np.zeros((2,4,2)))

        return FEATURE_DATA_IS_LOGGED
    
    def _fill_A(self, centered_pts):
        """
        centered_pts: shape (4,2)
        """
        x = centered_pts[:, 0]
        y = centered_pts[:, 1]

        A = self._A  # alias for speed

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

    def _getImgFeatures(self, pts):
        """
        pts : virtual feature points in normalized frame
            shape (N,2), already (x,y) = (u-cx)/fx, (v-cy)/fy
        """

        # ---- 1. Zeroth & first-order moments ----
        x = pts[:,0]
        y = pts[:,1]

        m00 = np.sum(np.ones_like(x))          # = N points (area surrogate)
        m10 = np.sum(x)
        m01 = np.sum(y)

        # ---- 2. Centroid in normalized frame ----
        xc = m10 / m00
        yc = m01 / m00
        Pc = np.array([xc, yc])

        # ---- 3. Centered coordinates ----
        Xc = x - xc
        Yc = y - yc

        # ---- 4. Second-order centered moments ----
        mu20 = np.sum(Xc**2)
        mu02 = np.sum(Yc**2)
        mu11 = np.sum(Xc * Yc)

        # ---- 5. Orientation of marker ----
        if abs(mu11) < 1e-6:
            alpha = 0.0
        else:
            alpha = 0.5 * np.arctan2(2 * mu11, (mu20 - mu02))

        # ---- 6. Feature vector (unnormalized) ----
        s = np.array([xc, yc, 1.0, alpha])

        self._img_feature_param.append(s)

    def _getVirtualPts(self, pts, quat):
        """Reproject camera-frame pixels onto the virtual image plane V.

        V is the MATLAB `I_R_V = rotz(yaw)` frame: a LEVEL frame
        (gravity-aligned z) that preserves the UAV's yaw heading. Roll and
        pitch are removed by aligning V's z-axis with world-down; yaw is
        carried over via the camera-y axis used to construct V's x-axis.
        """
        R = Quaternion([quat.w, quat.x, quat.y, quat.z]).to_DCM()
        # World-down expressed in the camera frame.
        g = R @ np.array([0, 0, 1])

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
        """Optical-flow + angular-velocity vector BEFORE _sensor_cal_hw. Used by output_calibration."""
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
        if len(self._img_feature_param) >= FILTER_WIN:
            sgf_buf = sgf(self._img_feature_param[-FILTER_WIN:],
                          FILTER_WIN, FILTER_POLYORDER, axis=0)
            return self._sensor_cal_s @ sgf_buf[int(FILTER_WIN / 2 + 1)]
        return self._sensor_cal_s @ np.mean(self._img_feature_param, axis=0)