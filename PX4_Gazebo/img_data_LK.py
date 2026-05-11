# **************************************************************************
# Changed class and node name used in gz_subscriber
# Used simulator time instead of system time
# Used virtual image feature points for optical flow calculation
# Detect nested Aruco markers of different IDs and select marker with smallest ID.
# Added more pixel points and used Lucas-Kanade method for optical flow calculation.
# **************************************************************************

"""
Code to compute real-time optical flow
    - https://ieeexplore.ieee.org/abstract/document/8753669/
    - https://docs.opencv.org/3.4/d4/dee/tutorial_optical_flow.html
"""
import numpy as np
import cv2 # OpenCV library
from threading import Thread 
import time
from scipy.signal import savgol_filter as sgf
from ahrs import Quaternion
from numerical_methods import extrapolate

from gz_subscriber import GZ_Subscriber, Image_Node

CHECK_NUM = 80
fx = 540
fy = 540

FILTER_WIN = 21
VIDEO = True

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

        self._sensor_cal_hw = np.diag([1, 1, 1, 1, 1, 1]) # Sensor calibration matrix
        self._sensor_cal_s = np.diag([1/4, 1/4, 1, 1]) # Sensor calibration matrix

        # ArUco marker detection setup
        _arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        _arucoParams = cv2.aruco.DetectorParameters()
        self._detector = cv2.aruco.ArucoDetector(_arucoDict, _arucoParams)
        
        # Parameters for Lucas Kanade algorithm
        self._lk_params = dict(
            winSize = (17, 17), maxLevel = 3, criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 20, 0.03)
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
        self._quats = []
        self._img_feature_param = []
        self._opt_flow_ang_vel_raw = []

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
                quaternions = self._image_node.getQuaternions()
                self._fps = self._image_node.getFPS()   
                # print(f"Image FPS: {self._fps}")

                # Check if at least 2 frames of images have been received
                if images[0] is not None and images[1] is not None:              
                    # if VIDEO:
                    #     # Resize display image
                    #     resized_img = cv2.resize(images[0], None, fx=4, fy=4, interpolation=cv2.INTER_AREA)
                    #     cv2.imshow('Image Streamer', resized_img)
                    #     if cv2.waitKey(1) == 27:
                    #         self.close()

                    # Uncomment the following code to record video/images
                    if self.RECORD and self.CONTROLLER_READY:
                        if self._video is None:
                            # Below VideoWriter object will store video in 'timestamp.avi' file. 
                            self.timestamp = time.ctime().replace(':', '-')
                            self._video = cv2.VideoWriter(f'/home/shubham/ws/Test_Data/Test_Videos/{self.timestamp}.mp4',  
                                    cv2.VideoWriter_fourcc(*'mp4v'), 
                                    self._capRate, self._resolution)

                        self._video.write(images[1])

                    gray_images = [cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) for img in images]

                    # Calculate the radial optical flow if it is AVAILABLE. Else the loop is restarted.
                    if self._imgProcess(gray_images, quaternions, showVideo = VIDEO) is AVAILABLE:
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

                else:
                    print("Waiting to receive at least 2 frames")

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
            time.sleep(1/100)
            if (self._time.perf_counter() - start_time) > 20:
                raise Exception("Unable to get image data.")
    
    def metrics(self):
        return {
            'fps': self._fps, 'img_process_freq':1/self._calc_time
        }
    
    def _get_scaled_quadrilaterals(self, pts):
        """
        Compute scaled versions of the quadrilateral.
        pts: shape (4,2) - the four corner points.
        Returns a list of 3 arrays, each (4,2): original, 2/3 scale, 1/3 scale.
        """
        centroid = np.mean(pts, axis=0)
        scales = [1.0, 2/3, 1/3]
        scaled_quads = []
        for s in scales:
            scaled_pts = centroid + s * (pts - centroid)
            scaled_quads.append(scaled_pts)
        return scaled_quads

    def _get_side_points(self, quad):
        """
        Divide each side of the quadrilateral into 15 discrete points.
        quad: shape (4,2)
        Returns array of shape (60,2) - 15 points per side * 4 sides.
        """
        points = []
        for i in range(4):
            A = quad[i]
            B = quad[(i+1) % 4]
            t_vals = np.linspace(0, 1, 15)
            side_points = np.outer(1 - t_vals, A) + np.outer(t_vals, B)
            points.append(side_points)
        return np.vstack(points)

    def _get_all_feature_points(self, pts):
        """
        Get all feature points: side points for the three scaled quadrilaterals.
        pts: shape (4,2)
        Returns array of shape (180,1,2) with dtype=np.float32, ready for Lucas-Kanade.
        """
        scaled_quads = self._get_scaled_quadrilaterals(pts)
        all_points = []
        for quad in scaled_quads:
            side_pts = self._get_side_points(quad)
            all_points.append(side_pts)
        all_points = np.vstack(all_points)
        # Reshape to (n,1,2) and ensure float32 for Lucas-Kanade
        return np.array(all_points, dtype=np.float32).reshape(-1, 1, 2)

    def _imgProcess(self, imgs, quats, showVideo = False):
        # This function will return True if the optical flow is AVAILABLE and calculate the optical flow. Else, it will return False.
        # Return type is a Boolean
        # Detect markers for both images
        size_factor = 1.0
        results = self._detector.detectMarkers(imgs[0])

        FEATURE_DATA_IS_LOGGED = False

        # Check if feature detection was successful
        if results[0]:# Ensure that at least one marker ID is detected in the first frame
            if min(results[1]) == 0:
                size_factor = 1/10.0
            corner_pts = results[0][np.argmin(results[1])][0]  # Select marker with smallest ID

            # Compute all feature points for optical flow
            feature_pts_0 = self._get_all_feature_points(corner_pts)

            # For successful run of cv2.calcOpticalFlowPyrLK() function, here's what you need to know.
            # 1. make sure the images are grayscale.
            # 2. your coordinate parameter that is i_old_pts should be single precision float meaning float32.
            # 3. the coordinate parameter i_old_pts(from your program) should be a numpy array with the dimension (n,1,2) where n represents the number of points.
            # Link: https://stackoverflow.com/questions/34540181/opencv-optical-flow-assertion
            feature_pts_1, status, _ = cv2.calcOpticalFlowPyrLK(
                imgs[0], imgs[1], np.array(feature_pts_0), None, **self._lk_params
            )

            # Condition to check flow status for the specified points in the image (Refer to the structure of dataset.)
            if len(status[status==1]) == 0:
                print("Unable to find optical flow at specified points.")
                time.sleep(1/60)

            else:
                C_nP = [pts[status==1] for pts in [feature_pts_0, feature_pts_1]]
                
                V_nP_norm = [self._getVirtualPts(p, a) for p, a in zip(C_nP, quats)]

                # Shows image with optical flow
                if showVideo:
                    self._showOptFlow(imgs[1], C_nP, V_nP_norm)
                
                # Compute optical flow
                A = self._fill_A(V_nP_norm[1])

                Y = np.reshape(V_nP_norm[1] - V_nP_norm[0], (-1,)) * self._fps

                B_v = np.linalg.lstsq(A, Y, rcond=None)[0]
                self._opt_flow_ang_vel_raw.append(size_factor * B_v)

                self._getImgFeatures(size_factor * V_nP_norm[1])

                if not self.FEATURE_IS_VISIBLE:
                    print("LANDING PAD VISIBLE NOW...")
                    self.FEATURE_IS_VISIBLE  = True 
                if self._count_check_img_feature > 0:
                    self._count_check_img_feature = 0

                # Log the updated data
                self._feature_pts.append(C_nP)
                self._virtual_feature_pts.append(V_nP_norm)
                FEATURE_DATA_IS_LOGGED = True

        elif self.FEATURE_IS_VISIBLE:
            self._count_check_img_feature +=1
            if self._count_check_img_feature > CHECK_NUM:
                print("LANDING PAD NOT VISIBLE...")
                self._count_check_img_feature = 0
                # Swap flag value to initiate necessary action to get the image feature in the field of view of the camera.
                self.FEATURE_IS_VISIBLE  = False

        if not FEATURE_DATA_IS_LOGGED:
            # Extrapolate the output data
            extrapolated_opt_flow_ang_vel_raw = extrapolate(self._time_log, self._opt_flow_ang_vel_raw, n=4, deg=1, default_shape=6)
            extrapolated_img_feature_param = extrapolate(self._time_log, self._img_feature_param, n=4, deg=1, default_shape=4)   

            # Log the extrapolated data
            self._opt_flow_ang_vel_raw.append(extrapolated_opt_flow_ang_vel_raw)
            self._img_feature_param.append(extrapolated_img_feature_param)

            # Log the previous data
            self._feature_pts.append(self._feature_pts[-1] if self._feature_pts else np.zeros((2,4,2)))
            self._virtual_feature_pts.append(self._virtual_feature_pts[-1] if self._virtual_feature_pts else np.zeros((2,4,2)))

        # Log the recorded data
        self._time_log.append(self._time.perf_counter())
        self._quats.append(quats)
        self._fps_log.append(self._fps)

        # if len(self._time_log) != len(self._feature_pts):
        #     print("Here")

        return FEATURE_DATA_IS_LOGGED
    
    def _fill_A(self, centered_pts):
        """
        centered_pts: shape (4,2)
        """
        x = centered_pts[:, 0]
        y = centered_pts[:, 1]

        A = np.zeros((len(x)*2, 6))

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
        # Extract roll, pitch (yaw does not affect virtual camera)
        R = Quaternion([quat.w, quat.x, quat.y, quat.z]).to_DCM()
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
        x = (pts[:,0] - cx) / fx
        y = (pts[:,1] - cy) / fy

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
        V_pts = [(pts * self.focal + self.center).astype(np.int32) for pts in V_nP_norm]

        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        
        # 3. Draw real flow (RED)
        for old, new in zip(C_pts[0], C_pts[1]):
            img = cv2.arrowedLine(img, old, new, (0,0,255), 1)

        # 4. Draw real marker polygon (GREEN)
        # cv2.polylines(img, [C_pts[1]], isClosed=True, color=(0,255,0), thickness=2)

        # 5. Draw virtual flow (GREEN)
        for old, new in zip(V_pts[0], V_pts[1]):
            img = cv2.arrowedLine(img, old, new, (0,255,0), 1)

        # 6. Draw virtual marker polygon (RED)
        # cv2.polylines(img, [V_pts[1]], isClosed=True, color=(0,0,255), thickness=2)

        # Resize display image
        resized_img = cv2.resize(img, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
        cv2.imshow('Image Streamer', resized_img)
        if cv2.waitKey(1) == 27:
            self.close()


    def getLogData(self):
        return {
            "Time": self._time_log,
            "Image Feature Pts": self._feature_pts,
            "Virtual Feature Pts": self._virtual_feature_pts,
            "Feature Params": self._img_feature_param,
            "Opt Flow Ang Vel": self._opt_flow_ang_vel_raw,
            "FPS": self._fps_log
        }
    
    def getParams(self):
        parameter = f"{{'Capture Rate':{self._capRate}, 'resolution':{self._resolution}}}"
        return parameter
    
    def getImgFeatureParam(self):
        if len(self._img_feature_param) == 0:
            return np.zeros(4)
        return self._sensor_cal_s @ self._img_feature_param[-1]
    
    def getOptFlowAngVel(self):
        if len(self._opt_flow_ang_vel_raw) == 0:
            return np.zeros(6)
    
        # Smooth the estimates of Optic Flow and Angular Velocity using Savitzky-Golay filter
        if len(self._opt_flow_ang_vel_raw) >= FILTER_WIN:
            opt_flow_ang_vel_sgf = sgf(self._opt_flow_ang_vel_raw[-FILTER_WIN:], FILTER_WIN, 2, axis=0)
            opt_flow_ang_vel = self._sensor_cal_hw @ opt_flow_ang_vel_sgf[int(FILTER_WIN/2 + 1)]
        else:
            opt_flow_ang_vel = np.mean(self._opt_flow_ang_vel_raw, axis=0)

        return opt_flow_ang_vel