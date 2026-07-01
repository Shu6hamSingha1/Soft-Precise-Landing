# **************************************************************************
# Changed class and node name
# Added time node for clock synchronization with simulator world
# **************************************************************************

#!/usr/bin/env python3
# Import the necessary libraries
import os
import time
from threading import Thread

import cv2 
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from rosgraph_msgs.msg import Clock # Clock message type for simulation time
from geometry_msgs.msg import PoseArray
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
from collections import deque # For popleft()
from rclpy.executors import SingleThreadedExecutor, ExternalShutdownException  # Use a single-threaded executor
import skimage as ski

class GZ_Subscriber(Thread):
    """
    This class subscribes to ROS2 topic.
    """
    def __init__(self, subscriber):
        Thread.__init__(self)
        self._subscriber = subscriber
        self._subscriber_name = self._subscriber.get_name()
        self._executor = SingleThreadedExecutor()  # Create a single-threaded executor
        self._executor.add_node(self._subscriber)
        # self._running = True  # Flag to control the thread loop
        self.start()

    # Calling destructor
    def __del__(self):
        print(f"{self._subscriber_name} deleted...")

    def run(self):
        try:
            self._executor.spin()  # Use the single-threaded executor to spin the node
        except ExternalShutdownException:
            print(f"{self._subscriber_name} executor shut down externally.")
        except Exception as e:
            print(f"Unexpected error in {self._subscriber_name}: {e}")
        finally:
            print(f'{self._subscriber_name} closed')

    def close(self):
        """
        Gracefully stop the thread and shutdown the executor.
        """
        # self._running = False
        self._executor.shutdown()  # Shutdown the executor
        self._subscriber.destroy_node()

class PoseData:
    """Simple class to hold UAV and target pose data"""
    def __init__(self):
        self.UAV = None
        self.target = None

class Pose_Node(Node):
    """
    This ROS2 node subscribes to the ground truth value of the pose of the UAV and target models in gazebo world.
    """
    def __init__(self):
        # Initiate the Node class's constructor and give it a name
        super().__init__('model_pose_subscriber')
    
        # Create the subscriber. This subscriber will receive an Image
        # from the video_frames topic. The queue size is 10 messages.
        self.subscription = self.create_subscription(
            PoseArray,
            '/pose',
            self.pose_callback,
            10)
        self.subscription  # prevent unused variable warning

        self._pose = PoseData()  # Initialize the pose data object

        # PoseArray index of the UAV / target top-level model poses. Both worlds
        # bridge the FULL /world/<world>/pose/info, whose PoseArray order is
        # stable: [ground_plane, <target>, <UAV>, ...links...] -> UAV=2, target=1.
        #   - stationary `aruco`: [ground_plane, aruco_marker, x500...]
        #   - moving `rover`    : [ground_plane, rover_aruco_1, x500...]
        # (Do NOT bridge dynamic_pose/info — it only carries entities that moved
        # that step, so its membership/order varies frame-to-frame.)
        # Env-overridable via POSE_IDX_UAV / POSE_IDX_TARGET for other layouts.
        # Inspect ordering with: gz topic -t /world/<world>/pose/info -e
        self._uav_idx = int(os.environ.get("POSE_IDX_UAV", "2"))
        self._target_idx = int(os.environ.get("POSE_IDX_TARGET", "1"))

    def pose_callback(self, data):
        try:
            self._pose.UAV = data.poses[self._uav_idx]
            self._pose.target = data.poses[self._target_idx]
            # Display the message on the console
            # self.get_logger().info(f"Model Pose: {self._pose}")
            # time.sleep(0.01)  # Sleep for a short time to avoid high CPU usage
        
        except KeyboardInterrupt:
            print("KeyboardInterrupt: Pose Subscriber\n")

        except RuntimeError as e:
            print("RuntimeError: Pose Subscriber: {e}\n")

        except SyntaxError as e:
            print("SyntaxError: Pose Subscriber: {e}\n")

        except Exception as e:
            print(f"Unexpected error: Pose Subscriber: {e}\n")

    def getPose(self):
        """
        Returns the pose of the UAV and target models in the world.
        """
        return self._pose

class Image_Node(Node):
    """
    This ROS2 node subscribes to the image from the model in gazebo world.
    """
    def __init__(self, time_keeper=time, controller=None):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give it a name
        super().__init__('image_subscriber')

        # Create the subscriber. This subscriber will receive an Image
        # from the video_frames topic. The queue size is 10 messages.
        self.subscription = self.create_subscription(
            Image,
            '/image',
            self.image_callback,
            10)
        self.subscription  # prevent unused variable warning
      
        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

        # Flight controller data
        self._FC = controller

        self._time_keeper = time_keeper

        # Initialize variables
        self._count = 0
        self._img_deque = deque([None, None])
        self._quat_deque = deque([None, None])
        self._angvel_deque = deque([None, None])   # IMU body rate (FRD), synced to the flow pair
        self._start_time = self._time_keeper.perf_counter()
        self._t0 = self._start_time
        self._t1 = self._start_time
        self._meanTimePerImage = None
        self._res = None

    def image_callback(self, msg):
        """
        Callback function.
        """
        try:
            # Display the message on the console
            # self.get_logger().info('Receiving video frame')
            while self._res is None:
                self._res = (msg.height, msg.width)
                # print(self._res)
                
                # time.sleep(0.01)
                if self._time_keeper.perf_counter() - self._start_time > 1.0:
                    raise Exception("Image resolution not received yet. Waiting...")
            
            # FPS from the image STAMP (sim-time CAPTURE time) = the true frame interval, jitter-free.
            # perf_counter() returns the /clock value AT THE CALLBACK, quantized to the ~250 Hz sim
            # clock -> the interval snaps to 4 ms steps -> fps jittered (62/83/125 Hz). msg.header.stamp
            # is the camera's own capture instant, which is regular. (2026-06-05)
            self._t0 = self._t1
            self._t1 = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

            if self._t0 != self._t1:
                self._fps = 1/(self._t1 - self._t0)

                # Convert ROS Image message to OpenCV image
                frame = self.br.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                quat = self._FC.getQuat()
                angvel = self._FC.getAngVelIMU()   # IMU body rate (FRD), same instant as quat

                # Default 90° CW rotation has been in place since the project's first
                # commit (no documented reason — likely matches a legacy camera mount).
                # Set GZ_NO_IMAGE_ROTATION=1 to disable: the lstsq's "camera-x" will
                # then align with body-x, raw axes match GT body-frame axes, and a
                # diagonal _sensor_cal_hw can hit ratio=1.0 per axis. WARNING: this
                # invalidates the controller's gain tuning (currently tuned in the
                # rotated frame); use only for calibration investigation.
                if os.environ.get("GZ_NO_IMAGE_ROTATION", "0") == "1":
                    rot_frame = frame
                else:
                    rot_frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)

                # filtered_img = ski.filters.gaussian(frame, sigma=3)

                # Append the decoded image to the deque
                self._img_deque.append(rot_frame)
                self._quat_deque.append(quat)
                self._angvel_deque.append(angvel)
                self._count = self._count + 1
                # FIX (2026-06-05): do NOT re-update t0/t1 here. That corrupted the next
                # frame's fps (line ~166) into 1/(idle gap) = 1/(frame_interval - processing_time),
                # which over-read (~84 vs 62 Hz) and jittered (28-250 Hz) with the processing time.
                # Leaving t0/t1 as the consecutive callback-fire SIM times (from /clock) makes
                # fps = 1/(true frame interval). meanTime uses a fresh sim-time read instead.
                self._meanTimePerImage = (self._time_keeper.perf_counter() - self._start_time) / self._count

                self._img_deque.popleft()
                self._quat_deque.popleft()
                self._angvel_deque.popleft()
                # time.sleep(0.01)  # Sleep for a short time to avoid high CPU usage
        
        except KeyboardInterrupt:
            print("KeyboardInterrupt: Image Streamer Thread\n")

        except RuntimeError as e:
            print("RuntimeError: Image Streamer Thread: {e}\n")

        except SyntaxError as e:
            print("SyntaxError: Image Streamer Thread: {e}\n")

        except Exception as e:
            print(f"Unexpected error: Image Streamer Thread: {e}\n")

    def getImages(self):
        return list(self._img_deque)

    def getQuaternions(self):
        return list(self._quat_deque)

    def getAngVels(self):
        """IMU body rate (FRD) paired with the flow images (frame0, frame1)."""
        return list(self._angvel_deque)

    def getFPS(self):
        # return 1/self._meanTimePerImage
        return self._fps

    def getStamp(self):
        # latest image CAPTURE stamp (msg.header.stamp, sim time); for clock diagnostics
        return self._t1

    def getImgResolution(self):
        if self._res is None:
            start_time = time.perf_counter()
            while self._res is None:
                if time.perf_counter() - start_time > 5.0:
                    raise Exception("Image resolution not received yet. Waiting...")
                time.sleep(0.01)
        return self._res

class Clock_Node(Node):
    """
    This class subscribes to the ground truth value of the clock of the simulator world.
    """
    def __init__(self):
        # Initiate the Node class's constructor and give it a name
        super().__init__('model_clock_subscriber')

        # Create the subscriber. This subscriber will receive an Image
        # from the video_frames topic. The queue size is 10 messages.
        self.subscription = self.create_subscription(
            Clock,
            '/clock',
            self.clock_callback,
            10)
        self.subscription  # prevent unused variable warning

        self._time = None

    def clock_callback(self, data):
        try:
            self._time = data.clock.sec + data.clock.nanosec * 1e-9  # Convert to seconds
            # Display the message on the console
            # self.get_logger().info(f"Model Time: {self._time}")
            # time.sleep(0.01)  # Sleep for a short time to avoid high CPU usage
        
        except KeyboardInterrupt:
            print("KeyboardInterrupt: Clock Subscriber\n")

        except RuntimeError as e:
            print("RuntimeError: Clock Subscriber: {e}\n")

        except SyntaxError as e:
            print("SyntaxError: Clock Subscriber: {e}\n")

        except Exception as e:
            print(f"Unexpected error: Clock Subscriber: {e}\n")

    def perf_counter(self):
        """
        Returns the time of the simulator world.
        """
        return self._time