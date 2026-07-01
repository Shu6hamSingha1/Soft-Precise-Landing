#!/usr/bin/env python3
"""
Record the static CHASE camera (external third-person view) to an mp4 — headless.

Subscribes to the bridged chase-camera image topic and writes frames to
test_data/Test_Videos/chase_<timestamp>.mp4 for RECORD_S seconds, then flushes
and exits cleanly (so the file is valid even if the landing run is still going).

Meant to run alongside a rover landing that has the chase camera bridged (the
`chase_cam` model in rover.sdf; run_rover_landing.sh CHASE_CAM=1 bridges it to
/chase_image and starts this recorder).

Env:
  CHASE_TOPIC   ROS topic to record (default /chase_image).
  RECORD_S      seconds to record (default 75).
  CHASE_FPS     output video fps (default 30, matches the sensor update_rate).
  CHASE_OUT     output path (default test_data/Test_Videos/chase_<ts>.mp4).
"""
import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

TOPIC = os.environ.get("CHASE_TOPIC", "/chase_image")
RECORD_S = float(os.environ.get("RECORD_S", "75"))
FPS = float(os.environ.get("CHASE_FPS", "30"))
_ts = time.strftime("%Y-%m-%d_%H-%M-%S")
OUT = os.environ.get(
    "CHASE_OUT",
    f"/home/shubham/Soft-Precise-Landing/PX4_Gazebo/test_data/Test_Videos/chase_{_ts}.mp4")


class ChaseRecorder(Node):
    def __init__(self):
        super().__init__("chase_recorder")
        self._bridge = CvBridge()
        self._writer = None
        self._n = 0
        os.makedirs(os.path.dirname(OUT), exist_ok=True)
        self.create_subscription(Image, TOPIC, self._cb, 10)
        self._t0 = time.time()
        print(f"[chase] recording {TOPIC} -> {OUT} for {RECORD_S:.0f}s", flush=True)

    def _cb(self, msg):
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            print(f"[chase] convert error: {e}", flush=True)
            return
        if self._writer is None:
            h, w = frame.shape[:2]
            self._writer = cv2.VideoWriter(
                OUT, cv2.VideoWriter_fourcc(*"mp4v"), FPS, (w, h))
            print(f"[chase] first frame {w}x{h}; writer open", flush=True)
        self._writer.write(frame)
        self._n += 1

    def done(self):
        if self._writer is not None:
            self._writer.release()
        print(f"[chase] wrote {self._n} frames -> {OUT}", flush=True)


def main():
    rclpy.init()
    node = ChaseRecorder()
    try:
        while rclpy.ok() and (time.time() - node._t0) < RECORD_S:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        node.done()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
