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
  CHASE_TOPIC     ROS topic to record (default /chase_image).
  RECORD_S        seconds to record ONCE recording starts (default 75).
  CHASE_FPS       output video fps (default 30, matches the sensor update_rate).
  CHASE_OUT       output path (default test_data/Test_Videos/chase_<ts>.mp4).
  CHASE_GATE_FILE if set, do NOT record until this flag file appears (the
                  controller touches it at descent-start, so the chase video
                  starts with the landing — matching the down-cam IMG_RECORD).
                  Frames before the gate are dropped.
  CHASE_GATE_TIMEOUT  max seconds to wait for the gate before giving up (default 120).
"""
import os
import signal
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
GATE_FILE = os.environ.get("CHASE_GATE_FILE", "")
GATE_TIMEOUT = float(os.environ.get("CHASE_GATE_TIMEOUT", "120"))
# CHASE_STOP_FILE: if set, stop recording the instant it appears (landing_test
# touches it at touchdown). This makes the chase video end EXACTLY at touchdown,
# same as the down-cam, so the montage syncs all sources over [start, touchdown].
STOP_FILE = os.environ.get("CHASE_STOP_FILE", "")
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
        self._rec_t0 = None            # set when recording actually starts (gate open)
        os.makedirs(os.path.dirname(OUT), exist_ok=True)
        self.create_subscription(Image, TOPIC, self._cb, 10)
        self._t0 = time.time()
        if GATE_FILE:
            print(f"[chase] armed on {TOPIC}; waiting for gate {GATE_FILE} "
                  f"(then record {RECORD_S:.0f}s)", flush=True)
        else:
            print(f"[chase] recording {TOPIC} -> {OUT} for {RECORD_S:.0f}s", flush=True)

    def gate_open(self):
        return (not GATE_FILE) or os.path.exists(GATE_FILE)

    def _cb(self, msg):
        # Drop frames until the gate opens (descent start), so the video begins
        # with the landing — matching the down-cam IMG_RECORD.
        if not self.gate_open():
            return
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            print(f"[chase] convert error: {e}", flush=True)
            return
        if self._writer is None:
            h, w = frame.shape[:2]
            self._writer = cv2.VideoWriter(
                OUT, cv2.VideoWriter_fourcc(*"mp4v"), FPS, (w, h))
            self._rec_t0 = time.time()
            print(f"[chase] gate open — recording {w}x{h} -> {OUT}", flush=True)
        self._writer.write(frame)
        self._n += 1

    def done(self):
        if self._writer is not None:
            self._writer.release()
        print(f"[chase] wrote {self._n} frames -> {OUT}", flush=True)


def main():
    rclpy.init()
    node = ChaseRecorder()

    # Flush + finalize the mp4 on SIGTERM (the launcher's cleanup kills this
    # process at the end of the run) so the file is always valid.
    def _on_term(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, _on_term)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            if node._rec_t0 is None:
                # Not recording yet — give up if the gate never opens.
                if GATE_FILE and (time.time() - node._t0) > GATE_TIMEOUT:
                    print(f"[chase] gate {GATE_FILE} never opened in "
                          f"{GATE_TIMEOUT:.0f}s — exiting.", flush=True)
                    break
            elif STOP_FILE and os.path.exists(STOP_FILE):
                print("[chase] stop flag seen (touchdown) — ending recording.", flush=True)
                break
            elif (time.time() - node._rec_t0) >= RECORD_S:
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.done()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
