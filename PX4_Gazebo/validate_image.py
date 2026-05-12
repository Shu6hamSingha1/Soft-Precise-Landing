#!/usr/bin/env python3
"""
Subscribe to /image, capture N frames, report dimensions / encoding / stats,
save a sample frame, and try ArUco detection on it.

Run AFTER `measure_image_fps.sh`-style infrastructure is up (or stand-alone
via validate_image_feed.sh which wraps the SITL bring-up).
"""
import sys
import time
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class ImageValidator(Node):
    def __init__(self, target_frames=10):
        super().__init__('image_validator')
        self._bridge = CvBridge()
        self._target = target_frames
        self.frames = []
        self.sub = self.create_subscription(Image, '/image', self._cb, 10)

    def _cb(self, msg):
        if len(self.frames) >= self._target:
            return
        try:
            img = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.frames.append({'error': str(e)})
            return
        self.frames.append({
            'shape': img.shape,
            'encoding': msg.encoding,
            'mean': float(img.mean()),
            'std': float(img.std()),
            'min': int(img.min()),
            'max': int(img.max()),
            'img': img,
        })


def main():
    out_dir = "/home/shubham/Soft-Precise-Landing/PX4_Gazebo/run_logs"
    rclpy.init()
    node = ImageValidator(target_frames=10)
    print("[validate] waiting for /image (up to 20s, target=10 frames) ...")

    start = time.time()
    while len(node.frames) < node._target and (time.time() - start) < 20:
        rclpy.spin_once(node, timeout_sec=0.2)

    if not node.frames:
        print("[validate] ERROR: no frames received on /image in 20s")
        rclpy.shutdown()
        return 1

    print(f"[validate] received {len(node.frames)} frames\n")
    print("  idx | shape           | encoding | mean   std    min  max")
    print("  --- | --------------- | -------- | ------ ------ ---- ----")
    for i, f in enumerate(node.frames):
        if 'error' in f:
            print(f"  {i:3d} | ERROR: {f['error']}")
            continue
        print(f"  {i:3d} | {str(f['shape']):15s} | {f['encoding']:8s} | "
              f"{f['mean']:6.2f} {f['std']:6.2f} {f['min']:4d} {f['max']:4d}")

    # ArUco detection on the most recent frame
    good = [f for f in node.frames if 'img' in f]
    if not good:
        print("[validate] no decodable frames")
        rclpy.shutdown()
        return 2

    mid = good[len(good) // 2]
    sample = mid['img']
    sample_path = f"{out_dir}/validate_sample.png"
    cv2.imwrite(sample_path, sample)
    print(f"\n[validate] saved sample frame -> {sample_path}")

    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    aruco_params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
    gray = cv2.cvtColor(sample, cv2.COLOR_BGR2GRAY)
    corners, ids, rejected = detector.detectMarkers(gray)

    print(f"\n[validate] ArUco detection on sample frame:")
    if ids is not None and len(ids) > 0:
        for i, c in zip(ids.flatten(), corners):
            c_int = c[0].astype(int)
            cx, cy = c_int.mean(axis=0).astype(int)
            print(f"  marker id={int(i):3d}  centroid=({cx}, {cy})  corners={c_int.tolist()}")
        annotated = sample.copy()
        cv2.aruco.drawDetectedMarkers(annotated, corners, ids)
        ann_path = f"{out_dir}/validate_sample_annotated.png"
        cv2.imwrite(ann_path, annotated)
        print(f"  annotated frame -> {ann_path}")
        print(f"  ✓ ArUco feed is valid and detectable")
    else:
        rej_n = len(rejected) if rejected is not None else 0
        print(f"  no markers detected (rejected candidates: {rej_n})")
        print(f"  ⚠ verify the drone is positioned above an ArUco target in the world")

    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
