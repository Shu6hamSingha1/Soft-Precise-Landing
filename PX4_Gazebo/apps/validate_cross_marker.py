#!/usr/bin/env python3
"""
Subscribe to /image, capture N frames, report dimensions / encoding / stats,
save a sample frame, and run the standalone cross-marker detector
(src/cross_marker_detector.py) against it. Live-feed counterpart to the
offline synthetic-texture test -- validates the detector against real
Gazebo rendering (lighting, camera noise, resolution) it wasn't tuned on.

Run via: WORLD=cross_marker VALIDATE_SCRIPT=apps/validate_cross_marker.py \
    bash scripts/validate_image_feed.sh
"""
import os
import sys
import time

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
import cross_marker_detector as cmd


class ImageValidator(Node):
    def __init__(self, target_frames=10):
        super().__init__('cross_marker_image_validator')
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
        self.frames.append({'shape': img.shape, 'encoding': msg.encoding, 'img': img})


def main():
    out_dir = "/home/shubham/Soft-Precise-Landing/PX4_Gazebo/run_logs"
    rclpy.init()
    node = ImageValidator(target_frames=15)
    print("[validate] waiting for /image (up to 20s, target=15 frames) ...")

    start = time.time()
    while len(node.frames) < node._target and (time.time() - start) < 20:
        rclpy.spin_once(node, timeout_sec=0.2)

    if not node.frames:
        print("[validate] ERROR: no frames received on /image in 20s")
        rclpy.shutdown()
        return 1

    good = [f for f in node.frames if 'img' in f]
    print(f"[validate] received {len(node.frames)} frames ({len(good)} decodable)\n")
    if not good:
        rclpy.shutdown()
        return 2

    # run the detector across ALL captured frames, not just one sample --
    # a single frame can't tell us about frame-to-frame jitter/stability
    print("  idx | shape           | ok    | center            | heading_deg")
    print("  --- | --------------- | ----- | ------------------ | -----------")
    centers = []
    for i, f in enumerate(good):
        det = cmd.detect(f['img'])
        row_center = f"({det.center[0]:7.1f},{det.center[1]:7.1f})" if det.center else "  n/a"
        row_heading = f"{det.heading_deg:6.1f}" if det.heading_deg is not None else "  n/a"
        print(f"  {i:3d} | {str(f['shape']):15s} | {str(det.ok):5s} | {row_center:18s} | {row_heading}")
        if det.ok:
            centers.append(det.center)

    mid = good[len(good) // 2]['img']
    sample_path = f"{out_dir}/validate_cross_sample.png"
    cv2.imwrite(sample_path, mid)

    det = cmd.detect(mid)
    annotated = mid.copy()
    if det.ok:
        cx, cy = int(det.center[0]), int(det.center[1])
        cv2.circle(annotated, (cx, cy), 6, (0, 0, 255), -1)
        cv2.putText(annotated, f"({cx},{cy})", (cx + 10, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    ann_path = f"{out_dir}/validate_cross_sample_annotated.png"
    cv2.imwrite(ann_path, annotated)
    print(f"\n[validate] sample -> {sample_path}")
    print(f"[validate] annotated -> {ann_path}")

    n_ok = sum(1 for f in good if cmd.detect(f['img']).ok)
    print(f"\n[validate] detection rate: {n_ok}/{len(good)} frames")
    if centers:
        xs = [c[0] for c in centers]
        ys = [c[1] for c in centers]
        jitter_x = max(xs) - min(xs)
        jitter_y = max(ys) - min(ys)
        print(f"[validate] center jitter across successful frames: dx={jitter_x:.1f}px dy={jitter_y:.1f}px")

    rclpy.shutdown()
    return 0 if n_ok > 0 else 3


if __name__ == "__main__":
    sys.exit(main())
