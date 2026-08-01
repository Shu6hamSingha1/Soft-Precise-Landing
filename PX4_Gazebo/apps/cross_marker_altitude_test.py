"""Minimal takeoff-hover-land test for validating the cross-marker detector
at altitude, away from the ground-level airframe self-occlusion seen in the
pad-sitting frames validate_cross_marker.py captures.

Does NOT run the PLASMC controller or ArUco-based IMG_PROCESSOR -- arms,
takes off to ALT under stock PX4 position control, holds briefly, captures
frames from /image, runs src/cross_marker_detector.py against each, lands.

Launch:
    PY_SCRIPT=apps/cross_marker_altitude_test.py HEADLESS=1 \
        bash scripts/run_cross_marker_altitude_test.sh
"""
import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src'))
import asyncio
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from flight_controller import FC
from gz_subscriber import GZ_Subscriber, Pose_Node, Clock_Node
import cross_marker_detector as cmd

ALT = float(os.environ.get("CROSS_ALT", "5.0"))       # hover altitude (m)
T_HOLD = float(os.environ.get("CROSS_THOLD", "3.0"))   # hold time before capture (s)
N_FRAMES = int(os.environ.get("CROSS_NFRAMES", "15"))
OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "run_logs")


class ImageCapture(Node):
    def __init__(self, target_frames):
        super().__init__('cross_marker_altitude_capture')
        self._bridge = CvBridge()
        self._target = target_frames
        self.frames = []
        self.capturing = False  # gated -- don't collect until explicitly armed post-takeoff,
                                 # else the buffer fills with pre-liftoff ground frames
        self.sub = self.create_subscription(Image, '/image', self._cb, 10)

    def _cb(self, msg):
        if not self.capturing or len(self.frames) >= self._target:
            return
        try:
            img = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.frames.append(img)
        except Exception:
            pass


async def main():
    pose_sub = time_sub = img_sub = FC_node = None
    try:
        rclpy.init()
        pose_node = Pose_Node(); pose_sub = GZ_Subscriber(pose_node)
        time_node = Clock_Node(); time_sub = GZ_Subscriber(time_node)
        t0 = time.perf_counter()
        while time_node.perf_counter() is None:
            if time.perf_counter() - t0 > 10:
                raise Exception("no sim time")

        FC_node = FC(time_node); await FC_node.start()
        t0 = time.perf_counter()
        while not FC_node.has_quat():
            if time.perf_counter() - t0 > 20:
                raise Exception("no FC data")
            time.sleep(0.05)

        img_node = ImageCapture(N_FRAMES); img_sub = GZ_Subscriber(img_node)

        print(f"[cross_alt] taking off to {ALT} m...")
        await FC_node.arm_and_takeoff(ALT)

        print(f"[cross_alt] holding {T_HOLD}s before capture...")
        t0 = time.perf_counter()
        while time.perf_counter() - t0 < T_HOLD:
            await FC_node.send_velocity_body(0.0, 0.0, 0.0, 0.0)
            await asyncio.sleep(0.05)

        print(f"[cross_alt] capturing {N_FRAMES} frames...")
        img_node.capturing = True
        t0 = time.perf_counter()
        while len(img_node.frames) < N_FRAMES and time.perf_counter() - t0 < 15:
            await FC_node.send_velocity_body(0.0, 0.0, 0.0, 0.0)
            await asyncio.sleep(0.05)
        print(f"[cross_alt] got {len(img_node.frames)} frames")

        # --- land ---
        print("[cross_alt] landing...")
        try:
            await FC_node.vehicle.action.land()
        except Exception:
            pass
        await asyncio.sleep(4)

        # --- run detector against every captured frame ---
        os.makedirs(OUT_DIR, exist_ok=True)
        print("\n  idx | ok    | center             | heading_deg")
        print("  --- | ----- | ------------------- | -----------")
        centers = []
        n_ok = 0
        for i, frame in enumerate(img_node.frames):
            det = cmd.detect(frame)
            row_c = f"({det.center[0]:7.1f},{det.center[1]:7.1f})" if det.center else "  n/a"
            row_h = f"{det.heading_deg:6.1f}" if det.heading_deg is not None else "  n/a"
            print(f"  {i:3d} | {str(det.ok):5s} | {row_c:19s} | {row_h}")
            if det.ok:
                n_ok += 1
                centers.append(det.center)

        if img_node.frames:
            mid = img_node.frames[len(img_node.frames) // 2]
            sample_path = f"{OUT_DIR}/cross_alt_sample.png"
            cv2.imwrite(sample_path, mid)
            det = cmd.detect(mid)
            annotated = mid.copy()
            if det.ok:
                cx, cy = int(det.center[0]), int(det.center[1])
                cv2.circle(annotated, (cx, cy), 6, (0, 0, 255), -1)
            cv2.imwrite(f"{OUT_DIR}/cross_alt_sample_annotated.png", annotated)
            print(f"\n[cross_alt] sample -> {sample_path}")

        print(f"\n[cross_alt] detection rate: {n_ok}/{len(img_node.frames)}")
        if centers:
            xs = [c[0] for c in centers]; ys = [c[1] for c in centers]
            print(f"[cross_alt] center jitter: dx={max(xs)-min(xs):.1f}px dy={max(ys)-min(ys):.1f}px")

    finally:
        for s in (img_sub, pose_sub, time_sub):
            try:
                if s is not None and s.is_alive():
                    s.close(); s.join()
            except Exception:
                pass
        try:
            if FC_node is not None:
                await FC_node.close()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    asyncio.run(main())
