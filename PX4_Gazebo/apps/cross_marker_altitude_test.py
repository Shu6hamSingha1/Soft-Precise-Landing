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
    def __init__(self, target_frames, pose_node=None):
        super().__init__('cross_marker_altitude_capture')
        self._bridge = CvBridge()
        self._target = target_frames
        self._pose_node = pose_node
        self.frames = []
        self.gt_poses = []   # (uav_x,uav_y,uav_z, tgt_x,tgt_y,tgt_z) at capture time, parallel to frames
        self.capturing = False  # gated -- don't collect until explicitly armed post-takeoff,
                                 # else the buffer fills with pre-liftoff ground frames
        self.sub = self.create_subscription(Image, '/image', self._cb, 10)

    def _cb(self, msg):
        if not self.capturing or len(self.frames) >= self._target:
            return
        try:
            img = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.frames.append(img)
            if self._pose_node is not None:
                p = self._pose_node.getPose()
                if p.UAV is not None and p.target is not None:
                    u, t = p.UAV.position, p.target.position
                    self.gt_poses.append((u.x, u.y, u.z, t.x, t.y, t.z))
                else:
                    self.gt_poses.append(None)
            else:
                self.gt_poses.append(None)
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

        img_node = ImageCapture(N_FRAMES, pose_node=pose_node); img_sub = GZ_Subscriber(img_node)

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
        _record_all = os.environ.get("CROSS_RECORD_ALL_FRAMES", "0") == "1"
        _frames_dir = os.path.join(OUT_DIR, "all_frames")
        if _record_all:
            os.makedirs(_frames_dir, exist_ok=True)
        print("\n  idx | ok    | center             | heading_deg | GT xy_err (m)  | GT alt (m)")
        print("  --- | ----- | ------------------- | ----------- | -------------- | ----------")
        centers = []
        n_ok = 0
        for i, frame in enumerate(img_node.frames):
            det = cmd.detect(frame)
            row_c = f"({det.center[0]:7.1f},{det.center[1]:7.1f})" if det.center else "  n/a"
            row_h = f"{det.heading_deg:6.1f}" if det.heading_deg is not None else "  n/a"
            gt = img_node.gt_poses[i] if i < len(img_node.gt_poses) else None
            if gt is not None:
                ux, uy, uz, tx, ty, tz = gt
                gt_xy_err = float(np.hypot(ux - tx, uy - ty))
                gt_alt = float(uz - tz)
                row_gt = f"{gt_xy_err:8.3f}       | {gt_alt:6.2f}"
            else:
                row_gt = "   n/a          |    n/a"
            print(f"  {i:3d} | {str(det.ok):5s} | {row_c:19s} | {row_h} | {row_gt}")
            if det.ok:
                n_ok += 1
                centers.append(det.center)
            if _record_all:
                annotated = frame.copy()
                if det.ok and det.center:
                    cx, cy = int(det.center[0]), int(det.center[1])
                    cv2.circle(annotated, (cx, cy), 6, (0, 255, 0) if det.ok else (0, 0, 255), -1)
                cv2.putText(annotated, f"{i:03d} ok={det.ok} {det.fail_reason or ''}",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                            (0, 255, 0) if det.ok else (0, 0, 255), 2)
                cv2.imwrite(os.path.join(_frames_dir, f"f{i:03d}.png"), annotated)

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

        if _record_all and img_node.frames:
            import glob
            frame_files = sorted(glob.glob(os.path.join(_frames_dir, "f*.png")))
            if frame_files:
                first = cv2.imread(frame_files[0])
                h, w = first.shape[:2]
                video_path = os.path.join(OUT_DIR, "cross_alt_frames.mp4")
                writer = cv2.VideoWriter(video_path, cv2.VideoWriter_fourcc(*'mp4v'), 3, (w, h))
                for fp in frame_files:
                    writer.write(cv2.imread(fp))
                writer.release()
                print(f"[cross_alt] recorded {len(frame_files)} frames -> {video_path}")

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
