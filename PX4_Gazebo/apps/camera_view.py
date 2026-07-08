#!/usr/bin/env python3
"""
Subscribe to /image and display a live feed in a cv2 window, with the
horizontal FoV and FoV-boundary gridlines overlaid, until the user closes
the window ('q' or Esc) or Ctrl+C.

Run via scripts/run_camera_view.sh (which brings up headless SITL + the
image bridge first). hfov / resolution below match
~/PX4-Autopilot/Tools/simulation/gz/models/mono_cam/model.sdf; update both
if that file changes.
"""
import sys
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

HFOV_RAD = 1.74
IMG_W = 640
IMG_H = 480
WINDOW_NAME = "camera_view (q/Esc to quit)"


class CameraView(Node):
    def __init__(self):
        super().__init__('camera_view')
        self._bridge = CvBridge()
        self.frame = None
        self.frame_count = 0
        self.sub = self.create_subscription(Image, '/image', self._cb, 10)

    def _cb(self, msg):
        try:
            self.frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.frame_count += 1
        except Exception as e:
            self.get_logger().warn(f"decode failed: {e}")


def draw_overlay(img):
    h, w = img.shape[:2]
    out = img.copy()
    cv2.line(out, (w // 2, 0), (w // 2, h), (0, 255, 0), 1)
    cv2.line(out, (0, h // 2), (w, h // 2), (0, 255, 0), 1)
    cv2.rectangle(out, (0, 0), (w - 1, h - 1), (0, 200, 255), 1)
    label = f"{w}x{h}  hfov={HFOV_RAD:.2f} rad ({np.degrees(HFOV_RAD):.1f} deg)"
    cv2.putText(out, label, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)
    return out


def main():
    rclpy.init()
    node = CameraView()
    print("[camera_view] waiting for /image ... (window opens on first frame)")
    print("[camera_view] press 'q' or Esc in the image window, or Ctrl+C here, to stop.")

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if node.frame is None:
                continue
            disp = draw_overlay(node.frame)
            cv2.imshow(WINDOW_NAME, disp)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27):  # q or Esc
                break
            if cv2.getWindowProperty(WINDOW_NAME, cv2.WND_PROP_VISIBLE) < 1:
                break
    except KeyboardInterrupt:
        pass
    finally:
        print(f"[camera_view] stopped after {node.frame_count} frames.")
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
