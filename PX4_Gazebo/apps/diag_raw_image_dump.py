# One-off diagnostic: dump raw camera frames (pre- and post-rotation) WITHOUT
# flying, to isolate whether the double-drone artifact seen in
# record_cross_marker_calibration.py originates at the Gazebo render / bridge
# level (would already be present sitting on the ground, disarmed) or is
# introduced somewhere in our own pipeline. No arm/takeoff -- just connects,
# waits for frames, and saves them. See the point-starvation/centroid-
# instability investigation (2026-08-01).
import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src'))
import asyncio
import time
import numpy as np
import cv2
import rclpy

from flight_controller import FC
from gz_subscriber import GZ_Subscriber, Image_Node, Clock_Node

OUT_DIR = os.environ.get("DIAG_RAW_OUT", "/home/shubham/Soft-Precise-Landing/PX4_Gazebo/calibration_data/diag_raw")
N_FRAMES = int(os.environ.get("DIAG_RAW_N", "10"))


async def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    rclpy.init()
    time_node = Clock_Node()
    time_subscriber = GZ_Subscriber(time_node)

    start_time = time.perf_counter()
    while time_node.perf_counter() is None:
        if (time.perf_counter() - start_time) > 20:
            raise Exception("Unable to get simulation time.")

    FC_node = FC(time_node)
    await FC_node.start()

    start_time = time.perf_counter()
    while not FC_node.has_quat():
        if (time.perf_counter() - start_time) > 20:
            raise Exception("Unable to get data from Flight Controller.")
        time.sleep(0.05)

    image_node = Image_Node(time_keeper=time_node, controller=FC_node)
    image_subscriber = GZ_Subscriber(image_node)

    print(f"[diag] connected, waiting for {N_FRAMES} distinct frames (drone stays on ground, disarmed)...")
    saved = 0
    last_stamp = None
    t0 = time.perf_counter()
    while saved < N_FRAMES and (time.perf_counter() - t0) < 30:
        stamp = image_node.getStamp()
        if stamp is None or stamp == last_stamp:
            time.sleep(0.02)
            continue
        last_stamp = stamp
        imgs = image_node.getImages()
        rot_frame = imgs[-1]
        if rot_frame is None:
            time.sleep(0.02)
            continue
        # rot_frame is the POST-rotation frame the rest of the pipeline consumes.
        cv2.imwrite(os.path.join(OUT_DIR, f"rot_{saved:03d}.png"), rot_frame)
        # Undo the rotation to recover what the raw bridged frame looked like
        # (ROTATE_90_CLOCKWISE's inverse is ROTATE_90_COUNTERCLOCKWISE) -- lets us
        # inspect the un-rotated geometry without re-plumbing image_callback.
        raw_frame = cv2.rotate(rot_frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
        cv2.imwrite(os.path.join(OUT_DIR, f"raw_{saved:03d}.png"), raw_frame)
        saved += 1
        print(f"[diag] saved frame {saved}/{N_FRAMES} (stamp={stamp})")

    print(f"[diag] done, {saved} frames saved -> {OUT_DIR}")

    image_subscriber.close(); image_subscriber.join()
    time_subscriber.close(); time_subscriber.join()
    await FC_node.close()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    asyncio.run(main())
