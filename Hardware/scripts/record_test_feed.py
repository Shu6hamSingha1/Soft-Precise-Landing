#!/usr/bin/env python3
"""
Record camera feed while running tests
Captures video + metadata for diagnostics
"""

import sys
import os
import time
import cv2
import subprocess
import threading
from datetime import datetime

sys.path.insert(0, ".")

from imgstreamer import imgstream


def record_camera_feed(duration=60, output_dir="Test_Data/Calibration/Test_Videos"):
    """Record camera feed to MP4 with timestamp"""

    os.makedirs(output_dir, exist_ok=True)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_file = os.path.join(output_dir, f"test_feed_{timestamp}.mp4")

    # Initialize stream
    # BUGFIX 2026-07-23: was (960, 720), which isn't a native IMX219 raw mode -
    # the sensor silently snapped to the nearest one, 1640x1232 (a genuinely
    # different sensor readout/binning path, not just a crop/scale of the
    # calibrated mode). Confirmed live: under the SAME manual-exposure
    # settings that give a well-exposed (mean 105/255, 90% ArUco decode) image
    # at 640x480, the 1640x1232 mode came out at mean 22/255 - essentially
    # black, 0% decode. 640x480 is also the ONLY resolution the project's
    # camera intrinsics (img_geometry.py CALIB_CX/CY/fx/fy) were ever measured
    # at - using anything else silently uses the wrong focal length too, a
    # second, independent reason this must match project convention exactly.
    stream = imgstream(resolution=(640, 480), capRate=60)
    time.sleep(2.0)  # let the camera/AEC/manual-exposure controls settle before recording

    # ACTUAL negotiated raw size, not the requested tuple - imgstreamer.py's
    # own docstring warns the sensor may snap to a different native mode
    # ("downstream code MUST use getResolution()"). Using the wrong size here
    # would make every VideoWriter.write() silently fail on a shape mismatch,
    # on top of the channel-count bug below.
    actual_res = stream.getResolution()

    print(f"Recording camera feed to: {output_file}")
    print(f"Duration: {duration}s")
    print(f"Resolution: {actual_res[0]}x{actual_res[1]} (negotiated)")
    print(f"FPS: 30")
    print()

    # Setup video writer
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    out = cv2.VideoWriter(output_file, fourcc, 30.0, actual_res)

    # Record
    start_time = time.time()
    frame_count = 0

    while time.time() - start_time < duration:
        imgs = stream.getImages()
        if imgs and len(imgs) > 0:
            frame = imgs[-1]
            if frame is not None:
                # BUGFIX 2026-07-23: getImages() returns the RAW single-channel
                # Bayer frame (imgstreamer.py's _unpack_raw10 output) - writing
                # that straight to a VideoWriter opened with the default
                # isColor=True (3-channel) silently rejects EVERY frame
                # ("expected 3 channels but got 1"), so the file was never
                # actually getting any frames despite frame_count climbing.
                # Debayer first (display_gain=1.0 - a real recording shouldn't
                # be artificially brightened like the live cv2.imshow preview
                # is; use the genuine raw-stream brightness).
                bgr = cv2.cvtColor(frame, cv2.COLOR_BAYER_BG2BGR)
                out.write(bgr)
                frame_count += 1

                # Progress
                elapsed = time.time() - start_time
                if frame_count % 30 == 0:
                    print(f"  {elapsed:.1f}s / {duration}s — {frame_count} frames")

        time.sleep(0.001)
    
    out.release()
    stream.close()
    
    elapsed = time.time() - start_time
    fps = frame_count / elapsed
    file_size = os.path.getsize(output_file) / (1024 * 1024)
    
    print()
    print("✓ Recording complete")
    print(f"  Duration: {elapsed:.1f}s")
    print(f"  Frames: {frame_count}")
    print(f"  FPS: {fps:.1f}")
    print(f"  File size: {file_size:.1f} MB")
    print(f"  Path: {output_file}")


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Record camera feed for testing")
    parser.add_argument("--duration", type=int, default=60, help="Recording duration in seconds")
    parser.add_argument("--output", default="Test_Data/Calibration/Test_Videos", help="Output directory")
    
    args = parser.parse_args()
    
    record_camera_feed(duration=args.duration, output_dir=args.output)

