#!/usr/bin/env python3
"""
Display camera feed with optical flow visualization
Press Ctrl+C to stop

All OpenCV GUI calls (imshow/waitKey/destroyAllWindows) happen inside
IMG_PROCESSOR's own background thread (img_data.py, when VIDEO=True) -
HighGUI/Qt is not thread-safe, so this script must NOT make any cv2 GUI
calls itself (that caused a cross-thread Qt deadlock on Ctrl+C).
"""

import sys
import os
sys.path.insert(0, ".")

# WIRED 2026-08-01 to match hardware_landing.py's now-validated real-flight
# defaults (FLIGHT_TEST_ANALYSIS_PROCEDURE.md catalog #12/#13).
os.environ.setdefault("CAM_MANUAL_EXPOSURE", "1")
os.environ.setdefault("CAPTURE_RATE_HZ", "30")
os.environ.setdefault("CAM_EXPOSURE_US", "20000")
os.environ.setdefault("CAM_AUTO_GAIN", "1")

import img_data
import time

# Enable video display (drawn by IMG_PROCESSOR's own thread)
img_data.VIDEO = True

from img_data import IMG_PROCESSOR

print("=" * 60)
print("Camera Display Test")
print("=" * 60)
print()
print("Configuration:")
print("  Resolution: 960x720")
print("  FPS: 47.8")
print("  Display: cv2.imshow() \"Image Streamer\"")
print()
print("Starting camera display...")
print("Window: \"Image Streamer\"")
print("Press ESC (in the image window) or Ctrl+C (here) to stop")
print()

processor = IMG_PROCESSOR()

frame_count = 0
start_time = time.time()

try:
    while processor._STAY_OPEN:
        time.sleep(0.02)  # ~50 Hz
        frame_count += 1

        if frame_count % 50 == 0:
            elapsed = time.time() - start_time
            fps = frame_count / elapsed
            print(f"Frames: {frame_count}, FPS: {fps:.1f}")

    print("\nESC was pressed in the image window, stopping...")

except KeyboardInterrupt:
    print("\n\nCtrl+C pressed, stopping...")

except Exception as e:
    print(f"\n✗ Error: {e}")
    import traceback
    traceback.print_exc()

finally:
    print("\nCleaning up...")
    processor.close()
    processor.join(timeout=3)
    if processor.is_alive():
        print("Warning: processor thread did not stop within 3s (forcing exit)")

    elapsed = time.time() - start_time
    fps = frame_count / elapsed if elapsed > 0 else 0

    print()
    print("=" * 60)
    print("Test Complete")
    print("=" * 60)
    print(f"Frames captured: {frame_count}")
    print(f"Duration: {elapsed:.1f}s")
    print(f"Average FPS: {fps:.1f}")
    print()

    sys.exit(0)
