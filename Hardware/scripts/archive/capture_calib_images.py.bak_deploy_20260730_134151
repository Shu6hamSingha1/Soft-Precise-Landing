#!/usr/bin/env python3
"""
Interactive frame capture for camera intrinsic calibration at the SAME
resolution/mode used by the landing pipeline (640x480 raw Bayer ->
debayered BGR, matching imgstreamer.py's actual negotiated mode).

Run directly on the Pi (RealVNC desktop, not over SSH - needs a live GUI
window). Move the checkerboard through the frame at varied angles/
distances/positions (tilt it, corners near edges too) and press SPACE to
save each frame - NO live detection required, just frame it reasonably in
view. Aim for 15-20 images. Press ESC/q to quit.

Usage: python3 capture_calib_images.py [output_dir]
"""
import sys
import os
import time
import cv2
from picamera2 import Picamera2

OUT_DIR = sys.argv[1] if len(sys.argv) > 1 else "calib_images"
os.makedirs(OUT_DIR, exist_ok=True)


def unpack_raw10(packed, width):
    h = packed.shape[0]
    groups = width // 4
    reshaped = packed[:, :groups * 5].reshape(h, groups, 5)
    return reshaped[:, :, :4].reshape(h, groups * 4)


def main():
    camera = Picamera2()
    config = camera.create_video_configuration(
        main={"format": "YUV420", "size": (320, 240)},
        raw={"size": (640, 480)},
        display=None,
    )
    camera.configure(config)
    camera.start()
    time.sleep(2)

    actual = camera.camera_configuration()["raw"]["size"]
    print(f"Raw stream negotiated size: {actual}")
    width = actual[0]

    print("SPACE = save frame, ESC/q = quit. No live detection - just frame the")
    print("checkerboard clearly, vary position/distance/tilt across shots.")

    saved = 0
    win = "Calibration Capture (SPACE=save, ESC=quit)"
    while True:
        packed = camera.capture_array("raw")
        bayer = unpack_raw10(packed, width)
        bgr = cv2.cvtColor(bayer, cv2.COLOR_BAYER_BG2BGR)
        bgr = cv2.convertScaleAbs(bgr, alpha=4.0, beta=0)  # display-only gain

        disp = bgr.copy()
        cv2.putText(disp, f"saved: {saved}", (8, 20), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (0, 255, 0), 1, cv2.LINE_AA)
        cv2.imshow(win, disp)

        key = cv2.waitKey(1) & 0xFF
        if key in (27, ord('q')):
            break
        if key == ord(' '):
            fname = os.path.join(OUT_DIR, f"calib_{saved:03d}.png")
            cv2.imwrite(fname, bgr)
            saved += 1
            print(f"Saved {fname} ({saved} total)")

    camera.stop()
    cv2.destroyAllWindows()
    print(f"\nDone. {saved} images saved to {OUT_DIR}/")
    if saved < 9:
        print("WARNING: need at least 9 good images for a reliable calibration.")


if __name__ == "__main__":
    main()
