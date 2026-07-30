"""Minimal live camera preview for AIMING the camera at the marker - separate
from record_rotation_check.py, which does the actual data recording.

Deliberately skips the full IMG_PROCESSOR pipeline (ArUco detection, optical
flow, gyro comp, KF/EKF) entirely - just raw camera frames -> cv2.imshow.
The earlier ~1.8Hz collapse (down from ~56Hz headless) when VIDEO=True ran
INSIDE the full pipeline points at cv2.imshow/Qt rendering itself as the
dominant cost, not the detection/flow math (a headless run of that SAME
pipeline hit ~56Hz fine) - so this throttles the DISPLAY refresh rate
(~4 Hz, plenty for human aiming) while the plain imgstream capture thread
underneath still runs at its own full rate, decoupling "how fast can we
capture" from "how often do we bother rendering a frame for a human".

Shows the ArUco-decode overlay too (cheap single-frame detectMarkers call at
display rate only, not every captured frame) so you can see live whether
it's actually decoding, not just whether the marker LOOKS visible.

Press Ctrl+C to stop. Throwaway diagnostic - not wired into anything.
"""
import os
# No CAM_MANUAL_EXPOSURE override here (unlike output_calibration.py/
# check_loop_freq.py) - this is an aiming/decode-check tool, not a motion
# recording script, so it should inherit imgstreamer.py's own default (auto
# exposure) same as any other unmodified run of the pipeline, not force the
# fixed-exposure recording setting that (2026-07-25 investigation) may itself
# suppress marker decode under some lighting.

import time
import cv2
from imgstreamer import imgstream
from img_data import build_aruco_detector

DISPLAY_INTERVAL_S = 0.25   # ~4 Hz refresh - cheap enough to stay responsive
DISPLAY_SCALE = 3.0         # upscale factor for the preview window - getImages()
                            # returns the "main" stream at MAIN_STREAM_SIZE
                            # (320x240, project-wide single-resolution
                            # convention), which renders tiny on high-DPI
                            # displays without this. 2026-07-31: an earlier
                            # attempt (DISPLAY_SCALE + cv2.resize + namedWindow
                            # (..., WINDOW_NORMAL)) visibly had NO effect, on
                            # BOTH the Pi's own local GUI and X410-forwarded X11
                            # -- ruling out an X11-forwarding/client-side cause.
                            # Root cause: WINDOW_NORMAL only makes a window
                            # user-resizable; it does NOT guarantee cv2.imshow()
                            # auto-grows the window's actual on-screen size to
                            # match a new (larger) frame -- a known OpenCV/Qt
                            # HighGUI quirk. The window was silently staying at
                            # whatever default/previous size Qt gave it and just
                            # rescaling the bigger image DOWN to fit. Fixed by
                            # explicitly calling cv2.resizeWindow() below.

# Shared with IMG_PROCESSOR (img_data.py) instead of a separate hardcoded
# copy - this file used to duplicate the params and silently drifted out of
# sync (still had the old maxMarkerPerimeterRate=0.5 after that was fixed to
# 4.0 in img_data.py, 2026-07-24/25 overflow-marker investigation).
_arucoDict, _arucoParams, _detector = build_aruco_detector()


def main():
    strm = imgstream(resolution=(640, 480), capRate=60)
    print("Live preview running at ~4Hz refresh (full capture continues faster "
          "underneath) - press ESC in the window or Ctrl+C here to stop.")
    cv2.namedWindow("Live Preview (~4Hz)", cv2.WINDOW_NORMAL)
    # Explicit resize -- see DISPLAY_SCALE comment above. namedWindow(WINDOW_NORMAL)
    # alone does NOT reliably grow the on-screen window to match a larger image;
    # this call sets the actual OS-level window size directly. MAIN_STREAM_SIZE is
    # 320x240 (project-wide convention); resizeWindow takes (width, height).
    cv2.resizeWindow("Live Preview (~4Hz)", int(320 * DISPLAY_SCALE), int(240 * DISPLAY_SCALE))
    try:
        while True:
            m = list(strm.getImages())[-1]
            if m is not None:
                corners, ids, _ = _detector.detectMarkers(m)
                decoded = ids is not None and len(ids) > 0
                vis = cv2.cvtColor(m, cv2.COLOR_GRAY2BGR)
                if decoded:
                    cv2.aruco.drawDetectedMarkers(vis, corners, ids)
                label = f"DECODED (id={ids.flatten().tolist()})" if decoded else "not decoded"
                cv2.putText(vis, label, (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (0, 255, 0) if decoded else (0, 0, 255), 1)
                vis = cv2.resize(vis, None, fx=DISPLAY_SCALE, fy=DISPLAY_SCALE,
                                  interpolation=cv2.INTER_NEAREST)
                cv2.imshow("Live Preview (~4Hz)", vis)
                if cv2.waitKey(1) == 27:   # ESC
                    break
            time.sleep(DISPLAY_INTERVAL_S)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        strm.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
