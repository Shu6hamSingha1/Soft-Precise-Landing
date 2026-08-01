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
# WIRED 2026-08-01 to match hardware_landing.py/check_loop_freq.py's now-
# validated real-flight defaults (see FLIGHT_TEST_ANALYSIS_PROCEDURE.md
# catalog #12/#13). SUPERSEDES the prior rationale below: this file used to
# deliberately skip CAM_MANUAL_EXPOSURE so aiming previews reflected whatever
# regime was live, but that meant an aiming check could look fine (or fail)
# under a DIFFERENT exposure/gain profile than what the drone actually flies
# with, which is misleading for its whole purpose (confirming decode before a
# flight). Now defaults to the same profile as the real landing flight;
# override on the command line (e.g. CAM_MANUAL_EXPOSURE=0 python3
# live_preview.py) to preview a different regime on purpose.
os.environ.setdefault("CAM_MANUAL_EXPOSURE", "1")
os.environ.setdefault("CAPTURE_RATE_HZ", "30")
os.environ.setdefault("CAM_EXPOSURE_US", "20000")
os.environ.setdefault("CAM_AUTO_GAIN", "1")

import time
import cv2
from imgstreamer import imgstream
from img_data import build_aruco_detector

DISPLAY_INTERVAL_S = 0.25   # ~4 Hz refresh - cheap enough to stay responsive
DISPLAY_SCALE = 6.0         # upscale factor for the preview window - getImages()
                            # returns the "main" stream at MAIN_STREAM_SIZE
                            # (320x240, project-wide single-resolution
                            # convention), which renders tiny on high-DPI
                            # displays without this. 2026-07-31: TWO earlier
                            # attempts failed -- (1) cv2.resize alone, (2)
                            # cv2.resize + explicit cv2.namedWindow(...,
                            # WINDOW_NORMAL) + cv2.resizeWindow() -- both had NO
                            # visible effect, on BOTH the Pi's own local GUI and
                            # X410-forwarded X11 (ruling out an X11-forwarding
                            # cause). Root cause, found by checking the PROVEN-
                            # WORKING equivalent in PX4_Gazebo/src/img_data.py
                            # (its VIDEO=True cv2.imshow preview, confirmed by
                            # user to actually scale correctly there): that code
                            # NEVER calls cv2.namedWindow() at all -- it just
                            # calls cv2.resize + cv2.imshow directly and lets
                            # OpenCV implicitly create the window. Explicitly
                            # pre-creating the window via namedWindow(...,
                            # WINDOW_NORMAL) turns OFF the auto-fit-to-image
                            # behavior an implicitly-created (WINDOW_AUTOSIZE)
                            # window has by default -- exactly backwards from
                            # what was needed. Fixed by removing namedWindow/
                            # resizeWindow entirely, matching the Gazebo
                            # pattern exactly (resize then imshow, nothing else).

# Shared with IMG_PROCESSOR (img_data.py) instead of a separate hardcoded
# copy - this file used to duplicate the params and silently drifted out of
# sync (still had the old maxMarkerPerimeterRate=0.5 after that was fixed to
# 4.0 in img_data.py, 2026-07-24/25 overflow-marker investigation).
_arucoDict, _arucoParams, _detector = build_aruco_detector()

CAPTURE_RATE_HZ = int(os.environ.get("CAPTURE_RATE_HZ", "60"))


def main():
    strm = imgstream(resolution=(640, 480), capRate=CAPTURE_RATE_HZ)
    print("Live preview running at ~4Hz refresh (full capture continues faster "
          "underneath) - press ESC in the window or Ctrl+C here to stop.")
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
                                  interpolation=cv2.INTER_AREA)
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
