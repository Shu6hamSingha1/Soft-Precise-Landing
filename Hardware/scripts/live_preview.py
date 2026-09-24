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

CROSS MARKER (2026-09-24): PREVIEW_MARKER selects the detector overlay:
  cross (default) - cross_marker_detector.detect() (hardware copy of the PX4
                    detector). Draws the centre, both arm fits, stub/heading
                    and bbox, and shows fail_reason when detection is rejected.
                    That reason is how to tell "marker not visible" from "a
                    SITL-tuned px threshold rejected it" (port-plan stage S3).
  aruco           - the original ArUco decode overlay.
  both            - both overlays on the same frame.
CROSS_DETECTOR=legacy (default) | stroke picks the cross detector
implementation. stroke is PX4's 2026-09-24 locked-design ridge-stroke detector
(cross_stroke_detector.py): no absolute intensity gate, stroke-matched working
resolution, so it is the closer fit for the no-marker-size port.
Also shows detect() time: cross cost grows with the marker's pixel count
(port-plan R2), so watch it when the camera is close to the marker.

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

import math
import time
import cv2
from imgstreamer import imgstream
from img_data import build_aruco_detector
import cross_marker_detector as cmd

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
PREVIEW_MARKER = os.environ.get("PREVIEW_MARKER", "cross").lower()   # cross | aruco | both

_FONT = cv2.FONT_HERSHEY_SIMPLEX
_GREEN, _RED, _YELLOW, _CYAN, _MAGENTA = (0, 255, 0), (0, 0, 255), (0, 255, 255), (255, 255, 0), (255, 0, 255)


def _draw_pts(vis, pts, color):
    H, W = vis.shape[:2]
    for p in (pts if pts is not None else ()):
        x, y = int(round(p[0])), int(round(p[1]))
        if 0 <= x < W and 0 <= y < H:
            vis[y, x] = color


def _overlay_cross(vis, frame_bgr, track_state, y):
    """Run the cross detector on this frame and draw its result. Returns the next text row."""
    t0 = time.perf_counter()
    det = cmd.detect(frame_bgr, track_state=track_state)
    dt_ms = (time.perf_counter() - t0) * 1e3
    if det.mask_bbox is not None:
        bx, by, bw, bh = (int(v) for v in det.mask_bbox)
        cv2.rectangle(vis, (bx, by), (bx + bw, by + bh), _YELLOW if det.ok else _RED, 1)
    if det.ok:
        _draw_pts(vis, det.line_points_i, _CYAN)
        _draw_pts(vis, det.line_points_j, _MAGENTA)
        _draw_pts(vis, det.stub_points, _YELLOW)
        cx, cy = det.center
        c = (int(round(cx)), int(round(cy)))
        cv2.drawMarker(vis, c, _GREEN if det.in_fov else _YELLOW, cv2.MARKER_CROSS, 12, 2)
        if det.heading_deg is not None:
            a = math.radians(det.heading_deg)
            cv2.arrowedLine(vis, c, (int(c[0] + 25 * math.cos(a)), int(c[1] + 25 * math.sin(a))),
                            _YELLOW, 1, tipLength=0.3)
        H, W = vis.shape[:2]
        ext = int(max(det.mask_bbox[2], det.mask_bbox[3])) if det.mask_bbox is not None else 0
        hd = f"{det.heading_deg:.0f}deg" if det.heading_deg is not None else "none"
        label = f"CROSS[{cmd.DETECTOR}] ok c=({cx:.0f},{cy:.0f}){'' if det.in_fov else ' OFF-FOV'} hdg={hd}"
        info = f"ext={ext}px fill={ext / min(H, W):.2f} detect {dt_ms:.1f}ms"
        color = _GREEN
    else:
        label, info, color = f"CROSS[{cmd.DETECTOR}] fail: {det.fail_reason}", f"detect {dt_ms:.1f}ms", _RED
    # Two short lines: the frame is only 320 px wide before the display upscale.
    cv2.putText(vis, label, (5, y), _FONT, 0.35, color, 1)
    cv2.putText(vis, info, (5, y + 12), _FONT, 0.35, color, 1)
    return y + 26


def _overlay_aruco(vis, gray, y):
    corners, ids, _ = _detector.detectMarkers(gray)
    decoded = ids is not None and len(ids) > 0
    if decoded:
        cv2.aruco.drawDetectedMarkers(vis, corners, ids)
    label = f"ARUCO DECODED (id={ids.flatten().tolist()})" if decoded else "ARUCO not decoded"
    cv2.putText(vis, label, (5, y), _FONT, 0.4, _GREEN if decoded else _RED, 1)
    return y + 14


def main():
    if PREVIEW_MARKER not in ("cross", "aruco", "both"):
        raise SystemExit(f"PREVIEW_MARKER={PREVIEW_MARKER!r}: expected cross | aruco | both")
    strm = imgstream(resolution=(640, 480), capRate=CAPTURE_RATE_HZ)
    print(f"Live preview [{PREVIEW_MARKER}] running at ~4Hz refresh (full capture continues "
          "faster underneath) - press ESC in the window or Ctrl+C here to stop.")
    # Owned here and passed to every detect() call so the detector's tracked-ROI fast path
    # works across frames. It sees only display-rate frames (~4 Hz), so the lock
    # re-acquires more often than it would at full rate.
    track_state = {'last_bbox': None, 'miss_count': 0}
    try:
        while True:
            m = list(strm.getImages())[-1]
            if m is not None:
                vis = cv2.cvtColor(m, cv2.COLOR_GRAY2BGR)
                y = 15
                if PREVIEW_MARKER in ("cross", "both"):
                    # detect() expects BGR. The stream is grayscale, so its HSV V channel is
                    # just the gray level, and the default V<100 dark gate still applies.
                    y = _overlay_cross(vis, vis.copy(), track_state, y)
                if PREVIEW_MARKER in ("aruco", "both"):
                    y = _overlay_aruco(vis, m, y)
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
