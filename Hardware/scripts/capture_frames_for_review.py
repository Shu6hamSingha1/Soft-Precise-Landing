"""Save a handful of real camera frames to disk (both the raw stream,
debayered+brightened for human/visual inspection, and the main/ISP-scaled
stream actually used for ArUco detection) so they can be pulled off the Pi
and looked at directly - instead of guessing blind from CPU-only diagnostics.

Captures TWO short bursts back to back: one with CAM_MANUAL_EXPOSURE=1 (this
session's fix, current calibration default) and one with auto-exposure
(CAM_MANUAL_EXPOSURE=0, the old default) - so the two can be compared
side-by-side to see whether manual exposure is under/over-exposing relative
to what auto-exposure would have shown.

Runs ArUco detection (same detector config as img_data.py) on every saved
main-stream frame and reports decode success/failure per frame, so the
saved images can be cross-referenced against why decode did or didn't work.

Throwaway diagnostic - not wired into anything, discard after use.
"""
import os
import sys
import time
import numpy as np
import cv2

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "Test_Data", "FrameReview", time.strftime("%a %b %d %H-%M-%S %Y"))
N_FRAMES = 6
CAPTURE_WINDOW_S = 8.0

_arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
_arucoParams = cv2.aruco.DetectorParameters()
_arucoParams.adaptiveThreshWinSizeMin = 15
_arucoParams.adaptiveThreshWinSizeMax = 15
_arucoParams.adaptiveThreshWinSizeStep = 10
_arucoParams.minMarkerPerimeterRate = 0.02
_arucoParams.maxMarkerPerimeterRate = 0.5
_arucoParams.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_NONE
_detector = cv2.aruco.ArucoDetector(_arucoDict, _arucoParams)


def debayer_bayer_to_bgr(frame, display_gain=4.0):
    """Ported from img_data.py - raw-stream capture bypasses the ISP's own
    gain/tone-mapping, so the debayered image is genuinely dark without
    this boost. Display/visual-inspection only."""
    if frame is None or frame.ndim != 2:
        return frame
    bgr = cv2.cvtColor(frame, cv2.COLOR_BAYER_BG2BGR)
    return cv2.convertScaleAbs(bgr, alpha=display_gain, beta=0)


def capture_burst(label, manual_exposure, out_subdir):
    os.environ["CAM_MANUAL_EXPOSURE"] = "1" if manual_exposure else "0"
    # imgstreamer reads its env-derived constants at IMPORT time, not per-call -
    # force a fresh import so this session's env change actually takes effect
    # (relevant here since we deliberately toggle it between the two bursts).
    for mod in ("imgstreamer",):
        sys.modules.pop(mod, None)
    from imgstreamer import imgstream

    print(f"\n=== {label} (CAM_MANUAL_EXPOSURE={os.environ['CAM_MANUAL_EXPOSURE']}) ===")
    strm = imgstream(resolution=(640, 480), capRate=60)
    time.sleep(2.0)  # let AEC/AGC (if on) or manual controls settle

    os.makedirs(out_subdir, exist_ok=True)
    saved = 0
    last_main = None
    t0 = time.perf_counter()
    results = []
    while saved < N_FRAMES and (time.perf_counter() - t0) < CAPTURE_WINDOW_S:
        m = list(strm.getMainImages())[-1]
        r = list(strm.getImages())[-1]
        if m is not None and r is not None and (last_main is None or not np.array_equal(m, last_main)):
            last_main = m
            corners, ids, _ = _detector.detectMarkers(m)
            decoded = ids is not None and len(ids) > 0
            main_path = os.path.join(out_subdir, f"frame{saved:02d}_main320x240.png")
            raw_path = os.path.join(out_subdir, f"frame{saved:02d}_raw640x480_debayered.png")
            main_annotated = cv2.cvtColor(m, cv2.COLOR_GRAY2BGR)
            if decoded:
                cv2.aruco.drawDetectedMarkers(main_annotated, corners, ids)
            cv2.imwrite(main_path, main_annotated)
            cv2.imwrite(raw_path, debayer_bayer_to_bgr(r))
            mean_brightness = float(np.mean(m))
            results.append((saved, decoded, mean_brightness))
            print(f"  frame{saved:02d}: decoded={decoded}  main_stream_mean_brightness={mean_brightness:.1f}/255")
            saved += 1
        time.sleep(0.1)
    strm.close()
    n_decoded = sum(1 for _, d, _ in results if d)
    print(f"  {n_decoded}/{len(results)} frames decoded, saved to {out_subdir}")
    return results


if __name__ == "__main__":
    print("Point the camera at the marker now - capturing two short bursts "
          "(manual exposure, then auto exposure) for comparison...")
    r1 = capture_burst("BURST 1: manual exposure (current calibration default)",
                        True, os.path.join(OUT_DIR, "manual_exposure"))
    time.sleep(1.0)
    r2 = capture_burst("BURST 2: auto exposure (old default)",
                        False, os.path.join(OUT_DIR, "auto_exposure"))
    print(f"\nAll frames saved under: {OUT_DIR}")
