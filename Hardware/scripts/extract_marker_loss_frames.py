"""Extract still frames from a landing-test video at given timestamps for
visual inspection of marker acquisition/loss, plus run the SAME ArUco
detector img_data.py uses (build_aruco_detector) on each frame so we get an
objective decode/corner-size readout alongside the image itself.

Usage: python extract_marker_loss_frames.py <video.mp4> <out_dir> <t1> <t2> ...
"""
import sys
import os
import cv2


def build_aruco_detector():
    """Inlined copy of img_data.py's build_aruco_detector() -- can't import
    img_data.py directly here since it transitively imports picamera2
    (Pi-only, not installed on this Windows workstation). Same params/
    defaults as the live detector so offline frame analysis matches
    on-device behavior."""
    arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    arucoParams = cv2.aruco.DetectorParameters()
    arucoParams.adaptiveThreshWinSizeMin = int(os.environ.get("ARUCO_THRESH_WIN_MIN", "15"))
    arucoParams.adaptiveThreshWinSizeMax = int(os.environ.get("ARUCO_THRESH_WIN_MAX", "15"))
    arucoParams.adaptiveThreshWinSizeStep = int(os.environ.get("ARUCO_THRESH_WIN_STEP", "10"))
    arucoParams.minMarkerPerimeterRate = float(os.environ.get("ARUCO_MIN_PERIMETER_RATE", "0.02"))
    arucoParams.maxMarkerPerimeterRate = float(os.environ.get("ARUCO_MAX_PERIMETER_RATE", "4.0"))
    arucoParams.cornerRefinementMethod = (
        cv2.aruco.CORNER_REFINE_NONE
        if os.environ.get("ARUCO_CORNER_REFINE", "subpix") == "none"
        else cv2.aruco.CORNER_REFINE_SUBPIX)
    if hasattr(arucoParams, "useAruco3Detection"):
        try:
            arucoParams.useAruco3Detection = os.environ.get("ARUCO_USE_ARUCO3", "0") == "1"
            arucoParams.minSideLengthCanonicalImg = int(os.environ.get("ARUCO_ARUCO3_CANON_SIDE", "32"))
        except Exception:
            pass
    detector = cv2.aruco.ArucoDetector(arucoDict, arucoParams)
    return arucoDict, arucoParams, detector


_arucoDict, _arucoParams, _detector = build_aruco_detector()


def main():
    video_path = sys.argv[1]
    out_dir = sys.argv[2]
    times = [float(x) for x in sys.argv[3:]]
    os.makedirs(out_dir, exist_ok=True)

    cap = cv2.VideoCapture(video_path)
    fps = cap.get(cv2.CAP_PROP_FPS)
    n_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    print(f"video: {video_path}  fps={fps:.2f}  frames={n_frames}  duration={n_frames/fps:.2f}s")

    for t in times:
        frame_idx = int(round(t * fps))
        if frame_idx >= n_frames:
            print(f"  t={t}s -> frame {frame_idx} OUT OF RANGE (max {n_frames-1})")
            continue
        cap.set(cv2.CAP_PROP_POS_FRAMES, frame_idx)
        ok, frame = cap.read()
        if not ok:
            print(f"  t={t}s -> frame {frame_idx} FAILED to read")
            continue
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
        corners, ids, rejected = _detector.detectMarkers(gray)
        decoded = ids is not None and len(ids) > 0
        vis = frame.copy()
        label = "not decoded"
        extent = None
        if decoded:
            cv2.aruco.drawDetectedMarkers(vis, corners, ids)
            c = corners[0][0]
            extent = float(max(c[:, 0].max() - c[:, 0].min(), c[:, 1].max() - c[:, 1].min()))
            label = f"DECODED id={ids.flatten().tolist()} extent={extent:.1f}px"
        cv2.putText(vis, label, (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                    (0, 255, 0) if decoded else (0, 0, 255), 1)
        out_path = os.path.join(out_dir, f"t{t:.2f}s_frame{frame_idx}.png")
        cv2.imwrite(out_path, vis)
        print(f"  t={t}s (frame {frame_idx}): {label}  n_rejected_candidates={len(rejected) if rejected is not None else 0}  -> {out_path}")

    cap.release()


if __name__ == "__main__":
    main()
