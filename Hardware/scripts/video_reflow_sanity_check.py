#!/usr/bin/env python3
"""QUALITATIVE-ONLY sanity check: re-run ArUco detection on an already-
recorded landing-test video and compare the reconstructed marker-motion
trace against that flight's own Control_Data.npy (s_dot_meas(t),
MARKER_EXTENT_PX(t)) - NOT a quantitative re-derivation.

WHY QUALITATIVE ONLY (2026-08-01): these old landing-test videos have no
per-frame timestamp tying them to Control_Data.npy's own clock (that gap is
fixed going forward via this session's hardware_landing.py/getImgData()
change, but not retroactively) - confirmed on "Thu Jul 30 17-59-22 2026.mp4"
(6.18s) vs its matching flight "17-59-41" (10.87s), no shared reference.
So this script deliberately does NOT attempt frame-level alignment or a
numeric ratio - it only compares RELATIVE pattern (detection-loss
proportion, rough trace shape) on each signal's own normalized 0-1
timeline. Confirms or contradicts the general story qualitatively; does
not replace the aggregate ratio analysis already done on properly-logged
flights.

Marker selection/detector params mirror img_data.py's build_aruco_detector()
(same dict/params) but WITHOUT the runtime's ROI-crop fast path or
temporal marker-ID locking (not needed for an offline, non-realtime pass) -
picks the largest-spread detected marker each frame independently, matching
the runtime's own INITIAL-lock heuristic (img_data.py ~line 1412).

Usage:
    python3 video_reflow_sanity_check.py <video.mp4> <landing_run_dir> [--out DIR]
"""
import argparse
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from img_data import build_aruco_detector
from img_geometry import CALIB_CX, CALIB_CY, fx, fy


def longest_streak(mask):
    best = cur = 0
    for v in mask:
        if v:
            cur += 1; best = max(best, cur)
        else:
            cur = 0
    return best


def reflow_video(video_path):
    _, _, detector = build_aruco_detector()
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        raise RuntimeError(f"could not open {video_path}")
    n_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0

    detected = np.zeros(n_frames, dtype=bool)
    cx = np.full(n_frames, np.nan); cy = np.full(n_frames, np.nan)
    for i in range(n_frames):
        ok, frame = cap.read()
        if not ok:
            break
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
        corners, ids, _ = detector.detectMarkers(gray)
        if ids is not None and len(ids) > 0:
            spreads = [float(np.std(c.reshape(-1, 2))) for c in corners]
            j = int(np.argmax(spreads))
            pts = corners[j].reshape(-1, 2)
            detected[i] = True
            cx[i] = (pts[:, 0].mean() - CALIB_CX) / fx
            cy[i] = (pts[:, 1].mean() - CALIB_CY) / fy
    cap.release()

    dt = 1.0 / fps
    vx = np.gradient(cx, dt); vy = np.gradient(cy, dt)
    speed = np.hypot(vx, vy)
    speed[~detected] = np.nan   # don't fabricate a rate across a lost-marker gap

    return dict(n_frames=n_frames, fps=fps, detected=detected, cx=cx, cy=cy, speed=speed)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("video")
    ap.add_argument("run_dir")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    print(f"Re-detecting ArUco markers in {args.video} ...")
    v = reflow_video(args.video)
    det_pct = 100.0 * v["detected"].sum() / v["n_frames"]
    longest_loss = longest_streak(~v["detected"])
    print(f"  video: {v['n_frames']} frames @ {v['fps']:.1f} declared fps")
    print(f"  detected: {det_pct:.1f}% of frames")
    print(f"  longest consecutive loss streak: {longest_loss} frames "
          f"({100.0*longest_loss/v['n_frames']:.1f}% of video)")

    cd_path = os.path.join(args.run_dir, "Control_Data.npy")
    d = np.load(cd_path, allow_pickle=True).item()
    mex = np.array(d["MARKER_EXTENT_PX(t)"])
    sdot = np.array(d["s_dot_meas(t)"])
    same = np.abs(np.diff(mex)) < 1e-9
    ctrl_longest = longest_streak(same)
    n_ctrl = len(mex)
    print(f"\n  Control_Data.npy: {n_ctrl} control-loop iterations")
    print(f"  longest MARKER_EXTENT_PX frozen (loss) streak: {ctrl_longest} iterations "
          f"({100.0*ctrl_longest/n_ctrl:.1f}% of flight)")

    print("\n=== QUALITATIVE COMPARISON (relative proportions only - NOT time-aligned) ===")
    print(f"  video marker-loss fraction:   {100.0*longest_loss/v['n_frames']:.1f}%")
    print(f"  flight marker-loss fraction:  {100.0*ctrl_longest/n_ctrl:.1f}%")
    print("  (if these are in the same ballpark, the video's own detection "
          "behavior is broadly consistent with what the flight logged - "
          "not proof of anything quantitative, just a sanity check)")

    out_dir = args.out or os.path.dirname(os.path.abspath(__file__))
    os.makedirs(out_dir, exist_ok=True)
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(2, 1, figsize=(10, 6))
        t_v = np.linspace(0, 1, v["n_frames"])
        axes[0].plot(t_v, v["speed"], lw=1)
        axes[0].set_title(f"Re-detected video centroid speed (normalized time) - {os.path.basename(args.video)}")
        axes[0].set_ylabel("norm. px/s")
        t_c = np.linspace(0, 1, n_ctrl)
        axes[1].plot(t_c, np.linalg.norm(sdot, axis=1), lw=1, color="tab:orange")
        axes[1].set_title(f"Flight s_dot_meas(t) magnitude (normalized time) - {os.path.basename(args.run_dir)}")
        axes[1].set_xlabel("normalized time (0-1, NOT frame-aligned to top plot)")
        fig.tight_layout()
        out_path = os.path.join(out_dir, "video_reflow_sanity_check.png")
        fig.savefig(out_path, dpi=120)
        print(f"\n  saved comparison plot -> {out_path}")
    except Exception as e:
        print(f"  [plot skipped: {e}]")


if __name__ == "__main__":
    main()
