#!/usr/bin/env python3
"""Align a recorded Test_Videos/*.mp4 to its run's Img_Data.npy log rows by
FRAME INDEX, not by video timestamp/fps (2026-07-31).

WHY INDEX, NOT TIME: img_data.py's run() loop writes exactly one video frame
(self._video.write(...), img_data.py ~line 810) and calls _optFlowAngVel(...)
(~line 813) - which appends exactly one row to "Time"/"Opt Flow Estimator
Tag"/"Opt Flow Ang Vel" via either the success branch or
_rescueOrCoastFeatureLog() - in the SAME loop iteration, gated by the same
"images available" check. So video frame N and Img_Data.npy row N should be
the same instant BY CONSTRUCTION. The video's own cv2.VideoWriter fps is a
fixed nominal self._capRate label used only for playback speed - real
inter-frame timing is irregular (see derive_pi_cal.py's gap/coast handling),
so aligning by elapsed video seconds would be wrong; aligning by frame COUNT
is what the code's control flow actually guarantees.

THIS HAS NEVER BEEN VALIDATED END-TO-END: as of 2026-07-31, zero archived
calibration runs have a paired video (root cause: output_calibration.py's
record-prompt was hardcoded to 'n', fixed in the same session - see its own
comment), and real hardware_landing.py flights didn't save Img_Data.npy at
all (also fixed this session). The FIRST run made after both fixes should be
checked with --validate before this tool is trusted for anything downstream.

Usage:
    python3 align_video_to_tags.py <run_dir> [--video PATH] [--validate]
        [--rows N [N ...]] [--out DIR]

    <run_dir>   directory containing Img_Data.npy (Output/<ts> or
                Landing/<ts>)
    --video     explicit video path; default: look for
                Test_Data/Landing/Test_Videos/<run_dir's own timestamp>.mp4
                (img_data.py names the video after self.timestamp, set at
                first RECORD write - NOT guaranteed identical to the run
                dir's own folder name, since that is chosen independently by
                the calling script's own save step; pass --video explicitly
                if the automatic match doesn't find it)
    --validate  just check video frame count vs len(Img_Data.npy["Time"])
                and print the estimator-tag breakdown; no frame extraction
    --rows      specific row indices to pull frames for (default: one frame
                near the start, middle, and end of the longest coast streak,
                if any, plus one clean 'lstsq' frame for reference)
    --out       directory to write extracted PNGs (default: alongside this
                script, align_frames/<run_dir_name>/)
"""
import argparse
import glob
import os
import sys

import numpy as np


def find_video(run_dir):
    """Best-effort match: look for a Test_Videos/*.mp4 whose own filename
    timestamp is within a few seconds of the run_dir's own timestamp-derived
    name. Falls back to None if nothing close is found - the caller should
    then require --video explicitly rather than guess wrong."""
    import re
    from datetime import datetime

    def _parse_ts(name):
        # img_data.py / output_calibration.py both name things from
        # time.ctime().replace(':', '-'), e.g. "Tue Jul 28 15-47-21 2026"
        m = re.search(r'([A-Za-z]{3} [A-Za-z]{3} \d{1,2} \d{2}-\d{2}-\d{2} \d{4})',
                       name)
        if not m:
            return None
        try:
            return datetime.strptime(m.group(1), "%a %b %d %H-%M-%S %Y")
        except ValueError:
            return None

    run_ts = _parse_ts(os.path.basename(os.path.normpath(run_dir)))
    if run_ts is None:
        return None

    candidates = []
    for base in (
        os.path.join(os.path.dirname(__file__), "Test_Data", "Landing", "Test_Videos"),
        os.path.join(os.path.dirname(__file__), "Test_Data", "Calibration", "Test_Videos"),
    ):
        candidates.extend(glob.glob(os.path.join(base, "*.mp4")))

    best, best_dt = None, None
    for c in candidates:
        ts = _parse_ts(os.path.basename(c))
        if ts is None:
            continue
        dt = abs((ts - run_ts).total_seconds())
        if best_dt is None or dt < best_dt:
            best, best_dt = c, dt
    # 120s window - generous, since video-file timestamp is written at first
    # successful RECORD write, which can lag the run dir's own save-time
    # timestamp by however long the recording ran.
    if best is not None and best_dt is not None and best_dt < 120:
        return best
    return None


def tag_breakdown(tags):
    tags = np.asarray([str(t) for t in tags])
    u, c = np.unique(tags, return_counts=True)
    return dict(zip(u.tolist(), c.tolist()))


def longest_streak(tags, value):
    tags = [str(t) for t in tags]
    best_len = best_start = cur_len = cur_start = 0
    for i, t in enumerate(tags):
        if t == value:
            if cur_len == 0:
                cur_start = i
            cur_len += 1
            if cur_len > best_len:
                best_len, best_start = cur_len, cur_start
        else:
            cur_len = 0
    return best_start, best_len


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run_dir")
    ap.add_argument("--video", default=None)
    ap.add_argument("--validate", action="store_true")
    ap.add_argument("--rows", type=int, nargs="+", default=None)
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    img_path = os.path.join(args.run_dir, "Img_Data.npy")
    if not os.path.exists(img_path):
        sys.exit(f"no Img_Data.npy in {args.run_dir}")
    img = np.load(img_path, allow_pickle=True).item()
    tags = img.get("Opt Flow Estimator Tag", [])
    t_arr = img.get("Time", [])
    n_log = len(t_arr)
    print(f"Img_Data.npy: {n_log} rows")
    if tags:
        print("estimator-tag breakdown:", tag_breakdown(tags))
    else:
        print("[warn] no 'Opt Flow Estimator Tag' in this run - older recording")

    video_path = args.video or find_video(args.run_dir)
    if video_path is None:
        sys.exit("no video found/matched - pass --video explicitly "
                  "(see find_video()'s matching window if this is surprising)")
    print(f"video: {video_path}")

    import cv2
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        sys.exit(f"could not open {video_path}")
    n_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    print(f"video frame count: {n_frames}")

    match = (n_frames == n_log)
    print(f"frame-count == log-row-count: {match}"
          + ("" if match else "  [MISMATCH - index alignment is NOT reliable "
             "for this run; see this tool's docstring on the 1:1-append "
             "invariant, and check whether this run predates both fixes]"))

    if args.validate:
        cap.release()
        return

    rows = args.rows
    if rows is None:
        rows = []
        if tags:
            start, length = longest_streak(tags, "coast")
            if length > 0:
                rows += [start, start + length // 2, min(start + length - 1, n_log - 1)]
            clean = [i for i, t in enumerate(tags) if str(t) in ("lstsq", "lstsq+klt")]
            if clean:
                rows.append(clean[len(clean) // 2])
        if not rows:
            rows = [0, n_log // 2, n_log - 1]
        rows = sorted(set(r for r in rows if 0 <= r < n_log))
    print(f"extracting rows: {rows}")

    out_dir = args.out or os.path.join(
        os.path.dirname(__file__), "align_frames",
        os.path.basename(os.path.normpath(args.run_dir)))
    os.makedirs(out_dir, exist_ok=True)

    for r in rows:
        if r >= n_frames:
            print(f"  row {r}: SKIP - beyond video frame count ({n_frames})")
            continue
        cap.set(cv2.CAP_PROP_POS_FRAMES, r)
        ok, frame = cap.read()
        if not ok:
            print(f"  row {r}: failed to read frame")
            continue
        tag = str(tags[r]) if tags else "?"
        t_val = float(t_arr[r]) if r < len(t_arr) else float('nan')
        out_path = os.path.join(out_dir, f"row{r:05d}_{tag}_t{t_val:.3f}.png")
        cv2.imwrite(out_path, frame)
        print(f"  row {r}: tag={tag} t={t_val:.3f} -> {out_path}")

    cap.release()


if __name__ == "__main__":
    main()
