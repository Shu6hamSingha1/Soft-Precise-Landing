#!/usr/bin/env python3
"""Offline per-stage wall-clock timing of CrossMarkerPerception.process_frame() vs marker
extent (2026-09-24). Replays a recorded IMG_RECORD=1 frame dir (test_data/Test_Videos/*_raw)
through the live pipeline, wrapping each stage with a perf_counter timer.

Why: in SITL the processed-frame gap grows 16 -> 48 -> 96 ms (every 1st -> 3rd -> 6th camera
frame) once the marker fills the image below ~1 m (SPercGTFB_AB). This finds which stage
scales with marker extent.

Caveats (read before trusting absolute numbers):
  - Recorded frames carry the flow-point overlay (CROSS_RING_OVERLAY_DBG, pure yellow dots);
    they are inpainted out here, which is approximate.
  - The raw dir holds only the frames the live pipeline PROCESSED, so at close range
    consecutive replay frames are 3-6 camera frames apart (larger LK motion than live).
  - Identity attitude, zero body rate (no recorded quat pairing) -- geometry only.
  - Offline = no GIL contention with the controller/ROS threads, no sim load.

Usage: python3 tools/time_perception_stages.py "<raw_dir>" [--repeat 3]
"""
import sys, os, time, glob, argparse
from types import SimpleNamespace
from collections import defaultdict
import numpy as np
import cv2

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
import cross_marker_detector as cmd
import cross_marker_perception as cmp_mod
from cross_marker_perception import CrossMarkerPerception

T = defaultdict(float)          # per-frame stage accumulators (reset each frame)


def timed(name, fn):
    def w(*a, **k):
        t0 = time.perf_counter()
        try:
            return fn(*a, **k)
        finally:
            T[name] += time.perf_counter() - t0
    return w


def clean(img):
    """Inpaint the burned-in pure-yellow overlay dots."""
    m = ((img[:, :, 0] < 60) & (img[:, :, 1] > 200) & (img[:, :, 2] > 200)).astype(np.uint8)
    if m.any():
        m = cv2.dilate(m, np.ones((3, 3), np.uint8))
        img = cv2.inpaint(img, m, 3, cv2.INPAINT_TELEA)
    return img


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("raw_dir")
    ap.add_argument("--repeat", type=int, default=1, help="replay passes (timings = min over passes)")
    a = ap.parse_args()
    files = sorted(glob.glob(os.path.join(a.raw_dir, "f*.png")))
    frames = [clean(cv2.imread(f)) for f in files]
    H, W = frames[0].shape[:2]                     # saved frames are post-ROTATE_90_CW
    res = (W, H)                                   # Image_Node._res = (msg.height, msg.width) of the RAW msg
    quat = SimpleNamespace(w=1.0, x=0.0, y=0.0, z=0.0)
    av = SimpleNamespace(forward_rad_s=0.0, right_rad_s=0.0, down_rad_s=0.0)
    fps = 62.5

    per_pass = []
    for p in range(a.repeat):
        perc = CrossMarkerPerception(resolution=res)
        # wrap stages (instance methods + detector module function)
        cmd_detect = cmd.detect
        cmp_mod.cmd.detect = timed("detect", cmd_detect)
        for nm in ("_compute_hw", "_kf_update_hw", "_stepCentroidKf", "_log_frame_data",
                   "_getVirtualPts", "_snapshotBridgeAnchor", "_tryCenterBridge",
                   "_compute_hw_bgflow_fallback", "_solve_jacobian", "_sample_flow_points_ring",
                   "_sample_flow_points_unconstrained", "_sample_flow_points"):
            if hasattr(perc, nm):
                setattr(perc, nm, timed(nm, getattr(perc, nm)))
        rows = []
        for i in range(1, len(frames)):
            T.clear()
            t0 = time.perf_counter()
            perc.process_frame(frames[i - 1], frames[i], i / fps, fps,
                               quat_prev=quat, quat_curr=quat, angvel_prev=av, angvel_curr=av)
            tot = time.perf_counter() - t0
            ext = perc._last_extent_bbox
            ext = max(ext[2], ext[3]) if ext is not None else np.nan
            rows.append(dict(i=i, total=tot, ext=ext, ok=perc._ok, **dict(T)))
        cmp_mod.cmd.detect = cmd_detect
        per_pass.append(rows)

    # min over passes per frame (removes OS scheduling noise)
    keys = sorted({k for r in per_pass[0] for k in r if k not in ("i", "ext", "ok")})
    rows = []
    for j in range(len(per_pass[0])):
        r = dict(per_pass[0][j])
        for k in keys:
            r[k] = min(pp[j].get(k, 0.0) for pp in per_pass)
        rows.append(r)

    bins = [(0, 100), (100, 150), (150, 200), (200, 250), (250, 290), (290, 400)]
    show = ["total", "detect", "_compute_hw", "_sample_flow_points_ring", "_sample_flow_points_unconstrained",
            "_solve_jacobian", "_getVirtualPts", "_kf_update_hw", "_log_frame_data"]
    show = [k for k in show if k in keys]
    print(f"{len(rows)} frames from {a.raw_dir} (res={res}, repeat={a.repeat}); median ms per frame")
    print(f"{'extent px':>10}{'N':>5}{'ok':>5}" + "".join(f"{k.strip('_')[:12]:>13}" for k in show))
    for lo, hi in bins:
        sel = [r for r in rows if lo <= r["ext"] < hi]
        if not sel: continue
        print(f"{lo:>4}-{hi:<5}{len(sel):>5}{np.mean([r['ok'] for r in sel]):>5.2f}"
              + "".join(f"{1e3*np.median([r.get(k, 0.0) for r in sel]):>13.1f}" for k in show))
    other = [r["total"] - sum(r.get(k, 0.0) for k in ("detect", "_compute_hw", "_getVirtualPts",
                                                       "_kf_update_hw", "_stepCentroidKf", "_log_frame_data",
                                                       "_snapshotBridgeAnchor", "_tryCenterBridge",
                                                       "_compute_hw_bgflow_fallback")) for r in rows]
    print(f"\nunattributed (inline in process_frame) median {1e3*np.median(other):.1f} ms")


if __name__ == "__main__":
    main()
