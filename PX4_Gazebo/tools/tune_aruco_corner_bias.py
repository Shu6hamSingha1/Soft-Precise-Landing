#!/usr/bin/env python3
"""Offline ArUco CORNER-BIAS tuner. Different question from tune_aruco_decode.py (which
measures DECODE success rate): this measures whether the DECODED corners are geometrically
TRUSTWORTHY -- specifically, whether marker extent (apparent size) grows monotonically as
altitude decreases, which it must (apparent size ~ 1/Z for a fixed-focal-length camera on a
non-drifting, level approach).

Motivation (2026-07-11, IC1 rep4 investigation): a live recording showed the PRIMARY marker's
decoded corner extent SHRINKING smoothly and monotonically (M ~ 2nd moment: 20002->10514) while
GT confirmed the drone was genuinely, monotonically approaching dead-center with negligible tilt
-- ruling out every physical explanation (ascent, lateral drift, marker-ID switch, tilt).
Independent reconstruction of loom directly from the corner extent (loom = -0.5 d(ln M)/dt)
matched the pipeline's own reported h_z in sign and magnitude, proving the lstsq/KF/cal pipeline
was faithfully computing loom from BAD corner input, not the source of the bug.

Leading hypothesis: cornerRefinementWinSize=5 (img_data.py, an 11x11 px SUBPIX search window,
tuned 2026-05-22 for low-contrast/far/small-marker detection, never re-validated for the
large-in-frame/close-range regime) is smaller than a single ArUco module once the marker is
large in frame (DICT_4X4_50 = 6x6 modules; at ~150-200px total extent each module is ~25-33px,
more than double the window) -- cornerSubPix may lock onto an INTERNAL module edge instead of
the true outer marker corner, biasing all 4 corners inward, worsening as the marker grows.

This tool tests that hypothesis directly and offline: sweeps cornerRefinementWinSize (+ a
CORNER_REFINE_NONE control) on ONE set of recorded frames (IMG_RECORD_RAW=1), and reports mean
corner extent per altitude band. A HEALTHY config's extent must increase monotonically as
altitude decreases (near bands > far bands). The CURRENT baseline (winSize=5) is expected to
be non-monotonic (near-band extent LOWER than a farther band) if the hypothesis is correct.

Usage: python3 tools/tune_aruco_corner_bias.py <raw_frame_dir> <gt_rep_dir>
  raw_frame_dir : test_data/Test_Videos/<ts>_raw  (f*.png + stamps.npy, from IMG_RECORD_RAW=1)
  gt_rep_dir    : the matching landing rep dir containing Ground_Truth.npy

Record raw frames for a descent through ~0.5-2m altitude (the band where the artifact was
found) with: IMG_RECORD_RAW=1 bash scripts/run_aruco_landing.sh
"""
import sys, os, glob
import numpy as np
import cv2

ADICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
BANDS = [(3.0, 5.2), (2.0, 3.0), (1.5, 2.0), (1.0, 1.5), (0.5, 1.0), (0.0, 0.5)]  # far -> near


def _params(**ov):
    """img_data.py baseline (src/img_data.py:206-219), overridden per-sweep-point."""
    p = cv2.aruco.DetectorParameters()
    p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    p.cornerRefinementWinSize = 5
    p.cornerRefinementMaxIterations = 30
    p.cornerRefinementMinAccuracy = 0.01
    p.adaptiveThreshConstant = 5.0
    p.errorCorrectionRate = 0.8
    p.minMarkerPerimeterRate = 0.02
    p.minOtsuStdDev = 3.0
    for k, v in ov.items():
        setattr(p, k, v)
    return p


def _load(raw_dir, gt_dir):
    frames = [cv2.imread(f, cv2.IMREAD_GRAYSCALE) for f in sorted(glob.glob(os.path.join(raw_dir, "f*.png")))]
    stamps = np.load(os.path.join(raw_dir, "stamps.npy"))
    gt = np.load(os.path.join(gt_dir, "Ground_Truth.npy"), allow_pickle=True).item()
    St = float(gt['Start Time']); tg = np.asarray(gt['Time'], float)
    u, tp = gt['UAV Pose'], gt['Target Pose']; n = min(len(tg), len(u), len(tp))
    altg = np.array([abs(tp[i].position.z - u[i].position.z) for i in range(n)])
    alt = np.interp(stamps - St, tg[:n], altg)
    return frames, alt


def row(label, frames, alt, p):
    """SEQUENTIAL, STATEFUL marker selection mirroring img_data.py's actual _locked_marker_id
    logic (NOT a per-frame largest-spread pick -- that was this tool's original bug, confirmed
    2026-07-11: it flip-flopped between the nested marker's two IDs whenever both happened to
    decode in the same frame, since their physical sizes differ ~10x, corrupting the banded
    extent averages with an artifact that has nothing to do with corner-position bias). Locks
    onto one ID (by largest spread, tie-break like img_data.py's initial lock) and STAYS locked
    until that ID is absent from a frame's decode -- only then re-locks."""
    det = cv2.aruco.ArucoDetector(ADICT, p)
    ext = np.full(len(frames), np.nan)
    locked_id = None
    for i, g in enumerate(frames):
        corners, ids, _ = det.detectMarkers(g)
        if ids is None or len(ids) == 0:
            continue
        ids_f = ids.flatten()
        if locked_id is None or locked_id not in ids_f:
            spreads = [float(np.std(c.reshape(-1, 2))) for c in corners]
            locked_id = int(ids_f[int(np.argmax(spreads))])
        idx = int(np.where(ids_f == locked_id)[0][0])
        c = corners[idx].reshape(-1, 2)
        ext[i] = float(np.mean(c.max(0) - c.min(0)))
    by_ext = []
    for lo, hi in BANDS:
        m = (alt >= lo) & (alt < hi) & np.isfinite(ext)
        by_ext.append(float(np.mean(ext[m])) if m.sum() >= 3 else np.nan)
    # monotonicity check: near bands (later in list) should be >= far bands (earlier), allowing
    # small noise -- flag any near-band mean that's LOWER than an earlier (farther) band's mean.
    inversions = sum(1 for i in range(1, len(by_ext))
                      if np.isfinite(by_ext[i]) and np.isfinite(by_ext[i-1]) and by_ext[i] < by_ext[i-1] - 1.0)
    n_dec = int(np.isfinite(ext).sum())
    print(f"{label:<38}{n_dec:>5}/{len(frames):<5}" +
          " ".join(f"{v:>6.1f}" if np.isfinite(v) else "     -" for v in by_ext) +
          f"   inversions={inversions}")


def main(raw_dir, gt_dir):
    frames, alt = _load(raw_dir, gt_dir)
    print(f"frames={len(frames)}, alt {alt[0]:.2f}->{alt[-1]:.2f}m   ({os.path.basename(raw_dir)})")
    print(f"{'config':<38}{'n_dec':<11}" + " ".join(f"{lo:.1f}-{hi:.1f}" for lo, hi in BANDS) + "  (mean corner extent px, far->near)")
    row("BASELINE winSize=5 (current)", frames, alt, _params())
    for ws in [11, 15, 21, 31]:
        row(f"  cornerRefinementWinSize={ws}", frames, alt, _params(cornerRefinementWinSize=ws))
    row("  CORNER_REFINE_NONE (control)", frames, alt, _params(cornerRefinementMethod=cv2.aruco.CORNER_REFINE_NONE))
    row("  CORNER_REFINE_CONTOUR (control)", frames, alt, _params(cornerRefinementMethod=cv2.aruco.CORNER_REFINE_CONTOUR))
    print("\n'inversions' = count of near-band mean extent < an earlier (farther) band's mean,")
    print("i.e. the marker appearing to SHRINK while genuinely getting closer. 0 = healthy.")


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print(__doc__); sys.exit(2)
    main(sys.argv[1], sys.argv[2])
