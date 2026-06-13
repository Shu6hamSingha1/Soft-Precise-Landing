#!/usr/bin/env python3
"""Offline ArUco-decode param tuner. OBJECTIVE: maximize DECODE RATE (decode is the
gate on corner survival — survival is decode-gated, not LK-gated; see tune_lk_survival.py).

Runs cv2.aruco.detectMarkers on LOSSLESS recorded frames (IMG_RECORD_RAW=1) with swept
DetectorParameters, reports decode% overall and by altitude band. Coordinate-style: varies
one param at a time from the current img_data baseline. No SITL, no recalibration needed
(decode is upstream of the flow/cal).

Baseline (img_data.py): adaptiveThreshConstant=5, errorCorrectionRate=0.8,
minMarkerPerimeterRate=0.02, minOtsuStdDev=3, maxMarkerPerimeterRate=4.0(default),
cornerRefine=SUBPIX. Defaults for the rest.

Usage: python3 tools/tune_aruco_decode.py <raw_frame_dir> <gt_rep_dir>
"""
import sys, os, glob
import numpy as np
import cv2

ADICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
BANDS = [(4.0, 5.2), (3.0, 4.0), (2.0, 3.0), (1.0, 2.0), (0.5, 1.0), (0.0, 0.5)]


def _params(**ov):
    p = cv2.aruco.DetectorParameters()
    # img_data baseline
    p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
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


def decode_rate(frames, p):
    det = cv2.aruco.ArucoDetector(ADICT, p)
    return np.array([0 if det.detectMarkers(g)[1] is None else len(det.detectMarkers(g)[1]) for g in frames])


def row(label, frames, alt, p):
    dec = decode_rate(frames, p)
    by = []
    for lo, hi in BANDS:
        m = (alt >= lo) & (alt < hi)
        by.append(f"{100*np.mean(dec[m]>0):>4.0f}" if m.sum() else "   -")
    print(f"{label:<34}{100*np.mean(dec>0):>6.0f}  |" + " ".join(by))


def main(raw_dir, gt_dir):
    frames, alt = _load(raw_dir, gt_dir)
    print(f"frames={len(frames)}, alt {alt[0]:.1f}->{alt[-1]:.2f}m   ({os.path.basename(raw_dir)})")
    print(f"{'config':<34}{'all%':>6}  | " + " ".join(f"{lo:.0f}-{hi:.0f}" for lo, hi in BANDS) + " m")
    row("BASELINE (img_data)", frames, alt, _params())
    # coordinate sweep — one param at a time
    sweep = [
        ("errorCorrectionRate", [0.6, 1.0, 1.2]),
        ("adaptiveThreshConstant", [3.0, 7.0, 9.0]),
        ("adaptiveThreshWinSizeMin", [3, 5]),
        ("adaptiveThreshWinSizeMax", [23, 53, 103]),
        ("adaptiveThreshWinSizeStep", [4, 10, 20]),
        ("polygonalApproxAccuracyRate", [0.03, 0.05, 0.08]),
        ("minMarkerPerimeterRate", [0.01, 0.03]),
        ("maxMarkerPerimeterRate", [4.0, 8.0]),
        ("perspectiveRemoveIgnoredMarginPerCell", [0.13, 0.20, 0.33]),
        ("minCornerDistanceRate", [0.05, 0.10]),
    ]
    for name, vals in sweep:
        for v in vals:
            row(f"  {name}={v}", frames, alt, _params(**{name: v}))


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print(__doc__); sys.exit(2)
    main(sys.argv[1], sys.argv[2])
