#!/usr/bin/env python3
"""Offline LK-param tuner — OBJECTIVE: maximize CORNER SURVIVAL (not GT accuracy).

Rationale (2026-06-13, user): the corner flow is sensor-calibrated, so absolute
scale/accuracy is the CALIBRATION's job — changing LK params makes the existing
sensor_cal stale (must be redone). So LK tuning must NOT target GT-loom accuracy;
it must keep more corners tracked, DEEPER into the descent (lower death altitude),
through close-range fast motion + ArUco decode failures. Scale is recalibrated after.

Replicates the live corner pipeline (img_data.py): ArUco decode -> seed marker
corners + goodFeaturesToTrack masked to the marker polygon; on decode-fail, KLT
fallback (LK-track the previous corners forward, up to MARKER_KLT_MAX_STEPS).
Sweeps winSize x maxLevel (+ optional GFT) on LOSSLESS recorded frames and reports
survival per config: median n_corn, the altitude at which n_corn->0, and % frames alive.

Usage:
    python3 tools/tune_lk_survival.py <raw_frame_dir> <gt_rep_dir>
      raw_frame_dir : test_data/Test_Videos/<ts>_raw  (PNG frames + stamps.npy from IMG_RECORD_RAW=1)
      gt_rep_dir    : the matching rep dir with Ground_Truth.npy (for the altitude axis)
"""
import sys, os, glob
import numpy as np
import cv2

ARUCO_DICT = cv2.aruco.DICT_4X4_50
GFT = dict(maxCorners=int(os.environ.get("IMG_GFT_MAX", "200")),
           qualityLevel=float(os.environ.get("IMG_GFT_QUALITY", "0.01")),
           minDistance=int(os.environ.get("IMG_GFT_MINDIST", "5")))
MAX_KLT = int(os.environ.get("MARKER_KLT_MAX_STEPS", "20"))


def _load(raw_dir, gt_dir):
    frames = sorted(glob.glob(os.path.join(raw_dir, "f*.png")))
    imgs = [cv2.imread(f, cv2.IMREAD_GRAYSCALE) for f in frames]
    stamps = np.load(os.path.join(raw_dir, "stamps.npy"))
    gt = np.load(os.path.join(gt_dir, "Ground_Truth.npy"), allow_pickle=True).item()
    St = float(gt['Start Time']); tg = np.asarray(gt['Time'], float)
    u, tp = gt['UAV Pose'], gt['Target Pose']
    n = min(len(tg), len(u), len(tp))
    alt_g = np.array([ (tp[i].position.z - u[i].position.z) for i in range(n) ])  # ENU z = altitude (rel)
    # frame altitude via stamp alignment (stamp - Start Time -> GT time)
    ft = stamps - St
    alt = np.interp(ft, tg[:n], np.abs(alt_g))
    return imgs, alt


def survival(imgs, win, lvl, gft=GFT):
    lk = dict(winSize=(win, win), maxLevel=lvl,
              criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01),
              minEigThreshold=1e-3)
    det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(ARUCO_DICT),
                                  cv2.aruco.DetectorParameters())
    ncorn = np.zeros(len(imgs), int)
    pts = None; klt = 0
    for i, g in enumerate(imgs):
        if g is None:
            ncorn[i] = 0; continue
        corners, ids, _ = det.detectMarkers(g)
        if ids is not None and len(corners) > 0:                     # ArUco decoded -> re-seed
            mc = corners[0][0].reshape(-1, 2).astype(np.float32)
            mask = np.zeros(g.shape[:2], np.uint8)
            cv2.fillConvexPoly(mask, mc.astype(np.int32), 255)
            gf = cv2.goodFeaturesToTrack(g, mask=mask, blockSize=7, **gft)
            pts = np.vstack([mc, gf.reshape(-1, 2)]) if gf is not None else mc
            pts = pts.astype(np.float32); klt = 0
            ncorn[i] = len(pts)
        elif pts is not None and len(pts) >= 1 and klt < MAX_KLT and i > 0:   # KLT fallback (LK forward)
            p1, st, _ = cv2.calcOpticalFlowPyrLK(imgs[i-1], g, pts.reshape(-1,1,2), None, **lk)
            if p1 is None or st is None:
                ncorn[i] = 0; pts = None; continue
            st = st.flatten().astype(bool)
            pts = p1.reshape(-1, 2)[st].astype(np.float32)
            klt += 1
            ncorn[i] = len(pts)
            if len(pts) < 1:
                pts = None
        else:
            ncorn[i] = 0; pts = None
    return ncorn


def main(raw_dir, gt_dir):
    imgs, alt = _load(raw_dir, gt_dir)
    print(f"frames={len(imgs)}, alt {alt[0]:.1f} -> {alt[-1]:.2f} m  (raw_dir={os.path.basename(raw_dir)})")
    print(f"sweep: winSize x maxLevel  (GFT max={GFT['maxCorners']} q={GFT['qualityLevel']} d={GFT['minDistance']}, MAX_KLT={MAX_KLT})\n")
    print(f"{'win':>4}{'lvl':>4}{'med_ncorn':>10}{'death_alt':>10}{'%alive':>8}{'alive<1.5m':>11}")
    base = None
    for win in [7, 11, 15, 21, 31]:
        for lvl in [3, 4, 5]:
            nc = survival(imgs, win, lvl)
            alive = nc > 0
            # death altitude = lowest altitude still alive (deeper = better)
            death_alt = alt[alive].min() if alive.any() else alt[0]
            pct = 100.0 * alive.mean()
            low = (alive & (alt < 1.5)).sum()                       # frames alive below 1.5 m
            tag = "  <- default" if (win == 21 and lvl == 3) else ""
            print(f"{win:>4}{lvl:>4}{np.median(nc[alive]) if alive.any() else 0:>10.0f}"
                  f"{death_alt:>10.2f}{pct:>8.0f}{low:>11d}{tag}")


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print(__doc__); sys.exit(2)
    main(sys.argv[1], sys.argv[2])
