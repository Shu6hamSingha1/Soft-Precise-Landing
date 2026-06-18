#!/usr/bin/env python3
"""Offline LK DYNAMIC-RANGE tuner — OBJECTIVE: does the corner optical-flow stay
PROPORTIONAL to ground-truth lateral velocity at HIGH speed, or SATURATE to ~0?

This is the metric the lateral wall needs (2026-06-19) — NOT corner survival
(tune_lk_survival.py). The combined-barrier IC2 A/B verdict showed the s_dot
feeding h_d under-reports the dominant lateral velocity ~0.5x during the
high-velocity overshoot (LK dynamic-range ceiling ~2 m/s). The documented lever
is pyramidal LK levels 3->4/5 + larger search window. This harness measures, on
LOSSLESS frames recorded through a high-lateral-velocity fly-away, the LATERAL
flow magnitude vs GT (gt_optical_flow) BINNED BY GT SPEED for each (winSize x
maxLevel) config: which config keeps |meas|/|GT| near 1 at >2 m/s instead of
collapsing.

Faithfully replicates the live corner-flow pipeline (img_data.py 791-976):
  ArUco decode -> 4 primary corners + goodFeaturesToTrack inside the marker poly
  -> calcOpticalFlowPyrLK frame_{i-1}->frame_i -> _getVirtualPts (V-frame level)
  -> _fill_A interaction matrix -> lstsq -> V_v[:2] lateral flow (raw, pre-cal).
Ratio is cal-invariant, so we report RAW lstsq flow (saturation lives there).

Usage:
    python3 tools/tune_lk_dynamic_range.py <raw_frame_dir> <gt_rep_dir>
      raw_frame_dir : test_data/Test_Videos/<ts>_raw   (f*.png + stamps.npy, IMG_RECORD_RAW=1)
      gt_rep_dir    : matching rep dir (Ground_Truth.npy + Img_Data.npy for Quat)
"""
import sys, os, glob
import numpy as np
import cv2
from ahrs import Quaternion

sys.path.insert(0, os.path.join(os.path.dirname(__file__)))
from gt_optical_flow import compute_gt_flow

ARUCO_DICT = cv2.aruco.DICT_4X4_50
FX = FY = 270.0
GFT = dict(maxCorners=int(os.environ.get("IMG_GFT_MAX", "200")),
           qualityLevel=float(os.environ.get("IMG_GFT_QUALITY", "0.01")),
           minDistance=int(os.environ.get("IMG_GFT_MINDIST", "5")))


def _virtual_pts(pts, quat_wxyz, cx, cy):
    """Port of img_data._getVirtualPts: reproject pixels onto the gravity-leveled
    V-frame image plane. quat_wxyz = (w,x,y,z)."""
    R = Quaternion(list(quat_wxyz)).to_DCM()
    g = R.T @ np.array([0., 0., 1.])
    z = g / np.linalg.norm(g)
    x = np.cross([0., 1., 0.], z); x /= np.linalg.norm(x)
    y = np.cross(z, x)
    C_R_V = np.column_stack([x, y, z])
    xr = (pts[:, 0] - cx) / FX
    yr = (pts[:, 1] - cy) / FY
    rays = np.column_stack([xr, yr, np.ones_like(xr)])
    Vr = rays @ C_R_V
    zv = Vr[:, 2]
    return np.column_stack([Vr[:, 0] / zv, Vr[:, 1] / zv])


def _fill_A(p):
    """Port of img_data._fill_A: (2N x 6) IBVS interaction matrix."""
    x, y = p[:, 0], p[:, 1]
    N = len(x); A = np.zeros((2 * N, 6))
    A[0::2, 0] = 1;          A[1::2, 1] = 1
    A[0::2, 2] = -x;         A[1::2, 2] = -y
    A[0::2, 3] = -x * y;     A[1::2, 3] = -(1 + y**2)
    A[0::2, 4] = 1 + x**2;   A[1::2, 4] = x * y
    A[0::2, 5] = -y;         A[1::2, 5] = x
    return A


def _load(raw_dir, gt_dir):
    frames = sorted(glob.glob(os.path.join(raw_dir, "f*.png")))
    imgs = [cv2.imread(f, cv2.IMREAD_GRAYSCALE) for f in frames]
    stamps = np.asarray(np.load(os.path.join(raw_dir, "stamps.npy")), float)
    img = np.load(os.path.join(gt_dir, "Img_Data.npy"), allow_pickle=True).item()
    ist = np.asarray(img['Image Stamp'], float); quat = np.asarray(img['Quat'], float)
    # nearest-stamp quaternion per frame
    qi = np.array([np.argmin(np.abs(ist - s)) for s in stamps])
    quats = quat[qi]                                          # (Nf,4) w,x,y,z
    g = compute_gt_flow(gt_dir)
    H, W = imgs[0].shape[:2]
    return imgs, stamps, quats, g, (W / 2.0, H / 2.0), float(np.median(1.0 / np.diff(stamps)))


def measure(imgs, quats, win, lvl, cx, cy, fps):
    """Returns per-frame raw lateral V-flow |V_v[:2]| (nan where unavailable)."""
    lk = dict(winSize=(win, win), maxLevel=lvl,
              criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01),
              minEigThreshold=1e-3)
    det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(ARUCO_DICT),
                                  cv2.aruco.DetectorParameters())
    n = len(imgs)
    latflow = np.full(n, np.nan); ncorn = np.zeros(n, int)
    for i in range(1, n):
        g0, g1 = imgs[i - 1], imgs[i]
        if g0 is None or g1 is None:
            continue
        corners, ids, _ = det.detectMarkers(g0)
        if ids is None or len(corners) == 0:
            continue
        mc = corners[0][0].reshape(-1, 2).astype(np.float32)
        mask = np.zeros(g0.shape[:2], np.uint8)
        cv2.fillConvexPoly(mask, mc.astype(np.int32), 255)
        gf = cv2.goodFeaturesToTrack(g0, mask=mask, blockSize=7, **GFT)
        p0 = np.vstack([mc, gf.reshape(-1, 2)]).astype(np.float32) if gf is not None else mc
        p1, stt, _ = cv2.calcOpticalFlowPyrLK(g0, g1, p0.reshape(-1, 1, 2), None, **lk)
        if p1 is None or stt is None:
            continue
        ok = stt.flatten().astype(bool)
        if ok.sum() < 4:
            continue
        a0 = p0[ok]; a1 = p1.reshape(-1, 2)[ok]
        V0 = _virtual_pts(a0, quats[i - 1], cx, cy)
        V1 = _virtual_pts(a1, quats[i], cx, cy)
        A = _fill_A(V1)
        Y = (V1 - V0).reshape(-1) * fps
        Vv, res, rank, sv = np.linalg.lstsq(A, Y, rcond=1e-3)
        cond = (sv[0] / sv[-1]) if (len(sv) and sv[-1] > 0) else np.inf
        if rank < 6 or cond > 1e4 or not np.all(np.isfinite(Vv)) or np.max(np.abs(Vv)) > 50:
            continue
        Vv = np.clip(Vv, -10, 10)
        latflow[i] = np.hypot(Vv[0], Vv[1])
        ncorn[i] = ok.sum()
    return latflow, ncorn


def main(raw_dir, gt_dir):
    imgs, stamps, quats, g, (cx, cy), fps = _load(raw_dir, gt_dir)
    # GT lateral flow magnitude per frame (align GT to frame stamps via Start Time)
    tg = g['t_g']; Vh = g['V_h_g']
    fk = stamps - g['start_time']
    gtlat = np.full(len(stamps), np.nan)
    m = np.isfinite(Vh[:, 0]) & np.isfinite(Vh[:, 1])
    gx = np.interp(fk, tg[m], Vh[m, 0]); gy = np.interp(fk, tg[m], Vh[m, 1])
    gtlat = np.hypot(gx, gy)
    # also GT lateral SPEED (m/s) for binning: |V_h| * |alt|
    altf = np.interp(fk, tg, np.abs(g['alt']))
    gtspeed = gtlat * altf
    print(f"frames={len(imgs)} fps~{fps:.0f} cx,cy=({cx:.0f},{cy:.0f})  "
          f"GT speed peak {np.nanmax(gtspeed):.1f} m/s  (raw={os.path.basename(raw_dir)})")
    bins = [(0, 1), (1, 2), (2, 4), (4, 99)]
    hdr = "".join(f"  v[{a}-{b})" for a, b in bins)
    print(f"{'win':>4}{'lvl':>4}{'med_nc':>7}{'%track':>7}" + hdr + "   (ratio |meas|/|GT| per GT-speed bin)")
    for win in [21, 31, 41, 51]:
        for lvl in [3, 4, 5]:
            lat, nc = measure(imgs, quats, win, lvl, cx, cy, fps)
            tracked = np.isfinite(lat)
            row = f"{win:>4}{lvl:>4}{int(np.median(nc[tracked])) if tracked.any() else 0:>7}{100*tracked.mean():>6.0f}%"
            for a, b in bins:
                sel = tracked & (gtspeed >= a) & (gtspeed < b) & (gtlat > 1e-3)
                if sel.sum() >= 3:
                    ratio = np.median(lat[sel] / gtlat[sel])
                    row += f"{ratio:>8.2f}"
                else:
                    row += f"{'-':>8}"
            tag = "  <-default" if (win == 21 and lvl == 3) else ""
            print(row + tag)


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print(__doc__); sys.exit(2)
    main(sys.argv[1], sys.argv[2])
