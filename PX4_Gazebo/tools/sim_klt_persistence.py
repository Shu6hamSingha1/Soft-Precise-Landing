#!/usr/bin/env python3
"""Offline simulation of KLT corner-track PERSISTENCE policies — sizes the live
img_data.py change BEFORE SITL (thread project_decode_availability_thread).

The availability dropout is 100% ArUco decode-fail (tune_lk_dynamic_range.py
breakdown). The live fallback already KLT-tracks the last-good corners through a
decode gap, but it (a) requires ALL 4 primary corners to stay in-FoV (one corner
leaving aborts the whole track) and (b) caps at MARKER_KLT_MAX_STEPS=20 frames.

This sim replays the recorded frames with the stateful fallback and compares:
  A_strict : current policy — reseed on decode; else KLT all-4-in-bounds, cap=20.
  B_percorner : per-corner gate — keep in-bounds tracked corners, drop off-screen
                ones, require >= MIN_CORN survivors, cap=CAP. Phantom-clip preserved
                (off-screen corners dropped, never extrapolated).
Metrics per policy: flow AVAILABILITY (% frames with a valid flow) overall and at
close range (<2.5m), and the flow-ratio |meas|/|GT| (does extrapolated flow stay
~1x or drift). Picks the (CAP, MIN_CORN) that maximizes availability without ratio drift.

Usage: python3 tools/sim_klt_persistence.py <raw_dir> <gt_rep_dir>
"""
import sys, os
import numpy as np
import cv2
sys.path.insert(0, os.path.abspath(os.path.dirname(__file__)))
import tune_lk_dynamic_range as T


def _flow(p0, p1, q0, q1, cx, cy, fps):
    """Raw lateral V-flow |V_v[:2]| from a tracked corner pair, or nan if rejected."""
    if len(p0) < 4:
        return np.nan
    V0 = T._virtual_pts(p0, q0, cx, cy); V1 = T._virtual_pts(p1, q1, cx, cy)
    A = T._fill_A(V1); Y = (V1 - V0).reshape(-1) * fps
    Vv, res, rank, sv = np.linalg.lstsq(A, Y, rcond=1e-3)
    cond = (sv[0] / sv[-1]) if (len(sv) and sv[-1] > 0) else np.inf
    if rank < 6 or cond > 1e4 or not np.all(np.isfinite(Vv)) or np.max(np.abs(Vv)) > 50:
        return np.nan
    Vv = np.clip(Vv, -10, 10)
    return np.hypot(Vv[0], Vv[1])


def run(imgs, quats, cx, cy, fps, cap, min_corn, per_corner):
    """Replay frames with the persistence policy. Returns per-frame lateral flow (nan=unavail)."""
    det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(T.ARUCO_DICT),
                                  cv2.aruco.DetectorParameters())
    lk = dict(winSize=(21, 21), maxLevel=3,
              criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01),
              minEigThreshold=1e-3)
    n = len(imgs)
    H, W = imgs[0].shape[:2]
    lat = np.full(n, np.nan)
    prev_pts = None; klt = 0
    for i in range(n):
        g = imgs[i]
        corners, ids, _ = det.detectMarkers(g)
        if ids is not None and len(corners) > 0:                  # ArUco decoded -> reseed
            mc = corners[0][0].reshape(-1, 2).astype(np.float32)
            mask = np.zeros(g.shape[:2], np.uint8)
            cv2.fillConvexPoly(mask, mc.astype(np.int32), 255)
            gf = cv2.goodFeaturesToTrack(g, mask=mask, blockSize=7, **T.GFT)
            cur = np.vstack([mc, gf.reshape(-1, 2)]).astype(np.float32) if gf is not None else mc
            klt = 0
        elif prev_pts is not None and klt < cap and i > 0:        # KLT fallback
            p1, st, _ = cv2.calcOpticalFlowPyrLK(imgs[i-1], g, prev_pts.reshape(-1,1,2), None, **lk)
            if p1 is None or st is None:
                prev_pts = None; continue
            st = st.flatten().astype(bool)
            tr = p1.reshape(-1, 2)
            inb = (tr[:, 0] >= 0) & (tr[:, 0] < W) & (tr[:, 1] >= 0) & (tr[:, 1] < H)
            keep = st & inb
            if per_corner:
                if keep.sum() < min_corn:                          # not enough survivors -> stale
                    prev_pts = None; continue
                cur = tr[keep].astype(np.float32)                  # phantom-clip: drop off-screen
            else:
                if not (st.sum() == len(prev_pts) and inb.all()):  # strict: all in-bounds
                    prev_pts = None; continue
                cur = tr.astype(np.float32)
            klt += 1
        else:
            prev_pts = None; continue
        # flow from previous-good -> current (need the matched pair); use frame i-1 -> i
        if prev_pts is not None and i > 0:
            p1, st, _ = cv2.calcOpticalFlowPyrLK(imgs[i-1], g, prev_pts.reshape(-1,1,2), None, **lk)
            if p1 is not None and st is not None:
                ok = st.flatten().astype(bool)
                if ok.sum() >= 4:
                    lat[i] = _flow(prev_pts[ok], p1.reshape(-1,2)[ok], quats[i-1], quats[i], cx, cy, fps)
        prev_pts = cur
    return lat


def main(raw, rep):
    imgs, stamps, quats, g, (cx, cy), fps = T._load(raw, rep)
    fk = stamps - g['start_time']; alt = np.interp(fk, g['t_g'], np.abs(g['alt']))
    Vh = g['V_h_g']; m = np.isfinite(Vh[:, 0]) & np.isfinite(Vh[:, 1])
    gtlat = np.hypot(np.interp(fk, g['t_g'][m], Vh[m, 0]), np.interp(fk, g['t_g'][m], Vh[m, 1]))
    close = alt < 2.5
    print(f"frames={len(imgs)} fps~{fps:.0f}  (raw={os.path.basename(raw)})")
    print(f"{'policy':>22}{'avail':>8}{'avail<2.5m':>12}{'ratio(when avail)':>20}")
    cfgs = [('A_strict cap20', dict(cap=20, min_corn=4, per_corner=False)),
            ('B_percorner cap20 m4', dict(cap=20, min_corn=4, per_corner=True)),
            ('B_percorner cap40 m4', dict(cap=40, min_corn=4, per_corner=True)),
            ('B_percorner cap60 m4', dict(cap=60, min_corn=4, per_corner=True)),
            ('B_percorner cap60 m2', dict(cap=60, min_corn=2, per_corner=True))]
    for name, kw in cfgs:
        lat = run(imgs, quats, cx, cy, fps, **kw)
        av = np.isfinite(lat)
        sel = av & (gtlat > 1e-3)
        ratio = np.median(lat[sel] / gtlat[sel]) if sel.sum() >= 3 else np.nan
        print(f"{name:>22}{100*av.mean():>7.0f}%{100*(av&close).sum()/max(close.sum(),1):>11.0f}%{ratio:>20.2f}")


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print(__doc__); sys.exit(2)
    main(sys.argv[1], sys.argv[2])
