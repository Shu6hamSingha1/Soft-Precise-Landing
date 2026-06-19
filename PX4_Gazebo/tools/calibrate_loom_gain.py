#!/usr/bin/env python3
"""Calibrate FLOW_LOOM_GAIN — the one scalar on the decoupled-loom estimator
(feedback_pinv_tol_loom_scaling). Replicates the RUNTIME estimator exactly
(img_data.py FLOW_LOOM_DECOUPLE path): on the primary marker (smallest ID),
M = μ20+μ02 = trace of the de-rotated V-frame corner scatter; loom_raw =
-½·d(ln M)/dt via a causal FLOW_LOOM_WIN-frame linear fit over a (stamp, ln M)
deque. Fits gain = argmin ||gain·loom_raw - GT_loom|| (LS through origin) aggregated
across recordings, and reports the per-altitude trend (FoV-overflow check).

Usage: python3 tools/calibrate_loom_gain.py <raw1>::<rep1> [<raw2>::<rep2> ...]
  each arg = "<raw_frame_dir>::<gt_rep_dir>"
"""
import sys, os, warnings
warnings.filterwarnings('ignore')
import numpy as np
import cv2
from collections import deque
sys.path.insert(0, os.path.abspath(os.path.dirname(__file__)))
import tune_lk_dynamic_range as T

WIN = int(os.environ.get("FLOW_LOOM_WIN", "9"))
_LAYOUT_PATH = os.path.join(os.path.dirname(__file__), '..', 'Images', 'aruco_board_layout.npy')
_LAY = np.load(_LAYOUT_PATH, allow_pickle=True).item() if os.path.exists(_LAYOUT_PATH) else {}
_SZ = {k: float(_LAY[k][2]) for k in _LAY}   # marker physical size for M/sz² normalization


def _mtrace(p):
    c = p.mean(0)
    return float(np.mean(np.sum((p - c) ** 2, axis=1)))


def series(raw, rep):
    """Returns (raw_loom, GT_loom, alt) aligned per frame, primary = smallest ID."""
    imgs, stamps, quats, g, (cx, cy), fps = T._load(raw, rep)
    det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(T.ARUCO_DICT),
                                  cv2.aruco.DetectorParameters())
    n = len(imgs); fk = stamps - g['start_time']
    hist = deque(maxlen=WIN); rl = np.full(n, np.nan)
    for i in range(n):
        c, ids, _ = det.detectMarkers(imgs[i])
        if ids is None:
            continue
        idl = ids.flatten().tolist()
        pid = min(idl)                                     # primary = smallest ID (runtime convention)
        mc = c[idl.index(pid)][0].reshape(-1, 2).astype(np.float32)
        M = _mtrace(T._virtual_pts(mc, quats[i], cx, cy))
        sz = _SZ.get(pid, 1.0) or 1.0
        M = M / (sz * sz)                                  # size-normalize: M/sz²=(f/Z)², switch-continuous
        if M <= 1e-12:
            continue
        hist.append((stamps[i], np.log(M)))
        if len(hist) >= 3:
            ta = np.array([h[0] for h in hist]); la = np.array([h[1] for h in hist])
            if ta.max() - ta.min() > 1e-4:
                rl[i] = np.clip(-0.5 * np.polyfit(ta - ta[0], la, 1)[0], -10, 10)
    glm = np.isfinite(g['loom'])
    gl = np.interp(fk, g['t_g'][glm], g['loom'][glm])
    alt = np.interp(fk, g['t_g'], np.abs(g['alt']))
    m = np.isfinite(rl) & np.isfinite(gl)
    return rl[m], gl[m], alt[m]


def main(pairs):
    R, G, A = [], [], []
    for p in pairs:
        raw, rep = p.split("::")
        r, gl, a = series(raw, rep)
        gain = float((r @ gl) / (r @ r)) if (r @ r) > 0 else np.nan
        print(f"{os.path.basename(rep):28} n={len(r):4d} alt {a.min():.2f}-{a.max():.2f}m  "
              f"per-rec gain {gain:.3f}  rmse@1.0 {np.sqrt(np.mean((r-gl)**2)):.3f}")
        R.append(r); G.append(gl); A.append(a)
    r = np.concatenate(R); gl = np.concatenate(G); a = np.concatenate(A)
    gain = float((r @ gl) / (r @ r))
    print(f"\nAGGREGATE n={len(r)}  GLOBAL best-fit FLOW_LOOM_GAIN = {gain:.3f}")
    print(f"  rmse  gain=1.0: {np.sqrt(np.mean((r-gl)**2)):.3f}   gain={gain:.2f}: {np.sqrt(np.mean((gain*r-gl)**2)):.3f}")
    print("  per-altitude gain (multiplier on raw to match GT):")
    for lo, hi in [(2.0, 3.0), (1.5, 2.0), (1.0, 1.5), (0.5, 1.0), (0.0, 0.5)]:
        s = (a >= lo) & (a < hi)
        if s.sum() >= 4:
            gb = float((r[s] @ gl[s]) / (r[s] @ r[s]))
            print(f"    alt[{lo},{hi}) n={s.sum():4d}  gain {gb:.2f}  |GT|~{np.median(np.abs(gl[s])):.2f}")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__); sys.exit(2)
    main(sys.argv[1:])
