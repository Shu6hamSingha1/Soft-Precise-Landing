#!/usr/bin/env python3
"""Per-frame perception-s vs GT-s error for a GT-FB rover rep, mapped to its IMG_RECORD raw
frame dir (2026-09-24). Raw frames are saved one per processed frame from CONTROLLER_READY
(= GT 'Start Time'), so frame k <-> the k-th Img_Data sample with Time >= Start Time.
Usage: PLASMC_GT_MARKER_DZ=0.5 python3 tools/rover_s_err_frames.py <rep_dir> <raw_dir> [out.png]"""
import sys, os, glob
import numpy as np, cv2
sys.path.insert(0, os.path.dirname(__file__))
from gt_optical_flow import compute_gt_flow
rep, raw = sys.argv[1], sys.argv[2]
out = sys.argv[3] if len(sys.argv) > 3 else None
G = compute_gt_flow(rep)
d = np.load(os.path.join(rep, "Img_Data.npy"), allow_pickle=True).item()
ti = np.asarray(d['Time']) - G['start_time']
sel = np.where(ti >= 0)[0]
fr = sorted(glob.glob(os.path.join(raw, "f*.png")))
sv = np.asarray(d['s_V'])[:, :2] * np.array([0.9574, 0.9503])
sg = np.column_stack([np.interp(ti, G['t_g'], G['V_s_g'][:, k]) for k in range(2)])
alt = np.interp(ti, G['t_g'], G['alt'])
su = sg * ((np.maximum(alt, 0) + 0.2) / np.maximum(alt, 1e-3))[:, None]
st = np.asarray(d['Fail Reason']); cpx = np.asarray(d['Center Px'])
print(f"{len(fr)} raw frames, {len(sel)} Img_Data samples after start")
rows = []
for k, i in enumerate(sel[:len(fr)]):
    e = float(np.linalg.norm(sv[i] - su[i]))
    rows.append((k, ti[i], alt[i], st[i], e, sv[i], su[i], cpx[i]))
for r in rows[::max(1, len(rows) // 40)]:
    k, t, a, s_, e, p, g, c = r
    print(f"f{k:05d} t={t:5.2f} alt={a:4.2f} {s_:>20s} err={e:.3f} perc=({p[0]:+.2f},{p[1]:+.2f}) gt=({g[0]:+.2f},{g[1]:+.2f}) cpx=({c[0]:.0f},{c[1]:.0f})")
if out:
    # montage of the worst 'ok' frames (confident-wrong detections)
    okr = [r for r in rows if r[3] == 'ok']
    worst = sorted(okr, key=lambda r: -r[4])[:6]
    tiles = []
    for r in sorted(worst):
        im = cv2.imread(fr[r[0]]); c = r[7]
        if np.all(np.isfinite(c)): cv2.drawMarker(im, (int(c[0]), int(c[1])), (0, 0, 255), cv2.MARKER_CROSS, 20, 2)
        cv2.putText(im, f"f{r[0]} a{r[2]:.1f} e{r[4]:.2f}", (3, 12), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
        tiles.append(im)
    cv2.imwrite(out, np.hstack(tiles))
    print("worst ok frames:", [(r[0], round(r[4], 3)) for r in sorted(worst)], "->", out)
