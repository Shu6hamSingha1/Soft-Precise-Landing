#!/usr/bin/env python3
"""Score cross-marker perception vs GT by WHERE the true marker centre sits relative to the frame edge
(in frame / outside by N px). Answers: are off-frame (partially visible) crosses handled?

Usage (repo root, python env with ahrs + scipy):
    python3 tools/score_partial_visibility.py [test_data_root=test_data] [MARKER_DZ=0.5]
Steps: (1) scan every rep with Ground_Truth + Img_Data ('Center Px', 'Detection Status', 's_V'),
(2) robust-fit GT V-frame s_true -> detector pixel (cross-marker cam 320x240 rotated: 240w x 320h),
(3) keep reps whose geometry fits (median centre resid < 8 px; others mix marker heights/worlds),
(4) table by distance the GT centre lies outside the frame.
Note: ignores camera tilt in the s->pixel map (fine for the small tilts here). Marker dz: 0.5 rover worlds,
0.0 flat worlds, 0.201 rover_cross_deck (see PerceptionEvalSet MANIFEST). Reps of other worlds fail the fit and drop out.
On Windows the pickled ROS msgs need a stub module; on Ubuntu with ROS sourced it just works."""
import sys, os, glob
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gt_optical_flow import compute_gt_flow

root = sys.argv[1] if len(sys.argv) > 1 else 'test_data'
mdz = float(sys.argv[2]) if len(sys.argv) > 2 else 0.5
W, H, F = 240, 320, 135.0
R = []
for d in sorted(set(os.path.dirname(p) for p in glob.glob(root + '/**/Ground_Truth.npy', recursive=True))):
    try:
        i = np.load(d + '/Img_Data.npy', allow_pickle=True).item()
        if 'Center Px' not in i or 'Detection Status' not in i:
            continue
        g = compute_gt_flow(d, marker_dz=mdz)
    except Exception:
        continue
    ti = np.asarray(i['Time'], float) - g['start_time']
    n = min(len(ti), len(i['Detection Status']), len(i['Center Px']), len(i['s_V']))
    ti = ti[:n]
    sel = (ti >= g['t_g'][0]) & (ti <= g['t_g'][-1])
    ok = np.array(i['Detection Status'])[:n] == 'ok'
    cp = np.asarray(i['Center Px'], float)[:n]
    sm = np.asarray(i['s_V'], float)[:n]
    st = np.column_stack([np.interp(ti, g['t_g'], g['V_s_true'][:, k]) for k in range(2)])
    alt = np.interp(ti, g['t_g'], g['alt'])
    for k in np.where(sel)[0]:
        R.append((d, ok[k], cp[k, 0], cp[k, 1], sm[k, 0], sm[k, 1], st[k, 0], st[k, 1], alt[k]))
A = np.array(R, dtype=object)
recs = A[:, 0]
ok = A[:, 1].astype(bool)
cpx, cpy = A[:, 2].astype(float), A[:, 3].astype(float)
sm = A[:, 4:6].astype(float)
st = A[:, 6:8].astype(float)
alt = A[:, 8].astype(float)
X = np.column_stack([st, np.ones(len(st))])
m = ok & (np.abs(st[:, 0]) < .6) & (np.abs(st[:, 1]) < .6) & np.isfinite(cpx)
for _ in range(8):
    cx = np.linalg.lstsq(X[m], cpx[m], rcond=None)[0]
    cy = np.linalg.lstsq(X[m], cpy[m], rcond=None)[0]
    e = np.hypot(cpx - X @ cx, cpy - X @ cy)
    m = m & (e < np.maximum(np.percentile(e[m], 60), 6))
gx, gy = X @ cx, X @ cy
err = np.hypot(cpx - gx, cpy - gy)
print('fit px_x=%.1f sx%+.1f sy%+.1f | px_y=%.1f sx%+.1f sy%+.1f ; inlier resid median %.1f px' % (*cx, *cy, np.median(e[m])))
mid = (np.abs(st[:, 0]) < .6) & (np.abs(st[:, 1]) < .6)
good = [r for r in set(recs) if ((recs == r) & ok & mid).sum() > 20 and np.median(err[(recs == r) & ok & mid]) < 8]
G = np.isin(recs, good)
print('consistent reps: %d of %d' % (len(good), len(set(recs))))
d = np.maximum.reduce([-gx, gx - W, -gy, gy - H, np.zeros_like(gx)])
es = np.hypot(*(sm - st).T) * F
print('%-16s %6s %9s %14s %14s' % ('GT centre', 'n', 'detected', 'centre err px', 's err px-equiv'))
for nm, lo, hi in [('in frame', -1, .01), ('out 0-20px', .01, 20), ('out 20-40px', 20, 40), ('out 40-80px', 40, 80), ('out >80px', 80, 1e9)]:
    s = G & (d > lo) & (d <= hi) & (alt > .3)
    o = s & ok
    if s.sum() < 5:
        print('%-16s n=%d' % (nm, s.sum()))
        continue
    print('%-16s %6d %8.0f%% %14.1f %14.1f' % (nm, s.sum(), 100 * ok[s].mean(),
          np.median(err[o]) if o.any() else np.nan, np.median(es[o]) if o.any() else np.nan))
