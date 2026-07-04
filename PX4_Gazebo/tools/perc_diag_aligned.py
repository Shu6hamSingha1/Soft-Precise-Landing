#!/usr/bin/env python3
"""Sim-time-aligned perception-vs-GT diagnostic (methodology-correct).

Two hard-won rules baked in (2026-07-04):
  1. Pose log (Ground_Truth) and image log (Img_Data) are on DIFFERENT clocks:
     pose sim-time = gt['Start Time'] + gt['Time'];  image sim-time = d['Time'].
     Align by NEAREST SIM-TIME, never by index.
  2. Only use PERCEPTION-DRIVEN runs (no PLASMC_GT_FEEDBACK). GT-FB freezes the
     shadow perception features, so its logged Feature Params are meaningless.

Usage: perc_diag_aligned.py <Landing_Test/rec_dir>
Reports, around the drift onset, the LIVE perception vs GT for s_xy / h_xy / loom.
"""
import sys, os, numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
from gt_feedback import GTFeedback

R = sys.argv[1]
gt = np.load(os.path.join(R, 'Ground_Truth.npy'), allow_pickle=True).item()
d = np.load(os.path.join(R, 'Img_Data.npy'), allow_pickle=True).item()
tG = gt['Start Time'] + np.asarray(gt['Time']).ravel()      # pose sim-time
tI = np.asarray(d['Time']).ravel()                          # image sim-time
U = np.array(gt['UAV Pose'], dtype=object); T = np.array(gt['Target Pose'], dtype=object)
ps = np.asarray(d['Feature Params']); ph = np.asarray(d['Opt Flow Ang Vel'])
nG = min(len(U), len(T), len(tG))
gf = GTFeedback()

rows = []
for i in range(nG):
    j = int(np.argmin(np.abs(tI - tG[i])))
    if abs(tI[j] - tG[i]) > 0.1:      # no image within 100 ms -> skip
        continue
    try: s4, f6 = gf.update(U[i], T[i], float(tG[i]))
    except Exception: continue
    lat = np.hypot(U[i].position.x - T[i].position.x, U[i].position.y - T[i].position.y)
    rows.append((tG[i], lat, s4[0], s4[1], f6[0], f6[1], f6[2],      # GT: s_x s_y h_x h_y loom
                 ps[j, 0], ps[j, 1], ph[j, 0], ph[j, 1], ph[j, 2]))  # perc same order
a = np.array(rows)
if len(a) < 10:
    print('too few aligned samples'); sys.exit()
t, lat = a[:, 0], a[:, 1]
peak = lat.max()
# drift onset: first sim-time where lat first exceeds 0.20 on the way out
onset = next((k for k in range(2, len(a)) if lat[k] > 0.20 and lat[k] > lat[k-1]), None)
print('rec %s | peak=%.2f | %s' % (os.path.basename(R.rstrip('/'))[-13:], peak,
      'FAIL' if peak > 0.5 else 'ok'))
if onset:
    print('  drift onset: simT=%.1f lat=%.2f' % (t[onset], lat[onset]))
lab = ['s_x', 's_y', 'h_x', 'h_y', 'loom']
print('  %-6s | %8s %8s %8s' % ('chan', 'GTmean', 'PERCmean', 'corr(off-center)'))
m = lat > 0.20
for k, name in enumerate(lab):
    g = a[m, 2 + k]; p = a[m, 7 + k]
    c = np.corrcoef(g, p)[0, 1] if (m.sum() > 8 and np.std(p) > 1e-6 and np.std(g) > 1e-6) else float('nan')
    print('  %-6s | %+8.3f %+8.3f   %+.2f' % (name, np.mean(g), np.mean(p), c))
