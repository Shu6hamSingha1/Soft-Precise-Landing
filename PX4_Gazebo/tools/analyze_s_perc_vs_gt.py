#!/usr/bin/env python3
"""Perception s (as fed to the control law, Control_Data 's(t)') vs analytic GT s
(tools/gt_optical_flow.compute_gt_flow V_s_g) by altitude band, for GT_ABLATE runs where
s comes from perception (2026-09-24, SPercGTFB_AB).

Alignment: Control_Data 't' is ABSOLUTE; GT 'Time' is 0-based from gt['Start Time'], so
t_ctrl - Start Time is on the GT axis. Altitude = camera->marker depth from compute_gt_flow.

Usage: python3 tools/analyze_s_perc_vs_gt.py <root> [arm_glob]
   e.g. tools/analyze_s_perc_vs_gt.py test_data/SPercGTFB_AB 'sperc_IC*'
"""
import sys, os, glob
import numpy as np
sys.path.insert(0, os.path.dirname(__file__))
from gt_optical_flow import compute_gt_flow

BANDS = [(4, 99), (2, 4), (1, 2), (0.5, 1), (0.3, 0.5), (0, 0.3)]   # altitude bands, m


def rep_err(rep):
    g = compute_gt_flow(rep)
    c = np.load(os.path.join(rep, "Control_Data.npy"), allow_pickle=True).item()
    tc = np.asarray(c['t'], float) - g['start_time']
    s = np.array([np.asarray(x, float)[:2] for x in c['s(t)']])
    n = min(len(tc), len(s)); tc, s = tc[:n], s[:n]
    ok = (tc >= g['t_g'][0]) & (tc <= g['t_g'][-1])
    tc, s = tc[ok], s[ok]
    sg = np.column_stack([np.interp(tc, g['t_g'], g['V_s_g'][:, k]) for k in range(2)])
    alt = np.interp(tc, g['t_g'], g['alt'])
    return tc, alt, s, sg


def main(root, arm_glob="sperc_IC*"):
    reps = sorted(glob.glob(os.path.join(root, arm_glob, "*", "")))
    E = {b: [] for b in BANDS}; per_rep = []
    for rep in reps:
        if not os.path.exists(os.path.join(rep, "Control_Data.npy")): continue
        tc, alt, s, sg = rep_err(rep)
        e = s - sg
        for b in BANDS:
            m = (alt >= b[0]) & (alt < b[1])
            if m.any(): E[b].append(e[m])
        # frozen-s fraction in the last 1 m: consecutive identical samples = stale perception
        m = alt < 1.0
        fr = float(np.mean(np.all(np.diff(s[m], axis=0) == 0, axis=1))) if m.sum() > 2 else np.nan
        per_rep.append((os.path.basename(os.path.dirname(rep.rstrip('/'))) + "/" + os.path.basename(rep.rstrip('/')),
                        float(np.sqrt(np.mean(e[alt < 1.0] ** 2))) if m.any() else np.nan, fr))
    print(f"{len(per_rep)} reps in {root}/{arm_glob}\n")
    print(f"{'alt band (m)':>13}{'N':>7}{'bias_x':>9}{'bias_y':>9}{'rmse_x':>9}{'rmse_y':>9}{'p95|e|':>9}")
    for b in BANDS:
        if not E[b]: continue
        e = np.vstack(E[b])
        print(f"{b[0]:>6}-{b[1]:<6}{len(e):>7}{e[:,0].mean():>9.3f}{e[:,1].mean():>9.3f}"
              f"{np.sqrt((e[:,0]**2).mean()):>9.3f}{np.sqrt((e[:,1]**2).mean()):>9.3f}"
              f"{np.percentile(np.linalg.norm(e, axis=1), 95):>9.3f}")
    print("\nper-rep RMSE(<1 m) and frozen-s fraction (<1 m):")
    for name, r, fr in per_rep:
        print(f"  {name:45s} rmse={r:.3f}  frozen={fr:.2f}")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else "sperc_IC*")
