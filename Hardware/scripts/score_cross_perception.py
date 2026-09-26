#!/usr/bin/env python3
"""Score cross detectors against the image-only visibility oracle (>=2 thin strokes on paper, partial view allowed).
Pipeline (control-window frames only):
  CROSS_DETECTOR=legacy python hw_perception_quality.py cross legacy.npy DAY...
  CROSS_DETECTOR=stroke python hw_perception_quality.py cross stroke.npy DAY...
  python oracle_scan.py oracle.npy            # edit its day list if not 09-24/09-25
  python score_cross_perception.py oracle.npy legacy.npy stroke.npy
Scores only frames the oracle calls visible; visibility itself is the controller's job and is not scored."""
import sys, numpy as np

def score(oracle_path, det_paths):
    O = np.load(oracle_path, allow_pickle=True)
    key = {(r[0], int(r[1])): i for i, r in enumerate(O)}
    ov = O[:, 2].astype(bool); ox = O[:, 3].astype(float); oy = O[:, 4].astype(float)
    print('oracle frames %d, cross visible (>=2 strokes): %d = %.0f%%' % (len(O), ov.sum(), 100 * ov.mean()))
    for path in det_paths:
        a = np.load(path, allow_pickle=True)
        idx = np.array([key.get((r[0], int(r[1])), -1) for r in a]); m = idx >= 0; a = a[m]; idx = idx[m]
        vis = ov[idx]; ok = a[:, 4].astype(bool); cx = a[:, 5].astype(float); cy = a[:, 6].astype(float)
        al = a[:, 7].astype(float); alt = a[:, 3].astype(float); err = np.hypot(cx - ox[idx], cy - oy[idx])
        print('\n%s | visible frames %d' % (path, vis.sum()))
        for b, mk in [('all', np.ones(len(a), bool)), ('>2.5m', alt > 2.5), ('1.2-2.5m', (alt <= 2.5) & (alt > 1.2)), ('<1.2m', alt <= 1.2)]:
            s = vis & mk
            if s.sum() < 20: continue
            print('  %-9s n=%4d  within 12px %.0f%% | within 25px %.0f%% | wrong >25px %.0f%% | no detection %.0f%%' % (
                b, s.sum(), 100 * (ok & (err < 12))[s].mean(), 100 * (ok & (err < 25))[s].mean(),
                100 * (ok & (err >= 25))[s].mean(), 100 * (~ok)[s].mean()))
        good = vis & ok & (err < 25)
        if good.sum() > 5:
            print('  hit centre error: median %.1f px, 90th %.1f px | heading given on %.0f%% of hits' % (
                np.median(err[good]), np.percentile(err[good], 90), 100 * np.isfinite(al[good]).mean()))
        print('  spurious detections on non-visible frames: %.0f%%' % (100 * ok[~vis].mean()))

if __name__ == '__main__':
    if len(sys.argv) < 3: print(__doc__); sys.exit(2)
    score(sys.argv[1], sys.argv[2:])
