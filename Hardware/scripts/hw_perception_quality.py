#!/usr/bin/env python3
"""Offline perception-quality scan of recorded hardware landing videos (control-window frames only).
usage: CROSS_DETECTOR=legacy|stroke python hw_perception_quality.py cross <out.npz> DAY...
       python hw_perception_quality.py aruco <out.npz> DAY...
Per frame stores: run, frame, t, alt(EKF), detected, center px, heading (cross), extent, sheet-blob (cross oracle)."""
import sys, os, numpy as np, cv2
from multiprocessing import Pool
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from perception_hw_common import *

def work(args):
    mode, rd, vid = args
    img, ctl, tel = load_run(rd)
    ti = np.asarray(img['Time'], float); tc = np.asarray(ctl['t'], float)
    if len(tc) < 30: return []
    dep, yaw, pos = depth_yaw(tel, ti)
    cap = cv2.VideoCapture(vid); rows = []
    if mode == 'cross':
        import cross_marker_detector as cd
        st = {'last_bbox': None, 'miss_count': 0}
    else:
        ar = cv2.aruco
        det = ar.ArucoDetector(ar.getPredefinedDictionary(ar.DICT_4X4_50), ar.DetectorParameters())
    name = os.path.basename(os.path.normpath(rd))
    for k in range(len(ti)):
        r, f = cap.read()
        if not r: break
        # run tracker through the whole clip (as the live pipeline would), score only the control window
        if mode == 'cross':
            d = cd.detect(f, track_state=st)
            if not (tc[0] <= ti[k] <= tc[-1]): continue
            g = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY); b = paper_blob(g)
            ext = max(d.mask_bbox[2], d.mask_bbox[3]) if (d.ok and d.mask_bbox) else np.nan
            rows.append((name, k, ti[k], dep[k], d.ok, d.center[0] if d.ok and d.center else np.nan, d.center[1] if d.ok and d.center else np.nan,
                         d.heading_deg if d.heading_deg is not None else np.nan, ext, *(b if b else (np.nan,) * 7)))
        else:
            if not (tc[0] <= ti[k] <= tc[-1]): continue
            g = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)
            c, ids, _ = det.detectMarkers(g)
            if ids is not None and len(ids):
                q = c[0].reshape(4, 2); ctr = q.mean(0); ext = np.sqrt(abs(cv2.contourArea(q)))
                rows.append((name, k, ti[k], dep[k], True, ctr[0], ctr[1], np.nan, ext) + (np.nan,) * 7)
            else:
                rows.append((name, k, ti[k], dep[k], False, np.nan, np.nan, np.nan, np.nan) + (np.nan,) * 7)
    return rows

if __name__ == '__main__':
    mode, out, days = sys.argv[1], sys.argv[2], sys.argv[3:]
    jobs = [(mode, rd, v) for d in days for rd, v in pair_runs(d)]
    print(len(jobs), 'runs', flush=True)
    with Pool(max(1, os.cpu_count() - 2)) as p: res = p.map(work, jobs, chunksize=1)
    rows = [r for x in res for r in x]
    np.save(out, np.array(rows, dtype=object), allow_pickle=True); print('frames', len(rows))
