"""Shared loaders for offline hardware perception-quality analysis (cross vs ArUco).
Reference = Control_Data s(t)/h(t) under PLASMC_HW_POS_FEEDBACK (analytic, EKF pose + snapshotted marker).
Frame k of Test_Videos/*.mp4 == Img_Data index k (IMG_RECORD=1 writes every processed frame)."""
import os, re, glob, datetime as dt
import numpy as np
import img_geometry as G

LAND = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'Test_Data', 'Landing')
W, H = 320, 240

def _ts(name):
    m = re.search(r'([A-Z][a-z]{2}) ([A-Z][a-z]{2}) +(\d+) (\d+)-(\d+)-(\d+) (\d{4})', name)
    return dt.datetime.strptime(' '.join(m.groups()), '%a %b %d %H %M %S %Y')

def pair_runs(day):
    """[(run_dir, video_path)] — video = latest video starting before the run folder whose frame count matches."""
    import cv2
    vids = sorted(glob.glob(os.path.join(LAND, day, 'Test_Videos', '*.mp4')), key=_ts)
    runs = sorted([d for d in glob.glob(os.path.join(LAND, day, '*/')) if 'Test_Videos' not in d and os.path.exists(d + 'Img_Data.npy')], key=lambda d: _ts(os.path.basename(os.path.normpath(d))))
    nfr = {v: int(cv2.VideoCapture(v).get(7)) for v in vids}
    out = []
    for r in runs:
        tr = _ts(os.path.basename(os.path.normpath(r)))
        try: n = len(np.load(r + 'Img_Data.npy', allow_pickle=True).item()['Time'])
        except Exception: continue
        cand = [v for v in vids if 0 <= (tr - _ts(os.path.basename(v))).total_seconds() < 90 and abs(nfr[v] - n) <= 3]
        if cand and n > 60: out.append((r, cand[-1]))
    return out

def load_run(rd):
    img = np.load(rd + 'Img_Data.npy', allow_pickle=True).item()
    ctl = np.load(rd + 'Control_Data.npy', allow_pickle=True).item()
    tel = np.load(rd + 'Telemetry_Data.npy', allow_pickle=True).item()
    return img, ctl, tel

def ref_series(ctl):
    """Reference s(t)[4], h(t)[3], w(t)[3] on the control clock (truncated to common length)."""
    t = np.asarray(ctl['t'], float); s = np.asarray(ctl['s(t)'], float); h = np.asarray(ctl['h(t)'], float); w = np.asarray(ctl['w(t)'], float)
    n = min(len(t), len(s), len(h), len(w))
    return t[:n], s[:n], h[:n], w[:n]

class _Q:
    def __init__(s, q): s.w, s.x, s.y, s.z = q.w, q.x, q.y, q.z

def quat_at(tel, tq):
    to = np.asarray(tel['Odometry Timestamp'], float)
    idx = np.clip(np.searchsorted(to, tq), 0, len(to) - 1)
    return [_Q(tel['Quaternion'][i]) for i in idx]

def ref_pixel(s_xy, q):
    return G.get_real_pts_from_v(np.asarray([s_xy], float), q)[0]


def paper_blob(gray):
    """Pseudo-localiser for the white paper sheet the cross is drawn on: largest bright, compact, locally-brighter blob.
    Returns (cx, cy, area) or None. Deliberately independent of the cross detectors under test."""
    import cv2
    g = cv2.GaussianBlur(gray, (5, 5), 0)
    thr = max(np.percentile(g, 92), np.median(g) + 25)
    m = (g >= thr).astype(np.uint8)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    n, lab, st, cen = cv2.connectedComponentsWithStats(m)
    best = None
    for i in range(1, n):
        x, y, w, h, a = st[i]
        if a < 150 or a > 0.35 * W * H: continue
        if w >= W - 2 or h >= H - 2: continue          # frame-spanning bright = sky/ground wash, not the sheet
        fill = a / float(w * h)
        if fill < 0.55: continue
        if best is None or a > best[2]: best = (cen[i][0], cen[i][1], a, x, y, w, h)
    return best

def depth_yaw(tel, tq):
    """Camera depth above the marker plane (m) and yaw (rad) at times tq, from EKF."""
    to = np.asarray(tel['Odometry Timestamp'], float)
    idx = np.clip(np.searchsorted(to, tq), 0, len(to) - 1)
    pos = np.array([[tel['Position Body'][i].x_m, tel['Position Body'][i].y_m, tel['Position Body'][i].z_m] for i in idx])
    yaw = []
    for i in idx:
        q = tel['Quaternion'][i]; yaw.append(np.arctan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y**2 + q.z**2)))
    return -pos[:, 2] - 0.15, np.array(yaw), pos
