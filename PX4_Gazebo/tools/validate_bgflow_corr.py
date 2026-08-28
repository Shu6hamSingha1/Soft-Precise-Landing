"""GT-correlation harness for the cross-marker BACKGROUND-TEXTURE optical-flow `h`
path (`cross_marker_perception._compute_hw_bgflow`), reusable across point-selection
strategies and texture types.

This is the 2026-08-27 ad-hoc validation (see
Memory/px4/project_20260827_framerate_and_h_texture_investigation.md) turned into a
tool, so every future "extract more signal from the texture" change is measured the
SAME way:

  * time-sync via Ground_Truth['Start Time'] (plain subtraction -- NO two-anchor
    rescale; that solved a non-existent problem),
  * video/frame index i  <->  Img_Data index (n_pre_engage + i),
  * correspondences fed through the REAL production de-rotation + interaction-matrix
    solve (CrossMarkerPerception._solve_jacobian), never a raw-pixel proxy.

Under PLASMC_GT_FEEDBACK=1 the recording's Control_Data.npy['h(t)'] IS the exact GT
h fed to the SMC -- that is the reference we correlate against.

INPUTS (one run dir):
  <run_dir>/Ground_Truth.npy, Control_Data.npy, Img_Data.npy      (always)
  frames: EITHER  --frames <dir>/f%05d.png (+ stamps.npy)   [IMG_RECORD_RAW=1, preferred: lossless]
          OR      --frames <file>.mp4                        [IMG_RECORD=1, lossy -- mp4v corrupts fine speckle flow]

USAGE
  python3 tools/validate_bgflow_corr.py <run_dir> --frames <raw_dir_or_mp4> [--strategy NAME | --all]
  python3 tools/validate_bgflow_corr.py <run_dir> --frames <...> --all --label old1024

Add a strategy by registering a fn (gray_prev, gray_curr, det) -> (prev_pts Nx2, curr_pts Nx2)
in STRATEGIES below; everything downstream (de-rotation, solve, time-sync, corr) is shared.
"""
import sys, os, argparse, glob
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src'))
import numpy as np
import cv2
from types import SimpleNamespace

import cross_marker_detector as cmd
from cross_marker_perception import CrossMarkerPerception

LK_WIN = (15, 15)
LK_MAX_LEVEL = 2
RESID_GATE = float(os.environ.get('BGF_RESID_GATE', '0.6'))
LAB3 = ['h_x', 'h_y', 'h_z']


# --------------------------------------------------------------------------- #
# point-selection strategies:  (gray_prev, gray_curr, det) -> (prev_pts, curr_pts)
# --------------------------------------------------------------------------- #
def _bg_mask(det, shape):
    return cmd.background_mask_from_detection(det, shape)


# near-black stroke threshold, matches _detect_core's first (color-gate) stage intent
_BLACK_MAX_V = 90


def _bg_mask_bboxonly(det, gray):
    """Extent box (from det.mask_bbox alone -- available even when det.ok is False)
    MINUS a threshold-derived near-black stroke mask. Lets bg-flow run on frames
    where the Hough line-fit failed but the color blob + bbox are still solid
    (~70% of otherwise-discarded frames), without depending on det.isolated_mask."""
    if det is None or det.mask_bbox is None:
        return None
    ext = cmd.extent_mask_from_detection(det, gray.shape)
    if ext is None or not ext.any():
        return None
    black = (gray < _BLACK_MAX_V).astype(np.uint8) * 255
    k = 2 * cmd.LINE_EXCLUDE_DILATE_PX + 1
    black = cv2.dilate(black, np.ones((k, k), np.uint8))
    return cv2.bitwise_and(ext, cv2.bitwise_not(black))


def _lk(gray_prev, gray_curr, pts, fb=False, fb_thresh=0.7):
    """Forward LK; optional forward-backward consistency rejection."""
    if pts is None or len(pts) < 3:
        return None, None
    p0 = pts.reshape(-1, 1, 2).astype(np.float32)
    p1, st, _ = cv2.calcOpticalFlowPyrLK(gray_prev, gray_curr, p0, None,
                                         winSize=LK_WIN, maxLevel=LK_MAX_LEVEL)
    st = st.flatten().astype(bool)
    if fb:
        p0b, stb, _ = cv2.calcOpticalFlowPyrLK(gray_curr, gray_prev, p1, None,
                                               winSize=LK_WIN, maxLevel=LK_MAX_LEVEL)
        rt = np.linalg.norm((p0b - p0).reshape(-1, 2), axis=1)
        st &= stb.flatten().astype(bool) & (rt < fb_thresh)
    a = p0.reshape(-1, 2)[st]
    b = p1.reshape(-1, 2)[st]
    return (a, b) if len(a) >= 5 else (None, None)


def _clahe(g, clip=2.0, tile=8):
    return cv2.createCLAHE(clipLimit=clip, tileGridSize=(tile, tile)).apply(g)


def strat_baseline(gp, gc, det):
    """Exactly _compute_hw_bgflow's current live path."""
    bm = _bg_mask(det, gc.shape)
    if bm is None or not bm.any():
        return None, None
    pts = cmd.multiscale_good_features(gp, bm, max_corners=80, quality=0.01, min_dist=4)
    return _lk(gp, gc, pts, fb=False)


def strat_clahe(gp, gc, det):
    bm = _bg_mask(det, gc.shape)
    if bm is None or not bm.any():
        return None, None
    gpe, gce = _clahe(gp), _clahe(gc)
    pts = cmd.multiscale_good_features(gpe, bm, max_corners=80, quality=0.01, min_dist=4)
    return _lk(gpe, gce, pts, fb=False)


def strat_clahe_fb(gp, gc, det):
    bm = _bg_mask(det, gc.shape)
    if bm is None or not bm.any():
        return None, None
    gpe, gce = _clahe(gp), _clahe(gc)
    pts = cmd.multiscale_good_features(gpe, bm, max_corners=80, quality=0.01, min_dist=4)
    return _lk(gpe, gce, pts, fb=True)


def strat_dense_dis(gp, gc, det):
    """DIS dense flow over the background mask, grid-sampled -> correspondences.
    Uses every textured pixel instead of only Shi-Tomasi corners -- the lever for
    fine / low-contrast / directional textures where sparse corners are too few."""
    bm = _bg_mask(det, gc.shape)
    if bm is None or not bm.any():
        return None, None
    try:
        dis = cv2.DISOpticalFlow_create(cv2.DISOPTICAL_FLOW_PRESET_FAST)
    except AttributeError:
        return None, None
    flow = dis.calc(gp, gc, None)                       # (H,W,2) dx,dy
    ys, xs = np.where(bm > 0)
    if len(xs) < 20:
        return None, None
    step = max(1, int(np.sqrt(len(xs) / 200)))         # ~<=200 sample points
    xs, ys = xs[::step], ys[::step]
    prev = np.column_stack([xs, ys]).astype(np.float32)
    disp = flow[ys, xs]
    curr = prev + disp
    keep = np.linalg.norm(disp, axis=1) < 15.0         # reject blown vectors
    prev, curr = prev[keep], curr[keep]
    return (prev, curr) if len(prev) >= 5 else (None, None)


def strat_clahe_fb_bbox(gp, gc, det):
    """CLAHE+FB sparse, but on the bbox-only background mask -> also runs on
    Hough-failed (det.ok=False) frames."""
    bm = _bg_mask_bboxonly(det, gc)
    if bm is None or not bm.any():
        return None, None
    gpe, gce = _clahe(gp), _clahe(gc)
    pts = cmd.multiscale_good_features(gpe, bm, max_corners=80, quality=0.01, min_dist=4)
    return _lk(gpe, gce, pts, fb=True)


def strat_dense_bbox(gp, gc, det):
    bm = _bg_mask_bboxonly(det, gc)
    if bm is None or not bm.any():
        return None, None
    try:
        dis = cv2.DISOpticalFlow_create(cv2.DISOPTICAL_FLOW_PRESET_FAST)
    except AttributeError:
        return None, None
    flow = dis.calc(gp, gc, None)
    ys, xs = np.where(bm > 0)
    if len(xs) < 20:
        return None, None
    step = max(1, int(np.sqrt(len(xs) / 200)))
    xs, ys = xs[::step], ys[::step]
    prev = np.column_stack([xs, ys]).astype(np.float32)
    disp = flow[ys, xs]
    keep = np.linalg.norm(disp, axis=1) < 15.0
    prev, curr = prev[keep], (prev + disp)[keep]
    return (prev, curr) if len(prev) >= 5 else (None, None)


def strat_hybrid_sparse(gp, gc, det):
    """PRODUCTION-SHAPED: high-quality sparse path when the full detection is
    good; bbox-only dense flow as the fallback (replaces _compute_hw_bgflow's
    current line-mask fallback)."""
    return strat_clahe_fb(gp, gc, det) if (det is not None and det.ok) \
        else strat_dense_bbox(gp, gc, det)


def strat_hybrid_dense(gp, gc, det):
    return strat_dense_dis(gp, gc, det) if (det is not None and det.ok) \
        else strat_dense_bbox(gp, gc, det)


strat_clahe_fb_bbox.allow_notok = True
strat_dense_bbox.allow_notok = True
strat_hybrid_sparse.allow_notok = True
strat_hybrid_dense.allow_notok = True

STRATEGIES = {
    'baseline': strat_baseline,
    'clahe': strat_clahe,
    'clahe_fb': strat_clahe_fb,
    'dense_dis': strat_dense_dis,
    'clahe_fb_bbox': strat_clahe_fb_bbox,
    'dense_bbox': strat_dense_bbox,
    'hybrid_sparse': strat_hybrid_sparse,
    'hybrid_dense': strat_hybrid_dense,
}


# --------------------------------------------------------------------------- #
# frame source
# --------------------------------------------------------------------------- #
class FrameSource:
    def __init__(self, path):
        self.is_dir = os.path.isdir(path)
        if self.is_dir:
            self.files = sorted(glob.glob(os.path.join(path, 'f*.png')))
            sp = os.path.join(path, 'stamps.npy')
            self.stamps = np.load(sp) if os.path.exists(sp) else None
            self.n = len(self.files)
        else:
            self.cap = cv2.VideoCapture(path)
            self.n = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))
            self.stamps = None
        if self.n < 2:
            raise SystemExit(f"frame source has {self.n} frames")

    def gray(self, i):
        if self.is_dir:
            im = cv2.imread(self.files[i], cv2.IMREAD_GRAYSCALE)
        else:
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, i)
            ok, im = self.cap.read()
            if not ok:
                return None
            im = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY) if im.ndim == 3 else im
        return im


def _quat(row):
    w, x, y, z = [float(v) for v in row]
    return SimpleNamespace(w=w, x=x, y=y, z=z)


def _pearson(a, b):
    m = np.isfinite(a) & np.isfinite(b)
    if m.sum() < 10 or np.std(a[m]) < 1e-9 or np.std(b[m]) < 1e-9:
        return np.nan, int(m.sum())
    return float(np.corrcoef(a[m], b[m])[0, 1]), int(m.sum())


def _detect_frames(fs, n_pairs):
    """Run detect() once per frame with a live-style track_state; return dict i->det."""
    ts = {'last_bbox': None, 'miss_count': 0}
    dets, ok = {}, 0
    for i in range(n_pairs + 1):
        g = fs.gray(i)
        if g is None:
            dets[i] = None
            continue
        det = cmd.detect(cv2.cvtColor(g, cv2.COLOR_GRAY2BGR), track_state=ts)
        dets[i] = det
        ok += int(det is not None and det.ok)
    return dets, ok


def run_strategy(name, fn, fs, img, cd_t, cd_h, n_pre, p, dets):
    # per-frame dt from Img_Data's own capture timeline (frame i <-> idx n_pre+i)
    img_time = np.asarray(img['Time'])
    quats = np.asarray(img['Quat'])
    sols, tstamps, npts, okflag = [], [], [], []
    n_pairs = min(fs.n, len(img_time) - n_pre) - 1
    dt_fallback = float(np.median(np.diff(img_time[n_pre:n_pre + n_pairs + 1])))
    gray_prev = fs.gray(0)
    for i in range(1, n_pairs):
        gray_curr = fs.gray(i)
        if gray_curr is None:
            gray_prev = None
            continue
        if gray_prev is None:
            gray_prev = gray_curr
            continue
        det = dets.get(i)
        if det is not None and (det.ok or getattr(fn, 'allow_notok', False)):
            a, b = fn(gray_prev, gray_curr, det)
            if a is not None:
                idx = n_pre + i
                dt = float(img_time[idx] - img_time[idx - 1])
                if not (dt > 1e-6):
                    dt = dt_fallback
                try:
                    sol, cond, rel_resid, *_ = p._solve_jacobian(
                        a, b, dt, prev_quat=_quat(quats[idx - 1]),
                        curr_quat=_quat(quats[idx]))
                    # quality gate on the FALLBACK path (det.ok=False): a high
                    # relative residual = the correspondences don't agree on any
                    # single global flow -> garbage in, skip (live: fall through
                    # to _compute_hw's line-mask path). Texture-agnostic.
                    if (not det.ok) and (not np.isfinite(rel_resid) or rel_resid > RESID_GATE):
                        gray_prev = gray_curr
                        continue
                    sols.append(sol[:3])
                    tstamps.append(img_time[idx])
                    npts.append(len(a))
                    okflag.append(bool(det.ok))
                except Exception:
                    pass
        gray_prev = gray_curr

    if len(sols) < 10:
        print(f"  {name:12s}  only {len(sols)} solved pairs -- INSUFFICIENT")
        return
    sols = np.asarray(sols); tstamps = np.asarray(tstamps)
    npts = np.asarray(npts); okflag = np.asarray(okflag)
    G = np.column_stack([np.interp(tstamps, cd_t, cd_h[:, k]) for k in range(3)])

    def _line(mask, tag):
        if mask.sum() < 10:
            return
        row = [f"{LAB3[k]}={_pearson(sols[mask, k], G[mask, k])[0]:+.3f}" for k in range(3)]
        print(f"  {name:14s}{tag:7s} {'  '.join(row)}   | n={int(mask.sum()):3d}  "
              f"pts mean={npts[mask].mean():.0f} min={npts[mask].min()}  "
              f"GT h_z std={np.std(G[mask, 2]):.3f}")

    all_m = np.ones(len(sols), bool)
    _line(all_m, "ALL")
    if 0 < okflag.sum() < len(sols):
        _line(okflag, "ok")
        _line(~okflag, "notok")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('run_dir')
    ap.add_argument('--frames', required=True, help='IMG_RECORD_RAW dir (f*.png+stamps.npy) or an .mp4')
    ap.add_argument('--strategy', default=None, choices=list(STRATEGIES))
    ap.add_argument('--all', action='store_true')
    ap.add_argument('--label', default='')
    args = ap.parse_args()

    d = args.run_dir
    gt = np.load(os.path.join(d, 'Ground_Truth.npy'), allow_pickle=True).item()
    cd = np.load(os.path.join(d, 'Control_Data.npy'), allow_pickle=True).item()
    img = np.load(os.path.join(d, 'Img_Data.npy'), allow_pickle=True).item()

    start = float(np.asarray(gt['Start Time']))
    img_t_rel = np.asarray(img['Time']) - start
    n_pre = int((img_t_rel < 0).sum())

    cd_t = np.asarray(cd['t'], dtype=float)                     # already absolute clock
    cd_h = np.asarray([np.asarray(r, dtype=float)[:3] for r in cd['h(t)']])
    if abs(cd_t[0] - start) > 5e-3:
        print(f"  WARN Control_Data t[0]={cd_t[0]:.4f} vs Start Time {start:.4f} "
              f"(diff {cd_t[0]-start:+.4f}s) -- larger than expected clock offset")

    fs = FrameSource(args.frames)
    # Time-sync invariant (verified across 5 reps 2026-08-28): the IMG_RECORD video
    # starts at CONTROLLER_READY and contains EXACTLY the post-engage Img_Data
    # frames, so video frame i <-> Img_Data index (n_pre + i). If a record-thread
    # drop ever broke that, every correlation below would be silently misaligned.
    n_post = len(np.asarray(img['Time'])) - n_pre
    if fs.is_dir and abs(n_post - fs.n) > 1:
        raise SystemExit(
            f"TIME-SYNC: post-engage Img_Data frames ({n_post}) != video frames "
            f"({fs.n}); frame<->index mapping is broken, refusing to correlate.")
    p = CrossMarkerPerception(resolution=(img.get('_res_rows', 240), img.get('_res_cols', 320))
                              if isinstance(img, dict) else (240, 320))
    # honor the recording's real intrinsics if present in Img_Params.txt
    ip = os.path.join(d, 'Img_Params.txt')
    if os.path.exists(ip):
        prm = eval(open(ip).read())
        p.center = np.array(prm['center'], dtype=float)
        p.focal = np.array(prm['focal'], dtype=float)

    lbl = f"[{args.label}] " if args.label else ""
    print(f"{lbl}{os.path.basename(d.rstrip('/'))}  frames={fs.n}  n_pre_engage={n_pre}  "
          f"center={tuple(p.center)}  focal={tuple(p.focal)}  GT h pairs={len(cd_t)}")
    n_pairs = min(fs.n, len(np.asarray(img['Time'])) - n_pre) - 1
    dets, n_ok = _detect_frames(fs, n_pairs)
    print(f"  detect-ok: {n_ok}/{n_pairs + 1} frames")
    names = list(STRATEGIES) if args.all else [args.strategy or 'baseline']
    for nm in names:
        run_strategy(nm, STRATEGIES[nm], fs, img, cd_t, cd_h, n_pre, p, dets)


if __name__ == '__main__':
    main()
