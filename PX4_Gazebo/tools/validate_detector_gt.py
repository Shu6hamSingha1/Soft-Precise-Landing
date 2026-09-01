"""Offline GT-scored cross-marker DETECTOR harness (2026-09-01).

Scores cross-marker detector FRONT-END variants against ground truth on recorded
raw frames -- the analogue of tools/validate_bgflow_corr.py (which scores the
FLOW solve), built to develop a lighting-/clutter-robust segmentation.

Per recorded frame it runs a candidate detector and compares:
  * detect-rate            -- fraction of frames with det.ok
  * centroid error         -- the LEVELED measured centroid
                              (CrossMarkerPerception._getVirtualPts(det.center, quat),
                               i.e. the exact `s` the controller consumes)
                              vs GT V-frame centroid bearing
                              (gt_optical_flow.compute_gt_flow -> V_s_g)
  * per-altitude-band breakdown

Time-sync mirrors validate_bgflow_corr.py: Img_Data['Time'] - Ground_Truth['Start
Time'] against gt_optical_flow's 0-based t_g; recorded frame i <-> Img_Data index
(len(Img_Data) - n_frames + i) since IMG_RECORD captures the tail (descent) window.

Add a variant: put an env dict in VARIANTS below (the detector reads CROSS_ADAPT_*
etc. at import; the harness reloads the module per variant). For a code-level
variant, gate it behind a new env flag in cross_marker_detector.py and add it here.

USAGE
  python3 tools/validate_detector_gt.py RUN[,RUN...] --frames RAW[,RAW...] [--variant N | --all] [--label TAG]
    RUN  = a Landing_Test/<ts> dir (Ground_Truth.npy + Img_Data.npy + Img_Params.txt)
    RAW  = its matching Test_Videos/<ts>_raw dir of f*.png   (paired positionally with RUN)
"""
import sys, os, argparse, glob, importlib, ast
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src'))
import numpy as np
import cv2
from types import SimpleNamespace

import cross_marker_detector as cmd
from cross_marker_perception import CrossMarkerPerception
from gt_optical_flow import compute_gt_flow

# ------------------------------------------------------------------ variants --
# name -> env overrides applied before importlib.reload(cmd). Everything the
# detector's front end keys on is env-configurable, so most variants are just
# a dict; a genuinely new segmentation path gets a code flag + an entry here.
VARIANTS = {
    "baseline":   {"CROSS_ADAPT_GATE": "0"},                                   # legacy inRange(V<=100)
    "adapt":      {"CROSS_ADAPT_GATE": "1"},                                   # CLAHE + adaptiveThreshold (+ Otsu fallback)
    "adapt_c12":  {"CROSS_ADAPT_GATE": "1", "CROSS_ADAPT_C": "12"},            # stricter local-contrast demand
    "adapt_b71":  {"CROSS_ADAPT_GATE": "1", "CROSS_ADAPT_BLOCK": "71"},        # larger local window
}
ALT_BANDS = [(4.0, 6.0), (3.0, 4.0), (2.0, 3.0), (1.3, 2.0), (0.7, 1.3)]

# --------------------------------------------------------------- gt reference --
def _perc_for(run_dir):
    """CrossMarkerPerception instance with this run's exact center/focal (from
    Img_Params.txt) -- used only for _getVirtualPts leveling."""
    p = CrossMarkerPerception(resolution=(240, 320))
    ip = os.path.join(run_dir, "Img_Params.txt")
    if os.path.isfile(ip):
        d = ast.literal_eval(open(ip).read().strip())
        p.center = np.asarray(d["center"], float)
        p.focal = np.asarray(d["focal"], float)
    return p


def _load_run(run_dir):
    img = np.load(os.path.join(run_dir, "Img_Data.npy"), allow_pickle=True).item()
    gt = compute_gt_flow(run_dir)
    return img, gt


# ----------------------------------------------------------------- one (v,run) --
def _score(run_dir, raw_dir, perc):
    img, gt = _load_run(run_dir)
    St = gt["start_time"]; tg = gt["t_g"]
    Vsg = gt["V_s_g"]; altg = np.abs(gt["alt"]); alphag = gt["alpha"]
    it = np.asarray(img["Time"], float)
    iq = img["Quat"]
    M = len(it)
    fs = sorted(glob.glob(os.path.join(raw_dir, "f*.png")))
    N = len(fs)
    off = M - N                                       # recorded = tail N frames
    ts = {"last_bbox": None, "miss_count": 0}
    rows = []
    for i, fp in enumerate(fs):
        j = off + i
        if j < 0 or j >= M:
            continue
        t = it[j] - St
        q = iq[j]
        qn = None
        if q is not None and np.all(np.isfinite(np.asarray(q, float))):
            q = np.asarray(q, float)
            qn = SimpleNamespace(w=q[0], x=q[1], y=q[2], z=q[3])
        s_gt = np.array([np.interp(t, tg, Vsg[:, 0]), np.interp(t, tg, Vsg[:, 1])])
        alt = float(np.interp(t, tg, altg))
        # GT centroid bearing blows up (1/(z_v+0.01)) when the target grazes the
        # FoV edge under a hard tilt -- unreliable reference there; skip scoring
        # centroid error on those frames (detect-rate still counts them).
        gt_bad = not np.all(np.isfinite(s_gt)) or float(np.hypot(*s_gt)) > 1.5
        frame = cv2.imread(fp)
        if frame is None:
            continue
        det = cmd.detect(frame, track_state=ts)
        err = np.nan
        if det.ok and qn is not None and det.center is not None and not gt_bad:
            try:
                s_meas = perc._getVirtualPts(np.array([det.center], float), qn, log_zv=False)[0]
                if np.all(np.isfinite(s_meas)):
                    err = float(np.hypot(*(s_meas - s_gt)))
            except Exception:
                pass
        rows.append((t, alt, bool(det.ok), err, det.fail_reason or ""))
    return rows


def _agg(rows, focal):
    if not rows:
        return None
    a = np.array([r[1] for r in rows], float)
    ok = np.array([r[2] for r in rows], bool)
    e = np.array([r[3] for r in rows], float)
    eok = e[np.isfinite(e)]
    out = {
        "n": len(rows),
        "detrate": 100.0 * ok.mean(),
        "n_scored": len(eok),
        "err_med_n": float(np.median(eok)) if len(eok) else np.nan,
        "err_med_px": float(np.median(eok) * focal) if len(eok) else np.nan,
        "hit_015": 100.0 * np.mean(eok < 0.15) if len(eok) else np.nan,   # frac of ok dets within 0.15 norm (~20px) of GT
        "bands": [],
    }
    from collections import Counter
    out["topfail"] = Counter(r[4] for r in rows if not r[2] and r[4]).most_common(3)
    for lo, hi in ALT_BANDS:
        m = (a >= lo) & (a < hi)
        if m.sum() < 3:
            out["bands"].append((lo, hi, m.sum(), np.nan))
            continue
        out["bands"].append((lo, hi, int(m.sum()), 100.0 * ok[m].mean()))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("runs", help="comma-separated Landing_Test/<ts> dirs")
    ap.add_argument("--frames", required=True, help="comma-separated matching <ts>_raw dirs")
    ap.add_argument("--variant", default=None, choices=list(VARIANTS))
    ap.add_argument("--all", action="store_true")
    ap.add_argument("--label", default="")
    args = ap.parse_args()

    runs = [r.strip() for r in args.runs.split(",")]
    raws = [r.strip() for r in args.frames.split(",")]
    assert len(runs) == len(raws), "runs and --frames must pair 1:1"
    names = [args.variant] if args.variant else (list(VARIANTS) if args.all else ["baseline"])

    percs = [_perc_for(r) for r in runs]
    focal = float(np.mean(percs[0].focal))

    for nm in names:
        for k in list(os.environ):
            if k.startswith("CROSS_ADAPT"):
                del os.environ[k]
        os.environ.update(VARIANTS[nm])
        importlib.reload(cmd)
        tag = f"{nm}" + (f" [{args.label}]" if args.label else "")
        print(f"\n================  {tag}  ================")
        for run_dir, raw_dir, perc in zip(runs, raws, percs):
            rows = _score(run_dir, raw_dir, perc)
            g = _agg(rows, focal)
            rn = os.path.basename(run_dir.rstrip("/"))
            if g is None:
                print(f"  {rn:32s}  (no frames scored)")
                continue
            bands = "  ".join(f"{lo:.1f}-{hi:.1f}:{('%.0f%%' % b) if np.isfinite(b) else '--':>4}"
                              for lo, hi, n, b in g["bands"])
            print(f"  {rn:32s} n={g['n']:4d}  detOK {g['detrate']:5.1f}%   "
                  f"centroid-err med {g['err_med_n']:.3f} ({g['err_med_px']:.1f}px)  "
                  f"within-0.15 {g['hit_015']:.0f}%  (scored {g['n_scored']})")
            print(f"      by alt: {bands}")
            if g["topfail"]:
                print(f"      top-fail: {g['topfail']}")


if __name__ == "__main__":
    main()
