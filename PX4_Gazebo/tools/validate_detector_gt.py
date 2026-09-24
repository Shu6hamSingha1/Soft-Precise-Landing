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
  # curated eval set (each <tag>/ holds Ground_Truth.npy + Img_Data.npy +
  # Img_Params.txt + frames/f*.png -- see test_data/DetectorFrameset/MANIFEST.md):
  python3 tools/validate_detector_gt.py --set test_data/DetectorFrameset --all

  # ad-hoc pair(s):
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
    "current":    {},                                                          # the shipped defaults, whatever they are
    "stroke":     {"CROSS_DETECTOR": "stroke"},                                # 2026-09-24 locked-design stroke detector
    "adapt":      {"CROSS_ADAPT_GATE": "1"},                                   # CLAHE + adaptiveThreshold (+ Otsu fallback)
    "adapt_c12":  {"CROSS_ADAPT_GATE": "1", "CROSS_ADAPT_C": "12"},            # stricter local-contrast demand
    "adapt_b71":  {"CROSS_ADAPT_GATE": "1", "CROSS_ADAPT_BLOCK": "71"},        # larger local window
    # 2026-09-03: candidate-mask ensemble -- legacy V-gate first, and only if that
    # yields no cross-shaped component, Otsu on L/a/b in both polarities scored by
    # the SAME shape test. Inert by construction wherever legacy already works.
    "ensemble":   {"CROSS_GATE_MODE": "ensemble"},
    # 2026-09-03 stage 3: positive geometry confirm. geom1 = in ADDITION to the
    # centroid proxy; geom2 = REPLACES it (the locked design). ens_geom2 = both stages.
    "geom1":      {"CROSS_GEOM_CONFIRM": "1"},
    "geom2":      {"CROSS_GEOM_CONFIRM": "2"},
    "ens_geom2":  {"CROSS_GATE_MODE": "ensemble", "CROSS_GEOM_CONFIRM": "2"},
    # 2026-09-03 ring-transition confirm: a cross junction has arms RADIATING from
    # it, a plate corner has two edges MEETING -> count mask crossings on a ring.
    "ring":       {"CROSS_RING_CONFIRM": "1"},
    "ring_t4":    {"CROSS_RING_CONFIRM": "1", "CROSS_RING_MIN_TRANSITIONS": "4"},
    "ens_ring":   {"CROSS_GATE_MODE": "ensemble", "CROSS_RING_CONFIRM": "1"},
    # 2026-09-04 span-balance confirm: a cross's centre sits INSIDE each arm's own
    # point span; a plate corner is the ENDPOINT of both edges meeting there.
    "balance":    {"CROSS_BALANCE_CONFIRM": "1"},
    "ring_balance": {"CROSS_RING_CONFIRM": "1", "CROSS_BALANCE_CONFIRM": "1"},
    "ens_ring_balance": {"CROSS_GATE_MODE": "ensemble", "CROSS_RING_CONFIRM": "1", "CROSS_BALANCE_CONFIRM": "1"},
    # margin=1.0 makes both rescue thresholds unreachable (RING_RESCUE_N doubles past
    # any real transition count; BALANCE_RESCUE_D goes to 0) -- approximates the
    # pre-899d8d26/a893a77e "both must independently pass" (OR-reject) behavior for
    # a before/after comparison on the SAME recordings.
    "ring_balance_norescue": {"CROSS_RING_CONFIRM": "1", "CROSS_BALANCE_CONFIRM": "1",
                              "CROSS_RING_BALANCE_RESCUE_MARGIN": "1.0"},
    "ens_ring_balance_norescue": {"CROSS_GATE_MODE": "ensemble", "CROSS_RING_CONFIRM": "1",
                                  "CROSS_BALANCE_CONFIRM": "1", "CROSS_RING_BALANCE_RESCUE_MARGIN": "1.0"},
}
_GT_STRICT = os.environ.get("CROSS_GT_WINDOW_STRICT", "1") == "1"
ALT_BANDS = [(4.0, 6.0), (3.0, 4.0), (2.0, 3.0), (1.3, 2.0), (0.7, 1.3), (0.3, 0.7), (0.0, 0.3)]
# 2026-09-24: the two terminal bands were missing -- the <0.7 m regime is exactly where the
# stationary perception-s landing is lost (SPercGTFB_AB), so it was never being scored.
# GT reference: "true" = V_s_true (x/z, what the camera measures; default) or "reg" = V_s_g
# (x/(z+0.2), the GT-FB feed; the pre-2026-09-24 reference -- books correct close-range
# bearings as 30-90% errors). CROSS_EVAL_REF or --ref.
_REF = os.environ.get("CROSS_EVAL_REF", "true")

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


def _meta(run_dir):
    """Optional per-set meta.json (e.g. {"marker_dz": 0.5, "world": "rover_cross"}).
    Rover worlds need marker_dz=0.5 (marker sits on the 0.5 m platform); default 0.0."""
    mp = os.path.join(run_dir, "meta.json")
    if os.path.isfile(mp):
        import json
        return json.load(open(mp))
    return {}


def _load_run(run_dir):
    img = np.load(os.path.join(run_dir, "Img_Data.npy"), allow_pickle=True).item()
    gt = compute_gt_flow(run_dir, marker_dz=_meta(run_dir).get("marker_dz"))
    return img, gt


def _frame_rows(raw_dir, img):
    """[(frame_path, Img_Data row)] -- EXACT via frames.tsv (saved-frame -> capture stamp,
    written by CrossMarkerNode since 2026-09-24) when present, else the legacy tail offset."""
    fs = sorted(glob.glob(os.path.join(raw_dir, "f*.png")))
    tsv = os.path.join(raw_dir, "frames.tsv")
    if os.path.isfile(tsv):
        st = np.asarray(img["Stamp"], float)
        lut = {}
        for j, v in enumerate(st):
            lut.setdefault(v, j)
        out = []
        for line in open(tsv).read().splitlines()[1:]:
            k, v = line.split("\t")
            fp = os.path.join(raw_dir, f"f{int(k):05d}.png")
            j = lut.get(float(v))
            if j is not None and os.path.isfile(fp):
                out.append((fp, j))
        return out
    off = len(img["Time"]) - len(fs)                  # recorded = tail N frames
    return [(fp, off + i) for i, fp in enumerate(fs) if 0 <= off + i < len(img["Time"])]


# ----------------------------------------------------------------- one (v,run) --
def _score(run_dir, raw_dir, perc):
    img, gt = _load_run(run_dir)
    St = gt["start_time"]; tg = gt["t_g"]
    Vsg = gt["V_s_true"] if _REF == "true" else gt["V_s_g"]; altg = np.abs(gt["alt"]); alphag = gt["alpha"]
    it = np.asarray(img["Time"], float)
    iq = img["Quat"]
    ts = {"last_bbox": None, "miss_count": 0}
    n_outside = [0]
    rows = []
    for fp, j in _frame_rows(raw_dir, img):
        t = it[j] - St
        # ⛔ GT-WINDOW GUARD (2026-09-03). np.interp CLAMPS outside t_g instead of
        # rejecting, so frames recorded after the GT log ends were being scored against
        # FROZEN touchdown values. Measured on RobustnessFrameset: 16-42% of frames per
        # variant fall past t_g[-1] (inv worst, 42%), which inflated inv within-0.15 to
        # 23% when the GT-valid truth is 0%, and DEFLATED base from ~100% to 78%.
        # Same trap as feedback_detector_offline_replay_gotchas §1 -- now enforced here
        # rather than left to each caller. CROSS_GT_WINDOW_STRICT=0 restores the old
        # (unsafe) behaviour for comparison only.
        if _GT_STRICT and not (tg[0] <= t <= tg[-1]):
            n_outside[0] += 1
            continue
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
    if n_outside[0]:
        print(f"      [gt-window] dropped {n_outside[0]} frame(s) outside the GT log window "
              f"({100.0*n_outside[0]/max(len(rows)+n_outside[0],1):.0f}% of recorded)")
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
        # POISON rate: confident-wrong -- ok detections > 0.15 off, as a share of ALL scored-
        # eligible frames. The quantity that flies the drone away (a miss is honest; this is not).
        "poison": 100.0 * np.sum(eok >= 0.15) / max(len(rows), 1),
        "bands": [],
    }
    from collections import Counter
    out["topfail"] = Counter(r[4] for r in rows if not r[2] and r[4]).most_common(3)
    for lo, hi in ALT_BANDS:
        m = (a >= lo) & (a < hi)
        if m.sum() < 3:
            out["bands"].append((lo, hi, m.sum(), np.nan, np.nan, np.nan))
            continue
        em = e[m & np.isfinite(e)]
        out["bands"].append((lo, hi, int(m.sum()), 100.0 * ok[m].mean(),
                             float(np.median(em)) if len(em) else np.nan,
                             100.0 * np.mean(em < 0.15) if len(em) else np.nan))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("runs", nargs="?", default=None, help="comma-separated Landing_Test/<ts> dirs")
    ap.add_argument("--frames", default=None, help="comma-separated matching <ts>_raw dirs")
    ap.add_argument("--set", dest="setdir", default=None,
                    help="curated frameset dir; each subdir holds *.npy + Img_Params.txt + frames/")
    ap.add_argument("--variant", default=None, choices=list(VARIANTS))
    ap.add_argument("--all", action="store_true")
    ap.add_argument("--label", default="")
    ap.add_argument("--ref", choices=["true", "reg"], default=None,
                    help="GT bearing reference: true x/z (default) or reg x/(z+0.2) (pre-2026-09-24)")
    args = ap.parse_args()
    global _REF
    if args.ref:
        _REF = args.ref
    print(f"[ref] scoring against {'TRUE bearing x/z (V_s_true)' if _REF == 'true' else 'REGULARIZED x/(z+0.2) (V_s_g)'}")

    if args.setdir:
        subs = sorted(d for d in glob.glob(os.path.join(args.setdir, "*"))
                      if os.path.isdir(os.path.join(d, "frames"))
                      and os.path.isfile(os.path.join(d, "Ground_Truth.npy")))
        runs = subs
        raws = [os.path.join(d, "frames") for d in subs]
    else:
        assert args.runs and args.frames, "give --set DIR, or RUNS + --frames"
        runs = [r.strip() for r in args.runs.split(",")]
        raws = [r.strip() for r in args.frames.split(",")]
    assert len(runs) == len(raws), "runs and --frames must pair 1:1"
    names = [args.variant] if args.variant else (list(VARIANTS) if args.all else ["baseline"])

    percs = [_perc_for(r) for r in runs]
    focal = float(np.mean(percs[0].focal))

    for nm in names:
        for k in list(os.environ):
            if (k.startswith("CROSS_ADAPT") or k.startswith("CROSS_RING_")
                    or k.startswith("CROSS_STROKE")
                    or k in ("CROSS_RING_BALANCE_RESCUE_MARGIN", "CROSS_GATE_MODE", "CROSS_GEOM_CONFIRM", "CROSS_BALANCE_CONFIRM",
                             "CROSS_DETECTOR")):
                del os.environ[k]   # GATE_MODE too, else it leaks into later variants
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
                              + (f"/{em:.3f}" if np.isfinite(em) else "")
                              for lo, hi, n, b, em, hb in g["bands"])
            print(f"  {rn:32s} n={g['n']:4d}  detOK {g['detrate']:5.1f}%   "
                  f"centroid-err med {g['err_med_n']:.3f} ({g['err_med_px']:.1f}px)  "
                  f"within-0.15 {g['hit_015']:.0f}%  POISON {g['poison']:.1f}%  (scored {g['n_scored']})")
            print(f"      by alt (detOK/err-med): {bands}")
            if g["topfail"]:
                print(f"      top-fail: {g['topfail']}")


if __name__ == "__main__":
    main()
