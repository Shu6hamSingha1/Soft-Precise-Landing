#!/usr/bin/env python3
"""GT-feedback control-tuning analyzer.

Quantifies what the GT-feedback tuning targets: per-axis CONVERGENCE and
BOUNDEDNESS of s_e_n (Task-1, outer) and h_e (Task-2, middle), plus a per-axis
LIMIT-CYCLE metric (sustained tail oscillation) for x, y, z and yaw (e_a).

Per rep, for each channel it reports over the terminal tail (last TAIL frac):
  conv   = |mean(tail)|                      (-> 0 means converged)
  bound  = max(|signal|) over whole flight   (boundedness)
  osc    = tail peak-to-peak of the detrended signal (limit-cycle amplitude)
  ncross = zero-crossings of the detrended tail (oscillation frequency proxy)
A limit cycle = osc not small AND ncross high (sustained, non-decaying ringing).

Usage:
    python3 tools/analyze_gt_tuning.py "<rep_dir>" [more reps...]
    python3 tools/analyze_gt_tuning.py --glob 'test_data/GTfb_IC2/*'
"""
import sys, os, glob
import numpy as np

TAIL = float(os.environ.get("GT_TAIL", "0.4"))   # fraction of flight treated as terminal


def _metrics(x):
    """conv, bound, osc, ncross for a 1-D signal x."""
    x = np.asarray(x, float)
    x = x[np.isfinite(x)]
    if len(x) < 6:
        return dict(conv=np.nan, bound=np.nan, osc=np.nan, ncross=0)
    n0 = int((1 - TAIL) * len(x))
    tail = x[n0:]
    # detrend the tail (remove the slow drift so osc = oscillation only)
    t = np.arange(len(tail), dtype=float)
    A = np.column_stack([t, np.ones_like(t)])
    coef, *_ = np.linalg.lstsq(A, tail, rcond=None)
    res = tail - A @ coef
    ncross = int(np.sum(np.diff(np.sign(res)) != 0))
    return dict(conv=float(abs(np.mean(tail))),
                bound=float(np.max(np.abs(x))),
                osc=float(np.ptp(res)),
                ncross=ncross)


def analyze(rep):
    cd = np.load(os.path.join(rep, "Control_Data.npy"), allow_pickle=True).item()
    sen = np.asarray(cd['s_e_n(t)'], float)            # (N,2)
    h   = np.asarray(cd['h(t)'], float)                # (N,3)
    h_d = np.asarray(cd['h_d(t)'], float)              # (N,3)
    e_a = np.asarray(cd['e_a(t)'], float)              # (N,)
    h_e = h - h_d
    out = {}
    out['sen_x'] = _metrics(sen[:, 0]); out['sen_y'] = _metrics(sen[:, 1])
    out['he_x'] = _metrics(h_e[:, 0]);  out['he_y'] = _metrics(h_e[:, 1]); out['he_z'] = _metrics(h_e[:, 2])
    out['e_a']  = _metrics(e_a)
    out['_nsamp'] = len(sen)
    return out


def main(reps):
    hdr = f"{'channel':7s} {'conv':>7s} {'bound':>8s} {'osc':>7s} {'ncross':>7s}"
    agg = {}
    for rep in reps:
        try:
            o = analyze(rep)
        except Exception as e:
            print(f"  SKIP {rep}: {e}"); continue
        print(f"\n### {os.path.basename(rep.rstrip('/'))}  (N={o['_nsamp']})")
        print(hdr)
        for ch in ['sen_x', 'sen_y', 'he_x', 'he_y', 'he_z', 'e_a']:
            m = o[ch]
            print(f"{ch:7s} {m['conv']:7.3f} {m['bound']:8.3f} {m['osc']:7.3f} {m['ncross']:7d}")
            agg.setdefault(ch, []).append(m)
    if len(reps) > 1 and agg:
        print(f"\n=== MEDIAN over {len(reps)} reps (TAIL={TAIL}) ===")
        print(hdr)
        for ch in ['sen_x', 'sen_y', 'he_x', 'he_y', 'he_z', 'e_a']:
            ms = agg[ch]
            print(f"{ch:7s} {np.nanmedian([m['conv'] for m in ms]):7.3f}"
                  f" {np.nanmedian([m['bound'] for m in ms]):8.3f}"
                  f" {np.nanmedian([m['osc'] for m in ms]):7.3f}"
                  f" {int(np.median([m['ncross'] for m in ms])):7d}")


if __name__ == "__main__":
    args = sys.argv[1:]
    if not args:
        print(__doc__); sys.exit(2)
    if args[0] == "--glob":
        reps = sorted(d for d in glob.glob(args[1]) if os.path.isfile(os.path.join(d, "Control_Data.npy")))
    else:
        reps = args
    main(reps)
