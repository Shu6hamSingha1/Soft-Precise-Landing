#!/usr/bin/env python3
"""
Explosion-chain analyzer: for one or more landing reps, determine WHICH
controller state departs from its normal envelope FIRST, and in what order
the rest follow. This turns "the landing failed" into a causal chain like:

    ds_d[x] spike  ->  h_e[x] > funnel  ->  zeta[x] saturates (3.66)
    ->  kappa[x] runaway  ->  |a_u| spike  ->  w_u clamp  ->  hard impact

Usage:
    python3 tools/analyze_explosion_chain.py <rep_dir> [<rep_dir> ...]
    python3 tools/analyze_explosion_chain.py --glob 'test_data/Landing_Test/Tue Jun  2 2*'

Per-channel "explosion" thresholds (chosen from MATLAB-normal ranges --
visualControl_IBVS_adaptive.m never exceeds these in a successful run):

    |s_e_n|       > 0.50        image error beyond half the normalized sensor
    |ds_d| (axis) > 5.0         desired-flow PID output spike (MATLAB peak ~2)
    |h_e|/p       > 0.95        funnel saturation (barrier ratio at clamp)
    |zeta| (axis) > 3.0         log-barrier near its 3.66 ceiling
    kappa/kappa_0 > 5.0         adaptive gain runaway
    |a_u| (axis)  > 20.0        commanded accel spike (MATLAB peak ~10)
    |w_u| (axis)  >= 0.99*clamp body-rate command pinned at W_U_MAX
    B_T           > 15.0        thrust deficit spike (N)

The analyzer reports, for each rep:
  - time of touchdown (last sample)
  - first-crossing time of each channel (or '-' if never crossed)
  - the resulting ordered chain
  - which AXIS (x/y/z) crossed first for the multi-axis channels
Then aggregates across reps: which channel is the most common FIRST mover.
"""
from __future__ import annotations
import argparse
import glob as globmod
import os
import sys

import numpy as np


def _arr(seq):
    return np.asarray([np.asarray(v, dtype=float) for v in seq])


def load_rep(rep_dir):
    cd = np.load(os.path.join(rep_dir, "Control_Data.npy"), allow_pickle=True).item()
    cp = np.load(os.path.join(rep_dir, "Control_Params.npy"), allow_pickle=True).item()
    gt_path = os.path.join(rep_dir, "Ground_Truth.npy")
    sp = {}
    if os.path.exists(gt_path):
        gt = np.load(gt_path, allow_pickle=True).item()
        sp = gt.get("SoftPrecise", {}) or {}
    return cd, cp, sp


def first_crossing(t, mask):
    """First time index where mask is True for >= 3 consecutive samples
    (debounce single-sample noise blips)."""
    idx = np.flatnonzero(mask)
    if idx.size == 0:
        return None, None
    # require 3-consecutive debounce
    for i in idx:
        if i + 2 < mask.size and mask[i] and mask[i + 1] and mask[i + 2]:
            return t[i], i
    return t[idx[0]], int(idx[0])   # fall back to first raw crossing


def analyze_rep(rep_dir, w_u_clamp=1.0, verbose=True):
    cd, cp, sp = load_rep(rep_dir)

    t = np.asarray(cd["t"], dtype=float)
    if t.size < 10:
        print(f"  [skip] {rep_dir}: only {t.size} samples")
        return None
    t = t - t[0]
    n = t.size

    def grab(key, width):
        x = _arr(cd[key])
        # channels are appended once per controller tick; some may be off by
        # one or two in length vs t -- trim to common length
        m = min(len(x), n)
        return x[:m]

    s_e_n  = grab("s_e_n(t)", 2)
    ds_d   = grab("ds_d(t)", 3)
    h      = grab("h(t)", 3)
    h_d    = grab("h_d(t)", 3)
    p      = grab("p(t)", 3)
    zeta   = grab("zeta(t)", 3)
    sigma  = grab("sigma(t)", 3)
    kappa  = grab("kappa(t)", 3)
    a_u    = grab("a_u(t)", 3)
    w_u    = grab("w_u(t)", 3)
    B_T    = grab("B_T(t)", 1)

    kappa_0 = np.asarray(cp.get("kappa_0", [0.15625, 0.15625, 0.3125]), dtype=float)

    m = min(len(s_e_n), len(ds_d), len(h), len(h_d), len(p), len(zeta),
            len(sigma), len(kappa) - 1, len(a_u), len(w_u), len(B_T), n)
    tt = t[:m]
    h_e = h[:m] - h_d[:m]
    sat_frac = np.abs(h_e) / p[:m]                      # (m, 3)
    kap_ratio = kappa[1:m + 1] / kappa_0[None, :]        # (m, 3)

    # ---- channel masks -------------------------------------------------
    AX = "xyz"
    checks = []   # (label, time, idx, axis_label)

    def add_check(label, series, thresh, axes=True):
        """series: (m,) or (m, k). Find first crossing across all axes."""
        if series.ndim == 1:
            tc, ic = first_crossing(tt, series > thresh)
            checks.append((label, tc, ic, ""))
        else:
            best = (None, None, "")
            for k in range(series.shape[1]):
                tc, ic = first_crossing(tt, series[:, k] > thresh)
                if tc is not None and (best[0] is None or tc < best[0]):
                    best = (tc, ic, AX[k] if axes else str(k))
            checks.append((label, best[0], best[1], best[2]))

    add_check("|s_e_n| > 0.5",       np.abs(s_e_n[:m]), 0.5)
    add_check("|ds_d| > 5",          np.abs(ds_d[:m]), 5.0)
    add_check("|h_e|/p > 0.95",      sat_frac, 0.95)
    add_check("|zeta| > 3.0",        np.abs(zeta[:m]), 3.0)
    add_check("|sigma| > 3.5",       np.abs(sigma[:m]), 3.5)
    add_check("kappa/k0 > 5",        kap_ratio, 5.0)
    add_check("|a_u| > 20",          np.abs(a_u[:m]), 20.0)
    add_check("|w_u| pinned",        np.abs(w_u[:m]), 0.99 * w_u_clamp)
    add_check("B_T > 15 N",          np.abs(B_T[:m]).reshape(-1), 15.0)

    chain = sorted([c for c in checks if c[1] is not None], key=lambda c: c[1])
    never = [c[0] for c in checks if c[1] is None]

    if verbose:
        name = os.path.basename(rep_dir.rstrip("/"))
        xy = sp.get("xy_err", float("nan"))
        rv = sp.get("rel_vel", float("nan"))
        tl = sp.get("target_lost", False)
        print(f"\n=== {name}  (xy={xy:.3f} m, rel_vel={rv:.3f} m/s, "
              f"{'TARGET_LOST' if tl else 'completed'})  T_flight={tt[-1]:.1f}s ===")
        print(f"  {'event':<22} {'t [s]':>8}  {'t-T_end [s]':>12}  axis")
        for label, tc, ic, ax in chain:
            print(f"  {label:<22} {tc:>8.2f}  {tc - tt[-1]:>12.2f}  {ax}")
        if never:
            print(f"  never crossed: {', '.join(never)}")

        # Touchdown zoom: last 1.5 s, per-axis peaks
        zoom = tt > (tt[-1] - 1.5)
        print(f"  -- final 1.5 s peaks --")
        print(f"     ds_d   peak/axis : {np.max(np.abs(ds_d[:m][zoom]), axis=0).round(2)}")
        print(f"     h_e    peak/axis : {np.max(np.abs(h_e[zoom]), axis=0).round(2)}")
        print(f"     h meas peak/axis : {np.max(np.abs(h[:m][zoom]), axis=0).round(2)}")
        print(f"     sat_frac max     : {np.max(sat_frac[zoom], axis=0).round(2)}")
        print(f"     zeta   peak/axis : {np.max(np.abs(zeta[:m][zoom]), axis=0).round(2)}")
        print(f"     kappa  end/axis  : {kappa[min(m, len(kappa)-1)].round(3)}  (kappa_0={kappa_0.round(3)})")
        print(f"     a_u    peak/axis : {np.max(np.abs(a_u[:m][zoom]), axis=0).round(1)}")
        print(f"     w_u    peak/axis : {np.max(np.abs(w_u[:m][zoom]), axis=0).round(2)}")

    return {
        "rep": os.path.basename(rep_dir.rstrip("/")),
        "chain": [(c[0], c[1], c[3]) for c in chain],
        "first": chain[0][0] if chain else None,
        "first_axis": chain[0][3] if chain else None,
        "xy": sp.get("xy_err"),
        "rel_vel": sp.get("rel_vel"),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("reps", nargs="*", help="rep directories")
    ap.add_argument("--glob", help="glob pattern for rep dirs")
    ap.add_argument("--w-u-clamp", type=float, default=1.0)
    args = ap.parse_args()

    reps = list(args.reps)
    if args.glob:
        reps += sorted(globmod.glob(args.glob))
    reps = [r for r in reps if os.path.isdir(r)]
    if not reps:
        print("No rep directories found.")
        sys.exit(1)

    results = []
    for rep in reps:
        try:
            r = analyze_rep(rep, w_u_clamp=args.w_u_clamp)
            if r:
                results.append(r)
        except Exception as e:
            print(f"  [error] {rep}: {e}")

    # ---- aggregate -------------------------------------------------------
    if len(results) > 1:
        print("\n" + "=" * 70)
        print("AGGREGATE: first-exploding channel across reps")
        print("=" * 70)
        from collections import Counter
        firsts = Counter((r["first"], r["first_axis"]) for r in results if r["first"])
        for (label, ax), cnt in firsts.most_common():
            print(f"  {cnt}/{len(results)} reps: {label}  (axis {ax})")
        # second movers
        seconds = Counter(r["chain"][1][0] for r in results if len(r["chain"]) > 1)
        print("  -- second movers --")
        for label, cnt in seconds.most_common():
            print(f"  {cnt}/{len(results)} reps: {label}")


if __name__ == "__main__":
    main()
