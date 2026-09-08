#!/usr/bin/env python3
"""Joint-QP convergence vs terminal blow-up — corpus scan.

Two modes, auto-selected per rep:

  DIRECT  — Control_Data.npy has `jqp_resid_final(t)` (runs from 2026-09-08 on,
            after the cbf_visibility.py instrumentation). Uses the real
            per-outer-iterate residual: converged = final < tol, chatter =
            resid_rising > 0.

  PROXY   — older reps. The joint QP writes no residual, so reconstruct the
            2026-09-04 signature (project_joint_qp_nonconvergence_kappa_ratchet)
            from what IS logged: frame-to-frame `theta_cone(t)` chatter
            (large, sign-reversing steps) co-located with a `kappa(t)` ratchet
            and an `a_u(t)` / `I_a(t)` terminal blow-up.

Per rep it prints: mode, terminal chatter score, terminal blow-up flag, and the
landing outcome (from a sibling summary.tsv when present). Then a 2x2 of
chatter x blow-up across the scanned corpus.

Usage:
  analyze_joint_qp_convergence.py [BUNDLE_GLOB ...]
  (default: the joint-QP-era bundles — ICValidation, Rover_Turning, Q8_SpinFF*)
"""
import sys, os, glob, csv
import numpy as np

TERM_FRAC = 0.20          # terminal window = last 20% of the post-takeoff samples
AU_BLOWUP = 50.0          # |a_u| (m/s^2) in the terminal window => blow-up
IA_BLOWUP = 30.0          # |I_a| (m/s^2) likewise
BURST_W = 25              # samples (~0.5 s @ ~50 Hz): the chatter is a sub-second BURST,
                          #   not a whole-window trend, so score the worst sliding window
DTH_CHATTER = 0.30        # rad: mean |d theta_cone| step inside the worst burst window
REV_FRAC = 0.35           # sign-reversal fraction of d theta_cone inside that window
RESID_TOL = 0.05          # m/s^2: DIRECT-mode "converged" threshold (matches CBF_JQP_RESID_TOL)


def _load(d):
    p = os.path.join(d, "Control_Data.npy")
    if not os.path.isfile(p):
        return None
    try:
        return np.load(p, allow_pickle=True).item()
    except Exception:
        return None


def _arr(cd, k):
    v = cd.get(k)
    if v is None:
        return None
    return np.asarray(v, dtype=float).squeeze()


def _outcome(d):
    """Landing result for rep dir `d` from a summary.tsv up the tree, else ''. """
    cur = d
    for _ in range(4):
        cur = os.path.dirname(cur)
        s = os.path.join(cur, "summary.tsv")
        if os.path.isfile(s):
            base = os.path.basename(d)
            with open(s) as fh:
                for row in csv.DictReader(fh, delimiter="\t"):
                    if row.get("result_dir") == base:
                        xy = row.get("xy_err_m", "?")
                        landed = row.get("landed", "?")
                        return f"landed={landed} xy={xy}"
    return ""


def rep_metrics(d):
    cd = _load(d)
    if cd is None:
        return None
    th = _arr(cd, "theta_cone(t)")
    if th is None or th.size < 30:
        return None
    n = th.size
    t0 = int(n * (1.0 - TERM_FRAC))
    term = slice(t0, n)

    jqp_f = _arr(cd, "jqp_resid_final(t)")
    jqp_rise = _arr(cd, "jqp_resid_rising(t)")
    direct = jqp_f is not None and np.isfinite(jqp_f).any()

    au = _arr(cd, "a_u(t)")
    ia = _arr(cd, "I_a(t)")
    if ia is not None and ia.ndim == 2:
        ia_mag = np.linalg.norm(ia, axis=1)
    else:
        ia_mag = None
    kap = _arr(cd, "kappa(t)")
    if kap is not None and kap.ndim == 2:
        kap = np.nanmax(np.abs(kap), axis=1)

    au_term = float(np.nanmax(np.abs(au[term]))) if au is not None else float("nan")
    ia_term = float(np.nanmax(ia_mag[term])) if ia_mag is not None else float("nan")
    blowup = (np.isfinite(au_term) and au_term > AU_BLOWUP) or \
             (np.isfinite(ia_term) and ia_term > IA_BLOWUP)

    # kappa ratchet in the terminal window: monotone-ish rise, end >> start
    ratchet = False
    if kap is not None and kap.size == n:
        k0 = np.nanmedian(kap[max(0, t0 - 20):t0 + 5]) if t0 > 5 else kap[0]
        k1 = np.nanmax(kap[term])
        ratchet = np.isfinite(k0) and np.isfinite(k1) and k1 > max(3.0 * k0, k0 + 1.0)

    if direct:
        mode = "DIRECT"
        f_term = jqp_f[term]
        f_term = f_term[np.isfinite(f_term)]
        rise_term = jqp_rise[term] if jqp_rise is not None else None
        conv_frac = float(np.mean(f_term < RESID_TOL)) if f_term.size else float("nan")
        chatter_score = float(np.nanmedian(f_term)) if f_term.size else float("nan")
        chatter = (f_term.size > 0 and np.nanmedian(f_term) > RESID_TOL) or \
                  (rise_term is not None and np.nanmean(rise_term) > 0.5)
        detail = f"conv_frac={conv_frac:.2f} resid_med={chatter_score:.3g}"
    else:
        mode = "PROXY"
        dd = np.diff(th[term])
        dth = np.abs(dd)
        sgn = np.sign(dd)
        # worst sub-second BURST: sliding window over the terminal region
        best_m, best_rev = 0.0, 0.0
        if dth.size >= BURST_W:
            for i in range(dth.size - BURST_W + 1):
                wdt = dth[i:i + BURST_W]
                ws = sgn[i:i + BURST_W]
                m = float(np.nanmean(wdt))
                if m > best_m:
                    best_m = m
                    best_rev = float(np.mean(ws[1:] * ws[:-1] < 0))
        else:
            best_m = float(np.nanmean(dth)) if dth.size else 0.0
            best_rev = float(np.mean(sgn[1:] * sgn[:-1] < 0)) if sgn.size > 2 else 0.0
        chatter_score = best_m
        chatter = np.isfinite(best_m) and best_m > DTH_CHATTER and best_rev > REV_FRAC
        detail = f"burst_d_theta={best_m:.3g} burst_sign_rev={best_rev:.2f}"

    return dict(dir=d, mode=mode, chatter=bool(chatter), blowup=bool(blowup),
                ratchet=bool(ratchet), au_term=au_term, ia_term=ia_term,
                detail=detail, outcome=_outcome(d))


def iter_rep_dirs(globs):
    for g in globs:
        for hit in glob.glob(g):
            if not os.path.isdir(hit):
                continue
            for root, _, files in os.walk(hit):
                if "Control_Data.npy" in files:
                    yield root


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    td = os.path.join(here, "..", "test_data")
    # DEFAULT = joint-QP era only (CBF_JOINT_QP default-on 2026-08-29). Older
    # bundles (e.g. all of Rover_Turning, July 2026) predate the joint QP entirely
    # -- their theta_cone(t) is from the theta-path CBF and is NOT a convergence
    # signal for this solver. Pass explicit globs to override.
    globs = sys.argv[1:] or [
        os.path.join(td, "ICValidation", "202609*"),
        os.path.join(td, "ICValidation", "2026083*"),
        os.path.join(td, "Q8_SpinFF*"),
        os.path.join(td, "SPCampaign_*202609*"),
        os.path.join(td, "Rover_*202609*"),
    ]
    rows = []
    for d in sorted(set(iter_rep_dirs(globs))):
        m = rep_metrics(d)
        if m:
            rows.append(m)

    if not rows:
        print("no reps with Control_Data.npy + theta_cone(t) found for:", globs)
        return

    w = max(len(os.path.relpath(r["dir"], td)) for r in rows)
    print(f"{'rep':<{w}}  mode    chat blow ratch  {'detail':<42}  outcome")
    print("-" * (w + 78))
    for r in sorted(rows, key=lambda r: (r["mode"], r["dir"])):
        print(f"{os.path.relpath(r['dir'], td):<{w}}  {r['mode']:<6}  "
              f"{'Y' if r['chatter'] else '.':<4} {'Y' if r['blowup'] else '.':<4} "
              f"{'Y' if r['ratchet'] else '.':<5}  {r['detail']:<42}  {r['outcome']}")

    # 2x2 chatter x blow-up
    import collections
    c = collections.Counter((r["chatter"], r["blowup"]) for r in rows)
    print("\n2x2  (rows=chatter, cols=terminal blow-up)      n =", len(rows))
    print(f"                 blow=Y   blow=N")
    print(f"   chatter=Y      {c[(True, True)]:>5}   {c[(True, False)]:>6}")
    print(f"   chatter=N      {c[(False, True)]:>5}   {c[(False, False)]:>6}")
    nb = sum(1 for r in rows if r["blowup"])
    nc = sum(1 for r in rows if r["chatter"])
    if nb:
        print(f"\n   P(chatter | blow-up)   = {c[(True, True)]}/{nb} = {c[(True, True)]/nb:.2f}")
    if nc:
        print(f"   P(blow-up | chatter)   = {c[(True, True)]}/{nc} = {c[(True, True)]/nc:.2f}")
    print(f"   P(blow-up | no chatter)= {c[(False, True)]}/{len(rows)-nc} = "
          f"{(c[(False, True)]/(len(rows)-nc)) if len(rows)-nc else float('nan'):.2f}")


if __name__ == "__main__":
    main()
