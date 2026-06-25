#!/usr/bin/env python3
"""Terminal yaw-alignment metric over a glob of landing rep dirs (read-only).

Usage: analyze_yaw_align.py '<glob>'   e.g. 'test_data/YawSweep/tau_ua/0.3/rep*'
       analyze_yaw_align.py '<glob>' --label tau_ua=0.3

Metric is on the CONTROL-RELEVANT e_a (pi-folded marker-orientation error the controller
acts on; the board is "square" when |e_a|->0, robust to the marker's 180 deg symmetry).
Windows by the t(t) array (loop is NOT fixed-rate). Convergence guard rejects diverged reps.
"""
import numpy as np, os, sys, glob, warnings; warnings.filterwarnings('ignore')

def rep_metrics(d):
    try:
        c = np.load(os.path.join(d, 'Control_Data.npy'), allow_pickle=True).item()
        g = np.load(os.path.join(d, 'Ground_Truth.npy'), allow_pickle=True).item()
    except Exception as e:
        return None
    t = np.asarray(c['t'], float)
    ea = np.asarray(c['e_a(t)'], float)            # pi-folded yaw error (control-relevant)
    ka = np.asarray(c['kappa_a(t)'], float)
    sen = np.asarray(c['s_e_n(t)'], float)
    smag = np.linalg.norm(sen, axis=1) if sen.ndim > 1 else np.abs(sen)
    n = min(len(t), len(ea), len(ka), len(smag))   # log arrays can differ by 1
    t, ea, ka, smag = t[:n] - t[0], ea[:n], ka[:n], smag[:n]
    W = t >= t[-1] - 1.0                            # terminal 1 s
    Wd = t >= t[-1] - 0.2                           # last 0.2 s for settle rate
    sp = g.get('SoftPrecise', {})
    deg = np.rad2deg
    dea = np.abs(np.diff(ea) / np.diff(t))
    # terminal IMPACT velocity (GT) — softness metric. The PX4 EKF rel_vel is UNRELIABLE near the
    # ground (under-reads ~50x: ~0 post-contact while true impact ~1 m/s). Use the clean GT pose:
    # drop the post-landing frozen tail, uniform-dt resample + savgol, velocity in the last 0.2s
    # BEFORE contact = the impact velocity.
    vz_e = vh_e = float('nan')
    P = g.get('UAV Pose', []); tg = np.asarray(g.get('Time', []), float)
    if len(P) > 12 and len(tg) > 12:
        from scipy.signal import savgol_filter
        pp = np.array([[q.position.x, q.position.y, q.position.z] for q in P])
        mP = min(len(pp), len(tg)); pp, tgg = pp[:mP], tg[:mP]
        mov = np.where(np.abs(np.diff(pp[:, 2])) > 1e-5)[0]
        last = (mov[-1] + 1) if len(mov) else mP - 1
        pp, tgg = pp[:last + 1], tgg[:last + 1]
        if len(tgg) > 12:
            tu = np.linspace(tgg[0], tgg[-1], len(tgg)); dtu = tu[1] - tu[0]
            ppu = np.column_stack([np.interp(tu, tgg, pp[:, k]) for k in range(3)])
            win = min(max(5, (int(0.4 / dtu) // 2) * 2 + 1), (len(tu) // 2) * 2 - 1)
            if win >= 5:
                pps = savgol_filter(ppu, win, 2, axis=0)
                vv = np.gradient(pps, axis=0) / dtu
                kk = max(1, int(0.2 / dtu))
                vz_e = float(np.nanmedian(np.abs(vv[-kk:, 2])))
                vh_e = float(np.nanmedian(np.linalg.norm(vv[-kk:, :2], axis=1)))
    return dict(
        vz_end=vz_e, vh_end=vh_e,
        rep=os.path.basename(d.rstrip('/'))[:19],
        alpha_engage=float(deg(abs(ea[0]))),
        alpha_tail_mean=float(deg(np.nanmean(np.abs(ea[W])))),
        alpha_final=float(deg(abs(ea[-1]))),
        dadt_end=float(deg(np.nanmean(dea[Wd[1:]]))) if Wd[1:].any() else 0.0,
        kappa_tail=float(np.nanmean(ka[W])),
        s_e_n_max=float(smag.max()),
        xy=float(sp.get('xy_err', np.nan)),
        rel_vel=float(sp.get('rel_vel', np.nan)),
        target_lost=bool(sp.get('target_lost', False)),
    )

def main():
    pat = sys.argv[1]
    label = sys.argv[sys.argv.index('--label')+1] if '--label' in sys.argv else pat
    dirs = sorted(glob.glob(pat))
    rows = [m for m in (rep_metrics(d) for d in dirs) if m]
    if not rows:
        print(f"no reps matched {pat}"); return
    print(f"\n=== {label}  (n={len(rows)}) ===")
    print(f"{'rep':21} {'engage':>7} {'tail|ea|':>8} {'vz_e':>5} {'vh_e':>5} "
          f"{'senmax':>6} {'xy':>5} {'rv':>5} {'TL':>3}")
    for r in rows:
        print(f"{r['rep']:21} {r['alpha_engage']:7.0f} {r['alpha_tail_mean']:8.1f} "
              f"{r['vz_end']:5.2f} {r['vh_end']:5.2f} "
              f"{r['s_e_n_max']:6.1f} {r['xy']:5.2f} {r['rel_vel']:5.2f} {'Y' if r['target_lost'] else '.':>3}")
    ok = [r for r in rows if not r['target_lost'] and r['s_e_n_max'] < 4]
    def med(k, rr): return float(np.median([r[k] for r in rr])) if rr else float('nan')
    def iqr(k, rr):
        v = [r[k] for r in rr]; return (np.percentile(v,75)-np.percentile(v,25)) if rr else float('nan')
    div = sum(1 for r in rows if r['target_lost'] or r['s_e_n_max'] >= 4)
    print(f"  --- converged n={len(ok)} (diverged {div}) ---")
    print(f"  median tail|e_a| = {med('alpha_tail_mean',ok):5.1f}deg  (IQR {iqr('alpha_tail_mean',ok):.1f})   "
          f"[PRIMARY: lower=better, target <=10]")
    print(f"  median dadt_end  = {med('dadt_end',ok):5.0f}deg/s (IQR {iqr('dadt_end',ok):.0f})   "
          f"[SETTLE: lower=better, target <=10]")
    print(f"  median vz_end = {med('vz_end',ok):.2f}m/s  vh_end = {med('vh_end',ok):.2f}m/s   "
          f"[SOFTNESS: lower=better, SP needs <0.2]")
    print(f"  median xy = {med('xy',ok):.2f}m  rel_vel = {med('rel_vel',ok):.2f}m/s   divergences = {div}")

if __name__ == '__main__':
    main()
