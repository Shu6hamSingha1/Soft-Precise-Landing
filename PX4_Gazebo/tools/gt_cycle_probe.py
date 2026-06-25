#!/usr/bin/env python3
"""Probe the lateral limit cycle + kappa-vs-effective-gain for a GT-feedback rep.

Reports, over the descent: whether kappa_xy tracks the effective gain G_xx (~1/Z),
how much time sigma_xy spends inside the boundary layer E (switching linearized
away), and the lateral cycle amplitude (sign-changes + peak metric overshoot).

Usage: python3 tools/gt_cycle_probe.py "<rep_dir>" [more...]
"""
import sys, os
import numpy as np

NED = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1.]])


def _alt_lat(gt):
    u, tp = gt['UAV Pose'], gt['Target Pose']
    m = min(len(u), len(tp))
    W = np.array([(NED @ np.array([tp[i].position.x, tp[i].position.y, tp[i].position.z])
                   - NED @ np.array([u[i].position.x, u[i].position.y, u[i].position.z])) for i in range(m)])
    return W[:, 2], np.linalg.norm(W[:, :2], axis=1)


def probe(rep):
    cd = np.load(os.path.join(rep, "Control_Data.npy"), allow_pickle=True).item()
    gt = np.load(os.path.join(rep, "Ground_Truth.npy"), allow_pickle=True).item()
    sig = np.asarray(cd['sigma(t)']); kap = np.asarray(cd['kappa(t)'])[:len(sig)]
    G = np.asarray(cd['G(t)']); sen = np.asarray(cd['s_e_n(t)'])
    cp = np.load(os.path.join(rep, "Control_Params.npy"), allow_pickle=True).item()
    E = np.diag(cp['E'])
    alt, lat = _alt_lat(gt)
    n = min(len(sig), len(alt))
    # descent window: alt 2.5m -> 0.5m (the cycle zone, before terminal)
    win = (alt[:n] < 2.5) & (alt[:n] > 0.5)
    Gxx = G[:n, 0, 0]
    insideBL = np.mean((np.abs(sig[:n, 0]) / E[0] < 1)[win]) if win.any() else np.nan
    # kappa-vs-G tracking: correlation over the descent
    kx = kap[:n, 0]
    corr = np.corrcoef(Gxx[win], kx[win])[0, 1] if win.sum() > 5 else np.nan
    print(f"### {os.path.basename(rep.rstrip('/'))}")
    print(f"  E_xy={E[0]:.2f}  N=({np.diag(cp['N'])})  P=({np.diag(cp['P'])})")
    print(f"  G_xx: {Gxx[win].min():.2f}->{Gxx[win].max():.2f} (eff gain ~1/Z)")
    print(f"  kappa_x over cycle: {kx[win].min():.3f}->{kx[win].max():.3f}   corr(kappa,G)={corr:+.2f}")
    print(f"  sigma_x inside boundary layer: {100*insideBL:.0f}% of cycle  (high => switching asleep)")
    sgn = np.sign(sen[:n, 0][win]); ncross = int(np.sum(np.diff(sgn) != 0))
    print(f"  lateral sign-changes in cycle: {ncross}   lat_max={lat[:n][win].max():.2f}m  lat_end={lat[n-1]:.2f}m")
    print()


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__); sys.exit(2)
    for r in sys.argv[1:]:
        try:
            probe(r)
        except Exception as e:
            print(f"  SKIP {r}: {e}\n")
