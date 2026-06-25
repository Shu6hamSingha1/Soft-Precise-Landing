#!/usr/bin/env python3
"""Per-axis behaviour breakdown for a GT-feedback rep — x, y (lateral), z (descent),
and yaw. Splits the flight into APPROACH / CYCLE / TERMINAL phases (by altitude) and
reports, per axis: the task error, the sliding variable sigma vs boundary layer E,
the adaptive gain kappa, the effective barrier gain G, and the command — so each
axis's convergence / boundedness / limit-cycle can be read off directly.

Usage: python3 tools/gt_per_axis.py "<rep_dir>"
"""
import sys, os
import numpy as np

NED = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1.]])


def _alt(gt, n):
    u, tp = gt['UAV Pose'], gt['Target Pose']
    m = min(len(u), len(tp), n)
    return np.array([(NED @ np.array([tp[i].position.x, tp[i].position.y, tp[i].position.z])
                      - NED @ np.array([u[i].position.x, u[i].position.y, u[i].position.z]))[2]
                     for i in range(m)]), m


def _cyc(x):
    """oscillation descriptors of a 1-D signal: detrended ptp + sign-changes."""
    x = np.asarray(x, float)
    if len(x) < 4:
        return 0.0, 0
    t = np.arange(len(x), dtype=float)
    A = np.column_stack([t, np.ones_like(t)])
    c, *_ = np.linalg.lstsq(A, x, rcond=None)
    r = x - A @ c
    return float(np.ptp(r)), int(np.sum(np.diff(np.sign(r)) != 0))


def main(rep):
    cd = np.load(os.path.join(rep, "Control_Data.npy"), allow_pickle=True).item()
    gt = np.load(os.path.join(rep, "Ground_Truth.npy"), allow_pickle=True).item()
    cp = np.load(os.path.join(rep, "Control_Params.npy"), allow_pickle=True).item()
    sen = np.asarray(cd['s_e_n(t)']); h = np.asarray(cd['h(t)']); hd = np.asarray(cd['h_d(t)'])
    he = h - hd
    sig = np.asarray(cd['sigma(t)']); kap = np.asarray(cd['kappa(t)']); G = np.asarray(cd['G(t)'])
    au = np.asarray(cd['a_u(t)'])
    ea = np.asarray(cd['e_a(t)']); siga = np.asarray(cd['sigma_a(t)']); kapa = np.asarray(cd['kappa_a(t)'])
    ua = np.asarray(cd['u_a(t)'])
    N = len(sen); alt, m = _alt(gt, N); N = m
    E = np.diag(cp['E']); Ea = cp['E_a']

    phases = [("APPROACH alt>2.5", alt > 2.5),
              ("CYCLE 2.5>alt>0.7", (alt <= 2.5) & (alt > 0.7)),
              ("TERMINAL alt<0.7", alt <= 0.7)]
    axes = [("x  s_e_n", sen[:N, 0], sig[:N, 0], kap[:N, 0], G[:N, 0, 0], au[:N, 0], E[0]),
            ("y  s_e_n", sen[:N, 1], sig[:N, 1], kap[:N, 1], G[:N, 1, 1], au[:N, 1], E[1]),
            ("z  h_e_z", he[:N, 2], sig[:N, 2], kap[:N, 2], G[:N, 2, 2], au[:N, 2], E[2]),
            ("yaw e_a ", ea[:N],     siga[:N],   kapa[:N],   np.full(N, np.nan), ua[:N], Ea)]

    print(f"rep: {os.path.basename(rep.rstrip('/'))}")
    print(f"E=({np.diag(E)}) E_a={Ea}  N=({np.diag(cp['N'])}) P=({np.diag(cp['P'])}) chi_r set in combined\n")
    for pname, pmask in phases:
        pm = pmask[:N]
        if pm.sum() < 3:
            continue
        print(f"=== {pname}  ({pm.sum()} samples) ===")
        print(f"{'axis':9s}{'err_mean':>9}{'err_end':>8}{'|err|max':>9}{'osc_pp':>8}{'ncross':>7}"
              f"{'|sig|/E max':>11}{'kappa rng':>14}{'|cmd|max':>9}")
        for name, err, s, k, g, cmd, e in axes:
            ep, ec = err[pm], err[pm][-1]
            opp, ncr = _cyc(ep)
            sge = np.max(np.abs(s[pm])) / e
            print(f"{name:9s}{np.mean(ep):9.3f}{ec:8.3f}{np.max(np.abs(ep)):9.3f}{opp:8.3f}{ncr:7d}"
                  f"{sge:11.2f}{k[pm].min():6.3f}->{k[pm].max():<6.3f}{np.max(np.abs(cmd[pm])):9.1f}")
        print()


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__); sys.exit(2)
    main(sys.argv[1])
