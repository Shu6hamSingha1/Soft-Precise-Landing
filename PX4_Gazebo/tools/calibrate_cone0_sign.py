#!/usr/bin/env python3
"""Offline sign calibration for FUNNEL_MODE=cone0 (CBF doc Assumption 1).

cone0 computes the toward-target inertial dir  t_hat = Rz(yaw_c) @ map(s_xy/|s_xy|), where
map = optional u/v swap + per-axis sign (env CONE0_SWAP / CONE0_SIGN_X / CONE0_SIGN_Y) and
yaw_c = wrap(BODY_YAW_ALPHA_K * alpha). A WRONG map points t_hat AWAY from the target -> the
clamp frees the OUTWARD accel and limits the re-centering -> drives the marker OUT.

This finds the correct map OFFLINE from LANDING recordings (Control_Data has s(t) and I_a(t) on
one clock; Ground_Truth has the poses). Two independent scores per map, mean cos(t_hat, .):
  GT  : direction to target from UAV/Target poses (NED) -- the ground truth.
  I_a : the controller's own commanded accel dir (re-centering) -- GT-free cross-check.
The map scoring ~ +1 on BOTH is the calibration. Weighted by |s_xy| (off-centre frames). Read-only.

Usage: calibrate_cone0_sign.py [glob ...]   (default: landing recordings under test_data/)
"""
import numpy as np, glob, os, sys

NED = np.array([[0., 1, 0], [1, 0, 0], [0, 0, -1]])             # NED_from_ENU (matches gt_v_flow)
ALPHA_K = float(os.environ.get("BODY_YAW_ALPHA_K", "-0.949"))
MAPS = [(sw, sx, sy) for sw in (0, 1) for sx in (1, -1) for sy in (1, -1)]


def wrap(a): return np.arctan2(np.sin(a), np.cos(a))


def apply_map(v, sw, sx, sy):
    if sw: v = v[..., ::-1]
    return v * np.array([sx, sy])


def load(d):
    cd = np.load(os.path.join(d, 'Control_Data.npy'), allow_pickle=True).item()
    gt = np.load(os.path.join(d, 'Ground_Truth.npy'), allow_pickle=True).item()
    t = np.asarray(cd['t'], float)
    s = np.asarray(cd['s(t)'], float)                          # (n,4): s_x,s_y,1,alpha
    Ia = np.asarray(cd['I_a(t)'], float)[:, :2]                # inertial lateral accel
    n = min(len(t), len(s), len(Ia)); t, s, Ia = t[:n], s[:n], Ia[:n]
    ts = float(np.atleast_1d(gt['Start Time'])[0]) if 'Start Time' in gt else 0.0
    tg = np.asarray(gt['Time'], float) + ts                    # GT Time is stored RELATIVE to Start Time
    U, T = gt['UAV Pose'], gt['Target Pose']
    m = min(len(tg), len(U), len(T))
    uav = np.array([[U[i].position.x, U[i].position.y, U[i].position.z] for i in range(m)])
    tgt = np.array([[T[i].position.x, T[i].position.y, T[i].position.z] for i in range(m)])
    if tg[:m].min() > t.max() or tg[:m].max() < t.min():       # clocks don't overlap
        return None
    uav_i = np.column_stack([np.interp(t, tg[:m], uav[:, k]) for k in range(3)])
    tgt_i = np.column_stack([np.interp(t, tg[:m], tgt[:, k]) for k in range(3)])
    rel = (NED @ (tgt_i - uav_i).T).T[:, :2]                   # GT horizontal offset to target (NED)
    return s[:, :2], s[:, 3], Ia, rel


def main():
    globs = sys.argv[1:] or ['test_data/Landing_Test/*/', 'test_data/*/rep*/', 'test_data/*/*rep*/']
    dirs = []
    for g in globs:
        dirs += [d for d in glob.glob(g) if os.path.exists(os.path.join(d, 'Control_Data.npy'))
                 and os.path.exists(os.path.join(d, 'Ground_Truth.npy'))]
    dirs = sorted(set(dirs), key=os.path.getmtime, reverse=True)[:20]
    S, A, IA, G = [], [], [], []
    used = 0
    for d in dirs:
        try:
            r = load(d)
        except Exception:
            r = None
        if r is None: continue
        s_xy, alpha, Ia, rel = r
        S.append(s_xy); A.append(alpha); IA.append(Ia); G.append(rel); used += 1
    if not S:
        print("no usable recordings (clock overlap?)"); return
    s_xy, alpha, Ia, g = np.vstack(S), np.concatenate(A), np.vstack(IA), np.vstack(G)
    sn = np.linalg.norm(s_xy, axis=1); gn = np.linalg.norm(g, axis=1); an = np.linalg.norm(Ia, axis=1)
    m = (sn > 0.05) & (gn > 0.03) & (an > 0.3) & np.isfinite(alpha)
    s_xy, alpha, Ia, g, sn = s_xy[m], alpha[m], Ia[m], g[m], sn[m]
    g_hat = g / np.linalg.norm(g, axis=1, keepdims=True)
    Ia_hat = Ia / np.linalg.norm(Ia, axis=1, keepdims=True)
    s_hat = s_xy / sn[:, None]
    yaw = wrap(ALPHA_K * alpha); cz, sz = np.cos(yaw), np.sin(yaw)
    print(f"{used} landing recordings, {m.sum()} off-centre frames (|s|>0.05, |offset|>0.03 m)\n")
    print(f"  {'map (SWAP,SIGN_X,SIGN_Y)':28s} {'cos·GT':>8} {'cos·I_a':>8}")
    rows = []
    for (sw, sx, sy) in MAPS:
        v = apply_map(s_hat, sw, sx, sy)
        t = np.stack([cz * v[:, 0] - sz * v[:, 1], sz * v[:, 0] + cz * v[:, 1]], axis=1)   # Rz(yaw)·v
        gtc = float(np.average(np.sum(t * g_hat, 1), weights=sn))
        iac = float(np.average(np.sum(t * Ia_hat, 1), weights=sn))
        rows.append((gtc, iac, sw, sx, sy))
    best = max(rows, key=lambda r: r[0])
    for gtc, iac, sw, sx, sy in sorted(rows, key=lambda r: -r[0]):
        tag = "  <== BEST (GT)" if (gtc, iac, sw, sx, sy) == best else ""
        print(f"  SWAP={sw} SIGN_X={sx:+d} SIGN_Y={sy:+d}        {gtc:+8.3f} {iac:+8.3f}{tag}")
    print(f"\n  => set CONE0_SWAP={best[2]} CONE0_SIGN_X={best[3]:+d} CONE0_SIGN_Y={best[4]:+d}"
          f"  (cos·GT={best[0]:+.2f}, cos·I_a={best[1]:+.2f})")
    if best[0] < 0.5:
        print("  WARNING: best cos·GT < 0.5 -> weak/ambiguous; needs more off-centre data or a live test.")


if __name__ == '__main__':
    main()
