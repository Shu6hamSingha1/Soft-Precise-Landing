#!/usr/bin/env python3
"""Calibrate/validate the optical-flow OUTPUT against Gazebo GT, all the way to touchdown.

Checks:
  (i)  the texture-free RING flow V_v_ring (calibrated) tracks GT V-frame flow into the final
       0.5m (R^2/slope) binned by ALTITUDE -> the L+/V-frame computation is bug-free to touchdown.
  (ii) marker->ring SWITCH continuity: corner V_v vs ring V_v_ring agreement over the overlap.
  (iii) angular w sign-check (manuscript w = -V_w_ug; flag if +V_w_ug fits better).

Improvements over v1:
  - GT V-frame flow computed locally WITHOUT the V_z>1 gate (down to z_floor) -> validates h to
    touchdown (replicates compute_gt_signals recipe; reuses agg helpers).
  - Alignment by brute-force offset that maximises corner-divergence vs GT-divergence correlation
    (the img/GT clocks differ + log past touchdown; time-to-touchdown align was too coarse).
Read-only.  Usage: calibrate_flow_to_touchdown.py <landing_dir>
"""
import numpy as np, os, sys, warnings; warnings.filterwarnings('ignore')
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import aggregate_calibration_phased as agg

CAL = np.array([
    [+0.7126,-0.0190,+0.0115,-0.0268,+0.4315,+0.0042],
    [-0.0291,+0.6817,+0.0072,-0.3727,+0.0732,-0.0099],
    [+0.0005,+0.0010,+0.8583,-0.0545,-0.0106,+0.0023],
    [+0.0050,-0.7570,-0.0153,+0.6147,-0.0552,-0.0101],
    [+0.7930,+0.0092,+0.0106,-0.0370,+0.6207,+0.0014],
    [+0.0052,+0.0364,+0.0238,+0.0171,-0.1310,+0.6088]])
LBL = ['h_x','h_y','h_z(div)','w_x','w_y','w_z']


def gt_v_flow(gt, z_floor=0.15):
    """compute_gt_signals recipe (aggregate_calibration_phased:64) but WITHOUT the V_z>1 gate
    (down to z_floor) so GT h is valid to touchdown. Returns t_g, Vh(n,3), Vw(n,3), Vz(n)."""
    NED = np.array([[0., 1, 0], [1, 0, 0], [0, 0, -1]])
    F2F = agg.FLU_2_FRD
    t_g = np.array(gt['Time']); U = np.array(gt['UAV Pose'], dtype=object); T = np.array(gt['Target Pose'], dtype=object)
    nm = min(len(t_g), len(U), len(T)); t_g, U, T = t_g[:nm], U[:nm], T[:nm]
    v = np.hstack(([True], np.diff(t_g) > 1e-6)); t_g, U, T = t_g[v], U[v], T[v]; n = len(t_g)
    R = np.zeros((n, 3, 3)); X = np.zeros((n, 3))
    for i in range(n):
        Rfe = agg.Quaternion([U[i].orientation.w, U[i].orientation.x, U[i].orientation.y, U[i].orientation.z]).to_DCM()
        R[i] = NED @ Rfe @ F2F
        X[i] = NED @ (np.array([T[i].position.x, T[i].position.y, T[i].position.z]) -
                      np.array([U[i].position.x, U[i].position.y, U[i].position.z]))
    win = min(101, (n // 2) * 2 - 1)
    Xs = agg.sgf(X, win, 2, axis=0) if win >= 5 else X
    V = np.gradient(Xs, t_g, axis=0)
    if win >= 5: V = agg.sgf(V, min(51, win), 2, axis=0)
    quats = np.array([[u.orientation.w, u.orientation.x, u.orientation.y, u.orientation.z] for u in U])
    Bw = (F2F @ agg._body_omega_from_quats(quats, t_g).T).T
    Vh = np.full((n, 3), np.nan); Vw = np.zeros((n, 3)); Vz = np.zeros(n)
    for i in range(n):
        yaw = np.arctan2(R[i, 1, 0], R[i, 0, 0])
        VR = np.array([[np.cos(yaw), np.sin(yaw), 0], [-np.sin(yaw), np.cos(yaw), 0], [0, 0, 1.]])
        Vw[i] = VR @ (R[i] @ Bw[i])
        z = X[i] @ VR[2]; Vz[i] = z
        Vh[i] = (VR @ V[i]) / (z + 0.01)   # regularize z->0 singularity (matches notebook GT)
    return t_g, Vh, Vw, Vz


def best_offset(ti, idiv, t_g, gdiv):
    """offset (s, in RELATIVE time) s.t. GT(ti_rel - off) aligns with the img frame. The clocks
    differ in absolute range AND the img includes takeoff, so off ~ takeoff duration (~12s).
    Maximise corner-div vs GT-div correlation over a wide scan."""
    ti_r = ti - ti[0]; tg_r = t_g - t_g[0]
    ic = np.isfinite(idiv); gc = np.isfinite(gdiv)
    hi = max(3.0, ti_r[-1] - tg_r[-1] + 4.0)
    best = (-2.0, 0.0)
    for off in np.arange(-2.0, hi, 0.05):
        g = np.interp(ti_r[ic] - off, tg_r[gc], gdiv[gc], left=np.nan, right=np.nan)
        m = np.isfinite(g)
        if m.sum() < 40: continue
        r = np.corrcoef(idiv[ic][m], g[m])[0, 1]
        if np.isfinite(r) and r > best[0]: best = (r, off)
    return best[1], best[0]


def r2_slope(meas, gt):
    m = np.isfinite(meas) & np.isfinite(gt)
    if m.sum() < 8: return (np.nan, np.nan)
    a = np.polyfit(gt[m], meas[m], 1); pred = a[0]*gt[m]+a[1]
    ss = 1 - np.sum((meas[m]-pred)**2)/(np.sum((meas[m]-np.mean(meas[m]))**2)+1e-12)
    return (float(ss), float(a[0]))


def main():
    d = sys.argv[1]
    img = np.load(os.path.join(d, 'Img_Data.npy'), allow_pickle=True).item()
    gt = np.load(os.path.join(d, 'Ground_Truth.npy'), allow_pickle=True).item()
    t_g, Vh, Vw, Vz = gt_v_flow(gt)
    GT = np.hstack([Vh, -Vw])                                    # manuscript sign on w
    ti = np.asarray(img['Time'], float)
    ring = np.asarray(img['Ring Opt Flow Ang Vel'], float); corn = np.asarray(img['Opt Flow Ang Vel'], float)
    ncr = np.asarray(img['N Ring Corners'], float); ncc = np.asarray(img['N Flow Corners'], float)
    n = min(len(ti), len(ring), len(corn), len(ncr), len(ncc))
    ti, ring, corn, ncr, ncc = ti[:n], ring[:n], corn[:n], ncr[:n], ncc[:n]
    ring_cal = (CAL @ ring.T).T; corn_cal = (CAL @ corn.T).T

    idiv_full = np.where(ncc > 0, corn_cal[:, 2], np.nan)
    off, xr = best_offset(ti, idiv_full, t_g, GT[:, 2])
    ti_r = ti - ti[0]; tg_r = t_g - t_g[0]
    GT_i = np.column_stack([np.interp(ti_r - off, tg_r, GT[:, k], left=np.nan, right=np.nan) for k in range(6)])
    Vz_i = np.interp(ti_r - off, tg_r, Vz, left=np.nan, right=np.nan)
    print(f"=== {os.path.basename(d.rstrip('/'))} ===  img {n}f, GT {len(t_g)}f")
    print(f"alignment: offset {off:+.2f}s (corner-div vs GT-div corr r={xr:.2f}); GT h ungated to touchdown\n")

    print("(i) RING V_v_ring (cal) vs GT V-frame flow, binned by ALTITUDE  [R^2 | slope]")
    print(f"{'altitude':>10} {'nfr':>4} " + " ".join(f"{l:>13}" for l in LBL))
    for hi, lo in [(9, 1.0), (1.0, 0.5), (0.5, 0.2), (0.2, 0.0)]:
        m = (Vz_i <= hi) & (Vz_i > lo) & (ncr > 0)
        row = f"{hi:.1f}-{lo:.1f}m".rjust(10) + f" {int(m.sum()):>4} "
        for k in range(6):
            r2, sl = r2_slope(ring_cal[m, k], GT_i[m, k]); row += f" {r2:5.2f}|{sl:5.2f}"
        print(row)
    print("  final 0.2-0.0m bin = to touchdown. R^2->1 slope->1 = tracks truth there.\n")

    print("(ii) SWITCH continuity: corner V_v vs ring V_v_ring (overlap) — the GT-free bug-check")
    both = (ncc > 0) & (ncr > 0)
    from scipy.signal import savgol_filter as sgf2
    rf = sgf2(ring_cal, 13, 1, axis=0); cf = sgf2(corn_cal, 13, 1, axis=0)
    print(f"  overlap {int(both.sum())}/{n}   {'comp':>10} {'R2 raw':>7} {'R2 savgol':>10} {'slope_savgol':>12}")
    for k in range(6):
        rr = r2_slope(ring_cal[both, k], corn_cal[both, k])[0]
        r2f, slf = r2_slope(rf[both, k], cf[both, k])
        print(f"  {'':>17} {LBL[k]:>10} {rr:7.2f} {r2f:10.2f} {slf:12.2f}")

    print("\n(iii) angular w sign-check (R^2 of ring w vs -V_w_ug [manuscript] vs +V_w_ug):")
    mok = (ncc > 0) & np.isfinite(Vz_i) & (Vz_i > 0.3)
    for k in (3, 4, 5):
        gpos = np.interp(ti_r - off, tg_r, Vw[:, k-3], left=np.nan, right=np.nan)
        r_minus = r2_slope(ring_cal[mok, k], -gpos[mok])[0]
        r_plus = r2_slope(ring_cal[mok, k], gpos[mok])[0]
        flag = "  <- +sign fits better (CHECK)" if r_plus > r_minus + 0.05 else ""
        print(f"  {LBL[k]}: R2(-V_w)={r_minus:.2f}  R2(+V_w)={r_plus:.2f}{flag}")


if __name__ == '__main__':
    main()
