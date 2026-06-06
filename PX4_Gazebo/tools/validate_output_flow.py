#!/usr/bin/env python3
"""VALIDATE the optical-flow OUTPUT against Gazebo GT, all the way to touchdown.

Read-only validation (NOT calibration — the cal is derived by derive_board_cal.py /
derive_ring_cal.py). It checks how well the ALREADY-calibrated flow tracks truth.

Checks:
  (i)  the texture-free RING flow V_v_ring (via _sensor_cal_ring; raw until M_ring derived) tracks GT V-frame flow into the final
       0.5m (R^2/slope) binned by ALTITUDE -> the L+/V-frame computation is bug-free to touchdown.
  (ii) marker->ring SWITCH continuity: corner V_v vs ring V_v_ring agreement over the overlap.
  (iii) angular w sign-check (manuscript w = -V_w_ug; flag if +V_w_ug fits better).

Improvements over v1:
  - GT V-frame flow computed locally WITHOUT the V_z>1 gate (down to z_floor) -> validates h to
    touchdown (replicates compute_gt_signals recipe; reuses agg helpers).
  - Alignment by brute-force offset that maximises corner-divergence vs GT-divergence correlation
    (the img/GT clocks differ + log past touchdown; time-to-touchdown align was too coarse).
Read-only.  Usage: validate_output_flow.py <landing_dir>

The single OUTPUT-flow validation/QA module (renamed from validate_flow_to_touchdown).
Views: channel_r2() (GT-direct per-channel — the multisine view), validate()
(GT-direct, altitude-binned to touchdown), cross_validate_ring() (GT-free
leave-one-run-out ring-cal check, folded in from compare_ring_corner_cal). prep()
is the shared load+cal+align core used by all views and by the notebook.
"""
import numpy as np, os, sys, warnings; warnings.filterwarnings('ignore')
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import aggregate_calibration_phased as agg

def _load_cal(name, default):
    """Load a sensor-cal matrix literal from src/img_data.py (single source of
    truth — no stale hard-coded copy). Falls back to `default` if not found."""
    import os, re
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src', 'img_data.py')
    try:
        src = open(path).read()
        m = re.search(r'self\.%s\s*=\s*(np\.\w+\(\[.*?\]\))' % name, src, re.S)
        return eval(m.group(1), {'np': np})
    except Exception:
        return default

# CORNER cal is what the runtime applies (getOptFlowAngVel = CAL @ corner-KF).
# RING cal is _sensor_cal_ring (identity until the co-sampled M_ring is derived);
# the runtime applies NONE to the ring today, so identity == runtime-faithful.
# NOTE the earlier bug: this tool used to apply the CORNER CAL to the ring, which
# the runtime never does — fixed to CAL_RING.
CAL      = _load_cal('_sensor_cal_hw', np.eye(6))
CAL_RING = _load_cal('_sensor_cal_ring', np.eye(6))
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
    if m.sum() < 8 or np.ptp(gt[m]) < 1e-9:   # need >=8 finite pts and non-degenerate x
        return (np.nan, np.nan)
    try:
        a = np.polyfit(gt[m], meas[m], 1)      # can throw SVD-non-converge on pathological data
    except np.linalg.LinAlgError:
        return (np.nan, np.nan)
    pred = a[0]*gt[m]+a[1]
    ss = 1 - np.sum((meas[m]-pred)**2)/(np.sum((meas[m]-np.mean(meas[m]))**2)+1e-12)
    return (float(ss), float(a[0]))


def ekf_offline(corn_cal, ring_cal, ncc, ncr, t):
    """Replicate the runtime fusion EKF (img_data._ekf_fuse_step) OFFLINE from
    CALIBRATED corner+ring flow, so the notebook's fused trace is cal-CONSISTENT
    (recomputed from raw + the CURRENT cal, NOT the recording-time-baked
    'Opt Flow Fused' log). Returns (fused[n,6]=[h_tr;w], h_tv[n,3]). corner_conf
    is approximated 1.0 (the runtime's KLT-depth down-weighting isn't logged;
    exact for clean-corner frames, the vast majority). Params mirror img_data."""
    I3 = np.eye(3); Z3 = np.zeros((3, 3))
    Hc = np.block([[I3, Z3, Z3], [Z3, Z3, I3]])     # measures [h_tr; w]
    Hr = np.block([[I3, I3, Z3], [Z3, Z3, I3]])     # measures [h_tr+h_tv; w]
    Rc = np.diag([0.05] * 6)
    Rr = np.diag([0.5, 0.5, 0.5, 1e6, 1e6, 1e6])
    Q  = np.diag([5.] * 3 + [0.2] * 3 + [5.] * 3)
    n = len(corn_cal); x = np.zeros(9); P = np.eye(9); init = False; prev = None
    fused = np.full((n, 6), np.nan); htv = np.full((n, 3), np.nan)
    def upd(x, P, z, H, R):
        S = H @ P @ H.T + R; K = P @ H.T @ np.linalg.inv(S)
        return x + K @ (z - H @ x), (np.eye(9) - K @ H) @ P
    for i in range(n):
        co = ncc[i] > 0 and np.all(np.isfinite(corn_cal[i]))
        ro = ncr[i] > 0 and np.all(np.isfinite(ring_cal[i]))
        if not init:
            if co:
                x[0:3] = corn_cal[i, 0:3]; x[6:9] = corn_cal[i, 3:6]
                if ro: x[3:6] = ring_cal[i, 0:3] - corn_cal[i, 0:3]
            elif ro:
                x[0:3] = ring_cal[i, 0:3]; x[6:9] = ring_cal[i, 3:6]
            else:
                continue
            P = np.eye(9); prev = t[i]; init = True
            fused[i] = np.concatenate([x[0:3], x[6:9]]); htv[i] = x[3:6]; continue
        dt = max(min(t[i] - prev, 0.1), 1e-3); prev = t[i]
        P = P + Q * dt                                       # predict (random walk F=I)
        if co: x, P = upd(x, P, corn_cal[i], Hc, Rc)         # corner_conf≈1.0
        if ro: x, P = upd(x, P, ring_cal[i], Hr, Rr)
        fused[i] = np.concatenate([x[0:3], x[6:9]]); htv[i] = x[3:6]
    return fused, htv


def prep(d):
    """SHARED load + calibrate + GT-align used by every validation view (the
    to-touchdown/altitude `validate`, the per-channel `channel_r2`, and the
    output-validation notebook plots). One source of truth for: GT V-frame flow,
    runtime-faithful cal application (corner=_sensor_cal_hw, ring=_sensor_cal_ring,
    EKF fused if present), and the corner-divergence time alignment. Returns a dict
    of img-grid aligned arrays."""
    img = np.load(os.path.join(d, 'Img_Data.npy'), allow_pickle=True).item()
    gt  = np.load(os.path.join(d, 'Ground_Truth.npy'), allow_pickle=True).item()
    t_g, Vh, Vw, Vz = gt_v_flow(gt)
    GT = np.hstack([Vh, -Vw])                                    # manuscript sign on w
    ti   = np.asarray(img['Time'], float)
    corn = np.asarray(img['Opt Flow Ang Vel'], float)
    ring = np.asarray(img.get('Ring Opt Flow Ang Vel', []), float)
    if ring.ndim != 2 or ring.shape[1:] != (6,) or len(ring) == 0:
        ring = np.full((len(corn), 6), np.nan)                   # ring-less recording -> nan
    ncr = np.asarray(img.get('N Ring Corners', np.zeros(len(corn))), float)
    ncc = np.asarray(img['N Flow Corners'], float)
    n = min(len(ti), len(ring), len(corn), len(ncr), len(ncc))
    ti, ring, corn, ncr, ncc = ti[:n], ring[:n], corn[:n], ncr[:n], ncc[:n]
    ring_cal = (CAL_RING @ ring.T).T
    corn_cal = (CAL @ corn.T).T
    # Recompute the fusion EKF OFFLINE from raw + the CURRENT cal so corner/ring/
    # fused are all cal-CONSISTENT (the recording's baked 'Opt Flow Fused' log is
    # recording-time-cal and would mismatch a refreshed cal). Works on any run,
    # even ones recorded before the EKF existed.
    fused, tvel = ekf_offline(corn_cal, ring_cal, ncc, ncr, ti)
    off, ar = best_offset(ti, np.where(ncc > 0, corn_cal[:, 2], np.nan), t_g, GT[:, 2])
    ti_r, tg_r = ti - ti[0], t_g - t_g[0]
    GTi = np.column_stack([np.interp(ti_r - off, tg_r, GT[:, k], left=np.nan, right=np.nan) for k in range(6)])
    Vzi = np.interp(ti_r - off, tg_r, Vz, left=np.nan, right=np.nan)
    return dict(name=os.path.basename(d.rstrip('/')), n=n, ti_r=ti_r, off=off, align_r=ar,
                corn_cal=corn_cal, ring_cal=ring_cal, fused=fused, target_vel=tvel, GTi=GTi, Vz_i=Vzi,
                ncc=ncc, ncr=ncr, GT=GT, Vw=Vw, t_g=t_g)


def channel_r2(d, show=True):
    """Per-CHANNEL R^2 of the calibrated corner / ring / EKF-fused flow vs GT —
    the MULTISINE view (cross-axis generalization). Returns {estimator: {chan: r2}}."""
    P = d if isinstance(d, dict) else prep(d)
    ests = [('corner', P['corn_cal']), ('ring', P['ring_cal'])]
    if P['fused'] is not None: ests.append(('EKF', P['fused']))
    out = {}
    if show:
        print(f"=== {P['name']} ===  align r={P['align_r']:.2f}, off={P['off']:+.1f}s")
        print(f"  {'estimator':>9} " + " ".join(f"{l:>8}" for l in ['hx','hy','hz','wx','wy','wz']))
    for nm, est in ests:
        rs = [r2_slope(est[:, k], P['GTi'][:, k])[0] for k in range(6)]
        out[nm] = dict(zip(['hx','hy','hz','wx','wy','wz'], rs))
        if show: print(f"  {nm:>9} " + " ".join(f"{r:8.2f}" for r in rs))
    return out


def validate(d):
    """To-touchdown (ALTITUDE-binned) validation on one landing dir, via prep().
    Importable from the validation notebook; main() wraps it for CLI use."""
    P = prep(d)
    corn_cal, ring_cal = P['corn_cal'], P['ring_cal']
    GT_i, Vz_i, ncr, ncc = P['GTi'], P['Vz_i'], P['ncr'], P['ncc']
    off, ti_r, Vw, t_g, n = P['off'], P['ti_r'], P['Vw'], P['t_g'], P['n']
    tg_r = t_g - t_g[0]
    if not np.any(np.isfinite(ring_cal)):
        print("  (no ring data in this recording — ring checks -> nan)")
    print(f"=== {P['name']} ===  img {n}f, GT {len(t_g)}f")
    print(f"alignment: offset {off:+.2f}s (corner-div vs GT-div corr r={P['align_r']:.2f}); GT h ungated to touchdown\n")

    print("(i) RING V_v_ring (_sensor_cal_ring) vs GT V-frame flow, binned by ALTITUDE  [R^2 | slope]")
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


def cross_validate_ring(glob_pat='calibration_data/output/*/', K=15):
    """GT-FREE ring-cal cross-validation (folded in from compare_ring_corner_cal).
    On block-denoised (K) data: (1) ring tracking survival, (2) ring_raw↔corner_raw
    correlation per DOF, (3) leave-one-run-out ring-cal R^2 (target = corner-cal
    proxy, co-sampled so no clock skew) — catches a ring cal that overfits one run.
    Read-only; no Gazebo GT needed."""
    import glob as _glob
    R, C, NR = [], [], []
    for d in sorted(_glob.glob(glob_pat), key=os.path.getmtime):
        try:
            img = np.load(os.path.join(d, 'Img_Data.npy'), allow_pickle=True).item()
            ring = np.asarray(img['Ring Opt Flow Ang Vel'], float)
            corn = np.asarray(img['Opt Flow Ang Vel'], float)
            ncr = np.asarray(img['N Ring Corners'], float); ncc = np.asarray(img['N Flow Corners'], float)
            n = min(len(ring), len(corn), len(ncr), len(ncc))
            idx = np.where((ncr[:n] > 0) & (ncc[:n] > 0))[0]
            rb = [ring[idx[j:j+K]].mean(0) for j in range(0, len(idx) - K, K)]
            cb = [corn[idx[j:j+K]].mean(0) for j in range(0, len(idx) - K, K)]
        except Exception:
            continue
        if len(rb) > 5:
            R.append(np.array(rb)); C.append(np.array(cb)); NR.append(ncr[:n])
    if len(R) < 2:
        print("cross_validate_ring: need >=2 usable runs"); return None
    NRall = np.concatenate(NR)
    print(f"GT-FREE ring-cal cross-validation: {len(R)} runs, {sum(len(r) for r in R)} blocks (K={K})")
    print(f"  (1) N_ring mean {np.nanmean(NRall):.0f}, %>=40: {100*np.mean(NRall >= 40):.0f}%")
    Rall, Call = np.vstack(R), np.vstack(C)
    print("  (2) ring_raw vs corner_raw corr:", " ".join(
        f"{LBL[k]}={np.corrcoef(Rall[:,k], Call[:,k])[0,1]:+.2f}" for k in range(6)))
    def _r2(p, t): return 1 - np.sum((p - t) ** 2, 0) / (np.sum((t - t.mean(0)) ** 2, 0) + 1e-9)
    held = np.zeros((len(R), 6))
    for i in range(len(R)):
        Rtr = np.vstack([R[j] for j in range(len(R)) if j != i])
        Ctr = np.vstack([C[j] for j in range(len(R)) if j != i])
        M, _, _, _ = np.linalg.lstsq(Rtr, (CAL @ Ctr.T).T, rcond=None)
        held[i] = _r2(R[i] @ M, (CAL @ C[i].T).T)
    print("  (3) ring-cal leave-one-run-out R^2:", dict(zip(LBL, np.round(np.nanmean(held, 0), 2))))
    return np.nanmean(held, 0)


def main():
    import argparse
    ap = argparse.ArgumentParser(description="output-flow validation/QA")
    ap.add_argument("dir", nargs="?", help="landing dir for to-touchdown validate()")
    ap.add_argument("--cross-ring", metavar="GLOB", nargs="?", const="calibration_data/output/*/",
                    help="GT-free ring-cal leave-one-out cross-validation over GLOB")
    a = ap.parse_args()
    if a.cross_ring:
        cross_validate_ring(a.cross_ring)
    elif a.dir:
        validate(a.dir)
    else:
        ap.print_help()


if __name__ == '__main__':
    main()
