"""GT-direct validation of the cross-marker's calibrated h,w/s vs Gazebo GT,
on an INDEPENDENT multisine/landing recording (apps/record_cross_marker_validation.py)
-- never on calibration_data/output_cross/ itself (see io-calibration skill's
train/validate discipline). Mirrors tools/validate_output_flow.py's channel_r2()
view but for CrossMarkerPerception's own raw hw/s + cal (no ring/map subsystems).

Auto-loads the LIVE cal straight from CrossMarkerPerception (instantiating it
just reads __init__'s self._sensor_cal_hw/_sensor_cal_s -- no camera needed),
so this always reflects whatever is currently pasted in the source, never a
stale copy.

Usage: python3 tools/validate_cross_marker_flow.py <validation_run_dir>
"""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src'))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np

from cross_marker_perception import CrossMarkerPerception
from aggregate_calibration_phased import (compute_gt_signals, kf_filter_causal,
                                           FLOW_KF_Q, FLOW_KF_R, FEAT_KF_Q, FEAT_KF_R)

LAB = ['Hx', 'Hy', 'Hz', 'Wx', 'Wy', 'Wz']


def prep(d):
    """Shared load+cal+align core -- returns aligned (G_hw, cal_hw_pred, V_z_g,
    det_ok, G_s, cal_s_pred) for either channel_r2() (whole-flight) or
    validate_landing() (altitude-binned) to slice as needed."""
    p = CrossMarkerPerception()
    cal_hw, cal_s = p._sensor_cal_hw, p._sensor_cal_s

    gt = np.load(os.path.join(d, 'Ground_Truth.npy'), allow_pickle=True).item()
    t, V_h_g, V_w_ug, V_xc_g, V_yc_g, valid, V_z_g = compute_gt_signals(gt)
    V_w_ug = V_w_ug.copy(); V_w_ug[:, 0:2] = 0.0   # V-frame w is yaw-only (see derive tools)

    raw = np.asarray(gt['Opt Flow Ang Vel'])
    raw_s = np.asarray(gt['Img Feature Params'])
    nm = min(len(raw), len(raw_s), len(valid))
    # t/V_h_g/etc from compute_gt_signals are already reduced to the `valid`
    # subsequence internally; det_ok must be reduced the SAME way (not
    # independently masked) to stay positionally aligned with them.
    det_ok_full = ~np.all(raw[:nm] == 0, axis=1)   # CrossMarkerPerception zeros
                                                     # _hw (holds stale s/alpha)
                                                     # whenever det.ok is False --
                                                     # a spurious zero-flow reading
                                                     # during dropout would otherwise
                                                     # silently corrupt the R^2 fit.
    det_ok_frac = det_ok_full.mean()
    raw = raw[:nm][valid[:nm]]; raw_s = raw_s[:nm][valid[:nm]]
    det_ok = det_ok_full[valid[:nm]]   # same subsequence/order as V_h_g/t now

    n_kf = min(len(raw), len(t))
    raw = raw[:n_kf]; raw_s = raw_s[:n_kf]; det_ok = det_ok[:n_kf]
    if n_kf > 1:
        raw = kf_filter_causal(raw, t[:n_kf], FLOW_KF_Q, FLOW_KF_R)
        raw_s = kf_filter_causal(raw_s, t[:n_kf], FEAT_KF_Q, FEAT_KF_R)

    n = min(len(raw), len(V_h_g), len(det_ok), len(V_z_g))
    G_hw = np.hstack([V_h_g[:n], -V_w_ug[:n]])          # GT [h;w], manuscript w = -V_w_ug
    cal_hw_pred = raw[:n] @ cal_hw.T                    # calibrated = cal @ raw
    cal_s_pred = raw_s[:n] @ cal_s.T
    G_s = np.column_stack([V_xc_g[:n], V_yc_g[:n]])
    V_z = V_z_g[:n]

    return dict(cal_hw=cal_hw, cal_s=cal_s, G_hw=G_hw, cal_hw_pred=cal_hw_pred,
                G_s=G_s, cal_s_pred=cal_s_pred, V_z=V_z, det_ok=det_ok,
                det_ok_frac=det_ok_frac, name=os.path.basename(d))


def channel_r2(d):
    P = prep(d)
    print("live cal loaded from CrossMarkerPerception:")
    np.set_printoptions(precision=4, suppress=True, linewidth=130)
    print(P['cal_hw']); print(P['cal_s'])
    print(f"detection ok fraction in this recording: {P['det_ok_frac']:.1%}")

    m = (np.all(np.isfinite(P['G_hw']), 1) & np.all(np.isfinite(P['cal_hw_pred']), 1)
         & P['det_ok'])
    G_hw, cal_hw_pred = P['G_hw'][m], P['cal_hw_pred'][m]
    print(f"\n{len(G_hw)} valid co-sampled points")

    ss = 1 - np.sum((G_hw - cal_hw_pred) ** 2, 0) / np.sum((G_hw - G_hw.mean(0)) ** 2, 0)
    print("\nper-channel R^2 (calibrated h,w vs GT, independent validation data):")
    for lab, r2 in zip(LAB, ss):
        print(f"  {lab:3s} R^2={r2:+.3f}")

    G_s, cal_s_pred, det_ok = P['G_s'], P['cal_s_pred'], P['det_ok']
    mx = np.isfinite(G_s[:, 0]) & np.isfinite(cal_s_pred[:, 0]) & det_ok
    my = np.isfinite(G_s[:, 1]) & np.isfinite(cal_s_pred[:, 1]) & det_ok
    r2x = 1 - np.sum((G_s[mx, 0] - cal_s_pred[mx, 0])**2) / np.sum((G_s[mx, 0] - G_s[mx, 0].mean())**2)
    r2y = 1 - np.sum((G_s[my, 1] - cal_s_pred[my, 1])**2) / np.sum((G_s[my, 1] - G_s[my, 1].mean())**2)
    print(f"\ncentroid R^2: xc={r2x:+.3f}  yc={r2y:+.3f}")


def _compute_gt_flow_zreg(rep_dir, Z_REG):
    """Depth-regularized GT flow, 1/(z+Z_REG) -- matches gt_feedback.py's
    Z_REG convention (its own extensively-documented history: Z_REG=0.01
    let computed z fall low enough that 1/z->100, a non-physical spike that
    caused a real terminal instability bug there; empirically the true
    touchdown floor is ~0.096-0.14m, landing gear + perception saturation
    -- baked to 0.1-0.2 there, NOT gt_optical_flow.py's much smaller 0.01,
    which is fine for its normal (in-flight, altitude-gated) use but produces
    the same non-physical blowup gt_feedback.py's history already diagnosed
    when used down to true touchdown, as a landing validation needs. No
    altitude gate (unlike compute_gt_flow's abs(zB)>=0.1) -- 1/(z+Z_REG)
    stays bounded to the deck on its own, same as gt_feedback.py's own
    comment: 'no altitude gate: 1/(z+Z_REG) stays bounded to the deck'."""
    from ahrs import Quaternion
    from gt_optical_flow import NED_FROM_ENU, FRD_2_FLU, _v_frame, _robust_vel
    gt = np.load(os.path.join(rep_dir, "Ground_Truth.npy"), allow_pickle=True).item()
    St = float(gt['Start Time'])
    tg = np.asarray(gt['Time'], float)
    u, tp = gt['UAV Pose'], gt['Target Pose']
    n = min(len(tg), len(u), len(tp))
    tg, u, tp = tg[:n], u[:n], tp[:n]
    keep = np.hstack(([True], np.diff(tg) > 1e-6))
    tg = tg[keep]; u = [u[i] for i in range(n) if keep[i]]; tp = [tp[i] for i in range(n) if keep[i]]
    n = len(tg)
    W_x_tu = np.zeros((n, 3)); Ru = np.zeros((n, 3, 3))
    for i in range(n):
        p, t = u[i], tp[i]
        Rfu = Quaternion([p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z]).to_DCM()
        Ru[i] = NED_FROM_ENU @ Rfu @ FRD_2_FLU
        up = NED_FROM_ENU @ np.array([p.position.x, p.position.y, p.position.z])
        tpp = NED_FROM_ENU @ np.array([t.position.x, t.position.y, t.position.z])
        W_x_tu[i] = tpp - up
    W_v_tu = _robust_vel(W_x_tu, tg)
    V_h_g = np.full((n, 3), np.nan)
    for i in range(n):
        zB = max(float(W_x_tu[i, 2]), 0.0)
        B_v = Ru[i].T @ W_v_tu[i]
        V_v = _v_frame(Ru[i]) @ B_v
        V_h_g[i] = V_v / (zB + Z_REG)

    def align(t_abs, y):
        ti = np.asarray(t_abs, float) - St
        return np.column_stack([np.interp(tg, ti, y[:, k]) for k in range(y.shape[1])])
    return dict(t_g=tg, alt=W_x_tu[:, 2], V_h_g=V_h_g, align=align)


def validate_landing(d, z_reg=None):
    """To-touchdown (ALTITUDE-binned) validation on one landing-profile recording.
    Uses _compute_gt_flow_zreg (PLASMC_GT_Z_REG-regularized, default 0.1,
    matching gt_feedback.py's own converged value) instead of prep()'s
    compute_gt_signals (Vz>1.0m gate, fine for calibration -- always flown at
    altitude -- but useless here: it NaNs out everything below 1m, exactly the
    range a landing validation exists to check). Only checks translational h
    (Hx,Hy,Hz) -- no GT angular-rate reference available here, fine for a
    straight-down zero-yaw descent profile."""
    Z_REG = z_reg if z_reg is not None else float(os.environ.get("PLASMC_GT_Z_REG", "0.1"))
    g = _compute_gt_flow_zreg(d, Z_REG)
    p = CrossMarkerPerception()
    cal_hw = p._sensor_cal_hw

    img = np.load(os.path.join(d, 'Img_Data.npy'), allow_pickle=True).item()
    t_img = np.asarray(img['Time'])            # true per-detect-call, absolute clock
    raw_hw = np.asarray(img['h_V'])             # raw (pre-cal) [Tx,Ty,Tz,Wx,Wy,Wz]
    cal_pred = raw_hw @ cal_hw.T

    h_meas = g['align'](t_img, cal_pred[:, :3])   # -> GT's own time grid
    alt, V_h_g = g['alt'], g['V_h_g']
    LABH = ['Hx', 'Hy', 'Hz']
    print(f"(Z_REG={Z_REG})")
    m = np.all(np.isfinite(V_h_g), 1) & np.all(np.isfinite(h_meas), 1) & np.isfinite(alt)
    alt_m, G, P_ = alt[m], V_h_g[m], h_meas[m]
    print(f"=== {os.path.basename(d)} ===  n={len(G)}  alt {alt[0]:.2f}->{alt[-1]:.2f}m\n")
    print("calibrated h (translation only) vs GT, binned by ALTITUDE  [R^2 | slope]")
    print(f"{'altitude':>10} {'nfr':>5} " + " ".join(f"{l:>13}" for l in LABH))
    for hi, lo in [(9, 2.0), (2.0, 1.0), (1.0, 0.5), (0.5, 0.2), (0.2, 0.0)]:
        bm = (alt_m <= hi) & (alt_m > lo)
        row = f"{hi:.1f}-{lo:.1f}m".rjust(10) + f" {int(bm.sum()):>5} "
        for k in range(3):
            gk, pk = G[bm, k], P_[bm, k]
            if bm.sum() < 5 or np.std(gk) < 1e-9:
                row += f" {'n/a':>13}"; continue
            r2 = 1 - np.sum((gk - pk) ** 2) / np.sum((gk - gk.mean()) ** 2)
            sl = np.polyfit(gk, pk, 1)[0] if np.std(gk) > 1e-9 else np.nan
            row += f" {r2:5.2f}|{sl:5.2f} "
        print(row)
    print("  final 0.2-0.0m bin = to touchdown. R^2->1 slope->1 = tracks truth there.")


def main():
    if len(sys.argv) < 2:
        print("usage: validate_cross_marker_flow.py <validation_run_dir> [--landing]"); return
    d = sys.argv[1]
    if '--landing' in sys.argv:
        validate_landing(d)
    else:
        channel_r2(d)


if __name__ == "__main__":
    main()
