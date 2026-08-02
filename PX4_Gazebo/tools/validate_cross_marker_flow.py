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


def main():
    if len(sys.argv) < 2:
        print("usage: validate_cross_marker_flow.py <validation_run_dir>"); return
    d = sys.argv[1]

    p = CrossMarkerPerception()
    cal_hw, cal_s = p._sensor_cal_hw, p._sensor_cal_s
    print("live cal loaded from CrossMarkerPerception:")
    np.set_printoptions(precision=4, suppress=True, linewidth=130)
    print(cal_hw)
    print(cal_s)

    gt = np.load(os.path.join(d, 'Ground_Truth.npy'), allow_pickle=True).item()
    t, V_h_g, V_w_ug, V_xc_g, V_yc_g, valid, _ = compute_gt_signals(gt)
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
    print(f"detection ok fraction in this recording: {det_ok_full.mean():.1%}")
    raw = raw[:nm][valid[:nm]]; raw_s = raw_s[:nm][valid[:nm]]
    det_ok = det_ok_full[valid[:nm]]   # same subsequence/order as V_h_g/t now

    n_kf = min(len(raw), len(t))
    raw = raw[:n_kf]; raw_s = raw_s[:n_kf]; det_ok = det_ok[:n_kf]
    if n_kf > 1:
        raw = kf_filter_causal(raw, t[:n_kf], FLOW_KF_Q, FLOW_KF_R)
        raw_s = kf_filter_causal(raw_s, t[:n_kf], FEAT_KF_Q, FEAT_KF_R)

    n = min(len(raw), len(V_h_g), len(det_ok))
    G_hw = np.hstack([V_h_g[:n], -V_w_ug[:n]])          # GT [h;w], manuscript w = -V_w_ug
    cal_hw_pred = raw[:n] @ cal_hw.T                    # calibrated = cal @ raw

    m = (np.all(np.isfinite(G_hw), 1) & np.all(np.isfinite(cal_hw_pred), 1)
         & det_ok[:n])
    G_hw, cal_hw_pred = G_hw[m], cal_hw_pred[m]
    print(f"\n{len(G_hw)} valid co-sampled points")

    ss = 1 - np.sum((G_hw - cal_hw_pred) ** 2, 0) / np.sum((G_hw - G_hw.mean(0)) ** 2, 0)
    print("\nper-channel R^2 (calibrated h,w vs GT, independent validation data):")
    for lab, r2 in zip(LAB, ss):
        print(f"  {lab:3s} R^2={r2:+.3f}")

    # centroid: GT xc/yc vs calibrated raw_s xc/yc
    ng = min(len(V_xc_g), len(raw_s), len(det_ok))
    cal_s_pred = raw_s[:ng] @ cal_s.T
    mx = np.isfinite(V_xc_g[:ng]) & np.isfinite(cal_s_pred[:, 0]) & det_ok[:ng]
    my = np.isfinite(V_yc_g[:ng]) & np.isfinite(cal_s_pred[:, 1]) & det_ok[:ng]
    r2x = 1 - np.sum((V_xc_g[:ng][mx] - cal_s_pred[mx, 0])**2) / np.sum((V_xc_g[:ng][mx] - V_xc_g[:ng][mx].mean())**2)
    r2y = 1 - np.sum((V_yc_g[:ng][my] - cal_s_pred[my, 1])**2) / np.sum((V_yc_g[:ng][my] - V_yc_g[:ng][my].mean())**2)
    print(f"\ncentroid R^2: xc={r2x:+.3f}  yc={r2y:+.3f}")


if __name__ == "__main__":
    main()
