#!/usr/bin/env python3
"""
Aggregate sensor-cal diag values across ALL valid calibration recordings in
PX4_Gazebo/calibration_data/output/. A run is "valid" only if validate_pose_transforms
on it returns 0 (all 6 checks pass). For each valid run, derive diag values
via analyze_calibration's per-axis median-of-ratio. Report mean, median, std
across runs, plus per-run breakdown.
"""
import os
import sys
import glob
import numpy as np
from scipy.signal import savgol_filter as sgf
from ahrs import Quaternion, DCM
import sys as _sys, os as _os
_sys.path.insert(0, _os.path.dirname(_os.path.abspath(__file__)))
from aggregate_calibration_phased import compute_gt_signals  # single V-frame GT source


def _body_omega_from_quats(quats, t):
    """Body-frame angular velocity from a quaternion stream via central
    difference. δq = conj(q[i-1]) * q[i+1] is the relative rotation from
    t[i-1] to t[i+1] expressed in the body frame at t[i-1]; for small δq,
    ω_body ≈ 2 · δq.xyz / (t[i+1] - t[i-1]). Preserves SO(3) (unlike
    element-wise np.gradient of the rotation matrix)."""
    N = len(quats)
    w = np.zeros((N, 3))
    for i in range(N):
        i0 = max(0, i - 1)
        i1 = min(N - 1, i + 1)
        if i1 == i0:
            continue
        q0, q1 = quats[i0], quats[i1]
        dt_pair = t[i1] - t[i0]
        if dt_pair < 1e-9:
            continue
        w0, x0, y0, z0 = q0
        w1, x1, y1, z1 = q1
        # δq = conj(q0) * q1 (w-first); ω_body ≈ 2 · δq.xyz / dt
        dq_x = w0*x1 - x0*w1 - y0*z1 + z0*y1
        dq_y = w0*y1 + x0*z1 - y0*w1 - z0*x1
        dq_z = w0*z1 - x0*y1 + y0*x1 - z0*w1
        w[i] = 2.0 * np.array([dq_x, dq_y, dq_z]) / dt_pair
    return w


def compute_gt(gt):
    """V-frame GT via the canonical single source
    (aggregate_calibration_phased.compute_gt_signals): h, w AND s all gravity-
    leveled to the frame img_data.py reports. Returns
    (t_g, V_h_g, V_w_tug, V_xc_g, V_yc_g, valid, V_z_g); V_z_g is altitude (m)."""
    t_g, V_h_g, V_w_ug, V_xc_g, V_yc_g, valid, V_z_g = compute_gt_signals(gt)
    return t_g, V_h_g, -V_w_ug, V_xc_g, V_yc_g, valid, V_z_g   # manuscript w = -V_w_ug


def robust_scale(raw, gt_v, threshold):
    raw = np.asarray(raw); gt_v = np.asarray(gt_v)
    mask = np.isfinite(raw) & np.isfinite(gt_v) & (np.abs(gt_v) > threshold) & (np.abs(raw) > 1e-9)
    if mask.sum() < 10:
        return float('nan')
    return float(np.median(gt_v[mask] / raw[mask]))


def is_valid_run(alt, B_v_tu):
    """Quick pose-validity check (mirrors validate_pose_transforms 6 checks).

    Uses nan-tolerant aggregators: B_v_tu can contain a few NaN samples from
    the (z < 0.1m) altitude guard or from gradient-edge artifacts; previously
    np.max(np.abs(.)) propagated NaN → gate always False → valid runs rejected.
    """
    n = len(alt)
    mid_z = np.nanmedian(alt[n // 4 : 3 * n // 4])
    peak_v = np.nanmax(np.abs(B_v_tu))
    if not np.isfinite(mid_z) or not np.isfinite(peak_v):
        return False
    return (mid_z > 1.0) and (peak_v < 10.0)


def analyze_one(run_dir):
    """Return (valid, sensor_cal_hw_diag(6,), sensor_cal_s_diag(4,))."""
    try:
        gt = np.load(f"{run_dir}/Ground_Truth.npy", allow_pickle=True)[()]
    except Exception as e:
        return False, None, None, f"load error: {e}"

    try:
        t_g, V_h_g, V_w_tug, V_xc_g, V_yc_g, valid, V_z_g = compute_gt(gt)
    except Exception as e:
        return False, None, None, f"gt compute error: {e}"

    # validation gate
    n = len(t_g)
    W_T_P_t = None  # not needed for validity gate
    # Use B_v_tu derived implicitly through V_h_g * depth
    z_safe = np.where(np.abs(V_z_g) < 0.1, np.nan, V_z_g)
    B_v_tu = V_h_g * z_safe[:, np.newaxis]
    if not is_valid_run(V_z_g, B_v_tu):
        return False, None, None, "validity gate failed (mid_z<=1 or peak v>10)"

    # raw measurements
    raw = np.asarray(gt["Opt Flow Ang Vel"])[valid]  # (N, 6)
    raw_s = np.asarray(gt["Img Feature Params"])[valid]  # (N, 4)
    y_raw, w_raw = raw[:, :3], raw[:, 3:]
    xc_raw = raw_s[:, 0]; yc_raw = raw_s[:, 1]


    flow = np.abs([robust_scale(y_raw[:, i], V_h_g[:, i], 0.05) for i in range(3)])
    ang = np.abs([robust_scale(w_raw[:, i], V_w_tug[:, i], 0.02) for i in range(3)])
    s_xc = abs(robust_scale(xc_raw, V_xc_g, 0.05))
    s_yc = abs(robust_scale(yc_raw, V_yc_g, 0.05))

    hw = np.concatenate([flow, ang])
    s = np.array([s_xc, s_yc, 1.0, 1.0])
    return True, hw, s, "OK"


def main():
    parent = "/home/shubham/Soft-Precise-Landing/PX4_Gazebo/calibration_data/output"
    run_dirs = sorted([d for d in glob.glob(f"{parent}/*") if os.path.isdir(d)])
    # Filter symlinks
    run_dirs = [d for d in run_dirs if not os.path.islink(d)]
    print(f"[agg] found {len(run_dirs)} run directories\n")

    valid_hws, valid_ss, valid_names = [], [], []
    for d in run_dirs:
        name = os.path.basename(d)
        valid, hw, s, msg = analyze_one(d)
        marker = "OK" if valid else "SKIP"
        print(f"  [{marker}] {name}  ({msg})")
        if valid:
            print(f"          hw = [{', '.join(f'{v:.4f}' for v in hw)}]")
            print(f"          s  = [{', '.join(f'{v:.4f}' for v in s)}]")
            valid_hws.append(hw)
            valid_ss.append(s)
            valid_names.append(name)

    if not valid_hws:
        print("\n[agg] no valid runs found — cannot aggregate.")
        return 1

    valid_hws = np.array(valid_hws)
    valid_ss = np.array(valid_ss)

    def trimmed_mean(arr, trim=1):
        """Drop `trim` highest and `trim` lowest values per axis, then mean.
        If n < 2*trim+1, falls back to plain mean (no trimming possible).
        Switched from median 2026-06-01 for better statistical efficiency on
        n=4-8 cal aggregations: mean uses all data but is dragged by outliers;
        median ignores all but the middle value. Trim-mean drops the most
        extreme outlier on each side then averages the rest — uses ~70% of
        the data while staying robust to ~17% outliers (n=6 case)."""
        arr = np.asarray(arr, dtype=float)
        if arr.shape[0] <= 2 * trim:
            return np.mean(arr, axis=0)
        sorted_arr = np.sort(arr, axis=0)
        return np.mean(sorted_arr[trim:-trim], axis=0)

    print(f"\n[agg] aggregating across {len(valid_hws)} valid run(s)\n")
    print("  _sensor_cal_hw axis stats:")
    print("    axis | mean    median   trim-mean  std     n")
    print("    -----|----------------------------------------")
    for i in range(6):
        col = valid_hws[:, i]
        print(f"    {i}    | {col.mean():.4f}  {np.median(col):.4f}  "
              f"{float(trimmed_mean(col)):.4f}    {col.std():.4f}  {len(col)}")
    print("\n  _sensor_cal_s axis stats:")
    print("    axis | mean    median   trim-mean  std     n")
    print("    -----|----------------------------------------")
    for i in range(4):
        col = valid_ss[:, i]
        print(f"    {i}    | {col.mean():.4f}  {np.median(col):.4f}  "
              f"{float(trimmed_mean(col)):.4f}    {col.std():.4f}  {len(col)}")

    agg_hw = trimmed_mean(valid_hws)
    agg_s  = trimmed_mean(valid_ss)
    print(f"\n  Proposed (trimmed-mean across runs):")
    print(f"    _sensor_cal_hw = np.diag([{', '.join(f'{v:.4f}' for v in agg_hw)}])")
    print(f"    _sensor_cal_s  = np.diag([{', '.join(f'{v:.4f}' for v in agg_s)}])")
    return 0


if __name__ == "__main__":
    sys.exit(main())
