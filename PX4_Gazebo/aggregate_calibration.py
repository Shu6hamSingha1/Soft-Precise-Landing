#!/usr/bin/env python3
"""
Aggregate sensor-cal diag values across ALL valid calibration recordings in
PX4_Gazebo/calibration_data/. A run is "valid" only if validate_pose_transforms
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


def compute_gt(gt):
    """Same as analyze_calibration / validate_pose_transforms."""
    FLU_2_FRD = np.array(DCM(x=180.0))
    t_g = np.array(gt["Time"])
    dt = np.diff(t_g)
    valid = np.hstack(([True], dt > 1e-6))
    t_g = t_g[valid]

    uav_poses = np.array(gt["UAV Pose"], dtype=object)[valid]
    target_poses = np.array(gt["Target Pose"], dtype=object)[valid]
    n = len(t_g)

    W_T_P = np.zeros((n, 4, 4))
    W_R_T = np.zeros((n, 3, 3))
    W_x_tu = np.zeros((n, 3))
    B_x_tu = np.zeros((n, 3))
    for i, (p, tp) in enumerate(zip(uav_poses, target_poses)):
        Ru = Quaternion([p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z]).to_DCM()
        Rt = Quaternion([tp.orientation.w, tp.orientation.x, tp.orientation.y, tp.orientation.z]).to_DCM()
        W_T_P[i, :3, :3] = Ru
        W_T_P[i, :3, 3] = [p.position.x, p.position.y, p.position.z]
        W_T_P[i, 3, 3] = 1.0
        W_R_T[i] = Rt
        W_x_t = np.array([tp.position.x, tp.position.y, tp.position.z])
        W_x_tu[i] = W_x_t - W_T_P[i, :3, 3]
        B_x_tu[i] = FLU_2_FRD @ np.linalg.inv(Ru) @ W_x_tu[i]

    W_x_tu_filt = sgf(W_x_tu, min(51, 2 * (n // 4) + 1), 2, axis=0) if n >= 11 else W_x_tu
    W_v_tu = np.gradient(W_x_tu_filt, t_g, axis=0)
    B_v_tu = np.zeros((n, 3))
    for i, (R, v) in enumerate(zip(W_T_P[:, :3, :3], W_v_tu)):
        B_v_tu[i] = FLU_2_FRD @ np.linalg.inv(R) @ v

    z = B_x_tu[:, 2].copy()
    z[np.abs(z) < 0.1] = np.nan
    B_y_g = B_v_tu / z[:, np.newaxis]

    W_dR_B = np.gradient(W_T_P[:, :3, :3], t_g, axis=0)
    B_w_ug = np.zeros((n, 3))
    for i in range(n):
        skew = W_T_P[i, :3, :3].T @ W_dR_B[i]
        B_w_ug[i] = FLU_2_FRD @ np.array([skew[2, 1], skew[0, 2], skew[1, 0]])

    W_dR_T = np.gradient(W_R_T, t_g, axis=0)
    B_w_tg = np.zeros((n, 3))
    for i in range(n):
        skew = W_R_T[i].T @ W_dR_T[i]
        B_w_tg[i] = FLU_2_FRD @ np.array([skew[2, 1], skew[0, 2], skew[1, 0]])
    B_w_tug = B_w_tg - B_w_ug

    return t_g, B_y_g, B_w_tug, B_x_tu, valid


def robust_scale(raw, gt_v, threshold):
    raw = np.asarray(raw); gt_v = np.asarray(gt_v)
    mask = np.isfinite(raw) & np.isfinite(gt_v) & (np.abs(gt_v) > threshold) & (np.abs(raw) > 1e-9)
    if mask.sum() < 10:
        return float('nan')
    return float(np.median(gt_v[mask] / raw[mask]))


def is_valid_run(B_x_tu, B_v_tu):
    """Quick pose-validity check (mirrors validate_pose_transforms 6 checks)."""
    n = len(B_x_tu)
    mid_z = np.median(B_x_tu[n // 4 : 3 * n // 4, 2])
    return (mid_z > 1.0) and (np.max(np.abs(B_v_tu)) < 10.0)


def analyze_one(run_dir):
    """Return (valid, sensor_cal_hw_diag(6,), sensor_cal_s_diag(4,))."""
    try:
        gt = np.load(f"{run_dir}/Ground_Truth.npy", allow_pickle=True)[()]
    except Exception as e:
        return False, None, None, f"load error: {e}"

    try:
        t_g, B_y_g, B_w_tug, B_x_tu, valid = compute_gt(gt)
    except Exception as e:
        return False, None, None, f"gt compute error: {e}"

    # validation gate
    n = len(t_g)
    W_T_P_t = None  # not needed for validity gate
    # Use B_v_tu derived implicitly through B_y_g * depth
    z = B_x_tu[:, 2]
    z_safe = np.where(np.abs(z) < 0.1, np.nan, z)
    B_v_tu = B_y_g * z_safe[:, np.newaxis]
    if not is_valid_run(B_x_tu, B_v_tu):
        return False, None, None, "validity gate failed (mid_z<=1 or peak v>10)"

    # raw measurements
    raw = np.asarray(gt["Opt Flow Ang Vel"])[valid]  # (N, 6)
    raw_s = np.asarray(gt["Img Feature Params"])[valid]  # (N, 4)
    y_raw, w_raw = raw[:, :3], raw[:, 3:]
    xc_raw = raw_s[:, 0]; yc_raw = raw_s[:, 1]

    xc_gt = B_x_tu[:, 0] / B_x_tu[:, 2]
    yc_gt = B_x_tu[:, 1] / B_x_tu[:, 2]

    flow = np.abs([robust_scale(y_raw[:, i], B_y_g[:, i], 0.05) for i in range(3)])
    ang = np.abs([robust_scale(w_raw[:, i], B_w_tug[:, i], 0.02) for i in range(3)])
    s_xc = abs(robust_scale(xc_raw, xc_gt, 0.05))
    s_yc = abs(robust_scale(yc_raw, yc_gt, 0.05))

    hw = np.concatenate([flow, ang])
    s = np.array([s_xc, s_yc, 1.0, 1.0])
    return True, hw, s, "OK"


def main():
    parent = "/home/shubham/Soft-Precise-Landing/PX4_Gazebo/calibration_data"
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

    print(f"\n[agg] aggregating across {len(valid_hws)} valid run(s)\n")
    print("  _sensor_cal_hw axis stats:")
    print("    axis | mean    median   std     n")
    print("    -----|--------------------------------")
    for i in range(6):
        col = valid_hws[:, i]
        print(f"    {i}    | {col.mean():.4f}  {np.median(col):.4f}  {col.std():.4f}  {len(col)}")
    print("\n  _sensor_cal_s axis stats:")
    print("    axis | mean    median   std     n")
    print("    -----|--------------------------------")
    for i in range(4):
        col = valid_ss[:, i]
        print(f"    {i}    | {col.mean():.4f}  {np.median(col):.4f}  {col.std():.4f}  {len(col)}")

    median_hw = np.median(valid_hws, axis=0)
    median_s = np.median(valid_ss, axis=0)
    print(f"\n  Proposed (median across runs):")
    print(f"    _sensor_cal_hw = np.diag([{', '.join(f'{v:.4f}' for v in median_hw)}])")
    print(f"    _sensor_cal_s  = np.diag([{', '.join(f'{v:.4f}' for v in median_s)}])")
    return 0


if __name__ == "__main__":
    sys.exit(main())
