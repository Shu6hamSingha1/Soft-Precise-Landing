#!/usr/bin/env python3
"""
Derive _sensor_cal_hw and _sensor_cal_s by comparing raw image-side
measurements against ground-truth pose-derived quantities, following the
same methodology as plotter_output_calibration.ipynb (cells 11, 16, 18).

Inputs:
  - PX4_Gazebo/calibration_data/output/<timestamp>/{Img_Data,Telemetry_Data,Ground_Truth}.npy
    (defaults to the most recent timestamped folder; pass a dir as argv[1] to pick one)

Outputs:
  - Proposed _sensor_cal_hw (6x6 diag) and _sensor_cal_s (4x4 diag)
  - Diagnostic stats per axis (raw vs gt magnitudes, ratio histograms)
"""
import os
import sys
import numpy as np
from scipy.signal import savgol_filter as sgf
from ahrs import Quaternion, DCM


def load_data(data_dir):
    return {
        "Img_Data": np.load(f"{data_dir}/Img_Data.npy", allow_pickle=True)[()],
        "Telemetry_Data": np.load(f"{data_dir}/Telemetry_Data.npy", allow_pickle=True)[()],
        "Ground_Truth": np.load(f"{data_dir}/Ground_Truth.npy", allow_pickle=True)[()],
    }


def compute_ground_truth_flow_and_w(gt):
    """Port of plotter_output_calibration.ipynb cells 16 + 18.

    Returns (t_g, B_y_g, B_w_tug, B_x_tu) all of shape (N, 3) [or (N,) for t_g].
    """
    FLU_2_FRD = np.array(DCM(x=180.0))

    t_g = np.array(gt["Time"])
    # Drop duplicate timesteps
    dt = np.diff(t_g)
    valid = np.hstack(([True], dt > 1e-6))
    t_g = t_g[valid]

    uav_poses = np.array(gt["UAV Pose"], dtype=object)[valid]
    target_poses = np.array(gt["Target Pose"], dtype=object)[valid]
    n = len(t_g)

    # UAV pose 4x4 in world frame
    W_T_P = np.zeros((n, 4, 4))
    for i, p in enumerate(uav_poses):
        W_T_P[i, :3, :3] = Quaternion(
            [p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z]
        ).to_DCM()
        W_T_P[i, :3, 3] = [p.position.x, p.position.y, p.position.z]
        W_T_P[i, 3, 3] = 1.0

    # Target pose & relative position
    W_R_T = np.zeros((n, 3, 3))
    W_x_tu = np.zeros((n, 3))
    B_x_tu = np.zeros((n, 3))
    for i, (p, R_uav) in enumerate(zip(target_poses, W_T_P[:, :3, :3])):
        W_R_T[i] = Quaternion(
            [p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z]
        ).to_DCM()
        W_x_t = np.array([p.position.x, p.position.y, p.position.z])
        W_x_tu[i] = W_x_t - W_T_P[i, :3, 3]
        B_x_tu[i] = FLU_2_FRD @ np.linalg.inv(R_uav) @ W_x_tu[i]

    # Body-frame target velocity wrt UAV
    W_x_tu_filt = sgf(W_x_tu, min(51, 2 * (n // 4) + 1), 2, axis=0) if n >= 11 else W_x_tu
    W_v_tu = np.gradient(W_x_tu_filt, t_g, axis=0)
    B_v_tu = np.zeros((n, 3))
    for i, (R, v) in enumerate(zip(W_T_P[:, :3, :3], W_v_tu)):
        B_v_tu[i] = FLU_2_FRD @ np.linalg.inv(R) @ v

    # Ground-truth optical flow (rad/s): velocity / depth, with z = B_x_tu[:,2]
    # Guard against near-zero depth
    z = B_x_tu[:, 2].copy()
    z[np.abs(z) < 0.1] = np.nan  # invalid
    B_y_g = B_v_tu / z[:, np.newaxis]

    # Ground-truth angular velocity (body frame): UAV
    W_R_B = W_T_P[:, :3, :3]
    W_dR_B = np.gradient(W_R_B, t_g, axis=0)
    B_w_ug = np.zeros((n, 3))
    for i, (R, dR) in enumerate(zip(W_R_B, W_dR_B)):
        skew = R.T @ dR
        B_w_ug[i] = FLU_2_FRD @ np.array([skew[2, 1], skew[0, 2], skew[1, 0]])

    # Ground-truth angular velocity (body frame): target
    W_dR_T = np.gradient(W_R_T, t_g, axis=0)
    B_w_tg = np.zeros((n, 3))
    for i, (R, dR) in enumerate(zip(W_R_T, W_dR_T)):
        skew = R.T @ dR
        B_w_tg[i] = FLU_2_FRD @ np.array([skew[2, 1], skew[0, 2], skew[1, 0]])
    B_w_tug = B_w_tg - B_w_ug

    return t_g, B_y_g, B_w_tug, B_x_tu, valid


def robust_scale(raw, gt, magnitude_threshold):
    """Per-axis median ratio gt / raw, computed only on samples where |gt| exceeds
    the threshold (so divisions are well-conditioned and not dominated by noise)."""
    raw = np.asarray(raw)
    gt = np.asarray(gt)
    mask = np.isfinite(raw) & np.isfinite(gt) & (np.abs(gt) > magnitude_threshold) & (np.abs(raw) > 1e-9)
    if mask.sum() < 10:
        return float('nan'), int(mask.sum())
    ratios = gt[mask] / raw[mask]
    # Use median; both signs should agree if axis convention is the same
    return float(np.median(ratios)), int(mask.sum())


def _most_recent_run_dir():
    parent = "/home/shubham/Soft-Precise-Landing/PX4_Gazebo/calibration_data/output"
    cands = [d for d in os.listdir(parent) if os.path.isdir(os.path.join(parent, d))]
    if not cands:
        return None
    return os.path.join(parent, max(cands, key=lambda d: os.path.getmtime(os.path.join(parent, d))))


def main():
    data_dir = sys.argv[1] if len(sys.argv) > 1 else _most_recent_run_dir()
    if not data_dir or not os.path.isdir(data_dir):
        print(f"[err] data dir not found: {data_dir}")
        return 1
    print(f"[analyze] using run dir: {data_dir}")

    print(f"[analyze] loading from {data_dir}")
    d = load_data(data_dir)
    gt = d["Ground_Truth"]

    t_g, B_y_g, B_w_tug, B_x_tu, valid = compute_ground_truth_flow_and_w(gt)
    n = len(t_g)

    # Raw image-side measurements (logged via getRawOptFlowAngVel — pre sensor_cal)
    raw = np.asarray(gt["Opt Flow Ang Vel"])[valid]   # (N, 6)
    y_raw, w_raw = raw[:, :3], raw[:, 3:]

    print(f"[analyze] {n} valid samples over {t_g[-1] - t_g[0]:.1f}s\n")

    # Print magnitude stats
    print("        Optical flow [0,1,2]                  Angular velocity [3,4,5]")
    print("        raw RMS  /  gt RMS                    raw RMS  /  gt RMS")
    for i in range(3):
        y_rms_r = np.sqrt(np.nanmean(y_raw[:, i] ** 2))
        y_rms_g = np.sqrt(np.nanmean(B_y_g[:, i] ** 2))
        w_rms_r = np.sqrt(np.nanmean(w_raw[:, i] ** 2))
        w_rms_g = np.sqrt(np.nanmean(B_w_tug[:, i] ** 2))
        print(f"  axis {i}: {y_rms_r:8.4f} / {y_rms_g:8.4f}    "
              f"            {w_rms_r:8.4f} / {w_rms_g:8.4f}")

    # Derive per-axis scaling factors
    print("\n[analyze] computing median(gt/raw) per axis ...")
    flow_thresh = 0.05   # rad/s — ignore samples below this for OF
    ang_thresh = 0.02    # rad/s — ignore samples below this for ω

    flow_scale = np.zeros(3)
    flow_n = np.zeros(3, dtype=int)
    for i in range(3):
        flow_scale[i], flow_n[i] = robust_scale(y_raw[:, i], B_y_g[:, i], flow_thresh)
    ang_scale = np.zeros(3)
    ang_n = np.zeros(3, dtype=int)
    for i in range(3):
        ang_scale[i], ang_n[i] = robust_scale(w_raw[:, i], B_w_tug[:, i], ang_thresh)

    print("\nPer-axis scaling factors (gt / raw):")
    print(f"  Optical flow:    [{flow_scale[0]:+.4f}, {flow_scale[1]:+.4f}, {flow_scale[2]:+.4f}]  "
          f"(n samples = {flow_n.tolist()})")
    print(f"  Angular vel:     [{ang_scale[0]:+.4f}, {ang_scale[1]:+.4f}, {ang_scale[2]:+.4f}]  "
          f"(n samples = {ang_n.tolist()})")

    # Build the matrix. Take absolute values (sign mismatches are a frame issue,
    # not a scaling issue — flag separately).
    scale_abs = np.concatenate([np.abs(flow_scale), np.abs(ang_scale)])
    new_hw = np.diag(scale_abs)

    print(f"\nProposed _sensor_cal_hw (6x6 diag):")
    print(f"  np.diag([{', '.join(f'{v:.4f}' for v in scale_abs)}])")

    print(f"\nCurrent in img_data.py:")
    print(f"  np.diag([1, 1, 1, 1/3, 1/3, 1])  ->  diag([1, 1, 1, 0.3333, 0.3333, 1])")

    # Sign warnings
    if np.any(np.sign(flow_scale) < 0):
        print("\n  WARN: optical-flow axis sign(s) inverted vs ground truth — check FLU/FRD/NED convention.")
    if np.any(np.sign(ang_scale) < 0):
        print("\n  WARN: angular-velocity axis sign(s) inverted vs ground truth — check FLU/FRD/NED convention.")

    # ---- _sensor_cal_s (image features) ----
    print("\n--- Image feature calibration (_sensor_cal_s, 4x4 diag) ---")
    raw_s = np.asarray(gt["Img Feature Params"])[valid]   # (N, 4)
    xc_raw, yc_raw, _, alpha_raw = raw_s[:, 0], raw_s[:, 1], raw_s[:, 2], raw_s[:, 3]

    # GT centroid: target position / depth (in camera body coords)
    xc_gt = B_x_tu[:, 0] / B_x_tu[:, 2]
    yc_gt = B_x_tu[:, 1] / B_x_tu[:, 2]

    print(f"  xc raw RMS: {np.sqrt(np.nanmean(xc_raw ** 2)):.4f}   "
          f"xc gt RMS: {np.sqrt(np.nanmean(xc_gt ** 2)):.4f}")
    print(f"  yc raw RMS: {np.sqrt(np.nanmean(yc_raw ** 2)):.4f}   "
          f"yc gt RMS: {np.sqrt(np.nanmean(yc_gt ** 2)):.4f}")

    s_thresh = 0.05
    xc_scale, xc_n = robust_scale(xc_raw, xc_gt, s_thresh)
    yc_scale, yc_n = robust_scale(yc_raw, yc_gt, s_thresh)

    # 3rd component (scale) is hardcoded 1.0 in img_data.py; scaling has no effect, leave at 1.
    # 4th component (alpha) calibration would need target yaw computation; flag for follow-up.
    print(f"  scaling: xc {xc_scale:+.4f} (n={xc_n})   yc {yc_scale:+.4f} (n={yc_n})")
    print(f"  alpha (idx 3) not auto-calibrated — needs target-yaw ground truth port; "
          f"keep current value 1.0")

    s_scale = np.abs(np.array([xc_scale, yc_scale, 1.0, 1.0]))
    print(f"\nProposed _sensor_cal_s (4x4 diag):")
    print(f"  np.diag([{s_scale[0]:.4f}, {s_scale[1]:.4f}, 1.0, 1.0])")
    print(f"Current in img_data.py: np.diag([1/6, 1/6, 1, 1])  -> diag([0.1667, 0.1667, 1, 1])")

    print("\n[analyze] done.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
