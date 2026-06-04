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
import sys as _sys, os as _os
_sys.path.insert(0, _os.path.dirname(_os.path.abspath(__file__)))
from aggregate_calibration_phased import compute_gt_signals  # single V-frame GT source


def load_data(data_dir):
    return {
        "Img_Data": np.load(f"{data_dir}/Img_Data.npy", allow_pickle=True)[()],
        "Telemetry_Data": np.load(f"{data_dir}/Telemetry_Data.npy", allow_pickle=True)[()],
        "Ground_Truth": np.load(f"{data_dir}/Ground_Truth.npy", allow_pickle=True)[()],
    }


def _body_omega_from_quats(quats, t):
    """Body-frame angular velocity from quaternions via central difference
    (δq = conj(q[i-1])·q[i+1]; ω_body ≈ 2·δq.xyz / dt). Preserves SO(3),
    unlike element-wise np.gradient on the rotation matrix."""
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
        dq_x = w0*x1 - x0*w1 - y0*z1 + z0*y1
        dq_y = w0*y1 + x0*z1 - y0*w1 - z0*x1
        dq_z = w0*z1 - x0*y1 + y0*x1 - z0*w1
        w[i] = 2.0 * np.array([dq_x, dq_y, dq_z]) / dt_pair
    return w


def compute_ground_truth_flow_and_w(gt):
    """V-frame GT via the canonical single source
    (aggregate_calibration_phased.compute_gt_signals): h, w AND s all gravity-
    leveled to the frame img_data.py reports. Returns
    (t_g, V_h_g, V_w_tug, V_xc_g, V_yc_g, valid)."""
    t_g, V_h_g, V_w_ug, V_xc_g, V_yc_g, valid, _z = compute_gt_signals(gt)
    return t_g, V_h_g, -V_w_ug, V_xc_g, V_yc_g, valid   # manuscript w = -V_w_ug


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

    t_g, V_h_g, V_w_tug, V_xc_g, V_yc_g, valid = compute_ground_truth_flow_and_w(gt)
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
        y_rms_g = np.sqrt(np.nanmean(V_h_g[:, i] ** 2))
        w_rms_r = np.sqrt(np.nanmean(w_raw[:, i] ** 2))
        w_rms_g = np.sqrt(np.nanmean(V_w_tug[:, i] ** 2))
        print(f"  axis {i}: {y_rms_r:8.4f} / {y_rms_g:8.4f}    "
              f"            {w_rms_r:8.4f} / {w_rms_g:8.4f}")

    # Derive per-axis scaling factors
    print("\n[analyze] computing median(gt/raw) per axis ...")
    flow_thresh = 0.05   # rad/s — ignore samples below this for OF
    ang_thresh = 0.02    # rad/s — ignore samples below this for ω

    flow_scale = np.zeros(3)
    flow_n = np.zeros(3, dtype=int)
    for i in range(3):
        flow_scale[i], flow_n[i] = robust_scale(y_raw[:, i], V_h_g[:, i], flow_thresh)
    ang_scale = np.zeros(3)
    ang_n = np.zeros(3, dtype=int)
    for i in range(3):
        ang_scale[i], ang_n[i] = robust_scale(w_raw[:, i], V_w_tug[:, i], ang_thresh)

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

    # GT centroid s = (V_xc_g, V_yc_g) — V-frame, from compute_ground_truth_flow_and_w.

    print(f"  xc raw RMS: {np.sqrt(np.nanmean(xc_raw ** 2)):.4f}   "
          f"xc gt RMS: {np.sqrt(np.nanmean(V_xc_g ** 2)):.4f}")
    print(f"  yc raw RMS: {np.sqrt(np.nanmean(yc_raw ** 2)):.4f}   "
          f"yc gt RMS: {np.sqrt(np.nanmean(V_yc_g ** 2)):.4f}")

    s_thresh = 0.05
    xc_scale, xc_n = robust_scale(xc_raw, V_xc_g, s_thresh)
    yc_scale, yc_n = robust_scale(yc_raw, V_yc_g, s_thresh)

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
