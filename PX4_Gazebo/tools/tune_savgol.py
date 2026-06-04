#!/usr/bin/env python3
"""
Tune Savitzky-Golay filter parameters (window, polyorder) for two contexts:

  (a) OFFLINE / notebook mode  — apply sgf to the entire recording at once
      (non-causal, uses past+future). Maximises mean|corr| with ground truth.
      Use this for analysis plots; the lag is irrelevant when reading recorded data.

  (b) RUNTIME mode (img_data.py pattern) — sliding window of length W ending
      at current time, take filtered estimate at the middle (~W/2 samples lag).
      Use this to pick parameters that will go into the live PLASMC controller.

Both modes evaluate mean Pearson |corr| against Gazebo ground truth across
all 5 calibration recordings × 8 channels (3 flow + 3 ang-vel + 2 centroid).
"""
import os, glob
import numpy as np
from scipy.signal import savgol_filter as sgf
from ahrs import Quaternion, DCM
from itertools import product

PARENT = "/home/shubham/Soft-Precise-Landing/PX4_Gazebo/calibration_data/output"
FLU_2_FRD = np.array(DCM(x=180.0))

# Currently-applied sensor_cal matrices (5-run median, 2026-05-12)
SC_HW = np.diag([0.1518, 0.1777, 0.0651, 0.2083, 0.2209, 0.2435])
SC_S  = np.diag([0.5814, 0.5809, 1.0,    1.0])

WINDOWS    = [5, 7, 9, 11, 13, 15, 17, 21, 25, 31, 41, 51, 71, 101]
POLYORDERS = [1, 2, 3, 4]


def _body_omega_from_quats(quats, t):
    """Body-frame ω from quaternions via central difference (preserves SO(3))."""
    N = len(quats); w = np.zeros((N, 3))
    for i in range(N):
        i0 = max(0, i - 1); i1 = min(N - 1, i + 1)
        if i1 == i0: continue
        dt_pair = t[i1] - t[i0]
        if dt_pair < 1e-9: continue
        w0, x0, y0, z0 = quats[i0]
        w1, x1, y1, z1 = quats[i1]
        dq_x = w0*x1 - x0*w1 - y0*z1 + z0*y1
        dq_y = w0*y1 + x0*z1 - y0*w1 - z0*x1
        dq_z = w0*z1 - x0*y1 + y0*x1 - z0*w1
        w[i] = 2.0 * np.array([dq_x, dq_y, dq_z]) / dt_pair
    return w


def compute_gt(gt):
    t_g = np.array(gt["Time"]); dt = np.diff(t_g); valid = np.hstack(([True], dt > 1e-6))
    t_g = t_g[valid]; n = len(t_g)
    uav = np.array(gt["UAV Pose"], dtype=object)[valid]
    tgt = np.array(gt["Target Pose"], dtype=object)[valid]
    W_T_P = np.zeros((n, 4, 4)); W_R_T = np.zeros((n, 3, 3))
    W_x_tu = np.zeros((n, 3));   B_x_tu = np.zeros((n, 3))
    for i, (p, tp) in enumerate(zip(uav, tgt)):
        Ru = Quaternion([p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z]).to_DCM()
        Rt = Quaternion([tp.orientation.w, tp.orientation.x, tp.orientation.y, tp.orientation.z]).to_DCM()
        W_T_P[i, :3, :3] = Ru
        W_T_P[i, :3, 3]  = [p.position.x, p.position.y, p.position.z]
        W_R_T[i] = Rt
        W_x_tu[i] = np.array([tp.position.x, tp.position.y, tp.position.z]) - W_T_P[i, :3, 3]
        B_x_tu[i] = FLU_2_FRD @ np.linalg.inv(Ru) @ W_x_tu[i]
    W_x_tu_f = sgf(W_x_tu, 51, 2, axis=0)
    W_v_tu = np.gradient(W_x_tu_f, t_g, axis=0)
    B_v_tu = np.zeros((n, 3))
    for i in range(n):
        B_v_tu[i] = FLU_2_FRD @ np.linalg.inv(W_T_P[i, :3, :3]) @ W_v_tu[i]
    z = B_x_tu[:, 2].copy(); z[np.abs(z) < 0.1] = np.nan
    B_h_g = B_v_tu / z[:, None]
    # Quaternion-difference for body-frame ω (preserves SO(3); element-wise
    # np.gradient on R breaks orthogonality and over-reports ω_z ~2×).
    uav_q = np.array([[u.orientation.w, u.orientation.x,
                       u.orientation.y, u.orientation.z] for u in uav])
    tgt_q = np.array([[t.orientation.w, t.orientation.x,
                       t.orientation.y, t.orientation.z] for t in tgt])
    B_w_ug = (FLU_2_FRD @ _body_omega_from_quats(uav_q, t_g).T).T
    B_w_tg = (FLU_2_FRD @ _body_omega_from_quats(tgt_q, t_g).T).T
    B_w_tug = B_w_tg - B_w_ug
    V_xc_g = B_x_tu[:, 0] / B_x_tu[:, 2]
    V_yc_g = B_x_tu[:, 1] / B_x_tu[:, 2]
    return valid, B_h_g, B_w_tug, V_xc_g, V_yc_g


def safe_corr(a, b):
    m = np.isfinite(a) & np.isfinite(b)
    if m.sum() < 10: return 0.0
    sa, sb = a[m], b[m]
    if np.std(sa) < 1e-9 or np.std(sb) < 1e-9: return 0.0
    return float(np.corrcoef(sa, sb)[0, 1])


def offline_filter(raw, window, polyorder):
    """Mode (a): single sgf call over the whole recording."""
    if len(raw) < window or polyorder >= window: return None
    return sgf(raw, window, polyorder, axis=0)


def runtime_filter(raw, window, polyorder):
    """Mode (b): img_data.py pattern — for each time index i, apply sgf to
    the last `window` samples ending at i, take the middle. The output
    represents the non-causal estimate at time (i - window/2)."""
    if len(raw) < window or polyorder >= window: return None
    N, D = raw.shape
    out = np.zeros_like(raw)
    mid = int(window / 2 + 1)
    for i in range(window - 1, N):
        seg = raw[i - window + 1 : i + 1]   # last `window` samples
        smoothed = sgf(seg, window, polyorder, axis=0)
        out[i] = smoothed[mid] if mid < window else smoothed[-1]
    # First `window-1` samples: use mean of available
    for i in range(window - 1):
        out[i] = np.mean(raw[:i + 1], axis=0)
    return out


def evaluate(run_dirs, window, polyorder, mode):
    """Return (mean_abs_corr, per_channel_dict) for given mode in {'offline', 'runtime'}."""
    chan_names = ['y_0', 'y_1', 'y_2', 'w_0', 'w_1', 'w_2', 'xc', 'yc']
    sums = np.zeros(8); counts = np.zeros(8)
    filt = offline_filter if mode == 'offline' else runtime_filter

    for d in run_dirs:
        gt = np.load(f"{d}/Ground_Truth.npy", allow_pickle=True)[()]
        try: valid, B_h_g, B_w_tug, V_xc_g, V_yc_g = compute_gt(gt)
        except Exception: continue
        raw_hw = np.asarray(gt["Opt Flow Ang Vel"])[valid]
        raw_s  = np.asarray(gt["Img Feature Params"])[valid]
        if window > len(raw_hw): return None, None
        hw_f = filt(raw_hw, window, polyorder)
        s_f  = filt(raw_s,  window, polyorder)
        if hw_f is None or s_f is None: return None, None
        cal_hw = (SC_HW @ hw_f.T).T
        cal_s  = (SC_S  @ s_f.T ).T
        for i in range(3):
            sums[i]   += abs(safe_corr(B_h_g[:, i],   cal_hw[:, i]));     counts[i]   += 1
            sums[3+i] += abs(safe_corr(B_w_tug[:, i], cal_hw[:, 3+i]));   counts[3+i] += 1
        sums[6] += abs(safe_corr(V_xc_g, cal_s[:, 0])); counts[6] += 1
        sums[7] += abs(safe_corr(V_yc_g, cal_s[:, 1])); counts[7] += 1

    means = sums / np.maximum(counts, 1)
    return float(np.mean(means)), dict(zip(chan_names, means))


def baseline_no_filter(run_dirs):
    """Baseline: no filtering at all (raw → sensor_cal)."""
    chan_names = ['y_0', 'y_1', 'y_2', 'w_0', 'w_1', 'w_2', 'xc', 'yc']
    sums = np.zeros(8); counts = np.zeros(8)
    for d in run_dirs:
        gt = np.load(f"{d}/Ground_Truth.npy", allow_pickle=True)[()]
        valid, B_h_g, B_w_tug, V_xc_g, V_yc_g = compute_gt(gt)
        raw_hw = np.asarray(gt["Opt Flow Ang Vel"])[valid]
        raw_s  = np.asarray(gt["Img Feature Params"])[valid]
        cal_hw = (SC_HW @ raw_hw.T).T; cal_s = (SC_S @ raw_s.T).T
        for i in range(3):
            sums[i] += abs(safe_corr(B_h_g[:, i], cal_hw[:, i])); counts[i] += 1
            sums[3+i] += abs(safe_corr(B_w_tug[:, i], cal_hw[:, 3+i])); counts[3+i] += 1
        sums[6] += abs(safe_corr(V_xc_g, cal_s[:, 0])); counts[6] += 1
        sums[7] += abs(safe_corr(V_yc_g, cal_s[:, 1])); counts[7] += 1
    means = sums / np.maximum(counts, 1)
    return float(np.mean(means)), dict(zip(chan_names, means))


def sweep_and_print(run_dirs, mode):
    print(f"\n{'='*72}")
    print(f"  Mode: {mode.upper()}")
    print(f"{'='*72}")
    results = []
    for w, p in product(WINDOWS, POLYORDERS):
        if p >= w: continue
        score, _ = evaluate(run_dirs, w, p, mode)
        if score is None: continue
        results.append((w, p, score))
    results.sort(key=lambda r: -r[2])
    print(f"\n  Top 10 (window, polyorder, mean|corr|):")
    print(f"  {'win':>4} {'poly':>4}  mean|corr|")
    for w, p, s in results[:10]:
        print(f"  {w:>4} {p:>4}    {s:.4f}")
    print(f"  ...")
    for w, p, s in results[-3:]:
        print(f"  {w:>4} {p:>4}    {s:.4f}")
    return results


def main():
    run_dirs = sorted([d for d in glob.glob(f"{PARENT}/*") if os.path.isdir(d)])
    print(f"Tuning across {len(run_dirs)} recordings × 8 channels")

    base_score, base_per = baseline_no_filter(run_dirs)
    print(f"\nNo-filter baseline:    mean|corr| = {base_score:.4f}")
    print(f"  per channel: " + ", ".join(f"{k}={v:.2f}" for k, v in base_per.items()))

    off_results = sweep_and_print(run_dirs, 'offline')
    rt_results  = sweep_and_print(run_dirs, 'runtime')

    print(f"\n{'='*72}")
    print(f"  Side-by-side comparison of headline configurations")
    print(f"{'='*72}")
    print(f"\n  {'config':32s}  mean|corr|  notes")
    print(f"  {'-'*32}  ----------  -----")
    print(f"  no filter                          {base_score:.4f}      raw (current img_data.py before this patch)")
    # MATLAB default
    s, _ = evaluate(run_dirs, 11, 2, 'offline')
    print(f"  MATLAB (11, 2) offline             {s:.4f}      MATLAB Constants.m default, non-causal")
    s, _ = evaluate(run_dirs, 11, 2, 'runtime')
    print(f"  MATLAB (11, 2) runtime             {s:.4f}      same config, sliding window, ~5 sample lag")
    # User's legacy
    s, _ = evaluate(run_dirs, 51, 2, 'offline')
    print(f"  Legacy (51, 2) offline             {s:.4f}      FILTER_WIN=51 from img_data.py legacy")
    s, _ = evaluate(run_dirs, 51, 2, 'runtime')
    print(f"  Legacy (51, 2) runtime             {s:.4f}      same config, sliding window, ~25 sample lag")
    s, _ = evaluate(run_dirs, 51, 3, 'runtime')
    print(f"  Legacy (51, 3) runtime             {s:.4f}      img_data_v7/v8 used polyorder 3")
    # Offline best
    w_off, p_off, s_off = off_results[0]
    print(f"  Offline best ({w_off}, {p_off})           {s_off:.4f}      non-causal, for notebook analysis")
    # Runtime best
    w_rt, p_rt, s_rt = rt_results[0]
    print(f"  Runtime best ({w_rt}, {p_rt})            {s_rt:.4f}      sliding window, ~{w_rt//2} sample lag")

    print(f"\n  -> Runtime recommendation:  FILTER_WIN={w_rt}, FILTER_POLYORDER={p_rt}")
    print(f"     ({w_rt//2} samples lag at ~{w_rt//2 / 30 * 1000:.0f} ms @ 30 Hz, "
          f"or ~{w_rt//2 / 60 * 1000:.0f} ms @ 60 Hz)")

    # Per-channel for runtime best vs current default
    print(f"\n  Per-channel runtime |corr|:")
    print(f"  {'channel':>7}  no_filt  (11,2)   (51,2)   (51,3)   runtime_best({w_rt},{p_rt})")
    chan_names = ['y_0', 'y_1', 'y_2', 'w_0', 'w_1', 'w_2', 'xc', 'yc']
    _, base = baseline_no_filter(run_dirs)
    _, c11 = evaluate(run_dirs, 11, 2, 'runtime')
    _, c51_2 = evaluate(run_dirs, 51, 2, 'runtime')
    _, c51_3 = evaluate(run_dirs, 51, 3, 'runtime')
    _, cbest = evaluate(run_dirs, w_rt, p_rt, 'runtime')
    for n in chan_names:
        print(f"  {n:>7}   {base[n]:.3f}    {c11[n]:.3f}    {c51_2[n]:.3f}    {c51_3[n]:.3f}    {cbest[n]:.3f}")


if __name__ == "__main__":
    main()
