"""Aggregator for input_calibration.py recordings.

Input-cal differs from output-cal in what it measures:
- Output cal validates the IMG-to-image-feature pipeline (sensor_cal_hw/s).
- Input cal validates the FC's tracking of commanded body-rate + thrust.

Per-axis metrics computed here (after cmd→tel interpolation):
  - Pearson r(cmd, tel)           how much of tel ω response is explained by cmd
  - cross-corr peak lag (ms)      FC rate-loop deadtime + EKF/MAVSDK transit
  - linear gain (tel/cmd)         steady-state response amplitude ratio

Across-run aggregation: MAD-trimmed mean (matches aggregate_calibration_phased).
"""
import os
import sys

import numpy as np
from scipy.interpolate import interp1d
from scipy.signal import correlate


CAL_DIR = '/home/shubham/Soft-Precise-Landing/PX4_Gazebo/calibration_data/input'
AXES = ['wx', 'wy', 'wz']        # cmd[:, 0..2] vs tel ω FRD[:, 0..2]


def per_run_metrics(run_dir):
    tel = np.load(f'{run_dir}/Telemetry_Data.npy', allow_pickle=True).item()
    gt  = np.load(f'{run_dir}/Ground_Truth.npy',   allow_pickle=True).item()

    t_cmd = np.array(gt['Time'])                                  # cmd timeline
    cmd   = np.array(gt['Command'])[:, :3]                        # ωx, ωy, ωz only
    imu_ts = np.array(tel['IMU Timestamp']) - gt['Start Time']    # align to cmd t=0
    w_tel  = np.array([[a.forward_rad_s, a.right_rad_s, a.down_rad_s]
                       for a in tel['Angular Velocity FRD']])

    # Clip tel to cmd window
    mask = (imu_ts >= t_cmd[0]) & (imu_ts <= t_cmd[-1])
    if mask.sum() < 100:
        return None
    imu_ts = imu_ts[mask]; w_tel = w_tel[mask]

    # Resample tel onto a uniform fine timeline within cmd window so xcorr lag
    # is well-defined; also resample cmd onto that same timeline.
    fs   = 200.0                                                  # Hz; finer than cmd's ~80 Hz
    t_uniform = np.arange(t_cmd[0], t_cmd[-1], 1.0/fs)
    cmd_u = np.column_stack([
        interp1d(t_cmd, cmd[:, k], bounds_error=False, fill_value=0.0)(t_uniform)
        for k in range(3)
    ])
    tel_u = np.column_stack([
        interp1d(imu_ts, w_tel[:, k], bounds_error=False, fill_value=0.0)(t_uniform)
        for k in range(3)
    ])

    out = {}
    for k, ax in enumerate(AXES):
        c = cmd_u[:, k]; t = tel_u[:, k]
        # Zero-mean for correlation / lag stability
        c_c = c - c.mean(); t_c = t - t.mean()

        # Pearson r
        denom = (np.std(c_c) * np.std(t_c))
        r = float(np.mean(c_c * t_c) / denom) if denom > 1e-9 else float('nan')

        # Cross-correlation peak lag
        xc = correlate(t_c, c_c, mode='full')
        lags = (np.arange(len(xc)) - (len(c_c) - 1)) / fs        # seconds
        # Search only physical lag window: tel lags cmd by 0..200 ms
        win = (lags >= 0) & (lags <= 0.5)
        if win.sum() > 0:
            lag_s = float(lags[win][np.argmax(xc[win])])
        else:
            lag_s = float('nan')

        # Linear gain at peak lag: tel ≈ gain * shift(cmd, lag)
        shift = int(round(lag_s * fs))
        if 0 <= shift < len(c) - 10:
            c_sh = c_c[:len(c)-shift]; t_sh = t_c[shift:]
            denom2 = float((c_sh * c_sh).sum())
            gain = float((c_sh * t_sh).sum() / denom2) if denom2 > 1e-9 else float('nan')
        else:
            gain = float('nan')

        out[ax] = dict(r=r, lag_ms=lag_s*1000, gain=gain)

    return out


def mad_trimmed_mean(values, k_mad=2.5):
    arr = np.array([v for v in values if np.isfinite(v)])
    if len(arr) < 3:
        return float(arr.mean()) if len(arr) else float('nan'), len(arr), 0
    med = float(np.median(arr))
    mad = float(np.median(np.abs(arr - med)))
    if mad < 1e-9:
        return float(arr.mean()), len(arr), 0
    kept = arr[np.abs(arr - med) <= k_mad * mad]
    return float(kept.mean()), int(len(kept)), int(len(arr) - len(kept))


def main():
    cal_dir = sys.argv[1] if len(sys.argv) > 1 else CAL_DIR
    runs = sorted([d for d in os.listdir(cal_dir)
                   if os.path.isdir(os.path.join(cal_dir, d))])
    if not runs:
        print(f'No recordings under {cal_dir}'); sys.exit(1)

    print(f'[input-cal] aggregating {len(runs)} runs under {cal_dir}\n')
    print(f'  Per-run metrics:')
    print(f'    {"run":32s}  {"axis":4s}  {"r":>6s}  {"lag_ms":>7s}  {"gain":>7s}')
    all_metrics = {ax: dict(r=[], lag_ms=[], gain=[]) for ax in AXES}
    for d in runs:
        m = per_run_metrics(os.path.join(cal_dir, d))
        if m is None:
            print(f'    {d:30s}  SKIPPED (insufficient samples)'); continue
        for ax in AXES:
            print(f'    {d:32s}  {ax:4s}  {m[ax]["r"]:6.3f}  {m[ax]["lag_ms"]:7.1f}  {m[ax]["gain"]:7.3f}')
            for key in ('r', 'lag_ms', 'gain'):
                all_metrics[ax][key].append(m[ax][key])

    print(f'\n  Across-run aggregation (MAD-trimmed mean, k_MAD=2.5):')
    print(f'    {"axis":4s}  {"r":>10s}  {"lag_ms":>11s}  {"gain":>10s}')
    for ax in AXES:
        r_tm, r_n, r_d   = mad_trimmed_mean(all_metrics[ax]['r'])
        l_tm, l_n, l_d   = mad_trimmed_mean(all_metrics[ax]['lag_ms'])
        g_tm, g_n, g_d   = mad_trimmed_mean(all_metrics[ax]['gain'])
        print(f'    {ax:4s}  {r_tm:6.3f} (n={r_n})  {l_tm:7.1f} (n={l_n})  {g_tm:7.3f} (n={g_n})')

    print(f'\n  Interpretation:')
    print(f'    r close to 1   → FC closely tracks cmd (good)')
    print(f'    r much < 1     → FC response dominated by disturbance rejection / noise')
    print(f'    lag_ms         → rate-loop deadtime (MAVSDK transit + PX4 inner loop)')
    print(f'                     baseline ~30-40 ms per feedback_impulse_response')
    print(f'    gain ≈ 1.0     → FC matches cmd amplitude; ≪ 1 = underdamped tracking')


if __name__ == '__main__':
    main()
