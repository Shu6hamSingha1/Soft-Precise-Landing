"""Aggregator for record_input_calibration.py recordings on the Pi hardware
pipeline. Ported directly from the Gazebo SITL
tools/aggregate_input_calibration.py - the saved data format is identical
(record_input_calibration.py was modeled on the SITL script and saves the
same Telemetry_Data/Ground_Truth keys), so this needed only the CAL_DIR
default changed to match the Pi's Test_Data/Calibration/Input convention.

Per-axis metrics computed here (after cmd<->tel interpolation):
  - Pearson r(cmd, tel)           how much of tel omega response is explained by cmd
  - cross-corr peak lag (ms)      FC rate-loop deadtime + EKF/MAVSDK transit
  - linear gain (tel/cmd)         steady-state response amplitude ratio

Across-run aggregation: MAD-trimmed mean.

Usage: python3 analyze_input_calibration.py [cal_dir]
"""
import os
import sys

import numpy as np
from scipy.interpolate import interp1d
from scipy.signal import correlate

CAL_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "Test_Data", "Calibration", "Input")
AXES = ["wx", "wy", "wz"]        # cmd[:, 0..2] vs tel omega FRD[:, 0..2]


def per_run_metrics(run_dir, return_series=False):
    tel = np.load(f"{run_dir}/Telemetry_Data.npy", allow_pickle=True).item()
    gt = np.load(f"{run_dir}/Ground_Truth.npy", allow_pickle=True).item()

    t_cmd = np.array(gt["Time"])
    cmd = np.array(gt["Command"])
    if cmd.ndim != 2 or cmd.shape[1] < 3:
        return None
    cmd = cmd[:, :3]  # wx, wy, wz only

    nm = min(len(t_cmd), len(cmd))
    if nm < 10:
        return None
    t_cmd, cmd = t_cmd[-nm:], cmd[-nm:]
    imu_ts = np.array(tel["IMU Timestamp"]) - gt["Start Time"]
    w_tel = np.array([[a.forward_rad_s, a.right_rad_s, a.down_rad_s]
                       for a in tel["Angular Velocity FRD"]])

    mask = (imu_ts >= t_cmd[0]) & (imu_ts <= t_cmd[-1])
    if mask.sum() < 100:
        return None
    imu_ts = imu_ts[mask]
    w_tel = w_tel[mask]

    fs = 200.0  # Hz
    t_uniform = np.arange(t_cmd[0], t_cmd[-1], 1.0 / fs)
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
        c = cmd_u[:, k]
        t = tel_u[:, k]
        c_c = c - c.mean()
        t_c = t - t.mean()

        denom = (np.std(c_c) * np.std(t_c))
        r = float(np.mean(c_c * t_c) / denom) if denom > 1e-9 else float("nan")

        xc = correlate(t_c, c_c, mode="full")
        lags = (np.arange(len(xc)) - (len(c_c) - 1)) / fs
        win = (lags >= 0) & (lags <= 0.5)
        if win.sum() > 0:
            lag_s = float(lags[win][np.argmax(xc[win])])
        else:
            lag_s = float("nan")

        shift = int(round(lag_s * fs))
        if 0 <= shift < len(c) - 10:
            c_sh = c_c[:len(c) - shift]
            t_sh = t_c[shift:]
            denom2 = float((c_sh * c_sh).sum())
            gain = float((c_sh * t_sh).sum() / denom2) if denom2 > 1e-9 else float("nan")
        else:
            gain = float("nan")

        out[ax] = dict(r=r, lag_ms=lag_s * 1000, gain=gain)

    if return_series:
        return out, dict(t=t_uniform, cmd=cmd_u, tel=tel_u)
    return out


def mad_trimmed_mean(values, k_mad=2.5):
    arr = np.array([v for v in values if np.isfinite(v)])
    if len(arr) < 3:
        return float(arr.mean()) if len(arr) else float("nan"), len(arr), 0
    med = float(np.median(arr))
    mad = float(np.median(np.abs(arr - med)))
    if mad < 1e-9:
        return float(arr.mean()), len(arr), 0
    kept = arr[np.abs(arr - med) <= k_mad * mad]
    return float(kept.mean()), int(len(kept)), int(len(arr) - len(kept))


def main():
    cal_dir = sys.argv[1] if len(sys.argv) > 1 else CAL_DIR
    if not os.path.isdir(cal_dir):
        print(f"No such directory: {cal_dir}")
        sys.exit(1)
    runs = sorted(d for d in os.listdir(cal_dir)
                  if os.path.isdir(os.path.join(cal_dir, d)))
    if not runs:
        print(f"No recordings under {cal_dir}")
        sys.exit(1)

    print(f"[input-cal] aggregating {len(runs)} runs under {cal_dir}\n")
    print("  Per-run metrics:")
    print(f'    {"run":32s}  {"axis":4s}  {"r":>6s}  {"lag_ms":>7s}  {"gain":>7s}')
    all_metrics = {ax: dict(r=[], lag_ms=[], gain=[]) for ax in AXES}
    for d in runs:
        m = per_run_metrics(os.path.join(cal_dir, d))
        if m is None:
            print(f"    {d:30s}  SKIPPED (insufficient samples)")
            continue
        for ax in AXES:
            print(f'    {d:32s}  {ax:4s}  {m[ax]["r"]:6.3f}  {m[ax]["lag_ms"]:7.1f}  {m[ax]["gain"]:7.3f}')
            for key in ("r", "lag_ms", "gain"):
                all_metrics[ax][key].append(m[ax][key])

    print("\n  Across-run aggregation (MAD-trimmed mean, k_MAD=2.5):")
    print(f'    {"axis":4s}  {"r":>10s}  {"lag_ms":>11s}  {"gain":>10s}')
    for ax in AXES:
        r_tm, r_n, r_d = mad_trimmed_mean(all_metrics[ax]["r"])
        l_tm, l_n, l_d = mad_trimmed_mean(all_metrics[ax]["lag_ms"])
        g_tm, g_n, g_d = mad_trimmed_mean(all_metrics[ax]["gain"])
        print(f"    {ax:4s}  {r_tm:6.3f} (n={r_n})  {l_tm:7.1f} (n={l_n})  {g_tm:7.3f} (n={g_n})")

    print("\n  Interpretation:")
    print("    r close to 1   -> FC closely tracks cmd (good)")
    print("    r much < 1     -> FC response dominated by disturbance rejection / noise")
    print("    lag_ms         -> rate-loop deadtime (MAVSDK transit + PX4 inner loop)")
    print("    gain ~= 1.0    -> FC matches cmd amplitude; << 1 = underdamped tracking")
    print("\n  NOTE: this characterizes RATE tracking only. The thrust slope (for")
    print("  HW_THRUST_SLOPE) is not derived here - use find_hover_throttle.py for the")
    print("  hover point, and compare commanded vs achieved climb rate across runs for slope.")


if __name__ == "__main__":
    main()
