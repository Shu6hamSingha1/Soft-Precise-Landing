"""Aggregator for record_input_calibration.py recordings on the Pi hardware
pipeline. Ported directly from the Gazebo SITL
tools/aggregate_input_calibration.py - the saved data format is identical
(record_input_calibration.py was modeled on the SITL script and saves the
same Telemetry_Data/Ground_Truth keys), so this needed only the CAL_DIR
default changed to match the Pi's Test_Data/Calibration/Input convention.

Per-axis metrics computed here (after cmd<->tel interpolation, cross-corr
lag-aligned before computing gain):
  - Pearson r(cmd, tel)           how much of tel response is explained by cmd
  - cross-corr peak lag (ms)      FC/actuator deadtime + EKF/MAVSDK transit
  - linear gain (tel/cmd)         steady-state response amplitude ratio

Across-run aggregation: MAD-trimmed mean.

2026-07-20: added a "thrust" pseudo-axis, ported from the Gazebo derivation
in Memory/feedback_input_cal_thrust_units.md. Command[:, 3] (B_T, Newtons of
thrust deficit -- positive B_T = LESS thrust, no leading minus sign per the
Gazebo memo's empirically-confirmed convention) is correlated/lag-aligned
against accelerometer a_down (down_m_s2) using the SAME cross-correlation
machinery already used for wx/wy/wz, instead of a naive same-instant linear
fit. The resulting "gain" IS 1/mass directly (a_down = B_T/mass at gain=1),
from which HW_THRUST_SLOPE and a mass cross-check both fall out. This
addresses two prior failure modes found during manual derivation:
  1. Wrong sign convention (assumed a_down = -B_T/mass; actually +B_T/mass -
     positive B_T means less thrust, so the drone falls, so a_down increases).
  2. Naive same-instant regression on unsettled/transient steps (STEP_HOLD_S
     was 0.5s in early runs, too short for the response to reach steady
     state) gave run-to-run inconsistent slopes (2.3kg vs 12.3kg derived
     mass on two similar runs). The cross-correlation lag search here finds
     the true response delay before computing gain, same as the rate axes,
     rather than assuming zero lag.

Usage: python3 analyze_input_calibration.py [cal_dir]
"""
import os
import sys

import numpy as np
from scipy.interpolate import interp1d
from scipy.signal import correlate, butter, filtfilt

CAL_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "Test_Data", "Calibration", "Input")
AXES = ["wx", "wy", "wz", "thrust"]   # wx/wy/wz: cmd[:,0..2] vs tel omega FRD
                                       # thrust: cmd[:,3] (B_T, N) vs a_down (m/s^2)
DRONE_MASS_KG = 1.204   # corrected 2026-07-21 (supersedes the 1.230 kg parts-sum
                        # estimate). Two independent thrust-axis gain regressions
                        # (this script's 2026-07-20 n=16 run, and this session's
                        # separate Input_Clean cross-corr regression) both derive a
                        # mass ~1.34-1.39 kg -- 130-190g off the known 1.204 kg,
                        # consistently in the same direction across two datasets.
                        # See project_hardware_drone_mass memory. Use the known mass
                        # for the slope correction, not either derived value.


LOWPASS_CUTOFF_HZ = 8.0   # 2026-07-22: genuine sensor/vibration noise floor filter.
                          # PSD of a_down on a representative run (Mon Jul 20
                          # 18-42-02) shows a clean bimodal split: ~51% of power
                          # below 2Hz (real maneuver/thrust-response signal),
                          # a quiet gap 2-20Hz (~5%), then ~44% of total power
                          # above 20Hz (propeller/motor vibration). 8Hz sits in
                          # the middle of that quiet gap -- removes the
                          # vibration floor without touching the real signal
                          # band. Applied to ALL telemetry signals (gyro for
                          # wx/wy/wz, accelerometer for thrust, velocity for
                          # the drag-correction term) before any correlation/
                          # regression, i.e. for every input-cal parameter.


def _lowpass(sig, fs, cutoff_hz=LOWPASS_CUTOFF_HZ, order=4):
    """Zero-phase (filtfilt) Butterworth low-pass -- removes the vibration
    noise floor without introducing lag, which would otherwise bias the
    cross-correlation lag/gain estimates computed downstream."""
    nyq = fs / 2.0
    b, a = butter(order, cutoff_hz / nyq, btype="low")
    return filtfilt(b, a, sig)


def _corr_metrics(c, t, fs, lag_win_s=0.5):
    """Shared cross-correlation r/lag/gain computation for one axis's
    (cmd, tel) pair, already resampled onto a common uniform timeline."""
    c_c = c - c.mean()
    t_c = t - t.mean()

    denom = (np.std(c_c) * np.std(t_c))
    r = float(np.mean(c_c * t_c) / denom) if denom > 1e-9 else float("nan")

    xc = correlate(t_c, c_c, mode="full")
    lags = (np.arange(len(xc)) - (len(c_c) - 1)) / fs
    win = (lags >= 0) & (lags <= lag_win_s)
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

    return dict(r=r, lag_ms=lag_s * 1000, gain=gain)


def per_run_metrics(run_dir, return_series=False):
    tel_path = f"{run_dir}/Telemetry_Data.npy"
    gt_path = f"{run_dir}/Ground_Truth.npy"
    if not (os.path.isfile(tel_path) and os.path.isfile(gt_path)):
        return None
    tel = np.load(tel_path, allow_pickle=True).item()
    gt = np.load(gt_path, allow_pickle=True).item()

    t_cmd = np.array(gt["Time"])
    cmd_full = np.array(gt["Command"])
    if cmd_full.ndim != 2 or cmd_full.shape[1] < 4:
        return None
    cmd = cmd_full[:, :3]      # wx, wy, wz
    B_T = cmd_full[:, 3]       # thrust command, Newtons (deficit-from-hover)

    nm = min(len(t_cmd), len(cmd))
    if nm < 10:
        return None
    t_cmd, cmd, B_T = t_cmd[-nm:], cmd[-nm:], B_T[-nm:]
    imu_ts = np.array(tel["IMU Timestamp"]) - gt["Start Time"]
    w_tel = np.array([[a.forward_rad_s, a.right_rad_s, a.down_rad_s]
                       for a in tel["Angular Velocity FRD"]])
    a_down = np.array([a.down_m_s2 for a in tel["Acceleration"]])

    mask = (imu_ts >= t_cmd[0]) & (imu_ts <= t_cmd[-1])
    if mask.sum() < 100:
        return None
    imu_ts = imu_ts[mask]
    w_tel = w_tel[mask]
    a_down = a_down[mask]

    fs = 200.0  # Hz
    t_uniform = np.arange(t_cmd[0], t_cmd[-1], 1.0 / fs)
    if len(t_uniform) < 20:   # filtfilt needs a minimum sample count (padlen)
        return None
    cmd_u = np.column_stack([
        interp1d(t_cmd, cmd[:, k], bounds_error=False, fill_value=0.0)(t_uniform)
        for k in range(3)
    ])
    tel_u_raw = np.column_stack([
        interp1d(imu_ts, w_tel[:, k], bounds_error=False, fill_value=0.0)(t_uniform)
        for k in range(3)
    ])
    B_T_u = interp1d(t_cmd, B_T, bounds_error=False, fill_value=0.0)(t_uniform)
    a_down_u_raw = interp1d(imu_ts, a_down, bounds_error=False, fill_value=float(a_down.mean()))(t_uniform)

    # Low-pass BOTH sides of each cmd<->tel pair (uniform grid, same fs) --
    # removes the genuine sensor/vibration noise floor from the telemetry,
    # and matches the command side's own effective bandwidth (a step profile
    # has no content above ~a few Hz anyway) so filtering doesn't introduce
    # an artificial phase/gain mismatch between cmd and tel.
    tel_u = np.column_stack([_lowpass(tel_u_raw[:, k], fs) for k in range(3)])
    a_down_u = _lowpass(a_down_u_raw, fs)
    cmd_u = np.column_stack([_lowpass(cmd_u[:, k], fs) for k in range(3)])
    B_T_u = _lowpass(B_T_u, fs)

    out = {}
    for k, ax in enumerate(["wx", "wy", "wz"]):
        out[ax] = _corr_metrics(cmd_u[:, k], tel_u[:, k], fs)

    out["thrust"] = _corr_metrics(B_T_u, a_down_u, fs)

    if return_series:
        series = dict(t=t_uniform, cmd=cmd_u, tel=tel_u, B_T=B_T_u, a_down=a_down_u)
        return out, series
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


def r2_weighted_mean(rs, values):
    """r^2-weighted mean across ALL runs (no hard reliability cutoff).
    2026-07-21: preferred over the r>=0.5-then-MAD-trim method -- that method
    has a hard boundary discontinuity (r=0.49 fully excluded, r=0.51 fully
    included) that's especially unstable for axes like wy where the
    aggregate r sits right at the cutoff. Weighting by r^2 (variance
    explained) instead lets every run contribute proportional to how much it
    should be trusted, with no arbitrary threshold. Confirmed on wy: hard
    cutoff (n=6) gave gain=1.746; r^2-weighted (all 47 runs) gives 1.430 --
    a large, threshold-driven difference, not noise."""
    num = den = 0.0
    n_eff = 0.0  # effective sample size (sum of weights, informational)
    for r, v in zip(rs, values):
        if not (np.isfinite(r) and np.isfinite(v)):
            continue
        w = r * r
        num += w * v
        den += w
        n_eff += w
    if den < 1e-9:
        return float("nan"), 0.0
    return num / den, n_eff


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
    print(f'    {"run":32s}  {"axis":6s}  {"r":>6s}  {"lag_ms":>7s}  {"gain":>7s}')
    all_metrics = {ax: dict(r=[], lag_ms=[], gain=[]) for ax in AXES}
    for d in runs:
        m = per_run_metrics(os.path.join(cal_dir, d))
        if m is None:
            print(f"    {d:30s}  SKIPPED (insufficient samples)")
            continue
        batt_v = None
        try:
            gt = np.load(os.path.join(cal_dir, d, "Ground_Truth.npy"), allow_pickle=True).item()
            batt_v = gt.get("Battery Voltage")
        except Exception:
            pass
        batt_str = f"{batt_v:.2f}V" if batt_v else "n/a"
        for ax in AXES:
            print(f'    {d:32s}  {ax:6s}  {m[ax]["r"]:6.3f}  {m[ax]["lag_ms"]:7.1f}  {m[ax]["gain"]:7.3f}  batt={batt_str}')
            for key in ("r", "lag_ms", "gain"):
                all_metrics[ax][key].append(m[ax][key])

    # 2026-07-21: filter by reliability (r >= RELIABLE_R) BEFORE MAD-trimming,
    # per-axis. MAD-trim alone filters by gain-VALUE outlier status, which is
    # not the same thing as data quality -- it was silently keeping near-zero/
    # negative-r noise runs (small gain magnitude looks "central") while
    # dropping genuinely good runs whose gain happened to sit further from the
    # pooled median. Confirmed on the thrust axis: unfiltered MAD-trim gave
    # gain=0.635 (implied mass +30.8% off known); r>=0.5-filtered-first gives
    # gain=0.745 (+11.5% off) -- a large, systematic difference, not noise.
    RELIABLE_R = 0.5
    print(f"\n  Across-run aggregation (r>={RELIABLE_R} filter applied per-axis, "
          f"then MAD-trimmed mean, k_MAD=2.5):")
    print(f'    {"axis":6s}  {"r":>10s}  {"lag_ms":>11s}  {"gain":>10s}  {"n_reliable":>11s}')
    reliable_gain_by_axis = {}
    for ax in AXES:
        r_list = all_metrics[ax]["r"]
        reliable_idx = [i for i, r in enumerate(r_list) if np.isfinite(r) and r >= RELIABLE_R]
        n_reliable = len(reliable_idx)
        r_vals = [r_list[i] for i in reliable_idx]
        l_vals = [all_metrics[ax]["lag_ms"][i] for i in reliable_idx]
        g_vals = [all_metrics[ax]["gain"][i] for i in reliable_idx]
        reliable_gain_by_axis[ax] = g_vals
        r_tm, r_n, r_d = mad_trimmed_mean(r_vals)
        l_tm, l_n, l_d = mad_trimmed_mean(l_vals)
        g_tm, g_n, g_d = mad_trimmed_mean(g_vals)
        print(f"    {ax:6s}  {r_tm:6.3f} (n={r_n})  {l_tm:7.1f} (n={l_n})  {g_tm:7.3f} (n={g_n})"
              f"  {n_reliable:>11d}/{len(r_list)}")

    print("\n  Across-run aggregation, ALTERNATIVE (r^2-weighted mean, all runs, no cutoff):")
    print(f'    {"axis":6s}  {"gain":>10s}  {"n_eff":>8s}')
    r2w_gain_by_axis = {}
    for ax in AXES:
        r2w_gain, n_eff = r2_weighted_mean(all_metrics[ax]["r"], all_metrics[ax]["gain"])
        r2w_gain_by_axis[ax] = r2w_gain
        print(f"    {ax:6s}  {r2w_gain:10.3f}  {n_eff:8.2f}")
    print("    (n_eff = sum of r^2 weights, informational -- not a literal sample count)")

    print("\n  Interpretation:")
    print("    r close to 1     -> FC closely tracks cmd (good)")
    print("    r much < 1       -> FC response dominated by disturbance rejection / noise")
    print("    lag_ms           -> loop deadtime (MAVSDK transit + PX4 inner loop / actuator)")
    print("    gain (wx/wy/wz) ~= 1.0 -> FC matches cmd amplitude; << 1 = underdamped tracking")
    print("    gain (thrust) = tel_a_down / B_T ~= 1/mass (physical, per Gazebo's")
    print("      feedback_input_cal_thrust_units convention: a_down = +B_T/mass, no leading")
    print("      minus -- positive B_T means LESS thrust, so the drone falls, so a_down rises).")

    thrust_gain = r2w_gain_by_axis["thrust"]
    if np.isfinite(thrust_gain) and thrust_gain > 1e-6:
        mass_derived = 1.0 / thrust_gain
        expected_gain = 1.0 / DRONE_MASS_KG
        # Gazebo's exact re-tuning recipe (feedback_input_cal_thrust_units.md):
        #   new_slope = old_slope * actual_gain / (1/mass)
        # i.e. scale the currently-used slope by how far the measured gain is
        # from the physically-expected 1/mass, using KNOWN mass (not the
        # noisier derived-mass cross-check) for the final recommendation.
        old_slope = 30.7  # current live HW_THRUST_SLOPE on the Pi (updated 2026-07-20,
                          # was 34.8 at the time this script's default was written)
        new_slope = old_slope * thrust_gain / expected_gain
        print(f"\n  Thrust-axis cross-check (r^2-weighted, all runs):")
        print(f"    measured gain = {thrust_gain:.4f} m/s^2/N   expected (1/mass) = {expected_gain:.4f} m/s^2/N")
        print(f"    derived mass = 1/gain = {mass_derived:.3f} kg  (known mass ~{DRONE_MASS_KG} kg, "
              f"{100*(mass_derived/DRONE_MASS_KG - 1):+.1f}% off)")
        print(f"    Gazebo re-tuning formula: new_slope = old_slope * gain / (1/mass)")
        print(f"      = {old_slope} * {thrust_gain:.4f} / {expected_gain:.4f} = {new_slope:.2f} N/unit")
        print(f"    -> HW_THRUST_SLOPE candidate: {new_slope:.2f} N/unit (current placeholder: {old_slope})")
    else:
        print("\n  Thrust-axis cross-check: insufficient/non-finite gain data across runs.")


if __name__ == "__main__":
    main()
