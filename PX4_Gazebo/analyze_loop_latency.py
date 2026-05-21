#!/usr/bin/env python3
"""
Phase 2: end-to-end loop-latency budget for the PX4 SITL pipeline.

Goal: identify where in the loop the dominant phase lag lives. Phase 1
showed the controller works in MATLAB (10/10 SP at IC1) but fails in PX4
(0/30 SP at same IC) — a 16× xy_mean ratio. Loop lag is a plausible
dominant gap source.

Pipeline stages (image → actuator response):
    1. Camera capture (Gazebo physics → image at /image topic)
    2. ROS 2 bridge: /image → Python image_processor
    3. Centroid + ArUco detection
    4. Savgol filter group delay = (WIN-1)/2 / fps
    5. Controller compute (image features → w_u + B_T)
    6. MAVSDK rate setpoint sent
    7. PX4 internal rate controller → motor PWM
    8. Motor response → body angular accel
    9. Integrated → body angular velocity (telemetry)

We can measure stages 4 + 5 + 6 + 7 + 8 + 9 together by cross-correlating
`w_u(t)` (controller-commanded rate) with `Angular Velocity Body(t)`
(measured rate). Peak lag is the round-trip controller → actuator.

Usage:
    python3 analyze_loop_latency.py <bundle_dir>
        — defaults to ~/ws/Test_Data/DefaultN10/<latest>
"""
from __future__ import annotations
import argparse
import os
import sys
import glob
from pathlib import Path

import numpy as np


def _to_array(seq):
    return np.asarray([np.asarray(x) for x in seq])


def load_rep(rep_dir: Path) -> dict | None:
    out = {"dir": rep_dir}
    for fname in ["Control_Data.npy", "Telemetry_Data.npy", "Img_Data.npy",
                  "Ground_Truth.npy"]:
        path = rep_dir / fname
        if not path.exists():
            return None
        data = np.load(path, allow_pickle=True)
        if data.dtype == object and data.shape == ():
            data = data.item()
        out[fname[:-4]] = data
    return out


def cmd_vs_actual_lag(rep):
    """Cross-correlate commanded body rates (w_u) vs measured (Angular Velocity Body).
    Returns lag-at-peak in milliseconds for each axis."""
    cd = rep["Control_Data"]
    td = rep["Telemetry_Data"]
    t_ctrl = np.asarray(cd["t"], dtype=float)
    w_u = _to_array(cd["w_u(t)"])
    if w_u.ndim != 2 or w_u.shape[1] != 3:
        return None

    # Telemetry's Angular Velocity Body — list of AngularVelocityBody objects
    avb = td.get("Angular Velocity Body", [])
    if not avb:
        return None
    # MAVSDK AngularVelocityBody fields: roll_rad_s / pitch_rad_s / yaw_rad_s
    w_meas = np.array([[v.roll_rad_s, v.pitch_rad_s, v.yaw_rad_s] for v in avb])

    # Time bases: controller has explicit t; telemetry doesn't necessarily
    # — assume uniform sampling between Start Time and last log time.
    # We resample both onto a common 1ms grid covering the controller's range.
    t_meas_start = float(t_ctrl[0])
    t_meas_end   = float(t_ctrl[-1])
    n_meas = len(w_meas)
    t_meas = np.linspace(t_meas_start, t_meas_end, n_meas)

    # Trim to controller window
    mask_c = (t_ctrl >= t_meas_start) & (t_ctrl <= t_meas_end)
    t_c = t_ctrl[mask_c]; w_c = w_u[mask_c]

    # Common time grid (5 ms steps — fine enough for 1-50 ms lag resolution)
    dt = 0.005
    t_grid = np.arange(t_c[0], t_c[-1], dt)

    lags_ms = []
    coeffs  = []
    for axis in range(3):
        # Interpolate to common grid
        c = np.interp(t_grid, t_c, w_c[:, axis])
        m = np.interp(t_grid, t_meas, w_meas[:, axis])
        # Detrend (remove DC)
        c = c - np.mean(c)
        m = m - np.mean(m)
        if np.std(c) < 1e-6 or np.std(m) < 1e-6:
            lags_ms.append(np.nan); coeffs.append(np.nan); continue
        # Normalized cross-correlation; lag k>0 means measured trails command
        corr = np.correlate(m, c, mode="full")
        norm = np.std(c) * np.std(m) * len(c)
        corr = corr / norm
        lags = (np.arange(len(corr)) - (len(c) - 1)) * dt * 1000  # ms
        # Restrict to plausible lag range: -50 to +300 ms
        mask = (lags >= -50) & (lags <= 300)
        peak_idx = np.argmax(corr[mask])
        peak_lag = lags[mask][peak_idx]
        peak_corr = corr[mask][peak_idx]
        lags_ms.append(peak_lag)
        coeffs.append(peak_corr)
    return {"lag_ms_xyz": lags_ms, "corr_xyz": coeffs}


def image_pipeline_delay(rep):
    """Look at the image FPS and the difference between Img_Data timestamps
    and Control_Data timestamps. Returns pipeline-side stats."""
    img = rep["Img_Data"]
    ctrl = rep["Control_Data"]
    t_img = np.asarray(img["Time"], dtype=float) if "Time" in img else None
    fps = np.asarray(img.get("FPS", []), dtype=float)
    out = {
        "fps_mean": float(np.mean(fps)) if len(fps) else float("nan"),
        "fps_min":  float(np.min(fps))  if len(fps) else float("nan"),
        "fps_p5":   float(np.percentile(fps, 5)) if len(fps) else float("nan"),
        "fps_std":  float(np.std(fps))  if len(fps) else float("nan"),
    }
    # Savgol group delay (img_data.py FILTER_WIN=7, applied symmetrically)
    win = 7
    out["savgol_group_delay_ms"] = (win - 1) / 2 / max(out["fps_mean"], 1.0) * 1000
    # Image age at controller tick — for each controller tick, find the most
    # recent image timestamp before t_ctrl; the gap is staleness.
    if t_img is not None and len(t_img) > 1:
        t_ctrl = np.asarray(ctrl["t"], dtype=float)
        # If timestamps share a base, ages make sense
        ages = []
        j = 0
        for t in t_ctrl:
            while j + 1 < len(t_img) and t_img[j + 1] <= t:
                j += 1
            if t_img[j] <= t:
                ages.append((t - t_img[j]) * 1000)
        ages = np.array(ages)
        if len(ages):
            out["image_age_ms_mean"] = float(np.mean(ages))
            out["image_age_ms_p95"]  = float(np.percentile(ages, 95))
            out["image_age_ms_max"]  = float(np.max(ages))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bundle", nargs="?",
                    default=None,
                    help="Bundle dir (defaults to latest DefaultN10)")
    args = ap.parse_args()

    if args.bundle is None:
        candidates = sorted(glob.glob(os.path.expanduser(
            "~/ws/Test_Data/DefaultN10/*")))
        if not candidates:
            sys.exit("No DefaultN10 bundle found; pass a path explicitly.")
        args.bundle = candidates[-1]

    bundle = Path(args.bundle)
    rep_dirs = sorted([d for d in bundle.iterdir()
                       if d.is_dir() and (d / "Control_Data.npy").exists()])
    print(f"Bundle: {bundle}  ({len(rep_dirs)} reps)\n")

    rate_lags = []
    img_stats = []
    for d in rep_dirs:
        rep = load_rep(d)
        if rep is None:
            continue
        rl = cmd_vs_actual_lag(rep)
        is_ = image_pipeline_delay(rep)
        if rl: rate_lags.append({"rep": d.name, **rl})
        if is_: img_stats.append({"rep": d.name, **is_})

    # ===== Rate-loop lag =====
    print("=" * 88)
    print("STAGE 6–9 (MAVSDK → PX4 rate ctrl → actuator → measured): cross-correlation peak lag")
    print("=" * 88)
    print(f"{'rep':<8} {'lag_X (ms)':>11} {'lag_Y':>10} {'lag_Z':>10}  "
          f"{'corr_X':>8} {'corr_Y':>7} {'corr_Z':>7}")
    by_axis = [[], [], []]
    for r in rate_lags:
        lx, ly, lz = r["lag_ms_xyz"]
        cx, cy, cz = r["corr_xyz"]
        print(f"  {r['rep']:<6} {lx:>11.1f} {ly:>10.1f} {lz:>10.1f}  "
              f"{cx:>8.3f} {cy:>7.3f} {cz:>7.3f}")
        for ax, val in zip(by_axis, [lx, ly, lz]):
            if not np.isnan(val): ax.append(val)
    if any(len(a) for a in by_axis):
        print()
        for name, vals in zip(["X (roll-rate)", "Y (pitch-rate)", "Z (yaw-rate)"], by_axis):
            if not vals: continue
            arr = np.array(vals)
            print(f"  {name:<18}  median={np.median(arr):>5.1f}ms  "
                  f"mean={arr.mean():>5.1f}ms  std={arr.std():>4.1f}")

    # ===== Image pipeline =====
    print()
    print("=" * 88)
    print("STAGE 1–4 (camera → ROS bridge → centroid → savgol): per-rep")
    print("=" * 88)
    print(f"{'rep':<8} {'fps_mean':>9} {'fps_p5':>7} {'fps_min':>8} "
          f"{'savgol_lag':>11} {'img_age_mean':>13} {'p95':>6}")
    for r in img_stats:
        sav = r.get("savgol_group_delay_ms", float("nan"))
        am = r.get("image_age_ms_mean", float("nan"))
        a95 = r.get("image_age_ms_p95", float("nan"))
        print(f"  {r['rep']:<6} {r['fps_mean']:>9.1f} "
              f"{r['fps_p5']:>7.1f} {r['fps_min']:>8.1f} "
              f"{sav:>10.1f}ms {am:>13.1f}ms {a95:>5.1f}ms")
    print()
    fps_means = np.array([r["fps_mean"] for r in img_stats])
    sav_lags  = np.array([r["savgol_group_delay_ms"] for r in img_stats])
    ages_mean = np.array([r.get("image_age_ms_mean", np.nan) for r in img_stats])
    print(f"  fps_mean across reps:        median={np.median(fps_means):.1f}  "
          f"min={fps_means.min():.1f}")
    print(f"  savgol group delay (WIN=7):  median={np.median(sav_lags):.1f}ms")
    if not np.all(np.isnan(ages_mean)):
        print(f"  image age at controller:     median={np.nanmedian(ages_mean):.1f}ms")

    # ===== Total budget =====
    print()
    print("=" * 88)
    print("TOTAL LOOP LATENCY BUDGET")
    print("=" * 88)
    total_lag_mean = []
    for r1, r2 in zip(rate_lags, img_stats):
        rate_med = np.nanmedian([l for l in r1["lag_ms_xyz"] if not np.isnan(l)])
        sav = r2.get("savgol_group_delay_ms", 0)
        age = r2.get("image_age_ms_mean", 0)
        total = sav + age + rate_med
        total_lag_mean.append(total)
    if total_lag_mean:
        arr = np.array(total_lag_mean)
        print(f"  Estimated full-loop delay (savgol + image_age + rate_loop):")
        print(f"    median={np.median(arr):.1f}ms  mean={arr.mean():.1f}ms  "
              f"std={arr.std():.1f}ms  range=[{arr.min():.0f}, {arr.max():.0f}]")
    print()
    print("  MATLAB reference: controller runs at dt=10ms with ZOH=3 (30 Hz");
    print("    image), so MATLAB's effective lag is ~13ms (dt + image age).")
    print("  Soft+precise xy=0.08m at descent vel=0.7m/s gives 114ms time-")
    print("    constant. If PX4 lag >> 100ms, the controller cannot react")
    print("    fast enough to image errors near touchdown — that's the gap.")


if __name__ == "__main__":
    main()
