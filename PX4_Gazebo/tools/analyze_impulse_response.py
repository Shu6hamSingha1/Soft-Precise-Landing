#!/usr/bin/env python3
"""Analyze impulse_response.py output to extract rate-loop deadtime + tau.

Methodology:
  For each impulse event (axis + magnitude + duration):
    1. Pre-impulse baseline = mean ω just before t_start
    2. Step amplitude        = commanded magnitude
    3. Locate t_first_response — first time after t_start where
       |ω(t) - baseline| > 10% of step amplitude (out of measurement noise)
    4. Deadtime t_d = t_first_response - t_start
    5. Within [t_first_response, t_first_response + 0.5s], fit first-order
       step response  ω(t) = baseline + A · (1 - exp(-(t - t_first)/τ))
       extract τ and A.  A should be ≈ magnitude if the rate loop saturates;
       smaller A means rate loop didn't fully reach the commanded value
       in the impulse window.
  Effective total lag for control purposes ≈ t_d + τ (63% rise) or
  t_d + 3τ (95% settle).

Usage:
    python3 analyze_impulse_response.py [<impulse_log.npy>]
"""
from __future__ import annotations
import argparse
import glob
import os
import sys

import numpy as np


def find_latest():
    cands = sorted(glob.glob(os.path.expanduser(
        "~/ws/Test_Data/ImpulseResponse/*/impulse_log.npy")))
    return cands[-1] if cands else None


def fit_first_order(t, y, baseline, t_step):
    """Fit y(t) = baseline + A * (1 - exp(-(t-t_step)/tau)) over the
    window [t_step, t_step + 0.5s]. Returns dict or None if no fit.
    """
    mask = (t >= t_step) & (t <= t_step + 0.5)
    if mask.sum() < 10:
        return None
    tw = t[mask] - t_step
    yw = y[mask] - baseline
    # Guess A from final value, tau from time to 63%
    A_guess = yw[-min(5, len(yw)):].mean()
    if abs(A_guess) < 0.01:
        return None
    target = 0.63 * A_guess
    idx = np.argmax(np.abs(yw) >= abs(target))
    tau_guess = max(tw[idx], 0.001) if idx > 0 else 0.05

    # Simple Levenberg-Marquardt
    try:
        from scipy.optimize import curve_fit
        def model(tt, A, tau):
            return A * (1 - np.exp(-tt / max(tau, 1e-4)))
        popt, pcov = curve_fit(model, tw, yw, p0=[A_guess, tau_guess],
                               maxfev=200, bounds=([-2, 0.001], [2, 1.0]))
        A_fit, tau_fit = popt
        residuals = yw - model(tw, *popt)
        ss_res = np.sum(residuals ** 2)
        ss_tot = np.sum((yw - yw.mean()) ** 2)
        r2 = 1 - ss_res / ss_tot if ss_tot > 0 else 0
        return {"A": float(A_fit), "tau": float(tau_fit), "r2": float(r2)}
    except (ImportError, RuntimeError, ValueError):
        return {"A": float(A_guess), "tau": float(tau_guess), "r2": 0.0}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log", nargs="?", default=None)
    ap.add_argument("--noise_thresh", type=float, default=0.1,
                    help="Fraction of step magnitude that counts as 'response started'")
    args = ap.parse_args()

    path = args.log or find_latest()
    if path is None:
        sys.exit("No impulse_log.npy found.")
    d = np.load(path, allow_pickle=True).item()
    t = d["t"]; cmd = d["cmd"]; meas = d["meas"]; event = d["event"]
    seq = d["sequence"]
    print(f"Loaded {path}")
    print(f"  samples: {len(t)}   sample rate: ~{(len(t)-1)/(t[-1]-t[0]):.0f} Hz")
    print(f"  impulses: {len(seq)}")
    print()
    print("=" * 92)
    print(f"{'#':>2} {'axis':>5} {'mag':>5}  {'t_d (ms)':>9}  {'tau (ms)':>9}  "
          f"{'A_fit':>7} {'r²':>5}  {'lag63 (ms)':>11}  {'lag95 (ms)':>11}")
    print("=" * 92)
    results_by_axis = {0: [], 1: [], 2: []}
    axis_name = ["roll", "pitch", "yaw"]
    for i, (axis, mag, dur) in enumerate(seq):
        # Slice this impulse + 1 settle window (we have settle following each impulse)
        mask = (event == i)
        if mask.sum() < 5:
            continue
        # Find t_step = first time of this event
        t_step = t[mask][0]
        # Pre-impulse baseline: 50 ms just before t_step
        pre_mask = (t > t_step - 0.05) & (t < t_step)
        if pre_mask.sum() < 3:
            continue
        baseline = meas[pre_mask, axis].mean()
        # Take this impulse + next settle period (which has event == -1
        # adjacent in time)
        full_mask = (t >= t_step) & (t < t_step + dur + 0.3)
        tw = t[full_mask]; yw = meas[full_mask, axis]
        # Deadtime: first time |y - baseline| crosses threshold
        thresh = args.noise_thresh * abs(mag)
        rising = np.abs(yw - baseline) >= thresh
        if not rising.any():
            print(f"{i+1:>2} {axis_name[axis]:>5} {mag:>+5.2f}  "
                  f"{'no-response':>9}  {'—':>9}  {'—':>7} {'—':>5}  {'—':>11}  {'—':>11}")
            continue
        first_idx = np.argmax(rising)
        t_first = tw[first_idx]
        t_d = (t_first - t_step) * 1000  # ms
        # Fit first-order from t_first
        fit = fit_first_order(t, meas[:, axis], baseline, t_first)
        if fit is None:
            print(f"{i+1:>2} {axis_name[axis]:>5} {mag:>+5.2f}  "
                  f"{t_d:>9.1f}  {'no-fit':>9}")
            continue
        tau_ms = fit["tau"] * 1000
        lag63 = t_d + tau_ms
        lag95 = t_d + 3 * tau_ms
        print(f"{i+1:>2} {axis_name[axis]:>5} {mag:>+5.2f}  "
              f"{t_d:>9.1f}  {tau_ms:>9.1f}  {fit['A']:>+7.3f} {fit['r2']:>5.2f}  "
              f"{lag63:>11.1f}  {lag95:>11.1f}")
        results_by_axis[axis].append({"t_d_ms": t_d, "tau_ms": tau_ms,
                                       "lag63": lag63, "lag95": lag95,
                                       "r2": fit["r2"]})

    # Per-axis summary
    print()
    print("=" * 92)
    print("PER-AXIS SUMMARY")
    print("=" * 92)
    print(f"{'axis':<6}  {'n':>3}  {'t_d (ms)':>15}  {'τ (ms)':>15}  "
          f"{'lag63 (ms)':>13}  {'lag95 (ms)':>13}")
    for axis in (0, 1, 2):
        rs = results_by_axis[axis]
        if not rs: continue
        td = np.array([r["t_d_ms"] for r in rs])
        ta = np.array([r["tau_ms"] for r in rs])
        l63 = np.array([r["lag63"] for r in rs])
        l95 = np.array([r["lag95"] for r in rs])
        print(f"{axis_name[axis]:<6}  {len(rs):>3}  "
              f"{td.mean():>5.1f} ± {td.std():>5.1f}   "
              f"{ta.mean():>5.1f} ± {ta.std():>5.1f}   "
              f"{l63.mean():>6.1f} ± {l63.std():>4.1f}   "
              f"{l95.mean():>6.1f} ± {l95.std():>4.1f}")

    # Bottom-line comparison
    print()
    print("=" * 92)
    print("VERDICT vs PHASE 2 ESTIMATE")
    print("=" * 92)
    all_l63 = []
    for axis in (0, 1, 2):
        for r in results_by_axis[axis]:
            all_l63.append(r["lag63"])
    if all_l63:
        arr = np.array(all_l63)
        print(f"  Effective rate-loop lag (deadtime + τ): "
              f"median={np.median(arr):.0f} ms  mean={arr.mean():.0f} ± {arr.std():.0f} ms")
        print(f"  Phase 2 noisy estimate from convergent flight: ~168 ms")
        print(f"  MATLAB reference: ~13 ms")
        print(f"  PX4 / MATLAB ratio: {np.median(arr) / 13:.1f}×")


if __name__ == "__main__":
    main()
