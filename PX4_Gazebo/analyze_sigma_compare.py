#!/usr/bin/env python3
"""Compare MATLAB σ(t) trajectory vs PX4 σ(t) trajectory at matched
descent rate (h_rd=-0.70).  Identifies whether the SMC sliding variable
behaves fundamentally differently between the two platforms.

Inputs:
  - MATLAB: MATLAB/Datasets/Phase5/matlab_sigma_h070.mat (rep 1 trace)
  - PX4: DefaultN10/<latest>/rep2  (a PRECISE rep, xy=0.033)
         and DefaultN10/<latest>/rep7 (a bad rep, xy=1.04)

Usage:
    python3 analyze_sigma_compare.py
"""
from __future__ import annotations
import glob
import os
import sys

import numpy as np
from scipy.io import loadmat


def load_matlab_sigma():
    path = "/home/shubham/Soft-Precise-Landing/MATLAB/Datasets/Phase5/matlab_sigma_h070.mat"
    if not os.path.exists(path):
        sys.exit(f"MATLAB σ trace not found: {path}")
    m = loadmat(path)
    return {
        "t":      np.asarray(m["t"]).flatten(),
        "sigma":  np.asarray(m["sigma"]),     # (3, N)
        "kappa":  np.asarray(m["kappa"]),
        "xy":     float(np.asarray(m["xy_err"]).item()),
    }


def load_px4_sigma(rep_dir: str):
    cd = np.load(os.path.join(rep_dir, "Control_Data.npy"),
                 allow_pickle=True).item()
    gt = np.load(os.path.join(rep_dir, "Ground_Truth.npy"),
                 allow_pickle=True).item()
    t = np.asarray(cd["t"], dtype=float)
    t = t - t[0]
    sigma = np.array([np.asarray(x) for x in cd["sigma(t)"]])  # (N, 3)
    sigma = sigma.T  # (3, N) to match MATLAB
    kappa = np.array([np.asarray(x) for x in cd["kappa(t)"]])[:len(t)].T
    return {
        "t":      t,
        "sigma":  sigma,
        "kappa":  kappa,
        "xy":     float(gt["SoftPrecise"]["xy_err"]),
    }


def summarize_trace(label, d):
    sigma_mag = np.linalg.norm(d["sigma"], axis=0)
    sigma_xy_mag = np.linalg.norm(d["sigma"][:2], axis=0)
    sigma_z = np.abs(d["sigma"][2])
    print(f"\n  {label}  (xy_end={d['xy']:.4f}, t_end={d['t'][-1]:.2f}s, n={len(d['t'])})")
    print(f"    |σ|_full   mean={sigma_mag.mean():.4f}  max={sigma_mag.max():.4f}  "
          f"tail(last 0.5s)={sigma_mag[d['t'] > d['t'][-1]-0.5].mean():.4f}")
    print(f"    |σ_xy|     mean={sigma_xy_mag.mean():.4f}  max={sigma_xy_mag.max():.4f}  "
          f"tail={sigma_xy_mag[d['t'] > d['t'][-1]-0.5].mean():.4f}")
    print(f"    |σ_z|      mean={sigma_z.mean():.4f}  max={sigma_z.max():.4f}  "
          f"tail={sigma_z[d['t'] > d['t'][-1]-0.5].mean():.4f}")
    kappa_mag = np.linalg.norm(d["kappa"], axis=0)
    print(f"    |κ|        mean={kappa_mag.mean():.4f}  max={kappa_mag.max():.4f}  "
          f"tail={kappa_mag[d['t'] > d['t'][-1]-0.5].mean():.4f}")


def diff_traces(label_a, a, label_b, b):
    """Sample-by-sample: how do the σ traces differ in shape (after
    time-normalizing both to [0, 1]) and in magnitude?"""
    # Normalize time to [0, 1]
    ta = a["t"] / a["t"][-1]; tb = b["t"] / b["t"][-1]
    common = np.linspace(0, 1, 200)
    print(f"\n=== {label_a}  vs  {label_b}  (time-normalized) ===")
    for name, idx in [("σ_X", 0), ("σ_Y", 1), ("σ_Z", 2)]:
        a_re = np.interp(common, ta, a["sigma"][idx])
        b_re = np.interp(common, tb, b["sigma"][idx])
        diff = a_re - b_re
        rmse = np.sqrt(np.mean(diff ** 2))
        # Final 10% of flight (terminal behaviour matters most)
        tail = (common > 0.9)
        a_tail = a_re[tail].mean(); b_tail = b_re[tail].mean()
        print(f"  {name}: rmse(time-normalized)={rmse:.4f}  "
              f"{label_a}_tail={a_tail:+.4f}  {label_b}_tail={b_tail:+.4f}  "
              f"|diff_tail|={abs(a_tail - b_tail):.4f}")


def main():
    print("=" * 88)
    print("MATLAB σ at h_rd=-0.70 (matches PX4 descent rate)")
    print("=" * 88)
    matlab = load_matlab_sigma()
    summarize_trace("MATLAB (Phase 5 rep 1)", matlab)

    # Find PX4 DefaultN10 bundle
    cands = sorted(glob.glob(os.path.expanduser(
        "~/ws/Test_Data/DefaultN10/*")))
    if not cands:
        sys.exit("No PX4 DefaultN10 bundle.")
    bundle = cands[-1]
    print(f"\nPX4 bundle: {bundle}")
    print("=" * 88)

    # Pick PRECISE and bad reps
    reps = []
    for d in sorted(os.path.join(bundle, x) for x in os.listdir(bundle)):
        if not os.path.isdir(d): continue
        gt_path = os.path.join(d, "Ground_Truth.npy")
        if not os.path.exists(gt_path): continue
        gt = np.load(gt_path, allow_pickle=True).item()
        sp = gt.get("SoftPrecise", {})
        if sp.get("target_lost"): continue
        reps.append((d, float(sp.get("xy_err", 1e9))))
    reps.sort(key=lambda x: x[1])
    best_rep, best_xy = reps[0]
    worst_rep, worst_xy = reps[-1]
    print(f"\nBest PX4 rep:  {os.path.basename(best_rep)}  xy={best_xy:.4f}")
    print(f"Worst PX4 rep: {os.path.basename(worst_rep)} xy={worst_xy:.4f}")

    px4_best  = load_px4_sigma(best_rep)
    px4_worst = load_px4_sigma(worst_rep)
    summarize_trace("PX4 best",  px4_best)
    summarize_trace("PX4 worst", px4_worst)

    diff_traces("MATLAB",   matlab,   "PX4 best", px4_best)
    diff_traces("MATLAB",   matlab,   "PX4 worst", px4_worst)
    diff_traces("PX4 best", px4_best, "PX4 worst", px4_worst)

    # Key diagnostic
    print()
    print("=" * 88)
    print("KEY DIAGNOSTIC: terminal |σ_xy| (how close to sliding surface at touchdown)")
    print("=" * 88)
    def tail_sxy(d):
        sxy = np.linalg.norm(d["sigma"][:2], axis=0)
        return sxy[d["t"] > d["t"][-1] - 0.5].mean()
    print(f"  MATLAB         tail |σ_xy| = {tail_sxy(matlab):.4f}    xy_end={matlab['xy']:.4f}")
    print(f"  PX4 best       tail |σ_xy| = {tail_sxy(px4_best):.4f}    xy_end={px4_best['xy']:.4f}")
    print(f"  PX4 worst      tail |σ_xy| = {tail_sxy(px4_worst):.4f}    xy_end={px4_worst['xy']:.4f}")
    print()
    print("  Interpretation: if MATLAB tail |σ_xy| is small but PX4 tail is large,")
    print("  the SMC sliding surface isn't being maintained on PX4 — controller is")
    print("  fighting plant dynamics, not converging. This would be the residual gap.")


if __name__ == "__main__":
    main()
