#!/usr/bin/env python3
"""
Per-timestep analysis: load a bundle of landing runs, rank them by final
xy_err, and diff time-series channels between best and worst reps to
identify what diverges. Goal is to pinpoint the mechanism of instability,
not just rank outcomes.

Each rep has:
  Ground_Truth.npy   — UAV / Target poses (ROS-rate), SoftPrecise summary
  Control_Data.npy   — controller-side per-step state (w_u, e_R, kappa, ...)
  Telemetry_Data.npy — FC odom (Position/Velocity/Quat/AngVel)
  Img_Data.npy       — image pipeline (Feature Params, Opt Flow, FPS)

Channels of interest (in Control_Data.npy):
  w_u      body-rate command (3,)  — clipped by PLASMC_W_U_MAX
  e_R      SO(3) attitude error (3,)
  I_a      filtered inertial accel command (3,)
  a_u      outer-loop accel before filter (3,)
  kappa    adaptive gain (3,)
  sigma    sliding variable (3,)
  s_e_n    normalized image error (2,)
  is_e_n   integrated normalized image error (2,)
  theta    cone-angle reference (scalar)
  rho_fov  FoV envelope (2,)
  B_T      thrust scalar
  EA_d     desired Euler (3,)
  p        UAV pos in target frame (3,)
  dp       UAV vel in target frame (3,)
"""
from __future__ import annotations
import argparse
import csv
import os
import sys
from pathlib import Path

import numpy as np


# Channels we'll diff: (name in Control_Data, label, idx_or_None)
SCALAR_CHANNELS = [
    ("|w_u|",         "w_u(t)",        "vec_norm"),
    ("|e_R|",         "e_R(t)",        "vec_norm"),
    ("|I_a|",         "I_a(t)",        "vec_norm"),
    ("|a_u|",         "a_u(t)",        "vec_norm"),
    ("|kappa|",       "kappa(t)",      "vec_norm"),
    ("|sigma|",       "sigma(t)",      "vec_norm"),
    ("|s_e_n|",       "s_e_n(t)",      "vec_norm"),
    ("|dp|",          "dp(t)",         "vec_norm"),
    ("theta_curr",    "theta_current(t)", "scalar"),
    ("kappa_a",       "kappa_a(t)",    "scalar"),
    ("B_T",           "B_T(t)",        "scalar"),
]


def _to_array(seq):
    """Robust convert list[scalar] or list[ndarray] → ndarray."""
    return np.asarray([np.asarray(x) for x in seq])


def load_rep(rep_dir: Path) -> dict:
    """Load all .npy files from a rep directory into a dict of arrays."""
    out = {"dir": rep_dir}
    for fname in [
        "Control_Data.npy",
        "Telemetry_Data.npy",
        "Img_Data.npy",
        "Ground_Truth.npy",
    ]:
        path = rep_dir / fname
        if not path.exists():
            return None
        data = np.load(path, allow_pickle=True)
        if data.dtype == object and data.shape == ():
            data = data.item()
        out[fname[:-4]] = data

    gt = out["Ground_Truth"]
    sp = gt.get("SoftPrecise", {})
    out["xy_err"] = float(sp.get("xy_err", np.nan))
    out["rel_vel"] = float(sp.get("rel_vel", np.nan))
    out["target_lost"] = bool(sp.get("target_lost", False))
    out["precise"] = bool(sp.get("precise", False))
    out["soft"] = bool(sp.get("soft", False))
    return out


def channel_series(rep: dict, key: str) -> tuple[np.ndarray, np.ndarray]:
    """Return (t, value) arrays from Control_Data, normalising t to start=0."""
    cd = rep["Control_Data"]
    t = np.asarray(cd["t(t)"] if "t(t)" in cd else cd["t"], dtype=float)
    t = t - t[0]
    v = _to_array(cd[key])
    n = min(len(t), len(v))
    return t[:n], v[:n]


def summarize_rep(rep: dict) -> dict:
    """Per-rep summary statistics across all SCALAR_CHANNELS."""
    stats = {
        "rep": rep["dir"].name,
        "xy_err": rep["xy_err"],
        "rel_vel": rep["rel_vel"],
        "target_lost": int(rep["target_lost"]),
        "precise": int(rep["precise"]),
        "soft": int(rep["soft"]),
    }
    for label, key, kind in SCALAR_CHANNELS:
        try:
            t, v = channel_series(rep, key)
        except KeyError:
            continue
        v = v.astype(float)
        if kind == "vec_norm":
            mag = np.linalg.norm(v, axis=-1) if v.ndim > 1 else np.abs(v)
        else:
            mag = np.abs(v)
        if mag.size == 0:
            continue
        stats[f"{label}_max"] = float(np.max(mag))
        stats[f"{label}_mean"] = float(np.mean(mag))
        stats[f"{label}_p95"] = float(np.percentile(mag, 95))
        # Final 1 second mean (terminal behavior)
        if len(t) > 30:
            tail_mask = t >= (t[-1] - 1.0)
            stats[f"{label}_tail"] = float(np.mean(mag[tail_mask]))
    # Time at end (flight duration of controller log)
    try:
        t_ctrl, _ = channel_series(rep, "w_u(t)")
        stats["t_ctrl_end"] = float(t_ctrl[-1])
    except Exception:
        pass
    # Image pipeline FPS stats
    try:
        fps = np.asarray(rep["Img_Data"]["FPS"], dtype=float)
        stats["fps_mean"] = float(np.mean(fps))
        stats["fps_min"] = float(np.min(fps))
        stats["fps_std"] = float(np.std(fps))
    except (KeyError, ValueError):
        pass
    # W_U clamp engagement: fraction of timesteps where any axis hits 1.0
    try:
        _, w_u = channel_series(rep, "w_u(t)")
        clamp_frac = float(np.mean(np.any(np.abs(w_u) >= 0.999, axis=-1)))
        stats["w_u_clamp_frac"] = clamp_frac
    except KeyError:
        pass
    return stats


def diff_best_vs_worst(reps: list, out_csv: Path):
    """For each scalar channel, compare best vs worst trajectories
    (sorted by xy_err with target_lost ranked worst). Output per-channel
    statistics + an aligned time-window comparison.
    """
    # Sort: TL last, then xy_err ascending
    reps_sorted = sorted(
        reps, key=lambda r: (1 if r["target_lost"] else 0, r["xy_err"])
    )
    print()
    print("=" * 90)
    print("Reps sorted by outcome (best → worst):")
    print(f"{'rep':<12} {'xy_err':>8} {'rel_vel':>8}  TL  P  S")
    for r in reps_sorted:
        print(f"  {r['dir'].name:<12} {r['xy_err']:>8.4f} {r['rel_vel']:>8.3f}  "
              f"{int(r['target_lost'])}  {int(r['precise'])}  {int(r['soft'])}")

    if len(reps_sorted) < 2:
        print("Need ≥ 2 reps to diff.")
        return

    best = reps_sorted[0]
    worst = reps_sorted[-1]
    print()
    print("=" * 90)
    print(f"DIFF: BEST = {best['dir'].name} (xy={best['xy_err']:.3f})")
    print(f"      WORST = {worst['dir'].name} (xy={worst['xy_err']:.3f})")
    print("=" * 90)
    print(f"{'channel':<15} {'best_max':>10} {'worst_max':>10}  "
          f"{'best_mean':>10} {'worst_mean':>10}  {'best_tail':>10} {'worst_tail':>10}  "
          f"{'ratio_max':>9}")
    for label, key, kind in SCALAR_CHANNELS:
        try:
            t_b, v_b = channel_series(best, key)
            t_w, v_w = channel_series(worst, key)
        except KeyError:
            continue
        v_b = v_b.astype(float); v_w = v_w.astype(float)
        if kind == "vec_norm":
            m_b = np.linalg.norm(v_b, axis=-1) if v_b.ndim > 1 else np.abs(v_b)
            m_w = np.linalg.norm(v_w, axis=-1) if v_w.ndim > 1 else np.abs(v_w)
        else:
            m_b = np.abs(v_b); m_w = np.abs(v_w)
        if m_b.size == 0 or m_w.size == 0:
            continue
        tail_b = np.mean(m_b[t_b >= t_b[-1] - 1.0]) if len(t_b) > 30 else float("nan")
        tail_w = np.mean(m_w[t_w >= t_w[-1] - 1.0]) if len(t_w) > 30 else float("nan")
        ratio = np.max(m_w) / np.max(m_b) if np.max(m_b) > 0 else float("inf")
        print(f"{label:<15} {np.max(m_b):>10.3f} {np.max(m_w):>10.3f}  "
              f"{np.mean(m_b):>10.3f} {np.mean(m_w):>10.3f}  "
              f"{tail_b:>10.3f} {tail_w:>10.3f}  {ratio:>9.2f}×")

    # Per-rep summary table → CSV
    rows = [summarize_rep(r) for r in reps]
    cols = sorted({k for row in rows for k in row.keys()})
    with open(out_csv, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        for row in rows:
            w.writerow(row)
    print(f"\nPer-rep stats CSV → {out_csv}")
    return rows


def correlate_with_xy(rows: list):
    """Find which per-rep summary metrics correlate best with final xy_err.
    Across N≥5 reps, this identifies which mid-flight signal predicts
    the bad outcome — i.e., the mechanism of instability.
    """
    # Exclude target_lost reps from the correlation (they're a different failure mode)
    landed = [r for r in rows if not r.get("target_lost")]
    if len(landed) < 3:
        print("\nToo few non-TL reps to correlate.")
        return
    xy = np.array([r["xy_err"] for r in landed])
    keys = [k for k in landed[0].keys()
            if isinstance(landed[0][k], (int, float))
            and k not in ("xy_err", "rel_vel", "target_lost", "precise", "soft")]
    print()
    print("=" * 90)
    print(f"PEARSON CORRELATION with xy_err (n={len(landed)} non-TL reps)")
    print("=" * 90)
    print("Positive ρ → high channel = bad xy.  Negative ρ → low channel = bad xy.")
    cors = []
    for k in keys:
        vals = np.array([r.get(k, np.nan) for r in landed], dtype=float)
        mask = ~np.isnan(vals)
        if mask.sum() < 3 or np.std(vals[mask]) == 0:
            continue
        if np.std(xy[mask]) == 0:
            continue
        rho = float(np.corrcoef(vals[mask], xy[mask])[0, 1])
        cors.append((k, rho, vals[mask].mean()))
    cors.sort(key=lambda x: -abs(x[1]))
    print(f"  {'metric':<30}  {'ρ':>6}  {'mean':>9}")
    for k, rho, m in cors[:20]:
        print(f"  {k:<30}  {rho:>+.3f}  {m:>9.3f}")


def find_divergence_time(reps: list, channel: str, kind: str = "vec_norm"):
    """For one channel, find the earliest time at which good vs bad reps
    have statistically distinguishable distributions. This gives a
    causal anchor: 'instability starts at t=X seconds'.
    """
    good = [r for r in reps if not r["target_lost"] and r["xy_err"] < 0.3]
    bad = [r for r in reps if r["target_lost"] or r["xy_err"] > 0.5]
    if len(good) < 2 or len(bad) < 2:
        return None

    # Resample onto common time grid (1ms)
    def get_mag(r):
        t, v = channel_series(r, channel)
        v = v.astype(float)
        if kind == "vec_norm" and v.ndim > 1:
            return t, np.linalg.norm(v, axis=-1)
        return t, np.abs(v)

    t_max = min(get_mag(r)[0][-1] for r in good + bad)
    t_grid = np.linspace(0, t_max, 200)

    def resample(r):
        t, m = get_mag(r)
        return np.interp(t_grid, t, m)

    good_arr = np.array([resample(r) for r in good])
    bad_arr = np.array([resample(r) for r in bad])
    # Per-time-bin gap: (mean_bad - mean_good) / pooled_std
    mu_g = good_arr.mean(axis=0); sd_g = good_arr.std(axis=0)
    mu_b = bad_arr.mean(axis=0); sd_b = bad_arr.std(axis=0)
    pooled = np.sqrt((sd_g ** 2 + sd_b ** 2) / 2 + 1e-9)
    z = (mu_b - mu_g) / pooled
    return t_grid, z, mu_g, mu_b


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bundle_dir", help="Directory containing repN subdirs")
    ap.add_argument("--out", default=None, help="Per-rep stats CSV path")
    args = ap.parse_args()

    bundle = Path(args.bundle_dir)
    if not bundle.exists():
        sys.exit(f"No such directory: {bundle}")
    rep_dirs = sorted(
        [d for d in bundle.iterdir() if d.is_dir() and (d / "Control_Data.npy").exists()]
    )
    print(f"Found {len(rep_dirs)} rep directories in {bundle}")
    reps = [r for r in (load_rep(d) for d in rep_dirs) if r is not None]
    print(f"Loaded {len(reps)} reps.")
    if not reps:
        sys.exit("No usable reps found.")

    out_csv = Path(args.out) if args.out else bundle / "timeseries_stats.csv"
    rows = diff_best_vs_worst(reps, out_csv) or []
    correlate_with_xy(rows)

    # Divergence-time analysis for the top channels
    print()
    print("=" * 90)
    print("DIVERGENCE-TIME ANALYSIS  (z-score of bad vs good distributions)")
    print("=" * 90)
    for label, key, kind in [
        ("|w_u|", "w_u(t)", "vec_norm"),
        ("|e_R|", "e_R(t)", "vec_norm"),
        ("|I_a|", "I_a(t)", "vec_norm"),
        ("|kappa|", "kappa(t)", "vec_norm"),
        ("|dp|", "dp(t)", "vec_norm"),
        ("|s_e_n|", "s_e_n(t)", "vec_norm"),
    ]:
        res = find_divergence_time(reps, key, kind)
        if res is None:
            continue
        t_grid, z, mu_g, mu_b = res
        # Earliest time at which |z| > 1 sustained for at least 5 bins
        sustained = np.zeros_like(z, dtype=bool)
        for i in range(len(z) - 5):
            if np.all(np.abs(z[i:i + 5]) > 1.0):
                sustained[i] = True
                break
        idx = np.argmax(sustained) if sustained.any() else None
        if idx is not None and sustained.any():
            print(f"  {label:<10} diverges at t≈{t_grid[idx]:.2f}s  "
                  f"(good_mean={mu_g[idx]:.3f}, bad_mean={mu_b[idx]:.3f}, z={z[idx]:+.2f})")
        else:
            print(f"  {label:<10} no sustained |z|>1 divergence — no clear good/bad separation")
            # Report peak |z| anyway
            i = int(np.argmax(np.abs(z)))
            print(f"             peak |z|={abs(z[i]):.2f} at t={t_grid[i]:.2f}s "
                  f"(good={mu_g[i]:.3f}, bad={mu_b[i]:.3f})")


if __name__ == "__main__":
    main()
