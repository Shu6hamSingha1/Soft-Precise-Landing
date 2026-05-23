#!/usr/bin/env python3
"""Per-rep diagnostic for the 2026-05-23 intervention bundle.

Question: why did only rep2 hit SOFT+PRECISE?  What's different about it?

For each rep we extract:
  - Outcome (xy_err, rel_vel, precise, soft, target_lost)
  - IC at controller engagement (vh0, σ0, sigma_xy0)
  - Marker-detection statistics (% frames with FEATURE_IS_STALE flagged, total stale events)
  - σ_xy trajectory (mean / max / tail) — does it converge or diverge?
  - Marker switching (count of side-jumps, side trajectory)
  - Image pipeline (fps, image age)

Then we rank reps by xy and look for variables that segregate SP from non-SP.
"""
from __future__ import annotations
import glob
import os
import sys
from pathlib import Path

import numpy as np


def _to_array(seq):
    return np.asarray([np.asarray(x) for x in seq])


def analyze_one(rep_dir: Path) -> dict:
    cd = np.load(rep_dir / "Control_Data.npy", allow_pickle=True).item()
    td = np.load(rep_dir / "Telemetry_Data.npy", allow_pickle=True).item()
    img = np.load(rep_dir / "Img_Data.npy", allow_pickle=True).item()
    gt = np.load(rep_dir / "Ground_Truth.npy", allow_pickle=True).item()

    t_ctrl = np.asarray(cd["t"], dtype=float)
    sigma = _to_array(cd["sigma(t)"])               # (N, 3)
    s_e_n = _to_array(cd["s_e_n(t)"])
    kappa = _to_array(cd["kappa(t)"])
    sigma_xy = np.linalg.norm(sigma[:, :2], axis=1)
    sen_mag  = np.linalg.norm(s_e_n, axis=1)

    # IC at engagement
    v = td["Velocity Body"][0]
    vh0 = float(np.hypot(v.x_m_s, v.y_m_s))
    vz0 = float(v.z_m_s)
    sigma_xy0 = float(np.linalg.norm(sigma[0, :2]))
    sigma_z0  = float(abs(sigma[0, 2]))
    sigma0_mag = float(np.linalg.norm(sigma[0]))

    # Trajectory statistics
    sigma_xy_mean = float(sigma_xy.mean())
    sigma_xy_max  = float(sigma_xy.max())
    tail = (t_ctrl > t_ctrl[-1] - 0.5)
    sigma_xy_tail = float(sigma_xy[tail].mean())
    sen_tail = float(sen_mag[tail].mean())

    # Marker-detection statistics from Img_Data
    feat_pts = _to_array(img["Image Feature Pts"])   # (N, 2, 4, 2)
    t_img = np.asarray(img["Time"])
    n_img = len(t_img)

    # Count "stuck" frames — same Image_Feature_Pts as previous frame.
    # This is the stale-feature signal that the new intervention catches.
    mc = feat_pts[:, 0, :, :]
    same_as_prev = np.array([np.array_equal(mc[i], mc[i-1]) for i in range(1, len(mc))])
    n_stale = int(same_as_prev.sum())
    stale_pct = 100 * n_stale / max(len(same_as_prev), 1)
    # Find the longest run of consecutive stale frames
    if same_as_prev.any():
        runs = []
        cur = 0
        for s in same_as_prev:
            if s: cur += 1
            else:
                if cur > 0: runs.append(cur)
                cur = 0
        if cur > 0: runs.append(cur)
        max_stale_run = int(max(runs))
        # Total time spent stale (in frames)
        total_stale_frames = int(sum(runs))
    else:
        max_stale_run = 0
        total_stale_frames = 0

    # Restrict to descent phase only (controller-time samples)
    t_ctrl_start = float(t_ctrl[0])
    img_descent_mask = t_img >= t_ctrl_start
    img_descent_count = int(img_descent_mask.sum())

    # Stale-frames during descent (most diagnostic)
    descent_pairs_mask = img_descent_mask[1:] & img_descent_mask[:-1]
    n_stale_descent = int(same_as_prev[descent_pairs_mask].sum()) if descent_pairs_mask.any() else 0
    stale_pct_descent = 100 * n_stale_descent / max(descent_pairs_mask.sum(), 1)

    # Side trajectory + switches during descent
    sides_full = np.linalg.norm(np.diff(np.concatenate([mc, mc[:, :1]], axis=1), axis=1), axis=2)
    side_mean_full = sides_full.mean(axis=1)
    side_desc = side_mean_full[img_descent_mask]
    if len(side_desc) > 1:
        ratios = side_desc[1:] / np.maximum(side_desc[:-1], 1e-3)
        switches = int(((ratios > 2) | (ratios < 0.5)).sum())
        side_min_d = float(side_desc.min())
        side_max_d = float(side_desc.max())
        side_first_d = float(side_desc[0])
        side_last_d = float(side_desc[-1])
    else:
        switches = 0
        side_min_d = side_max_d = side_first_d = side_last_d = float("nan")

    # FPS during descent
    fps = np.asarray(img.get("FPS", []), dtype=float)
    fps_desc = fps[img_descent_mask] if len(fps) == n_img else fps
    fps_mean = float(fps_desc.mean()) if len(fps_desc) else float("nan")
    fps_min = float(fps_desc.min()) if len(fps_desc) else float("nan")

    sp = gt.get("SoftPrecise", {})
    return {
        "rep": rep_dir.name,
        "xy_end": float(sp.get("xy_err", np.nan)),
        "vel_end": float(sp.get("rel_vel", np.nan)),
        "precise": int(sp.get("precise", False)),
        "soft": int(sp.get("soft", False)),
        "target_lost": int(sp.get("target_lost", False)),
        "soft_precise": int(sp.get("precise") and sp.get("soft") and not sp.get("target_lost")),

        # IC
        "vh0": vh0, "vz0": vz0,
        "sigma_xy0": sigma_xy0, "sigma_z0": sigma_z0, "sigma0_mag": sigma0_mag,

        # Trajectory
        "sigma_xy_mean": sigma_xy_mean,
        "sigma_xy_max":  sigma_xy_max,
        "sigma_xy_tail": sigma_xy_tail,
        "sen_tail": sen_tail,
        "flight_s": float(t_ctrl[-1] - t_ctrl[0]),

        # Marker / image
        "stale_pct_descent": stale_pct_descent,
        "max_stale_run_frames": max_stale_run,
        "switches": switches,
        "side_min_d": side_min_d,
        "side_max_d": side_max_d,
        "side_first_d": side_first_d,
        "side_last_d": side_last_d,
        "fps_mean": fps_mean,
        "fps_min": fps_min,
    }


def main():
    cands = sorted(glob.glob(os.path.expanduser("~/ws/Test_Data/Interventions/*")))
    if not cands:
        sys.exit("No intervention bundle.")
    bundle = Path(cands[-1])
    print(f"Bundle: {bundle}\n")

    rep_dirs = sorted([d for d in bundle.iterdir()
                       if d.is_dir() and (d / "Control_Data.npy").exists()],
                      key=lambda d: int(d.name.replace("rep", "")))
    rows = [analyze_one(d) for d in rep_dirs]
    rows.sort(key=lambda r: r["xy_end"])

    # Header
    print("=" * 130)
    print(f"{'rep':<6} {'xy':>7} {'vel':>6} {'P':>2} {'S':>2} {'SP':>3}  "
          f"{'vh0':>5} {'σxy0':>6} {'σxy_tail':>9} {'sen_tail':>8}  "
          f"{'stale%':>7} {'maxrun':>6} {'switches':>8}  "
          f"{'side_max':>8} {'fps':>5}  {'flight_s':>8}")
    print("=" * 130)
    for r in rows:
        sp_marker = "★" if r["soft_precise"] else " "
        print(f"  {r['rep']:<4}{sp_marker} {r['xy_end']:>7.3f} {r['vel_end']:>6.2f} "
              f"{r['precise']:>2} {r['soft']:>2} {r['soft_precise']:>3}  "
              f"{r['vh0']:>5.2f} {r['sigma_xy0']:>6.3f} {r['sigma_xy_tail']:>9.3f} "
              f"{r['sen_tail']:>8.3f}  "
              f"{r['stale_pct_descent']:>6.1f}% {r['max_stale_run_frames']:>6} "
              f"{r['switches']:>8}  {r['side_max_d']:>8.1f} {r['fps_mean']:>5.1f}  "
              f"{r['flight_s']:>8.2f}")

    # ===== Highlight rep2 vs others =====
    print()
    print("=" * 130)
    print("DIFF: rep2 (SOFT+PRECISE) vs OTHERS")
    print("=" * 130)
    rep2 = next(r for r in rows if r["rep"] == "rep2")
    others = [r for r in rows if r["rep"] != "rep2"]
    keys = [
        ("xy_end", "lower better"),
        ("vh0", "?"),
        ("sigma_xy0", "lower better"),
        ("sigma_xy_tail", "lower better"),
        ("sen_tail", "lower better"),
        ("stale_pct_descent", "lower better"),
        ("max_stale_run_frames", "lower better"),
        ("switches", "?"),
        ("side_max_d", "higher = small marker visible"),
        ("fps_mean", "higher better"),
        ("flight_s", "?"),
    ]
    print(f"  {'metric':<22} {'rep2':>8} {'others_mean':>12} {'others_std':>11} "
          f"{'others_min':>11} {'others_max':>11}  note")
    for k, note in keys:
        v_r2 = rep2[k]
        v_o = np.array([r[k] for r in others])
        if np.all(np.isnan(v_o)) or v_o.std() == 0:
            continue
        print(f"  {k:<22} {v_r2:>8.3f} {v_o.mean():>12.3f} {v_o.std():>11.3f} "
              f"{v_o.min():>11.3f} {v_o.max():>11.3f}  ({note})")

    # ===== Correlation across all reps =====
    print()
    print("=" * 130)
    print("CORRELATIONS WITH xy_end (all reps)")
    print("=" * 130)
    xy = np.array([r["xy_end"] for r in rows])
    keys = ["vh0", "vz0", "sigma_xy0", "sigma0_mag",
            "sigma_xy_mean", "sigma_xy_max", "sigma_xy_tail", "sen_tail",
            "stale_pct_descent", "max_stale_run_frames", "switches",
            "side_max_d", "fps_mean", "fps_min", "flight_s"]
    cors = []
    for k in keys:
        v = np.array([r[k] for r in rows], dtype=float)
        mask = ~np.isnan(v)
        if mask.sum() < 5 or v[mask].std() == 0: continue
        rho = float(np.corrcoef(v[mask], xy[mask])[0, 1])
        cors.append((k, rho, v[mask].mean(), v[mask].std()))
    cors.sort(key=lambda x: -abs(x[1]))
    print(f"  {'metric':<22} {'ρ':>6}  {'mean':>10} {'std':>10}")
    for k, rho, m, s in cors:
        print(f"  {k:<22} {rho:>+.3f}  {m:>10.3f} {s:>10.3f}")


if __name__ == "__main__":
    main()
