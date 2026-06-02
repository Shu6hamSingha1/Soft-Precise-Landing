#!/usr/bin/env python3
"""Analyze the dual-marker switch (big ID 10 → small ID 0) during descent.

Looks at:
  1. Marker size in pixels over time (per-frame corner-spacing → marker side)
  2. The altitude at which the switch occurs
  3. Centroid noise before vs after switch
  4. Whether back-and-forth switching happens in a "transition zone"
  5. Correlation with σ_xy divergence (cross-reference Control_Data)
"""
from __future__ import annotations
import glob
import os
import sys

import numpy as np


def _to_array(seq):
    return np.asarray([np.asarray(x) for x in seq])


def analyze_rep(rep_dir):
    img = np.load(os.path.join(rep_dir, "Img_Data.npy"), allow_pickle=True).item()
    cd  = np.load(os.path.join(rep_dir, "Control_Data.npy"), allow_pickle=True).item()
    gt  = np.load(os.path.join(rep_dir, "Ground_Truth.npy"), allow_pickle=True).item()

    feat_pts = _to_array(img["Image Feature Pts"])   # (N, 2, 4, 2)
    t_img    = np.asarray(img["Time"])
    sigma    = _to_array(cd["sigma(t)"])
    t_ctrl   = np.asarray(cd["t"])

    # The detected marker's corner points are at fp[:, 0, :, :].
    # Marker side in pixels ≈ mean(|corner[i] - corner[(i+1)%4]|).
    mc = feat_pts[:, 0, :, :]                       # (N, 4, 2)
    sides = np.linalg.norm(np.diff(np.concatenate(
        [mc, mc[:, :1, :]], axis=1), axis=1), axis=2)  # (N, 4)
    side_mean = sides.mean(axis=1)                  # (N,) marker side in pixels

    # UAV altitude over time (from ground truth)
    uav_poses = gt["UAV Pose"]
    target_pose = gt["Target Pose"]
    n_pose = min(len(uav_poses), len(target_pose))
    uav_z = np.array([uav_poses[i].position.z for i in range(n_pose)])
    tgt_z = np.array([target_pose[i].position.z for i in range(n_pose)])
    alt = uav_z[:n_pose] - tgt_z[:n_pose]
    t_pose = np.asarray(gt["Time"][:n_pose])

    # Heuristic to classify which marker (big=ID 10 ~34cm, small=ID 0 ~6cm).
    # Big-marker side at z=2m: 0.34*270/2 = 46 px;  at z=5m: 18 px.
    # Small-marker side at z=2m: 0.06*270/2 = 8 px; at z=1m: 16 px; at z=0.5m: 32 px.
    # Predicted big-marker pixel side from altitude: 0.34 * f / z
    # If actual side ≈ big-prediction → big in use; if ≈ small-prediction → small.
    f_px = 270.0
    big_side_m, small_side_m = 0.34, 0.06
    # Interpolate altitude onto Img_Data time
    if len(t_pose) > 1 and t_pose[-1] >= t_img[0]:
        alt_at_img = np.interp(t_img, t_pose, alt)
    else:
        alt_at_img = np.full_like(side_mean, np.nan)

    pred_big_px   = big_side_m   * f_px / np.maximum(np.abs(alt_at_img), 0.05)
    pred_small_px = small_side_m * f_px / np.maximum(np.abs(alt_at_img), 0.05)
    # Classification: closer to big or small prediction (log ratio)?
    eps = 1e-6
    ratio_big   = side_mean / (pred_big_px + eps)
    ratio_small = side_mean / (pred_small_px + eps)
    # If side ≈ big prediction → ratio_big ~ 1, ratio_small ~ 5.7
    # If side ≈ small prediction → ratio_big ~ 0.17, ratio_small ~ 1
    using_small = np.abs(np.log(ratio_small)) < np.abs(np.log(ratio_big))
    big_fraction = float((~using_small).sum() / len(using_small))
    small_fraction = float(using_small.sum() / len(using_small))

    # Find the transition: first sample where using_small is True
    if using_small.any():
        first_small_idx = int(np.argmax(using_small))
        switch_alt = float(abs(alt_at_img[first_small_idx])) if first_small_idx < len(alt_at_img) else np.nan
        switch_time = float(t_img[first_small_idx])
    else:
        switch_alt = np.nan
        switch_time = np.nan

    # Count flip-flops (switches per second)
    transitions = int((np.diff(using_small.astype(int)) != 0).sum())

    # Centroid jitter before / after switch (frame-to-frame Δ)
    centroid = mc.mean(axis=1)               # (N, 2)
    diff = np.diff(centroid, axis=0)
    jitter = np.linalg.norm(diff, axis=1)
    # Window before/after switch
    if not np.isnan(switch_time) and first_small_idx > 20 and first_small_idx < len(jitter) - 20:
        pre  = jitter[max(0, first_small_idx - 50): first_small_idx]
        post = jitter[first_small_idx: first_small_idx + 50]
        pre_jitter  = float(pre.mean())
        post_jitter = float(post.mean())
        jitter_ratio = post_jitter / max(pre_jitter, 1e-6)
    else:
        pre_jitter = post_jitter = jitter_ratio = float("nan")

    # σ_xy excursions
    sigma_xy = np.linalg.norm(sigma[:, :2], axis=1) if sigma.shape[1] == 3 else \
               np.linalg.norm(_to_array(cd["sigma(t)"])[:, :2], axis=1)

    return {
        "rep":             os.path.basename(rep_dir),
        "xy_end":          float(gt["SoftPrecise"]["xy_err"]),
        "target_lost":     bool(gt["SoftPrecise"].get("target_lost", False)),
        "n_img":           len(side_mean),
        "side_mean_mean":  float(side_mean.mean()),
        "side_mean_min":   float(side_mean.min()),
        "side_mean_max":   float(side_mean.max()),
        "switch_alt_m":    switch_alt,
        "switch_time_s":   switch_time,
        "small_fraction":  small_fraction,
        "transitions":     transitions,
        "pre_switch_jitter_px":  pre_jitter,
        "post_switch_jitter_px": post_jitter,
        "jitter_ratio":          jitter_ratio,
        "alt_first":     float(abs(alt_at_img[0])) if not np.isnan(alt_at_img[0]) else np.nan,
        "alt_last":      float(abs(alt_at_img[-1])) if not np.isnan(alt_at_img[-1]) else np.nan,
        "side_first":    float(side_mean[0]),
        "side_last":     float(side_mean[-1]),
    }


def main():
    cands = sorted(glob.glob(os.path.expanduser(
        "~/Soft-Precise-Landing/PX4_Gazebo/test_data/DefaultN10/*")))
    if not cands:
        sys.exit("No DefaultN10 bundle.")
    bundle = cands[-1]
    rep_dirs = sorted([d for d in glob.glob(f"{bundle}/rep*")
                       if os.path.isdir(d)])
    print(f"Bundle: {bundle}  ({len(rep_dirs)} reps)\n")
    print(f"Setup: f={270} px,  big marker side ≈ 34 cm,  small marker side ≈ 6 cm")
    print(f"Predicted big-marker pixels: z=5m → 18 px,  z=2m → 46 px,  z=0.5m → 184 px")
    print(f"Predicted small-marker pixels: z=2m → 8 px (sub-marginal),  z=1m → 16 px,  z=0.5m → 32 px")
    print()
    print("=" * 105)
    print(f"{'rep':<6} {'xy_end':>7} {'alt_start':>9} {'alt_end':>8} "
          f"{'side[0]':>8} {'side[-1]':>9} {'small_%':>7} {'switch_alt':>10} "
          f"{'flips':>5}  {'pre_jit':>7} {'post_jit':>8} {'jit_ratio':>9}")
    print("=" * 105)
    rows = []
    for d in rep_dirs:
        try:
            r = analyze_rep(d)
            rows.append(r)
            print(f"  {r['rep']:<6} {r['xy_end']:>7.3f} "
                  f"{r['alt_first']:>9.2f} {r['alt_last']:>8.2f} "
                  f"{r['side_first']:>8.1f} {r['side_last']:>9.1f} "
                  f"{100*r['small_fraction']:>6.1f}% "
                  f"{r['switch_alt_m']:>10.2f} "
                  f"{r['transitions']:>5} "
                  f"{r['pre_switch_jitter_px']:>7.2f} "
                  f"{r['post_switch_jitter_px']:>8.2f} "
                  f"{r['jitter_ratio']:>9.2f}")
        except Exception as e:
            print(f"  {os.path.basename(d):<6}  ERR: {e}")

    # Correlations
    print()
    print("=" * 105)
    print("CORRELATIONS WITH xy_end")
    print("=" * 105)
    landed = [r for r in rows if not r["target_lost"]]
    if len(landed) < 3:
        print("  too few non-TL reps"); return
    xy = np.array([r["xy_end"] for r in landed])
    for k in ("small_fraction", "transitions", "jitter_ratio", "switch_alt_m"):
        v = np.array([r[k] for r in landed], dtype=float)
        mask = ~np.isnan(v)
        if mask.sum() < 3 or v[mask].std() == 0: continue
        rho = float(np.corrcoef(v[mask], xy[mask])[0, 1])
        print(f"  ρ({k:<20}, xy_end) = {rho:+.3f}   mean={v[mask].mean():.3f}")


if __name__ == "__main__":
    main()
