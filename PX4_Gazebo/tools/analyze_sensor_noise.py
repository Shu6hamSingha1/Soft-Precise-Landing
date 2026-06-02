#!/usr/bin/env python3
"""
Phase 4: sensor-noise floor characterization for the PX4 SITL image pipeline.

Goal: measure actual centroid/optic-flow noise at PX4 IC1 (5 m altitude),
compare to the MATLAB depth-dependent pixel-noise model used in Phase 1
(sigma_px(z) = 0.3 + 0.5/(z+0.5) [px]).

If PX4 sensor noise is comparable to MATLAB (~0.39 px @ z=5m), sensor noise
is NOT the dominant 16× gap source (Phase 2 lag is). If PX4 noise is much
larger, sensor noise contributes alongside lag.

Approach:
- Img_Data records continuously, ~86 fps × ~30 s = ~2600 samples per rep
- Control_Data starts only when the controller engages
- Therefore: Img_Data samples BEFORE Control_Data['t'][0] are the pre-
  engagement hover period at IC altitude (drone holding at 5 m)
- Measure centroid std (px) over that quiescent window — that's σ_centroid
- Measure optic-flow std over the same window — that's σ_h
- Compare against MATLAB's σ_px(5m) = 0.39 px

Usage:
    python3 analyze_sensor_noise.py [bundle_dir]
"""
from __future__ import annotations
import argparse
import glob
import os
import sys
from pathlib import Path

import numpy as np


def _to_array(seq):
    return np.asarray([np.asarray(x) for x in seq])


def load_rep(rep_dir: Path) -> dict | None:
    out = {"dir": rep_dir}
    for fname in ["Control_Data.npy", "Img_Data.npy", "Telemetry_Data.npy"]:
        path = rep_dir / fname
        if not path.exists():
            return None
        d = np.load(path, allow_pickle=True)
        if d.dtype == object and d.shape == ():
            d = d.item()
        out[fname[:-4]] = d
    return out


def hover_window(rep):
    """Find the pre-engagement hover window in image-data time."""
    cd = rep["Control_Data"]
    img = rep["Img_Data"]
    t_img = np.asarray(img["Time"], dtype=float) if "Time" in img else None
    if t_img is None or len(t_img) == 0:
        return None
    t_ctrl_start = float(np.asarray(cd["t"], dtype=float)[0])
    # Hover = image samples taken at least 0.5 s before controller engaged,
    # and within 3 s of that point (drone is settled, hasn't moved much).
    hover_end = t_ctrl_start - 0.5
    hover_start = max(t_img[0], hover_end - 3.0)
    mask = (t_img >= hover_start) & (t_img < hover_end)
    if mask.sum() < 30:
        return None
    return mask, t_img, hover_start, hover_end


def centroid_pixel_std(rep, hover_mask):
    """Compute centroid σ in pixel space from the 4 ArUco corner points.
    Centroid = mean of 4 corners; std across the hover window is the
    measurement noise σ_centroid."""
    img = rep["Img_Data"]
    feat_pts = img.get("Image Feature Pts", [])
    if not feat_pts:
        return None
    # Each entry is (2, 4, 2) — likely (which-set, corner, xy)
    # img_data.py logs Image Feature Pts as detected ArUco corners; first
    # axis is probably [measured, virtual]. Centroid = mean of 4 measured
    # corner pixels.
    fp = _to_array(feat_pts)               # (N, 2, 4, 2)
    if fp.ndim != 4 or fp.shape[1:] != (2, 4, 2):
        # Try other shapes; bail if unexpected
        return None
    fp = fp[hover_mask]                    # (n_hover, 2, 4, 2)
    # Centroid per frame (measured set = first index along axis 1)
    cen = fp[:, 0, :, :].mean(axis=1)      # (n_hover, 2)
    # Method A: absolute std around mean — contaminated by drone motion if
    # the drone isn't truly stationary
    cen_detrend = cen - cen.mean(axis=0, keepdims=True)
    sigma_px = cen_detrend.std(axis=0)
    # Method B: frame-to-frame delta std / sqrt(2) — isolates high-frequency
    # noise from low-frequency drone motion.  If drone moves smoothly,
    # delta std is small; if pixel noise is large, delta std is large.
    diff = np.diff(cen, axis=0)            # (n_hover-1, 2)
    sigma_diff = diff.std(axis=0) / np.sqrt(2)
    return {
        "n":               int(hover_mask.sum()),
        "cen_mean_px":     cen.mean(axis=0).tolist(),
        "sigma_x_px":      float(sigma_px[0]),
        "sigma_y_px":      float(sigma_px[1]),
        "sigma_mag_px":    float(np.linalg.norm(sigma_px)),
        # High-frequency-isolated estimate (motion-contamination-free)
        "sigma_x_hf_px":   float(sigma_diff[0]),
        "sigma_y_hf_px":   float(sigma_diff[1]),
        "sigma_mag_hf_px": float(np.linalg.norm(sigma_diff)),
    }


def optic_flow_std(rep, hover_mask):
    img = rep["Img_Data"]
    flow_raw = _to_array(img["Opt Flow Ang Vel"])
    if flow_raw.ndim != 2 or flow_raw.shape[1] != 6:
        return None
    f = flow_raw[hover_mask]
    sigma = f.std(axis=0)
    return {
        "raw_sigma_xyz_trans": sigma[:3].tolist(),
        "raw_sigma_xyz_rot":   sigma[3:].tolist(),
        "raw_sigma_mag":       float(np.linalg.norm(sigma)),
    }


def velocity_during_hover(rep, t_img, hover_mask):
    """Cross-reference: how stationary is the drone during the hover window?
    If the drone is actually drifting, centroid std reflects motion, not noise."""
    td = rep["Telemetry_Data"]
    vels = td.get("Velocity Body", [])
    if not vels:
        return None
    v = np.array([[vv.x_m_s, vv.y_m_s, vv.z_m_s] for vv in vels])
    # Telemetry uses controller t base?  No — recorded from the start, like Img.
    # Use uniform sampling assumption over the same time range as Img_Data.
    n_v = len(v)
    t_vel = np.linspace(t_img[0], t_img[-1], n_v)
    hover_t = t_img[hover_mask]
    # Find vel samples within hover window
    mask_v = (t_vel >= hover_t[0]) & (t_vel <= hover_t[-1])
    if mask_v.sum() < 5:
        return None
    v_h = v[mask_v]
    return {
        "vh_mean":   float(np.hypot(v_h[:, 0], v_h[:, 1]).mean()),
        "vh_max":    float(np.hypot(v_h[:, 0], v_h[:, 1]).max()),
        "vz_mean":   float(v_h[:, 2].mean()),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bundle", nargs="?", default=None)
    args = ap.parse_args()

    if args.bundle is None:
        candidates = sorted(glob.glob(os.path.expanduser(
            "~/Soft-Precise-Landing/PX4_Gazebo/test_data/DefaultN10/*")))
        if not candidates:
            sys.exit("No DefaultN10 bundle found.")
        args.bundle = candidates[-1]

    bundle = Path(args.bundle)
    rep_dirs = sorted([d for d in bundle.iterdir()
                       if d.is_dir() and (d / "Img_Data.npy").exists()])
    print(f"Bundle: {bundle}  ({len(rep_dirs)} reps)\n")

    f_px = 270.0   # focal length (px), per CLAUDE.md
    z_ic = 5.0     # IC altitude (m)
    # MATLAB depth-dependent noise model used in Phase 1
    matlab_sigma_px = 0.3 + 0.5 / (z_ic + 0.5)   # ≈ 0.39 px @ z=5
    print(f"MATLAB noise model at z={z_ic}m:  σ_px = {matlab_sigma_px:.3f} px")
    print(f"                                  σ_lat = σ_px × z / f = "
          f"{matlab_sigma_px * z_ic / f_px * 1000:.2f} mm\n")

    print("=" * 96)
    print("PRE-ENGAGEMENT HOVER WINDOW: centroid σ in pixels")
    print("=" * 96)
    print(f"  Method A: absolute std around mean (motion-contaminated)")
    print(f"  Method B: frame-to-frame Δ / √2 (high-pass, isolates true sensor noise)")
    print()
    print(f"{'rep':<8} {'n_samp':>6}  {'σ_abs':>7} {'σ_hf':>6}  "
          f"{'vh_mean':>8} {'vh_max':>7}  {'σ_abs_lat_mm@5m':>15} {'σ_hf_lat_mm@5m':>15}")
    px_sigmas = []
    hf_sigmas = []
    for d in rep_dirs:
        rep = load_rep(d)
        if rep is None: continue
        h = hover_window(rep)
        if h is None:
            print(f"  {d.name:<8}  (no hover window)")
            continue
        mask, t_img, hs, he = h
        cs = centroid_pixel_std(rep, mask)
        vh = velocity_during_hover(rep, t_img, mask)
        if cs is None: continue
        vh_mean = vh["vh_mean"] if vh else float("nan")
        vh_max  = vh["vh_max"]  if vh else float("nan")
        sigma_abs_lat = cs["sigma_mag_px"] * z_ic / f_px * 1000
        sigma_hf_lat  = cs["sigma_mag_hf_px"] * z_ic / f_px * 1000
        print(f"  {d.name:<8} {cs['n']:>6}  "
              f"{cs['sigma_mag_px']:>7.3f} {cs['sigma_mag_hf_px']:>6.3f}  "
              f"{vh_mean:>8.3f} {vh_max:>7.3f}  "
              f"{sigma_abs_lat:>15.2f} {sigma_hf_lat:>15.2f}")
        px_sigmas.append(cs["sigma_mag_px"])
        hf_sigmas.append(cs["sigma_mag_hf_px"])

    if hf_sigmas:
        arr = np.array(hf_sigmas)
        print()
        print(f"  Median σ_hf (high-pass) across reps: {np.median(arr):.3f} px")
        print(f"  Mean σ_hf:                            {arr.mean():.3f} px (std {arr.std():.3f})")
        print(f"  Ratio vs MATLAB σ_px (z=5m):          {np.median(arr) / matlab_sigma_px:.2f}×")

    # Project HF (motion-isolated) σ to lateral position error
    print()
    print("=" * 96)
    print("TRANSLATING HF (TRUE-SENSOR) σ TO LATERAL POSITION ERROR")
    print("=" * 96)
    if hf_sigmas:
        med = np.median(hf_sigmas)
        for z in (5.0, 2.0, 1.0, 0.5, 0.2):
            sigma_lat = med * z / f_px * 1000
            print(f"  At z={z:>4.1f}m:  σ_lat = {med:.2f} px × {z:.2f} / {f_px:.0f} = {sigma_lat:.2f} mm")

    # Optic-flow noise
    print()
    print("=" * 96)
    print("OPTIC-FLOW NOISE DURING HOVER (raw, pre-savgol)")
    print("=" * 96)
    print(f"{'rep':<8} {'σ_trans_xyz (m/s)':<24} {'σ_rot_xyz (rad/s)':<24}")
    for d in rep_dirs:
        rep = load_rep(d)
        if rep is None: continue
        h = hover_window(rep)
        if h is None: continue
        mask, _, _, _ = h
        of = optic_flow_std(rep, mask)
        if of is None: continue
        st = [f"{s:.3f}" for s in of["raw_sigma_xyz_trans"]]
        sr = [f"{s:.3f}" for s in of["raw_sigma_xyz_rot"]]
        print(f"  {d.name:<8} [{','.join(st)}]  [{','.join(sr)}]")

    # Verdict
    print()
    print("=" * 96)
    print("VERDICT")
    print("=" * 96)
    if hf_sigmas and px_sigmas:
        ratio_hf  = np.median(hf_sigmas)  / matlab_sigma_px
        ratio_abs = np.median(px_sigmas) / matlab_sigma_px
        print(f"  PX4 σ_abs / MATLAB σ_px:  {ratio_abs:.1f}×  (motion-contaminated)")
        print(f"  PX4 σ_hf  / MATLAB σ_px:  {ratio_hf:.1f}×   (true sensor noise)")
        print(f"  Phase 2 found rate-loop lag at ~13× MATLAB")
        print()
        if ratio_hf > 5:
            print(f"  → Sensor noise is a SIGNIFICANT contributor to the 16× gap")
            print(f"     (alongside the 13× loop lag identified in Phase 2)")
        elif ratio_hf > 1.5:
            print(f"  → Sensor noise is a MINOR contributor; loop lag dominates")
        else:
            print(f"  → Sensor noise is essentially matched between MATLAB and PX4.")
            print(f"     The 16× gap is dominated by Phase 2's loop-lag finding.")


if __name__ == "__main__":
    main()
