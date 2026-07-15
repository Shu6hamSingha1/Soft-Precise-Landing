#!/usr/bin/env python3
"""Salvage diagnostic for the pre-checkpost-fix output_calibration.py
recordings: not everything in those runs is corrupted the same way.

FLOW_DH_MAX (lateral h_x/h_y) and FLOW_DS_MAX (centroid xc/yc) checkposts
freeze those 4 channels on any |delta| spike, which is most frame-to-frame
deltas on this noisy raw signal (confirmed 2026-07-10: 33-47% consecutive-
duplicate rate). But several other channels never pass through either
checkpost and are genuinely raw, continuously-varying data:
  - alpha (yaw feature, Feature Params[:,3]) - checkpost only touches [0:2]
  - the moment-based loom (Opt Flow Ang Vel[:,2], h_z) - REPLACED by a
    different estimator (FLOW_LOOM_DECOUPLE), not frozen, still live
  - Ring Opt Flow Ang Vel - a wholly separate signal, no checkpost applied

This script (1) measures per-channel freeze rate to confirm which channels
are actually corrupted, (2) for the corrupted lateral/centroid channels,
recovers a genuinely-raw SUBSET by keeping only the first sample of each
run of consecutive duplicates (a frozen run's first value was still a real
measurement, everything after it in that run is a stale repeat), and
(3) reports GT correlation per channel using whatever's actually usable -
so we know what (if anything) from the existing recording is trustworthy
ahead of a fresh checkpost-clean take.

Usage: python3 salvage_diagnose.py <run_dir>
"""
import sys
import numpy as np
from derive_pi_cal import compute_gt_flow, FILTER_WIN, POLYORDER
from scipy.signal import savgol_filter as sgf


def freeze_rate(x):
    dv = np.diff(x, axis=0)
    frozen = np.all(np.abs(dv) < 1e-9, axis=-1) if dv.ndim > 1 else np.abs(dv) < 1e-9
    return 100.0 * frozen.sum() / len(frozen)


def dedup_keep_first(t, x):
    """Keep only the first sample of each run of consecutive duplicates -
    recovers the genuinely-fresh subset from a checkpost-frozen channel."""
    x = np.asarray(x)
    if x.ndim == 1:
        x = x[:, None]
    keep = np.ones(len(x), dtype=bool)
    keep[1:] = np.any(np.abs(np.diff(x, axis=0)) > 1e-9, axis=1)
    return np.asarray(t)[keep], x[keep]


def corr(a, b):
    m = np.isfinite(a) & np.isfinite(b)
    if m.sum() < 10 or np.ptp(a[m]) < 1e-9 or np.ptp(b[m]) < 1e-9:
        return np.nan, int(m.sum())
    return float(np.corrcoef(a[m], b[m])[0, 1]), int(m.sum())


def main(run_dir):
    img = np.load(f"{run_dir}/Img_Data.npy", allow_pickle=True).item()
    g = compute_gt_flow(run_dir)

    t_img = np.asarray(img["Time"], float)
    raw_flow = np.asarray(img["Opt Flow Ang Vel"], float)
    raw_feat = np.asarray(img["Feature Params"], float)
    ring = np.asarray(img.get("Ring Opt Flow Ang Vel", []), float)
    n = min(len(t_img), len(raw_flow), len(raw_feat))
    t_img, raw_flow, raw_feat = t_img[:n], raw_flow[:n], raw_feat[:n]

    print(f"=== {run_dir} ===\n")
    print("Per-channel freeze rate (consecutive-identical %, confirms which channels are corrupted):")
    labels = ['h_x', 'h_y', 'h_z(moment loom)', 'w_x(unused)', 'w_y(unused)', 'w_z']
    for k, lab in enumerate(labels):
        print(f"  {lab:20s} {freeze_rate(raw_flow[:, k]):5.1f}%")
    print(f"  {'xc':20s} {freeze_rate(raw_feat[:,0]):5.1f}%")
    print(f"  {'yc':20s} {freeze_rate(raw_feat[:,1]):5.1f}%")
    print(f"  {'alpha':20s} {freeze_rate(raw_feat[:,3]):5.1f}%")
    if len(ring):
        print(f"  {'ring h_x':20s} {freeze_rate(ring[:,0]):5.1f}%")

    print("\n--- Channels NOT touched by any checkpost (should be trustworthy already) ---")

    # alpha vs GT yaw
    raw_feat_s = sgf(raw_feat, FILTER_WIN, POLYORDER, axis=0) if len(raw_feat) >= FILTER_WIN else raw_feat
    alpha_g = g["align"](t_img, raw_feat_s[:, 3])
    c, ns = corr(g["alpha"], alpha_g)
    print(f"  alpha (yaw feature) vs GT yaw:        corr={c:+.3f}  (n={ns})")

    # moment loom (h_z) vs GT loom - NOT frozen (different estimator, not held)
    raw_flow_s = sgf(raw_flow, FILTER_WIN, POLYORDER, axis=0) if len(raw_flow) >= FILTER_WIN else raw_flow
    hz_g = g["align"](t_img, raw_flow_s[:, 2])
    c, ns = corr(g["loom"], hz_g)
    print(f"  h_z (moment loom) vs GT loom:          corr={c:+.3f}  (n={ns})")

    # ring flow vs GT translational flow (ring h_x/h_y/h_z, if present)
    if len(ring):
        t_ring = t_img[:len(ring)]
        ring_s = sgf(ring, FILTER_WIN, POLYORDER, axis=0) if len(ring) >= FILTER_WIN else ring
        for k, lab in enumerate(['h_x', 'h_y', 'h_z']):
            ring_g = g["align"](t_ring, ring_s[:, k])
            c, ns = corr(g["V_h_g"][:, k], ring_g)
            print(f"  ring {lab:6s} vs GT V_h_g[{k}]:          corr={c:+.3f}  (n={ns})")

    print("\n--- Corrupted channels (h_x/h_y/xc/yc), dedup'd-first-sample subset only ---")
    for k, lab in zip([0, 1], ['h_x', 'h_y']):
        t_dd, x_dd = dedup_keep_first(t_img, raw_flow[:, k])
        gt_at_dd = g["align"](t_dd, x_dd[:, 0])
        # align() interpolates onto the GT's OWN clock, not t_dd - need the reverse:
        # interpolate GT onto t_dd instead, since we want per-sample-instant comparison
        ti_dd = t_dd - g["start_time"]
        gt_interp = np.interp(ti_dd, g["t_g"], g["V_h_g"][:, k])
        c, ns = corr(gt_interp, x_dd[:, 0])
        print(f"  {lab} (dedup, n_kept={len(t_dd)}/{n}):        corr={c:+.3f}  (n={ns})")

    for k, lab in zip([0, 1], ['xc', 'yc']):
        t_dd, x_dd = dedup_keep_first(t_img, raw_feat[:, k])
        ti_dd = t_dd - g["start_time"]
        gt_interp = np.interp(ti_dd, g["t_g"], g["V_s_g"][:, k])
        c, ns = corr(gt_interp, x_dd[:, 0])
        print(f"  {lab} (dedup, n_kept={len(t_dd)}/{n}):        corr={c:+.3f}  (n={ns})")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: salvage_diagnose.py <run_dir>")
        sys.exit(1)
    main(sys.argv[1])
