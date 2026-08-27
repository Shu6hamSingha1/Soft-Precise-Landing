"""Derive a NEAR-DIAGONAL cross-marker sensor cal from GT-feedback LANDING recordings.

WHY NOT derive_cross_marker_cal.py (the phased-excitation 6x6 fit):
  At 320x240 the phased x/y position sinusoid barely moves the drone -- PX4's
  horizontal position loop tracks only ~15% of a 0.5Hz lateral setpoint, so the
  achieved GT h_x/h_y std is ~0.007 (vs ~0.008 raw-noise floor). The Hx/Hy rows
  of the joint 6x6 lstsq then fit from ~4x noise and blow up EVERY row of M
  (validated: all per-axis R^2 came out negative). `compute_gt_signals` itself
  is correct -- the UAV-pose log confirms only ~6cm of travel during an x-phase.
  See Memory/px4/project_20260827_framerate_and_h_texture_investigation.md
  (2026-08-28 recal section).

WHAT THIS DOES INSTEAD:
  A GT-feedback off-center landing produces real, sustained lateral optical flow
  (GT h_x/h_y std ~0.06-0.08). On such a rep, at normal marker size
  (MARKER_EXTENT_PX < EXTENT_MAX), raw h_V vs GT h is a clean per-axis line
  through ~origin (measured: GT h_x ~= 0.74*raw, h_y ~= 0.66*raw, h_z ~= 0.86*raw;
  r = 0.976 / 0.979 / 0.993 -- NO cross-coupling). So the cal the current pipeline
  actually needs is a per-axis diagonal scale, derivable from landings.

  h-block  : diag(s_hx, s_hy, s_hz)   robust (MAD-trimmed) slope, forced through 0
  Wx / Wy  : 0                        (level-target convention, as the 6x6 cal does)
  Wz       : s_wz                     robust slope raw w_z -> GT w_z if the rep has
                                      enough yaw motion, else carried from --wz-fallback
  _sensor_cal_s : diag(s_sx, s_sy, 1, 1)  centroid slope from the same landings

Leave-one-out cross-validated: each run's R^2 is reported using a cal fit on the
OTHER runs only.

USAGE
  python3 tools/derive_cross_marker_landing_cal.py [run_dir ...]
    (default: every subdir of calibration_data/landing_cal_cross/ with Control_Data.npy)
  env: CROSS_LANDCAL_EXTENT_MAX (200)  CROSS_LANDCAL_WZ_FALLBACK (0.52)
       CROSS_LANDCAL_MIN_WZ_STD (0.02)  -- below this GT-w_z std, use the fallback
"""
import sys, os, glob
import numpy as np

EXTENT_MAX = float(os.environ.get("CROSS_LANDCAL_EXTENT_MAX", "200"))
WZ_FALLBACK = float(os.environ.get("CROSS_LANDCAL_WZ_FALLBACK", "0.52"))
MIN_WZ_STD = float(os.environ.get("CROSS_LANDCAL_MIN_WZ_STD", "0.02"))
DEFAULT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "..", "calibration_data", "landing_cal_cross")


def _load(d):
    gt = np.load(os.path.join(d, "Ground_Truth.npy"), allow_pickle=True).item()
    cd = np.load(os.path.join(d, "Control_Data.npy"), allow_pickle=True).item()
    img = np.load(os.path.join(d, "Img_Data.npy"), allow_pickle=True).item()
    start = float(np.asarray(gt["Start Time"]))
    it = np.asarray(img["Time"], float)
    hraw = np.asarray(img["h_V"], float)              # (N,6) raw pre-cal [Tx,Ty,Tz,Wx,Wy,Wz]
    sraw = np.asarray(img["s_V"], float)              # (N,3) raw [xc,yc,1]
    ext = np.asarray(img["MARKER_EXTENT_PX"], float)
    ct = np.asarray(cd["t"], float)
    gh = np.asarray([np.asarray(r, float)[:3] for r in cd["h(t)"]])     # GT h (GT-FB)
    gw = np.asarray([np.asarray(r, float)[:3] for r in cd["w(t)"]])     # GT w (yaw-only-ish)
    gs = np.asarray([np.asarray(r, float)[:3] for r in cd["s(t)"]]) if "s(t)" in cd else None
    m = it >= start
    G_h = np.column_stack([np.interp(it[m], ct, gh[:, k]) for k in range(3)])
    G_w = np.column_stack([np.interp(it[m], ct, gw[:, k]) for k in range(3)])
    G_s = (np.column_stack([np.interp(it[m], ct, gs[:, k]) for k in range(2)])
           if gs is not None else None)
    Hr, Sr, E = hraw[m], sraw[m], ext[m]
    keep = (~np.all(Hr == 0, axis=1)) & (E < EXTENT_MAX) & np.isfinite(G_h).all(1)
    return dict(name=os.path.basename(d.rstrip("/")),
                Hr=Hr[keep], Sr=Sr[keep], Gh=G_h[keep], Gw=G_w[keep],
                Gs=(G_s[keep] if G_s is not None else None), n=int(keep.sum()))


def _robust_slope(raw, gt, k_mad=3.0):
    """Least-squares slope through the origin (gt = s*raw), MAD-trimmed on the
    residual of a first pass. Returns (slope, r, n_used)."""
    raw = np.asarray(raw, float); gt = np.asarray(gt, float)
    good = np.isfinite(raw) & np.isfinite(gt)
    raw, gt = raw[good], gt[good]
    if len(raw) < 20 or raw.std() < 1e-9:
        return np.nan, np.nan, len(raw)
    s0 = float(raw @ gt / (raw @ raw))
    resid = gt - s0 * raw
    mad = np.median(np.abs(resid - np.median(resid))) + 1e-12
    ok = np.abs(resid - np.median(resid)) < k_mad * 1.4826 * mad
    s = float(raw[ok] @ gt[ok] / (raw[ok] @ raw[ok]))
    r = float(np.corrcoef(raw[ok], gt[ok])[0, 1])
    return s, r, int(ok.sum())


def _fit(runs):
    """Fit diag scales from a list of loaded runs (pooled)."""
    Hr = np.vstack([r["Hr"] for r in runs]); Gh = np.vstack([r["Gh"] for r in runs])
    Sr = np.vstack([r["Sr"] for r in runs])
    Gs = np.vstack([r["Gs"] for r in runs]) if runs[0]["Gs"] is not None else None
    Gw = np.vstack([r["Gw"] for r in runs])
    s_h = [_robust_slope(Hr[:, k], Gh[:, k])[0] for k in range(3)]
    # Wz: raw w_z is h_V col 5; GT w_z is Gw col 2
    wz_gt_std = float(np.nanstd(Gw[:, 2]))
    if wz_gt_std >= MIN_WZ_STD:
        s_wz, _, _ = _robust_slope(Hr[:, 5], Gw[:, 2])
    else:
        s_wz = WZ_FALLBACK
    s_s = ([_robust_slope(Sr[:, k], Gs[:, k])[0] for k in range(2)]
           if Gs is not None else [1.0, 1.0])
    return dict(s_h=s_h, s_wz=s_wz, s_s=s_s, wz_gt_std=wz_gt_std)


def _r2(cal, run):
    """R^2 of the calibrated prediction vs GT for one held-out run."""
    Hc = run["Hr"].copy()
    pred_h = np.column_stack([cal["s_h"][k] * run["Hr"][:, k] for k in range(3)])
    out = {}
    for k, lab in enumerate(["h_x", "h_y", "h_z"]):
        g, p = run["Gh"][:, k], pred_h[:, k]
        ss_res = np.nansum((g - p) ** 2); ss_tot = np.nansum((g - np.nanmean(g)) ** 2)
        out[lab] = 1.0 - ss_res / ss_tot if ss_tot > 1e-12 else np.nan
    return out


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("-")]
    if args:
        dirs = args
    else:
        dirs = sorted(d for d in glob.glob(os.path.join(DEFAULT_DIR, "*"))
                      if os.path.isdir(d) and os.path.exists(os.path.join(d, "Control_Data.npy")))
    if len(dirs) < 2:
        raise SystemExit(f"need >=2 landing runs (found {len(dirs)} in {DEFAULT_DIR})")
    runs = [_load(d) for d in dirs]
    for r in runs:
        print(f"  {r['name']:24s}  n={r['n']:5d} usable samples (extent<{EXTENT_MAX:.0f})")

    full = _fit(runs)
    print(f"\npooled fit ({len(runs)} runs):")
    for k, lab in enumerate(["s_hx", "s_hy", "s_hz"]):
        s, rr, n = _robust_slope(np.vstack([r["Hr"] for r in runs])[:, k],
                                 np.vstack([r["Gh"] for r in runs])[:, k])
        print(f"  {lab} = {s:+.4f}   (r={rr:+.3f}, n={n})")
    print(f"  s_wz = {full['s_wz']:+.4f}   (GT w_z std={full['wz_gt_std']:.4f}"
          f"{' -> FALLBACK' if full['wz_gt_std'] < MIN_WZ_STD else ''})")
    print(f"  s_sx = {full['s_s'][0]:+.4f}   s_sy = {full['s_s'][1]:+.4f}")

    print("\nleave-one-out R^2 (cal fit on the OTHER runs):")
    for i, held in enumerate(runs):
        cal = _fit([r for j, r in enumerate(runs) if j != i])
        r2 = _r2(cal, held)
        print(f"  {held['name']:24s}  h_x={r2['h_x']:+.3f}  h_y={r2['h_y']:+.3f}  h_z={r2['h_z']:+.3f}")

    sh = full["s_h"]; swz = full["s_wz"]; ss = full["s_s"]
    print("\n--- paste into CrossMarkerPerception.__init__ (src/cross_marker_perception.py) ---")
    print("        # 2026-08-28: near-diagonal cal from GT-FB landing recordings")
    print("        # (tools/derive_cross_marker_landing_cal.py) -- phased x/y excitation")
    print("        # is untrackable by PX4 at 320x240, landings give real lateral flow.")
    print("        self._sensor_cal_hw = np.array([")
    print(f"            [{sh[0]:+.4f}, +0.0000, +0.0000, +0.0000, +0.0000, +0.0000],")
    print(f"            [+0.0000, {sh[1]:+.4f}, +0.0000, +0.0000, +0.0000, +0.0000],")
    print(f"            [+0.0000, +0.0000, {sh[2]:+.4f}, +0.0000, +0.0000, +0.0000],")
    print(f"            [+0.0000, +0.0000, +0.0000, +0.0000, +0.0000, +0.0000],")
    print(f"            [+0.0000, +0.0000, +0.0000, +0.0000, +0.0000, +0.0000],")
    print(f"            [+0.0000, +0.0000, +0.0000, +0.0000, +0.0000, {swz:+.4f}]])")
    print(f"        self._sensor_cal_s = np.diag([{ss[0]:.4f}, {ss[1]:.4f}, 1.0, 1.0])")


if __name__ == "__main__":
    main()
