#!/usr/bin/env python3
"""Derive CROSS_ALPHA_0 (cross_marker_perception.py's self._alpha_0) from phased
cross-marker calibration flights.

alpha_0 is the constant reference-heading OFFSET subtracted from the raw disambiguated
2nd-moment principal angle:  alpha = wrap(a_disamb - alpha_0)  (-> s[3], the yaw feature).
The theoretical relationship is slope EXACTLY -1 (Jabbari Asl, Yoon & Tosunoglu 2014,
eq. 21-22:  alpha_dot = -psi_dot  for this same plain principal-angle formula in the
leveled/virtual image plane), so only the offset is fit, not a scale.

Mirrors the 2026-08-08 derivation (commit aad0f57b):
  * force slope = -1,  a_raw ~= -psi_rel + alpha_0   =>   alpha_0 = circ_mean(a_raw + psi_rel)
  * per-run offset + circular mean + inter-run std + R^2 of the slope=-1 model
  * unconstrained 2-param (slope, offset) fit logged as a cross-check only (unstable on
    this excitation profile -- long near-zero-yaw stretches leave the slope ill-posed).

USAGE
  # recordings MUST be flown with CROSS_ALPHA_0=0 so Img_Data 'alpha(t)' logs the RAW
  # pre-offset angle.  If a run was flown with a non-zero offset, pass it via
  # CROSS_ALPHA0_USED (deg) and it is added back.
  CROSS_ALPHA0_CAL_DIR=calibration_data/output_cross_alpha0_20260831 \
      ~/ws/scripts/env2025/bin/python3 tools/derive_cross_alpha0.py
"""
import glob
import os
import sys

import numpy as np
from ahrs import Quaternion

CAL_DIR = os.environ.get("CROSS_ALPHA0_CAL_DIR", "calibration_data/output_cross")
# If the recordings were flown with a non-zero CROSS_ALPHA_0, add it back to recover
# the raw pre-offset angle. 0.0 == flown with CROSS_ALPHA_0=0 (the correct procedure).
ALPHA0_USED = np.radians(float(os.environ.get("CROSS_ALPHA0_USED", "0.0")))
YAW_RATE_MIN = float(os.environ.get("CROSS_ALPHA0_YAWRATE_MIN", "0.03"))   # rad/s; keep yaw-excited samples
MIN_SAMPLES = int(os.environ.get("CROSS_ALPHA0_MIN_SAMPLES", "40"))


def _yaw_of(q):
    return Quaternion([q.w, q.x, q.y, q.z]).to_angles()[2]


def _circ_mean(a):
    return float(np.arctan2(np.nanmean(np.sin(a)), np.nanmean(np.cos(a))))


def _circ_std(a):
    R = np.hypot(np.nanmean(np.sin(a)), np.nanmean(np.cos(a)))
    return float(np.sqrt(-2.0 * np.log(max(R, 1e-12))))


def _load_run(d):
    im = np.load(os.path.join(d, "Img_Data.npy"), allow_pickle=True).item()
    gt = np.load(os.path.join(d, "Ground_Truth.npy"), allow_pickle=True).item()
    St = float(gt["Start Time"])
    t_im = np.asarray(im["Time"], float) - St
    a_log = np.asarray(im["alpha(t)"], float)
    ds = np.array([str(x) for x in im["Detection Status"]])
    ok = ds == "ok"
    # a held alpha (stub missed) repeats the previous value exactly -- drop those,
    # they are not fresh measurements.
    held = np.zeros(len(a_log), bool)
    held[1:] = a_log[1:] == a_log[:-1]
    ok &= ~held

    tg = np.asarray(gt["Time"], float)
    u, tp = gt["UAV Pose"], gt["Target Pose"]
    n = min(len(tg), len(u), len(tp))
    tg, u, tp = tg[:n], u[:n], tp[:n]
    # de-dup non-increasing GT timestamps (bridge jitter) -- np.gradient/np.interp need
    # a strictly increasing x (same guard as gt_optical_flow.py).
    keep = np.hstack(([True], np.diff(tg) > 1e-6))
    tg = tg[keep]; u = [u[i] for i in range(n) if keep[i]]; tp = [tp[i] for i in range(n) if keep[i]]
    n = len(tg)
    psi = np.array([np.arctan2(np.sin(_yaw_of(u[i].orientation) - _yaw_of(tp[i].orientation)),
                               np.cos(_yaw_of(u[i].orientation) - _yaw_of(tp[i].orientation)))
                    for i in range(n)])
    psi_u = np.unwrap(psi)                             # unwrap for clean interp + rate
    psi_im = np.interp(t_im, tg, psi_u, left=np.nan, right=np.nan)
    yr_im = np.interp(t_im, tg, np.gradient(psi_u, tg), left=np.nan, right=np.nan)

    base = ok & np.isfinite(a_log) & np.isfinite(psi_im)
    a_raw = a_log[base] + ALPHA0_USED                  # recover pre-offset angle
    psi_rel = np.arctan2(np.sin(psi_im[base]), np.cos(psi_im[base]))
    excited = np.abs(yr_im[base]) >= YAW_RATE_MIN      # subset used for the R^2 model check
    return a_raw, psi_rel, excited, int(base.sum()), int(ok.sum())


def main():
    runs = sorted(d for d in glob.glob(os.path.join(CAL_DIR, "*"))
                  if os.path.isfile(os.path.join(d, "Img_Data.npy"))
                  and os.path.isfile(os.path.join(d, "Ground_Truth.npy")))
    if not runs:
        sys.exit(f"no recordings with Img_Data.npy + Ground_Truth.npy under {CAL_DIR}")
    print(f"CAL_DIR = {CAL_DIR}   ({len(runs)} runs)   ALPHA0_USED = {np.degrees(ALPHA0_USED):.2f} deg\n")

    per_run = []
    for d in runs:
        try:
            a_raw, psi_rel, excited, n_base, n_ok = _load_run(d)
        except Exception as e:
            print(f"  {os.path.basename(d):32s}  SKIP ({e})")
            continue
        if n_base < MIN_SAMPLES:
            print(f"  {os.path.basename(d):32s}  SKIP (only {n_base} detection-ok samples)")
            continue
        # slope=-1 model:  a_raw = -psi + off   ->   off = circ_mean(a_raw + psi).
        # Fit the offset over ALL detection-ok samples (slope is fixed, so every sample
        # with a known GT relative yaw constrains it); use the yaw-EXCITED subset only to
        # score the model (R^2 needs psi spread).
        off = _circ_mean(a_raw + psi_rel)
        e = excited if excited.sum() >= 20 else np.ones_like(excited)
        ae, pe = a_raw[e], psi_rel[e]
        resid = np.arctan2(np.sin(ae + pe - off), np.cos(ae + pe - off))
        pred = -pe + off
        ss_res = float(np.sum(np.arctan2(np.sin(ae - pred), np.cos(ae - pred)) ** 2))
        ss_tot = float(np.sum((np.unwrap(ae) - np.mean(np.unwrap(ae))) ** 2))
        r2 = 1.0 - ss_res / ss_tot if ss_tot > 1e-9 else float("nan")
        A = np.column_stack([np.unwrap(pe), np.ones_like(pe)])
        (slope_u, _o), *_ = np.linalg.lstsq(A, np.unwrap(ae), rcond=None)
        per_run.append((os.path.basename(d), np.degrees(off), np.degrees(_circ_std(resid)),
                        r2, slope_u, n_base, int(excited.sum())))
        print(f"  {os.path.basename(d):32s}  off={np.degrees(off):7.2f} deg  "
              f"resid_std={np.degrees(_circ_std(resid)):5.2f} deg  R2(slope=-1)={r2:6.3f}  "
              f"[uncon slope={slope_u:+.2f}]  n_ok={n_base}  n_yawexc={int(excited.sum())}")

    if len(per_run) < 3:
        sys.exit(f"\nonly {len(per_run)} usable runs -- need >=5 (>=3 to print a value); record more.")

    offs = np.radians([r[1] for r in per_run])
    mean_off = _circ_mean(offs)
    inter_std = np.degrees(np.std(np.arctan2(np.sin(offs - mean_off), np.cos(offs - mean_off))))
    r2s = [r[3] for r in per_run]
    print("\n" + "=" * 68)
    print(f"  n runs used           : {len(per_run)}")
    print(f"  per-run offset (deg)  : {[round(r[1], 2) for r in per_run]}")
    print(f"  CIRCULAR MEAN         : {np.degrees(mean_off):.2f} deg   ({mean_off:.4f} rad)")
    print(f"  inter-run std         : {inter_std:.2f} deg")
    print(f"  R2(slope=-1) per run  : {[round(x, 3) for x in r2s]}   mean {np.mean(r2s):.3f}")
    print(f"  unconstrained slopes  : {[round(r[4], 2) for r in per_run]}  "
          f"(should cluster near -1 if the model holds)")
    print("=" * 68)
    print(f"\n  -> set in cross_marker_perception.py:\n"
          f"     self._alpha_0 = float(os.environ.get(\"CROSS_ALPHA_0\", "
          f"str(np.radians({np.degrees(mean_off):.2f}))))")
    print(f"\n  (previous value: np.radians(90.23) = 1.5748 rad; "
          f"delta = {np.degrees(mean_off) - 90.23:+.2f} deg)")


if __name__ == "__main__":
    main()
