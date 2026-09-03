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
        e = excited if excited.sum() >= 20 else np.ones_like(excited)
        ae, pe = a_raw[e], psi_rel[e]
        # UNCONSTRAINED fit first -- the 2026-08-08 derivation FORCED slope=-1 (Jabbari
        # Asl); let the data speak. Fit on unwrapped angles (yaw excitation spans <2pi).
        aeu, peu = np.unwrap(ae), np.unwrap(pe)
        A = np.column_stack([peu, np.ones_like(peu)])
        (slope_u, off_u), *_ = np.linalg.lstsq(A, aeu, rcond=None)
        r2_u = 1.0 - np.sum((aeu - A @ [slope_u, off_u]) ** 2) / np.sum((aeu - aeu.mean()) ** 2)
        # offset for each sign convention:  a_raw = s*psi + off  ->  off = circ_mean(a_raw - s*psi)
        off_p1 = _circ_mean(ae - pe)     # slope +1 (ArUco-like:  alpha ~ +psi)
        off_m1 = _circ_mean(ae + pe)     # slope -1 (the 2026-08-08 assumption)
        def _r2(sl, of):
            pr = sl * pe + of
            return 1.0 - np.sum(np.arctan2(np.sin(ae - pr), np.cos(ae - pr)) ** 2) \
                       / np.sum((aeu - aeu.mean()) ** 2)
        r2_p1, r2_m1 = _r2(1.0, off_p1), _r2(-1.0, off_m1)
        # the deployed pipeline consumes  alpha = wrap(a_raw - alpha_0)  then
        # yaw_c = BODY_YAW_ALPHA_K * alpha  (K=-1 for cross). For yaw_c to track NED yaw
        # (= -psi_ENU) the same way the compass path does, alpha must track +psi_ENU
        # (slope +1), so alpha_0 = off_p1.
        alpha0 = off_p1
        per_run.append((os.path.basename(d), np.degrees(alpha0), np.degrees(off_m1),
                        slope_u, r2_u, r2_p1, r2_m1, n_base, int(excited.sum())))
        print(f"  {os.path.basename(d):30s}  slope={slope_u:+.2f} (R2 {r2_u:.3f}) | "
              f"alpha_0(+1)={np.degrees(off_p1):7.2f} (R2 {r2_p1:.3f})  "
              f"alt(-1)={np.degrees(off_m1):7.2f} (R2 {r2_m1:.3f})  "
              f"n_yawexc={int(excited.sum())}")

    if len(per_run) < 3:
        sys.exit(f"\nonly {len(per_run)} usable runs -- need >=5 (>=3 to print a value); record more.")

    def _cm_std(vals_deg):
        r = np.radians(vals_deg); m = _circ_mean(r)
        return np.degrees(m), np.degrees(np.std(np.arctan2(np.sin(r - m), np.cos(r - m))))

    a0_p1, a0_p1_std = _cm_std([r[1] for r in per_run])
    a0_m1, a0_m1_std = _cm_std([r[2] for r in per_run])
    slopes = [r[3] for r in per_run]
    print("\n" + "=" * 72)
    print(f"  n runs used            : {len(per_run)}")
    print(f"  UNCONSTRAINED slope    : {[round(s, 2) for s in slopes]}   "
          f"mean {np.mean(slopes):+.2f}   (R2 {[round(r[4],3) for r in per_run]})")
    print(f"    -> the 2026-08-08 derivation forced slope=-1; the data says "
          f"{'+1' if np.mean(slopes) > 0 else '-1'}.")
    print(f"  alpha_0  (slope +1)    : per-run {[round(r[1],2) for r in per_run]}  "
          f"-> {a0_p1:.2f} deg  (std {a0_p1_std:.2f}, R2 {[round(r[5],3) for r in per_run]})")
    print(f"  alt      (slope -1)    : per-run {[round(r[2],2) for r in per_run]}  "
          f"-> {a0_m1:.2f} deg  (std {a0_m1_std:.2f}, R2 {[round(r[6],3) for r in per_run]})")
    print("=" * 72)
    best = a0_p1 if np.mean(slopes) > 0 else a0_m1
    print(f"\n  RECOMMENDED (matches the measured slope, ArUco convention K=-1 expects +psi):")
    print(f"     self._alpha_0 = float(os.environ.get(\"CROSS_ALPHA_0\", "
          f"str(np.radians({best:.2f}))))")
    print(f"\n  deployed: np.radians(90.23); delta = {best - 90.23:+.2f} deg. "
          f"⚠ If slope flipped +1<->-1 since 2026-08-08, BODY_YAW_ALPHA_K sign may also\n"
          f"    need review -- IC1-5 validate before merge.")


if __name__ == "__main__":
    main()
