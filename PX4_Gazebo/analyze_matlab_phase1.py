#!/usr/bin/env python3
"""
Phase 1 analyzer: compare MATLAB baseline performance against PX4 SITL.
Reads phase1_summary.mat (and per-rep .mat files if needed) and produces
the decisive comparison table for the question:

  "Does PLASMC achieve soft+precise in MATLAB at the same IC PX4 uses?"

Usage:
    python3 analyze_matlab_phase1.py <path-to-MATLAB/Datasets/Phase1>
"""
from __future__ import annotations
import argparse
import sys
from pathlib import Path

import numpy as np
from scipy.io import loadmat


PX4_REFERENCES = {
    # batch_name → (n, xy_mean, xy_std, xy_min, vel_mean, PREC, SOFT, SP)
    "PX4_LOOSE_IC":      (10, 0.484, 0.379, 0.033, 0.890, 2, 1, 0),
    "PX4_TIGHT_IC":      (9,  0.517, 0.286, 0.047, 0.840, 1, 3, 0),
    "PX4_ULTRATIGHT_IC": (9,  0.531, 0.263, 0.019, 0.640, 1, 1, 0),
}


def _struct_field(mat_struct, field):
    """Pull a column from a MATLAB struct-of-arrays loaded by loadmat."""
    arr = mat_struct[field]
    # loadmat wraps each cell as a 0-d ndarray; unwrap
    out = []
    for x in arr.ravel():
        if isinstance(x, np.ndarray) and x.shape == ():
            x = x.item()
        out.append(x)
    return out


def load_summary(path: Path):
    mat = loadmat(str(path), squeeze_me=False, struct_as_record=False)
    res = mat["results"]
    # res is a 1×N struct array
    if hasattr(res, "ravel"):
        rows = res.ravel()
    else:
        rows = res
    out = []
    for r in rows:
        out.append({
            "batch":     str(r.batch[0]) if hasattr(r.batch, '__len__') else str(r.batch),
            "rep":       int(np.atleast_1d(r.rep).item()),
            "IC":        np.atleast_1d(r.IC).ravel().tolist(),
            "noise":     int(np.atleast_1d(r.noise).item()),
            "landed":    int(np.atleast_1d(r.landed).item()),
            "t_land":    float(np.atleast_1d(r.t_land).item()),
            "alt_above": float(np.atleast_1d(r.alt_above).item()),
            "xy_err":    float(np.atleast_1d(r.xy_err).item()),
            "rel_vel":   float(np.atleast_1d(r.rel_vel).item()),
            "precise":   int(np.atleast_1d(r.precise).item()),
            "soft":      int(np.atleast_1d(r.soft).item()),
        })
    return out


def summarize_batch(rows):
    if not rows:
        return None
    xy = np.array([r["xy_err"] for r in rows], dtype=float)
    vel = np.array([r["rel_vel"] for r in rows], dtype=float)
    return {
        "n":          len(rows),
        "xy_mean":    float(np.nanmean(xy)),
        "xy_std":     float(np.nanstd(xy, ddof=1)) if len(xy) > 1 else 0.0,
        "xy_min":     float(np.nanmin(xy)),
        "xy_max":     float(np.nanmax(xy)),
        "vel_mean":   float(np.nanmean(vel)),
        "vel_std":    float(np.nanstd(vel, ddof=1)) if len(vel) > 1 else 0.0,
        "PREC":       int(sum(r["precise"] for r in rows)),
        "SOFT":       int(sum(r["soft"] for r in rows)),
        "SP":         int(sum(r["precise"] and r["soft"] for r in rows)),
        "landed":     int(sum(r["landed"] for r in rows)),
    }


def print_row(label, s, ic_str=""):
    if s is None:
        print(f"  {label:<26} NO DATA")
        return
    print(f"  {label:<26} {ic_str:<14} n={s['n']:<3}  "
          f"xy_mean={s['xy_mean']:.4f}  xy_std={s['xy_std']:.4f}  "
          f"xy_min={s['xy_min']:.4f}  vel_mean={s['vel_mean']:.4f}  "
          f"PREC={s['PREC']}/{s['n']}  SOFT={s['SOFT']}/{s['n']}  SP={s['SP']}/{s['n']}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("phase1_dir", help="Directory containing phase1_summary.mat")
    args = ap.parse_args()

    pdir = Path(args.phase1_dir)
    summary_path = pdir / "phase1_summary.mat"
    if not summary_path.exists():
        sys.exit(f"phase1_summary.mat not found in {pdir}")

    rows = load_summary(summary_path)
    print(f"Loaded {len(rows)} MATLAB-side runs from {summary_path}\n")

    batches = sorted({r["batch"] for r in rows})
    print("=" * 110)
    print("MATLAB Phase 1 — baseline performance per batch")
    print("=" * 110)
    matlab_summaries = {}
    for b in batches:
        batch_rows = [r for r in rows if r["batch"] == b]
        s = summarize_batch(batch_rows)
        matlab_summaries[b] = s
        ic = batch_rows[0]["IC"] if batch_rows else [0, 0, 0]
        ic_str = f"IC=({ic[0]:.0f},{ic[1]:.0f},{ic[2]:.0f})"
        print_row(b, s, ic_str)

    print()
    print("=" * 110)
    print("PX4 SITL reference (for comparison) — from N=10 runs 2026-05-21")
    print("=" * 110)
    for label, (n, xym, xys, xymn, vm, p, s_, sp) in PX4_REFERENCES.items():
        ic_str = "IC=(0,0,5)"
        fake = {"n": n, "xy_mean": xym, "xy_std": xys, "xy_min": xymn,
                "xy_max": 0, "vel_mean": vm, "vel_std": 0,
                "PREC": p, "SOFT": s_, "SP": sp, "landed": n}
        print_row(label, fake, ic_str)

    # Verdict
    print()
    print("=" * 110)
    print("VERDICT")
    print("=" * 110)
    a = matlab_summaries.get("A_PX4ic_noise")
    b = matlab_summaries.get("B_PX4ic_noiseless")
    c = matlab_summaries.get("C_canonical_noise")
    px4 = PX4_REFERENCES["PX4_LOOSE_IC"]

    if a is None:
        print("  (no batch A data — re-run phase1_baseline_sweep)")
        return

    sp_rate = a["SP"] / a["n"]
    print(f"\n  MATLAB SP rate at PX4-aligned IC (noise=50dB):  {a['SP']}/{a['n']} = {100*sp_rate:.0f}%")
    print(f"  PX4 SP rate at same IC (across ~30 runs):       0/30 = 0%")
    print(f"\n  MATLAB xy_mean (with noise):  {a['xy_mean']:.4f} m"
          f"   vs   PX4 xy_mean: {px4[1]:.4f} m  → ratio {px4[1]/max(a['xy_mean'],1e-3):.2f}×")

    if sp_rate >= 0.5:
        print("\n  → MATLAB achieves soft+precise reliably at this IC.")
        print("    The PX4 gap is SITL-specific (sensor noise, MAVSDK rate lag, PX4 rate-loop dynamics).")
        print("    Phase 2 (loop-latency budget) and Phase 4 (sensor-noise floor) are the next steps.")
    elif sp_rate >= 0.1:
        print("\n  → MATLAB occasionally hits soft+precise but isn't reliable either.")
        print("    The controller is near the threshold even in MATLAB. The descent profile or spec ")
        print("    may need to be re-examined.")
    else:
        print("\n  → MATLAB also fails to reliably hit soft+precise at this IC.")
        print("    The controller design itself cannot reach 0.08 m + 0.2 m/s from IC1 with this descent.")
        print("    Either relax the spec or restructure the descent profile.")

    if b is not None:
        print(f"\n  Noiseless MATLAB (theoretical floor): xy={b['xy_min']:.4f}, vel={b['vel_mean']:.4f}, "
              f"PREC={b['PREC']}/{b['n']}, SOFT={b['SOFT']}/{b['n']}, SP={b['SP']}/{b['n']}")
        if b["SP"] > 0:
            print("    → Noiseless does achieve soft+precise; noise budget gap is the issue.")
        else:
            print("    → Even with ZERO noise the controller doesn't achieve soft+precise. ")
            print("      The descent kinematics themselves are the bottleneck.")
    if c is not None:
        print(f"\n  Canonical IC (2,2,-5) noise sanity:  xy_mean={c['xy_mean']:.4f},  PREC={c['PREC']}/{c['n']}, SP={c['SP']}/{c['n']}")


if __name__ == "__main__":
    main()
