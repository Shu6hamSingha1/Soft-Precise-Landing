#!/usr/bin/env python3
"""
Aggregate the per-axis sensitivity sweep summary.tsv into:
  1. Sensitivity matrix:   for each (gain × axis × multiplier), the %
     change of each control-input statistic vs baseline.
  2. Top-leverage report:  for each control channel (B_T, w_x, w_y, w_z),
     the gains whose perturbation moves that channel's RMS most.
  3. Cross-coupling:       gains that affect a channel they "shouldn't"
     (e.g., a Y-axis gain moving B_T noticeably).
  4. Precision impact:     gains whose perturbation reduces |x_err|+|y_err|.

Usage:
    python3 aggregate_sensitivity.py <summary.tsv>
"""
import csv
import sys
from collections import defaultdict


def load(path):
    rows = []
    with open(path) as f:
        rdr = csv.DictReader(f, delimiter='\t')
        for r in rdr:
            if r["landed"] != "YES":
                continue
            rows.append(r)
    return rows


def parse_floats(rows):
    numeric_cols = [
        "xy_err", "x_err", "y_err", "rel_vel", "flight_s",
        "BT_mean", "BT_std", "BT_rms", "BT_max",
        "wx_mean", "wx_std", "wx_rms", "wx_max",
        "wy_mean", "wy_std", "wy_rms", "wy_max",
        "wz_mean", "wz_std", "wz_rms", "wz_max",
    ]
    for r in rows:
        for c in numeric_cols:
            try:
                r[c] = float(r[c])
            except (TypeError, ValueError):
                r[c] = float("nan")
    return rows, numeric_cols


def pct(new, base):
    if base == 0 or base != base:  # nan
        return float("nan")
    return 100.0 * (new - base) / abs(base)


def main():
    if len(sys.argv) < 2:
        print("usage: aggregate_sensitivity.py <summary.tsv>")
        sys.exit(1)

    rows = load(sys.argv[1])
    rows, cols = parse_floats(rows)

    base = next((r for r in rows if r["gain"] == "BASELINE"), None)
    if base is None:
        print("No BASELINE row found"); sys.exit(1)

    perturbations = [r for r in rows if r["gain"] != "BASELINE"]

    # ====== 1. Per-perturbation sensitivity table ======
    print("=" * 100)
    print("Sensitivity (% change vs baseline)  —  positive = larger after perturbation")
    print("=" * 100)
    hdr = f"{'gain':<11} {'mult':>5} | " + " ".join(f"{c:>10}" for c in
            ("xy_err", "BT_rms", "wx_rms", "wy_rms", "wz_rms", "BT_max", "wx_max", "wy_max", "wz_max"))
    print(hdr)
    print("-" * len(hdr))
    for r in perturbations:
        line = f"{r['gain']:<11} {r['mult']:>5} | "
        for c in ("xy_err", "BT_rms", "wx_rms", "wy_rms", "wz_rms",
                  "BT_max", "wx_max", "wy_max", "wz_max"):
            line += f"{pct(r[c], base[c]):>9.1f}% "
        print(line)

    # ====== 2. Top leverage per control channel ======
    print()
    print("=" * 100)
    print("Top-5 gains by absolute |% change of channel RMS|")
    print("=" * 100)
    for ch_label, ch_stat in [("B_T", "BT_rms"), ("w_x (roll rate)", "wx_rms"),
                              ("w_y (pitch rate)", "wy_rms"), ("w_z (yaw rate)", "wz_rms")]:
        ranked = sorted(
            perturbations,
            key=lambda r: -abs(pct(r[ch_stat], base[ch_stat])),
        )[:5]
        print(f"\n  {ch_label}:")
        for r in ranked:
            print(f"    {r['gain']:<11} {r['mult']:>5}  ΔRMS={pct(r[ch_stat], base[ch_stat]):+7.1f}%   "
                  f"|x_err|={abs(r['x_err']):.3f}m, |y_err|={abs(r['y_err']):.3f}m, "
                  f"xy={r['xy_err']:.3f}, vel={r['rel_vel']:.2f}")

    # ====== 3. Cross-coupling: gains affecting "wrong" channel ======
    print()
    print("=" * 100)
    print("Cross-coupling: gains that affect a channel they 'shouldn't'  (|ΔRMS| > 25%)")
    print("=" * 100)
    # By convention: X-axis gains drive w_y (pitch), Y-axis gains drive w_x (roll),
    # Z-axis gains drive B_T. Off-axis impact is "leakage".
    leak = []
    for r in perturbations:
        axis = r["axis"]
        if axis == "X":
            # X-axis gains expected to move w_y mostly; check w_x, B_T leakage
            for ch in ("wx_rms", "BT_rms"):
                d = pct(r[ch], base[ch])
                if abs(d) > 25:
                    leak.append((r, ch, d))
        elif axis == "Y":
            # Y-axis -> w_x; check w_y, B_T leakage
            for ch in ("wy_rms", "BT_rms"):
                d = pct(r[ch], base[ch])
                if abs(d) > 25:
                    leak.append((r, ch, d))
        elif axis == "Z":
            # Z-axis -> B_T; check w_x, w_y leakage
            for ch in ("wx_rms", "wy_rms"):
                d = pct(r[ch], base[ch])
                if abs(d) > 25:
                    leak.append((r, ch, d))
    leak.sort(key=lambda t: -abs(t[2]))
    for r, ch, d in leak[:15]:
        print(f"  {r['gain']:<11} {r['mult']:>5}  axis={r['axis']}  spills {d:+6.1f}% into {ch}")
    if not leak:
        print("  (no significant cross-axis leakage; each gain affects only its expected channel)")

    # ====== 4. Precision leverage ======
    print()
    print("=" * 100)
    print("Precision impact: gains that REDUCED |x_err|+|y_err| vs baseline")
    print("=" * 100)
    base_norm = abs(base["x_err"]) + abs(base["y_err"])
    rankprec = sorted(
        perturbations,
        key=lambda r: (abs(r["x_err"]) + abs(r["y_err"]))
    )
    print(f"  Baseline: x_err={base['x_err']:+.3f}, y_err={base['y_err']:+.3f}, |x|+|y|={base_norm:.3f}m")
    print()
    print(f"  {'gain':<11} {'mult':>5}  |x|+|y|   Δ vs base   x_err     y_err     xy_err   vel")
    for r in rankprec[:12]:
        norm = abs(r["x_err"]) + abs(r["y_err"])
        print(f"  {r['gain']:<11} {r['mult']:>5}  {norm:>6.3f}m  {norm-base_norm:>+7.3f}m  "
              f"{r['x_err']:>+6.3f}m  {r['y_err']:>+6.3f}m  {r['xy_err']:>5.3f}m  {r['rel_vel']:.2f}")

    # ====== 5. Soft-precise summary ======
    print()
    print("=" * 100)
    print("Soft-precise check (xy ≤ 0.08, rel_vel ≤ 0.2)")
    print("=" * 100)
    soft_prec = [r for r in rows
                 if r["xy_err"] <= 0.08 and r["rel_vel"] <= 0.2]
    if soft_prec:
        for r in soft_prec:
            print(f"  ✓ {r['gain']:<11} {r['mult']:>5}  xy={r['xy_err']:.3f}m, vel={r['rel_vel']:.3f}m/s")
    else:
        prec = [r for r in rows if r["xy_err"] <= 0.08]
        soft = [r for r in rows if r["rel_vel"] <= 0.2]
        print(f"  No soft+precise hit.  PRECISE only: {len(prec)},  SOFT only: {len(soft)}")
        for r in prec[:5]:
            print(f"    PRECISE {r['gain']:<11} {r['mult']:>5}  xy={r['xy_err']:.3f}m, vel={r['rel_vel']:.3f}m/s")
        for r in soft[:5]:
            print(f"    SOFT    {r['gain']:<11} {r['mult']:>5}  xy={r['xy_err']:.3f}m, vel={r['rel_vel']:.3f}m/s")


if __name__ == "__main__":
    main()
