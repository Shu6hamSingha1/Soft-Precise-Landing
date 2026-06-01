#!/usr/bin/env python3
"""
Aggregate the n=5 BIG sensitivity sweep. Produces:
  1. Per-cell n=5 statistics (mean ± std for every metric)
  2. Rankings:
     - Best precision (lowest mean xy_err, target_lost runs excluded from mean)
     - Best softness (lowest mean rel_vel)
     - Soft-precise composite score: mean(xy_err) + 2*mean(rel_vel) + 10*P(TL)
  3. Cross-coupling heatmap-ready: which gain perturbations affect which channels
  4. Stability check: cells with TARGET_LOST > 0 are flagged

Usage:
    python3 aggregate_big_sensitivity.py <summary.tsv> [--top N]
"""
import csv
import sys
import argparse
from collections import defaultdict
import math


def fnum(x):
    try:
        return float(x)
    except (TypeError, ValueError):
        return float("nan")


def mean(xs):
    xs = [x for x in xs if not math.isnan(x)]
    return sum(xs) / len(xs) if xs else float("nan")


def std(xs):
    xs = [x for x in xs if not math.isnan(x)]
    if len(xs) < 2:
        return float("nan")
    m = sum(xs) / len(xs)
    return math.sqrt(sum((x - m) ** 2 for x in xs) / (len(xs) - 1))


def main():
    p = argparse.ArgumentParser()
    p.add_argument("summary_tsv")
    p.add_argument("--top", type=int, default=10)
    args = p.parse_args()

    rows = list(csv.DictReader(open(args.summary_tsv), delimiter="\t"))
    landed_rows = [r for r in rows if r["landed"] == "YES"]
    print(f"Total runs: {len(rows)}, landed: {len(landed_rows)}, "
          f"failed: {len(rows) - len(landed_rows)}")

    # Group by cell
    cells = defaultdict(list)
    for r in landed_rows:
        cells[r["cell"]].append(r)

    print(f"Unique cells: {len(cells)}")
    print()

    # Baseline reference
    if "BASELINE" not in cells:
        print("WARNING: no BASELINE rows found")
        baseline = None
    else:
        baseline_reps = cells["BASELINE"]
        baseline = {
            "xy_mean": mean([fnum(r["xy_err"]) for r in baseline_reps]),
            "xy_std": std([fnum(r["xy_err"]) for r in baseline_reps]),
            "vel_mean": mean([fnum(r["rel_vel"]) for r in baseline_reps]),
            "tl_rate": sum(int(r["target_lost"]) for r in baseline_reps) / len(baseline_reps),
            "n": len(baseline_reps),
        }
        print(f"BASELINE (n={baseline['n']}):")
        print(f"  xy_err:  mean={baseline['xy_mean']:.3f}  std={baseline['xy_std']:.3f}")
        print(f"  rel_vel: mean={baseline['vel_mean']:.3f}")
        print(f"  TARGET_LOST rate: {baseline['tl_rate']*100:.0f}%")
        print()

    # Per-cell statistics
    summary = []
    for cell_name, reps in cells.items():
        if cell_name == "BASELINE":
            continue
        xy = [fnum(r["xy_err"]) for r in reps]
        vel = [fnum(r["rel_vel"]) for r in reps]
        fs = [fnum(r["flight_s"]) for r in reps]
        wmax = [fnum(r["w_max"]) for r in reps]
        osc = [fnum(r["Ia_z_osc"]) for r in reps]
        def _bool(v):
            try: return int(v)
            except (TypeError, ValueError): return 0
        tl_count = sum(_bool(r["target_lost"]) for r in reps)
        precise_count = sum(_bool(r["precise"]) for r in reps)
        soft_count = sum(_bool(r["soft"]) for r in reps)
        # Soft+precise needs both AND no target loss
        sp_count = sum(
            1 for r in reps
            if _bool(r["precise"]) and _bool(r["soft"]) and not _bool(r["target_lost"])
        )
        # Composite score (lower is better): mean xy + 2*mean vel + 10*P(TL)
        composite = mean(xy) + 2 * mean(vel) + 10 * (tl_count / len(reps))
        summary.append({
            "cell": cell_name,
            "env_var": reps[0]["env_var"],
            "env_val": reps[0]["env_val"],
            "n": len(reps),
            "xy_mean": mean(xy),
            "xy_std": std(xy),
            "xy_min": min(xy) if xy else float("nan"),
            "vel_mean": mean(vel),
            "vel_std": std(vel),
            "flight_mean": mean(fs),
            "wmax_mean": mean(wmax),
            "osc_mean": mean(osc),
            "tl_count": tl_count,
            "precise_count": precise_count,
            "soft_count": soft_count,
            "sp_count": sp_count,
            "composite": composite,
        })

    # === RANK BY PRECISION ===
    print("=" * 110)
    print(f"TOP {args.top} cells by mean xy_err (lower is better; TL counted as 5m penalty)")
    print("=" * 110)
    by_xy = sorted(summary, key=lambda x: x["xy_mean"])
    print(f"  {'cell':<18} {'n':>2}  {'xy_mean':>8}  {'xy_std':>7}  {'xy_min':>7}  {'vel_mean':>9}  TL  PREC  SOFT  SP")
    for s in by_xy[:args.top]:
        print(f"  {s['cell']:<18} {s['n']:>2}  "
              f"{s['xy_mean']:>8.3f}  {s['xy_std']:>7.3f}  {s['xy_min']:>7.3f}  "
              f"{s['vel_mean']:>9.3f}  {s['tl_count']:>2}  {s['precise_count']:>4}  "
              f"{s['soft_count']:>4}  {s['sp_count']:>2}")

    # === RANK BY SOFTNESS ===
    print()
    print("=" * 110)
    print(f"TOP {args.top} cells by mean rel_vel (lower is better; only TL-free reps count)")
    print("=" * 110)
    by_vel = sorted(summary, key=lambda x: x["vel_mean"])
    print(f"  {'cell':<18} {'n':>2}  {'vel_mean':>9}  {'vel_std':>8}  {'xy_mean':>8}  TL  SOFT  SP")
    for s in by_vel[:args.top]:
        print(f"  {s['cell']:<18} {s['n']:>2}  {s['vel_mean']:>9.3f}  "
              f"{s['vel_std']:>8.3f}  {s['xy_mean']:>8.3f}  {s['tl_count']:>2}  "
              f"{s['soft_count']:>4}  {s['sp_count']:>2}")

    # === COMPOSITE: SOFT-PRECISE SCORE ===
    print()
    print("=" * 110)
    print(f"TOP {args.top} by soft-precise composite (xy + 2*vel + 10*P(TL), lower is better)")
    print("=" * 110)
    by_comp = sorted(summary, key=lambda x: x["composite"])
    print(f"  {'cell':<18} {'n':>2}  {'composite':>9}  {'xy_mean':>8}  {'vel_mean':>9}  TL  SP")
    for s in by_comp[:args.top]:
        print(f"  {s['cell']:<18} {s['n']:>2}  {s['composite']:>9.3f}  "
              f"{s['xy_mean']:>8.3f}  {s['vel_mean']:>9.3f}  {s['tl_count']:>2}  {s['sp_count']:>2}")

    # === STABILITY FLAGS ===
    print()
    print("=" * 110)
    print(f"CELLS WITH HIGH TARGET_LOST (instability)")
    print("=" * 110)
    unstable = [s for s in summary if s["tl_count"] >= 2]
    unstable.sort(key=lambda x: -x["tl_count"])
    print(f"  {'cell':<18} {'TL':>3} of {'n':>2}  {'wmax_mean':>9}  {'osc_mean':>8}")
    for s in unstable[:args.top]:
        print(f"  {s['cell']:<18} {s['tl_count']:>3}    {s['n']:>2}  "
              f"{s['wmax_mean']:>9.3f}  {s['osc_mean']:>8.3f}")

    # === BIG IMPROVERS vs BASELINE ===
    if baseline:
        print()
        print("=" * 110)
        print(f"BIG IMPROVERS vs baseline (xy_mean improvement > 1 std)")
        print("=" * 110)
        thr = baseline["xy_mean"] - baseline["xy_std"]
        big = [s for s in summary if s["xy_mean"] < thr and s["tl_count"] == 0]
        big.sort(key=lambda x: x["xy_mean"])
        print(f"  Baseline mean xy = {baseline['xy_mean']:.3f}, std = {baseline['xy_std']:.3f}, threshold = {thr:.3f}")
        print(f"  {'cell':<18} {'xy_mean':>8}  Δ vs base   {'xy_std':>7}  {'vel_mean':>9}")
        for s in big[:args.top]:
            print(f"  {s['cell']:<18} {s['xy_mean']:>8.3f}  "
                  f"{s['xy_mean'] - baseline['xy_mean']:>+9.3f}  {s['xy_std']:>7.3f}  "
                  f"{s['vel_mean']:>9.3f}")
        if not big:
            print("  (none)")

    # === SOFT+PRECISE HITS ===
    print()
    print("=" * 110)
    print("CELLS WITH ANY SOFT+PRECISE LANDING")
    print("=" * 110)
    sp_cells = [s for s in summary if s["sp_count"] > 0]
    if not sp_cells:
        print("  (none)")
    else:
        sp_cells.sort(key=lambda x: -x["sp_count"])
        for s in sp_cells:
            print(f"  {s['cell']:<18}  SP={s['sp_count']}/{s['n']}  "
                  f"xy_mean={s['xy_mean']:.3f}  vel_mean={s['vel_mean']:.3f}")


if __name__ == "__main__":
    main()
