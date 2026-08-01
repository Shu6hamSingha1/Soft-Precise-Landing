"""Diagnose marker-acquisition failure across the 7 real 3m armed flights on
2026-07-30 -- all 7 ended in pilot takeover + final-descent timeout. Uses
Control_Data.npy's MARKER_EXTENT_PX(t) (marker pixel size) and s(t)/s_e_n(t)
(image feature error) to characterize exactly when/why the marker was lost.

Usage: python analyze_landing_marker_loss.py <Landing_dir>
"""
import sys
import os
import glob
import numpy as np

RUNS = [
    "Thu Jul 30 17-57-37 2026",
    "Thu Jul 30 17-58-11 2026",
    "Thu Jul 30 17-58-42 2026",
    "Thu Jul 30 17-59-07 2026",
    "Thu Jul 30 17-59-41 2026",
    "Thu Jul 30 18-02-10 2026",
    "Thu Jul 30 18-02-40 2026",
]


def analyze(landing_dir, run):
    path = os.path.join(landing_dir, run, "Control_Data.npy")
    if not os.path.isfile(path):
        print(f"  [MISSING] {path}")
        return
    d = np.load(path, allow_pickle=True).item()
    t_raw = np.array(d["t"])
    extent_raw = np.array(d.get("MARKER_EXTENT_PX(t)", []), dtype=float)
    n = min(len(t_raw), len(extent_raw))
    t = t_raw[:n]
    extent = extent_raw[:n]
    print(f"\n=== {run} ===")
    print(f"  samples: {n}, duration: {t[-1]-t[0]:.2f}s" if n else "  no samples")
    if n == 0:
        return

    valid = np.isfinite(extent) & (extent > 0)
    print(f"  extent valid (finite, >0) fraction: {valid.mean():.2%} ({valid.sum()}/{n})")
    if valid.any():
        print(f"  extent range while valid: {extent[valid].min():.1f}px - {extent[valid].max():.1f}px, "
              f"mean={extent[valid].mean():.1f}px")

    # find the FIRST loss (valid -> invalid transition) and its context
    first_loss_idx = None
    for i in range(1, n):
        if valid[i-1] and not valid[i]:
            first_loss_idx = i
            break
    if first_loss_idx is not None:
        t0 = t[0]
        pre = extent[max(0, first_loss_idx-5):first_loss_idx]
        print(f"  FIRST loss at t={t[first_loss_idx]-t0:.2f}s (sample {first_loss_idx}/{n})")
        print(f"    extent in the 5 samples before loss: {np.round(pre, 1).tolist()}")
    else:
        if valid.any():
            print("  never lost after first acquisition (or lost before t=0 window captured)")
        else:
            print("  marker was NEVER valid for the whole recorded window")

    # count total distinct loss events (valid->invalid transitions)
    transitions = int(np.sum(valid[:-1] & ~valid[1:]))
    print(f"  total loss events (valid->invalid transitions): {transitions}")


def main():
    landing_dir = sys.argv[1] if len(sys.argv) > 1 else "."
    for run in RUNS:
        analyze(landing_dir, run)


if __name__ == "__main__":
    main()
