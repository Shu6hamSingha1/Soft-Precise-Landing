#!/usr/bin/env python3
"""Direct (non-correlation) time-sync check between the GT/outer-poll loop
and the image flow loop, using ONLY real timestamps - no cross-correlation
curve-fitting, which is exactly what made check_time_lag.py's result
ambiguous (a correlation peak can be dragged around by signal quality, not
just by an actual time offset).

Both output_calibration.py's outer loop and img_data.py's flow loop run as
THREADS INSIDE THE SAME PYTHON PROCESS, so time.perf_counter() is the SAME
clock instance for both - there is no cross-machine/cross-driver clock-
source ambiguity here (unlike the camera hardware SensorTimestamp case,
which genuinely is a different clock domain). So this is a much stronger
test than a lag-scan: reconstruct each GT sample's real capture instant as
(Start Time + t_c), reconstruct each flow sample's real compute instant
directly from img_data.py's own Time log, and for every GT poll find the
flow sample that was ACTUALLY current at that instant (the most recent one
computed at or before the poll - getOptFlowAngVel() cannot return a value
from the future). The gap between them is genuine poll staleness, not a
free-fitted lag - and should be small and merely tied to loop-rate jitter,
not systematically drifting, if the two loops are cleanly synced by the
shared clock.

Usage: python3 check_loop_staleness.py <run_dir> [<run_dir> ...]
"""
import sys
import numpy as np


def main(run_dirs):
    for run_dir in run_dirs:
        print(f"\n=== {run_dir} ===")
        gt = np.load(f"{run_dir}/Ground_Truth.npy", allow_pickle=True).item()
        img = np.load(f"{run_dir}/Img_Data.npy", allow_pickle=True).item()

        St = float(gt["Start Time"])
        t_c = np.asarray(gt["Time"], float)
        abs_gt = St + t_c                      # real perf_counter() instant of each GT poll

        abs_img = np.asarray(img["Time"], float)   # real perf_counter() instant of each flow sample
        if len(abs_img) < 2:
            print("  too few img samples")
            continue

        # New-format recordings (post capture-stamp fix) also carry a
        # per-poll img timestamp captured AT POLL TIME (Img Time Stamp) and
        # the mocap's own real packet-arrival stamp (GT Pose Stamp) - use
        # those directly if present, they're more precise than re-deriving
        # via nearest-preceding-sample search below.
        has_new_fields = "Img Time Stamp" in gt and "GT Pose Stamp" in gt
        print(f"  new capture-stamp fields present: {has_new_fields}")

        order = np.argsort(abs_img)
        abs_img_sorted = abs_img[order]
        if not np.all(np.diff(abs_img_sorted) >= 0):
            print("  WARNING: img Time not monotonic even after sort (dup timestamps?)")

        # For each GT poll, the flow sample actually returned by
        # getOptFlowAngVel() at that instant is the most recent one computed
        # AT OR BEFORE the poll - i.e. searchsorted(..., side='right') - 1.
        idx = np.searchsorted(abs_img_sorted, abs_gt, side="right") - 1
        valid = idx >= 0
        n_before_start = int((~valid).sum())
        idx = idx[valid]
        abs_gt_v = abs_gt[valid]

        staleness = abs_gt_v - abs_img_sorted[idx]   # >=0 by construction; how old the flow sample was
        # Also: nearest img sample regardless of direction (both-sided) -
        # tells us whether the outer loop is polling roughly in sync with
        # the flow loop's OWN rate, or badly under/oversampling it.
        idx_all = np.searchsorted(abs_img_sorted, abs_gt)
        idx_all = np.clip(idx_all, 0, len(abs_img_sorted) - 1)
        idx_lo = np.clip(idx_all - 1, 0, len(abs_img_sorted) - 1)
        nearest_gap = np.minimum(np.abs(abs_gt - abs_img_sorted[idx_all]),
                                  np.abs(abs_gt - abs_img_sorted[idx_lo]))

        img_dt = np.diff(abs_img_sorted)
        gt_dt = np.diff(abs_gt)

        print(f"  n GT polls: {len(abs_gt)}  (before first flow sample: {n_before_start})")
        print(f"  n flow samples: {len(abs_img)}")
        print(f"  GT loop dt   : median={np.median(gt_dt)*1000:.1f} ms  "
              f"mean={np.mean(gt_dt)*1000:.1f} ms  std={np.std(gt_dt)*1000:.1f} ms")
        print(f"  flow loop dt : median={np.median(img_dt)*1000:.1f} ms  "
              f"mean={np.mean(img_dt)*1000:.1f} ms  std={np.std(img_dt)*1000:.1f} ms")
        print(f"  staleness (GT poll time minus the flow sample it actually reads):")
        print(f"    median={np.median(staleness)*1000:.1f} ms  mean={np.mean(staleness)*1000:.1f} ms  "
              f"std={np.std(staleness)*1000:.1f} ms  max={np.max(staleness)*1000:.1f} ms  "
              f"p95={np.percentile(staleness, 95)*1000:.1f} ms")
        print(f"  nearest-sample gap (either direction): "
              f"median={np.median(nearest_gap)*1000:.1f} ms  mean={np.mean(nearest_gap)*1000:.1f} ms")

        # A real, roughly-constant processing/poll latency would show a
        # tight staleness distribution (low std relative to median). A wide
        # spread relative to the median, or a staleness comparable to
        # several flow-loop periods, means the outer loop is aliasing
        # against the flow loop's own rate rather than cleanly time-synced.
        if np.median(staleness) > 0 and np.std(staleness) / max(np.median(staleness), 1e-6) > 0.5:
            print("  -> staleness is NOT tightly clustered: outer-loop poll timing is not "
                  "cleanly synced to the flow loop's own cadence (aliasing, not a fixed lag).")
        else:
            print("  -> staleness looks like a stable, roughly-constant poll latency.")

        if has_new_fields:
            gts = np.asarray(gt["Img Time Stamp"], dtype=object)
            gtp = np.asarray(gt["GT Pose Stamp"], dtype=object)
            gts_f = np.array([np.nan if v is None else v for v in gts], float)
            gtp_f = np.array([np.nan if v is None else v for v in gtp], float)
            direct_stale = t_c - gts_f   # both already relative to start_time
            fin = np.isfinite(direct_stale)
            if fin.sum():
                print(f"  DIRECT (from Img Time Stamp field): median staleness="
                      f"{np.median(direct_stale[fin])*1000:.1f} ms, "
                      f"std={np.std(direct_stale[fin])*1000:.1f} ms")
            gt_poll_gap = t_c - gtp_f    # how stale the polled QTM pose was vs this loop tick
            fin2 = np.isfinite(gt_poll_gap)
            if fin2.sum():
                print(f"  GT pose poll staleness (t_c - GT Pose Stamp): median="
                      f"{np.median(gt_poll_gap[fin2])*1000:.1f} ms, "
                      f"std={np.std(gt_poll_gap[fin2])*1000:.1f} ms")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: check_loop_staleness.py <run_dir> [<run_dir> ...]")
        sys.exit(1)
    main(sys.argv[1:])
