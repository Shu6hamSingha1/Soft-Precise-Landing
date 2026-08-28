"""Compare cross-marker flow point-distribution symmetry between
CROSS_RING_SAMPLING=0 and =1 flights, using the existing "Point Diag Log"
(CrossMarkerNode.get_point_diag_log(), saved into Ground_Truth.npy -- see
cross_marker_perception.py:1432 docstring: (t, prev_n, solved_Tz, curr_n, dt, sol)
per successful solve, prev_n/curr_n are (N,2) normalized V-frame points).

Per-frame metric (named origin_ratio to match the existing moment_loom usage
in cross_marker_perception.py:1025 -- same shape of quantity, spread-vs-offset):
    origin_ratio = mean(|p - centroid(p)|^2) / max(|centroid(p)|^2, eps)
High origin_ratio = points spread widely relative to how far their centroid
sits from the frame origin (image center in normalized coords) -- i.e. a
symmetric, well-distributed point set. Low origin_ratio = points clustered
to one side (the failure mode ring sampling targets).

Usage: python3 compare_ring_sampling.py <ring0_dir> <ring1_dir>
"""
import sys
import numpy as np


def load_origin_ratios(flight_dir):
    # "Point Diag Log" is saved into Img_Data.npy via CrossMarkerNode.getLogData()
    # (cross_marker_perception.py:1738), not Ground_Truth.npy.
    img = np.load(f"{flight_dir}/Img_Data.npy", allow_pickle=True).item()
    log = img.get("Point Diag Log", [])
    ratios = []
    for entry in log:
        t, prev_n, solved_Tz, curr_n, dt, sol = entry
        if prev_n is None or len(prev_n) < 2:
            continue
        centroid = prev_n.mean(axis=0)
        spread = float(np.mean(np.sum((prev_n - centroid) ** 2, axis=1)))
        origin_d2 = float(np.dot(centroid, centroid))
        ratios.append(spread / max(origin_d2, 1e-9))
    return np.array(ratios)


def summarize(name, ratios):
    if len(ratios) == 0:
        print(f"{name}: no Point Diag Log entries found")
        return
    print(f"{name}: n={len(ratios)} mean={ratios.mean():.3f} median={np.median(ratios):.3f} "
          f"p10={np.percentile(ratios, 10):.3f} p90={np.percentile(ratios, 90):.3f}")


if __name__ == "__main__":
    ring0_dir, ring1_dir = sys.argv[1], sys.argv[2]
    r0 = load_origin_ratios(ring0_dir)
    r1 = load_origin_ratios(ring1_dir)
    summarize("CROSS_RING_SAMPLING=0", r0)
    summarize("CROSS_RING_SAMPLING=1", r1)
    if len(r0) and len(r1):
        print(f"\nmedian ratio change: {np.median(r1) - np.median(r0):+.3f} "
              f"({'ring sampling more symmetric' if np.median(r1) > np.median(r0) else 'ring sampling LESS symmetric — investigate'})")
