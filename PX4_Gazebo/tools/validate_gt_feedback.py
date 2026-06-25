#!/usr/bin/env python3
"""Offline sanity for the GT-feedback port (PLASMC_GT_FEEDBACK).

Compares the GT-computed V-frame centroid `s` and flow `h` (from gt_optical_flow,
pose-only) against the perception `s(t)` / `h(t)` the controller ACTUALLY consumed
(Control_Data). Goal: pin the frame/rotation/sign convention that maps the pose-only
GT features onto the perception convention the controller's gains are tuned for
(the camera image is rotated 90° CW in gz_subscriber, so a rotation may be needed).

It searches candidate 2D rotations/flips of the lateral (x,y) channels and reports
which best matches the recorded perception, per channel, with correlation + slope.

Usage:
    python3 tools/validate_gt_feedback.py "<rep_dir>"
"""
import sys, os
import numpy as np
sys.path.insert(0, os.path.dirname(__file__))
from gt_optical_flow import compute_gt_flow

# candidate lateral-channel maps (applied to [x,y]); name -> 2x2 matrix
CANDS = {
    "identity":  np.array([[1, 0], [0, 1]]),
    "rot90cw":   np.array([[0, 1], [-1, 0]]),
    "rot90ccw":  np.array([[0, -1], [1, 0]]),
    "rot180":    np.array([[-1, 0], [0, -1]]),
    "swap":      np.array([[0, 1], [1, 0]]),
    "flipx":     np.array([[-1, 0], [0, 1]]),
    "flipy":     np.array([[1, 0], [0, -1]]),
}


def _fit(gt, meas):
    """corr + least-squares slope (meas ≈ slope*gt) over finite samples."""
    m = np.isfinite(gt) & np.isfinite(meas)
    if m.sum() < 8:
        return np.nan, np.nan, int(m.sum())
    g, y = gt[m], meas[m]
    corr = np.corrcoef(g, y)[0, 1]
    slope = float(g @ y / (g @ g)) if g @ g > 1e-9 else np.nan
    return corr, slope, int(m.sum())


def _score_cands(gt_xy, meas_xy):
    """For each candidate map, mean |corr| over the two lateral channels."""
    rows = []
    for name, M in CANDS.items():
        gx = M[0, 0] * gt_xy[:, 0] + M[0, 1] * gt_xy[:, 1]
        gy = M[1, 0] * gt_xy[:, 0] + M[1, 1] * gt_xy[:, 1]
        cx, sx, nx = _fit(gx, meas_xy[:, 0])
        cy, sy, ny = _fit(gy, meas_xy[:, 1])
        rows.append((name, cx, sx, cy, sy, np.nanmean([abs(cx), abs(cy)])))
    rows.sort(key=lambda r: -r[-1])
    return rows


def main(rep):
    g = compute_gt_flow(rep)
    cd = np.load(os.path.join(rep, "Control_Data.npy"), allow_pickle=True).item()
    St = g['start_time']
    t_ctrl = np.asarray(cd['t'], float)            # absolute sim time (time_node clock)

    def align(y):
        y = np.asarray(y, float)
        ti = t_ctrl - St
        if y.ndim == 1:
            return np.interp(g['t_g'], ti, y)
        return np.column_stack([np.interp(g['t_g'], ti, y[:, k]) for k in range(y.shape[1])])

    s_meas = align(np.asarray(cd['s(t)']))          # perception centroid s the controller saw
    h_meas = align(np.asarray(cd['h(t)']))          # perception flow h the controller saw

    print(f"rep: {rep}")
    print(f"GT descent {g['t_g'][-1]-g['t_g'][0]:.2f}s, alt {g['alt'][0]:.1f}->{g['alt'][-1]:.2f}m,"
          f" ctrl samples {len(t_ctrl)}\n")

    print("=== CENTROID s  (GT V_s_g vs perception s[:2]) — best lateral map ===")
    for r in _score_cands(g['V_s_g'], s_meas[:, :2]):
        print(f"  {r[0]:9s} x:corr {r[1]:+.2f} slope {r[2]:+.2f}   y:corr {r[3]:+.2f} slope {r[4]:+.2f}"
              f"   | mean|corr| {r[5]:.2f}")

    print("\n=== FLOW h lateral (GT V_h_g[:2] vs perception h[:2]) — best lateral map ===")
    for r in _score_cands(g['V_h_g'][:, :2], h_meas[:, :2]):
        print(f"  {r[0]:9s} x:corr {r[1]:+.2f} slope {r[2]:+.2f}   y:corr {r[3]:+.2f} slope {r[4]:+.2f}"
              f"   | mean|corr| {r[5]:.2f}")

    cz, sz, nz = _fit(g['loom'], h_meas[:, 2])
    print(f"\n=== LOOM h_z (GT vs perception) ===\n  corr {cz:+.2f} slope {sz:+.2f}  (n={nz})")
    print(f"  |GT loom| mean {np.nanmean(np.abs(g['loom'])):.2f}  |meas h_z| mean {np.nanmean(np.abs(h_meas[:,2])):.2f}")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__); sys.exit(2)
    main(sys.argv[1])
