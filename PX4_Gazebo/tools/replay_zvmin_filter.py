#!/usr/bin/env python3
"""Offline replay of CROSS_Z_V_MIN_FLOW's grazing-ray rejection filter, ISOLATED from
CROSS_FLOW_ANG_MAX, against ALREADY-RECORDED point correspondences -- before any live
SITL test.

Continues tools/replay_flow_solve_conditioning.py's finding (see
project_20260917_visibility_predictor_residual.md, 2026-09-22 entry): the terminal loom
(h_z) divergence is caused by near-grazing rays (small z_v) amplifying through the
perspective divide, NOT matrix ill-conditioning. CrossMarkerPerception's own comment
notes CROSS_Z_V_MIN_FLOW=0.4 was tested but only BUNDLED with CROSS_FLOW_ANG_MAX (which
backfired for an unrelated reason -- point starvation once the marker fills the frame) --
"0.4 in ISOLATION is untested." This script tests it in isolation, offline, exactly
replicating the live geometry-rejection + count-floor-fallback logic
(_solve_jacobian's `_geokeep` block, MIN_FLOW_POINTS_SOLVE=4), so a live SITL gate isn't
needed just to see whether the filter would even engage or what it would do to these
already-observed spike frames.

WHAT IT SHOWS, per candidate threshold, at each frame in the window:
  n_kept        how many points the filter would have kept (vs the original count)
  fallback      True if too few survived and the live code's count floor would have kept
                the ORIGINAL (unfiltered) set instead -- i.e. the filter is a no-op there
  sol_Tz_filt   the raw single-frame Tz solution AFTER filtering (compare to
                replay_flow_solve_conditioning.py's unfiltered sol_Tz for the same frame)

Same replication scope/limits as replay_flow_solve_conditioning.py: full 6-unknown solve,
no gyro de-rotation, no sensor cal, no KF -- see that file's docstring for why sol_Tz
should be compared to ITSELF across thresholds (does filtering suppress the spike?), not
numerically to the logged h_V_z.

Usage:
  replay_zvmin_filter.py <rep_dir> <lo_rel_s> <hi_rel_s> <start_time_s> [thresholds...]
  (thresholds default to 0.3,0.4,0.5,0.6,0.7 if omitted)
"""
import sys
import numpy as np

CENTER = np.array([120., 160.])
FOCAL = np.array([135., 135.])
MIN_FLOW_POINTS_SOLVE = 4   # matches src/cross_marker_perception.py exactly


def _dcm(qwxyz):
    w, x, y, z = qwxyz
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def get_virtual_pts(pts, quat):
    R = _dcm(quat)
    g = R.T @ np.array([0., 0., 1.])
    z_axis = g / np.linalg.norm(g)
    x_axis = np.cross([0., 1., 0.], z_axis)
    x_axis /= np.linalg.norm(x_axis)
    y_axis = np.cross(z_axis, x_axis)
    C_R_V = np.column_stack([x_axis, y_axis, z_axis])
    x = (pts[:, 0] - CENTER[0]) / FOCAL[0]
    y = (pts[:, 1] - CENTER[1]) / FOCAL[1]
    rays = np.column_stack([y, -x, np.ones_like(x)])
    V_rays = rays @ C_R_V
    z_v = V_rays[:, 2]
    out = np.column_stack([V_rays[:, 0] / z_v, V_rays[:, 1] / z_v])
    return out, z_v


def fill_A(centered_pts):
    x = centered_pts[:, 0]
    y = centered_pts[:, 1]
    n = len(x)
    A = np.zeros((2 * n, 6))
    A[0::2, 0] = 1
    A[1::2, 1] = 1
    A[0::2, 2] = -x
    A[1::2, 2] = -y
    A[0::2, 3] = -x * y
    A[1::2, 3] = -(1 + y ** 2)
    A[0::2, 4] = 1 + x ** 2
    A[1::2, 4] = x * y
    A[0::2, 5] = -y
    A[1::2, 5] = x
    return A


def solve_tz(prev_n, curr_n, dt):
    vel = (curr_n - prev_n) / dt
    A = fill_A(prev_n)
    b = vel.reshape(-1)
    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    return float(sol[2])


def replay(rep_dir, lo_rel, hi_rel, start_time, thresholds):
    im = np.load(rep_dir + "/Img_Data.npy", allow_pickle=True).item()
    T = np.asarray(im["Time"], float)
    fp = im["Flow Points Prev Px"]
    fc = im["Flow Points Curr Px"]
    q = im["Quat"]
    h_v = np.asarray(im["h_V"], float)
    lo, hi = start_time + lo_rel, start_time + hi_rel
    idx = np.where((T >= lo) & (T <= hi))[0]

    header = f"{'t_rel':>7} {'n0':>4} {'sol_Tz(unfilt)':>14} {'h_V_z(KF)':>10}"
    for th in thresholds:
        header += f" | Zv>={th:<4}"
    print(header)

    for i in idx:
        if i >= len(fp) or i >= len(fc) or i >= len(q):
            continue
        p, c = fp[i], fc[i]
        if p is None or c is None:
            continue
        p = np.asarray(p, float)
        c = np.asarray(c, float)
        if p.shape != c.shape or len(p) < 4:
            continue
        dt = T[i] - T[i - 1] if i > 0 else np.nan
        if not (dt > 0):
            continue
        qprev = q[i - 1] if i > 0 else q[i]
        qcurr = q[i]
        prev_n, zv_p = get_virtual_pts(p, qprev)
        curr_n, zv_c = get_virtual_pts(c, qcurr)
        sol_tz_unfilt = solve_tz(prev_n, curr_n, dt)

        row = f"{T[i] - start_time:>7.3f} {len(p):>4} {sol_tz_unfilt:>14.3f} {h_v[i, 2]:>10.3f}"
        for th in thresholds:
            keep = (zv_p >= th) & (zv_c >= th)
            n_keep = int(keep.sum())
            if n_keep >= MIN_FLOW_POINTS_SOLVE and n_keep < len(keep):
                sol_tz_filt = solve_tz(prev_n[keep], curr_n[keep], dt)
                row += f" | {n_keep:>3}pt {sol_tz_filt:>7.3f}"
            else:
                # count-floor fallback: too few survive, live code keeps the ORIGINAL set
                row += f" | fallback({n_keep:>2})"
        print(row)


if __name__ == "__main__":
    if len(sys.argv) < 5:
        print(__doc__)
        sys.exit(2 if len(sys.argv) != 1 else 0)
    rep_dir, lo, hi, start = sys.argv[1], float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])
    ths = [float(x) for x in sys.argv[5:]] if len(sys.argv) > 5 else [0.3, 0.4, 0.5, 0.6, 0.7]
    replay(rep_dir, lo, hi, start, ths)
