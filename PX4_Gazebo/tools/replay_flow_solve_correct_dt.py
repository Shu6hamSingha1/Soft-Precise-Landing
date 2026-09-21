#!/usr/bin/env python3
"""Offline reconstruction of the hw coast+freeze KF's loom (h_z) channel, using the
CORRECT dt: dt=1/fps (what process_frame() actually divides pixel displacement by), NOT
Time[i]-Time[i-1] (what every earlier replay tool in this thread used -- see
project_20260917_visibility_predictor_residual.md's 2026-09-22 "dt/fps" entries).

Found 2026-09-22: cross_marker_perception.py's process_frame(img_prev, img_curr, t, fps,
...) computes dt=1/fps and uses THAT for the raw flow solve's velocity -- not t-prev_t.
Img_Data["FPS"] was dead (always NaN) since the 2026-08-12 dt/frame-pairing rewrite --
self._pending_fps was read via getattr(..., default) but never assigned anywhere (fixed in
cross_marker_perception.py's process_frame(), commit a1ffbf02). In the terminal window
before touchdown, dt=1/fps can be 3-8x SMALLER than Time[i]-Time[i-1] -- the run() polling
loop's call cadence decouples from the camera's native frame-pair rate near touchdown (each
processed call's OWN frame pair stays one native interval apart, but real time -- and real
target motion -- can pass between CONSECUTIVE PROCESSED calls without the raw solve ever
seeing it). Using the too-large Time-delta dt divides displacement into an artificially
SMALL velocity, which is exactly why every prior reconstruction fell short of the logged
h_V_z spike by ~5.6x. Redoing the reconstruction with the correct dt (this script) matches
logged h_V_z to <2% even at a genuine large spike (IC1_rep3, 2026-09-22 test:
diffs 0.000-0.013 across a -0.21->-0.68 ramp). This CLOSES the reconstruction-accuracy
question this thread chased across 5 prior mechanism candidates (ill-conditioning,
near-grazing-rays, KF rate-buildup, R-schedule/scale-fuse/backstop/gate, sensor-cal) --
none of them were wrong exactly, they were just insufficient because the YARDSTICK
(the offline reconstruction) was itself off by the same dt bug in every comparison.

REQUIRES a recording taken AFTER commit a1ffbf02 (Img_Data["FPS"] must be live, not NaN --
check with `np.isnan(im["FPS"]).all()` before trusting this script's output on any given
rep; older recordings cannot be redone with this fix retroactively).

Usage:
  replay_flow_solve_correct_dt.py <rep_dir> <lo_rel_s> <hi_rel_s> <start_time_s>
  (window bounds relative to <start_time_s>; pass the rep's own last Time value to window
  on "seconds before the recording ends" -- see other replay_*.py docstrings for GT-start
  alignment if windowing on flight-relative time instead)
"""
import sys
import numpy as np

CENTER = np.array([120., 160.])
FOCAL = np.array([135., 135.])
FLOW_KF_Q = 5.0
FLOW_KF_R = 0.1


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
    return np.column_stack([V_rays[:, 0] / z_v, V_rays[:, 1] / z_v]), z_v


def fill_A(centered_pts):
    x = centered_pts[:, 0]
    y = centered_pts[:, 1]
    n = len(x)
    A = np.zeros((2 * n, 6))
    A[0::2, 0] = 1; A[1::2, 1] = 1
    A[0::2, 2] = -x; A[1::2, 2] = -y
    A[0::2, 3] = -x * y; A[1::2, 3] = -(1 + y ** 2)
    A[0::2, 4] = 1 + x ** 2; A[1::2, 4] = x * y
    A[0::2, 5] = -y; A[1::2, 5] = x
    return A


def solve_tz(prev_n, curr_n, dt):
    vel = (curr_n - prev_n) / dt
    A = fill_A(prev_n)
    b = vel.reshape(-1)
    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    return float(sol[2])


def kf_step_scalar_channel(val, rate, P, prev_t, initialized, z_i, t, q, r):
    if not initialized:
        return z_i, 0.0, np.eye(2), t, True
    dt = max(min(t - prev_t, 0.1), 1e-3)
    F = np.array([[1.0, dt], [0.0, 1.0]])
    Qm = q * np.array([[dt**4/4.0, dt**3/2.0], [dt**3/2.0, dt**2]])
    x = np.array([val, rate])
    x_pred = F @ x
    P_pred = F @ P @ F.T + Qm
    y = z_i - x_pred[0]
    S = P_pred[0, 0] + r
    K = P_pred[:, 0] / S
    x_new = x_pred + K * y
    P_new = P_pred - np.outer(K, P_pred[0, :])
    return float(x_new[0]), float(x_new[1]), P_new, t, True


def replay(rep_dir, lo_rel, hi_rel, start_time):
    im = np.load(rep_dir + "/Img_Data.npy", allow_pickle=True).item()
    T = np.asarray(im["Time"], float)
    fps = np.asarray(im["FPS"], float)
    if np.isnan(fps).all():
        print("WARNING: Img_Data['FPS'] is entirely NaN -- this recording predates "
              "commit a1ffbf02 (the dead-logging fix) and CANNOT be reconstructed with "
              "the correct dt. Re-record after that fix.", file=sys.stderr)
    fp = im["Flow Points Prev Px"]
    fc = im["Flow Points Curr Px"]
    q = im["Quat"]
    h_v = np.asarray(im["h_V"], float)
    n = min(len(T), len(fp), len(fc), len(q), len(h_v), len(fps))

    val, rate, P, prev_t, initialized = 0.0, 0.0, np.eye(2), None, False
    rows = []
    for i in range(n):
        p, c = fp[i], fc[i]
        if p is None or c is None:
            continue
        p = np.asarray(p, float); c = np.asarray(c, float)
        if p.shape != c.shape or len(p) < 4:
            continue
        dt_solve = 1.0 / fps[i] if fps[i] > 1 else np.nan
        if not (dt_solve > 0):
            continue
        qprev = q[i - 1] if i > 0 else q[i]
        prev_n, _ = get_virtual_pts(p, qprev)
        curr_n, _ = get_virtual_pts(c, q[i])
        z_loom = solve_tz(prev_n, curr_n, dt_solve)
        val, rate, P, prev_t, initialized = kf_step_scalar_channel(
            val, rate, P, prev_t, initialized, z_loom, T[i], FLOW_KF_Q, FLOW_KF_R)
        rows.append((T[i] - start_time, z_loom, val, rate, float(h_v[i, 2])))

    print(f"{'t_rel':>7} {'raw_solve(fps-dt)':>18} {'kf_recon_val':>13} "
          f"{'kf_recon_rate':>14} {'h_V_z(logged)':>14} {'|diff|':>8}")
    for t_rel, z_loom, v, rt, logged in rows:
        if lo_rel <= t_rel <= hi_rel:
            print(f"{t_rel:>7.3f} {z_loom:>18.3f} {v:>13.3f} {rt:>14.3f} "
                  f"{logged:>14.3f} {abs(v - logged):>8.3f}")


if __name__ == "__main__":
    if len(sys.argv) != 5:
        print(__doc__)
        sys.exit(2 if len(sys.argv) != 1 else 0)
    rep_dir, lo, hi, start = sys.argv[1], float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])
    replay(rep_dir, lo, hi, start)
