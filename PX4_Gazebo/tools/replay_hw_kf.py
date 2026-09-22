#!/usr/bin/env python3
"""Offline reconstruction of the hw coast+freeze KF's loom (h_z) channel, replaying
_kf_step EXACTLY across the WHOLE flight (not just a terminal window -- the KF is
stateful, so any partial-window replay needs the correct running state to be
meaningful) against the raw per-frame flow solves computed by
replay_flow_solve_conditioning.py's geometry.

Continues project_20260917_visibility_predictor_residual.md's investigation: why does
IC1_rep1's KF-reported h_V_z (-0.77) end up MORE extreme than any single raw
per-frame solve in the same window (max ~-0.2)? Ruled out (checked against the
already-logged fields, not assumed): the loom R-schedule (CROSS_LOOM_R_SCHEDULE,
"Loom R Mult" logged exactly 1.00 the whole flight), the scale-rate fusion
(CROSS_SCALE_RATE_FUSE, default off), the hard loom backstop (CROSS_LOOM_ABS_MAX=20.0,
far above anything observed), and the loom innovation gate ("Loom Gate" logged 0 the
whole flight). This replays the KF's own predict+update math (_kf_step, FLOW_KF_Q=5.0,
FLOW_KF_R=0.1) directly to test whether ordinary constant-velocity KF rate-state
buildup, from a sustained same-signed run of raw-solve innovations, explains it.

REPLICATION SCOPE: same as replay_flow_solve_conditioning.py (no gyro de-rotation, no
sensor cal -- feeds the RAW, uncalibrated solve as `z` into the KF, so the reconstructed
state is in RAW units; compare its SHAPE/relative behavior against the logged h_V_z, not
its absolute calibrated value). Initializes at the first frame with a measurement
(matching _kf_step's own not-yet-initialized branch); does not replicate the coast/
freeze streak logic in full (assumes a measurement every frame, valid here -- Detection
Status is "ok" 1346/1347 frames checked for IC1_rep1).

Usage:
  replay_hw_kf.py <rep_dir> <lo_rel_s> <hi_rel_s> <start_time_s>
  (window bounds are only for the PRINTED range; the KF itself always replays from the
  start of the recording so its running state is correct)
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


def solve_full(prev_n, curr_n, dt):
    vel = (curr_n - prev_n) / dt
    A = fill_A(prev_n)
    b = vel.reshape(-1)
    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    return sol   # (6,)


def kf_step_scalar_channel(val, rate, P, prev_t, initialized, z_i, t, q, r):
    """Single-channel (value, rate) constant-velocity KF -- ports _kf_step's math for
    ONE scalar channel (here, always channel 2 / loom) so this file has no dependency
    on cross_marker_perception.py's class internals."""
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
    fp = im["Flow Points Prev Px"]
    fc = im["Flow Points Curr Px"]
    q = im["Quat"]
    h_v = np.asarray(im["h_V"], float)
    n = min(len(T), len(fp), len(fc), len(q), len(h_v))

    val, rate, P, prev_t, initialized = 0.0, 0.0, np.eye(2), None, False
    rows = []
    for i in range(n):
        p, c = fp[i], fc[i]
        if p is None or c is None:
            continue
        p = np.asarray(p, float); c = np.asarray(c, float)
        if p.shape != c.shape or len(p) < 4:
            continue
        dt = T[i] - T[i - 1] if i > 0 else np.nan
        if not (dt > 0):
            continue
        qprev = q[i - 1] if i > 0 else q[i]
        prev_n, _ = get_virtual_pts(p, qprev)
        curr_n, _ = get_virtual_pts(c, q[i])
        sol = solve_full(prev_n, curr_n, dt)
        z_loom = float(sol[2])
        val, rate, P, prev_t, initialized = kf_step_scalar_channel(
            val, rate, P, prev_t, initialized, z_loom, T[i], FLOW_KF_Q, FLOW_KF_R)
        rows.append((T[i] - start_time, z_loom, val, rate, float(h_v[i, 2])))

    print(f"{'t_rel':>7} {'raw_solve':>10} {'kf_recon_val':>13} {'kf_recon_rate':>14} {'h_V_z(logged)':>14}")
    for t_rel, z_loom, v, rt, logged in rows:
        if lo_rel <= t_rel <= hi_rel:
            print(f"{t_rel:>7.3f} {z_loom:>10.3f} {v:>13.3f} {rt:>14.3f} {logged:>14.3f}")


if __name__ == "__main__":
    if len(sys.argv) != 5:
        print(__doc__)
        sys.exit(2 if len(sys.argv) != 1 else 0)
    rep_dir, lo, hi, start = sys.argv[1], float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])
    replay(rep_dir, lo, hi, start)
