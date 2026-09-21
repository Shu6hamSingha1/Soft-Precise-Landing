#!/usr/bin/env python3
"""Corrected offline reconstruction using the REDUCED 4-unknown gyro-derotated flow
solve, after resolving (2026-09-22) that gyro WAS live-available for the whole flight:
`Telemetry_Data.npy["Angular Velocity FRD"]` holds real, finite MAVSDK
AngularVelocityFrd samples throughout (the exact object `getAngVelIMU()` returns, fed
into gz_subscriber's `_angvel_deque` on every image callback -- see
`src/gz_subscriber.py:image_callback`, `angvel = self._FC.getAngVelIMU()`). So
`_solve_jacobian`'s `prev_angvel is not None and curr_angvel is not None` gate was true
essentially the whole flight, meaning the LIVE solve almost certainly used the reduced
[Tx,Ty,Tz,Wz] form, not the full 6-unknown fallback `replay_flow_solve_conditioning.py`
and `replay_hw_kf.py` used throughout this investigation.

This redoes the KF replay (see project_20260917_visibility_predictor_residual.md's
2026-09-22 "KF predict/update... ~5.6x short" entry) with the CORRECT solve, to test
whether that gap was an artifact of replicating the wrong solve path.

REPLICATION: `_getVirtualPts` + `_fill_A` (same as the other two tools) + the gyro
de-rotation block from `_solve_jacobian` exactly: `w_body = 0.5*(prev+curr angvel)`,
`w_V = _vframe_w(w_body, prev_quat)`, `b_derot = b - A[:,[3,4]]@w_V[[0,1]]`,
`A_reduced = A[:,[0,1,2,5]]`, solve for `[Tx,Ty,Tz,Wz]`. Angular velocity is aligned from
`Telemetry_Data.npy`'s `IMU Timestamp`/`Angular Velocity FRD` onto each Img_Data frame by
NEAREST timestamp (both confirmed on the same absolute clock -- IMU Timestamp range
overlaps Img_Data Time range directly, no offset needed).

Usage:
  replay_hw_kf_gyro.py <rep_dir> <lo_rel_s> <hi_rel_s> <start_time_s>
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


def v_frame_basis(quat):
    R = _dcm(quat)
    g = R.T @ np.array([0., 0., 1.])
    z_axis = g / np.linalg.norm(g)
    x_axis = np.cross([0., 1., 0.], z_axis)
    x_axis /= np.linalg.norm(x_axis)
    y_axis = np.cross(z_axis, x_axis)
    return np.column_stack([x_axis, y_axis, z_axis])  # C_R_V


def get_virtual_pts(pts, quat):
    C_R_V = v_frame_basis(quat)
    x = (pts[:, 0] - CENTER[0]) / FOCAL[0]
    y = (pts[:, 1] - CENTER[1]) / FOCAL[1]
    rays = np.column_stack([y, -x, np.ones_like(x)])
    V_rays = rays @ C_R_V
    z_v = V_rays[:, 2]
    return np.column_stack([V_rays[:, 0] / z_v, V_rays[:, 1] / z_v]), z_v


def vframe_w(w_body, quat):
    """Ports CrossMarkerPerception._vframe_w exactly."""
    C_R_V = v_frame_basis(quat)
    return C_R_V.T @ np.asarray(w_body, float)


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


def solve_reduced(prev_n, curr_n, dt, w_body, prev_quat):
    """Reduced [Tx,Ty,Tz,Wz] solve, matching _solve_jacobian's gyro-derotation branch."""
    vel = (curr_n - prev_n) / dt
    A = fill_A(prev_n)
    b = vel.reshape(-1)
    w_V = vframe_w(w_body, prev_quat)
    b_derot = b - A[:, [3, 4]] @ w_V[[0, 1]]
    A_reduced = A[:, [0, 1, 2, 5]]
    sol_reduced, *_ = np.linalg.lstsq(A_reduced, b_derot, rcond=None)
    return float(sol_reduced[2])  # Tz


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
    tel = np.load(rep_dir + "/Telemetry_Data.npy", allow_pickle=True).item()
    T = np.asarray(im["Time"], float)
    fp = im["Flow Points Prev Px"]
    fc = im["Flow Points Curr Px"]
    q = im["Quat"]
    h_v = np.asarray(im["h_V"], float)
    n = min(len(T), len(fp), len(fc), len(q), len(h_v))

    T_imu = np.asarray(tel["IMU Timestamp"], float)
    av = tel["Angular Velocity FRD"]
    # nearest-neighbor align each Img frame's absolute time onto the IMU stream
    idx_imu = np.clip(np.searchsorted(T_imu, T[:n]), 0, len(T_imu) - 1)

    def w_at(k):
        i = idx_imu[k]
        a = av[i]
        return np.array([a.forward_rad_s, a.right_rad_s, a.down_rad_s])

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
        w_body = 0.5 * (w_at(max(i - 1, 0)) + w_at(i))
        z_loom = solve_reduced(prev_n, curr_n, dt, w_body, qprev)
        val, rate, P, prev_t, initialized = kf_step_scalar_channel(
            val, rate, P, prev_t, initialized, z_loom, T[i], FLOW_KF_Q, FLOW_KF_R)
        rows.append((T[i] - start_time, z_loom, val, rate, float(h_v[i, 2])))

    print(f"{'t_rel':>7} {'raw_solve(gyro)':>15} {'kf_recon_val':>13} {'kf_recon_rate':>14} {'h_V_z(logged)':>14}")
    for t_rel, z_loom, v, rt, logged in rows:
        if lo_rel <= t_rel <= hi_rel:
            print(f"{t_rel:>7.3f} {z_loom:>15.3f} {v:>13.3f} {rt:>14.3f} {logged:>14.3f}")


if __name__ == "__main__":
    if len(sys.argv) != 5:
        print(__doc__)
        sys.exit(2 if len(sys.argv) != 1 else 0)
    rep_dir, lo, hi, start = sys.argv[1], float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])
    replay(rep_dir, lo, hi, start)
