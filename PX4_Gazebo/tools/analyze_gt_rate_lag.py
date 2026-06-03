#!/usr/bin/env python3
"""Ground-truth rate-loop lag from input-calibration recordings.

Unlike tools/aggregate_input_calibration.py (which correlates the COMMANDED
body rate against the MAVSDK-reported rate, and so folds in the telemetry
path), this derives the TRUE body angular velocity by differentiating the
Gazebo ground-truth pose quaternion (recorded in the same loop as the command,
so command and GT response share the Time grid — no cross-clock alignment).

Established 2026-06-03: the 287 ms yaw lag is REAL (GT 275 ms), not a telemetry
artifact — roll/pitch GT (40-45 ms) ≈ MAVSDK minus a small uniform telemetry
delay. Use this to A/B the yaw rate-loop bandwidth against airframe-init gain
changes (MC_YAWRATE_K/P) without the telemetry path muddying the comparison.

Usage:  analyze_gt_rate_lag.py <dir-of-recordings> [<dir2> ...]
        (each dir holds timestamped run subdirs with Ground_Truth.npy)
"""
import sys, os, glob
import numpy as np


def quat_diff_omega(q_wxyz, t):
    """Body-frame angular velocity (FLU) from a quaternion stream via
    omega = 2 * vec(q^-1 ⊗ q_dot). q_wxyz: (N,4) [w,x,y,z]; t: (N,) seconds."""
    q = q_wxyz.astype(float).copy()
    q /= np.linalg.norm(q, axis=1, keepdims=True)
    qd = np.zeros_like(q)
    qd[1:-1] = (q[2:] - q[:-2]) / (t[2:] - t[:-2])[:, None]
    qd[0] = (q[1] - q[0]) / (t[1] - t[0])
    qd[-1] = (q[-1] - q[-2]) / (t[-1] - t[-2])

    def qmul(a, b):
        w1, x1, y1, z1 = a.T
        w2, x2, y2, z2 = b.T
        return np.stack([w1*w2 - x1*x2 - y1*y2 - z1*z2,
                         w1*x2 + x1*w2 + y1*z2 - z1*y2,
                         w1*y2 - x1*z2 + y1*w2 + z1*x2,
                         w1*z2 + x1*y2 - y1*x2 + z1*w2], axis=1)
    qconj = q * np.array([1, -1, -1, -1])
    return 2 * qmul(qconj, qd)[:, 1:]


def lag_ms(cmd, resp, t, maxlag_s=0.6, dt=0.01):
    """Cross-correlation lag (ms) at which resp best matches cmd. Sign-agnostic
    (a frame sign flip flips the xcorr peak, not its location)."""
    tu = np.arange(t[0], t[-1], dt)
    c = np.interp(tu, t, cmd); r = np.interp(tu, t, resp)
    c = (c - c.mean()) / (c.std() + 1e-9)
    r = (r - r.mean()) / (r.std() + 1e-9)
    n = len(c)
    xc = np.correlate(r, c, mode='full')
    lags = np.arange(-n + 1, n) * dt
    m = np.abs(lags) <= maxlag_s
    return lags[m][np.argmax(np.abs(xc[m]))] * 1000


def analyze_dir(cal_dir):
    runs = sorted(d for d in glob.glob(os.path.join(cal_dir, '*/'))
                  if os.path.exists(os.path.join(d, 'Ground_Truth.npy')))
    rows = {0: [], 1: [], 2: []}
    for d in runs:
        gt = np.load(os.path.join(d, 'Ground_Truth.npy'), allow_pickle=True).item()
        t = np.asarray(gt['Time'], float)
        cmd = np.asarray(gt['Command'], float)
        if len(t) < 50:
            continue
        q = np.array([[p.orientation.w, p.orientation.x,
                       p.orientation.y, p.orientation.z] for p in gt['UAV Pose']])
        om = quat_diff_omega(q, t)
        for i in range(3):
            rows[i].append(lag_ms(cmd[:, i], om[:, i], t))
    return rows, len(rows[0])


def main():
    dirs = sys.argv[1:] or ['calibration_data/input']
    print(f"\n{'dir':40s} {'n':>3s} {'roll_ms':>8s} {'pitch_ms':>9s} {'yaw_ms':>8s}")
    for cal_dir in dirs:
        rows, n = analyze_dir(cal_dir)
        if n == 0:
            print(f"{os.path.basename(cal_dir.rstrip('/')):40s}  (no valid runs)")
            continue
        print(f"{os.path.basename(cal_dir.rstrip('/')):40s} {n:3d} "
              f"{np.median(rows[0]):8.0f} {np.median(rows[1]):9.0f} {np.median(rows[2]):8.0f}")


if __name__ == '__main__':
    main()
