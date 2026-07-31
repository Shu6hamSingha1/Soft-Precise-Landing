#!/usr/bin/env python3
"""Compute the GROUND-TRUTH optical flow [h; w], loom (h_z), virtual centroid s,
and yaw — the analytically-correct reference — from a recording's Ground_Truth.

THE CANONICAL GT-FLOW METHOD (ports plotter_output_calibration.ipynb cell 6 +
plotter_landing_test.ipynb cell 24). Use THIS instead of ad-hoc gradients.

WHY THIS EXISTS — the mistakes it prevents (made 2026-06-13, see
memory reference_gt_optical_flow):
  1. DO NOT fabricate the GT time axis with np.linspace(0, t_ctrl[-1], N).
     The GT has its OWN clock (gt['Time'], 0-based relative to gt['Start Time']).
     A wrong dt mis-SCALES velocity (gave a spurious ~6 m/s descent that
     contradicted the 0.37 m/s touchdown).
  2. DO NOT differentiate raw jittery GT position. Interpolate to a UNIFORM
     timebase, savgol-smooth, gradient, interp back (bridge jitter -> velocity noise).
  3. DO NOT align GT to image/control by FRACTIONAL INDEX. Align by the shared
     reference gt['Start Time']: t_img = img['Time'] - Start Time, t_g is already
     0-based; both are then seconds-since-descent-start (skew-free).
  4. GT loom = vz/Z is computed in the GRAVITY-LEVELED V-frame, divided by the
     true depth z_V (= relative altitude), exactly like the controller's
     _getVirtualPts (the de-rotation is verified correct).

Output convention: NED world (x=N,y=E,z=D), FRD body. GT flow h = V_v_tu / z_V
(virtual image velocity); loom = h[:,2] = vz/Z (negative = descending, matches
the controller's h_d_z = h_rd sign).

Usage:
    python3 tools/gt_optical_flow.py <rep_dir>          # prints GT loom vs measured h_z
    from gt_optical_flow import compute_gt_flow
    g = compute_gt_flow(rep_dir)   # dict: t_g, V_h_g, loom, alt, B_h_g, V_w, alpha, align()
"""
import sys, os
import numpy as np
from ahrs import Quaternion
from scipy.signal import savgol_filter as sgf

NED_FROM_ENU = np.array([[0., 1., 0.], [1., 0., 0.], [0., 0., -1.]])   # self-inverse
FRD_2_FLU    = np.diag([1., -1., -1.])                                  # DCM(x=180°)


def _robust_vel(x, t):
    """d/dt of x(t) the right way: uniform-dt interp -> savgol -> gradient -> interp back.
    x: (N,3) NED position, t: (N,) the GT's OWN time. Returns (N,3) velocity."""
    n = len(t)
    tu = np.linspace(t[0], t[-1], n)
    xu = np.column_stack([np.interp(tu, t, x[:, k]) for k in range(3)])
    W = max(5, int(round(0.225 / max(np.median(np.diff(tu)), 1e-6))) | 1)   # |1 -> odd
    if W >= n: W = n - 1 if (n - 1) % 2 else n - 2
    if W >= 5:
        xu = sgf(xu, W, 2, axis=0)
    vu = np.gradient(xu, tu, axis=0)
    return np.column_stack([np.interp(t, tu, vu[:, k]) for k in range(3)])


def _v_frame(R):
    """Gravity-leveled V-frame rotation (body-FRD -> V), per the controller's
    _getVirtualPts. R = body->NED DCM."""
    g = R.T @ np.array([0., 0., 1.])          # world-down in body
    z = g / np.linalg.norm(g)
    x = np.cross([0., 1., 0.], z); x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return np.column_stack([x, y, z]).T        # body -> V


def compute_gt_flow(rep_dir):
    """Returns a dict of GT reference signals on the GT time axis t_g (0-based, s):
       t_g, alt, W_x_tu, B_h_g(3), V_h_g(3), loom(=V_h_g[:,2]), alpha(yaw),
       start_time, and align(t_other, y_other) to resample a measured signal
       (e.g. Img 'Opt Flow Fused', timestamps absolute) onto t_g."""
    gt = np.load(os.path.join(rep_dir, "Ground_Truth.npy"), allow_pickle=True).item()
    St = float(gt['Start Time'])
    tg = np.asarray(gt['Time'], float)
    u, tp = gt['UAV Pose'], gt['Target Pose']
    n = min(len(tg), len(u), len(tp))
    tg, u, tp = tg[:n], u[:n], tp[:n]
    # de-dup non-increasing timestamps (jitter)
    keep = np.hstack(([True], np.diff(tg) > 1e-6))
    tg = tg[keep]; u = [u[i] for i in range(n) if keep[i]]; tp = [tp[i] for i in range(n) if keep[i]]
    n = len(tg)

    W_x_tu = np.zeros((n, 3)); Ru = np.zeros((n, 3, 3)); yaw = np.zeros(n)
    for i in range(n):
        p, t = u[i], tp[i]
        Rfu = Quaternion([p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z]).to_DCM()
        Ru[i] = NED_FROM_ENU @ Rfu @ FRD_2_FLU                       # body-FRD -> NED
        up  = NED_FROM_ENU @ np.array([p.position.x, p.position.y, p.position.z])
        tpp = NED_FROM_ENU @ np.array([t.position.x, t.position.y, t.position.z])
        W_x_tu[i] = tpp - up                                          # target-UAV, NED
        # FIXED 2026-07-28 (ported from the analogous Pi derive_pi_cal.py fix,
        # itself validated against this repo's own gt_feedback.py:146 formula
        # -- "ry = _yaw_of(qu) - _yaw_of(qt)", PLASMC_GT_ALPHA_SIGN default 1.0,
        # i.e. no extra sign flip beyond the relative subtraction itself): was
        # UAV yaw alone, silently dropping the target's own orientation (t was
        # only used for target POSITION above, never orientation). Matches
        # analyze_calibration.py's own flagged follow-up ("4th component
        # (alpha) calibration would need target yaw computation"). Numerically
        # identical to before for a stationary/non-yawing target (the common
        # case to date); now correct for a turning target (rover phase) too.
        uav_yaw = Quaternion([p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z]).to_angles()[2]
        tgt_yaw = Quaternion([t.orientation.w, t.orientation.x, t.orientation.y, t.orientation.z]).to_angles()[2]
        rel_yaw = uav_yaw - tgt_yaw
        yaw[i] = np.arctan2(np.sin(rel_yaw), np.cos(rel_yaw))         # wrap to (-pi, pi]

    W_v_tu = _robust_vel(W_x_tu, tg)                                  # NED relative velocity
    B_h_g = np.full((n, 3), np.nan); V_h_g = np.full((n, 3), np.nan)
    V_s_g = np.full((n, 2), np.nan)                                   # GT V-frame centroid bearing
    for i in range(n):
        zB = W_x_tu[i, 2]                                             # depth (= rel altitude)
        B_x = Ru[i].T @ W_x_tu[i]                                     # NED -> body-FRD (target rel pos)
        V_x = _v_frame(Ru[i]) @ B_x                                   # body -> V (leveled)
        V_s_g[i] = [V_x[0] / (V_x[2] + 0.01), V_x[1] / (V_x[2] + 0.01)]  # bearing; 1/(z+0.01) regularized (gear-bounded depth)
        B_v = Ru[i].T @ W_v_tu[i]                                     # NED -> body-FRD
        V_v = _v_frame(Ru[i]) @ B_v                                   # body -> V (leveled)
        if abs(zB) >= 0.1:
            B_h_g[i] = B_v / (zB + 0.01)
            V_h_g[i] = V_v / (zB + 0.01)
    out = dict(t_g=tg, start_time=St, alt=W_x_tu[:, 2], W_x_tu=W_x_tu,
               B_h_g=B_h_g, V_h_g=V_h_g, loom=V_h_g[:, 2], alpha=yaw, V_s_g=V_s_g)

    def align(t_abs, y):
        """Resample a measured signal y sampled at ABSOLUTE timestamps t_abs
        onto t_g (skew-free, via Start Time)."""
        ti = np.asarray(t_abs, float) - St
        y = np.asarray(y)
        if y.ndim == 1:
            return np.interp(tg, ti, y)
        return np.column_stack([np.interp(tg, ti, y[:, k]) for k in range(y.shape[1])])
    out['align'] = align
    return out


def _main(rep_dir):
    g = compute_gt_flow(rep_dir)
    img = np.load(os.path.join(rep_dir, "Img_Data.npy"), allow_pickle=True).item()
    meas = g['align'](np.asarray(img['Time']), np.asarray(img['Opt Flow Fused'])[:, 2])  # measured loom h_z
    tg, alt, loom = g['t_g'], g['alt'], g['loom']
    print(f"rep: {rep_dir}")
    print(f"GT descent: {tg[-1]-tg[0]:.2f}s, alt {alt[0]:.1f}->{alt[-1]:.2f}m, Start={g['start_time']:.2f}")
    print(f"{'t_g':>6}{'alt':>7}{'GT_loom':>9}{'meas_h_z':>10}{'under':>8}")
    for tt in np.linspace(tg[0], tg[-1], 9):
        i = int(np.argmin(np.abs(tg - tt)))
        ur = (loom[i] - meas[i]) if np.isfinite(loom[i]) else np.nan
        print(f"{tg[i]:>6.2f}{alt[i]:>7.2f}{loom[i]:>9.2f}{meas[i]:>10.2f}{ur:>8.2f}")
    fin = np.isfinite(loom)
    print(f"\n|GT_loom| mean {np.nanmean(np.abs(loom)):.2f} | |meas h_z| mean {np.nanmean(np.abs(meas[fin])):.2f}"
          f" | corr {np.corrcoef(loom[fin], meas[fin])[0,1]:.2f}")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__); sys.exit(2)
    _main(sys.argv[1])
