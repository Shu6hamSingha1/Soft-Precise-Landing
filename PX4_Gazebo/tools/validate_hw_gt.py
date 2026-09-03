#!/usr/bin/env python3
"""GT scoring of the FULL 6-DOF flow solve [h; w] from the background-texture
vector field (2026-09-03).

Answers what `validate_bgflow_corr.py` does NOT: that tool scores only the
TRANSLATION block (`sol[:3]` = h) and only by Pearson r. This one scores h AND w,
with SCALE (regression slope) and scale-fitted R^2 alongside r, per altitude band,
and reports the conditioning of both the full 6-unknown and the reduced 4-unknown
design matrices -- so "is w observable from this field?" is answered by the data
rather than by the 2026-08-08 aliasing argument, which was made when the only
available points sat on the small marker plate. A textured background spreads
points over the whole FoV, which is exactly the condition that argument assumed
away.

WHAT IS SCORED
  measured : Img_Data['h_V'] -- the live pre-sensor-cal 6-vector [Tx,Ty,Tz,Wx,Wy,Wz],
             one row per frame. ⚠ This is the KF-FILTERED value, NOT the raw solve:
             _log_frame_data logs self._hw, which _kf_update_hw has just written
             (cross_marker_perception.py ~2088-2098). getLogData's own "raw, before
             cal" comment means pre-CALIBRATION, not pre-filter. It is the right
             thing to score -- it is what the controller consumes -- but do not read
             these numbers as per-frame flow-solve quality; the KF contributes a
             large part of them (measured on `base`: raw per-frame h_x r=+0.73 vs
             +0.96 logged).
             NOTE: when the gyro is plumbed, cols 3:5 are the GYRO passed THROUGH
             (reduced solve), not flow-derived. The tool prints which mode each
             recording used (IMU AngVel all-NaN => full 6-unknown flow solve, so
             every column really is from the vector field).
  reference: h -> compute_gt_flow()['V_h_g']   (V-frame, depth-normalised)
             w -> V-frame body rate from the GT UAV attitude sequence:
                  omega_body = vee(Ru^T dRu/dt) with Ru = body-FRD -> NED, then the
                  SAME gravity-levelled basis _vframe_w builds (g = Ru^T e_z).

The raw solve is PRE-CAL, so absolute error is meaningless here -- the slope IS the
calibration the data is asking for. r and scale-fitted R^2 are what say whether the
signal is present at all. _fill_A's sign convention makes the expected slope
NEGATIVE for a correctly-observed component; |slope| is the scale.

TIME ALIGNMENT: Img_Data['Time'] - Ground_Truth['Start Time'] onto t_g, and frames
outside [t_g[0], t_g[-1]] are DROPPED, never np.interp-clamped
(feedback_recurring_analysis_mistakes class 1).

USAGE
  python3 tools/validate_hw_gt.py <run_dir> [<run_dir> ...]
  python3 tools/validate_hw_gt.py --set test_data/RobustnessFrameset
"""
import sys, os, argparse, glob
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src'))
import numpy as np
from ahrs import Quaternion
from scipy.signal import savgol_filter as sgf

from gt_optical_flow import compute_gt_flow, NED_FROM_ENU, FRD_2_FLU

LAB6 = ['h_x', 'h_y', 'h_z', 'w_x', 'w_y', 'w_z']
ALT_BANDS = [(4.0, 6.0), (3.0, 4.0), (2.0, 3.0), (1.3, 2.0), (0.7, 1.3)]
MIN_N = 15


# --------------------------------------------------------------------- GT w --
def gt_w_V(rep_dir):
    """V-frame angular velocity from the GT attitude sequence.

    omega_body = vee(Ru^T dRu/dt) is exact for a rotation sequence; the derivative
    is taken on a UNIFORM timebase with a savgol pass, per gt_optical_flow's rule 2
    (differentiating raw jittery GT is the documented way to manufacture noise).
    Returns (t_g, w_V (n,3)) on the same axis compute_gt_flow uses."""
    gt = np.load(os.path.join(rep_dir, "Ground_Truth.npy"), allow_pickle=True).item()
    tg = np.asarray(gt['Time'], float)
    u = gt['UAV Pose']
    n = min(len(tg), len(u))
    tg, u = tg[:n], u[:n]
    keep = np.hstack(([True], np.diff(tg) > 1e-6))
    tg = tg[keep]; u = [u[i] for i in range(n) if keep[i]]
    n = len(tg)

    Ru = np.zeros((n, 3, 3))
    for i, p in enumerate(u):
        Rfu = Quaternion([p.orientation.w, p.orientation.x,
                          p.orientation.y, p.orientation.z]).to_DCM()
        Ru[i] = NED_FROM_ENU @ Rfu @ FRD_2_FLU        # body-FRD -> NED

    # uniform timebase -> smooth -> differentiate -> back (gt_optical_flow rule 2)
    tu = np.linspace(tg[0], tg[-1], n)
    Rf = Ru.reshape(n, 9)
    Rf_u = np.column_stack([np.interp(tu, tg, Rf[:, k]) for k in range(9)])
    win = min(31, (n // 2) * 2 - 1)
    if win >= 5:
        Rf_u = sgf(Rf_u, win, 2, axis=0)
    dt_u = tu[1] - tu[0]
    dRf_u = np.gradient(Rf_u, dt_u, axis=0)

    w_V = np.full((n, 3), np.nan)
    for i in range(n):
        R = Rf_u[i].reshape(3, 3)
        dR = dRf_u[i].reshape(3, 3)
        S = R.T @ dR                                   # skew(omega_body)
        w_b = np.array([S[2, 1] - S[1, 2], S[0, 2] - S[2, 0], S[1, 0] - S[0, 1]]) * 0.5
        # same gravity-levelled basis as CrossMarkerPerception._vframe_w
        g = R.T @ np.array([0., 0., 1.])
        z_axis = g / np.linalg.norm(g)
        x_axis = np.cross([0., 1., 0.], z_axis); x_axis /= np.linalg.norm(x_axis)
        y_axis = np.cross(z_axis, x_axis)
        C_R_V = np.column_stack([x_axis, y_axis, z_axis])
        w_V[i] = C_R_V.T @ w_b
    # resample the uniform-grid result back onto tg
    return tg, np.column_stack([np.interp(tg, tu, w_V[:, k]) for k in range(3)])


# ------------------------------------------------------------------ metrics --
def _score(meas, ref):
    """(r, slope, R^2 of the scale+offset fit, n). Slope is meas = a*ref + b."""
    m = np.isfinite(meas) & np.isfinite(ref)
    if m.sum() < MIN_N or np.std(ref[m]) < 1e-9 or np.std(meas[m]) < 1e-9:
        return np.nan, np.nan, np.nan, int(m.sum())
    x, y = ref[m], meas[m]
    r = float(np.corrcoef(x, y)[0, 1])
    a, b = np.polyfit(x, y, 1)
    resid = y - (a * x + b)
    r2 = 1.0 - float(np.var(resid) / np.var(y))
    return r, float(a), r2, int(m.sum())


def _report(tag, meas, ref, alt, n_dropped):
    print(f"\n=== {tag} ===   n={len(meas)} frames scored"
          + (f"  ({n_dropped} dropped outside the GT window)" if n_dropped else ""))
    print(f"  {'':6s} {'r':>7s} {'slope':>8s} {'R2':>7s} {'GTstd':>8s} {'MEASstd':>8s} {'n':>5s}    "
          "(slope = measured per unit GT; sign is _fill_A's convention)")
    for k in range(6):
        r, a, r2, n = _score(meas[:, k], ref[:, k])
        # GT std is the EXCITATION check: a near-zero r against a near-zero-variance
        # reference says nothing about observability (recurring-mistakes class 6 --
        # a metric without its base rate). Report both so the two cases are separable.
        gs, ms = float(np.nanstd(ref[:, k])), float(np.nanstd(meas[:, k]))
        flag = ''
        if np.isfinite(r):
            flag = '  <-- STRONG' if abs(r) >= 0.8 else ('  <-- weak' if abs(r) < 0.4 else '')
        print(f"  {LAB6[k]:6s} {r:+7.3f} {a:+8.3f} {r2:+7.3f} {gs:8.4f} {ms:8.4f} {n:5d}{flag}")

    print("  per altitude band (|r|):")
    hdr = "  " + "band        " + "".join(f"{l:>8s}" for l in LAB6) + f"{'n':>7s}"
    print(hdr)
    for lo, hi in ALT_BANDS:
        m = (alt >= lo) & (alt < hi)
        if m.sum() < MIN_N:
            continue
        cells = []
        for k in range(6):
            r, _, _, _ = _score(meas[m, k], ref[m, k])
            cells.append(f"{r:+8.3f}" if np.isfinite(r) else f"{'--':>8s}")
        print(f"  {lo:.1f}-{hi:.1f} m  " + "".join(cells) + f"{int(m.sum()):7d}")


# ------------------------------------------------------------------- driver --
def run(rep_dir):
    name = os.path.basename(os.path.normpath(rep_dir))
    g = compute_gt_flow(rep_dir)
    tg = g['t_g']
    _, w_ref = gt_w_V(rep_dir)
    n = min(len(tg), len(w_ref), len(g['V_h_g']))
    tg, w_ref, h_ref = tg[:n], w_ref[:n], g['V_h_g'][:n]
    alt_g = np.abs(g['alt'][:n])

    img = np.load(os.path.join(rep_dir, "Img_Data.npy"), allow_pickle=True).item()
    t_img = np.asarray(img['Time'], float) - g['start_time']
    hv = np.asarray(img['h_V'], float)
    m = min(len(t_img), len(hv))
    t_img, hv = t_img[:m], hv[:m]

    gyro = np.asarray(img['IMU AngVel'], float)[:m]
    gyro_live = bool(np.isfinite(gyro).all(axis=1).any())
    mode = ("REDUCED 4-unknown + gyro passthrough (w_x,w_y are NOT flow-derived)"
            if gyro_live else "FULL 6-unknown flow solve (every column is from the vector field)")

    vis = np.asarray(img['FEATURE_IS_VISIBLE'], bool)[:m] if 'FEATURE_IS_VISIBLE' in img \
        else np.ones(m, bool)
    health = np.asarray(img['BgFlow Health'], float)[:m] if 'BgFlow Health' in img else None

    # STRICT GT window -- drop, never clamp (recurring-mistakes class 1)
    inwin = (t_img >= tg[0]) & (t_img <= tg[-1])
    sel = inwin & vis & np.isfinite(hv).all(axis=1)
    n_drop = int((~inwin).sum())

    ref = np.column_stack([np.interp(t_img[sel], tg, h_ref[:, k]) for k in range(3)]
                          + [np.interp(t_img[sel], tg, w_ref[:, k]) for k in range(3)])
    alt = np.interp(t_img[sel], tg, alt_g)

    hdr = f"{name}: {mode}"
    if health is not None:
        hh = health[sel]
        hdr += (f"\n  bg-flow health: rel_resid median={np.nanmedian(hh[:, 0]):.3f}"
                f"  n_pts median={np.nanmedian(hh[:, 1]):.0f}"
                f"  min={np.nanmin(hh[:, 1]):.0f}")
    _report(hdr, hv[sel], ref, alt, n_drop)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('runs', nargs='*')
    ap.add_argument('--set', dest='set_dir', default=None,
                    help='directory of variant subdirs, each a run dir')
    a = ap.parse_args()
    runs = list(a.runs)
    if a.set_dir:
        runs += sorted(d for d in glob.glob(os.path.join(a.set_dir, '*'))
                       if os.path.exists(os.path.join(d, 'Ground_Truth.npy')))
    if not runs:
        ap.error('no run dirs')
    for r in runs:
        try:
            run(r)
        except Exception as e:
            print(f"\n=== {r} ===  FAILED: {type(e).__name__}: {e}")


if __name__ == '__main__':
    main()
