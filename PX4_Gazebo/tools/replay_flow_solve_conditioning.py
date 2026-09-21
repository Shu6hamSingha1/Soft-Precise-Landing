#!/usr/bin/env python3
"""Offline reconstruction of the cross-marker flow solve's conditioning and per-point
grazing-ray diagnostic, against ALREADY-RECORDED point correspondences.

Built 2026-09-22 to answer: why does the perceived loom (h_z) diverge from the true
descent rate in the terminal ~150-450ms before touchdown (see
project_20260917_visibility_predictor_residual.md's 2026-09-21/22 entries)? Two
hypotheses were tested against this reconstruction:

  1. LSTSQ matrix ill-conditioning (point spread shrinking as the marker overfills the
     frame). FALSIFIED: cond(A_full) stays modest (7-16) throughout the divergence in
     both test reps -- no blow-up.
  2. Near-grazing-ray perspective-divide amplification (a mechanism
     CrossMarkerPerception's own 2026-08-02 comment predicted but never confirmed:
     "a near-zero z_v blows up the perspective divide... without tripping any of the
     existing n_kept/cond/rel_resid diagnostics"). CONFIRMED: at the exact frame the
     divergence spikes, the RAW single-frame solve jumps far more than the logged
     (KF-filtered) h_V_z does -- the KF is damping it, not causing it -- and `zv_min`
     (this script's own diagnostic) drops through the identical window, reaching its
     minimum right at the spike.

WHAT THIS REPLICATES, AND ITS LIMITS
  Faithfully reproduces `CrossMarkerPerception._getVirtualPts` (gravity-leveled V-frame
  reprojection) + `_fill_A` (the 6-DOF image Jacobian) + `np.linalg.lstsq`/`cond` on the
  FULL 6-unknown [h1,h2,h3,w1,w2,w3] system, from `Img_Data.npy`'s saved
  "Flow Points Prev/Curr Px" + "Quat".

  Does NOT replicate: the gyro de-rotation to a reduced 4-unknown [Tx,Ty,Tz,Wz] solve
  (live availability of prev/curr angvel could not be established from this recording --
  Img_Data["IMU AngVel"] is NaN throughout, but traced to a DIFFERENT, apparently-unwired
  logging path than what _solve_jacobian actually receives via getAngVels(), so this is
  NOT evidence gyro was unavailable live, just that this particular log doesn't answer
  it); the sensor calibration matrix (_sensor_cal_hw, applied downstream of the raw
  solve); or the hw coast+freeze KF that actually produces the logged h_V_z ("sol_Tz"
  printed here is the RAW single-frame solve, not the KF state -- the two differ
  substantially even in healthy frames, which is expected and was verified NOT to be a
  replication bug: the calibration matrix's z-diagonal is 0.9513, far too small to
  explain the gap, and the KF is a genuinely different, filtered quantity by design).

  So: trust `zv_min` and `cond(A_full)` as accurate reconstructions of the raw
  geometry/solve. Do NOT compare `sol_Tz` numerically against `h_V_z` expecting a match
  on healthy frames -- only their RELATIVE behavior (does sol_Tz spike at the same frame
  zv_min bottoms out, and does h_V_z track that spike with some lag/damping) is the
  valid comparison, and that comparison is what this script is for.

Usage:
  replay_flow_solve_conditioning.py <rep_dir> <lo_rel_s> <hi_rel_s> <start_time_s>

  <rep_dir>       path to a landing rep directory (containing Img_Data.npy)
  <lo_rel_s>/<hi_rel_s>  window bounds, in seconds RELATIVE TO <start_time_s>
  <start_time_s>  the absolute-clock anchor -- pass Ground_Truth.npy's "Start Time" to
                  align with Control_Data's own "t" convention (t=0 at flight start), or
                  0 to just window on Img_Data's own raw "Time" values directly.

Example (terminal 1.5s of a recorded rep, aligned to its own GT start):
  python3 - <<'EOF'
  import numpy as np
  gt = np.load("<rep_dir>/Ground_Truth.npy", allow_pickle=True).item()
  im = np.load("<rep_dir>/Img_Data.npy", allow_pickle=True).item()
  print(im["Time"][-1] - gt["Start Time"] - 1.5, im["Time"][-1] - gt["Start Time"])
  EOF
  # then: replay_flow_solve_conditioning.py <rep_dir> <lo> <hi> <gt_start_time>
"""
import sys
import numpy as np

CENTER = np.array([120., 160.])
FOCAL = np.array([135., 135.])


def _dcm(qwxyz):
    w, x, y, z = qwxyz
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def get_virtual_pts(pts, quat):
    """Replicates CrossMarkerPerception._getVirtualPts exactly (quat given, not the
    quat=None un-leveled fallback -- this recording always has a live quaternion)."""
    R = _dcm(quat)  # body->world DCM (matches Quaternion(...).to_DCM())
    g = R.T @ np.array([0., 0., 1.])
    z_axis = g / np.linalg.norm(g)
    x_axis = np.cross([0., 1., 0.], z_axis)
    x_axis /= np.linalg.norm(x_axis)
    y_axis = np.cross(z_axis, x_axis)
    C_R_V = np.column_stack([x_axis, y_axis, z_axis])
    x = (pts[:, 0] - CENTER[0]) / FOCAL[0]
    y = (pts[:, 1] - CENTER[1]) / FOCAL[1]
    rays = np.column_stack([y, -x, np.ones_like(x)])   # camera-mount [y,-x] swap, matches source
    V_rays = rays @ C_R_V
    z_v = V_rays[:, 2]
    out = np.column_stack([V_rays[:, 0] / z_v, V_rays[:, 1] / z_v])
    return out, z_v


def fill_A(centered_pts):
    """Mirrors CrossMarkerPerception's module-level _fill_A exactly: 6-DOF image
    Jacobian [h1,h2,h3,w1,w2,w3] ordering, depth-normalized (Z=1 folded into cal)."""
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


def reconstruct(rep_dir, lo_rel, hi_rel, start_time):
    im = np.load(rep_dir + "/Img_Data.npy", allow_pickle=True).item()
    T = np.asarray(im["Time"], float)
    fp = im["Flow Points Prev Px"]
    fc = im["Flow Points Curr Px"]
    q = im["Quat"]
    h_v = np.asarray(im["h_V"], float)
    lo, hi = start_time + lo_rel, start_time + hi_rel
    idx = np.where((T >= lo) & (T <= hi))[0]
    print(f"{'t_rel':>7} {'n_pts':>6} {'zv_min':>7} {'cond(A_full)':>13} "
          f"{'rel_resid':>10} {'sol_Tz(raw)':>11} {'h_V_z(KF,logged)':>17}")
    for i in idx:
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
        vel = (curr_n - prev_n) / dt
        A = fill_A(prev_n)
        b = vel.reshape(-1)
        sol, *_ = np.linalg.lstsq(A, b, rcond=None)
        cond = np.linalg.cond(A)
        b_norm = np.linalg.norm(b)
        rel_resid = np.linalg.norm(A @ sol - b) / b_norm if b_norm > 1e-12 else np.nan
        zv_min = min(zv_p.min(), zv_c.min())
        print(f"{T[i] - start_time:>7.3f} {len(p):>6} {zv_min:>7.3f} {cond:>13.2f} "
              f"{rel_resid:>10.4f} {sol[2]:>11.3f} {h_v[i, 2]:>17.3f}")


if __name__ == "__main__":
    if len(sys.argv) != 5:
        print(__doc__)
        sys.exit(2 if len(sys.argv) != 1 else 0)
    rep_dir, lo, hi, start = sys.argv[1], float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])
    reconstruct(rep_dir, lo, hi, start)
