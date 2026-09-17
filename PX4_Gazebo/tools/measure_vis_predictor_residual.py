#!/usr/bin/env python3
"""Measure the visibility CBF's OWN one-step predictor against realized centre motion.

This is the tool behind project_20260917_visibility_predictor_residual: it answers
"how well does c_next = r~ + L_e*(y - y_now) [+ tau*d] match where the marker centre
actually went", which is assumption (ii) of the forward-invariance argument and the
thing the buffer b must absorb.

Three predictors are compared at each horizon:
  null   : c_{k+H} = c_k                      (no model -- the honest baseline)
  rot    : c_k + L_e @ (y_now_{k+H} - y_now_k)   (what the QP uses; depth-free)
  drift  : rot + h_xy*dt                      (adds the translational term)

Reporting `null` is the point: a predictor that does not beat it is adding noise, and
at a 1-step horizon this one does not (dy ~ 0, so L_e mostly amplifies attitude noise).

TIME ALIGNMENT -- the load-bearing detail.
Attitude lives in Img_Data (its own cadence, LONGER than Control_Data: 1306 vs 1168
samples on IC1_rep1), so a min-length truncation silently misaligns -- the
feedback_recurring_analysis_mistakes §1 trap. Here Quat is interpolated onto the control
clock via Img_Data["Time"] and the alignment is SELF-CHECKED by reconstructing
arccos(R33) and comparing against the independently-logged theta_current(t). The check is
printed every run; if it degrades past ~1 deg, do not trust the residuals. Frames whose
nearest image sample is more than --max-gap away are dropped.

SCOPE / KNOWN LIMITS
  * `drift` uses raw h[:2] and the actual elapsed dt, NOT the condition_drift-conditioned
    d at fixed tau -- a proxy for the implemented term, not the term itself.
  * y_now is rebuilt from the recorded attitude, matching the module's
    y_now = P(yaw) @ (-R[:2,2]/R33). yaw_c(t) is the same yaw the module was passed.
  * Open-loop: this says "how good was the prediction on the observed trajectory", not
    "what would a different parameter have done in closed loop".

Usage:
  measure_vis_predictor_residual.py 'test_data/ICValidation/20260912-040029/IC*_rep*'
  measure_vis_predictor_residual.py --horizons 1,6,15,18 '<glob>'
"""
import sys
import os
import glob
import numpy as np

CENTER = np.array([120.0, 160.0])      # rotated detection frame (240 wide x 320 tall), (cx, cy)
FOCAL = np.array([135.0, 135.0])
# AXIS ORDER: marker_tangent applies _SWAP, so c[0] = +(y-cy)/f (spans +-cy/f = 1.185) and
# c[1] = -(x-cx)/f (spans +-cx/f = 0.889). The physical half-extent in c's own axis order is
# CENTER REVERSED. src/visibility_projection.fov_limit() uses CENTER/focal unreversed, which
# transposes the box against the measurement (2026-09-17 finding); this tool uses the true
# extents so "% over buffer" is the real exceedance rate, not the transposed one.
PHI_PHYS = CENTER[::-1] / FOCAL
BUFFER_FRAC = 0.15
BUF = BUFFER_FRAC * PHI_PHYS           # per-axis margin the residual must fit inside
M = np.array([[0.0, 1.0], [-1.0, 0.0]])


def _P(yaw):
    cz, sz = np.cos(yaw), np.sin(yaw)
    return np.array([[0.0, -1.0], [1.0, 0.0]]) @ np.array([[cz, sz], [-sz, cz]])


def load_rep(d, max_gap):
    """-> (c, y_now, h_xy, t, live) on the control clock, or None."""
    try:
        c = np.load(os.path.join(d, "Control_Data.npy"), allow_pickle=True).item()
        im = np.load(os.path.join(d, "Img_Data.npy"), allow_pickle=True).item()
    except Exception:
        return None
    if "vis_c(t)" not in c or "Quat" not in im or "Time" not in im:
        return None
    t = np.asarray(c["t"], float)
    T = np.asarray(im["Time"], float)
    q = np.asarray(im["Quat"], float)
    if len(T) < 2 or len(t) < 40:
        return None
    qi = np.stack([np.interp(t, T, q[:, k]) for k in range(4)], axis=1)
    qi /= np.linalg.norm(qi, axis=1, keepdims=True)
    j = np.clip(np.searchsorted(T, t), 1, len(T) - 1)
    gap = np.minimum(np.abs(T[j] - t), np.abs(T[j - 1] - t))

    vc = np.asarray(c["vis_c(t)"], float)
    yaw = np.asarray(c["yaw_c(t)"], float)
    h = np.asarray(c["h(t)"], float)
    n = min(len(vc), len(qi), len(yaw), len(t), len(h))
    if n < 40:
        return None
    w, x, y, z = qi[:n, 0], qi[:n, 1], qi[:n, 2], qi[:n, 3]
    R13, R23 = 2 * (x * z + w * y), 2 * (y * z - w * x)
    R33 = 1 - 2 * (x * x + y * y)
    R33 = np.sign(R33) * np.maximum(np.abs(R33), 1e-3)
    y_now = np.stack([_P(yaw[i]) @ np.array([-R13[i] / R33[i], -R23[i] / R33[i]])
                      for i in range(n)])

    # alignment self-check against an independently-logged quantity
    chk = np.nan
    if "theta_current(t)" in c:
        rec = np.arccos(np.clip(R33, -1, 1))
        log = np.asarray(c["theta_current(t)"], float)[:n]
        chk = np.degrees(np.abs(rec - log))

    live = np.any(vc[:n] != 0.0, axis=1) & (gap[:n] < max_gap)
    return vc[:n], y_now, h[:n, :2], t[:n], live, chk, gap[:n]


def main(argv):
    horizons = [1, 6, 15, 18]
    max_gap = 0.04
    pats = []
    it = iter(argv)
    for a in it:
        if a == "--horizons":
            horizons = [int(v) for v in next(it).split(",")]
        elif a == "--max-gap":
            max_gap = float(next(it))
        else:
            pats.append(a)
    if not pats:
        print(__doc__)
        return 1

    reps = []
    for p in pats:
        for d in sorted(glob.glob(p)):
            if os.path.isdir(d):
                r = load_rep(d, max_gap)
                if r:
                    reps.append(r)
    if not reps:
        print("no reps with vis_c(t) + Img_Data Time/Quat matched")
        return 1

    chk = np.concatenate([r[5] for r in reps if np.ndim(r[5])])
    gap = np.concatenate([r[6] for r in reps])
    print(f"reps={len(reps)}   ALIGNMENT SELF-CHECK (theta_current): "
          f"p50={np.percentile(chk, 50):.4f} deg  p99={np.percentile(chk, 99):.4f} deg  "
          f"max={chk.max():.4f} deg")
    print(f"   nearest image-sample gap p99={1e3 * np.percentile(gap, 99):.0f} ms; "
          f"dropped {100 * np.mean(gap >= max_gap):.2f}% of frames (>={1e3 * max_gap:.0f} ms)")
    if np.percentile(chk, 99) > 1.0:
        print("   *** p99 > 1 deg -- alignment suspect, do NOT trust the residuals below ***")
    print(f"\nper-axis buffer b*R/(2f) = {BUF.round(3)}  (b={BUFFER_FRAC})\n")
    print(f"{'horizon':>8} {'pred':<6} {'n':>6} {'p50':>8} {'p95':>8} {'p99':>8} "
          f"{'p99.9':>8} {'%>buffer':>9}")
    for H in horizons:
        acc = {k: [] for k in ("null", "rot", "drift")}
        for vc, y_now, hxy, t, live, _c, _g in reps:
            n = len(vc)
            for k in range(n - H):
                if not (live[k] and live[k + H]):
                    continue
                c0, c1 = vc[k], vc[k + H]
                xx, yy = c0
                Le = -(np.array([[xx * yy, -(1 + xx * xx)],
                                 [1 + yy * yy, -xx * yy]]) @ M)
                pred = c0 + Le @ (y_now[k + H] - y_now[k])
                acc["null"].append(np.abs(c1 - c0))
                acc["rot"].append(np.abs(c1 - pred))
                acc["drift"].append(np.abs(c1 - pred - hxy[k] * (t[k + H] - t[k])))
        if not acc["rot"]:
            continue
        ms = 1e3 * np.median([t[1] - t[0] for _v, _y, _h, t, _l, _c, _g in reps]) * H
        for key in ("null", "rot", "drift"):
            a = np.array(acc[key])
            nrm = np.linalg.norm(a, axis=1)
            over = 100 * np.mean(np.any(a > BUF, axis=1))
            lab = f"{H}(~{ms:.0f}ms)" if key == "null" else ""
            print(f"{lab:>8} {key:<6} {len(nrm):>6} {np.percentile(nrm, 50):>8.4f} "
                  f"{np.percentile(nrm, 95):>8.4f} {np.percentile(nrm, 99):>8.4f} "
                  f"{np.percentile(nrm, 99.9):>8.4f} {over:>8.3f}%")
        r = np.linalg.norm(np.array(acc["rot"]), axis=1)
        dd = np.linalg.norm(np.array(acc["drift"]), axis=1)
        print(f"{'':>8} drift beats rot on {100 * np.mean(dd < r):.1f}% of frames; "
              f"mean |res| reduction {100 * (1 - dd.mean() / r.mean()):.1f}%")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
