#!/usr/bin/env python3
"""Regression guard for the CBF tilt/pixel frame consistency (2026-08-31 Rz_p90b fix).

The visibility CBF works on the ACTUAL image plane; the pixel/centroid `cr2` carries
the camera mount via the `[y,-x]` swap (body-FRD axes). The tilt `th` derived from
`I_a` must land in that SAME frame -- the prior `Rz_p90b = Rz(+90deg)` rotated it 90deg
out AND cancelled CBF_LW_ROT's M90, so for a restoring command the box model predicted
a 90deg-TANGENTIAL feature move -> the FoV projection "corrected" perpendicular to the
recovery direction -> off-center drift -> fly-off (IC5).

This checks, on the exact live cbf2_filter:
  (A) th_desired points along -cr2 (toward re-centering), not 90deg off it.
  (B) a purely-restoring I_a (I_a[:2] == -cr2) at / past the FoV edge passes through.
  (C) a dft-triggered box violation is corrected RADIALLY (output keeps its restoring
      component), not rotated tangentially.

Run: python3 tools/validate_cbf_frame.py     (exit 0 iff all pass)
Set CBF_MOUNT_ROT=1 to see the pre-fix (broken) behaviour.
"""
import os, sys
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
from cbf_visibility import cbf2_filter

C = np.array([120.0, 160.0]); F = np.array([135.0, 135.0])
P10 = np.array([160/135.0, 120/135.0]); ACAP = 13.61
R = np.eye(3); YAW = 0.0
ENV = {**os.environ, "CBF_JOINT_QP": "1", "CBF_LW_ROT": "1",
       "CBF_AZ_COST_GAIN": "0.0", "CBF_MARGIN_RESERVE": "0.0"}
_fail = []

def _ang(a, b):
    return np.degrees(np.arccos(np.clip(
        np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b) + 1e-12), -1, 1)))

# (A) th_desired vs -cr2
print("(A) angle(th_desired, -cr2)  [want ~0 = points toward re-centering]")
for dpx, dpy in [(40, 0), (0, 40), (-40, 0), (0, -40), (50, 30), (-30, 45)]:
    xn, yn = dpx / F[0], dpy / F[1]
    cr2 = np.array([yn, -xn])
    Ia = np.array([-yn, xn, 0.0]); Ia[:2] *= 2.0 / (np.linalg.norm(Ia[:2]) + 1e-12)
    Ia[2] = -9.81
    _, _, ok, _, thd = cbf2_filter(Ia.copy(), R, 1.0, YAW,
                                   np.array([[C[0] + dpx, C[1] + dpy]], float),
                                   C, F, P10, np.deg2rad(43.9), None, np.zeros(2), {},
                                   radius=10.0, env={**ENV, "CBF_TAU": "0.0"}, A_CAP=ACAP)
    a = _ang(thd, -cr2)
    ok_ = a < 20
    print(f"    px({dpx:+d},{dpy:+d}): {a:5.1f} deg   {'ok' if ok_ else 'FAIL'}")
    if not ok_:
        _fail.append(f"(A) px({dpx},{dpy}) angle {a:.0f}")

# (B) restoring I_a at/past the edge passes through
print("(B) restoring I_a (== -cr2 dir) at/past FoV edge, dft=0  [want angle(in,out) ~0]")
for xn, yn in [(1.05, 0.0), (0.0, 1.05), (1.10, 0.5), (-1.10, 0.0), (1.30, 0.7)]:
    cr2 = np.array([yn, -xn])
    Ia = np.array([-yn, xn, 0.0]); Ia[:2] *= 6.0 / (np.linalg.norm(Ia[:2]) + 1e-12)
    Ia[2] = -9.81
    out, *_ = cbf2_filter(Ia.copy(), R, 1.0, YAW,
                          np.array([[C[0] + xn * F[0], C[1] + yn * F[1]]], float),
                          C, F, P10, np.deg2rad(43.9), None, np.zeros(2), {},
                          radius=8.0, env={**ENV, "CBF_TAU": "0.0"}, A_CAP=ACAP)
    a = _ang(Ia[:2], out[:2]); ok_ = a < 15
    print(f"    cr2=({xn:+.2f},{yn:+.2f}): {a:5.1f} deg   {'ok' if ok_ else 'FAIL'}")
    if not ok_:
        _fail.append(f"(B) cr2=({xn},{yn}) angle {a:.0f}")

# (C) dft-triggered violation corrected radially
print("(C) dft toward edge, restoring I_a inside box  [want output keeps restoring sign]")
for xn, yn in [(0.7, 0.0), (0.0, 0.7), (0.6, 0.4), (0.5, 0.6)]:
    cr2 = np.array([yn, -xn])
    Ia = np.array([-yn, xn, 0.0]); Ia[:2] *= 4.0 / (np.linalg.norm(Ia[:2]) + 1e-12)
    Ia[2] = -9.81
    worst = 1e9
    for tau in ("0.15", "0.3"):
        st = {"cr_prev": cr2 - 0.02 * np.sign(cr2), "d": np.sign(cr2) * 3.0}
        out, *_ = cbf2_filter(Ia.copy(), R, 1.0, YAW,
                              np.array([[C[0] + xn * F[0], C[1] + yn * F[1]]], float),
                              C, F, P10, np.deg2rad(43.9), 0.016, np.zeros(2), st,
                              radius=8.0, env={**ENV, "CBF_TAU": tau}, A_CAP=ACAP)
        radial = np.dot(out[:2], -cr2) / np.linalg.norm(cr2)     # >0 == still restoring
        worst = min(worst, radial)
    ok_ = worst > 0
    print(f"    marker({xn:.2f},{yn:.2f}): min out.(-cr2_hat) over tau = {worst:+.2f}   {'ok' if ok_ else 'FAIL'}")
    if not ok_:
        _fail.append(f"(C) marker=({xn},{yn}) radial {worst:+.2f}")

print()
if _fail:
    print("FAIL: " + "; ".join(_fail)); sys.exit(1)
print("PASS: CBF tilt frame is consistent with the [y,-x] pixel/cr2 convention.")
