#!/usr/bin/env python3
"""GROUND-TRUTH yaw analysis — use GT yaw, NOT e_a, to evaluate yaw OUTCOME.

`e_a` (= alpha - alpha_d, controller.py:_yawCtrl) is the controller's INPUT — the
image-derived yaw error. It is unreliable as an *outcome* metric in BOTH directions:
  - it OVER-reports (the marker filling the FoV near touchdown corrupts alpha -> e_a
    jumps to ±150-180° while the drone is actually HELD), and
  - it UNDER-reports (a real GT yaw divergence can read small in e_a).
So to judge whether the yaw CONVERGED / OVERSHOT / SETTLED, read GT yaw, not e_a.
(2026-06-08; see memory feedback_use_gt_yaw_not_ea.)

GT-yaw error is measured against the BOARD-SQUARE reference: the median GT yaw over
clean windows (|e_a|<15° AND s_e_n<0.5 AND non-terminal) across the given reps — i.e.
where the drone is provably aligned with good perception. Pass several reps from the
same world so the board reference is stable.

Usage: gt_yaw_analysis.py <rep_dir> [<rep_dir> ...]
"""
import sys, os, math, numpy as np

def _gtyaw(uav):
    def y(p):
        q = p.orientation
        return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
    return np.unwrap([y(p) for p in uav])

def _load(d):
    cd = np.load(os.path.join(d, "Control_Data.npy"), allow_pickle=True).item()
    gt = np.load(os.path.join(d, "Ground_Truth.npy"), allow_pickle=True).item()
    t = np.asarray(cd["t"], float); t = t - t[0]; m = len(t)
    ea = np.rad2deg(np.asarray(cd["e_a(t)"], float).ravel())[:m]
    sen = np.abs(np.asarray(cd["s_e_n(t)"], float)[:m]).max(axis=1)
    gy = _gtyaw(gt.get("UAV Pose", []))
    gyi = np.interp(t, np.linspace(0, t[-1], len(gy)), gy)   # GT yaw on the control clock
    return t, ea, sen, gyi

def board_square(reps):
    sq = []
    for d in reps:
        try:
            t, ea, sen, gyi = _load(d)
            good = (np.abs(ea) < 15) & (sen < 0.5) & (t < 0.8*t[-1])
            if good.sum() > 5:
                sq.append(np.median(gyi[good]))
        except Exception:
            pass
    return np.median(sq) if sq else 0.0

def analyze(d, board):
    t, ea, sen, gyi = _load(d)
    ge = np.rad2deg(gyi) - np.rad2deg(board)         # GT yaw error vs board-square
    ss = float(np.mean(np.abs(ge[t > t[-1]-1.0])))   # steady-state: mean|err| last 1s
    name = os.path.basename(os.path.dirname(os.path.dirname(d))) + "/" + os.path.basename(d)
    real_div = abs(ge[-1]) > 45
    print(f"  {name}: GTyaw_err {ge[0]:+.0f}->min{ge.min():+.0f}/max{ge.max():+.0f}->END{ge[-1]:+.0f} "
          f"SS(last1s)={ss:.0f}deg {'REAL-DIVERGE' if real_div else 'converged'}  (e_a END was {ea[-1]:+.0f} — ignore)")
    return ss, ge[-1]

if __name__ == "__main__":
    reps = sys.argv[1:]
    b = board_square(reps)
    print(f"board-square GT yaw = {math.degrees(b):+.1f} deg (from clean windows of {len(reps)} reps)")
    for d in reps:
        try: analyze(d, b)
        except Exception as e: print(f"  {d}: [error] {e}")
