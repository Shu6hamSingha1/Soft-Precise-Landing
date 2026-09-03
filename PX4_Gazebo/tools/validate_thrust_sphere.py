#!/usr/bin/env python3
"""Offline check for the CBF deliverability sphere (CBF_SPHERE_TRUE_THRUST, 2026-09-03).

Exercises the REAL cbf2_filter code path (not a reimplementation) both ways and asserts the
property that matters: does the returned command respect the vehicle's actual thrust limit
|I_a| <= A_CAP?

Convention (controller.py:3682-3689, and the B_T mapping at controller.py:4008-4012):
`I_a` is THRUST acceleration in NED, hover at I_a[2] = -g. So |I_a| is thrust/mass, and
|I_a + g*e3| is the VEHICLE's acceleration -- zero at hover. The legacy sphere bounds the
latter, which is not a thrust limit.

STATUS 2026-09-03: BAKED DEFAULT-ON after a matched IC2-5 SITL A/B
(test_data/ICValidation/20260903-110221 legacy vs -110618 fixed): 17 over-cap command samples
in the legacy arm (peaks 159.6%/180.5% of cap) vs ZERO with the fix, never binding in normal
flight (81-97% of cap). Landing outcomes corroborate at n=1: precise 0/4 -> 4/4.
NOTE the fix needed BOTH sites -- cbf_visibility.py's sphere AND controller.py's downstream
cap; with only the first, controller.py still passed a 15.14 m/s^2 command (111% of cap).

⚠ This offline check ALONE is not validation -- it shows the constraint bounds the right
quantity, nothing about flight. The SITL evidence above is what justified the bake.

    ~/ws/scripts/env2025/bin/python3 tools/validate_thrust_sphere.py
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
from cbf_visibility import cbf2_filter  # noqa: E402

G = 9.80
MASS, T_MAX, MARGIN = 2.114, 33.85, 0.85
A_CAP = T_MAX * MARGIN / MASS          # 13.610 m/s^2 == 1.389 g


def _scene(seed=0):
    """A benign in-frame marker so the QP takes its normal path (not the decode-fail branch)."""
    rng = np.random.default_rng(seed)
    center = np.array([120.0, 160.0])           # 320x240 rotated to 240w x 320h
    focal = np.array([135.0, 135.0])
    corners = center + rng.normal(0, 12, size=(4, 2))
    return dict(R=np.eye(3), R33=1.0, yaw_c=0.0, corners=corners, center=center,
                focal=focal, p_10=center / focal, theta_cone=np.deg2rad(43.94),
                dt_last=0.02, w_rp=np.zeros(2), radius=25.0)


def run(I_a, true_sphere, seed=0):
    s = _scene(seed)
    env = dict(os.environ)
    env["CBF_JOINT_QP"] = "1"
    env["CBF_SPHERE_TRUE_THRUST"] = "1" if true_sphere else "0"
    out, _cone, ok, _ts, _td = cbf2_filter(
        np.asarray(I_a, float).copy(), s["R"], s["R33"], s["yaw_c"], s["corners"],
        s["center"], s["focal"], s["p_10"], s["theta_cone"], s["dt_last"], s["w_rp"],
        {}, radius=s["radius"], env=env, A_CAP=A_CAP, g=G)
    return np.asarray(out, float), ok


def main() -> int:
    print(f"A_CAP = {A_CAP:.3f} m/s^2  ({A_CAP/G:.3f} g)\n")

    # 1. The headline case: a 2 g commanded climb is not deliverable (needs 19.60 > 13.61).
    Ia = [0.0, 0.0, -2 * G]
    leg, _ = run(Ia, False)
    fix, _ = run(Ia, True)
    print("2 g climb command  I_a = [0, 0, -19.60]")
    print(f"  legacy  -> |I_a| = {np.linalg.norm(leg):6.2f}   "
          f"{'ADMITTED (bug)' if np.linalg.norm(leg) > A_CAP + 1e-6 else 'bounded'}")
    print(f"  fixed   -> |I_a| = {np.linalg.norm(fix):6.2f}   "
          f"{'still over' if np.linalg.norm(fix) > A_CAP + 1e-6 else 'BOUNDED'}")

    # 2. Randomised sweep: how often does each form emit an undeliverable command?
    rng = np.random.default_rng(7)
    n = 4000
    bad_leg = bad_fix = 0
    worst_leg = worst_fix = 0.0
    for i in range(n):
        cand = [rng.normal(0, 6), rng.normal(0, 6), -G + rng.normal(0, 8)]
        if cand[2] >= 0:
            cand[2] = -3.0                       # caller's upright guard
        a, _ = run(cand, False, seed=i % 32)
        b, _ = run(cand, True, seed=i % 32)
        na, nb = np.linalg.norm(a), np.linalg.norm(b)
        worst_leg, worst_fix = max(worst_leg, na), max(worst_fix, nb)
        bad_leg += na > A_CAP + 1e-6
        bad_fix += nb > A_CAP + 1e-6
    print(f"\nrandomised sweep, n={n}: commands returned that EXCEED |I_a| <= A_CAP")
    print(f"  legacy |I_a+g*e3|<=A_CAP : {bad_leg:5d} ({100*bad_leg/n:5.1f}%)  worst |I_a| = {worst_leg:7.2f}")
    print(f"  fixed  |I_a|      <=A_CAP: {bad_fix:5d} ({100*bad_fix/n:5.1f}%)  worst |I_a| = {worst_fix:7.2f}")

    ok = bad_fix == 0 and bad_leg > 0
    print("\nRESULT:", "PASS — fix bounds thrust, legacy does not" if ok else "UNEXPECTED — inspect")
    print("⚠ Offline property check only — the SITL A/B above is what validated the bake.")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
