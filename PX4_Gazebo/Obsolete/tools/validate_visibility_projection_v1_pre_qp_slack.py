#!/usr/bin/env python3
"""Independent validation of src/visibility_projection.py.

Every check drives the REAL functions and scores them against a hand-rolled
pinhole + attitude camera model that shares no code with the module.

TIER 1 (visibility_project):
  1. BARRIER        -- after projection, the marker centre re-projected through the
                       real camera at the resulting attitude is inside phi.
  2. MIN-INTERVENE  -- an already-safe command is returned byte-identical.
  3. INWARD-FREE    -- a command that leans TOWARD an edge marker is not clipped.
  4. CONVENTIONS    -- lean<->accel and the camera-mount swap round-trip.
  5. PASSTHROUGH    -- marker None / degenerate a_z return the command unchanged.
  6. IDEMPOTENT     -- projecting an already-projected command is a no-op.

TIER 2 (descent_ease):
  7. ONE-WAY        -- never speeds a descent, never reverses one, floors at g_min.
  8. TRIGGERED-BY-CLOSING -- eases only when |c| is genuinely approaching the edge.
  9. SELF-RELEASE   -- as |c| shrinks, g_z returns to 1 (no latch).
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
from visibility_projection import (visibility_project, descent_ease,  # noqa: E402
                                   marker_tangent, fov_limit)

RNG = np.random.default_rng(0)
CENTER = np.array([160.0, 120.0])
FOCAL = np.array([135.0, 135.0])
G = 9.81
PHI = fov_limit(CENTER, FOCAL, 0.15)
SENSOR = CENTER / FOCAL          # the ACTUAL FoV edge (phi = SENSOR * (1 - buffer))
_R = []


def _rec(name, ok, detail=""):
    _R.append(ok)
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f"  -- {detail}" if detail else ""))


def Rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def _unswap(v):
    return np.array([-v[1], v[0]])


def project_centre(R, cam_pos, marker_pos):
    b = R.T @ (np.asarray(marker_pos, float) - np.asarray(cam_pos, float))
    if b[2] <= 1e-6:
        return np.array([1e9, 1e9])
    return np.array([b[1] / b[2], -(b[0] / b[2])])           # == _swap([X/Z, Y/Z])


def centre_to_px(c_img):
    return _unswap(np.asarray(c_img, float)) * FOCAL + CENTER


def R_from_lean(y_img, yaw):
    cz, sz = np.cos(yaw), np.sin(yaw)
    P = np.array([[0.0, -1.0], [1.0, 0.0]]) @ np.array([[cz, sz], [-sz, cz]])

    def lean_of(rpy):
        r, p = rpy
        Rm = (Rz(yaw)
              @ np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
              @ np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]]))
        return P @ (-Rm[:2, 2] / Rm[2, 2]), Rm

    rpy = np.array([0.0, 0.0])
    for _ in range(40):
        cur, Rm = lean_of(rpy)
        err = cur - y_img
        if np.linalg.norm(err) < 1e-12:
            break
        J = np.zeros((2, 2))
        for i in range(2):
            d = np.zeros(2); d[i] = 1e-6
            J[:, i] = (lean_of(rpy + d)[0] - cur) / 1e-6
        rpy = rpy - np.linalg.solve(J, err)
    return lean_of(rpy)[1]


def _scene():
    yaw = RNG.uniform(-np.pi, np.pi)
    alt = RNG.uniform(0.5, 8.0)
    off = RNG.uniform(-0.7, 0.7, 2) * alt
    return yaw, np.array([off[0], off[1], -alt]), np.array([0.0, 0.0, 0.0]), alt


# ---- TIER 1 ---------------------------------------------------------------
def test_barrier():
    """Barrier must hold for the per-cycle lean CHANGE the loop actually produces.
    At 50 Hz with a smooth outer loop that is small; test up to ~15deg/cycle and
    require ZERO true-sensor exits. (~24deg/cycle, only a transient, is reported.)"""
    for dymax, tag, allow in [(0.27, "~15deg/cycle", 0), (0.45, "~24deg/cycle", 30)]:
        worst, nbad, N = 0.0, 0, 0
        for _ in range(4000):
            yaw, cam, marker, _ = _scene()
            y0 = RNG.uniform(-0.15, 0.15, 2)
            R0 = R_from_lean(y0, yaw)
            c0 = project_centre(R0, cam, marker)
            if np.any(np.abs(c0) > PHI):
                continue
            N += 1
            cz, sz = np.cos(yaw), np.sin(yaw)
            P = np.array([[0.0, -1.0], [1.0, 0.0]]) @ np.array([[cz, sz], [-sz, cz]])
            dy = RNG.uniform(-1.0, 1.0, 2)
            dy *= min(1.0, dymax / (np.linalg.norm(dy) + 1e-9))
            a_z = RNG.uniform(6.0, 12.0)
            a_xy = a_z * (P.T @ (y0 + dy))
            a_s, y_s, info = visibility_project(np.array([a_xy[0], a_xy[1], -a_z]),
                                                R0, yaw, centre_to_px(c0), CENTER, FOCAL)
            c_true = project_centre(R_from_lean(y_s, yaw), cam, marker)
            over = float(np.max(np.abs(c_true) - SENSOR))     # vs the ACTUAL sensor edge
            if over > 1e-6:
                nbad += 1
            worst = max(worst, over)
        _rec(f"1. barrier holds ({tag}, vs the true sensor edge)",
             nbad <= allow, f"{nbad}/{N} true-sensor exits, worst {worst:+.3f} tangent")


def test_min_intervention():
    worst = 0.0
    for _ in range(3000):
        yaw, cam, marker, _ = _scene()
        R0 = R_from_lean(RNG.uniform(-0.1, 0.1, 2), yaw)
        c0 = project_centre(R0, cam, marker)
        if np.any(np.abs(c0) > 0.35 * PHI):
            continue
        a_z = RNG.uniform(7.0, 11.0)
        a_d = np.array([RNG.uniform(-0.08, 0.08) * a_z, RNG.uniform(-0.08, 0.08) * a_z, -a_z])
        a_s, y_s, info = visibility_project(a_d, R0, yaw, centre_to_px(c0), CENTER, FOCAL)
        worst = max(worst, float(np.max(np.abs(a_s - a_d))))
        if info["active"]:
            worst = 1e9
    _rec("2. minimal intervention (safe command unchanged)", worst < 1e-9,
         f"max |a_star - a_d| = {worst:.1e}")


def test_inward_free():
    strangled, n = 0, 0
    for _ in range(3000):
        yaw, cam, marker, _ = _scene()
        R0 = R_from_lean(RNG.uniform(-0.1, 0.1, 2), yaw)
        c0 = project_centre(R0, cam, marker)
        if np.max(np.abs(c0) / PHI) < 0.85 or np.any(np.abs(c0) > PHI):
            continue
        n += 1
        cz, sz = np.cos(yaw), np.sin(yaw)
        P = np.array([[0.0, -1.0], [1.0, 0.0]]) @ np.array([[cz, sz], [-sz, cz]])
        a_z = RNG.uniform(7.0, 11.0)
        a_xy = a_z * (P.T @ np.clip(0.9 * c0, -0.4, 0.4))   # lean toward the marker (bounded)
        a_s, y_s, info = visibility_project(np.array([a_xy[0], a_xy[1], -a_z]),
                                            R0, yaw, centre_to_px(c0), CENTER, FOCAL)
        if np.max(np.abs(a_s[:2] - a_xy)) > 1e-6:
            strangled += 1
    _rec("3. inward-free (leaning toward an edge marker is not clipped)",
         strangled == 0, f"{strangled}/{n} edge cases clipped an inward command")


def test_conventions():
    worst = 0.0
    for _ in range(5000):
        yaw = RNG.uniform(-np.pi, np.pi); a_z = RNG.uniform(4, 12)
        yv = RNG.uniform(-0.6, 0.6, 2)
        cz, sz = np.cos(yaw), np.sin(yaw)
        P = np.array([[0.0, -1.0], [1.0, 0.0]]) @ np.array([[cz, sz], [-sz, cz]])
        worst = max(worst, np.max(np.abs(P @ ((a_z * (P.T @ yv)) / a_z) - yv)))
        px = RNG.uniform(0, 320, 2)
        worst = max(worst, np.max(np.abs(
            centre_to_px(marker_tangent(px, CENTER, FOCAL)) - px)))
    _rec("4. conventions (lean<->accel and swap round-trip)", worst < 1e-9,
         f"max |Delta| = {worst:.1e}")


def test_passthrough():
    yaw, cam, marker, _ = _scene()
    R0 = R_from_lean(np.array([0.05, -0.03]), yaw)
    a_d = np.array([2.0, -1.0, -9.0])
    a1, y1, i1 = visibility_project(a_d, R0, yaw, None, CENTER, FOCAL)
    px = centre_to_px(project_centre(R0, cam, marker))
    a2, y2, i2 = visibility_project(np.array([2.0, -1.0, -0.1]), R0, yaw, px, CENTER, FOCAL)
    ok = ((not i1["active"]) and np.array_equal(a1, a_d)
          and (not i2["active"]) and np.allclose(a2, [2.0, -1.0, -0.1]))
    _rec("5. passthrough (marker None / degenerate a_z)", ok,
         f"None active={i1['active']}, a_z=0.1 active={i2['active']}")


def test_idempotent():
    worst = 0.0
    for _ in range(2000):
        yaw, cam, marker, _ = _scene()
        R0 = R_from_lean(RNG.uniform(-0.15, 0.15, 2), yaw)
        c0 = project_centre(R0, cam, marker)
        if np.any(np.abs(c0) > PHI):
            continue
        px = centre_to_px(c0); a_z = RNG.uniform(6, 12)
        a_d = np.array([RNG.uniform(-1.4, 1.4) * a_z, RNG.uniform(-1.4, 1.4) * a_z, -a_z])
        a1, y1, _i = visibility_project(a_d, R0, yaw, px, CENTER, FOCAL)
        a2, y2, _j = visibility_project(a1, R0, yaw, px, CENTER, FOCAL)
        worst = max(worst, float(np.max(np.abs(a2 - a1))))
    _rec("6. idempotent (re-projecting a projected command is a no-op)",
         worst < 3e-2, f"max |a2 - a1| = {worst:.1e}")


# ---- TIER 2 ------------------------------------------------------------------
def test_descent_oneway():
    bad = 0
    for _ in range(4000):
        a_z = RNG.uniform(-14.0, -4.0)                 # any vertical command
        c = RNG.uniform(-1.0, 1.0, 2) * PHI
        c_rate = RNG.uniform(-2.0, 2.0, 2)
        az_s, gz, _s = descent_ease(a_z, c, PHI, c_rate, G, g_min=0.2, dt=0.02, state={"gz": RNG.uniform(0.2, 1)})
        w0, w1 = a_z + G, az_s + G
        if w0 <= 0:                                     # was decelerating/climbing
            if abs(az_s - a_z) > 1e-9:
                bad += 1
        else:                                          # was descending
            if w1 > w0 + 1e-9:      bad += 1           # sped up  -> bad
            if w1 < -1e-9:          bad += 1           # reversed -> bad
            if w1 < 0.2 * w0 - 1e-6:  bad += 1        # below the g_min floor -> bad
    _rec("7. descent ease is one-way (never faster, never reversed, floored)",
         bad == 0, f"{bad}/4000 violations")


def test_descent_triggered():
    # centred, no closing -> g_z stays ~1 ; near edge and closing -> g_z drops
    st_far = {"gz": 1.0}; st_near = {"gz": 1.0}
    for _ in range(40):
        _a, gz_far, st_far = descent_ease(-4.0, np.array([0.0, 0.0]), PHI,
                                          np.array([0.0, 0.0]), G, dt=0.02, state=st_far)
        _b, gz_near, st_near = descent_ease(-4.0, 0.92 * PHI, PHI, np.array([0.4, 0.4]),
                                            G, dt=0.02, t_react=1.5, state=st_near)
    ok = gz_far > 0.98 and gz_near < 0.5
    _rec("8. triggered by CLOSING, not just proximity", ok,
         f"g_z centred/still={gz_far:.2f}, near-edge/closing={gz_near:.2f}")


def test_descent_selfrelease():
    # start near the edge & closing (g_z drops), then |c| shrinks -> g_z must recover to ~1
    st = {"gz": 1.0}
    c = 0.9 * PHI.copy()
    for k in range(80):
        c_rate = np.array([0.3, 0.3]) if k < 20 else -0.05 * c   # closing, then re-centring
        _a, gz, st = descent_ease(-4.0, c, PHI, c_rate, G, dt=0.02, t_react=1.5, state=st)
        c = np.clip(c + c_rate * 0.02, -PHI, PHI)
        if k == 25:
            gz_mid = gz
    ok = gz_mid < 0.7 and gz > 0.95
    _rec("9. self-releases as |c| shrinks (no latch)", ok,
         f"g_z during closing={gz_mid:.2f} -> after re-centring={gz:.2f}")


def main():
    print("=" * 70)
    print("visibility_projection.py -- independent validation")
    print("=" * 70)
    for t in (test_barrier, test_min_intervention, test_inward_free, test_conventions,
              test_passthrough, test_idempotent, test_descent_oneway,
              test_descent_triggered, test_descent_selfrelease):
        t()
    nf = _R.count(False)
    print("-" * 70)
    print(f"{len(_R) - nf}/{len(_R)} checks passed")
    sys.exit(1 if nf else 0)


if __name__ == "__main__":
    main()
