#!/usr/bin/env python3
"""Stage 1 — synthetic validation of the isolated cbf2 visibility CBF.

Runs the EXACT live code path (``src/cbf_visibility.cbf2_filter``) against
analytic / independent ground truth. No SITL, no recorded data. Eight checks:

  0. PARITY      — cbf2_filter reproduces the verbatim pre-refactor inline logic
                   bit-for-bit (proves the extraction did not change behavior).
  1. L_w FIDELITY— the model the QP enforces, cr + L_w@(th - th_curr), matches
                   the TRUE feature shift from an independent pinhole+attitude
                   camera model (the decisive correctness question:
                   docs/FUNNEL_CBF_DESIGN.md flags th_curr/L_w as reasoning-only).
  2. BARRIER     — after the QP the PREDICTED feature is inside the FoV box.
  3. END-TO-END  — the TRUE feature at the commanded tilt is inside the box
                   (barrier holds on real geometry, not just the linear model).
  4. MIN-INTERV. — a desired accel that is already feasible is passed through
                   unchanged (no spurious clamping).
  5. NO-STRANGLE — inward / recovery accel is left free even when large; only
                   the outward (toward-edge) component is bounded. This is the
                   property that distinguishes the CBF from the cone clamp.
  6. CONVENTIONS — Rz(-yaw)/Rz(yaw) round-trip and the a_xy<->theta map invert.
  7. PHASE-2     — decode-fail hysteresis + ramp behave (theta_cap clip moved to caller).

Run:  ~/ws/scripts/env2025/bin/python3 tools/validate_cbf.py
Exit code 0 iff every check passes.
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
from cbf_visibility import cbf2_filter   # noqa: E402

# ---- camera / controller constants (640x480 defaults, see CLAUDE.md) ----
CENTER = np.array([320.0, 240.0])
FOCAL = np.array([270.0, 270.0])
P_10 = CENTER / FOCAL                       # phi_max tangent half-extent ~[1.185, 0.889]
THETA_CAP = np.deg2rad(60.0)
G = 9.81

RNG = np.random.default_rng(0)
_RESULTS = []

# The cbf2 lean->rotation coupling fix is now default-ON (CBF_LW_ROT=1). Correctness
# checks set it explicitly; the parity check forces it OFF to match the verbatim
# pre-refactor reference.
FIX_ENV = {"CBF_LW_ROT": "1"}
M90 = np.array([[0.0, 1.0], [-1.0, 0.0]])     # omega = M90 @ theta_lean


def _record(name, ok, detail=""):
    _RESULTS.append((name, ok, detail))
    tag = "PASS" if ok else "FAIL"
    print(f"  [{tag}] {name}" + (f"  — {detail}" if detail else ""))


# ============================================================================
# Rotation + pinhole camera helpers (INDEPENDENT of the controller's math)
# ============================================================================
def Rx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def Ry(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def Rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


# CAMERA-MOUNT YAW FIX (2026-08-04, added to cbf_visibility.py; propagated into this
# oracle 2026-08-13 -- see verify_cbf_convention.py scratchpad check + memory
# project_20260813_cbf_extent_fix_followup): cbf2_filter's real camera-mount
# convention is not "camera=body-FRD aligned" -- there is an additional Rz(+90deg)
# between body/inertial axes and image axes (equivalently, a [y,-x] swap on raw
# BODY-FRD tangent to get IMAGE-frame tangent). Every helper below that produces or
# consumes an image-frame tangent/tilt must apply this, or it's silently checking a
# 90deg-rotated wrong quantity (this is exactly what made tests 2/3/5/0 fail before
# this fix -- confirmed via a standalone independent check, not just code-reading).
_SWAP = np.array([[0.0, 1.0], [-1.0, 0.0]])     # body (x,y) -> image (y,-x) role-swap
_RZ_P90B = np.array([[0.0, -1.0], [1.0, 0.0]])  # Rz(+90deg); cbf2_filter's inertial/body -> image
_RZ_M90B = np.array([[0.0, 1.0], [-1.0, 0.0]])  # Rz(-90deg); cbf2_filter's image -> inertial/body


def _swap(v):
    """body-FRD tangent (x,y) -> image tangent (y,-x); matches cbf_visibility.py's
    `ct = column_stack([ct[:,1], -ct[:,0]])`."""
    return np.array([v[1], -v[0]])


def _unswap(v):
    """Inverse of _swap: image tangent -> body-FRD tangent."""
    return np.array([-v[1], v[0]])


def project_tangent(R, cam_pos, P):
    """Downward pinhole (optical axis = body +z, FRD), IMAGE-frame tangent (after the
    camera-mount yaw swap -- this is what a marker's measured `cr` actually is)."""
    b = R.T @ (P - cam_pos)          # cam->point in body frame
    body = np.array([b[0] / b[2], b[1] / b[2]])
    return _swap(body)


def corners_from_tangent(cr, half_ext=(0.05, 0.05)):
    """Synthesize 4 raw-pixel corners whose mean IMAGE-frame tangent == cr and
    per-axis IMAGE-frame half-extent == half_ext (matches img_node._feature_pts[-1][1]
    format -- corners are stored in raw/body-FRD pixel space; cbf2_filter applies the
    swap when it reads them back, same as project_tangent does above for cr)."""
    hx_img, hy_img = half_ext
    bx, by = _unswap(cr)
    hx_body, hy_body = hy_img, hx_img   # a body-aligned box's half-extents swap axes too
    tang = np.array([[bx - hx_body, by - hy_body], [bx + hx_body, by - hy_body],
                     [bx + hx_body, by + hy_body], [bx - hx_body, by + hy_body]])
    return CENTER + FOCAL * tang     # back to pixels


def Ia_from_tilt(th_img, yaw, a_z):
    """Inverse of cbf2_filter's own forward map,
    I_a[:2] = a_z * Rz(yaw) @ Rz(-90deg) @ th (for test inputs)."""
    a_xy = a_z * (Rz(yaw)[:2, :2] @ (_RZ_M90B @ th_img))
    return np.array([a_xy[0], a_xy[1], -a_z])


def th_curr_of(R, yaw):
    """Matches cbf2_filter's own th_curr = Rz(+90deg) @ Rz(-yaw) @ (-R[:2,2]/R33)."""
    return _RZ_P90B @ (Rz(-yaw)[:2, :2] @ (-R[:2, 2] / max(abs(R[2, 2]), 1e-9)))


def R_from_image_tilt(th_img, yaw):
    """Realize a body attitude whose controller-extracted image tilt == th_img,
    NEWTON-refined (finite-difference Jacobian -- robust to the extra Rz(+90deg)
    camera-mount factor in th_curr_of, no hand-derived small-angle sign to get
    wrong) so the realized R reproduces th_img exactly under th_curr_of(R, yaw)."""
    # small-angle initial guess: th_curr_of's pre-swap quantity is ~(-pitch, roll)
    # (as in the pre-camera-mount-fix convention); the +90deg swap then maps that
    # to th_img ~= (-roll, -pitch), i.e. pitch ~= -th_img[1], roll ~= -th_img[0].
    pitch, roll = -th_img[1], -th_img[0]
    for _ in range(50):
        R = Rz(yaw) @ Ry(pitch) @ Rx(roll)
        th_cur = th_curr_of(R, yaw)
        err = th_img - th_cur
        if np.linalg.norm(err) < 1e-12:
            break
        eps = 1e-6
        Jp = (th_curr_of(Rz(yaw) @ Ry(pitch + eps) @ Rx(roll), yaw) - th_cur) / eps
        Jr = (th_curr_of(Rz(yaw) @ Ry(pitch) @ Rx(roll + eps), yaw) - th_cur) / eps
        J = np.column_stack([Jp, Jr])
        try:
            d = np.linalg.solve(J, err)
        except np.linalg.LinAlgError:
            break
        pitch += d[0]; roll += d[1]
    return Rz(yaw) @ Ry(pitch) @ Rx(roll)


# ============================================================================
# 0. PARITY — verbatim pre-refactor reference (operating on a state object)
# ============================================================================
class _S:                              # mock 'self' carrying the old _lw_* attrs
    pass


def _inline_reference(I_a, R, R33, yaw_c, corners, center, focal,
                      p_10, theta_cone, dt_last, w_rp, S, radius, env):
    """Independent (hand-copied, not imported) transcription of cbf2_filter, kept as
    a second implementation to catch ACCIDENTAL drift in the real one (typos,
    dropped terms) -- not a check against pre-refactor behavior, since cbf_visibility.py
    has since intentionally diverged from its original form (2026-08-04 camera-mount
    yaw fix propagated below; 2026-08-13 radius-based delta2 replaces the old
    corner-spread derivation, also propagated below -- see the module docstring on
    cbf_visibility.py's own `radius` param for why corner-spread is gone for good)."""
    foc = np.asarray(focal, float)
    a_z = abs(I_a[2]); ok = False
    try:
        if corners is None:
            raise ValueError
        rc = np.asarray(corners, float)
        ct = (rc - np.asarray(center, float)) / foc
        ct = np.column_stack([ct[:, 1], -ct[:, 0]])      # camera-mount yaw swap
        cr2 = ct.mean(0); x2, y2 = float(cr2[0]), float(cr2[1])
        Lw2 = np.array([[x2 * y2, -(1 + x2 * x2)], [1 + y2 * y2, -x2 * y2]])
        tau = float(env.get("CBF_TAU", "0.3"))
        delta2 = np.full(2, float(radius)) / foc
        if dt_last is not None and dt_last > 1e-6 and getattr(S, "_lw_delta_prev", None) is not None:
            S._lw_ddelta_ref = np.maximum((delta2 - S._lw_delta_prev) / dt_last, 0.0)
        else:
            S._lw_ddelta_ref = np.zeros(2)
        S._lw_delta_prev = delta2.copy()
        S._lw_decode_fail_n = 0
        S._lw_phase2_alpha = 0.0
        m2 = np.maximum(np.asarray(p_10, float), 1e-3)
        dft = np.zeros(2)
        if dt_last is not None and dt_last > 1e-6 and getattr(S, "_lw_cr_prev", None) is not None:
            d_raw = (cr2 - S._lw_cr_prev) / dt_last - Lw2 @ np.asarray(w_rp, float)
            ema = float(env.get("CBF_DMIN_EMA", "0.3"))
            S._lw_d = (1 - ema) * getattr(S, "_lw_d", np.zeros(2)) + ema * d_raw
            dft = tau * S._lw_d
        S._lw_cr_prev = cr2.copy()
        S._lw_Lw2_prev = Lw2.copy()
        cz, sz = np.cos(yaw_c), np.sin(yaw_c)
        Rzm = np.array([[cz, sz], [-sz, cz]])
        Rz_p90b = np.array([[0.0, -1.0], [1.0, 0.0]])
        Rz_m90b = np.array([[0.0, 1.0], [-1.0, 0.0]])
        th_curr = Rz_p90b @ (Rzm @ (-np.asarray(R[:2, 2], float) / max(abs(R33), 1e-3)))
        th = Rz_p90b @ (Rzm @ (np.asarray(I_a[:2], float) / max(a_z, 1e-6)))
        anchor = cr2 - Lw2 @ th_curr + dft
        for _ in range(10):
            f = anchor + Lw2 @ th
            for k in range(2):
                if f[k] > m2[k]:
                    r = Lw2[k]; th = th - (f[k] - m2[k]) / (r @ r + 1e-12) * r; f = anchor + Lw2 @ th
                elif f[k] < -m2[k]:
                    r = Lw2[k]; th = th - (f[k] + m2[k]) / (r @ r + 1e-12) * r; f = anchor + Lw2 @ th
        # theta_cap deliverability clip removed (now caller-applied) — see cbf2_filter.
        I_a[:2] = a_z * (np.array([[cz, -sz], [sz, cz]]) @ (Rz_m90b @ th))
        theta_cone = float(np.linalg.norm(th))
        ok = True
    except (IndexError, AttributeError, ValueError, TypeError):
        ok = False
    if not ok:
        S._lw_decode_fail_n = getattr(S, "_lw_decode_fail_n", 0) + 1
        fail_thresh = int(env.get("CBF_PHASE2_HYSTERESIS", "3"))
        ramp_frames = float(env.get("CBF_PHASE2_RAMP_FRAMES", "5"))
        if S._lw_decode_fail_n >= fail_thresh:
            alpha = getattr(S, "_lw_phase2_alpha", 0.0)
            S._lw_phase2_alpha = min(alpha + 1.0 / ramp_frames, 1.0)
        delta_ref = getattr(S, "_lw_delta_prev", None)
        Lw2_ref = getattr(S, "_lw_Lw2_prev", None)
        p2_alpha = getattr(S, "_lw_phase2_alpha", 0.0)
        if delta_ref is not None and Lw2_ref is not None and p2_alpha > 0:
            delta_eff = delta_ref * p2_alpha
            ddelta_eff = getattr(S, "_lw_ddelta_ref", np.zeros(2)) * p2_alpha
            tau_p2 = float(env.get("CBF_TAU", "0.3"))
            m2_p2 = np.maximum(np.asarray(p_10, float) - delta_eff - tau_p2 * ddelta_eff, 1e-3)
            cr_ref = np.asarray(getattr(S, "_lw_cr_prev", np.zeros(2)), float)
            dft_ref = tau_p2 * np.asarray(getattr(S, "_lw_d", np.zeros(2)), float)
            effective_margin = np.maximum(m2_p2 - np.abs(cr_ref + dft_ref), 0.0)
            row_norms = np.linalg.norm(Lw2_ref, axis=1)
            theta_tight = float(np.min(effective_margin / (row_norms + 1e-9)))
            theta_cone = float(min(theta_cone, max(theta_tight, 0.0)))
        a_xy_lim = a_z * np.tan(theta_cone); a_xy_n = np.linalg.norm(I_a[:2])
        if a_xy_n > a_xy_lim and a_xy_n > 1e-9:
            I_a[:2] = a_xy_lim * I_a[:2] / a_xy_n
    return I_a, theta_cone, ok


def test_parity():
    """Drive both implementations with identical randomized SEQUENCES (so the
    EMA/Phase-2 state evolves) and require bit-identical outputs."""
    worst = 0.0
    for trial in range(40):
        S = _S(); state = {}
        # CBF_LW_ROT=0: parity is against the verbatim OFF behavior (the fix is
        # now default-ON, so the inline reference == the OFF path).
        env = {"CBF_LW_ROT": "0",
               "CBF_TAU": str(RNG.uniform(0, 0.5)),
               "CBF_DMIN_EMA": str(RNG.uniform(0.1, 0.6)),
               "CBF_PHASE2_HYSTERESIS": str(RNG.integers(1, 5)),
               "CBF_PHASE2_RAMP_FRAMES": str(RNG.uniform(2, 8))}
        for step in range(12):
            yaw = RNG.uniform(-np.pi, np.pi)
            cr = RNG.uniform(-1.0, 1.0, 2)
            decode = RNG.random() > 0.3
            corners = corners_from_tangent(cr, RNG.uniform(0.02, 0.4, 2)) if decode else None
            a_z = RNG.uniform(3, 15)
            th_des = RNG.uniform(-1.0, 1.0, 2)
            I_a = Ia_from_tilt(th_des, yaw, a_z)
            R = R_from_image_tilt(np.clip(th_des, -0.3, 0.3), yaw)
            R33 = R[2, 2]
            dt = RNG.uniform(0.01, 0.03)
            w_rp = RNG.uniform(-0.5, 0.5, 2)
            radius = float(RNG.uniform(0.0, 20.0))     # px, arbitrary per-step test value
            out_ref, tc_ref, ok_ref = _inline_reference(
                I_a.copy(), R, R33, yaw, corners, CENTER, FOCAL, P_10,
                0.3, dt, w_rp, S, radius, env)
            out_new, tc_new, ok_new, _ths, _thd = cbf2_filter(
                I_a.copy(), R, R33, yaw, corners, CENTER, FOCAL, P_10,
                0.3, dt, w_rp, state, radius, env)
            d = max(np.max(np.abs(out_ref - out_new)), abs(tc_ref - tc_new))
            worst = max(worst, d)
            if ok_ref != ok_new or d > 1e-12:
                _record("0. parity vs verbatim inline", False,
                        f"trial {trial} step {step}: Δ={d:.2e} ok={ok_ref}/{ok_new}")
                return
    _record("0. parity vs verbatim inline", True, f"max|Δ|={worst:.1e} over 40×12 steps")


# ============================================================================
# 1. L_w FIDELITY — linear model vs true feature shift
# ============================================================================
def test_lw_fidelity():
    """The model the QP enforces, cr + L_w@coupling@(th - th_curr), vs the TRUE
    feature shift from an independent pinhole+attitude camera. Compares the
    current-code coupling (identity) against the fix (M90). The barrier is only
    meaningful if this model is faithful.

    2026-08-13: kept in the BODY-FRD frame (local helpers, not the module-level
    project_tangent/th_curr_of, which are now in the camera-mount-swapped IMAGE
    frame for the tests that call cbf2_filter). This test never calls cbf2_filter
    -- it's an independent check of the L_omega interaction-matrix model itself,
    which is a body-frame-native physics question (the M90 lean-vs-rotation-axis
    correction it validates is unrelated to the camera-mount fix). Mixing the two
    conventions here produced a bogus ~200% "error" that was a frame mismatch in
    the test, not a modeling error -- see the barrier/no-strangle root-cause
    write-up (project_20260813_cbf_extent_fix_followup memory)."""
    def _proj_body(R, cam_pos, P):
        b = R.T @ (P - cam_pos)
        return np.array([b[0] / b[2], b[1] / b[2]])

    def _th_curr_body(R, yaw):
        Rzm = Rz(-yaw)[:2, :2]
        return Rzm @ (-R[:2, 2] / max(abs(R[2, 2]), 1e-3))

    worst_id = 0.0; worst_fix = 0.0
    for _ in range(2000):
        yaw = RNG.uniform(-np.pi, np.pi)
        Z = RNG.uniform(1.0, 8.0)
        cam = np.array([0.0, 0.0, -Z])
        P = np.array([RNG.uniform(-0.7, 0.7) * Z, RNG.uniform(-0.5, 0.5) * Z, 0.0])
        roll0, pitch0 = RNG.uniform(-0.08, 0.08, 2)   # near-hover descent tilt envelope
        R0 = Rz(yaw) @ Ry(pitch0) @ Rx(roll0)
        cr0 = _proj_body(R0, cam, P)
        x, y = cr0
        L_w = np.array([[x * y, -(1 + x * x)], [1 + y * y, -x * y]])
        dr, dp = RNG.uniform(-1e-4, 1e-4, 2)     # tiny step => clean Jacobian test
        R1 = Rz(yaw) @ Ry(pitch0 + dp) @ Rx(roll0 + dr)
        cr1 = _proj_body(R1, cam, P)
        dth = _th_curr_body(R1, yaw) - _th_curr_body(R0, yaw)   # code's image-tilt change
        actual = cr1 - cr0
        denom = np.linalg.norm(actual) + 1e-6
        worst_id = max(worst_id, np.linalg.norm(L_w @ dth - actual) / denom)
        worst_fix = max(worst_fix, np.linalg.norm(L_w @ M90 @ dth - actual) / denom)
    _record("1. L_w fidelity — FIX (L_w@M90)", worst_fix < 0.08,
            f"worst rel err {worst_fix*100:.2f}% (<8%, near-hover tilt) over 2000 cases")
    _record("1b. L_w fidelity — current code (identity) is WRONG", worst_id > 0.5,
            f"worst rel err {worst_id*100:.1f}% — confirms the 90deg bug")


# ============================================================================
# 2 & 3. BARRIER (predicted) + END-TO-END (true feature) inside the box
# ============================================================================
def test_barrier_and_end_to_end():
    pred_ok = True; pred_worst = -1e9
    true_ok = True; true_worst = -1e9
    n_bound = 0
    for _ in range(3000):
        yaw = RNG.uniform(-np.pi, np.pi)
        Z = RNG.uniform(1.0, 6.0)
        cam = np.array([0.0, 0.0, -Z])
        # marker offset toward an edge so the barrier frequently binds
        P = np.array([RNG.uniform(-1.0, 1.0) * Z, RNG.uniform(-0.8, 0.8) * Z, 0.0])
        roll0, pitch0 = RNG.uniform(-0.1, 0.1, 2)
        R0 = Rz(yaw) @ Ry(pitch0) @ Rx(roll0)
        cr0 = project_tangent(R0, cam, P)
        if np.any(np.abs(cr0) > P_10):       # start already outside — skip
            continue
        corners = corners_from_tangent(cr0, (0.03, 0.03))
        a_z = RNG.uniform(4, 12)
        # desired tilt that pushes the feature further OUT (toward its own sign)
        th_des = np.sign(cr0) * RNG.uniform(0.3, 1.2, 2)
        I_a = Ia_from_tilt(th_des, yaw, a_z)
        out, tc, ok, _ts, _thd = cbf2_filter(I_a.copy(), R0, R0[2, 2], yaw, corners,
                                  CENTER, FOCAL, P_10, 0.3,
                                  None, np.zeros(2), {}, 0.0, FIX_ENV)
        if not ok:
            continue
        # recover commanded image tilt from the returned accel
        a_xy = out[:2]; th_star = _RZ_P90B @ (Rz(-yaw)[:2, :2] @ (a_xy / a_z))
        # (2) predicted feature under the (fixed) linear model the QP used
        L_w = np.array([[cr0[0]*cr0[1], -(1+cr0[0]**2)],
                        [1+cr0[1]**2, -cr0[0]*cr0[1]]]) @ M90
        f_pred = cr0 + L_w @ (th_star - th_curr_of(R0, yaw))
        pred_margin = np.min(P_10 - np.abs(f_pred))
        # box may be infeasible if a_z too small for a huge cr; only assert when
        # the linear-model solution is meant to be feasible (m2=phi_max always is)
        pred_worst = max(pred_worst, -pred_margin)
        if pred_margin < -1e-6:
            pred_ok = False
        # (3) TRUE feature when the body actually realizes th_star. The CBF is a
        # ONE-STEP linearized projection: its guarantee holds to within the
        # linearization error, which is only small for a realistic per-cycle tilt
        # step. Assert the true feature stays in the box when the commanded step
        # is small (how it actually runs); large one-shot steps break the
        # linearization (characterized separately, expected).
        dstep = np.linalg.norm(th_star - th_curr_of(R0, yaw))
        if np.linalg.norm(th_star - th_des) > 1e-3 and dstep < 0.10:
            Rt = R_from_image_tilt(np.clip(th_star, -THETA_CAP, THETA_CAP), yaw)
            cr_true = project_tangent(Rt, cam, P)
            true_margin = np.min(P_10 - np.abs(cr_true))
            n_bound += 1
            true_worst = max(true_worst, -true_margin)
            if true_margin < -0.02:        # 2% tangent slack for linearization
                true_ok = False
    _record("2. barrier holds (predicted feature)", pred_ok,
            f"worst overshoot {max(pred_worst,0):.2e} tangent")
    _record("3. barrier holds (TRUE feature)", true_ok,
            f"worst overshoot {max(true_worst,0):.3f} tangent over {n_bound} bound cases")


# ============================================================================
# 4. MINIMAL INTERVENTION — feasible desired accel passes through unchanged
# ============================================================================
def test_minimal_intervention():
    worst = 0.0
    for _ in range(2000):
        yaw = RNG.uniform(-np.pi, np.pi)
        cr = RNG.uniform(-0.3, 0.3, 2)        # marker well inside
        corners = corners_from_tangent(cr, (0.02, 0.02))
        a_z = RNG.uniform(5, 12)
        # tiny tilt that keeps the feature comfortably inside the box
        th_des = RNG.uniform(-0.05, 0.05, 2)
        I_a = Ia_from_tilt(th_des, yaw, a_z)
        out, tc, ok, _ts, _thd = cbf2_filter(I_a.copy(), R_from_image_tilt(th_des, yaw),
                                  np.cos(np.linalg.norm(th_des)), yaw, corners,
                                  CENTER, FOCAL, P_10, 0.3,
                                  None, np.zeros(2), {}, 0.0, FIX_ENV)
        worst = max(worst, np.max(np.abs(out[:2] - I_a[:2])))
    ok = worst < 1e-6
    _record("4. minimal intervention (feasible passthrough)", ok,
            f"max |Δa_xy| {worst:.2e} (<1e-6)")


# ============================================================================
# 5. NO-STRANGLE — inward recovery accel is left free; only outward bounded
# ============================================================================
def test_no_strangle():
    """Marker near an edge. Desired accel points INWARD (recovery). The CBF must
    NOT shrink it (a magnitude cone clamp would). Compare to a pure cone clamp."""
    free_ok = True; worst_shrink = 0.0
    for _ in range(2000):
        yaw = RNG.uniform(-np.pi, np.pi)
        edge = np.array([RNG.choice([-1, 1]) * RNG.uniform(0.7, 1.0) * P_10[0],
                         RNG.choice([-1, 1]) * RNG.uniform(0.5, 0.9) * P_10[1]])
        corners = corners_from_tangent(edge, (0.03, 0.03))
        a_z = RNG.uniform(5, 12)
        # SMALL inward image tilt = opposite sign of the feature offset (re-center
        # without swinging the feature past centre to the opposite edge, which the
        # CBF would correctly clamp — that is anti-overshoot, not strangling).
        th_des = -np.sign(edge) * RNG.uniform(0.02, 0.12, 2)
        I_a = Ia_from_tilt(th_des, yaw, a_z)
        out, tc, ok, _ts, _thd = cbf2_filter(I_a.copy(), R_from_image_tilt(np.clip(th_des, -0.3, 0.3), yaw),
                                  np.cos(0.2), yaw, corners, CENTER, FOCAL, P_10,
                                  0.3, None, np.zeros(2), {}, 0.0, FIX_ENV)
        if not ok:
            continue
        n_in = np.linalg.norm(I_a[:2]); n_out = np.linalg.norm(out[:2])
        shrink = (n_in - n_out) / (n_in + 1e-9)
        worst_shrink = max(worst_shrink, shrink)
        if shrink > 0.05:        # inward command shrunk >5% => strangling
            free_ok = False
    _record("5. no-strangle (inward recovery free)", free_ok,
            f"worst inward shrink {worst_shrink*100:.2f}% (<5%)")


# ============================================================================
# 6. CONVENTIONS — Rz round-trip + a_xy<->theta map invert
# ============================================================================
def test_conventions():
    worst = 0.0
    for _ in range(2000):
        yaw = RNG.uniform(-np.pi, np.pi)
        a_z = RNG.uniform(4, 12)
        th = RNG.uniform(-0.5, 0.5, 2)
        a_xy = a_z * (Rz(yaw)[:2, :2] @ th)        # forward map (controller out)
        th_back = Rz(-yaw)[:2, :2] @ (a_xy / a_z)  # inverse map (controller in)
        worst = max(worst, np.max(np.abs(th_back - th)))
    ok = worst < 1e-12
    _record("6. conventions (Rz round-trip / a_xy<->th)", ok, f"max|Δ| {worst:.1e}")


# ============================================================================
# 7. PHASE-2 — decode-fail hysteresis + ramp + theta_cap clip
# ============================================================================
def test_phase2():
    # prime Phase-1 state with a decoded frame so delta_prev/Lw2_prev exist
    state = {}
    yaw = 0.3
    corners = corners_from_tangent(np.array([0.4, 0.2]), (0.25, 0.2))
    I_a = Ia_from_tilt(np.array([0.1, 0.0]), yaw, 9.0)
    cbf2_filter(I_a.copy(), R_from_image_tilt(np.array([0.1, 0.0]), yaw),
                np.cos(0.1), yaw, corners, CENTER, FOCAL, P_10, 0.3,
                0.02, np.zeros(2), state, 0.0)
    env = {"CBF_PHASE2_HYSTERESIS": "3", "CBF_PHASE2_RAMP_FRAMES": "5"}
    alphas = []
    for _ in range(10):     # feed decode-fails (corners=None)
        I_a = Ia_from_tilt(np.array([0.5, 0.3]), yaw, 9.0)
        cbf2_filter(I_a.copy(), np.eye(3), 1.0, yaw, None, CENTER, FOCAL, P_10,
                    0.3, 0.02, np.zeros(2), state, 0.0, env)
        alphas.append(state.get("phase2_alpha", 0.0))
    # hysteresis: alpha stays 0 for first 2 fails (thresh=3), then ramps by 1/5
    hyst_ok = alphas[0] == 0.0 and alphas[1] == 0.0 and alphas[2] > 0.0
    ramp_ok = abs(alphas[2] - 0.2) < 1e-9 and abs(alphas[6] - 1.0) < 1e-9 and alphas[-1] == 1.0
    # NOTE: the former "7c. theta_cap post-QP clip" check was REMOVED (2026-06-16) — the
    # deliverability cap is no longer a CBF concern (moved to controller.py), so there is
    # nothing to test here. The CBF's only bound is the FoV box (tests 2/3/5).
    _record("7a. Phase-2 hysteresis gate", hyst_ok, f"alpha[:3]={[round(a,3) for a in alphas[:3]]}")
    _record("7b. Phase-2 ramp 1/5 -> 1.0", ramp_ok, f"alpha[2]={alphas[2]:.3f} alpha[6]={alphas[6]:.3f}")


# ============================================================================
# 8. FIX B — rd3 built from th_safe directly (controller.py R_d construction)
# ============================================================================
def test_fixB_rd3():
    """Replicate the controller's rd3 paths. (a) The th_safe-direct rd3 must equal
    the UNFILTERED accel-path rd3 exactly (no regression when no LPF). (b) The
    commanded lean th_safe must land EXACTLY on the desired attitude's tilt
    (th_curr_of(R_d) == th_safe) — the leak the LPF round-trip used to introduce."""
    worst_match = 0.0; worst_land = 0.0
    for _ in range(3000):
        yaw = RNG.uniform(-np.pi, np.pi)
        ts = RNG.uniform(-0.8, 0.8, 2)              # safe lean vector (theta_cap-bounded)
        a_z = RNG.uniform(4, 12)
        cy, sy = np.cos(yaw), np.sin(yaw)
        # --- controller's th_safe-direct rd3 (Fix B): a_xy_dir = Rz(yaw)@Rz(-90deg)@th_safe
        # (CAMERA-MOUNT YAW FIX, controller.py:2924-2930 -- propagated here 2026-08-13,
        # this test previously omitted the Rz(-90deg) factor) ---
        ts_rot = _RZ_M90B @ ts
        a_xy_dir = np.array([cy * ts_rot[0] - sy * ts_rot[1], sy * ts_rot[0] + cy * ts_rot[1]])
        rd3_dir = np.array([-a_xy_dir[0], -a_xy_dir[1], 1.0]); rd3_dir /= np.linalg.norm(rd3_dir)
        # --- unfiltered accel-path rd3 (what the LPF would converge to) ---
        I_a = np.array([a_z * a_xy_dir[0], a_z * a_xy_dir[1], -a_z])
        rd3_acc = -I_a / np.linalg.norm(I_a)
        worst_match = max(worst_match, np.max(np.abs(rd3_dir - rd3_acc)))
        # --- full R_d via the controller's Gram-Schmidt, then recover its tilt ---
        a_h = np.array([np.cos(yaw), np.sin(yaw), 0.0])
        rd2 = np.cross(rd3_dir, a_h); n2 = np.linalg.norm(rd2)
        if n2 < 1e-6:
            continue
        rd2 /= n2; rd1 = np.cross(rd2, rd3_dir)
        R_d = np.column_stack([rd1, rd2, rd3_dir])
        worst_land = max(worst_land, np.linalg.norm(th_curr_of(R_d, yaw) - ts))
    _record("8a. Fix B: direct rd3 == unfiltered accel rd3", worst_match < 1e-12,
            f"max|Δrd3| {worst_match:.1e}")
    _record("8b. Fix B: th_safe lands exactly on R_d tilt", worst_land < 1e-9,
            f"max|Δtilt| {worst_land:.1e} (LPF smear removed)")


def main():
    print("=" * 72)
    print("Stage 1 — synthetic validation of cbf2_filter (isolated CBF)")
    print("=" * 72)
    test_parity()
    test_lw_fidelity()
    test_barrier_and_end_to_end()
    test_minimal_intervention()
    test_no_strangle()
    test_conventions()
    test_phase2()
    test_fixB_rd3()
    print("-" * 72)
    n_fail = sum(1 for _, ok, _ in _RESULTS if not ok)
    print(f"{len(_RESULTS) - n_fail}/{len(_RESULTS)} checks passed")
    return 1 if n_fail else 0


if __name__ == "__main__":
    sys.exit(main())
