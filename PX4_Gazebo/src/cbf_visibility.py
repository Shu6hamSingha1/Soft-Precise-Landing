"""Isolated, pure-function port of the cbf2 target-visibility CBF.

CROSS-MARKER-DEDICATED (2026-08-13, user split): ArUco has its own separate
copy, ``cbf_visibility_aruco.py`` (real-4-corner-array ``delta2`` derivation,
plus a Phase-2 signed-projection rewrite under test there). This module now
assumes a SINGLE tracked point (the cross-marker's intersection) plus an
explicit ``radius`` param for size -- see ``cbf2_filter``'s docstring for why
the two marker types need genuinely different `delta2` derivations, not just
a shared function with an optional knob.

This module lifts the ``FUNNEL_MODE == "cbf2"`` block out of
``controller.py`` (formerly inlined at controller.py:1074-1167) so the
camera-plane visibility CBF can be exercised in isolation by the offline
validator (``tools/validate_cbf.py``) using the
EXACT code path the live controller runs — the test can never drift from prod.

Design reference: ``docs/FUNNEL_CBF_DESIGN.md`` §0 and ``docs/CBF_visibility.pdf``.

Barrier (real camera plane), per image axis k, on the MEASURED camera feature
``cr`` (tangent units ``(px - centre)/f``)::

    h_k = phi_max_k - |cr_k| - delta_k ,    phi_max = p_10 = centre/focal

Tilt->feature coupling is the rotational interaction matrix L_omega at the
measured centroid (exact, depth-free). The QP projects the desired image-axis
tilt ``theta_d`` onto the FoV box ``|cr + L_w (theta - theta_curr) + tau d| <= m``
(alternating projection), then applies a post-QP ``theta_cap`` deliverability
clip. Two-phase delta: Phase 1 (marker decodes) is centroid-only (``m2 =
phi_max``, always feasible); Phase 2 (decode-fail, hysteresis-gated) ramps a
magnitude-clamp fallback.

The only mutable state is the small ``state`` dict (the former ``self._lw_*``
attributes) carrying EMA drift, the previous centroid, and the Phase-2 ramp.
Pass the SAME dict across cycles; the live controller passes ``self._cbf_state``.
"""

import os

import numpy as np


def cbf2_filter(I_a, R, R33, yaw_c, corners, center, focal,
                p_10, theta_cone, dt_last, w_rp, state, radius, env=None):
    """Constrain the lateral accel command for target visibility (cbf2).

    2026-08-13 (user, cross-marker/ArUco split): this module is now the
    CROSS-MARKER-DEDICATED cbf2 implementation -- ArUco has its own separate
    copy, ``cbf_visibility_aruco.py``, which keeps the original real-corner-
    array ``delta2`` derivation. The two markers' visibility geometry differs
    fundamentally, not just in what data happens to be available: ArUco's
    hard requirement is that all 4 REAL corners stay resolvable, and their
    spread naturally reflects marker size at every altitude (near-coincident
    at high altitude, splayed out up close) -- ``delta2`` there is correctly
    DERIVED from the corner array itself, nothing else needed. The
    cross-marker only needs its single intersection point to stay in FoV
    (alpha/h,w degrade gracefully on their own); its size is a SEPARATE,
    independently-measured quantity (``MARKER_EXTENT_PX/2``, from the
    color-gated mask bbox) that has no implicit representation in a
    single tracked point at all -- there is no "radius=0.0 means don't use
    this" case to special-case, ``radius`` is simply always the input that
    conveys size for this marker type. Mandatory param (was optional with a
    corner-array-spread fallback in the same commit that introduced it;
    removed same day per user correction once this call/copy distinction
    was made explicit).

    Pure except for the explicit ``state`` dict. Mutates and returns ``I_a``.

    Parameters
    ----------
    I_a : (3,) ndarray
        Desired inertial (NED) accel command. The z-upright guard
        (``I_a[2] >= 0 -> -3``) must already be applied by the caller, exactly
        as in the live controller. ``I_a[:2]`` is replaced in place.
    R : (3,3) ndarray
        Current body->inertial rotation (DCM from the attitude quaternion).
    R33 : float
        ``R[2,2]`` (body-z inertial-z component); ``arccos`` gives current tilt.
    yaw_c : float
        Control yaw (rad) — the ``BODY_YAW_SOURCE`` heading used downstream.
    corners : (N,2) ndarray or None
        Latest measured marker corners in raw OpenCV (u,v) pixels
        (``img_node._feature_pts[-1][1]``). ``None`` (or malformed) routes to
        the Phase-2 decode-fail branch, exactly like the original try/except.
    center : (2,) ndarray
        Image centre pixel ``[cx, cy]``.
    focal : (2,) ndarray
        ``[fx, fy]`` in pixels.
    p_10 : (2,) ndarray
        ``centre/focal`` = phi_max, the FoV-edge tangent half-extent.
    theta_cone : float
        Scalar tilt-cone fallback computed upstream (used only by Phase 2).
    dt_last : float or None
        Most recent loop ``dt`` (s). ``None``/<=1e-6 disables the drift/loom
        finite differences this cycle.
    w_rp : (2,) ndarray
        Body roll/pitch rate ``[w_x, w_y]`` (rad/s) for the drift estimate.
    state : dict
        Persistent CBF state (the former ``self._lw_*``). Keys used:
        ``delta_prev, ddelta_ref, decode_fail_n, phase2_alpha, cr_prev, d,
        Lw2_prev``. Pass the same dict each cycle.
    radius : float
        Marker radius in raw PIXEL units (``MARKER_EXTENT_PX/2`` --
        ``CrossMarkerPerception.get_marker_radius_px()``). Converted to the SAME
        normalized-tangent units as ``ct`` (divide by focal length per axis -- a
        circle of pixel-radius r projects to an axis-aligned ellipse of
        half-extent r/fx, r/fy in tangent space) to become ``delta2``, the
        per-axis half-extent Phase-2's decode-fail fallback ramps toward. See
        this function's module-level docstring for why this is mandatory (not
        derived from ``corners``, unlike ArUco's separate copy) -- pass 0.0
        explicitly for a genuine zero-size/degenerate reading, not as an "off"
        sentinel.
    env : mapping, optional
        Environment overrides (defaults to ``os.environ``). Reads
        ``CBF_TAU, CBF_DMIN_EMA, CBF_PHASE2_HYSTERESIS, CBF_PHASE2_RAMP_FRAMES``.

    Returns
    -------
    I_a : (3,) ndarray
        The constrained command (same object, mutated).
    theta_cone : float
        Commanded tilt magnitude (Phase 1) or tightened cone (Phase 2) — the
        ``theta_cone(t)`` diagnostic the live controller logs.
    ok : bool
        True if the QP path ran (Phase 1, marker decoded); False if it fell
        through to the Phase-2 magnitude-clamp fallback.
    th_safe : (2,) ndarray or None
        The safe LEAN vector (image axes, UN-capped — the caller applies the
        deliverability tilt cap) the QP produced on the Phase-1 path; it follows
        that ``I_a[:2]`` is likewise un-capped. ``None`` on the Phase-2 fallback. The caller can build
        the desired attitude directly from this (Fix B) instead of round-tripping
        through ``I_a`` + the LPF: ``rd3 = [-Rz(yaw)@th_safe, 1]`` normalized.
    th_desired : (2,) ndarray or None
        The UNCONSTRAINED desired lean vector (``Rz(-yaw)@(a_xy/a_z)``, image axes),
        i.e. ``th_safe`` before the FoV-box projection loop. ``None`` on the Phase-2
        fallback (no projection ran, so "unmet demand" isn't a meaningful concept
        there). 2026-08-24 (AZ VISIBILITY FILTER v2, user design): the caller uses
        ``dtheta = th_desired - th_safe`` — how much lateral authority the
        visibility constraint is actively suppressing right now — to drive a
        POST-CBF descent-rate slowdown (replaces the earlier loom-margin-prediction
        approach): when the CBF is fighting the lateral controller hard, slow
        descent to buy the lateral loop time to converge instead of pressing on
        blind to the conflict.
    """
    if env is None:
        env = os.environ
    foc = np.asarray(focal, float)
    a_z = abs(I_a[2])
    ok = False
    th_safe = None
    th_desired = None
    try:
        if corners is None:
            raise ValueError("no corners")
        rc = np.asarray(corners, float)
        ct = (rc - np.asarray(center, float)) / foc
        # CAMERA-MOUNT YAW FIX (2026-08-04, CORRECTED): same physical camera-mount
        # change (yaw+=90deg on the pointing-down pitch) as cross_marker_perception.py/
        # img_data.py's _getVirtualPts -- this function does its OWN separate raw-pixel
        # normalization (not routed through _getVirtualPts), so it needs the identical
        # [y,-x] compensating swap (Rz(-90deg), corrected sign -- see _getVirtualPts's
        # comment for the empirical evidence that [-y,x]/Rz(+90) was backwards).
        ct = np.column_stack([ct[:, 1], -ct[:, 0]])
        cr2 = ct.mean(0)
        x2, y2 = float(cr2[0]), float(cr2[1])
        Lw2 = np.array([[x2 * y2, -(1 + x2 * x2)], [1 + y2 * y2, -x2 * y2]])
        tau = float(env.get("CBF_TAU", "0.3"))
        # actual per-axis half-extent: closed-form from radius (px -> tangent
        # units, via foc) -- see this function's `radius` param doc for why this
        # is the cross-marker's only source of size (no corner-array spread to
        # fall back to; ArUco's separate copy keeps that derivation instead).
        delta2 = np.full(2, float(radius)) / foc
        # Track delta and its loom rate for Phase 2 tau*ddelta_eff term
        if dt_last is not None and dt_last > 1e-6 and state.get("delta_prev") is not None:
            state["ddelta_ref"] = np.maximum((delta2 - state["delta_prev"]) / dt_last, 0.0)
        else:
            state["ddelta_ref"] = np.zeros(2)
        state["delta_prev"] = delta2.copy()
        # Two-phase delta (PDF Sec. 4): Phase 1 = central marker decoded (here).
        # delta_eff = 0: centroid-only barrier; deliberately allow the marker to grow
        # and overflow as the UAV closes in. m2 = phi_max only; tau*ddelta excluded.
        # Reset Phase 2 ramp counters on every successful decode.
        state["decode_fail_n"] = 0
        state["phase2_alpha"] = 0.0
        # RESERVE MARGIN (2026-08-25, IC5 FoV-exit root cause -- see
        # project_20260824_ic5_perception_fov_margin_gap memory): Phase 1's box bound was
        # centroid-only (delta_eff=0) by design, deliberately allowing the marker to grow
        # and overflow the FoV edge until decode ACTUALLY fails (reactive, Phase 2 only).
        # At tight-margin ICs (low altitude + large offset) that means the corners can
        # genuinely exit frame before the CBF ever tightens up. CBF_MARGIN_RESERVE (0-1,
        # default 0.0 = EXACT prior behavior, centroid-only) proactively reserves a
        # fraction of the marker's own measured footprint (delta2) in Phase 1 too, so the
        # box constrains theta BEFORE the corners reach the edge, not just after they've
        # already left it. Default-off: this changes live Phase-1 QP behavior, gate behind
        # A/B before baking.
        _margin_reserve = float(env.get("CBF_MARGIN_RESERVE", "0.0"))
        m2 = np.maximum(np.asarray(p_10, float) - _margin_reserve * delta2, 1e-3)
        dft = np.zeros(2)
        if dt_last is not None and dt_last > 1e-6 and state.get("cr_prev") is not None:
            d_raw = (cr2 - state["cr_prev"]) / dt_last - Lw2 @ np.asarray(w_rp, float)
            ema = float(env.get("CBF_DMIN_EMA", "0.3"))
            state["d"] = (1 - ema) * state.get("d", np.zeros(2)) + ema * d_raw
            dft = tau * state["d"]
        state["cr_prev"] = cr2.copy()
        state["Lw2_prev"] = Lw2.copy()                               # stash for Phase 2 headroom calc
        cz, sz = np.cos(yaw_c), np.sin(yaw_c)
        Rzm = np.array([[cz, sz], [-sz, cz]])                        # Rz(-yaw): inertial -> image
        # CAMERA-MOUNT YAW FIX (2026-08-04, CORRECTED): Rzm alone converts
        # inertial->BODY-aligned image axes (the OLD "camera=body-FRD aligned"
        # assumption); now needs an additional Rz(+90deg) to correctly land in the NEW
        # (post camera-yaw) image axes -- forward is ray_body = Rz(-90deg) @ ray_image
        # (corrected sign, see _getVirtualPts's comment for the empirical evidence),
        # so converting body/inertial -> image needs the inverse, Rz(+90deg).
        Rz_p90b = np.array([[0.0, -1.0], [1.0, 0.0]])                # Rz(+90deg)
        th_curr = Rz_p90b @ (Rzm @ (-np.asarray(R[:2, 2], float) / max(abs(R33), 1e-3)))   # current image-axis tilt
        th = Rz_p90b @ (Rzm @ (np.asarray(I_a[:2], float) / max(a_z, 1e-6)))     # theta_d = Rz(-yaw)@(a_xy/a_z)
        th_desired = th.copy()   # UNCONSTRAINED desired tilt, before the FoV-box projection below
                                  # mutates th -- see this function's return-value doc for dtheta=
                                  # th_desired-th_safe (2026-08-24, AZ VISIBILITY FILTER v2)
        # --- lean-vector -> rotation-axis correction (CBF_LW_ROT) ---
        # L_w couples the body ANGULAR-RATE vector omega_rp to the feature flow
        # (cr_dot = L_w @ omega_rp), but theta here is the LEAN-direction vector
        # (a_xy/a_z). Lean and rotation-axis differ by 90deg (to lean +x you rotate
        # about +y): omega = M @ theta, M = [[0,1],[-1,0]]. The original port (and
        # the §0 design doc) plugged theta straight into L_w -> the barrier
        # constrained a 90deg-rotated direction (validate_cbf.py: 237% model error).
        # DEFAULT-ON (2026-06-14, validated tools/validate_cbf.py); =0 reverts to
        # the original (90deg-wrong) coupling for A/B.
        if env.get("CBF_LW_ROT", "1") == "1":
            Lw2 = Lw2 @ np.array([[0.0, 1.0], [-1.0, 0.0]])         # L_w @ M (M orthogonal -> row norms unchanged)
        anchor = cr2 - Lw2 @ th_curr + dft                          # f = cr + L_w@(theta-theta_curr) + tau*d
        for _ in range(10):                                         # project onto FoV box only
            f = anchor + Lw2 @ th
            for k in range(2):
                if f[k] > m2[k]:
                    r = Lw2[k]; th = th - (f[k] - m2[k]) / (r @ r + 1e-12) * r; f = anchor + Lw2 @ th
                elif f[k] < -m2[k]:
                    r = Lw2[k]; th = th - (f[k] + m2[k]) / (r @ r + 1e-12) * r; f = anchor + Lw2 @ th
        # NOTE: the deliverability tilt cap (theta_cap saturation) is intentionally NOT
        # applied here — it is a thrust-DELIVERABILITY concern, not a visibility constraint.
        # The CALLER applies it post-CBF (controller.py), so this function stays a pure
        # visibility QP whose ONLY constraint is the FoV box projection above.
        th_safe = th.copy()                                        # safe LEAN vector (image axes, UN-capped) for direct->rd3 (Fix B)
        # CAMERA-MOUNT YAW FIX (2026-08-04, CORRECTED): image -> body/inertial applies
        # the FORWARD transform directly (Rz(-90deg), same direction as _getVirtualPts),
        # BEFORE the existing Rz(yaw) inertial-yaw-alignment step.
        Rz_m90b = np.array([[0.0, 1.0], [-1.0, 0.0]])              # Rz(-90deg)
        I_a[:2] = a_z * (np.array([[cz, -sz], [sz, cz]]) @ (Rz_m90b @ th))      # a_xy* = a_z*Rz(yaw)@Rz(-90deg)@theta*
        theta_cone = float(np.linalg.norm(th))                      # log the commanded tilt magnitude
        ok = True
    except (IndexError, AttributeError, ValueError, TypeError):
        ok = False
    if not ok:
        # Phase 2: central marker overflowed / decode failed.
        # Hysteresis-gate: require CBF_PHASE2_HYSTERESIS consecutive decode-fails
        # before activating, to suppress flicker near touchdown. Ramp delta_eff from
        # 0 -> last measured 1/2 ptp over CBF_PHASE2_RAMP_FRAMES frames.
        state["decode_fail_n"] = state.get("decode_fail_n", 0) + 1
        fail_thresh = int(env.get("CBF_PHASE2_HYSTERESIS", "3"))
        ramp_frames = float(env.get("CBF_PHASE2_RAMP_FRAMES", "5"))
        if state["decode_fail_n"] >= fail_thresh:
            alpha = state.get("phase2_alpha", 0.0)
            # clamp ramp so delta never lags the fill
            state["phase2_alpha"] = min(alpha + 1.0 / ramp_frames, 1.0)
        delta_ref = state.get("delta_prev")
        Lw2_ref = state.get("Lw2_prev")
        p2_alpha = state.get("phase2_alpha", 0.0)
        if delta_ref is not None and Lw2_ref is not None and p2_alpha > 0:
            delta_eff = delta_ref * p2_alpha
            ddelta_eff = state.get("ddelta_ref", np.zeros(2)) * p2_alpha
            tau_p2 = float(env.get("CBF_TAU", "0.3"))
            m2_p2 = np.maximum(np.asarray(p_10, float) - delta_eff - tau_p2 * ddelta_eff, 1e-3)
            # Per-axis headroom -> conservative theta tightening for magnitude clamp.
            cr_ref = np.asarray(state.get("cr_prev", np.zeros(2)), float)
            dft_ref = tau_p2 * np.asarray(state.get("d", np.zeros(2)), float)
            effective_margin = np.maximum(m2_p2 - np.abs(cr_ref + dft_ref), 0.0)
            row_norms = np.linalg.norm(Lw2_ref, axis=1)
            theta_tight = float(np.min(effective_margin / (row_norms + 1e-9)))
            theta_cone = float(min(theta_cone, max(theta_tight, 0.0)))
        # magnitude-clamp fallback (direction preserved)
        a_xy_lim = a_z * np.tan(theta_cone)
        a_xy_n = np.linalg.norm(I_a[:2])
        if a_xy_n > a_xy_lim and a_xy_n > 1e-9:
            I_a[:2] = a_xy_lim * I_a[:2] / a_xy_n
    return I_a, theta_cone, ok, th_safe, th_desired
