"""ArUco-test variant of the cbf2 CBF -- carries the 2026-08-13 Phase-2 rewrite.

=============================================================================
WHY THIS COPY EXISTS (2026-08-13)
-----------------------------------------------------------------------------
``cbf_visibility.py`` is the module the CROSS-MARKER pipeline runs and is under
active development; it is deliberately left at its committed state. This copy
carries the Phase-2 (no-corners fallback) rewrite so it can be exercised against
the ARUCO pipeline in Gazebo without perturbing cross-marker work at all.

2026-08-13 UPDATE: ``cbf_visibility.py`` has since been dedicated fully to the
cross-marker (mandatory ``radius`` param, no corner-array ``delta2`` fallback --
see its module docstring), so it can no longer serve ArUco's real-corner-array
case at all. ``controller.py``'s import now routes by MARKER_TYPE: ArUco always
imports THIS file, cross-marker always imports ``cbf_visibility.py``.
``CBF_PHASE2_FIX`` no longer selects between the two files (that selection is
now MARKER_TYPE's job) -- it is currently unused as an import switch; if a
toggle between the Phase-2 rewrite here and the original Phase-2 fallback is
still wanted for ArUco A/B testing, it would need to become an in-module
branch instead (not implemented).

WHAT DIFFERS from cbf_visibility.py: only ``_project_box`` (new module-level
helper) and the body of the ``if not ok:`` Phase-2 branch. The Phase-1 path is
byte-identical -- verified by extracting the ``try:``..``if not ok:`` region from
both files and diffing.

The three Phase-2 defects fixed here, measured across 256 recorded Pi flights
(memory ``project_pi_cbf_phase2_zeroes_lateral_2026_08_12``): per-axis bound
collapsed to a scalar by ``np.min``; a direction-preserving magnitude clamp that
throttles recovery as hard as drift; and a margin flooring at 0 that drives
lateral authority to EXACTLY zero and self-latches. See the ``if not ok:`` block
for the full write-up.

NOTE for cross-marker: those defects are largely defused there anyway, because
``cbf_corners`` is a single centre point (controller.py ~line 2515), so
``delta_prev == 0`` and the Phase-2 box never shrinks. Only the scalar/
direction-blind clamp (defect 1) still applies, and only when the last-known
centre sits near the FoV edge. That is why this fix is scoped to ArUco for now.
=============================================================================

Isolated, pure-function port of the cbf2 target-visibility CBF.

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


def _project_box(th, anchor, Lw, m, iters=10):
    """Signed per-axis projection of the lean ``th`` onto ``|anchor + Lw@th| <= m``.

    Extracted 2026-08-13 from the Phase-1 loop so Phase 2 can reuse the SAME
    algorithm instead of its own (defective) isotropic magnitude clamp. Only the
    violated side of each axis is corrected, so a command that moves the feature
    back INSIDE the box passes through untouched -- that asymmetry is the whole
    point of a barrier and is what the old Phase-2 clamp destroyed.
    """
    th = np.asarray(th, float).copy()
    for _ in range(iters):
        f = anchor + Lw @ th
        for k in range(2):
            if f[k] > m[k]:
                r = Lw[k]; th = th - (f[k] - m[k]) / (r @ r + 1e-12) * r; f = anchor + Lw @ th
            elif f[k] < -m[k]:
                r = Lw[k]; th = th - (f[k] + m[k]) / (r @ r + 1e-12) * r; f = anchor + Lw @ th
    return th


def cbf2_filter(I_a, R, R33, yaw_c, corners, center, focal,
                p_10, theta_cone, dt_last, w_rp, state, radius=0.0, env=None):
    # `radius` (2026-08-13): call-site compatibility with cbf_visibility.py's
    # now-mandatory cross-marker `radius` param (controller.py calls
    # cbf2_filter through one shared call site regardless of which module got
    # imported) -- ACCEPTED BUT UNUSED here. ArUco's real 4-corner `corners`
    # array already encodes marker size at every altitude via its own spread
    # (see delta2 below); an explicit radius has no role in this copy.
    """Constrain the lateral accel command for target visibility (cbf2).

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
        delta2 = 0.5 * (ct.max(0) - ct.min(0))                       # actual per-axis half-extent
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
        # A/B before baking. Mirrors the same knob in cbf_visibility.py (cross-marker copy).
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
                                  # mutates th -- mirrors cbf_visibility.py's identical addition
                                  # (2026-08-24, AZ VISIBILITY FILTER v2) -- keep both files in sync.
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
        # ---- Phase 2: no usable corners this tick (decode fail / marker overflow) ----
        # REWRITTEN 2026-08-13. The previous implementation collapsed the per-axis FoV
        # constraint to a SCALAR via min(effective_margin / row_norms) and applied it as
        # an isotropic magnitude clamp with "direction preserved". Three consequences,
        # all measured across 256 recorded Pi flights (see memory
        # project_pi_cbf_phase2_zeroes_lateral_2026_08_12):
        #   1. np.min() collapses a PER-AXIS bound to a scalar, so a marker near ONE
        #      edge removed lateral authority on BOTH axes.
        #   2. "direction preserved" cannot distinguish a command that pushes the marker
        #      further OUT from one that pulls it back IN -- the recovery action was
        #      throttled exactly as hard as the drift causing the problem.
        #   3. effective_margin floors at 0 -> theta_cone = 0 -> a_xy_lim = a_z*tan(0)
        #      = 0, i.e. lateral authority EXACTLY zero. That state self-latches: no
        #      authority -> cannot re-centre -> cannot decode -> stays in Phase 2.
        #      Measured pass-through |I_a_xy| out/in: 1.02 in Phase 1, 0.000 in Phase 2,
        #      over 70% of all ticks and ~100% of every terminal approach.
        # Now: run the SAME signed per-axis box projection Phase 1 uses (_project_box),
        # on a conservatively shrunken box built from the last-known geometry. An anchor
        # already outside the box yields a corrective INWARD lean instead of zero.
        state["decode_fail_n"] = state.get("decode_fail_n", 0) + 1
        fail_thresh = int(env.get("CBF_PHASE2_HYSTERESIS", "20"))    # was 3
        ramp_frames = float(env.get("CBF_PHASE2_RAMP_FRAMES", "20"))  # was 5
        max_frames = int(env.get("CBF_PHASE2_MAX_FRAMES", "60"))      # NEW: give-up horizon
        if state["decode_fail_n"] >= fail_thresh:
            alpha = state.get("phase2_alpha", 0.0)
            # clamp ramp so delta never lags the fill
            state["phase2_alpha"] = min(alpha + 1.0 / ramp_frames, 1.0)
        delta_ref = state.get("delta_prev")
        Lw2_ref = state.get("Lw2_prev")
        p2_alpha = state.get("phase2_alpha", 0.0)
        if (delta_ref is not None and Lw2_ref is not None and p2_alpha > 0
                and state["decode_fail_n"] <= max_frames):
            tau_p2 = float(env.get("CBF_TAU", "0.3"))
            delta_eff = np.asarray(delta_ref, float) * p2_alpha
            ddelta_eff = np.asarray(state.get("ddelta_ref", np.zeros(2)), float) * p2_alpha
            # Defect 3 floor: the box may narrow as the stale marker grows, but must
            # never close -- 1e-3 was numerically "open" yet physically zero authority.
            box_min = float(env.get("CBF_PHASE2_BOX_MIN", "0.05"))
            m2_p2 = np.maximum(np.asarray(p_10, float) - delta_eff - tau_p2 * ddelta_eff,
                               box_min)
            # Lw2_prev is stashed BEFORE Phase 1 applies the CBF_LW_ROT lean->rotation-axis
            # correction (stash site ~line 137 vs rotation ~line 159). The OLD scalar path
            # used only its ROW NORMS, which M leaves unchanged (M orthogonal), so the
            # omission was invisible. A DIRECTIONAL projection needs the rotated form.
            Lw2_p2 = np.asarray(Lw2_ref, float)
            if env.get("CBF_LW_ROT", "1") == "1":
                Lw2_p2 = Lw2_p2 @ np.array([[0.0, 1.0], [-1.0, 0.0]])
            # Same image-axis geometry as Phase 1, recomputed here rather than hoisted so
            # the (working, validated) Phase-1 path is left byte-identical. th_curr comes
            # from ATTITUDE, not the camera -- it is fully available during a decode gap.
            cz, sz = np.cos(yaw_c), np.sin(yaw_c)
            Rzm = np.array([[cz, sz], [-sz, cz]])                 # Rz(-yaw): inertial -> image
            Rz_p90b = np.array([[0.0, -1.0], [1.0, 0.0]])         # Rz(+90deg) camera-mount
            Rz_m90b = np.array([[0.0, 1.0], [-1.0, 0.0]])         # Rz(-90deg) camera-mount
            th_curr = Rz_p90b @ (Rzm @ (-np.asarray(R[:2, 2], float) / max(abs(R33), 1e-3)))
            th_p2 = Rz_p90b @ (Rzm @ (np.asarray(I_a[:2], float) / max(a_z, 1e-6)))
            cr_ref = np.asarray(state.get("cr_prev", np.zeros(2)), float)
            dft_ref = tau_p2 * np.asarray(state.get("d", np.zeros(2)), float)
            anchor_p2 = cr_ref - Lw2_p2 @ th_curr + dft_ref
            th_p2 = _project_box(th_p2, anchor_p2, Lw2_p2, m2_p2)
            # Bound what a STALE anchor may demand. Phase 1 needs no equivalent (its
            # anchor is measured this tick); the caller's theta_cap is 60deg, far too
            # loose to serve as this bound.
            th_max = float(env.get("CBF_PHASE2_THETA_MAX", "0.15"))   # rad, ~8.6 deg
            n_p2 = float(np.linalg.norm(th_p2))
            if n_p2 > th_max:
                th_p2 = th_p2 * (th_max / n_p2)
            I_a[:2] = a_z * (np.array([[cz, -sz], [sz, cz]]) @ (Rz_m90b @ th_p2))
            theta_cone = float(np.linalg.norm(th_p2))
        # else: no usable reference yet, or the anchor is staler than
        # CBF_PHASE2_MAX_FRAMES -- pass the command through UNTOUCHED. A barrier enforced
        # on a guess constrains nothing real while still blocking recovery. Genuine marker
        # loss is owned by MARKER_LOSS_GRACE -> hold -> search-climb -> abort in the app
        # loop, which acts on POSITION rather than tilt and is the right instrument for it.
        # Set CBF_PHASE2_MAX_FRAMES huge to restore always-constrain behaviour.
    return I_a, theta_cone, ok, th_safe, th_desired
