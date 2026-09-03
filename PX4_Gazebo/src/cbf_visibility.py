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
                p_10, theta_cone, dt_last, w_rp, state, radius, env=None, h_z=0.0,
                A_CAP=None, g=9.81):
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
        ``CBF_TAU, CBF_DMIN_EMA, CBF_PHASE2_HYSTERESIS, CBF_PHASE2_RAMP_FRAMES,
        CBF_HZ_AWARE_DRIFT, CBF_JOINT_QP``.
    A_CAP : float, optional
        Vehicle's max achievable thrust/mass (``controller.py``'s module-level
        ``A_CAP``). Default ``None`` disables ``CBF_JOINT_QP`` regardless of the
        env var (an extra safety net so callers that don't pass it can't
        accidentally enable the joint solve; ``controller.py`` itself always
        passes it, so the joint path is what actually runs there).
        2026-08-29 (user design, DEFAULT ON as of 2026-08-29): with
        ``CBF_JOINT_QP`` unset or ``=1``, solves the FoV visibility box directly for the full
        ``I_a`` (lateral AND vertical), interleaved with a projection onto the
        deliverability sphere -- ``|I_a|<=A_CAP`` with ``CBF_SPHERE_TRUE_THRUST=1``, or the
        legacy default ``|I_a+g*e3|<=A_CAP`` which despite its name bounds VEHICLE
        acceleration (zero at hover), not thrust; see the warning block at the projection --
        rather than
        treating ``a_z`` as a fixed given and patching it separately downstream
        (the ``PLASMC_AZ_JOINT`` controller.py-level approach). See the
        ``CBF_JOINT_QP`` block below for the derivation (solving directly in
        ``I_a`` units via ``M = Lw2@P/a_z`` instead of the theta-normalized
        ``Lw2@theta``, per the user's own "why does theta need to exist at all"
        question this session).
    g : float, optional
        Gravity (m/s^2), used only by the ``CBF_JOINT_QP`` sphere projection to
        build the gravity-shifted thrust vector, same convention as
        controller.py's own ``I_a + g*e3``. Default 9.81 (unused unless
        ``CBF_JOINT_QP`` engages).
    h_z : float, optional
        Scale-free loom/closing-rate proxy (``self._h[-1][2]``, ``~= -Zdot/Z``,
        negative while closing/descending -- see the codebase's touchdown-detect
        sign convention). Default 0.0 (fully backward compatible: with h_z=0 the
        drift extrapolation below is byte-identical to the pre-2026-08-29
        constant-velocity formula). 2026-08-29 (user design): the drift term's
        constant-velocity extrapolation (``dft = tau*d``) assumes the CURRENT
        measured drift rate ``d`` holds steady over the lookahead horizon
        ``tau`` -- but under continued descent with any lateral offset, the
        normalized/tangent-space feature position scales roughly as
        ``offset/Z``, so translational drift itself ACCELERATES as Z shrinks
        (the same closing-rate factor h_z measures). The constant-velocity
        model systematically undershoots true near-touchdown drift, making the
        QP reactive (only tightens after drift has already ramped up) instead
        of proactive. Gated behind ``CBF_HZ_AWARE_DRIFT`` (default "0") pending
        IC3/IC5 validation -- see the accelerating-extrapolation derivation at
        the ``dft`` computation below.

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
            # HZ-AWARE ACCELERATING DRIFT (2026-08-29, user design, gated behind
            # CBF_HZ_AWARE_DRIFT, default off): the plain `tau*d` extrapolation
            # below assumes the measured drift rate `d` holds CONSTANT over the
            # lookahead horizon `tau`. Under continued descent it doesn't --
            # translational drift scales with 1/Z, so as Z keeps shrinking during
            # that same horizon the true drift rate keeps growing. Model d(s) ~
            # d(0)*exp(closing_rate*s) for s in [0,tau] (closing_rate = max(-h_z,
            # 0), only amplified while actually closing -- h_z<0 -- never while
            # climbing/hovering) and integrate: displacement = d(0) *
            # (exp(closing_rate*tau)-1)/closing_rate. Reduces EXACTLY to the
            # original tau*d as closing_rate->0 (backward-compatible at hover/
            # h_z=0, verified via the Taylor limit below), grows super-linearly
            # as closing_rate increases -- makes the QP anticipate near-touchdown
            # drift instead of only reacting after it's already ramped up.
            if env.get("CBF_HZ_AWARE_DRIFT", "0") == "1":
                _closing_rate = max(-float(h_z), 0.0)
                if _closing_rate * tau > 1e-6:
                    dft = state["d"] * (np.expm1(_closing_rate * tau) / _closing_rate)
                else:
                    dft = tau * state["d"]   # Taylor limit as closing_rate*tau->0
            else:
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
        # CBF_JOINT_QP (2026-08-29, user design, DEFAULT ON as of 2026-08-29): solve the visibility box
        # DIRECTLY for I_a (lateral AND vertical) instead of the theta-normalized lean
        # vector, interleaved with a projection onto a deliverability sphere (see the
        # WRONG-QUANTITY warning at that projection; CBF_SPHERE_TRUE_THRUST selects the
        # correct |I_a|<=A_CAP form). Derivation: theta = P@I_a[:2]/a_z where P = Rz_p90b@Rzm (the
        # SAME forward rotation used to build `th` above), so Lw2@theta =
        # (Lw2@P/a_z)@I_a[:2] -- the box constraint's Jacobian w.r.t. I_a[:2] directly is
        # M = Lw2@P/a_z. Solving in I_a units removes the theta round-trip (compute I_a ->
        # normalize by a_z -> solve -> un-normalize) entirely; a_z itself becomes part of
        # the SAME iteration (outer loop) rather than a fixed input reconstructed
        # downstream (the PLASMC_AZ_JOINT controller.py-level approach). Requires A_CAP
        # (falls through to the theta-based path if not provided, e.g. old callers/tests).
        _joint_qp = env.get("CBF_JOINT_QP", "1") == "1" and A_CAP is not None and A_CAP > 0
        _az_cost_gain = float(env.get("CBF_AZ_COST_GAIN", "5.0"))   # m/s^2 descent-rate relief per rad of box-suppressed tilt; 0 disables
        # CBF_SPHERE_TRUE_THRUST (2026-09-03): 1 = bound the ACTUAL thrust |I_a| <= A_CAP;
        # 0 (default, legacy) = the gravity-shifted |I_a+g*e3| <= A_CAP, which bounds vehicle
        # acceleration instead. See the block comment at the projection below for why the
        # legacy form is wrong and why the fix is not defaulted on yet (needs the IC2-5 gate).
        _true_thrust_sphere = env.get("CBF_SPHERE_TRUE_THRUST", "0") == "1"
        if _joint_qp:
            P = Rz_p90b @ Rzm                                        # forward inertial->image rotation (pre a_z-scale)
            Ia_lat = np.asarray(I_a[:2], float).copy()                # start from the UNCONSTRAINED desired lateral accel
            Ia_z = float(I_a[2])
            for _outer in range(6):
                _az_now = max(abs(Ia_z), 1e-6)
                M = (Lw2 @ P) / _az_now                              # box-constraint Jacobian w.r.t. I_a[:2] at the CURRENT a_z estimate
                for _inner in range(5):                              # box projection (same alternating-projection algorithm, on I_a directly)
                    f = anchor + M @ Ia_lat
                    for k in range(2):
                        if f[k] > m2[k]:
                            r = M[k]; Ia_lat = Ia_lat - (f[k] - m2[k]) / (r @ r + 1e-12) * r
                        elif f[k] < -m2[k]:
                            r = M[k]; Ia_lat = Ia_lat - (f[k] + m2[k]) / (r @ r + 1e-12) * r
                # a_z SOFT COST (CBF_AZ_COST_GAIN, default on). Folds in what
                # controller.py's downstream _dtheta_correction used to do -- "if the FoV
                # box just had to suppress lateral demand, slow the descent so the lateral
                # loop gets more time" -- but INSIDE this constrained solve, so the output
                # I_a stays self-consistent (the old bolt-on modified I_a[2] AFTER the QP,
                # outside its constraint set, and could pile unbounded lift on top of the
                # z-SMC -> the terminal climb-away). Weight = ||th_desired - th_safe||, the
                # radians of tilt the box suppressed this cycle. HARD SAFETY: the relief
                # can bring the descent-accel command toward hover (-g) but NEVER past it,
                # so this term alone can only slow a descent, never reverse it into a
                # climb. Only engages while the z-SMC is itself commanding a descent
                # (I_a[2] > -g); if the SMC or the sphere already command lift, leave it.
                if _az_cost_gain > 0.0 and float(I_a[2]) > -g:
                    _th_safe_now = P @ (Ia_lat / _az_now)
                    _lat_supp = float(np.linalg.norm(th_desired - _th_safe_now))
                    _az_relieved = float(I_a[2]) - _az_cost_gain * _lat_supp   # more negative = more lift = slower descent
                    Ia_z = min(Ia_z, max(_az_relieved, -g))                    # min: deliverability/SMC lift wins; max(-g): relief can't command a climb
                # DELIVERABILITY SPHERE.
                #
                # ⚠ 2026-09-03 -- the legacy form below (`|I_a + g*e3| <= A_CAP`, kept as the
                # default until an IC2-5 gate clears the fix) DOES NOT BOUND THRUST. `I_a` is
                # THRUST acceleration in NED with hover at `I_a[2] = -g` (controller.py:3682-3689,
                # and the B_T mapping at controller.py:4008-4012 depends on exactly that), so
                #     I_a + g*e3 == a_vehicle
                # and the legacy sphere bounds the VEHICLE's acceleration -- zero at hover --
                # not the thrust the rotors must produce. Consequences, measured:
                #   * at zero lateral it admits I_a[2] in [-23.41, +3.81] m/s^2, i.e. thrust accel
                #     up to 2.39 g when A_CAP is 1.389 g -- a 72% over-permit;
                #   * a 2 g commanded climb (I_a=[0,0,-2g], |I_a|=19.60 > A_CAP=13.61) passes,
                #     because its DEVIATION from hover is only g=9.80 < A_CAP;
                #   * it is inconsistent with THETA_CAP_DEG_DERIVED = arccos(g/A_CAP)
                #     (controller.py:197) and with the arccos(a_z/A_CAP) sanity clip below, BOTH
                #     of which are written in total-thrust semantics.
                # Note the shift is not merely mis-signed: under the opposite reading (I_a as
                # vehicle accel) the thrust vector would be `I_a - g*e3`. Neither convention
                # yields `+g`, so there is no interpretation under which the legacy form is right.
                #
                # In 6 recent reps (5010 samples) the legacy sphere peaked at 4.32-7.70 vs the
                # 13.61 cap in clean flight (never binding), while the CORRECT bound sits at
                # 10.94-12.50 (~92% of cap) -- so this is not a cosmetic fix: enabling it makes
                # the constraint start regulating in normal descent. Hence env-gated + default
                # OFF pending the mandatory IC2-5 gate, per feedback_ic_validation and the
                # Rz_p90b precedent (a synthetic-validated CBF "fix" that regressed every IC).
                if _true_thrust_sphere:
                    thrust_vec = np.array([Ia_lat[0], Ia_lat[1], Ia_z])       # I_a IS the thrust accel
                    _T = float(np.linalg.norm(thrust_vec))
                    if _T > A_CAP and _T > 1e-9:
                        thrust_vec = thrust_vec * (A_CAP / _T)
                        Ia_lat = thrust_vec[:2].copy()
                        Ia_z = float(thrust_vec[2])
                else:
                    thrust_vec = np.array([Ia_lat[0], Ia_lat[1], Ia_z + g])   # LEGACY: bounds a_vehicle
                    _T = float(np.linalg.norm(thrust_vec))
                    if _T > A_CAP and _T > 1e-9:
                        thrust_vec = thrust_vec * (A_CAP / _T)
                        Ia_lat = thrust_vec[:2].copy()
                        Ia_z = float(thrust_vec[2] - g)
            _az_final = max(abs(Ia_z), 1e-6)
            th_safe = P @ (Ia_lat / _az_final)                        # derived, for Fix B / dtheta consumers -- not the QP's own variable here
            # SANITY CLIP (2026-08-29, caught in offline stress-testing before SITL): the
            # sphere projection above bounds THRUST MAGNITUDE, not the derived angle
            # RATIO -- if Ia_z ends up small while Ia_lat stays large, th_safe can still
            # blow up non-physically (observed: 3.52 rad, >200deg, in a 500-trial extreme-
            # input sweep). Same class of bug as the first PLASMC_AZ_JOINT draft. Clip to
            # the SAME a_z-aware bound (arccos(a_z/A_CAP), always finite 0..pi/2) validated
            # there, and keep I_a[:2] consistent with the (possibly-clipped) th_safe.
            _cap_eff_j = float(np.arccos(np.clip(_az_final / A_CAP, -1.0, 1.0)))
            _tn_j = float(np.linalg.norm(th_safe))
            if _tn_j > _cap_eff_j:
                th_safe = th_safe * (_cap_eff_j / max(_tn_j, 1e-9))
                Ia_lat = _az_final * (P.T @ th_safe)                  # P orthogonal -> P.T == P^-1; keep I_a[:2]/az ratio == clipped th_safe exactly
            I_a[:2] = Ia_lat
            I_a[2] = Ia_z                                             # NOTE: unlike the theta-based path, this branch CAN modify I_a[2]
            theta_cone = float(np.linalg.norm(th_safe))
        else:
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
