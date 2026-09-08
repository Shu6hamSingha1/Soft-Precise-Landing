"""visibility_projection.py -- keep the marker in the camera image plane by
conditioning the acceleration command. Clean-slate replacement for
cbf_visibility.py / cbf_visibility_aruco.py and the joint-QP / deliverability-
sphere / descent-rate-relief / two-phase-delta machinery.

REQUIREMENT
-----------
1. Keep the marker's measured centre inside the REAL camera image plane (NOT a
   de-rotated / virtual plane), with a fixed buffer from the FoV edge.
   Cross marker -> the visibility target is a single point (the crossed-lines
   intersection); no 4-corner containment, no marker-size term.
2. Do it by making the SMALLEST useful change to the desired inertial
   acceleration command.

TWO TIERS -- different physical timescales, so NOT one fused QP
--------------------------------------------------------------
The command reaches the marker centre ``c`` (tangent units = px / focal) through
two channels:

  * ATTITUDE (lean).  A tilt rotates the FoV -> ``c`` shifts THIS control step,
    depth-free.  This is the HARD constraint.
  * DESCENT rate.  ``|c| ~= |offset| / Z`` grows as ``Z`` shrinks, at a rate
    ``~ |c| * (-Zdot / Z)``.  A change in the vertical command only affects ``c``
    two integrators later -- so descent is a SOFT, predictive influence: if the
    marker will hit the buffer before the lateral loop can re-centre it, ease off
    the descent to buy that loop time.

    Tier 1 -- ``visibility_project``  : hard, this-cycle lean projection.
              ``a_d[2]`` is a fixed input; ``a_xy`` is projected so
              ``|c + Lw@M@(y - y_now) + tau*d|_k <= phi_k``.
    Tier 2 -- ``descent_ease``        : soft, self-releasing scale ``g_z in
              [g_min, 1]`` on the DOWNWARD part of ``a_d[2]``, driven by the
              measured time-for-``|c|``-to-reach-the-edge.  Never reverses a
              descent (floor ``g_min > 0``, hard clamp at hover).

Neither tier reads a separate "descent reference" -- both only alter the
already-generated command (requirement 2).  Thrust deliverability
(``||a*|| <= A_cap`` / a lean cap) stays the CALLER's concern, applied last.

WHY TIER 2 IS NOT THE OLD ``CBF_AZ_COST_GAIN`` RELIEF
----------------------------------------------------
  * trigger is a MEASURED external quantity (|c| and its rate), not a QP-internal
    norm that self-inflates;
  * it runs AFTER Tier 1, scaling ``a_z`` once -- it never fights the lean solve;
  * ``g_z`` is a bounded SCALE (``g_min>0`` always descends), not a
    ``min(., -g)`` that can pin at hover;
  * it self-releases: the bought time shrinks ``|c|`` -> margin grows -> g_z->1.

SCOPE
-----
Guarantees the centre stays geometrically inside ``phi`` WHEN perception reports
one.  Not a perception fix (terminal overfill still corrupts detection).
"""
import numpy as np

# lean-direction -> rotation-axis: to lean +x you rotate about +y.  omega = M @ y.
_M = np.array([[0.0, 1.0], [-1.0, 0.0]])
# camera-mount tangent swap: body/FRD tangent (x, y) -> image tangent (y, -x) == Rz(-90deg).
# (physical fact of the sim's downward camera: yaw += 90deg on the down-pitch; matches _getVirtualPts)
_SWAP = np.array([[0.0, 1.0], [-1.0, 0.0]])

_AZ_MIN = 0.5            # m/s^2 : below this the lean a_xy/a_z is meaningless (near free-fall)
_R33_MIN = 1.0e-3


def _P(yaw):
    """Rz(+90deg) @ Rz(-yaw) : inertial x-y -> image axes (pre a_z scale)."""
    cz, sz = np.cos(yaw), np.sin(yaw)
    return np.array([[0.0, -1.0], [1.0, 0.0]]) @ np.array([[cz, sz], [-sz, cz]])


def marker_tangent(marker_center_px, center_px, focal_px):
    """Raw detection-frame pixel centre -> image-frame tangent (camera-mount swap applied)."""
    return _SWAP @ ((np.asarray(marker_center_px, float).reshape(2)
                     - np.asarray(center_px, float)) / np.asarray(focal_px, float))


def fov_limit(center_px, focal_px, buffer_frac=0.15):
    """Per-axis FoV-edge tangent half-extent minus the buffer -> phi.
    ``buffer_frac`` may be a scalar or a per-axis (2,) array."""
    return ((np.asarray(center_px, float) / np.asarray(focal_px, float))
            * (1.0 - np.asarray(buffer_frac, float)))


# ===========================================================================
# TIER 1 -- hard, this-cycle lean projection
# ===========================================================================
def visibility_project(a_d, R, yaw, marker_center_px, center_px, focal_px,
                       buffer_frac=0.15, tau=0.0, drift=None, n_iter=4):
    """Minimal change to ``a_d[:2]`` so the marker centre stays inside ``phi``.

    ``a_d`` : (3,) desired inertial specific-thrust command (NED, hover a_d[2]=-g).
              ``a_d[2]`` is passed through unchanged.
    ``R``   : (3,3) body->inertial DCM (current attitude).
    ``yaw`` : control yaw (rad).
    ``marker_center_px`` : (2,) raw detection-frame pixels, or None -> passthrough.
    ``center_px``/``focal_px`` : (2,) intrinsics for the detection frame.
    ``buffer_frac`` : edge margin as a fraction of the half-FoV. One number, meant
              to also absorb one-cycle linearisation error + attitude-tracking lag.
    ``tau``/``drift`` : moving-target look-ahead (s) and measured centre drift
              rate (tangent units/s, gyro-stripped); tau=0 disables.
    ``n_iter`` : alternating-projection sweeps over the two rows (3-4 is plenty).

    Returns ``(a_star, y_star, info)`` where ``y_star`` is the safe lean (image
    axes) for a caller that builds attitude directly, and ``info`` is a dict with
    ``c``, ``phi``, ``y_now``, ``y_desired``, ``active``.
    """
    a_d = np.asarray(a_d, float).reshape(3)
    R = np.asarray(R, float)
    center_px = np.asarray(center_px, float)
    focal_px = np.asarray(focal_px, float)
    drift = np.zeros(2) if drift is None else np.asarray(drift, float).reshape(2)

    a_z = max(abs(float(a_d[2])), _AZ_MIN)
    P = _P(yaw)
    y_d = P @ (a_d[:2] / a_z)
    phi = fov_limit(center_px, focal_px, buffer_frac)

    if marker_center_px is None or abs(float(a_d[2])) < _AZ_MIN:
        info = dict(c=None, phi=phi, y_now=None, y_desired=y_d, active=False)
        return a_d.copy(), y_d, info

    c = marker_tangent(marker_center_px, center_px, focal_px)
    x, y = float(c[0]), float(c[1])
    Lw = np.array([[x * y, -(1.0 + x * x)],
                   [1.0 + y * y, -x * y]])
    # d(c)/d(lean), linearised at the MEASURED feature.  Sign fixed against an
    # independent pinhole+attitude model (tools/validate_visibility_projection.py
    # check 1): for the DOWNWARD camera the feature moves opposite to the standard
    # forward-camera IBVS Lw@M.  The single-point linearisation is <~8% off for a
    # per-cycle lean CHANGE up to ~25deg (the realistic regime at 50 Hz with a
    # smooth outer loop); ``buffer_frac`` absorbs that residual.  It is NOT
    # accurate for a one-shot 50deg correction -- rely on the caller's lean cap
    # and the incremental (every-cycle) nature of the correction for that.
    Le = -(Lw @ _M)

    R33 = float(R[2, 2])
    R33 = np.sign(R33) * max(abs(R33), _R33_MIN) if R33 != 0.0 else _R33_MIN
    y_now = P @ (-np.asarray(R[:2, 2], float) / R33)

    anchor = c - Le @ y_now + float(tau) * drift

    # alternating projection from the unconstrained desired lean; a row is touched
    # ONLY when predicted to breach -> minimal intervention, inward/tangential free.
    y_s = y_d.copy()
    for _ in range(max(int(n_iter), 1)):
        f = anchor + Le @ y_s
        for k in (0, 1):
            r = Le[k]
            rr = float(r @ r) + 1e-12
            if f[k] > phi[k]:
                y_s = y_s - (f[k] - phi[k]) / rr * r
            elif f[k] < -phi[k]:
                y_s = y_s - (f[k] + phi[k]) / rr * r
            f = anchor + Le @ y_s

    a_star = a_d.copy()
    a_star[:2] = a_z * (P.T @ y_s)
    info = dict(c=c, phi=phi, y_now=y_now, y_desired=y_d,
                active=not np.allclose(a_star[:2], a_d[:2], atol=1e-9, rtol=0.0))
    return a_star, y_s, info


# ===========================================================================
# TIER 2 -- soft, predictive descent ease
# ===========================================================================
def descent_ease(a_z_cmd, c, phi, c_rate, g,
                 g_min=0.2, t_react=1.5, dt=0.02, gz_tau=0.25, state=None):
    """Scale the DOWNWARD part of ``a_z_cmd`` toward hover when the marker centre
    is predicted to reach the buffer before the lateral loop can re-centre it.

    ``a_z_cmd`` : float, the (Tier-1-passed-through) vertical command a_star[2].
    ``c``       : (2,) measured marker centre, image-frame tangent (info['c']).
    ``phi``     : (2,) FoV limit (info['phi']).
    ``c_rate``  : (2,) measured d(c)/dt, tangent units/s (finite-diff of c,
                  lightly filtered). Only the OUTWARD component matters.
    ``g``       : gravity (m/s^2), so hover is a_z = -g.
    ``g_min``   : descent-scale floor (>0 -> always some descent). 0.2 -> 20%.
    ``t_react`` : lateral-loop reaction timescale (s). If the marker won't hit the
                  buffer for longer than this, no easing (g_z = 1).
    ``dt``      : control step (s), for the g_z low-pass.
    ``gz_tau``  : 1-pole time constant on g_z so a_z doesn't step.
    ``state``   : dict carrying ``gz`` between calls (created if None).

    Returns ``(a_z_star, g_z, state)``. ``a_z_star == a_z_cmd`` when not
    descending or when g_z == 1.
    """
    if state is None:
        state = {}
    gz_prev = float(state.get("gz", 1.0))

    w_cmd = float(a_z_cmd) + g              # commanded vehicle vertical accel: >0 descending
    if c is None or w_cmd <= 0.0:
        state["gz"] = gz_prev + (dt / max(gz_tau, dt)) * (1.0 - gz_prev)
        return float(a_z_cmd), state["gz"], state

    c = np.asarray(c, float).reshape(2)
    phi = np.asarray(phi, float).reshape(2)
    c_rate = np.asarray(c_rate, float).reshape(2)

    # worst axis: smallest remaining margin, and how fast it is closing
    rem = np.maximum(phi - np.abs(c), 0.0)                  # tangent distance to the buffer, per axis
    closing = np.maximum(np.sign(c) * c_rate, 0.0)          # outward speed of |c|, per axis (>=0)
    with np.errstate(divide="ignore", invalid="ignore"):
        t_edge = np.where(closing > 1e-6, rem / closing, np.inf)
    t_min = float(np.min(t_edge))

    frac = np.clip(t_min / max(t_react, 1e-6), 0.0, 1.0)
    g_t = g_min + (1.0 - g_min) * frac                      # target scale
    g_z = gz_prev + (dt / max(gz_tau, dt)) * (g_t - gz_prev)
    state["gz"] = g_z

    a_z_star = w_cmd * g_z - g                              # scale only the downward part
    return float(a_z_star), float(g_z), state


# ===========================================================================
# convenience: run both tiers
# ===========================================================================
def condition_for_visibility(a_d, R, yaw, marker_center_px, center_px, focal_px,
                             prev_center_tangent=None, dt=0.02, *,
                             buffer_frac=0.15, tau=0.0, drift=None, n_iter=4,
                             descent_ease_on=True, g=9.81, g_min=0.2, t_react=1.5,
                             gz_tau=0.25, state=None):
    """Tier 1 then (optionally) Tier 2. Returns ``(a_star, y_star, info)`` with
    ``info`` extended by ``g_z`` and ``c`` reused for the caller's next-step
    ``prev_center_tangent``.
    """
    a1, y1, info = visibility_project(a_d, R, yaw, marker_center_px, center_px,
                                      focal_px, buffer_frac, tau, drift, n_iter)
    info["g_z"] = 1.0
    if descent_ease_on and info["c"] is not None:
        c_rate = (np.zeros(2) if prev_center_tangent is None
                  else (info["c"] - np.asarray(prev_center_tangent, float)) / max(dt, 1e-3))
        az, gz, state = descent_ease(a1[2], info["c"], info["phi"], c_rate, g,
                                     g_min=g_min, t_react=t_react, dt=dt,
                                     gz_tau=gz_tau, state=state)
        a1 = a1.copy()
        a1[2] = az
        info["g_z"] = gz
        info["state"] = state
    return a1, y1, info
