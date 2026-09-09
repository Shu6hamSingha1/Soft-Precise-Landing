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

    Tier 1 -- ``visibility_project``  : hard, this-cycle lean projection, solved
              as a tiny convex QP over the safe lean ``y`` and a per-axis
              visibility slack ``sigma >= 0``:
                  min  1/2||y - y_d||^2 + 1/2 rho ||sigma||^2
                  s.t. |c + Le(y - y_now) + tau*d|_k <= phi_k + sigma_k
                       ||y|| <= y_max            (deliverability: lean/thrust ball)
              ``a_d[2]`` is a fixed input.  The deliverability ball is folded in
              so the returned ``a_xy`` is actuator-feasible BY CONSTRUCTION (no
              post-hoc lean-cap scale-back); the slack gives graceful degradation
              -- visibility stays effectively hard for finite-but-large ``rho``
              while the solve never goes infeasible when the ball and the
              visibility set are disjoint (thrust-saturated / hard off-centre).
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


def condition_drift(h_xy, *, resid=None, resid_gate=0.45, d_max=0.5,
                    med_buf=None, filt=None, lpf_alpha=0.12):
    """Condition the raw translational optic flow ``h_xy`` before it is used as
    the Tier-1 moving-target lead ``d``.

    The 7-profile rover sweep (2026-09-09) showed raw ``h_xy`` is unusable as a
    lead: single-frame spikes to |d| 4-16 tangent/s on aggressive trajectories,
    and |d| ~ 0.1-2.5 of pure noise on a *static* target (there is no drift to
    lead). Real target drift at the tested speeds is |d|_p50 ~ 0.1.

    Pipeline:  validity gate -> median-of-3 (kills isolated spikes) -> 1-pole LPF
               -> radial magnitude clamp to ``d_max``.

    ``h_xy``      : (2,) raw translational flow this frame (image-tangent frame).
    ``resid``     : the flow solve's own ``rel_resid`` (||A@sol-b||/||b||), or None
                    to skip the confidence gate. ``> resid_gate`` (or non-finite)
                    -> the whole ``h`` vector is untrusted this frame -> feed 0.
    ``d_max``     : hard cap on |d| (tangent/s). Direction preserved.
    ``med_buf``   : list of up to 3 recent *gated* h_xy (caller-held); updated and
                    returned.
    ``filt``      : (2,) LPF carry (caller-held); updated and returned.
    ``lpf_alpha`` : 1-pole coefficient (~dt / (tau_lpf + dt)); smaller = smoother.

    Returns ``(d, med_buf, filt)``.
    """
    h_xy = np.asarray(h_xy, float).reshape(2)
    med_buf = [] if med_buf is None else list(med_buf)
    filt = np.zeros(2) if filt is None else np.asarray(filt, float).reshape(2).copy()

    gated = (np.zeros(2) if (resid is not None
                             and (not np.isfinite(resid) or resid > resid_gate))
             else h_xy)

    med_buf.append(gated)
    med_buf = med_buf[-3:]
    m = np.median(np.stack(med_buf, axis=0), axis=0)

    filt = filt + float(lpf_alpha) * (m - filt)

    n = float(np.linalg.norm(filt))
    d = filt * (d_max / n) if n > d_max > 0.0 else filt.copy()
    return d, med_buf, filt


def fov_limit(center_px, focal_px, buffer_frac=0.15):
    """Per-axis FoV-edge tangent half-extent minus the buffer -> phi.
    ``buffer_frac`` may be a scalar or a per-axis (2,) array."""
    return ((np.asarray(center_px, float) / np.asarray(focal_px, float))
            * (1.0 - np.asarray(buffer_frac, float)))


# ===========================================================================
# TIER 1 -- hard, this-cycle lean projection
# ===========================================================================
def _solve_tier1(y_d, anchor, Le, phi, y_max, rho, n_iter):
    """min  1/2||y-y_d||^2 + 1/2 rho sum_k max(|g_k(y)|-phi_k, 0)^2
       s.t. ||y|| <= y_max,     g(y) = anchor + Le@y.
    The >=0 slack is eliminated in closed form (sigma_k = max(|g_k|-phi_k, 0)),
    leaving a smooth convex 2-D problem.  Projected Newton finds the interior
    (minimal-intervention) optimum exactly; when the ball binds -- the
    ball-vs-visibility disjoint case -- Newton only projects radially and can
    stall off the true circle optimum, so that case is finished with a 1-D search
    over the circle angle.  Returns (y, slack)."""
    y_d = np.asarray(y_d, float)

    def _obj(yv):
        over = np.maximum(np.abs(anchor + Le @ yv) - phi, 0.0)
        return 0.5 * float(np.dot(yv - y_d, yv - y_d)) + 0.5 * rho * float(np.dot(over, over))

    if y_max <= 0.0:                       # thrust-saturated: no lean budget at all
        return np.zeros(2), np.maximum(np.abs(anchor) - phi, 0.0)

    y = y_d.copy()
    nrm = float(np.linalg.norm(y))
    if np.isfinite(y_max) and nrm > y_max:
        y *= y_max / nrm
    I2 = np.eye(2)
    for _ in range(max(int(n_iter), 1)):
        g = anchor + Le @ y
        over = np.abs(g) - phi
        grad = y - y_d
        H = I2.copy()
        for k in (0, 1):
            if over[k] > 0.0:
                r = Le[k]
                grad = grad + rho * over[k] * np.sign(g[k]) * r
                H = H + rho * np.outer(r, r)
        y_new = y - np.linalg.solve(H, grad)
        nrm = float(np.linalg.norm(y_new))
        if np.isfinite(y_max) and nrm > y_max:
            y_new *= y_max / nrm
        if float(np.linalg.norm(y_new - y)) < 1e-10:
            y = y_new
            break
        y = y_new

    # ball binding -> refine on the circle ||y|| = y_max (1-D, convex enough for a
    # coarse grid + golden-section; keeps the better of the two iterates)
    if np.isfinite(y_max) and float(np.linalg.norm(y)) >= y_max * (1.0 - 1e-6):
        th = np.linspace(0.0, 2.0 * np.pi, 289)
        pts = y_max * np.stack([np.cos(th), np.sin(th)], axis=1)
        j = int(np.argmin([_obj(p) for p in pts]))
        lo, hi = th[j] - (th[1] - th[0]), th[j] + (th[1] - th[0])
        gr = 0.5 * (np.sqrt(5.0) - 1.0)
        a, b = lo, hi
        c1, c2 = b - gr * (b - a), a + gr * (b - a)
        for _ in range(40):
            p1 = y_max * np.array([np.cos(c1), np.sin(c1)])
            p2 = y_max * np.array([np.cos(c2), np.sin(c2)])
            if _obj(p1) < _obj(p2):
                b, c2 = c2, c1
                c1 = b - gr * (b - a)
            else:
                a, c1 = c1, c2
                c2 = a + gr * (b - a)
        y_circ = y_max * np.array([np.cos(0.5 * (a + b)), np.sin(0.5 * (a + b))])
        if _obj(y_circ) < _obj(y):
            y = y_circ

    slack = np.maximum(np.abs(anchor + Le @ y) - phi, 0.0)
    return y, slack


def visibility_project(a_d, R, yaw, marker_center_px, center_px, focal_px,
                       buffer_frac=0.15, tau=0.0, drift=None, n_iter=6,
                       a_cap=None, rho=2000.0):
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
    ``n_iter`` : projected-Newton iterations for the Tier-1 QP (4-6 is plenty).
    ``a_cap`` : thrust magnitude the caller can deliver (same units as ``a_d``),
              or None to skip the deliverability ball (post-hoc caps then stay
              the caller's job).  With it set, ``||y|| <= sqrt(a_cap^2/a_z^2 - 1)``
              == the lean cap ``arccos(a_z/a_cap)`` -- folded in so ``a_star`` is
              actuator-feasible by construction.
    ``rho`` : visibility-slack penalty weight.  Large -> visibility effectively
              hard; finite -> graceful trade against ``||y - y_d||`` (and the
              ball) when the sets conflict.

    Returns ``(a_star, y_star, info)`` where ``y_star`` is the safe lean (image
    axes) for a caller that builds attitude directly, and ``info`` is a dict with
    ``c``, ``phi``, ``y_now``, ``y_desired``, ``active``, ``slack``, ``y_max``,
    ``deliverable``.
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

    # deliverability ball on the lean tangent: ||a*|| = a_z*sqrt(1+||y||^2) <= a_cap
    if a_cap is None:
        y_max = np.inf
    else:
        a_cap = float(a_cap)
        y_max = np.sqrt(max(a_cap * a_cap / (a_z * a_z) - 1.0, 0.0)) if a_cap > a_z else 0.0

    if marker_center_px is None or abs(float(a_d[2])) < _AZ_MIN:
        info = dict(c=None, phi=phi, y_now=None, y_desired=y_d, active=False,
                    slack=np.zeros(2), y_max=y_max, deliverable=True)
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

    # moving-target lead tau*d, but never PREDICT the centre crossing to the
    # opposite side of an axis: a linear extrapolation that flips sign is past the
    # model's validity (and would ask for a backwards correction).  Clamp the
    # predicted centre to at most the axis origin on any inward-overshooting axis.
    lead = float(tau) * drift
    _pred = c + lead
    lead = np.where((np.sign(_pred) != np.sign(c)) & (c != 0.0), -c, lead)
    anchor = c - Le @ y_now + lead

    # Tier-1 QP: minimal change to the desired lean s.t. the predicted centre
    # stays inside phi (softly, weight rho) AND the lean is deliverable (hard
    # ball).  Zero penalty gradient at y_d whenever y_d already satisfies both
    # -> minimal intervention, inward/tangential moves stay free.
    y_s, slack = _solve_tier1(y_d, anchor, Le, phi, y_max, float(rho), n_iter)

    a_star = a_d.copy()
    a_star[:2] = a_z * (P.T @ y_s)
    deliverable = bool(np.max(slack) <= 1e-6)   # residual < ~0.1 px -> visibility met
    info = dict(c=c, phi=phi, y_now=y_now, y_desired=y_d,
                slack=slack, y_max=y_max, deliverable=deliverable,
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
                             buffer_frac=0.15, tau=0.0, drift=None, n_iter=6,
                             a_cap=None, rho=2000.0,
                             descent_ease_on=True, g=9.81, g_min=0.2, t_react=1.5,
                             gz_tau=0.25, state=None):
    """Tier 1 then (optionally) Tier 2. Returns ``(a_star, y_star, info)`` with
    ``info`` extended by ``g_z`` and ``c`` reused for the caller's next-step
    ``prev_center_tangent``.
    """
    a1, y1, info = visibility_project(a_d, R, yaw, marker_center_px, center_px,
                                      focal_px, buffer_frac, tau, drift, n_iter,
                                      a_cap=a_cap, rho=rho)
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
