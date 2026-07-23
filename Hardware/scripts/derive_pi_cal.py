#!/usr/bin/env python3
"""Derive the Pi hardware sensor-calibration matrix from output_calibration.py
mocap recordings. Pi-adapted port of PX4_Gazebo/tools/gt_optical_flow.py's
canonical GT-flow method + derive_board_cal.py's lstsq fit, for QTM mocap
instead of Gazebo's simulated ROS Pose/quaternion GT.

CONFIRMED FRAME TRANSFORM (user, 2026-07-10, "match drone FRD with mocap
XYZ" calibration run): QTM reports X=Forward, Y=Left, Z=Up (right-handed
FLU, standard optical-mocap convention). The drone/controller body frame is
FRD (X=Forward, Y=Right, Z=Down). FLU -> FRD is a 180 degree rotation about
the shared X (forward) axis:
    pos_FRD = diag(1, -1, -1) @ pos_QTM
    yaw_FRD = -yaw_QTM   (rotation about the flipped Z axis also flips sign)
    R_FRD (body->world) = T @ R_FLU(euler) @ T,  T = diag(1,-1,-1)
(T is orthogonal and self-inverse, so this is a valid similarity transform
applying the SAME axis flip to both the world and body sides of the DCM -
correct because mocaptools reports body roll/pitch/yaw in QTM's own FLU
convention, not already in FRD.)

Gravity ("down") is +Z in this FRD-consistent frame, matching Gazebo's NED
convention exactly, so the rest of the V-frame math (_v_frame, gravity-
leveled bearing/flow) is a direct port with no further changes needed.

*** PROVISIONAL / FIRST-PASS: the calibration below has NOT been validated
against a second independent recording, R^2 has not been reviewed, and it
must not be pasted into img_data.py without that review (see PX4_Gazebo's
own workflow: derive -> inspect R^2/inter-run stability -> THEN apply). ***

FILTER: KF, not Savgol (fixed 2026-07-11, mirroring a bug PX4_Gazebo found
and fixed the same day, feedback_kf_savgol_cal_mismatch). img_data.py's
runtime getOptFlowAngVel() defaults to IMG_FILTER='kf' - fitting a
calibration against a Savgol-filtered raw signal and applying it to a
differently-shaped KF-filtered signal at runtime is not guaranteed valid.
kf_filter_causal() below is a direct port of aggregate_calibration_phased.py's
version, which is itself kept in lockstep with img_data.py::_kf_step - same
process/measurement model, using the Pi's own FLOW_KF_Q/R and
IMG_FEAT_KF_Q/R defaults (5.0/0.1 and 5.0/0.004).

Usage:
    python3 derive_pi_cal.py [<run_dir> ...]     # defaults to all GOOD runs
                                                   # under Test_Data/Calibration
"""
import os
import sys
import glob
import numpy as np
from scipy.signal import savgol_filter as sgf   # GT position->velocity smoothing only

CAL_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "Test_Data", "Calibration")

T_QTM_TO_FRD = np.diag([1.0, -1.0, -1.0])   # FLU -> FRD, 180deg about shared X
LAB = ['Hx', 'Hy', 'Hz', 'Wx', 'Wy', 'Wz']
RL = ['h0', 'h1', 'h2', 'w0', 'w1', 'w2']
FLOW_KF_Q = float(os.environ.get("FLOW_KF_Q", "5.0"))       # matches img_data.py default
FLOW_KF_R = float(os.environ.get("FLOW_KF_R", "0.1"))
FEAT_KF_Q = float(os.environ.get("IMG_FEAT_KF_Q", "5.0"))
FEAT_KF_R = float(os.environ.get("IMG_FEAT_KF_R", "0.004"))
# Confirmed 2026-07-11 (check_loop_staleness.py): every recording has 7-10
# marker-loss gaps of 1-18s (KLT fallback is capped at MARKER_KLT_MAX_STEPS
# and can't cover these). The flow sample immediately after such a gap is
# unreliable for TWO independent reasons: (1) it may reflect a large
# spurious pixel jump from marker reacquisition, not real target motion, and
# (2) kf_filter_causal's causal KF state was built on data from before the
# gap - see GAP_RESET_S below. Excluded from the FIT, not from the array
# (the KF still needs to run over every sample in order).
GAP_EXCLUDE_S = float(os.environ.get("CAL_GAP_EXCLUDE_S", "1.0"))
# kf_filter_causal used to silently CAP dt at 0.1s regardless of the true
# gap (max(min(ti-prev_t, 0.1), 1e-3)) - so after an 18s marker-loss gap it
# still fed the filter dt=0.1, barely inflating uncertainty and blending a
# stale pre-gap velocity estimate into the post-gap output. Gaps at or above
# this threshold now RESET the filter (re-initialize, like the very first
# sample) instead of predicting across them.
GAP_RESET_S = float(os.environ.get("CAL_GAP_RESET_S", "1.0"))


def kf_filter_causal(raw, t, q, r):
    """Causal constant-velocity 2-state KF per channel, run over a full
    (N, C) raw array + matching timestamps t (N,). Ported verbatim from
    PX4_Gazebo/tools/aggregate_calibration_phased.py::kf_filter_causal,
    which itself mirrors img_data.py::_kf_step exactly - if that function's
    model ever changes, mirror the change here too. Returns (N, C) filtered."""
    raw = np.asarray(raw, dtype=float)
    t = np.asarray(t, dtype=float)
    n, c = raw.shape
    out = np.zeros_like(raw)
    x = np.zeros((c, 2)); P = np.tile(np.eye(2), (c, 1, 1))
    initialized = False
    prev_t = 0.0
    for i in range(n):
        z = raw[i]; ti = t[i]
        # A gap this long means whatever the filter believed before is
        # stale (marker was lost, not just a slow frame) - re-initialize
        # rather than predicting across it with a capped/fake dt.
        if initialized and (ti - prev_t) >= GAP_RESET_S:
            initialized = False
        if not initialized:
            x[:, 0] = z; x[:, 1] = 0.0
            P = np.tile(np.eye(2) * 1.0, (c, 1, 1))
            prev_t = ti; initialized = True
            out[i] = z
            continue
        dt = max(min(ti - prev_t, 0.1), 1e-3)
        prev_t = ti
        F = np.array([[1.0, dt], [0.0, 1.0]])
        Q = q * np.array([[dt**4 / 4.0, dt**3 / 2.0],
                           [dt**3 / 2.0, dt**2]])
        x_pred = x @ F.T
        P_pred = F @ P @ F.T + Q
        y = z - x_pred[:, 0]
        S = P_pred[:, 0, 0] + r
        K = P_pred[:, :, 0] / S[:, None]
        x = x_pred + K * y[:, None]
        P = P_pred - K[:, :, None] * P_pred[:, 0:1, :]
        out[i] = x[:, 0]
    return out


def reject_outliers(raw, window=7, k=5.0, label=""):
    """Hampel-filter sanity guard for the offline derivation, separate from
    the runtime checkposts (which are intentionally disabled during
    calibration recording, per the recording-must-be-truly-raw fix - so
    nothing filters a genuine numerical spike either, e.g. an
    ill-conditioned lstsq solve right at a marker reacquisition).

    Per-channel: for each sample, compute the median and MAD (median
    absolute deviation) over a `window`-sample local neighborhood centered
    on it, and flag the sample an outlier if it deviates from that LOCAL
    median by more than k * 1.4826 * MAD (1.4826 scales MAD to be
    consistent with a Gaussian std, the standard Hampel-filter constant).
    This is more robust than a fixed absolute threshold: it adapts to each
    channel's own local noise level and signal trend, rather than assuming
    one global cutoff is meaningful across every channel and every
    recording's excitation level - and median/MAD are themselves outlier-
    resistant (unlike mean/std, which the outlier being detected would
    itself skew). Flagged samples are replaced by linear interpolation from
    the nearest valid (non-outlier) neighbors, so a single bad frame can't
    bleed into the causal KF's subsequent samples via a garbage
    measurement. Confirmed need 2026-07-11: one row (h_z=-21.7, everything
    else in a sane ~[-2,2] range) found via a data-integrity check on a
    real recording."""
    raw = np.array(raw, dtype=float, copy=True)
    n, c = raw.shape
    idx = np.arange(n)
    half = window // 2
    total_rejected = 0
    for ch in range(c):
        x = raw[:, ch]
        med = np.empty(n)
        mad = np.empty(n)
        for i in range(n):
            lo, hi = max(0, i - half), min(n, i + half + 1)
            w = x[lo:hi]
            med[i] = np.median(w)
            mad[i] = np.median(np.abs(w - med[i]))
        scale = 1.4826 * mad
        # A locally-flat/quiet window gives scale~0, which would flag any
        # tiny wobble as an outlier - fall back to this channel's overall
        # typical scale rather than an arbitrarily tight zero-tolerance band.
        _valid_scale = scale[scale > 1e-6]
        _fallback = float(np.median(_valid_scale)) if len(_valid_scale) else 1e-3
        scale = np.where(scale < 1e-6, _fallback, scale)
        bad = np.abs(x - med) > k * scale
        nbad = int(bad.sum())
        if nbad == 0:
            continue
        total_rejected += nbad
        good = ~bad
        if good.sum() == 0:
            continue   # entire channel is garbage - nothing to interpolate from
        raw[bad, ch] = np.interp(idx[bad], idx[good], raw[good, ch])
    if total_rejected:
        print(f"  outlier guard{(' ('+label+')') if label else ''}: "
              f"{total_rejected} sample(s) rejected (Hampel filter, window={window}, k={k})")
    return raw


def _euler_to_dcm_flu(roll_deg, pitch_deg, yaw_deg):
    """Standard aerospace ZYX Euler -> body->world DCM, in QTM's own FLU
    convention (roll about X-forward, pitch about Y-left, yaw about Z-up)."""
    r, p, y = np.deg2rad([roll_deg, pitch_deg, yaw_deg])
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx   # body -> world (FLU)


def _pose_to_frd(pose):
    """mocaptools.Pose (QTM FLU, degrees) -> (pos_FRD(3,), R_FRD body->world(3,3))."""
    pos_qtm = np.array([pose.x, pose.y, pose.z])
    pos_frd = T_QTM_TO_FRD @ pos_qtm
    R_flu = _euler_to_dcm_flu(pose.roll, pose.pitch, pose.yaw)
    R_frd = T_QTM_TO_FRD @ R_flu @ T_QTM_TO_FRD
    return pos_frd, R_frd


def _v_frame(R):
    """Gravity-leveled V-frame rotation (body-FRD -> V). R = body->world DCM,
    +Z = down (gravity direction) - identical convention to Gazebo's
    gt_optical_flow._v_frame, now that the QTM data is FRD-consistent."""
    g = R.T @ np.array([0., 0., 1.])
    z = g / np.linalg.norm(g)
    x = np.cross([0., 1., 0.], z); x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return np.column_stack([x, y, z]).T


def _robust_vel(x, t):
    """d/dt via uniform-dt interp -> savgol -> gradient -> interp back
    (ports gt_optical_flow._robust_vel - don't differentiate raw jittery
    mocap position directly, it amplifies noise into velocity)."""
    n = len(t)
    if n < 6:
        return np.zeros_like(x)
    tu = np.linspace(t[0], t[-1], n)
    xu = np.column_stack([np.interp(tu, t, x[:, k]) for k in range(x.shape[1])])
    W = max(5, int(round(0.225 / max(np.median(np.diff(tu)), 1e-6))) | 1)
    if W >= n:
        W = n - 1 if (n - 1) % 2 == 0 else n - 2
    if W >= 5:
        xu = sgf(xu, W, 2, axis=0)
    vu = np.gradient(xu, tu, axis=0)
    return np.column_stack([np.interp(t, tu, vu[:, k]) for k in range(x.shape[1])])


def compute_gt_flow(run_dir):
    """Pi-adapted port of gt_optical_flow.compute_gt_flow. Returns a dict
    with GT reference signals on the GT's own time axis (0-based, seconds
    since Start Time), plus align(t_abs, y) to resample a measured signal."""
    gt = np.load(os.path.join(run_dir, "Ground_Truth.npy"), allow_pickle=True).item()
    St = float(gt["Start Time"])
    tg = np.asarray(gt["Time"], float)
    u_all, tp_all = gt["UAV Pose"], gt["Target Pose"]
    n = min(len(tg), len(u_all), len(tp_all))
    tg, u_all, tp_all = tg[:n], u_all[:n], tp_all[:n]

    # Drop samples with a missing (None) pose OR NaN position/orientation
    # (QTM tracking dropouts - see the yaw-span bug fixed 2026-07-10 for why
    # these MUST be filtered before any differencing/unwrap-style math).
    def _valid(p, t):
        if p is None or t is None:
            return False
        vals = [p.x, p.y, p.z, p.roll, p.pitch, p.yaw, t.x, t.y, t.z]
        return all(np.isfinite(v) for v in vals)
    keep = np.array([_valid(u_all[i], tp_all[i]) for i in range(n)])
    tg = tg[keep]; u = [u_all[i] for i in range(n) if keep[i]]
    tp = [tp_all[i] for i in range(n) if keep[i]]
    n = len(tg)
    if n < 20:
        raise ValueError(f"too few valid (non-NaN) samples after filtering: {n}")

    # de-dup non-increasing timestamps (jitter)
    order = np.hstack(([True], np.diff(tg) > 1e-6))
    tg = tg[order]; u = [u[i] for i in range(len(u)) if order[i]]
    tp = [tp[i] for i in range(len(tp)) if order[i]]
    n = len(tg)

    W_x_tu = np.zeros((n, 3)); Ru = np.zeros((n, 3, 3)); yaw = np.zeros(n)
    for i in range(n):
        p, t = u[i], tp[i]
        up, Ru[i] = _pose_to_frd(p)
        tpp, _ = _pose_to_frd(t)
        W_x_tu[i] = tpp - up
        yaw[i] = -np.deg2rad(p.yaw)   # QTM -> FRD yaw sign flip

    W_v_tu = _robust_vel(W_x_tu, tg)
    B_h_g = np.full((n, 3), np.nan); V_h_g = np.full((n, 3), np.nan)
    V_s_g = np.full((n, 2), np.nan)
    for i in range(n):
        zB = W_x_tu[i, 2]
        B_x = Ru[i].T @ W_x_tu[i]
        V_x = _v_frame(Ru[i]) @ B_x
        V_s_g[i] = [V_x[0] / (V_x[2] + 0.2), V_x[1] / (V_x[2] + 0.2)]
        B_v = Ru[i].T @ W_v_tu[i]
        V_v = _v_frame(Ru[i]) @ B_v
        if abs(zB) >= 0.1:
            B_h_g[i] = B_v / (zB + 0.2)
            V_h_g[i] = V_v / (zB + 0.2)

    out = dict(t_g=tg, start_time=St, alt=W_x_tu[:, 2],
               B_h_g=B_h_g, V_h_g=V_h_g, loom=V_h_g[:, 2], alpha=yaw, V_s_g=V_s_g)

    def align(t_abs, y):
        ti = np.asarray(t_abs, float) - St
        y = np.asarray(y)
        if y.ndim == 1:
            return np.interp(tg, ti, y)
        return np.column_stack([np.interp(tg, ti, y[:, k]) for k in range(y.shape[1])])
    out["align"] = align

    def gap_mask(t_abs, gap_max):
        """True at each tg[j] that is safely bracketed by two real samples
        of t_abs no more than gap_max apart (and within t_abs's own span).
        align() LINEARLY INTERPOLATES across whatever gap exists between
        real samples - for a normal ~30Hz flow this is a harmless few-ms
        fill, but for a multi-second marker-loss gap (confirmed common on
        this dataset, see check_loop_staleness.py) it silently fabricates
        potentially hundreds of GT-grid points along a straight line that
        has no reason to track GT's actual (non-linear) motion during the
        gap - injecting mismatched, deceptively dense-looking noise into
        the fit rather than merely losing coverage. Use this to mask those
        fabricated points out wherever align() is used for fitting."""
        ti = np.asarray(t_abs, float) - St
        if len(ti) < 2:
            return np.zeros(len(tg), dtype=bool)
        idx = np.clip(np.searchsorted(ti, tg), 1, len(ti) - 1)
        bracket_w = ti[idx] - ti[idx - 1]
        inside = (tg >= ti[0]) & (tg <= ti[-1])
        return inside & (bracket_w < gap_max)
    out["gap_mask"] = gap_mask
    return out


def derive_one(run_dir):
    # Prefer the mount-rotation-corrected recompute (recompute_raw_flow.py)
    # when present - it reflects img_data.py's CURRENT _getVirtualPts math
    # (R_CAM_TO_BODY fix, 2026-07-11), re-derived offline from this same
    # run's own logged pixel corners/quaternion/gyro. The original
    # Img_Data.npy's "Opt Flow Ang Vel" was computed by the OLD, unfixed
    # code at capture time and can't reflect a fix that changed the
    # ray-projection math itself.
    recomputed_path = os.path.join(run_dir, "Img_Data_Recomputed.npy")
    if os.path.exists(recomputed_path):
        img = np.load(recomputed_path, allow_pickle=True).item()
        print("  using Img_Data_Recomputed.npy (mount-rotation-corrected)")
    else:
        img = np.load(os.path.join(run_dir, "Img_Data.npy"), allow_pickle=True).item()
    g = compute_gt_flow(run_dir)

    t_img_abs = np.asarray(img["Time"], float)
    raw_flow = np.asarray(img["Opt Flow Ang Vel"], float)     # pre-cal, [h;w]
    raw_feat = np.asarray(img["Feature Params"], float)       # [xc, yc, 1, alpha]
    tag = np.asarray(img.get("Opt Flow Estimator Tag", [''] * len(raw_flow)))
    n = min(len(t_img_abs), len(raw_flow), len(raw_feat), len(tag))
    t_img_abs, raw_flow, raw_feat, tag = t_img_abs[:n], raw_flow[:n], raw_feat[:n], tag[:n]

    if np.any(tag != ''):
        _u, _c = np.unique(tag, return_counts=True)
        print("  h estimator coverage: " + ", ".join(f"{u}={c}" for u, c in zip(_u, _c)))

    # Per-frame raw flow is noise-dominated (raw std ~1.4 vs true signal
    # ~0.05, per project convention) - MUST filter on its own native time
    # axis BEFORE resampling/fitting. Uses the SAME KF the runtime
    # getOptFlowAngVel() actually applies (IMG_FILTER='kf' default) - NOT
    # Savgol, which was the original (2026-07-10) approach and matched a
    # filter-mismatch bug PX4_Gazebo independently found and fixed
    # 2026-07-11 (feedback_kf_savgol_cal_mismatch): fitting against one
    # filter shape and applying at runtime through a differently-shaped
    # filter is not guaranteed valid, and moved every diagonal entry on
    # SITL's dataset (loom +127%, w_z +64%).
    if n > 1:
        raw_flow = reject_outliers(raw_flow, label="flow")
        raw_feat = reject_outliers(raw_feat, label="feature")
        raw_flow = kf_filter_causal(raw_flow, t_img_abs, FLOW_KF_Q, FLOW_KF_R)
        raw_feat = kf_filter_causal(raw_feat, t_img_abs, FEAT_KF_Q, FEAT_KF_R)

    raw_flow_g = g["align"](t_img_abs, raw_flow)     # raw flow resampled onto GT clock
    raw_feat_g = g["align"](t_img_abs, raw_feat)

    # Marker-loss gap guard (see GAP_EXCLUDE_S / g["gap_mask"]): align()
    # linearly interpolates raw_flow/raw_feat across whatever real-sample
    # gaps exist in t_img_abs - for a multi-second marker-loss gap that
    # fabricates many mismatched GT-grid points (see gap_mask docstring).
    # Excluded here, on the ALIGNED (GT-grid) index space, not on
    # t_img_abs's own sparse index - a mask built on the flow's native
    # samples would not line up with raw_flow_g/raw_feat_g's indexing.
    gok = g["gap_mask"](t_img_abs, GAP_EXCLUDE_S)
    n_excluded = int((~gok).sum())
    if n_excluded:
        print(f"  gap guard: {n_excluded}/{len(gok)} GT-grid sample(s) excluded from fit "
              f"(interpolated across a >= {GAP_EXCLUDE_S:.1f}s marker-loss gap)")

    # GT rhs: manuscript w = -alpha_dot-derived yaw rate is folded into
    # V_h_g already having only translational h; the GT yaw-rate (w_z) is
    # d(alpha)/dt, matching img_data's B_v[5].
    #
    # FIXED 2026-07-11: this used to be a bare np.gradient(alpha, t_g) -
    # raw differentiation of jittery QTM yaw with NO smoothing, despite the
    # comment above claiming "the same _robust_vel style" (never actually
    # implemented - _robust_vel is used for V_h_g/W_v_tu above but not here).
    # Two compounding problems: (1) _robust_vel's own docstring warns against
    # differentiating raw jittery mocap data directly, exactly what this did;
    # (2) alpha is arctan2-wrapped to (-pi, pi] and was never unwrapped, so
    # ANY yaw crossing that boundary during the recording produced a huge
    # spurious np.gradient spike right at the wrap (same bug class already
    # found in the yaw-span calc, see the NaN-before-unwrap fix elsewhere).
    # Both would inject severe, GT-uncorrelated noise specifically into the
    # Wz fit target - consistent with Wz being the worst-behaved axis in
    # every derived calibration so far, independent of the other fixes.
    alpha_unwrapped = np.unwrap(g["alpha"])
    yaw_rate = _robust_vel(alpha_unwrapped[:, None], g["t_g"])[:, 0]
    G = np.column_stack([g["V_h_g"], np.zeros((len(yaw_rate), 2)), yaw_rate])  # [h(3); w=(0,0,wz)] = 6 cols

    m = np.all(np.isfinite(G), 1) & np.all(np.isfinite(raw_flow_g), 1) & gok
    Gm, Rm = G[m], raw_flow_g[m]
    if len(Rm) < 200:
        raise ValueError(f"too few aligned finite samples: {len(Rm)}")

    # Diagnostic: raw per-axis Pearson correlation BEFORE the full 6x6 fit -
    # a sanity check independent of fit quality. If diagonal-ish entries
    # (Hx-h0, Hy-h1, Hz-h2, Wz-w2) aren't the strongest in their row/col,
    # something upstream (frame transform, alignment, units) is likely wrong
    # rather than the fit itself being the problem.
    corr = np.corrcoef(np.hstack([Gm, Rm]).T)[:6, 6:]
    print("  raw corrcoef(GT, raw) [rows=GT Hx/Hy/Hz/Wx/Wy/Wz, cols=raw h0/h1/h2/w0/w1/w2]:")
    print(" ", np.array2string(corr, precision=3, suppress_small=True).replace("\n", "\n   "))

    Msol, _, _, _ = np.linalg.lstsq(Rm, Gm, rcond=None)   # G = R @ Msol
    cal = Msol.T                                          # GT = cal @ raw
    pred = Rm @ Msol
    r2 = 1 - np.sum((Gm - pred) ** 2, 0) / np.sum((Gm - Gm.mean(0)) ** 2, 0)

    # centroid scale: GT bearing vs raw feature xc/yc, plain regression
    # slope (no phase tags on this continuous hand-move data, unlike
    # Gazebo's phased std_ratio - a simple lstsq slope per axis instead).
    gs = g["V_s_g"]
    ms = np.all(np.isfinite(gs), 1) & np.all(np.isfinite(raw_feat_g[:, :2]), 1) & gok
    sx = float(np.polyfit(raw_feat_g[ms, 0], gs[ms, 0], 1)[0]) if ms.sum() > 10 else np.nan
    sy = float(np.polyfit(raw_feat_g[ms, 1], gs[ms, 1], 1)[0]) if ms.sum() > 10 else np.nan

    return cal, r2, (sx, sy), len(Rm)


def derive_ring_one(run_dir):
    """Ring-flow analogue of derive_one() - fits _sensor_cal_ring against the
    SAME GT this run's corner cal uses, but on the ring's OWN per-frame log
    ("Ring Opt Flow Ang Vel" + "Ring Time"), not "Time"/"Opt Flow Ang Vel".
    Ported alongside the img_data.py change (2026-07-22) that made ring
    logging unconditional (independent of corner-marker detection success,
    mirroring PX4_Gazebo) - the ring array is no longer index-aligned with
    the corner flow/feature arrays, so it needs its own alignment pass here
    rather than reusing derive_one()'s t_img_abs. Requires a recording made
    AFTER that img_data.py change (the 7 runs recorded 2026-07-10/11 predate
    it and have no "Ring Time" key - raises ValueError, same as any other
    run this tool considers unusable)."""
    img = np.load(os.path.join(run_dir, "Img_Data.npy"), allow_pickle=True).item()
    if "Ring Time" not in img:
        raise ValueError("no 'Ring Time' - this run predates unconditional ring logging")
    g = compute_gt_flow(run_dir)

    t_ring_abs = np.asarray(img["Ring Time"], float)
    raw_ring = np.asarray(img["Ring Opt Flow Ang Vel"], float)   # pre-cal, [h;w]
    n = min(len(t_ring_abs), len(raw_ring))
    t_ring_abs, raw_ring = t_ring_abs[:n], raw_ring[:n]

    raw_ring = reject_outliers(raw_ring, label="ring flow")
    raw_ring = kf_filter_causal(raw_ring, t_ring_abs, FLOW_KF_Q, FLOW_KF_R)
    raw_ring_g = g["align"](t_ring_abs, raw_ring)

    # Ring's own gap guard: the ring is designed to survive a CORNER dropout,
    # but it has its own (much rarer) failure mode - texture too sparse / all
    # stations off-frame - which also shows up as a real time gap in its
    # native "Ring Time" log. Same GAP_EXCLUDE_S convention as the corner fit.
    gok = g["gap_mask"](t_ring_abs, GAP_EXCLUDE_S)
    n_excluded = int((~gok).sum())
    if n_excluded:
        print(f"  ring gap guard: {n_excluded}/{len(gok)} GT-grid sample(s) excluded from fit")

    alpha_unwrapped = np.unwrap(g["alpha"])
    yaw_rate = _robust_vel(alpha_unwrapped[:, None], g["t_g"])[:, 0]
    G = np.column_stack([g["V_h_g"], np.zeros((len(yaw_rate), 2)), yaw_rate])

    m = np.all(np.isfinite(G), 1) & np.all(np.isfinite(raw_ring_g), 1) & gok
    Gm, Rm = G[m], raw_ring_g[m]
    if len(Rm) < 200:
        raise ValueError(f"too few aligned finite ring samples: {len(Rm)}")

    corr = np.corrcoef(np.hstack([Gm, Rm]).T)[:6, 6:]
    print("  ring raw corrcoef(GT, raw) [rows=GT Hx/Hy/Hz/Wx/Wy/Wz, cols=raw h0/h1/h2/w0/w1/w2]:")
    print(" ", np.array2string(corr, precision=3, suppress_small=True).replace("\n", "\n   "))

    Msol, _, _, _ = np.linalg.lstsq(Rm, Gm, rcond=None)
    cal = Msol.T
    pred = Rm @ Msol
    r2 = 1 - np.sum((Gm - pred) ** 2, 0) / np.sum((Gm - Gm.mean(0)) ** 2, 0)
    return cal, r2, len(Rm)


def main():
    if len(sys.argv) > 1:
        runs = sys.argv[1:]
    else:
        runs = sorted(d for d in glob.glob(os.path.join(CAL_DIR, "*")) if os.path.isdir(d))

    np.set_printoptions(precision=4, suppress=True, linewidth=130)
    cals, r2s, calS = [], [], []
    for run_dir in runs:
        name = os.path.basename(run_dir)
        try:
            cal, r2, (sx, sy), nfit = derive_one(run_dir)
        except Exception as e:
            print(f"  skip {name}: {e}")
            continue
        print(f"\n=== {name} (n={nfit} fit samples) ===")
        print("per-axis R^2: " + "  ".join(f"{LAB[k]}={r2[k]:.2f}" for k in range(6)))
        print(cal)
        print(f"centroid slope: sx={sx:.4f}  sy={sy:.4f}")
        cals.append(cal); r2s.append(r2); calS.append([sx, sy])

    if not cals:
        print("\nNo usable runs.")
        return

    cals = np.array(cals); r2s = np.array(r2s); calS = np.array(calS)
    M = cals.mean(0); Mstd = cals.std(0) if len(cals) > 1 else np.zeros_like(M)
    sx = float(np.nanmedian(calS[:, 0])); sy = float(np.nanmedian(calS[:, 1]))

    print(f"\n\n=== AGGREGATE across {len(cals)} run(s) — PROVISIONAL, review before use ===")
    print("per-axis R^2 (mean):  " + "  ".join(f"{LAB[k]}={r2s[:,k].mean():.2f}" for k in range(6)))
    if len(cals) > 1:
        print("\ninter-run STD of M (small = robust):")
        print("       " + "  ".join(f"{r:>6}" for r in RL))
        for i in range(6):
            print(f"  {LAB[i]:>2} " + "  ".join(f"{Mstd[i,j]:6.3f}" for j in range(6)))
    print(f"\ncentroid cal_s:  sx={sx:.4f}  sy={sy:.4f}")

    print("\n--- candidate paste for img_data.py (DO NOT apply without reviewing R^2 above) ---")
    rows = ",\n            ".join(
        "[" + ", ".join(f"{M[i,j]:+.4f}" for j in range(6)) + "]" for i in range(6))
    print(f"        self._sensor_cal_hw = np.array([\n            {rows}])")
    print(f"        self._sensor_cal_s  = np.diag([{sx:.4f}, {sy:.4f}, 1.0, 1.0])")

    # Ring cal, from the SAME runs' "Ring Opt Flow Ang Vel"/"Ring Time" logs
    # (see derive_ring_one docstring - requires a recording made after the
    # 2026-07-22 img_data.py change; the 7 runs recorded 2026-07-10/11 will
    # all skip here with "no 'Ring Time'").
    print("\n\n=== RING CAL (independent of the corner fit above) ===")
    ring_cals, ring_r2s = [], []
    for run_dir in runs:
        name = os.path.basename(run_dir)
        try:
            cal, r2, nfit = derive_ring_one(run_dir)
        except Exception as e:
            print(f"  skip {name}: {e}")
            continue
        print(f"\n=== {name} (n={nfit} ring fit samples) ===")
        print("per-axis R^2: " + "  ".join(f"{LAB[k]}={r2[k]:.2f}" for k in range(6)))
        print(cal)
        ring_cals.append(cal); ring_r2s.append(r2)

    if not ring_cals:
        print("\nNo usable ring runs.")
        return

    ring_cals = np.array(ring_cals); ring_r2s = np.array(ring_r2s)
    Mr = ring_cals.mean(0)
    Mrstd = ring_cals.std(0) if len(ring_cals) > 1 else np.zeros_like(Mr)
    print(f"\n\n=== RING AGGREGATE across {len(ring_cals)} run(s) — PROVISIONAL, review before use ===")
    print("per-axis R^2 (mean):  " + "  ".join(f"{LAB[k]}={ring_r2s[:,k].mean():.2f}" for k in range(6)))
    if len(ring_cals) > 1:
        print("\ninter-run STD of ring M (small = robust):")
        print("       " + "  ".join(f"{r:>6}" for r in RL))
        for i in range(6):
            print(f"  {LAB[i]:>2} " + "  ".join(f"{Mrstd[i,j]:6.3f}" for j in range(6)))
    rows_r = ",\n            ".join(
        "[" + ", ".join(f"{Mr[i,j]:+.4f}" for j in range(6)) + "]" for i in range(6))
    print("\n--- candidate paste for img_data.py (DO NOT apply without reviewing R^2 above) ---")
    print(f"        self._sensor_cal_ring = np.array([\n            {rows_r}])")


if __name__ == "__main__":
    main()
