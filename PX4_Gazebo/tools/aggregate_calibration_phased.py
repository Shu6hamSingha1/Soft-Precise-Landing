"""Phase-aware aggregator for the redesigned record_output_calibration.py.

The new excitation drives each axis ALONE in sequence (x → y → z → yaw)
with a 'Phase' tag per sample stored in Ground_Truth.npy. For each cal
axis, we derive the scale factor from ONLY the samples tagged with that
axis's phase — giving clean, decorrelated lstsq inputs.

Cal axes vs phases:
  sensor_cal_hw[0,0] (flow_x)  ← phase 'x' samples
  sensor_cal_hw[1,1] (flow_y)  ← phase 'y'
  sensor_cal_hw[2,2] (flow_z)  ← phase 'z'   ← KEY: previously NaN
  sensor_cal_hw[3,3] (ω_x)     ← phase 'y'   (y-motion induces roll dynamics)
  sensor_cal_hw[4,4] (ω_y)     ← phase 'x'   (x-motion induces pitch dynamics)
  sensor_cal_hw[5,5] (ω_z)     ← phase 'yaw'

  sensor_cal_s[0,0] (centroid_x) ← phase 'x'
  sensor_cal_s[1,1] (centroid_y) ← phase 'y'

For each rep: extract raw[phase_mask] + ground-truth[phase_mask],
solve the std-ratio scale (σ_GT / σ_raw_filtered). Median across reps.
"""
import os, glob, sys
import numpy as np
from scipy.signal import savgol_filter as sgf
from ahrs import Quaternion, DCM

FLU_2_FRD = np.array(DCM(x=180.0))
# Filter params MUST match src/img_data.py runtime (FILTER_WIN=13, POLY=1).
# Previously used offline-best (101, 3) which gave σ(raw_filtered) much smaller
# than the runtime sees → derived cal too small → controller sees weak error
# signal at runtime → TARGET_LOST drift (observed 2026-06-02 IC1 landing).
FILTER_WIN = 13
POLYORDER  = 1

# Cal-axis ↔ which excitation phase to derive it from
AXIS_PHASE_HW = {0: 'x', 1: 'y', 2: 'z', 3: 'y', 4: 'x', 5: 'yaw'}
AXIS_PHASE_S  = {0: 'x', 1: 'y'}

# KF params MUST match src/img_data.py runtime (FLOW_KF_Q/R, IMG_FEAT_KF_Q/R).
# Runtime default filter switched Savgol(13,1) -> KF on 2026-06-06 (commit 6e0b44f,
# "Switched to KF by default": Savgol's ~110ms group delay was hurting the outer
# loop). These derive tools kept filtering with Savgol for the cal fit while every
# runtime getter (getOptFlowAngVel/getRingFlowAngVel/getImgFeatureParam) applies the
# baked cal matrix to the KF-filtered state instead — the cal was derived against a
# signal shape the runtime never actually consumes. kf_filter_causal() replicates
# img_data.py::_kf_step exactly so the derived cal matches what's really applied to.
FLOW_KF_Q = float(os.environ.get("FLOW_KF_Q", "5.0"))
FLOW_KF_R = float(os.environ.get("FLOW_KF_R", "0.1"))
FEAT_KF_Q = float(os.environ.get("IMG_FEAT_KF_Q", "5.0"))
FEAT_KF_R = float(os.environ.get("IMG_FEAT_KF_R", "0.004"))


def kf_filter_causal(raw, t, q, r):
    """Causal constant-velocity 2-state KF per channel, run over a full (N, C) raw
    array + matching timestamps t (N,). Same process/measurement model as
    src/img_data.py::_kf_step (kept in lockstep with it — if that function changes,
    mirror the change here). Returns the (N, C) filtered value trace."""
    raw = np.asarray(raw, dtype=float)
    t = np.asarray(t, dtype=float)
    n, c = raw.shape
    out = np.zeros_like(raw)
    x = np.zeros((c, 2)); P = np.tile(np.eye(2), (c, 1, 1))
    initialized = False
    prev_t = 0.0
    for i in range(n):
        z = raw[i]; ti = t[i]
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


def _body_omega_from_quats(quats, t):
    """Body-frame angular velocity from quaternions via central difference
    (δq = conj(q[i-1])·q[i+1]; ω_body ≈ 2·δq.xyz / dt). Preserves SO(3),
    unlike np.gradient on rotation-matrix elements."""
    N = len(quats)
    w = np.zeros((N, 3))
    for i in range(N):
        i0 = max(0, i - 1)
        i1 = min(N - 1, i + 1)
        if i1 == i0:
            continue
        q0, q1 = quats[i0], quats[i1]
        dt_pair = t[i1] - t[i0]
        if dt_pair < 1e-9:
            continue
        w0, x0, y0, z0 = q0
        w1, x1, y1, z1 = q1
        dq_x = w0*x1 - x0*w1 - y0*z1 + z0*y1
        dq_y = w0*y1 + x0*z1 - y0*w1 - z0*x1
        dq_z = w0*z1 - x0*y1 + y0*x1 - z0*w1
        w[i] = 2.0 * np.array([dq_x, dq_y, dq_z]) / dt_pair
    return w


def compute_gt_signals(gt):
    """V-frame GT signals matching the post-2026-06-01 img_data.py runtime
    (after the _getVirtualPts g-sign fix).

    Runtime img_data.py:_getVirtualPts produces image-side quantities in
    the V-frame: gravity-leveled (V_z = world-down), drone-yaw-aligned
    (V_x = drone heading in horizontal plane). So the GT comparison must
    also be in V-frame, else tilt-induced apparent motion appears on
    one side but not the other.

    Pre-2026-06-01 this function used body-FRD GT (B_x_tu /B_x_tu[2]).
    For level drone V-frame ≈ body-FRD, so the bug stayed hidden. With
    aggressive maneuvers (5–10° tilt during phased excitation) the
    difference is large and drives huge inter-run variance in derived
    sensor_cal values.
    """
    NED_from_ENU = np.array([[0.0, 1.0,  0.0],
                              [1.0, 0.0,  0.0],
                              [0.0, 0.0, -1.0]])

    t_g = np.array(gt['Time'])
    UAV = np.array(gt['UAV Pose'], dtype=object)
    TGT = np.array(gt['Target Pose'], dtype=object)
    n_min = min(len(t_g), len(UAV), len(TGT))
    t_g = t_g[:n_min]; UAV = UAV[:n_min]; TGT = TGT[:n_min]
    dt = np.diff(t_g)
    valid = np.hstack(([True], dt > 1e-6))
    t_g = t_g[valid]; UAV = UAV[valid]; TGT = TGT[valid]
    n = len(t_g)

    # Build R_FRD_NED (body-FRD → world-NED) and NED positions/velocities.
    R_FRD_NED = np.zeros((n, 3, 3))
    W_x_tu_NED = np.zeros((n, 3))
    for i in range(n):
        R_FLU_ENU = Quaternion([UAV[i].orientation.w, UAV[i].orientation.x,
                                 UAV[i].orientation.y, UAV[i].orientation.z]).to_DCM()
        R_FRD_NED[i] = NED_from_ENU @ R_FLU_ENU @ FLU_2_FRD
        wxu = np.array([UAV[i].position.x, UAV[i].position.y, UAV[i].position.z])
        wxt = np.array([TGT[i].position.x, TGT[i].position.y, TGT[i].position.z])
        W_x_tu_NED[i] = NED_from_ENU @ (wxt - wxu)

    win = min(101, (n // 2) * 2 - 1)
    W_x_tu_smooth = sgf(W_x_tu_NED, win, 2, axis=0) if win >= 5 else W_x_tu_NED
    W_v_tu_NED = np.gradient(W_x_tu_smooth, t_g, axis=0)
    if win >= 5:
        W_v_tu_NED = sgf(W_v_tu_NED, min(51, win), 2, axis=0)

    # Body-FRD UAV angular velocity (quaternion-difference, SO(3)-preserving).
    uav_quats = np.array([[U.orientation.w, U.orientation.x,
                           U.orientation.y, U.orientation.z] for U in UAV])
    B_w_ug = (FLU_2_FRD @ _body_omega_from_quats(uav_quats, t_g).T).T

    # V-frame projection (gravity-leveled, drone-yaw-aligned) for h, w AND s, all
    # onto the SAME per-sample V basis so every channel is in the frame img_data.py
    # reports (no body/virtual mix). [Was: w kept in body-FRD as an O(tilt)
    # approximation; corrected 2026-06-05 so w is virtual like h and s.]
    V_h_g  = np.zeros((n, 3))      # virtual image velocity h
    V_w_ug = np.zeros((n, 3))      # UAV angular velocity in V (manuscript w = -V_w_ug)
    V_xc_g = np.zeros(n)           # virtual image position s_x
    V_yc_g = np.zeros(n)
    V_z_g  = np.zeros(n)           # V-frame depth (altitude) — for validity gates
    for i in range(n):
        yaw = np.arctan2(R_FRD_NED[i, 1, 0], R_FRD_NED[i, 0, 0])
        V_x_NED = np.array([np.cos(yaw), np.sin(yaw),  0.0])
        V_y_NED = np.array([-np.sin(yaw), np.cos(yaw), 0.0])
        V_z_NED = np.array([0.0, 0.0, 1.0])
        V_R_NED = np.array([V_x_NED, V_y_NED, V_z_NED])   # rows = V axes in NED
        # ω: re-express body ω in V coords (body → NED → V). This re-expresses the
        # SAME physical vector; it omits the 2nd-order rotation rate of the leveling
        # itself (vs the leveled-camera-relative rate) — sub-noise at our tilts,
        # folds into the EKF-leveling residual. See memory outputcal-flow-validation.
        V_w_ug[i] = V_R_NED @ (R_FRD_NED[i] @ B_w_ug[i])
        rel = W_x_tu_NED[i]
        Vx = rel @ V_x_NED;  Vy = rel @ V_y_NED;  Vz = rel @ V_z_NED
        V_z_g[i] = Vz
        if not (Vz > 1.0):
            V_xc_g[i] = np.nan; V_yc_g[i] = np.nan
            V_h_g[i] = np.nan
            continue
        V_xc_g[i] = Vx / Vz
        V_yc_g[i] = Vy / Vz
        relv = W_v_tu_NED[i]
        V_h_g[i] = np.array([relv @ V_x_NED, relv @ V_y_NED, relv @ V_z_NED]) / Vz

    return t_g, V_h_g, V_w_ug, V_xc_g, V_yc_g, valid, V_z_g


def std_ratio(gt, raw, mask, k_mad_sample=3.0):
    """Signed scale factor with SAMPLE-LEVEL outlier rejection on matched pairs.

    Method (2026-06-02 rewrite — user's "eliminate outliers, remove corresponding
    GT samples, ratio the magnitudes"):

    1. Restrict to the phase mask, drop non-finite pairs.
    2. Initial robust fit: σ_g = std(GT_cleaned), σ_r = std(raw_cleaned),
       β_init = sign(corr) × σ_g / σ_r.
    3. Compute residuals r_i = raw_i − GT_i / β_init  (model: raw ≈ GT/β).
    4. Drop the matched (GT_i, raw_i) pairs where
       |r_i − median(r)| > k_mad_sample × MAD(r).
       This is regression-residual outlier detection (IRLS-style hard reject)
       — robust because it uses GT as the reference axis, so noisy raw
       samples are caught without dragging down by their own MAD.
    5. Final α = sign(corr_clean) × σ(GT_clean) / σ(raw_clean).
       For zero-mean sinusoidal excitation, σ ≡ RMS ≡ √(2/π)·mean(|·|) ×
       proportionality constant — so this IS the magnitude-ratio per the
       user's proposal, applied to outlier-cleaned matched pairs.

    Returns NaN if too few samples survive or if corr collapses (signal below
    noise floor — no observability)."""
    g = gt[mask]; r = raw[mask]
    finite = np.isfinite(g) & np.isfinite(r)
    if finite.sum() < 30:
        return float('nan')
    gf = g[finite]; rf = r[finite]

    sg0 = float(np.std(gf))
    sr0 = float(np.std(rf))
    if sr0 < 1e-9 or sg0 < 1e-9:
        return float('nan')

    co0 = float(np.corrcoef(gf, rf)[0, 1])
    if not np.isfinite(co0) or abs(co0) < 0.05:
        # No correlation — signal below noise floor. Don't bother cleaning.
        return float('nan')
    sign0 = +1.0 if co0 > 0 else -1.0
    beta_init = sign0 * sg0 / sr0

    # Residuals of raw vs model raw_hat = GT / beta_init
    raw_hat = gf / beta_init
    resid = rf - raw_hat
    med = float(np.median(resid))
    mad = float(np.median(np.abs(resid - med)))
    if mad < 1e-12:
        # Constant residuals — no scatter, accept initial fit
        return beta_init

    keep = np.abs(resid - med) <= k_mad_sample * mad
    if keep.sum() < 30:
        # Too aggressive — relax to keep enough samples
        return beta_init
    gc = gf[keep]; rc = rf[keep]

    sg = float(np.std(gc))
    sr = float(np.std(rc))
    if sr < 1e-9 or sg < 1e-9:
        return float('nan')
    co = float(np.corrcoef(gc, rc)[0, 1])
    if not np.isfinite(co) or abs(co) < 0.05:
        return float('nan')
    sign = +1.0 if co > 0 else -1.0
    return sign * sg / sr


def main():
    # Optional dir arg so the same aggregator can be pointed at alternate
    # recording sets (e.g. post-reboot recalibration vs the pre-reboot set).
    cal_dir = sys.argv[1] if len(sys.argv) > 1 else \
        '/home/shubham/Soft-Precise-Landing/PX4_Gazebo/calibration_data/output'
    runs = sorted([d for d in glob.glob(os.path.join(cal_dir, '*')) if os.path.isdir(d)])
    print(f"[phased] {len(runs)} run directories under {cal_dir}\n")
    per_run_hw = []
    per_run_s  = []
    for d in runs:
        try:
            gt = np.load(os.path.join(d, 'Ground_Truth.npy'), allow_pickle=True).item()
        except Exception as e:
            print(f"  [SKIP] {os.path.basename(d)}: load error {e}"); continue
        if 'Phase' not in gt:
            print(f"  [SKIP] {os.path.basename(d)}: no Phase tag (run with old script)")
            continue

        try:
            t_g, V_h_g, V_w_ug, V_xc_g, V_yc_g, valid_mask, _ = compute_gt_signals(gt)
        except Exception as e:
            print(f"  [SKIP] {os.path.basename(d)}: gt compute error {e}")
            continue

        raw_hw = np.asarray(gt['Opt Flow Ang Vel'])  # (N, 6)
        raw_s  = np.asarray(gt['Img Feature Params'])  # (N, 4)
        # Per-sample ESTIMATOR TAG (which computation produced each raw sample — primary/KLT-fallback
        # lstsq, observer, board homography, single-marker moment, or synthetic 'coast'). Older
        # recordings predate this field; back-compat defaults to '' (unknown -- can't filter/split).
        hw_tag = np.asarray(gt.get('Opt Flow Estimator Tag', [''] * len(raw_hw)))
        s_tag  = np.asarray(gt.get('Img Feature Estimator Tag', [''] * len(raw_s)))
        # truncate to common length, then apply same valid mask
        n_min = min(len(raw_hw), len(raw_s), len(valid_mask), len(hw_tag), len(s_tag))
        raw_hw = raw_hw[:n_min][valid_mask[:n_min]]
        raw_s  = raw_s[:n_min][valid_mask[:n_min]]
        hw_tag = hw_tag[:n_min][valid_mask[:n_min]]
        s_tag  = s_tag[:n_min][valid_mask[:n_min]]
        phase = np.array(gt['Phase'])[:n_min][valid_mask[:n_min]]

        # KF-filter raw inputs (matches runtime IMG_FILTER=kf default — see
        # kf_filter_causal docstring). t_g is post-valid-filter (same mask just
        # applied above), so it's index-aligned with raw_hw/raw_s here.
        n_kf = min(len(raw_hw), len(t_g))
        raw_hw = raw_hw[:n_kf]; raw_s = raw_s[:n_kf]; phase = phase[:n_kf]
        hw_tag = hw_tag[:n_kf]; s_tag = s_tag[:n_kf]
        if n_kf > 1:
            raw_hw_f = kf_filter_causal(raw_hw, t_g[:n_kf], FLOW_KF_Q, FLOW_KF_R)
            raw_s_f  = kf_filter_causal(raw_s,  t_g[:n_kf], FEAT_KF_Q, FEAT_KF_R)
        else:
            raw_hw_f = raw_hw.copy(); raw_s_f = raw_s.copy()

        # Align lengths between phase array and GT signals (gt may be a bit longer if dt dedup removed some samples)
        ngt = min(len(V_h_g), len(phase))
        phase = phase[:ngt]; hw_tag = hw_tag[:ngt]; s_tag = s_tag[:ngt]

        # Coverage report + exclude synthetic 'coast' samples (extrapolation, not real data) from
        # the fit -- see feedback_estimator_blind_calibration.
        if np.any(hw_tag != ''):
            _u, _c = np.unique(hw_tag, return_counts=True)
            print(f"        h estimator coverage: " + ", ".join(f"{u}={c}" for u, c in zip(_u, _c)))
        if np.any(s_tag != ''):
            _u, _c = np.unique(s_tag, return_counts=True)
            print(f"        s estimator coverage: " + ", ".join(f"{u}={c}" for u, c in zip(_u, _c)))
        hw_real = (hw_tag != 'coast')
        s_real  = (s_tag != 'coast')

        # Per-axis derivation using only that axis's phase samples
        hw_diag = np.full(6, np.nan)
        for k, want_phase in AXIS_PHASE_HW.items():
            mask = (phase == want_phase) & hw_real
            if mask.sum() < 30: continue
            if k < 3:   # virtual image VELOCITY h (= target rel camera vel / depth)
                gt_sig = V_h_g[:ngt, k]
            else:       # virtual image ANGULAR velocity w = -ω_camera (target rel
                        # to camera, manuscript convention). For stationary target:
                        #   w = ω_target - ω_camera = -ω_UAV_body  (since target ω = 0)
                        # Plotter compares V_w_cal to V_w_tug = -V_w_ug; the aggregator
                        # must match this convention or the derived cal sign is wrong.
                gt_sig = -V_w_ug[:ngt, k - 3]
            hw_diag[k] = std_ratio(gt_sig, raw_hw_f[:ngt, k], mask)

        s_diag = np.full(4, np.nan)
        for k, want_phase in AXIS_PHASE_S.items():
            mask = (phase == want_phase) & s_real
            if mask.sum() < 30: continue
            gt_sig = V_xc_g[:ngt] if k == 0 else V_yc_g[:ngt]
            s_diag[k] = std_ratio(gt_sig, raw_s_f[:ngt, k], mask)
        # axes 2,3 of s (h, alpha) — keep at 1.0 (current convention)
        s_diag[2] = 1.0
        s_diag[3] = 1.0

        print(f"  [OK] {os.path.basename(d)}")
        print(f"        hw = [{', '.join(f'{v:.4f}' for v in hw_diag)}]")
        print(f"        s  = [{', '.join(f'{v:.4f}' for v in s_diag)}]")
        per_run_hw.append(hw_diag); per_run_s.append(s_diag)

    if not per_run_hw:
        print("[phased] no phase-tagged runs found.\n")
        print("  Run record_output_calibration.py (the new phased version) to generate")
        print("  recordings with the 'Phase' tag.")
        return

    hw_arr = np.array(per_run_hw); s_arr = np.array(per_run_s)
    print(f"\n[phased] aggregating across {len(per_run_hw)} valid run(s)")

    def trimmed_mean(vals, k_mad=2.5):
        """MAD-based outlier rejection then mean. Robust to a few bad runs
        while still using more information than the median alone."""
        finite = vals[np.isfinite(vals)]
        if len(finite) == 0: return float('nan'), 0, 0
        med = float(np.median(finite))
        mad = float(np.median(np.abs(finite - med)))
        if mad < 1e-9:
            return float(finite.mean()), len(finite), 0
        keep = np.abs(finite - med) <= k_mad * mad
        n_drop = int((~keep).sum())
        kept = finite[keep]
        return float(kept.mean()), int(len(kept)), n_drop

    print(f"\n  _sensor_cal_hw axis stats:")
    print(f"    axis |  mean    median    std     n   trim_mean  n_kept  n_dropped")
    hw_trim = np.full(6, np.nan)
    for k in range(6):
        vals = hw_arr[:, k]
        finite = vals[np.isfinite(vals)]
        if len(finite) == 0:
            print(f"    {k}    |  nan     nan     nan     0   nan        0       0")
            continue
        tm, n_keep, n_drop = trimmed_mean(vals)
        hw_trim[k] = tm
        print(f"    {k}    | {finite.mean():.4f}  {np.median(finite):.4f}  "
              f"{finite.std():.4f}  {len(finite)}   {tm:.4f}     {n_keep}       {n_drop}")

    print(f"\n  _sensor_cal_s axis stats:")
    print(f"    axis |  mean    median    std     n   trim_mean  n_kept  n_dropped")
    s_trim = np.full(4, np.nan)
    for k in range(4):
        vals = s_arr[:, k]
        finite = vals[np.isfinite(vals)]
        if len(finite) == 0:
            print(f"    {k}    |  nan     nan     nan     0   nan        0       0")
            continue
        tm, n_keep, n_drop = trimmed_mean(vals)
        s_trim[k] = tm
        print(f"    {k}    | {finite.mean():.4f}  {np.median(finite):.4f}  "
              f"{finite.std():.4f}  {len(finite)}   {tm:.4f}     {n_keep}       {n_drop}")

    print(f"\n  Proposed (MAD-outlier-rejected mean across runs, k_MAD=2.5):")
    print(f"    _sensor_cal_hw = np.diag({list(np.round(hw_trim, 4))})")
    print(f"    _sensor_cal_s  = np.diag({list(np.round(s_trim, 4))})")


if __name__ == '__main__':
    main()
