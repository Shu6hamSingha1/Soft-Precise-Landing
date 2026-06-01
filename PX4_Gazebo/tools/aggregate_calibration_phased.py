"""Phase-aware aggregator for the redesigned output_calibration.py.

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
FILTER_WIN = 101  # offline-best savgol
POLYORDER  = 3

# Cal-axis ↔ which excitation phase to derive it from
AXIS_PHASE_HW = {0: 'x', 1: 'y', 2: 'z', 3: 'y', 4: 'x', 5: 'yaw'}
AXIS_PHASE_S  = {0: 'x', 1: 'y'}


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

    # V-frame projection (gravity-leveled, drone-yaw-aligned).
    B_y_g = np.zeros((n, 3))      # virtual image velocity h in V-frame
    xc_gt = np.zeros(n)           # virtual image position s_x in V-frame
    yc_gt = np.zeros(n)
    for i in range(n):
        yaw = np.arctan2(R_FRD_NED[i, 1, 0], R_FRD_NED[i, 0, 0])
        V_x_NED = np.array([np.cos(yaw), np.sin(yaw),  0.0])
        V_y_NED = np.array([-np.sin(yaw), np.cos(yaw), 0.0])
        V_z_NED = np.array([0.0, 0.0, 1.0])
        rel = W_x_tu_NED[i]
        Vx = rel @ V_x_NED;  Vy = rel @ V_y_NED;  Vz = rel @ V_z_NED
        if not (Vz > 1.0):
            xc_gt[i] = np.nan; yc_gt[i] = np.nan
            B_y_g[i] = np.nan
            continue
        xc_gt[i] = Vx / Vz
        yc_gt[i] = Vy / Vz
        relv = W_v_tu_NED[i]
        B_y_g[i] = np.array([relv @ V_x_NED, relv @ V_y_NED, relv @ V_z_NED]) / Vz

    # ω: V-frame ω = body-FRD ω (V-frame z = world-down; for small tilts
    # body-z ≈ V-z so ω_z components agree; ω_x/ω_y components differ by
    # O(tilt), which is small for our recordings — keep body-FRD ω for now).
    uav_quats = np.array([[U.orientation.w, U.orientation.x,
                           U.orientation.y, U.orientation.z] for U in UAV])
    B_w_ug = (FLU_2_FRD @ _body_omega_from_quats(uav_quats, t_g).T).T

    return t_g, B_y_g, B_w_ug, xc_gt, yc_gt, valid


def std_ratio(gt, raw, mask):
    """Signed scale factor = σ(GT)/σ(raw_filtered) × sign(median(GT/raw_active)).

    Captures both MAGNITUDE (from std-ratio, robust to outliers and DC offset)
    AND SIGN (from median ratio where signal is above noise floor). The signed
    median catches conventions where the LSTSQ recovery is opposite-sign of
    the GT — happens on w_x/w_y axes where L matrix col [1] (translation) and
    col [3] (rotation) are anti-parallel at image center, so LSTSQ chooses a
    minimum-norm trade-off whose sign depends on numerical conditioning, not
    on a clean convention. Pre-2026-06-01 this function was unsigned, which
    forced the controller to see anti-correlated w_x/w_y vs body ω."""
    g = gt[mask]; r = raw[mask]
    finite = np.isfinite(g) & np.isfinite(r)
    if finite.sum() < 30:
        return float('nan')
    gf = g[finite]; rf = r[finite]
    sg = float(np.std(gf))
    sr = float(np.std(rf))
    if sr < 1e-9:
        return float('nan')
    # Sign: from Pearson correlation between (raw, GT). This is the actual
    # invariant we care about — we want sign(cal·raw) = sign(GT). Per-sample
    # ratio median was too noisy when signal is weak (LSTSQ-recovered ω is
    # near-noise-floor when L conditioning is poor at small marker spread).
    co = float(np.corrcoef(gf, rf)[0, 1])
    if not np.isfinite(co) or abs(co) < 0.05:
        return sg / sr   # unsigned fallback when there is no signal to sign
    sign = +1.0 if co > 0 else -1.0
    return sign * sg / sr


def main():
    cal_dir = '/home/shubham/Soft-Precise-Landing/PX4_Gazebo/calibration_data/output'
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
            t_g, B_y_g, B_w_ug, xc_gt, yc_gt, valid_mask = compute_gt_signals(gt)
        except Exception as e:
            print(f"  [SKIP] {os.path.basename(d)}: gt compute error {e}")
            continue

        raw_hw = np.asarray(gt['Opt Flow Ang Vel'])  # (N, 6)
        raw_s  = np.asarray(gt['Img Feature Params'])  # (N, 4)
        # truncate to common length, then apply same valid mask
        n_min = min(len(raw_hw), len(raw_s), len(valid_mask))
        raw_hw = raw_hw[:n_min][valid_mask[:n_min]]
        raw_s  = raw_s[:n_min][valid_mask[:n_min]]
        phase = np.array(gt['Phase'])[:n_min][valid_mask[:n_min]]

        # savgol-filter raw inputs
        if len(raw_hw) >= FILTER_WIN:
            raw_hw_f = sgf(raw_hw, FILTER_WIN, POLYORDER, axis=0)
            raw_s_f  = sgf(raw_s,  FILTER_WIN, POLYORDER, axis=0)
        else:
            raw_hw_f = raw_hw.copy(); raw_s_f = raw_s.copy()

        # Align lengths between phase array and GT signals (gt may be a bit longer if dt dedup removed some samples)
        ngt = min(len(B_y_g), len(phase))
        phase = phase[:ngt]

        # Per-axis derivation using only that axis's phase samples
        hw_diag = np.full(6, np.nan)
        for k, want_phase in AXIS_PHASE_HW.items():
            mask = (phase == want_phase)
            if mask.sum() < 30: continue
            if k < 3:   # virtual image VELOCITY h (= target rel camera vel / depth)
                gt_sig = B_y_g[:ngt, k]
            else:       # virtual image ANGULAR velocity w = -ω_camera (target rel
                        # to camera, manuscript convention). For stationary target:
                        #   w = ω_target - ω_camera = -ω_UAV_body  (since target ω = 0)
                        # Plotter compares w_cal to B_w_tug = -B_w_ug; the aggregator
                        # must match this convention or the derived cal sign is wrong.
                gt_sig = -B_w_ug[:ngt, k - 3]
            hw_diag[k] = std_ratio(gt_sig, raw_hw_f[:ngt, k], mask)

        s_diag = np.full(4, np.nan)
        for k, want_phase in AXIS_PHASE_S.items():
            mask = (phase == want_phase)
            if mask.sum() < 30: continue
            gt_sig = xc_gt[:ngt] if k == 0 else yc_gt[:ngt]
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
        print("  Run output_calibration.py (the new phased version) to generate")
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
