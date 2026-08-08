"""Derive the output-calibration for the cross+stub marker
(apps/record_cross_marker_calibration.py recordings, calibration_data/output_cross/).

Same fit as derive_board_cal.py (full 6x6 M for h/w, GT = M @ raw; diagonal
scale for the centroid s), but trimmed for the cross-marker pipeline: no
ring flow, no PlanarFeatureMap, no per-estimator tags -- CrossMarkerNode has
none of those subsystems (src/cross_marker_perception.py), so this tool
only reads 'Opt Flow Ang Vel' / 'Img Feature Params' from the GT dict.

Outputs (ready to paste into CrossMarkerPerception.__init__, replacing the
identity placeholders):
  self._sensor_cal_hw = np.array([... 6x6 ...])      # GT = M @ raw
  self._sensor_cal_s  = np.diag([sx, sy, 1.0, 1.0])  # centroid scale
"""
import numpy as np, os, glob, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from aggregate_calibration_phased import (compute_gt_signals, std_ratio,
                                           kf_filter_causal, FLOW_KF_Q, FLOW_KF_R,
                                           FEAT_KF_Q, FEAT_KF_R)

CAL_DIR = os.environ.get(
    "CROSS_CAL_DIR",
    '/home/shubham/Soft-Precise-Landing/PX4_Gazebo/calibration_data/output_cross')
LAB = ['Hx', 'Hy', 'Hz', 'Wx', 'Wy', 'Wz']
RL  = ['h0', 'h1', 'h2', 'w0', 'w1', 'w2']

# 2026-08-01/02 (roi_frac investigation): even with the anisotropic ROI crop fix,
# runs are bimodal -- most land at 99-100% detection ok-rate, but a residual
# minority (marker/ghost merging at altitude, or occasional SITL spawn flakes --
# see Memory/px4/project_cross_marker_pipeline_20260801.md) drop to 55-80%.
# Blindly averaging in a degraded run corrupts the fit disproportionately: the
# centroid scale (sx/sy) and lateral h/w rows are derived FROM the exact x/y-phase
# samples the dropout concentrates in, so R^2 and inter-run std both cratered when
# a bad run was included. Gate on the per-frame 'Diag Log' ok-rate (recorded by
# apps/record_cross_marker_calibration.py via CrossMarkerNode.get_diag_log()) and
# skip runs below threshold -- cheaper and more principled than manually curating
# which run directories to feed in.
MIN_OK_RATE = float(os.environ.get("CROSS_CAL_MIN_OKRATE", "0.95"))

# 2026-08-02 (centroid sx/sy convergence): Hardware/scripts/derive_pi_cal_clean_axis.py
# hit the same problem on real hardware -- phase_labels()/the maneuver script's own
# phase tag only requires the labeled axis to carry the PLURALITY of a window's
# excitation energy, not that other axes are near-zero. A window tagged "x phase" can
# still have real simultaneous yaw/z motion (PX4 tracking imperfection), which can
# distort or even INVERT the fitted sx. Pi's fix (2/6 runs there were giving a
# physically-impossible negative sx): a STRICT purity gate computed directly from the
# GT velocity itself, not trusted from the script's phase label -- the dominant axis
# must be clearly excited AND every OTHER axis must sit below an absolute floor for
# the whole window. Adopted here the same way.
PURITY_MAX = float(os.environ.get("CROSS_PURITY_MAX", "0.06"))       # V_h_g units (h=v/z), non-dominant translation
PURITY_YAW_MAX = float(os.environ.get("CROSS_PURITY_YAW_MAX", "0.15"))  # rad/s, yaw during a translation window
WIN_S = float(os.environ.get("CROSS_PURITY_WIN_S", "0.5"))


def clean_axis_mask(t, phase, V_h_g, yaw_rate):
    """Per-sample boolean 'genuinely clean' mask for the axis the maneuver
    script LABELS as active in `phase` (0=X, 1=Y). Unlike
    derive_pi_cal_clean_axis.py's version (which also auto-detects the
    dominant axis from GT, needed there since Pi's flight isn't a clean
    single-axis script), we already trust the phased-excitation script's own
    axis label for WHICH axis should be dominant -- the purity gate only
    needs to verify every OTHER axis is genuinely near-zero for that window,
    not inferred contamination like a freeform flight would need."""
    vx, vy, vz = V_h_g[:, 0], V_h_g[:, 1], V_h_g[:, 2]
    clean = np.zeros(len(t), dtype=bool)
    if len(t) < 4:
        return clean
    edges = np.arange(t[0], t[-1], WIN_S)
    for a, b in zip(edges[:-1], edges[1:]):
        w = (t >= a) & (t < b)
        if w.sum() < 3:
            continue
        ph = phase[w]
        if not (np.all(ph == 'x') or np.all(ph == 'y')):
            continue   # window straddles a phase boundary -- skip, don't guess
        axis = 0 if ph[0] == 'x' else 1
        others = {0: vx, 1: vy, 2: vz}
        oth_vals = [np.nanmedian(np.abs(others[k][w])) for k in (0, 1, 2) if k != axis]
        yaw_val = np.nanmedian(np.abs(yaw_rate[w]))
        if not all(np.isfinite(oth_vals)) or not np.isfinite(yaw_val):
            continue
        if all(v < PURITY_MAX for v in oth_vals) and yaw_val < PURITY_YAW_MAX:
            clean[w] = True
    return clean


def main():
    runs = sorted(d for d in glob.glob(os.path.join(CAL_DIR, '*')) if os.path.isdir(d))
    Ms, R2s, calS = [], [], []
    used = 0
    for d in runs:
        try:
            gt = np.load(os.path.join(d, 'Ground_Truth.npy'), allow_pickle=True).item()
        except Exception:
            continue
        if 'Phase' not in gt:
            continue
        diag_log = gt.get('Diag Log', [])
        if diag_log:
            # 2026-08-02 (s/alpha calibration investigation): Diag Log starts as soon as
            # CrossMarkerNode is constructed, well before CONTROLLER_READY/arm -- the
            # pre-arm/settling period's detections (drone on the ground, camera pointed
            # arbitrarily) are frequently bad and were dragging this metric down even
            # though that period is NEVER part of the recorded flight data (Opt Flow Ang
            # Vel/Img Feature Params only start appending post-CONTROLLER_READY). A run
            # measured at "86% ok" this way was actually 100% ok over the samples that
            # matter. Filter to t >= Start Time before computing the rate.
            start = gt.get('Start Time')
            if start is not None:
                diag_log = [r for r in diag_log if r[0] - start >= 0]
            ok_rate = sum(r[1] for r in diag_log) / max(len(diag_log), 1)
            if ok_rate < MIN_OK_RATE:
                print(f"  skip {os.path.basename(d)}: detection ok_rate {ok_rate:.1%} "
                      f"< {MIN_OK_RATE:.0%} threshold (degraded run, likely ghost/merge "
                      f"or SITL spawn flake -- see cross_marker_detector.py roi history)")
                continue
        try:
            t, V_h_g, V_w_ug, V_xc_g, V_yc_g, valid, _ = compute_gt_signals(gt)
            # See derive_board_cal.py's identical comment: the V-frame is
            # gravity-leveled, so the GT w-axis is yaw-only.
            V_w_ug = V_w_ug.copy(); V_w_ug[:, 0:2] = 0.0
        except Exception as e:
            print(f"  skip {os.path.basename(d)}: {e}"); continue

        raw = np.asarray(gt['Opt Flow Ang Vel'])
        raw_s = np.asarray(gt['Img Feature Params'])
        t_outer = np.asarray(gt['Time'])   # outer 200Hz loop clock, relative to Start Time
        nm = min(len(raw), len(raw_s), len(valid), len(t_outer))
        raw = raw[:nm][valid[:nm]]
        raw_s = raw_s[:nm][valid[:nm]]
        t_outer = t_outer[:nm][valid[:nm]]
        phase = np.array(gt['Phase'])[:nm][valid[:nm]]

        # STRICT purity gate (see Hardware/scripts/derive_pi_cal_clean_axis.py) --
        # computed on t/phase's own native alignment, before the sync_t interpolation
        # below re-indexes onto the outer loop's clock. t/V_h_g/V_w_ug (length =
        # compute_gt_signals' own valid.sum()) and phase (length = valid[:nm].sum())
        # can differ by a few samples if raw/raw_s were shorter than compute_gt_signals'
        # own working range -- truncate to the common length rather than assume equal.
        n_native = min(len(t), len(phase))
        clean_native = clean_axis_mask(t[:n_native], phase[:n_native],
                                        V_h_g[:n_native], -V_w_ug[:n_native, 2])
        # 2026-08-06: clean_zyaw_mask (the z/yaw purity gate previously applied here)
        # was REMOVED -- it was built on a false premise. The "contaminated z-phase"
        # finding that motivated it traced to a bug in an AD-HOC DIAGNOSTIC SCRIPT (not
        # this tool): that script truncated gt['Phase'] via a naive [:n] slice instead
        # of applying the SAME valid-mask filter compute_gt_signals() applies internally
        # to build t/V_h_g (which drops scattered duplicate-timestamp samples, not just
        # a trailing block) -- so phase[:n] and t/V_h_g were desynced, and the
        # "contaminated" window it found didn't correspond to the real z-phase data at
        # all. Re-checked all 6 recordings with correct alignment (phase[:nm][valid[:nm]],
        # matching this tool's own indexing) and z/yaw/yawagg are genuinely clean in
        # every run (z: vz~0.15-0.35 dominant, wz~0.001-0.003; yaw/yawagg: wz~1.1-1.3
        # dominant). The gate made Hz/Wz WORSE (not better) when it was live, consistent
        # with it having no real contamination to remove. See project memory for the
        # full trace; Hz/Wz weakness (if it persists here) is NOT a data-purity issue --
        # look elsewhere (observability/excitation-amplitude/geometry) before re-adding
        # a gate like this.

        # PER-SAMPLE time sync (2026-08-02, replaces same-index alignment): raw/raw_s
        # are read from the async perception thread (~60Hz) by the outer 200Hz polling
        # loop, so each sample's TRUE effective timestamp is whenever that thread last
        # finished a frame -- NOT t_outer[i], which is up to one camera period (~16.7ms)
        # later and, critically, staggers differently sample-to-sample (not a single
        # fixed lag for the whole run -- confirmed empirically: per-sample sync moved
        # correlation by +0.01 to +0.07 depending on channel/run, more than a constant
        # shift alone). Flow Diag Log records each REAL frame's own capture stamp
        # (post-2026-08-02 dt fix, jitter-free); for each outer-loop sample, find the
        # most recently completed frame at or before that poll and interpolate GT at
        # THAT timestamp instead of at t_outer directly.
        flog = gt.get('Flow Diag Log', [])
        start = gt.get('Start Time')
        if flog and start is not None:
            flog_t_rel = np.array([r[0] for r in flog]) - start
            idx = np.searchsorted(flog_t_rel, t_outer, side='right') - 1
            idx = np.clip(idx, 0, len(flog_t_rel) - 1)
            sync_t = flog_t_rel[idx]
        else:
            sync_t = t_outer   # older recordings without Flow Diag Log -- fall back

        n_kf = min(len(raw), len(sync_t))
        raw = raw[:n_kf]; raw_s = raw_s[:n_kf]; phase = phase[:n_kf]; sync_t = sync_t[:n_kf]
        if n_kf > 1:
            raw = kf_filter_causal(raw, sync_t, FLOW_KF_Q, FLOW_KF_R)
            raw_s = kf_filter_causal(raw_s, sync_t, FEAT_KF_Q, FEAT_KF_R)

        # interpolate GT at each sample's TRUE (sync_t) timestamp, not by naive index
        G = np.column_stack([np.interp(sync_t, t, V_h_g[:, k], left=np.nan, right=np.nan) for k in range(3)] +
                             [np.interp(sync_t, t, -V_w_ug[:, k], left=np.nan, right=np.nan) for k in range(3)])
        R = raw

        # nearest-match the strict-purity mask (computed on t[:n_native]'s native
        # alignment) onto sync_t -- boolean, so nearest-neighbor is the right
        # interpolation (not linear), same principle as derive_pi_cal_clean_axis.py's
        # discrete-tag nearest-match. Moved BEFORE the M-matrix fit (2026-08-06) so it
        # can also gate that fit -- previously only computed after and used for sx/sy.
        idx_c = np.clip(np.searchsorted(t[:n_native], sync_t), 0, n_native - 1)
        clean_at_sync = clean_native[idx_c]

        # NOTE: clean_axis_mask (clean_native) only ever marks x/y-phase windows --
        # it `continue`s (leaves False) for z/yaw/yawagg/settle. Gating the M-matrix
        # fit on clean_at_sync would therefore silently drop 100% of the z/yaw/yawagg
        # samples that make up the Hz/Wz rows, which is wrong (that was a bug
        # introduced alongside the now-reverted clean_zyaw_mask, not present
        # originally). The M-fit uses a finite-value filter only, same as it always
        # did before the 2026-08-06 investigation.
        m = np.all(np.isfinite(G), 1) & np.all(np.isfinite(R), 1)
        G, R = G[m], R[m]
        if len(R) < 200:
            print(f"  skip {os.path.basename(d)}: only {len(R)} valid (purity-gated) samples")
            continue
        # REDUCED regressor fit (2026-08-08, post gyro-derotation fix): only fit
        # Hx,Hy,Hz,Wz (cols/rows 0,1,2,5) against Tx,Ty,Tz,Wz raw columns -- NOT against
        # raw Wx,Wy (cols 3,4), even though those are now clean gyro values (not noisy
        # per-frame-solve artifacts) post _solve_jacobian's de-rotation fix. Reasoning:
        # an independent-multisine holdout check after that fix STILL showed Hx/Wz
        # badly overfit (Hx R^2 0.80 train -> 0.15 held-out, Wz -0.81) despite the
        # per-frame aliasing being resolved -- because a full 6-column joint fit still
        # puts a LARGE coefficient on w1(Wy) for the Hx row (comparable to the diagonal
        # term), which only transfers between flights if the TRUE Tx:Wy coupling ratio
        # (an artifact of how translate-via-pitch executes for a GIVEN maneuver shape)
        # is universal -- it isn't: the phased calibration's single-axis-at-a-time
        # excitation has a different effective Tx:Wy ratio than the multisine's
        # continuous combined excitation, so a large cross-term tuned to the training
        # maneuver doesn't generalize. Since the per-frame de-rotation ALREADY removes
        # the geometric rotation contamination from h_Tx/h_Ty directly (that's the
        # actual fix), the calibration step doesn't need a Wx/Wy correction term at all
        # -- excluding them removes the maneuver-dependent overfitting risk entirely
        # rather than trading one flavor of it for another.
        idx = [0, 1, 2, 5]   # Hx,Hy,Hz,Wz <- Tx,Ty,Tz,Wz (Wx,Wy excluded as regressors)
        R_red, G_red = R[:, idx], G[:, idx]
        Msol_red, _, _, _ = np.linalg.lstsq(R_red, G_red, rcond=None)
        pred_red = R_red @ Msol_red
        ss_red = 1 - np.sum((G_red - pred_red) ** 2, 0) / np.sum((G_red - G_red.mean(0)) ** 2, 0)
        cal = np.zeros((6, 6))
        for oi, o in enumerate(idx):
            for ii, i in enumerate(idx):
                cal[o, i] = Msol_red[ii, oi]   # Msol_red: GT_red = R_red @ Msol_red -> cal = Msol_red.T, indexed back into 6x6
        ss = np.full(6, np.nan)
        for oi, o in enumerate(idx):
            ss[o] = ss_red[oi]
        Ms.append(cal); R2s.append(ss)

        xc_true = np.interp(sync_t, t, V_xc_g, left=np.nan, right=np.nan)
        yc_true = np.interp(sync_t, t, V_yc_g, left=np.nan, right=np.nan)
        sx = std_ratio(xc_true, raw_s[:, 0], (phase == 'x') & clean_at_sync)
        sy = std_ratio(yc_true, raw_s[:, 1], (phase == 'y') & clean_at_sync)
        n_clean_x = int(((phase == 'x') & clean_at_sync).sum())
        n_clean_y = int(((phase == 'y') & clean_at_sync).sum())
        n_clean_z = int(((phase == 'z') & clean_at_sync).sum())
        n_clean_yaw = int(((phase == 'yaw') & clean_at_sync).sum())
        n_clean_yawagg = int(((phase == 'yawagg') & clean_at_sync).sum())
        n_z_total = int((phase == 'z').sum())
        print(f"  {os.path.basename(d)}: strict-purity clean samples -- x={n_clean_x} y={n_clean_y} "
              f"z={n_clean_z}/{n_z_total} yaw={n_clean_yaw} yawagg={n_clean_yawagg} "
              f"(M-fit total clean samples: {int(m.sum())})")
        calS.append([sx, sy])
        used += 1

    if not Ms:
        print(f"no cross-marker calibration recordings found in {CAL_DIR}"); return
    Ms = np.array(Ms); R2s = np.array(R2s); calS = np.array(calS)
    M = Ms.mean(0); Mstd = Ms.std(0)
    sx = float(np.nanmedian(calS[:, 0])); sy = float(np.nanmedian(calS[:, 1]))

    np.set_printoptions(precision=4, suppress=True, linewidth=130)
    print(f"=== derived from {used} cross-marker run(s) ({CAL_DIR}) ===\n")
    print("per-axis R^2 (mean):  " + "  ".join(f"{LAB[k]}={R2s[:,k].mean():.2f}" for k in range(6)))
    print("\ninter-run STD of M (small = robust):")
    print("       " + "  ".join(f"{r:>6}" for r in RL))
    for i in range(6):
        print(f"  {LAB[i]:>2} " + "  ".join(f"{Mstd[i,j]:6.3f}" for j in range(6)))
    print(f"\ncentroid cal_s:  sx={sx:.4f}  sy={sy:.4f}  (per-run: {np.round(calS,3).tolist()})")

    print("\n--- paste into CrossMarkerPerception.__init__ (src/cross_marker_perception.py) ---")
    rows = ",\n            ".join(
        "[" + ", ".join(f"{M[i,j]:+.4f}" for j in range(6)) + "]" for i in range(6))
    print(f"        self._sensor_cal_hw = np.array([\n            {rows}])")
    print(f"        self._sensor_cal_s  = np.diag([{sx:.4f}, {sy:.4f}, 1.0, 1.0])")


if __name__ == "__main__":
    main()
