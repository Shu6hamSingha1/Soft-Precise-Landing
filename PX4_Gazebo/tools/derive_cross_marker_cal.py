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
        nm = min(len(raw), len(raw_s), len(valid))
        raw = raw[:nm][valid[:nm]]
        raw_s = raw_s[:nm][valid[:nm]]
        phase = np.array(gt['Phase'])[:nm][valid[:nm]]
        n_kf = min(len(raw), len(t))
        raw = raw[:n_kf]; raw_s = raw_s[:n_kf]; phase = phase[:n_kf]
        if n_kf > 1:
            raw = kf_filter_causal(raw, t[:n_kf], FLOW_KF_Q, FLOW_KF_R)
            raw_s = kf_filter_causal(raw_s, t[:n_kf], FEAT_KF_Q, FEAT_KF_R)

        n = min(len(raw), len(V_h_g))
        G = np.hstack([V_h_g[:n], -V_w_ug[:n]])
        R = raw[:n]
        m = np.all(np.isfinite(G), 1) & np.all(np.isfinite(R), 1)
        G, R = G[m], R[m]
        if len(R) < 200:
            print(f"  skip {os.path.basename(d)}: only {len(R)} valid samples")
            continue
        Msol, _, _, _ = np.linalg.lstsq(R, G, rcond=None)
        cal = Msol.T
        pred = R @ Msol
        ss = 1 - np.sum((G - pred) ** 2, 0) / np.sum((G - G.mean(0)) ** 2, 0)
        Ms.append(cal); R2s.append(ss)

        ng = min(len(V_xc_g), len(phase), len(raw_s))
        sx = std_ratio(V_xc_g[:ng], raw_s[:ng, 0], (phase[:ng] == 'x'))
        sy = std_ratio(V_yc_g[:ng], raw_s[:ng, 1], (phase[:ng] == 'y'))
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
