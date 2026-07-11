"""Derive the board-era sensor calibration from output_calibration recordings.

Board-era replacement for the diagonal aggregate_calibration_phased.py.
Because the multi-marker board makes [h;w] full-rank but with a fixed geometric
h<->w coupling (L cols v_x<->w_y, v_y<->w_x), the correct calibration is a FULL
6x6 matrix M with GT[h;w] = M @ raw[h;w], not a per-axis diagonal. The centroid
s=(xc,yc) is reconstructed by the board homography directly in V-frame units, so
its calibration is expected ~identity; we verify rather than assume.

Outputs (ready to paste into src/img_data.py):
  self._sensor_cal_hw = np.array([... 6x6 ...])      # M (GT = M @ raw)
  self._sensor_cal_s  = np.diag([sx, sy, 1.0, 1.0])  # centroid scale

Reports inter-run stability (std) and per-axis R^2 so we know how trustworthy
each entry is.
"""
import numpy as np, os, glob, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from aggregate_calibration_phased import (compute_gt_signals, std_ratio,
                                           kf_filter_causal, FLOW_KF_Q, FLOW_KF_R,
                                           FEAT_KF_Q, FEAT_KF_R)

CAL_DIR = '/home/shubham/Soft-Precise-Landing/PX4_Gazebo/calibration_data/output'
LAB = ['Hx', 'Hy', 'Hz', 'Wx', 'Wy', 'Wz']
RL  = ['h0', 'h1', 'h2', 'w0', 'w1', 'w2']


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
        try:
            t, V_h_g, V_w_ug, V_xc_g, V_yc_g, valid, _ = compute_gt_signals(gt)
            # HONEST GT (2026-06-06): the V-frame is gravity-leveled, so body
            # roll/pitch produces NO V-frame flow -> the GT w-axis must be
            # YAW-ONLY. Without this the Wx/Wy cal rows are garbage (R^2 ~0.4)
            # that the runtime zeroes anyway (CTRL_ZERO_WXY=1), and the h-block
            # fit is unaffected (columns independent). NOTE: this is a V-LEVELING
            # consequence (r/p de-rotated out of the V-frame here), NOT "wx/wy
            # unobservable from image flow" — that claim was OVERTURNED 2026-06-07
            # (wx/wy ARE observable in the raw/body frame w/ the spread board;
            # memory wxy-unobservable-imu-fusion-deferred). See memory
            # feedback_vframe_rhs_yaw_only / the cell-20 fix (86509bb).
            V_w_ug = V_w_ug.copy(); V_w_ug[:, 0:2] = 0.0
        except Exception as e:
            print(f"  skip {os.path.basename(d)}: {e}"); continue

        raw = np.asarray(gt['Opt Flow Ang Vel'])
        raw_s = np.asarray(gt['Img Feature Params'])
        hw_tag = np.asarray(gt.get('Opt Flow Estimator Tag', [''] * len(raw)))
        s_tag  = np.asarray(gt.get('Img Feature Estimator Tag', [''] * len(raw_s)))
        nm = min(len(raw), len(raw_s), len(valid), len(hw_tag), len(s_tag))
        raw = raw[:nm][valid[:nm]]
        raw_s = raw_s[:nm][valid[:nm]]
        hw_tag = hw_tag[:nm][valid[:nm]]
        s_tag  = s_tag[:nm][valid[:nm]]
        phase = np.array(gt['Phase'])[:nm][valid[:nm]]
        # KF-filter (matches runtime IMG_FILTER=kf default) instead of Savgol —
        # see kf_filter_causal in aggregate_calibration_phased.py. t is post-valid,
        # index-aligned with raw/raw_s here (same mask just applied above).
        n_kf = min(len(raw), len(t))
        raw = raw[:n_kf]; raw_s = raw_s[:n_kf]; phase = phase[:n_kf]
        hw_tag = hw_tag[:n_kf]; s_tag = s_tag[:n_kf]
        if n_kf > 1:
            raw = kf_filter_causal(raw, t[:n_kf], FLOW_KF_Q, FLOW_KF_R)
            raw_s = kf_filter_causal(raw_s, t[:n_kf], FEAT_KF_Q, FEAT_KF_R)

        n = min(len(raw), len(V_h_g))
        hw_tag = hw_tag[:n]
        if np.any(hw_tag != ''):
            _u, _c = np.unique(hw_tag, return_counts=True)
            print(f"        h estimator coverage: " + ", ".join(f"{u}={c}" for u, c in zip(_u, _c)))
        G = np.hstack([V_h_g[:n], -V_w_ug[:n]])      # manuscript w = -V_w_ug
        R = raw[:n]
        # Exclude synthetic 'coast' samples (predict-only extrapolation, not real data) from the
        # fit -- see feedback_estimator_blind_calibration. Back-compat: '' (untagged, older
        # recordings) is NOT excluded -- we can't tell, so don't silently drop half the data.
        m = np.all(np.isfinite(G), 1) & np.all(np.isfinite(R), 1) & (hw_tag != 'coast')
        G, R = G[m], R[m]
        if len(R) < 200:
            continue
        Msol, _, _, _ = np.linalg.lstsq(R, G, rcond=None)   # G = R @ Msol
        cal = Msol.T                                        # GT = cal @ raw
        pred = R @ Msol
        ss = 1 - np.sum((G - pred) ** 2, 0) / np.sum((G - G.mean(0)) ** 2, 0)
        Ms.append(cal); R2s.append(ss)

        # centroid scale: GT xc/yc vs board-homography xc/yc (raw_s[:,0:2]),
        # using x/y phases (signed std-ratio, same convention as aggregator), excluding coast.
        ng = min(len(V_xc_g), len(phase), len(raw_s))
        s_real = (s_tag[:ng] != 'coast')
        if np.any(s_tag[:ng] != ''):
            _u, _c = np.unique(s_tag[:ng], return_counts=True)
            print(f"        s estimator coverage: " + ", ".join(f"{u}={c}" for u, c in zip(_u, _c)))
        sx = std_ratio(V_xc_g[:ng], raw_s[:ng, 0], (phase[:ng] == 'x') & s_real)
        sy = std_ratio(V_yc_g[:ng], raw_s[:ng, 1], (phase[:ng] == 'y') & s_real)
        calS.append([sx, sy])
        used += 1

    if not Ms:
        print("no board recordings found"); return
    Ms = np.array(Ms); R2s = np.array(R2s); calS = np.array(calS)
    M = Ms.mean(0); Mstd = Ms.std(0)
    sx = float(np.nanmedian(calS[:, 0])); sy = float(np.nanmedian(calS[:, 1]))

    np.set_printoptions(precision=4, suppress=True, linewidth=130)
    print(f"=== derived from {used} board run(s) ===\n")
    print("per-axis R^2 (mean):  " + "  ".join(f"{LAB[k]}={R2s[:,k].mean():.2f}" for k in range(6)))
    print("\ninter-run STD of M (small = robust):")
    print("       " + "  ".join(f"{r:>6}" for r in RL))
    for i in range(6):
        print(f"  {LAB[i]:>2} " + "  ".join(f"{Mstd[i,j]:6.3f}" for j in range(6)))
    print(f"\ncentroid cal_s:  sx={sx:.4f}  sy={sy:.4f}  (per-run: {np.round(calS,3).tolist()})")

    # ready-to-paste literals
    print("\n--- paste into src/img_data.py ---")
    rows = ",\n            ".join(
        "[" + ", ".join(f"{M[i,j]:+.4f}" for j in range(6)) + "]" for i in range(6))
    print(f"        self._sensor_cal_hw = np.array([\n            {rows}])")
    print(f"        self._sensor_cal_s  = np.diag([{sx:.4f}, {sy:.4f}, 1.0, 1.0])")


if __name__ == "__main__":
    main()
