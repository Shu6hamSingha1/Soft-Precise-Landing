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
from scipy.signal import savgol_filter as sgf
from aggregate_calibration_phased import compute_gt_signals, std_ratio, FILTER_WIN, POLYORDER

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
            t, B_y_g, B_w_ug, xc_gt, yc_gt, valid = compute_gt_signals(gt)
        except Exception as e:
            print(f"  skip {os.path.basename(d)}: {e}"); continue

        raw = np.asarray(gt['Opt Flow Ang Vel'])
        raw_s = np.asarray(gt['Img Feature Params'])
        nm = min(len(raw), len(raw_s), len(valid))
        raw = raw[:nm][valid[:nm]]
        raw_s = raw_s[:nm][valid[:nm]]
        phase = np.array(gt['Phase'])[:nm][valid[:nm]]
        if len(raw) >= FILTER_WIN:
            raw = sgf(raw, FILTER_WIN, POLYORDER, axis=0)
            raw_s = sgf(raw_s, FILTER_WIN, POLYORDER, axis=0)

        n = min(len(raw), len(B_y_g))
        G = np.hstack([B_y_g[:n], -B_w_ug[:n]])      # manuscript w = -B_w_ug
        R = raw[:n]
        m = np.all(np.isfinite(G), 1) & np.all(np.isfinite(R), 1)
        G, R = G[m], R[m]
        if len(R) < 200:
            continue
        Msol, _, _, _ = np.linalg.lstsq(R, G, rcond=None)   # G = R @ Msol
        cal = Msol.T                                        # GT = cal @ raw
        pred = R @ Msol
        ss = 1 - np.sum((G - pred) ** 2, 0) / np.sum((G - G.mean(0)) ** 2, 0)
        Ms.append(cal); R2s.append(ss)

        # centroid scale: GT xc/yc vs board-homography xc/yc (raw_s[:,0:2]),
        # using x/y phases (signed std-ratio, same convention as aggregator).
        ng = min(len(xc_gt), len(phase), len(raw_s))
        sx = std_ratio(xc_gt[:ng], raw_s[:ng, 0], (phase[:ng] == 'x'))
        sy = std_ratio(yc_gt[:ng], raw_s[:ng, 1], (phase[:ng] == 'y'))
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
