"""Reduced-solve std-ratio lateral cal (h_x, h_y) for the SINGLE-MARKER path.

Matches the 2026-06-25 method baked in img_data.py: with FLOW_LAT_REDUCED=1 the
recorded 'Opt Flow Ang Vel' is the reduced solve (w_xy cols dropped), so h_xy is a
pure per-axis scale beta = sign(corr)*sigma_GT/sigma_raw (std_ratio, IRLS reject),
taken on the matching lateral phase, median over runs. Prints the assembled
_sensor_cal_hw candidate (h_xy diagonal + loom/w_z from the board fit) and
_sensor_cal_s, with per-run stability. Read-only; pastes nothing.
"""
import numpy as np, os, glob, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from scipy.signal import savgol_filter as sgf
from aggregate_calibration_phased import compute_gt_signals, std_ratio, FILTER_WIN, POLYORDER

CAL = '/home/shubham/Soft-Precise-Landing/PX4_Gazebo/calibration_data/output'

def main():
    runs = sorted(d for d in glob.glob(os.path.join(CAL, '*')) if os.path.isdir(d))
    bx, by = [], []
    for R in runs:
        gt = np.load(os.path.join(R, 'Ground_Truth.npy'), allow_pickle=True).item()
        if 'Phase' not in gt:
            print(f"  [SKIP] {os.path.basename(R)}: no Phase tag"); continue
        t, V_h_g, V_w_ug, V_xc_g, V_yc_g, valid, V_z_g = compute_gt_signals(gt)
        raw = np.asarray(gt['Opt Flow Ang Vel'])
        nm = min(len(raw), len(valid))
        raw = raw[:nm][valid[:nm]]
        phase = np.array(gt['Phase'])[:nm][valid[:nm]]
        if len(raw) >= FILTER_WIN:
            raw = sgf(raw, FILTER_WIN, POLYORDER, axis=0)
        n = min(len(raw), len(V_h_g), len(phase))
        px = std_ratio(V_h_g[:n, 0], raw[:n, 0], phase[:n] == 'x')
        py = std_ratio(V_h_g[:n, 1], raw[:n, 1], phase[:n] == 'y')
        bx.append(px); by.append(py)
        print(f"  {os.path.basename(R)[:20]}: beta_x={px:+.4f}  beta_y={py:+.4f}")
    bx = np.array(bx); by = np.array(by)
    beta_x, beta_y = float(np.median(bx)), float(np.median(by))
    print(f"\n  MEDIAN  beta_x={beta_x:+.4f} (std {bx.std():.3f})  beta_y={beta_y:+.4f} (std {by.std():.3f})")
    print(f"  baked (old single-marker 1m): beta_x=+0.7300  beta_y=+0.5900")
    return beta_x, beta_y

if __name__ == '__main__':
    main()
