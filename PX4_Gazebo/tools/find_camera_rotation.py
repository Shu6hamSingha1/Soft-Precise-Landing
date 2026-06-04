"""Empirically determine the camera→body rotation (R_BC) from recorded GT.

Theory says the LSTSQ recovers [h_cam; w_cam] in the (cv2-rotated) optical
frame, while the controller/GT live in body-FRD. The map between them is a
signed axis permutation (a rotation by multiples of 90°, since the SDF mount
is pure 90° pitches and cv2 is a 90° in-plane rotation).

Rather than trust a by-hand frame chain (too many sign traps: optical vs
Gazebo-sensor, FLU vs FRD, cv2 direction, double-pitch SDF), we read the
mapping straight off the data: for each single-axis excitation phase, correlate
every raw LSTSQ axis against every GT axis. The dominant entry per column,
with its sign, IS the rotation.

GT convention here (to find the raw→GT map; sign discussion separate):
  h GT  = V_h_g  (V-frame virtual image velocity, 3-vec)
  w GT  = B_w_ug (body-FRD UAV angular velocity, 3-vec)
Raw = gt['Opt Flow Ang Vel'] (the 6-vec LSTSQ output, pre-cal).

Phases excite one body axis at a time, so the correlation is clean:
  'x'   → body x motion (and the pitch ω_y it induces via PID)
  'y'   → body y motion (and roll ω_x)
  'z'   → body z motion
  'yaw' → body yaw ω_z
"""
import os, glob, sys
import numpy as np
from scipy.signal import savgol_filter as sgf

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from aggregate_calibration_phased import compute_gt_signals, FILTER_WIN, POLYORDER

CAL_DIR = '/home/shubham/Soft-Precise-Landing/PX4_Gazebo/calibration_data/output'

RAW_LABELS = ['h0', 'h1', 'h2', 'w0', 'w1', 'w2']   # LSTSQ output axes
GT_LABELS  = ['Hx', 'Hy', 'Hz', 'Wx', 'Wy', 'Wz']   # body/V-frame GT axes
PHASES = ['x', 'y', 'z', 'yaw']


def corr(a, b):
    fa = np.isfinite(a) & np.isfinite(b)
    if fa.sum() < 30:
        return np.nan
    a = a[fa]; b = b[fa]
    if np.std(a) < 1e-9 or np.std(b) < 1e-9:
        return np.nan
    return float(np.corrcoef(a, b)[0, 1])


def main():
    runs = sorted([d for d in glob.glob(os.path.join(CAL_DIR, '*')) if os.path.isdir(d)])
    # Accumulate the 6x6 correlation matrix, averaged over runs, per phase.
    # We pool by stacking samples across runs (concatenate), which is more
    # robust than averaging per-run corr coefficients.
    pooled_raw = {ph: [] for ph in PHASES}
    pooled_gt  = {ph: [] for ph in PHASES}

    n_ok = 0
    for d in runs:
        try:
            gt = np.load(os.path.join(d, 'Ground_Truth.npy'), allow_pickle=True).item()
        except Exception as e:
            continue
        if 'Phase' not in gt:
            continue
        try:
            t_g, V_h_g, B_w_ug, V_xc_g, V_yc_g, valid = compute_gt_signals(gt)
        except Exception as e:
            print(f"  skip {os.path.basename(d)}: {e}")
            continue

        raw = np.asarray(gt['Opt Flow Ang Vel'])            # (N,6)
        n_min = min(len(raw), len(valid))
        raw = raw[:n_min][valid[:n_min]]
        phase = np.array(gt['Phase'])[:n_min][valid[:n_min]]

        if len(raw) >= FILTER_WIN:
            raw_f = sgf(raw, FILTER_WIN, POLYORDER, axis=0)
        else:
            raw_f = raw.copy()

        # Full 6-vec GT: [V_h_g (3), B_w_ug (3)]
        ngt = min(len(V_h_g), len(phase), len(raw_f))
        gt6 = np.hstack([V_h_g[:ngt], B_w_ug[:ngt]])         # (ngt,6)
        raw_f = raw_f[:ngt]
        phase = phase[:ngt]

        for ph in PHASES:
            m = (phase == ph)
            if m.sum() < 30:
                continue
            pooled_raw[ph].append(raw_f[m])
            pooled_gt[ph].append(gt6[m])
        n_ok += 1

    print(f"[rotation] pooled across {n_ok} runs\n")

    # For each phase, the active GT axes (what was excited). Print correlation
    # of each raw axis vs each GT axis.
    print("Per-phase raw↔GT correlation (only |r|>0.3 shown):\n")
    # Build an aggregate "best GT axis per raw axis" vote across phases.
    for ph in PHASES:
        if not pooled_raw[ph]:
            continue
        R = np.vstack(pooled_raw[ph])
        G = np.vstack(pooled_gt[ph])
        print(f"=== phase '{ph}'  (n={len(R)}) ===")
        # GT signal magnitude per axis (is the excitation present in GT?)
        gt_std = [float(np.nanstd(G[:, j])) for j in range(6)]
        raw_std = [float(np.nanstd(R[:, i])) for i in range(6)]
        print("  GT  std: " + "  ".join(f"{GT_LABELS[j]}={gt_std[j]:6.3f}" for j in range(6)))
        print("  RAW std: " + "  ".join(f"{RAW_LABELS[i]}={raw_std[i]:6.3f}" for i in range(6)))
        header = "        " + "  ".join(f"{g:>6}" for g in GT_LABELS)
        print(header)
        for i in range(6):
            row = []
            for j in range(6):
                c = corr(R[:, i], G[:, j])
                if np.isnan(c):
                    row.append("   .  ")
                elif abs(c) > 0.15:
                    row.append(f"{c:+5.2f}")
                else:
                    row.append("   .  ")
            print(f"  {RAW_LABELS[i]:>4}  " + "  ".join(f"{s:>6}" for s in row))
        print()


if __name__ == "__main__":
    main()
