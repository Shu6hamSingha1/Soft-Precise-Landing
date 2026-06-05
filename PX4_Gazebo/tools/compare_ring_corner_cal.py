#!/usr/bin/env python3
"""Compare ring vs corner optical flow + cross-validate the ring cal.

Usage: compare_ring_corner_cal.py [glob]      (default calibration_data/output/*/)

Reports, on block-averaged (K=15, removes fps/LK per-frame noise) data:
  (1) N_ring survival  -- does ring tracking hold (>=40 stations)?
  (2) ring_raw vs corner_raw agreement per DOF (matched LK should raise corr)
  (3) leave-one-run-out ring-cal R^2 per DOF (target = corner-cal GT proxy,
      co-sampled in Img_Data so no img/GT clock skew). Read-only.
"""
import numpy as np, os, glob, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import calibrate_flow_to_touchdown as cft
CAL = cft.CAL
LBL = ['h_x', 'h_y', 'h_z(div)', 'w_x', 'w_y', 'w_z']


def load(d, K=15):
    img = np.load(os.path.join(d, 'Img_Data.npy'), allow_pickle=True).item()
    ring = np.asarray(img['Ring Opt Flow Ang Vel'], float)
    corn = np.asarray(img['Opt Flow Ang Vel'], float)
    ncr = np.asarray(img['N Ring Corners'], float)
    ncc = np.asarray(img['N Flow Corners'], float)
    n = min(len(ring), len(corn), len(ncr), len(ncc))
    ring, corn, ncr, ncc = ring[:n], corn[:n], ncr[:n], ncc[:n]
    idx = np.where((ncr > 0) & (ncc > 0))[0]
    rb, cb = [], []
    for j in range(0, len(idx) - K, K):
        sl = idx[j:j + K]
        rb.append(ring[sl].mean(0)); cb.append(corn[sl].mean(0))
    return np.array(rb), np.array(cb), ncr


def main():
    g = sys.argv[1] if len(sys.argv) > 1 else 'calibration_data/output/*/'
    dirs = sorted(glob.glob(g), key=os.path.getmtime)
    R, C, NR, runs = [], [], [], []
    for d in dirs:
        try:
            r, c, ncr = load(d)
        except Exception as e:
            print(f"  skip {os.path.basename(d.rstrip('/'))}: {e}"); continue
        if len(r) > 5:
            R.append(r); C.append(c); NR.append(ncr); runs.append(d)
    if len(runs) < 2:
        print("need >=2 usable runs"); return
    Rall, Call = np.vstack(R), np.vstack(C)
    NRall = np.concatenate(NR)
    print(f"{len(runs)} runs, {len(Rall)} denoised blocks (K=15)")
    print(f"\n(1) N_ring: mean {np.nanmean(NRall):.0f}   %frames>=40: "
          f"{100*np.mean(NRall >= 40):.0f}%   (tracking survival)")
    print("\n(2) ring_raw vs corner_raw  (matched LK -> corr should rise toward ~1):")
    for k in range(6):
        cr = np.corrcoef(Rall[:, k], Call[:, k])[0, 1]
        sl = np.polyfit(Call[:, k], Rall[:, k], 1)[0]
        print(f"    {LBL[k]:9s} corr={cr:+.2f} slope={sl:+.2f}")

    def r2(p, t):
        return 1 - np.sum((p - t) ** 2, 0) / (np.sum((t - t.mean(0)) ** 2, 0) + 1e-9)
    held = np.zeros((len(runs), 6))
    for i in range(len(runs)):
        Rtr = np.vstack([R[j] for j in range(len(runs)) if j != i])
        Ctr = np.vstack([C[j] for j in range(len(runs)) if j != i])
        M, _, _, _ = np.linalg.lstsq(Rtr, (CAL @ Ctr.T).T, rcond=None)
        held[i] = r2(R[i] @ M, (CAL @ C[i].T).T)
    print("\n(3) ring cal leave-one-run-out R^2 (per DOF):")
    print("   ", dict(zip(LBL, np.round(np.nanmean(held, 0), 2))))


if __name__ == '__main__':
    main()
