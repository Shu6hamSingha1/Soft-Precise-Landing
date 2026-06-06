"""Derive the TEXTURE-FREE ring sensor calibration M_ring — ring analogue of
derive_board_cal.py. The ring lstsq is NOT depth-mixed (board coplanar + V-frame
leveling -> uniform depth; ring h_z ~ corner h_z at r~0.95), so a FIXED 6x6
M_ring generalizes: GT[h;w] = M_ring @ ring_raw.

Two modes (RING_CAL_MODE):

  'transfer' (DEFAULT, best for existing data) — calibrate the ring against the
      ALREADY-CALIBRATED CORNER as a transfer standard:  M_ring @ ring_raw ~=
      M_corner @ corner_raw.  Both raws live in Img_Data on the SAME clock, so
      there is NO cross-clock alignment (the binding error source for 'gt' on old
      recordings). Uses rich descent motion (landings + output runs). Bonus: makes
      ring_cal ~= corner_cal, so the marker->ring handoff is continuous. Inherits
      the corner cal's yaw-only-w convention automatically (corner Wx/Wy rows are
      0 -> the transfer target has Wx/Wy=0 -> ring Wx/Wy rows come out 0).

  'gt' — GT-direct (the 'true' cal). Uses co-sampled ring from Ground_Truth when
      present (clean), else RETRO-aligns the ring from Img_Data via the shared
      corner signal (cross-clock; noisier). Yaw-only w GT (V_w_ug[:, :2]=0) unless
      RING_CAL_FULL_W=1. PREFER this once co-sampled recordings exist (re-run
      output_calibration, which now co-samples the ring).

Outputs a paste-ready _sensor_cal_ring 6x6 for src/img_data.py.
"""
import numpy as np, os, glob, sys, re
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from scipy.signal import savgol_filter as sgf
from aggregate_calibration_phased import compute_gt_signals, FILTER_WIN, POLYORDER

OUT_DIR  = '/home/shubham/Soft-Precise-Landing/PX4_Gazebo/calibration_data/output'
LAND_DIR = '/home/shubham/Soft-Precise-Landing/PX4_Gazebo/test_data/RingFlow'
LAB = ['Hx', 'Hy', 'Hz', 'Wx', 'Wy', 'Wz']
RL  = ['h0', 'h1', 'h2', 'w0', 'w1', 'w2']
MODE = os.environ.get('RING_CAL_MODE', 'transfer').lower()


def _load_corner_M():
    src = open(os.path.join(os.path.dirname(__file__), '..', 'src', 'img_data.py')).read()
    m = re.search(r'self\._sensor_cal_hw\s*=\s*(np\.array\(\[.*?\]\))', src, re.S)
    return eval(m.group(1), {'np': np})


# ---------------------------------------------------------------- transfer mode
def transfer_mode():
    Mc = _load_corner_M()
    dirs = sorted(glob.glob(os.path.join(LAND_DIR, '*/'))) + sorted(glob.glob(os.path.join(OUT_DIR, '*/')))
    Ms, R2s = [], []
    for d in dirs:
        f = os.path.join(d, 'Img_Data.npy')
        if not os.path.exists(f):
            continue
        try:
            img = np.load(f, allow_pickle=True).item()
        except Exception:
            continue
        if 'Ring Opt Flow Ang Vel' not in img:
            continue
        ring = np.asarray(img['Ring Opt Flow Ang Vel'], float)
        corn = np.asarray(img['Opt Flow Ang Vel'], float)
        ncr = np.asarray(img['N Ring Corners'], float); ncc = np.asarray(img['N Flow Corners'], float)
        n = min(len(ring), len(corn), len(ncr), len(ncc))
        ring, corn, ncr, ncc = ring[:n], corn[:n], ncr[:n], ncc[:n]
        msk = (ncr > 0) & (ncc > 0) & np.all(np.isfinite(ring), 1) & np.all(np.isfinite(corn), 1)
        ring, corn = ring[msk], corn[msk]
        if len(ring) < 300:
            continue
        ring = sgf(ring, FILTER_WIN, POLYORDER, axis=0)
        corn = sgf(corn, FILTER_WIN, POLYORDER, axis=0)
        corn_cal = (Mc @ corn.T).T                                   # transfer standard ~ GT
        Mr_t, _, _, _ = np.linalg.lstsq(ring, corn_cal, rcond=None)  # ring @ Mr_t = corn_cal
        ss = 1 - np.sum((corn_cal - ring @ Mr_t) ** 2, 0) / (np.sum((corn_cal - corn_cal.mean(0)) ** 2, 0) + 1e-12)
        Ms.append(Mr_t.T); R2s.append(ss)
        print(f"  {os.path.basename(d.rstrip('/')):28} n={len(ring):6}  R2 " + " ".join(f"{x:.2f}" for x in ss))
    return Ms, R2s


# -------------------------------------------------------------------- gt mode
def retro_align_ring(gt, d):
    try:
        img = np.load(os.path.join(d, 'Img_Data.npy'), allow_pickle=True).item()
    except Exception:
        return None
    if 'Ring Opt Flow Ang Vel' not in img or 'Opt Flow Ang Vel' not in img:
        return None
    gt_t = np.asarray(gt['Time'], float)
    gt_c = np.asarray(gt['Opt Flow Ang Vel'], float)
    it = np.asarray(img['Time'], float)
    ic = np.asarray(img['Opt Flow Ang Vel'], float)
    ir = np.asarray(img['Ring Opt Flow Ang Vel'], float)
    n = min(len(it), len(ic), len(ir)); it, ic, ir = it[:n], ic[:n], ir[:n]
    gt_r = gt_t - gt_t[0]; it_r = it - it[0]
    k = int(np.nanargmax(np.nanvar(gt_c, axis=0)))
    gk, ik = gt_c[:, k], ic[:, k]
    gm, im = np.isfinite(gk), np.isfinite(ik)
    hi = max(3.0, it_r[-1] - gt_r[-1] + 4.0)
    best = (-2.0, 0.0)
    for off in np.arange(-2.0, hi, 0.05):
        g = np.interp(gt_r[gm] + off, it_r[im], ik[im], left=np.nan, right=np.nan)
        m = np.isfinite(g)
        if m.sum() < 40:
            continue
        r = np.corrcoef(gk[gm][m], g[m])[0, 1]
        if np.isfinite(r) and r > best[0]:
            best = (r, off)
    rcorr, off = best
    if rcorr < 0.3:
        return None
    print(f"  retro-align: off={off:+.2f}s r={rcorr:.2f} (ch {LAB[k]})")
    return np.column_stack([np.interp(gt_r + off, it_r, ir[:, j], left=np.nan, right=np.nan) for j in range(6)])


def gt_mode():
    runs = sorted(d for d in glob.glob(os.path.join(OUT_DIR, '*')) if os.path.isdir(d))
    policy = os.environ.get('RING_CAL_COSAMPLED_ONLY', 'auto').lower()
    have_cs = any('Ring Opt Flow Ang Vel' in np.load(os.path.join(d, 'Ground_Truth.npy'), allow_pickle=True).item()
                  for d in runs if os.path.exists(os.path.join(d, 'Ground_Truth.npy')))
    cs_only = (policy in ('1', 'true', 'yes')) or (policy == 'auto' and have_cs)
    if cs_only:
        print("[co-sampled-only] retro-aligned runs excluded\n")
    Ms, R2s = [], []
    for d in runs:
        try:
            gt = np.load(os.path.join(d, 'Ground_Truth.npy'), allow_pickle=True).item()
        except Exception:
            continue
        if 'Phase' not in gt:
            continue
        try:
            t, V_h_g, V_w_ug, V_xc_g, V_yc_g, valid, _ = compute_gt_signals(gt)
        except Exception as e:
            print(f"  skip {os.path.basename(d)}: {e}"); continue
        if 'Ring Opt Flow Ang Vel' in gt:
            ring = np.asarray(gt['Ring Opt Flow Ang Vel'], float); src = 'co-sampled'
        else:
            if cs_only:
                continue
            ring = retro_align_ring(gt, d); src = 'retro'
            if ring is None:
                continue
        nm = min(len(ring), len(valid))
        ring = ring[:nm][valid[:nm]]
        m_fin = np.all(np.isfinite(ring), 1)
        if m_fin.sum() < FILTER_WIN + 50:
            continue
        ring = ring[m_fin]
        if len(ring) >= FILTER_WIN:
            ring = sgf(ring, FILTER_WIN, POLYORDER, axis=0)
        n = min(len(ring), len(V_h_g))
        Vw = np.asarray(-V_w_ug[:n], float)
        if os.environ.get('RING_CAL_FULL_W', '0') != '1':
            Vw[:, :2] = 0.0
        G = np.hstack([V_h_g[:n], Vw]); R = ring[:n]
        m = np.all(np.isfinite(G), 1) & np.all(np.isfinite(R), 1)
        G, R = G[m], R[m]
        if len(R) < 200:
            continue
        Msol, _, _, _ = np.linalg.lstsq(R, G, rcond=None)
        ss = 1 - np.sum((G - R @ Msol) ** 2, 0) / (np.sum((G - G.mean(0)) ** 2, 0) + 1e-12)
        Ms.append(Msol.T); R2s.append(ss)
        print(f"  used {os.path.basename(d)} [{src}]  n={len(R)}")
    return Ms, R2s


def main():
    print(f"=== RING_CAL_MODE={MODE} ===")
    Ms, R2s = transfer_mode() if MODE == 'transfer' else gt_mode()
    if not Ms:
        print("no ring recordings usable"); return
    Ms = np.array(Ms); R2s = np.array(R2s)
    M = Ms.mean(0); Mstd = Ms.std(0)
    np.set_printoptions(precision=4, suppress=True, linewidth=130)
    print(f"\n=== M_ring from {len(Ms)} recording(s), mode={MODE} ===\n")
    print("per-axis R^2 (mean):  " + "  ".join(f"{LAB[k]}={np.nanmean(R2s[:,k]):.2f}" for k in range(6)))
    print("\ninter-run STD of M_ring (small = robust):")
    print("       " + "  ".join(f"{r:>6}" for r in RL))
    for i in range(6):
        print(f"  {LAB[i]:>2} " + "  ".join(f"{Mstd[i,j]:6.3f}" for j in range(6)))
    print("\n--- paste into src/img_data.py (applied to the ring [h;w]) ---")
    rows = ",\n            ".join("[" + ", ".join(f"{M[i,j]:+.4f}" for j in range(6)) + "]" for i in range(6))
    print(f"        self._sensor_cal_ring = np.array([\n            {rows}])")


if __name__ == "__main__":
    main()
