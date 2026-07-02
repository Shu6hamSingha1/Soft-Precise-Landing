#!/usr/bin/env python3
"""Offline c3 spectral check for the DHD_SRC arms (rover heading-hold Circular).

Per rep, over the tracking window (GT rel-alt 0.8-3.5 m), band-rms in the cycle
band 1.4-2.1 rad/s (fundamental ~1.7) of:
  - c3 = -dh_d (logged, clamped) per lateral axis  -> did the c-term content drop?
  - GT lateral error e_x, e_y                      -> did the CYCLE change?
  - e-vector rotation rate (harness-consistent)     -> cross-check
Same tool across all arms; only arm-vs-arm contrasts are meaningful.
"""
import numpy as np, os, sys, glob

BAND = (1.4, 2.1)  # rad/s

def band_rms(x, dt, band):
    x = np.asarray(x, float)
    x = x - x.mean()
    n = len(x)
    if n < 64:
        return np.nan
    X = np.fft.rfft(x * np.hanning(n))
    w = 2 * np.pi * np.fft.rfftfreq(n, dt)
    m = (w >= band[0]) & (w <= band[1])
    if not m.any():
        return np.nan
    # Parseval with hann window compensation (rms of the band-limited component)
    win_pow = (np.hanning(n) ** 2).mean()
    psd = (np.abs(X) ** 2) / (n * n * win_pow)
    return float(np.sqrt(2.0 * psd[m].sum()))

def rep_spectral(d):
    gt = np.load(os.path.join(d, "Ground_Truth.npy"), allow_pickle=True).item()
    cd = np.load(os.path.join(d, "Control_Data.npy"), allow_pickle=True).item()
    up, tp, T = gt["UAV Pose"], gt["Target Pose"], np.asarray(gt["Time"], float)
    n = min(len(up), len(tp), len(T)); T = T[:n] - T[0]
    uz = np.array([p.position.z for p in up[:n]])
    ux = np.array([p.position.x for p in up[:n]]); uy = np.array([p.position.y for p in up[:n]])
    tx = np.array([p.position.x for p in tp[:n]]); ty = np.array([p.position.y for p in tp[:n]])
    ex, ey = ux - tx, uy - ty
    ct = np.asarray(cd["t"], float); ct_rel = ct - ct[0]
    dhd = np.asarray(cd["dh_d(t)"], float)
    # GT alt on controller time; tracking window mask
    z_c = np.interp(ct_rel, T, uz)
    m_c = (z_c < 3.5) & (z_c > 0.8)
    if m_c.sum() < 100:
        return None
    dt_c = float(np.median(np.diff(ct_rel[m_c])))
    # uniform resample inside the window
    tw = ct_rel[m_c]
    tu = np.arange(tw[0], tw[-1], dt_c)
    out = {}
    for ax, name in [(0, "x"), (1, "y")]:
        c3u = np.interp(tu, tw, dhd[m_c, ax])
        out[f"c3_{name}"] = band_rms(c3u, dt_c, BAND)
    # GT error in the same window (GT clock)
    m_g = (uz < 3.5) & (uz > 0.8)
    dt_g = float(np.median(np.diff(T[m_g])))
    tg = T[m_g]; tgu = np.arange(tg[0], tg[-1], dt_g)
    for sig, name in [(ex, "ex"), (ey, "ey")]:
        s_u = np.interp(tgu, tg, sig[m_g])
        out[f"e_{name[-1]}"] = band_rms(s_u, dt_g, BAND)
    ang = np.unwrap(np.arctan2(ey[m_g], ex[m_g]))
    dtg = np.gradient(T[m_g]); dtg[dtg <= 0] = 1e-3
    out["e_rot"] = float(np.median(np.gradient(ang) / dtg))
    out["dhd_std_w"] = float(dhd[m_c, :2].std())
    return out

def arm(name, pattern):
    dirs = sorted(d for d in glob.glob(pattern) if os.path.isdir(d))
    rows = []
    for d in dirs:
        try:
            r = rep_spectral(d)
        except Exception as e:
            print(f"  {os.path.basename(d)}: ERR {e}")
            continue
        if r:
            rows.append(r)
            print(f"  {os.path.basename(d.rstrip('/')):>28s}: "
                  f"c3_x {r['c3_x']:.3f} c3_y {r['c3_y']:.3f} | "
                  f"e_x {r['e_x']:.3f} e_y {r['e_y']:.3f} | "
                  f"e_rot {r['e_rot']:+.2f} | dhd_std {r['dhd_std_w']:.2f}")
    if rows:
        med = {k: float(np.median([r[k] for r in rows])) for k in rows[0]}
        print(f"  {name} MEDIAN (n={len(rows)}): c3_x {med['c3_x']:.3f} c3_y {med['c3_y']:.3f} | "
              f"e_x {med['e_x']:.3f} e_y {med['e_y']:.3f} | e_rot {med['e_rot']:+.2f} | dhd_std {med['dhd_std_w']:.2f}")
    print()

if __name__ == "__main__":
    base = "/home/shubham/Soft-Precise-Landing/PX4_Gazebo/test_data"
    print("== full baseline (yawhold_arm_n3, HD_KR=0.5, dh_d differentiates full h_d) ==")
    arm("full", f"{base}/Rover_Turning/yawhold_arm_n3/*/")
    print("== hdkr_0 (cycle_gain_sweep, k_r FUNCTION removed) ==")
    arm("hdkr0", f"{base}/Rover_Turning/cycle_gain_sweep/hdkr_0/*/")
    print("== nokr (k_r kept, its dh_d branch dropped) ==")
    arm("nokr", f"{base}/Rover_Turning/dhd_src_sweep/dhd_nokr/*/")
    print("== nos (k_r kept, whole rate term dropped from dh_d) ==")
    arm("nos", f"{base}/Rover_Turning/dhd_src_sweep/dhd_nos/*/")
