#!/usr/bin/env python3
"""Calibrate/validate the optical-flow OUTPUT against Gazebo GT, all the way to touchdown.

Two checks (per the plan FUNNEL_CBF_DESIGN / quirky-baking-perlis):
  (i)  the texture-free RING flow V_v_ring (calibrated) tracks GT V-frame flow into the final
       0.5m (R^2, slope) -> the L+/V-frame computation is bug-free to touchdown.
  (ii) marker->ring SWITCH continuity: corner flow V_v vs ring flow V_v_ring agreement over the
       overlap + the handoff discontinuity Delta.

Reuses compute_gt_signals (tools/aggregate_calibration_phased.py) for the GT V-frame flow.
Read-only.  Usage: calibrate_flow_to_touchdown.py <landing_dir>
"""
import numpy as np, os, sys, warnings; warnings.filterwarnings('ignore')
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from aggregate_calibration_phased import compute_gt_signals

# runtime flow calibration (img_data.py:_sensor_cal_hw, multisine-derived 2026-06-02)
CAL = np.array([
    [+0.7126,-0.0190,+0.0115,-0.0268,+0.4315,+0.0042],
    [-0.0291,+0.6817,+0.0072,-0.3727,+0.0732,-0.0099],
    [+0.0005,+0.0010,+0.8583,-0.0545,-0.0106,+0.0023],
    [+0.0050,-0.7570,-0.0153,+0.6147,-0.0552,-0.0101],
    [+0.7930,+0.0092,+0.0106,-0.0370,+0.6207,+0.0014],
    [+0.0052,+0.0364,+0.0238,+0.0171,-0.1310,+0.6088]])
LBL = ['h_x','h_y','h_z(div)','w_x','w_y','w_z']

def r2_slope(meas, gt):
    m = np.isfinite(meas) & np.isfinite(gt)
    if m.sum() < 8: return (np.nan, np.nan)
    a = np.polyfit(gt[m], meas[m], 1)              # meas ~ a0*gt + a1
    pred = a[0]*gt[m] + a[1]
    ss = 1 - np.sum((meas[m]-pred)**2)/(np.sum((meas[m]-np.mean(meas[m]))**2)+1e-12)
    return (float(ss), float(a[0]))

def main():
    d = sys.argv[1]
    img = np.load(os.path.join(d,'Img_Data.npy'),allow_pickle=True).item()
    gt  = np.load(os.path.join(d,'Ground_Truth.npy'),allow_pickle=True).item()

    # --- GT V-frame flow (manuscript sign: w = -V_w_ug) ---
    t_g, V_h_g, V_w_ug, _,_,_, V_z = compute_gt_signals(gt)
    GT = np.hstack([V_h_g, -V_w_ug])               # (ng,6) depth-normalized
    ng = len(t_g)

    # --- measured ring + corner flow (raw -> calibrated) ---
    ti  = np.asarray(img['Time'],float)
    ring = np.asarray(img['Ring Opt Flow Ang Vel'],float)
    corn = np.asarray(img['Opt Flow Ang Vel'],float)
    ncr  = np.asarray(img['N Ring Corners'],float)
    ncc  = np.asarray(img['N Flow Corners'],float) if 'N Flow Corners' in img else np.zeros(len(ti))
    n = min(len(ti),len(ring),len(corn),len(ncr),len(ncc)); ti,ring,corn,ncr,ncc = ti[:n],ring[:n],corn[:n],ncr[:n],ncc[:n]
    ring_cal = (CAL @ ring.T).T
    corn_cal = (CAL @ corn.T).T

    # --- align by time-to-touchdown (both bundles end ~at touchdown; removes clock offset) ---
    ttd_i = ti[-1]-ti
    ttd_g = t_g[-1]-t_g
    GT_i = np.column_stack([np.interp(ttd_i[::-1], ttd_g[::-1], GT[::-1,k])[::-1] for k in range(6)])
    Vz_i = np.interp(ttd_i[::-1], ttd_g[::-1], V_z[::-1])[::-1]

    print(f"=== {os.path.basename(d.rstrip('/'))} ===  img {n}f, GT {ng}f, span {ti[-1]-ti[0]:.1f}s")
    print(f"alignment: by time-to-touchdown (img clock range {ti[-1]-ti[0]:.1f}s vs GT {t_g[-1]-t_g[0]:.1f}s)")

    # ===== (i) RING vs GT, binned by time-to-touchdown =====
    print("\n(i) RING flow V_v_ring (calibrated) vs GT V-frame flow  [R^2 | slope]")
    bins = [(3.0,1.5),(1.5,0.8),(0.8,0.4),(0.4,0.0)]
    print(f"{'t_to_td':>10} {'nfr':>4} " + " ".join(f"{l:>14}" for l in LBL))
    for hi,lo in bins:
        m = (ttd_i<=hi)&(ttd_i>lo)&(ncr>0)
        row = f"{hi:.1f}-{lo:.1f}s".rjust(10) + f" {int(m.sum()):>4} "
        for k in range(6):
            r2,sl = r2_slope(ring_cal[m,k], GT_i[m,k]); row += f"  {r2:5.2f}|{sl:5.2f} "
        print(row)
    print("  (R^2->1, slope->1 = ring flow tracks truth in that bin. Final bin = to touchdown.)")

    # ===== (ii) marker->ring SWITCH continuity =====
    print("\n(ii) SWITCH continuity: corner V_v vs ring V_v_ring")
    both = (ncc>0)&(ncr>0)                          # overlap: both fronts valid
    print(f"  overlap frames (both valid): {int(both.sum())}/{n}")
    if both.sum()>8:
        print(f"  {'comp':>10} {'R2(Vv,Vvr)':>11} {'slope':>6} {'mean|Vv-Vvr|':>13}")
        for k in range(6):
            r2,sl = r2_slope(ring_cal[both,k], corn_cal[both,k])
            off = float(np.nanmean(np.abs(ring_cal[both,k]-corn_cal[both,k])))
            print(f"  {LBL[k]:>10} {r2:11.2f} {sl:6.2f} {off:13.3f}")
    # handoff discontinuity: last frame corner valid -> ring continues
    cv = np.where(ncc>0)[0]
    if len(cv)>0 and cv[-1]+1 < n:
        j = cv[-1]
        dlt = ring_cal[j] - corn_cal[j]
        print(f"  handoff at frame {j} (t_to_td={ttd_i[j]:.2f}s, Z={Vz_i[j]:.2f}m): "
              f"Delta = Vv_ring - Vv = " + " ".join(f"{LBL[k]}:{dlt[k]:+.2f}" for k in range(6)))
    print("  small overlap |Vv-Vvr| + small handoff Delta = glitch-free marker->ring switch")

if __name__ == '__main__':
    main()
