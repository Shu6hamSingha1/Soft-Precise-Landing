"""One-off local analysis: compare GT (QTM) yaw vs FC telemetry (onboard EKF)
yaw across the output-calibration recordings, to characterize how much yaw
estimation error the FC itself has - distinct from the offline calibration-
derivation bugs already fixed. This matters because img_data.py's runtime
_getVirtualPts/_vframe_w use the LIVE FC quaternion (not GT) to de-rotate the
image into the virtual frame - if the FC's own yaw estimate is off, that
error is baked into the RUNTIME flow computation itself, not just into the
offline calibration fit.

Stubs `qtm` in sys.modules before importing mocaptools, since unpickling a
recorded Ground_Truth.npy's Pose/PoseData objects requires `mocaptools` to be
importable (pickle resolves classes by module path) even though it never
touches a live QTM connection. Not a repo change - ad hoc, local only.
"""
import sys
import types
import glob
import os
import numpy as np

sys.modules['qtm'] = types.ModuleType('qtm')  # stub: only Pose/PoseData needed, not QTMWrapper

RUNS = sorted(glob.glob('../Test_Data/Calibration/Output/*'))
RUNS = [r for r in RUNS if os.path.isdir(r) and os.path.exists(os.path.join(r, 'Ground_Truth.npy'))]


def quat_to_yaw_frd(q):
    """FRD/NED yaw from [w,x,y,z]."""
    w, x, y, z = q
    return np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


for run_dir in RUNS:
    name = os.path.basename(run_dir)
    gt = np.load(os.path.join(run_dir, 'Ground_Truth.npy'), allow_pickle=True).item()
    tel_path = os.path.join(run_dir, 'Telemetry_Data.npy')
    if not os.path.exists(tel_path):
        print(f'{name}: no Telemetry_Data.npy, skip')
        continue
    tel = np.load(tel_path, allow_pickle=True).item()

    St = float(gt['Start Time'])
    t_gt = np.asarray(gt['Time'], float) + St     # absolute perf_counter
    up = gt['UAV Pose']
    n = min(len(t_gt), len(up))
    t_gt, up = t_gt[:n], up[:n]
    valid = [p is not None and np.isfinite(p.yaw) for p in up]
    t_gt = t_gt[valid]
    yaw_gt_qtm = np.array([up[i].yaw for i in range(n) if valid[i]])   # QTM FLU, degrees
    yaw_gt_frd = np.deg2rad(-yaw_gt_qtm)   # QTM -> FRD sign flip (confirmed frame transform)
    yaw_gt_frd = np.unwrap(yaw_gt_frd)

    t_odo = np.asarray(tel['Odometry Timestamp'], float)
    quats = tel['Quaternion']
    n2 = min(len(t_odo), len(quats))
    t_odo, quats = t_odo[:n2], quats[:n2]
    valid2 = [q is not None for q in quats]
    t_odo = t_odo[valid2]
    yaw_fc = np.array([quat_to_yaw_frd([quats[i].w, quats[i].x, quats[i].y, quats[i].z])
                        for i in range(n2) if valid2[i]])
    yaw_fc = np.unwrap(yaw_fc)

    if len(t_gt) < 10 or len(t_odo) < 10:
        print(f'{name}: too few valid samples (gt={len(t_gt)}, fc={len(t_odo)}), skip')
        continue

    # Resample FC yaw onto GT's time axis (same process clock, perf_counter -
    # both share the same origin via the shared start_time variable).
    yaw_fc_on_gt = np.interp(t_gt, t_odo, yaw_fc, left=np.nan, right=np.nan)
    err = np.arctan2(np.sin(yaw_gt_frd - yaw_fc_on_gt), np.cos(yaw_gt_frd - yaw_fc_on_gt))
    m = np.isfinite(err)
    if m.sum() < 10:
        print(f'{name}: no time overlap between GT and FC telemetry, skip')
        continue
    err_deg = np.rad2deg(err[m])
    print(f'{name}: n={m.sum()}  mean_err={np.mean(err_deg):+.2f} deg  '
          f'RMS_err={np.sqrt(np.mean(err_deg**2)):.2f} deg  '
          f'max_abs={np.max(np.abs(err_deg)):.2f} deg  std={np.std(err_deg):.2f} deg')
