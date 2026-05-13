#!/usr/bin/env python3
"""
Validate every pose transformation used in analyze_calibration.py by
printing intermediate quantities and checking sign/magnitude expectations
against geometric ground truth from the calibration recording.

Checks (each labelled CHECK):
  1. UAV start pose: world position around (0,0,~spawn_alt), quaternion ≈ identity
  2. Target start pose: stationary ArUco marker on the ground at world Z≈0
  3. Frame conventions: Gazebo world = ENU, Gazebo body = FLU
  4. FLU_2_FRD = rotation about x by 180° (-Y, -Z flip)
  5. B_x_tu (target wrt UAV in FRD): x≈0, y≈0, z>0 (target below drone)
  6. Body velocity sign: if commanded x sinusoid, B_v_tu[:,0] should be sinusoidal
  7. Optical flow direction: when drone moves +x, target appears to move -x in
     image; the B_y_g x-component should be NEGATIVE of B_v_tu x-component
     divided by depth (since flow is target apparent motion, opposite of UAV)
  8. UAV angular velocity matches Gazebo's reported body-rate roughly

Run AFTER a calibration recording exists in
PX4_Gazebo/calibration_data/output/<timestamp>/
(defaults to the most recent; pass a specific dir as argv[1] if needed).
"""
import os
import sys
import numpy as np
from scipy.signal import savgol_filter as sgf
from ahrs import Quaternion, DCM


def banner(s):
    print()
    print("=" * 78)
    print(f"  {s}")
    print("=" * 78)


def _most_recent_run_dir():
    parent = "/home/shubham/Soft-Precise-Landing/PX4_Gazebo/calibration_data/output"
    cands = [d for d in os.listdir(parent) if os.path.isdir(os.path.join(parent, d))]
    if not cands:
        return None
    return os.path.join(parent, max(cands, key=lambda d: os.path.getmtime(os.path.join(parent, d))))


def main():
    data_dir = sys.argv[1] if len(sys.argv) > 1 else _most_recent_run_dir()
    if not data_dir or not os.path.isdir(data_dir):
        print(f"[err] data dir not found: {data_dir}")
        return 1

    gt = np.load(f"{data_dir}/Ground_Truth.npy", allow_pickle=True)[()]
    print(f"loaded GT from {data_dir}")

    t_g = np.array(gt["Time"])
    n = len(t_g)
    uav_poses = list(gt["UAV Pose"])
    target_poses = list(gt["Target Pose"])
    print(f"{n} samples over {t_g[-1] - t_g[0]:.2f}s\n")

    # ---- CHECK 1: UAV start pose ----
    banner("CHECK 1: UAV start pose")
    p0 = gt["Start Pose"]
    print(f"  start_pose.position = ({p0.position.x:.3f}, {p0.position.y:.3f}, {p0.position.z:.3f})  [Gazebo world]")
    print(f"  start_pose.orient   = ({p0.orientation.w:.3f}, {p0.orientation.x:.3f}, "
          f"{p0.orientation.y:.3f}, {p0.orientation.z:.3f})  [w x y z]")
    q0 = Quaternion([p0.orientation.w, p0.orientation.x, p0.orientation.y, p0.orientation.z])
    eul0 = np.rad2deg(q0.to_angles())
    print(f"  start_pose.euler    = (r={eul0[0]:+.1f}°, p={eul0[1]:+.1f}°, y={eul0[2]:+.1f}°)")
    print(f"  EXPECT: position ≈ (0, 0, ~0.2)   orient ≈ identity (rpy ≈ 0)")
    print(f"  RESULT: {'OK' if (abs(p0.position.x) < 0.5 and abs(p0.position.y) < 0.5 and abs(eul0[1]) < 5 and abs(eul0[0]) < 5) else 'FAIL'}")

    # ---- CHECK 2: Target start pose ----
    banner("CHECK 2: Target (ArUco) pose — should be stationary on ground")
    t0 = target_poses[0]
    tN = target_poses[-1]
    print(f"  target[0].position  = ({t0.position.x:.3f}, {t0.position.y:.3f}, {t0.position.z:.3f})")
    print(f"  target[-1].position = ({tN.position.x:.3f}, {tN.position.y:.3f}, {tN.position.z:.3f})")
    tx = np.array([t.position.x for t in target_poses])
    ty = np.array([t.position.y for t in target_poses])
    tz = np.array([t.position.z for t in target_poses])
    print(f"  target position range over run: x±{tx.max()-tx.min():.3f}, "
          f"y±{ty.max()-ty.min():.3f}, z±{tz.max()-tz.min():.3f}")
    print(f"  EXPECT: ArUco is stationary (range ~ 0)")
    print(f"  RESULT: {'OK' if (tx.max()-tx.min() < 0.1 and ty.max()-ty.min() < 0.1) else 'FAIL'}")

    # ---- CHECK 3: World frame convention (gravity direction) ----
    banner("CHECK 3: World frame convention from UAV altitude during takeoff")
    uav_z = np.array([p.position.z for p in uav_poses])
    print(f"  UAV world Z: first={uav_z[0]:.2f}, max={uav_z.max():.2f}, min={uav_z.min():.2f}")
    print(f"  EXPECT (ENU world): Z grows upward when drone takes off")
    print(f"          (NED world): Z grows negative (down) when drone takes off")
    if uav_z.max() > uav_z[0] + 1.0:
        print(f"  RESULT: world looks like ENU (Z=up) — matches Gazebo default")
        gazebo_world_is_enu = True
    elif uav_z.min() < uav_z[0] - 1.0:
        print(f"  RESULT: world looks like NED (Z=down)")
        gazebo_world_is_enu = False
    else:
        print(f"  RESULT: indeterminate — drone may not have taken off cleanly")
        gazebo_world_is_enu = True

    # ---- CHECK 4: FLU_2_FRD matrix ----
    banner("CHECK 4: FLU_2_FRD = DCM(x=180°)")
    FLU_2_FRD = np.array(DCM(x=180.0))
    print(f"  FLU_2_FRD =")
    print(f"    {FLU_2_FRD}")
    print(f"  EXPECT: diag-like [+1, -1, -1] (X stays, Y and Z flip sign)")
    print(f"  test vector [1, 2, 3]_FLU -> [{FLU_2_FRD @ np.array([1,2,3])}]_FRD")
    print(f"  EXPECT: [1, -2, -3]")
    expected = np.array([1, -2, -3])
    print(f"  RESULT: {'OK' if np.allclose(FLU_2_FRD @ np.array([1,2,3]), expected, atol=1e-6) else 'FAIL'}")

    # Build all the transformations to use below.
    W_T_P0 = np.eye(4)
    W_T_P0[:3, 3] = [p0.position.x, p0.position.y, p0.position.z]
    W_T_P0[:3, :3] = q0.to_DCM()

    W_T_P = np.zeros((n, 4, 4))
    W_R_T = np.zeros((n, 3, 3))
    W_x_tu = np.zeros((n, 3))
    B_x_tu = np.zeros((n, 3))
    for i, (pu, pt) in enumerate(zip(uav_poses, target_poses)):
        Ru = Quaternion([pu.orientation.w, pu.orientation.x, pu.orientation.y, pu.orientation.z]).to_DCM()
        Rt = Quaternion([pt.orientation.w, pt.orientation.x, pt.orientation.y, pt.orientation.z]).to_DCM()
        W_T_P[i, :3, :3] = Ru
        W_T_P[i, :3, 3] = [pu.position.x, pu.position.y, pu.position.z]
        W_T_P[i, 3, 3] = 1.0
        W_R_T[i] = Rt
        W_x_t = np.array([pt.position.x, pt.position.y, pt.position.z])
        W_x_tu[i] = W_x_t - W_T_P[i, :3, 3]
        B_x_tu[i] = FLU_2_FRD @ np.linalg.inv(Ru) @ W_x_tu[i]

    # ---- CHECK 5: B_x_tu — target wrt UAV in body FRD ----
    banner("CHECK 5: B_x_tu (target wrt UAV, body FRD frame)")
    print(f"  B_x_tu[0]    = ({B_x_tu[0,0]:+.3f}, {B_x_tu[0,1]:+.3f}, {B_x_tu[0,2]:+.3f})  [first sample, on ground]")
    print(f"  B_x_tu[n/4]  = ({B_x_tu[n//4,0]:+.3f}, {B_x_tu[n//4,1]:+.3f}, {B_x_tu[n//4,2]:+.3f})  [mid-flight]")
    print(f"  B_x_tu[n/2]  = ({B_x_tu[n//2,0]:+.3f}, {B_x_tu[n//2,1]:+.3f}, {B_x_tu[n//2,2]:+.3f})  [mid-flight]")
    print(f"  range over run: x∈[{B_x_tu[:,0].min():+.2f}, {B_x_tu[:,0].max():+.2f}], "
          f"y∈[{B_x_tu[:,1].min():+.2f}, {B_x_tu[:,1].max():+.2f}], "
          f"z∈[{B_x_tu[:,2].min():+.2f}, {B_x_tu[:,2].max():+.2f}]")
    print(f"  EXPECT during flight: x,y small (sinusoidal swing), z > 0 and large (target BELOW drone)")
    mid_z = np.median(B_x_tu[n // 4 : 3 * n // 4, 2])
    print(f"  median z during mid-flight = {mid_z:+.2f}  (positive = target below drone in FRD-z)")
    print(f"  RESULT: {'OK' if mid_z > 1.0 else 'FAIL — z should be > 1m positive during flight'}")

    # ---- CHECK 6: Body velocity from gradient ----
    banner("CHECK 6: Body-frame target velocity B_v_tu from gradient of W_x_tu")
    W_x_tu_filt = sgf(W_x_tu, 51, 2, axis=0)
    W_v_tu = np.gradient(W_x_tu_filt, t_g, axis=0)
    B_v_tu = np.zeros((n, 3))
    for i in range(n):
        B_v_tu[i] = FLU_2_FRD @ np.linalg.inv(W_T_P[i, :3, :3]) @ W_v_tu[i]
    print(f"  W_v_tu RMS: ({np.sqrt(np.mean(W_v_tu[:,0]**2)):.3f}, "
          f"{np.sqrt(np.mean(W_v_tu[:,1]**2)):.3f}, {np.sqrt(np.mean(W_v_tu[:,2]**2)):.3f})  [world]")
    print(f"  B_v_tu RMS: ({np.sqrt(np.mean(B_v_tu[:,0]**2)):.3f}, "
          f"{np.sqrt(np.mean(B_v_tu[:,1]**2)):.3f}, {np.sqrt(np.mean(B_v_tu[:,2]**2)):.3f})  [body FRD]")
    print(f"  Commanded amplitude was 1.0 m at 15π/1000-sample frequency. With ~30Hz sampling that's")
    print(f"  freq ≈ {15*np.pi/(34.3):.3f} rad/s, expected peak v = 1.0 * freq = {15*np.pi/34.3:.2f} m/s.")
    print(f"  EXPECT: B_v_tu peak ≈ 1-2 m/s (commanded sweep was 1 m amplitude sinusoid)")
    peak_v = np.max(np.abs(B_v_tu), axis=0)
    print(f"  actual peak: ({peak_v[0]:.2f}, {peak_v[1]:.2f}, {peak_v[2]:.2f})  m/s")

    # ---- CHECK 7: Optical flow sign — opposite of UAV motion ----
    banner("CHECK 7: Optical flow B_y_g = B_v_tu / depth")
    B_y_g = B_v_tu / B_x_tu[:, 2][:, np.newaxis]
    print(f"  B_y_g RMS: ({np.sqrt(np.mean(B_y_g[:,0]**2)):.4f}, "
          f"{np.sqrt(np.mean(B_y_g[:,1]**2)):.4f}, {np.sqrt(np.mean(B_y_g[:,2]**2)):.4f})  [rad/s]")
    print(f"  EXPECT: rad/s scale (v_body / depth), typically 0.1-0.5 rad/s during flight")
    print(f"  Note: B_v_tu is *target* velocity wrt UAV, so when UAV moves +x, target moves -x in")
    print(f"        body frame. So sign of B_y_g should be OPPOSITE of UAV body velocity.")

    # ---- CHECK 8: Angular velocity in body FRD ----
    banner("CHECK 8: B_w_ug — UAV angular velocity in body FRD")
    W_dR_B = np.gradient(W_T_P[:, :3, :3], t_g, axis=0)
    B_w_ug = np.zeros((n, 3))
    for i in range(n):
        skew = W_T_P[i, :3, :3].T @ W_dR_B[i]
        B_w_ug[i] = FLU_2_FRD @ np.array([skew[2, 1], skew[0, 2], skew[1, 0]])
    print(f"  B_w_ug RMS: ({np.sqrt(np.mean(B_w_ug[:,0]**2)):.4f}, "
          f"{np.sqrt(np.mean(B_w_ug[:,1]**2)):.4f}, {np.sqrt(np.mean(B_w_ug[:,2]**2)):.4f})  [rad/s]")
    print(f"  EXPECT: moderate roll/pitch rates during oscillation (typically <1 rad/s)")
    print(f"          yaw rate from the 15° yaw command sweep")

    # ---- Summary ----
    banner("SUMMARY — sanity verdict for the calibration analyzer")
    all_ok = []
    all_ok.append(abs(p0.position.x) < 0.5 and abs(p0.position.y) < 0.5)
    all_ok.append(tx.max() - tx.min() < 0.1 and ty.max() - ty.min() < 0.1)
    all_ok.append(np.allclose(FLU_2_FRD @ np.array([1, 2, 3]), [1, -2, -3], atol=1e-6))
    all_ok.append(mid_z > 1.0)
    all_ok.append(np.max(np.abs(B_v_tu)) < 10.0)   # peak v reasonable
    all_ok.append(np.max(np.abs(B_y_g)) < 5.0)     # optical flow reasonable

    for i, ok in enumerate(all_ok, 1):
        print(f"  CHECK {i}: {'OK' if ok else 'FAIL'}")

    print()
    if all(all_ok):
        print("  All transformations look correct. Scaling factors should be trustworthy.")
    else:
        print("  At least one check failed — re-examine the calibration sweep before applying matrices.")
    return 0 if all(all_ok) else 2


if __name__ == "__main__":
    sys.exit(main())
