#!/usr/bin/env python3
"""Recompute raw optical flow (and feature params) from ALREADY-LOGGED raw
data (pixel corner points, onboard attitude quaternion, gyro, fps) using
img_geometry.py's CURRENT math (the shared source of truth img_data.py's
runtime also uses) - specifically the camera-to-FC mount-rotation fix
(R_CAM_TO_BODY, 2026-07-11) that landed in get_virtual_pts AFTER every
existing Test_Data/Calibration recording was already captured. Imports
img_geometry, NOT img_data, so this runs anywhere numpy/scipy/ahrs are
installed - no picamera2/qtm dependency (see img_geometry.py's own docstring).

Why this works WITHOUT a new recording: the mount-rotation fix only changes
a STATELESS function of (pixel corners, attitude quaternion) -> V-frame
points -> flow (_getVirtualPts -> _scaled_quad_points -> _fill_A -> gyro-comp
-> lstsq, plus _getImgFeatures for the centroid/alpha feature vector). None
of that touches live camera/mocap hardware state - it's re-run here against
Img_Data.npy's own logged "Image Feature Pts" / "Quaternion" /
"Angular Velocity" / "FPS" fields to reconstruct what the flow WOULD have
been with the fix applied.

CAVEAT: img_data.py only logs the CURRENT frame's quaternion per sample
(self._quat_log = quats[1]), not the paired PREVIOUS frame's quaternion
(quats[0]) that _getVirtualPts also needs for the t0 side of each pairing.
Reconstructed here as the PREVIOUS LOGGED SAMPLE's current-quat - valid for
consecutive (non-gapped) samples (~34ms apart, same cadence as the live
pairing), which is all that survives derive_pi_cal.py's gap guard anyway.
The very first sample of a run has no predecessor and is dropped (NaN).

Also mirrors output_calibration.py's recording environment: FLOW_DH_MAX,
FLOW_DS_MAX and FLOW_LOOM_DECOUPLE are forced OFF during calibration
recording (see output_calibration.py's module-level os.environ.setdefault
calls), so the ORIGINAL captures never ran through those checkposts/the loom
override either - this recompute correctly omits them too, rather than
introducing behavior the original recording never had.

Usage: python3 recompute_raw_flow.py <run_dir> [<run_dir> ...]
Writes Img_Data_Recomputed.npy alongside the original Img_Data.npy in each
run_dir - same keys as Img_Data.npy but with "Opt Flow Ang Vel" and
"Feature Params" replaced by the mount-rotation-corrected recomputation.
Everything else (Time, Quaternion, FPS, ...) is passed through unchanged.
derive_pi_cal.py prefers Img_Data_Recomputed.npy over Img_Data.npy when
present.
"""
import os
import sys
import numpy as np
import img_geometry as IG   # pure math, NO picamera2/qtm dependency (unlike img_data.py)

_CENTER = (IG.CALIB_CX, IG.CALIB_CY)


class _Quat:
    __slots__ = ("w", "x", "y", "z")


class _AngVel:
    __slots__ = ("forward_rad_s", "right_rad_s", "down_rad_s")


def _mkquat(arr):
    q = _Quat()
    q.w, q.x, q.y, q.z = float(arr[0]), float(arr[1]), float(arr[2]), float(arr[3])
    return q


def _mkangvel(arr):
    av = _AngVel()
    av.forward_rad_s, av.right_rad_s, av.down_rad_s = float(arr[0]), float(arr[1]), float(arr[2])
    return av


def recompute_one(run_dir):
    path = os.path.join(run_dir, "Img_Data.npy")
    img = np.load(path, allow_pickle=True).item()

    C_nP_log = img["Image Feature Pts"]                  # per-sample: [corners0(4,2), corners1(4,2)]
    quat_log = np.asarray(img["Quaternion"], float)       # per-sample: [w,x,y,z], CURRENT frame only
    av_log = np.asarray(img["Angular Velocity"], float)   # per-sample: [fwd,right,down], CURRENT frame
    fps_log = np.asarray(img["FPS"], float)
    n = len(C_nP_log)
    if not (len(quat_log) == n and len(av_log) == n and len(fps_log) == n):
        raise ValueError(f"length mismatch: corners={n} quat={len(quat_log)} "
                          f"av={len(av_log)} fps={len(fps_log)}")

    new_flow = np.full((n, 6), np.nan)
    new_feat = np.full((n, 4), np.nan)
    new_vfeat = [None] * n   # [old(4,2), new(4,2)] V-frame corners per sample, mount-rotation-corrected

    for i in range(1, n):   # sample 0 has no predecessor quat - left NaN
        C_nP = [np.asarray(C_nP_log[i][0], np.float32).reshape(-1, 2),
                np.asarray(C_nP_log[i][1], np.float32).reshape(-1, 2)]
        q0 = _mkquat(quat_log[i - 1])   # previous LOGGED sample's current-quat
        q1 = _mkquat(quat_log[i])
        quats = [q0, q1]

        dense_px = [IG.scaled_quad_points(p) for p in C_nP]
        V_dense = [IG.get_virtual_pts(dp, q, center=_CENTER) for dp, q in zip(dense_px, quats)]

        A = IG.fill_A(V_dense[1])
        Y = np.reshape(V_dense[1] - V_dense[0], (-1,)) * fps_log[i]

        av1 = _mkangvel(av_log[i])
        wv = IG.vframe_w([av1.forward_rad_s, av1.right_rad_s, av1.down_rad_s], q1)
        wz = float(wv[2])
        Yc = Y - A[:, 5] * wz
        h = np.linalg.lstsq(A[:, 0:3], Yc, rcond=1e-3)[0]
        new_flow[i] = [h[0], h[1], h[2], 0.0, 0.0, wz]

        V_nP0 = IG.get_virtual_pts(C_nP[0], q0, center=_CENTER)
        V_nP1 = IG.get_virtual_pts(C_nP[1], q1, center=_CENTER)
        new_vfeat[i] = [V_nP0, V_nP1]
        new_feat[i] = IG.get_img_features(V_nP1)

    out = dict(img)
    out["Opt Flow Ang Vel"] = new_flow
    out["Feature Params"] = new_feat
    out["Virtual Feature Pts"] = new_vfeat   # [V_nP0(4,2), V_nP1(4,2)] per sample, mount-rotation-corrected
    out_path = os.path.join(run_dir, "Img_Data_Recomputed.npy")
    np.save(out_path, out)
    n_valid = int(np.all(np.isfinite(new_flow), axis=1).sum())
    print(f"  wrote {out_path}  ({n_valid}/{n} samples recomputed; sample 0 dropped)")
    return out_path


def main():
    if len(sys.argv) < 2:
        print("Usage: recompute_raw_flow.py <run_dir> [<run_dir> ...]")
        sys.exit(1)
    for run_dir in sys.argv[1:]:
        print(f"=== {run_dir} ===")
        try:
            recompute_one(run_dir)
        except Exception as e:
            print(f"  FAILED: {e}")


if __name__ == "__main__":
    main()
