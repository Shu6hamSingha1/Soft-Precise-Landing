#!/usr/bin/env python3
"""Kabsch/Wahba-based check for a camera-to-body mounting rotation, separate
from (and more robust to noise than) the general unconstrained 6x6
sensor-cal fit in derive_pi_cal.py. An unconstrained lstsq fit can't
distinguish a real mounting rotation from residual scale/gain
miscalibration - it can "cheat" by absorbing noise into spurious scale
terms. This instead solves for the BEST-FIT ORTHOGONAL rotation on paired
GT/raw flow DIRECTION vectors (magnitude-normalized, so scale can't leak
in), which is forced to stay a valid rotation.

img_data.py currently ASSUMES no mount rotation (comment near _vframe_w:
"The camera is body-FRD (no mount rotation)"). This checks that assumption
against real GT data instead of taking it on faith.

*** PRELIMINARY: same limited/noisy dataset caveats as derive_pi_cal.py
(single usable run so far, WEAK on X-span, low R^2 on the general fit)
apply here too - this is a first look at whether there's an obvious
mounting rotation to be aware of, not a final validated answer. ***

Usage: python3 check_camera_rotation.py <run_dir>
"""
import sys
import numpy as np
from derive_pi_cal import compute_gt_flow, kf_filter_causal, reject_outliers, FLOW_KF_Q, FLOW_KF_R


def kabsch(A, B):
    """Best-fit rotation R (3x3, proper - det=+1) minimizing sum|B_i - R@A_i|^2
    for paired unit vectors A_i, B_i (N,3) - standard Kabsch/Wahba solution
    via SVD of the cross-covariance H = A.T @ B."""
    H = A.T @ B
    U, S, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    D = np.diag([1.0, 1.0, d])
    return Vt.T @ D @ U.T


def main(run_dir):
    img = np.load(f"{run_dir}/Img_Data.npy", allow_pickle=True).item()
    g = compute_gt_flow(run_dir)

    t_img_abs = np.asarray(img["Time"], float)
    raw_flow = np.asarray(img["Opt Flow Ang Vel"], float)
    n = min(len(t_img_abs), len(raw_flow))
    t_img_abs, raw_flow = t_img_abs[:n], raw_flow[:n]
    if n > 1:
        raw_flow = reject_outliers(raw_flow, label="flow")
        raw_flow = kf_filter_causal(raw_flow, t_img_abs, FLOW_KF_Q, FLOW_KF_R)
    raw_flow_g = g["align"](t_img_abs, raw_flow)

    GT = g["V_h_g"]           # (N,3) GT translational flow, V-frame
    RAW = raw_flow_g[:, 0:3]  # (N,3) raw translational flow, camera/image frame

    m = np.all(np.isfinite(GT), 1) & np.all(np.isfinite(RAW), 1)
    GT, RAW = GT[m], RAW[m]
    if len(GT) < 50:
        print(f"Too few finite paired samples: {len(GT)}")
        return

    # Direction-only: normalize each row, keep only the more-excited half
    # (direction is undefined/noise-dominated when true motion is tiny).
    gt_mag = np.linalg.norm(GT, axis=1)
    raw_mag = np.linalg.norm(RAW, axis=1)
    keep = (gt_mag > max(np.percentile(gt_mag, 50), 1e-6)) & \
           (raw_mag > max(np.percentile(raw_mag, 50), 1e-6))
    GTn = GT[keep] / gt_mag[keep, None]
    RAWn = RAW[keep] / raw_mag[keep, None]
    print(f"n samples used (top-50% magnitude): {keep.sum()} / {len(GT)}")

    R = kabsch(RAWn, GTn)   # GT_direction ~= R @ raw_direction
    np.set_printoptions(precision=4, suppress=True)
    print("\nBest-fit rotation R (GT_direction ~= R @ raw_direction):")
    print(R)

    ang = np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1)))
    print(f"\nRotation angle from identity: {ang:.1f} deg")
    if ang < 10:
        print("Close to identity - consistent with img_data.py's assumed 'no mount rotation'.")
    else:
        print("NOT close to identity - suggests a real camera mounting rotation")
        print("relative to body FRD, beyond what img_data.py currently assumes.")

    pred = RAWn @ R.T
    resid = np.linalg.norm(pred - GTn, axis=1)
    print(f"\nmean direction residual: {resid.mean():.3f}  "
          f"(0=perfect, sqrt(2)~1.41=orthogonal directions, 2=opposite)")
    print("(High residual here means the direction data itself is too noisy for")
    print(" this check to be conclusive yet - consistent with the low R^2 already")
    print(" seen on the general 6x6 fit for this same dataset.)")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: check_camera_rotation.py <run_dir>")
        sys.exit(1)
    main(sys.argv[1])
