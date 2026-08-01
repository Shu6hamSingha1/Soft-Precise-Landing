"""Phase-0 feature-count/resolution tradeoff for PlanarFeatureMap, validated
against the REAL marker scene - AND, using the same live capture (camera
pointed at the marker, hand-swept), a direct empirical check of the
FC<->camera mount rotation (R_CAM_TO_BODY in img_geometry.py).

Rotation check idea: R_CAM_TO_BODY currently assumes a PURE single-axis (yaw)
mount rotation - no roll/pitch offset (comment: "the board itself is mounted
rotated -90 deg about that shared z axis"). If that's right, then during a
SWEEP THAT INCLUDES YAW MOTION over the static marker, the raw image-plane
marker angle (alpha, computed straight from pixels via
img_geometry.marker_principal_angle - no assumed rotation applied) should
track the FC's own yaw 1:1 (up to sign + a fixed phase offset = the mount
angle itself). A clean linear fit with slope near +-1 confirms the pure-yaw-
mount assumption and gives the actual offset; a bad fit / inconsistent sign
would mean the current R_CAM_TO_BODY assumption needs revisiting. IMPORTANT:
sweep needs to include some YAW rotation of the camera/drone by hand over the
marker (not just translation) for this part to have signal.

Throwaway diagnostic - not wired into anything, discard after use.
"""
import os
# WIRED 2026-08-01 to match hardware_landing.py's now-validated real-flight
# defaults (FLIGHT_TEST_ANALYSIS_PROCEDURE.md catalog #12/#13) - this bench
# needs real decode success against real scene texture, so it should run
# under the same exposure/gain regime the drone actually flies with.
os.environ.setdefault("CAM_MANUAL_EXPOSURE", "1")
os.environ.setdefault("CAM_EXPOSURE_US", "20000")
os.environ.setdefault("CAM_AUTO_GAIN", "1")

import asyncio
import time
import numpy as np
import cv2
from imgstreamer import imgstream
from planar_map import PlanarFeatureMap
from img_geometry import marker_principal_angle
from flight_controller import FC

N_FRAMES = 200
CAPTURE_SECONDS = 18
HOLD_TIMEOUT_S = 60.0

_arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
_arucoParams = cv2.aruco.DetectorParameters()
_arucoParams.adaptiveThreshWinSizeMin = 15
_arucoParams.adaptiveThreshWinSizeMax = 15
_arucoParams.adaptiveThreshWinSizeStep = 10
_arucoParams.minMarkerPerimeterRate = 0.02
_arucoParams.maxMarkerPerimeterRate = 0.5
_arucoParams.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_NONE
_detector = cv2.aruco.ArucoDetector(_arucoDict, _arucoParams)


def decode(gray):
    corners, ids, _ = _detector.detectMarkers(gray)
    if ids is None or len(ids) == 0:
        return None
    best = max(range(len(ids)), key=lambda i: cv2.contourArea(corners[i].reshape(-1, 2).astype(np.float32)))
    return corners[best].reshape(-1, 2).astype(np.float64)


def run_config(frames, decodes, max_features, label):
    pm = None
    cpu_ms = []
    pos_err = []
    n_tracked_hist = []
    conf_hist = []
    for f, dec in zip(frames, decodes):
        if pm is None:
            if dec is None:
                continue
            pm = PlanarFeatureMap(max_features=max_features, refill_below=int(max_features * 0.2))
            pm.bootstrap(f, marker_px_corners=dec)
            continue
        t0 = time.perf_counter()
        pm.update(f)
        cpu_ms.append((time.perf_counter() - t0) * 1000.0)
        n_tracked_hist.append(pm.n_tracked)
        conf_hist.append(pm.map_confidence)

        pred = pm.get_marker_center()   # held-out prediction BEFORE seeing this frame's decode
        if dec is not None:
            if pred is not None:
                gt_center = dec.mean(axis=0)
                pos_err.append(float(np.linalg.norm(pred - gt_center)))
            pm.loop_closure_correct(dec)

    cpu_ms = np.array(cpu_ms) if cpu_ms else np.array([np.nan])
    pos_err = np.array(pos_err) if pos_err else np.array([np.nan])
    n_tracked_hist = np.array(n_tracked_hist) if n_tracked_hist else np.array([0])
    conf_hist = np.array(conf_hist) if conf_hist else np.array([0.0])
    print(f"{label}: cpu mean={np.nanmean(cpu_ms):.1f}ms p95={np.nanpercentile(cpu_ms,95):.1f}ms | "
          f"pos_err(px) mean={np.nanmean(pos_err):.2f} p95={np.nanpercentile(pos_err,95):.2f} "
          f"n={np.sum(~np.isnan(pos_err))} | n_tracked end={n_tracked_hist[-1] if len(n_tracked_hist) else 0} "
          f"min={n_tracked_hist.min() if len(n_tracked_hist) else 0} | map_conf end={conf_hist[-1] if len(conf_hist) else 0:.2f}")


def check_mount_rotation(frames, decodes, quats):
    """Raw image-plane marker alpha vs FC yaw, both unwrapped, over frames
    where ArUco decoded AND a quaternion was available. Fits alpha = a*yaw+b;
    |a| near 1 with a tight residual supports the pure-yaw-mount assumption."""
    alphas, yaws = [], []
    for dec, q in zip(decodes, quats):
        if dec is None or q is None:
            continue
        alphas.append(marker_principal_angle(dec))
        w, x, y, z = q.w, q.x, q.y, q.z
        yaws.append(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
    n = len(alphas)
    print(f"\n=== Mount-rotation check: {n} decoded+quat frames ===")
    if n < 15:
        print("  too few paired samples (need real yaw motion over the marker during the sweep) - skipping")
        return
    alphas = np.unwrap(np.array(alphas))
    yaws = np.unwrap(np.array(yaws))
    yaw_span = float(yaws.max() - yaws.min())
    print(f"  FC yaw span during capture: {np.degrees(yaw_span):.1f} deg "
          f"({'OK, enough yaw motion' if yaw_span > np.radians(15) else 'WARNING: little yaw motion, fit may be unreliable'})")
    A = np.column_stack([yaws, np.ones(n)])
    (slope, intercept), *_ = np.linalg.lstsq(A, alphas, rcond=None)
    pred = A @ [slope, intercept]
    resid = alphas - pred
    corr = float(np.corrcoef(alphas, yaws)[0, 1])
    print(f"  alpha = {slope:+.3f} * yaw + {np.degrees(intercept) % 360:.1f}deg   "
          f"corr={corr:.3f}   resid_std={np.degrees(np.std(resid)):.1f}deg")
    print(f"  (expect slope near +1 or -1 for a pure-yaw mount; current R_CAM_TO_BODY "
          f"implies a -90deg mount rotation - compare intercept, mod 90/180, against that)")


async def main():
    fc = FC()
    await fc.start()
    t0 = time.perf_counter()
    while not fc.has_quat() and (time.perf_counter() - t0) < 20:
        await asyncio.sleep(0.05)
    print(f"FC quat available: {fc.has_quat()}")

    strm = imgstream(resolution=(640, 480), capRate=30)
    await asyncio.sleep(2.0)

    print(f"Point the camera at the marker - waiting up to {HOLD_TIMEOUT_S:.0f}s for a decode "
          f"before starting the timed capture...")
    t0 = time.perf_counter()
    last_print = 0.0
    while (time.perf_counter() - t0) < HOLD_TIMEOUT_S:
        m = list(strm.getImages())[-1]
        if m is not None and decode(m) is not None:
            print(f"marker detected after {time.perf_counter()-t0:.1f}s - starting capture now")
            break
        if time.perf_counter() - last_print > 3.0:
            print(f"  ...still waiting ({time.perf_counter()-t0:.0f}s elapsed)")
            last_print = time.perf_counter()
        await asyncio.sleep(0.05)
    else:
        print("timed out waiting for a marker decode - starting capture anyway")

    print(f"Capturing for {CAPTURE_SECONDS}s - please sweep/move the camera over the marker "
          f"like a calibration run, INCLUDING SOME YAW ROTATION (for the mount-rotation check)...")
    frames, quats, last = [], [], None
    t0 = time.perf_counter()
    while len(frames) < N_FRAMES and (time.perf_counter() - t0) < CAPTURE_SECONDS:
        m = list(strm.getImages())[-1]
        if m is not None and (last is None or not np.array_equal(m, last)):
            frames.append(m.copy())
            quats.append(fc.getQuat() if fc.has_quat() else None)
            last = m
        await asyncio.sleep(0.02)
    strm.close()
    await fc.close()
    print(f"captured {len(frames)} distinct frames")

    decodes = [decode(f) for f in frames]
    n_decoded = sum(1 for d in decodes if d is not None)
    print(f"ArUco decoded on {n_decoded}/{len(frames)} frames ({100.0*n_decoded/max(len(frames),1):.0f}%)")

    if n_decoded < 10:
        print("Too few decodes to run a meaningful test - marker may not have been in view. Aborting.")
        return

    for mf in (300, 150, 100, 60):
        run_config(frames, decodes, mf, f"max_features={mf}")

    check_mount_rotation(frames, decodes, quats)


if __name__ == "__main__":
    asyncio.run(main())
