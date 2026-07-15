#!/usr/bin/env python3
"""
Compute camera intrinsics (fx, fy, cx, cy) from checkerboard images saved by
capture_calib_images.py. Square physical size doesn't matter for fx/fy in
pixels (only affects translation/distance scale, which we don't need) - unit
squares are used.

Tries several common inner-corner board sizes automatically (in case the
exact size is unknown) and uses the more robust findChessboardCornersSB
detector (falls back to the legacy detector with adaptive-threshold flags +
CLAHE contrast boost, since raw sensor captures are dim).

Usage: python3 compute_camera_calib.py [image_dir] [cols x rows]
  e.g. python3 compute_camera_calib.py calib_images 9x6
"""
import sys
import glob
import numpy as np
import cv2

IMAGE_DIR = sys.argv[1] if len(sys.argv) > 1 else "calib_images"

if len(sys.argv) > 2:
    c, r = sys.argv[2].lower().split("x")
    CANDIDATE_SIZES = [(int(c), int(r))]
else:
    CANDIDATE_SIZES = [(9, 6), (8, 6), (7, 6), (6, 5), (7, 5), (8, 5),
                       (6, 9), (6, 8), (6, 7), (5, 6), (5, 7), (5, 8)]

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))

images = sorted(glob.glob(f"{IMAGE_DIR}/*.png"))
print(f"Found {len(images)} images in {IMAGE_DIR}/")
if not images:
    print("No images found - run capture_calib_images.py first.")
    sys.exit(1)


def _canonicalize_order(corners):
    # A plain checkerboard has no unique orientation marker, so a ~180-degree
    # rotation of the board between shots yields a geometrically valid but
    # REVERSED corner sequence - this silently corrupts the joint calibration
    # (each image implicitly assumes a different objp<->corner mapping).
    # Canonicalize: corner[0] should be nearer the image origin (top-left)
    # than corner[-1] (bottom-right), by x+y. If not, the sequence is flipped.
    first_sum = corners[0, 0, 0] + corners[0, 0, 1]
    last_sum = corners[-1, 0, 0] + corners[-1, 0, 1]
    if first_sum > last_sum:
        corners = corners[::-1].copy()
    return corners


def find_board(gray, board_size):
    # Try the modern robust detector first.
    found, corners = cv2.findChessboardCornersSB(gray, board_size, cv2.CALIB_CB_EXHAUSTIVE)
    if found:
        # SB detector returns (N,2) CV_32FC1; normalize to (N,1,2) CV_32FC2 to
        # match the legacy detector's shape (calibrateCamera/projectPoints/norm
        # all assume this shape).
        corners = corners.reshape(-1, 1, 2).astype('float32')
        return True, _canonicalize_order(corners)
    # Fall back to legacy detector with adaptive threshold + normalization.
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FILTER_QUADS
    found, corners = cv2.findChessboardCorners(gray, board_size, flags)
    if found:
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        corners = _canonicalize_order(corners)
    return found, corners


# Probe: try each candidate size against a handful of images to find the one that matches.
probe_imgs = images[:min(5, len(images))]
best_size = None
for size in CANDIDATE_SIZES:
    hits = 0
    for fname in probe_imgs:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray_eq = clahe.apply(gray)
        found, _ = find_board(gray_eq, size)
        if found:
            hits += 1
    print(f"  board size {size[0]}x{size[1]}: {hits}/{len(probe_imgs)} probe images detected")
    if hits == len(probe_imgs) and hits > 0:
        best_size = size
        break
    if best_size is None and hits > 0:
        best_size = size  # partial match, keep as fallback candidate

if best_size is None:
    print("\nNo candidate board size detected the checkerboard in ANY probe image.")
    print("Possible causes: wrong corner count, board out of frame/too small/blurry,")
    print("or insufficient contrast. Re-capture with the board filling more of the")
    print("frame and better lighting, or pass the exact size: compute_camera_calib.py DIR COLSxROWS")
    sys.exit(1)

print(f"\nUsing board size {best_size[0]}x{best_size[1]}")
BOARD_SIZE = best_size

objp = np.zeros((BOARD_SIZE[0] * BOARD_SIZE[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:BOARD_SIZE[0], 0:BOARD_SIZE[1]].T.reshape(-1, 2)

objpoints = []
imgpoints = []
kept_files = []
gray_shape = None

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray_shape = gray.shape[::-1]
    gray_eq = clahe.apply(gray)
    found, corners = find_board(gray_eq, BOARD_SIZE)
    if found:
        objpoints.append(objp)
        imgpoints.append(corners)
        kept_files.append(fname)
        print(f"  OK    {fname}")
    else:
        print(f"  SKIP  {fname} (no board found)")

if len(objpoints) < 9:
    print(f"\nOnly {len(objpoints)} usable images - need at least 9. Capture more.")
    sys.exit(1)

# Constrained fit: 5 free distortion params + free principal point is too many
# degrees of freedom for a dataset with limited viewpoint coverage -> classic
# overfit symptom is huge |distortion| and cx/cy landing outside the image.
# Fix k3 (rarely needed for a simple lens), zero tangential distortion (only
# matters for meaningfully decentered/tilted lens elements), and constrain fx=fy
# (square pixels - true for the IMX219) as a physically-motivated prior.
calib_flags = (cv2.CALIB_ZERO_TANGENT_DIST | cv2.CALIB_FIX_K3
               | cv2.CALIB_FIX_ASPECT_RATIO | cv2.CALIB_FIX_PRINCIPAL_POINT)
init_mtx = np.array([[500.0, 0, gray_shape[0] / 2],
                      [0, 500.0, gray_shape[1] / 2],
                      [0, 0, 1]])

# Iterative outlier rejection: a few bad individual detections (motion blur,
# dim-image corner noise) can dominate and corrupt an otherwise-good fit.
# Fit, compute PER-IMAGE reprojection error, drop the worst outliers
# (> 2x median), refit on the clean subset. Repeat until stable or too few
# images remain.
for iteration in range(4):
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, gray_shape, init_mtx, None, flags=calib_flags)

    per_image_err = []
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
        error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
        per_image_err.append(error)
    per_image_err = np.array(per_image_err)
    median_err = np.median(per_image_err)
    print(f"\n[iter {iteration}] {len(objpoints)} images, "
          f"median per-image error={median_err:.3f} px, max={per_image_err.max():.3f} px")

    outlier_mask = per_image_err > max(2.0 * median_err, 1.0)
    n_outliers = int(outlier_mask.sum())
    if n_outliers == 0 or (len(objpoints) - n_outliers) < 9:
        break
    dropped = [kept_files[i] for i in range(len(kept_files)) if outlier_mask[i]]
    print(f"  dropping {n_outliers} outlier(s): {dropped}")
    objpoints = [p for p, m in zip(objpoints, outlier_mask) if not m]
    imgpoints = [p for p, m in zip(imgpoints, outlier_mask) if not m]
    kept_files = [f for f, m in zip(kept_files, outlier_mask) if not m]

fx, fy = mtx[0, 0], mtx[1, 1]
cx, cy = mtx[0, 2], mtx[1, 2]

mean_error = 0
for i in range(len(objpoints)):
    imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
    error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
    mean_error += error
mean_error /= len(objpoints)

print(f"\nFinal: used {len(objpoints)}/{len(images)} images (after outlier rejection). "
      f"Reprojection error: {mean_error:.4f} px")
print(f"\nCamera matrix:\n{mtx}")
print(f"Distortion: {dist.ravel()}")
print(f"\nfx = {fx:.2f}")
print(f"fy = {fy:.2f}")
print(f"cx = {cx:.2f}  (image width/2 = {gray_shape[0]/2:.1f})")
print(f"cy = {cy:.2f}  (image height/2 = {gray_shape[1]/2:.1f})")

np.savetxt(f"{IMAGE_DIR}/cameraMatrix.txt", mtx, delimiter=",")
np.savetxt(f"{IMAGE_DIR}/cameraDistortion.txt", dist, delimiter=",")
print(f"\nSaved {IMAGE_DIR}/cameraMatrix.txt and cameraDistortion.txt")
