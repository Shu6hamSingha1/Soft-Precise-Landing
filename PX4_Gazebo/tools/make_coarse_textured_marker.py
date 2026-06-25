"""Add COARSE trackable texture to the nested ArUco marker for 640x480 LK.

The fine-stipple 0-small_10-big_textured.png failed at 640x480 (fx=270): its
dots were sub-pixel at altitude -> LK aliasing -> noisy flow (R^2 0.1-0.6 vs
board 0.55-0.90). Root cause = texture too fine for our resolution (it worked
at the legacy 1280x960/fx=540).

Fix: coarse features that resolve at 640x480. We overlay small black SQUARES
(LK tracks their corners well) on a coarse grid, MASKED to the marker's white
regions only, so:
  - features are big enough to resolve at altitude (sized below),
  - they are SPARSE (small area fraction) so white ArUco cells still read white
    -> the marker still decodes,
  - they sit on the target (white marker regions) -> target-rigid -> moving-
    target-safe.

Sizing: a 2 m marker at 5 m spans ~108 px in the image (texture 1182 px ->
image scale ~0.091). For a feature to be >=~5 px in the image it must be
>=~55 px in the texture. We use SQ_TEX px squares on GRID_TEX px spacing.
"""
import os
import numpy as np
import cv2

SRC = '/home/shubham/Soft-Precise-Landing/PX4_Gazebo/Images/0-small_10-big.png'
OUT = '/home/shubham/Soft-Precise-Landing/PX4_Gazebo/Images/0-small_10-big_coarse.png'

SQ_TEX   = int(os.environ.get("COARSE_SQ",   "55"))   # square side, texture px (~5px@5m)
GRID_TEX = int(os.environ.get("COARSE_GRID", "120"))  # grid spacing, texture px
DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)


def main():
    img = cv2.imread(SRC, cv2.IMREAD_GRAYSCALE)
    H, W = img.shape
    print(f"src {SRC}  {W}x{H}")
    white = (img > 200)                       # marker white regions
    out = img.copy()
    half = SQ_TEX // 2
    placed = 0
    # coarse grid of candidate square centres
    for cy in range(GRID_TEX // 2, H, GRID_TEX):
        for cx in range(GRID_TEX // 2, W, GRID_TEX):
            y0, y1 = cy - half, cy + half
            x0, x1 = cx - half, cx + half
            if y0 < 0 or x0 < 0 or y1 > H or x1 > W:
                continue
            # only place where the square sits ENTIRELY in a white region
            # (keeps it inside a single ArUco white cell -> decode-safe,
            #  and avoids straddling the black coded cells)
            if white[y0:y1, x0:x1].all():
                out[y0:y1, x0:x1] = 0         # black square -> 4 LK corners
                placed += 1
    cv2.imwrite(OUT, out)
    print(f"placed {placed} coarse squares (SQ={SQ_TEX} GRID={GRID_TEX}px) -> {OUT}")

    # verify both nested IDs still decode across altitude-equivalent scales
    det = cv2.aruco.ArucoDetector(DICT, cv2.aruco.DetectorParameters())
    print("\n  scale  texpx   decoded IDs")
    for sc in [1.0, 0.5, 0.25, 0.12, 0.06]:
        w = int(W * sc); h = int(H * sc)
        rs = cv2.resize(out, (w, h), interpolation=cv2.INTER_AREA)
        pad = 40
        cv = np.full((h + 2*pad, w + 2*pad), 255, np.uint8)
        cv[pad:pad+h, pad:pad+w] = rs
        _, ids, _ = det.detectMarkers(cv)
        print(f"  {sc:4.2f}  {w:5d}   {sorted(int(i) for i in ids.flatten()) if ids is not None else []}")


if __name__ == "__main__":
    main()
