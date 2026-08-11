"""Thin the cross-marker's drawn cross+stub stroke to roughly half its current
width, via morphological erosion, leaving everything else in the texture
(background, plate size, marker layout) unchanged.

Measured (2026-08-05): current stroke width ~24px in the 1024x1024 texture
(distance-transform half-width median 12px * 2). Halving to ~12px needs an
erosion that removes ~6px from each side of the stroke -> kernel radius 6.

Usage:
    python3 tools/halve_cross_marker_linewidth.py
"""
import os
import numpy as np
import cv2

SRC = '/home/shubham/PX4-Autopilot/Tools/simulation/gz/models/cross_marker/cross_marker.png'
OUT = os.environ.get("CROSS_TEXTURE_OUT", SRC)

CROSS_THRESH = int(os.environ.get("CROSS_THRESH", "60"))   # V < this = cross/stub pixel
ERODE_RADIUS = int(os.environ.get("ERODE_RADIUS", "6"))     # px removed from each side (halves ~24px stroke)


def main():
    img = cv2.imread(SRC, cv2.IMREAD_COLOR)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    cross_mask = (gray < CROSS_THRESH).astype(np.uint8) * 255

    dist_before = cv2.distanceTransform(cross_mask, cv2.DIST_L2, 5)
    nz_before = dist_before[dist_before > 0]
    print(f"before: stroke coverage={np.mean(cross_mask>0)*100:.2f}%, "
          f"implied width={2*np.median(nz_before):.1f}px")

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2*ERODE_RADIUS+1, 2*ERODE_RADIUS+1))
    thinned_mask = cv2.erode(cross_mask, kernel)

    dist_after = cv2.distanceTransform(thinned_mask, cv2.DIST_L2, 5)
    nz_after = dist_after[dist_after > 0]
    print(f"after:  stroke coverage={np.mean(thinned_mask>0)*100:.2f}%, "
          f"implied width={2*np.median(nz_after):.1f}px")

    # Composite: restore original background everywhere the (now-eroded-away)
    # cross pixels no longer belong; keep the marker's own dark pixel colors
    # (not a flat fill) where the thinned mask still covers.
    out = img.copy()
    removed = (cross_mask > 0) & (thinned_mask == 0)
    # Background fill for removed stroke pixels: sample the median of nearby
    # background (non-cross) pixels, flat-fill (this base texture has a
    # uniform background; the fine-hatch variants are separate files).
    bg_level = int(np.median(gray[cross_mask == 0]))
    out[removed] = bg_level

    cv2.imwrite(OUT, out)
    print(f"-> {OUT}")


if __name__ == '__main__':
    main()
