"""Build the cross-marker background from the user-supplied reference texture
(Images/Gemini_Generated_Image_ist2vxist2vxist2.png), which measured far
better structurally than the original marker background or the first
from-scratch grid attempt:
  - autocorrelation feature diameter ~16px (vs the original marker's ~46px
    -- much less periodic, less prone to LK matching the wrong repeat)
  - GFT corner density ~69 per 100x100px (very rich)

BUT its native contrast is incompatible with the color-gate detector: 42.3%
of its pixels are near-black (V<20), which would swamp the V<20 gate used
to find the marker's own cross+stub (see the FIRST retexture attempt,
2026-08-02, which broke detection to 1% ok-rate with only 3.5% of the frame
crossing that threshold -- this reference image is >10x worse on that
metric if used with its native contrast). So: keep the image's SHAPE, remap
its tone range into the safe light-gray band the original working texture
proved out (background never below V=140) instead of its native black/white
contrast, then composite the marker's own cross+stub back on top pixel-
identical, exactly as the first retexture attempt did.
"""
import os
import numpy as np
import cv2

SRC = '/home/shubham/Soft-Precise-Landing/PX4_Gazebo/Images/Gemini_Generated_Image_ist2vxist2vxist2.png'
MARKER = '/home/shubham/PX4-Autopilot/Tools/simulation/gz/models/cross_marker/cross_marker.png.bak_before_finetex_20260802'
OUT = os.environ.get(
    "CROSS_TEXTURE_OUT",
    '/home/shubham/PX4-Autopilot/Tools/simulation/gz/models/cross_marker/cross_marker_reftex.png')

CROSS_THRESH = int(os.environ.get("CROSS_THRESH", "60"))
# Safe remap range: the ORIGINAL working marker background measured min=140,
# p5=177, mean=194, max=242 -- pick a similarly safe band, comfortably clear
# of the V<20 gate even after Gazebo's lighting attenuates the albedo.
REMAP_LO = int(os.environ.get("REMAP_LO", "160"))
REMAP_HI = int(os.environ.get("REMAP_HI", "235"))
TEX_SIZE = int(os.environ.get("TEX_SIZE", "1024"))


def main():
    marker = cv2.imread(MARKER, cv2.IMREAD_GRAYSCALE)
    h, w = marker.shape
    cross_mask = marker < CROSS_THRESH
    print(f"cross+stub coverage: {cross_mask.mean()*100:.1f}% of plate")

    ref = cv2.imread(SRC, cv2.IMREAD_GRAYSCALE)
    # center-crop to square, then resize to the marker texture's resolution
    rh, rw = ref.shape
    side = min(rh, rw)
    y0 = (rh - side) // 2; x0 = (rw - side) // 2
    ref_sq = ref[y0:y0+side, x0:x0+side]
    ref_rs = cv2.resize(ref_sq, (TEX_SIZE, TEX_SIZE), interpolation=cv2.INTER_AREA)
    if (h, w) != (TEX_SIZE, TEX_SIZE):
        ref_rs = cv2.resize(ref_rs, (w, h), interpolation=cv2.INTER_AREA)

    # remap tone: preserve the reference's SHAPE (relative dark/light pattern)
    # but compress into the safe light-gray band, min->REMAP_LO, max->REMAP_HI
    rmin, rmax = ref_rs.min(), ref_rs.max()
    remapped = REMAP_LO + (ref_rs.astype(np.float64) - rmin) * (REMAP_HI - REMAP_LO) / max(rmax - rmin, 1)
    remapped = remapped.astype(np.uint8)

    out = remapped.copy()
    out[cross_mask] = marker[cross_mask]   # restore the EXACT original cross/stub pixels

    cv2.imwrite(OUT, out)
    print(f"remapped background range: [{out[~cross_mask].min()}, {out[~cross_mask].max()}]")
    print(f"-> {OUT}")


if __name__ == '__main__':
    main()
