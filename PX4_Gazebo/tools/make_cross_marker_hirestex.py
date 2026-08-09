"""Regenerate the cross-marker's texture at a HIGHER native resolution, to
push the close-range texture-resolution crossover altitude down (see
project_cross_marker_pipeline_20260801 memory, 2026-08-09 entries, and the
io-calibration skill's Hz/Wz section for the underlying mechanism: the
camera resolves finer than one texel below Z ~= (plane_m/tex_px)*fx, and
GPU texture filtering visibly blurs the fine speckle pattern GFT/LK need
for corner gradients once that happens).

Does NOT upscale the existing 1024x1024 cross_marker.png pixel-for-pixel
(that would just make existing blur blockier, not add real detail).
Instead: extracts the cross+stub shape mask (pixel-exact, thresholded same
as make_cross_marker_finetexture.py), upscales ONLY that mask (nearest-
neighbor -- keeps the drawn lines crisp, no antialiasing softening), and
generates FRESH independent speckle noise at the new resolution with the
same statistics as the live texture (measured: background mean~194, std~10)
-- genuinely finer noise grain, not just more pixels of the same coarse
grain.

Usage: python3 tools/make_cross_marker_hirestex.py
  (writes cross_marker_hires.png next to the source; does NOT overwrite
  cross_marker.png or touch model.sdf -- swap those in separately once the
  output is inspected)
"""
import os
import numpy as np
import cv2

SRC = '/home/shubham/PX4-Autopilot/Tools/simulation/gz/models/cross_marker/cross_marker.png'
OUT = os.environ.get(
    "CROSS_TEXTURE_OUT",
    '/home/shubham/PX4-Autopilot/Tools/simulation/gz/models/cross_marker/cross_marker_hires.png')

SCALE = int(os.environ.get("CROSS_TEX_SCALE", "3"))       # 1024 -> 3072 (3x) by default
CROSS_THRESH = int(os.environ.get("CROSS_THRESH", "60"))   # V < this = cross/stub pixel
SEED = int(os.environ.get("SEED", "20260809"))


def main():
    orig = cv2.imread(SRC, cv2.IMREAD_GRAYSCALE)
    h, w = orig.shape
    new_h, new_w = h * SCALE, w * SCALE

    cross_mask = orig < CROSS_THRESH
    print(f"source {w}x{h}, cross+stub coverage: {cross_mask.mean()*100:.2f}%")

    # Upscale the cross+stub mask with NEAREST (no antialiasing blur on the
    # lines themselves -- we want a crisp, larger version of the same shape).
    mask_u8 = (cross_mask * 255).astype(np.uint8)
    mask_hires = cv2.resize(mask_u8, (new_w, new_h), interpolation=cv2.INTER_NEAREST) > 127

    # Background statistics measured directly from the live texture's own
    # non-cross pixels, so the new background matches its look (mean~194,
    # std~10 speckle noise), just at genuinely finer native grain.
    bg_pixels = orig[~cross_mask]
    bg_mean, bg_std = float(bg_pixels.mean()), float(bg_pixels.std())
    print(f"background stats measured from source: mean={bg_mean:.1f} std={bg_std:.1f}")

    rng = np.random.default_rng(SEED)
    noise = rng.normal(bg_mean, bg_std, size=(new_h, new_w))
    out = np.clip(noise, 0, 255).astype(np.uint8)

    # Restore the cross+stub as solid near-black (matching the source's own
    # cross pixel values, not just a flat threshold value) -- sample the
    # source's actual cross-pixel value distribution rather than assuming 10.
    cross_val = int(np.median(orig[cross_mask]))
    out[mask_hires] = cross_val

    cv2.imwrite(OUT, out)
    texel_cm = 300.0 / new_w   # 3.0m plate, cm/texel
    fx = 270.0
    crossover_z = (texel_cm / 100.0) * fx
    print(f"-> {OUT}  ({new_w}x{new_h}, {texel_cm:.4f}cm/texel, "
          f"predicted crossover Z ~= {crossover_z:.3f}m, was ~0.79m at 1024px)")


if __name__ == '__main__':
    main()
