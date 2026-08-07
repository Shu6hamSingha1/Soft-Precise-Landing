"""Replace the cross-marker's background texture with a fine, irregular
cross-hatch (gradient-rich) pattern -- keeps the EXISTING black cross+stub
geometry pixel-for-pixel (extracted from the current texture via threshold),
only the background around it changes.

Why cross-hatch, not stipple/blobs: tools/make_fineline_textured_board.py
(this project's ArUco ring-flow texture) already tried a sub-pixel stipple
pattern and it FAILED -- dots alias away with scale. Fine lines survive
better because a line still reads as a continuous gradient even when thin.
Adapted here for the cross marker's Hx/Hy weak-correlation investigation
(2026-08-02): the ORIGINAL cross_marker.png background measured via
autocorrelation at ~13.5cm/~46px correlation length -- comparable to or
larger than the 15x15 LK window at typical operating altitudes, an aperture/
correspondence-ambiguity risk (repeating blob-like regions look alike to
LK). This texture targets a genuinely FINER, IRREGULAR structure instead:
- a jittered, gap-broken cross-hatch grid (not a perfect period, so no
  exact repeat for LK to confuse a match with)
- fine base line spacing so several grid crossings fall inside one LK
  window even at ~5-7m altitude (unlike the single ~46px-period blobs)
- a coarser secondary jittered grid overlaid for large-scale disambiguation
  (avoids a single fine period being globally periodic across the whole
  plate)
"""
import os
import numpy as np
import cv2

SRC = '/home/shubham/PX4-Autopilot/Tools/simulation/gz/models/cross_marker/cross_marker.png'
OUT = os.environ.get(
    "CROSS_TEXTURE_OUT",
    '/home/shubham/PX4-Autopilot/Tools/simulation/gz/models/cross_marker/cross_marker_finetex.png')

CROSS_THRESH = int(os.environ.get("CROSS_THRESH", "60"))   # V < this = cross/stub pixel (keep as-is)
BG_LEVEL = int(os.environ.get("BG_LEVEL", "210"))           # base background gray level
LINE_DARK = int(os.environ.get("LINE_DARK", "90"))          # fine-grid line gray level
LINE_W = int(os.environ.get("LINE_W", "3"))                 # fine line width, texture px
SPACING = int(os.environ.get("SPACING", "20"))               # fine grid nominal spacing, texture px
JITTER = int(os.environ.get("JITTER", "5"))                  # +/- px random offset per grid line
GAP_PROB = float(os.environ.get("GAP_PROB", "0.35"))          # per-segment chance of a break (non-periodicity)
GAP_LEN = int(os.environ.get("GAP_LEN", "18"))                # break length, texture px
COARSE_SPACING = int(os.environ.get("COARSE_SPACING", "140")) # secondary coarse grid spacing
COARSE_DARK = int(os.environ.get("COARSE_DARK", "150"))       # secondary grid line gray level (subtle)
SEED = int(os.environ.get("SEED", "20260802"))


def make_hatch(h, w, spacing, jitter, line_w, dark, gap_prob, gap_len, rng):
    """One irregular cross-hatch layer: jittered line positions + random
    per-segment gaps, so it's NOT a perfectly periodic pattern (avoids the
    exact-repeat aperture-confusion case a clean grid would still have)."""
    canvas = np.zeros((h, w), np.uint8)   # 0 = no line here (compose later)
    hatch_mask = np.zeros((h, w), bool)

    for y0 in range(0, h + spacing, spacing):
        y = int(np.clip(y0 + rng.integers(-jitter, jitter + 1), 0, h - 1))
        row = np.ones(w, bool)
        x = 0
        while x < w:
            seg = int(rng.integers(spacing, spacing * 3))
            if rng.random() < gap_prob:
                gl = int(rng.integers(1, gap_len))
                row[x:min(x + gl, w)] = False
                x += gl
            else:
                x += seg
        hatch_mask[y:min(y + line_w, h), :] |= row

    for x0 in range(0, w + spacing, spacing):
        x = int(np.clip(x0 + rng.integers(-jitter, jitter + 1), 0, w - 1))
        col = np.ones(h, bool)
        y = 0
        while y < h:
            seg = int(rng.integers(spacing, spacing * 3))
            if rng.random() < gap_prob:
                gl = int(rng.integers(1, gap_len))
                col[y:min(y + gl, h)] = False
                y += gl
            else:
                y += seg
        hatch_mask[:, x:min(x + line_w, w)] |= col[:, None]

    canvas[hatch_mask] = dark
    return hatch_mask, canvas


def main():
    orig = cv2.imread(SRC, cv2.IMREAD_GRAYSCALE)
    h, w = orig.shape
    cross_mask = orig < CROSS_THRESH   # the drawn cross+stub -- keep pixel-identical
    print(f"cross+stub coverage: {cross_mask.mean()*100:.1f}% of plate")

    rng = np.random.default_rng(SEED)
    out = np.full((h, w), BG_LEVEL, np.uint8)

    fine_mask, _ = make_hatch(h, w, SPACING, JITTER, LINE_W, LINE_DARK, GAP_PROB, GAP_LEN, rng)
    coarse_mask, _ = make_hatch(h, w, COARSE_SPACING, JITTER * 2, LINE_W + 1, COARSE_DARK, GAP_PROB, GAP_LEN * 2, rng)

    out[coarse_mask] = COARSE_DARK
    out[fine_mask] = LINE_DARK          # fine grid takes priority where they overlap
    out[cross_mask] = orig[cross_mask]  # restore the EXACT original cross/stub pixels last

    cv2.imwrite(OUT, out)
    print(f"fine-grid coverage: {fine_mask.mean()*100:.1f}%  coarse-grid coverage: {coarse_mask.mean()*100:.1f}%")
    print(f"-> {OUT}")


if __name__ == '__main__':
    main()
