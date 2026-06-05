"""Add FINE bidirectional line texture to the ENTIRE aruco board for the ring-flow LK.

The ring (texture-free) optic flow needs gradients at EVERY ring station, but the board's
solid black/white cells are featureless. Add a fine cross-hatch: WHITE lines in the black
regions, BLACK lines in the white regions (invert the board colour at the line pixels).

Why FINE LINES (not the failed sub-pixel STIPPLE, 0-small_10-big_textured.png): lines are
continuous along their length, so they read as a gradient even when thin; and near touchdown
the board OVERFLOWS the frame -> the visible region is magnified -> the lines resolve well,
which is exactly where the ring flow is the safety net. At altitude the lines shrink/alias,
but there the CORNER flow (decoding the marker) covers it.

Decode-safe: lines are thin + sparse so each ArUco cell stays majority original colour.
Tunable: FINE_LINE_W (px), FINE_SPACING (px).  Output is a NEW file (original untouched).
"""
import os
import numpy as np
import cv2

BOARD = '/home/shubham/PX4-Autopilot/Tools/simulation/gz/models/arucotag/aruco_board.png'
OUT = '/home/shubham/PX4-Autopilot/Tools/simulation/gz/models/arucotag/aruco_board_fineline.png'
LINE_W  = int(os.environ.get("FINE_LINE_W", "3"))     # line width, texture px
SPACING = int(os.environ.get("FINE_SPACING", "28"))   # line spacing, texture px
DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)


def make(line_w, spacing):
    img = cv2.imread(BOARD, cv2.IMREAD_GRAYSCALE)
    H, W = img.shape
    mask = np.zeros((H, W), bool)
    for y in range(0, H, spacing):
        mask[y:min(y + line_w, H), :] = True      # horizontal lines
    for x in range(0, W, spacing):
        mask[:, x:min(x + line_w, W)] = True      # vertical lines
    out = img.copy()
    out[mask & (img < 96)] = 255                  # white line on black region
    out[mask & (img >= 96)] = 0                   # black line on white region
    return img, out, mask.mean() * 100.0


def main():
    orig, out, inv = make(LINE_W, SPACING)
    cv2.imwrite(OUT, out)
    H, W = orig.shape
    print(f"board {W}x{H}; cross-hatch W={LINE_W}px spacing={SPACING}px; inverted {inv:.0f}% of board")
    print(f"-> {OUT}\n")
    # DECODE-SAFETY: textured vs original at altitude-equivalent downscales
    # (board fills 640px frame at Z~1m -> scale 0.31; smaller scale = higher altitude)
    det = cv2.aruco.ArucoDetector(DICT, cv2.aruco.DetectorParameters())
    print("  approx_Z  scale  imgpx   decoded IDs (textured | original)")
    for z, sc in [(0.5, 0.62), (1.0, 0.31), (2.0, 0.16), (3.0, 0.10), (4.0, 0.08)]:
        to = cv2.resize(out, None, fx=sc, fy=sc, interpolation=cv2.INTER_AREA)
        oo = cv2.resize(orig, None, fx=sc, fy=sc, interpolation=cv2.INTER_AREA)
        ct, it, _ = det.detectMarkers(to)
        co, ic, _ = det.detectMarkers(oo)
        idt = sorted(it.flatten().tolist()) if it is not None else []
        idc = sorted(ic.flatten().tolist()) if ic is not None else []
        print(f"  {z:6.1f}m  {sc:5.2f} {to.shape[0]:5d}   {str(idt):>20} | {idc}")
    print("\n  textured-IDs == original-IDs at every scale = decode-safe (texture is invisible to ArUco)")


if __name__ == '__main__':
    main()
