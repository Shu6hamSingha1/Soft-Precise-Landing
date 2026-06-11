"""Generate a MULTI-SCALE coarse-textured nested ArUco marker for 640x480 LK.

Problem (2026-06-11, CONTROL_FRAMEWORK_REVIEW): the 6-DOF flow lstsq goes
ill-conditioned at low altitude (n_flow_corners -> 4) when the small marker
fills the FoV -> spurious h up to 14 rad/s -> funnel breach -> kappa-runaway.
The existing 0-small_10-big_coarse.png uses a UNIFORM 120px grid, so the small
central marker (~12% of the plane, fills the FoV at touchdown) gets ~1 square
-> no low-altitude corners. Coarse-density helped only at altitude.

Fix: SELF-SIMILAR texture -- square size and spacing SCALE WITH RADIUS from the
center. Fine dense squares in the center (resolve when the 6cm marker fills the
640x480 FoV at ~0.2m), coarsening outward (resolve when the whole plane spans
~108px at 5m). At ANY altitude the FoV-edge features are ~the right pixel size
-> corner density stays roughly constant across altitude -> lstsq stays
over-determined at every scale.

Squares are black, MASKED to white marker regions only and required to sit
ENTIRELY inside white (decode-safe: keeps black coded cells intact; ArUco
samples cell centers, so squares are placed OFF cell-centers via the radial
ring layout). Both nested IDs are verified to still decode across scales, and
the CENTRAL-crop corner count (the low-altitude proxy) is reported.
"""
import os
import numpy as np
import cv2

SRC = '/home/shubham/Soft-Precise-Landing/PX4_Gazebo/Images/0-small_10-big.png'
OUT = '/home/shubham/Soft-Precise-Landing/PX4_Gazebo/Images/0-small_10-big_multiscale.png'

# Self-similar sizing: square side = SQ_FRAC * radius, clamped to [SQ_MIN, SQ_MAX] (texture px).
SQ_FRAC  = float(os.environ.get("MS_SQ_FRAC", "0.10"))
SQ_MIN   = int(os.environ.get("MS_SQ_MIN",  "8"))
SQ_MAX   = int(os.environ.get("MS_SQ_MAX",  "60"))
RING_F   = float(os.environ.get("MS_RING_F", "1.30"))   # geometric ring radius factor
ANG_GAP  = float(os.environ.get("MS_ANG_GAP","2.0"))    # angular center-to-center = ANG_GAP*size
DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)


def place_squares(img):
    H, W = img.shape
    white = (img > 200)
    black = (img < 50)
    out = img.copy()
    cx, cy = W / 2.0, H / 2.0
    r_max = 0.72 * (W / 2.0)            # stop before the outer border/quiet zone
    # EXCLUDE the small central marker (+ its quiet zone) so it stays clean and DECODES through
    # touchdown. The low-alt corner starvation is image-degradation at 0.2-0.4m where the WHOLE plane
    # is in view, so the big-marker texture (in view + blur-resistant) supplies the corners.
    r_excl = float(os.environ.get("MS_R_EXCLUDE", "100"))
    r_min = max(float(SQ_MIN), r_excl)
    placed = 0
    R = r_min
    while R <= r_max:
        size = int(np.clip(round(SQ_FRAC * R), SQ_MIN, SQ_MAX))
        half = size // 2
        n_ang = max(6, int(round(2 * np.pi * R / (ANG_GAP * size))))
        for k in range(n_ang):
            th = 2 * np.pi * k / n_ang + (R * 0.013)   # slight per-ring rotation -> off cell-centers
            px = int(round(cx + R * np.cos(th)))
            py = int(round(cy + R * np.sin(th)))
            y0, y1, x0, x1 = py - half, py + half, px - half, px + half
            if y0 < 0 or x0 < 0 or y1 > H or x1 > W:
                continue
            if white[y0:y1, x0:x1].all():
                out[y0:y1, x0:x1] = 0        # black square inside a white cell
                placed += 1
            elif black[y0:y1, x0:x1].all():
                out[y0:y1, x0:x1] = 255      # white square inside a black cell -> uniform coverage
                placed += 1
        R *= RING_F
    return out, placed


def count_corners(gray):
    g = cv2.goodFeaturesToTrack(gray, maxCorners=200, qualityLevel=0.01, minDistance=5)
    return 0 if g is None else len(g)


def main():
    img = cv2.imread(SRC, cv2.IMREAD_GRAYSCALE)
    H, W = img.shape
    print(f"src {SRC}  {W}x{H}")
    out, placed = place_squares(img)
    cv2.imwrite(OUT, out)
    print(f"placed {placed} radius-scaled squares -> {OUT}\n")

    det = cv2.aruco.ArucoDetector(DICT, cv2.aruco.DetectorParameters())
    # scale -> physical altitude proxy: whole plane spans 'imgpx' px in the 640-wide camera.
    # central crop = the small marker (~12% of plane) -> the FoV-fill / low-altitude view.
    print(f"  {'scale':>5} {'planepx':>8} {'decode IDs':>14} {'corners(full)':>13} {'corners(center12%)':>18}")
    cw = int(0.12 * W)
    cen = out[H//2 - cw//2:H//2 + cw//2, W//2 - cw//2:W//2 + cw//2]
    for sc in [1.0, 0.5, 0.25, 0.12, 0.06]:
        w = int(W * sc); h = int(H * sc)
        rs = cv2.resize(out, (w, h), interpolation=cv2.INTER_AREA)
        pad = 40
        cv = np.full((h + 2*pad, w + 2*pad), 255, np.uint8); cv[pad:pad+h, pad:pad+w] = rs
        _, ids, _ = det.detectMarkers(cv)
        full_c = count_corners(rs)
        ids_l = sorted(int(i) for i in ids.flatten()) if ids is not None else []
        print(f"  {sc:5.2f} {w:8d} {str(ids_l):>14} {full_c:13d} {'-':>18}")
    # low-altitude proxy: small marker filling the 640px FoV (center crop scaled up to ~640)
    cen_up = cv2.resize(cen, (640, 640), interpolation=cv2.INTER_NEAREST)
    print(f"\n  LOW-ALT proxy (center {cw}px crop = small marker, scaled to 640px FoV): "
          f"corners = {count_corners(cen_up)}  (need >=12 for a well-conditioned 6-DOF lstsq)")
    # does the small marker (ID 0) still decode when it fills the frame?
    _, ids_c, _ = det.detectMarkers(cv2.copyMakeBorder(cen_up, 60,60,60,60, cv2.BORDER_CONSTANT, value=255))
    print(f"  small-marker decode at touchdown: {sorted(int(i) for i in ids_c.flatten()) if ids_c is not None else []}")


if __name__ == "__main__":
    main()
