"""Generate a multi-scale, non-overlapping ArUco landing board + verify co-decode.

WHY a board (not concentric nesting):
  Concentric/overlapping markers are MUTUALLY EXCLUSIVE in decode (the inner
  marker clobbers the outer's data cells) — verified on the old
  0-small_10-big.png: only one ID ever decodes per frame, so they give the
  6-DOF lstsq no corner spread. A board places markers at KNOWN, SEPARATED
  offsets so several decode SIMULTANEOUSLY. Their corners then span a large
  normalized image radius, which breaks the L-matrix rank deficiency on the
  lateral/tilt axes (cond 802 -> ~60; see tools/find_camera_rotation.py and
  the numerical separation test).

DESIGN: graduated sizes for altitude coverage on one planar pad.
  - small central marker  -> framed + decodable at touchdown (<0.5 m)
  - medium ring           -> mid-descent (~1-2 m)
  - large corner markers  -> spread at takeoff altitude (~5 m); they leave FoV
                             as the drone descends, by which point the inner
                             markers carry the signal.
  All markers are rigid on the target, so the recovered [h;w] is true
  camera-rel-target motion -> moving-target-safe (unlike off-marker points).

OUTPUTS:
  Images/aruco_board.png            -- the texture (white background = quiet zone)
  Images/aruco_board_layout.npy     -- {id: (x_m, y_m, size_m)} board offsets,
                                        used by img_data.py to reconstruct the
                                        board centre from any visible subset.

The board frame: +x_board = texture column (right), +y_board = texture row
(down). Offsets are metres from the board centre. World alignment is set when
wiring the Gazebo model (user runs Gazebo); this generator is self-consistent.
"""
import os
import numpy as np
import cv2

DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

# ---- Board geometry (metres) ----------------------------------------------
PAD_M  = 2.5                      # physical pad side (Gazebo plane <size>)
TEX_PX = 2048                     # texture resolution (square)

# layout: id -> (cx_m, cy_m, size_m)   (centre offset from pad centre, marker side)
LAYOUT = {
    0: ( 0.00,  0.00, 0.15),      # small centre — touchdown framing
    1: ( 0.55,  0.00, 0.30),      # medium ring (mid-descent)
    2: (-0.55,  0.00, 0.30),
    3: ( 0.00,  0.55, 0.30),
    4: ( 0.00, -0.55, 0.30),
    5: ( 0.90,  0.90, 0.55),      # large corners (high-altitude spread)
    6: (-0.90,  0.90, 0.55),
    7: ( 0.90, -0.90, 0.55),
    8: (-0.90, -0.90, 0.55),
}

# Camera (matches mono_cam SDF): 640x480, fx=270, hfov=1.74.
FX = 270.0
IMG_W, IMG_H = 640, 480


def m_to_px(v_m):
    """metres on the pad -> texture pixels."""
    return v_m / PAD_M * TEX_PX


def render_board():
    canvas = np.full((TEX_PX, TEX_PX), 255, np.uint8)   # white = quiet zone
    c = TEX_PX / 2.0
    for mid, (cx, cy, sz) in LAYOUT.items():
        side_px = int(round(m_to_px(sz)))
        mk = cv2.aruco.generateImageMarker(DICT, mid, side_px)
        # top-left pixel of this marker (board +x -> col, +y -> row)
        u0 = int(round(c + m_to_px(cx) - side_px / 2.0))
        v0 = int(round(c + m_to_px(cy) - side_px / 2.0))
        if u0 < 0 or v0 < 0 or u0 + side_px > TEX_PX or v0 + side_px > TEX_PX:
            raise ValueError(f"marker {mid} ({sz} m at {cx},{cy}) falls off the {PAD_M} m pad")
        canvas[v0:v0 + side_px, u0:u0 + side_px] = mk
    return canvas


def check_overlap():
    """Reject any layout where marker boxes (incl. a 0.5*cell quiet margin) collide."""
    boxes = {}
    for mid, (cx, cy, sz) in LAYOUT.items():
        # quiet margin ~ one ArUco cell = sz/6 each side
        m = sz / 6.0
        boxes[mid] = (cx - sz/2 - m, cx + sz/2 + m, cy - sz/2 - m, cy + sz/2 + m)
    ids = list(boxes)
    bad = []
    for i in range(len(ids)):
        for j in range(i + 1, len(ids)):
            a, b = boxes[ids[i]], boxes[ids[j]]
            if not (a[1] <= b[0] or b[1] <= a[0] or a[3] <= b[2] or b[3] <= a[2]):
                bad.append((ids[i], ids[j]))
    return bad


def verify_codecode(tex):
    """Render the pad as seen from several altitudes; report which IDs co-decode."""
    det = cv2.aruco.ArucoDetector(DICT, cv2.aruco.DetectorParameters())
    print("\n  altitude  pad_px   marker px (small/med/large)   co-decoded IDs")
    print("  " + "-" * 72)
    for Z in [5.0, 3.0, 2.0, 1.0, 0.5, 0.3]:
        # pad spans PAD_M metres -> at altitude Z it subtends PAD_M*FX/Z pixels
        pad_px = PAD_M * FX / Z
        scale = pad_px / TEX_PX
        # render scaled, place on a 640x480 grey frame centred (clip to frame)
        sw = max(1, int(TEX_PX * scale))
        rs = cv2.resize(tex, (sw, sw), interpolation=cv2.INTER_AREA)
        frame = np.full((IMG_H, IMG_W), 200, np.uint8)
        # centre-crop/place
        oy, ox = (IMG_H - sw) // 2, (IMG_W - sw) // 2
        ys0, xs0 = max(0, oy), max(0, ox)
        yr0, xr0 = max(0, -oy), max(0, -ox)
        hh = min(sw - yr0, IMG_H - ys0)
        ww = min(sw - xr0, IMG_W - xs0)
        if hh > 0 and ww > 0:
            frame[ys0:ys0+hh, xs0:xs0+ww] = rs[yr0:yr0+hh, xr0:xr0+ww]
        corners, ids, _ = det.detectMarkers(frame)
        found = sorted(int(i) for i in ids.flatten()) if ids is not None else []
        # per-class marker pixel sizes
        psmall = 0.15 * FX / Z
        pmed   = 0.30 * FX / Z
        plarge = 0.55 * FX / Z
        print(f"  {Z:4.1f} m   {pad_px:6.0f}   {psmall:4.0f}/{pmed:4.0f}/{plarge:4.0f}"
              f"                  {found}")


def main():
    bad = check_overlap()
    if bad:
        print(f"[FAIL] overlapping markers (incl. quiet margin): {bad}")
        return
    print(f"[ok] {len(LAYOUT)} markers, no overlap. pad={PAD_M} m, tex={TEX_PX}px")

    tex = render_board()
    out_dir = '/home/shubham/Soft-Precise-Landing/PX4_Gazebo/Images'
    tex_path = os.path.join(out_dir, 'aruco_board.png')
    cv2.imwrite(tex_path, tex)
    print(f"[ok] wrote {tex_path}")

    layout_path = os.path.join(out_dir, 'aruco_board_layout.npy')
    np.save(layout_path, {k: tuple(v) for k, v in LAYOUT.items()})
    print(f"[ok] wrote {layout_path}  (id -> (x_m, y_m, size_m))")

    verify_codecode(tex)


if __name__ == "__main__":
    main()
