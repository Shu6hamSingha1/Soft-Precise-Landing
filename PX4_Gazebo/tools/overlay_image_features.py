#!/usr/bin/env python3
"""
Overlay the perception-layer image features (s, h, w, alpha) onto a recorded
onboard (down-cam) video from a PX4/Gazebo landing run.

Background (see MEMORY.md-adjacent conversation / image_feature.m /
cross_marker_perception.py / img_data.py for the full derivation):

  s      image-moment position feature. For MARKER_TYPE=cross this is
         'Center Px' (raw pixel centroid, SAME pixel frame as the recorded
         video -- no rotation/derotation needed) + 's_V' (the calibrated
         V-frame [xc,yc,1] the controller actually consumes).
  alpha  marker in-plane orientation (rad). NOT in the raw pixel frame --
         cross_marker_perception.py computes it from the raw stub/arm points
         via _getVirtualPts (gravity-leveled, quaternion-based V-frame
         reprojection + a fixed [y,-x] mount-axis swap) and THEN subtracts a
         calibration offset CROSS_ALPHA_0 (default 90.23 deg). So alpha is
         calibrated for the yaw CONTROL loop, not directly plottable in pixel
         space. This tool draws it via an APPROXIMATE inverse (see
         _alpha_to_pixel_dir) that undoes the mount-axis swap + CROSS_ALPHA_0
         offset but ignores the (small, by-design-decoupled) gravity-leveling
         tilt correction -- exact for level flight, approximate under roll/pitch.
         Logged separately ('alpha(t)') even though controller.py treats it
         as s[3] -- it is computed once, in the same feature-extraction pass.
         When the stub isn't detected a frame, cross_marker_perception.py
         HOLDS the last-good alpha rather than recomputing (see its
         'stub not found this frame' branch) -- this tool flags a held value
         (alpha[i] == alpha[i-1] exactly) by drawing it gray/dashed instead
         of magenta, so a frozen reading is visually obvious.
  h      translational optical flow v/Z (h_x, h_y lateral, h_z = LOOM /
         divergence -- the same physical quantity the old ArduPilot ring-
         flow script (visual_postprocessing.py) drew as radial arrows).
  w      apparent rotational flow (w_x, w_y, w_z) recovered from the SAME
         corner-pair least-squares solve as h (the interaction-matrix
         translational/rotational decomposition). w_z (yaw) is the
         well-observed axis; w_x/w_y are often zeroed/IMU-substituted
         downstream (see W_XY_DEROT in controller.py) -- shown here as
         MEASURED regardless.
  h, w together are logged as 'h_V' (6,): [hx,hy,hz,wx,wy,wz].

Only the cross-marker (MARKER_TYPE=cross) Img_Data.npy schema is fully
supported (this is what every current landing run + the montage tool uses --
see feedback_aruco_perception_scope memory: ArUco is comparison-only). A
best-effort fallback for the legacy ArUco img_data.py schema (Feature Params/
Opt Flow Ang Vel/Image Feature Pts) is included but less exercised.

ALIGNMENT: both the recorded video and every per-frame log array here are
written by CrossMarkerNode's own capture loop, one video frame + one log
append per process_frame() call, but logging starts at thread creation while
the video only starts recording at CONTROLLER_READY (see cross_marker_
perception.py run loop). So the log arrays have a longer PRE-ENGAGE prefix
than the video has frames; this script aligns the two by trimming every log
array to its LAST N_video_frames entries (N_video_frames read directly off
the video file), the same "align from the end" heuristic make_landing_
montage.py uses to align GT against video via touchdown.

Usage:
  tools/overlay_image_features.py --video test_data/Test_Videos/<ts>.mp4 \
      --run test_data/Landing_Test_Cross/<run>/ \
      --out test_data/Test_Videos/overlay_<ts>.mp4
"""
import argparse
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))
import cross_marker_detector as _cmd  # noqa: E402 -- re-run detect() per frame to recover the
                                       # fitted arm lines + their intersection for the s overlay
                                       # (Img_Data.npy only logs the FINAL s_V/Center Px, not the
                                       # intermediate line_points_i/j det() computed them from)


def _load_cross_marker_features(d, n):
    """Cross-marker (MARKER_TYPE=cross) schema -- see cross_marker_perception.py's
    CrossMarkerNode.getLogData(). Returns a dict of (n,...) arrays, trimmed to the
    last n samples (n = video frame count; see module docstring on alignment)."""
    def tail(key, default=None):
        v = d.get(key, default)
        if v is None:
            return None
        v = np.asarray(v)
        return v[-n:] if len(v) >= n else v

    out = dict(
        center_px=tail("Center Px"),
        s_v=tail("s_V"),
        alpha=tail("alpha(t)"),
        h_v=tail("h_V"),
        extent=tail("MARKER_EXTENT_PX"),
        status=tail("Detection Status"),
        visible=tail("FEATURE_IS_VISIBLE"),
    )
    return out


def _load_legacy_aruco_features(d, n):
    """Best-effort fallback for the legacy (MARKER_TYPE=aruco) img_data.py
    schema. Center Px is derived as the mean of the raw corner points (no
    direct pixel-centroid log exists in this schema); s_v/alpha come from the
    calibrated 'Feature Params' V-frame vector [xc,yc,1,alpha]."""
    def tail(key, default=None):
        v = d.get(key, default)
        if v is None:
            return None
        v = np.asarray(v, dtype=object) if key == "Image Feature Pts" else np.asarray(v)
        return v[-n:] if len(v) >= n else v

    feat_pts = tail("Image Feature Pts")
    center_px = None
    if feat_pts is not None:
        pts_list = []
        for fp in feat_pts:
            try:
                cur = np.asarray(fp[1], dtype=float)   # (prev, curr) convention
                pts_list.append(cur.mean(axis=0) if len(cur) else (np.nan, np.nan))
            except Exception:
                pts_list.append((np.nan, np.nan))
        center_px = np.asarray(pts_list, dtype=float)

    feat = tail("Feature Params")   # [xc,yc,1,alpha], calibrated V-frame
    alpha = feat[:, 3] if feat is not None and feat.shape[1] >= 4 else None
    h_v6 = None
    for key in ("Opt Flow Fused", "Opt Flow KF", "Opt Flow Ang Vel"):
        cand = tail(key)
        if cand is not None and cand.size and not np.all(np.isnan(cand)):
            h_v6 = cand
            break

    return dict(center_px=center_px, s_v=feat, alpha=alpha, h_v=h_v6,
                extent=None, status=None, visible=None)


def _fmt(v, n=3):
    return "nan" if v is None or (isinstance(v, float) and np.isnan(v)) else f"{v:+.{n}f}"


# Same default as cross_marker_perception.py's self._alpha_0 (CROSS_ALPHA_0 env,
# default radians(90.23)) -- see that module for the calibration derivation.
CROSS_ALPHA_0 = float(os.environ.get("CROSS_ALPHA_0", str(np.radians(90.23))))


def _draw_full_line(frame, pts, color, thick=2):
    """Fit a line through `pts` (same cv2.fitLine call cross_marker_detector.py's
    _robust_fit_line uses) and draw it clipped across the whole frame, so the
    decoded cross-arm is visible as a LINE, not just its sample points."""
    pts_arr = np.asarray(pts, dtype=np.float32).reshape(-1, 1, 2)
    if len(pts_arr) < 2:
        return None
    vx, vy, x0, y0 = cv2.fitLine(pts_arr, cv2.DIST_L2, 0, 0.01, 0.01).flatten()
    # extend far past the frame in both directions; cv2.line clips to the canvas
    L = 2000
    p1 = (int(x0 - vx * L), int(y0 - vy * L))
    p2 = (int(x0 + vx * L), int(y0 + vy * L))
    cv2.line(frame, p1, p2, color, thick, cv2.LINE_AA)
    for px, py in pts:
        cv2.circle(frame, (int(px), int(py)), 2, color, -1, cv2.LINE_AA)
    return (float(vx), float(vy), float(x0), float(y0))


def _alpha_to_pixel_dir(alpha):
    """Approximate INVERSE of cross_marker_perception.py's alpha computation,
    for drawing a pixel-space direction vector. Forward path (see module
    docstring): raw stub pixel offset (dx,dy) -> _getVirtualPts (gravity-level
    tilt correction + [y,-x] mount-axis swap) -> V-frame (Vdx,Vdy) -> a_disamb
    = atan2(Vdy,Vdx) -> alpha = wrap(a_disamb - CROSS_ALPHA_0). Dropping the
    (small, by-design-decoupled) tilt correction, the swap alone gives
    Vdx=dy, Vdy=-dx, i.e. a_disamb = atan2(-dx, dy). Setting theta =
    alpha + CROSS_ALPHA_0 = a_disamb and solving atan2(-dx,dy) = theta for
    (dx,dy) on the unit circle gives dx=-sin(theta), dy=cos(theta). Exact for
    level flight; degrades gracefully under roll/pitch (the same regime
    where the true tilt-correction term is largest)."""
    theta = alpha + CROSS_ALPHA_0
    return -np.sin(theta), np.cos(theta)


# cv2.putText's Hershey fonts can't render Greek glyphs (alpha shows as '?' or
# blank) -- use PIL/DejaVuSans (has full Greek coverage) for the HUD text
# instead, composited onto the cv2 frame via a numpy roundtrip.
from PIL import Image, ImageDraw, ImageFont

_HUD_FONT = ImageFont.truetype(
    "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 14)


def _draw_hud(frame, lines, org=(8, 20), color=(255, 255, 255)):
    x, y = org
    pad = 4
    pil_img = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
    draw = ImageDraw.Draw(pil_img, "RGBA")
    widths = [draw.textlength(l, font=_HUD_FONT) for l in lines]
    w = int(max(widths)) + 2 * pad
    h = 18 * len(lines) + 2 * pad
    draw.rectangle([x - pad, y - 14, x - pad + w, y - 14 + h], fill=(0, 0, 0, 140))
    rgb_color = (color[2], color[1], color[0])   # BGR -> RGB
    for i, line in enumerate(lines):
        draw.text((x, y - 10 + 18 * i), line, font=_HUD_FONT, fill=rgb_color)
    frame[:] = cv2.cvtColor(np.array(pil_img), cv2.COLOR_RGB2BGR)


CHANNELS = ("s", "h", "alpha")   # w is bundled with h (same lstsq solve) but not drawn
                                  # separately unless explicitly requested via draw_w=True

# --split groups: s+alpha together (both static-pose features from the same
# feature-extraction pass -- image POSITION and ORIENTATION), h on its own
# (the frame-pair flow measurement -- image MOTION).
SPLIT_GROUPS = (("s", "alpha"), ("h",))


def annotate(video_path, feats, out_path, channels=CHANNELS, draw_w=False):
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        raise SystemExit(f"could not open {video_path}")
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    writer = cv2.VideoWriter(out_path, cv2.VideoWriter_fourcc(*'mp4v'), fps, (w, h))

    center_px = feats["center_px"]
    s_v = feats["s_v"]
    alpha = feats["alpha"]
    h_v = feats["h_v"]
    extent = feats["extent"]
    status = feats["status"]
    visible = feats["visible"]
    n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    # Visualization gains (pixels per unit) -- these are display-only scale factors,
    # NOT calibration; tuned so a typical descent produces a visible-but-not-crazy arrow.
    FLOW_GAIN = 150.0
    FLOW_ARROW_MAX = 70
    ALPHA_LEN = 40
    LOOM_RING_R0 = 24    # nominal loom-ring radius at h_z=0
    LOOM_RING_GAIN = 400.0
    FLOW_FIELD_GAIN = 6.0   # per-point RAW pixel displacement magnification (small
                             # arrows) -- NOT the same units/gain as FLOW_GAIN, which
                             # scales the already-normalized (v/Z) summary hx/hy
    FLOW_FIELD_MAX = 25
    MAX_FLOW_PTS = 40

    # h flow-field state: Img_Data.npy only logs the FINAL solved hx/hy/hz, not the
    # per-point LK correspondences _solve_jacobian's lstsq fit them from -- like the
    # s-lines above, reconstruct a best-effort version by tracking our OWN Shi-Tomasi
    # points (re-seeded each frame inside the detected marker mask, matching the live
    # pipeline's per-frame-pair point selection) across the recorded video with LK.
    # Same caveat as the s re-detection: runs on the lossily-recoded mp4, not the raw
    # frames the live solve saw, so these are illustrative correspondences, not the
    # exact ones that produced the logged hx/hy/hz numbers.
    prev_gray = None
    prev_pts = None

    k = 0
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        if frame.ndim == 2:
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

        i = min(k, (len(center_px) - 1) if center_px is not None else 0)
        cx = cy = None
        if center_px is not None and i < len(center_px):
            _cx, _cy = center_px[i]
            if np.isfinite(_cx) and np.isfinite(_cy):
                cx, cy = float(_cx), float(_cy)

        hx = hy = hz = wx = wy = wz = None
        if h_v is not None and i < len(h_v):
            hx, hy, hz, wx, wy, wz = [float(v) for v in h_v[i]]

        a = None
        a_held = False
        if alpha is not None and i < len(alpha) and np.isfinite(alpha[i]):
            a = float(alpha[i])
            a_held = i > 0 and np.isfinite(alpha[i - 1]) and alpha[i] == alpha[i - 1]

        # Re-run the detector on this frame to recover the fitted cross-arm lines
        # (line_points_i/j) -- Img_Data.npy only logs the FINAL s_V/Center Px, not
        # the intermediate lines they were intersected from. Cheap: 1 full-frame
        # detect() per overlay frame (this tool runs offline, not realtime).
        det = None
        gray = None
        if "s" in channels or "h" in channels:
            det = _cmd.detect(frame)
        if "h" in channels:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # draw the PREVIOUS pair's tracked correspondences as a flow field --
            # one small arrow per point, showing the per-point displacements the
            # summary hx/hy/hz arrow+ring are aggregated FROM.
            if prev_gray is not None and prev_pts is not None and len(prev_pts) > 0:
                curr_pts, st, _err = cv2.calcOpticalFlowPyrLK(
                    prev_gray, gray, prev_pts, None, winSize=(15, 15), maxLevel=2,
                    criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 20, 0.03))
                if curr_pts is not None:
                    good_prev = prev_pts[st.flatten() == 1].reshape(-1, 2)
                    good_curr = curr_pts[st.flatten() == 1].reshape(-1, 2)
                    for (px, py), (qx, qy) in zip(good_prev, good_curr):
                        dxp, dyp = qx - px, qy - py
                        gx, gy = qx + dxp * FLOW_FIELD_GAIN, qy + dyp * FLOW_FIELD_GAIN
                        m2 = float(np.hypot(gx - qx, gy - qy))
                        if m2 > FLOW_FIELD_MAX:
                            gx = qx + (gx - qx) * FLOW_FIELD_MAX / m2
                            gy = qy + (gy - qy) * FLOW_FIELD_MAX / m2
                        cv2.arrowedLine(frame, (int(px), int(py)), (int(gx), int(gy)),
                                         (0, 200, 255), 1, tipLength=0.4, line_type=cv2.LINE_AA)
                        cv2.circle(frame, (int(qx), int(qy)), 2, (0, 200, 255), -1, cv2.LINE_AA)

            # re-seed points inside THIS frame's detected marker mask for the NEXT
            # pair -- matches the live pipeline picking fresh Shi-Tomasi points per
            # frame-pair rather than tracking one point set across the whole descent.
            mask_for_lk = (det.isolated_mask if det is not None and det.ok
                            and det.isolated_mask is not None else None)
            new_pts = None
            if mask_for_lk is not None:
                new_pts = cv2.goodFeaturesToTrack(
                    gray, maxCorners=MAX_FLOW_PTS, qualityLevel=0.01, minDistance=5,
                    mask=mask_for_lk.astype(np.uint8))
            prev_gray, prev_pts = gray, new_pts

        if cx is not None:
            if "s" in channels:
                # decoded cross-arm lines the detector fit + intersected to get s
                # (see _robust_fit_line / _line_intersection in cross_marker_detector.py)
                if det is not None and det.ok and det.line_points_i and det.line_points_j:
                    _draw_full_line(frame, det.line_points_i, (0, 165, 255))   # orange
                    _draw_full_line(frame, det.line_points_j, (255, 255, 0))   # cyan

                # centroid crosshair = s, the two lines' intersection
                cv2.drawMarker(frame, (int(cx), int(cy)), (0, 255, 0),
                                cv2.MARKER_CROSS, 14, 2)
                cv2.putText(frame, "s", (int(cx) + 10, int(cy) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

                # detected marker extent (context box, not a controller quantity)
                if extent is not None and i < len(extent) and np.isfinite(extent[i]) and extent[i] > 0:
                    r = int(extent[i] / 2)
                    cv2.circle(frame, (int(cx), int(cy)), r, (180, 180, 180), 1, cv2.LINE_AA)

            if "alpha" in channels and a is not None:
                # alpha=0 reference direction (see _alpha_to_pixel_dir's docstring):
                # theta = alpha + CROSS_ALPHA_0, so alpha=0 <=> theta = CROSS_ALPHA_0
                # itself, i.e. the raw calibration-offset direction. Drawn dashed/gray
                # so it reads as a fixed reference, not a live measurement.
                rdx, rdy = _alpha_to_pixel_dir(0.0)
                rex = cx + ALPHA_LEN * rdx
                rey = cy + ALPHA_LEN * rdy
                cv2.arrowedLine(frame, (int(cx), int(cy)), (int(rex), int(rey)),
                                 (180, 180, 180), 1, tipLength=0.3, line_type=cv2.LINE_4)

                # alpha: orientation line from centroid, mapped back to the raw
                # pixel frame (see _alpha_to_pixel_dir) -- NOT cos(a)/sin(a)
                # directly, alpha lives in a de-rotated + offset V-frame.
                dx, dy = _alpha_to_pixel_dir(a)
                ex = cx + ALPHA_LEN * dx
                ey = cy + ALPHA_LEN * dy
                col = (150, 150, 150) if a_held else (255, 0, 255)   # gray = held/stale
                cv2.arrowedLine(frame, (int(cx), int(cy)), (int(ex), int(ey)),
                                 col, 2, tipLength=0.3,
                                 line_type=cv2.LINE_4 if a_held else cv2.LINE_AA)

                # arc + label showing alpha as the angle FROM the reference TO the
                # live arrow (this angle, by definition, IS alpha itself)
                ref_deg = float(np.degrees(np.arctan2(rdy, rdx)))
                cur_deg = float(np.degrees(np.arctan2(dy, dx)))
                cv2.ellipse(frame, (int(cx), int(cy)), (26, 26), 0,
                            ref_deg, ref_deg + np.degrees(a), (255, 255, 255), 1, cv2.LINE_AA)
                cv2.putText(frame, f"{np.degrees(a):+.1f} deg", (int(cx) + 30, int(cy) + 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1, cv2.LINE_AA)

            if "h" in channels and hx is not None:
                # h_xy: lateral flow arrow (red)
                fx = hx * FLOW_GAIN
                fy = hy * FLOW_GAIN
                mag = float(np.hypot(fx, fy))
                if mag > FLOW_ARROW_MAX:
                    fx *= FLOW_ARROW_MAX / mag
                    fy *= FLOW_ARROW_MAX / mag
                cv2.arrowedLine(frame, (int(cx), int(cy)),
                                 (int(cx + fx), int(cy + fy)), (0, 0, 255), 2, tipLength=0.3)

                # h_z (loom/divergence): colored ring, red=expanding, cyan=contracting
                if hz is not None:
                    r = int(np.clip(LOOM_RING_R0 + hz * LOOM_RING_GAIN, 4, 120))
                    col = (0, 0, 255) if hz > 0 else (255, 255, 0)
                    thick = int(np.clip(abs(hz) * 30, 1, 5))
                    cv2.circle(frame, (int(cx), int(cy)), r, col, thick, cv2.LINE_AA)

            if draw_w and wz is not None and abs(wz) > 1e-3:
                # w_z: small rotation arc icon near the centroid (yellow), CW/CCW by sign
                icon_c = (int(cx) + 34, int(cy) - 34)
                r_icon = 12
                ang0, ang1 = (0, min(300, abs(wz) * 200)) if wz > 0 else (0, -min(300, abs(wz) * 200))
                cv2.ellipse(frame, icon_c, (r_icon, r_icon), 0, ang0, ang1, (0, 255, 255), 2)

        lines = []
        if "s" in channels:
            sx = _fmt(s_v[i, 0]) if s_v is not None and i < len(s_v) else "nan"
            sy = _fmt(s_v[i, 1]) if s_v is not None and i < len(s_v) else "nan"
            lines.append(f"s = ({sx}, {sy})")
        if "alpha" in channels:
            held_txt = " (held)" if a_held else ""
            lines.append(f"α = {_fmt(np.degrees(a), 1) if a is not None else 'nan'}°{held_txt}")
        if "h" in channels:
            lines.append(f"h=({_fmt(hx)}, {_fmt(hy)}, {_fmt(hz)})")
        if draw_w:
            lines.append(f"w: wx={_fmt(wx)} wy={_fmt(wy)} wz={_fmt(wz)}")
        if lines:
            _draw_hud(frame, lines)

        writer.write(frame)
        k += 1

    cap.release()
    writer.release()
    print(f"[overlay] {k} frames -> {out_path}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--video", required=True, help="recorded down-cam mp4")
    ap.add_argument("--run", required=True, help="run dir containing Img_Data.npy")
    ap.add_argument("--out", required=True,
                     help="output mp4 path (combined mode), or path STEM used to derive "
                          "per-channel filenames <stem>_<channel><ext> when --split is set")
    ap.add_argument("--split", action="store_true",
                     help="write one video per group -- (s,alpha) [image position+orientation] "
                          "and (h) [image motion/flow] -- instead of one combined video with "
                          "all three overlaid together")
    ap.add_argument("--channels", default="s,h,alpha",
                     help="comma-separated subset of {s,h,alpha} to draw (combined mode only; "
                          "--split always emits all three as separate files)")
    ap.add_argument("--draw-w", action="store_true",
                     help="also draw w (rotational flow) -- bundled with h in the same "
                          "lstsq solve but not requested by default")
    args = ap.parse_args()

    img_path = os.path.join(args.run, "Img_Data.npy")
    d = np.load(img_path, allow_pickle=True).item()

    cap = cv2.VideoCapture(args.video)
    n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    cap.release()
    if n <= 0:
        raise SystemExit(f"could not read frame count from {args.video}")

    if "Center Px" in d:
        feats = _load_cross_marker_features(d, n)
        print("[overlay] using cross-marker (MARKER_TYPE=cross) schema")
    else:
        feats = _load_legacy_aruco_features(d, n)
        print("[overlay] using legacy ArUco img_data.py schema (best-effort)")

    if args.split:
        stem, ext = os.path.splitext(args.out)
        for group in SPLIT_GROUPS:
            out_grp = f"{stem}_{'_'.join(group)}{ext}"
            annotate(args.video, feats, out_grp, channels=group,
                     draw_w=(args.draw_w and "h" in group))
    else:
        channels = tuple(c.strip() for c in args.channels.split(",") if c.strip())
        annotate(args.video, feats, args.out, channels=channels, draw_w=args.draw_w)


if __name__ == "__main__":
    main()
