"""Standalone detector for the cross+stub fiducial (see docs/cross_marker.pdf).

Decode-free alternative to ArUco: recovers marker center (and optionally heading)
via rotation-invariant relative-angle line clustering + robust line intersection,
rather than ID decode. Self-contained — no dependency on img_data.py/controller.py
internals, so it can be validated offline before any live-pipeline wiring.

Pipeline (matches the design note, PX4_Gazebo/docs/cross_marker.pdf discussion):
  1. color/shape gate -> binary mask
  2. Canny + probabilistic Hough line segments
  3. cluster segment angles by *relative* angle (not fixed absolute buckets)
  4. robust line fit per cross cluster (Huber-weighted iterative fit)
  5. sub-pixel intersection of the two cross lines -> center, with ill-conditioning guards
  6. optional: identify the stub cluster (~45 deg off both cross lines, not through center) -> heading
"""
from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np

# Default color gate: tuned for a near-black marker (see cross_marker.png generation,
# diffuse fill (10,10,10)) over a light speckle background. Re-tune per deployment scene.
# V<90 (original synthetic-texture tuning) was far too loose against live Gazebo
# rendering -- median scene V under real lighting/shadows sat well below 90, so the
# gate caught roughly a third of the frame (background + drone airframe) instead of
# just the marker stroke. Tightened to V<20 against live SITL captures (2026-08-01):
# at both ground-level (marker fills frame) and 5m altitude (marker ~90px), actual
# marker-stroke pixels sit at V<10 while background/non-marker-dark clutter (drone
# body, propeller tips) sits higher -- V<20 leaves margin without re-admitting them.
DEFAULT_LOWER = np.array([0, 0, 0])
DEFAULT_UPPER = np.array([180, 255, 20])  # low-V gate: marker is dark, background is light

MIN_INTER_LINE_ANGLE_DEG = 15.0   # below this, lines are too near-parallel to trust intersection
STUB_REL_ANGLE_DEG = 45.0
STUB_REL_ANGLE_TOL_DEG = 12.0
MIN_CLUSTER_SUPPORT = 2   # min Hough segments in a cluster to trust it as a real line over noise
                           # (see the cross-arm pairing fix in detect() for why this exists)


@dataclass
class CrossMarkerDetection:
    center: Optional[tuple]        # (x, y) sub-pixel, or None if not found
    heading_deg: Optional[float]   # stub direction from center, or None if not identified
    ok: bool
    mask_bbox: Optional[tuple] = None  # (x, y, w, h) of the color-gated blob, for sanity bounds
    # Diagnostic (2026-08-01, point-starvation/centroid-instability investigation):
    # which of detect()'s early-return gates fired, or None on ok=True. Lets a caller
    # log WHY a frame failed instead of just that it did, without re-running the
    # pipeline with breakpoints. See docstrings at each return site below for meaning.
    fail_reason: Optional[str] = None
    # Added for cross_marker_perception.py (h/w/alpha computation) -- populated only on
    # ok=True detections; empty/None on failure (callers must not assume presence).
    line_points_i: tuple = ()          # real detected pixels fit to cross-arm line i
    line_points_j: tuple = ()          # real detected pixels fit to cross-arm line j
    stub_points: Optional[tuple] = None    # real detected pixels fit to the stub line, if found
    isolated_mask: Optional[np.ndarray] = None   # final (ROI+shape-isolated) binary mask this frame
    in_fov: bool = True    # False when `center` is a valid off-frame extrapolation (see the
                            # off-frame sanity path in detect()) -- callers that treat s as a
                            # control-relevant measurement should check this before trusting an
                            # in_fov=False reading the same as an in-frame one (lower confidence:
                            # extrapolated beyond the observed line segment, not directly measured)


def _angle_deg(x1, y1, x2, y2):
    a = np.degrees(np.arctan2(y2 - y1, x2 - x1))
    return a % 180.0  # lines are undirected


def _circ_diff(a, b):
    """Smallest difference between two angles on a 0-180 deg circle."""
    d = abs(a - b) % 180.0
    return min(d, 180.0 - d)


def _cluster_line_angles(angles, max_clusters=3, merge_tol_deg=12.0):
    """Greedy angle clustering on a 0-180 deg circle -- rotation-invariant:
    operates on relative spacing between detected angles, not fixed buckets."""
    if len(angles) == 0:
        return []
    angles = sorted(angles)
    clusters = [[angles[0]]]
    for a in angles[1:]:
        if _circ_diff(a, np.mean(clusters[-1])) <= merge_tol_deg:
            clusters[-1].append(a)
        else:
            clusters.append([a])
    # wrap-around merge (first/last cluster may be the same physical direction near 0/180)
    if len(clusters) > 1 and _circ_diff(np.mean(clusters[0]), np.mean(clusters[-1])) <= merge_tol_deg:
        clusters[0] = clusters[0] + clusters.pop()
    clusters.sort(key=len, reverse=True)
    return clusters[:max_clusters]


def _robust_fit_line(points, iters=3):
    """Iterative Huber-weighted line fit through 2D points. Returns (vx, vy, x0, y0)."""
    pts = np.asarray(points, dtype=np.float64)
    weights = np.ones(len(pts))
    vx = vy = x0 = y0 = None
    for _ in range(iters):
        line = cv2.fitLine(pts, cv2.DIST_HUBER, 0, 0.01, 0.01)
        vx, vy, x0, y0 = [float(v) for v in line]
        # residuals: perpendicular distance from each point to the fitted line
        d = np.abs((pts[:, 0] - x0) * vy - (pts[:, 1] - y0) * vx)
        scale = np.median(d) + 1e-6
        weights = 1.0 / (1.0 + (d / scale) ** 2)
        pts = np.asarray(points, dtype=np.float64)
    return vx, vy, x0, y0


def _line_intersection(l1, l2):
    """l1, l2: (vx, vy, x0, y0). Returns (px, py) or None if too near-parallel."""
    vx1, vy1, x01, y01 = l1
    vx2, vy2, x02, y02 = l2
    denom = vx1 * vy2 - vy1 * vx2
    if abs(denom) < 1e-9:
        return None
    t = ((x02 - x01) * vy2 - (y02 - y01) * vx2) / denom
    return (x01 + t * vx1, y01 + t * vy1)


def _restrict_to_center_roi(mask, roi_frac_x=1.0, roi_frac_y=0.55):
    """First-pass clutter reduction: the downward camera is mounted centrally
    and the perception/CBF stack's whole job is to keep the marker near
    frame-center, so propeller clutter that sits at frame corners/far edges
    can be zeroed out by restricting to a central ROI. NOT sufficient alone --
    see _isolate_marker_by_shape, which handles clutter (e.g. propeller arms)
    that still falls within this ROI when the drone has some lateral offset.

    roi_frac history (2026-08-01):
    - Started as a single symmetric roi_frac=0.65 (crops x and y equally).
    - Tightened to 0.5 after diag_raw_image_dump.py exposed a pre-existing
      Gazebo camera-render artifact -- a mirrored ghost of the drone's own
      body in the outer ~18-25% top/bottom margin of EVERY frame (both
      aruco and cross_marker worlds; not marker-specific -- ArUco's small
      centered tag just never reached it). 0.5 excluded the ghost but also
      cropped x, costing real x/y-phase translation range (2/5 runs in the
      5-run recal sweep lost the marker out of the 0.5 crop, corrupting
      exactly the samples the derive tool needs most).
    - Reverted to a loose 0.65 to test whether _isolate_marker_by_shape's
      squareness gate alone rejects the ghost blob without cropping x at
      all: inconsistent (99.4% ok one run, 64.6% the next -- when the
      marker is small/distant, e.g. the z-altitude phase, its bbox is
      close enough in scale to the ghost's that MORPH_CLOSE bridges them
      into one merged blob that evades the shape filter).
    - SPLIT x/y (this revision): the ghost is specifically a top/bottom
      (vertical, in this rotated-frame orientation) artifact -- the raw
      dumps never showed a left/right intrusion post-rotation. So crop
      ONLY vertically to the observed ghost band (~23% margin per side,
      0.55 keeps clear with margin) and leave x uncropped (1.0) to keep
      full x/y-phase translation range."""
    h, w = mask.shape
    x0, x1 = int(w * (1 - roi_frac_x) / 2), int(w * (1 + roi_frac_x) / 2)
    y0, y1 = int(h * (1 - roi_frac_y) / 2), int(h * (1 + roi_frac_y) / 2)
    out = np.zeros_like(mask)
    out[y0:y1, x0:x1] = mask[y0:y1, x0:x1]
    return out


GHOST_MAX_EXTENT = 0.5   # reject components filled beyond this fraction of their own bbox


def _reject_blobby_components(mask, max_extent=GHOST_MAX_EXTENT, min_area=15):
    """Position-independent ghost-fragment rejection (2026-08-02). The ROI crop
    (_restrict_to_center_roi) only excludes a fixed screen-space band -- it can't
    help when the marker itself is large enough to reach near that boundary, or
    when the ghost overlaps the marker's own visible region. This filters by
    SHAPE instead of position: measured directly off a real ghost-overlap frame
    (calibration_data/output_cross/.../lost_002375_streak60_lt2_angle_clusters.png,
    2026-08-02), the ghost's rotor-hub/motor-housing fragments are small,
    near-square, near-FILLED blobs (extent = area/bbox_area ~0.65-0.74, solidity
    ~0.8-1.0), while the marker's own cross strokes -- even where an individual
    connected component only captures one arm or the crossing -- are thin and
    sparse within their bbox (extent ~0.15-0.25). Zero out any component whose
    extent exceeds max_extent; NOT sufficient alone -- a ghost sliver that
    directly touches/crosses a marker arm merges into one still-thin blob that
    this can't retroactively split (see _isolate_marker_by_shape's squareness
    gate and the ROI crop, which remain the other two layers of defense).

    CROSS-AWARE (2026-08-02, retexture investigation): a textured background
    can introduce faint near-black noise (compression/antialiasing/mipmap
    fringing at the texture's own edges) that MORPH_CLOSE bridges onto the
    cross's own rounded line-caps -- a filled circle is naturally high-extent
    (~0.785) even in isolation, so a cross+noise merge can push the WHOLE
    marker's extent over threshold and get discarded outright (observed:
    'after ROI crop: 0' -- the entire cross vanished, not just background
    clutter). The ghost's rotor-hub fragments were always small (measured
    area ~300-2500px, see the module docstring's reference frame); the real
    marker, even fused with noise, is the dominant coherent structure on the
    plate. So: NEVER reject the single LARGEST surviving component by area,
    regardless of its extent -- that guarantees the cross survives even when
    merged with noise, while smaller extent-violating fragments (the actual
    ghost pieces) still get removed normally."""
    n, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    if n <= 1:
        return mask
    areas = stats[1:, cv2.CC_STAT_AREA]
    largest_lbl = int(np.argmax(areas)) + 1 if len(areas) else None
    out = mask.copy()
    for lbl in range(1, n):
        if lbl == largest_lbl:
            continue
        x, y, bw, bh, area = stats[lbl]
        if area < min_area:
            continue
        extent = area / max(bw * bh, 1)
        if extent > max_extent:
            out[labels == lbl] = 0
    return out


def _isolate_marker_by_shape(mask, min_area=15):
    """The drone's own airframe (propeller blades/tips) is also near-black and
    survives the center-ROI crop whenever the drone has any lateral offset
    from the marker (props are still within the ROI, just not at frame
    corners). Distinguish by SHAPE instead of position: the cross marker's
    bounding box is roughly square (it's an X inscribed in a square), while
    propeller arms are long and thin (width >> height or vice versa). Keep
    only the most "square" sufficiently-large connected component. Falls back
    to the unfiltered mask if nothing qualifies (e.g. marker fills the frame
    at very close range, where its own bbox may not be square either)."""
    n, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    if n <= 1:
        return mask
    best_label, best_squareness = None, 1e18
    for lbl in range(1, n):
        x, y, bw, bh, area = stats[lbl]
        if area < min_area:
            continue
        aspect = max(bw, bh) / max(min(bw, bh), 1)  # 1.0 = perfectly square bbox
        if aspect > 2.5:   # propeller arms are typically >4:1; a cross's bbox is near 1:1
            continue
        # among square-ish candidates, prefer the larger one (more of the true
        # marker vs. a small square-ish clutter fleck)
        score = -area
        if score < best_squareness:
            best_squareness, best_label = score, lbl
    if best_label is None:
        return mask
    return np.where(labels == best_label, 255, 0).astype(np.uint8)


def detect(frame_bgr, lower=DEFAULT_LOWER, upper=DEFAULT_UPPER,
           min_line_length=15, max_line_gap=10, identify_stub=True,
           roi_frac_x=1.0, roi_frac_y=0.65):
    """Run the full detection pipeline on one BGR frame. Returns CrossMarkerDetection.

    Ghost-rejection is now layered (2026-08-02): _reject_blobby_components runs
    FIRST and is position-independent (shape/extent-based, catches the ghost's
    fat rotor-hub fragments wherever they land); _restrict_to_center_roi is a
    position-based backstop for whatever slips through (loosened back toward
    0.65 vertically now that the shape filter is the primary defense -- 0.55
    was costing legitimate marker content when the plate itself sat near the
    crop boundary, without actually fixing the direct-touch/merge case the
    shape filter also can't fix); _isolate_marker_by_shape's squareness gate
    runs last as the final component-selection step."""
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower, upper)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    mask = _reject_blobby_components(mask)
    mask = _restrict_to_center_roi(mask, roi_frac_x, roi_frac_y)
    mask = _isolate_marker_by_shape(mask)

    ys, xs = np.nonzero(mask)
    if len(xs) < 20:
        return CrossMarkerDetection(None, None, False, fail_reason='color_gate_empty')
    bbox = (int(xs.min()), int(ys.min()), int(xs.max() - xs.min()), int(ys.max() - ys.min()))

    edges = cv2.Canny(mask, 50, 150, apertureSize=3)
    lines = cv2.HoughLinesP(edges, rho=1, theta=np.pi / 180, threshold=25,
                             minLineLength=min_line_length, maxLineGap=max_line_gap)
    if lines is None or len(lines) < 2:
        return CrossMarkerDetection(None, None, False, bbox, fail_reason='hough_lt2_lines')

    segs = lines[:, 0, :]
    angles = [_angle_deg(*s) for s in segs]
    clusters = _cluster_line_angles(angles)
    if len(clusters) < 2:
        return CrossMarkerDetection(None, None, False, bbox, fail_reason='lt2_angle_clusters')

    # Pick the two clusters that are the real cross arms. Closest-to-90-degrees-apart
    # alone is NOT a reliable criterion on its own (2026-08-01, found via the off-frame
    # fallback test): a tilted camera view means the marker's two (physically 90deg)
    # arms don't necessarily project to exactly 90deg apart in the image (perspective
    # skew), so angular closeness to 90 is a noisier signal than it looks -- and
    # picking PURELY by "closest to 90" let a pair of weak, spurious single-line
    # clusters (noise landing near exactly 90 apart by chance) beat a real,
    # strongly-supported arm cluster that was a few degrees off 90 because its true
    # partner was mostly out of frame. Fix: gate on SUPPORT first (require both
    # clusters in a candidate pair to have at least MIN_CLUSTER_SUPPORT Hough segments
    # -- a real physical line gets re-detected as multiple overlapping segments, noise
    # usually doesn't), then pick by closest-to-90 only among support-qualified pairs.
    # Falls back to the unfiltered (all-pairs) search if nothing meets the support bar,
    # so a generally weak-signal frame still degrades gracefully instead of returning
    # nothing outright.
    def _best_pair(require_support):
        best, best_err = None, 1e9
        for ii in range(len(clusters)):
            for jj in range(ii + 1, len(clusters)):
                if require_support and (len(clusters[ii]) < MIN_CLUSTER_SUPPORT
                                         or len(clusters[jj]) < MIN_CLUSTER_SUPPORT):
                    continue
                err = abs(_circ_diff(np.mean(clusters[ii]), np.mean(clusters[jj])) - 90.0)
                if err < best_err:
                    best_err, best = err, (ii, jj)
        return best

    best_pair = _best_pair(require_support=True) or _best_pair(require_support=False)
    if best_pair is None:
        return CrossMarkerDetection(None, None, False, bbox, fail_reason='no_pair_found')
    i, j = best_pair
    ang_i, ang_j = np.mean(clusters[i]), np.mean(clusters[j])
    if _circ_diff(ang_i, ang_j) < MIN_INTER_LINE_ANGLE_DEG:
        return CrossMarkerDetection(None, None, False, bbox, fail_reason='near_parallel_pair')

    # Approximate center (mask bbox center) used only to select full-mask pixels by
    # angle-from-center; NOT used as the final answer. Using only Hough segment
    # endpoints here biases the fit toward whichever edge of the stroke Hough happens
    # to sample -- a thick line's two edges aren't sampled symmetrically by
    # HoughLinesP, so an endpoints-only fit is systematically off-center. Selecting
    # from the full binary mask by angle is symmetric about the true centerline.
    bx0, by0, bw0, bh0 = bbox
    approx_cx, approx_cy = bx0 + bw0 / 2.0, by0 + bh0 / 2.0
    min_radius = 0.12 * max(bw0, bh0, 1)  # exclude the arm-overlap region near the true center

    dx_all = xs.astype(np.float64) - approx_cx
    dy_all = ys.astype(np.float64) - approx_cy
    r_all = np.hypot(dx_all, dy_all)
    pt_angle_all = np.degrees(np.arctan2(dy_all, dx_all)) % 180.0

    def _circ_diff_vec(a, b):
        d = np.abs(a - b) % 180.0
        return np.minimum(d, 180.0 - d)

    def _cluster_points_from_mask(cluster_idx, tol_deg=10.0):
        target = clusters[cluster_idx]
        keep = r_all > min_radius
        match = np.zeros(len(xs), dtype=bool)
        for t in target:
            match |= _circ_diff_vec(pt_angle_all, t) <= tol_deg
        sel = keep & match
        return list(zip(xs[sel].tolist(), ys[sel].tolist()))

    pts_i, pts_j = _cluster_points_from_mask(i), _cluster_points_from_mask(j)
    if len(pts_i) < 2 or len(pts_j) < 2:
        return CrossMarkerDetection(None, None, False, bbox, fail_reason='lt2_mask_points_on_arm')

    line_i, line_j = _robust_fit_line(pts_i), _robust_fit_line(pts_j)
    center = _line_intersection(line_i, line_j)
    if center is None:
        return CrossMarkerDetection(None, None, False, bbox, fail_reason='ill_conditioned_intersection')

    # Sanity check -- SPLIT by whether the fitted intersection lands in-frame,
    # because the two cases need different validity criteria (2026-08-01, fixing
    # a real conflict: the original single check rejected legitimate off-frame
    # fallback centers, not just bad fits).
    #
    # IN-FRAME: the fitted intersection must be near the mask's OWN centroid, not
    # just "somewhere inside the bbox+margin". The old bbox+25%-margin check alone
    # was too loose to catch bad fits -- the characterization sweep (2026-08-01,
    # rotation x occlusion) found cases with det.ok=True but 19-147px error, because
    # a biased intersection can still land inside a generous bbox margin. The mask
    # centroid is robust to exactly the kind of single-arm occlusion that breaks the
    # line fit (it's an average over ALL surviving marker pixels, not dependent on
    # which two line clusters got picked), so a large fit-vs-centroid gap is a
    # strong, cheap signal the fit is unreliable even when Hough/clustering
    # "succeeded" structurally. This check is INAPPLICABLE off-frame by
    # construction -- an off-frame center is, by definition, far from the
    # in-frame visible-pixel centroid, so applying it there would reject every
    # legitimate off-frame fallback result, not just bad fits.
    #
    # OFF-FRAME (fallback: any 2 of the 3 concurrent lines still gives the true
    # center, even if it's currently outside the visible frame -- see design
    # discussion): validated differently -- MIN_INTER_LINE_ANGLE_DEG already
    # guards the worst near-parallel/ill-conditioned cases upstream; here we only
    # add a generous cap on extrapolation distance, to catch a genuinely
    # degenerate solve (e.g. a borderline near-parallel pair that passed the angle
    # gate but is still poorly conditioned) rather than to enforce precision --
    # extrapolated results are inherently lower-confidence than in-frame ones
    # (see CrossMarkerDetection.in_fov), not rejected outright for being off-frame.
    frame_h, frame_w = mask.shape
    bx, by, bw, bh = bbox
    in_fov = (0 <= center[0] < frame_w) and (0 <= center[1] < frame_h)

    if in_fov:
        centroid_x, centroid_y = float(xs.mean()), float(ys.mean())
        centroid_err = np.hypot(center[0] - centroid_x, center[1] - centroid_y)
        max_centroid_err = 0.12 * max(bw, bh, 1)   # tight: true crossing point should sit very close to centroid
        if centroid_err > max_centroid_err:
            return CrossMarkerDetection(None, None, False, bbox, fail_reason='centroid_mismatch')
        margin = 0.25 * max(bw, bh, 1)
        if not (bx - margin <= center[0] <= bx + bw + margin and
                by - margin <= center[1] <= by + bh + margin):
            return CrossMarkerDetection(None, None, False, bbox, fail_reason='center_outside_bbox_margin')
    else:
        frame_diag = float(np.hypot(frame_w, frame_h))
        bcx, bcy = bx + bw / 2.0, by + bh / 2.0
        extrap_dist = float(np.hypot(center[0] - bcx, center[1] - bcy))
        MAX_EXTRAPOLATION_DIAGS = 3.0   # generous -- a sanity cap, not a precision bound
        if extrap_dist > MAX_EXTRAPOLATION_DIAGS * frame_diag:
            return CrossMarkerDetection(None, None, False, bbox, fail_reason='extrapolation_too_far')

    heading = None
    stub_points_out = None
    if identify_stub and len(clusters) >= 3:
        for k in range(len(clusters)):
            if k in (i, j):
                continue
            ang_k = np.mean(clusters[k])
            off_i = _circ_diff(ang_k, ang_i)
            off_j = _circ_diff(ang_k, ang_j)
            if (abs(off_i - STUB_REL_ANGLE_DEG) <= STUB_REL_ANGLE_TOL_DEG and
                    abs(off_j - STUB_REL_ANGLE_DEG) <= STUB_REL_ANGLE_TOL_DEG):
                pts_k = _cluster_points_from_mask(k)
                if len(pts_k) < 2:
                    continue
                pk = np.asarray(pts_k, dtype=np.float64)
                # keep only points that do NOT sit near the already-found center
                # (a true stub extends one-sided FROM the center, so its far endpoint
                # is what determines heading; points right at the center are ambiguous)
                d_from_center = np.hypot(pk[:, 0] - center[0], pk[:, 1] - center[1])
                far = pk[d_from_center > np.median(d_from_center)]
                if len(far) == 0:
                    continue
                far_pt = far[np.argmax(np.hypot(far[:, 0] - center[0], far[:, 1] - center[1]))]
                heading = float(np.degrees(np.arctan2(far_pt[1] - center[1], far_pt[0] - center[0])))
                stub_points_out = tuple(pts_k)
                break

    return CrossMarkerDetection((float(center[0]), float(center[1])), heading, True, bbox,
                                 line_points_i=tuple(pts_i), line_points_j=tuple(pts_j),
                                 stub_points=stub_points_out, isolated_mask=mask, in_fov=in_fov)
