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
import math
import os
from dataclasses import dataclass, replace
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
#
# LOOSENED to V<100 (2026-08-05): the 2026-08-04 camera-mount yaw fix + camera Z
# offset reduction (.20->.18) shrank the drone-body ghost's footprint enough that
# _reject_blobby_components/_isolate_marker_by_shape reject it on shape alone, even
# with far more of it passing this color gate -- verified clean (correct bbox, no
# ghost contamination) at 2.5m, 5m, AND 10m, both with and without the ROI crop.
# Needed because the cross_marker.png line width was independently halved this same
# session (2026-08-05, tools/halve_cross_marker_linewidth.py) -- at 10m the thinner,
# anti-aliased stroke's own pixels only reached V~60-140 (background sits ~140-160),
# so V<20 (or even V<60) left too few qualifying pixels for Hough line detection to
# find the marker at all, despite it being clearly visible to the eye. If either the
# ghost's footprint grows again (camera remounted further from body) or a scene with
# heavier background shadowing is used, re-verify this doesn't reintroduce the
# original V<90 regression before trusting it.
DEFAULT_LOWER = np.array([0, 0, 0])
# 2026-08-07: env-overridable to test tightening this back down (Hz/Wz raw-signal
# regression investigation -- see feedback_cross_marker_radial_spread_ceiling memory).
# Hypothesis: V<100 admits too much position-correlated anti-aliased edge noise into
# the GFT mask, disproportionately corrupting the position-weighted Hz/Wz raw columns.
# Default unchanged at 100 unless CROSS_COLOR_GATE_V_MAX is set.
_COLOR_GATE_V_MAX = int(os.environ.get("CROSS_COLOR_GATE_V_MAX", "100"))
DEFAULT_UPPER = np.array([180, 255, _COLOR_GATE_V_MAX])  # low-V gate: marker is dark, background is light

# 2026-08-04 CORRECTED: earlier same-day investigation (a "single-loss-event root cause"
# analysis, near-touchdown ROI-crop-truncation narrative) was run against the WRONG
# Gazebo world (run_aruco_landing.sh's hardcoded PX4_GZ_WORLD=aruco, not the dedicated
# cross_marker.sdf world run_cross_marker_altitude_test.sh actually loads) -- that
# evidence (mask_before_roi=3957px -> mask_before_shape=44px on a 68x412 bbox) was
# analyzing the drone-body ghost + a small ArUco tag, NOT the real cross marker at all,
# and does not directly apply. Separately, and independently: the camera mount
# (x500_mono_cam_down/model.sdf) was re-oriented this same day (yaw+=90deg on top of the
# existing pitch=90deg, verified via gz model pose query + direct visual before/after
# comparison in the correctly-rotated frame) to move the drone's own landing-leg mirrored
# ghost from the TOP/BOTTOM image margins to the LEFT/RIGHT margins instead. So the crop
# axis needs to swap to match: X gets the tight crop now (ghost lives there), Y stays
# uncropped. The EXTENT-ADAPTIVE mechanism itself (loosen the crop once a large real
# detection is confirmed, computed from the ROI-independent blobby-stage largest-
# component extent to avoid a caller-side bootstrap deadlock) remains a reasonable
# defensive design regardless of which axis it applies to -- kept, just redirected to X.
ROI_FRAC_X_DEFAULT = float(os.environ.get("CROSS_ROI_FRAC_X", "0.65"))
ROI_FRAC_X_LOOSENED = float(os.environ.get("CROSS_ROI_FRAC_X_LOOSENED", "1.0"))
EXTENT_ADAPTIVE_ROI_THRESHOLD_PX = float(os.environ.get("CROSS_EXTENT_ADAPTIVE_ROI_PX", "250"))

# TRACKING-BASED ROI (2026-08-05): ported from Hardware/scripts/img_data.py's ArUco
# ARUCO_ROI_MARGIN_PX fast path. Confirmed via direct test (shift a real marker sample
# progressively off-center): the STATIC central crop above (ROI_FRAC_X_DEFAULT) starts
# truncating an off-center marker's own mask past ~150-180px of lateral offset (~28% of
# frame width), degrading through insufficient_fit_points -> centroid_mismatch ->
# outright picking up a different (ghost) component -- while the marker stays fully
# visible in the raw frame the whole time. The static crop has no notion of WHERE the
# marker actually is, only the frame's geometric center.
#
# Fix: track the last-known-good bbox and crop AROUND IT (not the frame center) with a
# margin, same as the ArUco reference. If that tracked-crop attempt fails, fall back to
# the existing full-frame path (static ROI + extent-adaptive loosening) -- so a missed
# frame costs one slower full-frame search, never a silent, permanent loss the way the
# static-only crop could. After TRACK_MAX_MISSES consecutive full-frame-path frames
# in a row, the lock is presumed genuinely gone (not just a transient miss) and stays
# full-frame until a fresh detection re-establishes it.
TRACK_MARGIN_PX = int(os.environ.get("CROSS_TRACK_MARGIN_PX", "60"))
TRACK_MAX_MISSES = int(os.environ.get("CROSS_TRACK_MAX_MISSES", "5"))

MIN_INTER_LINE_ANGLE_DEG = 15.0   # below this, lines are too near-parallel to trust intersection
# 2026-08-03 (near-parallel-fit root cause): MIN_INTER_LINE_ANGLE_DEG above only gates the
# PRE-FIT Hough-segment cluster mean angle -- a different, thin-segment-derived signal from
# the ACTUAL fitted line direction _robust_fit_line produces from _cluster_points_from_mask's
# (much larger, angle-from-centroid-selected) point set. Confirmed via LINE_DIAG: at close
# range (large marker extent) one arm's fixed real-world stroke width subtends an increasingly
# wide angle from the centroid, swamping its point count (seen up to 27729) while the other
# arm gets angularly squeezed to a handful (seen as low as 16) -- a fit through that few,
# poorly-constrained points can land almost anywhere, including near-parallel to the dominant
# arm, DESPITE the Hough-segment pre-check showing a healthy ~90 deg separation (segments are
# always thin regardless of mask blob size, so they don't see this failure mode at all).
MIN_FIT_INTER_LINE_ANGLE_DEG = 15.0   # same bar, but applied to the POST-FIT line directions
MIN_FIT_POINTS = 20   # a fit through fewer points than this is unstable/unconstrained regardless
                        # of the angle it happens to land on -- lt2_mask_points_on_arm's floor of
                        # 2 only guards against an outright empty cluster, not this
STUB_REL_ANGLE_DEG = 45.0
STUB_REL_ANGLE_TOL_DEG = 12.0

# CORNER-JOIN segment filter (2026-08-24, shadow-contamination hardening): every real
# marker line meets ANOTHER real marker line near the marker center, at one of exactly
# two relative angles -- 90 deg (the two cross arms) or STUB_REL_ANGLE_DEG=45 (stub to
# either arm). The drone's own cast shadow, by contrast, produces Hough segments that
# generally do NOT share a near-common endpoint with a real line at one of those angles
# -- it's a separate blob merged into the same connected component, not a corner of the
# cross. Filtering at the raw Hough-segment stage (before angle-clustering/fitting) means
# an unpaired shadow segment never gets the chance to seed or pollute a cluster in the
# first place, complementing (not replacing) _robust_fit_line's later per-point pruning.
CORNER_JOIN_ANGLE_TOL_DEG = 15.0
CORNER_JOIN_TARGET_ANGLES_DEG = (90.0, STUB_REL_ANGLE_DEG)
MIN_CLUSTER_SUPPORT = 2   # min Hough segments in a cluster to trust it as a real line over noise
                           # (see the cross-arm pairing fix in detect() for why this exists)

HOUGH_DIAG_LOG = []   # 2026-08-04 root-cause diagnostic, see detect()'s hough_lt2_lines path


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


def _robust_fit_line(points, iters=3, outlier_thresh=2.5, min_keep_frac=0.3):
    """Iterative Huber-weighted line fit through 2D points, with GENUINE
    progressive outlier rejection. Returns (vx, vy, x0, y0, inlier_mask) where
    inlier_mask is a bool array over the ORIGINAL `points` order.

    BUGFIX (2026-08-24, robust feature-point selection): the previous version
    computed per-point Huber weights from the residuals each iteration, then
    immediately discarded them -- `pts = np.asarray(points, ...)` reset the
    working set back to the FULL, unfiltered input on every pass, so the 3
    "iterations" just refit cv2.fitLine(DIST_HUBER,...) on unchanged data 3x.
    Any contamination admitted upstream by the angle-only cluster gate (e.g.
    the drone's own cast shadow overlapping a marker line -- see the
    2026-08-24 shadow-interference investigation) was never actually
    excluded, only down-weighted WITHIN cv2.fitLine's own single-call
    M-estimation (bounded influence, not rejection).

    Now each pass genuinely refits on the surviving inlier subset: a point is
    kept only while its perpendicular distance to the CURRENT fit is within
    `outlier_thresh` times the median residual (a robust MAD-like scale).
    Stops pruning (keeps the last good fit) if fewer than `min_keep_frac` of
    the current points would survive, so a single bad iteration can't
    collapse the fit to too few points."""
    orig = np.asarray(points, dtype=np.float64)
    idx = np.arange(len(orig))          # indices into `orig` still under consideration
    vx = vy = x0 = y0 = None
    for _ in range(iters):
        pts = orig[idx]
        if len(pts) < 2:
            break
        line = cv2.fitLine(pts, cv2.DIST_HUBER, 0, 0.01, 0.01)
        vx, vy, x0, y0 = [float(v) for v in line]
        d = np.abs((pts[:, 0] - x0) * vy - (pts[:, 1] - y0) * vx)
        scale = np.median(d) + 1e-6
        keep = d <= outlier_thresh * scale
        if keep.sum() < max(2, min_keep_frac * len(pts)):
            break   # would over-prune -- keep this iteration's fit + point set as final
        idx = idx[keep]
    inlier_mask = np.zeros(len(orig), dtype=bool)
    inlier_mask[idx] = True
    return vx, vy, x0, y0, inlier_mask


def _filter_segments_by_corner_join(segs, angles, max_gap_px):
    """Keep only Hough segments that geometrically join (share a near-common
    endpoint, within max_gap_px) with another segment at one of the marker's
    known relative angles (CORNER_JOIN_TARGET_ANGLES_DEG). A segment with no
    such partner is treated as likely spurious contamination (e.g. an edge of
    the drone's own cast shadow) rather than a real marker line.

    Falls back to keeping everything if the filter would leave fewer than 2
    segments -- matching _best_pair's graceful-degradation pattern below, so
    an overly strict corner gate can never be the SOLE reason a genuinely
    weak-signal (but uncontaminated) frame fails."""
    n = len(segs)
    keep = np.zeros(n, dtype=bool)
    endpoints = [((float(s[0]), float(s[1])), (float(s[2]), float(s[3]))) for s in segs]
    for a in range(n):
        for b in range(a + 1, n):
            diff = _circ_diff(angles[a], angles[b])
            if not any(abs(diff - t) <= CORNER_JOIN_ANGLE_TOL_DEG for t in CORNER_JOIN_TARGET_ANGLES_DEG):
                continue
            gap = min(math.hypot(p1[0] - p2[0], p1[1] - p2[1])
                      for p1 in endpoints[a] for p2 in endpoints[b])
            if gap <= max_gap_px:
                keep[a] = keep[b] = True
    if int(keep.sum()) < 2:
        return np.ones(n, dtype=bool)
    return keep


def _line_intersection(l1, l2):
    """l1, l2: (vx, vy, x0, y0). Returns (px, py) or None if too near-parallel."""
    vx1, vy1, x01, y01 = l1
    vx2, vy2, x02, y02 = l2
    denom = vx1 * vy2 - vy1 * vx2
    if abs(denom) < 1e-9:
        return None
    t = ((x02 - x01) * vy2 - (y02 - y01) * vx2) / denom
    return (x01 + t * vx1, y01 + t * vy1)


def _restrict_to_center_roi(mask, roi_frac_x=ROI_FRAC_X_DEFAULT, roi_frac_y=1.0):
    """First-pass clutter reduction: the downward camera is mounted centrally
    and the perception/CBF stack's whole job is to keep the marker near
    frame-center, so propeller clutter that sits at frame corners/far edges
    can be zeroed out by restricting to a central ROI. NOT sufficient alone --
    see _isolate_marker_by_shape, which handles clutter (e.g. propeller arms)
    that still falls within this ROI when the drone has some lateral offset.

    roi_frac history (2026-08-01 through 2026-08-04):
    - Started as a single symmetric roi_frac=0.65 (crops x and y equally).
    - Tightened to 0.5 after diag_raw_image_dump.py exposed a pre-existing
      Gazebo camera-render artifact -- a mirrored ghost of the drone's own
      body (its landing legs) in the outer ~18-25% margin of EVERY frame
      (both aruco and cross_marker worlds; not marker-specific -- ArUco's
      small centered tag just never reached it). 0.5 excluded the ghost but
      also cropped the other axis, costing real translation range.
    - Reverted to a loose 0.65, then SPLIT x/y (crop only the ghost's axis,
      leave the other at 1.0) once the ghost was confirmed to sit specifically
      along ONE axis in the rotated frame the pipeline actually processes
      (originally the y/vertical margins -- top/bottom).
    - 2026-08-04: the camera mount itself (x500_mono_cam_down/model.sdf) was
      re-oriented (yaw+=90deg added on top of the existing pointing-down
      pitch) to relocate the ghost from the y/top-bottom margins to the
      x/left-right margins instead (verified via gz model pose query +
      direct before/after visual comparison in the correctly-rotated frame).
      The crop axis swaps to match: x gets the tight crop now, y stays
      uncropped -- see ROI_FRAC_X_DEFAULT's module-level comment."""
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


def _detect_core(frame_bgr, lower=DEFAULT_LOWER, upper=DEFAULT_UPPER,
                  min_line_length=15, max_line_gap=10, identify_stub=True,
                  roi_frac_x=ROI_FRAC_X_DEFAULT, roi_frac_y=1.0):
    """Run the full detection pipeline on one BGR frame. Returns CrossMarkerDetection.

    Ghost-rejection is now layered (2026-08-02): _reject_blobby_components runs
    FIRST and is position-independent (shape/extent-based, catches the ghost's
    fat rotor-hub fragments wherever they land); _restrict_to_center_roi is a
    position-based backstop for whatever slips through -- as of 2026-08-04 this
    crops X (the ghost's axis post camera-rotation fix, see ROI_FRAC_X_DEFAULT's
    module-level comment), not Y; _isolate_marker_by_shape's squareness gate
    runs last as the final component-selection step.

    Pure, stateless, frame-local coordinates -- see detect() for the tracking-based
    ROI wrapper (mirrors Hardware/scripts/img_data.py's ARUCO_ROI_MARGIN_PX fast
    path) that calls this on either a crop or the full frame and owns the
    coordinate-offset bookkeeping."""
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower, upper)
    _px_raw = int(np.sum(mask > 0))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    _px_close = int(np.sum(mask > 0))
    mask = _reject_blobby_components(mask)
    _px_blobby = int(np.sum(mask > 0))

    # EXTENT-ADAPTIVE ROI (2026-08-04): decide the x-crop tightness HERE, from the
    # BLOBBY-STAGE mask's own largest-component extent -- this data is ROI-independent
    # (computed before any crop) and already ghost-shape-filtered (by
    # _reject_blobby_components above), so it can't suffer a bootstrap deadlock a
    # caller-side decision based on a prior SUCCESSFUL bbox would hit (if the crop
    # truncates the marker right as it crosses the threshold, a post-crop bbox never
    # records the marker's true size, so the loosened crop never triggers). Mirrors
    # what _isolate_marker_by_shape will eventually pick anyway (largest candidate).
    _n_lbl, _lbls, _stats_b, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    _largest_extent = 0
    if _n_lbl > 1:
        _areas = _stats_b[1:, cv2.CC_STAT_AREA]
        _best_lbl = 1 + int(np.argmax(_areas))
        _largest_extent = max(_stats_b[_best_lbl, cv2.CC_STAT_WIDTH],
                               _stats_b[_best_lbl, cv2.CC_STAT_HEIGHT])
    _roi_frac_x_eff = (ROI_FRAC_X_LOOSENED if _largest_extent >= EXTENT_ADAPTIVE_ROI_THRESHOLD_PX
                       else roi_frac_x)
    mask = _restrict_to_center_roi(mask, _roi_frac_x_eff, roi_frac_y)
    _px_roi = int(np.sum(mask > 0))
    mask = _isolate_marker_by_shape(mask)
    _px_shape = int(np.sum(mask > 0))
    _stage_px = (_px_raw, _px_close, _px_blobby, _px_roi, _px_shape)

    ys, xs = np.nonzero(mask)
    if len(xs) < 20:
        # DIAG (2026-08-04): which stage killed the mask? See HOUGH_DIAG_LOG.
        HOUGH_DIAG_LOG.append({
            'mask_px': int(len(xs)), 'bbox': None, 'n_edge_px': 0, 'n_lines': 0,
            'stage_px': _stage_px, 'reason': 'color_gate_empty',
        })
        return CrossMarkerDetection(None, None, False, fail_reason='color_gate_empty')
    bbox = (int(xs.min()), int(ys.min()), int(xs.max() - xs.min()), int(ys.max() - ys.min()))

    edges = cv2.Canny(mask, 50, 150, apertureSize=3)
    lines = cv2.HoughLinesP(edges, rho=1, theta=np.pi / 180, threshold=25,
                             minLineLength=min_line_length, maxLineGap=max_line_gap)
    if lines is None or len(lines) < 2:
        # DIAG (2026-08-04, hough_lt2_lines root-cause investigation): hough_lt2_lines is
        # the dominant failure mode (60-70% of misses) -- log every occurrence's mask/bbox
        # stats (cheap, no image write) so the distribution of WHY it fires can be seen
        # across a whole run, not just a hand-sampled subset. stage_px tracks pixel count
        # after each filter stage (raw, morph-close, blobby-reject, roi-crop, shape-isolate)
        # to see WHICH stage is killing the mask when bbox is large but mask_px is tiny.
        HOUGH_DIAG_LOG.append({
            'mask_px': int(len(xs)), 'bbox': bbox,
            'n_edge_px': int(np.sum(edges > 0)), 'n_lines': 0 if lines is None else int(len(lines)),
            'stage_px': _stage_px, 'reason': 'hough_lt2_lines',
        })
        return CrossMarkerDetection(None, None, False, bbox, fail_reason='hough_lt2_lines')

    segs = lines[:, 0, :]
    angles = [_angle_deg(*s) for s in segs]
    # Corner-join shadow filter (2026-08-24) -- see _filter_segments_by_corner_join
    # docstring. Scale the join-gap tolerance to the marker's own bbox so it stays
    # meaningful across altitude (a fixed px gap would be too loose far away, too
    # tight up close).
    _corner_max_gap_px = max(20.0, 0.2 * max(bbox[2], bbox[3], 1))
    _corner_keep = _filter_segments_by_corner_join(segs, angles, _corner_max_gap_px)
    segs = segs[_corner_keep]
    angles = [a for a, k in zip(angles, _corner_keep) if k]
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
    if len(pts_i) < MIN_FIT_POINTS or len(pts_j) < MIN_FIT_POINTS:
        # See MIN_FIT_POINTS' 2026-08-03 comment: a fit through too few points is
        # unconstrained regardless of what angle it happens to land on.
        return CrossMarkerDetection(None, None, False, bbox, fail_reason='insufficient_fit_points')

    vx_i, vy_i, x0_i, y0_i, mask_i = _robust_fit_line(pts_i)
    vx_j, vy_j, x0_j, y0_j, mask_j = _robust_fit_line(pts_j)
    line_i, line_j = (vx_i, vy_i, x0_i, y0_i), (vx_j, vy_j, x0_j, y0_j)
    # ROBUST-SELECTED points (2026-08-24): _robust_fit_line's inlier mask drops
    # perpendicular outliers to the converged fit -- e.g. contamination that
    # passed the coarse angle-window gate above (_cluster_points_from_mask)
    # but doesn't actually lie along the real marker line (the drone's own
    # cast shadow overlapping an arm is the motivating case). These pruned
    # points are what get stored/returned (line_points_i/j below), not the
    # raw angle-gated set -- alpha's moment computation (cross_marker_
    # perception.py's _unweighted_principal_angle) consumes these directly,
    # so pruning here also protects alpha, not just the line fit/center.
    pts_i = [p for p, keep in zip(pts_i, mask_i) if keep]
    pts_j = [p for p, keep in zip(pts_j, mask_j) if keep]
    if len(pts_i) < MIN_FIT_POINTS or len(pts_j) < MIN_FIT_POINTS:
        return CrossMarkerDetection(None, None, False, bbox, fail_reason='insufficient_fit_points_post_prune')
    # POST-FIT near-parallel gate (2026-08-03): ang_i/ang_j (checked above, line ~319) is the
    # PRE-FIT Hough-segment cluster mean -- a different signal from the line _robust_fit_line
    # actually produces from the (much larger, angle-from-centroid-selected) mask-pixel set.
    # Confirmed via LINE_DIAG these can diverge sharply (Hough sep ~90 deg, actual fit sep
    # <1 deg) specifically at close range / large marker extent -- check the REAL fitted
    # directions before trusting their intersection, not just the pre-fit proxy.
    _fit_ang_i = np.degrees(np.arctan2(line_i[1], line_i[0])) % 180.0
    _fit_ang_j = np.degrees(np.arctan2(line_j[1], line_j[0])) % 180.0
    if _circ_diff(_fit_ang_i, _fit_ang_j) < MIN_FIT_INTER_LINE_ANGLE_DEG:
        return CrossMarkerDetection(None, None, False, bbox, fail_reason='near_parallel_fit')

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
                # ROBUST-SELECTED stub points (2026-08-24): same rationale as the
                # arm-line pruning above -- the coarse angle-window gate
                # (_cluster_points_from_mask) only checks bearing-from-approx-
                # center, not collinearity, so a contamination source sharing a
                # similar bearing (e.g. the drone's own shadow falling across
                # the stub near touchdown -- the motivating case for this fix)
                # can leak into pts_k and, being far from center, dominates the
                # quadratic-moment alpha computation downstream (cross_marker_
                # perception.py's _unweighted_principal_angle). Only proceed
                # with a candidate whose points collapse onto a single line
                # (>=2 points survive _robust_fit_line's inlier pruning);
                # otherwise this cluster is NOT a real stub -- try the next one
                # instead of accepting contaminated points.
                if len(pts_k) >= 4:   # _robust_fit_line needs >=2 pts; skip the
                                       # refit for tiny clusters where pruning can't
                                       # meaningfully distinguish signal from noise
                    _, _, _, _, stub_mask = _robust_fit_line(pts_k)
                    pts_k_pruned = [p for p, keep in zip(pts_k, stub_mask) if keep]
                    if len(pts_k_pruned) < 2:
                        continue
                    pts_k = pts_k_pruned
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


def _shift_detection(det, x0, y0, full_shape):
    """Offset a CrossMarkerDetection computed on a CROP (top-left at x0,y0 in full-frame
    coords) back into full-frame coordinates. isolated_mask is embedded into a
    full_shape-sized zero canvas at the crop's offset -- required because downstream
    consumers (cross_marker_perception.py's _compute_hw) pass it to
    cv2.goodFeaturesToTrack(gray, mask=...) alongside the FULL-FRAME gray image, which
    requires matching shapes."""
    if not det.ok:
        return det
    new_center = (det.center[0] + x0, det.center[1] + y0) if det.center else None
    new_bbox = ((det.mask_bbox[0] + x0, det.mask_bbox[1] + y0, det.mask_bbox[2], det.mask_bbox[3])
                if det.mask_bbox else None)
    shift = np.array([x0, y0], dtype=np.float64)
    new_line_i = tuple(map(tuple, np.asarray(det.line_points_i, dtype=np.float64) + shift)) if det.line_points_i else ()
    new_line_j = tuple(map(tuple, np.asarray(det.line_points_j, dtype=np.float64) + shift)) if det.line_points_j else ()
    new_stub = (tuple(map(tuple, np.asarray(det.stub_points, dtype=np.float64) + shift))
                if det.stub_points else None)
    full_mask = None
    if det.isolated_mask is not None:
        full_mask = np.zeros(full_shape, dtype=det.isolated_mask.dtype)
        ch, cw = det.isolated_mask.shape[:2]
        full_mask[y0:y0 + ch, x0:x0 + cw] = det.isolated_mask
    return replace(det, center=new_center, mask_bbox=new_bbox,
                   line_points_i=new_line_i, line_points_j=new_line_j,
                   stub_points=new_stub, isolated_mask=full_mask)


def detect(frame_bgr, lower=DEFAULT_LOWER, upper=DEFAULT_UPPER,
           min_line_length=15, max_line_gap=10, identify_stub=True,
           roi_frac_x=ROI_FRAC_X_DEFAULT, roi_frac_y=1.0, track_state=None):
    """Tracking-based-ROI wrapper around _detect_core (see TRACK_MARGIN_PX's module-level
    comment for the full design rationale -- ported from Hardware/scripts/img_data.py's
    ArUco ARUCO_ROI_MARGIN_PX fast path).

    track_state: optional mutable dict the CALLER owns and passes the SAME instance of
    on every call (mirrors cbf_visibility.py's own explicit-state-dict pattern). Keys:
    'last_bbox' ((x,y,w,h) in full-frame px, or None) and 'miss_count' (int). Pass None
    (default) to get the exact old behavior (always full-frame, no tracking fast path).

    Fast path: if a recent lock exists (miss_count < TRACK_MAX_MISSES), crop to
    last_bbox + TRACK_MARGIN_PX and run _detect_core on JUST that region first -- much
    cheaper than a full-frame search when the marker hasn't moved far. On success,
    offset the result back to full-frame coords and refresh the lock. On failure,
    silently fall through to the full-frame path below (this frame just costs a slower
    search, not a lost detection) and count the miss; after TRACK_MAX_MISSES consecutive
    full-frame frames the lock is presumed stale and stays cleared until a fresh
    full-frame detection re-establishes it.
    """
    if (track_state is not None and track_state.get('last_bbox') is not None
            and track_state.get('miss_count', 0) < TRACK_MAX_MISSES):
        H, W = frame_bgr.shape[:2]
        bx, by, bw, bh = track_state['last_bbox']
        x0 = max(0, bx - TRACK_MARGIN_PX)
        y0 = max(0, by - TRACK_MARGIN_PX)
        x1 = min(W, bx + bw + TRACK_MARGIN_PX)
        y1 = min(H, by + bh + TRACK_MARGIN_PX)
        if x1 > x0 and y1 > y0:
            crop = frame_bgr[y0:y1, x0:x1]
            det = _detect_core(crop, lower, upper, min_line_length, max_line_gap,
                                identify_stub, roi_frac_x=1.0, roi_frac_y=1.0)   # no static
                                # crop inside the already-small tracked window -- the tight
                                # bbox+margin crop IS the ROI here; a second static crop on
                                # top would risk re-truncating the marker for no benefit.
            if det.ok:
                det = _shift_detection(det, x0, y0, frame_bgr.shape[:2])
                track_state['last_bbox'] = det.mask_bbox
                track_state['miss_count'] = 0
                return det
            track_state['miss_count'] = track_state.get('miss_count', 0) + 1

    # Full-frame path: first lock, tracked-crop miss this call, or lock presumed stale.
    det = _detect_core(frame_bgr, lower, upper, min_line_length, max_line_gap,
                        identify_stub, roi_frac_x=roi_frac_x, roi_frac_y=roi_frac_y)
    if track_state is not None:
        if det.ok:
            track_state['last_bbox'] = det.mask_bbox
            track_state['miss_count'] = 0
        # else: leave miss_count as incremented above (or untouched if we never
        # attempted the fast path this call, e.g. no lock yet) -- a full-frame miss
        # doesn't itself increment miss_count further; TRACK_MAX_MISSES counts
        # consecutive FAST-PATH misses specifically, matching the ArUco reference's
        # own roi_miss_count semantics.
    return det
