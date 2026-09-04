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

# ADAPTIVE CONTRAST GATE (2026-09-01). The fixed absolute-V gate above (inRange
# V<=100) assumes "dark marker strokes, bright everything else". It breaks
# whenever global illumination drops (the plate itself falls below the cutoff), a
# large dark object enters view (landing-platform wall, rover body, deep shadow),
# or the feed is a flat frame (two-instance camera warm-up on rover_cross: whole
# frame V~=90 -> the gate returns all 76800 px -> every downstream stage fails).
# Replace it with CLAHE + adaptiveThreshold, which keys on LOCAL contrast ("dark
# relative to its neighbourhood") -- illumination-invariant, and ~empty on a
# uniform region. CLAHE params match tools/validate_bgflow_corr.py's
# GT-validated _clahe (clip 2.0, tile 8). CROSS_ADAPT_GATE=0 -> legacy inRange.
_ADAPT_GATE       = os.environ.get("CROSS_ADAPT_GATE", "0") == "1"
_ADAPT_CLAHE_CLIP = float(os.environ.get("CROSS_ADAPT_CLAHE_CLIP", "2.0"))
_ADAPT_CLAHE_TILE = int(os.environ.get("CROSS_ADAPT_CLAHE_TILE", "8"))
_ADAPT_BLOCK      = int(os.environ.get("CROSS_ADAPT_BLOCK", "51"))       # local window (forced odd); > stroke width
_ADAPT_C          = float(os.environ.get("CROSS_ADAPT_C", "8"))          # bias: higher -> fewer pixels kept
_ADAPT_V_CEIL     = int(os.environ.get("CROSS_ADAPT_V_CEIL", "255"))     # optional loose absolute sanity bound; 255 = off. Lower (~160) trims bright textured ground from the mask (cheaper Hough) but also eats CLAHE-brightened thin strokes -> hurts far detection; off by default.
_ADAPT_ERODE      = int(os.environ.get("CROSS_ADAPT_ERODE", "0"))        # optional Nx N erode to kill speckle; 0 = off. Eats thin foreshortened strokes -> off by default.
_ADAPT_MIN_STD    = float(os.environ.get("CROSS_ADAPT_MIN_STD", "6.0"))  # frame gray-std floor; below -> structureless feed
_ADAPT_MAX_FILL   = float(os.environ.get("CROSS_ADAPT_MAX_FILL", "0.55"))  # mask fill ceiling; above -> degenerate threshold
_SEG_CAP          = int(os.environ.get("CROSS_SEG_CAP", "120"))         # keep only the N longest Hough segments (bounds the angle loop + pairwise corner-join)


# ─────────────────────────────────────────────────────────────────────────────
# POLARITY-AGNOSTIC CONTRAST GATE (2026-09-03).  CROSS_GATE_MODE=contrast
#
# WHY. Both existing gates encode "the marker is DARK": `inRange(V<=100)` absolutely,
# and CROSS_ADAPT_GATE's THRESH_BINARY_INV relatively. Measured on the robustness eval
# set (test_data/RobustnessFrameset), what inRange retains is
#   base  5.2% (the cross)  |  inv  94.8% (the PLATE)  |  col  0.0% (empty)
# -> on a polarity flip the detector stays at 82% detOK while returning centroids 53.9 px
# off (only 23% within-0.15): CONFIDENTLY WRONG, because Hough then fits plate edges.
# On a chromatic iso-V marker the mask is empty by construction.
#
# THE INVARIANT (user, 2026-09-01): we cannot guarantee a BLACK marker, but we can
# guarantee a marker that CONTRASTS with its background. So segment on *deviation from
# the local background*, with no sign and no absolute level:
#
#   D(p) = || Lab(p) - blur(Lab)(p) ||      (Lab: L = luminance, a/b = chroma)
#
# - POLARITY-AGNOSTIC: it is a magnitude, so dark-on-light and light-on-dark are identical.
# - CHROMA-AWARE: a/b carry the signal when luminance is matched (the `col` case, V 150/150).
# - ILLUMINATION-RELATIVE: the local mean is subtracted, so scene brightness cancels; the
#   threshold is a percentile of D within the ROI, not a fixed level.
# BG_KSIZE must exceed the stroke width (so a stroke deviates from its own neighbourhood)
# and stay below the marker extent (so the whole marker is not its own background).
_GATE_MODE      = os.environ.get("CROSS_GATE_MODE", "legacy")     # legacy | contrast | ensemble
_CG_BG_KSIZE    = int(os.environ.get("CROSS_CG_BG_KSIZE", "41"))  # local-background box (odd)
_CG_CHROMA_GAIN = float(os.environ.get("CROSS_CG_CHROMA_GAIN", "2.0"))  # weight on a/b vs L
_CG_PCT         = float(os.environ.get("CROSS_CG_PCT", "92.0"))   # keep the top (100-PCT)% of D
_CG_MIN_D       = float(os.environ.get("CROSS_CG_MIN_D", "6.0"))  # absolute floor: reject flat frames


# ─────────────────────────────────────────────────────────────────────────────
# STROKE-PROFILE VALIDATION (2026-09-03).  CROSS_STROKE_VALIDATE=1
#
# WHY (measured, not assumed). A contrast gate alone REGRESSES: base 100->89 % detOK,
# dim 100->67 %, and `inv` within-0.15 only 23->29 %. Reason: the PLATE BOUNDARY is a
# genuine high-contrast feature, so contrast segmentation admits it exactly as strongly as
# the cross strokes, and Hough still fits plate edges. That is the whole point of the
# locked 3-stage design -- segmentation cannot do this job alone.
#
# THE DISCRIMINANT is profile SHAPE across a segment, which is polarity- and colour-blind:
#   a marker STROKE  -> RIDGE or VALLEY: background | extremum | background,
#                       so the two OUTER shoulders match each other and differ from the core.
#   a plate/shadow EDGE -> STEP: monotonic, the two shoulders differ from EACH OTHER.
# Test per segment (sampled at several points along it, perpendicular):
#   step_ness  = |mean(outer_left) - mean(outer_right)|
#   stroke_ness= |mean(core) - mean(both outers)|
# keep iff stroke_ness > STROKE_MIN_AMP and step_ness < STROKE_MAX_STEP * stroke_ness.
# Uses |.| throughout -> identical for dark-on-light and light-on-dark.
_STROKE_VALIDATE = os.environ.get("CROSS_STROKE_VALIDATE", "0") == "1"
_SV_HALF     = int(os.environ.get("CROSS_SV_HALF", "7"))       # perpendicular half-length (px)
_SV_CORE     = int(os.environ.get("CROSS_SV_CORE", "2"))       # +/- core half-width (px)
_SV_SAMPLES  = int(os.environ.get("CROSS_SV_SAMPLES", "5"))    # sample points along the segment
_SV_MIN_AMP  = float(os.environ.get("CROSS_SV_MIN_AMP", "8.0"))    # min |core-shoulder|
_SV_MAX_STEP = float(os.environ.get("CROSS_SV_MAX_STEP", "0.8"))   # step/stroke ratio ceiling


def _validate_stroke_segments(gray, segs):
    """Keep only segments whose perpendicular profile is a RIDGE/VALLEY, not a STEP.
    Returns a bool keep-mask over `segs`. Polarity-agnostic (magnitudes only)."""
    H, W = gray.shape[:2]
    keep = np.zeros(len(segs), dtype=bool)
    g = gray.astype(np.float32)
    for si, (x1, y1, x2, y2) in enumerate(segs):
        dx, dy = float(x2 - x1), float(y2 - y1)
        L = math.hypot(dx, dy)
        if L < 4.0:
            continue
        ux, uy = dx / L, dy / L
        nx, ny = -uy, ux                      # unit normal
        votes = 0; tried = 0
        for k in range(_SV_SAMPLES):
            f = (k + 0.5) / _SV_SAMPLES
            cx, cy = x1 + dx * f, y1 + dy * f
            core, oL, oR = [], [], []
            for t in range(-_SV_HALF, _SV_HALF + 1):
                px, py = int(round(cx + nx * t)), int(round(cy + ny * t))
                if not (0 <= px < W and 0 <= py < H):
                    core = []; break
                v = g[py, px]
                if abs(t) <= _SV_CORE:            core.append(v)
                elif t > _SV_HALF - 4:            oR.append(v)
                elif t < -(_SV_HALF - 4):         oL.append(v)
            if not core or len(oL) < 2 or len(oR) < 2:
                continue
            tried += 1
            mL, mR, mC = float(np.mean(oL)), float(np.mean(oR)), float(np.mean(core))
            step   = abs(mL - mR)
            stroke = abs(mC - 0.5 * (mL + mR))
            if stroke >= _SV_MIN_AMP and step <= _SV_MAX_STEP * stroke:
                votes += 1
        if tried and votes >= max(1, tried // 2):     # majority of sampled points agree
            keep[si] = True
    return keep


def _contrast_gate_mask(frame_bgr):
    """Polarity-agnostic, chroma-aware marker-stroke mask. See _GATE_MODE comment.
    Returns (mask, reason) with reason='color_gate_empty' on a structureless frame."""
    lab = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2LAB).astype(np.float32)
    k = _CG_BG_KSIZE | 1
    bg = cv2.blur(lab, (k, k))                       # local background estimate
    d = lab - bg
    d[:, :, 1] *= _CG_CHROMA_GAIN                    # chroma carries the iso-luminant case
    d[:, :, 2] *= _CG_CHROMA_GAIN
    D = cv2.magnitude(cv2.magnitude(d[:, :, 0], d[:, :, 1]), d[:, :, 2])
    thr = float(np.percentile(D, _CG_PCT))
    if thr < _CG_MIN_D:                              # nothing deviates -> no structure
        return np.zeros(D.shape, np.uint8), 'color_gate_empty'
    mask = (D >= thr).astype(np.uint8) * 255
    return mask, None


def _adaptive_gate_mask(frame_bgr):
    """CLAHE + adaptive-threshold marker-stroke mask (see _ADAPT_GATE comment).
    Returns (mask, reason). reason is None on success, else a short tag the caller
    maps to a fail_reason:
      'adapt_lowstd'  -- frame is structureless (dead feed / uniform shadow); no
                         perception algorithm can recover a marker from it.
      (no other reason is returned: an over-full adaptive mask is NOT rejected,
      it falls back to a global Otsu split on the CLAHE image -- still contrast-
      relative and lighting-robust, but global so it can't speckle.)"""
    v = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)[:, :, 2]
    if float(v.std()) < _ADAPT_MIN_STD:
        return np.zeros(v.shape, np.uint8), 'adapt_lowstd'
    ve = cv2.createCLAHE(clipLimit=_ADAPT_CLAHE_CLIP,
                         tileGridSize=(_ADAPT_CLAHE_TILE, _ADAPT_CLAHE_TILE)).apply(v)
    mask = cv2.adaptiveThreshold(ve, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                 cv2.THRESH_BINARY_INV, _ADAPT_BLOCK | 1, _ADAPT_C)  # dark -> 255
    if _ADAPT_V_CEIL < 255:
        mask &= (v < _ADAPT_V_CEIL).astype(np.uint8) * 255
    if _ADAPT_ERODE > 0:
        mask = cv2.erode(mask, np.ones((_ADAPT_ERODE, _ADAPT_ERODE), np.uint8))
    if float(mask.mean()) > _ADAPT_MAX_FILL * 255:
        # adaptive threshold degenerated (very low local contrast everywhere, e.g.
        # a small oblique low-contrast marker at an offset IC) -> global Otsu on
        # the CLAHE image instead of rejecting the frame.
        _t, mask = cv2.threshold(ve, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        if _ADAPT_V_CEIL < 255:
            mask &= (v < _ADAPT_V_CEIL).astype(np.uint8) * 255
        if float(mask.mean()) > _ADAPT_MAX_FILL * 255:
            return mask, 'adapt_lowstd'   # even Otsu can't split -> genuinely no marker
    return mask, None

# 2026-08-28: 320x240 OBLIQUE-LOW retune. At a steep off-nadir view + low altitude
# (IC5 real-perception: ~3m alt, ~2.8m lateral -> ~43deg oblique) one cross diagonal
# is heavily foreshortened -- short, and its anti-aliased stroke is patchy -- so the
# old HoughLinesP(threshold=25, minLineLength=15, maxLineGap=10) on Canny(binary mask)
# frequently found <2 line segments or only segments along the ONE unforeshortened
# arm (fail histogram on that rep: hough_lt2_lines 224, lt2_angle_clusters 313 --
# both dominant). These loosen the line extraction; the downstream _robust_fit_line
# / _filter_segments_by_corner_join / squareness / centroid_mismatch gates still
# reject spurious fits. All env-overridable to revert.
HOUGH_THRESHOLD   = int(os.environ.get("CROSS_HOUGH_THRESHOLD", "16"))    # was 25
HOUGH_MIN_LINE_LEN = int(os.environ.get("CROSS_HOUGH_MIN_LINE_LEN", "10"))  # was 15
HOUGH_MAX_LINE_GAP = int(os.environ.get("CROSS_HOUGH_MAX_LINE_GAP", "16"))  # was 10
HOUGH_MASK_DILATE_PX = int(os.environ.get("CROSS_HOUGH_MASK_DILATE_PX", "1"))  # thicken the patchy foreshortened stroke before Canny; 0 = off
ISOLATE_MAX_ASPECT = float(os.environ.get("CROSS_ISOLATE_MAX_ASPECT", "3.2"))  # was 2.5 -- a foreshortened cross bbox isn't 1:1
# FILL-RATIO BAND for _isolate_marker_by_shape's component pick (2026-09-02).
# The old score=-area picked the LARGEST aspect-passing component. On rover_cross
# a diagonal slice of the platform's shadowed side wall has a square-ish
# axis-aligned bbox (aspect 2.6, clears 3.2) and ~3x the cross's area -> it wins
# and the cross is discarded (measured: wrong on 100/128 acquisition frames =
# the 78% lt2_angle_clusters rate; 0/123 on flat-clean). The two fill ratios are
# cleanly bimodal with a 4x gap: a cross-in-square is ~0.07-0.12 filled (thin X),
# a wall/blob chunk 0.3+. Restrict the pick to a cross-plausible fill band, then
# take max area within it; fall back to the old behaviour if none qualify (the
# marker-fills-frame regime, where the cross's own fill climbs as arms clip the
# edges). CROSS_ISOLATE_FILL_HI huge (e.g. 1.0) reverts.
# CENTROID-GATE SPAN RESCUE (2026-09-02, DEFAULT OFF -- needs a SITL gate).
# `centroid_mismatch` validates the fitted intersection against the MASK PIXEL
# CENTROID. When foreign structure merges into the SAME connected component (the
# rover_cross platform edge fuses to a cross arm) that centroid is dragged off the
# junction: the FIT is right, the REFERENCE is contaminated. Measured on the 9
# static-offset perception rover_cross runs: the detector returns ZERO detections
# from 5 m down to 1 m and `centroid_mismatch` is 79% of all failures (1298/~1640),
# so the controller flies open-loop the whole descent.
# This rescue keeps the legacy check as the FAST PATH and, only when it fails, asks a
# contamination-immune question instead: does the intersection project INSIDE both
# fitted arms' own inlier spans? Clean scenes pass legacy and never reach the rescue,
# so they cannot regress -- verified bit-identical on flat_IC2.
# Offline eval (DetectorFrameset, descent band alt>1 m), detOK:
#   flat_IC2   100%  -> 100%   (identical; err 0.012, within-0.15 100%)
#   rover_IC2  28.5% -> 94.8%  (err med 0.065->0.054)
#   clutter_IC2 50%  -> 87.7%  (err med 0.304->0.034)
# ⚠ It buys RECALL, not accuracy: rover within-0.15 drops 95.9%->75.9%. Per 100
# descent frames that is ~27 good/~1 bad -> ~72 good/~23 bad. Against a 0% live
# baseline that is very likely the better trade, but it is a SITL question --
# CROSS_S_JUMP_GATE (default on) is the outlier defence that must absorb the bad ones.
# ⛔ REJECTED alternative, do not re-try: replacing the mask centroid with the two
# arms' INLIER-point centroid makes it WORSE (rover 28.5%->11.0%) -- unequal inlier
# counts / asymmetric spans put that mean systematically off the junction.
# DEFAULT FLIPPED 0 -> 1 on 2026-09-03 after an interleaved n=5/arm SITL gate on
# flat + clutter (zero launch flakes), with the fill ceiling in place:
#   flat    : PRECISE 0/5 -> 3/5 (xy median 0.143 -> 0.046); detOK 100% both arms, so
#             the gain is in the LATE APPROACH -- 1-3 rescues/run recover frames after
#             the mask centroid starts drifting but before the ceiling cuts in.
#   clutter : TARGET_LOST 5/5 -> 1/5; detOK median ~11% -> ~53%. The failure MODE
#             changes from diving blind to the ground 0.8-2.7 m off, to holding station.
#   overfill rescues: 0 in EVERY run of both worlds (24-120 consulted, all refused) --
#             the ceiling holds, and the pre-ceiling 12.3/12.6 m clutter fly-aways are gone.
# Set CROSS_CENTROID_SPAN_RESCUE=0 to revert.
CENTROID_SPAN_RESCUE = os.environ.get("CROSS_CENTROID_SPAN_RESCUE", "1") == "1"
# Diag: how often the legacy centroid check FAILED (so the rescue was consulted) and
# how often the rescue then admitted the frame. Reported per-run by CrossMarkerNode.
SPAN_RESCUE_STATS = {"consulted": 0, "rescued": 0, "consulted_lowalt": 0, "rescued_lowalt": 0}
# FILL CEILING (2026-09-02, added after the first SITL gate). The rescue is an
# ACQUISITION fix and must NOT fire at overfill: the legacy tolerance already ramps
# 0.12 -> 0.72 as the marker fills the frame, precisely because the mask centroid
# legitimately drifts off the junction there, so a frame failing even that loosened
# check is badly wrong -- and admitting it at touchdown is where a bad centroid hurts
# most. Evidence (SITL, this session): flat IC2 n=5, the ONE fly-away (27.8 m,
# TARGET_LOST) was the ONLY run with overfill rescues (2); its approach was healthy
# (lateral 2.87->0.11 m, centroid err 2-20 mm) until alt<0.3 m where the error
# exploded 0.077->0.86 in <1 s. clutter n=5: 13-41 overfill rescues/run and outcomes
# WORSE than off (median xy ~1.9 -> ~5.6 m, two 12 m fly-aways) despite detOK
# 5-7% -> 22-73%. Set >= 1.0 to disable the ceiling.
CENTROID_SPAN_FILL_MAX = float(os.environ.get("CROSS_CENTROID_SPAN_FILL_MAX", "0.6"))
CENTROID_SPAN_MARGIN = float(os.environ.get("CROSS_CENTROID_SPAN_MARGIN", "0.25"))
ISOLATE_FILL_LO = float(os.environ.get("CROSS_ISOLATE_FILL_LO", "0.02"))
ISOLATE_FILL_HI = float(os.environ.get("CROSS_ISOLATE_FILL_HI", "0.25"))
ANGLE_MERGE_TOL_DEG = float(os.environ.get("CROSS_ANGLE_MERGE_TOL_DEG", "12.0"))

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

# ─────────────────────────────────────────────────────────────────────────────
# STAGE 3 — GEOMETRY-FIRST CONFIRM (2026-09-03).  CROSS_GEOM_CONFIRM=0|1|2
#
# The locked design's third stage. Everything upstream is NEGATIVE: gates that
# reject pixels/segments that look wrong. Nothing ever asks the POSITIVE question
# "are these two accepted strokes actually a cross?", so any two long, roughly
# non-parallel edges that survive become a detection -- which is exactly how `inv`
# produces a confident 110 px error with every gate passing.
#
# This asks it, using ONLY the two arms' own fitted geometry -- no mask centroid,
# no absolute intensity, no polarity, no scale:
#   1. PERPENDICULAR   a cross's arms meet near 90 deg. MIN_FIT_INTER_LINE_ANGLE_DEG
#      =15 is a LOWER BOUND only, so a 20 deg pair -- impossible for a cross --
#      passes today. Perspective skew widens the true band, hence a generous default.
#   2. JUNCTION ON BOTH ARMS   the intersection must lie inside each arm's own inlier
#      span (+margin). Same computation CROSS_CENTROID_SPAN_RESCUE already performs,
#      promoted from last-resort rescue to a primary positive test.
#   3. COMPARABLE STROKE WIDTH   both arms are the same painted stroke, so their
#      robust perpendicular thickness should agree within a factor. A stroke paired
#      with a plate edge fails this -- an edge has no consistent width.
#   4. COMPARABLE LENGTH   loose sanity bound only; clipping and foreshortening make
#      genuine arm lengths quite unequal, so this is not a discriminator.
#
# MODES: 0 = off (default). 1 = confirm runs IN ADDITION to the legacy
# centroid_mismatch proxy (conservative). 2 = confirm REPLACES that proxy, which is
# the actual point of the locked design -- it removes the "marker is the only thing
# in the mask" assumption instead of patching it a fourth time. Measured separately.
#
# ⛔⛔ MEASURED 2026-09-03 AND IT DOES NOT WORK. Kept in-tree, DEFAULT OFF, with the
# numbers, because the design memo listed stage 3 as the obvious untried next step
# and someone will otherwise re-derive it. It is a DEAD END for THIS marker:
#
#   RobustnessFrameset, geom1/geom2:  base detOK 100.0% -> 80.1% (55 rejects, all
#   'geom_not_perpendicular'), while base within-0.15 stayed 97% -- i.e. it rejects
#   ACCURATE detections. inv within-0.15 stayed 0% in every mode. ens_geom2 (both
#   stages) drove inv detOK to 32.2%, still 0% usable.
#
#   ROOT CAUSE -- test 1 is FALSE BY DESIGN here. |fitted angle - 90| on clean `base`
#   is strictly BIMODAL: 63% below 15 deg, and 36% in a 35-55 deg band centred on
#   STUB_REL_ANGLE_DEG = 45. This marker is a 3-armed cross WITH A 45 deg STUB, and
#   in over a third of CORRECT detections _best_pair legitimately selects a
#   stub+arm pair -- which still yields the right junction, because the stub also
#   passes through the centre. "The two accepted strokes are mutually perpendicular"
#   is simply not true of this marker.
#
#   ⚠ AND IT IS NOT A FRAME-CONVENTION BUG (user hypothesis, tested 2026-09-03):
#   the arms of a physically-90 deg cross DO skew in the RAW image plane when the
#   drone is tilted, so the check arguably belongs in the gravity-levelled VIRTUAL
#   plane. Measured both, re-fitting det.line_points_i/j through
#   CrossMarkerPerception._getVirtualPts with each frame's own quaternion:
#       base  raw  p50 1.0  p75 44.9  <15deg 58%  35-55deg 41%
#       base  VIRT p50 1.0  p75 44.8  <15deg 58%  35-55deg 41%
#       inv   raw  p50 6.9  <15deg 64%   |   inv VIRT p50 8.1  <15deg 62%
#   De-rotation moves the distribution by ~0.1 deg and the bimodality is untouched
#   (tilt is only ~21-29 deg here, and that barely rotates a line DIRECTION near the
#   image centre). So the 45 deg mode is the stub, not perspective skew.
#
#   AND IT DOES NOT SEPARATE ANYWAY: on `inv` (every detection wrong) 62-67% of
#   frames are within 15 deg of perpendicular -- MORE cross-like than base's correct
#   ones (58%).
#   Width ratio overlaps too (base p90 2.41 vs inv p90 2.49). So the wrong structure
#   `inv` locks onto is geometrically a better cross than the real thing, and NO
#   self-consistency test on the two lines' relative geometry can separate them --
#   the distinguishing information is not present in that comparison.
#
#   Test 2 (junction inside both arms' spans) is the only sub-test that is sound,
#   and CROSS_CENTROID_SPAN_RESCUE already implements it.
#
#   IMPLICATION: `inv` is not a segmentation failure (the ensemble already picks the
#   right channel/polarity on 60% of its frames) and not a geometry-confirm failure.
#   The next hypothesis has to come from OUTSIDE the two-line model.
# ─────────────────────────────────────────────────────────────────────────────
# RING-TRANSITION CONFIRM (2026-09-03).  CROSS_RING_CONFIRM=1
#
# The test that stage 3 should have been. Visualising what `inv` actually locks
# onto (Memory/px4/feedback_cross_detector_robustness_requirement.md) showed the
# detector fits the PLATE'S OUTER EDGES and returns a plate CORNER as the junction:
# on a polarity-flipped scene inRange(V<=100) keeps the whole plate, Canny/Hough
# find its outline, and two adjacent plate edges meet at a clean right angle.
#
# That is why every stage-3 sub-test failed. A rectangle's corner is EXACTLY 90 deg;
# its two edges are mask boundaries of IDENTICAL width; and the corner lies INSIDE
# both edges' spans. All four geometry criteria pass on a corner by construction.
# The distinguishing information is not in the two lines' relative geometry at all --
# it is in the NEIGHBOURHOOD of the junction:
#
#     a cross junction has arms RADIATING from it;  a corner has two edges MEETING.
#
# So: sample a ring around the candidate junction and count mask transitions.
# A real junction crosses every stroke twice (this marker: 4 arms + stub = ~10);
# a plate corner gives 2 (one contiguous inside-arc). Polarity-agnostic (counts
# BOUNDARIES, not levels), scale-free (radius is a fraction of marker extent),
# and needs no intensity threshold of its own.
#
# MEASURED on test_data/RobustnessFrameset (reject when transitions < T):
#     T    base  bright  darkbg    dim  lowsun    col  ||   inv
#     4    0.0%    1.2%    0.0%   0.3%    0.0%   1.7%  ||  45.8%
#     6    0.0%    5.2%    1.0%   4.2%    0.0%  10.4%  ||  72.0%
#          <-------- false-positive cost -------->     ||  catch
# `base` never falls below 6 (min 6, mode 10 = 5 strokes x 2 edges); 46% of `inv`
# sits at <=2. Default T=6: 72% of the wrong detections caught at 0% cost on
# base/lowsun. For contrast, every stage-3 sub-test had a lift of ~0.02.
#
# ⚠ WHAT THIS DOES AND DOES NOT DO. It REJECTS wrong detections; it does not make
# them right. `inv` within-0.15 stays 0% -- the survivors are still corners. The
# gain is converting `inv` from "99.1% detOK, silently wrong" into "mostly refused",
# so the controller is not fed poisoned measurements and TARGET_LOST engages
# honestly. A failure-mode fix, not a detection fix. Judge it on that.
#
# Uses the CLOSE-STAGE mask deliberately (before blobby-reject / ROI / shape
# isolation): the ring test NEEDS the surrounding context -- the plate boundary is
# precisely the structure that reveals a corner -- and the isolated mask has had it
# stripped out. Skips (never vetoes) when the ring falls outside the frame, which is
# the overfill regime; "don't veto on ignorance" matches the rest of this module.
RING_CONFIRM = os.environ.get("CROSS_RING_CONFIRM", "0") == "1"
RING_MIN_TRANSITIONS = int(os.environ.get("CROSS_RING_MIN_TRANSITIONS", "6"))
RING_R_FRAC = float(os.environ.get("CROSS_RING_R_FRAC", "0.30"))
RING_NSAMP = int(os.environ.get("CROSS_RING_NSAMP", "180"))
RING_MIN_COVER = float(os.environ.get("CROSS_RING_MIN_COVER", "0.75"))
RING_CONFIRM_STATS = {"checked": 0, "pass": 0, "rejected": 0, "skipped_offframe": 0}


def _ring_transitions(mask, cx, cy, R, nsamp=None):
    """Mask boundary crossings around a ring at (cx, cy). None if too much of the
    ring falls outside the frame to judge; 0 for an all-inside or all-outside ring."""
    if nsamp is None:
        nsamp = RING_NSAMP
    th = np.linspace(0.0, 2.0 * np.pi, nsamp, endpoint=False)
    xs = cx + R * np.cos(th)
    ys = cy + R * np.sin(th)
    h, w = mask.shape
    ok = (xs >= 0) & (xs < w - 1) & (ys >= 0) & (ys < h - 1)
    if ok.sum() < nsamp * RING_MIN_COVER:
        return None
    b = (mask[np.clip(ys.astype(int), 0, h - 1),
              np.clip(xs.astype(int), 0, w - 1)] > 0).astype(np.int8)[ok]
    if b.all() or not b.any():
        return 0
    return int(np.sum(b != np.roll(b, 1)))


def _confirm_ring(mask_close, center, bbox):
    """(ok, reason, n_transitions). See RING_CONFIRM at module top."""
    bw, bh = bbox[2], bbox[3]
    R = RING_R_FRAC * max(bw, bh, 1)
    n = _ring_transitions(mask_close, float(center[0]), float(center[1]), R)
    if n is None:
        RING_CONFIRM_STATS["skipped_offframe"] += 1
        return True, None, None
    RING_CONFIRM_STATS["checked"] += 1
    if n < RING_MIN_TRANSITIONS:
        RING_CONFIRM_STATS["rejected"] += 1
        return False, 'ring_not_a_junction', n
    RING_CONFIRM_STATS["pass"] += 1
    return True, None, n


# ─────────────────────────────────────────────────────────────────────────────
# SPAN-BALANCE CONFIRM (2026-09-04, user-proposed).  CROSS_BALANCE_CONFIRM=1
#
# A second, independent answer to "is this really a cross junction, not a plate
# corner" -- different information from the ring test (span geometry, not
# neighbourhood topology), so the two are complementary rather than redundant.
#
# GEOMETRY: a true cross's centre sits INSIDE each arm's own point span --
# points exist on both sides of it. A rectangle corner is the ENDPOINT of both
# edges meeting there -- points sit almost entirely on ONE side. So: for a
# matched line, find where the candidate centre falls along that line's OWN
# fitted span (0 = one end, 0.5 = perfectly centred, 1 = other end); reject
# when it sits too close to an end.
#
# ⚠ FIRST VERSION FAILED (measured, not guessed): scoring BOTH arms gave
# 40%+ false-positive cost on the clean scene -- barely better than random.
# CAUSE: this marker carries a STUB (a short third arm, see
# STUB_REL_ANGLE_DEG=45) -- _best_pair sometimes legitimately matches that
# short, GENUINE stub against a long arm, and a real arm that is short or
# foreshortened has the exact same "centre near one end" signature as a
# corner. Same trap stage 3's perpendicularity test fell into, via a
# different test.
#
# FIX: score ONLY the LONGER (more-supported) of the two matched arms. A real
# long arm's true centre sits mid-span even when its partner (the stub) is
# short; a corner's long edge is STILL truncated at the vertex no matter how
# long it runs. Verified this actually fixes it (RobustnessFrameset):
#   both-arms:   base FP ~45%, inv catch ~72%  (unusable -- FP >= catch)
#   longer-only: base/darkbg/lowsun FP 0-1%, dim/bright FP 1-4%, inv catch 48%
#     col is the outlier (FP ~19%) -- most col detections bypass the line fit
#     entirely (ROI/shape fallback), so this test is not well-matched there.
# Combined with ring confirm (both independently reject): inv catch
# 72.0% -> 76.3%, only ~4 points of overlap -- genuinely complementary, not
# redundant, and still near-zero added cost on the clean scenes.
#
# Uses det.line_points_i/j -- the FINAL, robust-pruned per-arm point sets
# (post _robust_fit_line, same points the actual line fit and alpha moment
# consume) -- not the raw angle-gated set, so this judges the fit that was
# actually used, not an earlier candidate.
BALANCE_CONFIRM = os.environ.get("CROSS_BALANCE_CONFIRM", "0") == "1"
BALANCE_MAX_DIST = float(os.environ.get("CROSS_BALANCE_MAX_DIST", "0.30"))
BALANCE_CONFIRM_STATS = {"checked": 0, "pass": 0, "rejected": 0, "skipped_short": 0}


def _arm_balance(pts, center):
    """Where the candidate centre falls along this arm's own TLS-fitted span,
    as a distance from 0.5 (perfectly centred); 0.5 = centre sits AT an
    endpoint (the corner signature). None if too few points to fit."""
    p = np.asarray(pts, dtype=np.float64)
    if len(p) < 4:
        return None
    mu = p.mean(axis=0)
    pc = p - mu
    _, _, vt = np.linalg.svd(pc, full_matrices=False)
    v = vt[0]
    t = pc @ v
    tc = float((np.asarray(center, dtype=np.float64) - mu) @ v)
    lo, hi = float(t.min()), float(t.max())
    if hi - lo < 1e-6:
        return None
    return abs((tc - lo) / (hi - lo) - 0.5)


def _confirm_balance(pts_i, pts_j, center):
    """(ok, reason, dist). See BALANCE_CONFIRM at module top -- scores only the
    LONGER (more-supported) of the two arms; see the "first version failed"
    note for why both-arms scoring doesn't work on a marker with a stub."""
    longer = pts_i if len(pts_i) >= len(pts_j) else pts_j
    d = _arm_balance(longer, center)
    if d is None:
        BALANCE_CONFIRM_STATS["skipped_short"] += 1
        return True, None, None
    BALANCE_CONFIRM_STATS["checked"] += 1
    if d >= BALANCE_MAX_DIST:
        BALANCE_CONFIRM_STATS["rejected"] += 1
        return False, 'balance_not_centered', d
    BALANCE_CONFIRM_STATS["pass"] += 1
    return True, None, d


GEOM_CONFIRM = int(os.environ.get("CROSS_GEOM_CONFIRM", "0"))
GEOM_PERP_TOL_DEG = float(os.environ.get("CROSS_GEOM_PERP_TOL_DEG", "35.0"))
GEOM_SPAN_MARGIN = float(os.environ.get("CROSS_GEOM_SPAN_MARGIN", "0.25"))
GEOM_WIDTH_RATIO_MAX = float(os.environ.get("CROSS_GEOM_WIDTH_RATIO_MAX", "3.0"))
GEOM_LEN_RATIO_MAX = float(os.environ.get("CROSS_GEOM_LEN_RATIO_MAX", "6.0"))
GEOM_CONFIRM_DIAG = []   # (perp_err_deg, width_ratio, len_i, len_j) per checked frame
GEOM_CONFIRM_STATS = {"checked": 0, "pass": 0,
                      "rej_perp": 0, "rej_span": 0, "rej_width": 0, "rej_len": 0}


def _arm_geometry(pts, line):
    """One arm described in its own along/across frame. width is a ROBUST (MAD)
    perpendicular thickness so a few stray inliers cannot inflate it. Scale-free
    and polarity-blind by construction. None if there is too little to judge."""
    p = np.asarray(pts, dtype=float)
    v = np.array([line[0], line[1]], dtype=float)
    nv = float(np.linalg.norm(v))
    if nv < 1e-9 or len(p) < 4:
        return None
    v = v / nv
    nrm = np.array([-v[1], v[0]])
    mu = p.mean(axis=0)
    t = (p - mu) @ v
    w = (p - mu) @ nrm
    return dict(lo=float(t.min()), hi=float(t.max()),
                length=float(t.max() - t.min()),
                width=2.0 * 1.4826 * float(np.median(np.abs(w - np.median(w)))),
                mu=mu, v=v)


def _confirm_cross_geometry(pts_i, pts_j, line_i, line_j, center):
    """Positive test that the two fitted strokes ARE a cross. -> (ok, reason)."""
    gi = _arm_geometry(pts_i, line_i)
    gj = _arm_geometry(pts_j, line_j)
    if gi is None or gj is None:
        return True, None          # too little to judge -- don't veto on ignorance
    GEOM_CONFIRM_STATS["checked"] += 1
    ai = np.degrees(np.arctan2(line_i[1], line_i[0])) % 180.0
    aj = np.degrees(np.arctan2(line_j[1], line_j[0])) % 180.0
    _perp_err = abs(_circ_diff(ai, aj) - 90.0)
    _wr = (max(gi['width'], gj['width']) / max(min(gi['width'], gj['width']), 1e-6))
    GEOM_CONFIRM_DIAG.append((_perp_err, _wr, gi['length'], gj['length']))
    if _perp_err > GEOM_PERP_TOL_DEG:
        GEOM_CONFIRM_STATS["rej_perp"] += 1
        return False, 'geom_not_perpendicular'
    c = np.asarray(center, dtype=float)
    for g in (gi, gj):
        tc = float((c - g['mu']) @ g['v'])
        m = GEOM_SPAN_MARGIN * max(g['length'], 1e-9)
        if not (g['lo'] - m <= tc <= g['hi'] + m):
            GEOM_CONFIRM_STATS["rej_span"] += 1
            return False, 'geom_junction_off_arm'
    wi, wj = gi['width'], gj['width']
    if min(wi, wj) > 1e-6 and max(wi, wj) / min(wi, wj) > GEOM_WIDTH_RATIO_MAX:
        GEOM_CONFIRM_STATS["rej_width"] += 1
        return False, 'geom_width_mismatch'
    li, lj = gi['length'], gj['length']
    if min(li, lj) > 1e-6 and max(li, lj) / min(li, lj) > GEOM_LEN_RATIO_MAX:
        GEOM_CONFIRM_STATS["rej_len"] += 1
        return False, 'geom_length_mismatch'
    GEOM_CONFIRM_STATS["pass"] += 1
    return True, None
                           # (see the cross-arm pairing fix in detect() for why this exists)

HOUGH_DIAG_LOG = []   # 2026-08-04 root-cause diagnostic, see detect()'s hough_lt2_lines path
PAIR_SELECT_DIAG = []  # 2026-09-04: per-frame (picked pair, all candidate pairs + support)
                        # diagnostic behind CROSS_DIAG_PAIR_SELECT=1 -- used to find and
                        # verify the _best_pair support-priority fix (see its comment);
                        # kept for future debugging of the OTHER stub-pick cause (no near-90
                        # pair available that frame at all -- a Hough-recall problem, not a
                        # selection-logic one, still open).

# SUB-PIXEL JUNCTION REFINE (2026-08-31, s_dot_meas noise-floor investigation).
# `center` is the analytic intersection of two per-frame Huber line fits
# (_line_intersection) -- no refinement against the actual image and no temporal
# filter, so frame-to-frame anti-aliased-edge / mask-boundary noise on the two line
# SLOPES walks the intersection point. Measured on a real perception landing: ~0.2 px
# centroid jitter at altitude, growing to ~1.3 px as the marker fills the frame ->
# differentiated at ~62 Hz that is the bulk of the s_dot_meas noise floor
# (~0.13 u/s vs GT-FB ~0.04). See project_20260831_perception_mode_landing.
# Attempted fix: fit a local quadratic to the GREYSCALE in a small window at the
# intersection and take its stationary point. The X-junction of two dark strokes on a
# light plate is a local intensity MINIMUM (both strokes overlap there -> darkest,
# most-covered point), so a valid refine has a positive-definite fitted Hessian. Uses
# every pixel in the window rather than two fragile slopes -> ZERO added lag. Guarded:
# the analytic center is kept if the Hessian isn't a clean minimum, the window is flat
# (saturated black blob near touchdown), or the shift exceeds CROSS_SUBPIX_MAX_SHIFT_FRAC
# * marker_extent (locked onto other structure).
#
# ⚠ DEFAULT OFF (2026-08-31): an apples-to-apples offline replay (recorded IC2 landing,
# same frames) showed NO jitter reduction from either this quadratic fit OR
# cv2.cornerSubPix, at any shift guard -- baseline 2.05/1.68 px, every variant within
# ±0.05 px (cornerSubPix even made a clean ~0.9 px mean sub-pixel correction, and the
# jitter still didn't move). Conclusion: the centroid jitter is NOT in the junction
# localization -- it's in the two line SLOPES feeding _line_intersection (which mask
# pixels pass the anti-aliased / re-compressed edge each frame -> fit direction wobbles
# -> the intersection walks). A junction-region intensity fit has no independent
# leverage on slope noise. The real levers are temporal (motion-compensated multi-frame
# line fit) or intensity-weighted sub-pixel line-centerline fitting. Kept as a knob
# because the offline replay is on the 2x-downscaled + H.264 IMG_RECORD mp4, not the
# live raw-frame path -- set CROSS_SUBPIX_JUNCTION=1 for a live SITL A/B before trusting.
_SUBPIX_JUNCTION = os.environ.get("CROSS_SUBPIX_JUNCTION", "0") == "1"
_SUBPIX_MAX_SHIFT_FRAC = float(os.environ.get("CROSS_SUBPIX_MAX_SHIFT_FRAC", "0.10"))
SUBPIX_STATS = {"applied": 0, "rej_no_fit": 0, "rej_shift": 0, "shift_sum": 0.0}

# INTENSITY-WEIGHTED SUB-PIXEL CENTERLINE FIT (2026-08-31, s_dot_meas noise-floor lever).
# The junction refine above failed because it re-reads the same content: the jitter is
# in the two arm-line SLOPES, and those wobble because _robust_fit_line runs on BINARY
# mask pixels -- whichever anti-aliased edge pixels clear the colour gate that frame ->
# whole rows of the arm toggle in/out -> the fitted direction walks. This attacks it at
# the source: march along each arm and, at every station, take the INTENSITY-WEIGHTED
# centroid of the dark band in the perpendicular strip -> one sub-pixel centerline
# sample whose response to a half-pixel edge shift is a fraction of a pixel (continuous
# greyscale ramp), not a toggled row. TLS-fit the line through the stations. Falls back
# to the _robust_fit_line result if it can't get enough clean stations.
_SUBPIX_CENTERLINE = os.environ.get("CROSS_SUBPIX_CENTERLINE", "0") == "1"   # default OFF pending live A/B
_CENTERLINE_STEP_PX = float(os.environ.get("CROSS_CENTERLINE_STEP_PX", "2.0"))
_CENTERLINE_MIN_STATIONS = int(os.environ.get("CROSS_CENTERLINE_MIN_STATIONS", "8"))
_CENTERLINE_MIN_DARKNESS = float(os.environ.get("CROSS_CENTERLINE_MIN_DARKNESS", "20.0"))  # grey levels below local bg
CENTERLINE_STATS = {"applied": 0, "fallback": 0, "stations_used": 0}


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


def _cluster_line_angles(angles, max_clusters=3, merge_tol_deg=None):
    """Greedy angle clustering on a 0-180 deg circle -- rotation-invariant:
    operates on relative spacing between detected angles, not fixed buckets."""
    if merge_tol_deg is None: merge_tol_deg = ANGLE_MERGE_TOL_DEG
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


def _refine_junction_subpix(gray, cx, cy, half_win):
    """Refine (cx, cy) to sub-pixel via a local quadratic fit to `gray` (2-D array,
    same pixel coords as cx, cy). Returns (rx, ry), or None so the caller keeps the
    analytic intersection. The junction of two dark strokes on a light plate is a
    local intensity MINIMUM -> require a positive-definite fitted Hessian; also bail
    on a flat window (saturated black blob) or a stationary point outside the window.
    See the SUBPIX_JUNCTION block at module top for why."""
    h = int(half_win)
    if h < 3:
        return None
    x0, y0 = int(round(cx)), int(round(cy))
    H, W = gray.shape[:2]
    if x0 - h < 0 or y0 - h < 0 or x0 + h >= W or y0 + h >= H:
        return None
    win = gray[y0 - h:y0 + h + 1, x0 - h:x0 + h + 1].astype(np.float64)
    if float(win.max() - win.min()) < 1.0:
        return None                                    # flat -> nothing to localize
    win = cv2.GaussianBlur(win, (3, 3), 0)
    ax = np.arange(-h, h + 1, dtype=np.float64)
    X, Y = np.meshgrid(ax, ax)
    xf, yf, zf = X.ravel(), Y.ravel(), win.ravel()
    # z = a x^2 + b xy + c y^2 + d x + e y + f   (local coords, origin at x0,y0)
    A = np.column_stack([xf * xf, xf * yf, yf * yf, xf, yf, np.ones_like(xf)])
    coef, _res, _rank, _sv = np.linalg.lstsq(A, zf, rcond=None)
    a, b, c, d, e, _f = coef
    if not (a > 0.0 and (4.0 * a * c - b * b) > 1e-9):
        return None                                    # not a clean local minimum
    try:
        dxy = np.linalg.solve(np.array([[2.0 * a, b], [b, 2.0 * c]]),
                              np.array([-d, -e]))
    except np.linalg.LinAlgError:
        return None
    if not np.all(np.isfinite(dxy)) or float(np.hypot(dxy[0], dxy[1])) > h:
        return None                                    # stationary point outside the window
    return (x0 + float(dxy[0]), y0 + float(dxy[1]))


def _sample_bilinear(img, x, y):
    """Bilinear sample of a 2-D array at (x, y). None if outside the interpolable range."""
    hh, ww = img.shape[:2]
    if x < 0.0 or y < 0.0 or x >= ww - 1 or y >= hh - 1:
        return None
    ix, iy = int(x), int(y)
    fx, fy = x - ix, y - iy
    a = float(img[iy, ix]);         b = float(img[iy, ix + 1])
    c = float(img[iy + 1, ix]);     d = float(img[iy + 1, ix + 1])
    return (a * (1.0 - fx) + b * fx) * (1.0 - fy) + (c * (1.0 - fx) + d * fx) * fy


def _fit_arm_centerline_subpix(gray, pts, prelim_line, step=None,
                               min_stations=None, min_darkness=None):
    """Re-fit one cross arm to sub-pixel precision by sampling its centerline.

    gray        : 2-D greyscale, same pixel coords as `pts` / `prelim_line`.
    pts         : the arm's cluster points (bound the arc-length extent + perp spread).
    prelim_line : (vx, vy, x0, y0) from _robust_fit_line -- seed direction + point.

    March along the seed direction; at each station take the intensity-weighted
    centroid of the dark band in the perpendicular strip -> a sub-pixel centerline
    sample. TLS-fit through the samples. Returns (vx, vy, x0, y0) or None (caller keeps
    the _robust_fit_line result). See the SUBPIX_CENTERLINE block at module top."""
    if step is None:            step = _CENTERLINE_STEP_PX
    if min_stations is None:    min_stations = _CENTERLINE_MIN_STATIONS
    if min_darkness is None:    min_darkness = _CENTERLINE_MIN_DARKNESS
    P = np.asarray(pts, dtype=np.float64)
    if P.ndim != 2 or len(P) < min_stations:
        return None
    vx, vy, lx, ly = prelim_line
    d = np.array([vx, vy], dtype=np.float64)
    nd = np.linalg.norm(d)
    if nd < 1e-9:
        return None
    d /= nd
    n = np.array([-d[1], d[0]])
    p0 = np.array([lx, ly], dtype=np.float64)
    s = (P - p0) @ d
    u = (P - p0) @ n
    s_lo, s_hi = float(s.min()), float(s.max())
    if s_hi - s_lo < 2.0 * step:
        return None
    half_band = float(np.clip(2.5 * (np.median(np.abs(u - np.median(u))) + 1e-6), 3.0, 12.0))
    n_off = int(round(half_band))
    offs = np.arange(-n_off, n_off + 1, dtype=np.float64)
    st = max(float(step), 0.5)
    samples = []
    k = 0
    while True:
        sc = s_lo + k * st
        k += 1
        if sc > s_hi:
            break
        base = p0 + sc * d
        prof = np.empty(len(offs))
        ok = True
        for m, uo in enumerate(offs):
            g = _sample_bilinear(gray, base[0] + uo * n[0], base[1] + uo * n[1])
            if g is None:
                ok = False
                break
            prof[m] = g
        if not ok:
            continue
        bg = float(prof.max())                       # brightest pixel in the strip = local background
        wgt = np.clip(bg - prof, 0.0, None)
        if float(wgt.max()) < min_darkness:          # no real dark stroke here (off arm end, or filled-black plateau)
            continue
        wsum = float(wgt.sum())
        if wsum < 1e-6:
            continue
        u_star = float((wgt * offs).sum() / wsum)
        if abs(u_star) > half_band - 0.5:            # band centroid pinned to the strip edge -> unreliable
            continue
        samples.append(base + u_star * n)
    if len(samples) < min_stations:
        return None
    S = np.asarray(samples)
    mean = S.mean(axis=0)
    cov = np.cov((S - mean).T)
    if not np.all(np.isfinite(cov)):
        return None
    evals, evecs = np.linalg.eigh(cov)
    dvec = evecs[:, int(np.argmax(evals))]
    CENTERLINE_STATS["stations_used"] += len(samples)
    return (float(dvec[0]), float(dvec[1]), float(mean[0]), float(mean[1]))


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


def _best_cross_component(mask, min_area=15):
    """Pick the most cross-plausible connected component of `mask`.

    Returns (labels_img, best_label, best_area); best_label is None when NOTHING
    in the mask is cross-shaped. Extracted from _isolate_marker_by_shape (2026-09-03)
    so the candidate-mask ensemble can ask the SAME question of several masks and
    compare the answers -- the selection rule is therefore identical whether it runs
    once (legacy) or K times (ensemble), by construction rather than by convention.

    The criteria are unchanged: bbox aspect below ISOLATE_MAX_ASPECT (propeller arms
    are >4:1, a foreshortened cross up to ~3:1) and fill inside
    [ISOLATE_FILL_LO, ISOLATE_FILL_HI] (a cross-in-square is a thin X filling
    ~0.07-0.12 of its own bbox; a wall slice or a solid plate is 0.3+), preferring
    the largest survivor."""
    n, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    if n <= 1:
        return labels, None, 0
    best_label, best_area = None, -1
    for lbl in range(1, n):
        x, y, bw, bh, area = stats[lbl]
        if area < min_area:
            continue
        aspect = max(bw, bh) / max(min(bw, bh), 1)  # 1.0 = perfectly square bbox
        if aspect > ISOLATE_MAX_ASPECT:
            continue
        fill = area / max(bw * bh, 1)
        if not (ISOLATE_FILL_LO <= fill <= ISOLATE_FILL_HI):
            continue
        if area > best_area:
            best_area, best_label = area, lbl
    return labels, best_label, max(best_area, 0)


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
    labels, best_label, _area = _best_cross_component(mask, min_area)
    if best_label is None:
        return mask
    return np.where(labels == best_label, 255, 0).astype(np.uint8)


# ─────────────────────────────────────────────────────────────────────────────
# CANDIDATE-MASK ENSEMBLE (2026-09-03).  CROSS_GATE_MODE=ensemble
#
# WHY, and why this is not a third thresholding rule. The two prior front-end
# attempts (CROSS_GATE_MODE=contrast, CROSS_STROKE_VALIDATE=1) both changed the
# RULE that decides which PIXELS are marker, and both failed for the same reason:
# the plate boundary is genuine high-contrast structure, so no single pixel-level
# criterion separates it from the strokes. This changes the SELECTION instead --
# generate several plausible segmentations and let the existing cross-SHAPE test
# (_best_cross_component) say which one actually contains a cross. The shape test
# already works on clean scenes; it was simply never given a second candidate.
#
# The two headline failures on test_data/RobustnessFrameset are both single-
# candidate failures, not shape-test failures:
#   inv (polarity flip): inRange(V<=100) keeps the PLATE (94.8% of pixels at low
#     altitude), so the mask is the plate with the cross as HOLES. _isolate_marker_
#     by_shape's fill band CORRECTLY rejects that blob (fill ~1.0) -- and then the
#     function returns the UNFILTERED mask anyway, so Hough fits plate structure and
#     nothing downstream objects. 99.1% detOK, 0% within-0.15: it LIES.
#   col (chromatic, iso-V): the V gate keeps ~0.7% of pixels by construction; every
#     detection comes from the ROI/shape fallback. 62.4% detOK.
# Both are recoverable if the SAME shape test is offered a mask built on a channel
# and polarity that actually separate marker from plate.
#
# LEGACY-FIRST, by design. If the legacy V-gate already yields a cross-shaped
# component, that candidate is returned unchanged and no other candidate is even
# scored. So on every scene where the current detector works (base/dim/bright/
# lowsun/darkbg -- 95-100% detOK) this mode is INERT BY CONSTRUCTION, not by
# tuning. Same shape as CROSS_CENTROID_SPAN_RESCUE (legacy check as the fast path,
# alternatives consulted only on failure), which is the pattern that survived its
# SITL gate. Cost on clean scenes is therefore one extra connectedComponents call.
#
# ⚠ NOT a fix for terminal overfill. At overfill the marker legitimately fills the
# frame and no candidate is cross-shaped; the ensemble then falls back to the
# legacy mask exactly as today, PRESERVING the documented close-range behaviour
# (_isolate_marker_by_shape's own "marker fills the frame" fallback). The loud
# failure below fires only for the degenerate case the fallback was never meant to
# cover: a SOLID mask, which cannot be a cross at any scale or distance.
_ENS_SOLID_FILL_MAX = float(os.environ.get("CROSS_ENSEMBLE_SOLID_FILL_MAX", "0.50"))
_ENS_MIN_AREA       = int(os.environ.get("CROSS_ENSEMBLE_MIN_AREA", "15"))
ENSEMBLE_STATS = {"legacy_ok": 0, "rescued": 0, "no_cross": 0, "by_channel": {}}


def _ensemble_candidates(frame_bgr, lower, upper):
    """(name, raw_mask) candidates. Legacy FIRST so the caller can short-circuit.

    Otsu is per-frame and per-channel, so it is illumination-adaptive (dim/bright)
    without an absolute level; taking BOTH polarities of each channel removes the
    dark-marker assumption (inv); including a and b makes it chroma-aware, which is
    the only way to see an iso-luminance marker (col). Lab rather than HSV because
    hue is unstable at low saturation and wraps."""
    out = [("legacy", cv2.inRange(cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV), lower, upper))]
    lab = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2LAB)
    for ci, cn in ((0, "L"), (1, "a"), (2, "b")):
        _, m_lo = cv2.threshold(lab[:, :, ci], 0, 255,
                                cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        out.append((cn + "-", m_lo))                    # marker darker than plate
        out.append((cn + "+", cv2.bitwise_not(m_lo)))   # marker brighter than plate
    return out


def _ensemble_gate_mask(frame_bgr, lower, upper):
    """Returns (raw_mask, fail_reason). Preprocesses each candidate the same way
    _detect_core's own chain does up to the shape stage (morph-close, then
    _reject_blobby_components) so the scores are comparable to what the main path
    will actually see, then returns the RAW winning mask for that chain to redo.
    fail_reason is not None only for the solid-mask case described above."""
    cands = _ensemble_candidates(frame_bgr, lower, upper)
    k3 = np.ones((3, 3), np.uint8)
    legacy_raw = cands[0][1]
    best = (None, None, -1)   # (name, raw_mask, area)
    for name, raw in cands:
        m = cv2.morphologyEx(raw, cv2.MORPH_CLOSE, k3)
        m = _reject_blobby_components(m)
        _, lbl, area = _best_cross_component(m, _ENS_MIN_AREA)
        if lbl is None:
            continue
        if name == "legacy":
            ENSEMBLE_STATS["legacy_ok"] += 1
            return raw, None            # short-circuit: provably inert where legacy works
        if area > best[2]:
            best = (name, raw, area)
    if best[1] is not None:
        ENSEMBLE_STATS["rescued"] += 1
        ENSEMBLE_STATS["by_channel"][best[0]] = ENSEMBLE_STATS["by_channel"].get(best[0], 0) + 1
        return best[1], None
    # Nothing anywhere is cross-shaped. Distinguish the two reasons that can happen:
    #   (a) overfill/close range -- the cross is real but its own bbox is no longer
    #       cross-shaped. The legacy mask is still the right input; keep today's
    #       behaviour (fall through, _isolate_marker_by_shape will pass it unfiltered).
    #   (b) the mask is a SOLID region (the inv plate). That cannot be a marker at any
    #       scale, and passing it on is what makes inv fail SILENTLY. Fail loudly.
    ENSEMBLE_STATS["no_cross"] += 1
    m = _reject_blobby_components(cv2.morphologyEx(legacy_raw, cv2.MORPH_CLOSE, k3))
    n, _lbl, st, _ = cv2.connectedComponentsWithStats(m, connectivity=8)
    if n > 1:
        areas = st[1:, cv2.CC_STAT_AREA]
        b = 1 + int(np.argmax(areas))
        fill = st[b, cv2.CC_STAT_AREA] / max(st[b, cv2.CC_STAT_WIDTH] * st[b, cv2.CC_STAT_HEIGHT], 1)
        if fill > _ENS_SOLID_FILL_MAX:
            return legacy_raw, 'no_cross_shaped_component'
    return legacy_raw, None


def _detect_core(frame_bgr, lower=DEFAULT_LOWER, upper=DEFAULT_UPPER,
                  min_line_length=None, max_line_gap=None, identify_stub=True,
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
    if min_line_length is None: min_line_length = HOUGH_MIN_LINE_LEN
    if max_line_gap is None: max_line_gap = HOUGH_MAX_LINE_GAP
    if _GATE_MODE == "ensemble":
        mask, _ens_reason = _ensemble_gate_mask(frame_bgr, lower, upper)
        if _ens_reason is not None:
            _pxd = int(np.sum(mask > 0))
            HOUGH_DIAG_LOG.append({'mask_px': _pxd, 'bbox': None, 'n_edge_px': 0, 'n_lines': 0,
                                   'stage_px': (_pxd,)*5, 'reason': _ens_reason})
            return CrossMarkerDetection(None, None, False, fail_reason=_ens_reason)
    elif _GATE_MODE == "contrast":
        mask, _adapt_reason = _contrast_gate_mask(frame_bgr)
        if _adapt_reason is not None:
            _pxd = int(np.sum(mask > 0))
            HOUGH_DIAG_LOG.append({'mask_px': _pxd, 'bbox': None, 'n_edge_px': 0, 'n_lines': 0,
                                   'stage_px': (_pxd,)*5, 'reason': _adapt_reason})
            return CrossMarkerDetection(None, None, False, fail_reason=_adapt_reason)
    elif _ADAPT_GATE:
        mask, _adapt_reason = _adaptive_gate_mask(frame_bgr)
        if _adapt_reason is not None:
            _pxd = int(np.sum(mask > 0))
            HOUGH_DIAG_LOG.append({
                'mask_px': _pxd, 'bbox': None, 'n_edge_px': 0, 'n_lines': 0,
                'stage_px': (_pxd, _pxd, _pxd, _pxd, _pxd), 'reason': _adapt_reason,
            })
            return CrossMarkerDetection(None, None, False, fail_reason=_adapt_reason)
    else:
        mask = cv2.inRange(cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV), lower, upper)
    _px_raw = int(np.sum(mask > 0))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    _px_close = int(np.sum(mask > 0))
    # Snapshot for the ring confirm, which needs the context the next stages strip.
    _mask_close = mask.copy() if RING_CONFIRM else None
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

    _hough_in = mask
    if HOUGH_MASK_DILATE_PX > 0:
        _k = 2 * HOUGH_MASK_DILATE_PX + 1
        _hough_in = cv2.dilate(mask, np.ones((_k, _k), np.uint8))
    edges = cv2.Canny(_hough_in, 50, 150, apertureSize=3)
    lines = cv2.HoughLinesP(edges, rho=1, theta=np.pi / 180, threshold=HOUGH_THRESHOLD,
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
    if _STROKE_VALIDATE:
        # Reject plate/shadow EDGES (steps) before they can be clustered as cross arms.
        _gray_sv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        _sv_keep = _validate_stroke_segments(_gray_sv, segs)
        if _sv_keep.any():
            segs = segs[_sv_keep]
        # if NOTHING survives, fall through with the unfiltered set rather than dying --
        # the downstream geometry gates still have to agree.
    # SEGMENT CAP (2026-09-01): the adaptive contrast gate admits more real edge
    # structure (ground texture near the marker) than the old near-black inRange,
    # which inflates the O(n) angle loop + the pairwise corner-join below. The
    # cross arms are always among the LONGEST segments; short texture slivers are
    # not -- keep the longest CROSS_SEG_CAP and bound the cost. Generous default
    # (a real cross yields <~30 segments); only bites on pathological frames.
    if len(segs) > _SEG_CAP:
        _slen = (segs[:, 0] - segs[:, 2]) ** 2 + (segs[:, 1] - segs[:, 3]) ** 2
        segs = segs[np.argsort(_slen)[::-1][:_SEG_CAP]]
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
    # partner was mostly out of frame.
    #
    # ⛔ 2026-08-01's FIX for that was a HARD two-phase gate (support-required search,
    # falling back to unfiltered only if NOTHING passes) -- and it had its own bug,
    # found 2026-09-04 (user challenge on why a "longer arm only" balance-confirm
    # heuristic was needed at all, traced back to HERE): a hard gate that returns the
    # first support-passing pair, however bad its angle, NEVER compares it against a
    # much-better pair that only narrowly missed the support bar. This marker is an X
    # (two through-center diagonal arms, ~90 deg apart) plus a separate ONE-SIDED stub
    # at 45 deg from each diagonal (STUB_REL_ANGLE_DEG) -- so whenever one diagonal's
    # Hough segments happened to fall one below MIN_CLUSTER_SUPPORT, the hard gate
    # would reject the near-perfect (|rel-90|~0.6 deg) diagonal pair and commit to a
    # fully-supported but ~45 deg-wrong stub pair instead. Measured on clean `base`
    # frames: happened in 12.5% of ALL frames (41.6% of a downstream balance-test's
    # false positives were caused by exactly this).
    #
    # FIX: a single SOFT-PENALIZED search across every pair (no hard gate, no two-
    # phase fallback) -- support becomes a tiebreaker, not a veto, so it can no longer
    # override a much better angle fit, while still doing its original job of keeping
    # a spurious near-90 noise pair from beating a real, mostly-off-frame arm.
    # PAIR_SUPPORT_PENALTY (degrees-equivalent per missing support point below
    # MIN_CLUSTER_SUPPORT, summed over both clusters) is deliberately between the two
    # scales the two failure modes operate at: the stub-vs-diagonal gap is ~44-45 deg
    # (so even a 2-point support deficit, ~2x penalty, must lose to it) while the
    # original noise-pair problem's real arm was only "a few degrees" off 90 (so a
    # 1-point deficit must still beat a fully-unsupported near-0 noise pair, whose
    # own 2-point deficit costs twice as much). Verified offline (RobustnessFrameset,
    # replaying every candidate pair captured per frame): eliminates the bug's
    # stub-picks on 5/7 scenes entirely (base/bright/darkbg/lowsun 100%, dim/col/inv
    # partially -- their residual stub-picks have NO near-90 pair available that
    # frame at all, a Hough-recall problem this scoring change can't fix), ZERO
    # regressions (no frame where a previously-good pick became a stub pick) across
    # all 7 scenes, and the result is insensitive to the exact penalty value (tested
    # 5.0-15.0, identical outcome) -- not a fragile hand-tune.
    PAIR_SUPPORT_PENALTY = float(os.environ.get("CROSS_PAIR_SUPPORT_PENALTY", "8.0"))

    def _best_pair():
        best, best_score = None, 1e18
        for ii in range(len(clusters)):
            for jj in range(ii + 1, len(clusters)):
                err = abs(_circ_diff(np.mean(clusters[ii]), np.mean(clusters[jj])) - 90.0)
                deficit = (max(0, MIN_CLUSTER_SUPPORT - len(clusters[ii]))
                           + max(0, MIN_CLUSTER_SUPPORT - len(clusters[jj])))
                score = err + PAIR_SUPPORT_PENALTY * deficit
                if score < best_score:
                    best_score, best = score, (ii, jj)
        return best

    best_pair = _best_pair()
    if os.environ.get("CROSS_DIAG_PAIR_SELECT") == "1" and best_pair is not None:
        _ii, _jj = best_pair
        _ra = _circ_diff(np.mean(clusters[_ii]), np.mean(clusters[_jj]))
        PAIR_SELECT_DIAG.append({
            'picked_rel': float(_ra),
            'n_clusters': len(clusters),
            'cluster_means': [float(np.mean(c)) for c in clusters],
            'cluster_support': [len(c) for c in clusters],
            'all_pairs': [(float(np.mean(clusters[a])), float(np.mean(clusters[b])),
                           float(_circ_diff(np.mean(clusters[a]), np.mean(clusters[b]))),
                           len(clusters[a]), len(clusters[b]))
                          for a in range(len(clusters)) for b in range(a+1, len(clusters))],
        })
    if best_pair is None:
        return CrossMarkerDetection(None, None, False, bbox, fail_reason='no_pair_found')
    i, j = best_pair
    ang_i, ang_j = np.mean(clusters[i]), np.mean(clusters[j])
    if _circ_diff(ang_i, ang_j) < MIN_INTER_LINE_ANGLE_DEG:
        return CrossMarkerDetection(None, None, False, bbox, fail_reason='near_parallel_pair')

    # ─── ORIGIN-FREE arm-pixel selection (2026-08-31) ──────────────────────────────
    # PREVIOUS approach (removed): selected each arm's mask pixels by their ANGULAR
    # BEARING FROM THE BBOX CENTRE, keeping pixels within +/-10 deg of the cluster's
    # Hough angle. That is circular -- it uses a guessed centre (bbox centre) to find
    # the pixels whose line fit produces the real centre -- and bbox centre is a poor
    # junction estimate whenever the cross is asymmetric (the stub), partly clipped,
    # perspective-skewed, or (fatally) overfilling the frame: the arm pixels then fan
    # out over a wide bearing range from the wrong origin, the +/-10 deg gate discards
    # most of them, and detect() bailed with insufficient_fit_points even with two
    # clean full arms in view (traced on a real perception-mode landing at alt 0.73 m:
    # `s` froze -> ~0.5 m of blind lateral drift -> 1.75 m miss).
    #
    # NEW: "which pixels lie on this arm" is a PERPENDICULAR-DISTANCE question and needs
    # no reference point. Per angle cluster, build a representative line straight from
    # that cluster's own Hough segments (total-least-squares fit of their endpoints ->
    # direction + offset), then select full-mask pixels within a perpendicular band of
    # that line. Origin-free, still symmetric about the true centreline (the property
    # the old angle-from-centre selection was reaching for), and identical whether the
    # true junction is on- or off-frame -- which is exactly what feeds the existing
    # off-frame (`in_fov=False`) extrapolation path below.
    bx0, by0, bw0, bh0 = bbox
    _band = float(np.clip(0.05 * max(bw0, bh0, 1), 5.0, 35.0))   # perp half-width; _robust_fit_line prunes residual outliers
    _cluster_means = [float(np.mean(c)) for c in clusters]
    # assign every surviving Hough segment to its nearest angle cluster
    _seg_of_cluster = [[] for _ in clusters]
    for _m, _a in enumerate(angles):
        _k = int(np.argmin([_circ_diff(_a, cm) for cm in _cluster_means]))
        _seg_of_cluster[_k].append(segs[_m])

    def _rep_line(cluster_idx):
        """(vx, vy, x0, y0): the cluster's representative line. DIRECTION comes from
        the cluster-mean Hough angle -- an average over every Hough re-detection of
        that arm, so it is stable even when only a few short segments survive. The
        POINT is the centroid of the member segments' endpoints, which sits on/near
        the arm centreline (any residual half-stroke bias is well inside the
        selection band, and _robust_fit_line converges to the true centreline from
        the banded pixels anyway). Origin-free -- no junction guess."""
        _th = np.radians(_cluster_means[cluster_idx] if cluster_idx < len(_cluster_means) else 0.0)
        _ss = _seg_of_cluster[cluster_idx] if cluster_idx < len(_seg_of_cluster) else []
        if _ss:
            _ep = np.asarray([[s[0], s[1]] for s in _ss] + [[s[2], s[3]] for s in _ss], dtype=np.float64)
            _c = _ep.mean(axis=0)
        else:
            _c = np.array([bx0 + bw0 / 2.0, by0 + bh0 / 2.0])
        return float(np.cos(_th)), float(np.sin(_th)), float(_c[0]), float(_c[1])

    _rep = {i: _rep_line(i), j: _rep_line(j)}
    # provisional junction from the two representative lines -- used ONLY to carve out
    # the arm-overlap blob (a pixel there is within-band of BOTH arms and would bias
    # both fits toward the crossing). No effect on the final answer.
    _prov = _line_intersection(_rep[i], _rep[j])
    _min_radius = 0.10 * max(bw0, bh0, 1)
    _xs_f = xs.astype(np.float64)
    _ys_f = ys.astype(np.float64)

    def _cluster_points_from_mask(cluster_idx):
        vx, vy, x0, y0 = _rep.get(cluster_idx) or _rep_line(cluster_idx)
        _n = np.hypot(vx, vy) or 1.0
        vx, vy = vx / _n, vy / _n
        perp = np.abs((_xs_f - x0) * vy - (_ys_f - y0) * vx)   # signed-free perp distance to the line
        sel = perp <= _band
        if _prov is not None:
            sel &= np.hypot(_xs_f - _prov[0], _ys_f - _prov[1]) > _min_radius
        return list(zip(xs[sel].tolist(), ys[sel].tolist()))

    pts_i, pts_j = _cluster_points_from_mask(i), _cluster_points_from_mask(j)
    if len(pts_i) < MIN_FIT_POINTS or len(pts_j) < MIN_FIT_POINTS:
        # See MIN_FIT_POINTS' 2026-08-03 comment: a fit through too few points is
        # unconstrained regardless of what angle it happens to land on.
        return CrossMarkerDetection(None, None, False, bbox, fail_reason='insufficient_fit_points')

    vx_i, vy_i, x0_i, y0_i, mask_i = _robust_fit_line(pts_i)
    vx_j, vy_j, x0_j, y0_j, mask_j = _robust_fit_line(pts_j)
    line_i, line_j = (vx_i, vy_i, x0_i, y0_i), (vx_j, vy_j, x0_j, y0_j)
    # INTENSITY-WEIGHTED SUB-PIXEL CENTERLINE RE-FIT (see module top). Replaces the two
    # binary-mask Huber fits with greyscale centerline fits so a half-pixel edge shift
    # moves each arm's SLOPE a fraction of a pixel instead of toggling whole rows. Both
    # arms must succeed, else keep the _robust_fit_line pair (the pruning + near-parallel
    # + centroid_mismatch gates below all still run on whatever pair is chosen).
    if _SUBPIX_CENTERLINE:
        _g_cl = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        _cl_i = _fit_arm_centerline_subpix(_g_cl, pts_i, line_i)
        _cl_j = _fit_arm_centerline_subpix(_g_cl, pts_j, line_j)
        if _cl_i is not None and _cl_j is not None:
            line_i, line_j = _cl_i, _cl_j
            CENTERLINE_STATS["applied"] += 1
        else:
            CENTERLINE_STATS["fallback"] += 1
    # ROBUST-SELECTED points (2026-08-24): _robust_fit_line's inlier mask drops
    # perpendicular outliers to the converged fit -- e.g. contamination that
    # passed the coarse perpendicular-band gate above (_cluster_points_from_mask)
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

    # SUB-PIXEL JUNCTION REFINE (see module top). Only when the junction is actually
    # in the frame; the refined center then flows through the centroid_mismatch /
    # bbox-margin sanity checks below, so a bad refine can only be rejected, never
    # sneak a worse center through.
    if _SUBPIX_JUNCTION and in_fov:
        _hw = int(np.clip(round(0.15 * max(bw, bh)), 4, 15))
        _ref = _refine_junction_subpix(cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY),
                                        center[0], center[1], _hw)
        if _ref is None:
            SUBPIX_STATS["rej_no_fit"] += 1
        else:
            _sh = float(np.hypot(_ref[0] - center[0], _ref[1] - center[1]))
            if _sh <= _SUBPIX_MAX_SHIFT_FRAC * max(bw, bh, 1):
                center = _ref
                SUBPIX_STATS["applied"] += 1
                SUBPIX_STATS["shift_sum"] += _sh
            else:
                SUBPIX_STATS["rej_shift"] += 1

    # STAGE 3 (see GEOM_CONFIRM at module top) -- on the FINAL center, so the
    # sub-pixel refine above is included in what gets confirmed.
    if GEOM_CONFIRM:
        _g_ok, _g_reason = _confirm_cross_geometry(pts_i, pts_j, line_i, line_j, center)
        if not _g_ok:
            return CrossMarkerDetection(None, None, False, bbox, fail_reason=_g_reason)

    if RING_CONFIRM and _mask_close is not None:
        _r_ok, _r_reason, _r_n = _confirm_ring(_mask_close, center, bbox)
        if not _r_ok:
            return CrossMarkerDetection(None, None, False, bbox, fail_reason=_r_reason)

    if BALANCE_CONFIRM:
        _b_ok, _b_reason, _b_d = _confirm_balance(pts_i, pts_j, center)
        if not _b_ok:
            return CrossMarkerDetection(None, None, False, bbox, fail_reason=_b_reason)

    if in_fov:
        centroid_x, centroid_y = float(xs.mean()), float(ys.mean())
        centroid_err = np.hypot(center[0] - centroid_x, center[1] - centroid_y)
        # The visible-pixel centroid is only a valid junction proxy while the WHOLE
        # cross is in frame. Once the marker overfills (arms clipped by the edges,
        # stub making it asymmetric) the centroid drifts off the true junction, so a
        # correct off-centre intersection would be rejected here for no good reason --
        # the same wrong assumption the old angle-from-bbox-centre pixel selection
        # made (2026-08-31). Ramp the tolerance from tight (0.12) toward the bbox-
        # margin check as the marker fills the frame; the two robust line fits already
        # cleared MIN_FIT_POINTS + the near-parallel gates, so support is not in doubt.
        _fill = max(bw, bh, 1) / max(min(frame_h, frame_w), 1)
        _centroid_tol_frac = 0.12 + 0.60 * float(np.clip((_fill - 0.6) / 0.4, 0.0, 1.0))
        max_centroid_err = _centroid_tol_frac * max(bw, bh, 1)
        if GEOM_CONFIRM == 2:
            # Stage 3 REPLACES this proxy (mode 2). Neutralise it here rather than
            # skipping the branch, so the bbox-margin check below and the off-frame
            # `else` path keep running untouched.
            max_centroid_err = float('inf')
        if centroid_err > max_centroid_err:
            # SPAN RESCUE (see CENTROID_SPAN_RESCUE at module top): the mask centroid
            # can be contaminated by structure fused into the same component, so ask
            # instead whether the intersection lies ON both fitted arms. Only reached
            # when the legacy check has already failed -> clean scenes are unaffected.
            _rescued = False
            SPAN_RESCUE_STATS["consulted"] += 1
            _big = _fill >= CENTROID_SPAN_FILL_MAX   # overfilling -> terminal regime
            if _big:
                SPAN_RESCUE_STATS["consulted_lowalt"] += 1
            if CENTROID_SPAN_RESCUE and not _big:
                _rescued = True
                for _p, _ln in ((np.asarray(pts_i, float), line_i),
                                (np.asarray(pts_j, float), line_j)):
                    _v = np.array([_ln[0], _ln[1]], float)
                    _nv = float(np.linalg.norm(_v))
                    if _nv < 1e-9 or len(_p) < 4:
                        continue          # too few points to bound a span; don't veto on it
                    _v = _v / _nv
                    _mu = _p.mean(axis=0)
                    _t = (_p - _mu) @ _v
                    _tc = float((np.asarray(center, float) - _mu) @ _v)
                    _lo, _hi = float(_t.min()), float(_t.max())
                    _mar = CENTROID_SPAN_MARGIN * (_hi - _lo)   # junction may sit near an arm end
                    if not (_lo - _mar <= _tc <= _hi + _mar):
                        _rescued = False
            if _rescued:
                SPAN_RESCUE_STATS["rescued"] += 1
                if _big:
                    SPAN_RESCUE_STATS["rescued_lowalt"] += 1
            else:
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
                # arm-line pruning above -- the coarse perpendicular-band gate
                # (_cluster_points_from_mask) only checks distance to the cluster's
                # representative line, not that the points form a single coherent
                # stroke, so a contamination source lying near that line (e.g. the
                # drone's own shadow falling across the stub near touchdown -- the
                # motivating case for this fix) can leak into pts_k and, being far
                # from center, dominates the
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


WHOLE_PLATE_FLOW = os.environ.get("CROSS_WHOLE_PLATE_FLOW", "0") == "1"
WHOLE_PLATE_SCALE = float(os.environ.get("CROSS_WHOLE_PLATE_SCALE", "1.3"))


def extent_mask_from_detection(det, frame_shape):
    """VISUAL-VERIFICATION-ONLY, NOT YET WIRED INTO THE LIVE FLOW SOLVE (2026-08-27).
    REPLACES the earlier plate_mask_from_detection() attempt (cv2.minAreaRect on the
    isolated line mask) -- that was REJECTED after a visual check against real frames:
    it fit a rectangle to the X shape's OWN rotation, which only coincidentally matches
    the physical plate's true edges (the plate is a square; the arms run along its
    DIAGONALS, not parallel to its sides), so the estimate was inconsistently
    misaligned frame to frame -- sometimes fine, sometimes spilling off two corners
    onto the ground while missing real plate area on the other two.

    Also a DELIBERATE reframing per user correction (2026-08-27): this is not about
    finding "the plate" at all (that's a sim-only concept -- a real cross painted on
    concrete/grass/dirt has no separate bounded plate). It's about using the marker's
    own CURRENT APPARENT EXTENT in this frame as an axis-aligned region to search for
    trackable texture in, mirroring img_data.py's ArUco ring-flow design intent
    (sample broadly around the target, scale-free, sized to its own current apparent
    size) rather than restricting to a thin band around the drawn lines.

    Approach: axis-aligned box centered on det.mask_bbox's own center, sized to
    WHOLE_PLATE_SCALE (default 1.3) times mask_bbox's own (w,h) -- NOT rotated to the
    cross's fitted line angle. An axis-aligned box around a rotated square is looser
    at the corners than a precisely-oriented one, but that's a much safer failure
    direction than the previous approach's 45-degree full misalignment: worst case it
    admits a modest amount of extra background near its own corners, it does not
    systematically miss real plate area on two whole sides. Still a GUESS scale, not
    derived from the texture's known cross-to-plate ratio -- MUST be checked visually
    against real frames (various altitudes/angles) before trusting it. Returns None if
    mask_bbox is missing.
    """
    if det is None or det.mask_bbox is None:
        return None
    bx, by, bw, bh = det.mask_bbox
    cx, cy = bx + bw / 2.0, by + bh / 2.0
    hw, hh = (bw * WHOLE_PLATE_SCALE) / 2.0, (bh * WHOLE_PLATE_SCALE) / 2.0
    fh, fw = frame_shape[:2]
    x0 = int(np.clip(cx - hw, 0, fw))
    x1 = int(np.clip(cx + hw, 0, fw))
    y0 = int(np.clip(cy - hh, 0, fh))
    y1 = int(np.clip(cy + hh, 0, fh))
    mask = np.zeros(frame_shape[:2], dtype=np.uint8)
    mask[y0:y1, x0:x1] = 255
    return mask


LINE_EXCLUDE_DILATE_PX = int(os.environ.get("CROSS_LINE_EXCLUDE_DILATE_PX", "3"))


def background_mask_from_detection(det, frame_shape):
    """VISUAL-VERIFICATION-ONLY, NOT YET WIRED INTO THE LIVE FLOW SOLVE (2026-08-27).
    extent_mask_from_detection() MINUS a dilated version of det.isolated_mask --
    i.e. the extent box with the drawn cross/stub lines (and a small margin around
    them) carved OUT, so cv2.goodFeaturesToTrack physically cannot propose a point
    on or near a line edge.

    WHY THIS MATTERS (found empirically, 2026-08-27): merely widening the search
    mask to extent_mask_from_detection() did NOT make GFT prefer background texture
    -- corners still clustered almost exactly on the line edges in real captured
    frames, because GFT ranks candidates by Shi-Tomasi corner strength, and the
    line edges (a sharp ~180-value jump) are a far stronger signal than background
    speckle (std~10) wherever both are eligible. Excluding the lines entirely fixes
    this two ways: (1) obviously, a line-adjacent point can no longer be proposed at
    all; (2) less obviously, cv2.goodFeaturesToTrack's qualityLevel threshold is
    relative to the STRONGEST corner found WITHIN the searched region -- once line
    corners are excluded, that threshold auto-adapts to whatever the background
    actually offers, instead of always being judged against an unbeatable line
    gradient elsewhere in the same mask. This should generalize across texture
    strength, not just the one grain size tested so far.

    LINE_EXCLUDE_DILATE_PX (default 3) sets how much margin around the raw line
    mask gets carved out, on top of MASK_DILATE_PX's own dilation already baked
    into det.isolated_mask -- keeps candidates comfortably clear of the line's own
    anti-aliased edge pixels, not just off the binary mask by one pixel.
    """
    ext = extent_mask_from_detection(det, frame_shape)
    if ext is None or det.isolated_mask is None:
        return ext
    k = 2 * LINE_EXCLUDE_DILATE_PX + 1
    line_dilated = cv2.dilate(det.isolated_mask, np.ones((k, k), np.uint8))
    return cv2.bitwise_and(ext, cv2.bitwise_not(line_dilated))


LINE_EXCLUDE_BLACK_MAX_V = int(os.environ.get("CROSS_LINE_EXCLUDE_BLACK_MAX_V", "90"))


def background_mask_bboxonly_from_detection(det, gray):
    """Like background_mask_from_detection(), but derives the line-exclusion mask
    from a plain near-black threshold on `gray` instead of det.isolated_mask --
    so it works on frames where the Hough line-fit FAILED (det.ok is False) but
    the colour-gated blob + det.mask_bbox are still solid. That set is ~70% of
    otherwise-discarded frames, and it is disproportionately the near-touchdown
    marker-overflow phase where h_z (loom) matters most for the descent axis
    (validated 2026-08-27, tools/validate_bgflow_corr.py -- see
    project_20260827_framerate_and_h_texture_investigation memory).

    NOT a drop-in for the det.ok path: the threshold mask is coarser than the
    shape-isolated one, and the extent box near touchdown is close to the whole
    frame, so callers MUST gate the resulting solve on fit quality (rel_resid)
    before committing it -- on a texture whose grain is below the camera's
    delivered resolution this path produces near-zero noise at best and a
    sign-flipped h_z at worst without that gate.
    """
    if det is None or det.mask_bbox is None:
        return None
    ext = extent_mask_from_detection(det, gray.shape)
    if ext is None or not ext.any():
        return None
    black = (gray < LINE_EXCLUDE_BLACK_MAX_V).astype(np.uint8) * 255
    k = 2 * LINE_EXCLUDE_DILATE_PX + 1
    black = cv2.dilate(black, np.ones((k, k), np.uint8))
    return cv2.bitwise_and(ext, cv2.bitwise_not(black))


MULTISCALE_LEVELS = [int(x) for x in os.environ.get("CROSS_MULTISCALE_LEVELS", "1,2,4").split(",")]
# Performance caps (2026-08-27, process_frame >=30Hz work). goodFeaturesToTrack
# cost scales with the searched image AREA and with how many candidate corners
# clear qualityLevel (it sorts them all) -- both blow up when the marker fills
# the frame near touchdown (measured: multiscale GFT 2.5ms mid-descent ->
# 37ms/frame near the deck). GFT_WORK_MAX_PX bounds the working resolution
# (crop to the mask bbox, then downscale so its long side fits this budget);
# GFT_QUALITY raised 0.01->0.03 so far fewer weak candidates are found/sorted.
GFT_WORK_MAX_PX = int(os.environ.get("CROSS_GFT_WORK_MAX_PX", "200"))
GFT_QUALITY = float(os.environ.get("CROSS_GFT_QUALITY", "0.03"))


def _grid_dedup(pts, lvls, min_dist):
    """O(n) replacement for the old O(n^2) pairwise de-dup: bucket points into
    min_dist-sized grid cells, keep the first (already scale-1-first ordered)
    point per cell. Not identical to true radial de-dup at cell boundaries, but
    close enough for flow-point seeding and vastly cheaper when GFT returns
    hundreds-to-thousands of candidates."""
    order = np.argsort(lvls, kind="stable")
    seen, kept = set(), []
    inv = 1.0 / max(1, min_dist)
    for i in order:
        x, y = pts[i]
        key = (int(x * inv), int(y * inv))
        if key not in seen:
            seen.add(key)
            kept.append((x, y))
    return np.array(kept, dtype=np.float32).reshape(-1, 1, 2) if kept else None


def multiscale_good_features(gray, mask, max_corners=80, quality=None, min_dist=4):
    """Runs cv2.goodFeaturesToTrack at each downsample factor in MULTISCALE_LEVELS
    (default 1x/2x/4x) within `mask`'s region, maps surviving points back to
    full-resolution coordinates and pools them (grid-deduplicated by min_dist).
    Rationale: a single-scale detector can only resolve texture whose spatial
    frequency matches its window -- real surfaces won't cooperate with one fixed
    scale (confirmed empirically: coarse grain registered at 1x, fine grain
    needed downsampling before GFT found anything). Pooling across scales means
    SOME level registers real corners regardless of grain size. Coarse-scale
    points carry proportionally larger position uncertainty -- callers should not
    treat scale-1 and scale-4 points as equally precise.

    Marker-size-independent cost (2026-08-27): the work is done on the mask's
    bounding box, downscaled so its long side is <= GFT_WORK_MAX_PX, then points
    are mapped back (origin offset + scale). Wired live via
    cross_marker_perception._compute_hw_bgflow.
    """
    if quality is None:
        quality = GFT_QUALITY
    ys, xs = np.nonzero(mask)
    if len(xs) == 0:
        return None
    x0, x1 = int(xs.min()), int(xs.max()) + 1
    y0, y1 = int(ys.min()), int(ys.max()) + 1
    g_roi = gray[y0:y1, x0:x1]
    m_roi = mask[y0:y1, x0:x1]
    rh, rw = g_roi.shape[:2]
    pre = 1
    if max(rh, rw) > GFT_WORK_MAX_PX:
        pre = int(np.ceil(max(rh, rw) / GFT_WORK_MAX_PX))
        g_roi = cv2.resize(g_roi, (max(1, rw // pre), max(1, rh // pre)), interpolation=cv2.INTER_AREA)
        m_roi = cv2.resize(m_roi, (g_roi.shape[1], g_roi.shape[0]), interpolation=cv2.INTER_NEAREST)
    h, w = g_roi.shape[:2]
    all_pts, all_lvls = [], []
    for lvl in sorted(set(MULTISCALE_LEVELS)):
        if lvl <= 1:
            g, m = g_roi, m_roi
        else:
            g = cv2.resize(g_roi, (max(1, w // lvl), max(1, h // lvl)), interpolation=cv2.INTER_AREA)
            m = cv2.resize(m_roi, (g.shape[1], g.shape[0]), interpolation=cv2.INTER_NEAREST)
        if not m.any():
            continue
        pts = cv2.goodFeaturesToTrack(g, maxCorners=max_corners, qualityLevel=quality,
                                       minDistance=max(1, min_dist // lvl), mask=m)
        if pts is None:
            continue
        # map back: sub-scale -> ROI-work-scale -> ROI -> full frame
        pts = pts.reshape(-1, 2) * lvl * pre + np.array([x0, y0], dtype=np.float32)
        all_pts.append(pts)
        all_lvls.extend([lvl] * len(pts))
    if not all_pts:
        return None
    return _grid_dedup(np.concatenate(all_pts, axis=0), np.array(all_lvls), min_dist)


DETECT_WORK_MAX_PX = int(os.environ.get("CROSS_DETECT_WORK_MAX_PX", "200"))


def _scale_detection(det, f, crop_shape):
    """Scale a CrossMarkerDetection computed on a frame downscaled by integer
    factor `f` back up to `crop_shape` (h, w) coordinates. Inverse of the
    pre-downscale in detect()'s tracked-crop fast path. isolated_mask is
    NEAREST-resized back to crop_shape so the subsequent _shift_detection can
    embed it into the full frame as usual."""
    if not det.ok or f == 1:
        return det
    c = (det.center[0] * f, det.center[1] * f) if det.center else None
    bb = ((det.mask_bbox[0] * f, det.mask_bbox[1] * f,
           det.mask_bbox[2] * f, det.mask_bbox[3] * f) if det.mask_bbox else None)
    li = tuple(map(tuple, np.asarray(det.line_points_i, float) * f)) if det.line_points_i else ()
    lj = tuple(map(tuple, np.asarray(det.line_points_j, float) * f)) if det.line_points_j else ()
    sp = (tuple(map(tuple, np.asarray(det.stub_points, float) * f))
          if det.stub_points else None)
    im = det.isolated_mask
    if im is not None:
        im = cv2.resize(im, (crop_shape[1], crop_shape[0]), interpolation=cv2.INTER_NEAREST)
    return replace(det, center=c, mask_bbox=bb, line_points_i=li, line_points_j=lj,
                   stub_points=sp, isolated_mask=im)


def _detect_core_capped(frame, *args, work_max_px=DETECT_WORK_MAX_PX):
    """_detect_core, but when `frame`'s long side exceeds work_max_px, run on an
    integer-downscaled copy and scale the result back. _detect_core's cost is
    ~quadratic in marker pixel size (mask pixels + Hough segments), which spikes
    to 90-190ms when the marker fills the frame near touchdown (measured); the
    line geometry it recovers is scale-invariant, so a bounded working resolution
    keeps detect() near-constant-cost without changing what it computes. Only the
    tracked-crop fast path uses this -- the (rarer, acquisition) full-frame path
    is left exact."""
    h, w = frame.shape[:2]
    f = 1
    if max(h, w) > work_max_px:
        f = int(np.ceil(max(h, w) / work_max_px))
        frame = cv2.resize(frame, (max(1, w // f), max(1, h // f)), interpolation=cv2.INTER_AREA)
    det = _detect_core(frame, *args)
    return _scale_detection(det, f, (h, w))


def detect(frame_bgr, lower=DEFAULT_LOWER, upper=DEFAULT_UPPER,
           min_line_length=None, max_line_gap=None, identify_stub=True,
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
            det = _detect_core_capped(crop, lower, upper, min_line_length, max_line_gap,
                                identify_stub, 1.0, 1.0)   # no static
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
    # With the adaptive contrast gate (_ADAPT_GATE), the acquisition path's mask is
    # denser (more real edges admitted) -> Canny/Hough cost ~1.5x the legacy inRange.
    # Route it through the same bounded-working-resolution wrapper the tracked-crop
    # path already uses (line geometry is scale-invariant) so full-frame acquisition
    # stays comfortably above the 30 Hz process_frame floor. Legacy gate -> exact old
    # path unchanged.
    if _ADAPT_GATE:
        det = _detect_core_capped(frame_bgr, lower, upper, min_line_length, max_line_gap,
                                  identify_stub, roi_frac_x, roi_frac_y)
    else:
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
