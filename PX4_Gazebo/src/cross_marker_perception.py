"""Standalone perception pipeline for the cross+stub fiducial -- NOT routed
through IMG_PROCESSOR/img_data.py's ArUco-shaped machinery (no synthetic
4-corner packaging, no PlanarFeatureMap rescue, no marker handover). Those
subsystems exist to patch specific ArUco weaknesses (decode-or-nothing
dropouts; multi-marker nested boards needed for decode range) that the cross
marker's line-geometry-based, single-marker, partial-credit-native detection
doesn't have -- see docs/cross_marker.pdf and the design discussion history.

Computes the manuscript's actual image-feature quantities (manuscript.tex,
Sec. "Image Features & Optical Flow"), NOT the project's copied
ArUco-specific conventions:
  s     = homogeneous image position [xc, yc, 1] -- the line-intersection
          point directly (cross_marker_detector.detect()), no corner
          averaging detour.
  alpha = unweighted 2nd-moment principal angle over the REAL detected
          arm/stub pixels (no ArUco-style artificial corner weighting --
          the stub's asymmetry is real, not synthesized). Hold-last-good
          when the stub cluster isn't found this frame (no separate
          observability-metric gate -- yaw control tolerates interruption).
  h, w  = optical flow (image velocity / image angular velocity), recovered
          via the SAME image-Jacobian pseudo-inverse solve used for ArUco
          corners (img_data.py's _fill_A, mirrored here), but fed
          Shi-Tomasi points sampled across the WHOLE color/shape-gated
          marker plate (drawn cross+stub AND the surrounding texture) --
          not just the 4 corners of a synthetic quad. This fixes the two
          documented ArUco flow problems instead of patching around them:
          point starvation (a flat ArUco marker has ~4-8 resolvable
          corners; GFT regressed there -- see img_data.py:2018-2021) and
          Jacobian ill-conditioning at small (x,y) (img_data.py:1944-1947)
          -- the textured plate gives many REAL, well-spread trackable
          points instead of 180 synthetic points interpolated from 4 real
          ones.

No KLT tracking of the intersection point itself (see design discussion --
the feature KF's predict step already provides the smoothing/coasting a
dedicated tracker would duplicate; a near-identical mechanism was built and
then removed as redundant for ArUco, 2026-07-17).
"""
import os
import time
from collections import deque
import queue
from threading import Thread

import cv2
import numpy as np
from ahrs import Quaternion

import cross_marker_detector as cmd
from img_data import fx, fy   # reuse the project's calibrated intrinsics, don't refork them
from gz_subscriber import GZ_Subscriber, Image_Node   # Image_Node is decode-agnostic raw capture --
                                                        # no ArUco logic in it, safe to reuse as-is

# 2026-08-03 (s_e_n single-frame-spike investigation): _getVirtualPts's perspective divide
# (V_rays[:,0]/z_v) blows up into a huge or sign-flipped point once a ray's z_v approaches
# or crosses zero (grazing/behind-camera obliqueness under a transient tilt) -- reproduced
# via Z_V_DIAG with min_z_v as low as -1.33 and output up to 41x normal magnitude on the
# CENTROID ray specifically (feeds s/s_e_n). 0.5 is a conservative margin (real, healthy
# frames observed well above this); below it, reject the frame rather than accept a
# perspective-divide artifact -- see process_frame's centroid plausibility gate.
Z_V_MIN_CENTROID = float(os.environ.get("CROSS_Z_V_MIN_CENTROID", "0.5"))

GFT_MAX_CORNERS = int(os.environ.get("CROSS_GFT_MAX_CORNERS", "60"))
GFT_QUALITY = float(os.environ.get("CROSS_GFT_QUALITY", "0.02"))
GFT_MIN_DIST = float(os.environ.get("CROSS_GFT_MIN_DIST", "6"))
# 2026-08-02 (Hx/Hy investigation): the marker's speckle texture measured at
# ~13-18px/blob across the useful altitude range (see
# feedback_duplicated_math_diff_check's sibling investigation), comparable to
# the default 15x15 LK window -- a window that size can straddle multiple
# repeating blobs, an aperture/correspondence-ambiguity risk. Env-overridable
# to test smaller windows without a code edit each time.
_lk_win = int(os.environ.get("CROSS_LK_WIN", "15"))
LK_WIN = (_lk_win, _lk_win)
LK_MAX_LEVEL = 2

# Diagnosed 2026-08-01: at 5m altitude the isolated marker mask is only ~76x76px,
# which has a physical ceiling of ~7 GFT corners regardless of quality/minDistance
# tuning (verified directly -- retuning those knobs changed nothing). With a single
# MIN_FLOW_POINTS cliff at 6, one lost point (LK failure or mask-membership reject)
# was forcing a full resample + a zero-output frame on the very next sample. Two
# separate thresholds replace that cliff:
MIN_FLOW_POINTS_SOLVE = 4     # attempt the lstsq with whatever survived, down to this floor
                               # (2*4=8 >= 6 unknowns -- still solvable, just less overdetermined)
RESAMPLE_TRIGGER = 10          # proactively top up the point pool below this count, but don't
                               # discard current tracking to do it (see _compute_hw)

# 2026-08-07 (Hz/Wz run-to-run variance root-cause): RESAMPLE_TRIGGER alone means
# _sample_flow_points is effectively called ONCE per flight (whenever the pool first
# drops below 10) and the SAME point set is then tracked forward via LK for the rest
# of the flight, since count-only-shrinks-or-tops-up never revisits WHERE the points
# are. Confirmed via Flow Diag Log: 4/5 calibration runs showed a perfectly CONSTANT
# n_kept for their entire ~500-frame z-phase (24, 38, 37, 12 -- zero variation) --
# i.e. no resample happened at all during that whole window; whatever corner subset
# got picked at some earlier, arbitrary moment (near takeoff) just persisted. Hx/Hy
# are largely immune (their raw columns are position-INDEPENDENT constants -- any
# reasonably-tracking point set works), but Hz/Wz's columns are position-WEIGHTED
# (`_fill_A`'s [-x,-y]/[-y,x]), so whichever specific (x,y) locations that one-time
# draw happened to land on determines the WHOLE flight's Hz/Wz bias/noise
# characteristic -- explaining both the large run-to-run spread (each flight's GFT
# seeding moment is subject to small stochastic rendering differences) and why
# single-parameter bisection (color gate, camera Z) couldn't find a clean effect:
# none of those settings touch this mechanism. Fix: force a periodic refresh
# (independent of pool size) so the tracked set gets re-diversified regularly
# instead of being frozen from one early draw. See
# feedback_cross_marker_radial_spread_ceiling memory for the full trace.
RESAMPLE_PERIOD_S = float(os.environ.get("CROSS_RESAMPLE_PERIOD_S", "1.0"))
MASK_DILATE_PX = int(os.environ.get("CROSS_MASK_DILATE_PX", "4"))
                               # dilation radius for BOTH GFT sampling bounds and the post-LK
                               # mask-membership retention check -- a fresh per-frame recomputed
                               # mask jitters by a few px frame-to-frame; a tracked point that's
                               # still physically on the marker but just outside this frame's
                               # exact mask boundary was being discarded as "off-target," which
                               # was avoidable attrition, not a real off-target rejection.
                               # 2026-08-02 (Hx/Hy texture investigation): the color gate
                               # (V<20) only sees the black cross+stub, not the surrounding
                               # textured background -- so this dilation radius is ALSO what
                               # decides how much of that background texture GFT actually gets
                               # to sample. Default 4px was a thin margin; env-overridable to
                               # test a much wider band so a retextured background can matter
                               # for optical flow at all.

# 2026-08-06 (Hz/Wz observability fix, ported from the radial-spread root-cause dig --
# see feedback_cross_marker_radial_spread_ceiling memory): unbiased GFT over the whole
# marker mask lets the cross's central intersection -- the single strongest Shi-Tomasi
# corner in the shape -- dominate the point pool every frame. Measured (Radial Diag Log,
# a real calibration recording): tracked points reach ~0.26 normalized radius at their
# MAX but only ~0.05 on AVERAGE -- i.e. a far corner is occasionally found, but rarely
# selected/kept. The flow Jacobian's Hz/Wz/Wx/Wy columns are all (x,y)-dependent (see
# _fill_A), so this centroid-clustering starves exactly those rows of real signal while
# leaving Hx/Hy (position-independent columns) unaffected -- matches the observed R^2
# asymmetry. Fix: exclude a central disk (sized relative to the mask's OWN current
# extent, so it scales with altitude) from the GFT mask, biasing candidate corners
# toward the arm/stub tips; fall back to the unbiased mask if too few peripheral
# corners survive (e.g. marker small/far, altitude high).
#
# 2026-08-10 (raw-signal-regression bisection, see HANDOVER_cross_marker_hz_
# signflip_20260809.md and project_cross_marker_hz_regression_bisection_20260810
# memory): 0.55 TRIED AND REVERTED. A narrow z-phase-only raw Tz-vs-GT
# correlation check looked promising (0.35->0.55 raised the mean 0.804->0.835,
# n=5) -- but the actual whole-flight joint calibration fit (derive_cross_
# marker_cal.py, which uses ALL phases, not just z) showed 0.55 is a NET
# REGRESSION: Hz whole-flight R^2 0.48->0.07, and Hx/Hy dropped too
# (0.69->0.48, 0.76->0.65) -- consistent across all 5 runs, not an outlier.
# Mechanism: the more aggressive exclusion starves the point pool (n_kept
# median 10-17 -> 8-10), and that overall point-count reduction adds noise
# to EVERY phase (x/y/yaw dominate sample count and were made noisier), which
# outweighs the narrow z-phase leverage gain the isolated check was measuring.
# Lesson: don't judge a point-sampling change from an isolated single-phase
# diagnostic -- always re-derive+check the whole-flight fit before adopting.
CROSS_FLOW_CENTER_EXCLUDE_FRAC = float(os.environ.get("CROSS_FLOW_CENTER_EXCLUDE_FRAC", "0.35"))
MIN_PERIPHERAL_POINTS = int(os.environ.get("CROSS_FLOW_MIN_PERIPHERAL_PTS", "4"))
# Exclude candidate corners this close to the TRUE frame edge -- during lateral or
# descent motion these are exactly the points most likely to leave the frame within a
# few LK steps (correspondence lost), so keeping them out of the candidate pool avoids
# repeatedly proposing-then-losing the far-from-center points the bias above is trying
# to add in the first place.
FLOW_BOUNDARY_MARGIN_PX = int(os.environ.get("CROSS_FLOW_BOUNDARY_MARGIN_PX", "20"))

# 2026-08-13 (ring-style sampling, user-proposed -- see origin-spread gate finding
# in project_20260812_cross_marker_flow_architecture_investigation memory): ArUco's
# img_data.py guarantees multi-radius/angular coverage by sampling FIXED ring
# stations over the whole (generally-textured) background. The cross-marker's
# candidate region is instead restricted to the thin dilated-mask band around the
# drawn lines (see CROSS_FLOW_CENTER_EXCLUDE_FRAC's comment above -- "structural
# corner scarcity"), so fixed station positions would mostly land on untextured
# background with nothing to track. Adapted design: partition the SAME eligible
# (boundary-margin) mask into radius-band x angle-sector cells around the marker's
# own centroid, and draw GFT candidates per-cell -- guarantees whatever texture
# DOES exist gets sampled from multiple directions instead of letting one strong
# local region (e.g. the central intersection) dominate via a single unconstrained
# GFT call. Off by default; CROSS_RING_SAMPLING=1 to enable.
RING_SAMPLING = os.environ.get("CROSS_RING_SAMPLING", "0") == "1"
RING_N_BANDS = int(os.environ.get("CROSS_RING_N_BANDS", "2"))
RING_N_SECTORS = int(os.environ.get("CROSS_RING_N_SECTORS", "8"))   # must be even -- paired-opposite
                                                                      # rejection needs a diametric partner
RING_PTS_PER_CELL = int(os.environ.get("CROSS_RING_PTS_PER_CELL", "2"))


def _unweighted_principal_angle(pts):
    """Manuscript alpha (Sec. Image Features, eq. ~197): plain 2nd-moment
    principal angle over N>=3 non-collinear points, pi-disambiguated via the
    (here: unweighted vs a coarse asymmetric sub-split) centroid displacement.
    Unlike img_data.py's _marker_principal_angle, NO [4,3,2,1] weighting --
    that hack exists only because a symmetric ArUco square has mu_11==0
    under uniform weights. The cross marker's stub is a REAL asymmetry, so
    the plain formula is directly usable when the stub cluster is present.
    `pts` must already include the stub points for disambiguation to work;
    callers are responsible for holding-last when the stub isn't detected.

    This is EXACTLY eq. (21) of Jabbari Asl, Yoon & Tosunoglu, "Adaptive
    dynamic image-based visual servoing of a quadrotor UAV" (2014):
    alpha = (1/2)*atan2(2*mu11, mu20-mu02), evaluated in the virtual/leveled
    image plane (same convention as this module's V-frame). Their eq. (22)
    derives the theoretically EXACT rotational dynamics for this same
    formula: alpha_dot = -psi_dot (psi = yaw) -- confirms the slope=-1
    relationship used by self._alpha_0 (__init__) and by controller.py's
    BODY_YAW_ALPHA_K default under MARKER_TYPE=cross is not just an
    empirical fit, it's the closed-form result for this exact feature.
    """
    pts = np.asarray(pts, dtype=np.float64)
    x, y = pts[:, 0], pts[:, 1]
    xc, yc = float(np.mean(x)), float(np.mean(y))
    Xc, Yc = x - xc, y - yc
    mu20 = float(np.sum(Xc * Xc))
    mu02 = float(np.sum(Yc * Yc))
    mu11 = float(np.sum(Xc * Yc))
    a = 0.0 if abs(mu11) < 1e-9 else 0.5 * np.arctan2(2 * mu11, mu20 - mu02)
    return a, (xc, yc)


def _disambiguate_angle(a, asym_vec):
    """pi -> 2pi disambiguation using a real asymmetry vector (e.g. stub
    centroid minus arm-only centroid) instead of ArUco's weighted-vs-geometric
    centroid trick -- same idea, applied to the cross marker's own asymmetry."""
    dx, dy = asym_vec
    if dx * dx + dy * dy < 1e-12:
        return a
    d = a - np.arctan2(dy, dx)
    if abs(np.arctan2(np.sin(d), np.cos(d))) > np.pi / 2:
        a += np.pi
    return float(np.arctan2(np.sin(a), np.cos(a)))


def _fill_A(centered_pts):
    """Image Jacobian for a set of normalized (x,y) points, depth-normalized
    (Z=1 folded into calibration, matching img_data.py's convention). Mirrors
    img_data.py's _fill_A EXACTLY (same 6-DOF [h1,h2,h3,w1,w2,w3] ordering) --
    duplicated rather than imported so this module has no dependency on
    IMG_PROCESSOR's class internals, only the shared math."""
    x = centered_pts[:, 0]
    y = centered_pts[:, 1]
    N = len(x)
    A = np.zeros((2 * N, 6))
    A[0::2, 0] = 1
    A[1::2, 1] = 1
    A[0::2, 2] = -x
    A[1::2, 2] = -y
    A[0::2, 3] = -x * y
    A[1::2, 3] = -(1 + y**2)
    A[0::2, 4] = 1 + x**2
    A[1::2, 4] = x * y
    A[0::2, 5] = -y
    A[1::2, 5] = x
    return A


class CrossMarkerPerception:
    """Stateful, ROS-independent core -- call process_frame(img, t) once per
    captured frame. Kept separate from any ROS/threading wrapper so it can be
    unit-tested offline against saved frames (this project's established
    validation pattern), same as cross_marker_detector.detect() was."""

    def __init__(self, resolution=(480, 640)):
        # resolution is `_image_node.getImgResolution()` = (msg.height, msg.width)
        # of the ORIGINAL pre-rotation image, i.e. (480, 640) for this camera --
        # NOT the rotated frame's own (h, w). Because cv2.ROTATE_90_CW swaps the
        # two dimensions, `resolution[0]` (480) happens to equal the ROTATED
        # frame's width and `resolution[1]` (640) its height, so
        # np.array(resolution)/2 = (240, 320) is ALREADY (cx, cy) with no
        # reversal needed. A `[::-1]` here silently transposes cx/cy, corrupting
        # every point normalization downstream (the h,w Jacobian solve AND the
        # `s` centroid). See img_data.py:69-91's identical convention -- that
        # file's own comment documents someone adding this exact `[::-1]` on
        # 2026-06-01, believing it fixed a bug, then reverting it after
        # empirical verification showed the original was correct. This module
        # repeated that same mistake fresh on 2026-08-01; fixed 2026-08-02 after
        # a flow-accuracy investigation (near-zero raw-vs-GT correlation despite
        # healthy point counts/conditioning/in-sample fit residual) traced back
        # to this transpose. Do NOT re-add `[::-1]` without re-deriving why.
        self._resolution = resolution
        self.center = np.array(resolution) / 2.0   # (cx, cy) = (240, 320)
        self.focal = np.array([fx, fy])

        # 2026-08-12: self._prev_gray/_prev_quat/_prev_angvel/_prev_frame_t used to be
        # persisted here (cross-call state for the LK "prev" reference) -- REMOVED, not
        # just fixed, by the dt/frame-pairing architecture change. process_frame() now
        # receives its own adjacent frame pair + quat/angvel pair + native dt directly
        # from the caller every call, so no persisted "prev" state can go stale across a
        # detection dropout. Only the tracked POINT IDENTITY (_prev_flow_pts, positions)
        # still needs to persist across calls -- everything else is supplied fresh.
        # See project_20260812_cross_marker_flow_architecture_investigation memory.
        self._prev_flow_pts = None   # (N,2) pixel coords tracked for h,w
        # 2026-08-13 (ring sampling): parallel (N,) array of cell-id (band*RING_N_SECTORS+
        # sector) per point in self._prev_flow_pts, only populated when RING_SAMPLING is on
        # (else None). Lets _compute_hw find a lost point's diametric partner (opposite
        # sector, same band) and drop it too -- mirrors img_data.py's PLASMC_RING_PAIRED.
        self._prev_flow_cell_id = None
        self._last_resample_t = None  # timestamp of the last _sample_flow_points() call (2026-08-07
                                       # periodic-refresh fix -- see RESAMPLE_PERIOD_S's comment)
        self._z_v_log = []           # min(z_v) per _getVirtualPts call -- degeneracy diagnostic
        self._last_alpha = 0.0
        self._alpha_valid_once = False
        # ALPHA_0 (2026-08-08, RE-DERIVED from 5 phased calibration flights): equilibrium/
        # reference offset, same role as img_data.py's _moment_alpha_0 -- the raw
        # principal-angle alpha is a real geometric measurement of the marker's
        # orientation in the V-frame, but that frame's zero-heading reference doesn't line
        # up with GT yaw's zero (camera-mount + stub-geometry convention), so a constant
        # needs subtracting before alpha means "vehicle yaw relative to marker". Derived
        # from TRUE per-detect-rate, causally time-aligned samples (Img_Data.npy's own
        # 'Time'/'alpha(t)' log against GT, using gt['Time'] + gt['Start Time'] for the GT
        # clock -- NOT the outer-loop-polled 'Img Feature Params', which oversamples ~2.7x
        # per real detection and corrupts any offset/slope fit).
        #
        # Methodology note: an UNCONSTRAINED 2-parameter affine fit (slope+offset via
        # np.polyfit) is numerically unstable on this excitation profile -- per-run slopes
        # ranged -2.88 to +6.7 (R^2 0.01-0.97), because the x/y/z phases contribute long
        # stretches of near-zero yaw that leave the slope poorly constrained by anything
        # but noise. Forcing the PHYSICALLY-EXPECTED slope=-1 (a world-fixed marker
        # counter-rotates 1:1 with vehicle yaw -- no independent scale factor should be
        # needed) and solving only for the offset instead gives a dramatically more
        # consistent fit. slope=-1 is not just a reasonable assumption here -- it is the
        # theoretically EXACT result for this alpha formula: Jabbari Asl, Yoon & Tosunoglu
        # (2014), eq. (21)-(22), derive alpha_dot = -psi_dot for the identical plain
        # 2nd-moment principal angle in the leveled/virtual image plane (see
        # _unweighted_principal_angle's docstring). Across 5 independent phased-excitation
        # flights (yaw+-30deg and
        # yawagg+-90deg phases; calibration_data/output_cross/2026-08-08 runs, recorded
        # with CROSS_ALPHA_0=0 to log the pre-offset angle), per-run alpha_0 = 90.11,
        # 90.39, 89.98, 90.48, 90.18 deg -- circular mean 90.23deg, inter-run std ONLY
        # 0.18deg, R^2 0.974-0.995 (mean 0.988). That tightness is itself confirmation
        # slope=-1 is the correct model (an unstable/wrong model wouldn't reproduce this
        # consistently across independent flights). Supersedes the 2026-08-08 single-flight
        # 88.70deg estimate. Suspiciously close to exactly 90deg -- plausibly a direct
        # consequence of the 90deg camera-mount yaw rotation -- but pasted as the fitted
        # value, not hand-rounded, per project convention (see img_data.py's alpha_0).
        self._alpha_0 = float(os.environ.get("CROSS_ALPHA_0", str(np.radians(90.23))))

        # last computed outputs, for the getter interface
        self._s = np.array([0.0, 0.0, 1.0])
        self._center_px = None   # (2,) raw pixel [cx, cy] of the last successful detection,
                                  # for the visibility CBF (cbf_visibility.cbf2_filter takes
                                  # raw pixels and normalizes internally -- self._s is already
                                  # normalized, wrong units for that call)
        self._center_fresh = False   # True only on frames where det.ok this exact frame
        self._alpha = 0.0
        self._hw = np.zeros(6)
        self._ok = False
        self._last_t = None
        # Color-gated mask bbox (x, y, w, h) of the last SUCCESSFUL detection -- scale-free
        # proxy for marker span, feeds MARKER_EXTENT_PX (controller.py) via
        # CrossMarkerNode._feature_pts. Held (not cleared) across a loss streak, same
        # hold-last-good convention as _s/_alpha -- MARKER_EXTENT_PX returning 0.0 is meant
        # for "never detected yet", not "lost this exact frame".
        self._last_bbox = None

        # Output calibration (GT = cal @ raw), same convention as img_data.py's
        # _sensor_cal_hw / _sensor_cal_s -- the cross marker's h,w come from its
        # own image-Jacobian solve (not img_data.py's), so it needs its own
        # empirical correction, not a reuse of the ArUco board's cal.
        # RE-DERIVED 2026-08-05 (supersedes the 2026-08-03 version) via
        # apps/record_cross_marker_calibration.py + tools/derive_cross_marker_cal.py,
        # from 5 clean phased-excitation runs (95% detection ok-rate gate; a 6th run
        # at 88.2% was correctly auto-skipped). Required after a cluster of same-day
        # changes that all touch the raw h/w/s signal: camera-mount yaw+90deg (moved
        # the landing-leg ghost out of top/bottom into left/right margins), camera Z
        # offset .20->.18/.15, cross_marker.png line width halved, color-gate
        # threshold 20->100 (needed for the thinner line's anti-aliasing at range),
        # tracking-based ROI added, and -- most consequential for this specific
        # matrix -- the [-y,x]->[y,-x] axis-sign-flip fix in _getVirtualPts (see that
        # function's own comment for the empirical derivation). The stale
        # pre-2026-08-05 recordings this fit would otherwise have mixed in were moved
        # to calibration_data/output_cross_stale_pre20260805/, not deleted.
        # R^2 (this fit): Hx=0.55 Hy=0.63 Hz=0.22 Wz=0.57 (Wx/Wy forced 0, same
        # level-target convention as the ArUco board cal). Centroid scale sx=1.044
        # sy=1.012, inter-run spread modest (~5-11% across the 5 runs).
        #
        # 2026-08-06 ROOT-CAUSE FOLLOW-UP on Hz/Wz weakness: ruled out data
        # contamination. An initial diagnostic (ad-hoc script, not this tool) appeared
        # to find z-phase windows dominated by constant yaw-rate (~1.0-1.4 rad/s)
        # instead of vz -- that "finding" was itself a bug: the script truncated
        # gt['Phase'] via a naive [:n] slice instead of applying the same
        # valid-mask filter compute_gt_signals() uses internally (which drops
        # scattered duplicate-timestamp samples, not a trailing block), desyncing
        # phase labels from the GT arrays. Re-checked with correct alignment
        # (matching derive_cross_marker_cal.py's own indexing) across all 6
        # recordings: z/yaw/yawagg phases are genuinely clean in every run (z:
        # vz~0.15-0.35 dominant, wz~0.001-0.003; yaw/yawagg: wz~1.1-1.3 dominant,
        # as expected). A purity gate built on the false premise (clean_zyaw_mask,
        # briefly added to derive_cross_marker_cal.py) made Wz worse, not better,
        # and has been reverted -- consistent with there being no real
        # contamination for it to remove. This exact M-matrix (identical to what's
        # here) is confirmed reproducible from the reverted tool.
        #
        # So Hz(R^2=0.22)/Wz(STD 1.2-3.0, coefficients up to 12.9) reflect a genuine
        # REGRESSION-CONDITIONING/OBSERVABILITY limit, not a data-quality bug: Wz's
        # own row coefficients are large on the h1/w1 (y-flow) columns specifically,
        # the signature of near-collinearity -- the yaw phase's raw-flow response
        # isn't cleanly separable from y-translation flow in this pipeline's
        # regressor. Hz's low R^2 likely reflects genuinely weak/noisy vertical-flow
        # sensitivity for this marker/geometry. Treat Hz/Wz-derived signals (loom,
        # yaw-rate coupling) with proportionally less confidence than Hx/Hy/centroid;
        # improving this would need better-separated/larger-amplitude z and yaw
        # excitation phases in the recorder, not a purity-gate fix on the derivation
        # side.
        #
        # 2026-08-06 RE-DERIVED again after landing the peripheral-corner-bias GFT
        # sampling change (CROSS_FLOW_CENTER_EXCLUDE_FRAC, see _sample_flow_points)
        # -- 4 clean runs (a 5th at 71.9% ok-rate auto-skipped). CONFIRMED raw
        # correlation matrix of the 6 raw columns has two near-zero eigenvalues
        # (0.0015, 0.0021): raw h1(Ty)/w0(Wx) correlate r=0.98-1.00 in EVERY phase
        # (x/y/z/yaw/yawagg/settle alike) -- a structural degeneracy in the per-frame
        # Jacobian solve itself (_fill_A's Wx column -(1+y^2) stays ~constant when
        # |y| is small, same shape as Ty's constant +1 column), not an excitation-
        # purity artifact. This is the same radial-spread ceiling as
        # feedback_cross_marker_radial_spread_ceiling, sharpened with a mechanism.
        # Net effect of the peripheral-bias change, isolated (same camera/geometry
        # config as the 2026-08-05 fit above, ONLY the point-sampling changed):
        # Hx 0.55->0.73, Hy 0.63->0.79 (real win, now BEATS the pre-08-05 baseline
        # 0.70/0.71) -- Hz stayed EXACTLY 0.22, Wz roughly flat 0.57->0.53 (a wash,
        # not a regression -- consistent with the degeneracy above being unrelated
        # to point radial spread within the achievable range). DEPLOYED: net win,
        # no measured downside. Hz's real culprit is upstream of this change (in
        # the 08-03->08-05 camera-mount/axis-sign changes) and Wz's is the raw-
        # signal collinearity above -- neither is fixable by more calibration runs
        # or point-sampling tweaks; see the radial-spread-ceiling memory for the
        # next untried lever (bigger physical marker).
        # 2026-08-07 RE-DERIVED again after the periodic-resample fix (see
        # RESAMPLE_PERIOD_S's comment) -- root-caused and fixed the large
        # run-to-run Hz/Wz variance that made every single-parameter bisection
        # this session (color gate, camera Z-offset) inconclusive: without a
        # periodic forced refresh, RESAMPLE_TRIGGER alone meant the tracked
        # point set was effectively seeded ONCE near takeoff and then just
        # tracked forward via LK for the whole flight (confirmed via Flow Diag
        # Log: 4/5 pre-fix runs showed a perfectly CONSTANT n_kept for their
        # entire ~500-frame z-phase, i.e. zero resamples during that window).
        # Whichever specific (x,y) locations that one-time early draw landed on
        # then determined the WHOLE flight's Hz/Wz bias (position-WEIGHTED
        # columns in _fill_A), while Hx/Hy (position-INDEPENDENT columns) were
        # largely immune -- exactly the asymmetry observed all session.
        # 5 clean runs post-fix (95%+ ok-rate; SITL flake rate was unusually
        # high this session, several runs auto-skipped): R^2 Hx=0.69 Hy=0.76
        # **Hz=0.48** Wz=0.50 -- Hz more than DOUBLED vs the 2026-08-06
        # peripheral-bias cal below (0.22->0.48), Hx/Hy/Wz shifted by only
        # 0.03-0.06 (well inside this session's normal run-to-run noise, e.g.
        # +-0.1-0.2+ swings seen everywhere else). Net win with no measured
        # cost. Wx/Wy still correctly forced 0 (same collinearity ceiling as
        # before, untouched by this fix -- it's a different mechanism, see
        # feedback_cross_marker_radial_spread_ceiling memory).
        # 2026-08-08 RE-DERIVED after the GYRO DE-ROTATION fix (_solve_jacobian): an
        # independent-multisine holdout validation of the 2026-08-07 cal above found it
        # badly OVERFIT -- training R^2 (Hx=0.69, Wz=0.50) collapsed on held-out data
        # (Hx->0.14, Wz->-2.0, worse than predicting the mean). Root cause: raw h_Tx
        # correlates -0.99 with raw w_Wy (and h_Ty +0.99 with w_Wx) because _fill_A's
        # Wy column [1+x^2,xy] numerically ALIASES the constant Tx column [1,0] at this
        # marker's achievable point spread -- confirmed via TRUE gyro Wy correlating only
        # +0.04 with h_Tx, so the apparent collinearity was a per-frame LSTSQ artifact,
        # not a real physical relationship, and NOT fixable by better excitation.
        # FIX, STAGE 1 (_solve_jacobian): SUBTRACT the gyro's exactly-known Wx/Wy
        # contribution from the observed flow before solving (gz_subscriber.
        # Image_Node._angvel_deque was already plumbed, synced to the flow-point pair
        # the same way quat is -- just never consumed here), then solve a REDUCED
        # 4-unknown [Tx,Ty,Tz,Wz] problem from the remaining (non-aliased) columns.
        # Confirmed this alone fixes the PER-FRAME conditioning (inter-run STD of the
        # fitted cross-terms dropped to ~4-11% of their magnitude, vs an
        # effectively-arbitrary split before) -- but a re-derivation that still used the
        # now-clean Wx/Wy as CALIBRATION regressors (R^2 Hx=0.80 Hy=0.84 train) STILL
        # overfit on holdout (Hx->0.15, Wz->-0.81) -- because a large fitted cross-term
        # bakes in the TRAINING maneuver's specific Tx:Wy coupling ratio (phased
        # single-axis excitation), which doesn't transfer to a different maneuver shape
        # (multisine's continuous combined excitation) even though Wy itself is now
        # accurate. FIX, STAGE 2 (tools/derive_cross_marker_cal.py): since the per-frame
        # de-rotation already removes the rotation contamination from h_Tx/h_Ty
        # directly, the CALIBRATION step doesn't need a Wx/Wy correction term at all --
        # excludes raw Wx,Wy as regressors entirely (block-diagonal 4x4 fit: Hx,Hy,Hz,Wz
        # from Tx,Ty,Tz,Wz only). Re-derived from 5 fresh phased-calibration flights
        # (95%+ ok-rate; 2 more auto-skipped/discarded to SITL flakiness): TRAIN R^2
        # Hx=0.52 Hy=0.69 Hz=0.48 Wz=0.51 (lower than the rejected cross-term fit, as
        # expected for a less flexible model) -- but on the SAME independent multisine
        # holdout: Hx=0.16 Hy=0.32 Hz=0.74 **Wz=+0.01** (up from -2.0 originally,
        # -0.81 with cross-terms -- no channel is catastrophically negative anymore).
        # NET RESULT: the original overfitting bug (task: "Fix cross-marker Hx/Wz
        # overfitting") is resolved -- no more negative-R^2 holdout collapse.
        # CORRECTION (2026-08-08, later same day): the Hx=0.16/Hy=0.32 holdout numbers
        # above were themselves measured with a flawed tool (tools/validate_cross_marker_
        # flow.py's prep() was reading the OUTER-LOOP-POLLED gt['Opt Flow Ang Vel'], ~3x
        # oversampled vs the true ~38-60Hz detect() rate -- the identical artifact already
        # root-caused for alpha earlier this session). Fixed to use Img_Data.npy's true
        # per-frame log: re-measured on the SAME holdout flight, Hx R^2 0.16->0.42-0.73,
        # Hy 0.32->0.53-0.71 (range spans KF-filtered vs raw). There is NO genuine Hx/Hy
        # noise floor -- that framing was itself a measurement artifact, not a real
        # sensor/geometry limitation. Don't cite the 0.16/0.32 figures going forward
        # here. Wx/Wy rows stay forced 0 (level-target convention, unchanged).
        # 2026-08-09 (Hz-regression investigation): re-derived EXCLUDING lowalt_x/y/yaw
        # samples from the M-fit (tools/derive_cross_marker_cal.py,
        # CROSS_CAL_EXCLUDE_LOWALT=1 default). M is supposed to be altitude-invariant
        # by construction (both raw and GT sides are already Z-normalized via
        # h=v/(z+Z_REG) -- see feedback_scale_free_depth_free) so the lowalt_* phases
        # were solving the wrong problem: they added low-altitude COVERAGE, but
        # investigation found the low-alt transition never settles within its phase
        # window (still-descending transient, ~9s to actually arrive at target alt),
        # so those samples are noisier, not more representative -- including them was
        # actively corrupting the Hz row (position-weighted _fill_A column gives the
        # noisy low-alt samples outsized leverage). Same underlying leg-extension +
        # hi-res-texture flights (5 runs, CALIB_LOW_ALT=0.7), just fit on the clean
        # (higher-altitude) phases only. Wx/Wy rows stay forced 0 (level-target
        # convention, unchanged).
        self._sensor_cal_hw = np.array([
            [+0.1335, +0.0099, +0.0013, +0.0000, +0.0000, -0.0015],
            [-0.0104, +0.1719, +0.0010, +0.0000, +0.0000, -0.0071],
            [-0.2212, +0.0532, +0.4331, +0.0000, +0.0000, -0.0214],
            [+0.0000, +0.0000, +0.0000, +0.0000, +0.0000, +0.0000],
            [+0.0000, +0.0000, +0.0000, +0.0000, +0.0000, +0.0000],
            [+0.0195, +1.9644, -0.0426, +0.0000, +0.0000, +0.5201]])
        self._sensor_cal_s = np.diag([1.0320, 0.9984, 1.0, 1.0])

        # Diagnostic instrumentation (2026-08-01, point-starvation/centroid-instability
        # investigation): per-frame (t, ok, fail_reason, bbox_area) log, always cheap
        # to keep. Frame dumps around ok-state transitions are opt-in via
        # CROSS_DIAG_SAVE_DIR (off by default -- full-res frame writes are not free).
        self._diag_log = []
        self._diag_save_dir = os.environ.get("CROSS_DIAG_SAVE_DIR", "")
        self._diag_frame_idx = 0
        self._diag_lost_streak = 0
        if self._diag_save_dir:
            os.makedirs(self._diag_save_dir, exist_ok=True)

        # Point-count/conditioning/correspondence diagnostic (2026-08-02, flow-accuracy
        # investigation): per-flow-solve (t, n_kept, cond(A), solved, rel_resid,
        # px_disp_median, px_disp_std) log. n_kept/cond(A) ruled out point starvation and
        # ill-conditioning as the cause of near-zero raw-vs-GT correlation on clean
        # (det.ok=True) frames. rel_resid (||A@sol-b||/||b||, the LSTSQ fit's own
        # in-sample residual) now checks whether the tracked points at least agree with
        # EACH OTHER on a single global flow (low resid -> systematic bug elsewhere, e.g.
        # scale/sign/frame convention) or disagree with each other too (high resid ->
        # real per-point LK correspondence noise, e.g. from the marker's self-similar
        # speckle texture). px_disp_median/std are the raw per-point pixel displacements
        # feeding that fit, for a sanity check against plausible motion magnitude.
        self._flow_diag_log = []
        self._radial_diag_log = []   # TEMP DIAG 2026-08-03, see _solve_jacobian
        self._radial_diag = (np.nan, np.nan, np.nan, np.nan)
        # TEMP DIAG 2026-08-09 (Hz sign-flip point-set-composition follow-up, see
        # HANDOVER_cross_marker_hz_signflip_20260809.md): _radial_diag_log only has
        # aggregate max/mean spread, not per-point resolution -- can't tell whether the
        # low-altitude point set is symmetric-but-wider (SNR-only) or asymmetric
        # (biased to one side, which could alias into the reduced 4-unknown solve). Log
        # the full normalized point set fed into A (prev_n) plus the solved Tz (sol[2])
        # per solve so this can be checked directly on a new flight.
        self._point_diag_log = []
        self._point_diag = (None, np.nan, None, np.nan, None)

        # Time-series logging (2026-08-04, perception-quality debugging): log full
        # frame-by-frame history so getLogData() can return it for comparison with GT.
        # Each appended once per process_frame() call, matching img_data.py's pattern.
        self._time_log = []
        self._quat_log = []      # quaternion (for V-frame reconstruction)
        self._s_log = []         # (3,) per frame — centroid in V-frame
        self._hw_log = []        # (6,) per frame — optical flow Jacobian solve
        self._alpha_log = []     # scalar per frame — marker orientation
        self._n_flow_corners_log = []  # per-frame KLT corner count
        self._feature_visibility_log = []  # True if detection succeeded
        self._detection_reason_log = []  # fail_reason when det.ok=False
        self._marker_extent_log = []  # MARKER_EXTENT_PX per frame
        self._center_px_log = []  # raw pixel center (for CBF use)

        # 2026-08-05: parity fields ported from img_data.py's IMG_PROCESSOR getLogData()
        # (see that class's own field list) -- everything here has a direct, generic
        # analog for a single-marker pipeline. Fields that are ArUco/ring-flow/
        # PlanarFeatureMap-specific (KLT Diag, Ring *, Opt Flow Fused, Target Vel,
        # Centroid/Alpha/Flow Map *, Planar Map Shadow) have NO counterpart here --
        # this module has no ring-flow, no multi-marker board, no map-based rescue --
        # so they're intentionally not ported (see CrossMarkerNode's own class
        # docstring for the documented scope/gaps vs IMG_PROCESSOR).
        self._imu_angvel_log = []      # (3,) [forward,right,down] rad/s, paired to the same frame as quat
        self._fps_log = []             # per-frame reported capture FPS (self._image_node.getFPS())
        self._stamp_log = []           # raw capture stamp (vs Time, which is the same value here --
                                        # img_data.py distinguishes perf_counter-based Time from the
                                        # frame's own header stamp; this module already uses the
                                        # frame's own stamp as `t` throughout, so the two coincide)
        self._feature_pts_raw_log = []  # raw (line_i + line_j + stub) pixel points, this frame's detection
        self._img_feature_param_log = []  # CALIBRATED [xc,yc,1,alpha] (getImgFeatureParam() output),
                                            # as opposed to _s_log/_alpha_log which are pre-calibration

        # TRACKING-BASED ROI (2026-08-05): persistent state dict passed into cmd.detect()
        # every call -- see cross_marker_detector.py's TRACK_MARGIN_PX comment for the
        # full design (ported from Hardware/scripts/img_data.py's ArUco
        # ARUCO_ROI_MARGIN_PX fast path). Owned here (not module-global) so it resets
        # cleanly per-instance/per-flight.
        self._track_state = {'last_bbox': None, 'miss_count': 0}

    @staticmethod
    def _dilate_mask(mask):
        kernel = np.ones((2 * MASK_DILATE_PX + 1, 2 * MASK_DILATE_PX + 1), np.uint8)
        return cv2.dilate(mask, kernel)

    def _sample_flow_points(self, gray, dilated_mask):
        """Returns (points, cell_ids). cell_ids is None unless RING_SAMPLING is on
        (see _sample_flow_points_ring's docstring) -- callers must handle both."""
        if RING_SAMPLING:
            return self._sample_flow_points_ring(gray, dilated_mask)
        return self._sample_flow_points_unconstrained(gray, dilated_mask), None

    def _sample_flow_points_ring(self, gray, dilated_mask):
        """Ring-style adaptation for the cross-marker (2026-08-13, user-proposed;
        RE-CENTERED 2026-08-13 after a design error was caught -- see below).
        Partitions the boundary-margin-eligible mask into RING_N_BANDS radius bands x
        RING_N_SECTORS angle sectors CENTERED ON THE IMAGE CENTER (self.center), and
        draws up to RING_PTS_PER_CELL GFT candidates from each cell independently --
        guarantees multi-directional coverage of whatever eligible texture exists
        (mirrors img_data.py's fixed ring stations, but cell-gated by real texture
        instead of fixed absolute pixel positions, since the marker's own eligible
        band is thin/sparse -- see RING_SAMPLING's top-of-file comment). RING_N_SECTORS
        must be even: sector s's diametric opposite is (s + RING_N_SECTORS//2) %
        RING_N_SECTORS, used by _compute_hw for paired-opposite LK-failure rejection.

        WHY IMAGE CENTER, NOT MARKER CENTROID (2026-08-13 correction -- the first cut
        of this method centered on the mask's own centroid, which is WRONG and was
        caught before it shipped): img_data.py's ring design (`_ring_pts0_V`,
        `_ring_opp_idx`) centers on the V-frame nadir specifically so that two
        diametrically-opposite stations -- same radius, angle i and i+N/2, both
        measured from the IMAGE CENTER -- see a translation-induced flow component
        that CANCELS between the pair (equal magnitude, opposite sign) while a
        divergence/Tz-induced (radial) component ADDS (same sign at both, since
        "radially outward" points in opposite real-world directions for the two
        stations). That cancellation is the entire reason `_ring_opp_idx`'s
        paired-rejection rule ("drop a station if its opposite partner also failed")
        is meaningful -- it's preserving a symmetry that only exists relative to the
        image center. Centering on the marker's own centroid instead breaks this:
        the marker is not generally at the image center, so "diametrically opposite
        across the marker centroid" is a different, unrelated pair of directions with
        no such cancellation property. Re-centered here to match img_data.py's actual
        (verified by re-reading the code) design.

        Returns (points (N,2) float32, cell_ids (N,) int -- band*RING_N_SECTORS+sector).
        Falls back to the unconstrained sampler (cell_ids=None) if the mask is empty."""
        h, w = dilated_mask.shape
        m = FLOW_BOUNDARY_MARGIN_PX
        boundary_mask = dilated_mask
        if m > 0 and h > 2 * m and w > 2 * m:
            boundary_mask = dilated_mask.copy()
            boundary_mask[:m, :] = 0
            boundary_mask[-m:, :] = 0
            boundary_mask[:, :m] = 0
            boundary_mask[:, -m:] = 0

        if not boundary_mask.any():
            return np.zeros((0, 2), dtype=np.float32), np.zeros((0,), dtype=int)

        # ring geometry centered on the IMAGE center (self.center = (cx, cy)), NOT
        # the mask/marker centroid -- see docstring above. Normalized by R_max =
        # min(h,w)/2 (resolution-adaptive, same convention as img_data.py's _Rmax),
        # not by the mask's own extent, since the mask's position relative to center
        # is exactly what must NOT bias the radius/angle assignment.
        mcx, mcy = float(self.center[0]), float(self.center[1])
        r_max = float(min(h, w)) / 2.0

        # per-pixel (radius, angle) relative to the IMAGE center
        yy, xx = np.mgrid[:h, :w]
        r_norm = np.sqrt((xx - mcx) ** 2 + (yy - mcy) ** 2) / r_max
        theta = np.arctan2(yy - mcy, xx - mcx)   # (-pi, pi]
        band_edges = np.linspace(0.0, max(r_norm[boundary_mask > 0].max(), 1e-6) + 1e-6,
                                  RING_N_BANDS + 1)
        sector_width = 2 * np.pi / RING_N_SECTORS

        all_pts = []
        all_cell_ids = []
        for b in range(RING_N_BANDS):
            band_mask = (r_norm >= band_edges[b]) & (r_norm < band_edges[b + 1])
            for s in range(RING_N_SECTORS):
                lo = -np.pi + s * sector_width
                hi = -np.pi + (s + 1) * sector_width
                sector_mask = (theta >= lo) & (theta < hi)
                cell_mask = (boundary_mask > 0) & band_mask & sector_mask
                if not cell_mask.any():
                    continue
                cell_mask_u8 = cell_mask.astype(np.uint8) * 255
                pts = cv2.goodFeaturesToTrack(
                    gray, maxCorners=RING_PTS_PER_CELL, qualityLevel=GFT_QUALITY,
                    minDistance=GFT_MIN_DIST, mask=cell_mask_u8)
                if pts is None:
                    continue
                cell_pts = pts.reshape(-1, 2).astype(np.float32)
                all_pts.append(cell_pts)
                all_cell_ids.extend([b * RING_N_SECTORS + s] * len(cell_pts))

        if not all_pts:
            # no eligible cell yielded a corner (very sparse texture) -- fall back
            # rather than returning nothing this call
            return self._sample_flow_points_unconstrained(gray, dilated_mask), None
        return np.vstack(all_pts)[:GFT_MAX_CORNERS], np.array(all_cell_ids[:GFT_MAX_CORNERS], dtype=int)

    def _sample_flow_points_unconstrained(self, gray, dilated_mask):
        h, w = dilated_mask.shape

        # boundary-margin mask: never propose a corner this close to the true frame
        # edge -- it's likely to exit frame (LK correspondence lost) within a few
        # frames under lateral/descent motion, see FLOW_BOUNDARY_MARGIN_PX's comment.
        m = FLOW_BOUNDARY_MARGIN_PX
        boundary_mask = dilated_mask
        if m > 0 and h > 2 * m and w > 2 * m:
            boundary_mask = dilated_mask.copy()
            boundary_mask[:m, :] = 0
            boundary_mask[-m:, :] = 0
            boundary_mask[:, :m] = 0
            boundary_mask[:, -m:] = 0

        # peripheral mask: exclude a central disk around the marker's OWN centroid,
        # sized relative to its OWN current extent -- biases GFT candidates toward
        # the arm/stub tips instead of letting the cross's central intersection
        # (the strongest corner in the shape) dominate the point pool.
        peripheral_mask = boundary_mask
        ys, xs = np.nonzero(boundary_mask)
        if len(xs) > 0:
            mcx, mcy = float(xs.mean()), float(ys.mean())
            half_extent = max(float(xs.max() - xs.min()), float(ys.max() - ys.min())) / 2.0
            if half_extent > 0:
                exclude_r = CROSS_FLOW_CENTER_EXCLUDE_FRAC * half_extent
                yy, xx = np.ogrid[:h, :w]
                center_disk = (xx - mcx) ** 2 + (yy - mcy) ** 2 < exclude_r ** 2
                peripheral_mask = boundary_mask.copy()
                peripheral_mask[center_disk] = 0

        pts = cv2.goodFeaturesToTrack(
            gray, maxCorners=GFT_MAX_CORNERS, qualityLevel=GFT_QUALITY,
            minDistance=GFT_MIN_DIST, mask=peripheral_mask)
        peripheral_pts = (pts.reshape(-1, 2).astype(np.float32) if pts is not None
                          else np.zeros((0, 2), dtype=np.float32))
        if len(peripheral_pts) >= MIN_PERIPHERAL_POINTS:
            return peripheral_pts

        # too few peripheral corners survived (small/distant marker) -- fall back to
        # the unbiased boundary-margin-only mask so this never returns FEWER points
        # than the pre-2026-08-06 behavior would have.
        pts = cv2.goodFeaturesToTrack(
            gray, maxCorners=GFT_MAX_CORNERS, qualityLevel=GFT_QUALITY,
            minDistance=GFT_MIN_DIST, mask=boundary_mask)
        if pts is None:
            return peripheral_pts
        full_pts = pts.reshape(-1, 2).astype(np.float32)
        if len(peripheral_pts) == 0:
            return full_pts
        # merge without near-duplicates of what peripheral sampling already found
        keep = np.ones(len(full_pts), dtype=bool)
        for i, p in enumerate(full_pts):
            if np.any(np.linalg.norm(peripheral_pts - p, axis=1) < GFT_MIN_DIST):
                keep[i] = False
        merged = np.vstack([peripheral_pts, full_pts[keep]])
        return merged[:GFT_MAX_CORNERS]

    def _getVirtualPts(self, pts, quat):
        """Reproject camera-frame pixels onto the virtual image plane V: a
        LEVEL frame (gravity-aligned z) that preserves the UAV's yaw heading.
        Direct port of img_data.py's _getVirtualPts (same V-frame convention
        compute_gt_signals uses for GT, so raw and GT are finally in the SAME
        frame) -- adapted to this module's self.center/self.focal instead of
        img_data.py's module-level cx,cy/fx,fy.

        MISSING FROM THIS MODULE UNTIL 2026-08-02: the raw h,w computation
        previously used un-leveled camera-frame coordinates directly, with NO
        attitude compensation at all -- comparing that against V-frame GT
        (which removes roll/pitch by construction) is comparing different
        physical quantities whenever the drone tilts, which happens
        specifically during the x/y translation phases that excite Hx/Hy.
        Root-caused as the likely dominant remaining cause of the Hx/Hy
        weak-correlation investigation -- see
        Memory/px4/project_cross_marker_pipeline_20260801.md. quat=None
        (not yet available, e.g. before FC connects) falls back to the
        un-leveled normalization so callers degrade gracefully rather than
        crash.
        """
        # CAMERA-MOUNT YAW FIX (2026-08-04, CORRECTED): x500_mono_cam_down/model.sdf's
        # camera mount now has yaw+=90deg on top of the pre-existing pointing-down
        # pitch=90deg (moved the drone's landing-leg mirrored ghost from the top/bottom
        # to the left/right image margins -- see _restrict_to_center_roi's docstring).
        # The OLD "camera=body-FRD aligned" ray=[x,y,1] convention needs a compensating
        # rotation. Initial derivation (Rz(+90deg) -> [-y,x]) was empirically WRONG:
        # verified against the marker's known world-yaw=0deg (cross_marker/model.sdf's
        # static pose) via the alpha (heading) computation -- unswapped raw alpha read
        # ~90deg (not ~0deg) after the mount change, and [-y,x] added ANOTHER +90deg
        # instead of correcting it. Also independently confirmed by s_e_n DIVERGING
        # (0.056->0.459 over a 5s test) instead of converging with the [-y,x] version --
        # a live closed-loop instability signature consistent with a 180deg-opposite
        # control direction. Correct transform is [y,-x] (Rz(-90deg)) -- i.e. swap x
        # and y, negate the new y. Applied to BOTH the quat-based V-frame path and the
        # quat=None raw fallback for consistency (same physical camera).
        if quat is None:
            cx, cy = self.center
            fxx, fyy = self.focal
            x = (pts[:, 0] - cx) / fxx
            y = (pts[:, 1] - cy) / fyy
            return np.column_stack([y, -x])

        R = Quaternion([quat.w, quat.x, quat.y, quat.z]).to_DCM()
        g = R.T @ np.array([0, 0, 1])   # world-down in body/camera frame (camera=body-FRD aligned)

        z_axis = g / np.linalg.norm(g)
        x_axis = np.cross([0, 1, 0], z_axis)
        x_axis /= np.linalg.norm(x_axis)
        y_axis = np.cross(z_axis, x_axis)
        C_R_V = np.column_stack([x_axis, y_axis, z_axis])

        cx, cy = self.center
        fxx, fyy = self.focal
        x = (pts[:, 0] - cx) / fxx
        y = (pts[:, 1] - cy) / fyy
        rays = np.column_stack([y, -x, np.ones_like(x)])
        V_rays = rays @ C_R_V
        z_v = V_rays[:, 2]
        # Diagnostic (2026-08-02, V-frame regression debug): img_data.py tracks this
        # exact quantity for the exact same reason (its own comment: "z_v -> 0 or
        # negative means this corner's ray... is not representable in the
        # gravity-leveled V-frame -- grazing/behind-camera obliqueness"). A near-zero
        # z_v blows up the perspective divide into a huge or sign-flipped point,
        # exactly the kind of thing that could turn a real translational signal into
        # noise/garbage without tripping any of the existing n_kept/cond/rel_resid
        # diagnostics (those check the LSTSQ fit, not the per-point projection that
        # feeds it).
        self._z_v_log.append(float(np.min(z_v)) if len(z_v) else np.nan)
        # TEMP DIAG (2026-08-03, s_e_n/h_y single-frame-spike investigation): confirm/deny
        # the near-zero-z_v perspective-divide-blowup mechanism the comment above already
        # theorizes. Fires on ANY ray this close to grazing, npts distinguishes the
        # centroid call (npts=1, feeds s) from the flow-point call (npts=N, feeds h).
        if len(z_v) and np.min(z_v) < 0.5:
            # tilt-from-vertical computed from this SAME quat/DCM -- self-contained, no
            # cross-clock lookup needed (2026-08-03, "is 60deg cap enough?" follow-up).
            _tilt_deg = float(np.degrees(np.arccos(np.clip(R[2, 2], -1.0, 1.0))))
            _i_worst = int(np.argmin(z_v))
            print(f"[Z_V_DIAG] t={self._last_t} npts={len(z_v)} min_z_v={np.min(z_v):.4f} "
                  f"tilt_deg={_tilt_deg:.1f} x,y(worst)={x[_i_worst]:.3f},{y[_i_worst]:.3f} "
                  f"max|out|={np.max(np.abs(np.column_stack([V_rays[:,0]/z_v, V_rays[:,1]/z_v]))):.3f}")
        return np.column_stack([V_rays[:, 0] / z_v, V_rays[:, 1] / z_v])

    def _vframe_w(self, w_body, quat):
        """Rotate a body-FRD angular velocity into the virtual (gravity-leveled) frame,
        using the SAME C_R_V basis as _getVirtualPts. w_V = V_R_C.w_body = C_R_V.T.w_body.
        Ported verbatim from img_data.py's _vframe_w (validated there for the
        centroid-rate observer's gyro rotation-compensation term -- that observer's
        REGRESSION was in differentiating noisy decoded centroid positions, not in this
        rotation transform, so the transform itself carries over trusted)."""
        R = Quaternion([quat.w, quat.x, quat.y, quat.z]).to_DCM()
        g = R.T @ np.array([0, 0, 1.0])
        z_axis = g / np.linalg.norm(g)
        x_axis = np.cross([0, 1, 0], z_axis); x_axis /= np.linalg.norm(x_axis)
        y_axis = np.cross(z_axis, x_axis)
        C_R_V = np.column_stack([x_axis, y_axis, z_axis])
        return C_R_V.T @ np.asarray(w_body, float)

    def _solve_jacobian(self, prev_pts, curr_pts, dt, prev_quat=None, curr_quat=None,
                         prev_angvel=None, curr_angvel=None):
        # Level EACH frame's points with THAT frame's own quaternion -- img_data.py's
        # comment on this exact point ("aruco_pts_0 belongs to frame-0 -> level with
        # quats[0], not quats[1]") warns that using the wrong quat leaves a residual
        # tilt proportional to angular rate, a source of yaw/rate leakage.
        prev_n = self._getVirtualPts(prev_pts, prev_quat)
        curr_n = self._getVirtualPts(curr_pts, curr_quat)
        vel = (curr_n - prev_n) / dt   # (N,2) per-point normalized velocity
        A = _fill_A(prev_n)
        b = vel.reshape(-1)

        # GYRO DE-ROTATION (2026-08-08): the full 6-unknown solve is geometrically
        # rank-deficient at this marker's achievable point spread -- at small (x,y) the
        # Wy column [1+x^2, xy] converges to [1,0], numerically ALIASING the constant Tx
        # column [1,0] (same mechanism for Wx vs Ty). Confirmed via an independent
        # gyro-vs-image-Jacobian correlation check: raw h_Tx correlates -0.99 with raw
        # w_Wy in calibration data, while TRUE gyro Wy correlates only +0.04 with h_Tx --
        # the near-total apparent collinearity is a per-frame LSTSQ aliasing artifact, not
        # a real physical relationship, so it can't be fixed by better excitation, only by
        # not jointly fitting the aliased pair. Wx/Wy are independently, accurately known
        # from the gyro (already plumbed: gz_subscriber.Image_Node._angvel_deque, synced
        # to the flow-point pair the same way quat is) -- SUBTRACT their exactly-known
        # geometric contribution from the observed flow, then solve a REDUCED 4-unknown
        # problem [Tx,Ty,Tz,Wz] from the remaining (well-conditioned -- Tx,Ty,Tz's columns
        # are position-independent constants/linear, Wz's [-y,x] doesn't alias with them)
        # columns. Wz stays a SOLVED unknown (not gyro-substituted) -- unlike Wx/Wy it
        # isn't aliased with translation, only low-sensitivity at small extent (a genuine
        # marker-size-limited SNR ceiling, not a collinearity bug -- see
        # feedback_cross_marker_radial_spread_ceiling / this file's Wz-row comments).
        # Falls back to the old full 6-unknown solve when gyro isn't available (matches
        # _getVirtualPts's quat=None graceful-degradation convention).
        w_V = None
        if prev_angvel is not None and curr_angvel is not None and prev_quat is not None:
            w_body = 0.5 * (np.asarray([prev_angvel.forward_rad_s, prev_angvel.right_rad_s,
                                         prev_angvel.down_rad_s]) +
                             np.asarray([curr_angvel.forward_rad_s, curr_angvel.right_rad_s,
                                         curr_angvel.down_rad_s]))
            w_V = self._vframe_w(w_body, prev_quat)   # same basis prev_n/A are built in
            b_derot = b - A[:, [3, 4]] @ w_V[[0, 1]]
            A_reduced = A[:, [0, 1, 2, 5]]
            sol_reduced, *_ = np.linalg.lstsq(A_reduced, b_derot, rcond=None)
            sol = np.array([sol_reduced[0], sol_reduced[1], sol_reduced[2],
                             w_V[0], w_V[1], sol_reduced[3]])
            cond = np.linalg.cond(A_reduced)
        else:
            sol, *_ = np.linalg.lstsq(A, b, rcond=None)
            cond = np.linalg.cond(A)
        # In-sample fit quality (2026-08-02, LK-correspondence-noise investigation):
        # relative residual = ||A@sol - b|| / ||b||. LOW means the tracked points
        # broadly AGREE with each other on a single global rigid/affine flow (so if
        # GT correlation is still poor, the bug is a systematic scale/sign/frame
        # issue elsewhere, not per-point tracking noise). HIGH means the points
        # themselves are inconsistent -- direct evidence of bad LK correspondences
        # (e.g. the speckle texture's self-similarity causing mismatches), which no
        # amount of recalibration downstream could fix.
        b_norm = np.linalg.norm(b)
        rel_resid = np.linalg.norm(A @ sol - b) / b_norm if b_norm > 1e-12 else np.nan
        # (rel_resid measures the ORIGINAL joint-solve fit quality -- computed BEFORE
        # the moment-loom Tz override below, so it stays a clean diagnostic of the
        # pinv fit regardless of whether the override fires.)

        # MOMENT-LOOM Tz OVERRIDE (2026-08-12, validated on real flip-event data --
        # see project_20260812_cross_marker_flow_architecture_investigation memory
        # §3). Ported from img_data.py's ring-moment: Tz as a closed-form scalar
        # from the rate of change of the tracked point cloud's own spread (mean
        # squared distance from its own centroid, in V-frame), instead of reading
        # it out of the joint pseudo-inverse. On 3 real documented sign-flip/
        # overshoot events this recovered the correct sign (pinv wrongly flipped
        # positive twice; moment-loom stayed correctly negative both times) and
        # matched GT more closely once outlier rejection was added -- see below.
        # Does NOT fix the SEPARATE, larger near-touchdown dynamic-range problem
        # (perceived h_z pinned well below true GT magnitude) -- that is a known,
        # still-open gap, not something this override addresses.
        #
        # MAD-based outlier rejection (SAME pattern/threshold as img_data.py's
        # ring-moment, k=3.0*1.4826 on per-point flow magnitude) is NOT optional:
        # confirmed on real data that 2 of 15 genuinely mistracked points (15-40x
        # every other point's displacement) skewed the unweighted mean badly
        # enough to make moment-loom WORSE than the pinv solve without it
        # (-1.151 vs pinv's -0.511, GT=-0.567); with rejection, -0.617, between
        # pinv and GT.
        #
        # Gyro-derotation tested and NOT applied here: at this dataset's highest
        # observed rotation rate (Wx=0.284 rad/s), derotating shifted the moment
        # estimate by only +0.0040 -- negligible against the actual problem scale.
        #
        # The joint solve above is UNCHANGED -- per user directive, Tz's column
        # stays in the pinv fit (confirmed via SVD that removing it wouldn't
        # affect Tx/Ty/Wz conditioning anyway, but the fit SHAPE is kept as-is
        # regardless); this only overrides the reported sol[2].
        #
        # 2026-08-12: default LOWERED 8->6, per a live-flight miss found right
        # after first deploying this override -- a real flip frame had n=7
        # (below the original threshold of 8), so the override never fired and
        # fell back to the wrong pinv value (+0.248); recomputing offline showed
        # moment-loom WOULD have given -0.619 (correct sign, close to GT -0.599)
        # had it been allowed to run. 6 is still 2 above MIN_FLOW_POINTS_SOLVE=4
        # (kept some margin above the absolute pinv floor for the mean-based
        # moment to have a little averaging benefit) but no longer excludes this
        # specific documented case. Not yet re-validated at scale -- watch for
        # whether 6 introduces its OWN instability (a smaller n makes the mean
        # itself noisier) on the next batch of live flights.
        moment_min_pts = int(os.environ.get("CROSS_MOMENT_LOOM_MIN_PTS", "6"))
        if len(prev_n) >= moment_min_pts:
            fm = np.linalg.norm(curr_n - prev_n, axis=1)
            med = np.median(fm)
            mad = np.median(np.abs(fm - med)) + 1e-6
            valid = fm < med + 3.0 * 1.4826 * mad
            if valid.sum() >= max(4, moment_min_pts // 2):
                pv, cv_ = prev_n[valid], curr_n[valid]
                c0, c1 = pv.mean(0), cv_.mean(0)
                M0 = float(np.mean(np.sum((pv - c0) ** 2, axis=1)))
                M1 = float(np.mean(np.sum((cv_ - c1) ** 2, axis=1)))
                # ORIGIN-SPREAD GATE (2026-08-13, user-proposed, quantified same day --
                # see project_20260812_cross_marker_flow_architecture_investigation
                # memory's t=39.444 case). Tz's per-point contribution is -Tz*(x,y):
                # when the validated points all sit at nearly the SAME (x,y) -- i.e.
                # clustered in one region FAR from the image origin, regardless of how
                # internally consistent their tracking is -- that contribution is
                # nearly IDENTICAL for every point, which looks like a uniform shift
                # of the whole cluster, not a spread change. Centroid-relative variance
                # is insensitive to a uniform shift BY CONSTRUCTION (same reason Tx/Ty
                # don't affect it), so the real Tz signal barely reaches this
                # computation in that geometry, regardless of point count or
                # cleanliness. ratio = M0 (the cluster's own internal spread) /
                # ||centroid||^2 (how far the whole cluster sits from the origin)
                # quantifies this directly: a real, decisive separation was measured
                # on 4 real documented cases -- the one KNOWN-FAILED case (wrong sign
                # despite clean, consistent tracking) had ratio=0.028; all 3 KNOWN-
                # WORKING cases had ratio 5.3-8823, orders of magnitude higher.
                # CROSS_MOMENT_LOOM_MIN_ORIGIN_RATIO=1.0 sits with huge margin on both
                # sides of that measured gap. Below it, hold the pinv Tz instead --
                # moment-loom has no real signal to work with in this geometry, no
                # matter how the point set is otherwise filtered/cleaned.
                origin_dist2 = float(np.sum(c0 ** 2))
                origin_ratio = M0 / max(origin_dist2, 1e-9)
                min_origin_ratio = float(os.environ.get("CROSS_MOMENT_LOOM_MIN_ORIGIN_RATIO", "1.0"))
                if M0 > 1e-12 and M1 > 1e-12 and dt > 0 and origin_ratio >= min_origin_ratio:
                    sol[2] = float(np.clip(-0.5 * (np.log(M1) - np.log(M0)) / dt, -20.0, 20.0))
        # else: too few points for a stable moment estimate (untested below
        # ~8-10 points) -- hold the pinv-solved Tz unmodified.

        px_disp = np.linalg.norm(curr_pts - prev_pts, axis=1)   # raw pixel displacement per point
        # TEMP DIAG (2026-08-03, Wx/Wy investigation): radial extent of the
        # normalized points actually fed into A -- max/mean |x|,|y| in prev_n.
        # w1/w2 (cols 3,4 of A) are QUADRATIC in x,y (x*y, x^2, y^2), unlike h3/w3
        # (cols 2,5) which are linear -- so w1/w2 need much larger radial spread
        # for equivalent leverage/conditioning. Logging this to check if the
        # cross-marker's actual tracked-point spread is the limiting factor.
        self._radial_diag = (float(np.max(np.abs(prev_n[:, 0]))),
                              float(np.max(np.abs(prev_n[:, 1]))),
                              float(np.mean(np.abs(prev_n[:, 0]))),
                              float(np.mean(np.abs(prev_n[:, 1]))))
        # TEMP DIAG 2026-08-09: full point set + solved Tz, see __init__'s
        # _point_diag_log comment. prev_n is what _fill_A/A_reduced were actually
        # built from; sol[2] is the Tz this exact point set produced.
        #
        # EXTENDED 2026-08-12 (sign-flip root-cause, per-point noise investigation):
        # the original 2-tuple only let us reconstruct the SOLVE (prev_n -> A -> Tz),
        # not what fed it -- couldn't tell whether a Tz swing came from genuine
        # per-point LK-tracked velocity noise (the live hypothesis) vs. something in
        # the solve itself (already ruled out separately -- the matrix is
        # well-conditioned, cond~2, and per-point leverage is evenly graded, not
        # concentrated). Added curr_n (so vel=(curr_n-prev_n)/dt is fully
        # reconstructable per point), dt (vel's own denominator -- small dt inflates
        # any fixed position-tracking jitter into larger apparent velocity noise),
        # and the full sol vector (not just Tz) for completeness.
        self._point_diag = (prev_n.copy(), float(sol[2]), curr_n.copy(), float(dt), sol.copy())
        return sol, cond, rel_resid, float(np.median(px_disp)), float(np.std(px_disp))

    def _compute_hw(self, gray_prev, gray_curr, mask, dt, quat_prev=None, quat_curr=None,
                     angvel_prev=None, angvel_curr=None):
        """LK-track flow points from THIS call's own adjacent frame pair, solve the
        image Jacobian pseudo-inverse for [h1,h2,h3,w1,w2,w3]. Two-threshold
        hysteresis (see MIN_FLOW_POINTS_SOLVE/RESAMPLE_TRIGGER comments):
        attempt the solve with however many points survived tracking, down to
        a hard floor; only DISCARD current tracking for a full fresh-sample
        when the count drops below that floor.

        REWRITTEN 2026-08-12 (dt/frame-pairing architecture fix, see
        project_20260812_cross_marker_flow_architecture_investigation memory).
        OLD BUG: this method used to take a single `gray` (this frame only) and
        rely on `self._prev_gray`/`self._prev_frame_t`/`self._prev_quat`/
        `self._prev_angvel`, persisted across calls and only updated on a
        SUCCESSFUL solve. On a detection dropout (mask unavailable that frame,
        e.g. `centroid_mismatch` near touchdown -- see cross_marker_detector.py),
        this method wasn't even called, so that persisted state froze for the
        whole dropout streak; once detection recovered, LK had to bridge the
        ENTIRE accumulated gap in one step (large real displacement, degraded
        correspondence) instead of one native frame interval, and `dt` was
        inflated to match (not wrong arithmetic, but the resulting LARGE GAP
        itself degrades LK correspondence quality -- confirmed empirically:
        chronic point starvation + degraded detection reliability compound in
        exactly this terminal-descent window).

        NEW: caller (`process_frame`) now supplies its OWN `gray_prev`/
        `gray_curr` (from `gz_subscriber.Image_Node`'s rolling 2-frame deque,
        i.e. always a genuinely adjacent pair, regardless of consumer-loop
        timing or whether detection succeeded on intervening frames) and `dt`
        (`1/fps`, the camera's own native frame interval, NEVER derived from
        "time since my own state was last updated"). `mask` is None on a
        detection-miss frame (no fresh mask to validate points against) --
        the LK step still runs (keeps tracked points CURRENT every call, never
        stale), but no resample/solve is attempted without a mask. No
        `self._prev_gray`/`_prev_frame_t`/`_prev_quat`/`_prev_angvel` state is
        read or written by this method anymore -- eliminated, not just fixed."""
        dilated_mask = self._dilate_mask(mask) if mask is not None else None

        if self._prev_flow_pts is None or len(self._prev_flow_pts) == 0:
            # cold start / pool empty -- only recoverable on a frame with a fresh
            # mask to sample candidates from
            if dilated_mask is None:
                return np.zeros(6), False
            self._prev_flow_pts, self._prev_flow_cell_id = self._sample_flow_points(gray_curr, dilated_mask)
            self._last_resample_t = self._last_t
            return np.zeros(6), False

        tracked, status, _ = cv2.calcOpticalFlowPyrLK(
            gray_prev, gray_curr, self._prev_flow_pts, None,
            winSize=LK_WIN, maxLevel=LK_MAX_LEVEL)
        status = status.flatten().astype(bool)

        # 2026-08-13 (ring sampling, paired-opposite rejection, user-proposed): if a
        # point failed LK this frame, ALSO drop its diametrically-opposite partner
        # (same band, sector -> sector+RING_N_SECTORS//2) from this frame's kept set --
        # mirrors img_data.py's PLASMC_RING_PAIRED. Rationale: a ring cell's opposite
        # partner exists specifically to balance that cell's contribution to the
        # centroid/moment; keeping the opposite alone while the pair-partner vanishes
        # re-introduces exactly the one-sided centroid bias pairing was meant to avoid.
        # Only meaningful when RING_SAMPLING populated cell ids this flight.
        if self._prev_flow_cell_id is not None and len(self._prev_flow_cell_id) == len(status):
            half = RING_N_SECTORS // 2
            failed_cells = set(self._prev_flow_cell_id[~status].tolist())
            opp_failed_cells = set()
            for c in failed_cells:
                b, s = divmod(int(c), RING_N_SECTORS)
                opp_failed_cells.add(b * RING_N_SECTORS + (s + half) % RING_N_SECTORS)
            if opp_failed_cells:
                is_opp_of_failed = np.array(
                    [cid in opp_failed_cells for cid in self._prev_flow_cell_id])
                status = status & ~is_opp_of_failed

        if dilated_mask is not None:
            # keep only points that (a) tracked successfully and (b) are still within
            # the DILATED marker mask -- on-target guarantee, but tolerant of a few
            # px of frame-to-frame mask-boundary jitter (see MASK_DILATE_PX)
            in_mask = np.zeros(len(tracked), dtype=bool)
            h, w = dilated_mask.shape
            for k, p in enumerate(tracked):
                xi, yi = int(round(p[0])), int(round(p[1]))
                if 0 <= xi < w and 0 <= yi < h:
                    in_mask[k] = dilated_mask[yi, xi] > 0
            keep = status & in_mask
        else:
            # detection miss this frame -- no fresh mask to validate on-target
            # membership against; keep whatever LK tracked successfully so the
            # point set stays current (never frozen), skip resample this frame
            keep = status

        prev_pts = self._prev_flow_pts[keep]
        curr_pts = tracked[keep]
        n_kept = len(prev_pts)
        kept_cell_id = (self._prev_flow_cell_id[keep]
                         if self._prev_flow_cell_id is not None else None)

        if dilated_mask is not None:
            # 2026-08-07: force a periodic refresh even when the pool is healthy, not just
            # when it nearly empties -- see RESAMPLE_PERIOD_S's comment. Without this, a
            # point set that never drops below RESAMPLE_TRIGGER just tracks the SAME
            # corners for the whole flight, freezing in whatever radial-spread bias that
            # one early draw happened to have (the actual root cause of the large
            # run-to-run Hz/Wz variance, not per-frame noise).
            due_for_refresh = (self._last_resample_t is None or
                                (self._last_t - self._last_resample_t) >= RESAMPLE_PERIOD_S)
            if n_kept < RESAMPLE_TRIGGER or due_for_refresh:
                # top up the pool with fresh candidates rather than discarding what's
                # still tracking -- only replace outright if the fresh yield actually
                # beats what survived (never make the pool WORSE)
                fresh, fresh_cell_id = self._sample_flow_points(gray_curr, dilated_mask)
                self._last_resample_t = self._last_t
                if len(fresh) > n_kept:
                    self._prev_flow_pts, self._prev_flow_cell_id = fresh, fresh_cell_id
                else:
                    self._prev_flow_pts, self._prev_flow_cell_id = curr_pts, kept_cell_id
            else:
                self._prev_flow_pts, self._prev_flow_cell_id = curr_pts, kept_cell_id
        else:
            # no mask to resample against; just advance tracking
            self._prev_flow_pts, self._prev_flow_cell_id = curr_pts, kept_cell_id

        if dilated_mask is None or n_kept < MIN_FLOW_POINTS_SOLVE or not (dt > 0):
            self._flow_diag_log.append((self._last_t, n_kept, np.nan, False, np.nan, np.nan, np.nan))
            return np.zeros(6), False

        sol, cond, rel_resid, px_disp_med, px_disp_std = self._solve_jacobian(
            prev_pts, curr_pts, dt, prev_quat=quat_prev, curr_quat=quat_curr,
            prev_angvel=angvel_prev, curr_angvel=angvel_curr)
        self._flow_diag_log.append((self._last_t, n_kept, cond, True, rel_resid, px_disp_med, px_disp_std))
        self._radial_diag_log.append((self._last_t,) + self._radial_diag)
        self._point_diag_log.append((self._last_t,) + self._point_diag)
        return sol, True   # [h1,h2,h3,w1,w2,w3]

    def process_frame(self, img_bgr_prev, img_bgr_curr, t, fps, quat_prev=None, quat_curr=None,
                       angvel_prev=None, angvel_curr=None):
        """REWRITTEN 2026-08-12 (dt/frame-pairing architecture fix): now takes the
        caller's own adjacent frame pair (img_bgr_prev/img_bgr_curr, from
        gz_subscriber.Image_Node's rolling 2-frame deque) and the camera's native
        fps, instead of a single frame + relying on internally-persisted "prev"
        state that could go stale across a detection dropout -- see _compute_hw's
        docstring and project_20260812_cross_marker_flow_architecture_
        investigation memory for the full mechanism/rationale."""
        # self._last_t is a general "last frame seen" timestamp (used by
        # _flow_diag_log entries etc.)
        self._last_t = t
        dt = 1.0 / fps if (isinstance(fps, (int, float)) and fps > 1) else np.nan

        # EXTENT-ADAPTIVE ROI (2026-08-04): decided INSIDE detect() itself now, from the
        # blobby-stage mask's own largest-component extent (see cross_marker_detector's
        # ROI_FRAC_X_DEFAULT/_LOOSENED comment) -- a caller-side decision based on
        # self._last_bbox (the last SUCCESSFUL detection) hit a bootstrap deadlock: if
        # the tight crop truncates the marker right as it crosses the adaptive
        # threshold, successful detections never record the marker's true (larger)
        # size, so the loosened crop never triggers. Computing it fresh from
        # ROI-independent, already-ghost-filtered data every call avoids that.
        det = cmd.detect(img_bgr_curr, track_state=self._track_state)
        gray_curr = cv2.cvtColor(img_bgr_curr, cv2.COLOR_BGR2GRAY)
        gray_prev = cv2.cvtColor(img_bgr_prev, cv2.COLOR_BGR2GRAY)
        bbox_area = (det.mask_bbox[2] * det.mask_bbox[3]) if det.mask_bbox else 0
        self._diag_log.append((t, bool(det.ok), det.fail_reason, bbox_area))
        self._diag_frame_idx += 1

        if not det.ok:
            self._ok = False
            self._center_fresh = False   # explicit: this frame did NOT confirm the center is
                                          # in-frame -- the CBF must not treat a stale self._center_px
                                          # as a live in-FoV measurement (see get_center_px())
            # 2026-08-12: still advance LK point tracking (mask=None -- see _compute_hw's
            # docstring) so the tracked point set never goes stale across a dropout streak,
            # even though no Jacobian solve can be committed without a fresh mask this frame.
            self._compute_hw(gray_prev, gray_curr, None, dt, quat_prev=quat_prev, quat_curr=quat_curr,
                              angvel_prev=angvel_prev, angvel_curr=angvel_curr)
            # hold last s/alpha; hw defaults to zero (no motion info without a fix)
            self._hw = np.zeros(6)
            self._diag_lost_streak += 1
            # dump the frame at the START of a loss streak (streak==1) and every 30
            # frames thereafter, so a long freeze is sampled across its duration
            # instead of just once -- lets us see whether the cause is static (same
            # failure mode throughout) or drifts (e.g. lighting/tilt changing).
            if self._diag_save_dir and (self._diag_lost_streak == 1 or self._diag_lost_streak % 30 == 0):
                fn = os.path.join(self._diag_save_dir,
                                   f"lost_{self._diag_frame_idx:06d}_streak{self._diag_lost_streak}_{det.fail_reason}.png")
                cv2.imwrite(fn, img_bgr_curr)
            # Log even on detection misses
            self._log_frame_data(t, quat_curr, hw_ok=False, det=det)
            return self.get_output()

        if self._diag_save_dir and self._diag_lost_streak >= 5:
            # recovery frame right after a real (>=5-frame) loss streak
            fn = os.path.join(self._diag_save_dir,
                               f"recovered_{self._diag_frame_idx:06d}_afterstreak{self._diag_lost_streak}.png")
            cv2.imwrite(fn, img_bgr_curr)
        self._diag_lost_streak = 0

        cx, cy = det.center
        # Level the centroid into the SAME V-frame compute_gt_signals' GT is in
        # (2026-08-02 fix -- was raw camera-frame, tilt-uncompensated, same gap as h,w).
        s_xy = self._getVirtualPts(np.array([[cx, cy]]), quat_curr)[0]
        # PLAUSIBILITY GATE (2026-08-03, s_e_n single-frame-spike investigation): confirmed
        # via Z_V_DIAG that a transient tilt can push the centroid ray's z_v near/below zero
        # for exactly one frame -- _getVirtualPts's own comment already predicted this would
        # "blow up the perspective divide into a huge or sign-flipped point" invisibly (no
        # existing diagnostic caught it). Reproduced: min_z_v as low as -1.33, output up to
        # 41x normal magnitude. That single-frame garbage s fed sigma -> the adaptive-gain
        # ODE ratcheted kappa 0.29->3.18 in ~130ms and never recovered (frozen there by the
        # staleness gate once perception subsequently dropped out), driving a real a_u/accel
        # spike that falsely triggered the impact detector well above the marker. self._z_v_log
        # was just appended by the call above (nothing else calls _getVirtualPts in between),
        # so [-1] is this exact centroid ray's min z_v -- treat like a genuine miss (same
        # fields as the `not det.ok` branch above) rather than accept a point from a ray
        # that's behind/grazing the virtual camera.
        _z_v_centroid = self._z_v_log[-1] if self._z_v_log else float('nan')
        if not (np.isfinite(_z_v_centroid) and _z_v_centroid > Z_V_MIN_CENTROID):
            self._diag_z_v_reject_count = getattr(self, '_diag_z_v_reject_count', 0) + 1
            self._ok = False
            self._center_fresh = False
            # 2026-08-12: same LK-continuity rationale as the not-det.ok branch above --
            # this is a genuine miss (degenerate centroid ray), treat identically.
            self._compute_hw(gray_prev, gray_curr, None, dt, quat_prev=quat_prev, quat_curr=quat_curr,
                              angvel_prev=angvel_prev, angvel_curr=angvel_curr)
            self._hw = np.zeros(6)
            self._diag_lost_streak += 1
            return self.get_output()

        self._center_px = np.array([cx, cy])
        self._center_fresh = True
        if det.mask_bbox is not None:
            self._last_bbox = det.mask_bbox
        self._s = np.array([s_xy[0], s_xy[1], 1.0])

        # --- alpha: unweighted moment over REAL detected arm/stub pixels ---
        pts_for_alpha = list(det.line_points_i) + list(det.line_points_j)
        if det.stub_points is not None and len(det.stub_points) >= 2:
            stub_pts_px = np.asarray(det.stub_points, dtype=np.float64)
            arm_pts_px = np.asarray(pts_for_alpha, dtype=np.float64)
            # V-FRAME LEVELING FIX (2026-08-08): alpha previously used ONLY the flat
            # [y,-x] camera-mount-yaw axis swap (a fixed relabeling), never the
            # quaternion-based gravity-leveling _getVirtualPts reprojection that s
            # (line ~745 above) and h,w (_solve_jacobian) both got on 2026-08-02.
            # Root-caused via an independent-multisine validation: alpha vs GT yaw
            # correlation FLIPPED SIGN between low-tilt (<1.7deg, r=+0.31) and
            # high-tilt (>3.7deg, r=-0.87) subsets -- the un-leveled principal-angle
            # moment was picking up tilt-induced foreshortening of the cross+stub
            # shape as spurious rotation, dominating the true-yaw signal whenever the
            # drone pitched/rolled to translate (i.e. throughout most of a real
            # flight). _getVirtualPts already performs the equivalent mount-rotation
            # axis handling internally (both its quat=None fallback and quat-present
            # branches return [y,-x]-style V-frame coordinates), so the old standalone
            # [y,-x] swap is REPLACED here, not stacked on top.
            stub_pts = self._getVirtualPts(stub_pts_px, quat_curr)
            arm_pts = self._getVirtualPts(arm_pts_px, quat_curr)
            all_pts = np.vstack([arm_pts, stub_pts])
            a_raw, _ = _unweighted_principal_angle(all_pts)
            asym = (float(stub_pts[:, 0].mean() - arm_pts[:, 0].mean()),
                    float(stub_pts[:, 1].mean() - arm_pts[:, 1].mean()))
            a_disamb = _disambiguate_angle(a_raw, asym)
            # ALPHA_0 offset (see __init__'s self._alpha_0 comment) -- same
            # subtract-then-wrap convention as img_data.py's _moment_alpha_0.
            a = float(np.arctan2(np.sin(a_disamb - self._alpha_0), np.cos(a_disamb - self._alpha_0)))
            self._last_alpha = a
            self._alpha_valid_once = True
        # else: stub not found this frame -- hold last-good alpha (no fresh compute)
        self._alpha = self._last_alpha if self._alpha_valid_once else 0.0

        # --- h, w: image Jacobian solve over Shi-Tomasi points on the marker plate ---
        mask = det.isolated_mask if det.isolated_mask is not None else np.zeros(gray_curr.shape, np.uint8)
        hw, hw_ok = self._compute_hw(gray_prev, gray_curr, mask, dt, quat_prev=quat_prev, quat_curr=quat_curr,
                                      angvel_prev=angvel_prev, angvel_curr=angvel_curr)
        self._hw = hw if hw_ok else np.zeros(6)

        self._ok = True

        # Log full time-series (2026-08-04, frame-by-frame debugging)
        self._log_frame_data(t, quat_curr, hw_ok)

        return self.get_output()

    def _log_frame_data(self, t, quat, hw_ok, det=None):
        """Log full frame-by-frame state for time-series analysis."""
        try:
            self._time_log.append(float(t) if t is not None else np.nan)
            # Log quat as tuple (w,x,y,z) to avoid pickle issues
            if quat is not None:
                self._quat_log.append(tuple([float(getattr(quat, k, np.nan)) for k in ['w','x','y','z']]))
            else:
                self._quat_log.append(None)
            self._s_log.append(self._s.copy())
            self._hw_log.append(self._hw.copy())
            self._alpha_log.append(float(self._alpha))
            # hw_ok tracks whether LK flow succeeded (for debugging corner loss)
            self._n_flow_corners_log.append(int(len(self._prev_flow_pts)) if self._prev_flow_pts is not None else 0)
            self._feature_visibility_log.append(bool(self._ok))
            self._detection_reason_log.append("ok" if self._ok else "miss")
            # Marker extent from bbox (scale-free proximity proxy)
            if self._last_bbox is not None:
                extent = float(max(self._last_bbox[2], self._last_bbox[3]))  # max(w, h)
            else:
                extent = 0.0
            self._marker_extent_log.append(extent)
            # Center px as simple tuple
            if self._center_px is not None:
                self._center_px_log.append(tuple(float(x) for x in self._center_px))
            else:
                self._center_px_log.append((np.nan, np.nan))

            # 2026-08-05: parity fields (see __init__'s comment for scope). angvel/fps/
            # stamp are set as instance attrs by CrossMarkerNode.run() just before
            # calling process_frame() -- avoids changing process_frame()'s own call
            # signature (other callers, e.g. offline replay scripts, call it directly).
            _av = getattr(self, '_pending_angvel', None)
            if _av is not None:
                self._imu_angvel_log.append((float(_av.forward_rad_s), float(_av.right_rad_s), float(_av.down_rad_s)))
            else:
                self._imu_angvel_log.append((np.nan, np.nan, np.nan))
            self._fps_log.append(float(getattr(self, '_pending_fps', np.nan)))
            self._stamp_log.append(float(getattr(self, '_pending_stamp', np.nan)))

            # Raw feature points this frame (arm + stub lines), if a fresh detection
            # happened -- held empty on a miss (no fresh points to report).
            if det is not None and det.ok:
                _raw_pts = list(det.line_points_i) + list(det.line_points_j)
                if det.stub_points is not None:
                    _raw_pts += list(det.stub_points)
                self._feature_pts_raw_log.append(np.asarray(_raw_pts, dtype=np.float64))
            else:
                self._feature_pts_raw_log.append(np.zeros((0, 2)))

            # Calibrated feature param [xc,yc,1,alpha] (getImgFeatureParam()'s own
            # output) -- distinct from _s_log/_alpha_log, which are pre-calibration.
            self._img_feature_param_log.append(
                self._sensor_cal_s @ np.array([self._s[0], self._s[1], self._s[2], self._alpha]))
        except Exception as e:
            # If logging fails, don't crash the perception pipeline
            print(f"[CrossMarkerNode._log_frame_data] warning: {e}")

    def get_output(self):
        return dict(ok=self._ok, s=self._s.copy(), alpha=self._alpha, hw=self._hw.copy())

    # ---- getters mirroring IMG_PROCESSOR's public interface, for controller.py ----
    def getRawImgFeatureParam(self):
        """Uncalibrated [xc, yc, 1, alpha] -- what the calibration recorder logs."""
        return np.array([self._s[0], self._s[1], self._s[2], self._alpha])

    def getRawOptFlowAngVel(self):
        """Uncalibrated [h1,h2,h3,w1,w2,w3] -- what the calibration recorder logs."""
        return self._hw.copy()

    def getImgFeatureParam(self):
        return self._sensor_cal_s @ self.getRawImgFeatureParam()

    def getOptFlowAngVel(self):
        return self._sensor_cal_hw @ self.getRawOptFlowAngVel()

    def get_diag_log(self):
        """List of (t, ok, fail_reason, bbox_area) -- one entry per process_frame call."""
        return list(self._diag_log)

    def get_flow_diag_log(self):
        """List of (t, n_kept, cond(A), solved, rel_resid, px_disp_median, px_disp_std)
        -- one entry per flow-solve attempt (only logged on det.ok=True frames, since
        _compute_hw only runs then)."""
        return list(self._flow_diag_log)

    def get_radial_diag_log(self):
        """TEMP DIAG 2026-08-03: (t, max|x|, max|y|, mean|x|, mean|y|) of the
        normalized points fed into A, per successful solve."""
        return list(self._radial_diag_log)

    def get_z_v_log(self):
        """List of min(z_v) per _getVirtualPts call (2 per flow solve: prev_pts,
        curr_pts, plus 1 for the centroid) -- degeneracy diagnostic."""
        return list(self._z_v_log)

    def get_point_diag_log(self):
        """TEMP DIAG 2026-08-09, EXTENDED 2026-08-12: list of
        (t, prev_n, solved_Tz, curr_n, dt, sol) per successful solve -- see
        __init__'s _point_diag_log comment. prev_n/curr_n are the (N,2) normalized
        V-frame points _fill_A/A were built from (before/after this solve's LK
        step); per-point velocity is fully reconstructable as
        (curr_n-prev_n)/dt. sol is the full solved [Tx,Ty,Tz,Wx,Wy,Wz] vector
        (not just Tz). Per-point-resolution follow-up to get_radial_diag_log()."""
        return list(self._point_diag_log)

    def get_center_px(self):
        """(2,) raw pixel [cx, cy] for the visibility CBF, or None if this frame
        did not confirm the center is in-frame (see _center_fresh in
        process_frame) -- the CBF must see None on a genuine miss, not a
        silently-stale pixel value, so its own Phase-2 fallback engages
        correctly rather than being fed a frozen "live" measurement."""
        if not self._center_fresh:
            return None
        return self._center_px.copy()

    def get_bbox_corners(self):
        """4 corner points (4,2) of the last successful color-gated mask bbox, in the
        (x,y,w,h) -> [[x,y],[x+w,y],[x,y+h],[x+w,y+h]] convention MARKER_EXTENT_PX
        (controller.py) expects -- or None if never detected yet."""
        if self._last_bbox is None:
            return None
        x, y, w, h = self._last_bbox
        return np.array([[x, y], [x + w, y], [x, y + h], [x + w, y + h]], dtype=float)

    @property
    def FEATURE_IS_VISIBLE(self):
        return self._ok


class CrossMarkerNode(Thread):
    """Thread wrapper around CrossMarkerPerception, exposing the minimal subset
    of IMG_PROCESSOR's public interface that controller.py actually calls
    (catalogued directly from controller.py's `self._img_node.*` usages,
    2026-08-01) -- NOT a full IMG_PROCESSOR replacement. Attributes tied to
    ArUco-specific subsystems this design deliberately drops (ring-loom
    fusion, PlanarFeatureMap rescue, marker handover, terminal-kick gating)
    are NOT implemented; if controller.py code paths that touch those are
    exercised under MARKER_TYPE=cross (e.g. a real descent/terminal-kick,
    vs. the hover-only sanity path this was validated against), expect
    AttributeErrors there -- that is a known, currently out-of-scope gap,
    not a silent failure.

    Reuses gz_subscriber.Image_Node verbatim for capture -- it is decode-
    agnostic (raw BGR frame + the existing 90deg-CW rotation convention),
    no ArUco logic lives in it.
    """

    def __init__(self, time_keeper=time, controller=None):
        Thread.__init__(self)
        self._time = time_keeper
        self._FC = controller
        self._image_node = Image_Node(time_keeper=time_keeper, controller=controller)
        self._image_sub = GZ_Subscriber(self._image_node)

        self._resolution = self._image_node.getImgResolution()   # blocks until first frame
        self._perception = CrossMarkerPerception(resolution=self._resolution)
        self.center = self._perception.center
        self.focal = self._perception.focal

        # plain-attribute stand-ins for controller.py's other self._img_node.* touches
        self.RECORD = os.environ.get("IMG_RECORD", "0") == "1"   # IMG_RECORD=1 saves the descent video
        self.CONTROLLER_READY = False
        self._feature_pts = deque(maxlen=5)   # only fp_list[-1] is ever read (MARKER_EXTENT_PX)
        self._ring_loom_source = "n/a"
        # Live cv2.VideoWriter.write() from this background thread was found to silently
        # drop frames (153 write() calls -> 1 decodable frame on disk, headless/offscreen
        # OpenCV video backend + non-main-thread write is unreliable) -- dump raw PNG
        # frames instead (same pattern as img_data.py's IMG_RECORD_RAW) and stitch to mp4
        # in close(), single-threaded, after the flight.
        self._rec_dir = None
        self._rec_n = 0
        # WRITER QUEUE (2026-08-09): synchronous cv2.imwrite() inside this SAME thread's
        # run() loop (the one also driving process_frame()) was found to destabilize the
        # vehicle specifically near touchdown -- every IMG_RECORD=1 landing-validation
        # flight crashed hard at the very end (impact forces up to 329 m/s^2) while the
        # identical profile landed cleanly with IMG_RECORD=0. Per-frame disk I/O blocking
        # this thread delays how promptly the next frame gets processed and FEATURE_IS_
        # VISIBLE/flow gets updated, exactly when position tracking needs to be tightest
        # (touchdown). Fix: enqueue a frame copy and let a SEPARATE writer thread own the
        # actual cv2.imwrite() calls, so this thread never blocks on disk I/O. Queue is
        # unbounded by design -- frames are small/compressed fast enough in practice that
        # it drains continuously, and dropping frames under backpressure would silently
        # corrupt the recording in a way that's hard to notice, worse than using more RAM
        # for the duration of one flight.
        self._rec_queue = queue.Queue() if self.RECORD else None
        self._rec_writer_thread = None
        if self.RECORD:
            self._rec_writer_thread = Thread(target=self._rec_writer_loop, daemon=True)
            self._rec_writer_thread.start()

        self._running = True
        self.start()

    def _rec_writer_loop(self):
        """Background writer thread (2026-08-09, see _rec_queue's __init__ comment) --
        owns ALL disk I/O for IMG_RECORD, off the frame-processing thread's hot path."""
        while True:
            item = self._rec_queue.get()
            if item is None:   # sentinel, see close()
                self._rec_queue.task_done()
                break
            idx, frame = item
            cv2.imwrite(f'{self._rec_dir}/f{idx:05d}.png', frame, [cv2.IMWRITE_PNG_COMPRESSION, 1])
            self._rec_queue.task_done()

    def run(self):
        last_stamp = None
        while self._running:
            try:
                stamp = self._image_node.getStamp()
                if stamp is None or stamp == last_stamp:
                    time.sleep(0.002)
                    continue
                last_stamp = stamp
                imgs = self._image_node.getImages()
                if imgs[0] is None or imgs[1] is None:
                    time.sleep(0.002)
                    continue
                # Use the frame's OWN capture stamp (msg.header.stamp, sim time,
                # already fetched above via getStamp()) for dt, not perf_counter()
                # at the polling callback -- img_data.py's own comment documents
                # why: perf_counter() is quantized to the ~250 Hz sim clock and
                # jitters the apparent frame interval (62/83/125 Hz instead of a
                # clean ~60), which directly corrupts vel = (curr_n-prev_n)/dt.
                # Found 2026-08-02 during the Hx/Hy flow-accuracy investigation --
                # this module had reproduced the jittered pattern fresh instead of
                # the stamp-based fix, same class of mistake as the resolution
                # transpose bug (see feedback_duplicated_math_diff_check).
                #
                # quat: live attitude for the V-frame gravity-leveling transform
                # (_getVirtualPts) -- img_data.py's IMG_PROCESSOR does this for
                # every point; this module never did until this same 2026-08-02
                # investigation traced the missing step as the likely dominant
                # remaining cause of Hx/Hy weakness (raw camera-frame flow vs
                # V-frame GT is comparing different physical quantities under
                # any real tilt).
                #
                # FIRST ATTEMPT (regressed Hx instead of fixing it) called
                # self._FC.getQuat() separately here -- measured lag against the
                # frame's own stamp was small (<=28ms) so that wasn't the whole
                # story. Root cause: img_data.py NEVER calls FC.getQuat() from its
                # own polling loop at all -- Image_Node.image_callback samples
                # quat = self._FC.getQuat() SYNCHRONOUSLY inside the same callback
                # that captures the frame, storing both into paired deques
                # (self._img_deque / self._quat_deque, appended back-to-back) --
                # see gz_subscriber.py's image_callback. getQuaternions() returns
                # that ALREADY-paired quat, tightly synced to imgs[-1] by
                # construction, not independently re-polled later from a separate
                # thread with its own scheduling jitter. Use that instead.
                quats = self._image_node.getQuaternions()
                # Gyro body rate, SAME synchronous-in-callback pairing as quat (see
                # gz_subscriber.Image_Node.image_callback -- angvel is sampled in the
                # same callback as quat/the frame, already deque-paired identically) --
                # feeds _solve_jacobian's gyro-derotation fix (2026-08-08).
                angvels = self._image_node.getAngVels()
                # 2026-08-12 (dt/frame-pairing architecture fix, see
                # project_20260812_cross_marker_flow_architecture_investigation memory):
                # pass the FULL adjacent frame pair (imgs[0]=prev, imgs[1]=curr) plus the
                # native-rate fps, instead of only the newest frame. Previously
                # process_frame() only ever saw imgs[-1] and relied on its own
                # self._prev_gray/_prev_frame_t persisted across calls -- which went stale
                # across a detection-dropout streak (self._prev_gray frozen at whatever
                # frame the LAST SUCCESSFUL solve used, so LK ended up bridging the WHOLE
                # dropout gap in one step once detection recovered, instead of one native
                # frame interval). imgs[0]/imgs[1]/getFPS() are always mutually consistent
                # (updated together in the same image_callback), so every call now gets a
                # correct, small, native dt by construction -- see img_data.py's own
                # imgs[0]/imgs[1]/self._fps pattern, which this mirrors.
                fps = self._image_node.getFPS()
                self._perception.process_frame(
                    imgs[0], imgs[1], stamp, fps,
                    quat_prev=quats[0] if quats else None, quat_curr=quats[1] if quats else None,
                    angvel_prev=angvels[0] if angvels else None, angvel_curr=angvels[1] if angvels else None)

                # Feed MARKER_EXTENT_PX (controller.py) -- reads self._img_node._feature_pts
                # as fp_list[-1][1], a (4,2) corner array (ArUco's [prev,curr]-paired
                # convention; only [1] is ever read there). Without this, MARKER_EXTENT_PX
                # is permanently 0.0 under MARKER_TYPE=cross, silently disabling the
                # marker-fills-FoV terminal-commit trigger (_terminalCommitStep).
                _corners = self._perception.get_bbox_corners()
                if _corners is not None:
                    self._feature_pts.append((None, _corners))

                # IMG_RECORD=1 -> save the descent video (mirrors img_data.py's IMG_PROCESSOR;
                # gated the same way: only once the controller has engaged).
                if self.RECORD and self.CONTROLLER_READY:
                    frame = imgs[-1]
                    if self._rec_dir is None:
                        self._rec_ts = time.ctime().replace(':', '-')
                        self._rec_dir = ('/home/shubham/Soft-Precise-Landing/PX4_Gazebo/'
                                          f'test_data/Test_Videos/{self._rec_ts}_raw')
                        os.makedirs(self._rec_dir, exist_ok=True)
                        _fps = self._image_node.getFPS()
                        self._rec_fps = _fps if (isinstance(_fps, (int, float)) and _fps > 1) else 30.0
                    # Enqueue only -- the actual cv2.imwrite() happens on _rec_writer_loop's
                    # own thread (see _rec_queue's __init__ comment). .copy() because `frame`
                    # is imgs[-1], a live deque slot that gets overwritten by the next
                    # image_callback -- without a copy the writer thread could see a
                    # different frame's data than what was queued (a torn/overwritten array).
                    self._rec_queue.put((self._rec_n, frame.copy()))
                    self._rec_n += 1
            except Exception as e:
                print(f"[CrossMarkerNode] frame processing error: {e}")
                time.sleep(0.01)

    def getImgFeatureParam(self):
        return self._perception.getImgFeatureParam()

    def getOptFlowAngVel(self):
        return self._perception.getOptFlowAngVel()

    def getRawImgFeatureParam(self):
        return self._perception.getRawImgFeatureParam()

    def getRawOptFlowAngVel(self):
        return self._perception.getRawOptFlowAngVel()

    def get_diag_log(self):
        return self._perception.get_diag_log()

    def get_flow_diag_log(self):
        return self._perception.get_flow_diag_log()

    def get_radial_diag_log(self):
        return self._perception.get_radial_diag_log()

    def get_point_diag_log(self):
        return self._perception.get_point_diag_log()

    def get_z_v_log(self):
        return self._perception.get_z_v_log()

    def get_center_px(self):
        return self._perception.get_center_px()

    @property
    def FEATURE_IS_VISIBLE(self):
        return self._perception.FEATURE_IS_VISIBLE

    @property
    def RESCUE_ACTIVE(self):
        """Always False: RESCUE_ACTIVE is the ArUco PlanarFeatureMap rescue signal
        (img_data.py) -- this pipeline has no rescue/map fallback (see class docstring),
        so it can never be "rescuing" a frame. Stubbed so apps/landing_test.py's
        feature_fresh OR-check doesn't AttributeError under MARKER_TYPE=cross."""
        return False

    def getLogData(self):
        """Return full time-series logs for frame-by-frame comparison with GT."""
        return {
            "Time": self._perception._time_log,
            "Quat": self._perception._quat_log,
            "s_V": self._perception._s_log,         # centroid in V-frame
            "h_V": self._perception._hw_log,        # optical flow (raw, before cal)
            "alpha(t)": self._perception._alpha_log,
            "N Flow Corners": self._perception._n_flow_corners_log,
            "FEATURE_IS_VISIBLE": self._perception._feature_visibility_log,
            "Detection Status": self._perception._detection_reason_log,
            "MARKER_EXTENT_PX": self._perception._marker_extent_log,
            "Center Px": self._perception._center_px_log,
            # 2026-08-12 (hard-landing sign-flip reconstruction): previously only the
            # calibration/validation recorder apps exposed these via their own explicit
            # get_point_diag_log()/get_radial_diag_log() calls into Ground_Truth.npy --
            # the real landing-test path (this getLogData(), saved as Img_Data.npy by
            # apps/landing_test.py) never did, so a real flight's terminal-descent point
            # geometry couldn't be reconstructed after the fact. Both logs are already
            # accumulated by the underlying CrossMarkerPerception regardless of caller;
            # just exposing them here.
            "Point Diag Log": self._perception.get_point_diag_log(),
            "Radial Diag Log": self._perception.get_radial_diag_log(),
        }

    def getParams(self):
        # img_data.py's IMG_PROCESSOR.getParams() returns a str (apps/landing_test.py
        # writes it verbatim via f.write()) -- match that contract, not a dict.
        return f"{{'center':{tuple(self.center)}, 'focal':{tuple(self.focal)}}}"

    def getFailureCause(self):
        """Always "UNKNOWN": DRIFT_OFF/OVERFLOW are img_data.py's ring-loom-fusion-derived
        tags (self._last_drifted_off/_last_overflow), a signal this single-marker pipeline
        has no counterpart for (see class docstring) -- matches img_data.py's own fallback
        return when neither ArUco-specific signal fired."""
        return "UNKNOWN"

    def update_cbf_handover_signal(self, cbf_overflow):
        """No-op: marker handover is an ArUco dual-slot (big/small marker) concept
        that has no counterpart in the single cross-marker pipeline (see class
        docstring). Stubbed so apps/landing_test.py's per-step call doesn't
        AttributeError under MARKER_TYPE=cross."""
        pass

    def _stitch_recording(self):
        if self._rec_dir is None or self._rec_n == 0:
            return
        import glob
        frames = sorted(glob.glob(f'{self._rec_dir}/f*.png'))
        if not frames:
            return
        first = cv2.imread(frames[0])
        h, w = first.shape[0], first.shape[1]
        out_path = ('/home/shubham/Soft-Precise-Landing/PX4_Gazebo/'
                    f'test_data/Test_Videos/{self._rec_ts}.mp4')
        writer = cv2.VideoWriter(out_path, cv2.VideoWriter_fourcc(*'mp4v'),
                                  self._rec_fps, (w, h), isColor=(first.ndim == 3))
        for fp in frames:
            writer.write(cv2.imread(fp))
        writer.release()
        print(f"[CrossMarkerNode] IMG_RECORD: stitched {len(frames)} frames -> {out_path}")

    def _print_diag_summary(self):
        """One-off teardown summary distinguishing a genuinely-slow process_frame() CALL
        rate (thread starvation / camera delivery) from a fast call rate that mostly FAILS
        detection (s(t) then holds last-good, which looks identical to a slow rate from the
        controller's side -- see the 2026-08-03 perception-loop-rate investigation)."""
        log = self._perception.get_diag_log()
        if len(log) < 2:
            return
        ts = np.array([e[0] for e in log], dtype=float)
        oks = np.array([e[1] for e in log], dtype=bool)
        dt = np.diff(ts)
        dt = dt[dt > 0]
        call_hz = 1.0 / np.mean(dt) if len(dt) else float('nan')
        from collections import Counter
        fails = Counter(e[2] for e in log if not e[1])
        print(f"[CrossMarkerNode] diag: {len(log)} process_frame() calls over "
              f"{ts[-1]-ts[0]:.2f}s (mean call rate {call_hz:.1f} Hz), "
              f"detect ok {oks.sum()}/{len(oks)} ({100*oks.mean():.0f}%)"
              + (f", fail reasons: {dict(fails)}" if fails else ""))

        # 2026-08-04: hough_lt2_lines root-cause breakdown -- see cross_marker_detector's
        # HOUGH_DIAG_LOG (populated on every hough_lt2_lines occurrence, not just a sample).
        hlog = [e for e in cmd.HOUGH_DIAG_LOG if e.get('bbox') is not None]
        n_empty = sum(1 for e in cmd.HOUGH_DIAG_LOG if e.get('bbox') is None)
        if hlog:
            mask_px = np.array([e['mask_px'] for e in hlog])
            bboxes = np.array([e['bbox'] for e in hlog])  # (N,4): x,y,w,h
            n_edge = np.array([e['n_edge_px'] for e in hlog])
            stage_px = np.array([e['stage_px'] for e in hlog])  # (N,5): raw,close,blobby,roi,shape
            bbox_w, bbox_h = bboxes[:, 2], bboxes[:, 3]
            bbox_extent = np.maximum(bbox_w, bbox_h)
            print(f"[CrossMarkerNode] HOUGH_DIAG ({len(hlog)} hough_lt2_lines events"
                  + (f", {n_empty} color_gate_empty" if n_empty else "") + "): "
                  f"mask_px min/med/max={mask_px.min()}/{int(np.median(mask_px))}/{mask_px.max()}, "
                  f"bbox_extent min/med/max={bbox_extent.min()}/{int(np.median(bbox_extent))}/{bbox_extent.max()}")
            # Breakdown by extent bucket, WITH per-stage pixel survival, to see which
            # filter stage (raw color-gate / morph-close / blobby-reject / roi-crop /
            # shape-isolate) is killing the mask in each extent regime.
            for lo, hi in [(0, 50), (50, 100), (100, 200), (200, 300), (300, 500)]:
                bmask = (bbox_extent >= lo) & (bbox_extent < hi)
                if np.any(bmask):
                    sp = stage_px[bmask]
                    print(f"    extent {lo:3d}-{hi:3d}px: {np.sum(bmask):4d} events -- "
                          f"stage_px med: raw={int(np.median(sp[:,0]))} close={int(np.median(sp[:,1]))} "
                          f"blobby={int(np.median(sp[:,2]))} roi={int(np.median(sp[:,3]))} "
                          f"shape={int(np.median(sp[:,4]))}")

            # 2026-08-04: DISTINCT-pattern dedup -- a repeated identical (bbox, mask_px)
            # combo strongly suggests one frozen frame (e.g. during the post-TARGET_LOST
            # open-loop tail, where the same unchanging view gets detect()'d hundreds of
            # times) counted many times, not genuine per-frame variety. Report the true
            # distinct-pattern count so the extent-bucket breakdown above isn't misread.
            from collections import Counter
            patterns = Counter((tuple(bboxes[i]), int(mask_px[i])) for i in range(len(hlog)))
            print(f"    {len(patterns)} DISTINCT (bbox, mask_px) patterns out of {len(hlog)} events "
                  f"(top 5 by count): {patterns.most_common(5)}")

            # Cross-reference against the per-frame _diag_log's own (t, ok, fail_reason,
            # bbox_area) tuples -- both lists are appended in the same call order (once
            # per process_frame -> detect() call), so the Nth hough_lt2_lines entry in
            # _diag_log aligns positionally with the Nth cmd.HOUGH_DIAG_LOG entry. Lets
            # us see WHEN these fire without touching detect()'s signature.
            full_log = self._perception.get_diag_log()
            hough_ts = [e[0] for e in full_log if e[2] == 'hough_lt2_lines']
            if len(hough_ts) == len(cmd.HOUGH_DIAG_LOG):
                hough_ts = np.array(hough_ts)
                t0, t1 = hough_ts.min(), hough_ts.max()
                print(f"    time range of ALL hough_lt2_lines events: {t0:.2f}s-{t1:.2f}s "
                      f"(over the {full_log[-1][0]-full_log[0][0]:.1f}s run)")
                # Same distinct-pattern dedup, but only within the FIRST half of the run
                # (active tracking) vs SECOND half (likely post-TARGET_LOST open-loop tail)
                mid = (full_log[0][0] + full_log[-1][0]) / 2.0
                early = hough_ts < mid
                print(f"    {np.sum(early)} events before t={mid:.1f}s (active-tracking half), "
                      f"{np.sum(~early)} after (likely open-loop tail)")

                # GHOST-FILTERED active-half breakdown (2026-08-04): the (206,513,68,14)
                # bbox is a fixed-pixel-location rendering artifact (the documented
                # top/bottom-margin body-ghost, see _restrict_to_center_roi's docstring)
                # that dominates the raw counts above but isn't the marker at all --
                # filter it out and re-show what's LEFT in the active-tracking half, to
                # see the genuine (non-ghost) failure pattern during real tracking.
                GHOST_BBOX = (206, 513, 68, 14)
                is_ghost = np.array([tuple(bboxes[i]) == GHOST_BBOX for i in range(len(hlog))])
                active_nonghost = early & ~is_ghost
                print(f"    active-half, GHOST-FILTERED: {np.sum(active_nonghost)} genuine "
                      f"events (vs {np.sum(early)} raw, {np.sum(early & is_ghost)} were ghost)")
                if np.any(active_nonghost):
                    ext_ag = bbox_extent[active_nonghost]
                    mp_ag = mask_px[active_nonghost]
                    print(f"      extent min/med/max={ext_ag.min()}/{int(np.median(ext_ag))}/{ext_ag.max()}, "
                          f"mask_px min/med/max={mp_ag.min()}/{int(np.median(mp_ag))}/{mp_ag.max()}")
                    ag_patterns = Counter((tuple(bboxes[i]), int(mask_px[i]))
                                          for i in np.where(active_nonghost)[0])
                    print(f"      top 5 active-half non-ghost patterns: {ag_patterns.most_common(5)}")
            else:
                print(f"    WARNING: hough_ts count ({len(hough_ts)}) != HOUGH_DIAG_LOG count "
                      f"({len(cmd.HOUGH_DIAG_LOG)}) -- positional alignment assumption broken")

    def close(self):
        self._running = False
        try:
            self._print_diag_summary()
        except Exception as e:
            print(f"[CrossMarkerNode] diag summary failed: {e}")
        if self.RECORD and self._rec_writer_thread is not None:
            # Flush the writer queue (all frames enqueued during the flight, disk I/O
            # deferred off the hot path -- see _rec_queue's __init__ comment) BEFORE
            # stitching, so _stitch_recording() doesn't glob a partially-written directory.
            try:
                self._rec_queue.put(None)   # sentinel
                self._rec_writer_thread.join(timeout=30.0)
            except Exception as e:
                print(f"[CrossMarkerNode] recording writer flush failed: {e}")
        try:
            self._stitch_recording()
        except Exception as e:
            print(f"[CrossMarkerNode] video stitch failed: {e}")
        try:
            self._image_sub.close()
        except Exception:
            pass

    def __del__(self):
        self._running = False
