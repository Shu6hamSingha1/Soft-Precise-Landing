"""Online, ID/size/depth-free 2D feature map (PlanarFeatureMap), built incrementally via
KLT tracking + per-frame HOMOGRAPHY-fit to a growing reference frame (upgraded from a
rigid similarity fit 2026-07-14: a 4-DOF similarity can't represent the perspective
distortion from real camera tilt between frames, which showed up as large held-out
marker-position errors (p90=36px, max=111px) in initial validation; this project's own
existing _board_feature/self-cal already uses a full homography for exactly this reason).

Motivation (2026-07-14, see feedback_klt_no_drift_detection): KLT tracking the ArUco
marker's own 4 corners has no way to detect its own drift -- LK reports high confidence
("accepted") even while silently converging onto the wrong (but locally similar) feature,
which corrupted a live landing's loom signal and caused a false touchdown-detect disarm.
Root idea (user directive): instead of tracking ONLY the marker's 4 corners, track ALL
visible scene features and build a shared 2D map online (SLAM-like, but purely in the
image/projective plane -- no depth or metric scale anywhere, staying within the project's
hard scale-free/depth-free constraint). The marker's position becomes one entry in this
map, inferable even when the marker itself is briefly untrackable, as long as enough other
mapped features remain visible to anchor the current frame. ArUco decode becomes a sparse,
adaptive LOOP-CLOSURE correction (like SLAM loop closure) rather than the primary tracker,
which also directly serves the compute-budget goal (max frame rate for KLT/tracking, decode
run only occasionally) needed for real-hardware (Raspberry Pi) deployment.

MULTI-SLOT marker tracking (2026-07-15, user directive): the project's landing pad carries
TWO nested ArUco markers (small + big, concentric). Near their handover boundary (the
altitude range where both are marginally decodable), the ArUco decoder's reported ID
flickers frame-to-frame between them -- a genuine, expected upstream signal instability
(this is exactly why PLASMC_SINGLE_MARKER locking exists at the img_data.py layer). Rather
than treating each decode as updating "the" marker and resetting on an identity change
(the earlier single-slot design), this module now maintains up to two persistent SLOTS
(one per physical marker) simultaneously, and routes every fresh decode into whichever
slot it GEOMETRICALLY belongs to -- by reprojection match against each slot's current
KLT-tracked prediction first (definitive when available), falling back to relative
quad-size classification (the two markers differ enough in printed size that this is
unambiguous) when no slot has a usable prediction yet. The decoded ArUco ID itself is
NEVER used to decide slot routing -- only pixel geometry -- so a misdecoded ID (right
corners, wrong label) routes correctly, and the small<->big flicker no longer disturbs
either slot's map entries at all: each flickering decode just updates its own slot in
place, while the OTHER slot's corners keep being passively KLT-tracked as ordinary map
features regardless of whether they're being decoded this frame.

This module is standalone and side-effect-free (no img_data.py/controller.py dependency)
so it can be validated offline against recorded frames before being wired into the live
capture loop -- see tools/validate_planar_map.py.
"""
import os
import numpy as np
import cv2


def _line_intersect(a1, a2, b1, b2):
    """Intersection of line (a1,a2) with line (b1,b2), 2D. None if near-parallel.
    Used for the projective-invariant marker center (diagonal intersection)."""
    a1 = np.asarray(a1, float); a2 = np.asarray(a2, float)
    b1 = np.asarray(b1, float); b2 = np.asarray(b2, float)
    da = a2 - a1; db = b2 - b1
    den = da[0] * db[1] - da[1] * db[0]
    if abs(den) < 1e-9:
        return None
    t = ((b1[0] - a1[0]) * db[1] - (b1[1] - a1[1]) * db[0]) / den
    return a1 + t * da

# Canonical TL/TR/BR/BL corner weights (2026-07-15). Ported from img_data.py's
# _marker_principal_angle: a perfectly square marker has mu11==0 for ANY yaw under
# uniform weighting (the 2nd-moment orientation axis is undefined), so an asymmetric
# per-corner weight is synthesized to make yaw observable at all. get_marker_orientation's
# caller (img_data.py) reuses this SAME array (imported, not re-typed) so the two modules
# can never silently drift apart on the convention.
CORNER_WEIGHTS = np.array([4.0, 3.0, 2.0, 1.0])


class PlanarFeatureMap:
    """Incremental 2D feature map. All positions (map and tracked) are in PIXEL units of
    an arbitrary, self-consistent gauge (the map's own coordinate frame, established at
    bootstrap) -- no physical size or depth ever enters. Marker corners are folded in as
    just another set of mapped features, tagged (per physical-marker SLOT) so their map
    position can be queried separately (`get_marker_frame_pts`, `get_marker_center`).

    DENSE POINTS (2026-07-15): each slot also seeds a deterministic (texture-independent)
    set of interior points on nested nested-scale quads at slot creation/re-decode, each
    carrying a WEIGHT linearly interpolated between the 4 corners' CORNER_WEIGHTS along its
    originating edge -- see _seed_dense_points docstring for why this asymmetry must be
    preserved (a naive uniformly-weighted dense cloud on a near-square marker would make
    orientation estimation MORE degenerate, not less). `get_marker_points` exposes the full
    corner+dense set (position, weight) for a moment-based ORIENTATION estimate with lower
    variance than the 4 corners alone; `get_marker_center` is a separate, deliberately
    UNWEIGHTED single-point homography projection of the corners' fixed map-gauge mean, for
    POSITION -- averaging happens ONCE in the stable map gauge rather than per-frame in
    noisy, independently-tracked image pixels (a homography does not commute with
    averaging: mean(H(p_i)) != H(mean(p_i)) in general, so this is a genuinely different,
    smoother estimate, not just a repackaging of get_marker_frame_pts()'s corner mean).
    """

    def __init__(self, max_features=300, min_tracked=8, refill_below=60,
                 lk_win=(21, 21), lk_max_level=3, gft_quality=0.01, gft_min_dist=7,
                 ransac_thresh_px=3.0, conf_track_floor=15, conf_resid_ceiling=4.0,
                 marker_rigid_thresh=0.35, max_marker_slots=2, slot_size_margin=0.4,
                 center=None, focal=None):
        self.max_features = max_features
        self.min_tracked = min_tracked
        self.refill_below = refill_below
        self.lk_params = dict(winSize=lk_win, maxLevel=lk_max_level,
                               criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
        self.gft_quality = gft_quality
        self.gft_min_dist = gft_min_dist
        self.ransac_thresh_px = ransac_thresh_px
        self.conf_track_floor = conf_track_floor
        self.conf_resid_ceiling = conf_resid_ceiling
        self.marker_rigid_thresh = marker_rigid_thresh
        self.slot_inlier_fail_streak_max = int(os.environ.get("PLANAR_MAP_SLOT_INLIER_STREAK_MAX", "3"))  # see update()'s inlier-survival tracking
        self.max_marker_slots = max_marker_slots   # exactly 2 physical markers (small+big) by design
        # GYRO-SEEDED KLT (2026-07-17, user design): calcOpticalFlowPyrLK has no motion
        # model -- it seeds its search from the PREVIOUS frame's pixel position (implicit
        # zero-motion prior), which fails under large real inter-frame rotation (confirmed
        # live: IC5's terminal divergence coincided with real attitude tilt climbing
        # 4deg->24deg in ~1.5s, producing a held-out reprojection error swinging 0.3-199px
        # -- KLT locking onto a locally-similar-but-wrong patch, not smooth drift). The
        # quaternion is available from the FC every frame REGARDLESS of whether ArUco
        # decodes anything (user correction, 2026-07-17) -- so this can run through
        # marker-loss stretches too, where it matters most (a better seed here reduces how
        # often the pipeline falls into the coast/rescue regime at all). center/focal are
        # optional -- pass None to disable (keeps the module's "pixel-geometry only" default
        # when the caller doesn't want the camera-intrinsics coupling this requires).
        self.center = np.asarray(center, dtype=float) if center is not None else None
        self.focal = np.asarray(focal, dtype=float) if focal is not None else None
        self._prev_quat_R = None   # previous frame's body->NED DCM (3x3), for the rotation delta
        self.slot_size_margin = slot_size_margin   # relative quad-size tolerance for same-slot classification

        self.map_pts = {}              # feature_id -> (x, y) in MAP gauge
        self.next_id = 0
        self.tracked_px = {}       # feature_id -> current-frame pixel position
        self.prev_gray = None
        self.frame_to_map = None   # 3x3 homography: map_pt(homog) ~ frame_to_map @ [px;1]
        self.confidence = 0.0
        self.map_confidence = 0.0   # marker-independent (track_conf*resid_conf only) -- see update()
        self.n_tracked = 0
        self.resid_px = np.inf
        # (b) HOMOGRAPHY ILL-CONDITIONING GATE (2026-07-19, user). If the surviving tracked points
        # collapse to a near-LINE (marker foreshortened to a sliver at a grazing view / overflow),
        # the 8-DOF frame->map homography is UNDER-DETERMINED and reprojections extrapolate
        # arbitrarily off-screen. When the 2nd/1st singular-value spread of the centred src points
        # falls below this, reject the fit -> hold last-good frame_to_map + resid_px=inf so
        # map_confidence DROPS (the map SELF-DIAGNOSES the degenerate view instead of reporting a
        # confident-wrong pose -- the fix for the 0.575-confidence-at-overflow blind spot).
        # Default 0 = OFF; MAP_HOMOG_MIN_SPREAD~0.05 enables.
        self._homog_min_spread = float(os.environ.get("MAP_HOMOG_MIN_SPREAD", "0"))
        self.initialized = False
        self.frames_since_decode = 0

        # --- Marker SLOTS (see module docstring) ---------------------------------------
        # slot_name -> {'corner_ids': [fid,...] ORDERED (TL/TR/BR/BL, by construction --
        #               never recovered by sorting, same reasoning as the old single-slot
        #               design), 'size': last quad size (mean of the 2 diagonals, PIXELS,
        #               EMA-smoothed), 'shape_sig': last _marker_shape_signature,
        #               'rigid_ok': bool, 'shape_change': float}
        self.marker_slots = {}
        self._next_slot_id = 0
        # Backward-compatible scalar view of the PRIMARY slot (largest current size --
        # matches the project's existing "bigger marker priority" convention). Updated
        # every update()/loop_closure_correct() call. Callers that only care about "the"
        # marker (img_data.py shadow wiring, tools/validate_planar_map.py) can keep using
        # these exactly as before; multi-slot-aware callers can index self.marker_slots
        # directly.
        self.marker_rigid_ok = True
        self.marker_shape_change = 0.0
        self._rigid_fail_streak = 0    # consecutive frames marker_rigid_ok has been False (see update())
        self.rigid_fail_streak_max = int(os.environ.get("PLANAR_MAP_RIGID_FAIL_STREAK_MAX", "3"))

    def _new_ids(self, n):
        ids = list(range(self.next_id, self.next_id + n))
        self.next_id += n
        return ids

    @staticmethod
    def _quad_size(corners):
        """Scale metric for a 4-point marker quad: mean of the two diagonals (TL-BR,
        TR-BL). Robust to rotation/aspect within normal camera-tilt ranges, and -- unlike
        area -- degrades gracefully (doesn't collapse to ~0) if the quad is mildly
        skewed. Used ONLY for slot classification when no geometric (reprojection)
        match is available; never compared across markers of genuinely different scale
        classes with a fixed absolute threshold -- see _classify_slot."""
        c = np.asarray(corners, dtype=np.float64).reshape(-1, 2)
        if len(c) != 4:
            return None
        return float((np.linalg.norm(c[0] - c[2]) + np.linalg.norm(c[1] - c[3])) / 2.0)

    def bootstrap(self, gray, marker_px_corners=None, marker_id=None, quat_R=None):
        """First frame: seed the map directly from this frame's own pixel coords (map
        gauge = frame-0 pixel gauge, arbitrary but self-consistent). marker_px_corners,
        if given (Nx2, from a fresh ArUco decode), seed the FIRST marker slot. marker_id
        is accepted for caller-side logging only -- never used to decide slot identity
        (see module docstring). quat_R (3,3) is this frame's body->NED DCM, seeding
        _prev_quat_R so the FIRST update() call already has a rotation delta to use --
        see update()'s docstring."""
        self._prev_quat_R = quat_R
        feats = cv2.goodFeaturesToTrack(gray, maxCorners=self.max_features,
                                         qualityLevel=self.gft_quality, minDistance=self.gft_min_dist)
        pts = feats.reshape(-1, 2) if feats is not None else np.zeros((0, 2))
        ids = self._new_ids(len(pts))
        for fid, p in zip(ids, pts):
            self.map_pts[fid] = tuple(p)
            self.tracked_px[fid] = tuple(p)
        self.prev_gray = gray.copy()
        self.frame_to_map = np.eye(3)   # identity homography at bootstrap
        self.initialized = True
        self.confidence = 1.0
        self.map_confidence = 1.0
        self.n_tracked = len(self.tracked_px)
        self.resid_px = 0.0
        self._rigid_fail_streak = 0
        if marker_px_corners is not None:
            self._seed_new_slot(marker_px_corners)
            self._update_primary_scalars()

    @staticmethod
    def _apply_h(H, pts):
        """Apply a 3x3 homography to Nx2 points with the perspective divide."""
        pts = np.asarray(pts, dtype=np.float64).reshape(-1, 2)
        homog = np.column_stack([pts, np.ones(len(pts))])
        out = homog @ H.T
        w = out[:, 2:3]
        w = np.where(np.abs(w) < 1e-12, 1e-12, w)
        return out[:, :2] / w

    def _fit_homography(self, src, dst):
        """RANSAC homography (full 8-DOF projective) src->dst. Returns (H 3x3, inlier_mask)
        or (None, None) if too few points / degenerate. Upgraded from a rigid similarity fit
        (see module docstring) so real camera-tilt-induced perspective distortion between
        frames is represented, not just scale+rotation+translation."""
        if len(src) < 4:
            return None, None
        H, inliers = cv2.findHomography(
            src.astype(np.float32), dst.astype(np.float32),
            method=cv2.RANSAC, ransacReprojThreshold=self.ransac_thresh_px)
        if H is None:
            return None, None
        return H, inliers.flatten().astype(bool) if inliers is not None else np.ones(len(src), bool)

    @staticmethod
    def _marker_shape_signature(corners):
        """Scale-invariant shape descriptor for a 4-point marker quad: the 6 pairwise
        corner distances, normalized by their own mean. Invariant to uniform scale
        (real, legitimate altitude change), sensitive to any RELATIVE distortion of the
        4 points (drift, mistracking, one corner sliding independently of the others).
        Not invariant to genuine perspective change (a real tilted view DOES change these
        ratios) -- that's intentional: this is a per-frame SMOOTHNESS check (has the shape
        changed abruptly since last frame), not an absolute rigidity constraint, so it
        tolerates gradual perspective change from real camera motion while catching a
        sudden, non-physical distortion (e.g. two corners collapsing together)."""
        c = np.asarray(corners, dtype=np.float64).reshape(-1, 2)
        if len(c) != 4:
            return None
        idx = [(0, 1), (0, 2), (0, 3), (1, 2), (1, 3), (2, 3)]
        d = np.array([np.linalg.norm(c[i] - c[j]) for i, j in idx])
        m = d.mean()
        if m < 1e-6:
            return None
        return d / m

    def _check_slot_rigidity(self, slot, corners, max_relative_change=None):
        """Per-slot analogue of the old check_marker_rigidity -- see
        _marker_shape_signature docstring. Updates slot['shape_sig']/['rigid_ok']/
        ['shape_change'] in place."""
        if max_relative_change is None:
            max_relative_change = self.marker_rigid_thresh
        sig = self._marker_shape_signature(corners)
        if sig is None:
            slot['rigid_ok'] = False
            slot['shape_change'] = np.inf
            return
        if slot['shape_sig'] is None:
            slot['shape_sig'] = sig
            slot['rigid_ok'] = True
            slot['shape_change'] = 0.0
            return
        change = float(np.max(np.abs(sig - slot['shape_sig'])))
        slot['shape_sig'] = sig
        slot['shape_change'] = change
        slot['rigid_ok'] = change <= max_relative_change

    def _primary_slot_name(self):
        """The slot the project's existing 'bigger marker priority' convention would pick:
        largest current size among slots that currently have a valid frame prediction.
        Falls back to largest size among ALL slots if none currently predict (e.g. right
        after bootstrap, before update() has run once)."""
        if not self.marker_slots:
            return None
        predicting = [name for name in self.marker_slots if self._slot_frame_pts(name) is not None]
        pool = predicting if predicting else list(self.marker_slots.keys())
        return max(pool, key=lambda n: (self.marker_slots[n]['size'] or 0.0))

    def secondary_slot_name(self):
        """The non-PRIMARY marker slot -- by construction (max_marker_slots=2,
        _primary_slot_name always picks the LARGEST), this is the smaller physical
        marker, if it has ever been seen/seeded at all. None before the small marker's
        first decode (no slot exists yet), or if only one physical marker is tracked.
        2026-07-17 (user): the CBF needs to read the SMALL marker specifically once it's
        confidently mapped (more FoV headroom -- it doesn't overflow near touchdown the
        way the big marker does), independently of which marker is "primary" for the
        flow/s pipeline (which stays big-priority for h_x/h_y observability -- see
        IMG_MARKER_PRIORITY doc in img_data.py). This is the slot-lookup half of that."""
        primary = self._primary_slot_name()
        others = [n for n in self.marker_slots if n != primary]
        return others[0] if others else None

    def get_slot_confidence(self, name):
        """Confidence for a SPECIFIC slot, independent of whether it's currently primary
        (self.confidence/self.map_confidence only ever reflect the primary/largest slot
        -- see _update_primary_scalars). Same formula as self.confidence (track_conf *
        resid_conf * this-slot's-own rigidity * this-slot's-own inlier-survival), so a
        caller can ask 'is the SMALL slot trustworthy right now' even while the big slot
        is primary. Includes the SAME slot_inlier_fail_streak persistence decay as
        map_confidence (2026-07-17) -- a slot whose corners are tracked-but-RANSAC-
        rejected against the global fit is exactly as untrustworthy for THIS slot as it
        is for the primary. NOTE: does not (yet) apply the per-slot analogue of
        _rigid_fail_streak's sustained-failure decay for marker_conf specifically --
        that's tracked only for the primary slot currently; a slot-specific rigidity
        streak is a known follow-up if this matters in practice."""
        slot = self.marker_slots.get(name)
        if slot is None:
            return 0.0
        track_conf = min(1.0, self.n_tracked / max(self.conf_track_floor, 1))
        resid_conf = max(0.0, 1.0 - (self.resid_px / self.conf_resid_ceiling)) if np.isfinite(self.resid_px) else 0.0
        shape_change = slot.get('shape_change', np.inf)
        marker_conf = (max(0.0, 1.0 - shape_change / max(self.marker_rigid_thresh, 1e-6))
                       if np.isfinite(shape_change) else 0.0)
        _streak = slot.get('inlier_fail_streak', 0)
        slot_survival_conf = max(0.0, 1.0 - _streak / max(self.slot_inlier_fail_streak_max, 1))
        return float(track_conf * resid_conf * marker_conf * slot_survival_conf)

    def _update_primary_scalars(self):
        """Refresh the backward-compatible marker_rigid_ok/marker_shape_change scalars
        from the current primary slot (see class docstring)."""
        name = self._primary_slot_name()
        if name is None:
            self.marker_rigid_ok = False
            self.marker_shape_change = np.inf
            return
        slot = self.marker_slots[name]
        self.marker_rigid_ok = slot['rigid_ok']
        self.marker_shape_change = slot['shape_change']
        # SUSTAINED-failure streak (2026-07-17, found via IC2 SITL trace): the 2026-07-16 fix
        # correctly HOLDS shape_change on a partial/momentary corner drop (so a single benign
        # blip doesn't zero confidence) -- but that hold means marker_conf, which is derived
        # from shape_change, never reflects rigid_ok going False for MANY consecutive frames
        # either (shape_change just sits at its last healthy value forever). Confirmed live:
        # IC2's terminal corner collapse had marker_rigid_ok=False for its entire tail while
        # confidence stayed 0.75-0.94. Track how long rigid_ok has been persistently False and
        # let the confidence computation below decay marker_conf on SUSTAINED failure while
        # still ignoring a single-frame blip (mirrors STALE_THRESH's consecutive-miss pattern).
        self._rigid_fail_streak = 0 if self.marker_rigid_ok else self._rigid_fail_streak + 1

    def update(self, gray, quat_R=None):
        """Track existing features frame-to-frame via KLT, refill if running low, refit
        the frame->map homography, update confidence. Call once per frame after bootstrap.

        quat_R : (3,3) ndarray or None -- the CURRENT frame's body->NED DCM (e.g.
        Quaternion([w,x,y,z]).to_DCM(), the same call img_data.py's _getVirtualPts already
        uses -- pass that exact result, don't recompute the quaternion->matrix conversion
        here, so the sign convention stays in ONE validated place). When provided (and a
        previous frame's quat_R + self.center/self.focal are available), seeds KLT's
        search with a gyro-rotation-compensated prediction instead of the implicit
        zero-motion prior -- see __init__ comment for why this matters under large real
        inter-frame rotation. None (default) preserves the original zero-motion-prior
        behavior exactly."""
        if not self.initialized:
            raise RuntimeError("PlanarFeatureMap.update() called before bootstrap()")

        ids = list(self.tracked_px.keys())
        prev_pts = np.array([self.tracked_px[i] for i in ids], dtype=np.float32).reshape(-1, 1, 2)

        next_pts_seed = None
        lk_kwargs = dict(self.lk_params)
        if (quat_R is not None and self._prev_quat_R is not None
                and self.center is not None and self.focal is not None):
            try:
                # R_delta rotates a CAMERA-FRAME ray from the PREVIOUS frame's orientation
                # to its equivalent ray under the CURRENT frame's orientation, for a
                # world-fixed point: d_curr = R_curr.T @ d_world = R_curr.T @ R_prev @ d_prev
                # (camera=body-aligned, same convention as _getVirtualPts). This predicts
                # where a static scene point SHOULD appear to have moved due to the drone's
                # own rotation alone -- KLT then only has to resolve the RESIDUAL motion
                # (translation + any real target motion), not the full displacement.
                R_delta = quat_R.T @ self._prev_quat_R
                cx, cy = self.center
                fx, fy = self.focal
                p = prev_pts.reshape(-1, 2)
                rays = np.column_stack([(p[:, 0] - cx) / fx, (p[:, 1] - cy) / fy, np.ones(len(p))])
                pred_rays = rays @ R_delta.T
                z = pred_rays[:, 2]
                safe = np.abs(z) > 1e-6   # degenerate rays (near-90deg rotation) skip compensation
                pred_px = p.copy()
                pred_px[safe, 0] = fx * pred_rays[safe, 0] / z[safe] + cx
                pred_px[safe, 1] = fy * pred_rays[safe, 1] / z[safe] + cy
                next_pts_seed = pred_px.reshape(-1, 1, 2).astype(np.float32)
                lk_kwargs['flags'] = cv2.OPTFLOW_USE_INITIAL_FLOW
            except (ValueError, TypeError):
                next_pts_seed = None
        self._prev_quat_R = quat_R if quat_R is not None else self._prev_quat_R

        new_pts, status, _ = cv2.calcOpticalFlowPyrLK(self.prev_gray, gray, prev_pts, next_pts_seed, **lk_kwargs)
        status = status.flatten().astype(bool) if status is not None else np.zeros(len(ids), bool)

        survivors = {}
        for i, ok, p in zip(ids, status, new_pts.reshape(-1, 2)):
            if ok:
                survivors[i] = tuple(p)

        # Fit frame(this)->map homography using survivors with a KNOWN map position
        common_ids = [i for i in survivors if i in self.map_pts]
        if len(common_ids) >= 4:
            src = np.array([survivors[i] for i in common_ids])
            dst = np.array([self.map_pts[i] for i in common_ids])
            H, inliers = self._fit_homography(src, dst)
            # (b) reject an under-determined fit from near-collinear (sliver) tracked points:
            # spread = 2nd/1st singular value of the centred src (0=line, ~1=isotropic).
            if H is not None and self._homog_min_spread > 0.0:
                _c = src - src.mean(axis=0)
                _sv = np.linalg.svd(_c, compute_uv=False)
                _spread = float(_sv[1] / _sv[0]) if (len(_sv) >= 2 and _sv[0] > 1e-9) else 0.0
                if _spread < self._homog_min_spread:
                    H = None   # -> else-branch: hold frame_to_map, resid_px=inf, confidence drops
            if H is not None:
                self.frame_to_map = H
                pred = self._apply_h(H, src)
                self.resid_px = float(np.sqrt(np.mean(np.sum((pred[inliers] - dst[inliers]) ** 2, axis=1)))) \
                    if inliers.sum() > 0 else np.inf
                # PER-SLOT INLIER-SURVIVAL (2026-07-17, found via IC5 SITL trace): map_confidence
                # = track_conf * resid_conf, BOTH computed only over RANSAC INLIERS (resid_px sums
                # inliers only; the loop below drops outlier ids from survivors entirely) -- so a
                # marker whose OWN corners are actively drifting/chaotic (repeatedly getting
                # RANSAC-rejected) is structurally invisible to map_confidence as long as enough
                # OTHER tracked scene features stay well-behaved. Confirmed live: IC5's marker had
                # a held-out reprojection error swinging 0.3-199px (genuinely chaotic, not smooth)
                # while map_confidence stayed 0.72-0.99 throughout -- the rescue gate would have
                # happily fired on that chaos. Track, per slot, how many CONSECUTIVE frames its own
                # corner_ids were tracked (in common_ids) yet MAJORITY rejected by RANSAC -- this
                # is a DIFFERENT signal from marker_rigid_ok (internal shape consistency): this one
                # measures agreement with the GLOBAL map fit specifically, which rigidity can't see
                # (a rigid-but-globally-wrong quad is exactly RANSAC's job to catch). Only degrades
                # when the slot's corners ARE tracked but rejected -- a slot with ZERO of its
                # corner_ids in common_ids this frame (genuine occlusion, zero information) is left
                # untouched, so map_confidence still survives occlusion exactly as designed
                # (see feedback_planar_map_plausibility_gate's 2026-07-16 rescue-vs-override split).
                _idx_of = {cid: k for k, cid in enumerate(common_ids)}
                for _slot in self.marker_slots.values():
                    _cids = [c for c in _slot['corner_ids'] if c in _idx_of]
                    if not _cids:
                        continue   # not tracked at all this frame -- hold last streak (occlusion-safe)
                    _inlier_frac = float(np.mean([inliers[_idx_of[c]] for c in _cids]))
                    if _inlier_frac < 0.5:
                        _slot['inlier_fail_streak'] = _slot.get('inlier_fail_streak', 0) + 1
                    else:
                        _slot['inlier_fail_streak'] = 0
                # drop outlier tracks (bad KLT converges that RANSAC rejected) -- treat as lost
                for i, keep in zip(common_ids, inliers):
                    if not keep:
                        survivors.pop(i, None)
            else:
                self.resid_px = np.inf
        else:
            self.resid_px = np.inf

        self.tracked_px = survivors
        self.n_tracked = len(survivors)

        # Per-slot marker-local rigidity check (independent of the map/homography fit --
        # see _check_slot_rigidity docstring). Only meaningful if all 4 of a slot's
        # corners survived KLT this frame.
        #
        # BUGFIX (2026-07-16, found via IC5 SITL trace): a PARTIAL corner drop (1-3 of 4
        # missing this frame -- routine KLT jitter or a corner nearing the frame edge as
        # the marker grows large near touchdown, NOT necessarily genuine shape distortion)
        # used to FORCE shape_change=inf / rigid_ok=False. Since self.confidence is a
        # PRODUCT of three terms (track_conf * resid_conf * marker_conf) and marker_conf
        # derives from shape_change, that forced inf collapsed the WHOLE map's confidence
        # to exactly 0.0 -- regardless of how many hundreds of OTHER scene features were
        # tracked perfectly well, or how good the overall homography residual was. This
        # masked exactly the frames a map-based rescue (img_data.py's PLANARFEATUREMAP
        # RESCUE, get_marker_center/get_marker_frame_pts -- both already tolerant of a
        # partial corner set) needs most: confirmed live, IC5's map confidence was 0.000
        # for the entire half-second BEFORE full corner loss even happened, with the
        # held-out prediction error only gradually growing (3.8->32.5px), not the sudden
        # jump a genuine shape failure would produce. Fix: on a partial drop, HOLD the
        # slot's last known rigid_ok/shape_change (this codebase's standing "hold last
        # good, never fabricate/never punish-to-worst-case" principle, used pervasively
        # elsewhere -- e.g. ds/dh outlier-hold, s-extrapolation) rather than forcing
        # invalidation -- a missing corner this frame doesn't retroactively make LAST
        # frame's confirmed-rigid shape wrong. Only a FULL drop (0 of 4, genuinely no
        # information at all) forces rigid_ok=False; even then shape_change is held (not
        # inf) so a single all-corners-lost frame doesn't permanently poison the value if
        # corners return next frame with a benign gap.
        for slot in self.marker_slots.values():
            pts = [survivors[i] for i in slot['corner_ids'] if i in survivors]
            if len(pts) == 4:
                self._check_slot_rigidity(slot, np.array(pts))
            elif len(pts) == 0:
                slot['rigid_ok'] = False   # genuinely zero information this frame
                # shape_change deliberately NOT touched -- held at its last value
            # else (1-3 survived): hold rigid_ok/shape_change entirely, no update at all
        self._update_primary_scalars()

        # Refill: add fresh features (map them via current frame_to_map) if running low
        if self.n_tracked < self.refill_below and self.frame_to_map is not None:
            mask = np.full(gray.shape[:2], 255, dtype=np.uint8)
            for p in survivors.values():
                cv2.circle(mask, (int(p[0]), int(p[1])), self.gft_min_dist, 0, -1)
            n_want = self.max_features - self.n_tracked
            feats = cv2.goodFeaturesToTrack(gray, maxCorners=max(n_want, 0),
                                             qualityLevel=self.gft_quality,
                                             minDistance=self.gft_min_dist, mask=mask)
            if feats is not None:
                new_px = feats.reshape(-1, 2)
                new_ids = self._new_ids(len(new_px))
                mapped_pts = self._apply_h(self.frame_to_map, new_px)
                for fid, p, mp in zip(new_ids, new_px, mapped_pts):
                    self.tracked_px[fid] = tuple(p)
                    self.map_pts[fid] = tuple(mp)
                self.n_tracked = len(self.tracked_px)

        self.prev_gray = gray.copy()
        self.frames_since_decode += 1

        # Confidence: 1.0 when well-tracked + low residual + primary-slot-locally-rigid,
        # decaying as any degrades. marker_conf targets the PRIMARY slot specifically (see
        # _check_slot_rigidity docstring) -- without it, a drifting marker can hide inside
        # a healthy-looking global fit (it's a small fraction of the tracked pool and
        # barely moves the aggregate residual).
        track_conf = min(1.0, self.n_tracked / max(self.conf_track_floor, 1))
        resid_conf = max(0.0, 1.0 - (self.resid_px / self.conf_resid_ceiling)) if np.isfinite(self.resid_px) else 0.0
        marker_conf = max(0.0, 1.0 - self.marker_shape_change / max(self.marker_rigid_thresh, 1e-6)) \
            if np.isfinite(self.marker_shape_change) else 0.0
        # Persistence decay (2026-07-17, see _update_primary_scalars): a held shape_change
        # can't see a SUSTAINED rigid_ok=False run on its own -- scale marker_conf down as
        # the failure streak grows, reaching 0 at rigid_fail_streak_max consecutive frames.
        # A single-frame blip (streak=1) is barely dented; only persistence collapses it.
        rigid_persist = max(0.0, 1.0 - self._rigid_fail_streak / max(self.rigid_fail_streak_max, 1))
        marker_conf *= rigid_persist
        self.confidence = float(track_conf * resid_conf * marker_conf)
        # MAP CONFIDENCE (2026-07-16, user correction): a SEPARATE, marker-INDEPENDENT
        # metric -- track_conf * resid_conf only, deliberately excluding marker_conf. This
        # is the actual, intended use case for the whole module: "provide correct visual
        # cues even when the marker is partially visible or has vanished totally" (user,
        # verbatim) -- i.e. infer the marker's position from the GLOBAL homography fit,
        # built from whatever OTHER scene features are currently tracked, entirely
        # independent of whether the marker's OWN corners currently survive at all. The
        # combined self.confidence above is the wrong gate for that: marker_conf is FORCED
        # toward 0 the instant the marker's corners are untrackable (0/4 or a shape check
        # skipped) -- exactly the condition the rescue needs to survive THROUGH, not be
        # blocked by. self.confidence (still marker_conf-weighted) stays for its own
        # purpose: detecting a marker that's drifting/mistracking WHILE STILL VISIBLE,
        # hiding inside an otherwise-healthy global fit -- a genuinely different question
        # ("is the marker's own corner data trustworthy") from "can the map still infer
        # position at all" (map_confidence). Callers that want a position/orientation
        # estimate during partial/total marker occlusion (img_data.py's PLANARFEATUREMAP
        # RESCUE) should gate on map_confidence; callers verifying a currently-decoded
        # marker's own geometry (loop-closure sanity, shadow diagnostics) keep using
        # confidence.
        #
        # SLOT INLIER-SURVIVAL term (2026-07-17, see update()'s tracking comment): unlike
        # marker_conf (rigidity, deliberately excluded above), this is NOT blind during
        # occlusion -- it only degrades when the primary slot's corners are ACTIVELY
        # tracked-but-RANSAC-rejected (chaotic drift against the global fit), which is
        # exactly the gap IC5 exposed (map_confidence 0.72-0.99 while held-out reprojection
        # error swung 0.3-199px). A slot with zero corner_ids in common_ids this frame
        # (genuine occlusion) leaves its streak untouched by update()'s tracking loop, so
        # this term stays at whatever it last was -- occlusion still doesn't punish
        # map_confidence, preserving the rescue's core design intent.
        _primary = self._primary_slot_name()
        _pslot = self.marker_slots.get(_primary) if _primary is not None else None
        _streak = _pslot.get('inlier_fail_streak', 0) if _pslot is not None else 0
        slot_survival_conf = max(0.0, 1.0 - _streak / max(self.slot_inlier_fail_streak_max, 1))
        self.map_confidence = float(track_conf * resid_conf * slot_survival_conf)

    def _slot_frame_pts(self, slot_name):
        """Current-frame pixel estimate of one slot's marker corners, via map->frame
        (inverse of frame_to_map + perspective divide). None if the slot is unknown, has
        no mapped corners left, or frame_to_map is unavailable."""
        slot = self.marker_slots.get(slot_name)
        if slot is None or self.frame_to_map is None or not slot['corner_ids']:
            return None
        try:
            Hinv = np.linalg.inv(self.frame_to_map)
        except np.linalg.LinAlgError:
            return None
        mids = [fid for fid in slot['corner_ids'] if fid in self.map_pts]
        if not mids:
            return None
        mapped = np.array([self.map_pts[fid] for fid in mids])
        return self._apply_h(Hinv, mapped)

    def get_marker_frame_pts(self, slot=None):
        """Current-frame pixel estimate of a marker's corners. slot=None (default)
        returns the PRIMARY slot (largest current size -- matches the project's existing
        'bigger marker priority' convention), preserving the old single-marker API for
        callers that only track "the" marker (img_data.py shadow wiring,
        tools/validate_planar_map.py). Pass an explicit slot name (see self.marker_slots)
        for multi-slot-aware callers. None if no matching slot / no valid prediction."""
        if slot is None:
            slot = self._primary_slot_name()
            if slot is None:
                return None
        return self._slot_frame_pts(slot)

    def get_marker_center(self, slot=None):
        """Current-frame POSITION estimate: the corners' current-frame reprojections
        (get_marker_frame_pts-equivalent), averaged AFTER projection.

        REVISED 2026-07-15 -- the original design averaged the corners' map-gauge
        positions FIRST, then applied the homography's full projective transform
        (including the perspective divide) to that single averaged point. Held-out
        validation against fresh-decode ground truth found this badly wrong under real
        perspective (large, close, tilted marker near touchdown): at one traced frame,
        individually reprojecting each corner then averaging gave 0.28px error, while
        averaging first then projecting once gave 42.84px error on the SAME frame/same
        underlying map_pts/same homography. The perspective (w-)divide is highly
        nonlinear and can differ meaningfully across 4 corners spanning a large marker
        under real camera tilt -- averaging their map-gauge coordinates BEFORE that
        divide is not a "smoother" estimate, it's simply wrong (produces a point with no
        consistent physical interpretation, since w varies per-corner). Projecting each
        corner individually (each gets its OWN correct w) and averaging the results
        afterward is the mathematically sound way to get a position "from the
        homography" -- this is exactly what it now does, via _slot_frame_pts. Kept as a
        separate accessor (not just get_marker_frame_pts(...).mean(0)) so callers doing
        pure position don't need to know it's corner-derived, and so a future genuinely
        homography-native center (a single dedicated tracked feature id at the marker's
        true geometric center) could replace the implementation without an API change --
        deferred: the current corner-average form already validates well below."""
        pred = self._slot_frame_pts(slot if slot is not None else self._primary_slot_name())
        if pred is None or len(pred) == 0:
            return None
        return pred.mean(axis=0)

    def get_marker_center_native(self, slot=None):
        """The marker's TRUE projective center (2026-07-18, user), reprojected to the
        current frame. This is the deferred 'homography-native center' get_marker_center's
        docstring flags -- done RIGHT: the center is the DIAGONAL-INTERSECTION of the slot's
        GAUGE corners, a PROJECTIVE INVARIANT (the intersection of the two diagonals maps to
        the image of the marker's real center under ANY projective transform), reprojected
        via inv(frame_to_map). This differs from BOTH the corner-MEAN (get_marker_center /
        get_marker_frame_pts.mean -- perspective SHIFTS a midpoint off the true center) AND
        the rejected gauge-MEAN-then-project (42px error, 2026-07-15: the gauge mean isn't
        the center). WHY it's more robust for a partially-visible marker: the gauge corners
        are FIXED registry points (so the center never shifts as individual corners drop from
        this frame's tracked set), and the reprojecting homography is fit from ALL visible
        features incl. interior texture -- so when the marker OVERFLOWS the FoV (corners past
        the edges) but the centre+texture stay visible, the reprojected center still tracks.
        None if unavailable (caller falls back to the corner mean / decode)."""
        slot_name = slot if slot is not None else self._primary_slot_name()
        s = self.marker_slots.get(slot_name)
        if s is None or self.frame_to_map is None or not s.get('corner_ids'):
            return None
        mids = [fid for fid in s['corner_ids'] if fid in self.map_pts]
        if len(mids) != 4:                                   # need the full quad for diagonals
            return None
        g = np.array([self.map_pts[fid] for fid in mids])    # ordered TL,TR,BR,BL gauge corners
        c = _line_intersect(g[0], g[2], g[1], g[3])          # TL-BR x TR-BL = projective center
        if c is None:
            return None
        try:
            Hinv = np.linalg.inv(self.frame_to_map)
        except np.linalg.LinAlgError:
            return None
        out = self._apply_h(Hinv, c.reshape(1, 2))
        return out[0] if (out is not None and len(out) == 1 and np.all(np.isfinite(out[0]))) else None

    def get_marker_points(self, slot=None):
        """ALL currently-tracked points belonging to a slot -- the 4 corners (in ArUco
        [TL,TR,BR,BL] order, CORNER_WEIGHTS) plus surviving dense interior points (their
        stored edge-interpolated weights, see _seed_dense_points) -- reprojected to
        CURRENT-FRAME pixel coords via the map homography. The richer (position, weight)
        input for a moment-based ORIENTATION estimate with lower variance than the 4
        corners alone (see class docstring for why the asymmetric weights must be
        preserved). Returns (pts (N,2) float64, weights (N,) float64), or (None, None) if
        unavailable. NOT used for position -- see get_marker_center."""
        if slot is None:
            slot = self._primary_slot_name()
            if slot is None:
                return None, None
        s = self.marker_slots.get(slot)
        if s is None or self.frame_to_map is None:
            return None, None
        try:
            Hinv = np.linalg.inv(self.frame_to_map)
        except np.linalg.LinAlgError:
            return None, None
        ids, wts = [], []
        for i, fid in enumerate(s['corner_ids']):
            if fid in self.map_pts:
                ids.append(fid)
                wts.append(float(CORNER_WEIGHTS[i % 4]))
        for fid, w in s.get('dense_w', {}).items():
            if fid in self.map_pts:
                ids.append(fid)
                wts.append(w)
        if not ids:
            return None, None
        mapped = np.array([self.map_pts[fid] for fid in ids])
        px = self._apply_h(Hinv, mapped)
        return px, np.asarray(wts, dtype=np.float64)

    def should_trigger_decode(self, min_interval_frames=3, max_interval_frames=90,
                               conf_thresh=0.5):
        """Adaptive loop-closure scheduling: decode more often when confidence is low,
        but never more often than min_interval (decode has real cost) and never less
        often than max_interval (bound worst-case drift even if confidence looks fine).
        A failed marker-rigidity check on the PRIMARY slot (see _check_slot_rigidity)
        OVERRIDES min_interval -- that's a direct, targeted drift detection on the exact
        points that matter, not a global proxy, so it's worth paying the decode cost
        immediately rather than waiting out the throttle."""
        if not self.marker_rigid_ok:
            return True
        if self.frames_since_decode < min_interval_frames:
            return False
        if self.frames_since_decode >= max_interval_frames:
            return True
        return self.confidence < conf_thresh

    def reset_marker(self, slot=None):
        """Manual utility: demote one slot's (or, if slot=None, ALL slots') corner_ids
        back to ordinary (untagged) scene features. NOT called internally anymore --
        the multi-slot routing in loop_closure_correct (see its docstring) means a
        small<->big ID flicker no longer needs a reset at all, since each decode routes
        to its OWN slot by geometry rather than overwriting a single shared one. Kept for
        callers that want to force a full re-seed (e.g. after a confirmed full occlusion
        this module has no way to detect on its own). The old points are NOT deleted --
        they just stop being marker corners and remain as plain tracked map features
        (harmless, still useful for the homography fit)."""
        names = list(self.marker_slots.keys()) if slot is None else [slot]
        for name in names:
            self.marker_slots.pop(name, None)
        self._update_primary_scalars()

    def _seed_new_slot(self, decoded_corners):
        """Create a brand-new marker slot from a fresh decode with no existing-slot match
        (used at bootstrap, and whenever loop_closure_correct sees a decode that doesn't
        geometrically or size-wise belong to any existing slot). No-ops (silently drops
        the decode) once self.max_marker_slots slots already exist -- exactly 2 physical
        markers exist by design, so a 3rd distinct slot would indicate a spurious decode,
        not a new physical target."""
        if len(self.marker_slots) >= self.max_marker_slots:
            return None
        name = f"m{self._next_slot_id}"
        self._next_slot_id += 1
        new_ids = self._new_ids(len(decoded_corners))
        # BUGFIX (2026-07-15, found while adding get_marker_center/orientation): seed
        # via frame_to_map, matching every other seeding path (update()'s refill,
        # loop_closure_correct's re-anchor) -- storing raw pixels directly as map_pts was
        # only correct for the FIRST slot at bootstrap() (frame_to_map is identity there,
        # by construction, at the point bootstrap() calls this). For a SECOND slot
        # discovered mid-flight (the multi-slot small<->big design's whole point),
        # frame_to_map has already diverged from identity, so storing raw pixels would
        # silently seed that slot at the wrong map-gauge position -- a systematic offset
        # only visible once get_marker_center()/get_marker_frame_pts() is compared against
        # ground truth for a marker discovered after t=0. Harmless no-op at bootstrap
        # (frame_to_map == eye(3) there, so _apply_h is the identity map).
        H = self.frame_to_map if self.frame_to_map is not None else np.eye(3)
        mapped_pts = self._apply_h(H, decoded_corners)
        for fid, p, mp in zip(new_ids, decoded_corners, mapped_pts):
            self.map_pts[fid] = tuple(mp)
            self.tracked_px[fid] = tuple(p)
        self.marker_slots[name] = {
            'corner_ids': list(new_ids),
            'size': self._quad_size(decoded_corners),
            'shape_sig': None,
            'rigid_ok': True,
            'shape_change': 0.0,
            'dense_w': {},   # fid -> weight, for get_marker_points/orientation
            'inlier_fail_streak': 0,   # consecutive frames THIS SLOT's own corners were tracked
                                       # but RANSAC-rejected -- see update()'s inlier-survival check
        }
        self._seed_dense_points(name, decoded_corners, H)
        return name

    def _seed_dense_points(self, slot_name, decoded_corners, H,
                            n_per_side=6, scales=(1.0, 0.66, 0.33)):
        """Deterministic (texture-independent) interior points at fixed fractional
        positions along nested scaled quads -- same principle as img_data.py's
        _scaled_quad_points (LK tracks these from local ArUco edge/cell gradient
        structure, not photographic texture, so this works on the project's current
        plain/untextured marker -- see feedback_textured_marker_falsified for why an
        actually-textured marker was tried and reverted). Each point's WEIGHT is
        linearly interpolated between its originating edge's two CORNER_WEIGHTS
        endpoints -- preserves the TL-heavy asymmetry get_marker_orientation's moment
        computation needs (see class docstring); uniform weights here would make a
        large point cloud on a near-square marker MORE orientation-degenerate, not
        less. Seeded via the SAME homography H as the caller's corner seeding, so
        dense points land at the correct map-gauge position immediately (not just at
        the next _seed_new_slot bugfix's H)."""
        slot = self.marker_slots.get(slot_name)
        if slot is None:
            return
        c = np.asarray(decoded_corners, dtype=np.float64).reshape(-1, 2)
        if len(c) != 4:
            return
        ctr = c.mean(axis=0)
        pts, wts = [], []
        for sc in scales:
            quad = ctr + (c - ctr) * sc
            for i in range(4):
                a, b = quad[i], quad[(i + 1) % 4]
                wa, wb = CORNER_WEIGHTS[i], CORNER_WEIGHTS[(i + 1) % 4]
                for t in np.linspace(0.0, 1.0, n_per_side, endpoint=False):
                    pts.append(a + (b - a) * t)
                    wts.append(wa * (1.0 - t) + wb * t)
        pts = np.asarray(pts, dtype=np.float32)
        new_ids = self._new_ids(len(pts))
        mapped_pts = self._apply_h(H, pts)
        for fid, p, mp, w in zip(new_ids, pts, mapped_pts, wts):
            self.map_pts[fid] = tuple(mp)
            self.tracked_px[fid] = tuple(p)
            slot['dense_w'][fid] = float(w)

    def _topup_dense_points(self, slot_name, decoded_corners, dense_floor=0.5):
        """Re-seed a slot's dense points from a fresh, decode-accurate quad whenever
        KLT attrition has dropped survivors below `dense_floor` of the original count
        -- called from loop_closure_correct (a fresh decode is exactly when a clean
        reference quad is available).

        BUGFIX (2026-07-15, found via held-out orientation validation): the original
        version left old survivors in place and only ADDED a fresh batch on top. Because
        dense points rarely all die at once, `alive` stayed above the floor almost every
        call, but the ones that survived kept accumulating anyway on later calls that DID
        dip below floor -- n_dense_used was observed growing monotonically over a single
        descent (72 -> 144 -> 216 -> ... -> 648), each addition compounding two problems:
        unbounded KLT/RANSAC compute cost, AND (found the same session) stale points
        seeded when the marker was a very different apparent size/pose diluting any
        moment-based computation over the set with increasingly outdated geometry. Now
        REPLACES: drops this slot's entire existing dense set before reseeding a fresh
        target-sized batch anchored to the current decode, so the count never exceeds
        target and no point is ever older than one topup cycle."""
        slot = self.marker_slots.get(slot_name)
        if slot is None or self.frame_to_map is None:
            return
        target = 4 * 3 * 6   # 3 scales x 4 sides x 6 per side (default _seed_dense_points count)
        alive = sum(1 for fid in slot['dense_w'] if fid in self.tracked_px)
        if alive >= dense_floor * target:
            return
        for fid in list(slot['dense_w'].keys()):
            self.map_pts.pop(fid, None)
            self.tracked_px.pop(fid, None)
        slot['dense_w'] = {}
        self._seed_dense_points(slot_name, decoded_corners, self.frame_to_map)

    def identify_slot(self, decoded_corners, reproj_thresh_px=None):
        """PUBLIC: which existing marker slot does decoded_corners geometrically belong
        to (or None if it doesn't match any yet)? This is the same routing decision
        loop_closure_correct uses internally (see _classify_slot), exposed so CALLERS can
        get a held-out prediction for the RIGHT slot before deciding to correct anything --
        critical for any comparison/logging code that wants "what would the map alone have
        predicted for THIS specific decode", since get_marker_frame_pts(slot=None) instead
        returns the map's own notion of "primary" (largest slot), which is independent of
        and can disagree with whichever physical marker a caller's decode actually is (a
        real bug found 2026-07-15: img_data.py's shadow-mode logging compared its live
        decode against get_marker_frame_pts()'s primary-by-size slot regardless of which
        marker was actually decoded, producing spurious ~100px "errors" whenever the two
        notions of "primary" disagreed -- not a fault in the map itself, a fault in
        comparing it to the wrong slot). ALWAYS call this (or rely on
        loop_closure_correct's internal use of the same logic) rather than assuming
        slot=None is "the marker you decoded."""
        if reproj_thresh_px is None:
            reproj_thresh_px = 4.0 * self.ransac_thresh_px
        return self._classify_slot(decoded_corners, reproj_thresh_px)

    def _classify_slot(self, decoded_corners, reproj_thresh_px, reproj_thresh_frac=0.10):
        """Decide which physical-marker SLOT a fresh decode belongs to, using ONLY pixel
        geometry -- never the decoded ArUco id (see module docstring: a misdecoded id
        with correct corners must still route correctly, and the small<->big flicker must
        never be treated as identity evidence). Two-tier decision:

        1. GEOMETRIC (reprojection) match: compare decoded_corners against each existing
           slot's CURRENT KLT-tracked prediction (_slot_frame_pts). This is definitive
           when available -- if the decode lands close to where a slot's own tracked
           corners already are, it IS that slot, full stop, regardless of relative size
           (handles real scale changes as altitude varies). The acceptance tolerance for
           a given candidate is max(reproj_thresh_px, reproj_thresh_frac * that
           candidate's own current predicted size) -- a FIXED absolute px tolerance is
           too tight for a large, fast-growing near-touchdown marker: found 2026-07-15 by
           held-out validation (get_marker_center/get_marker_points against fresh-decode
           ground truth, tools/validate_planar_map.py's methodology) that a genuinely
           correct 15.68px match was REJECTED by the old fixed 12px threshold at marker
           size ~200px (only ~8% relative error), falling through to the ambiguous Tier-2
           size fallback below -- which then misrouted, because the OTHER (stale, not
           recently re-decoded) slot's remembered size happened to coincide with the
           correctly-growing marker's CURRENT size at that instant (the exact "handover
           boundary" scenario the multi-slot design exists for -- two markers' apparent
           sizes crossing). Once misrouted, loop_closure_correct's re-anchor SNAPS the
           wrong slot's map_pts to the wrong marker's geometry, and none of the existing
           local smoothness checks (resid_px, marker_rigid_ok, confidence) detect it
           afterward, since they only check consistency with the (now-corrupted) new
           reference, not absolute truth -- a single bad frame can silently corrupt a
           slot indefinitely. The relative-tolerance fix keeps the tight ABSOLUTE floor
           for small/distant markers (where a 10%-of-size tolerance would be too loose)
           while correctly accepting a genuine geometric match on a large one.
        2. SIZE fallback, only when no slot has a usable/close prediction (e.g. right
           after bootstrap, or a slot's corners were fully dropped by KLT): classify by
           quad size (see _quad_size) against each slot's last-known size. The project's
           two physical markers are printed at deliberately different scales specifically
           so this is unambiguous EXCEPT transiently right at a size crossover (see
           above) -- Tier 1's widened tolerance is what actually protects that window;
           this tier remains a coarser last resort. slot_size_margin bounds how close a
           decode's size must be (relative) to count as "the same slot" rather than
           triggering a new one.

        Returns an existing slot name, or None (meaning: seed a brand-new slot)."""
        size = self._quad_size(decoded_corners)

        # Tier 1: geometric match against tracked predictions, per-candidate relative tolerance.
        best_name, best_err, best_thresh = None, np.inf, reproj_thresh_px
        for name in self.marker_slots:
            pred = self._slot_frame_pts(name)
            if pred is None or len(pred) != len(decoded_corners):
                continue
            err = float(np.mean(np.linalg.norm(
                pred - np.asarray(decoded_corners, dtype=np.float64), axis=1)))
            cand_size = self._quad_size(pred)
            cand_thresh = max(reproj_thresh_px, reproj_thresh_frac * cand_size) \
                if cand_size and cand_size > 0 else reproj_thresh_px
            if err < best_err:
                best_name, best_err, best_thresh = name, err, cand_thresh
        if best_name is not None and best_err <= best_thresh:
            return best_name

        # Tier 2: size-based fallback.
        if size is not None and self.marker_slots:
            def size_key(name):
                s = self.marker_slots[name]['size']
                return abs(np.log(size / s)) if s and s > 0 else np.inf
            nearest = min(self.marker_slots, key=size_key)
            if size_key(nearest) <= np.log(1.0 + self.slot_size_margin):
                return nearest

        return None   # no existing slot matches -- caller should seed a new one

    def loop_closure_correct(self, decoded_marker_px_corners, marker_id=None, reproj_thresh_px=None):
        """Fresh ArUco decode succeeded this frame: use it to correct the map. The
        decoded corners are ABSOLUTE truth for this frame (no drift); route them to the
        correct physical-marker SLOT via _classify_slot (pixel geometry only, never the
        decoded id -- see module + _classify_slot docstrings), then re-anchor that slot's
        map entries to be exactly consistent with this decode. marker_id is accepted for
        caller-side logging only; it plays NO role in the routing decision.
        reproj_thresh_px defaults to 4x ransac_thresh_px if not given."""
        if reproj_thresh_px is None:
            reproj_thresh_px = 4.0 * self.ransac_thresh_px
        name = self._classify_slot(decoded_marker_px_corners, reproj_thresh_px)

        if name is None:
            self._seed_new_slot(decoded_marker_px_corners)
            self.frames_since_decode = 0
            self._update_primary_scalars()
            return

        slot = self.marker_slots[name]
        corner_ids = slot['corner_ids']
        if len(corner_ids) != len(decoded_marker_px_corners) or self.frame_to_map is None:
            # Slot exists but its corner-id bookkeeping doesn't line up 1:1 with this
            # decode (e.g. some corner ids were pruned elsewhere) -- re-seed IN PLACE
            # (same slot name/size history retained, just fresh corner_ids) rather than
            # silently mismatching count via zip.
            new_ids = self._new_ids(len(decoded_marker_px_corners))
            mapped_pts = (self._apply_h(self.frame_to_map, decoded_marker_px_corners)
                          if self.frame_to_map is not None else decoded_marker_px_corners)
            for fid, p, mp in zip(new_ids, decoded_marker_px_corners, mapped_pts):
                self.map_pts[fid] = tuple(mp)
                self.tracked_px[fid] = tuple(p)
            slot['corner_ids'] = list(new_ids)
        else:
            # Re-anchor: recompute what map position each corner id SHOULD have (via the
            # current frame_to_map), and snap it to be exactly decode-consistent.
            mapped_pts = self._apply_h(self.frame_to_map, decoded_marker_px_corners)
            for fid, p, mp in zip(corner_ids, decoded_marker_px_corners, mapped_pts):
                self.tracked_px[fid] = tuple(p)
                self.map_pts[fid] = tuple(mp)

        # EMA-smooth the slot's size estimate (used by _classify_slot's size fallback).
        new_size = self._quad_size(decoded_marker_px_corners)
        if new_size is not None:
            slot['size'] = new_size if slot['size'] is None else 0.7 * slot['size'] + 0.3 * new_size

        # Top up dense points from this fresh, decode-accurate quad if KLT attrition has
        # thinned them out (see _topup_dense_points docstring).
        if 'dense_w' not in slot:
            slot['dense_w'] = {}
        self._topup_dense_points(name, decoded_marker_px_corners)

        self.frames_since_decode = 0
        self._update_primary_scalars()
