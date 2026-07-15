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
import numpy as np
import cv2


class PlanarFeatureMap:
    """Incremental 2D feature map. All positions (map and tracked) are in PIXEL units of
    an arbitrary, self-consistent gauge (the map's own coordinate frame, established at
    bootstrap) -- no physical size or depth ever enters. Marker corners are folded in as
    just another set of mapped features, tagged (per physical-marker SLOT) so their map
    position can be queried separately (`get_marker_frame_pts`).
    """

    def __init__(self, max_features=300, min_tracked=8, refill_below=60,
                 lk_win=(21, 21), lk_max_level=3, gft_quality=0.01, gft_min_dist=7,
                 ransac_thresh_px=3.0, conf_track_floor=15, conf_resid_ceiling=4.0,
                 marker_rigid_thresh=0.35, max_marker_slots=2, slot_size_margin=0.4):
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
        self.max_marker_slots = max_marker_slots   # exactly 2 physical markers (small+big) by design
        self.slot_size_margin = slot_size_margin   # relative quad-size tolerance for same-slot classification

        self.map_pts = {}              # feature_id -> (x, y) in MAP gauge
        self.next_id = 0
        self.tracked_px = {}       # feature_id -> current-frame pixel position
        self.prev_gray = None
        self.frame_to_map = None   # 3x3 homography: map_pt(homog) ~ frame_to_map @ [px;1]
        self.confidence = 0.0
        self.n_tracked = 0
        self.resid_px = np.inf
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

    def bootstrap(self, gray, marker_px_corners=None, marker_id=None):
        """First frame: seed the map directly from this frame's own pixel coords (map
        gauge = frame-0 pixel gauge, arbitrary but self-consistent). marker_px_corners,
        if given (Nx2, from a fresh ArUco decode), seed the FIRST marker slot. marker_id
        is accepted for caller-side logging only -- never used to decide slot identity
        (see module docstring)."""
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
        self.n_tracked = len(self.tracked_px)
        self.resid_px = 0.0
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

    def update(self, gray):
        """Track existing features frame-to-frame via KLT, refill if running low, refit
        the frame->map homography, update confidence. Call once per frame after bootstrap."""
        if not self.initialized:
            raise RuntimeError("PlanarFeatureMap.update() called before bootstrap()")

        ids = list(self.tracked_px.keys())
        prev_pts = np.array([self.tracked_px[i] for i in ids], dtype=np.float32).reshape(-1, 1, 2)
        new_pts, status, _ = cv2.calcOpticalFlowPyrLK(self.prev_gray, gray, prev_pts, None, **self.lk_params)
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
            if H is not None:
                self.frame_to_map = H
                pred = self._apply_h(H, src)
                self.resid_px = float(np.sqrt(np.mean(np.sum((pred[inliers] - dst[inliers]) ** 2, axis=1)))) \
                    if inliers.sum() > 0 else np.inf
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
        # corners survived KLT this frame; if any were dropped (tracking loss or the
        # RANSAC outlier rejection above), that slot's shape can't be checked this frame
        # (itself a signal for that slot specifically -- does NOT affect the other slot).
        for slot in self.marker_slots.values():
            pts = [survivors[i] for i in slot['corner_ids'] if i in survivors]
            if len(pts) == 4:
                self._check_slot_rigidity(slot, np.array(pts))
            else:
                slot['rigid_ok'] = False
                slot['shape_change'] = np.inf
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
        self.confidence = float(track_conf * resid_conf * marker_conf)

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
        for fid, p in zip(new_ids, decoded_corners):
            self.map_pts[fid] = tuple(p)
            self.tracked_px[fid] = tuple(p)
        self.marker_slots[name] = {
            'corner_ids': list(new_ids),
            'size': self._quad_size(decoded_corners),
            'shape_sig': None,
            'rigid_ok': True,
            'shape_change': 0.0,
        }
        return name

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

    def _classify_slot(self, decoded_corners, reproj_thresh_px):
        """Decide which physical-marker SLOT a fresh decode belongs to, using ONLY pixel
        geometry -- never the decoded ArUco id (see module docstring: a misdecoded id
        with correct corners must still route correctly, and the small<->big flicker must
        never be treated as identity evidence). Two-tier decision:

        1. GEOMETRIC (reprojection) match: compare decoded_corners against each existing
           slot's CURRENT KLT-tracked prediction (_slot_frame_pts). This is definitive
           when available -- if the decode lands close to where a slot's own tracked
           corners already are, it IS that slot, full stop, regardless of relative size
           (handles real scale changes as altitude varies).
        2. SIZE fallback, only when no slot has a usable/close prediction (e.g. right
           after bootstrap, or a slot's corners were fully dropped by KLT): classify by
           quad size (see _quad_size) against each slot's last-known size. The project's
           two physical markers are printed at deliberately different scales specifically
           so this is unambiguous; slot_size_margin bounds how close a decode's size must
           be (relative) to count as "the same slot" rather than triggering a new one.

        Returns an existing slot name, or None (meaning: seed a brand-new slot)."""
        size = self._quad_size(decoded_corners)

        # Tier 1: geometric match against tracked predictions.
        best_name, best_err = None, np.inf
        for name in self.marker_slots:
            pred = self._slot_frame_pts(name)
            if pred is None or len(pred) != len(decoded_corners):
                continue
            err = float(np.mean(np.linalg.norm(
                pred - np.asarray(decoded_corners, dtype=np.float64), axis=1)))
            if err < best_err:
                best_name, best_err = name, err
        if best_name is not None and best_err <= reproj_thresh_px:
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

        self.frames_since_decode = 0
        self._update_primary_scalars()
