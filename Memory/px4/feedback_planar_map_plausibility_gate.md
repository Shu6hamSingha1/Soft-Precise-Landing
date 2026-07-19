---
name: feedback_planar_map_plausibility_gate
description: "PlanarFeatureMap override/rescue paths need independent plausibility rejection (position+size), not clipping, and separate gates; V_aruco_norm[0] vs [1] decode-vs-tracked distinction."
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 7bc77c5e-027e-4e24-82cb-7e2996f36559
---

PlanarFeatureMap (src/planar_map.py) predicts single-frame corner positions and may
OVERRIDE (`V_aruco_norm[1]`, when raw decode already succeeded) or RESCUE
(`extrapolated_img_feature_param`, when raw decode fails) img_data.py's feature vector.
Both paths must independently REJECT (not clip) implausible predictions before
consuming them, checked via shared `_planarMapPredictionPlausible(pm_px, quat)`
(src/img_data.py, added 2026-07-16, right after `_computeFeatureVec`):
- POSITION bound: FoV-aware (`self.center/self.focal + half-extent of last-held
  `_feature_pts`, margin 1.5x`)
- SIZE bound: ratio of implied pixel extent vs `_last_real_extent_px` (last genuine
  raw-decode span), reject if outside [1/2.0, 2.0]

**Why:** A `np.clip(s, -5, 5)` inherited from the old polynomial-extrapolation guard was
numerically wrong for this camera (real FoV bound is ~±1.2 normalized units, not ±5) —
`s_e_n = s_e/p_10` amplifies even a "clipped" bad value into a double-digit normalized
error, triggering the same κ-ratchet/a_u explosion this project has repeatedly traced to
bad position signals. Confirmed live: this exact chain caused IC1's 142m fly-away
(ICValidation/20260716-211434) when the OVERRIDE path had zero plausibility checking
(only RESCUE did, and rescue/override were sharing one gate, `_planar_map_gate_on`
map_confidence-based) — a confident map can still project outside the FoV if its
homography drifted during an extended dropout.

Fix applied 2026-07-16: (1) split into two gates — `_planar_map_gate_on`
(map_confidence, marker-independent, for RESCUE) vs `_planar_map_override_gate_on`
(confidence, marker-aware/stricter, for OVERRIDE, since override replaces already-good
data); (2) both consumer sites now call the shared `_planarMapPredictionPlausible` helper
instead of duplicated/absent inline checks.

Also: `V_aruco_norm[0]` is built from `aruco_pts_0` — the genuine fresh
`cv2.ArucoDetector.detectMarkers()` output (img_data.py:1584, on `imgs[0]`).
`V_aruco_norm[1]` is built from `aruco_pts_1` — `calcOpticalFlowPyrLK`-TRACKED corners
propagated from `aruco_pts_0` into `imgs[1]`, NOT a re-decode. Only `[0]` is "raw decode";
`[1]` (the one PlanarFeatureMap conditionally overrides) is one KLT hop removed. Don't
conflate "computed from raw decode" with `V_aruco_norm[1]` in future discussion — say
"computed from real image-derived (KLT-tracked) data" instead.

**How to apply:** Any future PlanarFeatureMap consumer site (or any other module that
conditionally substitutes a geometric prediction for a real measurement) needs its own
plausibility gate — a high `map_confidence`/`confidence` score is necessary but NOT
sufficient; it says nothing about whether THIS SPECIFIC frame's projection is
geometrically sane. See [[project_planar_map_architecture]] if it exists, else this is
the standing reference.
