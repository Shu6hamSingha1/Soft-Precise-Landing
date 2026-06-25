---
name: Marker detection breakdown is the failure mechanism, not lag
description: Failed PX4 landings have frozen Image_Feature_Pts for the final 100-150 ms — marker detection stops, controller uses stale data, drifts laterally. The 16× outcome gap likely has TWO causes (lag AND occluded marker detection at low altitude), not one.
type: feedback
originSessionId: 3ffdaf6f-0da4-4c9f-b4df-fb9a91a7eb04
---

> ⛔ SUPERSEDED/CORRECTED 2026-06-26: The decode-fail-on-in-FoV and nan-quat-sentinel observations stand, but the prescribed perception fixes (KLT corner-track / multi-marker / ArUco tuning / corners-CBF) are obsolete and moot now that fly-aways are eliminated (marker stays in FoV). The PX4 lateral "wall" was a gain-parity bug + the velocity-damping lever (tighten the lateral flow funnel XI2_xy), NOT a perception/architecture/inner-loop-velocity limit; the combined sliding surface σ=ζ_h+χ_r·ζ_r is baked default-on and gives 10/10 bounded landings. The residual is a terminal SOFT velocity kick (≈38ms lag), not a precision wall. See [[feedback_flow_funnel_zetah_works]]. Content below kept as history.

## ⭐ 2026-06-10 — QUANTIFIED (12-TL audit) + the κ-runaway link
Re-categorized **all 12 campaign TLs by the ACTUAL image centroid/corners** (not the virtual): **9/12 had the marker FULLY in-FoV at the loss (4/4 corners in, mostly centered) → ArUco DECODE failure on a visible marker** — confirming this memory's "degraded image-quality, NOT geometric" (line 59), now quantified. Only **3/12 were clips** (≥1 corner out), and 2 of those = the drone had already flown off. **The marker does NOT geometrically leave; the detector freezes on an in-frame marker.**
- **The nan quaternion in `Img_Data["Quat"]` is the marker-LOST sentinel** (`img_data.py:1007-1008` — `# marker lost: no synced IMU pairing`), NOT an attitude failure (FC telemetry quat valid throughout, norm=1). On detection-fail img_data HOLDS `_feature_pts` + EXTRAPOLATES `_img_feature_param` + logs nan quat (`:1008-1014`).
- **This detection loss is the TRIGGER of the lateral κ-runaway** ([[feedback_lateral_kappa_runaway]]): loss at 0.80 of flight PRECEDES the κ-runaway at 0.88. Freeze → held/extrapolated feature → the controller's VIRTUAL centroid reprojects OFF-SCREEN (s≈−820px, via the unguarded `_getVirtualPts` z_v→0 divide under tilt) → `cross(w_i,s)` → wrong `h_d` → funnel breach → κ runaway → fly-away.
- **The real ArUco corner points ARE stored** (`_feature_pts` = 4 actual corners/frame; `Img_Data["Image Feature Pts"]`) — and are the lever (user insight 2026-06-10): when ArUco can't decode an **in-FoV** marker, **KLT corner-track** the corners through the decode gap instead of freezing + extrapolating the centroid off-screen. `MARKER_KLT_MAX_STEPS=20` does this but bridges only ~0.33 s (losses run 19% of flight across 3 episodes → exhausts → freeze). **Levers:** extend/improve KLT corner-tracking (corners are in-FoV — track them); a **corners-based CBF** (constrain all 4 corners in-FoV, not the Phase-1 centroid-only); clip-tolerance for the 3/12 edge cases; and guard the `_getVirtualPts` z_v divide so a stale/tilted feature can't fabricate an off-screen `s`.
- **DIRECTIVE (user, 2026-06-10): during marker-LOST, USE whatever data is GENUINE — don't nan + extrapolate.** The current branch (`img_data.py:1007-1009`) discards genuine signals: it nan's the quat + IMU and **extrapolates** the centroid (which reprojects off-screen → wrong `h_d` → κ-runaway). But genuine data exists: the **FC attitude quat** (valid throughout), the **KLT-tracked in-FoV corners**, the **partial (still-in-FoV) corners**, the **FC IMU body-rate**. Replace nan+extrapolate with: reproject the genuine KLT/partial corners using the genuine FC quat → a bounded, in-FoV `s`; **never extrapolate `s` off-screen**. Degrade gracefully on real data, don't synthesize a phantom.

## Top finding (2026-05-22)

The σ-trajectory comparison (`feedback_phase5_sigma_divergence.md`) showed PX4 failed reps diverge in σ_xy in the final descent. The mechanism is now clear: **marker detection breaks down in the final 100-150 ms**, causing the controller to act on stale image-feature data.

Direct evidence from DefaultN10 bundle:

| Rep | xy_end | Last 15 side measurements | Pattern |
|---|---|---|---|
| rep2 (PRECISE) | 0.033 | 122 → 130 → 141 → 167 → 186 → 209 → 27 → 36 → 54 → 80 → 92 → 92 → 88 | Updating every frame |
| rep7 (FAILED) | 1.039 | 86 → 86 → 86 → 86 → 86 → 86 → 86 → ... (16 frames at 86.1 px) | **FROZEN** |
| rep6 (FAILED) | 0.996 | 81 → 81 → 81 → ... (15 frames at 81.4 px) | **FROZEN** |

`img_data.py:408-421` holds the last marker position when detection fails ("marker hasn't moved much in a frame or two"). Used in PRECISE reps for brief recovery; in failed reps it persists for 100+ ms while the drone descends and drifts.

## Image-quality issues (visible in validate_image.py sample)

1. **Drone propellers and body in FoV** — the downward camera sees the drone's own rotors framing the marker
2. **Drone body shadow** falls on the marker, reducing contrast
3. **Big marker (34 cm) exceeds the FoV at z<0.3m** — the marker's edges leave the frame so ArUco can't detect the complete pattern
4. **Small marker (6 cm) is detectable only at z≤1m** — its narrow detectable range coincides with the period where the drone body occludes it most

## Dual-marker setup

- Big marker ID 10, ~34 cm — detectable z ∈ [0.3, 5+] m
- Small marker ID 0, ~6 cm — detectable z ∈ [0.0, ~1] m
- Both painted concentric on a 50×50 cm plane
- `img_data.py:320`: code prefers smallest ID → picks small when both detect

The switch itself is BENIGN — markers are concentric, no centroid jump (Δcentroid < 5 px). PRECISE reps switch 1-2 times during descent.

## The risky regime

**Altitudes z ∈ [0.1, 0.3] m** where BOTH markers can fail simultaneously:
- Big marker too big for FoV + occluded by drone body
- Small marker should be detectable but drone shadow can occlude it intermittently

This is exactly when the controller most needs accurate measurement.

## Revised understanding of the 16× gap

Previously attributed entirely to MAVSDK lag (`feedback_impulse_response.md`, `feedback_phase5_sigma_divergence.md`). Now appears to have two contributing causes:

1. **MAVSDK rate-loop lag** (~30 ms) → deterministic ~6× slowdown of convergence
2. **Marker detection breakdown at low altitude** → stochastic stale-data failures in the final 100-150 ms

Both contribute; ratio between them isn't separately quantified yet but likely both significant.

## Design constraint (user-stated 2026-05-22)

**The small marker size and camera position are DELIBERATELY chosen so that the small marker fits the camera FoV exactly when the drone has landed precisely over the target.** Enlarging the small marker would push it out of frame at touchdown and break the design. Tilting/repositioning the camera would similarly break the at-touchdown FoV match.

So the geometric capacity for detection IS THERE even when drone is moderately off-center (small marker has ~26 cm of lateral tolerance at z=0.2m before leaving frame). The detection failure is NOT a geometric issue — it's a **degraded image-quality** issue: drone body shadow lowers contrast over the marker, ArUco's strict detection criteria fail on the degraded image, feature data freezes.

## Practical interventions (respecting the design constraint)

| Intervention | Mechanism | Cost |
|---|---|---|
| **Tune ArUco DetectorParameters** for robust low-contrast detection | Lower `adaptiveThreshConstant`, relax `polygonalApproxAccuracyRate`, raise `errorCorrectionRate` | `img_data.py` edit |
| **Freeze rate setpoint on stale feature** instead of applying derivative blindly | Prevents the "use stale data → drift" spiral | `controller.py` or `landing_test.py` edit |
| **Detect feature staleness explicitly** — when current `Image Feature Pts` exactly matches previous frame, treat as "lost" with shorter grace | Stops the slow-fail mode without depending on ArUco's own detection-fail signal | `img_data.py` flag |
| **Add multiple small markers at offsets** within the same camera-aligned patch | Robust to single-marker shadow occlusion; preserves the at-touchdown FoV constraint | Texture + img_data.py update |
| **Increase camera exposure dynamically at low altitude** | The drone shadow drops local brightness; auto-exposure or fixed bias may help | Gazebo SDF + camera plugin |
| **Reduce CHECK_NUM grace** (currently 80 frames ≈ 1s) | Properly flags these failures as TARGET_LOST so they don't masquerade as "soft" landings | One line in img_data.py |

**NOT recommended (would violate the design constraint):**
- Enlarge small marker — pushes it out of FoV at touchdown
- Reposition camera lower/tilted — breaks the FoV match at touchdown

## How to apply

- When the user asks why some PX4 landings fail dramatically: cite marker-detection breakdown as the proximate cause, not just lag.
- The MATLAB controller is still correct (Phase 1) — MATLAB doesn't have drone-body occlusion or this marker-detection issue because it uses analytical projection of feature points.
- Before tuning MC_*RATE_P or migrating to uXRCE-DDS, addressing the marker-detection breakdown may give the bigger win (especially the small-marker-size fix).

## Data + tooling

- `PX4_Gazebo/analyze_marker_switch.py` — per-rep side-trajectory analyzer; spots frozen-side patterns
- `~/PX4-Autopilot/Tools/simulation/gz/models/arucotag/0-small_10-big.png` — the dual-marker texture
- `~/PX4-Autopilot/Tools/simulation/gz/models/arucotag/model.sdf` — 0.5×0.5 m plane definition
- `PX4_Gazebo/run_logs/validate_sample_annotated.png` — sample frame showing drone occlusion
