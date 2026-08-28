---
name: project_20260828_guaranteed_sp_synthesis
description: "2026-08-28 SYNTHESIS: guaranteed SOFT+PRECISE across IC1-5 achieved UNDER GT-FEEDBACK (25/25 SP, n=5/IC, kappa_ratchet campaign). This file collects WHY every prior landing-failure era happened and the CUMULATIVE set of fixes (no single silver bullet) that closed them. ⚠ SCOPE: GT-FB = control isolated from perception. REAL PERCEPTION (GT-FB OFF) cross-marker is NOT there yet -- separate open problems (320x240 sensor-cal accuracy, oblique-low detection collapse, off-center kappa-LEAKAGE convergence wall)."
metadata:
  node_type: memory
  type: project
---

## RESULT: guaranteed SP across IC1-5 UNDER GT-FEEDBACK

2026-08-28 kappa-ratchet campaign (tune-plasmc methodology, n=5 GT-FB per IC,
cross_marker world), 25 reps total:

| IC | ENU | result | kappa_xy_max | sigma_xy_max (E=1) | breach | a_u_xy_max |
|----|-----|--------|-------------|--------------------|--------|-----------|
| IC1 | 0,0,5  | 5/5 SOFT+PRECISE | 0.50 | 0.42 | 0% | ~0.5 |
| IC2 | 2,2,5  | 5/5 SOFT+PRECISE | 0.50 | 0.32 | 0% | 1.4 |
| IC3 | -2,2,5 | 5/5 SOFT+PRECISE | 0.50 | 0.50 | 0% | 1.5 |
| IC4 | 2,2,7  | 5/5 SOFT+PRECISE | 0.50 | 0.25 | 0% | 1.1 |
| IC5 | 2,2,3  | 5/5 SOFT+PRECISE | 0.50 | 0.46 | 0% | 2.1 |

**25/25 SP.** xy_err 0.001-0.047 m, rel_vel 0.007-0.032 m/s. kappa_xy stays PINNED
at its 0.50 init on EVERY rep (never adapts up -- sigma never saturates); the
`switch = theta*sat(sigma)*kappa` term stays near zero.

---

## WHY EVERY PRIOR FAILURE ERA HAPPENED (root causes, chronological)

The PLASMC PX4/Gazebo campaign spent ~2000 SITL reps across ~15 months hitting a
sequence of distinct walls. Each was real; none was the whole story:

1. **MAVSDK rate-loop lag (architectural ceiling).** ~38 ms roll/pitch, ~287 ms
   yaw actuation delay. MATLAB gains DON'T port (chi_r=2.0 -> catastrophic; PX4
   ceiling chi_r=0.5). Set a floor on terminal velocity (~0.2 m/s / 0.37 m/s
   rel_vel) that no gain removes.

2. **Kappa-runaway / ratchet.** The adaptive-gain ODE
   `dkappa/dt = theta*N*G*|sigma| - N*P*kappa` runs away when |sigma| stays
   saturated with NO valid tracking signal (off-center IC + degrading perception,
   OR the terminal 1/Z regime). Near touchdown the 1/Z geometry breached the
   INNER funnel (|h_e/p_2|->0.99) -> zeta->5.3 -> sigma->3.6 -> growth term
   (16.1) overwhelmed leakage (0.10) by 160x -> kappa_xy slammed up. kappa_xy was
   UNCAPPED (KAPPA_MAX=[1e6,1e6,3.0], only z capped). theta is a SHARED SCALAR ->
   the blow-up detonated every axis. a_u_xy reached 162, 9e3, 1e6.

3. **Z_REG harness artifact.** Under GT-FB the computed camera-marker depth fell
   below the physical landing-gear floor -> fake unbounded 1/z -> 100 -> kappa_eq
   107 / sigma 3.66 blow-up. The ENTIRE "0-SP terminal wall" era was substantially
   this ARTIFACT (no perception-ON analog). Killed by Z_REG=0.2 in gt_feedback.py.

4. **Frozen kappa_xy.** `N_xy=0.02` gave the kappa-ODE tau = 1/(N*P) = 33 s, far
   longer than the ~7 s descent -> kappa_xy NEVER adapted -> zero lateral
   adaptive robustness. (A de-facto safeguard against #2, but it also meant no
   convergence authority.) The lateral "wall" that many gain sweeps chased was
   substantially this frozen-kappa bug, not a fundamental limit.

5. **Outer-funnel demand starvation.** The outer barrier-PID is feedback-
   linearizing and PROVABLY drives s_e -> 0 -- but only if the inner loop
   achieves the commanded feature velocity. It doesn't (`s_e_dot ~= ds_d - h_e`),
   so near the funnel edge the unmodeled h_e dominates and s_e diverges. At the
   bound the back-mapped demand saturates (`G_s^-1 ~ p_s -> 0`) -> s_e_n PINS at
   the +-0.95 margin then BREACHES. No outer P/I/D gain fixes this.

6. **Terminal 1/Z positive-feedback fly-away.** The loom `h_z = vz/z` has loop
   gain proportional to 1/z -> infinity near touchdown; it self-excites
   (h_z up -> thrust up -> ascent -> corner-velocity up -> h_z up). Noise only
   SEEDS it; geometry provides the gain. kappa can't fix it (the spike enters
   `u_eq`'s c-term `-h_z*h`, quadratic in flow, bypassing the switching term).

7. **Ring-fusion loom corruption.** The corner+ring fusion EKF fed the
   controller's h(t). Fixed ring radii (41-199 px) physically overlap the marker
   once MARKER_EXTENT_PX > ~160 px near touchdown -> whole ring tiers sample the
   MARKER (depth-mixing + LK aperture failure) SIMULTANEOUSLY, so the MAD median
   can't reject -> ring loom -0.86..-1.9 vs GT ~0 -> lateral kick via the
   loom x flow cross term. The default-ON fusion had never been validated.

8. **Yaw double-integrator limit cycle.** `u_a` is a yaw-RATE command
   (`psi_d = integral u_a`), so `Omega_a * ie_a` is a SECOND integrator with no
   phase margin vs the PX4 inner lag -> growing +-40 deg yaw cycle that pumps the
   lateral cycles through image-frame coupling. The yaw cycle was the hidden
   MASTER driver of the lateral cycles.

9. **Compass yaw drift at descent START.** The IC rig held/gated on the EKF yaw
   while the true (Gazebo) yaw was ~77 deg off -> the drone BEGAN the descent
   physically yawed ~77 deg -> psi_d -> +-180 deg -> xy 2-16 m. `alpha_start ~=
   GT_yaw_start` every rep (alpha was always correct -- three alpha redesigns all
   hit 180 deg chasing the wrong cause). It was a TEST-RIG bug, not a controller
   or feature bug.

10. **Funnel FLOOR as the mechanical breach trigger.** `P2INF_XY = 0.5/1.0` was
    directly traced (IC2 GT-FB n=5 A/B) as the MECHANICAL TRIGGER for the
    funnel-breach / Singhal-containment / dh_d-leak / a_u-thrash chain: s_e_n was
    SMALL and still CONVERGING right up to the moment p(t) hit its floor. The
    funnel getting tight -- NOT the tracking error growing -- triggered every
    observed breach. 0.5 gave 2/4 severe (a_u to 162) + catastrophic outliers
    (a_u to 1e6) once AU_LEAD/KF-retune were layered on top trying to patch the
    consequences.

11. **Rover / moving-target infra + cycle** (separate, not IC1-5): POSE_IDX
    landmine (bridge FULL pose/info); no landing platform (marker was visual-only
    +0.5 m, no collision) -> rover fly-away; a residual ~1.3-1.7 rad/s rotating
    lateral limit cycle on curved paths that gain knobs CANNOT damp (they can't
    add command quadrature) -- exit is PLASMC_AU_LEAD or platform size.

---

## THE FIXES THAT CLOSED IT (cumulative -- NO single silver bullet)

### Control-law bakes (2026-06 .. 2026-08, committed in controller.py / gt_feedback.py)

| Fix | Kills | Value |
|-----|-------|-------|
| `Z_REG = 0.2` (gt_feedback.py) | #3 the harness 1/z artifact -- bounds the GT-FB disturbance so Lyapunov Assumption 1 (beta=1/Z bounded) holds | 0.2 m (gear floor) |
| `KAPPA0_XY = 0.5` | #4 frozen kappa -- arms lateral adaptation WITHOUT runaway (safe only paired with the velocity damping below) | 0.5 (was 0.156) |
| `P2INF_XY = 2.5` (REBAKE 2026-08-28) | #10 funnel-floor breach trigger -- widens the floor so p(t) never gets tight enough to mechanically trigger the chain while s_e_n is still converging | 2.5 (was 1.0, was 0.5) |
| `W_U_MAX = 2.0` | #6 the 1.0 body-rate clamp DISCONTINUITY seeded the terminal limit cycle (clamp->bang-bang->bigger cmd->more clamp); at 2.0 the cmd stays below the cap -> no discontinuity -> cycle not seeded | 2.0 rad/s (was 1.0) |
| `XIR (gamma_s) = 0.10` | #5 demand starvation on off-center ICs -- SLOW outer-funnel contraction keeps p_r WIDE so off-center s_e_n converges AND STAYS converged (fast XIR contracts onto the off-center error -> edge-forces) | 0.10 (was 0.15) |
| `chi_z = 0.1` | #6 the 1/z fly-away's under-braked-at-altitude half -- strengthened descent integral drives h_ez -> 0 (fly-aways 35 -> 3 / 300) | 0.1 (was 0.025) |
| `YAW_OMEGA = 0.1` + conditional `ie_a` integration | #8 the yaw double-integrator cycle -- removes the un-phase-margined 2nd integrator; freeze ie_a while heading-rate saturated (windup 102 deg -> -22 deg) | 0.1 (was 0.5) |
| 2026-06-11 lateral gain chain: `KD_XY=0.5, KI_XY=0.1, XIS_XY=0.5, PSINF_XY=0.35, XI2_XY=0.6, KAPPA0_XY=0.5` | #5 + the "lazy middle funnel" (MATLAB gamma_2=0.2 kept sigma at 5-20% of E for ~10 s -> SMC asleep, kappa LEAKS) -- XI2_XY 0.2->0.6 closes a 2 m offset in ~3 s; KD_XY restores phase lead (structural overshoot); KI_XY 1.0->0.1 stops the windup pushing ds_d through center | see table |
| `E_z = 0.5`, `KAPPA0_Z = 1.0`, `N_z = 0.1`, `P2INF_z = 1.5`, `P_z = 2.5` | descent softness -- z limit cycle rang 100% inside the boundary layer (sat linearized -> kappa's switching damping locked out); KAPPA0_Z bootstrap gives z braking from t=0 | see table |
| `E_xy = 1.0` (KEPT -- reducing it is a CONFIRMED dead-end even at N=1) | -- the relay+lag mechanism is FUNDAMENTAL: tightening E engages kappa-switching as a relay the 38 ms lag feeds in-phase | 1.0 |

### Perception-pipeline bakes

| Fix | Kills | Notes |
|-----|-------|-------|
| `FLOW_FUSE_RING = 0` (2026-07-09) | #7 ring-loom corruption -- ALSO removed a sigma driver that fed the kappa-ratchet (skill failure mode 11) | default OFF |
| Moment loom `-0.5 * d(ln M)/dt` (size-normalized corner spread) | terminal VERTICAL launch -- a direct area-rate scalar, no pinv(L_s), CONTINUOUS across marker switches (\|dh_z\| 4.4 -> 0.4) | |
| KLT fallback + relaxed 3/4-corner gate + parallelogram completion | short ArUco decode outages -> TARGET_LOST | scale-free, 2D-pixels-only |
| IC rig servos NED yaw to null the TRUE (Gazebo) yaw + gates on truth | #9 the compass-drift descent-start yaw runaway | landing_test.py, NOT the controller |
| COMPASS_FREE_VALIDATE (EKF2 mag OFF at engage) + BODY_YAW_SOURCE=alpha | compass yaw drift under maneuvers -- SO(3) loop uses drift-free alpha, not euler[2] | default ON |

### This session (2026-08-27/28) -- 320x240 + cross-marker perception hardening

| Fix | Commit | What |
|-----|--------|------|
| camera 640x480 -> 320x240 (fx 270 -> 135) | e31fbfd | restores MATLAB-native scale; needed >= 30 Hz process_frame() |
| process_frame() 17.5 -> 46 Hz | bf7af54 | root-caused the low rate to two marker-size-QUADRATIC hot spots (detect()'s Canny/Hough, multiscale_good_features' GFT-over-full-extent + O(n^2) dedup) -- bounded both to a fixed working resolution. NOT GPU/PBR (raw cam publishes 60 Hz). |
| h optical flow via background texture | bf7af54 | extent-ROI + line-exclusion + multiscale GFT through the REAL _solve_jacobian -- h was structurally never using the textured surface (design-intent gap). GT-corr r=0.66-0.78 on resolvable grain. |
| hybrid bg-flow (default ON) | aa8d0ee | CLAHE + forward-back LK on the det.ok path; rel_resid-gated bbox-only dense-DIS fallback on miss frames (695 fires on a real-perception IC5 miss-heavy rep) |
| 320x240 sensor cal | d56c0be, then 6-run | near-DIAGONAL from GT-FB landing recordings (the phased x/y excitation is untrackable by PX4 at 320x240 -> drone moves ~6 cm -> Hx/Hy rows fit from noise -> whole 6x6 blows up NEGATIVE R^2). diag(0.79, 0.79, 0.95, ., ., 0.59). Held-out landing r ~ 0.97. |
| terminal h_x/h_y via centroid-rate (default ON) | 47333a0 | when the bg-flow solve is UNHEALTHY (rel_resid high), blend h_x/h_y toward a KF-smoothed de-loomed CENTROID-RATE `d(s_V)/dt + h_z*s_V`. Scale-free, motion-agnostic. LIVE: terminal corr ~0 -> +0.60/+0.81, std ~1.3 -> ~0.05. GATE = bg-flow health (rel_resid + n_pts), NOT MARKER_EXTENT_PX (no proximity proxy). |
| IMU angvel/fps/stamp logging; z_v<=0 flow guard | f6e102d | lets offline analysis reproduce the LIVE gyro-derotated solve; drops behind-camera (sign-flipped) flow points |
| 320x240 oblique-low detector retune | 6b69c7b | HOUGH_THRESHOLD 25->16, MIN_LINE_LEN 15->10, MAX_LINE_GAP 10->16, MASK_DILATE_PX 0->1, ISOLATE_MAX_ASPECT 2.5->3.2. GT-FB IC5 validation: oblique (>=25 deg off-nadir) detect-ok 65% -> 79%, near-nadir 100% -> 100%. All env-revertible. |

### Verification (2026-08-28 kappa-ratchet campaign)

Ran the tune-plasmc single-knob-sweep methodology campaign IC1 -> IC2 -> ... -> IC5,
n=5 GT-FB each. The BASELINE PASS came back 25/25 SP with kappa_xy frozen at 0.50
and 0% sigma breach on every rep -> the kappa-ratchet DOES NOT OCCUR in the current
config -> NO knob sweep was needed. See project_20260828_kappa_ratchet_campaign.

---

## ⚠ WHAT IS NOT SOLVED -- REAL PERCEPTION (GT-FB OFF)

Guaranteed SP is a GT-FEEDBACK result (control isolated from perception). Under
real perception, cross-marker at 320x240 still fails:

- **IC1 centered: FAIL ~0.29 m / 1.2 m/s** -- diagonal-cal accuracy (the held-out
  check flagged h_x/h_y run ~1.15-1.35x hot; the cal is a ~5 m-regime fit).
- **IC5 (2,2,3): detection collapse** at the oblique-low engage viewpoint (~43 deg
  off-nadir, 3 m alt) -> 0% detect-ok post-engage -> TARGET_LOST. Partially helped
  by the 6b69c7b oblique retune (65 -> 79% on oblique frames) -- not a full fix.
- **off-center CONVERGENCE wall** (project_20260824_crossmarker_offcenter_convergence
  _wall): a SEPARATE phenomenon from the ratchet -- kappa LEAKS down and s_e_n
  stays 2-4 rad (won't converge), vs the ratchet where kappa runs UP. GT-verified
  as the same pre-existing ArUco kappa-leakage/funnel wall.
- **high-altitude cal gap** (> 6 m): the h-block scale is not constant with
  altitude; IC4 (7 m start) recorded twice, both weak hold-outs; runs archived to
  calibration_data/landing_cal_cross_highalt_excluded/.

A real-perception version of the kappa-ratchet campaign is in progress (2026-08-28).
Early: IC1 rep1 = FAIL at 100% detect-ok (control/cal, not a ratchet or a
detection issue). Expect the off-center ICs to hit the convergence wall, not the
ratchet.

## NEXT (real-perception path)
1. Tighten the 320x240 diagonal cal (more off-center landings; height-scheduled M).
2. Close the off-center kappa-LEAKAGE convergence wall (control-side, separate campaign).
3. Finish the oblique-low detector retune (79% -> higher) or accept IC5 needs a
   nadir-biased approach.
