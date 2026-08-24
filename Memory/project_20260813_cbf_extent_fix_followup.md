---
name: project_20260813_cbf_extent_fix_followup
description: "Results of the 3 follow-up items from HANDOVER_cbf_extent_fix_and_nan_bug_20260813.md — ArUco regression check (passed), ring-sampling origin_ratio comparison (inconclusive n=1), and validate_cbf.py test-harness bugs found/partially fixed"
metadata: 
  node_type: memory
  type: project
  originSessionId: bc71d54e-36e7-454b-9b2e-33cc103d6bc0
  modified: 2026-08-13T20:05:45.784Z
---

Follow-up work on `HANDOVER_cbf_extent_fix_and_nan_bug_20260813.md`'s priority list (items 2-4;
item 1, the Hardware-side NaN fix, deliberately left to the user).

**Step 3 — ArUco regression check: PASSED.** Headless ArUco SITL flight (`WORLD=aruco` default,
`MARKER_TYPE` unset) landed SOFT+PRECISE (xy_err=0.011m, rel_vel=0.025 m/s). Confirms the
`MARKER_TYPE`-based CBF import routing change (`cbf_visibility.py` for cross-marker vs
`cbf_visibility_aruco.py` for ArUco, `src/controller.py` top-of-file) does not regress the live
ArUco pipeline.

**Step 4 — ring-sampling `origin_ratio` comparison: INCONCLUSIVE (n=1 each side).** Ran matched
IC2 cross-marker flights (`CROSS_RING_SAMPLING=0` vs `=1`, `WORLD=cross_marker MARKER_TYPE=cross`,
INITIAL_DRONE_ENU="2.0,2.0,5.0"), saved to `test_data/RingSampling_AB/ring0_IC2` /
`ring1_IC2`. Built `tools/compare_ring_sampling.py` — reuses the existing "Point Diag Log"
already saved in `Img_Data.npy` (via `CrossMarkerNode.getLogData()`,
`cross_marker_perception.py:1738`), no new instrumentation needed. Metric: per-frame
`origin_ratio = mean(|p-centroid|^2) / max(|centroid|^2, eps)` on the normalized flow points —
high = symmetric/well-distributed, low = clustered to one side.
- Both single-rep flights **failed the landing** (target_lost, xy_err 3.2–3.6m) — too noisy to
  draw a landing-outcome conclusion, consistent with [[feedback_sensitivity_sweep_methodology]]'s
  "n=1 is noise" rule.
- The symmetry metric itself showed no clean win: median origin_ratio barely moved (+0.004:
  0.296→0.300), while mean/p90 blew up under ring sampling (17.2→198.2, 54.2→278.9) with fewer
  solves overall (334→176) — outlier-dominated, not a clean improvement. Needs an n≥5 rerun
  before concluding either way.
- **Rover-world ring-sampling guard test: not run.** No cross-marker rover model exists in
  `~/PX4-Autopilot/Tools/simulation/gz/models/` (only `rover_aruco`), so `CROSS_RING_SAMPLING`
  (cross-marker-only path) has nothing to exercise there currently — would need a cross-marker
  rover model built first.

**Bonus finding — `tools/validate_cbf.py` had drifted out of sync with `cbf_visibility.py`,
in two independent ways, both predating this session:**
1. The 2026-08-13 cross-marker/ArUco split made `radius` a mandatory positional param
   (before `env`) in `cbf2_filter()`; the validator's 6 call sites were never updated, so it
   was either crashing (`test_phase2`) or silently passing the `env` dict into the `radius`
   slot, forcing every case onto the Phase-2 fallback (`ok=False`) — which made tests 2/3/5
   **vacuously pass on zero real samples** ("0 bound cases" was the tell). Patched all 6 call
   sites to pass `radius=0.0` explicitly (commit not yet made — still local edit as of
   2026-08-13 session end).
2. After that fix, tests 2 (barrier) and 5 (no-strangle) started genuinely failing (2.06 tangent
   overshoot, 96.5% inward shrink). Root-caused via a standalone check
   (`/tmp/.../verify_cbf_convention.py`, not committed — scratchpad only) that recomputes the
   barrier margin using `cbf2_filter`'s own internal camera-mount-yaw convention (the
   unconditional `[y,-x]` corner swap + `Rz(±90°)` accel↔tilt rotation added 2026-08-04): worst
   overshoot dropped to 4.4e-16 over 2717 cases. **Confirmed: this is a stale-test-oracle bug,
   not a live CBF bug** — `validate_cbf.py`'s helper functions (`project_tangent`,
   `Ia_from_tilt`, `R_from_image_tilt`, `th_curr_of`, `_inline_reference`) all predate the
   2026-08-04 camera-mount-yaw fix and were never updated to include it, so they check the QP's
   output against a 90°-rotated wrong "true feature."
- **RESOLVED same session:** propagated the camera-mount-yaw fix into `validate_cbf.py`'s
  helpers (`project_tangent`, `corners_from_tangent`, `Ia_from_tilt`, `th_curr_of` now
  image-frame/swap-convention; `R_from_image_tilt` switched to a finite-difference Newton
  Jacobian to avoid re-deriving analytic signs by hand; `_inline_reference` gained the swap +
  `radius`-based `delta2` + `Rz(±90°)` terms to match current `cbf_visibility.py`;
  `test_fixB_rd3` gained the missing `Rz(-90°)` factor matching `controller.py:2924-2930`'s
  real Fix-B path). `test_lw_fidelity` (test 1) deliberately kept on LOCAL body-FRD-only
  helpers (`_proj_body`/`_th_curr_body` inside the test function) since it never calls
  `cbf2_filter` and is a body-frame-native physics check (L_omega interaction-matrix
  fidelity, unrelated to the camera-mount fix) — mixing conventions there produced a bogus
  ~200% "error" that was a test-frame mismatch, not a modeling error.
  **Result: 12/12 checks pass, with real (non-vacuous) sample counts** — test 3's "0 bound
  cases" became "30 bound cases", tests 2/5 now exercise genuine QP output instead of the
  Phase-2-fallback-only vacuous pass. `tools/validate_cbf.py` is trustworthy as a regression
  gate again. Not yet committed as of session end — local working-tree edit only.

**Step 2 — n=5 IC1-5 sweep with the CBF extent fix: DOES NOT REPRODUCE the n=1 smoke-test
result. ⛔ Standing correction to the handover's optimism.** Ran `WORLD=cross_marker
MARKER_TYPE=cross HEADLESS=1 N_REPS=5 bash scripts/run_ic_validation.sh` (25 flights,
`run_logs/cbf_extent_fix_sweep_ic1to5_n5_20260813_214129.log`). Result: **0/25 precise, 1/25
soft** (IC1 rep2 only). Per-IC mean xy_err: IC1=0.495m (1 soft), IC2=2.887m, IC3=7.724m (max
28.67m outlier, rep5), IC4=4.780m (max 14.75m outlier, rep4), IC5=3.100m. All 25 flights
touched down (`landed=YES`) but with large terminal position error on every off-center IC,
including IC2 — the exact case the handover's n=1 smoke test reported as improved (77.7%→
95.6-100% detect rate). **Better detection did not translate to better landing precision at
n≥5** — the IC3/IC4 outliers (28.7m, 14.8m) look like a fly-away/drift-off failure mode
distinct from the detection-collapse the extent fix targeted. This directly falsifies treating
the extent fix as "validated, just needs a bigger sweep to confirm" — it needs root-cause
diagnosis of what's driving the large terminal xy error (likely NOT the extent-blindness bug,
since that's specifically fixed and confirmed via the offline validator above; look elsewhere
— possibly the still-flagged-open items from
[[project_20260812_cross_marker_flow_architecture_investigation]] §3b/§3c, or something new).
**ROOT-CAUSED same session (via `diagnose-flight-data` discipline — GT poses, verified
timestamp alignment gt_t=gt['Time']+gt['Start Time'] vs img['Time'], NOT the controller's own
h(t)/reference):** the near-touchdown detection collapse is a LATERAL POSITION-CONVERGENCE-RATE
deficiency for off-center ICs (IC2-5), not a marker-size/CBF problem. Evidence (IC2_rep3):
xy_err shrinks only slowly during descent (2.77m@t=0 -> 2.30m@t=2.88s, minimum) while altitude
drops fast (5.02m->3.40m over the same window) -- by the time xy_err bottoms out, the LOS angle
atan(xy_err/z) alone already consumes 68% of the camera's 0.87 rad half-FOV budget. At that
EXACT point, xy_err reverses and starts GROWING again, coinciding with the first persistent
detection-miss run. Cross-checked across 4 failing reps (position-angle + marker-half-extent-
angle vs the 0.87 rad budget at the moment of loss): IC2_rep3 1.068, IC2_rep4 1.052, IC3_rep5
1.314, IC4_rep4 1.330 -- all over budget, and in every case POSITION ANGLE ALONE is already
62-100% of the budget (0.629-0.881 rad) before marker size is even added. **The extent fix
can't prevent this because it's deliberately soft/reactive** (Phase-1 stays centroid-only;
radius only feeds the Phase-2 ramp + drift-off pullback, both gated behind several consecutive
decode-fails) -- by the time those engage, the FOV margin is already consumed by position
error. Matches the project's pre-existing "lateral wall" issue class
([[feedback_lateral_kappa_runaway]], [[feedback_lateral_wall_anti_restoring_au]]) previously
characterized on the ArUco pipeline -- this sweep is the first evidence it reproduces on the
cross-marker pipeline too. **CONFIRMED the CBF itself is NOT the cause (user asked explicitly,
checked directly):** compared `I_a_raw(t)` (pre-CBF desired accel) vs `I_a(t)` (post-CBF)
alignment with the true target-ward direction (from GT) through the IC2_rep3 stall window. The
RAW command itself degrades from well-aligned (dot=0.95 @t=1.8s) to pointing AWAY from the
target (dot=-0.58 @t=2.8s) well BEFORE the marker leaves frame (~3.09s) and before the CBF binds
meaningfully (`d_min_fov` does hit 0 in this window, but the CBF-filtered `I_a(t)` stays MORE
aligned with the target than the raw command throughout -- e.g. 0.29 vs -0.09 @t=2.4s --
consistent with its designed no-strangle property, not evidence of throttling a needed
correction). **The root cause is upstream, in the outer/middle PLASMC control loop's own
reference tracking** (likely perception-signal degradation as range/geometry gets marginal,
feeding a bad desired-accel direction into the SMC/adaptive-gain law) -- NOT the visibility CBF,
and not necessarily the ArUco-side "lateral wall" gain-parity issue (that's still worth checking
since the SYMPTOM matches, but this is a different, more specific finding). **Next step: trace
WHY `I_a_raw`'s direction degrades starting ~t=2.0s in IC2_rep3 (and presumably similarly in
the other failing reps) -- compare the perceived `s(t)`/`h(t)` in Control_Data against
independently-computed GT flow (`tools/gt_optical_flow.py`, Z_REG-regularized) over that exact
window to see whether a perception error is driving the bad reference, before assuming a
controller-gain problem.**

**Step 2 RERUN (2026-08-14) with commit `27410ed` (radius split, see
[[project_20260814_cbf_radius_split_and_hardware_check]]): fly-away outliers ELIMINATED,
overall precision unchanged.** Same n=5 IC1-5 sweep
(`run_logs/cbf_radius_split_sweep_ic1to5_n5_20260814_010909.log`,
`test_data/ICValidation/<20260814 bundle>/`). IC3 mean/max xy_err: 7.724/28.673 ->
**2.091/2.877**. IC4: 4.780/14.752 -> **2.431/3.430**. IC1/IC2/IC5 roughly unchanged
(IC1 0.495/1.202->0.794/2.091, IC2 2.887/4.066->3.246/4.228, IC5 3.100/4.178->2.241/3.497).
Still 0/25 precise, 1/25 soft (same single IC1 rep both times). **Confirms the radius-split
fix (OVERFLOW-misclassification guard) works exactly as designed** -- it eliminates the
catastrophic fly-away failure mode specifically, but does NOT touch the separately-root-caused
lateral-convergence-rate deficiency (see above), which remains the dominant gate on overall
landing precision. Both fixes are real and validated; neither alone gets the cross-marker
pipeline to precise/soft landings yet.

See also [[project_20260812_cross_marker_flow_architecture_investigation]] §3d-3g for the
original CBF extent fix + NaN bug this session followed up on.
