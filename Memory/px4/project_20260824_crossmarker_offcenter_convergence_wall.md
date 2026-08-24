---
name: project_20260824_crossmarker_offcenter_convergence_wall
description: "Cross-marker perception-mode IC2 (off-center) shows a severe y-axis lateral non-convergence (xy_err 2.3-4.0m) — GT-verified as the SAME pre-existing ArUco kappa-leakage/funnel \"off-center wall\" (project_bake_and_sp_walls), not a cross-marker-specific bug; confirmed independent of the separate raw-detection-flicker issue"
metadata: 
  node_type: memory
  type: project
  originSessionId: db521dfb-a6f1-44ca-b3f6-432e98fc4866
  modified: 2026-08-24T05:09:35.520Z
---

Found 2026-08-24 while regression-checking the az-visibility-CBF + CBF_CORNERS_STALE-flicker-fix commits (fe071cc/36318e1/0431eab) on cross-marker. GT-verified per the diagnose-flight-data skill.

**Symptom:** `WORLD=cross_marker MARKER_TYPE=cross`, IC2 (off-center spawn, ENU 2,2,5), perception-mode (no GT-feedback): `xy_err` 2.3-4.0m across every rep tested — essentially missing the target, not a precision shortfall.

**Bisection: confirmed pre-existing, not a regression.** Checked out the commit immediately before today's three commits (`87b8896`, via an isolated `git worktree`, main working tree never touched) and re-ran the identical IC2 test: `xy_err` 2.27-2.40m there too. Both pre- and post-change are catastrophic failures of the same magnitude — the ~70% delta between them is noise at this failure scale, not a real regression signal. **This is the first time this session (and likely this project) that cross-marker perception-mode has been exercised at an off-center IC at all** — prior cross-marker work was almost entirely IC1-only or GT-feedback-based.

**GT-verified mechanism, one specific rep (IC2_rep1, test_data/ICValidation/20260823-211658/):**
- `ux` genuinely converges (1.94→-0.11m, even overshoots past zero — real x-axis authority) while `uy` stays flat the whole ~4.5s flight (1.99→2.44m, drifting *away*, never meaningfully reduced). Axis-specific, not a global failure.
- Controller's own `s_e_n[1]` tracks the same shape GT shows (dips to ~0.32, grows back to ~0.55) — **not a perception sign-bug**; the controller genuinely sees the y-error persisting.
- **`kappa[1]` *decreases* (0.50→0.27) while `s_e_n[1]` stays persistently large** — the adaptive-gain leakage term is dominating the growth term, meaning `sigma`/`zeta_r` (the funnel-transformed error the ODE actually integrates) reads as small even though the raw normalized error doesn't shrink.
- **`Detection Status` was `'ok'` for 450/450 frames this rep** — raw cross-marker detection was clean and continuous throughout. This wall is NOT caused by detection flicker/dropout; it's a pure control-law phenomenon.

**This matches an already-documented, still-unsolved ArUco mechanism — not a new bug.** `project_bake_and_sp_walls`: `"SP=2 INDEPENDENT walls: (1) PRECISION=off-center s_e_n never converges (ζ_r/g_r authority peaks S_r=0.649 then vanishes...)"` — identical symptom shape, and that entry's own "NEXT" note (`"stacked-barrier backstepping (only non-dead lever)"`) was never implemented. See also `feedback_kappa0_unfreezes_lateral`, `project_ic1_kappa_leakage_drift`.

**Checked whether the ArUco-validated levers transfer to cross-marker's config — they do, bit-for-bit, so there's nothing to port:**
- `P` (leakage), `kappa_0` (bootstrap), `gamma`/`XI2` (funnel stiffness), `chi_r` (velocity damping) are all computed through the same generic env-parsing helper in `controller.py`, with zero `MARKER_TYPE` branching anywhere.
- Camera intrinsics (`center`, `focal`, hence `p_10`/the funnel bound): `cross_marker_perception.py:50` does `from img_data import fx, fy` — imports the literal same constants, not a re-derived copy. Same `(240,320)` center convention (with a documented anti-regression comment against re-adding a `[::-1]` transpose bug already found/fixed once).

**Conclusion:** cross-marker is running the exact ArUco-validated funnel/kappa configuration and hitting the exact same never-fully-solved wall — this is a shared, pre-existing control-architecture gap independently rediscovered, not a cross-marker perception defect and not introduced by today's commits.

**Unconfirmed hypothesis, not yet tested:** cross-marker's noisier raw detection (see [[feedback_cross_marker_detection_flicker]]) *could* compound an already-marginal wall on OTHER reps/ICs even though it wasn't the cause in this specific 450/450-clean rep — would need `s_e_n` noise-statistics comparison between marker types on a matched IC to confirm.

**Not done:** implementing the only previously-identified further lever (stacked-barrier backstepping) for either marker type; checking whether this wall's severity varies across IC2/3/5 or is IC2-specific; extending the bisection/diagnosis to the other off-center ICs.
