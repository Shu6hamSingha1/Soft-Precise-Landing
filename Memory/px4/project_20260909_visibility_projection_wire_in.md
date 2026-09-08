---
name: project_20260909_visibility_projection_wire_in
description: "The visibility CBF was rebuilt from clean requirements as src/visibility_projection.py (Tier-1 hard lean projection on the measured cross-marker CENTRE in the real camera plane + Tier-2 soft self-releasing descent ease) and wired into controller.py (82fa9c16), retiring cbf_visibility.py / cbf_visibility_aruco.py / the joint-QP / deliverability-sphere / descent-relief / two-phase-delta / rho_fov-cone machinery. IC2-5 n=5 A/B vs the old machinery (worktree @ d380901c): PASS -- both 20/20 land / 0 TL; NEW pooled mean xy 0.14 vs OLD 0.24, precise 11 vs 8, max rel_vel 1.47 vs 1.95. No regression, tighter + softer, ~half the code."
metadata:
  node_type: memory
  type: project
---

## What replaced what

User directed a clean-slate rebuild from two requirements (2026-09-08/09):
1. keep the marker's measured CENTRE (cross-marker: the crossed-lines intersection,
   a single point -- NOT 4 corners) inside the REAL camera image plane with a fixed
   buffer from the FoV edge;
2. do it by the smallest change to the generated acceleration command.

**`src/visibility_projection.py`** (240 lines), two tiers on different timescales:

- **Tier 1 `visibility_project`** -- hard, this-cycle. Barrier `h_k = phi_k - |c_k|`,
  `phi = R/(2f)*(1 - buffer_frac)` (fixed camera constant). Jacobian
  `Le = -(Lw(c) @ M)` -- **sign is NEGATED vs the old `Lw@M`**; the old sign was
  backwards for a downward camera (verified against an independent pinhole+attitude
  oracle, `tools/validate_visibility_projection.py` check 1; the old `validate_cbf.py`
  oracle's own historical sign bugs had masked it). Alternating projection of the
  desired lean onto the two affine rows; only a predicted-breach row is touched ->
  minimal intervention, inward/tangential lean never clipped. `a_d[2]` is a FIXED
  input. Single-point linearisation is <8% off for a per-cycle lean CHANGE <=~15deg
  (the real 50 Hz regime); `buffer_frac=0.15` absorbs it. NOT accurate for a one-shot
  ~50deg correction -- relies on the lean cap + the every-cycle incremental nature.
- **Tier 2 `descent_ease`** -- soft, predictive, the user's "relaxing descent helps
  visibility" point done right. Scales ONLY the downward part of `a_z` by
  `g_z in [g_min, 1]` on the measured time-for-`|c|`-to-reach-the-edge
  (`t_min / t_react`). Unlike `CBF_AZ_COST_GAIN`: trigger is an EXTERNAL measured
  quantity (not a QP-internal norm that self-inflates), runs AFTER Tier 1 (never
  fights the lean solve), bounded SCALE (g_min>0 always descends, not a `min(.,-g)`
  that pins at hover), and SELF-RELEASES (the bought time shrinks `|c|` -> g_z->1).
  `CBF_DESCENT_EASE=0` disables.

Scale-free / depth-free: every input is a camera intrinsic, an attitude, an
image-space measurement, a ratio / pass-through of the caller's own command, or a
time constant. `t_edge = image-distance / image-speed = seconds` is the scale-free
"time to leave frame".

**`controller.py` wire-in (`82fa9c16`)**: the `cbf2_filter` call + the
`rho_fov`/`d_min_fov`/`theta_cone` cone + overflow/drift-off classification + the
`PLASMC_AZ_JOINT` block -> one `condition_for_visibility()` call. -289 net lines.
Kept: the freshness-gated marker-centre source (`cbf_corners` selection), the
z-upright guard, `CBF_LPF_BEFORE`, the downstream `|I_a|<=A_CAP` thrust cap + a lean
cap `arccos(a_z/A_CAP)` (deliverability stays controller-side). Drift-off pull-back
re-expressed as a per-axis `buffer_frac` bump on a persistent one-sided phi breach
(`CBF_DRIFT_PULLBACK_FRAC`). `CBF_OVERFLOW` is now permanently False (cross-marker
has no handover target). New env: `CBF_BUFFER_FRAC` (0.15), `CBF_DESCENT_EASE` (1),
`CBF_GMIN` (0.2), `CBF_TREACT` (1.5). New logs: `vis_gz(t)`, `vis_active(t)`.
Dropped logs: `rho_fov(t)`, `d_min_fov(t)`, `dtheta_az(t)`, `theta_desired(t)`,
`jqp_resid_*(t)`, `CBF Overflow Diag Log`.

**Removed -> `Obsolete/`**: `cbf_visibility.py`, `cbf_visibility_aruco.py`,
`validate_cbf.py`, `analyze_joint_qp_convergence.py`, `validate_thrust_sphere.py`,
`apps/cbf_isolation_test.py` (~2000 lines). New validator:
`tools/validate_visibility_projection.py` (10/10 vs an independent oracle).

## The gate -- PASS

`scripts/run_visproj_gate.sh` (`ac535a10`), IC2-5 n=5 interleaved, cross-marker
headless. NEW = main tree; OLD = `git worktree` at `d380901c` (`82fa9c16^`,
cbf_visibility joint-QP). `test_data/VisProjGate/20260909-005328/`.

| metric | NEW (visibility_projection) | OLD (cbf_visibility joint-QP) |
|---|---|---|
| land / TL | 20/20 / 0 | 20/20 / 0 |
| pooled mean xy (m) | **0.14** | 0.24 |
| pooled median xy | 0.09 | 0.12 |
| pooled max xy | 0.76 | 0.88 |
| pooled max rel_vel | 1.47 | 1.95 |
| precise | **11/20** | 8/20 |
| precise + soft | **15/20** | 12/20 |

Per-IC: NEW wins IC3 (mean 0.09 vs 0.23) and IC4 (0.18 vs 0.38); IC2 and IC5 a
wash. OLD's tail is worse (IC3r4 0.76/1.32, IC4r3 0.88/1.66/9s, IC4r4 0.68/1.95).
NEW's worst IC5r2 0.76/1.47. Smoke IC2 NEW-arm: precise xy=0.040, `vis_active` 0%
(pure minimal-intervention passthrough on a normal descent).

**Verdict: no regression in landing rate or target-loss; NEW is tighter and softer
at ~half the code.** buffer_frac=0.15 / descent-ease default-on stand.

## Gotchas

- **`run_visproj_gate.sh` driver bug (not a result bug)**: it logged every OLD-arm
  rep as "NO save" because BOTH arms' `landing_test.py` autosaved into the MAIN
  tree's `test_data/Landing_Test/` -- the worktree's `run_landing.sh` `cd $SCRIPT_DIR/..`
  did not redirect the save path, and `LANDING_OUT_BASE` was not set per arm. All 40
  recordings ARE present, recovered by timestamp-matching (see
  `test_data/VisProjGate/20260909-005328/RECOVERED_summary.md`). For any future
  two-tree A/B: set a distinct `LANDING_OUT_BASE` per arm.
- Worktree left at `~/Soft-Precise-Landing-old` (12 GB, `d380901c`) for a possible
  re-run; `git worktree remove` when done.
- Not yet done: moving-target `drift` wiring (Tier-1 `tau=0`/`drift=None` today --
  stationary is exact); an ArUco 4-corner path (module is cross-marker only, ArUco is
  comparison-only and its CBF is retired -- re-derive if ArUco numbers are ever needed).
