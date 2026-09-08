---
name: project_20260908_visibility_cbf_simplification_audit
description: "Audit of the visibility-CBF stack for over-complication (2026-09-08). Verdict: the CERTIFIED CORE is small and fine; the accretion is the joint-QP + deliverability sphere + descent-rate relief layer (2026-08-29), which exists mainly to host a descent-rate-relief feature that has now failed in 3 forms and never won a SITL gate. Concrete simplification list, ranked by safety."
metadata:
  node_type: memory
  type: project
---

Investigation prompted by "did we overcomplicate the visibility CBF?" after the
`CBF_JQP_RELIEF_REF_AZ0` gate failure ([[project_20260908_jqp_relief_refaz0_gate_failed]]).
Evidence base: `src/cbf_visibility.py` (525 lines) + the controller call site
(`controller.py` ~3790-3990, ~4130) + fire-rate stats from the IC2-5 gate bundle
`test_data/JQP_ReliefRefAz0_AB/20260908-160523` default arm (20 reps, 9889 control
frames).

## The certified core is NOT over-complicated

`h_k = phi_max_k - |cr_k|`, coupling `L_w @ M`, one alternating-projection QP,
`th_safe -> rd3`. ~40 lines. Has the manuscript theorem. `validate_cbf.py`
tests 0-8 cover it. Leave it alone.

## Where the accretion is — fire rates on the LIVE cross-marker pipeline

| block | lines | runs | actually does work? |
|---|---|---|---|
| joint-QP (`CBF_JOINT_QP=1`, 2026-08-29) | ~152 (318-470) | 99.9% | the FoV box inside it: yes (~18% of frames). The rest: no |
| ...deliverability sphere (`CBF_SPHERE_TRUE_THRUST`) | in above | 99.9% | **binds 0.0%** (4/9889 frames; 81-97% of cap in normal flight) |
| ...descent-rate relief (`CBF_AZ_COST_GAIN=5`) | in above | fires ~17% | **net-negative** — deadlock-prone, 0 SITL wins in 3 implementations |
| ...per-iterate residual instrumentation (2026-09-08) | ~20 | 99.9% | diagnostic only; keep while the joint QP exists |
| theta-path (`CBF_JOINT_QP=0`) | ~20 (471-491) | **0.0%** | dead under default; kept for A/B |
| Phase-2 decode-fail (hysteresis/ramp/`m2_p2`/`theta_tight`/mag-clamp) | ~32 (493-524) | **0.1%** (10/9889) | near-dead on cross-marker; matters more on bad-perception days / ArUco |
| `CBF_HZ_AWARE_DRIFT` accelerating-drift extrapolation | ~8 (259-266) | 0% (default off) | never validated |
| `CBF_MARGIN_RESERVE` Phase-1 footprint reserve | 1 mul (238-239) | always x0 | gate-failed 2026-08-25, default 0 |
| controller `rho_fov`/`d_min_fov`/`theta_cone`/overflow/drift-off | ~90 (3790-3860) | feeds a 0.1% path + the drift pullback | `PLASMC_LFOV=0` freezes `rho_fov` at `rho_fov_0`; "deliberately dormant" |
| controller `PLASMC_AZ_JOINT` az-aware clip branch | ~25 (3964-3990) | dead under default (skipped when `CBF_JOINT_QP` on) | kept for `CBF_JOINT_QP=0` A/B only |
| controller `PLASMC_DTHETA_HREF` h_ref shaping | ~20 (2740-2759) | 0% (default off) | survivor of the removed `_dtheta_correction`; never validated |

## Key structural finding: the joint QP ~= the theta-path when the sphere doesn't bind

Algebra: `theta = P@(I_a[:2]/a_z)` and `M = Lw2@P/a_z`, so `M@I_a[:2] == Lw2@theta`
and the box constraint is identical in both parametrisations. With the relief off
(`CBF_AZ_COST_GAIN=0`) `a_z` is constant across the 6 outer iterates, so they collapse
to the theta-path's single projection loop. Empirically (3000 random cases, sphere
disabled via `A_CAP=1e6`, relief off): `worst |Δth_safe| = 0.064 rad`, median ~0 —
the residual difference is alternating-projection ITERATION ORDER (joint QP
re-evaluates `f` once per outer, theta-path once per row), not a better answer.

So the entire `CBF_JOINT_QP` addition — the 6x5 loop, the sphere, `CBF_SPHERE_TRUE_THRUST`,
`_az0`/`_relief_ref_az0`, the residual instrumentation, and the `PLASMC_AZ_JOINT` dead
branch in the controller — exists to host **(a)** an az-aware deliverability bound that
never binds and **(b)** a descent-rate relief that has failed as `_dtheta_correction`
(removed `e110b8a7`), `PLASMC_DTHETA_HREF` (obsolete, unvalidated), and
`CBF_AZ_COST_GAIN`+`REF_AZ0` (deadlock, gate-failed).

## Simplification list, ranked by safety (NONE done — needs user sign-off + the
## usual IC2-5 cross-marker SITL gate, since these touch the live control path)

1. **Kill the descent-rate relief** (`CBF_AZ_COST_GAIN -> 0` default, then delete the
   block + `_relief_ref_az0`/`_az0` + `PLASMC_DTHETA_HREF` h_ref shaping). Strongest
   case: 3 failed implementations, 0 SITL wins, actively harmful (deadlock). This is
   the "trade descent rate for lateral margin" idea — it has never worked; stop
   carrying it. GATE: A/B `CBF_AZ_COST_GAIN=5` vs `=0`, IC2-5 n=5. Expect `=0` to win
   or wash (the deadlock only bites `=5` reps, per the gate trace).
2. **Collapse the joint QP back to the theta-path** once (1) lands, keeping only a
   single post-solve `arccos(a_z/A_CAP)` clip for az-aware deliverability (already
   written, it's the `PLASMC_AZ_JOINT` branch — promote it, drop the flag). Removes
   ~150 lines + the sphere + `CBF_SPHERE_TRUE_THRUST` + the residual instrumentation
   + the dead theta-path/`PLASMC_AZ_JOINT` A/B forks. Net: one parametrisation, one
   projection loop, one deliverability clip. GATE: IC2-5 n=5 vs current.
3. **Bake the correctness flags, delete the `=0` forks:** `CBF_LW_ROT=1`,
   `CBF_RD3_DIRECT=1` are validated defaults; the `=0` paths are footguns (a whole
   `validate_cbf.py` test exists only to document `CBF_LW_ROT=0` is wrong). Keep the
   flags readable-but-fixed or delete outright.
4. **Retire the dormant legacy cone:** `PLASMC_LFOV`/`rho_fov_curr`/`d_min_fov` and
   the Phase-2 fallback cone. Only live consumer is the `_cbf_drift_off` ->
   `p_10_eff` pullback (`CBF_DRIFT_PULLBACK_FRAC`) — keep THAT (it's the one legacy
   path into the real barrier and it does fire on IC5), but it can classify
   drift-off directly off `p_10` and the measured corner spread without the frozen
   `rho_fov` scaffold. ~80 controller lines.
5. **Delete `CBF_HZ_AWARE_DRIFT`, `CBF_MARGIN_RESERVE`** — dormant / gate-failed,
   default-off/zero, no path to revival.

## What NOT to touch

- The certified core (barrier + `L_w@M` + the box projection + `th_safe->rd3`).
- The `_cbf_drift_off` -> `p_10_eff` pullback (fires on IC5, real effect).
- Phase-2's EXISTENCE (decode-fail must fall through to *something*) — but its
  internal ramp/hysteresis/`theta_tight` machinery could be a plain "hold last
  `th_safe`, shrink the cone linearly" and probably lose nothing measurable at 0.1%
  duty. Low priority.
- The camera-mount-yaw handling (`[y,-x]` swap / `Rz(+-90)` / `P = Rz_p90b@Rzm`) —
  necessary, error-prone, already caused 2 bugs; do NOT "simplify" it without
  re-deriving against `_getVirtualPts`.

## Methodological note

Every simplification above still needs an IC2-5 cross-marker SITL A/B — the same
gate that just caught `REF_AZ0`. "It's obviously dead code" has been wrong before
(`d_min_fov` looked dead, the drift pullback was live). Verify fire-rate on a fresh
bundle, then remove, then gate.
