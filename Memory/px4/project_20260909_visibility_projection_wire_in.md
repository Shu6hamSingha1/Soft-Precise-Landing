---
name: project_20260909_visibility_projection_wire_in
description: "The visibility CBF was rebuilt from clean requirements as src/visibility_projection.py (Tier-1 = a discrete-time one-step CBF-QP: min-norm lean projection keeping the measured cross-marker CENTRE inside the buffered FoV on the real camera plane; Tier-2 = a soft, non-CBF, self-releasing descent-ease governor) and wired into controller.py (82fa9c16), retiring cbf_visibility.py / cbf_visibility_aruco.py / the joint-QP / deliverability-sphere / descent-relief / two-phase-delta / rho_fov-cone machinery. IC2-5 n=5 A/B vs the old machinery (worktree @ d380901c): PASS -- both 20/20 land / 0 TL; NEW pooled mean xy 0.14 vs OLD 0.24, precise 11 vs 8. WHY NEW>OLD (traced from the bundle): median is a WASH (CBF idle on clean approaches); NEW wins the TAIL because the old joint-solve/relief/pullback/theta_cone-floor stack THRASHED on marginal off-center approaches -- intervened when it shouldn't (OLD dtheta_az fired 4-5% of terminal frames vs NEW vis_active 0%; fought the SMC re-centering), chattered theta_cone 3-5x into the kappa ratchet (a_u 32-55 vs 5-6, kappa 0.5 vs 0.1), added descent-slowing pressure. All OLD tail failures end with s_e_n~0.9-1.0 (the unresolved off-center wall); NEW doesn't fix that wall, it stops the CBF from worsening it (NEW still hits it once, IC5r2)."
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

  **Tier 1 IS a control barrier function** -- a discrete-time, one-step-horizon
  CBF-QP: explicit barrier `h_k` defining a safe set (centroid inside the buffered
  FoV); the QP constraint `|c + Le(y - y_now) + tau*d|_k <= phi_k` enforces
  `h_k(c_next) >= 0` on the predicted feature (the DTCBF condition
  `h(x_{k+1}) >= gamma*h(x_k)` at `gamma=0`, hard invariance, plus a `tau*d`
  look-ahead for target motion); min-norm objective = minimal intervention.
  Caveats vs a *rigorous* CBF: the invariance rests on the one-step linearisation
  (holds for the <=~15deg/cycle regime, buffer_frac is the margin -- no proven
  Lipschitz/exact-map bound); no `alpha(h)` rate relaxation (uses the hard
  `h(c_next)>=0`, more conservative near the edge, re-solved every 20 ms); no
  stability/feasibility theorem is claimed (the retired note claimed a disputed
  one). **Tier 2 `descent_ease` is NOT a CBF** -- a soft bounded self-releasing
  governor on the descent rate, no barrier, no invariance claim.
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

## WHY NEW beat OLD — mechanism (traced 2026-09-09 from the gate bundle)

The pooled median is nearly identical (0.09 vs 0.12) — on a clean approach NEITHER
CBF does anything (NEW `vis_active`=0%, OLD `dtheta_az`~0). **The entire difference
is in the TAIL**: OLD had 3 bad reps (IC3r4 0.76/1.32, IC4r3 0.88/1.66/9s,
IC4r4 0.68/1.95), NEW had 1 milder one (IC5r2 0.76/1.47).

Terminal-window (last 30%) trace of the 3 bad OLD reps vs their matched NEW reps:

| rep | arm | interv | θcone chatter (mean\|Δθ\|) | reliefT | a_u maxT | κ maxT | s_e_n end |
|---|---|---|---|---|---|---|---|
| IC3r4 | OLD | 0.04 | **0.047** | 0.09 | **55** | 0.5 | **0.99** |
| IC3r4 | NEW | 0.00 | 0.015 | 0.03 | 6 | 0.1 | 0.10 |
| IC4r4 | OLD | 0.05 | **0.056** | 0.08 | **32** | 0.5 | **0.90** |
| IC4r4 | NEW | 0.00 | 0.012 | 0.03 | 5 | 0.1 | 0.09 |
| IC4r3 | OLD | 0.00 | 0.004 | 0.00 | 1 | 0.2 | 0.24 (**alt +3.98 m, 8 s flight — never descended**) |
| IC4r3 | NEW | 0.00 | 0.017 | 0.02 | 10 | 0.5 | 0.11 |

**All the OLD tail failures end with `s_e_n ≈ 0.9-1.0`** (marker centroid pinned at
the FoV edge) + `a_u` 30-55 + `κ` ratcheted to 0.5. That is the known, unresolved
off-center lateral-convergence wall ([[project_20260824_crossmarker_offcenter_convergence_wall]],
[[project_20260903_controller_population_analysis]]'s "control-bound residual, cause
NOT identified"). **Neither CBF causes it and neither fixes it** — NEW still hits it
once (IC5r2: NEW `s_e_n` end 0.95, `a_u` 77, OLD centered that rep). The rebuild did
NOT solve the wall; it stopped the CBF from making it worse.

**Three ways the OLD machinery was actively counterproductive on marginal-but-
recoverable off-center approaches — exactly what IC2-5 stress:**

1. **It intervened when it shouldn't.** OLD's `dtheta_az` fired 4-5% of terminal
   frames in IC3r4 / IC4r4; NEW's `vis_active` was **0%** in all three. The OLD
   joint QP + drift-off `p_10_eff` pullback + θ_cone floor modify the lateral
   command the SMC is using to re-centre, and that modification is not cleanly
   outward-only. NEW's alternating projection is provably minimal-intervention +
   outward/tangential-free (validator checks 2, 3), so on a marginal approach it
   stays out of the SMC's way and the vehicle converges (`s_e_n` end 0.10 vs 0.99).
2. **θ_cone chatter 3-5×** (OLD 0.047-0.056 vs NEW 0.012-0.017 mean frame-to-frame).
   The 6×5 joint solve with `a_z` in the loop does not converge frame-to-frame near
   touchdown ([[project_joint_qp_nonconvergence_kappa_ratchet]]) → feeds `κ` a
   non-settling `σ` → κ ratchets 0.5 vs 0.1 → `a_u` amplifies 32-55 vs 5-6. NEW's
   single fixed-`a_z` alternating projection gives a smooth θ_cone, so κ settles.
3. **Descent-relief terminal pressure.** OLD `reliefT` 0.08-0.09 vs NEW 0.02-0.03 in
   the tail reps — not a full deadlock here, but the residual of the deadlock-prone
   `CBF_AZ_COST_GAIN` mechanism, always in the "slow the descent while off-centre"
   direction that prolongs edge exposure.

**Bottom line:** the median wash confirms the two are equivalent when the CBF is
idle (the common case). NEW wins the tail because the joint solve / relief / pullback
/ θ_cone-floor stack THRASHED on exactly the tight off-centre approaches the gate
targets — fighting the re-centring, chattering θ_cone into the κ ratchet, adding
descent-slowing pressure — while the minimal outward-only projection got out of the
way. Simplicity here is not just fewer lines; the removed machinery was net-negative
on the hardest cases. (IC4r3 OLD's 8 s / 4 m-stuck flight is a separate descent /
acquisition failure, plausibly SITL variance in a back-to-back interleaved run, not
a traced CBF effect — but it counts in the tally.)

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

---

## UPDATE 2026-09-09 (commit `e63751e2`): Tier-1 QP + deliverability + slack + h-lead

User pushed five refinement ideas; outcome:

| idea | verdict |
|---|---|
| 1. fold deliverability into the feasible set | **DONE** |
| 2. exact ray-rotation map / SCP instead of first-order | **skip** — per-cycle lean change is ~1°/cycle p99 (gate logs), worst spike ~11°, all inside the ~15° envelope + buffer; naive successive re-linearisation was already tried in the module build and *regressed* the barrier test |
| 3. softened CBF with slack | **DONE** (with 1) |
| 4. joint lean+descent optimisation | **held** — `vis_gz<1` fires ~35-64% of gate frames (frequent) BUT gently (`gz_min` 0.76-0.93) and outcomes are clean; re-coupling reintroduces the `a_z`-solved-variable structure we deleted (retired joint-QP). Cheap lever first: `t_react`/`buffer_frac` sweep. |
| 5. state-dependent horizon tau | **partial** — the inward-overshoot clamp (below) is a mild state-dependence; full `tau_k = f(closing rate)` folded into the rover-phase work (stationary centre is quasi-static, no benefit there) |

### What changed
- **Tier 1 is now a convex QP**, not alternating half-plane projection:
  `min ½‖y−y_d‖² + ½ρ‖s‖²  s.t. |c_next(y)|_k ≤ φ_k + s_k,  ‖y‖ ≤ y_max,  s ≥ 0`
  - `y_max = √(A_cap²/a_z² − 1)` — the thrust ball, identical to the `arccos(a_z/A_CAP)`
    lean cap in tangent form. Folded in ⇒ returned `I_a` is actuator-feasible **by
    construction**; the post-hoc lean-cap scale-back in `controller.py` is demoted to a
    redundant guard (still covers the raw path + the degenerate `A_CAP ≤ g` branch).
  - Per-axis slack `s ≥ 0`, penalty `CBF_VIS_RHO` (default 2000): visibility stays
    effectively hard when achievable within the ball; **graceful degradation** (never
    infeasible / no blow-up) when box and ball are disjoint (near-saturated hover +
    marker far off-centre). `info["deliverable"]`, `info["slack"]` exposed; logged as
    `vis_slack(t)`.
  - **Solver:** slack eliminated in closed form (`s_k = max(|c_k|−φ_k, 0)`) → smooth
    strictly-convex 2-D objective; projected Newton (constant Hessian per active set,
    exact interior optimum) + a 1-D circle refine when the ball binds. A few 2×2 ops.
  - Minimal-intervention / inward-free / idempotent behaviour is preserved — validator
    checks 2/3/6 still pass, and check 1 (barrier) is unchanged (QP reproduces the hard
    projection in the normal regime).
- **Moving-target lead** (`τ·d` term), previously stubbed:
  - `d` is sourced from the **pipeline's translational optic flow `h_xy`**
    (`self._h[-1][:2]`) — already de-rotated (`L_ω·ω` solved out), `_sensor_cal_hw`-
    calibrated, savgol/KF-smoothed. Chosen over a bespoke `Δc/dt − L_e·Δy_now/dt`
    estimator (user's call): cleaner SNR, and it's the same `h` the middle SMC uses.
    `img_data.py:~1044` confirms `h_x = ṡ_x + x0·h_z + (L_w·w)_x` with the rotational
    term removed = exactly `d`.
  - Mapped to the module tangent frame with `_SWAP` (the same swap `marker_tangent`
    applies). **Sign of the `h_xy`→module-frame map still needs one rover recording to
    confirm** (used the analytically-consistent `_SWAP`; flagged in-code). Optional
    `CBF_DRIFT_LOOM_STRIP` removes the `c·h_z` descent-scale term (default OFF pending
    the same recording).
  - Flow-validity gated (`_observer_valid` / not `_last_drifted_off`) → `d = 0`
    fallback (= stationary behaviour, never noise).
  - **Inward-overshoot clamp** in the module: a linear extrapolation `c + τ·d` is never
    allowed to predict the centre crossing an axis origin (past first-order validity;
    would ask for a reversed correction). This is what makes a fast target not trigger
    spurious projection (validator check 13).
  - **`CBF_DRIFT_TAU=0` default ⇒ the whole path is inert; stationary behaviour is
    byte-identical.** `vis_drift(t)` logged.

### Validation
- `validate_visibility_projection.py`: +4 checks (10 deliverable-by-construction /
  11 graceful degradation / 12 moving-target lead recovers would-be-lost targets /
  13 no spurious inward trigger). **14/14 across 5 seeds.** Solver provably within a
  few % of grid-optimal in the disjoint corner (0 violations across the whole suite),
  exact in the interior.
- HEADLESS smoke test (IC2, NEW arm): clean land `xy=0.028 m` / `rel_vel=0.12 m/s`,
  42 Hz, all new log fields present, `vis_slack`/`vis_drift`/`vis_active` all 0 on the
  clean approach (as expected).
- **IC2–5 stationary A/B gate RUNNING**: `test_data/VisProjQPGate/20260909-142528`,
  NEW (QP, HEAD) vs OLD (`e63751e2^` = ccc41071, pre-QP). Harness
  `scripts/run_visproj_qp_gate.sh` — this one sets `LANDING_OUT_BASE` **per arm**
  (fixes the `run_visproj_gate.sh` autosave-collision bug). Result pending.

### Files
- `src/visibility_projection.py` — `_solve_tier1` added; `visibility_project` +
  `a_cap`/`rho` args; `condition_for_visibility` passes them through.
- `src/controller.py` — CBF call site: `a_cap=A_CAP`, `rho`, `tau=_tau`, `drift=_drift`
  (h-sourced, gated); `_vis_slack_log` / `_vis_drift_log`; params `CBF_VIS_RHO` /
  `CBF_DRIFT_TAU` / `CBF_DRIFT_LOOM_STRIP` in both dicts; `vis_slack(t)` /
  `vis_drift(t)` in the log dict. Peer (`soft-precise-landing-fc`/`-29`) yaw + loom
  work is on other lines — no overlap.
- `docs/CBF_visibility.tex`/`.pdf` — rewritten §Tier 1 (eq:qp now has slack + ball),
  §Deliverability (folded in), Prop inv (s*=0 condition, always-solvable), Solver
  (projected Newton + circle refine), §Feature-response + Assumptions (h-sourced d).
  6 pp.
- `docs/CONTROL_FRAMEWORK_REVIEW.md` §4C — `e63751e2` sub-bullet.
- Backups: `Obsolete/{src,tools}/*_v1_pre_qp_slack.py`.

### Still open
- IC2–5 QP gate result (running).
- `h_xy`→module-frame sign confirmation from a rover recording.
- `CBF_DRIFT_TAU` sweep + rover re-gate — behind the upstream perception blockers
  (oblique-view detector collapse, terminal-overfill loom); a wired lead won't land
  the rover until those do.
- Idea 4 + `t_react`/`buffer_frac` descent-ease knob sweep.
- OLD worktree at `~/Soft-Precise-Landing-old` now at `ccc41071` — `git worktree
  remove` when the QP gate is done.

---

## UPDATE 2026-09-09 (cont.): QP gate result + h-sign fix + rover status

### IC2-5 stationary QP A/B gate — `test_data/VisProjQPGate/20260909-142528`
NEW (QP, HEAD) vs OLD (`e63751e2^`=ccc41071, pre-QP alternating projection). n=5/cell.

| pool | land | TL | mean xy | med | max | precise | P+S |
|---|---|---|---|---|---|---|---|
| NEW (QP) | 20/20 | 0 | 0.07 | 0.06 | 0.24 | 15 | 16 |
| OLD | 20/20 | 0 | 0.06 | 0.05 | 0.21 | 17 | 18 |

**Verdict: statistical wash, PASSES the reject bar** (no failed landing either arm).
IC3 exact tie; IC5 (the only IC where `vis_active` genuinely fires, ~10% of terminal
frames, BOTH arms) near-identical -> QP and alternating projection behave the same
when active. `vis_slack` fired on exactly one rep (IC2/new5): 5 frames in the last
0.4 s at ~0.1 m alt with the vehicle 0.24 m off-centre (the known terminal-overfill
wall) -> slack absorbed it (max 0.188), landing completed 0.24 m soft. The thrust
ball was NOT binding there; it was the visibility constraint itself unsatisfiable in
that terminal geometry. Graceful degradation did its job.
The 2-rep P+S gap is within n=5 SITL noise. **QP is functionally equivalent on
stationary; the structural wins (deliverability-by-construction, graceful slack,
moving-target infra) come free.** -> BAKED as the standing Tier-1 (no runtime flag;
unconditional since e63751e2; rollback = `Obsolete/src/visibility_projection_v1_pre_qp_slack.py`).

### h_xy -> module-frame map: IDENTITY (not _SWAP) -- commit `28e4417b`
Regressed `d(vis_c)/dt` on the pipeline's `h(t)[:2]` over a real approach
(`test_data/_hsign`, one HEADLESS rep with the new `vis_c(t)` log):

| candidate map | median cos vs d(c)/dt |
|---|---|
| **+I** | **+0.87** |
| -I | -0.87 |
| +/-SWAP, SWAP^T, diag(±1,∓1) | ~0 |

So `h_xy` is ALREADY in the module tangent frame -- the perception front-end applies
the camera-mount swap upstream (consistent with the module doc's "_SWAP matches
_getVirtualPts"). The wire-in's `_drift = _SWAP @ h_xy` was WRONG; fixed to
`_drift = h_xy`. LS scale ~0.45 (rotational term not removed in the check + h's own
cal), so `CBF_DRIFT_TAU` absorbs residual scale as well as being the lead horizon.
Added `vis_c(t)` telemetry permanently.

### Rover: what "working CBF for rover" still needs
The CBF module + wiring are now CORRECT for a moving target (QP + slack + validated
`tau*d` lead + correct frame map). Outstanding:
1. **`CBF_DRIFT_TAU` value** -- needs a rover A/B (`0` vs ~`0.3-0.5`). Not baked;
   env-gated; the rover launcher does NOT set it yet (kept out of the control path
   per the no-rover-conditionals rule).
2. **Rover SITL gate** -- BLOCKED on perception: rover doesn't complete a landing
   today (oblique-view detector collapse from ~5 m + terminal-overfill loom collapse
   ~1.1 m, per STATUS block / [[project_20260901_moving_rover_landing]]). A wired
   moving-target lead cannot land the rover until those are fixed. A scoped A/B
   ("does `tau>0` keep the marker in frame longer / delay `_last_drifted_off` /
   reduce `vis_active`") is possible on runs that don't complete -- that measures the
   CBF's job without needing a successful landing.
3. Optional: Tier-2 `c_rate` gyro-strip (currently raw finite-diff of the smoothed
   `c`); refinement, not a blocker.

---

## UPDATE 2026-09-09 (cont.): rover CBF sweep — all 7 motion profiles

`test_data/RoverCBFSweep/20260909-163929` — 7 `ROVER_TRAJ` (Static/Linear/Circular/
EightShape/Sinusoidal/Lissajous/CircularYaw) × A/B `CBF_DRIFT_TAU` 0 vs 0.4 × n=2,
cross-marker rover, `ROVER_MOTION=1`, HEADLESS. Analyzer `tools/analyze_rover_cbf_sweep.py`.
Judged on CBF trigger correctness + moving-target-lead behaviour, NOT SP (user directive).
Ran with `CROSS_LOOM_INNOV_GATE=1` (peer bake `7e9843ae`) consistently — shared by
both arms, gates h_z only, does not touch the CBF's h_xy lead or trigger metrics.

### CBF machinery — TRIGGERS CORRECTLY on all 7 profiles
- `off` arm: `|d|` identically 0 (correct — `CBF_DRIFT_TAU=0` → lead inert).
- `lead` arm: `|d|` live every frame on every moving profile (`p50` ~0.05–0.21
  tangent/s = plausible real drift). `vis_active` fires on every cell (5–17% of
  frames). QP runs. **`_last_drifted_off` NEVER logged (0/2 every cell)** — the
  marker never left the FoV off-centre. Marker in-frame ~99.7–100%, last-seen
  ~99.4–99.9% both arms. The wired mechanism does what it should.

### The `τ=0.4` lead is NOT usable as-is — `h_xy` is too noisy raw
1. **Static rover (true target drift ≈ 0): the lead injects noise.** `|d|p95` up
   to 1.4, `|d|max` up to 2.5 — all self-motion flow + `h_xy` sensor noise, no real
   target motion to lead. `maxC/φ` went **0.59 → 1.49** (Static/off/2 vs lead/2):
   the lead pushed the marker OUT of the buffered box on a static target. It should
   be a no-op there.
2. **Moving profiles: huge `h_xy` transient spikes pass the validity gate.**
   `|d|max` = 9.9 (EightShape), **16.4 (Lissajous)**, 4.3 (Linear), 3.4 (CircularYaw).
   `τ·|d|` = 0.4×16.4 = 6.6 tangent predicted lead — clamped by the inward-overshoot
   guard but still a massive one-frame Tier-1 perturbation. The `_flow_ok` gate
   (`_observer_valid` / not `_last_drifted_off`) does NOT catch these.
3. **TL correlation (weak, tiny n):** the only 3 TL events in the sweep are all
   `lead` reps (Circular/lead/1, Lissajous/lead/1&2); 0 `off` reps TL'd. Base state
   is "everything fails anyway" so this is suggestive, not conclusive.
4. `maxC/φ` improvement is inconsistent: lead helps Circular (1.24→0.81) and
   EightShape (0.99→0.69), hurts Static (0.88→1.23) and Lissajous (1.08→1.39).

### Can't judge flight/landing quality
Nearly every rep on BOTH arms ends `crash/flyoff/timeout` — the rover approach is
not stable (perception-blocked upstream). Flight durations are bimodal SITL noise
(Linear/off: 7.3 s and 2.0 s; EightShape/off: 2.7 s and 18.7 s), so the aggregate
"lead flights shorter" is not attributable to the lead.

### Verdict / next
- **CBF machinery: works for moving targets.** `d` from `h_xy` is live, QP triggers,
  no drift-off, marker stays in frame. Stationary QP is baked and equivalent.
- **Moving-target LEAD: needs `d` conditioning before it can be defaulted.**
  Required: (a) hard magnitude clamp on `d` (~0.5 tangent/s — real drift `p50`~0.1),
  (b) light LPF / median filter on `d`, (c) tighter flow-validity gate (the spikes
  slip through `_observer_valid`), and/or (d) much smaller `τ` (0.1–0.2). Then
  re-sweep. **`CBF_DRIFT_TAU=0` stays the default.**
  - **Ready-made gate input (peer, 2026-09-09):** `_solve_jacobian` already logs
    `rel_resid = ||A@sol−b||/||b||` + `cond(A)` per flow solve — a scenario-agnostic
    "trust this whole h vector this frame" measure, in the flow diag log. Gate `d`
    (and possibly Tier-2's `c_rate`) on `rel_resid` high / `cond` bad. Same signal
    could become a 3rd condition on the loom gate. See `project_20260908_line_width_loom_investigation`.
- A clean "does the lead help" A/B is not possible until the rover approach itself
  survives past ~10 s consistently — that's the perception thread
  ([[project_20260901_moving_rover_landing]]).
- Harness: `scripts/run_rover_cbf_sweep.sh` + `tools/analyze_rover_cbf_sweep.py`.

### `condition_drift()` — BUILT (commit `e1b094e8`)
Caller-side conditioning of `h_xy` before it is passed as the Tier-1 lead `d`:
`rel_resid` gate (`self._img_node._bgflow_health[0]` > `CBF_DRIFT_RESID_GATE`=0.45,
the perception layer's own threshold → feed 0) → median-of-3 (kills isolated
single-frame spikes) → 1-pole LPF (`CBF_DRIFT_LPF_ALPHA`=0.12 ≈ τ 0.15 s @ 50 Hz)
→ radial clamp (`CBF_DRIFT_MAX`=0.5 tangent/s, direction preserved).
Replayed the sweep's 14 lead-arm `h(t)` traces through it: `|d|max` 16.4 / 9.9 /
4.3 → capped at 0.5 on every rep, `|d|p50` unchanged (real slow drift passes), only
0–5 frames/rep hit the clamp. Validator +check 14 (gate / clamp / spike-reject /
tracks a slow drift); **15/15**.
**Still `CBF_DRIFT_TAU=0` default.** Next SITL step: re-sweep with the conditioning
+ a smaller `τ` (~0.15) so Static-rover residual noise (`|d|p50`~0.05) →
`τ·|d|`~0.007 tangent (inert). Gated on the rover approach being stable enough to
judge (perception thread).

### Rover CBF RE-SWEEP with condition_drift + τ=0.15 — `test_data/RoverCBFSweep/20260909-182607`
Same 7 profiles × A/B `CBF_DRIFT_TAU` 0 vs **0.15** (down from 0.4) × n=2, WITH
`condition_drift` active (commit `e1b094e8`).

**`condition_drift` works in-loop — decisive:**
- `vis_drift` `|d|max` **capped at exactly 0.5** on every profile (vs raw sweep's
  9.9 / 16.4 / 4.3 / 3.4). `|d|p50` 0.05–0.21 preserved. Only 0–8 frames/rep hit
  the clamp (median-3 + LPF handle the rest first).

**`τ=0.15` + conditioning is SAFE — no regression vs `τ=0`:**
- **Static** (stationary rover): first sweep `maxC/φ` lead 1.23–1.49 (lead pushed
  marker OUT of box). Re-sweep: off 1.25 / **lead 0.73** — the lead arm is now no
  worse (slightly better; n=2 noise). `τ·|d|` ≈ 0.15×0.11 ≈ 0.017 tangent = inert.
- **No TL events** anywhere this sweep (raw sweep had 3, all lead).
- Slack: `slk_max` CircularYaw off **3.0** vs lead 0.43; Lissajous off 0.51 vs lead
  0.17 — the conditioned lead arm has LOWER peak slack than off on the noisy
  profiles (opposite of the raw sweep). Conditioning helped.
- `drift_off` 0/2 every cell both arms; marker in-frame ~100%.

**"Does the moving-target lead improve rover visibility" — STILL UNANSWERABLE.**
Flight durations bimodal 2–20 s on BOTH arms (Linear/off 1.9 s, EightShape/off
2.8 s, Sinusoidal/lead 2.5 s — never descended). When a run dies at 2–3 s the CBF
metrics are meaningless (`t1stAct=nan`, `maxC` tiny). `maxC/φ` off-vs-lead deltas
track flight duration (longer flight → marker gets closer to the edge) more than
the lead. Same wall: the rover approach must survive a stable window first —
perception thread ([[project_20260901_moving_rover_landing]]).

**Bottom line:** the CBF is now **correct and safe for moving targets** — machinery
triggers on all 7 profiles, `d` properly conditioned, no regressions, `τ=0.15`
ready. It **cannot be shown beneficial on the rover** until the rover flies a
survivable approach. `CBF_DRIFT_TAU=0` stays the default (no unvalidated-benefit
default); flip to 0.15 whenever the rover approach is fixed — it's safe and inert
on stationary already.

---

## ===== SESSION CLOSE 2026-09-09 — consolidated state =====

**Read this block first; the UPDATE sections above are the detail trail.**

### What the visibility CBF IS now (all baked / committed / pushed)
`src/visibility_projection.py`, spec `docs/CBF_visibility.pdf` (rewritten). ONE
convex QP per cycle, no scenario branching, zero rover conditionals in
`controller.py`:

    (y*, s*) = argmin ½‖y − y_d‖² + ½ρ‖s‖²
      s.t. |c + L_e(y − y_now) + τ·d|_k ≤ φ_k + s_k        (visibility, soft)
           ‖y‖ ≤ y_max = √(A_CAP²/a_z² − 1)               (deliverability, hard)
           s ≥ 0

- `L_e = −(L_ω M)` (sign negated vs the retired machinery; verified).
- Deliverability ball = the `arccos(a_z/A_CAP)` lean cap / thrust sphere, FOLDED
  IN → `I_a` actuator-feasible by construction; `controller.py` lean/thrust caps
  are now redundant guards (cover the raw path + degenerate `A_CAP ≤ g`).
- Slack `s` penalised by `CBF_VIS_RHO`=2000 → graceful degradation, never
  infeasible, when the FoV box and the thrust ball are disjoint.
- `a_d[2]` is a FIXED input to the QP. Solver: slack eliminated in closed form →
  projected Newton (exact interior) + 1-D circle refine when the ball binds.
- Tier 2 `descent_ease` (`CBF_DESCENT_EASE`=1): scales ONLY the downward part of
  `a_d[2]` on a measured time-to-edge, self-releasing. NOT a CBF.

### Moving-target lead `τ·d`
- `d` = the front-end's de-rotated optic flow `h_xy`, **identity map** to the CBF
  frame (verified: `+I` cos 0.87, every other signed permutation ≈0 —
  `h_xy` is already post-camera-mount-swap).
- Conditioned by `condition_drift()`: `rel_resid` gate
  (`_img_node._bgflow_health[0]` > `CBF_DRIFT_RESID_GATE`=0.45 → `d=0`) →
  median-of-3 → 1-pole LPF (`CBF_DRIFT_LPF_ALPHA`=0.12) → radial clamp
  (`CBF_DRIFT_MAX`=0.5 tangent/s). Raw `h_xy` without this is unusable (spikes
  `|d|` 4–16 on aggressive target motion; noise on a static target).
- `τ` = `CBF_DRIFT_TAU`, **DEFAULT 0** (reactive-only). A flip to 0.15 was tried
  on 2026-09-09 (user direction) and **REVERTED** — the IC2-5 stationary confirm
  gate (`test_data/DriftTauConfirm/20260909-200537`) FAILED:
  `τ=0.15` gave **3× the hard touchdowns** (max rel_vel 2.80 vs 1.55, incl. a
  0.53 m / 2.80 m/s IC4 impact — effectively a failed landing `τ=0` doesn't have),
  **P+S 11/20 vs 14/20**, **IC4 4P→1P**. Median xy unchanged (0.052 vs 0.056) → a
  terminal-noise TAIL regression: on a stationary target `d` is self-motion `h_xy`,
  and even conditioned `τ·|d|`~0.02 tangent perturbs the terminal command. No
  moving-target benefit to offset (still unmeasurable). **Set `CBF_DRIFT_TAU>0`
  per-run for rover work only.**

### Env knobs (all default-safe)
`CBF_BUFFER_FRAC`=0.15 · `CBF_VIS_RHO`=2000 · `CBF_DRIFT_TAU`=0 (flip to 0.15 tried + reverted, 2026-09-09) ·
`CBF_DRIFT_MAX`=0.5 · `CBF_DRIFT_RESID_GATE`=0.45 · `CBF_DRIFT_LPF_ALPHA`=0.12 ·
`CBF_DRIFT_LOOM_STRIP`=0 · `CBF_DESCENT_EASE`=1 · `CBF_GMIN`=0.2 · `CBF_TREACT`=1.5 ·
`CBF_DRIFT_PULLBACK_FRAC`=0.4
Logs: `vis_active(t)` `vis_slack(t)` `vis_drift(t)` `vis_c(t)` `vis_gz(t)`.

### Validation status
- Offline: `tools/validate_visibility_projection.py` **15/15** across 5 seeds
  (independent pinhole+attitude oracle; +checks for deliverability-by-construction,
  graceful degradation, moving-target lead, condition_drift).
- **Stationary IC2-5 SITL A/B (QP vs pre-QP):** wash / PASS — 20/20 land both arms,
  0 TL, pooled mean xy 0.07 vs 0.06. `test_data/VisProjQPGate/20260909-142528`.
- **Rover 7-profile sweeps** (`test_data/RoverCBFSweep/20260909-163929` raw,
  `.../20260909-182607` conditioned+τ=0.15): CBF machinery **triggers correctly on
  all 7 motion profiles** — `d` live, `vis_active` fires, `_last_drifted_off` never
  logged, marker in-frame ~100%. `condition_drift` caps `|d|` at 0.5 in-loop,
  `τ=0.15` shows **no regression** vs `τ=0` (no TL, Static no longer pushed out of
  box, lower peak slack on noisy profiles).
  **BUT "does the lead improve rover visibility" is UNANSWERABLE** — the rover
  approach dies in 2–20 s (bimodal, perception-driven) on both arms, no stable
  window to measure. NOT a CBF problem.

### What's DONE
Stationary visibility CBF: designed, built, validated, gated, baked, documented.
Moving-target machinery: built, conditioned, offline-validated, frame-verified,
regression-free in SITL. One implementation serves both.

### What's OPEN (not CBF-blocking)
1. **`CBF_DRIFT_TAU` — SHOULD bake at 0.15; blocked by ONE fixable mechanism.
   Default is 0 for now.** (See the "CBF-BEHAVIOUR A/B" UPDATE section below for
   the full analysis.)
   - **The case FOR τ=0.15:** on a MOVING target, reactive-only τ=0 cannot keep
     the marker in frame — it chases excursions after the centre has already left
     the FoV/sensor (278 sensor-exit frames / 24 rover reps vs 0 on stationary;
     Linear reaction lag never resolves). `τ·d` is exactly the anticipation that
     closes this gap. This is the main reason to bake it.
   - **The blocker:** `h_xy` corruption in the terminal-overfill zone (coherent
     ramp to ~1.5 tangent/s; `rel_resid` gate can't see it) → with `τ>0` becomes a
     phantom lead → the IC2-5 confirm gate's IC4r2 (2.80 m/s TD, 22% terminal
     frames intervened). Same #1 perception blocker, second exposure path.
   - **The fix:** add an **overfill gate** to `d` (zero `d` when
     `MARKER_EXTENT_PX` says the marker fills the frame, ~`ext>270`). Not scenario
     branching — a perception-health gate. Then re-run `run_drifttau_confirm_gate.sh`
     (stationary, must not regress) + a rover CBF-behaviour check (sensor-exits
     must drop). If both hold → bake single `τ=0.15`.
   - Perception-thread dependency for a full moving-target verdict still stands
     ([[project_20260901_moving_rover_landing]]): oblique-view detector collapse
     (~5 m) + terminal-overfill loom collapse (~1.1 m).
2. Idea-4 descent-ease knob sweep (`t_react`/`buffer_frac`) — designed
   (`docs/DESCENT_EASE_KNOB_SWEEP.md`), not run. Low priority (governor is
   frequent but gentle, outcomes clean).
3. `CBF_DRIFT_LOOM_STRIP` sign never confirmed on a rover recording (default OFF).
4. ArUco 4-corner variant — module is cross-marker-only; retired ArUco CBF.
   Re-derive only if ArUco comparison numbers are needed for the paper.

### Harnesses / tools built this thread
`scripts/run_visproj_qp_gate.sh`, `scripts/run_rover_cbf_sweep.sh`,
`tools/analyze_rover_cbf_sweep.py`, `tools/validate_visibility_projection.py`.
Backups: `Obsolete/{src,tools}/*_v1_pre_qp_slack.py`,
`Obsolete/src/visibility_projection_v_pre_visproj.py` (+ pre-visproj pair).

### Key commits
`8884f43d` module · `82fa9c16` wire-in · `e63751e2` QP+slack+deliverability+lead ·
`28e4417b` h_xy identity-map fix + `vis_c(t)` · `e1b094e8` `condition_drift` ·
`16eb5d2d` all-docs · `9f08c799` re-sweep memory. (Peer `032e79ea` ported the
two-tier design to MATLAB + manuscript.)

---

## UPDATE 2026-09-09 (cont.): CBF_DRIFT_TAU flip → confirm gate FAILED → reverted

The `0 → 0.15` default flip (commit `b71a9505`, user direction) was checked by an
IC2-5 stationary A/B confirm gate: `scripts/run_drifttau_confirm_gate.sh`,
`test_data/DriftTauConfirm/20260909-200537`, `CBF_DRIFT_TAU` 0 vs 0.15, n=5
interleaved, cross-marker stationary, HEADLESS.

| metric | τ=0 | τ=0.15 |
|---|---|---|
| landed / TL | 20/20 / 0 | 20/20 / 0 |
| **P+S** | **14/20** | **11/20** |
| mean xy | 0.094 | 0.121 |
| median xy | 0.056 | 0.052 (≈) |
| mean / max rel_vel | 0.49 / 1.55 | **0.68 / 2.80** |
| hard touchdowns (>1 m/s) | 1 | **3** |
| per-IC P+S | IC2 4 · IC3 3 · IC4 4 · IC5 3 | IC2 3 · IC3 **5** · **IC4 1** · IC5 2 |

**REJECT.** No TL, but `τ=0.15`: 3× the hard touchdowns incl. a 0.53 m / 2.80 m/s
IC4 impact (effectively a failed landing `τ=0` lacks); P+S −3; **IC4 collapses
4P→1P**. Median xy unchanged → a terminal-noise TAIL regression: on a stationary
target `d` is self-motion `h_xy`, and even the conditioned `τ·|d|`~0.02 tangent
perturbs the terminal command on the marginal reps (IC4 = the 7 m-start IC, also
has the known >6 m cal gap). No moving-target benefit to offset (unmeasurable —
rover perception-blocked).

**Reverted** (`controller.py` 3 reads back to `"0.0"` + comment; docs
CBF_visibility.tex/.pdf / PLASMC_TUNING_GUIDE / PARAMETER_ANALYSIS /
CONTROL_FRAMEWORK_REVIEW / CLAUDE.md; this memory). `condition_drift` + all other
knobs KEPT (gated on `τ>0`, unaffected). `run_drifttau_confirm_gate.sh` kept for
any future re-test (e.g. a smaller `τ` like 0.05, or after the rover approach is
fixed). **Lesson: "regression-free on the rover re-sweep" ≠ safe to default — the
rover reps die in 2-20 s and can't show a terminal-tail regression; the stationary
IC2-5 gate can and did.**

---

## UPDATE 2026-09-09 (cont.): CBF-BEHAVIOUR A/B (τ=0 vs τ=0.15) — moving target is why τ=0.15 SHOULD bake

User: don't judge τ by SP (wrong lens here); judge by CBF trigger correctness +
whether the safe control input is better. Analysed `DriftTauConfirm/20260909-200537`
(stationary) and the two rover sweeps (`RoverCBFSweep/20260909-163929` +
`.../182607`, `off` arm = τ=0) on CBF-internal metrics.

### Stationary (DriftTauConfirm, 20 reps/arm) — τ=0 is better HERE
| metric | τ=0 | τ=0.15 |
|---|---|---|
| false-positive fires (Tier 1 fired while `|c|/φ`<0.4) | **0 / 20 reps** | 32 frames / 3 reps (21 = IC4r2) |
| TPR (fired on near-edge & rising) | 0.47 | 0.45 |
| **sensor-exit frames** (`|c|` > physical FoV edge) | **0** | **0** |
| mean / max `maxC/φ` | 0.70 / 1.02 | 0.71 / 1.03 |
| intervention → `|c|/φ` drops next 3 fr | 42% | 35% |
On a STATIONARY target `d` = self-motion `h_xy` + noise (not target drift), so
`τ·d` fabricates breaches → 32 spurious interventions on a centred healthy command.
Real safety metric (sensor exits) identical = 0 both. τ=0 = exact minimal
intervention. **Stationary verdict: τ=0.**

### Moving target (rover sweeps, `off`=τ=0, 6 motion profiles, 24 reps) — τ=0 FAILS its core job
| profile | `vis_active`% | t₁ₛₜ fire (flight%) | maxC/φ | **sensor-exit frames** | reaction lag (fr) |
|---|---|---|---|---|---|
| Linear | 4.7 | 79 | 0.54 | **71** | **40 = never caught** |
| Circular | 6.4 | 60 | 1.06 | 15 | 4.5 |
| EightShape | 3.5 | 62 | 0.57 | 9 | 1.5 |
| Sinusoidal | 8.1 | 84 | 1.24 | **71** | 11 |
| Lissajous | 10.2 | 85 | 1.08 | 3 | 0.9 |
| CircularYaw | 9.6 | 63 | 1.17 | **109** | 3.1 |
| **TOTAL** | — | — | — | **278 / 24 reps** | — |

τ=0 on a moving target: **triggers correctly** (`vis_active` 3.5–10% > stationary's
2.5%; fires within 1–5 fr of `|c|/φ` crossing 0.7 on ~every excursion — 100%
fired-within-5) **but reactive-only is structurally one control step behind
continuous target motion.** It projects the lean for where `c` IS, not where it's
GOING, so the centre crosses φ and keeps going out before the pull-back catches it
— **278 frames where the marker centre left the physical sensor across 24 reps, vs
0 on stationary.** Linear is the sharpest: steady drift → reaction lag never
resolves (40 = window end) → 71 post-edge frames. It corrects the marker back in
AFTER it has left the FoV, rather than keeping it in.

### THE CASE FOR BAKING τ=0.15 (recorded per user direction)
**The moving-target gap above is the main reason to bake τ=0.15.** Reactive-only
τ=0 cannot keep a moving marker in frame — it chases excursions after the fact.
`τ·d` is exactly the anticipation that closes that gap; the lead concept is right
for the moving case. On a stationary target the lead is near-neutral (self-motion
flow) — the only reason τ=0 "wins" stationary is the spurious fires, which are a
symptom of the *source*, not the *concept*.

### THE BLOCKER (single τ, one value for both)
τ=0.15 cannot bake as-is because of ONE mechanism, not a moving/stationary split:
**`h_xy` corruption in the terminal-overfill zone** (marker fills frame, LK flow
ramps coherently to ~1.5 tangent/s, `rel_resid` stays low so `condition_drift`'s
gate passes it; `vis_c` also jumps ±0.9φ). With `τ>0` that garbage becomes a
phantom lead → IC4r2: 22% of terminal frames intervened, 2.80 m/s touchdown. This
is the #1 open perception blocker ([[project_20260901_rover_cross_perception_diagnosis]]),
now with a second exposure path via the lead. `condition_drift` `rel_resid` +
median-3 + clamp do NOT catch coherent overfill corruption.

### PATH TO BAKING ONE τ=0.15
1. Add an **overfill gate** to `d` (zero `d` when `MARKER_EXTENT_PX` says the
   marker fills the frame, ~`ext > 270` on the 240×320 frame). NOT scenario
   branching — a perception-health gate, same category as the `rel_resid` gate,
   just one that detects coherent overfill. Rationale independent of target type:
   below ~0.5 m you are <1 s from touchdown, `0.15 s` of lead is pointless there.
2. Re-run BOTH: `run_drifttau_confirm_gate.sh` (stationary, must not regress) AND a
   rover CBF-behaviour check (sensor-exit frames should drop vs τ=0).
3. If stationary holds and moving improves → bake single `τ=0.15`.
Until then the single default is `τ=0` (safe stationary; known moving-target gap
documented above).
