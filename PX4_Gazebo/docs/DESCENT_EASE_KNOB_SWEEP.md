# Tier-2 descent-ease knob sweep — design (2026-09-09)

**Question (idea 4 from the QP-refinement thread):** the Tier-2 descent governor
(`descent_ease`) engages on ~35–64 % of terminal frames in the IC2–5 gate
(`vis_gz < 1`), `gz_min` 0.76–0.93. That is *frequent*. Is it doing useful work, or
is it steady-state friction that a coupled lean+descent optimisation (or just a
looser knob) would remove? Decide **before** considering the coupled optimisation —
which reintroduces the `a_z`-as-solved-variable structure the joint-QP retirement
deliberately removed.

## Hypotheses

- **H0 (keep as is):** the easing is gentle, self-releasing, and outcomes are clean
  (gate: 20/20 land, tight `xy`). Frequency ≠ harm.
- **H1 (knob too tight):** `t_react = 1.5 s` / `buffer_frac = 0.15` make the governor
  fire on nominal terminal loom that the lateral loop would handle anyway →
  it costs descent time (longer `flight_s`) for no precision gain.
- **H2 (knob too loose elsewhere):** on the *hard* off-centre reps the easing is what
  keeps the marker in frame; loosening regresses those.

## Sweep

Two knobs, per-axis-irrelevant (scalar), 1-D each — **not** a grid (n cost):

| knob | env | baked | sweep points | rationale |
|---|---|---|---|---|
| `t_react` | `CBF_TREACT` | 1.5 | **1.0, 1.5, 2.5** | horizon below which easing starts; lower = fires less/later |
| `buffer_frac` | `CBF_BUFFER_FRAC` | 0.15 | **0.10, 0.15, 0.20** | wider buffer = `rem` smaller = `t_edge` smaller = fires *more*; also shifts Tier-1 |

Run `t_react` first (pure Tier-2). Only sweep `buffer_frac` if `t_react` is
inconclusive — it moves Tier 1 too, so it is not a clean Tier-2 lever.

## Protocol

- **Cells:** IC2–5, **n ≥ 5**, interleaved arms (shared SITL drift), cross-marker,
  HEADLESS. Same harness shape as `run_visproj_qp_gate.sh` (per-arm
  `LANDING_OUT_BASE`), one env var swapped per arm instead of a worktree.
- **Baseline arm:** the QP default (`CBF_TREACT=1.5`), so this stacks on the QP gate
  result, not the pre-QP module.
- **Gate on a SINGLE failed landing** (project rule). A knob that lands 20/20 and
  changes nothing else is a no-op → keep the baked value (don't default a change
  without a win).

## Metrics (per arm, pooled + per-IC)

1. `vis_gz` engaged-frame fraction and `gz_min` (does the knob actually move the
   governor?).
2. `flight_s` terminal segment (last 2 m) — **the H1 tell**: if a looser knob lands
   equally precise but faster, the governor was friction.
3. `xy_err`, `rel_vel`, precise/soft counts — must not regress (H2 guard).
4. `s_e_n` at touchdown + `a_u` / `κ` terminal peaks — did easing off change the
   lateral-convergence picture on the marginal reps?

## Decision

| outcome | action |
|---|---|
| looser `t_react` (1.0): equal precision, shorter terminal `flight_s`, no TL | **default `CBF_TREACT` down**; governor was over-active; coupled optimisation not needed |
| looser `t_react`: any TL or `xy` regression on IC2–5 | keep 1.5; the easing is load-bearing on the tail → H0; revisit coupled optimisation only if a *specific* rep shows lean+descent fighting |
| no measurable difference across 1.0–2.5 | keep 1.5 (baked, passed the gate); governor is cheap and inert-enough; close idea 4 |

## Not in scope

Coupled lean+descent MPC. Only pursue if this sweep shows the *separation* itself
(not the knob) causing a measurable loss — which none of the gate evidence so far
suggests.
