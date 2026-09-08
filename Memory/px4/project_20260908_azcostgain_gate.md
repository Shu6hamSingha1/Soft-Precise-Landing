---
name: project_20260908_azcostgain_gate
description: "IC2-5 cross-marker SITL A/B gate, CBF_AZ_COST_GAIN=5 (default, descent-rate relief ON) vs =0 (relief OFF). On a CLEAN loom base (post-e173b05c). Result: relief=0 wins/ties — pooled meanXY 0.27->0.18, maxXY 1.30->0.57, maxVe 3.18->1.48, prec+soft 7/20->10/20, 0 TL both. The relief adds terminal tail-risk (hard impacts) with no upside. Recommend removing it (simplification step 1); makes the CBF_JQP_RELIEF_REF_AZ0 re-gate moot."
metadata:
  node_type: memory
  type: project
---

The decisive gate for [[project_20260908_visibility_cbf_simplification_audit]] step 1
("kill the descent-rate relief"). Run 2026-09-08, `test_data/JQP_AzCostGain_AB/20260908-204120`,
`scripts/run_azcostgain_gate.sh`. Base = `main` @ `bd384a66` — **includes the loom fix
`e173b05c`** (`CROSS_TZ_VETO_R_MULT=1.0`), so no frozen-loom confound (unlike
[[project_20260908_jqp_relief_refaz0_gate_failed]]). Verified single SITL stack, no
concurrent session. Arms interleaved per rep, n=5/arm/IC, headless, `WORLD=cross_marker`.

- `gain5` = `CBF_AZ_COST_GAIN=5` (current default — relief ON)
- `gain0` = `CBF_AZ_COST_GAIN=0` (relief OFF; eq-6 of the pdf otherwise unchanged)

## Result

| IC | arm | meanXY | medXY | maxXY | maxVe | prec | soft | TL |
|---|---|---|---|---|---|---|---|---|
| IC2 | gain5 | 0.20 | 0.10 | 0.62 | 1.54 | 2 | 0 | 0 |
| IC2 | **gain0** | 0.15 | 0.13 | 0.24 | 0.98 | 1 | 0 | 0 |
| IC3 | gain5 | 0.17 | 0.12 | 0.35 | 0.89 | 2 | 0 | 0 |
| IC3 | gain0 | 0.28 | 0.21 | 0.57 | 1.48 | 2 | 0 | 0 |
| IC4 | gain5 | 0.44 | 0.13 | **1.30** | **3.18** | 2 | 0 | 0 |
| IC4 | **gain0** | 0.14 | 0.12 | 0.23 | 0.71 | 1 | 3 | 0 |
| IC5 | gain5 | 0.29 | 0.19 | 0.76 | 2.29 | 1 | 0 | 0 |
| IC5 | **gain0** | 0.13 | 0.13 | 0.24 | 0.85 | 2 | 1 | 0 |

**Pooled (n=20/arm):**

| | gain5 (relief ON) | gain0 (relief OFF) |
|---|---|---|
| mean xy | 0.27 | **0.18** |
| median xy | 0.12 | 0.13 (wash) |
| **max xy** | **1.30** | **0.57** |
| **max rel_vel** | **3.18** | **1.48** |
| precise | 7 | 6 (wash) |
| precise + soft | 7 | **10** |
| TL / landed | 0 / 20 | 0 / 20 |
| hard/loose reps (xy>0.5 or ve>2) | **4** | **1** |

## Read

`gain0` is **at least as good on the middle of the distribution** (median xy, precise
count are a wash) and **clearly better on the tail**: 3× smaller worst xy, half the worst
touchdown velocity, 4 hard/loose reps -> 1. Every `gain5` hard/loose outlier is either a
cold-start rep1 or a terminal `rel_vel` spike (IC4r5: 1.30 m / 3.18 m/s impact; IC5r5:
0.76 / 2.29) — the relief occasionally over-relieving into the box<->relief deadlock
([[project_20260908_jqp_relief_refaz0_gate_failed]]), now on a clean loom base where it
degrades a touchdown instead of cascading to a full crash.

**The descent-rate relief adds terminal tail-risk with no measurable upside.** This is the
"kill the relief" result. Also worth noting `gain0` produced 4 SOFT landings (IC4 3, IC5 1)
vs `gain5`'s 0 — consistent with the relief's `-g` pin producing harder arrivals.

## Recommended action (needs user sign-off — touches shared `cbf_visibility.py`)

1. `CBF_AZ_COST_GAIN` default `5.0 -> 0.0`, or delete the relief block + `_relief_ref_az0`
   / `_az0` entirely (audit step 1).
2. `CBF_JQP_RELIEF_REF_AZ0` and its provisional-reject re-gate become **moot** — no point
   re-testing a fix to a term being removed. Downgrade
   [[project_20260908_jqp_relief_refaz0_gate_failed]] to "superseded — relief removed".
3. Then audit step 2 (collapse joint-QP -> tilt-QP + one `arccos(a_z/A_CAP)` clip) becomes
   the natural follow-up, since with the relief gone the joint QP is the tilt path plus a
   sphere that binds 0.0%.

## DEEP DIVE (user-requested before deleting): the relief's failure decomposed

Frame-level trace of the `gain5` hard/loose reps (`test_data/JQP_AzCostGain_AB/20260908-204120`).
**My earlier "box<->relief DEADLOCK" framing was over-attributed** — on the CLEAN base the
`-g`-pin fraction is similar in both arms (~30-50% terminal); that was partly the frozen loom
on the confounded base. The real decomposition, three components:

### (1) PHANTOM RELIEF / self-inflation — the one genuine relief bug (REF_AZ0=0 default)
Across all 20 `gain5` reps: relief fires >0.1 m/s^2 with `dtheta_az < 0.05` (**box NOT
binding**) in **433 frames = 2.2%**, vs only **279 frames = 1.4%** where it fires on a real
box bind. **61% of all relief firings are phantom.**
Cause: the relief computes `_lat_supp = ||th_desired - P@(Ia_lat/_az_ref)||` with
`_az_ref = _az_now` (the RUNNING vertical accel, already inflated by the relief's own prior
cycles) instead of the fixed unconstrained `_az0`. So `P@(Ia_lat/_az_now) != th_desired` on a
pure `1/az` scale difference even when `Ia_lat == I_a[:2]` (box open) -> `_lat_supp > 0` ->
relief fires -> `Ia_z` stays inflated -> loop latches at ~1.2 m/s^2.
Trace (IC4/gain5/rep1, frames 1274-1278, alt 0.06-0.07 m): `dtheta_az = 0.000`,
`th_cone == th_des` (box demonstrably open), yet the CBF pushes `I_a[2]` 1.2 below the raw
SMC for ~0.1 s -> the vehicle FLOATS at 6 cm, held in the `ext=318` overfill danger zone.
**`CBF_JQP_RELIEF_REF_AZ0=1` fixes exactly this** (measures `_lat_supp` at `_az0`).

### (2) NOT the relief — terminal-overfill centroid corruption (the #1 open blocker)
The two HARDEST reps (IC4r5 1.30 m / 3.18 m/s, IC5r5 0.76 / 2.29): `s_e_n` jumps
0.05 -> 2.39 in ONE frame at `ext=318`, `I_a_raw[2]` -> -64, `a_u` -> 139. **The relief is
GATED OFF during this** (`float(I_a[2]) > -g` is false when the SMC commands hard lift).
`th_desired > 1.5 rad` (nonsense tilt) occurs in 48 frames total, 60% with the relief gated
off. `gain0`'s matched rep5s simply didn't hit the glitch (independent per-rep sim noise).
So `gain5`'s 2 worst reps are NOT relief-caused.

### (3) No demonstrated benefit
No IC shows the relief improving landing outcome. IC3 `gain5` mean 0.17 vs `gain0` 0.28, but
that's 2 loose `gain0` reps at n=5. The "trade descent for lateral-convergence time" premise
is unproven ([[project_20260908_visibility_cbf_simplification_audit]], [[feedback_descent_softness]],
[[feedback_backstep_tried_clamps_are_lever]]).

## "REF_AZ0 / the re-gate is moot" — CONDITIONAL, corrected

- **Moot IFF the relief is removed.** `_relief_ref_az0` / `_az0` are read only inside
  `if _az_cost_gain > 0.0 and I_a[2] > -g:`. With `CBF_AZ_COST_GAIN=0` that block never runs
  -> dead variables; the `CBF_JQP_RELIEF_REF_AZ0` env gates nothing; the provisional-reject
  re-gate compares two ways of computing a term that won't execute.
- **NOT moot if the relief is kept.** `REF_AZ0=1` fixes component (1) (61% of firings are
  phantom). It would then belong as the default, and its loom-confounded provisional-reject
  ([[project_20260908_jqp_relief_refaz0_gate_failed]]) needs a proper 3-way clean gate:
  `relief_off | relief_on/REF_AZ0=0 | relief_on/REF_AZ0=1`.

## Verdict (unchanged, now better grounded)

Remove the relief. `REF_AZ0=1`'s ceiling is "the relief becomes neutral" (approaches
`relief_off` — it relieves strictly LESS, 926/926 offline cases), i.e. no upside to preserve.
Removal deletes ~152 lines of joint-QP scaffolding, moots a knob + a re-gate, and eliminates
the phantom-relief mode outright instead of patching it. Component (2), the biggest cost, is
the terminal-overfill blocker and is untouched either way.
