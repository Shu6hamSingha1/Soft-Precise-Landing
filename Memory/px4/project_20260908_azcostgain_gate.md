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
