---
name: project_20260903_loom_r_schedule_ab
description: "NULL RESULT (2026-09-03, n=5 interleaved SITL A/B, flat cross_marker IC2): CROSS_LOOM_R_SCHEDULE (landed 06b28ebc, DEFAULT OFF) produces NO flight-outcome difference -- 3/5 precise both arms, median xy 0.078 vs 0.083. The offline gain (1.3-2.0 m loom r +0.309->+0.405) did NOT convert. Keep it OFF. Confirms the loom failure is upstream of the estimator: a noise model cannot fix coherently-corrupted flow."
metadata:
  node_type: memory
  type: project
---

## What was tested

`CROSS_LOOM_R_SCHEDULE=1` — a U-shaped multiplier on the hw-KF's `r[2]` (loom row only)
keyed on `MARKER_EXTENT_PX`, widening measurement noise as extent moves away from its
measured ~68 px sweet spot. Fitted to 3093 frames / 11 runs / 2 sessions. Landed
`06b28ebc`, **default OFF**.

Harness `test_data/LoomR_AB_harness/loomr_ab.sh` (copied from `spanrescue_ab.sh`),
n=5 **interleaved per rep**, `HEADLESS=1 WORLD=cross_marker MARKER_TYPE=cross`,
IC2 (2,2,5). Raw: `run_logs/loomr_ic2.tsv`.

## Result — NULL on flight outcome

| | precise | median xy | vz 1.3-3.0 | vz 0.5-1.3 | vz 0.05-0.5 | balloon | h_z term std |
|---|---|---|---|---|---|---|---|
| OFF | 3/5 | 0.078 | 0.931 | 0.449 | 0.120 | 0.000 | 0.608 |
| ON | 3/5 | 0.083 | **1.073** | 0.421 | 0.123 | 0.000 | 0.621 |

- **Landing outcome identical.** 3/5 precise both arms. ON also produced the single best
  rep of the batch (SOFT+PRECISE, xy 0.013) and two FAILs — pure stochasticity.
- **Engagement VERIFIED** (the [[feedback_ab_baseline_verify_concurrent_commits]] trap):
  `Loom R Mult` median 1.87-1.91 / max 4.000 on every ON rep, exactly 1.000 on every OFF
  rep. The arms genuinely differed.
- **The predicted under-brake did NOT appear at the terminal**: vz 0.05-0.5 m is 0.120 vs
  0.123 (identical); vz 0.5-1.3 m is slightly GENTLER on ON (0.449 -> 0.421).
- ⚠ **But vz 1.3-3.0 m is ~15% FASTER on ON** (0.931 -> 1.073; 4 of 5 ON reps exceed 4 of 5
  OFF reps, ranges overlap). That is the band where the schedule ramps R up, and it is the
  direction [[feedback_pinv_tol_loom_scaling]] warned about. It did not propagate to the
  terminal band or to outcome, but it is the one consistent signal in the batch.
- **Terminal loom variance essentially unchanged** (0.608 -> 0.621) — the schedule did NOT
  smooth the loom. A 4x R bump is too gentle to matter against innovations that large.

## Why this matters more than the null itself

It empirically confirms the diagnosis: below ~1.3 m the loom is **coherently corrupted**,
not noisy-but-unbiased — points are plentiful (190), well-spread (coverage 2.19), and
well-conditioned (cond 7.8) while correlation with GT is +0.013. Telling the KF the
measurement is noisier cannot recover information that is not in the measurement. Any real
fix has to change **what is measured**, not how it is weighted.

## ⛔ Do not re-run this A/B expecting a different answer

Keep `CROSS_LOOM_R_SCHEDULE` default OFF. n=5 / one IC / one world, so this is
direction-of-effect only — but the mechanism argument above is independent of n.
If anyone does enable it, watch the 1.3-3.0 m descent rate.

## Harness bug worth not repeating

The in-harness `vz_term` came back `nan` on every rep: `np.gradient` divides by zero on
duplicate GT timestamps. `gt_optical_flow.compute_gt_flow` de-dups (`np.diff(t) > 1e-6`)
for exactly this reason. All metrics above were recomputed offline with de-duplication.
