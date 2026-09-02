---
name: project_20260902_archive_rescore_false_precise
description: "Archive-wide re-score of every saved landing (4446 runs) against a surface-relative altitude floor: 159 of 1192 'precise' verdicts (13.4%) were computed on a MID-AIR sample -- 21 aborted runs that never descended (a centered IC starts above the marker so xy_err~0 is degenerate) and 138 whose GT log terminated mid-descent (112 still descending, median last altitude 0.486 m). Concentrated in exactly two eras (Jul 19-23, Aug 22-24); the Aug 28-31 headline gates are CLEAN. Found via test_data/Final/IC5, which is scored precise at 0.42 m altitude."
metadata:
  node_type: memory
  type: project
  originSessionId: 5f1d366c-f4b6-4a4f-9d5b-05c93b9a480f
  modified: 2026-09-02T00:00:00.000Z
---

**2026-09-02.** Archive-wide audit triggered by a spot-check of `test_data/Final/`.
Extends [[feedback_false_sp_frozen_gt]] (which found ONE false SP, of a different
mechanism) from a single case to a measured population.

## The trigger: Final/IC5 is not a landing

`test_data/Final/IC5` is published (MANIFEST + [[reference_final_landing_recordings]]) as
**PRECISE 0.058 m**, the re-recorded 8/8 attempt that replaced a TARGET_LOST montage. Its
dataset ends with the drone at **0.422 m above the marker plane, still descending at
-0.065 m/s**, `B_T` collapsed to +0.030 (IC2 at the same point: -1.197) and
`MARKER_EXTENT_PX` saturated at 318 -- the terminal-overfill signature. All five Final GT
logs stop 4.4-4.8 s before their control logs (a post-touchdown recording tail), so **GT
log end = the touchdown latch**. IC1-4 latch at 0.04-0.14 m while moving UPWARD (post-contact
bounce, physically consistent). IC5 latched 0.42 m up, mid-descent.

Final's real score is **1/5 verified precise (IC2)**, not 2/5. The flight may well have
landed during the ~4.8 s of control that follows the log -- what is unsupported is the
VERDICT, not necessarily the landing.

## The audit

Every `Ground_Truth.npy` under `test_data/` (4456 found, **4446 parsed**; 10 skipped for
<10 samples or load failure). Per run: relative altitude `uav.z - target.z`, its minimum,
the trailing-0.25 s `dz/dt`, and the stored `SoftPrecise`. Post-dedupe (the `1f0b6cab`
cleanup had already removed the byte-identical Landing_Test mirrors -- **0 duplicates among
the flagged set**, verified).

**⚠ Use a SURFACE-relative floor, not a target-pose-relative one.** The first pass flagged
193 and was WRONG: a legitimate landing on the 0.4 m rover platform sits ~0.48 m above the
rover's target-pose origin (z=0.02). Floors used: **flat 0.20 m, rover 0.70 m** -- this
rescues **34** genuine platform landings. Discriminate with `median(target.z) > 0.01`.

| | count |
|---|---|
| runs parsed | 4446 |
| scored `precise` | 1192 (1187 deduped) |
| **unsupported (never reached the floor)** | **159 = 13.4%** |
| -- ABORTED (dur <3 s, or never below 2 m) | 21 |
| -- TRUNCATED MID-DESCENT | 138 (112 still descending at the last sample) |

**Class ABORTED** scores precise for a degenerate reason: at a centered IC the drone
**starts directly above the marker**, so `xy_err` ~ 0 at t=0. Median duration 1.29 s, median
`alt_min` 4.84 m, median xy 0.028 m. Worst never got below 5.78 m.

**Class TRUNCATED** — two confirmed latch mechanisms end the descent loop early: the
touchdown detector false-firing at altitude, and **the IMU impact detector** (`|a|>50` ->
`FC_node.LANDED=True`), which fires in **26.6 %** of runs that end airborne and is
**mode-independent** (a GT-feedback rover run latched on `|a|=65.6` while 0.49 m up, still
descending). ⛔ `Disarming denied: not landed` is NOT corroboration of either — it appears in
97.4 % of genuine landings (2396 paired runs). Median last altitude **0.486 m**; profiles show a steady descent that
simply stops (`AzLiftGain_IC1/20260823`: 2.85 -> 0.49 m at ~0.7 m/s, then log ends) versus a
clean control (`ICValidation/20260829`) descending smoothly to -0.01 m. Cause is almost
certainly a touchdown detector false-latching at altitude (the documented bug class --
[[project_20260830_perception_touchdown_detect_broken]],
[[project_20260828_ic_motion_batch_gt_touchdown_and_kappa_ratchet]]).

## Two bounded eras -- everything else is clean

| period | precise | unsupported |
|---|---|---|
| **Jul 19-23** | 43 | 24 (75-100 % on 07-20/21/22) |
| **Aug 22-24** | 100 | 88 (78-100 %) |
| all other dates | ~1050 | ~0 % |

## The headline results SURVIVE

| dataset | precise | unsupported | corrected |
|---|---|---|---|
| `ICValidation/20260829` | 103 | 0 | 103 |
| `ICValidation/20260828` | 27 | 0 | 27 |
| `ICValidation/20260831-144626` (post-alpha0 gate) | 13 | 1 | **12** |
| `DetectorFrameset` | 4 | 0 | 4 |
| `Final/` | 2 | 1 | **1** |
| `Multi_IC/postrevert_20260901` | 1 | 1 | **0** |

The post-alpha0 gate -- the evidence that stationary cross-marker landing works IC1-5
([[project_20260831_perception_mode_landing]]) -- **stands**. Its one flag is `IC3_rep3` at
`alt_min` 0.211 m with dz -0.031: a boundary case against a 0.20 m floor, not a clear miss.
Its `IC1_rep3/rep4` (`alt_min` 5.7, `xy=nan`) are the already-known test-rig IC-convergence
false positives.

## THE FIX — IMPLEMENTED `d6610ea7` (2026-09-02)

**⚠ An earlier version of this file recommended a `|dz/dt|` terminal-rest check, citing its
CATCH rate without measuring its FALSE-POSITIVE rate. That was WRONG — see "rejected" below.**

**The defect was WHEN `xy_err` is sampled.** `apps/landing_test.py:835` read it from
`pose_node.getPose()` at **whatever instant the control loop exited**. The comment there
claimed "Evaluated at touchdown (PX4 reports LANDED here)", but nothing enforced it — a
touchdown-detect latch, a timeout or an abort all exit that loop too. The file already
tracked `_min_alt` / `_min_alt_xy` / `_min_alt_relvel` (lines 558-592) but used them as
diagnostics only; they gated nothing.

### Shipped

A `NOT_LANDED` branch in the existing `target_lost` / `descent_anomaly` chain (same standing,
same shape — it follows the file's own pattern):

- **gate (a)** LOWEST altitude above the LANDING SURFACE <= `LANDING_TOUCHDOWN_ALT_MAX`
  (0.20 m). Catches all 159, **0 false positives** on the 1033 legitimate `precise` runs.
- **gate (b)** surface-agnostic backstop, `min_alt <= LANDING_DESCENT_FRAC_MAX (0.15) *
  start_alt`. Catches 45/159, 0/1033 FP. ⚠ **Fires ZERO times on today's archive** — gate (a)
  always catches first. It is insurance for a misconfigured surface height on a future world,
  not an active contributor; don't cite it as doing work.
- `SOFT_PRECISE` now records `terminal_state_ok`, `not_landed_reason`,
  `alt_above_surface_end`, `alt_above_surface_min`, `landing_surface_dz`. **Never silently
  flip a flag** — a silent flip is how this survived two months.
- `tools/rescore_softprecise.py` re-scores archived runs **non-destructively** (never writes
  `Ground_Truth.npy`); archived runs cannot be re-flown.

### Two things learned DURING implementation (both changed the design)

1. **Gate on `alt_MIN`, not `alt_end`.** The first cut gated the endpoint and returned 163,
   not the validated 159. The four extras had touched down and then **ballooned upward** —
   they DID land, and their endpoint xy is post-kick, a separate pre-existing issue that
   `_min_alt_xy` already exists to expose. Gating on the minimum keeps the change to exactly
   what was measured.
2. **The surface height already existed: `PLASMC_GT_MARKER_DZ`** (0.0 flat, 0.5 rover, set by
   `scripts/run_rover_landing.sh`, see `src/gt_feedback.py:45-50`). Reused it instead of
   adding a second name for the same number (`LANDING_SURFACE_DZ` remains as an override).
   **This caught a regression about to ship:** with a flat 0.20 m floor **every rover landing
   would have been flagged NOT_LANDED**, because a real landing on the raised platform sits
   ~0.48 m above the rover's own pose origin. 0.5 + 0.20 also reproduces this audit's 0.70
   rover floor exactly. General lesson: **before adding a world-specific constant, grep for an
   existing one** — this codebase already had it, correctly plumbed.

### Verification

| check | result |
|---|---|
| full archive | 4446 runs, 1192 -> **1033** precise, **159 lost** — matches this audit exactly |
| `test_data/Final/` | 2 -> **1** (IC5 flagged at 0.421 m; IC2 keeps) |
| post-alpha0 gate `20260831-144626` | 13 -> **12** (its one flag marginal at 0.211 m) |
| gate expression | 9/9 real edge cases incl. rover-surface + balloon |

### REJECTED candidates — measured, do not re-try

- **`|dz/dt|` terminal-rest gate: UNUSABLE.** Catches 152/159 at 0.05 m/s but **rejects
  600/1033 (58 %) of LEGITIMATE landings**; 65.7 % at 0.03. Real touchdowns still carry
  vertical motion in their last 0.3 s (settling, bounce, log ending at contact). A catch rate
  is meaningless without its false-positive rate.
- **IMU contact-spike gate: NO SEPARATION.** max|a| over each run's final 15 %: **27.0 %** of
  unsupported vs **26.7 %** of legit runs exceed 50 m/s².

### Blast radius (verified, not assumed)

No perception or control change. The block runs ~230 lines AFTER the control loop exits, its
inputs were already diagnostic-only, and the new env vars are read only there — flight
behaviour is byte-identical, only the recorded verdict changes. Retry wrappers are exit-code
driven and never read the flag. **One consumer does close a loop:**
`tools/coord_descent_tune.py` optimises on `(prec and soft and not tl)`, so its objective
tightens and it will steer to different gains.

### STILL OPEN

- Campaign summaries / `.ods` / manuscript numbers that quote SP rates are **not** re-scored.
  Archive-wide SP drops ~13 %, and the Jul 19-23 / Aug 22-24 campaign results move materially.
  That is a call for the user, not a silent edit.

Reproduce the audit: `python3 tools/rescore_softprecise.py test_data` (~0.2 min).
