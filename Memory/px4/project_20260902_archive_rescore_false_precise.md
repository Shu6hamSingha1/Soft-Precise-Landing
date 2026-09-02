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

**Class TRUNCATED** median last altitude **0.486 m**; profiles show a steady descent that
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

## THE FIX (not yet implemented)

`SoftPrecise` in `apps/landing_test.py` evaluates at the lowest logged sample with **no
absolute altitude floor and no terminal-state check**, so a log that stops mid-descent scores
as a touchdown. It needs BOTH halves of
[[feedback_relative_flatness_needs_absolute_gate]]:

1. an **absolute floor** on altitude above the LANDING SURFACE (not the target-pose origin);
2. a **terminal-state check** -- `|dz/dt|` small at the last sample. This alone would have
   caught **112 of the 138** truncated runs.

Reproduce: walk `**/Ground_Truth.npy`, compute `uav.z - target.z`, its min, and the
trailing-0.25 s slope; flag `precise=True` with `alt_min > floor(world)`. ~0.2 min for the
whole archive.
