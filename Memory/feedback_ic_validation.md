---
name: Always validate IC2-5 before changing defaults
description: Rule learned 2026-05-20/21 across multiple regressions — IC1-tuned configs frequently break off-center initial conditions
type: feedback
originSessionId: 3ffdaf6f-0da4-4c9f-b4df-fb9a91a7eb04
---
**Rule:** Any change to a default value, gain, or controller behavior that improves IC1 must be validated on IC2-5 (`run_ic_validation.sh`) before merging to defaults.

**SCOPE (user correction, 2026-08-10): this rule is about CONTROLLER GAIN tuning specifically.**
The mechanism it guards against (IC1-tuned lateral gains trading convergence speed for
precision, which only works when already centered) is a controller/gain phenomenon, not a
universal law. **During perception-pipeline development** (calibration, point-sampling,
detector/tracker changes — not gain tuning), a fix that demonstrably works on ANY single IC
can be baked without an IC2-5 gate first — perception correctness isn't IC-position-dependent
the way lateral-authority tradeoffs are. Don't over-apply this rule to perception work; ask
which regime a proposed default change falls into before invoking it as a blocker.

**SEQUENCING (user directive, 2026-06-10): do NOT run IC2-5 at all until IC1 is performing *perfectly* — clean, repeatable, no TL/drift.** IC2-5 is the pre-merge *gate*, but it is wasted effort while IC1 still fails; fix IC1 first, then gate on IC2-5. (As of 2026-06-10 IC1 is NOT clean — no verified SP, ~2/5 perception TLs — so IC2-5 testing is **on hold**. All current sweeps are IC1-only.)

## Why

The MATLAB standard 5-IC set is (NED):
```
IC1: (0, 0, -5)   centered, 5m altitude  ← most-tested, where this session did all tuning
IC2: (2, 2, -5)   off-center diagonal, same altitude
IC3: (2, -2, -5)  off-center opposite diagonal
IC4: (2, 2, -7)   off-center, higher altitude → more descent time
IC5: (2, 2, -3)   off-center, lower altitude → less descent time
```

Multiple times in the 2026-05-20/21 session, IC1 improvements **catastrophically regressed IC2-5**:

| Change | IC1 mean xy | IC2-5 mean xy | Verdict |
|---|---|---|---|
| `REF_RAD=-0.70` | 0.28 (down from 0.71) | 1.6-2.5, IC5 crashed at 7.1m | reverted, env-only |
| Virtual-compass SO(3) | 0.36 | IC4: 1.59 (was 0.87) | env-only |
| Best stable config (Kr+clamp+h_rd+WIN) | 0.387 | 1.6-2.5 | env-only |

The mechanism is consistent: off-center starts (2.83 m diagonal offset) need lateral travel time. IC1-tuned configs typically trade lateral convergence speed for precision, which works when the drone starts ALREADY centered. Off-center starts run out of altitude before lateral converges.

## What to do

- Before proposing a default change: run `HEADLESS=1 <env_vars> bash run_ic_validation.sh` (8 runs, ~25 min). The script tests 2 reps each on IC2-IC5.
- Don't accept "IC1 result looks great" alone as sufficient evidence.
- If the change helps IC1 but hurts IC2-5: keep it as an env-overridable option, don't make it the default.

## What to do when the user wants a clear IC1 precision win

- Default behavior: leave defaults alone. Document the IC1-optimized env-var set in `feedback_precision_tuning_lessons.md`.
- For paper/analysis runs at IC1 only, set the env vars per-run.

## Reference

The standard IC validator: `PX4_Gazebo/run_ic_validation.sh`. Tests IC2/IC3/IC4/IC5 at 2 reps each. Total time with the SITL flakiness fixes (commits 8214dd8, 0f98301): ~25 min.

## How to apply

- When the user asks to make a tuning result the default: check IC2-5. If not validated, run the validator first.
- When IC1 result is good but IC2-5 isn't tested: explicitly call this out and offer to run the validator.
- When user pushes back on running the validator: cite this rule; explain a flipside (e.g., "REF_RAD=-0.70 was a 60% IC1 improvement but IC5 crashed"). Respect "no" if they accept the risk.


---

**IC2-5 GATE on the RESTORED baseline (2026-06-04, all 3 knobs baked + validated gains, n=5/IC):**
| IC | start ENU | SP | TL | mean xy |
|---|---|---|---|---|
| IC1 ref | (0,0,5) | 2/10 | 0 | 0.18 m |
| IC2 | (2,2,5) | 0/5 | 0 | 2.21 m |
| IC3 | (-2,2,5) | 0/5 | 0 | 2.0 m (+1 21m flake) |
| IC4 | (2,2,7) | 0/5 | 0 | 0.88 m (best) |
| IC5 | (2,2,3) | 0/5 | 0 | 2.37 m |
Outcome: STABLE off-center (no TL except 1/20 flake, no catastrophic divergence -> the 3 baked defaults are
SAFE off-axis, bake validated), but 0 SP at IC2-5 (~2-2.5m miss). This is the CLASSIC documented IC1-specificity,
NOT a new regression: the ×0.35 lateral gains tuned for IC1 precision/softness lack the AUTHORITY to close a 2m
lateral offset within the descent — and raising lateral gain to close it faster breaks perception (LK breakdown,
staircase c3 catastrophic). So IC2-5 precision sits on the SAME perception-limited lateral frontier. Evidence:
IC4 (higher start, 7m -> more descent time) is the best off-center (0.88m), IC5 (lower, 3m -> less time) the
worst (2.37m) -> it's a lateral-convergence-time problem, not instability. The baseline is mergeable as the IC1
reference (stable everywhere, precise at IC1); off-center PRECISION is the open problem, and it = the perception
ceiling (can't fly laterally fast enough without LK breaking). Next real work for off-center: terminal lateral
braking / perception robustness, not lateral gain magnitude (exhausted).