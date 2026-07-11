---
name: project_touchdown_detect_velocity_gate_gap
description: "OPEN, confirmed on n=5 IC1 (2026-07-11, post cal-refit + KF-coast fix): TOUCHDOWN_LOOM fires on h_z>0 sign+persistence+centering ALONE, no velocity/rate check -- 4/5 successful detections landed PRECISE (positioning excellent, 0.002-0.038m) but 0/5 SOFT, clustered right at/just above the 0.2 m/s threshold (0.205-0.244 m/s, one outlier 0.723). Not yet fixed."
metadata:
  node_type: memory
  type: project
  originSessionId: a8922284-2fe3-4a78-9355-9949c3be5a10
---

## Finding (n=5 IC1, 2026-07-11, after merging the parallel session's KF-refit calibration
with this session's sign-guard removal + KF dt-decoupling + flow-KF reset + predict-only coast)

| Rep | xy_err | rel_vel | precise | soft |
|---|---|---|---|---|
| 1 | 0.023m | 0.238 | ✓ | ✗ |
| 3 | 0.038m | 0.205 | ✓ | ✗ |
| 4 | 0.013m | 0.244 | ✓ | ✗ |
| 5 | 0.002m | 0.723 | ✓ | ✗ |
(rep 2: unrelated `TARGET_LOST [DRIFT_OFF]`, lateral drift out of frame during descent)

Positioning is now excellent — this validates the calibration + KF-coast fixes worked. But
**velocity at touchdown is clustered right at/just above the 0.2 m/s soft-landing threshold**
in 3 of 4 successful detections (0.205-0.244), not randomly scattered — a systematic bias, not
noise. Rep5's 0.723 m/s is a separate outlier within this group, not yet investigated.

## Root cause (design gap, confirmed by code read)

`_touchdownDetect()` (controller.py) latches `LANDED` on:
```python
self._td_streak = self._td_streak + 1 if h_z > 0.0 else 0
if self._td_streak >= self._td_frames and |s_e_n| < self._td_sen:
    self._touchdown = True
```
This checks loom SIGN (persistence: 3 frames) and lateral CENTERING — **it never checks whether
vertical velocity has actually been arrested.** `h_z` inverting to positive signals a geometric
rebound event (physically plausible even with residual downward momentum, e.g. landing-gear
compression), which is sufficient to prove "contact has begun" but not "the contact was soft."

## First traced earlier this session (the "failed SP" analysis, IC1_rep4 from an earlier
untextured n=5 batch) — this later n=5 run CONFIRMS it as the dominant remaining failure mode
once the freezing/misattribution issues were fixed. Before those fixes, this gap was masked by
larger, more catastrophic failures (frozen h_z, terminal a_u explosions) — now that those are
gone, this narrower gap is the visible bottleneck.

## NOT yet fixed

Candidate fix (proposed, not implemented): add an explicit vertical-rate/velocity check to
`_touchdownDetect()` — e.g. require the estimated descent rate (from `h_z` itself, or a
separate observer) to be below a small threshold, or extend the persistence window, before
latching `LANDED`. User's framing from earlier in this thread: `h_z` IS meant to be the
velocity gate ([[feedback_kf_frozen_during_marker_loss]] fixed why it was structurally broken)
— but the current SIGN-ONLY check doesn't use its MAGNITUDE, which is where the remaining
softness information lives. A magnitude/rate-aware version of the same signal, not a
redundant separate one, is likely the right fix — but not designed or tested yet.

## Rep5 outlier (0.723 m/s) investigated — SAME mechanism, heavier tail

Checked: zero decode gaps this rep (clean run), `|s_e_n|=0.08` (excellent centering), so not a
marker-loss/misattribution issue. GT altitude near detect-time was already slightly negative
(−0.09 to −0.14m, i.e. past nominal ground-contact plane, gear-compression territory) with a
noisy but consistently-elevated descent rate right up to detection. **Not a distinct bug** — a
heavier-tail case of the same stochastic descent-rate variation the 0.2-0.24m/s cluster shows.
Strengthens the case for a single magnitude-aware fix rather than a separate investigation.

## Next steps
- Design + implement a magnitude-aware touchdown gate (e.g. hold until `h_z` has been positive
  AND below some rate-of-change threshold for N frames, not just sign-positive for N frames).
- Re-validate n≥5 after any change (this project's standing methodology rule).
