---
name: project_rover_speed_sweep
description: "Moving-rover SPEED SWEEP (GT-FB, Linear, n=3/cell, 2026-07-02): reliable landing envelope <=1.09 m/s (9/9 on-platform); binds at 1.56 m/s (1/3 + near-miss + miss). Mechanism = steady tracking lag proportional to target speed (0.40/0.72/0.94/1.65 m at 0.47/0.78/1.09/1.56 m/s = equivalent servo lag ~0.9-1.0 s); the terminal closure nulls it up to ~1.1 m/s, at 1.56 the residual eats the 0.3 m platform margin. Lever candidates: target-velocity feedforward / outer-loop bandwidth, NOT the 38 ms actuation lag."
metadata:
  node_type: memory
  type: project
---

**Moving-rover SPEED SWEEP (2026-07-02) — GT-FB, Linear trajectory, gate-started
motion, n=3 per cell, platform half-width 0.3 m = on-platform threshold.**

| SPEED_MULT | rover speed | on-platform | touchdown rel-lat (m) | touchdown rel-spd (m/s) | steady lag* (m) |
|-----------|-------------|-------------|------------------------|--------------------------|-----------------|
| 0.3 | 0.47 m/s | **3/3** | 0.28 / 0.044 / 0.048 | 0.08–0.23 | 0.40 |
| 0.5 | 0.78 m/s | **3/3** | 0.034 / 0.047 / 0.143 | 0.19–0.26 | 0.72 |
| 0.7 | 1.09 m/s | **3/3** | 0.048 / 0.092 / 0.110 | 0.06–0.15 | 0.94 |
| 1.0 | 1.56 m/s | **1/3** (+1 near-miss 0.324, +1 MISS 1.06/relspd 2.16) | 0.167–1.06 | 0.45–2.16 | 1.65 |
*steady lag = mean relative lateral error over the 3→1 m altitude tracking phase.

**ENVELOPE: reliable ≤ ~1.1 m/s (9/9); binds at ~1.5 m/s.** No fly-aways at ANY
speed (peaks all 5.0–5.1 m; the 1.56 MISS descended to ground beside the platform,
lat 1.06 m, not a launch — the platform-era fly-away mode stays gone at speed).

**MECHANISM: steady tracking lag ∝ target speed, equivalent lag τ ≈ 0.85–1.05 s**
(0.40/0.47 = 0.85 s; 0.72/0.78 = 0.92; 0.94/1.09 = 0.86; 1.65/1.56 = 1.06). A classic
type-1 servo steady-state velocity error: h_e→0 velocity-matches, but the position loop
carries e_ss ≈ v_target·τ during the descent. The TERMINAL phase closes the lag (funnel
tightening + 1/Z gain growth) — which succeeds up to ~1.1 m/s; at 1.56 m/s the 1.65 m
residual can't be nulled in the terminal runway → misses/near-miss + touchdown rel-speed
grows (0.45–2.16 = no longer velocity-matched). NOTE τ≈0.9–1.0 s is OUTER-LOOP bandwidth,
~25× the 38 ms actuation lag → likely tunable/architectural at the outer loop, e.g. a
target-velocity feedforward (the flow h already carries relative velocity; the missing
piece is an integral/FF that nulls the position lag against a CONSTANT target velocity —
K_ri exists, may be too slow) — NOT an actuation-lag wall. Untested; next lever if >1.1 m/s
capability is needed. Also candidate: start the descent AFTER the lag transient settles
(the velocity-step transient peaks ~0.74 m at 0.47 m/s; gate-start means the step happens
at descent start).

Harness: scratchpad speed_sweep.sh (cells via LANDING_OUT_BASE); data
scratchpad/spd_{0.5,0.7,1.0}/ + the 0.3 reps in test_data/Landing_Test (09-14/16/18).
n=3/cell = direction-of-effect per the ±5–7 noise-floor rule; the lag CURVE (monotone,
tight per-rep spread ±0.1–0.3) is the robust finding, the exact 1.56 rate is not.
Continues [[project_moving_rover_landing_works]]. NEXT: Circular r=0.8 turning validation
(~0.38 m/s tangential = well inside the envelope), perception-ON (flow dynamic range vs
target speed — LK ceiling ~2 m/s relative), lag-lever test if faster targets needed.
