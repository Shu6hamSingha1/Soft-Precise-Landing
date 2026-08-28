---
name: project_20260828_ic_motion_batch_gt_touchdown_and_kappa_ratchet
description: "2026-08-28: an 11-run IC1-5 + rover-motion video batch on rover_cross/GT-feedback revealed the GT touchdown detector (added 2026-08-26/27) was falsely firing at depths 0.3-3.3m on EVERY run, not just the ones the harness's altitude check rejected. Fixed with an absolute depth ceiling. Once fixed, IC1 shows a genuine kappa-ratchet divergence (w_u saturates on Y -> zeta/sigma/s_e_n breach -> X-axis kappa ratchets ~8x -> a_u explodes to 2239) -- a known failure class (tune-plasmc mode 11) but unexplored for this rover_cross/GT-feedback gain set."
metadata:
  node_type: memory
  type: project
  originSessionId: 8e3cbb0e-2db1-4a43-9a72-a13e19dca7eb
  modified: 2026-08-28T08:00:28.121Z
---

## What happened

User asked to record IC1-5 + all rover-motion videos (batch_ic_and_motions.sh, 11 configs:
IC1-5 on Linear, plus Circular/CircularYaw/Sinusoidal/EightShape/Lissajous/Static at IC1).
IC1/IC3/all 6 motions reported ON-platform; IC2/IC4/IC5 failed after 6 tries each.

## Bug found: GT touchdown detector had NO absolute-depth gate

Diagnosing IC2's failure (`Diagnose why IC2 missed the platform`) surfaced that its
"TOUCHDOWN-DETECT (GT)" fired at `depth=1.287m` — nowhere near the ground. Checking
**every** run in the batch found the SAME bug fired in ALL 11 runs (including the 7
"successful" ones), at depths from 0.3m to 3.3m:

```
ic1: depth=0.299  ic2(x5): 1.631/0.761/1.287/1.168/0.942  ic4(x3): 3.267/2.765/3.181
ic5(x6): 0.514/2.079/0.954/0.902/0.985/1.402
circular: 0.596  circularyaw: 0.731  sinusoidal: 0.324  eightshape: 1.274  lissajous: 0.389
```

Root cause: the 2026-08-26/27 GT-only touchdown fix ([[project_20260826_ic1_gt_touchdown_fix]]
if it exists, else see controller.py `_touchdownDetect` GT branch) confirmed touchdown via
(1) rate-of-depth-change flattened + (2) real progress since arming + (3) near the running
minimum — but NEVER checked that depth was actually SMALL. An ORDINARY mid-descent
deceleration (e.g. the vehicle transiently slowing its vertical closure while correcting a
lateral overshoot) satisfies all three at ANY altitude. The 7 "on-platform" results only
passed the harness's loose accept band (0.35-1.15m absolute UAV altitude, a DIFFERENT
quantity than the touchdown detector's camera-marker relative depth) by coincidence.

**Fix applied** (`src/controller.py`, `_td_gt_max_depth` env `PLASMC_TD_GT_MAX_DEPTH`,
default 0.25m): require `depth < 0.25m` (near the documented true floor ~0.1-0.15m, see
`gt_feedback.py`'s Z_REG derivation comments) in addition to the existing rate/progress/
near-min gates. Verified: a fresh IC1 run now fires at `depth=0.000m`, genuine contact.

**General lesson** (see [[feedback_relative_flatness_needs_absolute_gate]]): a "value has
stopped changing relative to its own recent history" check is NEVER sufficient on its own
to detect a terminal/settled state — it only tests LOCAL stationarity, not proximity to the
actual target state. Always pair it with an absolute-value gate confirming the signal is
actually near where the terminal state should be.

## Bigger finding: fixing the bug exposes a real kappa-ratchet divergence

With truthful touchdown detection, IC1 (rover_cross, GT-feedback, Linear @0.3x speed) no
longer "succeeds" — it converges to `lat≈0.19-0.29m` around t=7-8.5s (a near-hover plateau,
exactly what the old buggy detector mistook for landed), then DIVERGES: climbs back up
(uz 1.0->1.9m), drifts sideways, touches down 3.6m off-target at t=14.5s.

`tools/analyze_explosion_chain.py` on that run gives a clean causal chain:
```
|w_u| pinned    t=6.64  axis=y   (body-rate command saturates FIRST)
|zeta| > 3.0    t=8.32  axis=y
|sigma| > 3.5   t=8.42  axis=y
|s_e_n| > 0.5   t=8.65  axis=y
|h_e|/p > 0.95  t=10.11 axis=x
kappa/k0 > 5    t=10.34 axis=x   (X-axis kappa ratchets ~8x, kappa_0=0.5 -> end=3.92)
|a_u| > 20      t=14.03 axis=y
final: h_meas_x=9.55 (rad/s, implausible), a_u_x=2239 (m/s^2, implausible)
```

This is `tune-plasmc` skill's documented **failure mode #11 (kappa-ratchet on sustained
off-center/saturated error)** — but that mode's known fixes (FLOW_FUSE_RING=0, various
kappa caps/Gamma/E/N/P levers) were all explored for the STATIONARY-target ArUco/cross-
marker controller, not this rover_cross/GT-feedback config's own heavily-customized gain
set (printed at every run start: `XI2_XY=1, P20_XY=15, P20_Z=10, P2INF_XY=1, OMEGA=0.1,
GAMMA_XY=0.25, E_Z=0.5, N=0.1, P_XY=2.5, KAPPA0_XY=0.5, KAPPA0_Z=0.25`).

**NOT yet investigated further** — per user decision (2026-08-28), stopped here rather
than starting a full n>=5 sweep campaign (the `tune-plasmc` skill's own methodology for
this failure class has historically taken hundreds of SITL runs). The IC1-5 + rover-motion
video batch is BLOCKED on this — re-running it now would just produce more of the same
divergence (now correctly detected as a failure instead of masked as a false landing).

## How to apply

- Before trusting ANY "on-platform"/"landed" video/data generated between 2026-08-26 (when
  the GT touchdown fix was first added) and 2026-08-28 (this depth-ceiling fix), re-verify —
  it may be a false touchdown at altitude, not genuine ground contact. This includes
  `montage_final_lowangle_touchdowncrop_cross.mp4` and everything from the batch this session
  (`montage_ic1_final.mp4`, `montage_ic3_final.mp4`, `montage_motion_*_final.mp4`) — none of
  these have been re-validated against the depth-ceiling-fixed detector.
- Next session on this thread: either (a) run the proper kappa-ratchet sweep campaign for
  rover_cross/GT-feedback (start with W_U_MAX — why does w_u saturate on Y at t=6.6s, well
  before the visible divergence — and a kappa cap on the X axis), or (b) if video recording
  is more urgent than fixing control, consider whether a LOOSER/more permissive on-platform
  acceptance criterion is acceptable for demo purposes vs a genuinely precise landing.
