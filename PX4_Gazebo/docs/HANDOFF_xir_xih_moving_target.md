# HANDOFF — Xi_r / Xi_h for moving-target landing (from MATLAB, 2026-09-10)

## TL;DR — one experiment to run in SITL

**Test `PLASMC_XIR_{X,Y} = 0.20` together with `PLASMC_XI2_{X,Y} = 0.20`** (currently 0.10 and 1.0).
Keep everything else at its baked value. Gate: the stationary IC1–5 must-not-regress **plus**
a moving-target run (rover, and/or the ArUco/cross world with a scripted target velocity).

MATLAB (realistic plant: pixel noise + ground effect + 1-step delay) says this pair is clean on
**both** regimes; `XIR` alone is not enough and `XI2=1.0` is the reason.

## ⛔ SITL RESULT (2026-09-11) — REFUTED, do NOT bake

Ran the exact experiment above: GT-FB, **rover world only** (per user), 2 arms
{base XIR0.10/XI2 1.0, cand XIR0.20/XI2 0.20} × 2 motion {moving Circular @ nominal,
static ROVER_MOTION=0} × 2 IC {IC2 = discriminator, IC1 = kappa-leakage canary}, n=3.
Harness + data: `test_data/XirXi2_RoverGTFB/`.

| cell | base (XIR0.10/XI2 1.0) | cand (XIR0.20/XI2 0.20) |
|---|---|---|
| moving IC2 | **3/3 precise**, xy 0.03–0.05, terminal s_e_n ~0.1, **converged, no growth-back** | 2/4 precise, xy →0.107, p_r collapses ~1.9, larger terminal s_e_n |
| moving IC1 | 2/3 precise, bounded (kappa_xy ≤1.22) | **1/3 DETONATION** — kappa_xy 0.5→30 (=cap), a_u_xy 1.3e5, xy 12 m |
| static IC2/IC1 | 3/3 soft+precise | 3/3 soft+precise, marginally worse xy |

**The MATLAB failure does not reproduce.** Baseline XIR=0.10 already lands 3/3 precise on
moving IC2 with `s_e_n` converged and *staying* converged through the terminal phase — the
predicted "terminal |s_e_xy| grows to 0.79 → FoV loss" is a noiseless-MATLAB artifact; SITL
flow lag / plant dynamics regulate the moving residual fine at the wide funnel.

**The candidate fails success-criterion #3.** Moving IC1 rep0 is exactly the
`project_ic1_kappa_leakage_drift` fly-away this doc warned about: XI2→0.20 drops terminal
G-exposure → `P_XY=2.5` over-leaky → κ drains → error grows inside the wide funnel → ratchet.

**Conclusion:** keep `PLASMC_XIR_{X,Y}=0.10`, `PLASMC_XI2_{X,Y}=1.0`. This is a
`feedback_gain_values_not_portable_either_direction` case — the funnel-deficit *mechanism*
has no SITL correlate. The moving-rover precision lever is not XIR/XI2. Any future XI2 drop
must walk `P_XY` down from 2.5 first with an IC1 re-check — but there is no SITL evidence of
the problem this was meant to solve. (Caveats on the refutation: n=3, GT-FB only — perception-ON
rover is independently blocked on detector collapse — single trajectory.)
Memory: `px4/feedback_xir_xi2_handoff_refuted_sitl`.

## Why — the mechanism (MATLAB-traced)

`p_r(t) = p_r_inf + (p_r0 − p_r_inf)·e^(−XIR·t)`, and **the position-funnel width *is* the barrier
gain** `G_r ≈ 2/p_r`. Wide funnel → weak position regulation.

- `XIR = 0.10` keeps `p_r` wide (~4 of 10 at t=10 s) the whole descent — the deliberate
  "keep `S_r = s_e_n/p_r` tiny so `zeta_r` never hits the steep tanh edge → no edge-forcing"
  strategy. **This is safe only when `s_e_n` converges on its own** — i.e. a *stationary* target.
- On a **moving** target `s_e_n` never converges (persistent chase error ∝ target speed). The weak
  wide funnel leaves it unregulated; MATLAB trace (IC2, Circular, realistic):

  | `|s_e_xy|` | XIR=0.10 | XIR=0.30 |
  |---|---|---|
  | t=2 s | 0.37 | 0.32 |
  | mid-descent | 0.17 | 0.15 |
  | **terminal** | **0.79 → FoV loss @ t≈7 s, xy 1.1 m** | **0.018 → land, xy 0.009 m** |

  Same early reduction; then `XIR=0.30`'s contracting funnel *squeezes* the residual to precision
  while `XIR=0.10` lets it grow back and walk the marker out of frame.
  `kappa` peaks at its cap early then leaks in both — **not** a kappa-ratchet, a pure funnel-gain
  deficit.

- **Speed dependence (your "moving too fast" hypothesis — confirmed):** with the full PX4 gain set,
  MATLAB lands soft-precise on all 4 moving trajectories at `speed_mult ≈ 0.3` and degrades/fails
  as speed rises to nominal. `XIR=0.10` works for *slow* motion; it can't track a fast deck.

## Why `XI2` (Xi_h) has to move too

MATLAB realistic, moving gate (5 IC × 4 moving traj = 20) | stationary (5 IC × 3 seed = 15):

| XIR | XI2=1.0 | XI2=0.20 |
|---|---|---|
| 0.10 | mov 4/20, stat 15/15 | mov 3/20, stat 15/15 |
| 0.20 | mov 10/20 (4 FoV), stat 14/14 | **mov 20/20, 0 FoV, stat 15/15** |
| 0.25 | mov 16/20 (4 FoV), stat 13/13 | mov 20/20, 0 FoV, stat 15/15 |
| 0.30 | mov 18/20 (2 FoV), **stat 12/12** ← regresses | mov 25/25 full gate (current MATLAB) |

With `XI2=1.0`, raising `XIR` trades moving for stationary — no clean point. With `XI2=0.20`,
`XIR ∈ [0.20, 0.30]` is clean on both. Full-gate confirm at `XIR=0.20, XI2=0.20`
(PX4 `chi_r=1.5`, `h_rd=−0.30` otherwise): **25/25 SP, 0 FoV** (mean t_f 16.7 s, worst xy 7.1 cm —
precision is marginal at these otherwise-PX4 gains; see "optional" below).

⚠ `PLASMC_XI2_XY = 1.0` was baked 2026-07-22 **paired with `PLASMC_P_XY = 2.5`** to bound
`kappa_eq = θ·G·|σ|/P` against the extra terminal G-exposure (see the `self._P` comment /
`project_ic1_kappa_leakage_drift_20260721`). If you drop `XI2` back to 0.20 the terminal
G-exposure drops with it, so `P_XY = 2.5` is probably over-leaky again — **re-check the IC1
kappa-leakage-drift fly-away** at `XI2=0.20`, and if it returns, sweep `P_XY` down toward the
MATLAB `[2.5, 2.5, 5.0]`… actually MATLAB already runs `P_XY = 2.5`, so this pairing may already
be fine. Watch `kappa_xy` and `a_u_xy` on IC1 specifically.

## Optional further gain (bigger departure from current PX4)

MATLAB's full manuscript config also uses `chi_r = 2.0` (PX4: 1.5) and `h_rd = −0.38` (PX4: −0.30).
Adding those to the candidate takes MATLAB from `worst xy 7.1 cm / t_f 16.7 s` → `3.5 cm / 11.4 s`.
`chi_r` and `h_rd` are documented SITL-timing divergences (SITL flow lag adds overshoot the
noiseless model lacks; slow descent gives short-runway ICs time to arrest) — don't move them
without their own A/B. Priority order for SITL: (1) `XIR 0.10→0.20` + `XI2 1.0→0.20`;
(2) if precision still short, `chi_r 1.5→2.0`; (3) `h_rd −0.30→−0.38` only if landing-time budget
allows and IC4/IC5 hold.

## Success criteria for the SITL run

1. Stationary IC1–5 (n≥5): no worse than the current gate (SP rate, 0 fly-aways, xy).
2. Moving target: `s_e_n` **converges and stays converged** through the terminal phase
   (the MATLAB discriminator — trace it, not just the SP verdict); no FoV breach; xy within δ_r.
3. `kappa_xy` bounded (≤ ~0.5, no ratchet) and `a_u_xy` bounded on IC1.

## Provenance

MATLAB R2025b, `run_simulation` (VDF-ASMC blocks), `VDF_OVERRIDE.theta_per_axis=1`, realistic
config `NOISE=GE=delay=1`, seed 1 (+ multi-seed for stationary). Reconciliation thread:
`[[project_matlab_yaw_rate_law_port_2026_09_09]]`, `[[feedback_gain_values_not_portable_either_direction]]`.
