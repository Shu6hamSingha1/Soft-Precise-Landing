---
name: project_20260916_curve_qgate_revalidation
description: "CURVED-TARGET re-validation of PLASMC_AU_LEAD_QGATE (2026-09-16), the load-bearing open question. TWO results: (1) ✅ BLOCKER CLEARED — the gate does NOT neuter the curve benefit (gated vs ungated lead: e_mean 0.253 vs 0.250 p=0.72, osc_std p=0.49, both 4/4 ON-PLATFORM; gate transmits ~33% in the tracking window, enough). (2) ⚠ PREMISE CHANGED — the curved-target limit cycle is GONE from the current baseline: no-lead is now 3/4 ON-PLATFORM, e_rot +0.05 (was +1.11), osc_std 0.033 (was 0.06-0.07), vs July's 0/4 at 1.0-1.7 m. AU_LEAD is now a modest refinement (e_mean 0.345->0.250, p=0.0001), not a rescue. (3) ✅ CAUSE IDENTIFIED: the TWO-TIER VISIBILITY-QP REWRITE killed the cycle — all four env-togglable gain reverts (CBF_DRIFT_TAU=0, P_xy=1.5, P2INF=1.0, XI2=0.7) are NULL (13/13 ON-PLATFORM, no cycle), and the old cone rotated the lateral cmd 0.345-0.473 rad at 100% duty vs the new QP's 0.058-0.106 rad / 0-16% active — the old cone WAS the cycle's amplitude-setting describing-function element. ⇒ AU_LEAD is very likely REDUNDANT. Also: my offline pre-check predicting the gate would kill the curve was WRONG by 2x — it divided July 640x480 extents by today's 320x240 frame_min."
metadata: 
  node_type: memory
  type: project
  originSessionId: 878fdadb-dd99-4085-bcf2-e19879f48082
  modified: 2026-09-16T14:33:53.570Z
---

**Closes the last load-bearing open item from [[feedback_aulead_stationary_regresses]] /
[[feedback_session_20260909_12_audit]].** Config copied verbatim from the campaign harness
that produced the original "BEST CURVED CONFIG" (`Rover_AB_harness/aulead_commitoff_arms.sh`):
GT-FB, heading-hold (`YAW_{GAMMA,KAPPA0,OMEGA,N}=0`, `ALPHA_FILT=0`), `ROVER_TRAJ=Circular`,
`SPEED_MULT=1.0`, `TERMINAL_COMMIT=0`, lead `wz=0.9/wp=3.5 r=0.5`. n=4/arm.
Data `test_data/Rover_Turning/qgate_revalidation/{A_base,B_ungated,C_gated}`.

## ✅ RESULT 1 — the gate does NOT neuter the curve. BLOCKER CLEARED.

| arm | ON-PLATFORM | touchdown lat (m) | e_mean (m) | osc_std | qg_total (0.8-3.5 m) |
|---|---|---|---|---|---|
| A no lead | 3/4 | 0.130 | 0.345 | 0.0328 | — |
| B lead, `QGATE=0` | **4/4** | 0.022 | **0.250** | 0.0310 | — |
| C lead, `QGATE=1` | **4/4** | 0.056 | **0.253** | 0.0300 | **0.331** |

**Gated vs ungated lead: e_mean p=0.72, touchdown lat p=0.16, osc_std p=0.49 — all
non-significant.** The gate transmits ~33 % of the lead through the 0.8-3.5 m tracking
window (qg_mag 0.73 × qg_ext 0.33) and that is enough to retain the full benefit. The
feared "magnitude gate suppresses the curve's sustained |I_a_raw|" conflict **does not
materialise** — measured tracking-window `|I_a_raw|` median is only **0.44-0.49**, not the
1.0-1.5 the old memory recorded, so the magnitude term sits mostly open (0.73).

⇒ **`PLASMC_AU_LEAD` + both `QGATE` terms is now defensible for the rover scenario**:
helps the curve (below), costs nothing measurable there, and restores stationary parity
([[feedback_aulead_stationary_regresses]]).

## ⚠ RESULT 2 — the PREMISE changed: the curve limit cycle is GONE from the baseline

| metric (GT-derived, so directly comparable across the camera change) | July 2026-07-03 | **today** |
|---|---|---|
| no-lead ON-PLATFORM | **0/4** | **3/4** |
| no-lead touchdown lat | 1.0-1.7 m | **0.130 m** |
| `e_rot` (epicycle rotation = THE cycle signature) | **+1.11 rad/s** | **+0.03…+0.09** |
| `osc_std` | 0.06-0.07 | **0.033** |
| `e_mean` | ~0.70 | **0.345** |

**The self-sustained rotating lateral limit cycle that AU_LEAD was designed to damp
([[project_rover_turning_open]]) is essentially absent on the current stack.** `e_rot`
collapsing from +1.11 to ~+0.05 is the decisive number — that IS the cycle.

So AU_LEAD is no longer a rescue, it is a **modest refinement of an already-working case**:
e_mean 0.345 → 0.250 (**−27 %, p=0.0001**, very tight data), ON-PLATFORM 3/4 → 4/4
(ns at n=4). Real, but nothing like the July 0/4 → 2/3 step.

**✅ IDENTIFIED (same day, isolation sweep): the TWO-TIER VISIBILITY-QP REWRITE killed it.**

Diffed July's vs today's recorded `Control_Params` on the curve → exactly four
env-togglable lateral-loop changes. Reverted each ONE AT A TIME, no lead, GT-FB Circular
(`test_data/Rover_Turning/cycle_isolation/`):

| arm | revert | result |
|---|---|---|
| D | `CBF_DRIFT_TAU` 0.15→0 | 4/4 ON-PLATFORM, e_rot +0.05…+0.10 — **null** |
| E | `P_xy` 2.5→1.5 (κ leakage) | 3/3 ON-PLATFORM, e_rot +0.07…+0.08 — **null** |
| F | `P2INF_xy` 2.5→1.0 (funnel floor) | 3/3 ON-PLATFORM, e_rot +0.03…+0.08 — **null** |
| G | `XI2_xy` 1.0→0.7 | 3/3 ON-PLATFORM, e_rot +0.05…+0.07 — **null** |

**13/13 ON-PLATFORM, zero cycle in any arm** (e_rot never above +0.10, osc_std 0.029-0.039,
lat 0.017-0.080 m). None of the gain bakes did it. ⇒ by elimination the cause is the one
NON-env-togglable change: **the two-tier visibility-projection rewrite** (Tier-1 QP
replacing the `rho_fov`/`theta_cone`/joint-QP stack).

**POSITIVE mechanism evidence, not just elimination** — cone activity in the 0.8-3.5 m
tracking window:

| | JULY (old CBF) | TODAY (new QP) |
|---|---|---|
| `theta_cone` mean | **0.345-0.473 rad (20-27°)** | **0.058-0.106 rad (3-6°)** — 5-8× smaller |
| `vis_active` | n/a | **0-16 %** (0 % in 2 of 4 reps) |
| `rho_fov` | 358, always present | channel gone |

This matches [[project_rover_turning_open]]'s OWN diagnosis of the cycle, which named the
cone as the amplitude-setting element: *"tau_ia+cone −9° but gain 0.43 (cone active 26-38 %
of samples — **the DF that caps growth**)"* and *"A·W*² = a_osc ≈ 1.0 m/s² CONSTANT across
reps → **amplitude is authority-set**"*. A limit cycle needs a nonlinearity to set its
amplitude; the old chattering cone WAS that nonlinearity. Replace it with a QP that sits
idle on a clean approach and the describing-function element sustaining the orbit is gone.

⚠ Caveat: `theta_cone(t)`'s computation may itself have changed in the rewrite, so treat
the 5-8× as corroboration rather than a like-for-like measurement. The decisive evidence is
the 13/13 null elimination plus the structural replacement. **Gold-standard confirmation
still available if wanted: a worktree at the pre-rewrite commit** (`d380901c`, the OLD arm
of the VisProjGate A/B) run on the same curve recipe.

⇒ **AU_LEAD is very likely REDUNDANT.** It was built to damp a cycle that the visibility-QP
rewrite already removed. It still buys ~27 % tracking error on the curve (e_mean 0.345→0.250),
but that is a refinement of a solved case, not the rescue it was designed to be — weigh that
against its complexity (two gate terms, four knobs) before any bake.

(`PLASMC_YAW_RATE_LAW` was never a candidate here: this curve recipe runs heading-hold with
`YAW_{GAMMA,KAPPA0,OMEGA,N}=0`, so the yaw law is inert.)

## ⛔ MY OFFLINE PRE-CHECK WAS WRONG — 2× resolution error

Before running I predicted from the archived July reps that the gate would transmit
**<1.5 %** (`qg_ext`≈0.013) and kill the curve. **Wrong.** I divided July-era
`MARKER_EXTENT_PX` by **today's** `frame_min=240`, but July ran at **(480,640)** →
`frame_min=480` (today is (240,320) → 240, after the 2026-08-27 camera drop). July fill was
246/480 = **0.51** (below `QGATE_LO=0.55`, gate OPEN), not 246/240 = 1.02 (gate closed).
**Every pixel-domain quantity must be renormalised across the 2026-08-27 640×480→320×240
change before any cross-epoch comparison** — the same trap CLAUDE.md flags for `rho_fov`
and the sensor cal. GT-derived metrics (`e_mean`, `e_rot`, `osc_std`, lat) are resolution-
independent and ARE safe to compare across that boundary.

## Caveats
- n=4/arm, one session; the binary ON-PLATFORM contrasts are underpowered (only the
  continuous `e_mean` separation is significant). Per [[feedback_session_20260909_12_audit]],
  do not read the 3/4-vs-4/4 as an effect on its own.
- **GT-FB only.** Real rover *perception* is still broken (cluster A/B, 0/5), so this is a
  control-layer result; a real-perception rover pass is still required before any bake.
- `TERMINAL_COMMIT=0` as per the campaign recipe (it is the baked default anyway).

**How to apply:** the gate/curve conflict is settled — stop treating it as a blocker. The
open questions are now (1) what actually killed the curve cycle, and (2) is AU_LEAD still
worth its complexity given the baseline already lands. [[project_rover_turning_open]]
