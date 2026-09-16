---
name: project_20260916_curve_qgate_revalidation
description: "CURVED-TARGET re-validation of PLASMC_AU_LEAD_QGATE (2026-09-16), the load-bearing open question. TWO results: (1) ✅ BLOCKER CLEARED — the gate does NOT neuter the curve benefit (gated vs ungated lead: e_mean 0.253 vs 0.250 p=0.72, osc_std p=0.49, both 4/4 ON-PLATFORM; gate transmits ~33% in the tracking window, enough). (2) ⚠ PREMISE CHANGED — the curved-target limit cycle is GONE from the current baseline: no-lead is now 3/4 ON-PLATFORM, e_rot +0.05 (was +1.11), osc_std 0.033 (was 0.06-0.07), vs July's 0/4 at 1.0-1.7 m. AU_LEAD is now a modest refinement (e_mean 0.345->0.250, p=0.0001), not a rescue. Also: my offline pre-check predicting the gate would kill the curve was WRONG by 2x — it divided July 640x480 extents by today's 320x240 frame_min."
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

**NOT yet identified: what fixed it.** Prime suspects, all landed between 07-03 and now:
the two-tier visibility-projection rewrite (Tier-1 QP replacing the joint-QP/cone stack),
**`CBF_DRIFT_TAU=0.15` — itself a `τ·d` MOVING-TARGET lead term in the visibility QP**, the
`PLASMC_YAW_RATE_LAW` bake, `P2INF_X` → 2.5. Worth isolating: whichever one killed the
cycle is a more fundamental win than AU_LEAD, and if it is `CBF_DRIFT_TAU` then the two
mechanisms are doing the same job and AU_LEAD may be redundant.

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
