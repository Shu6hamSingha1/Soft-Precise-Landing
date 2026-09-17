---
name: project_20260916_curve_qgate_revalidation
description: "⛔ READ THE TOP BANNER FIRST. 2026-09-17: EVERY curve result in this file ran on a NON-CURVE. `ROVER_TRAJ=Circular` has not driven a real circle since commit b816fea0 (2026-07-03 12:13) changed ROVER_CIRCLE_R 0.8m->10m; measured target paths are a 9.4-12.1 m radius / 11-24 deg arc vs July's 0.86 m / 234 deg. DECISIVE: today's HEAD with ROVER_CIRCLE_R=0.8 gives median|e_rot|=0.70 (r_fit 0.88 m) = CYCLE RETURNS, in July's +0.58..+1.10 band, one rep missing by 7.20 m. ⇒ The curved-target limit cycle is ALIVE on current HEAD — never fixed, never re-tested. OVERTURNED: "cycle is gone", "AU_LEAD is redundant", "the QGATE does not neuter the curve" (p=0.72, measured on a straight path — question is OPEN again), and "the cause is not in the repo" (it IS b816fea0; my July anchor was picked by date and landed 11 h AFTER the change). SURVIVES: all stationary work (AU_LEAD regression, GT-FB refutation, KAPPA_DZ, 2-term QGATE) — no rover involved. To exercise the cycle you MUST set ROVER_CIRCLE_R=0.8; the default r=10 is a gentle-arc test PRELIMINARY r=0.8 re-run (2026-09-17, n=3-4/arm, IN PROGRESS): the gate suppresses the ungated lead's terminal peak |I_a_xy| (mean 1.58 vs 3.14, back near baseline 1.64) while keeping its tracking benefit (e_mean 0.347 vs 0.345 vs baseline 0.457) -- matches the July raw-lead detonation frontier. All arms landed 4/4 or 3/3 this run (unlike July baseline 0/4) so landing-rate is NOT yet a claim; top-up to n>=7-8/arm in progress. Also found+fixed a harness bug: a crashed rep silently re-read the prior rep's stale output dir (feedback_recurring_analysis_mistakes sec 17)."
metadata: 
  node_type: memory
  type: project
  originSessionId: 878fdadb-dd99-4085-bcf2-e19879f48082
  modified: 2026-09-16T14:33:53.570Z
---
---

# ⛔⛔⛔ READ THIS FIRST — 2026-09-17: EVERY CURVE RESULT BELOW WAS RUN ON A NON-CURVE

**`ROVER_TRAJ=Circular` has not driven a real circle since 2026-07-03 12:13.** Commit
**`b816fea0`** changed `ROVER_CIRCLE_R` **0.8 m → 10 m** (its own comment: *"the small-radius
circle (0.8 m, ~Ackermann min turn radius) made the TARGET motion jerky (steering saturated)
… only the path curvature drops ~12x"*). Measured from the GT target paths:

| | fitted radius | arc swept | path length |
|---|---|---|---|
| July cycling runs (11:14-11:48, pre-`b816fea0`) | **0.86 m** | **234°** | 3.53 m |
| every run today (r=10) | **9.4-12.1 m** | **11-24°** | 2.7-4.0 m |

A 12 m-radius 20° arc is a straight line. **The curve stimulus was absent from every
experiment in this file.**

**DECISIVE CONFIRMATION** — today's HEAD, unchanged, with `ROVER_CIRCLE_R=0.8`
(`test_data/Rover_Turning/r08_confirm/`, n=3): **`median|e_rot| = 0.70` (+0.47, +0.78,
+0.70), r_fit 0.88 m — CYCLE RETURNS**, in July's +0.58…+1.10 band, with rep 1 reproducing
the July failure signature (e_mean 1.75, osc_std 0.377, **7.20 m miss**).

**⇒ The curved-target limit cycle is ALIVE on current HEAD. It was never fixed — it was
never re-tested.** Everything below that says "the cycle is gone" is an artifact of the
radius change.

## What this OVERTURNS (all of it in this file)
- ⛔ "The curve limit cycle is GONE from the baseline" — **FALSE.** Alive at r=0.8.
- ⛔ "AU_LEAD is redundant / a refinement not a rescue" — **UNSUPPORTED.** The cycle it was
  built to damp is alive; AU_LEAD has not been tested against a real curve since July.
- ⛔ "The QGATE does not neuter the curve benefit" (e_mean 0.253 vs 0.250, p=0.72) — measured
  on a near-straight path. **The load-bearing question is OPEN again** and is now cheaply
  answerable: rerun the 3-arm A/B with `ROVER_CIRCLE_R=0.8`.
- ⛔ "The cause is NOT in the repository" — **FALSE.** It is `b816fea0`. My July anchor
  `edb546f0` was picked by date (`--before 2026-07-03 23:59`) and landed at **23:30, eleven
  hours AFTER** the 12:13 radius change, so the "endpoint doesn't reproduce" signal meant
  *my anchor was wrong*, not *out-of-repo*. I asserted the latter without excluding the former.
- ⛔ The 4-arm gain-revert sweep, the `d380901c` worktree test, and the 640×480 camera test
  are all **uninformative about the cycle** — every one ran on a non-curve. (The camera test
  remains a valid negative for the camera *specifically*, and the `d380901c` tail-variance
  observation stands as a tail/robustness result, just not a cycle result.)

## ⭐ PRELIMINARY r=0.8 QGATE RE-RUN (2026-09-17, small n, IN PROGRESS)

Re-ran the 3-arm A/B with `ROVER_CIRCLE_R=0.8` pinned explicitly (the actual cycle stimulus).
Every rep verified on the genuine curve (Kasa fit `r_fit` 0.86-0.91 m, swept 270-281 deg).
Bundles `test_data/Rover_Turning/qgate_revalidation_r08/{A_base,B_ungated,C_gated}`.

| arm | n | landing | touchdown lat | tracking e_mean | e_rot | osc_std | terminal peak `\|I_a_xy\|` |
|---|---|---|---|---|---|---|---|
| A base (no lead) | 4 | 4/4 ON-PLATFORM | 0.082 | 0.457 | +0.69 | 0.034 | mean 1.64, max 1.80 |
| B ungated lead | 4 | 4/4 ON-PLATFORM | 0.103 | **0.345** | +0.69 | 0.028 | **mean 3.14, max 4.03** |
| C gated lead | 3 | 3/3 ON-PLATFORM | 0.086 | **0.347** | +0.71 | 0.029 | mean 1.58, max 1.87 |

**Coherent, matching the July frontier this file already documented:** `e_rot` ~+0.69-0.71 in
ALL THREE arms including baseline (the rotating cycle is present regardless of the lead — it
is a tracking-error signature, not solely a pass/fail determinant at this n). AU_LEAD cuts
tracking error ~24% in both lead arms. The UNGATED lead pays the terminal-command cost the
July campaign found (raw lead peaked 65-1639 m/s^2, ratio=1.0 re-admitted instability) — peak
`|I_a_xy|` nearly 2x baseline. **The GATE suppresses that terminal spike back to baseline
levels (1.58 vs 1.64) while keeping the tracking benefit (0.347 vs 0.345)** — i.e. doing
exactly its intended job at the REAL curve, unlike the uninformative r=10 result above.

⚠ **NOT YET A FINDING — n=3-4/arm, ALL FOUR arms landed 4/4 or 3/3 this run**, unlike July's
baseline 0/4 and the earlier single r08_confirm run (1/3 missed by 7.2 m). Session-to-session
variance on this exact scenario is real and already documented (July: "coin-flip Circular
landings", "χ-vs-Wτ margin fluctuating rep to rep") — do not read 100% landing here as
"solved". The terminal-peak-command separation is the more trustworthy signal (continuous,
not binary). **Top-up run in progress (N=4 more/arm) to reach n>=7-8/arm before any claim.**

**⚠ Harness bug found and fixed mid-run:** C_gated rep4's SITL launch crashed on attempt 1
(`rc=1`, not classified retriable) and produced NO output directory, but the harness's
`d=$(ls -dt "$out"/*/ | head -1)` fallback silently RE-READ rep3's still-newest directory
instead of detecting the missing rep — rep3 and rep4 printed IDENTICAL numbers. Fixed with a
before/after directory snapshot (`before=$(ls -dt ... | head -1)` captured pre-run, compared
against post-run `$d`) in `scratchpad/curve_qgate_ab_r08.sh`. **General lesson**: any harness
that infers success by "did a new output directory appear" must snapshot BEFORE the run, not
just check for non-empty AFTER — an unlucky launch failure can leave the most-recent-directory
query pointing at a real (older) rep, which silently duplicates data instead of erroring.

## What SURVIVES
- **All stationary work is unaffected** — the AU_LEAD stationary regression, the GT-FB
  refutation of the κ-ratchet story, `PLASMC_KAPPA_DZ_*`, and the 2-term `AU_LEAD_QGATE`
  (18/25 ≈ baseline, IC5 0/5→5/5) were IC1-5 stationary tests with no rover involved.
  [[feedback_aulead_stationary_regresses]] [[feedback_adaptive_law_noise_behavior]]
- The statistical calibration (identical baseline spans 17-20/25) stands.
- Today's "curve lands 3/4" is really the **straight-line moving-target** case, which
  [[project_rover_speed_sweep]] already records as solved — consistent, not contradictory.

**How to apply:** `ROVER_TRAJ=Circular` at its default r=10 is a GENTLE-ARC test, not a
curvature test. **To exercise the turning-target limit cycle you must set
`ROVER_CIRCLE_R=0.8`** (and note the trade the r=10 comment records: at 0.8 m the Ackermann
steering saturates and target motion is jerky — so 0.8 tests the cycle, 10 tests smooth
moving-target tracking; they are different experiments and both are legitimate, but they are
NOT interchangeable). [[project_rover_turning_open]]

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

**⛔ PARTIALLY RETRACTED — see the worktree test at the end of this section. The
elimination sweep below is valid; the conclusion drawn from it was NOT.**

**Isolation sweep (valid):**

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

This SEEMED to match [[project_rover_turning_open]]'s OWN diagnosis of the cycle, which named
the cone as the amplitude-setting element: *"tau_ia+cone −9° but gain 0.43 (cone active 26-38 %
of samples — **the DF that caps growth**)"* and *"A·W*² = a_osc ≈ 1.0 m/s² CONSTANT across
reps → **amplitude is authority-set**"*. A limit cycle needs a nonlinearity to set its
amplitude; the old chattering cone WAS that nonlinearity. Replace it with a QP that sits
idle on a clean approach and the describing-function element sustaining the orbit is gone.

## ⛔ WORKTREE TEST REFUTES THE "REWRITE KILLED THE CYCLE" CONCLUSION (same day)

Ran the identical curve recipe (no lead, n=4) on a sparse worktree at **`d380901c`** — the
commit immediately BEFORE `82fa9c16` "wire in visibility_projection.py, retire the CBF
machinery". Verified the old path was live (`from cbf_visibility import cbf2_filter`;
measured `theta_cone` 0.358-0.478, i.e. exactly July's 0.345-0.473 band). Camera was
already 320×240 at that commit, so resolution is held constant.
Data `test_data/Rover_Turning/worktree_d380901c/`.

| | JULY (old stack, 640×480) | **OLD CODE TODAY (d380901c)** | NEW today |
|---|---|---|---|
| ON-PLATFORM | 0/4 | **1/4** | 3/4 |
| touchdown lat | 1.0-1.7 m | **2.88 m mean (0.018-5.90)** | 0.130 (0.030-0.352) |
| e_mean | ~0.70 | **1.26 (0.32-1.78)** | 0.345 (0.340-0.350) |
| osc_std | 0.06-0.07 | **0.199 (0.033-0.309)** | 0.033 (0.030-0.035) |
| **`e_rot`** (THE cycle signature) | **+1.11** | **+0.05…+0.21** | +0.03…+0.09 |
| `theta_cone` | 0.345-0.473 | 0.358-0.478 | 0.058-0.106 |

**THE KEY NEGATIVE: `e_rot` on the OLD code today is +0.05…+0.21, NOT July's +1.11.** The
rotating epicycle does **not** reproduce even with the old cone stack fully active at July's
magnitude. **⇒ the visibility-QP rewrite did NOT kill the rotating limit cycle.** Something
else between 2026-07-03 and 2026-09-09 did, and it is **still unidentified** (it is also not
any of the four gain reverts above, which were tested on today's code and came back null).

**What the worktree test DOES support:** the rewrite substantially improves curve
performance and, above all, **consistency** — but ⚠ **at n=4 nothing reaches significance**
(ON-PLATFORM 1/4 vs 3/4 Fisher p=0.486; e_mean Welch p=0.077 / MWU p=0.304; osc_std p=0.076;
lat p=0.117). The striking part is the **variance**, matching the peer's independent
stationary VisProjGate finding (*"median is a WASH… NEW wins the TAIL; the old stack
THRASHED on marginal approaches"*):

| spread across 4 reps | OLD | NEW |
|---|---|---|
| e_mean sd | 0.691 (0.32→1.78) | **0.006** (0.340→0.350) |
| osc_std sd | 0.125 | **0.002** |
| lat sd | 2.527 (0.018→5.90) | **0.150** |

The old stack is bimodal — one rep at 0.018 m, three at 1.8-5.9 m; the new one is
almost perfectly repeatable. So the rewrite's benefit on the curve is **tail/variance
elimination**, not cycle removal.

⇒ **Two distinct phenomena, not one:** (a) July's rotating limit cycle (`e_rot`≈1.11) — gone
in BOTH old and new code today, cause UNKNOWN; (b) the old CBF stack's tail-thrashing —
fixed by the rewrite. Conflating them was the error.

⇒ **AU_LEAD's redundancy claim is now WEAKER but still stands directionally**: it was built
to damp (a), and (a) is gone regardless of cause. It still buys ~27 % curve tracking error
(e_mean 0.345→0.250) on an already-working baseline. Do not bake it on the strength of a
mechanism story — the mechanism is not established.

**⚠ METHODOLOGICAL NOTE — this is the SECOND time this session an observational
log-diff mechanism claim was refuted by a controlled test:** the κ-ratchet story fell to the
GT-FB A/B, and the cone/DF story fell to this worktree. Both times the observational
evidence looked strong (correlations, magnitudes, matching prior analysis). **Treat
log-diff mechanism inferences as hypotheses to be tested, never as findings.**

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

## ⛔⛔ BISECT ABORTED — THE CAUSE IS NOT IN THE REPOSITORY (2026-09-16)

Attempted to bisect Jul3→Sep9 for what killed the rotating cycle. **Validated the endpoints
first, and the July endpoint FAILED to reproduce**, which invalidates the whole bisect:

**Ran the EXACT July commit `edb546f0` (2026-07-03 — the commit that produced the cycling
data) on a worktree TODAY, same curve recipe, n=2:**
`NO-CYCLE, median |e_rot| = 0.13` (reps +0.13, +0.13) — versus that same commit's own
archived July data at **e_rot +0.58…+1.10, 27/27 cycle reps**.

**⇒ Same commit. Cycle in July, no cycle today. The cause is OUTSIDE the git repo.**
A commit bisect would have falsely converged on the earliest commit. Do not attempt it.

**Leading hypothesis: the camera SDF** (`~/PX4-Autopilot/Tools/simulation/gz/models/
mono_cam/model.sdf`, OUTSIDE the repo), changed **2026-08-27 from 640×480/fx=270 to
320×240/fx=135**. It is the one clear, dated environmental difference: `Img_Params`
records `resolution (480,640)` for every cycling July run and `(240,320)` for every
non-cycling run today (July-code-today, d380901c, HEAD). Logged `FPS` is 62.5 in all (that
is the capture rate, not the achieved `process_frame` rate, so it does not discriminate).

Mechanism fit is strong: the cycle was characterised as **lag-pumped** — *"anti-position
command on a circulating error + ANY lag → pumps ∝ sin(Wτ)"*, damping needs *"χ > Wτ ≈
25-40°"*, and *"K_R=2.5 worked because it cut Wτ (real phase)"*
([[project_rover_turning_open]]). CLAUDE.md records that the 640×480→320×240 drop was made
precisely to recover `process_frame()` rate (~15-23 Hz → ~38 Hz). **Halving the image
workload ≈ halving perception latency ≈ cutting τ — which flips the `χ > Wτ` pump
condition.** That would kill the cycle without any controller change, exactly as observed.

**Other out-of-repo candidates not yet excluded:** the Gazebo world/marker assets
(`cross_marker.png` texture changed 08-09; `rover_cross.sdf`/`cross_marker.sdf` have dated
`.bak`s), PX4-Autopilot version/params, Gazebo/ROS versions, host load.

**DECISIVE TEST (not yet run, needs a temporary edit to the SHARED out-of-repo PX4 install):**
restore `mono_cam/model.sdf` to 640×480/fx=270, run today's HEAD on the same curve recipe
n=2-3. Cycle returns ⇒ confirmed. Back up and restore the SDF (trap on exit) — it is shared
with other sessions.

**How to apply:** ⚠ **this project's behaviour depends on out-of-repo state** (camera SDF,
world/marker assets, PX4 version). A git worktree does NOT reconstruct a historical
experiment. Before attributing any behaviour change to a commit, verify the old commit
still reproduces the old behaviour — endpoint validation is mandatory, not optional.
