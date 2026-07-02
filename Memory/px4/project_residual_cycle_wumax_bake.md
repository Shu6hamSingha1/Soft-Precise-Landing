---
name: project_residual_cycle_wumax_bake
description: "Post-Z_REG residual terminal limit cycle fully characterized (GT-FB): drift term chi_r*zeta_dot_r/G is the a_u oscillation driver (s_dot_meas, single impact axis); W_U_MAX=2.0 BAKED (clamp discontinuity SEEDS the cycle); P2INF_xy un-saturates zeta_h -> 0 fly-aways; IC5 = largest-error+least-runway; next lever = terminal descent pacing."
metadata:
  node_type: memory
  type: project
  originSessionId: a378d3e9-67aa-42fc-ae09-63da27f370a9
---

**2026-06-30 (user-led GT-FB session). After Z_REG=0.2 killed the unbounded-1/Z ARTIFACT
([[feedback_zreg_gear_floor_artifact]]), what remains is a small RESIDUAL terminal limit
cycle. This session characterized it end-to-end and landed one bake.**

## What the residual is (VALIDATED)
The not-soft / fly reps are a **terminal lateral LIMIT CYCLE** — a ~0.8–1.4 Hz oscillation in
`s_e_n` (3–6 zero-crossings) and `v_x` (13–18), matching the lateral double-integrator + 38 ms-lag
+ 1/Z loop frequency. Bounded by Z_REG=0.2 (1/Z capped ~5) but **ignites in the last <0.25 m** where
`1/Z≈4–5` is still enough loop gain. SOFT reps have the SAME loop, damped to nothing (`s_e_n` 1
crossing, v_lat 0.03). Caveat: terminal window is short (touchdown), so it's the cycle *igniting/
growing*, not a proven steady-state oscillation.

## THE a_u OSCILLATION DRIVER = the drift term `chi_r·ζ̇_r/G` (NOT c-term / switching)
a_u decomposition (terminal, IC5r1 not-soft vs IC4r2 soft): drift amp **25.7** dominates (c-term 11,
switching 9.6, reaching/funnel ~0). Decomposing the drift: the oscillation is **entirely in
`dr_bar_e = s_dot_meas/p_10`** (the measured bearing rate), amp 64 — NOT `g_r` (flat 0.33–0.47,
`s_e_n` well inside the funnel `S_r≈0.1`, **no barrier-edge blowup**) and NOT `1/G` (amp ~0.8). It's
a **raw velocity-MEASUREMENT feedback**: `s=lat/Z` → `s_dot` carries the 1/Z²-amplified terminal
centroid rate → rings → drift → a_u → pumps the cycle. Concentrated on the **single impact axis**
(IC5r1=X, IC4r3=Y; orthogonal ~10–15× smaller — matches "impact axis = explosion axis").
- **Filters REPORT the cycle, don't generate it.** `s_dot_meas` (VDS-KF) AND flow `h_x` (amp 3.6,
  22 crossings) both oscillate at the cycle freq; `dh_d` (DHD-KF) big-but-SLOW swings (3–5 cross);
  `dw` (DW-KF) xy-zeroed. So smoothing ONE channel (VDS_KF_Q down) can't fix a physical oscillation
  that enters via multiple channels (h still hits `ζ_h`). VDS-KF is tuned LOW-LAG (q=10,R=1e-3) so it
  passes the oscillation through; q-sweep was re-run but is lower-priority by this logic.

## BAKE: W_U_MAX 1.0 -> 2.0 (controller.py:2188 + 1781)
The **1.0 body-rate clamp's DISCONTINUITY SEEDS the cycle** (clamp→bang-bang→bigger cmd→more clamp;
biting ~24% terminal on bad reps). Raising to 2.0 (above where the cmd lands) → discontinuity never
fires → cycle not seeded → smooth settle → SP. **The cmd is LOWER at 2.0, not higher** (IC5r2 SP:
terminal |w_u| max 0.28 ≪ 2.0, 0% clamped, rel 0.545→0.0075) — REFUTES the "throttled-brake"
hypothesis; the clamp was a cycle DRIVER. 2.0 still bounds the worst chatter (unlike the OLD
1000-uncap that grew the cycle in the Z_REG=0.01 regime). LK-perception-safe. Result on P2INF=1.5:
**3 SP** (IC4r1, IC4r3, **IC5r2** = first IC5 SP). Partly stochastic (IC5r1/r3 still borderline — the
cycle has the OTHER driver, the s_dot_meas/drift).

> **⛔ 2026-06-30 UPDATE — the P2INF=1.5 claim below is REVERSED at the new baked config.** A/B at
> {h_rd=-0.30, XIR=0.15} (IC2/IC4/IC5 n=3): **P2INF=1.0 = 7/9 SP, rel_vel med 0.026, 0 fly** vs **1.5 =
> 5/9, 0.144, 1 fly** vs **2.0 = 5/9, 0.172**. So **P2INF=1.0 is BEST** — kept baked 1.0 (briefly baked 1.5
> 2026-06-30 then reverted). WHY the reversal: at this config the terminal flow error h_e is large enough
> that zeta_h SATURATES (3.66) even at P2INF=1.5 (verified) -> the "un-saturate zeta_h" benefit is GONE;
> only the cost remains (wider p_2 = smaller/shallower zeta_h = WEAKER velocity damping in the run-up).
> Smaller p_2 = steeper zeta_h = stronger terminal velocity damping (the [[feedback_flow_funnel_zetah_works]]
> lever). NON-monotonic (1.5 worse than 2.0) = n=3 IC4-stochastic noise; the solid signal is 1.0 >= both.
> The 2 P2INF=1.0 non-SPs (IC4r1/IC5r1) are NOT a distinct mode (no containment/no zeta_r breach/same zeta_h
> sat as the SP reps) — they're the high tail of a high-VARIANCE terminal velocity (rel 0.007-0.563 across
> 9 reps) = the SOFTNESS wall (marginally-damped loop -> phase-sensitive touchdown vel). Current baked config
> = {h_rd=-0.30, XIR=0.15, P2INF=1.0} UNCOMMITTED; needs a full IC1-5 n=5 gate (D-gate 20/25 was at P2INF=1.5).

## P2INF_xy (flow funnel) = the fly-away fix (env, NOT baked) [SUPERSEDED — see banner above]
Widening the flow funnel floor **un-saturates `ζ_h`** → breaks the chain
`v_lat→h_e>p_h→ζ_h pins@3.66→σ out of layer→bang-bang→κ-runaway→FLY`. P2INF_xy 2.0/3.0: **3 SP, 0
fly** (vs baked 1.0 = flies). P2INF_xy=1.5 = tightest un-saturated → **stronger ζ_h damping** (mean
terminal v_lat 0.4→0.23, several reps rel 0.01–0.04) BUT trades into a **soft↔precise** frontier (2
SP; stronger velocity damping de-prioritizes centering, χ_r maxed at 1.5 by the 38 ms lag). NOT yet
baked — open which P2INF_xy value + gate.

## Why IC5 fails (CORRECTED — NOT the funnel ceiling)
IC5 = same (2,2) offset, 3 m alt → `s_e_n0=lat/Z=0.93` (vs IC2 0.56, IC4 0.42). This is the LARGEST
normalized error but **NOT near any funnel edge** (funnel starts `PR0=10` → `S_r=0.93/10≈0.09`; the
`p_r∞≈1` Standing-Condition is the FLOOR, not the initial width). Real cause: largest error + LEAST
runway (3 m, 8 s) → controller centers POSITION at HIGH velocity (`v_lat=2.32` when `s_e_n` first
<0.3, vs IC4 0.05) → **overshoots through center** → cycle. Mean 1/Z is FLAT (~3) across ICs — NOT a
1/Z-gain effect. The lateral velocity arrest is **lag-limited AND decoupled from position**; the
descent paces on `s_e_n` (position), which centers while `v_lat` is still high → descends into the
terminal zone un-arrested.

## Ruled OUT this session (corrections to my own mis-reads)
- **w×s / yaw is NOT the driver**: `h_d`'s transport uses `w_i` (V-frame, **xy-zeroed** by
  CTRL_ZERO_WXY), yaw-only & quiescent → 1–2% of `h_e`. The yaw-rate spike (w_i_z→4.9) is a
  POST-onset CONSEQUENCE (+0.86–2.28 s after σ onset), not a cause. (I wrongly used the body rate
  `_w` instead of `w_i`.) `h_d` formula itself is correct vs MATLAB.
- **CBF cone clamp NOT binding** in the terminal re-accel: real a_u cut only 4–15% (cmd within the
  cone, tilt 6–17° < cone 14–18°). The "13–18% cone_clamp" saturation-audit metric was DEGENERATE
  (counts centered-near-zero θ_cone≈θ_cmd≈0 cases; soft rep showed 65% with θ_cone=1°).
- **Terminal commit fires ONLY on the reps that succeed** (need centered+settled; not-soft reps
  oscillate → never settle → never commit). The commit's `s_e_n→0` ramp zeros `ζ̇_r`/drift at the
  source → the committed (SP) reps stay WITHIN p_r (no breach) with drift=0. So "zero the drift
  post-commit" is INERT (already 0); the lever would be zero-drift-on-CENTERED (drop the settled
  latch) to catch the oscillating-but-centered not-soft reps — UNTESTED.

## NEXT LEVER (in progress): terminal descent PACING (scale-free)
The descent gate `dgate` (controller.py:96–100, 1325–1335) paces `h_ref` on `|s_e_n|` ONLY
(slo=0.4/shi=0.8/gmin=0.15/tau=0.5). It RELEASES when position centers — but velocity isn't arrested
yet (lag) → descends with residual v_lat → cycle. Proposed: make the gate **velocity-aware** (also
slow on `|s_dot_meas|`/`|h_e_xy|`, both scale-free) so the descent WAITS for the lateral velocity to
settle before the terminal 1/Z zone. Respects [[feedback_scale_free_depth_free]] (image-space
signals only). Pairs with the W_U_MAX clamp-seeding fix as the two source-level attacks on the cycle.

Baked state this session: Z_REG=0.2, **W_U_MAX=2.0 (NEW)**, Γ=0.25, PR0=10, XIR=0.10, chi_r=1.5,
N_xy=0.1, TERMINAL_COMMIT on. P2INF_xy candidate (1.5 damping vs 2.0/3.0 no-fly) OPEN.
Supersedes the terminal-cycle "fundamental wall" framings; refines [[feedback_zreg_gear_floor_artifact]],
[[feedback_terminal_smc_actuator_wall]], [[feedback_flow_funnel_zetah_works]].
