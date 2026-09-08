---
name: project-yaw-rate-law-sign-bug-and-validation
description: "PLASMC_YAW_RATE_LAW (new yaw control, direct integrator on w_z bypassing psi_d/e_R) — a real actuation-chain sign bug was found+fixed via measurement; GT-feedback validation is a clean win INCLUDING beyond the old sin(dpsi) ceiling; real-perception is NOT yet safe (w_z inherits terminal-overfill corruption) — needs a confidence gate next."
metadata: 
  node_type: memory
  type: project
  originSessionId: 0f4a1549-4ee5-4e61-9344-dfa2c3a8081c
  modified: 2026-09-08T13:53:06.315Z
---

**STATE as of 2026-09-05: `PLASMC_YAW_RATE_LAW` exists in `controller.py`, default OFF, GT-feedback
VALIDATED (clean win, including beyond the old ceiling), REAL-PERCEPTION NOT YET SAFE (known,
diagnosed cause). This is the natural pick-up point for a new session.**

## The design (context: [[project_q8_omega_d_ff_fixes_ceiling]], [[project_q8_yaw_ff_harmful_with_headroom]])

User's redesign brief: use `α` and `ω` (specifically `w_z`, the flow-lstsq rotation component,
`self._w_i[-1][2]`) directly, without ever computing `ω_t` as a named quantity. Kinematic identity
(validated in `gt_feedback.py`, `-0.91` correlation with real body yaw rate): `e_a_dot = -w_z`
exactly — `w_z` already IS `e_a`'s derivative, flow-measured, no differentiation of `α` needed.
Direct integrator: `d(w_u[2])/dt = k_p·e_a − w_z` (see the ⚠ SIGN note below — this is the
CORRECTED form). One integrator only (unlike the ASMC's separate `ie_a`), so nothing to
double-count against. Bypasses `psi_d`/`e_R[2]`/`-K_R·sin(Δψ)` for yaw entirely — `psi_d := yaw_c`
is set directly in `_attCtrl` so `R_d`'s heading basis (needed for roll/pitch IK) stays correct.

Implementation: `controller.py` `__init__` (env reads + derivation comment, ~line 581),
`_yawCtrl` (computation, unconditional — runs alongside the ASMC for comparison even when not
driving output), `_attCtrl` (application — REPLACES `w_u[2]` entirely, mutually exclusive with
`OMEGA_D_FF`). Knobs: `PLASMC_YAW_RATE_LAW` (default 0), `PLASMC_YAW_RL_KP` (default 0.3),
`PLASMC_YAW_RL_KI` (default 0.0 — light robustness term, off by default). Logged:
`yaw_rl_cmd(t)`, `yaw_rl_ie(t)`. Recorded in `Control_Params.npy`'s resolved config.

## ⚠⚠ SIGN BUG found and fixed 2026-09-05 — read before touching this law again

**The actuation chain from `w_u[2]` to the ACHIEVED body yaw rate is INVERTED in this codebase.**
Measured directly (GT poses, independent of the law's own state): commanding `w_u[2]=+2.0 rad/s`
(saturated) produced an ACTUAL drone yaw rate of **−2.03 rad/s**. `psi_dot_b_TRUE ≈ -w_u[2]`, not
`+w_u[2]`. Substituting into `w_z = ω_t,z − ψ̇_b` (itself confirmed correct — independently verified
by integrating `-w_z` against measured `e_a` on an unrelated rep, trend matched) gives
`w_z = ω_t,z + w_u[2]` — a POSITIVE gain from command to `w_z`. The FIRST-WRITTEN law
(`Δw_u2 = w_z − k_p·e_a`) put `w_u2` on the RHS of its own update with coefficient **+1**
(`dw_u2/dt = w_u2 + …`), an unstable ODE — exactly the observed runaway to saturation
(first SITL attempt: `e_a` steady-state −39° with `max|a_u|` up to 1663, rate tracking −421%).
**Corrected form: `Δw_u2 = k_p·e_a − w_z`** (both terms flip, not one — re-derived cleanly from
the confirmed relation, not patched ad hoc).

**How this was found — worth the process note.** Two intermediate hypotheses were tried and
discarded before landing on the real cause, each ruled out by direct evidence rather than more
guessing: (1) closed-loop sample correlation between the command and `w_z` — invalid methodology,
confounded (both signals are functions of each other through the law itself); (2) my own
hand-rolled quaternion→yaw formula being wrong — ruled out by comparing against this project's own
`ahrs.Quaternion([w,x,y,z]).to_angles()[2]` convention on the same samples, matched to 1e-15 deg.
The decisive check was measuring the ACTUAL yaw rate from raw GT poses during a saturated window
and comparing directly against the command — cheap, independent, unambiguous.

**Offline verification before every SITL attempt** (the discipline that kept this cheap): a
closed-loop Euler-integration simulator mirroring the exact discrete update, tested against
whatever plant relationship was believed true at each point. It caught nothing the first two times
(both simulator runs "converged" — they were self-consistent with their own wrong assumptions, not
validated against reality) — the lesson: **an offline sim only proves your algebra is internally
consistent; it does not prove the assumed plant relationship is the real one.** Only a real SITL
measurement settled it.

## Results — GT-feedback: a clean, decisive win

n=3/arm (except the smoke tests, n=1), attribution-verified
(`test_data/YawRateLaw/{ceiling048,beyond060}`), cross-marker, `PLASMC_GT_FEEDBACK=1`.

| test | `e_a` steady-state | `max\|a_u\|` | xy_err |
|---|---|---|---|
| ceiling 0.48 rad/s, `off` (baseline ASMC) | −137 to −142° | 6–447 | 0.60–2.74 |
| **ceiling 0.48 rad/s, new law** | **−0.5 to −0.9°** | 1.9–6.2 | 0.03–0.27 |
| **0.60 rad/s — unreachable by ANY prior mechanism** | **−0.6 to −1.0°** | 2.0–2.7 | **0.11–0.12** |

The 0.60 rad/s result is the one that matters most: `OMEGA_D_FF` and every `e_R`-routed command
are hard-capped by `K_R_yaw·|sin Δψ| ≤ 0.5 rad/s` — this is a genuinely new capability, not an
incremental improvement, and it's tight/repeatable (not a lucky single rep).

## ⛔ Real perception: NOT yet safe — diagnosed, not a design flaw

Stationary regression check (`test_data/YawRateLaw/stationary/`, real perception, no GT-FB,
`PLASMC_YAW_RATE_LAW=1`, IC1-5 n=1): IC4 regressed badly — `yaw_rl_cmd` saturated to −2.0,
xy=0.741 (vs the OLD/buggy-sign smoke test's 0.166 on a similar config, i.e. genuinely worse than
even the broken version, because THAT one happened to be locally stable for ω_t=0 specifically).

**Root cause, confirmed via direct correlation with `MARKER_EXTENT_PX` and altitude**: real
(unfiltered) `w_z` tracks marker extent almost exactly — stays small while `extent<~250px`, then
grows monotonically as extent saturates near the frame (318px) and altitude drops below ~1m,
reaching >1.0 rad/s on a target that is not rotating at all. **This is the documented terminal-
overfill mechanism** (already the #1 open blocker for `h_y`, [[project_20260901_rover_cross_perception_diagnosis]])
extended to a NEW channel (`w_z`) that had not previously been load-bearing for anything, so this
corruption mode was never exposed. The OLD ASMC path happened to be incidentally protected — its
damped/adaptive structure reacts more slowly to one bad signal than a pure unfiltered integrator.

**This is not a flaw in the sign fix or the design** — GT-feedback (exact `w_z`, no corruption
possible) validates cleanly. It's that the new law has zero filtering and zero confidence gating
on its one input, inheriting real perception's known failure mode directly.

## Confidence gate BUILT + first SITL run (2026-09-07) — gate works, law still not viable on real perception

`controller.py` `PLASMC_YAW_RL_GATE` (default ON when the law is on). Freezes `_yaw_rl_cmd` +
`_yaw_rl_ie` when w_z is untrusted: either `|w_z| > PLASMC_YAW_RL_WZ_MAX` (0.9) OR overfill
(`MARKER_EXTENT_PX` ≥ `EXT_FRAC`·running-max AND ≥ `EXT_ABS` px; defaults 0.9 / 280). Logs
`yaw_rl_gated(t)`. Params in `_buildLogDict` (Control_Data), not Control_Params.npy.

IC1-5 n=3 headless, real perception, `PLASMC_YAW_RATE_LAW=1` (`test_data/ICValidation/20260907-161500`):
**15/15 land, 0 TL, `yaw_rl_cmd` never saturates (max ~1.7, no −2.0 runaway)** — the narrow goal
(kill the saturation blow-up the pre-gate stationary check showed) is met. But NOT a pass:
- xy_mean IC1 0.31 / IC2 0.21 / IC3 0.24 / IC4 0.77 / IC5 0.57; 0/15 precise|soft. Worse than the
  baseline ASMC stationary gate (`20260831-144626`, IC2/3/4 ~0.10 m).
- **`e_a` diverges to −50°…−110° BEFORE overfill** — at the frame the gate first fires, `e_a` is
  already −58° to −96° (IC4 rep1 −96° @93% through; IC2 rep1 −58° @66%). The real-perception
  failure is mostly UPSTREAM of the terminal corruption the gate targets: noisy/biased real w_z,
  not just the extent-tracking blow-up. GT-FB (exact w_z) → e_a −0.5°; real → law never converges.
- **Freeze semantics are wrong when the law is mid-correction**: it holds the last `_yaw_rl_cmd`,
  which can be large (IC1 rep3 froze at 1.678 rad/s, IC5 rep3 at 0.375) → the drone keeps yawing
  while frozen → `e_a` drifts further after the gate. Should ramp cmd→0 on gate, not hold.
- Gate onset a touch late (ext ≈284 vs max ≈318; corruption starts ~250).

**Verdict: gate is necessary but not sufficient. The blocker is now real-w_z quality during the
approach, not the terminal overfill.**

### 2026-09-08 — ROOT CAUSE of the pre-overfill spin-up: perception w_z has the OPPOSITE sign to the GT-FB w_z the law was validated against

Checked `cross_marker_perception.py`'s `w_z` and `alpha` against GT body yaw rate
(`ψ̇_b,ENU` from raw UAV quaternions, `test_data/ICValidation/20260907-161500`, pre-overfill
window, sync p95 ≤8 ms). NEITHER perception signal has a sign bug in its own convention:
- **`alpha` — correct.** `corr(alpha, +ψ_uav,ENU) = +0.95..+0.98`, slope `+1.1..+1.2`. Matches
  its own `_unweighted_principal_angle` closed form (`alpha_dot = -ψ̇_NED = +ψ̇_ENU`) and the
  2026-08-31 `_alpha_0` re-derivation (slope +1.0). The `e_a` → −50°…−110° is a CONSEQUENCE of
  the spin (alpha aliases past ±180° after ~½ turn), not an alpha fault.
- **`w_z` (calibrated `self._w_i[-1][2]`, lstsq col-5 `A[:,5]=[-y;x]`) — correct SIGN vs the
  manuscript rotational-flow definition** `w_z = -ψ̇_b,NED = +ψ̇_b,ENU`: `corr(w_z_perc, +ψ̇_b,ENU)
  = +0.66..+0.86` (6/7 reps; IC3 +0.20 = low yaw excitation). **But magnitude ~3× LOW: slope
  +0.2..+0.38** (should be +1) — the documented structural under-observability of the yaw column
  in the per-frame 6-DOF lstsq (`cross_marker_perception.py` ~L715-731); `s_wz=0.587` in
  `_sensor_cal_hw` doesn't recover it.

**The law breaks on a convention mismatch, not a perception bug.** Perception `w_z ≈ +0.3·ψ̇_b,ENU
≈ +0.27·alpha_dot` — i.e. **same sign as `alpha_dot`**. The kinematic identity is therefore
`e_a_dot = alpha_dot ≈ +w_z_perc`. The law `Δw_u2 = k_p·e_a − w_z` was derived/validated for
`e_a_dot = −w_z`, which is what `gt_feedback.py` supplies: `w[2] = -_asign·d(ry)/dt` with
default `PLASMC_GT_ALPHA_SIGN=+1` (the NON-perception alpha convention; `gt_feedback.py` L193-204
spells this out) → GT-FB `w_z` is the **opposite sign** to perception `w_z`. So under perception
the `−w_z` term becomes **positive feedback on `w_u2`** (`dw_u2/dt ≈ +c·w_u2 + …`, c>0) →
continuous yaw ~0.5–1 rad/s → GT relative yaw reaches +200°…+420° (1–2 full turns) well before
overfill → `e_a` aliases. Same class as the 2026-09-05 actuation-chain sign bug; GT-FB can't
catch it because GT-FB overrides `w_z` with its own oppositely-signed construction. The working
ASMC yaw loop never exposed it — it is pure `e_a`-SMC and does not consume `w_z` sign at all.

**Fix APPLIED + VALIDATED 2026-09-08.** `controller.py`: `PLASMC_YAW_RL_WZ_SIGN`
(default **−1** on perception → effective `+w_z`; auto **+1** under `PLASMC_GT_FEEDBACK=1`) and
`PLASMC_YAW_RL_WZ_SCALE` (default 1.0). Law increment is now
`k_p·e_a − (WZ_SIGN·WZ_SCALE·w_z)`; gate rate-guard still on raw `|w_z|`.

IC1-5 n=3 headless, real perception, `PLASMC_YAW_RATE_LAW=1` (WZ_SIGN=−1 auto)
`test_data/ICValidation/20260908-110746`:

| IC | mean xy | max xy | mean vel | precise |
|---|---|---|---|---|
| IC1 | 0.031 | 0.047 | 0.49 | 2/3 |
| IC2 | 0.096 | 0.212 | 0.59 | 2/3 |
| IC3 | 0.106 | 0.145 | 0.57 | 1/3 |
| IC4 | 0.126 | 0.172 | 0.37 | 1/3 |
| IC5 | 0.129 | 0.208 | 0.52 | 2/3 |

**15/15 land, 0 TL, 8/15 precise, all reps soft (rel_vel ≤0.76).** On par with / slightly better
than the baseline ASMC stationary gate (`20260831-144626`). Mechanism confirmed fixed:
GT yaw travel −8°…−24° (was +200°…+420°, 1–2 full turns); `yaw_rl_cmd` bounded ±0.23, no
runaway; `e_a` final −11°…+18°.

**Residual `e_a` ~10–22° (|mean last 20%|)** — the ~3× `w_z` magnitude deficit (weak
rate-cancellation term) + `k_p=0.3` only. **NEXT: sweep `PLASMC_YAW_RL_WZ_SCALE` (~2–3) and/or
`k_i`** to close it. Still open: n=5 + turning-target IC gate; ASMC/`psi_d` removal still deferred
until that's done. WZ_SIGN/SCALE not yet in `Control_Params.npy` (the `_buildLogDict` param dict
isn't that file — behaviour confirms it applied). Next levers, in order: (1) ramp `_yaw_rl_cmd`→0 on gate
instead of freeze-hold; (2) fix/characterise real w_z bias+noise pre-terminal (this is the big
one — filter, or bias-correct against alpha-rate); (3) earlier gate onset (EXT_ABS→250, WZ_MAX↓).
Blend-to-ASMC is weak here: the ASMC `u_a` running alongside is itself saturated ±2 on IC4.

## 2026-09-08 — WZ_SCALE=2.5 IC1-5 n=5 gate: FAILED, but the cause is a SEPARATE loom regression, NOT WZ_SCALE

`test_data/ICValidation/20260908-165248` (n=5, `PLASMC_YAW_RATE_LAW=1 PLASMC_YAW_RL_WZ_SCALE=2.5`,
collision-clean): IC1 4/5 precise (0.029 m); **IC2/IC3/IC5 catastrophic** (mean xy 0.54 / 5.40 /
4.66 m, IC5 all 5 reps ~5 m/s impact, 3-8 s flights); IC4 mixed.

⚠ I first said "the peer's commits regressed the landing" without proof — premature phrasing.
Investigated; the direction is now well-supported:

**Mechanism = loom (`h_z`) sign-flip for OFF-CENTER markers.** `h_z` in the first ~1 s of each rep
vs outcome: IC1 (centered) `h_z_early ≈ 0.00` → PASS; IC2/IC3/IC5/IC4-fail (`h_z_early = +0.27..
+1.30`) → CRASH. **Perfect correlation** (`h_z_early > +0.1 ⟺ crash`) across 11 reps. Same
off-center ICs on the OLD base (`20260908-110746`, pre-peer-commits): `h_z_early ≈ 0.00`, all PASS.
Time-course (IC5_rep1): `h_z = +0.43` from t=0, holds +0.43→+0.54 the entire descent while GT alt
drops 3.0→0.14 m in 3.2 s (a descending drone must read *negative* loom; every passing rep
`h_z_min ≈ −0.45`). Loom stuck positive → descent goes open-loop → unbraked fast descent → terminal
1/Z spike (`a_u_xy` 1219 in the last 2 frames only, not mid-flight).

**Not WZ_SCALE:** it scales `w_z` (channel 5, yaw) — arithmetically cannot flip `h_z` (channel 2);
IC1 passes cleanly at 2.5; `yaw_rl_cmd` bounded ±0.32, `e_a` ≤33° on every fail rep. Ruled out: my
`FLOW_KF_Q` commit `5796816d` (numerically verified no-op), my yaw gate `5849ceaa` (writes
`w_u[2]` only), `7b81ac1f` (additive logging).

**Narrows to `c3a46d1a`** (cross-marker perception) — its `origin_ratio` Tz-veto was restructured
to fire in a regime it never covered before, `r[2] *= 1e6` (≈ veto the loom measurement → KF
coasts `h_z`). `origin_ratio` is lower for an off-center marker (centroid farther from image
origin) → veto fires on IC2/3/5, freezes `h_z` at a small positive rest-noise value. Peer's own
`c3a46d1a` validation checked the new *width* loom's GT correlation in shadow mode — no off-center
landing gate on the restructured control-path moment-loom. Peer messaged 2026-09-08.

**WZ_SCALE=2.5's own n=5 verdict is DEFERRED** until the loom regression is fixed (re-running on
the broken base fails the same way). No further SITL until resolved.

### 2026-09-08 — loom fixed (`e173b05c`, `CROSS_TZ_VETO_R_MULT` 1e6→1.0), WZ_SCALE=2.5 re-gate

`test_data/ICValidation/20260908-182815` (n=5, WZ_SCALE=2.5, fixed base, collision-clean).
h_z sanity: `hz_early ≈ 0.00` on all 25 reps (was +0.27..+1.30), `hz_min` negative everywhere —
loom regression GONE.

| IC | result | vs broken-base run |
|---|---|---|
| IC1 | 5/5 land 0.01-0.09 m, 4/5 precise | (was fine) |
| IC2 | 5/5 land 0.01-0.21 m, 2/5 precise | **0.54 mean / 0 precise → fixed** |
| IC3 | 5/5 land 0.03-0.06 m, 4/5 precise | **5.40 mean / 0 → fixed** |
| IC4 | 5/5 land 0.05-0.30 m, 2/5 precise | **0.92 mean → fixed** |
| IC5 | **0/5 crash** (9.4/4.9/4.9/4.9/34 m, 3-13 m/s) | still fails |

**IC5 failure is TERMINAL 1/Z loom over-report, NOT WZ_SCALE, NOT the (fixed) veto.** IC5_rep1:
descends clean from 3 m, h_z tracks normally (−0.05→−0.47) and a_u ≈ 1-9 until t=5.0 s / alt
0.8 m — then h_z spikes to −6.54, a_u_z to −67.8, a_u_xy to 71 in the last 2 frames. This is the
documented "#1 open blocker" terminal-overfill mechanism (tuning-guide STATUS). yaw_rl_cmd bounded
±0.31, e_a ≤42° on every IC5 rep — yaw law is fine. IC5 = 3 m start = least runway before the
terminal danger zone (memory: "IC5 fails = LARGEST normalized error + LEAST runway").

### 2026-09-08 — WZ_SCALE 1.0 vs 2.5 A/B (n=5, fixed base) → WZ_SCALE=2.5 REJECTED, default 1.0 stands

`test_data/ICValidation/20260908-185421` (WZ_SCALE=1.0) vs `20260908-182815` (2.5), both n=5, fixed
loom base, collision-clean.

| IC | WZ=1.0 mean/max/prec | WZ=2.5 mean/max/prec |
|---|---|---|
| IC1 | 0.05 / 0.09 / **5/5** | 0.05 / 0.09 / 4/5 |
| IC2 | 0.21 / 0.49 / 3/5 | **0.10 / 0.21 / 2/5** |
| IC3 | 0.15 / 0.38 / 2/5 | **0.04 / 0.06 / 4/5** |
| IC4 | 0.09 / 0.16 / **3/5** | 0.15 / 0.30 / 2/5 |
| IC5 | **1.59 / 4.24 / 3/5** (3/5 clean ≤0.09, 2/5 crash) | 11.64 / 34.07 / **0/5** |

**WZ_SCALE=2.5 REJECTED.** It tightens IC2/IC3 precision but takes IC5 from 3/5 clean → **0/5
catastrophic** and marginally hurts IC4. The 2.5× amplified w_z (with its ~R²0.5 noise) eats IC5's
already-thin terminal margin (IC5 = 3 m start = least runway before the terminal-1/Z zone). The
`yaw_rl_cmd` stayed bounded on every IC5 crash → not a yaw runaway, it's the extra terminal
disturbance tipping a marginal case.

**WZ_SCALE=1.0 (code default) stands.** IC1-4 land 5/5; IC5 3/5 clean + 2/5 crash — the 2/5 is the
pre-existing terminal-1/Z overfill (#1 open blocker), NOT the yaw law.

### VERDICT — the yaw-rate law (PLASMC_YAW_RATE_LAW=1, default OFF) WORKS as an opt-in on stationary

Config: `WZ_SIGN=-1` (auto on perception), `WZ_GATE=1`, `WZ_SCALE=1.0`, `k_i=0`. IC1-5 n=5:
21/25 acceptable landings (≤0.5 m), 16/25 precise, 0 TL — comparable to / better than the baseline
ASMC stationary gate. Residual `e_a` ~15-20° (the 3× lstsq-w_z magnitude deficit) — NOT worth
closing with WZ_SCALE (noise cost > precision gain, proven above).

**Better lever for the deficit = per-channel `FLOW_KF_Q_WZ`** (committed infra `5796816d`): smooth
w_z harder → then a larger effective scale is tolerable. Next experiment: GT-scored `FLOW_KF_Q_WZ`
sweep + cross-marker cal re-derive (needs phased cal recordings — user-run).

**Still open (thread):** (1) `FLOW_KF_Q_WZ` sweep + recal; (2) turning-target IC gate (the law's
actual purpose — all validation so far is stationary); (3) IC5 terminal-1/Z overfill = separate
blocker; (4) ASMC/`psi_d` removal — keep deferred until (1)+(2) done.

## Next step (not started)

A confidence gate on `w_z`, using `MARKER_EXTENT_PX` (or a rate-of-change guard on `w_z` itself) to
freeze or blend toward the old ASMC path once overfill is detected — exactly what the original
task spec's own step 3 anticipated ("wire the measured path, gated on decode confidence, fall back
to adaptive rejection when confidence is low"), now with a concrete, measured reason it's needed
rather than a generic precaution. Do this before considering `PLASMC_YAW_RATE_LAW` for any default
flip or further real-perception testing.

## Also still open (unchanged from the Q8 investigation)

- No IC2-5 gate run yet (this is a turning-target mechanism; the existing gate assumes a
  stationary target, needs adapting).
- `k_i` (light integral robustness term) untested — default 0.0, pure P+rate-cancellation only.
- The ASMC/`psi_d`/CV-KF removal question (user asked "is it okay to remove both") — NOT yet acted
  on. Recommendation stands: keep the ASMC running (cheap, useful for comparison/fallback — and
  now, per the gating need above, likely load-bearing as the fallback target) until the confidence
  gate is built and the real-perception path is proven safe; only then reconsider removal.
