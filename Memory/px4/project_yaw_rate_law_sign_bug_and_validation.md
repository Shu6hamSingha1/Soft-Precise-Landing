---
name: project-yaw-rate-law-sign-bug-and-validation
description: "PLASMC_YAW_RATE_LAW (new yaw control, direct integrator on w_z bypassing psi_d/e_R) — a real actuation-chain sign bug was found+fixed via measurement; GT-feedback validation is a clean win INCLUDING beyond the old sin(dpsi) ceiling; real-perception is NOT yet safe (w_z inherits terminal-overfill corruption) — needs a confidence gate next."
metadata: 
  node_type: memory
  type: project
  originSessionId: 0f4a1549-4ee5-4e61-9344-dfa2c3a8081c
  modified: 2026-09-08T18:42:18.698Z
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

### 2026-09-08 — `FLOW_KF_Q_WZ` sweep + n=5 gate → best config found, residual e_a NOT closed

**Sweep** (IC1+IC5, n=3, `WZ_SCALE=2.5` fixed, `test_data/YawRLQwzSweep/`): sharp threshold at
`FLOW_KF_Q_WZ ≈ 1.0`. IC5: `Q_wz` 10/5/2 → 3/3 crash+TL (7-16 m); `Q_wz` 1.0/0.5 → **3/3 precise
0.03-0.10 m**. IC1 clean throughout. Confirms `w_z` is **noise-limited, not lag-limited** —
`Q_wz`=10 (less smoothing) is the WORST; "less delay for w → higher Q" intuition falsified for `w_z`.

**n=5 gate** `FLOW_KF_Q_WZ=1.0` + `WZ_SCALE=2.5` (`test_data/ICValidation/20260908-222553`,
collision-clean): **24/25 acceptable (≤0.5 m), 0 TL.** IC1 5/5 (0.067) · IC2 5/5 (**0.069**,
tightest off-center yet) · IC3 5/5 (0.112) · IC4 5/5 (0.098, all soft) · IC5 **4/5 ≤0.10 m** (rep4
crash = terminal-1/Z, `hz_min −6.6`, yaw_cmd bounded).
vs: `WZ_SCALE=1.0` 21/25 IC5 3/5 IC2 0.21 · `WZ_SCALE=2.5` alone 20/25 **IC5 0/5** · this 24/25 IC5 4/5.

⚠ **`e_a` residual did NOT improve** — per-IC `|e_a|` tail ~10-17°, same as `WZ_SCALE=1.0`. The
sweep's rationale (close the residual) did not pan out; residual is the ~3× magnitude deficit +
weak `k_p=0.3` + likely a terminal alpha offset, not `w_z` noise. `Q_wz=1.0`+`2.5` = a modest
robustness/precision gain, not a residual fix.

**Recal DEFERRED — not a clean "just run it":**
- ⚠ Methodology Q: `record_cross_marker_calibration.py:262` logs `getRawOptFlowAngVel()` which for
  CROSS-marker returns `self._hw` = KF-FILTERED (unlike `img_data.py`'s pre-KF raw), then
  `derive_cross_marker_cal.py:184` applies `kf_filter_causal` AGAIN → cross-marker cal is fit to a
  DOUBLE-filtered signal while runtime KFs once. Pre-existing (affects current `s_wz=0.587`),
  probably benign (2nd KF pass ≈ idempotent on a smooth signal), but verify before any recal.
- Needs ≥5 user-run phased cross-marker cal flights with `FLOW_KF_Q_WZ=1.0`.
- Low marginal value: moves the gain from a runtime knob into `s_wz`; e_a residual unaffected.

### STOPPING POINT

`PLASMC_YAW_RATE_LAW=1` (default OFF) = a working opt-in on stationary. Best config: `WZ_SIGN=-1`
(auto) + `WZ_GATE=1` + **`FLOW_KF_Q_WZ=1.0` + `WZ_SCALE=2.5`** → 24/25 land, 0 TL, tightest
off-center. NOT a clear win over the ASMC for stationary (residual e_a ~15°, IC5 terminal blocker).
Its value is turning targets — UNTESTED. Next real step for the thread = the turning-target gate,
not the recal. (`8884f43d` visibility_projection.py = new CBF module, NOT wired into controller.py,
inert — base stable.)

### 2026-09-08 — TURNING-TARGET GATE: yaw-rate law wins the yaw channel decisively

Adapted the IC gate: GT-FB + `PLASMC_GT_SPIN_WZ=0.48` (target rotates in place at 27°/s —
`gt_feedback.py:111`; isolates yaw from the moving-rover translation blocker). A/B on
`PLASMC_YAW_RATE_LAW`, IC1-5 n=3. Bundles `test_data/ICValidation/{20260908-230706 LAW=1,
20260908-232039 ASMC}`.

**YAW result — the core claim, CONFIRMED in the full landing loop:**
| arm | `\|e_a\|` tail | `e_a` end | `yaw_cmd` peak |
|---|---|---|---|
| **LAW=1** | **1-4°** (IC5 11° once) | ±3-9° | 0.62 rad/s (ABOVE the ASMC 0.5 sin-ceiling) |
| ASMC | 94-167° | −116° to −202° | 2.0 clamped, can't hold |

ASMC does exactly what the Q8 sin(Δψ) analysis predicted: 60-80° lag → runaway to the ±180° alias.
The new law holds yaw error to a few degrees at a spin rate no `e_R`-routed command can reach.
**`PLASMC_YAW_RATE_LAW` is validated for turning targets on the yaw channel.**

**LANDING xy — CONFOUNDED.** Both arms land poorly on IC2-5 (~2-17 m). Gate ran with
`PLASMC_GT_ALPHA_SIGN=+1` (default); `gt_feedback.py:193-204` — `+ry` is "self-consistent at
e_a≈0" (so yaw holds) but on a persistently rotating target "drives the loop through the inverted
disturbance path", corrupting lateral coupling — IC1 (centered) survives, off-center flies off.
Re-run done (`test_data/ICValidation/{20260908-233929 LAW=1, 20260908-235435 ASMC}`,
`PLASMC_GT_ALPHA_SIGN=-1`): **the `-1` flip BROKE the new law's yaw** — `|e_a|` tail 33-68° (vs
1-4° at `+1`), huge `a_u_xy` 1600-3756 fly-offs on off-center. Cause: `WZ_SIGN` is auto-set on
`PLASMC_GT_FEEDBACK` ONLY, so it did NOT follow the `_asign` flip → law's `-w_z` term wrong-signed
again. **The FIRST run (`GT_ALPHA_SIGN=+1`, default) is the correct config for the new law.**

### TURNING-TARGET GATE — CONCLUSION

1. ✅ **Yaw-rate law validated for turning targets.** Correct config = `GT_ALPHA_SIGN=+1` +
   `WZ_SIGN=+1` (auto under GT-FB). Holds `e_a` 1-4° on a 27°/s spin; `yaw_cmd` 0.62 rad/s (past
   the ASMC 0.5 sin-ceiling); ASMC runs away to −200°. **This is the deliverable — done.**
2. ⚠ `WZ_SIGN` must track `GT_ALPHA_SIGN` — the GT-FB auto-logic assumes `+1`. Harmless for real
   perception (`_asign` fixed there, `WZ_SIGN=-1`); a note, not a bug.
3. ❌ **Full landing on a spinning target NOT solved.** Even with perfect yaw (`e_a`~2°, config
   #1), off-center ICs land ~6 m off with short flights. Root: the drone genuinely yaws ~0.48 rad/s
   to track → `cross(w_i, s)` injects a persistent ROTATING lateral-flow disturbance (~0.3 m/s,
   scales with offset) = the documented turning-target lateral limit cycle
   ([[project_rover_turning_open]], proposed fix `PLASMC_AU_LEAD`). SEPARATE thread; the yaw law
   neither addresses nor was meant to address it.

**THREAD DONE.** `PLASMC_YAW_RATE_LAW` (default OFF, opt-in): sign bug fixed, `w_z` confidence gate
built, stationary-validated (best config `WZ_SIGN=-1`+`WZ_GATE=1`+`FLOW_KF_Q_WZ=1.0`+`WZ_SCALE=2.5`,
24/25 land), **turning-target YAW validated** (the actual purpose). Not a stationary win over ASMC
(residual e_a ~15°). Its value is realised only once the turning-target LATERAL limit cycle is
also fixed. Deferred/handed off: recal (methodology Q + user-run flights); lateral limit cycle
(`project_rover_turning_open`); ASMC/`psi_d` removal (keep as fallback).

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

### 2026-09-09 — stationary residual e_a ~15° is NOT alpha_0 (checked)

User asked if re-deriving `alpha_0` (`CROSS_ALPHA_0`) would fix the ~15° stationary residual.
RULED OUT: (a) `e_a` itself sits at ~±15° — the law isn't nulling its OWN metric (an alpha_0
bias leaves e_a→0 with the physical yaw carrying the offset; here e_a and GT yaw agree, both
~15°); (b) the sign is IC-dependent — IC1 consistently +9..+18°, offset ICs consistently
−3..−29° — a single constant offset can't do that; it's yaw↔lateral coupling during the offset
approach. `alpha_0` is also recently re-derived (radians(0.58), `derive_cross_alpha0.py`,
2026-08-31 `b963e207`/`aad0f57b`), not stale. Couldn't fit offset/slope from the stationary
reps (drone barely yaws → near-zero-variance regression, slopes came out −5..+8 = noise).
The residual is control-side: weak `−w_z` term (3× lstsq magnitude deficit) + `k_p=0.3` can't
null the last 15° vs the 38 ms lag. Levers = `k_i` (0.0, untested) or larger effective `w_z`
(recal / `WZ_SCALE`), NOT `alpha_0`.

### 2026-09-09 — the ~15° residual is a GATE-FREEZE terminal artifact, NOT a steady-state control-gain limit

⚠ CORRECTS the "control-side weak k_p / w_z deficit" framing above. Traced the per-%-flight `e_a`
on `20260908-222553` (Q_wz=1.0+WZ_SCALE=2.5, perception-ON, stationary):
- **Mid-descent (50-65% flight, alt ~0.8-1.8 m): `e_a` is a few degrees** — the yaw law converges
  fine; `yaw_rl_cmd` is actively adjusting (−0.16..+0.11).
- **At ~0.8-0.9 m the confidence gate fires** (MARKER_EXTENT_PX overfill 280→318) and FREEZES
  `_yaw_rl_cmd` at its current small NON-ZERO value (~+0.04..+0.09 rad/s) AND freezes `_yaw_rl_ie`
  (anti-windup).
- **Last ~1-2 s to touchdown: `e_a` drifts to ±15-30°** — the frozen non-zero command keeps the
  drone slowly yawing with no feedback to stop it (GT yaw ends +8-13° off), and `alpha` corrupts
  as the marker overfills (the terminal ±334°/−143°/−155° jumps).
- IC-dependent sign = sign of the frozen command at gate-fire time (approach geometry) + alpha's
  terminal drift direction. IC1 +17°, offset ICs −10..−32°.

**So `k_i` likely won't help the terminal residual** — the gate freezes `_yaw_rl_ie` too, so
integral action is suspended in exactly the window where the residual accumulates. (KI sweep
{0.0,0.1,0.3,0.6} IC1+IC2 n=3 running to confirm — `test_data/YawRLKiSweep/`.)

**Real fix = ramp `_yaw_rl_cmd` → 0 on gate** (a frozen NON-ZERO command is what keeps yawing the
drone), and/or gate later than `EXT_ABS=280` (fires at ~0.8 m, early). If xy is good (it is —
24/25), a ±15° yaw error post-touchdown is largely cosmetic.

**`w_z_eff` = `w_iz` reconciled**, not extra: `WZ_SIGN=-1` (perception `w_iz` = `+α̇` vs the GT-FB
`w_z` = `−α̇` the law was built against) + `WZ_SCALE=2.5` (lstsq col-5≈Ty aliasing attenuates the
estimate ~3×; `s_wz=0.587` bakes that in; scale restores the end-to-end gain — proper fix is the
deferred recal).

### 2026-09-09 — k_i sweep: DEAD END, confirms the residual is gate-freeze not integral-deficiency

`test_data/YawRLKiSweep/` (KI {0.0,0.1,0.3,0.6}, IC1+IC2 n=3, best config): `|e_a|` tail
12.7°→13.9°→17.8°→**29.4°** — MONOTONICALLY WORSE (k_i=0.6 has a +91° windup outlier). xy
unaffected (≤0.18). `ie_a|max|` grows 0.19→0.57 (integrator accumulates but is powerless):
the gate freezes `_yaw_rl_ie` in exactly the terminal window where the residual forms, and
in the approach k_i just adds lag to the 287 ms yaw loop → larger e_a at gate-fire → larger
terminal residual. **Keep `PLASMC_YAW_RL_KI=0.0`.** The residual needs ramp-cmd→0-on-gate (or
a later gate), not integral gain.

### 2026-09-09 — WZ_SIGN=-1 is analytically forced, but belongs in gt_feedback.py not a knob

Full chain: `α̇ = −ψ̇_b,NED` (Jabbari Asl eq 22, confirmed by the 2026-08-31 `_alpha_0` re-derive:
`alpha ≈ +ψ_ENU`), and manuscript `w_z = −ψ̇_b,NED`, so **`e_a_dot = +w_z`** — NOT `−w_z`. The
law's `−w_z` term comes from `gt_feedback.py:234` `w[2] = -_asign·d(ry)/dt = −α̇` (GT-FB's own
construction). Perception `w_iz` IS correctly signed per the manuscript (`_fill_A = −L(s)` but
the cal absorbs it — `s_wz = +0.587 > 0`; measured `corr(w_iz, +ψ̇_b,ENU) = +0.7..0.86`), i.e.
`w_iz ≈ +w_z,manuscript`. So `WZ_SIGN=-1` = feeding `+w_z,manuscript` into a term that wants
`−w_z,manuscript`. Analytically forced; SITL just surfaced it.
**Clean fix (queued, needs own GT-FB re-validation): (1) `gt_feedback.py` `w[2] = +α̇` (drop the
leading `−`); (2) re-derive the law's rate term from `e_a_dot = +w_z`; (3) re-validate GT-FB
ceiling048/beyond060; (4) delete `WZ_SIGN`. `WZ_SCALE` stays (separate magnitude-deficit → recal).**

### 2026-09-09 — WZ_SIGN clean fix: frame math NAILED, gt_feedback.py flip BLOCKED on the lateral path

Numerically verified `_yaw_of` (gt_feedback.py) returns **ENU yaw** (docstring "NED yaw" is
WRONG — a +30°-about-z quat → +30°). So:
- `gt_feedback.py:234` `w[2] = -_asign·d(ry)/dt`, `ry` ENU, `_asign=+1` → `w[2] = -psi_dot_b,ENU`
- Manuscript `w_z = -psi_dot_b,NED = +psi_dot_b,ENU`
- ∴ **`gt_feedback w[2] = -w_z,manuscript`**; perception `w_iz ≈ +w_z,manuscript` (`s_wz=+0.587`,
  corr +0.66..+0.86). Genuinely opposite → `WZ_SIGN` split (`+1` GT-FB / `-1` percep) is
  ANALYTICALLY FORCED, not a knob-tune. Comment in `controller.py:587` now carries the derivation.

**gt_feedback.py flip is BLOCKED (2 reasons):**
1. `gt_feedback`'s `w[2]` feeds `self._w_i`, consumed by the LATERAL `h_d`/c-term path
   (`cross(w_i,s)`, `2·cross(w,h)` at `controller.py:2678`/`2959-2961` — sign-sensitive in `w_z`
   even with `CTRL_ZERO_WXY=1`: `cross([0,0,w_z],s)=[-w_z·s_y, +w_z·s_x, 0]`). `feedback_gtfb_wz_sign_bug`
   (2026-06-25) deliberately set `w[2] = -_slope` FOR that path — other sign → IC4 flew OUT at
   altitude (2.5→6.4→5.4 m). Flipping needs the lateral GT-FB IC2-5 gate re-run.
2. That 2026-06-25 memory claims `-_slope` "matches perception". The frame math above says it's
   the OPPOSITE. Reconcile before flipping (one of the two is wrong).

**Status:** `WZ_SIGN=-1` (perception) stays — it IS the correct value. The gt_feedback.py flip +
`WZ_SIGN` deletion is queued behind: (a) lateral GT-FB IC2-5 re-gate, (b) yaw GT-FB re-gate
(ceiling048/beyond060), (c) resolving the 2026-06-25 sign-claim contradiction.

### 2026-09-09 — ramp-cmd→0-on-gate is STATIONARY-ONLY, wrong for a moving target

The terminal `e_a` residual fix (ramp `_yaw_rl_cmd`→0 when the gate fires) only works if the true
required yaw rate is 0 = stationary target. On a target turning at ω_t the drone must KEEP yawing
at ~ω_t to touchdown; ramping to 0 grows the relative yaw error in the final approach. For a
CONSTANT-rate turn the current FREEZE (hold last cmd) is accidentally closer to right (last cmd ≈
the tracking rate). General fix: make it scenario-aware, OR drop the `MARKER_EXTENT_PX` overfill
trigger from the yaw gate and keep only the `|w_z| > WZ_MAX` rate-guard (overfill drives `w_z`
large so the rate-guard catches it anyway; the extent trigger is what prematurely kills the
command at ~0.8 m).

### 2026-09-09 — WZ_SIGN CLEAN FIX IMPLEMENTED (unify on manuscript convention)

**Done** (commit pending): `gt_feedback.py:234` `w[2] = -_asign·_slope` → `w[2] = +_asign·_slope`;
`controller.py` yaw law increment `k_p·e_a - _wz_eff` → `k_p·e_a + _wz_eff`; `WZ_SIGN` default
`+1` for BOTH paths (dropped the `_gt_fb ? +1 : -1` branch; env override kept).

**Frame derivation (settled):**
- `_yaw_of` (gt_feedback) returns **ENU yaw** — numerically verified (+30°-about-z quat → +30°;
  docstring "NED yaw" is WRONG). So `ry = ψ_uav,ENU − ψ_tgt,ENU`, `d(ry)/dt = ψ̇_b,ENU` (stationary).
- Manuscript `w_z = ω_t,z − ψ̇_b,NED` → stationary `w_z = −ψ̇_b,NED = +ψ̇_b,ENU`.
- Jabbari Asl eq 22 + 2026-08-31 `_alpha_0` re-derive: `α̇ = −ψ̇_b,NED`. **∴ `α̇ = w_z,manuscript`
  and `e_a_dot = +w_z`** (NOT `−w_z`).
- OLD `gt_feedback w[2] = −_slope = −ψ̇_b,ENU = −w_z,manuscript`; perception `w_iz` (calibrated,
  `s_wz=+0.587`, `corr(w_iz,+ψ̇_b,ENU)=+0.66..0.86`) `= +w_z,manuscript`. OLD state had the two
  sources OPPOSITE-signed → the empirical `WZ_SIGN` split papered over it.
- NEW `gt_feedback w[2] = +_slope = +w_z,manuscript` → both sources agree.

**Convergence law (manuscript convention):** `d(w_u2)/dt = k_p·e_a + w_z`. Closed loop with
`e_a_dot = ω_t,z − w_u2` (actuation inversion `ψ̇_b,ENU ≈ −w_u2`) → `e_a_dot → −k_p·e_a`; char.
`s²+s+k_p=0`, stable (k_p=0.3 → damped complex, Re −0.5). `w_u2 → ω_t,z` (absorbs the spin).

**`- w_z_gtfb_old ≡ + w_z_manuscript`** → the yaw-path `w_u[2]` is **bit-identical before/after**
for BOTH perception and GT-FB (numerically checked). Pure relabel. The empirical `WZ_SIGN` split
is RETIRED — it existed only because the pre-fix GT-FB `w[2]` was mis-signed.

**⚠ UNVALIDATED — the LATERAL GT-FB path.** `gt_feedback w[2]` also feeds `cross(w_i,s)` / c-term
(`controller.py:2678`/`2959-2961` — sign-sensitive in `w_z` even with `CTRL_ZERO_WXY=1`).
`feedback_gtfb_wz_sign_bug` (2026-06-25) set it to `−_slope` citing `alpha_dot = +psi_dot_b,NED`
— which is the ERRONEOUS sign (paper eq 22 says `−`). That flip left GT-FB opposite to perception.
Reverting SHOULD match perception (which lands fine), but the 2026-06-25 IC4 altitude-flyout claim
(n=2, flaky) needs a **lateral GT-FB IC2-5 re-gate** to confirm no regression. Yaw GT-FB gate
(`ceiling048`/`beyond060`) is a no-op regression check (relabel).

**Perception paths (stationary 24/25, turning-target yaw) are UNCHANGED** — `gt_feedback` isn't in
the perception path and the yaw term relabel is bit-identical.

### 2026-09-09 — WZ_SIGN CLEAN FIX **VALIDATED** (n=1 GT-FB, decisive)

n=1 is sufficient here: the failure mode is binary+multi-metre (monotone-converge vs altitude
fly-out) and GT-FB removes perception noise. Bundles `test_data/ICValidation/{20260909-020940
(yaw spin IC1), 20260909-021038 (lateral IC2/3/4)}`, base `4acc9ef0`.

- **Yaw path (relabel no-op):** IC1, spin 0.48 rad/s → lands xy=0.007 m, `lat_err ≤ 0.09 m`
  throughout; yaw held on the spinning target. Confirms `- w_z_gtfb_old ≡ + w_z_manuscript`.
- **Lateral GT-FB (sign check):** IC2/IC3/IC4 no-spin → **monotone convergence, ZERO fly-out**
  (2.8 → 2.6 → 1.6 → 0.5 → 0.15 → 0.02 m; `max lat_err after t=1s` = start offset). IC4 — which
  `feedback_gtfb_wz_sign_bug` claimed flew OUT 2.5→6.4→5.4 m on the other sign — converges
  2.78 → 0.02 m.

⇒ `gt_feedback.py w[2] = +_slope` (unified manuscript convention) is correct for BOTH the yaw
law and the lateral `cross(w_i,s)`/c-term path. The 2026-06-25 `-_slope` was the wrong sign
(erroneous `alpha_dot = +psi_dot_b,NED`); its n=2 IC4-flyout was flaky/confounded. **`WZ_SIGN`
is retired.** Perception paths were bit-identical throughout (unchanged).

**Yaw-rate-law thread status:** sign bugs (actuation-chain + w_z convention) both fixed and
frame-derived; `w_z` confidence gate built; stationary IC1-5 n=5 = 24/25 (best config
`FLOW_KF_Q_WZ=1.0`+`WZ_SCALE=2.5`); turning-target YAW validated (e_a 1-4° vs ASMC runaway);
k_i dead end; terminal residual = gate-freeze (fix = ramp-cmd→0, stationary-only). Remaining:
WZ_SCALE recal (deferred); turning-target LATERAL limit cycle ([[project_rover_turning_open]]);
ASMC/psi_d removal (keep as fallback). `PLASMC_YAW_RATE_LAW` stays default OFF.
