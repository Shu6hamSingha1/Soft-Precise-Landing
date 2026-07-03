---
name: convergence-ordering
description: "The design's convergence ordering (lateral 18x faster than vertical, yaw 2.4x) collapses in PX4 because per-channel bandwidth demands are inverted vs per-channel actuation lags; yaw DIVERGES in half the reps; fix = re-match each channel's gains to its own chain (Batch 8)"
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 7faf44bf-c5f1-4b57-a701-f6d868abfdc1
---

> ⛔ **STAMP 2026-07-03: contaminated/superseded-era analysis** (pre-honest-cal, pre-combined-surface). The bandwidth-inversion 'collapse' framing predates the resolved lateral wall (gain-parity bug + velocity damping) and the Z_REG artifact; Batch-9 'wins' were open-loop-handoff INVALID. History; the per-channel bandwidth-matching principle survives in later files (yaw-first kΩ_z etc.).
**User requirement (2026-06-03): x/y and yaw must converge FASTER than z. Measurement shows this ordering has collapsed in PX4.**

**Design intent (manuscript gains):** lateral τ = p₁₀/K_rp = 0.13 s, yaw τ ≈ 1/(γ_α+χ_α) = 1.0 s, vertical τ = 1/|h_rd| = 2.4 s → lateral 18× faster than vertical.

**Measured (envelope-fix reps + SP rep, IC1):**
- lateral t₅₀% = 0.6–4.7 s, sometimes NEVER settles (<0.1 m) — ~35× slower than design, underdamped/limit-cycling
- **yaw DIVERGES in 3/6 reps** (|e_α| ends at 80–89°, the ±90° wrap boundary). Yaw divergence correlates 1:1 with lateral failure — diverged yaw rotates the V frame and mis-aims lateral corrections.
- vertical t₅₀% = 3.8–4.1 s, completes every rep — the only reliable channel

**Why (the mechanism):** each channel's commanded bandwidth vs its own actuation-chain bandwidth:

| Channel | Demand | Achievable | Consequence |
|---|---|---|---|
| lateral | 7.6 /s | ~2–3 /s (roll/pitch rate-loop 52–61 ms + image ~100 ms) | 2.5–4× over-driven → limit cycle |
| yaw | ~1.0 /s | ~1.7 /s (yaw rate-loop **287 ms**, 5× roll/pitch — [[feedback-input-cal-yaw-lag]]) | zero margin → wrap divergence |
| vertical | 0.42 /s | ~5–10 /s (direct thrust) | 12–24× under-driven → clean |

The gains assume MATLAB's uniform ~10 ms lag; PX4's lag is per-channel asymmetric (thrust fast, roll/pitch slow, yaw very slow) — the fastest-demand loop sits on the slowest chain.

**The per-axis fix (Batch 8, b8A bundle):** lateral KP/KI/KD ×0.35 + sched off (K_rp=1.4, demand 1.2/s), yaw χ_α/γ_α ×0.4 (demand 0.4/s), vertical h_rd −0.42→−0.15 (τ=6.7 s, deliberately slowest). Restores ordering: lateral < yaw < vertical. KEY: previous K_rp reductions all failed ("frontier slide") because they ran against the unchanged 8 s descent — the gain cut and the descent slowdown must move TOGETHER to preserve the race.

**How to apply:** when setting any channel's gain, check its actuation-chain lag first (roll/pitch 52–61 ms, yaw 287 ms, thrust ~10 ms); demand ≤ 0.4–0.5× the chain's bandwidth. Never compare channels by gain value — compare by demand/achievable ratio.


---

**RESULTS (Batch 9, 2026-06-03): 6 SP in 10 reps — the ordering fix works.**
- Arm A (full, slow descent): 3/5 SP incl. **xy=9.6mm/vel=7.5mm/s (project record, beats MATLAB worst-case 5×)**, 1 TL from marker loss on the long flight
- Arm B (no yaw fix): 0/5 SP, 2 TL — **yaw bandwidth matching is load-bearing**
- Arm C (normal descent −0.42): 3/5 SP, **0 TL, all reps ≤0.12 m** — slow descent NOT needed; **this is the winning config**

WINNING CONFIG (env): KP_SCHED_ENABLE=0, KP/KI/KD_X/Y×0.35, YAW_OMEGA/GAMMA×0.4, RHOFOV0_V×1.5,
RHOFOVINF_U×2.75, RHOFOVINF_V×3.75, LANDING_IBVS_HANDOFF_ALT=0.25. No clamps (DH_D_MAX=50, no floor).
Pending: IC2-5 gate, then bake as defaults.

---

**INVALIDATION + CORRECTION (2026-06-03, user):** the Batch-9 results above are INVALID — they used an
"IBVS handoff at 0.25 m" that made the last 25 cm open-loop. **zf = 0.2 m in the manuscript is the LANDING
GEAR HEIGHT**, not a controller validity envelope: MATLAB's termination at alt ≤ 0.2 m IS gear contact (the
same physical event as PX4's LandedState). The controller is designed to control THROUGH touchdown; analyzing
and fixing its touchdown behaviour is the research objective — never bypass it with an open-loop final drop.
The handoff code was reverted from landing_test.py. Batch 10 re-runs the same arms with the controller in
authority to gear contact. Note: with K_rp=1.4 the funnel-saturation threshold is err > 2.1·Z (vs 0.33·Z at
K_rp=9), so the ordering config may well survive the final 25 cm on its own — that is what Batch 10 tests.

---

**CAMPAIGN FINAL STATE (2026-06-03, ~160 reps total).** The final config family (per-axis ordering gains +
p1 envelope + GAMMA_Z x0.5 + KLT=20 + span-commitment 100px, no clamps, controller to gear contact):
**n=25 pooled: 28% SP @ 10cm** (vs ~2% historical), xy median 0.195 / best 0.018, vel median 0.179 / best 0.014.
Catastrophic 3/25. The two engineered failure modes (touchdown perception loss, z-transient) are handled; the
residual ~72% non-SP is mid-flight perception/lateral variance under SITL lag — the architectural floor.
"SP every rep" requires the lag fix ([[dds-lag-fix-blocker]]). Config recorded as parameter_record trial 46;
env recipe in the trial-46 remark. NOT yet IC2-5 gated.

---

**KNOB TRANSLATION (2026-06-03 cleanup — scale factors removed, direct values only).** The validated config
in the new direct-value knob names:
```
PLASMC_KP_X=1.4 PLASMC_KP_Y=1.4  PLASMC_KI_X=0.35 PLASMC_KI_Y=0.35  PLASMC_KD_X=0.5031 PLASMC_KD_Y=0.5031
PLASMC_YAW_OMEGA=0.2 PLASMC_YAW_GAMMA=0.2
PLASMC_RHOFOV0_V=315 PLASMC_RHOFOVINF_U=220 PLASMC_RHOFOVINF_V=300
PLASMC_GAMMA_Z=0.375
MARKER_KLT_MAX_STEPS=20 LANDING_STALE_COMMIT_EXTENT=100
CTRL_ZERO_WXY=1 BOARD_ALPHA0=1.23 V_YAW_SOURCE=compass
```
Old *_SCALE knobs no longer exist; K_rp scheduling/DSD clamps/THETA_FLOOR/VFRAME_ROT/PX4_RATE_SCALE removed
(parameter_record.ods sheet Removed_Parameters documents each).

---

**MATLAB DELAY-ROBUSTNESS ENVELOPE (2026-06-03, both sweeps run on Ubuntu MATLAB R2026a).**
Two delay models in run_simulation.m: `delay` (torque, inside attitude loop — manuscript model) and the NEW
`delay_outer` (attitude/thrust reference stale, inner SO(3) feedback live — the PX4 lag structure).

SP rate by delay, manuscript gains / PX4-validated gains:

| | 10ms | 30ms | 50ms | 100ms | 150ms | 300ms |
|---|---|---|---|---|---|---|
| INNER (torque) | 100/60 | 100/100 | 60/20 | 0/0 | 0/0 | 0/0 |
| OUTER (PX4 structure) | 100/60 | 100/100 | **100**/20 | **100**/40 | 60/40 | 0/**40** |

Conclusions: (1) **delay LOCATION matters more than magnitude** — manuscript gains survive 100ms outer but die
at 50ms inner; (2) PX4's lateral lag (~100-170ms) sits exactly at the manuscript-gain cliff → explains the PX4
variance; yaw lag 287ms is past the cliff → explains yaw divergence; (3) the PX4-validated reduced gains trade
low-delay performance for a ~40% floor at high delay — MATLAB predicts ~40% SP at PX4-like delays, PX4 delivers
28% (the gap = perception effects MATLAB doesn't model); (4) the delay-robustness claim of the manuscript is
TRUE and now quantified as an envelope — a strong manuscript figure.
Data: MATLAB/Datasets/Sweeps/DelayRobustness/delay_sweep_results_{inner,outer}.mat

**MIXED (PER-CHANNEL) DELAY SWEEP (2026-06-03, MATLAB Test 3) — the lateral ×0.35 detuning is NOT justified by lag.**
Per-channel outer delay (`delay_outer_lat`/`delay_outer_yaw` in run_simulation.m) at PX4-matched profiles
(lat=60/100/150ms, yaw=290ms fixed), n=5, SP@10cm:

| Arm | lat=60 | lat=100 | lat=150 |
|---|---|---|---|
| A manuscript | 100% (xy 0.040) | 100% (0.023) | 100% (0.049) |
| B current PX4 config (lat ×0.35 + yaw 0.2) | **20% (xy 1.365)** | **0% (0.189)** | **20% (1.786)** |
| C manuscript lateral + yaw 0.2 | 100% (0.040) | 100% (0.023) | 100% (0.049) |

Manuscript lateral gains handle the entire PX4 lateral-lag range perfectly; the ×0.35 detuning produces
meter-scale misses in the same conditions. **If ×0.35 helps on PX4, it is compensating perception effects
(LK breakage under aggressive maneuvers, mode 4), not lag.** Caveat: yaw channel is unexcited in this test
(static target + aligned IC) → A≡C; yaw-detuning rationale rests on PX4 evidence only.
Next PX4 step: restore lateral gains stepwise (×0.5 → ×0.7 → ×1.0 of manuscript), yaw kept at 0.2, n≥5 each,
IC1; expect the binding constraint to shift from "lag" to perception.
Data: MATLAB/Datasets/Sweeps/DelayRobustness/mixed_delay_sweep_results.mat

> **UPDATE 2026-06-04 — MATLAB Test 3 is likely VINDICATED, not refuted.** The "next PX4 step" above (restore
> lateral gains) WAS run as the LateralRestore staircase, which "refuted" the MATLAB prediction (manuscript-lateral
> catastrophic, KP=9→5/5 TL). But that staircase was **WX/WY-CONTAMINATED** (CTRL_ZERO_WXY=0, live garbage w_x/w_y
> up to 3.55 rad/s → body-rate spikes → LK breakage falsely blamed on the gains; see [[per-axis-tuning]]). The
> MATLAB IC2 off-center trace (2026-06-04) confirms the real differentiator is **K_rp** — MATLAB converges off-center
> with K_rp=9 by driving s_e_n to 0.09 by mid-flight, while PX4's K_rp=1.4 (×0.35) leaves it at 0.55. So MATLAB
> Test 3's "manuscript lateral works" prediction looks CORRECT; re-running KP=9 with CTRL_ZERO_WXY=1 to confirm.

---

**THE COMPLETE VALIDATED-CONFIG RECIPE (written 2026-06-03 after two reconstruction failures).**
The 28%-SP config is NOT just gains. Three mechanisms beyond the env gains are load-bearing, were defaults/env
at b10C/b13/b14 time, and got dropped by the 14:20 refactor (each loss = catastrophic regression, median xy
0.2m -> 2.5-3.7m):

| Component | Value | Where it lives now |
|---|---|---|
| Lateral PID | KP=1.4, KI=0.35, KD=0.5031 (X and Y) | env PLASMC_KP/KI/KD_X/Y |
| Yaw | Omega_a=Gma_a=0.2 | env PLASMC_YAW_OMEGA/GAMMA |
| GAMMA_Z | 0.375 | env PLASMC_GAMMA_Z |
| p1 envelope→sensor | RHOFOV0_V=315, RHOFOVINF_U=220, RHOFOVINF_V=300 | env PLASMC_RHOFOV0_V etc. |
| **THETA_FLOOR=60** | disables d_min cone collapse | **controller.py default (restored)** |
| **DH_D_MAX=5.0** | breaks κ-runaway at touchdown | **controller.py default (restored)** |
| KLT=20, stale-commit 100px, PRECISE_TOL=0.10 | perception/classification | img_data/landing_test defaults |

Cross-check method when a config seems unreproducible: `analyze_saturation_audit.py` on an old-vs-new rep —
mismatched duty-cycle profiles point at the missing mechanism (cone 4.9% vs 43.8% found THETA_FLOOR).

---

**EMPIRICAL alpha/yaw impact on landing (2026-06-03, recordings analysis) — alpha is INERT w.r.t. stationary-board landing performance; confirms yaw is low-leverage.**
Analyzed Feature Params[:,3]=alpha + e_a(t) (yaw error) across pre-reboot SP-producing batches (b10C/b13/b14,
~58Hz) and post-reboot fails (c1now, ~84Hz). Median alpha/yaw, mid-flight (40-70%) vs terminal (last 15%):

| group | SP | alpha_mid | alpha_term | ea_mid | ea_term |
|---|---|---|---|---|---|
| b10C/b13/b14 (working) | 6/25 | ~0.44 | ~0.05 | ~0.03 | ~0.04 |
| c1now (FPS-broken) | 0/10 | 1.12 | 1.06 | 0.21 | 0.20 |

In the WORKING regime: alpha collapses to ~0.05 rad (~3°) at touchdown and yaw error stays ~0.03-0.04 rad
(~2°) throughout — the SMC is barely active. **SP reps vs non-SP reps are statistically indistinguishable in
alpha/yaw** (alpha_mid 0.44 vs 0.43; ea_mid 0.032 vs 0.038). Pearson r of alpha/ea vs xy_err all weak
(|r|≤0.27). So alpha has NO measurable causal impact on landing precision for the stationary aligned board.
The large alpha (1.0 rad) + 5× yaw error in c1now is a SYMPTOM of the FPS-broken lateral control (miss → board
seen oblique → alpha stays large), NOT a cause — alpha is a passenger.
**Conclusions:** (1) the uncalibrated identity alpha (cal_s[3]=1.0, board_alpha_0=0) is ADEQUATE — calibrating
it would not improve stationary-board landing; (2) confirms [[input-cal-yaw-lag-anomaly]] that yaw is
low-leverage here; (3) alpha is NOT a confound for the FILTER_WIN/FPS re-run. Caveat: alpha becomes
load-bearing for MOVING/ROTATING targets or large-yaw-offset ICs (must track target heading) — not this case.

---

**REGRESSION ROOT CAUSE FOUND (2026-06-03): the c1 baseline broke because CTRL_ZERO_WXY=1 was a VALIDATED-CONFIG element that I never carried forward. NOT timing/FPS/RTF (that was a red herring).**
Per-signal diff of b13 SP reps vs c1-now reps localized it:
- Control-loop dt IDENTICAL (125 Hz both) — loop rate was NOT the regression (the 58/84Hz I chased was the
  separate IMAGE thread; the PLASMC loop is 125Hz regardless).
- sigma similar, kappa bounded (~0.15, no runaway — DH_D_MAX=5 working).
- THE DIFFERENCE: w_i angular-flow feedforward. ALL 20 b13/b14 reps have max|w_x|=max|w_y|=0.0000 (CTRL_ZERO_WXY=1).
  ALL 10 c1-now reps have LIVE w_x/w_y up to 3.4 rad/s — the badly-calibrated roll/pitch angular flow
  (user confirmed via virtual_img_ang_vel.png: w_x/w_y under-amplitude 2-3x, weakly observable for a planar
  marker / downward cam) feeding the cross(w_i,S) feedforward → inflates a_u 2-4x → saturates body rate (1.0
  vs b13's 0.38) from engagement → diverges (0 SP, 3-7m).
Fix: CTRL_ZERO_WXY=1 (restores b13/b14). Verifying via c1_zerowxy n=10.

**THE VALIDATED CONFIG HAS *THREE* NON-DEFAULT KNOBS THAT MUST BE SET (all dropped during the refactor/reconstruction, each = catastrophic regression):**
1. PLASMC_DH_D_MAX=5  (now a controller.py default — restored)
2. PLASMC_THETA_FLOOR_DEG=60  (now a controller.py default — restored)
3. **CTRL_ZERO_WXY=1  (env-only, NOT a default — THE missing one)** ← zeros the unreliable w_x/w_y angular ff
Plus the env gains: KP/KI/KD=1.4/0.35/0.5031, YAW_OMEGA/GAMMA=0.2, GAMMA_Z=0.375, RHOFOV0_V=315/RHOFOVINF_U=220/
RHOFOVINF_V=300, LANDING_STALE_COMMIT_EXTENT=100, FILTER_WIN=13(default).
RECOMMENDATION: bake CTRL_ZERO_WXY=1 as a controller default (static-target) so it can't be dropped again, OR
fix the w_x/w_y de-rotation (use IMU body-rate, since image w_x/w_y are structurally unobservable (OVERTURNED 2026-06-07 — wx/wy ARE observable w/ the spread board; zeroing is a level-target choice; [[wxy-unobservable-imu-fusion-deferred]])).
Lesson: when a config "won't reproduce," diff the w_i/control SIGNALS of an old-good vs new-bad rep — the
zeroed-vs-live w_x/w_y was invisible in gains/banners but obvious in the logged signals.

---

**IMU-based w_xy de-rotation IMPLEMENTED + A/B TESTED (2026-06-04). Correct & stable, but NOT a win for the stationary target — keep W_XY_DEROT=zero default.**
Added W_XY_DEROT env knob in controller.py (zero|imu|image; default zero = the validated CTRL_ZERO_WXY=1).
'imu' replaces the (formerly-"unobservable") image w_x,w_y (OVERTURNED 2026-06-07 — wx/wy ARE observable w/ the spread board; zeroing is a level-target choice; [[wxy-unobservable-imu-fusion-deferred]]) with the GYRO body rate transformed body->V via Ry(pitch)Rx(roll)
(compass-aligned V), keeping image w_z. Transform validated: bounded ~0.07 rms / 0.19 max (vs image 3.4 garbage).
A/B c1 n=10: imu = 0 SP / 0 PRECISE / 4 SOFT / 0 TL, median xy 0.232m, vel 0.62 vs zeroed = 2 SP / 4 PRECISE /
2 SOFT / 0 TL, median 0.184m, vel 0.30. Verdict: (1) sign/transform CORRECT (zero divergence/TL — a flipped sign
would be positive-feedback catastrophic); (2) but the real de-rotation does NOT improve precision — adds a ~0.15m
steady lateral offset (0 sub-10cm vs zeroed's 4) and lands softer-but-faster. Mechanism: stationary target ->
gentle flight (body rate ~0.07 rad/s) -> de-rotation term is small, and the controller was TUNED with w_xy=0, so
the correct-but-small term perturbs the tuned operating point more than it helps. KEEP zero default. The 'imu' mode
is a validated tool for AGGRESSIVE maneuvers / MOVING targets (larger body rates, where de-rotation matters) — not
the stationary ArUco case. This closes the w_x/w_y-uncalibrated thread: the answer is zero (not IMU) for static.

**W_XY_DEROT=imu confirmed WORSE off-center too (2026-06-04).** Baseline ×0.35 + IMU-wxy (non-zero gyro wx/wy)
IC4 = 2.34m vs zeroed baseline 0.88m. Matches the earlier IC1 result (imu slightly worse than zero). So even
accurate IMU de-rotation perturbs more than it helps for the stationary board, at BOTH centered and off-center.
ZEROING (CTRL_ZERO_WXY=1 default) is correct for the stationary ArUco. The W_XY_DEROT=imu mode stays a validated
tool for MOVING/ROTATING targets only (where real wx/wy is load-bearing).

**IMU de-rotation HAS retuning scope but it is a JOINT retune, not a single combo (2026-06-04).** Signal check:
at ×0.35 gains the IMU de-rotation makes control GENTLER (a_u 1.14 vs 1.36, w_u 0.053 vs 0.073) with slightly
BETTER mid-convergence (s_e_n 0.136 vs 0.156) — it strips rotational-flow contamination, freeing headroom. So the
earlier "IMU worse" was a gain-mismatch (gains tuned for wx/wy=0), KM-fix pattern. BUT: KP=9 + IMU de-rot IC4
(n=2) = high variance: rep2 s_e_n 0.237 / xy 0.74m (FIRST off-center result below the 0.29 plateau!), rep1 0.412 /
2.45m. Body rate SATURATED at 1.0 in both -> the de-rot headroom (visible at ×0.35) does NOT carry to KP=9 (gain
saturates the body rate regardless). So de-rot's gentleness benefit is at LOWER/MODERATE gain, washed out at KP=9.
SCOPE IS REAL (rep2 below plateau + low-gain gentleness) but realizing it needs a JOINT retune: de-rot ON + gain
in the moderate range where body rate isn't saturated + terminal-phase tuning (the ×0.35+de-rot terminal velocity
regressed). NOT a single-knob win. Candidate: de-rot ON + KP~4 (body-rate-unsaturated) + n>=5.