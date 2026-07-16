---
name: per-axis-tuning
description: "User directive (2026-06-03): tune control parameters per individual axis, not uniform scales. Plus the measured x/y loop-gain asymmetry that makes this matter."
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 7faf44bf-c5f1-4b57-a701-f6d868abfdc1
---

**User directive during the SP campaign: "You need to tune control parameter for individual axis, instead of a scale parameter tuning."**

**Why:** uniform scale knobs (PLASMC_PID_SCALE, uniform DSD_CLAMP, …) ignore the systematic per-axis asymmetry and the per-rep variation in which axis fails.

**Measured per-axis facts (floor=60 era, IC1):**
1. **The outer loop is 1.39× hotter on image-x than image-y for the same physical error**: p_10 = [0.889, 1.185] (norm) × cal_s = [1.0986, 1.0562] → x effective gain 1.236 vs y 0.891. K_rp/K_rd/K_ri are symmetric, so this imbalance is never compensated. (image-x → pitch → North motion; image-y → roll → East.)
2. **The touchdown impact axis = the ds_d explosion axis in every rep** (East impacts come from ds_d_y spikes, North from ds_d_x).
3. **Which axis explodes varies per rep** — it's the axis with residual error at terminal phase, not a fixed culprit.
4. The proven-SP rep's own per-axis envelope: ds_d_x ≤ 2.3, ds_d_y ≤ 1.9.

**How to apply:**
- Prefer per-axis env knobs: `PLASMC_KP_X/Y_SCALE`, `KD_X/Y`, `KI_X/Y`, `P2INF_X/Y/Z`, `OMEGA_X/Y/Z`, `GAMMA_X/Y/Z`, `DSD_CLAMP_X/_Y` (per-axis clamp added 2026-06-03).
- Candidate per-axis fixes: (a) equalize axes — KP/KD/KI_X_SCALE ≈ 0.72 to bring x down to y's effective gain; (b) clamp each axis at the SP rep's own envelope — DSD_CLAMP_X=2.5, DSD_CLAMP_Y=2.0.
- When reporting sweep results, always break out per-axis peaks (ds_d_x/y, h_e_x/y, sat_x/y%, κ per axis) — `tools/analyze_explosion_chain.py` prints these.


---

> ⚠️ **WX/WY-CONTAMINATED — INVALID, re-verifying (flagged 2026-06-04).** Everything in the "LateralRestore
> staircase" + "c4 n=10" + "LATERAL GAIN TUNING IS EXHAUSTED" blocks below ran with **CTRL_ZERO_WXY=0** — the
> badly-calibrated w_x/w_y angular-flow feedforward was LIVE (garbage up to **3.55 rad/s**, confirmed in the
> c3 KP=9 rep). That garbage inflated a_u → body-rate spikes → LK breakage. So **"KP=9 → 5/5 TL", "raising
> lateral gain breaks perception", "lateral tuning EXHAUSTED", "KI is the aggression driver", "MATLAB Test 3
> REFUTED", and "binding constraint is perception" are ALL INVALID** — they blame the GAINS for the wx/wy bug.
> The MATLAB IC2 off-center trace (2026-06-04) shows the real differentiator is **K_rp** (MATLAB 9 vs PX4 1.4 ×0.35),
> and MATLAB *converges* with K_rp=9. RE-RUNNING the staircase/KP=9 with CTRL_ZERO_WXY=1 (now default) to get the
> correct conclusions. **Do NOT cite the blocks below.**

**LateralRestore staircase (2026-06-03, faithful baseline: DH_D_MAX=5 + THETA_FLOOR=60 defaults, KM reverted) — raising lateral gain HURTS in PX4; the driver is KI (integral windup), not KP; mechanism is PERCEPTION breakdown.**
IC1, n=5/cell. Peak roll/pitch body rate in parens (LK optical flow breaks >~1.7 rad/s):

| cell | KP/KI/KD | SP | TL | median xy | peak ω (rad/s) |
|---|---|---|---|---|---|
| c1 ×0.35 (validated) | 1.4/0.35/0.50 | 0 (1 precise 0.026m) | 2 | 1.93 m | med 5.2, max 17.6 |
| c2 undo ×0.35 | 4.0/1.0/1.44 | 0 | 0 | 3.59 m | — |
| c3 KP=9 | 9.0/1.0/1.44 | 0 | **5** | **13.7 m** | **med 15.0, max 20.8** |
| c4 KP=9 + manuscript KI | 9.0/0.1/1.44 | 0 | 1 | 1.90 m | **med 2.35, max 6.4** |

Findings: (1) **MATLAB Test 3's "manuscript-lateral beats ×0.35" prediction is REFUTED in PX4** — no cell beat
the validated baseline; raising gains degraded monotonically when KI was high. MATLAB modeled only outer-loop
delay; PX4's binding constraint is perception. (2) c3 = 5/5 TARGET_LOST, "OPTIC FLOW UNAVAILABLE", peak body
rate 15 rad/s (~10× the LK-breakdown threshold), 13-35m fly-aways → unambiguous perception breakdown from
violent maneuvers, NOT lateral divergence. (3) **The dominant body-rate-aggression driver is KI, not KP**:
c4 (KP=9, KI=0.1) has the LOWEST peak rate (2.35) — gentler than c1 (KP=1.4, KI=0.35, 5.2) — because the 10×
SITL KI-boost (1.0 vs manuscript 0.1) causes integral windup → sustained large commands → LK breaks. (4) c1
reproduced the validated gains faithfully but underperformed b13/b14 at n=5 (0 SP, 2 TL vs ~20%/batch, 0 TL) —
likely n=5 variance, but even the validated config occasionally drives LK-breaking transients (peak 17.6).
**Actionable:** the promising untested direction is c4 (KP=9 manuscript + KI=0.1 manuscript, drop the SITL
KI-boost) — lowest body rates, perception-preserving, decent median (rep2 0.30m, rep4 0.69m); needs n=10.
The real ceiling is PERCEPTION (LK robustness under body-rate transients), not lateral gain magnitude.

**c4 n=10 confirm (2026-06-03) — FALSIFIED. KP=9/KI=0.1 is NOT a win; the n=5 staircase was misleading.**
n=10: PRECISE=1 SOFT=1 SP=0 TL=2, median xy 4.06m (vs c1 ×0.35 1.93m), median vel 1.97, body-rate peak
median 8.24 rad/s / max 18.7. The n=5 staircase had claimed c4 was the GENTLEST cell (2.35 rad/s, median
1.90m) — at n=10 that collapsed: body rates 8.24 (worse than c1's 5.2) and median 4.06m. Classic
n=5-winner-fails-at-n=10 ([[feedback_sensitivity_sweep_methodology]]).
Mechanism: dropping KI 1.0→0.1 removed the integral term that corrects SITL lateral drift → steady lateral
offset → 4m median; it did NOT reduce body-rate aggression as n=5 suggested. So the KI=1.0 boost is doing real
work (drift correction); it can't simply be removed. The ×0.35 config (KP=1.4, KI=0.35) balances enough KI for
drift correction against not-too-much KP·KI for windup — which is why it's hard to beat.

**LATERAL GAIN TUNING IS EXHAUSTED** (4 configs: ×0.35, KP=4, KP=9, KP=9/KI=0.1 — none beat ×0.35, none reach
SP). The binding constraint is PERCEPTION (LK breakdown under body-rate transients), not lateral gain magnitude.
**OPEN CONCERN:** this session's post-reboot runs are NOT reproducing b13/b14's ~28% SP (c1 n=5: 0 SP/2 TL;
c4 n=10: 0 SP/2 TL). Either a 3rd regression beyond DH_D_MAX+THETA_FLOOR, or env shift (reboot/FPS/timing).
NEXT: run exact validated c1 (×0.35) at n=10 to confirm baseline still hits ~28% before any further work.

**c1 + IMG_FILTER_WIN=19 (2026-06-03) — savgol-window-match did NOT recover the baseline. FALSIFIED.**
Re-ran validated c1 with FILTER_WIN 13->19 (restores the ~224ms savgol time-window at the post-reboot ~84Hz).
Result n=10: 0 SP, 0 PRECISE, 1 SOFT, 2 TL, median xy 3.74m — statistically identical to the plain broken
c1 baseline (0 SP, 3 TL, median 3.46m). So matching the savgol window alone does NOT fix the FPS regression.
Conclusion: the 58->84Hz shift damages MORE than the frame-count savgol window — the per-frame optical-flow
scaling (LK Δpx/frame) and the dt-coupled control terms (dh_d=Δh/dt, κ-ODE RK5) are also off, and FILTER_WIN
can't touch those. The PROPER fix is to make the loop rate DETERMINISTIC — pin the processing loop to ~58-60Hz
(reproducing b13/b14's actual timing, correcting window+flow+dt together) OR make the rate-sensitive pieces
time-based instead of frame-count. NOT YET TESTED (user paused runs). This is the top open item: the validated
baseline is non-reproducible until the loop rate is pinned, so ALL SITL tuning is on hold behind it.

**IC2-5 ROOT CAUSE = V-FRAME ROTATION mis-directs the terminal correction (2026-06-04), NOT braking.**
Trajectory of an IC2 miss: drone approaches and NEARLY STOPS at 1.2m (speed 0 @60% flight), then ds_d REVERSES
(-2.5) and drives it BACK OUT to 2.65m with error GROWING. Not an overshoot-can't-brake — something actively
pushes it away after closest approach. Cause: the yaw runaway (psi_d->60-120deg, confounded alpha + compass drift)
rotates the V-frame mid-approach -> lateral correction applied in a spun frame -> mis-directed -> drone leaves
target. So the off-center miss AND the alpha/yaw issue are the SAME problem.
FALSIFIED braking fixes: K_rd 0.5031->1.4375 made it WORSE (IC2 2.21->3.48; derivative amplifies centroid noise).
yaw gain 0.2->0.05 only marginal (psi_d still +-81deg). TESTING: hold heading (yaw gains->0) so drone doesn't yaw
-> no compass drift -> V stays aligned -> correction stays pointed at target.

**IC2-5 OFF-CENTER: 5 single-param fixes FALSIFIED (2026-06-04), it is TERMINAL DIVERGENCE not slow convergence.**
Tried (all n=3, baseline=IC2 2.21/IC3 2.0/IC4 0.88/IC5 2.37m, 0 SP): yaw 0.2->0.05 (marginal), K_rd 0.5031->1.4375
(WORSE, derivative amplifies centroid noise), hold-heading yaw=0 (mixed, IC4 9.86m blowup), slower descent
REF_RAD -0.42->-0.30 (WORSE/no help despite +3-4s more flight time). KEY: more time did NOT help -> the lateral
DIVERGES in the terminal phase, it doesn't just converge slowly. Trajectory: drone reaches target (~1.2m,
speed 0) then ds_d reverses (-2.5) and drives it OUT, error grows monotonically. Mechanism = integral windup over
the long off-center approach + 1/Z terminal loop-gain amplification of the large residual offset -> terminal
instability. UNTESTED lever pointing the right way: LOWER outer gain (K_rp/K_ri) reduces terminal loop gain +
windup -> may stabilize, at the cost of IC1 precision (the precision-vs-robustness frontier). Honest read: IC1
sharp-gain precision and IC2-5 off-center stability may be in genuine tension; resolution is likely error-scheduled
gain (gentle far, sharp close — scale-free) which is a DESIGN change, not a single param. Stop single-param shots.

**IC2-5 dormant-κ ROOT CAUSE refined (2026-06-04): it is the FUNNEL not engaging, not N.** N_x/y 0.02->0.1
FALSIFIED — κ STILL didn't grow (max 0.156, decayed to 0.10). The κ-ODE equilibrium κ_eq≈N|σ|/P means κ only
rises above κ_0=0.156 when |σ| > P·κ0/N (≈12 at N=0.02, ≈2.3 at N=0.1). But σ_x only reaches 1.08 off-center.
So κ stays pinned regardless of N. WHY σ is small: the funnel BARRIER never engages — flow error (~5) stays
within the WIDE funnel (p decays 25->only 11, vs error 5 -> |e/p|<0.5), so the barrier transform doesn't amplify
e into a large σ. MATLAB (same p_2, same N) DOES engage (error reaches boundary, σ blows up, κ grows). The PX4
funnel p_2 (25->2.5, MATLAB-tuned) is too wide for the PX4 calibrated-flow error SCALE -> whole barrier->σ->κ
adaptive chain dormant. LEVER = funnel width p_2 (narrow so it engages the ~5 flow error -> σ grows -> κ grows ->
converge). Risk: breach->blowup if κ can't keep up (funnel=gain). Testing p_2_0_xy 25->8.
NOTE the funnel-vs-flow SCALE mismatch likely traces to flow calibration (sensor_cal_hw) — the funnel was tuned
for MATLAB flow units; if PX4 flow is scaled differently the funnel no longer matches. May be the real fix.

**IC2-5 ROOT CAUSE CONFIRMED (2026-06-04): dormant-κ from FUNNEL-vs-FLOW SCALE MISMATCH.**
Narrowing funnel p_2_0_xy 25->8 ENGAGED the barrier: sigma_x 1.08->2.61, kappa_x 0.156->0.31 (FIRST time κ moved
off-center). Proves the chain: wide funnel -> barrier dormant -> small σ -> κ pinned -> no convergence. The PX4
funnel p_2 (MATLAB-tuned, 25->2.5) is too wide for the PX4 calibrated-flow error scale (~5) -> never engages,
whereas MATLAB's flow reaches the funnel and κ grows. BUT engaging κ via narrow funnel trades into HIGH touchdown
velocity (2.6-3.5 m/s) + occasional blowup (6.97m) — the precision-softness frontier (more gain = converges but
hard). IC4 improved (0.84-1.06m) but 0 SP, high vel. So off-center IS fixable in principle (κ CAN engage) but
needs funnel+adaptation+softness CO-TUNING, not a single knob.
PRINCIPLED FIX (the real lever): match the funnel scale to the PX4 flow-error scale so the barrier engages
NATURALLY (as in MATLAB) without aggressive narrowing — i.e. scale p_2 down ~3-5x OR rescale the optic-flow cal
(sensor_cal_hw) up so PX4 flow matches the MATLAB units the funnel was tuned for. This is a CALIBRATION-scale
issue at heart (funnel was never re-tuned for PX4 flow units). 7 IC2-5 single-knob tests done; stop blind knobs,
the fix is funnel/flow SCALE matching + softness co-tune.

**IC2-5 narrow-funnel+slow-descent: FALSIFIED on 2-rep IC4 (2026-06-04).** xy decent (0.74-0.93m) but vel HIGH
(2.5-5.1 m/s) — slow descent did NOT soften the engaged-κ aggression. Confirms the PRECISION-SOFTNESS FRONTIER
off-center: κ dormant (wide funnel) -> low gain -> doesn't converge xy (miss); κ engaged (narrow funnel) -> converges
xy but high touchdown vel (hard). Cannot get both with this funnel/κ mechanism by simple knob combos. 8 IC2-5 tests
total. The engaged-κ drives hard AT touchdown (funnel narrows = engages MORE late); for SOFT you'd need κ to engage
EARLY (converge at altitude) then RELAX terminally — i.e. funnel DECAY-PROFILE shaping, OR the principled funnel/
flow SCALE match so κ engages proportionally (like MATLAB, which gets both). This is deeper retune/recal work, not
a single/combo knob. STOP single-knob shots; decision point with user.

**CORRECTION (2026-06-04): the "dormant-κ is the IC2-5 bug" hypothesis is WRONG.** MATLAB sigma_trace
(Datasets/Phase3/matlab_sigma_trace.mat, IC1) shows MATLAB's κ is ALSO dormant — κ_x 0.125->0.125 (growth 1.0x),
converges to xy 0.015m. So κ NOT growing is NORMAL; convergence comes from the FIXED SMC gain + funnel, not κ
adaptation. My earlier comparison was flawed (PX4 off-center vs MATLAB CENTERED). The narrow-funnel test that made
κ grow (0.31) "worked" by raising the fixed gain (funnel=gain), not by fixing an adaptation defect. So the real
IC2-5 differentiator is NOT κ. Need a MATLAB OFF-CENTER (IC2=[2,2,-5]) internal trace to find the actual difference
(σ/flow/funnel/PID scales). LEADING untested hypothesis: K_ri integral WINDUP — PX4 K_ri=0.35 vs MATLAB 0.1 (3.5x);
the long off-center approach winds up the outer-PID integral -> overshoot -> drive-out (matches the trajectory:
reach target, ds_d reverses, error grows). Testable cheaply (2-rep, K_ri 0.35->0.1). 8 IC2-5 tests done; re-diagnose
before more — get MATLAB off-center reference OR test K_ri windup.

**IC2-5 REAL DIFFERENTIATOR FOUND (2026-06-04, MATLAB IC2 off-center trace vs PX4):** NOT funnel/flow/kappa
(all similar: MATLAB flow-err rms 1.64 vs PX4 2.02, funnel 25->8.7 vs 25->10.9, kappa const both). It is the
OUTER-LOOP CONVERGENCE SPEED. MATLAB drives centroid error s_e_n 0.52->0.09 by mid-flight (closes offset at
altitude); PX4 s_e_n stuck ~0.55-0.65 (converges 6.4x too slow) -> offset still open at terminal 1/Z phase ->
divergence (s_e_n 0.36->1.30). Initial s_e_n matches (0.54 vs 0.62, same 2m offset) so it's convergence rate not
perception scale. ROOT: MATLAB outer PID K_rp=9 vs PX4 1.4 (x0.35 detuning) = 6.4x weaker. PX4 CAN'T use K_rp=9 —
staircase showed it breaks LK (body-rate spikes from amplifying noisy centroids -> 5/5 TL). So off-center needs
FASTER convergence via a PERCEPTION-SAFE path: K_ri (integral) is SMOOTH (averages out centroid noise) so it adds
convergence gain WITHOUT the body-rate spikes K_rp causes. Test direction = K_ri UP (0.35->0.7), not down.
This is the precision/perception/convergence frontier, but K_ri is the un-tried perception-safe lever.

---

**✅ CORRECTED CONCLUSIONS (2026-06-04 redo with CTRL_ZERO_WXY=1 — supersedes the contaminated block above).**
Re-ran KP=9 with wx/wy zeroed:
| condition | contaminated (wx/wy live) | REDO (wx/wy fixed) |
|---|---|---|
| KP=9 IC1 | 5/5 TL, 13.7m, peak 15-20 rad/s | **0 TL, 1.3m, peak 1.0 rad/s** |
| KP=9 IC4 | (was off-center, similar) | **0 TL, ~2m, peak 1.0, s_e_n@mid 0.29 vs baseline 0.56** |

**The corrected truths:**
1. **"KP=9 breaks LK perception / 5/5 TL" is FALSE** — it was the wx/wy garbage (3.55 rad/s feedforward → body-rate
   spikes). With the fix, KP=9 caps cleanly at W_U_MAX=1.0, ZERO TL. Perception was NEVER the binding constraint.
2. **"LATERAL GAIN TUNING IS EXHAUSTED" is FALSE** — we abandoned restoring K_rp toward manuscript based on a bug.
   KP=9 is fully usable.
3. **KP=9 DOES help off-center convergence** (s_e_n@mid 0.29 vs baseline 0.56, toward MATLAB's 0.09) — confirms the
   MATLAB IC2 trace: K_rp is the real differentiator (MATLAB 9 vs PX4 1.4 ×0.35), [[convergence-ordering]] Test 3
   VINDICATED.
4. **New binding limit = W_U_MAX=1.0** (body-rate clamp), not perception. KP=9 saturates it -> can't converge as
   fast as MATLAB (which has no clamp). And W_U_MAX=1.0 was itself set to protect LK from the wx/wy spikes that NO
   LONGER EXIST -> likely safe to raise now. PROMISING UNTESTED LEVER: KP=9 + higher W_U_MAX (LK-safe post-fix).
5. **At IC1, KP=9 (1.3m) is still worse than ×0.35 (0.2m, 2 SP/10)** — but via NOISE amplification (high KP on small
   centroid error), NOT TL. So ×0.35 stays best for centered IC1; the precision-vs-convergence tension is real but
   it is NOT the false "perception breakdown". MATLAB does both with constant K_rp=9 (IC1 1.5cm, IC2 6.4cm) because
   it has no W_U_MAX clamp + clean centroids -> the PX4 gap = W_U_MAX clamp + centroid noise, both addressable.


**IC2-5 convergence plateau is PERCEPTION-limited (lag+noise), NOT gain (2026-06-04).** Off-center s_e_n@mid
plateaus at ~0.29-0.32 ROBUSTLY: KP=9 (0.29), KP=15 (0.32, xy WORSE 3.5m), W_U_MAX 1->2 (no change, body rate
only 1.4). So beyond KP=9 the gain path is EXHAUSTED. The plateau is the PX4 centroid perception: savgol(13)
lag -> 0.29, centroid-KF (less lag but q=5 noisier) -> 0.42 (WORSE), MATLAB (clean, no lag/noise) -> 0.09.
The lag-vs-noise tradeoff caps PX4 at ~0.29 vs MATLAB 0.09. This is the GOOD kind of perception limit (sensor
lag+noise), NOT LK breakage. Levers left: (1) better centroid filter (low-lag AND low-noise — the q=5 KF was
too noisy; needs its OWN cal per user, output recal with IMG_FEATURE_FILTER=kf); (2) reduce centroid noise at
source. KP=9 IS the right gain (matches MATLAB, no LK break post-wxy-fix); the residual gap to MATLAB is the
centroid lag/noise, which is the next real lever.