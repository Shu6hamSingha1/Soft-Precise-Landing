---
name: reference-tuning-trajectory
description: "Chronological trajectory of every PX4 PLASMC tuning trial — what parameter changed, why, and what it achieved. The connected spine that ties the per-decision memories together; read alongside the tune-plasmc skill."
metadata: 
  node_type: memory
  type: reference
  originSessionId: 7415f420-9591-41b1-8349-bb9361a8dc82
---

> ⚠ **POINTER UPDATE 2026-07-03:** 'later eras in project_current_state' is stale — project_current_state is itself historical; the live era ledger is `px4/MEMORY.md` (top banners + entries).
> ⛔ SUPERSEDED/CORRECTED 2026-06-26: The era-by-era historical log is valid AS history; but the present-tense conclusions — theme #10 'current binding limit = stochastic perception / next lever pyramidal LK', the 'Where it stands' open-frontier = code-level perception robustness, and the ~100-170ms lateral-lag 'architectural floor' figure (superseded by the 38ms rate-loop lag) — are obsolete. The PX4 lateral "wall" was a gain-parity bug + the velocity-damping lever (tighten the lateral flow funnel XI2_xy), NOT a perception/architecture/lag limit; the combined sliding surface σ=ζ_h+χ_r·ζ_r is baked default-on (10/10 bounded landings). See [[feedback_flow_funnel_zetah_works]]. Content below kept as history.

# PX4 PLASMC tuning trajectory (Jun 2–10, 2026)

The **connected timeline** of the whole SITL tuning campaign — what each trial varied, *why* (the hypothesis), and what it achieved. The playbook (methodology, dead-ends, parameter inventory) is the **tune-plasmc skill** (`.claude/skills/tune-plasmc/SKILL.md`); the raw quantitative record is `PX4_Gazebo/test_data/Landing_Test/parameter_record.ods` (4 sheets); the per-decision *why* lives in ~50 `feedback_*`/`project_*` memories, linked inline. This file is the index that orders them into one arc.

**Trial-ID convention here:** `G#` = PX4_Gain_Record sheet (old eras), `NC#` = PX4_NewCal_Record sheet (current era). The two sheets both renumber from 1, so always prefix.

**SP = SOFT+PRECISE landing** (xy<10cm AND vel<0.2 m/s). The whole campaign is the hunt to raise SP rate above the ~2% historical floor.

---

## The master variable: CAL REGIME (results only compare *within* a regime)

Nothing in the trajectory makes sense without this. The image→[h;w] sensor cal was re-derived three times; a result under one cal does NOT carry to another. See [[feedback_historical_cal_confound]], [[project_tuning_campaign_newcal_reset]], [[project_camera_calibration_status]].

| Regime | When | Cal | Status of its results |
|---|---|---|---|
| **R0 (broken)** | pre-Jun | legacy diag, 2–13× mis-scaled (controller ran at 0.08–0.53× design gains) | ~2000 reps CONTAMINATED. "Lag is the floor" conclusions confounded; only the *lag timing* numbers survive (38ms roll / 52–61ms / 287ms yaw). |
| **R1/R2 (multisine)** | Jun 2–4 | 4-run multisine, still had V-frame g-sign bug | G1–G59. The "28% SP @10cm" (G46) is from here → now a **HYPOTHESIS, not a result**. |
| **R3 (honest)** | Jun 5–10 | honest 8-run yaw-only-GT cal (`d60973a`) | NC1–NC49 = the current, trustworthy campaign. |

---

## ERA 0 — Reference frame (MATLAB), why PX4 is hard

MATLAB hits 10/10 SP at the same IC; PX4 hit 0/30 early. The gap is SITL-specific and **lag is the architectural ceiling**, quantified by the MATLAB delay-robustness sweep (MATLAB_Test_Record):
- Manuscript gains hold **100% SP to 100ms *outer* delay**, die past it; only to 50ms *inner* delay. **Delay *location* matters more than magnitude.**
- PX4 lateral lag ~100–170ms sits **exactly on the outer-delay cliff** (100%→60%→0%) → explains IC1 variance. Yaw lag 287ms is **past** the cliff → explains yaw divergence.
- PX4-validated gains (K_rp=1.4, K_ri=0.35, K_rd=0.503, yaw 0.2) trade low-delay performance for a **~40% SP floor out to 300ms**; MATLAB predicts ~40% at PX4-like delay, PX4 delivered 28% (G46) — the residual gap = perception effects MATLAB doesn't model.

Phase 1–5 diagnosis memories (CAL-CONTAMINATED, but the lag/noise/IC structure holds): [[feedback_phase1_matlab_baseline]], [[feedback_phase2_loop_latency]], [[feedback_impulse_response]], [[feedback_phase3_ic_robustness]], [[feedback_phase4_sensor_noise]], [[feedback_phase5_sigma_divergence]].

---

## ERA 1 — Multisine-cal campaign (Jun 2–3, G1–G46)

Opened with the **multisine cal**, which **collapses IC2-5 for ANY gain config** ([[project_multisine_cal_ic25_collapse]]) — old-cal IC2-5 was 0.7–2m, multisine made it 5–7m. So this era is really an IC1 campaign.

**G1–G16 — funnel/PID exploration, found the explosion chain.** Varied KP_far (9→5), p_2_inf (×2, ×4), uniform P, per-axis funnel, K_rd, DSD_CLAMP. Lesson after lesson that **n=1 is noise** (G3 gave 0.634 *and* G10 gave 2.87 at *identical* config → variance band ~4.5× — [[feedback_sensitivity_sweep_methodology]]). G16 (n=6 defaults) traced the **explosion chain** end-to-end: `ds_d 1/Z spike → dh_d pins at ±50 clamp → θ~50 → κ-ODE runaway (10–100× κ_0) → a_u 400–7000 m/s² → hard impact`. This chain is THE failure mode; the rest of the campaign bounds each link.

**G17 — `DH_D_MAX 50→5`: first mechanism fix.** The clamp value feeds Θ_norm; 5.0 breaks the κ-runaway (κ 4.5–98→0.16–3.5, a_u 7051→33, rel_vel 9.5→0.4 m/s), xy unchanged (lag-limited). [[feedback_dsd_touchdown_spike]]. (Later re-classified as physics-guard, restored to 50 — see ERA 4.)

**G18–G20 — IC2-5 gate: both arms collapse 5–7m.** DH_D_MAX=5 *passes* the gate (no regression) but **both arms collapsed** → confirmed the multisine cal, not the gain, owns the off-center failure. PID_SCALE=0.54 (cal-compensation 1/1.85) improved *vel* every IC but xy unchanged → **off-center xy is NOT loop-gain-limited**.

**G21–G34 — FoV-cone deadlock → SP #6, then the hard wall.**
- `THETA_FLOOR=60` (disables the cone-clamp d_min collapse that strangled 94–100% of terminal correction) → **SP #6 ever** (G22: xy=0.060, vel=0.149, *first mechanism-driven SP*). [[feedback_fov_cone_clamp_deadlock]].
- Then a systematic refutation of every other knob as a κ-fix: per-axis KP equalization (wrong direction — raise the cold axis, don't lower the hot one), DSD per-axis clamp at the SP-rep envelope → **49m flyaway** (the envelope is an *outcome, not a recipe*), N×0.1 (**N cancels in κ_eq**), P×10 (**κ_eq=Θ·G·|σ|/P, would need P~9000**), Xi2 widening (**funnel width = gain coupling — never widen a funnel**).
- **G34 CAMPAIGN CONCLUSION:** saturation chain fully traced; *no control parameter owns the lag*; remaining fixes are architectural (uXRCE-DDS / MC_RATE airframe edit). [[feedback_fix_causes_not_limits]], [[feedback_strict_coord_descent_dry]].

**G35–G46 — the convergence-ordering breakthrough (→ 28% SP, the R1/R2 hypothesis).** Per-channel **bandwidth matching**: lateral KP/KI/KD ×0.35, yaw ×0.4, p1 envelope→sensor (rho0_v=315/rhoinf=220,300), GAMMA_Z ×0.5, KLT=20, span-based stale commitment (100px) — all admissible knobs, **zero clamps**, controller to gear contact. [[feedback_convergence_ordering]].
- G35–37: first SPs (0.0096/0.0075 = project record, beats MATLAB worst-case 5×) but **INVALID — last 25cm was open-loop handoff** (zf=0.2m is *gear height*, not a controller envelope — the controller must control *through* touchdown). Handoff knob removed.
- G38–40: honest re-runs (control to gear contact). 2 SP @8cm. Failures root-caused: rep = perception breakdown at gear contact (16 KLT/stale events), rep = z-funnel p2inf_z too narrow → bounce.
- G41 ABORTED: widening p2inf_z backfired (**2nd proof: never widen a funnel to absorb a transient**).
- G44 (stale-commitment + GAMMA_Z): **catastrophic mode ELIMINATED** (worst 9.41m→2.54m); found the commitment-extent bug (distance-from-center fires for small-marker-at-edge → switched to corner-to-corner SPAN).
- **G46 CAMPAIGN FINAL (pooled n=25): 28% SP @10cm** (xy med 0.195, best 0.018, vel med 0.179), vs ~2% historical. ⚠️ Under multisine cal → now a hypothesis.

---

## ERA 2 — New-cal prep & the great regression debug (Jun 4, G47–G59)

The 28% config **regressed to 0 SP** when re-run. The debug is the most important methodology lesson of the campaign.

**G47 — RESTORED via `CTRL_ZERO_WXY=1`.** The regression cause was **3 missing config knobs** (`CTRL_ZERO_WXY=1`, `DH_D_MAX=5`, `THETA_FLOOR=60`), **NOT timing/FPS/cal** (those were *red herrings*, found by signal-diffing an old-good vs new-bad rep). CTRL_ZERO_WXY zeros the badly-calibrated wx/wy angular-flow feedforward — left live it overdrove `cross(V_w,S)` → body-rate saturation → 0 SP. IC1 n=10 reproduced b13/b14 (2 SP, median 0.184m). All 3 knobs are now baked defaults.

**G48–G59 — the IC2-5 yaw-runaway saga (root cause ≠ where it looked).** Off-center starts drove `psi_d→±180°`. Tried: yaw gains 0.05 (G49), K_rd=1.4375 manuscript braking (G50, worse — D amplifies centroid noise), yaw=0 hold-heading (G51), slow descent -0.30 (G52, **more time doesn't help → it's terminal DIVERGENCE not slow convergence**), N=0.1 + narrow p_2_0 (G53–54, found **κ is DORMANT off-center** — leakage-dominated, funnel too wide to engage). Then **3 alpha redesigns** (geometric-unify, moment-avg, moment-2π) — **all hit 180°**; geometric reverted (`0008ba1`) because the yaw SMC is **tuned to the moment-based alpha convention** (π-period, [4,3,2,1] weights, −0.9379 offset) = load-bearing ([[feedback_moment_yaw_canonical]]).
- **ROOT CAUSE PINNED (G58):** **compass drift at landing START.** EKF yaw drifts ~77° during takeoff+IC; the IC rig holds/gates on EKF yaw → the drone *begins* the descent physically yawed ~77° → psi_d→180°. **Alpha is correct** (alpha_start≈GT_yaw_start every rep, tracks GT r=1.00). Fix = **test rig, not controller**. [[feedback_yaw_compass_drift_ic_start]], [[feedback_use_gt_yaw_not_ea]], [[feedback_matlab_yaw_square_start]].
- **G55 — the contaminated-staircase correction.** The earlier "KP=9 → 5/5 TL, breaks LK, lateral EXHAUSTED" verdict was **wx/wy-contaminated** (garbage w_y 3.55 rad/s). REDO with CTRL_ZERO_WXY=1: KP=9 → 0 TL, peak w_u 1.0 (vs 5/5 TL, 15–20 rad/s). **Perception was NEVER binding; KP=9 is usable.** [[feedback_per_axis_tuning]] rewritten.

---

## ERA 3 — Honest cal + perception/visibility infrastructure (Jun 5–7)

Not gain tuning, but it **reset the baseline** (every ERA-1 result became gain-starved / regime-1-2). Git milestones:
- `d60973a` **honest 8-run sensor cal** (yaw-only GT) — R3 begins. [[reference_cal_data_provenance]], [[reference_aggregate_calibration]].
- cbf2 **visibility CBF** built up cone→cone0→cbf1→cbf2 (`e6a072c`…`21d27b0` default cbf2): theta-QP on the camera-frame feature, two-phase δ. [[feedback_cbf_theta_cap]], [[project_cbf_visibility_design]].
- `0ed583d` **FLOW_FUSE_RING ON** by default (later EXONERATED as a no-descent cause — [[feedback_ekf_default_breaks_descent]]); centroid **KF default** (`6e0b44f`, savgol added lag+noise — [[feedback_centroid_kf_default]]).
- `b5b3327` **"wx/wy unobservable" OVERTURNED** — it was an under-excitation + V-leveled-frame artifact ([[feedback_wxy_unobservable_imu_fusion]]); but observable≠calibrated → CTRL_ZERO_WXY stays for stationary target.

---

## ERA 4 — Honest-cal campaign (Jun 7–10, NC1–NC49) — the current arc

The trustworthy campaign. Standing baseline: P=5/5/5, E=[varies], KP=9, KI=1, DH_D_MAX=5, FLOOR=60, SEN_FUNNEL=1, cbf2, BODY_YAW_SOURCE=alpha. [[feedback_newcal_tuning_results]], [[project_tuning_campaign_newcal_reset]].

**NC1–3 — bound κ via P (κ_eq ∝ 1/P).** Default E=1/P=1.5 → catastrophic κ_z runaway (39). `P_Z 2.5→5` tames κ_z 39→6.5 (MATLAB parity) → **baked**. P_XY→3 interim (later →5).

**NC4–7 — KP, and the n=1 trap (again).** KP 9→12 looked *best at n=1* (1.53m) → **debunked at n=5** (mean 6.1m, 0 SP). `KI>1` → integral windup → κ-runaway returns (dead-end). `KP>12` → overshoot (dead-end). [[feedback_sensitivity_sweep_methodology]].

**NC8–10 — cbf2 is MASKING the κ-runaway (key insight).** `W_U_MAX>1.7` breaks LK (TARGET_LOST). `THETA_FLOOR 20→60` (relax cbf2) *alone* → WORSE (10.4m) because it **exposes the κ-runaway cbf2 was containing**. `E=2.5 all-axes` → κ fully bounded + lateral **0.04m PERFECT** but **HOVER** (too soft to descend). → **cbf2 is a safety net, not a controller; bound κ at the control level.** [[feedback_dont_conclude_lag_floor]] (don't call a masking-clamp "load-bearing"; one-knob-one-job: P bounds κ, E sets stiffness).

**NC11–24 — E-decoupling + the yaw sub-campaign + P_XY=5.**
- E decoupled: stiff E_XY for lateral, low E_Z for descent. yaw **2π-wrap fix** kills the limit cycle (`ad55700`, baked).
- Yaw tuning swept exhaustively: PSID_RATE 0.7→1.0 (large-yaw reps now slew to ~0, baked — but overshoot past zero = 287ms lag), **conditional ie_a integration** replaces the _ie_a_clamp band-aid (windup peak 2.0→0.4, baked — [[feedback_clamps_during_tuning]]), KR_YAW {2,1,0.5} (2.0 optimal), YAW_OMEGA 1.0 (worse — SS error is unsettled transient not constant offset), YAW_GAMMA 1.0 (wrong way). **Conclusion: residual yaw overshoot = PX4 287ms rate-loop lag; inner-loop yaw levers exhausted.**
- `P_XY=5` bounds lateral κ amp at E=1.5 (a_u 33589→440) → **baked P=5/5/5.**

**NC25–42 — descent investigation + the KP×E coupling.**
- TERMINAL_STABILIZE (freeze on marker-fill) → dead-end (holds *accel* not *velocity* → keeps falling, reverted `f2891bf`).
- `SEN_FUNNEL=1` was silently OFF in prior runs → baked (`f812895`); with it, KP=12 is *safe* (gates the t=0 LK spike). **KP×E COUPLING discovered ([[feedback_kp_e_coupling]]): KP=12 only with E_XY≤1.5; KP=9 with E_XY≥2.5.** KP=12+E=2.5 is WORSE (4.79 vs 3.80) — wide E keeps κ≈κ_0 so higher KP just drives h_d windup faster → earlier funnel breach.
- `KI=0.35` (MATLAB parity) → **dead-end** (TL 4/5 — KI=1.0 is load-bearing for FoV retention). `E_Z=0.5` marginal best. `N_Z=0.05`/`N_XY=0.05`/`GAMMA_Z>0.75`/`tau_ua=0.3`/`P_XY=3`/`P2INF_Z≠1.5`/`EXTENT≠70` all confirmed defaults or dead-ends.
- **Binding failure surfaced: stochastic LK/ArUco collapse** — 1–2 TL per 5 reps even at perfect IC; LK dynamic-range ceiling ~2 m/s; NOT gain-tunable. The xy_min (~2m) vs xy_med (~4–6m) gap is *entirely* stochastic perception. [[feedback_lk_dynamic_range_limit]], [[feedback_marker_detection_stale]]. **All gain-side levers exhausted.**

**NC43–47e — D-term + outer-funnel analysis → the current winner.** [[feedback_dterm_outer_funnel_analysis]], [[feedback_dh_d_overload_lpf]].
- `DH_D_MAX=50` + `W_U_MAX=1.5` (relax clamps to expose bare law) → catastrophic 15.7m → **both clamps are load-bearing physics-guards, not band-aids** (DH_D_MAX=5 prevents 10× cmd amplification at first step; W_U_MAX>1.7 breaks LK).
- `TAU_DS=0.05` (50ms LPF on ds_d) → dead-end (adds outer-PID lag; can't help when the *error itself* is large).
- `K_rd=0` alone → dead-end (11.96m — D-term was load-bearing for drift arrest). But then **`gamma_s` sweep** (outer-funnel contraction rate, the primary binding constraint — funnel too slow → P-term too weak to arrest drift before 1/alt amplification): 0.1→0.2→0.5→0.8→**1.0 = WINNER** (NC47e: median **1.32m**, lowest κ; the logged "1/5 SP at 0.03m" is ⚠️ **UNVERIFIED** — no saved recording, likely a frozen-GT artifact, see [[feedback_false_sp_frozen_gt]]). **`K_rd=0 + gamma_s=1.0` baked** — fast funnel pressure replaces the D-term's damping role without the D-term's noise spikes.

**NC48a–e — descent bootstrap.** gamma_s=1.0 fast-centering caused a 6m **hover** (κ_z too slow to start descent). `KAPPA0_Z 0.31→1.0` bootstraps descent in 0.4–0.9s — but exposed κ_z runaway (107). `KAPPA_MAX_Z=3.0` + **κ-freeze on outlier-containment axes** fix it → **baked** (NC48e: xy_med 2.34m, n=5, 0 TL). [[feedback_descent_softness]]. (Singhal-2025 outlier containment on the barrier, `bf21e52`, feeds the κ-freeze.)

**NC48d/49 — θ_norm=616 root cause + the current pending trial.** The κ_z runaway investigation traced θ_norm=191–616 to **KLT off-screen drift**: when ArUco fails and the KLT fallback extrapolates corners off-screen, `s[0]→3.15 rad` AND `w_z→4.54 rad/s` (same source) → `cross(dw, s[:3])→689`. **Fix: stop KLT when any corner exits image bounds** (`img_data.py`, `d186180`). [[feedback_theta_norm_klt_drift]]. **NC49 = CURRENT TRIAL, n=0 — IC1 n≥5 validation pending, then IC2-5 gate.** [[project_current_state]].

---

## Cross-cutting themes — *why* parameters kept moving

1. **Cal regime is the master variable.** Half the "discoveries" were re-discoveries under a new cal; always check the regime before trusting a number.
2. **One failure chain explains ~everything:** lateral drift → h_d windup → funnel breach (σ>E) → κ runaway → a_u blowup → hard impact/TL. Each trial bounds one link.
3. **κ-runaway bounding:** **P** bounds it cleanly (κ_eq∝1/P); **E** bounds it but *softens tracking* (one-knob-one-job violation); **cbf2 MASKS it** (relaxing cbf2 *exposes* it, which is the point); **N, Γ are mathematically excluded** as fixes.
4. **Lag is the architectural floor** (287ms yaw, ~100–170ms lateral) — every inner-loop/yaw lever dead-ends against it; MATLAB confirms PX4 sits on the outer-delay cliff. But [[feedback_dont_conclude_lag_floor]]: don't *invoke* lag while control levers remain.
5. **Funnel width = gain coupling** — proven ≥3× (Xi2, p2inf_z): **never widen a funnel to absorb a transient.**
6. **n=1 is noise** — debunked repeatedly (G3/G10 variance 4.5×; NC4 KP=12). [[feedback_sensitivity_sweep_methodology]], [[feedback_ic_validation]].
7. **Clamps are band-aids** — DH_D_MAX/THETA_FLOOR/DSD_CLAMP/_ie_a_clamp all removed or re-justified as physics-guards; tune the bare law, re-engage protective clamps after. [[feedback_clamps_during_tuning]]. Removed-knob history in the `Removed_Parameters` sheet.
8. **Per-axis, not uniform** — image-x is 1.39× hotter than image-y; uniform scales can't fix it (all `_SCALE` knobs removed 2026-06-03). [[feedback_per_axis_tuning]].
9. **Yaw IC2-5 failure = compass drift at START**, a test-rig bug, not the controller/alpha. [[feedback_yaw_compass_drift_ic_start]].
10. **Current binding limit = stochastic perception** (LK dynamic-range ~2 m/s), not gains. Next lever is code-level (pyramidal LK levels 2→3), not a knob. [[feedback_lk_dynamic_range_limit]].

## Best results, in order
SP #6 (G22, R2): xy 0.060. Project-record SP (G35, R2, but INVALID open-loop handoff): 0.0096. R2 campaign (G46): 28% SP @10cm — hypothesis. **R3 current best (NC47e): xy_med 1.32m → with bootstrap (NC48e) 2.34m n=5.** The NC47e "1/5 SP at 0.03m" is ⚠️ **UNVERIFIED** (2026-06-10 audit: the only sub-10cm rep in all saved R3 data is a frozen-GT artifact, marked FALSE) — **no verified SP under the honest cal**. See [[feedback_false_sp_frozen_gt]].

## Where it stands
NC49 (KLT-bounds fix) baked, **IC1 n≥5 pending → then IC2-5 gate**. Gain-side levers exhausted under R3; the open frontier is **code-level perception robustness** + the **IC-rig compass-drift fix** for IC2-5 yaw.
