# PLASMC Controller — Comprehensive Parameter Analysis & Failure Diagnosis

_Rewritten 2026-06-10 under the **honest sensor cal**. Supersedes the 2026-05-24 version (in git history),
whose "1205 reps / SP = 0.08% / lag is the architectural floor" conclusions were **confounded** by a 2–13×
mis-scaled calibration that ran the controller at 0.08–0.53× its design gains. Sources of truth:
`test_data/Landing_Test/parameter_record.ods` (sheet `PX4_NewCal_Record`, trials NC1–55), `src/controller.py`,
and the memory `reference-tuning-trajectory`._

---

## 0. The master variable — calibration regime

No result is interpretable without its cal regime. The image→`[h;w]` sensor cal was re-derived three times:

| Regime | When | What it means for the numbers |
|---|---|---|
| **R0 (broken)** | pre-June | ~2000 reps at 2–13× mis-scale → controller at 0.08–0.53× gains. **All conclusions confounded.** Only the lag *timing* (38 / 52–61 / 287 ms) survives. |
| **R1/R2 (multisine)** | Jun 2–4 | The "28% SP @ 10 cm" headline (trial G46) lives here — now a **hypothesis**, not a result (still had the V-frame g-sign bug). |
| **R3 (honest, 8-run)** | Jun 5–10 | `d60973a`. The **only trustworthy** campaign (NC1–55). Everything below is R3. |

A number from one regime does **not** transfer to another. This is the single most common analysis error in this project's history (see memory `feedback_historical_cal_confound`).

---

## 1. Executive summary (R3)

Under the honest cal the controller engages correctly; the campaign is now a hunt to convert "lands within a few metres" into "repeatably soft-and-precise". Progression of the best IC1 configs:

| Config (n=5 IC1) | xy median | Note |
|---|---|---|
| KP=9, E=[2.5,2.5,0.5], P=5 ("best stack" Jun-9) | 3.80 m | superseded |
| **K_rd=0 + gamma_s=1.0** (NC47e, baked) | **1.32 m** | current winner on median xy — but the logged "1/5 SP at 0.03 m" is **UNVERIFIED** (no saved recording; see §4) |
| + KAPPA0_Z=1.0 bootstrap (NC48e, baked) | 2.34 m | descent now starts in 0.4–0.9 s; 0/5 TL |

**There is no *verified* SP under R3 at all** (2026-06-10 audit): across all 101 Jun-9/10 recordings the *only* sub-10 cm rep is a frozen-GT logging artifact (marked FALSE), and the headline NC47e "SP at 0.03 m" has no saved recording behind it (§4). The story is **not** "lag is the floor". Two mechanisms dominate:

1. **The κ-runaway explosion chain** (§2) — the controller-side failure, and the thing every gain bounds.
2. **Stochastic LK/ArUco perception collapse** (§4) — the current *binding* limit. 1–2 TARGET_LOST per 5 reps even at a perfect IC; the gap between xy_min (~2 m) and xy_median (~4–6 m) is *entirely* stochastic perception, **not** gain-tunable. The next real lever is code-level (pyramidal LK), not a knob.

Lag (38 ms roll / 287 ms yaw) is real and bounds the yaw loop, but it is the **last** explanation, not the first — relaxing a safety-net clamp that makes failure *worse* means the clamp was *masking* an under-tuned law, not that lag is load-bearing (memory `feedback_dont_conclude_lag_floor`).

---

## 2. The dominant failure mechanism — the κ-runaway explosion chain

Almost every catastrophic rep (hard impact, fly-away, TL) is one chain. Each link is owned by a parameter, and the whole campaign is bounding the links:

```
lateral drift (perception/IC)
   → outer PID demands h_d:  h_d ≈ K_rp · s_e_n / G_s   (windup as s_e_n grows)
   → h_d exceeds what the loom funnel can track within p_2_inf
   → funnel barrier S = h_e/p → ±1,  ζ = log((1+S)/(1−S)) → ∞
   → |σ| > E  → κ-ODE leaves equilibrium and runs away (κ ≫ κ_0)
   → a_u = κ · G · sat(σ/E) blows up (seen: 33 589 m/s²)
   → cone/CBF clamp saturates OR the drone hard-impacts / flies away
```

> **θ_norm is *contained downstream*, not *eliminated* (corrected 2026-06-10).** θ_norm = ‖Θ‖_F drives `dκ/dt`; it is ~99% `cross(dw, s)` at a spike. It still **spikes** — ~480 at the baked config, up to 1226 when E_z is narrowed — from two sources: (1) *sustained* off-screen-KLT drift (removed by the `img_data` KLT-bounds guard), and (2) the **dominant residual: a frame-jump `dw` artifact** (`dw` = finite-diff of frame-held `w_i` ÷ control-dt → 252 rad/s², unphysical). What stops the *runaway* is **downstream**: the spikes are brief (they barely move the integrated κ), `P_z=5` keeps `κ_eq∝1/P` low, and `KAPPA_MAX_Z=3.0` is a backstop that **didn't even bind at the baked config** (κ_z≤1.6). **Do NOT source-fix θ_norm by rewriting `dw`** — an image-rate `dw` was tried 2026-06-10 (cut θ_norm offline 480→53) but `dw` feeds the load-bearing `c` feedforward, so closed-loop it caused *more* κ-runaways (3/5 vs 1/5). Containment is correct.

Bounding rules that fell out of this (each verified in R3):
- **`P` bounds κ cleanly** — `κ_eq ∝ 1/P`. `P=5/5/5` dropped a_u from 33 589 → 440. The clean κ knob.
- **`E` also bounds κ but *softens tracking*** — wide `E` keeps `|σ|<E` so κ≈κ_0, but the same wideness detunes the lateral hold (→ drift) and descent (→ hover). **One knob, one job:** use `P` for κ, `E` for stiffness, per-axis.
- **`N`, `Γ` cannot fix it** — `N` cancels in `κ_eq`; `Γ` only changes the *rate* of runaway. Mathematically excluded.
- **cbf2 *masks* it** — the visibility CBF clamps the blown-up `a_xy`, so if it fires in normal ops the control law is failing, not the CBF. Keep it relaxed (`THETA_FLOOR=60`) and bound κ at the control level.
- **Funnel width = barrier gain** (`G⁻¹ ≈ p/2`). Never widen a funnel component to "make room" for a transient — it raises that axis's gain proportionally (learned twice: Ξ₂ and p_2inf_z).

> **BUT the lateral κ-runaway at touchdown is a FUNNEL BREACH that NO gain bounds (2026-06-10).** "P bounds κ cleanly" holds for *moderate* growth — it **fails at the barrier singularity**. Decomposed (P_z=8 rep3, κ_xy=7.26): at alt<0.5 m the 1/Z geometry breaches the *lateral* funnel (`|h_e/p_2|→0.99`) → ζ→5.3 → σ→3.6, G→3.1 → growth `θ·N·G·|σ|`=16.1 overwhelms leakage `N·P·κ`=0.10 by **160×** (to balance it you'd need `P_xy≈800`). So **P can't bound a breach, θ-freeze can't** (θ moderate ~37–72), **and the Singhal freeze misses it** (fires at `|h_e/p|≥1.0`; growth is at 0.9–0.99). Worse, **κ_xy is UNCAPPED** (`KAPPA_MAX=[1e6,1e6,3.0]` — only z capped, which is why κ_x hit 7.26 vs κ_z's 3.0). The fix is **convergence-ordering** — gate the descent on `|s_e_n|` so lateral centers *before* the 1/Z zone — + a κ_xy-cap backstop, NOT a gain. **REFINED 2026-06-10 (GT-verified):** the breach is a **WRONG `h_d` from the off-screen VIRTUAL centroid** — `cross(w_i,s)` where `_getVirtualPts`'s unguarded z_v→0 divide reprojects an *in-FoV* feature off-screen under the touchdown tilt (NOT a flow spike: `ds_d`≈0, measured `h` is physical/matches GT v/Z). The controller uses the VIRTUAL centroid; the cbf2 uses the ACTUAL — so the CBF (guarding the in-FoV actual centroid) can't see it. **TRIGGERED by ArUco decode-loss** (loss precedes runaway): 9/12 TLs had the marker fully in-FoV (decode-fail, not geometric loss). Fix: clamp/guard the virtual `s` + KLT corner-track + use-genuine-data on marker-LOST. See `feedback_lateral_kappa_runaway`, `feedback_marker_detection_stale`.

---

## 3. Parameter-by-parameter (current **direct per-axis** knobs)

All `*_SCALE` factors were removed 2026-06-03 — knobs are now direct values `PLASMC_<PARAM>_{X,Y,Z}`. Defaults below are the **live `controller.py` baked values as of 2026-06-10**.

### 3.1 Outer loop — image error → desired loom `h_d`

| Param | Default | Knob | Empirical (R3) |
|---|---|---|---|
| `K_rp` (P) | `9, 9` | `PLASMC_KP_{X,Y}` | **12→9 (baked).** KP=12 drives `h_d`→14 rad/s at `s_e_n≈0.5` → infeasible demand → funnel fills → κ explosion; also exceeds LK dynamic range at t=0. KP=9 = 52% better (3.06 vs 6.41 m). **KP≥13 dead-end.** KP×E coupled: KP=12 only safe with E_XY≤1.5 (+SEN_FUNNEL); KP=9 with E_XY≥2.5. |
| `K_ri` (I) | `1, 1` | `PLASMC_KI_{X,Y}` | **Load-bearing for FoV retention.** 10× MATLAB's 0.1. `KI=0.35` (MATLAB parity) → 4/5 TL (less correction authority → marker drifts out). `KI≥2` → integral windup → κ-runaway returns. **Both directions dead-end; keep 1.0.** |
| `K_rd` (D) | **`0, 0` (baked 2026-06-10)** | `PLASMC_KD_{X,Y}` | **D-term removed.** A close-range ArUco corner jump makes `dζ_s` spike (Δr≈0.57 in one 42 Hz frame → 54 rad/s²); the D-term is **99.7%** of the resulting `ds_d` spike and does **not** self-limit (P-term does). `K_rd=0` *alone* is a dead-end (11.96 m — D was load-bearing for drift arrest); the fix is `K_rd=0` **paired with** `gamma_s=1.0`. |
| `gamma_s` (outer funnel rate) | **`1.0, 1.0` (baked)** | `PLASMC_XIS_{X,Y}` | **The 2026-06-10 winner.** Outer PPC funnel on `s_e_n` (`SEN_FUNNEL=1`, default ON). gamma_s sweep 0.1→1.0: 0.1=11.8 m (funnel too slow, drift not arrested before 1/alt amplifies it), 0.2=2.76, 0.8=2.21, **1.0=1.32 m** (the "1/5 SP at 0.03 m" here is **UNVERIFIED** — see §1/§4). Fast funnel pressure replaces the D-term's damping role without its noise. `p_s_0=1.2`, `p_s_inf=0.1`. **gamma_s >1.0 was NEVER swept** (correction 2026-06-10) — higher = faster lateral convergence; sweeping 1.2/1.4/1.6/2.0 now (NC56+, IC1). Ceiling: if it outruns the LK-limited error reduction, the *outer* funnel breaches too. |
| `tau_ds` (LPF on `ds_d`) | `0.05 s` | `PLASMC_TAU_DS` | **Dead-end found (flag to revert to 0).** 50 ms LPF on the outer-PID output; n=5 gave 6.41 m WORSE than 0. It adds outer-PID lag that can't help once `s_e_n` is already large. |

### 3.2 Middle loop — optic-flow SMC (funnel + adaptive gain)

| Param | Default | Knob | Empirical (R3) |
|---|---|---|---|
| `Ξ₂` (`gamma`) | `0.2, 0.2, 0.2` | `PLASMC_XI2_{X,Y,Z}` | Sliding-surface gain. Not a mover; left at MATLAB. |
| `p_2_0` | `25, 25, 4` | `PLASMC_P20_{X,Y,Z}` | Initial funnel half-width. Narrowing `p_2_0_xy` 25→8 *did* engage the dormant off-center κ but traded into high vel + blow-up. |
| `p_2_inf` | `2.5, 2.5, 1.5` | `PLASMC_P2INF_{X,Y,Z}` | **Load-bearing.** Terminal funnel = gain. `P2INF_Z` sweep [1.5,1.0,0.5]: 1.5 optimal; 1.0→31 m, 0.5→10.6 m. Never widen to absorb the touchdown transient. |
| `Ω` (`Omega`) | `0.05, 0.05, 0.025` | `PLASMC_OMEGA_{X,Y,Z}` | **Load-bearing** κ-leakage. Left at MATLAB. |
| `Γ` (`Gamma`) | `0.4375, **1.0**, 0.75` | `PLASMC_GAMMA_{X,Y,Z}` | `GAMMA_Y` 0.5→1.0 baked (lateral braking). `GAMMA_Z` sweep [0.75,1.5,2.5]: **0.75 optimal** (1.5→51 m: faster κ_z growth → runaway). |
| `E` (boundary layer) | **`1.5, 1.5, 1.0` (baked)** | `PLASMC_E_{X,Y,Z}` | Stiffness, **not** the κ knob (use `P`). `E=2.5` all-axes bounds κ + holds lateral 0.04 m but is too soft → **hover** (no descent). The winner stack uses `E=[1.5,1.5,1.0]`. `E_Z≥1.5` dead-end (catastrophic); `E_Z=0.5` marginal. Tune per-axis: E_XY (lateral stiffness) vs E_Z (descent) conflict. |
| `N` | `0.02, 0.02, 0.02` | `PLASMC_N_{X,Y,Z}` | Adaptive growth. `N_XY=0.05` dead-end (5.87 vs 3.80 — faster κ without σ signal = noisy). `N_Z=0.05` alone dead-end. |
| `P` (κ leakage) | **`5, 5, 5` (baked)** | `PLASMC_P_{X,Y,Z}` | **The clean κ-bounding knob** (`κ_eq ∝ 1/P`). `P_Z`: 2.5→5 tamed κ_z 39→6.5; `P_XY`: 1.5→5 dropped a_u 33 589→440. `P_XY=3` dead-end (9.75 m). |
| `kappa_0` | `0.156, 0.156, **1.0**` | `PLASMC_KAPPA0_{X,Y,Z}` | **`KAPPA0_Z` 0.3125→1.0 baked (descent bootstrap).** With gamma_s=1.0's fast centering, the old κ_z bootstrapped too slowly → 6 m **hover**; KAPPA0_Z=1.0 starts descent in 0.4–0.9 s. (Smaller, e.g. 0.5, is *worse* for the κ_z runaway it exposed.) |
| `kappa_max` | `1e6, 1e6, **3.0**` | `PLASMC_KAPPA_MAX_{X,Y,Z}` | **`KAPPA_MAX_Z=3.0` baked** — caps κ_z so a high-flow `θ_norm` spike can't run it away. Paired with the Singhal κ-freeze on funnel-breach (`\|h_e/p\|≥1`) axes. **NOTE: κ_xy is UNCAPPED (1e6)** — *that's why* the lateral touchdown breach runs κ_x to 7.26 while κ_z stops at 3.0 (§2). A symmetric **κ_xy cap=3.0 is the backstop (B1)** for the lateral breach. |

### 3.3 Yaw SMC

`Ω_a=0.5, Γ_a=0.5, n_a=1, p_a=2, κ_a0=2, E_a=3` (`PLASMC_YAW_*`); `KR_YAW=2.0`; `YAW_PSID_RATE=1.0`; `TAU_UA=0.1`. All **confirmed optimal** in R3; every off-default is a dead-end (`YAW_OMEGA=1.0` adds oscillation; `YAW_GAMMA=1.0` worsens — it's a *large-initial-yaw* problem not a gain one; `KR_YAW≠2` looser/worse; `TAU_UA=0.3` more lag).

**The real yaw failure is not a gain.** `_ie_a_clamp` was replaced by **conditional integration** (freeze `ie_a` while heading-rate saturated; halved overshoot). The IC2-5 "yaw runaway" is **compass drift at landing start**: EKF yaw drifts ~77° during takeoff/IC so the drone *begins* the descent yawed → `psi_d`→180°. `alpha` is correct (tracks GT r=1.00). **Fix is the test rig** (servo true yaw), not the controller — three alpha redesigns all failed because the cause is the bad start. Yaw is image-`alpha` end-to-end (`BODY_YAW_SOURCE=alpha`, compass-free); compass enters only the rotation matrix.

### 3.4 Visibility barrier — cbf2 (replaces the cone clamp)

`FUNNEL_MODE=cbf2` (default), `THETA_FLOOR_DEG=60` (= θ_cap → the old d_min collapse is OFF). cbf2 is a camera-plane tilt-QP (`docs/FUNNEL_CBF_DESIGN.md`): theta_cap post-QP, two-phase δ. **It is a safety net, not a controller** — if it bites in normal ops, bound κ at the control level instead (it was *masking* the κ-runaway). `RHOFOV0=[290,210]`, `RHOFOVINF=[80,80]`, `LFOV=0` (rho_fov held constant). The old analysis's "RHOFOVINF is the strongest lever" is **false** — a cal artifact of the mapped precision-softness frontier (memory `feedback_precision_softness_frontier`).

### 3.5 Inner loop (SO(3)) & misc

`K_R=2,2,2` (`PLASMC_KR_*`; higher → LK break), `W_U_MAX=1.0` (`>1.7` → LK break → TL), `DH_D_MAX=50` (physics-guard on h_d derivative; **load-bearing** — `DH_D_MAX=5` prevents 10× first-step command amplification, but kept at 50 to *expose* real failures during tuning), `tau_ia=0.08` (not env; 0.04/0.20 both worse).

### 3.6 Image pipeline (`img_data.py`)

`IMG_FEATURE_FILTER=kf` (centroid KF default; savgol(13) added ~110 ms lag + 2× noise). `MARKER_KLT_MAX_STEPS=20` bridges ArUco outages by LK-tracking last good corners — **now stops when any corner exits the image** (removes the *sustained* off-screen-KLT θ_norm source: `s[0]=3.15` + `w_z=4.54` → `cross(dw,s)` — but does **not** eliminate θ_norm, which is contained downstream; see §2). ArUco interventions (`ARUCO_*`) baked. `BODY_YAW_SOURCE=alpha`, `CTRL_ZERO_WXY=1` (wx/wy observable but uncalibrated → zero for the stationary target).

### 3.7 Landing-test (`landing_test.py`)

`REF_RAD_OPT_FLOW=-0.42` (MATLAB default; `-0.70` wins IC1 but regresses IC2-5 — **use the default, ask before overriding**). IC tolerances drive σ0/vh0; tightening just moves variance, doesn't add SP.

---

## 4. Cross-rep — what actually separates good from bad reps (R3)

> **⚠️ SP-flag integrity (2026-06-10 audit).** The SoftPrecise flag can fire on a **degenerate GT**: if the logged `UAV Pose` freezes at the IC pose then resets to the origin, `xy_err`→~0 trips "precise" with no real landing. Exactly one such **false SP** exists in the saved R3 data (`Landing_Test/Wed Jun 10 01-22-38` = NC48d, marked FALSE in its `summary.tsv` + `FALSE_SP.md`). It was the **only** sub-10 cm rep across all 101 Jun-9/10 recordings → **there is no genuine SP in the saved honest-cal data**, and the NC47e "0.03 m SP" headline is unverified. Always sanity-check an SP against its trajectory (did the drone descend? is `xy_err` a physical number, not ~1e-21?).

**It is not a single tunable.** The old doc's "lateral velocity at touchdown is the single 40× factor" was a cal artifact. Under R3:

- **Stochastic perception is the binding split.** A rep with a perfect IC (pos 0.044 m, vel 0.009 m/s) can still give the *worst* result (10.8 m TL) because LK/ArUco collapse is independent of the gains. LK dynamic-range ceiling ≈ **2 m/s** — `KP=12` + 4% centroid offset alone demands `v_req=2.45 m/s` at 5 m, exceeding it → flow collapses in ~3 frames.
- **The explosion chain (§2)** explains the catastrophic tail (hard impact / fly-away) when perception *does* hold but a gain link is unbounded.
- **Hover-at-center** is a distinct, *non-SP* failure (xy=0/vel=0/flight>50 s) — boundary-layer too wide (`σ<E` throughout) so the controller never descends. Classify FAIL.

`analyze_explosion_chain.py` (first-exploding state) and `analyze_saturation_audit.py` (limit duty cycle) are the per-rep tools; `diagnose_failure_cause.py <rep>` splits perception-vs-control at onset.

---

## 5. Known dead-ends (do NOT retry without new evidence — all R3)

`KI≥2` or `KI=0.35` · `KP≥13` · `KP=12 with E_XY=2.5` · `W_U_MAX>1.7` · `E_Z≥1.5` · `N_XY=0.05` · `N_Z=0.05` alone · `tau_ua=0.3` · `P_XY=3` · `TAU_DS=0.05` · `K_rd=0` alone (needs gamma_s=1.0) · `gamma_s=0.1` · `KR_YAW≠2.0` · `YAW_OMEGA=1.0` · `YAW_GAMMA=1.0` · RHOFOVINF/THETACAP terminal sweeps (frontier is mapped) · `MC_*RATE_P>1.0` via MAVSDK (preflight fail) · enlarging the marker (FoV-match constraint) · sensor-cal refresh via `aggregate_calibration.py` (7–10× wrong) · **image-rate-`dw` HOLD rewrite** (poisons the κ-integrator → 3/5 runaways; the *no-hold* cleaner-dw is BAKED, commit 85e1011) · **sustained-θ κ-freeze** (mis-targeted — κ ratchets at moderate θ, not the spikes) · **`E_z=0.5 + P_z=8`** (NC55: the touchdown runaway is LATERAL, P_z is z-only — §2).

---

## 6. Current best config & open problems

**Baked R3 defaults:** `K_rp=9, K_ri=1, K_rd=0, gamma_s=1.0, P=5/5/5, E=1.5/1.5/1.0, Γ=0.4375/1.0/0.75, Ω=0.05/0.05/0.025, N=0.02, KAPPA0=0.156/0.156/1.0, KAPPA_MAX=·/·/3.0, FLOOR=60, cbf2, SEN_FUNNEL=1, W_U_MAX=1.0, BODY_YAW_SOURCE=alpha, KR_YAW=2, YAW_PSID_RATE=1.0`.

**Gain-side levers MOSTLY exhausted; `gamma_s>1.0` + `KP=12` now SWEPT (NC56-60, IC1 n=5):** gamma_s=1.2 → 0 TL (vs baseline's 2) but 1/5 hover (over-centers → weak descent); gamma_s≥1.4 degrades (descent-weakening, not a demand-breach). **`KP=12+E=1.5` → tightest landings of any cell (0.34 m, no t=0 LK collapse) but 1/5 the touchdown lateral breach fires harder (κ_xy=0.85) → next experiment = `KP=12 + κ_xy cap`.** Neither cleanly beats baseline at n=5. The binding limit underneath is still the LK dynamic range. Open problems, in priority:
1. **Lateral convergence-ordering** — gate the descent on `\|s_e_n\|` (center before the 1/Z zone) + a κ_xy cap, to bound the **touchdown funnel breach (§2)** that no gain fixes. This is the principled fix for the lateral κ-runaway.
2. **Code-level perception robustness** — pyramidal LK levels 2→3 in `getLKFlowAngVel` to lift the ~2 m/s dynamic-range ceiling (the binding stochastic-TL failure + the close-range touchdown marker-loss = 4/6 of TLs). The ultimate enabler.
3. **IC-rig compass-drift fix** — servo *true* (Gazebo) yaw to null EKF drift so IC2-5 descents start aligned (the yaw-runaway cause; controller untouched).
4. **Status:** NC49 KLT-bounds IC1 n=5 **DONE** (2 TL + 1 hover, not clean); descent-hover thread **closed** (cleaner-dw baked; E_z=0.5 / P_z=8 / θ-freeze / dw-rewrite all dead-ends). **IC2-5 gate only once IC1 is clean** (user directive). Revert `TAU_DS`→0 (worse than baked 0.05).

---

## 7. Data sources

- `test_data/Landing_Test/parameter_record.ods` — `PX4_NewCal_Record` (NC1–55, R3) is canonical; each row now carries its `Bundle`/`Timestamp` provenance. `PX4_Gain_Record` (G1–G60) is the R1/R2 history.
- `test_data/<bundle>/` — named R3 bundles (incl. the Jun 9–10 `GammaS_sweep_n25`, `BootstrapFix_n21`, etc.); raw per-landing recordings under `test_data/Landing_Test/`.
- `src/controller.py`, `src/img_data.py`, `apps/landing_test.py` — code source of truth.
- Memory: `reference-tuning-trajectory` (the connected arc) + the `feedback_*`/`project_*` topic files it links.
- The `tune-plasmc` skill — the tuning playbook (methodology, dead-ends, parameter inventory).
