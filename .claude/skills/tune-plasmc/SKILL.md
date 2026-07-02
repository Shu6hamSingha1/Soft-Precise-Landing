---
name: tune-plasmc
description: Systematic parameter tuning + failure diagnosis for the PLASMC vision-based landing controller on PX4/Gazebo SITL. Use when the user asks to tune the controller, diagnose why a landing failed, sweep a gain, or understand parameter sensitivity. Encodes the complete parameter inventory, the n≥5 sweep methodology, known-bad and known-good configs, and the diagnostic chain that traces a failed rep back to root cause.
---

# Tune PLASMC

A systematic procedure for tuning the PLASMC controller in `PX4_Gazebo/` and for diagnosing why a given landing failed. Read this end-to-end before any tuning sweep. The memory under `~/.claude/projects/-home-shubham-Soft-Precise-Landing/memory/` has the historical findings — this skill is the **playbook** that ties them together.

> **🟢🟢🟢 2026-07-03 (LATEST) — CYCLE MECHANISM CORRECTED (deep-dive): the −120° actuation phase and the "locked 1.7 rad/s + harmonics" were METHOD ARTIFACTS. Real fuel = anti-position command + ANY lag pumps a rotating error; amplitude is CONE-CLAMP-set; damping quadrature is destroyed inside the barrier chain. PLASMC_AU_LEAD approved 07-03 + under test. k_r RESOLVED same thread (HD_KR=0 = wrong reference, rejected; DHD_SRC falsified c3-noise; defaults stand). Memory [[project_rover_turning_open]] (whole causal chain).**
> - **Corrected mechanism (9-rep deep-dive; tools `Rover_AB_harness/cycle_{tone_fit_v2,stage_phase,branch_audit}.py`; ⚠ fit complex tones in ENU — NED conjugates the rotation):** ONE rotating phasor, **W* = 1.3–1.7 rad/s REP-SCATTERED** (the "1.7 fundamental + 3.4/5.1 harmonics" were the FFT bins of the 3.7 s tracking window), A 0.4–0.6 m, orbit-sense rotation, 80–93% of error variance. **A·W\*² ≈ 1 m/s² CONSTANT → amplitude = clamp-limited authority / W\*²** (cone duty 26–38%; tone gain through LPF+cone 0.43). Stage budget at W\*: tau_ia −9°, PX4 attitude −8°, geometry/drag −15° → **actuation total −25…−55°, NOT −120°**; validated by the kinematic closure arg(a_del/e)=±180° holding 9/9 (the check the old cross-spectral method lacked).
> - **Energy ledger (the fuel):** an anti-position command delivered through ANY lag lands ahead of the target direction on a rotating error → pumps (P_cyc>0 in 7/9, ≈0 in 2); damping needs command quadrature χ > Wτ ≈ 25–40°, and the χ-vs-Wτ margin predicts the P_cyc sign 9/9. **The one Circular landing ever (hdkr_0 21-57-07, 0.200 m) is exactly the χ≈Wτ≈97° neutral rep** — the coin-flip landings are that margin fluctuating.
> - **Why gains can't damp it (exact branch audit, recon resid 0%):** command-tone share/χ — switch θ·sat·κ **0.47/+7°**; drift χ_r·ζ̇_r/G **0.25/−1.5°** (the DESIGNED damping branch arrives with zero quadrature — killed by scale-free normalization ė/Z+|h_rd|·e/Z, funnel-rate mixing, and barrier-slope AM by the radial error component); c3 0.24/−1.4°; loom 0.08/**+85°** (the only correctly-phased branch). ~96% of the tone is centripetal → **gain knobs re-balance χ≈0 branches and cannot move the total phase** (why P/E/P2INF/HD_KR/DHD_SRC all "re-balanced"); K_R=2.5 was a genuine Wτ cut. Exit sized: **+25–40° of quadrature at 1.3–1.7 rad/s** — `PLASMC_AU_LEAD` (first-order lead on the shared lateral world command; HF gain lands on the cone) or restoring drift-branch quadrature at the source.
> - **Lever sweep (heading-hold Circular n=3):** `HD_KR 0.5→0` looked best (1/3 ON, 0.200, first-ever Circular landing; e-rot 1.11→0.85) — **RESOLVED late-session (user-led): REJECTED, wrong REFERENCE.** The back-map prescribes `ζ̇_r,d=−k_r·ζ_r`; k_r=0 ⇒ ζ_r=const (freeze-the-offset) ⇒ ζ_h damps the closure χ_r·ζ_r commands (channels contradict off-center). Stationary cost confirmed: IC1 n=5 = 1/5 SP vs 4/5 baseline. Its Circular win = **demand-level detuning**, PROVEN by the complete-job A/B: new `PLASMC_DHD_SRC` knob (full|nokr|nos, default full; nokr keeps k_r=0.5 in h_d, drops only its barrier-inflated branch from dh_d) → **c3 band-rms drops to hdkr_0's level but the cycle is UNCHANGED (e_rot ~1.02, 0/3)** — the DF re-balances (single-source model confirmed surgically); the 06-29 "full-h_d differentiation regression" framing is CORRECTED (honest ḣ_d = load-bearing-neutral). `nos` = 2/3 NEAR 0.34 + 1 WANDER 78 m (stiffness-loss tail). **HD_KR=0.5 + DHD_SRC=full stand.** `P_xy=5` FALSIFIED (adaptive-leakage: κ drains → σ grows → ratchet 3.04, worse). `E_xy=1.5` + `P2INF 2.0/3.0` FLAT/weak (DF re-balances; P2INF also adds a wander outlier).
> - **FRONTIER (updated 07-03):** gain levers move along the bias↔cycle frontier because they cannot add command quadrature. Exits: `PLASMC_AU_LEAD` (**approved 07-03, implemented for test** — needs only +25–40° at 1.3–1.7 rad/s, not +120), drift-branch quadrature restoration (control-law redesign), or platform size. **⛔ K_R=2.5 spent (sharp optimum, real Wτ cut); ⛔ uXRCE-DDS RULED OUT by user — never re-propose.**
> - **⛔ Oracle a_t-FF + lead pursuit REVERTED** (manuscript Problem Statement forbids target-pose derivatives; keep as the ORACLE-BOUND ablation: curve steady lag 0.9–1.6 → 0.31–0.51 m = the value of target-motion knowledge; deployable only under a declared cooperative-platform extension). **✅ COMPASS_FREE_VALIDATE BAKED default-ON** (every descent cuts EKF2 mag at engage, readback-verified; =0 legacy). Data: test_data/Rover_Turning/{yawhold_arm_n3,p2inf_sweep,cycle_gain_sweep}/, Rover_VelFF/.

> **🟢🟢🟢 2026-07-02 morning — SP ACHIEVED (GT-FB 19/25): the terminal "wall" was substantially the Z_REG HARNESS ARTIFACT; stationary chapter closed → MOVING-TARGET (rover) PHASE, first moving landings 3/3. Memory [[project_why_sp_achieved]] [[feedback_zreg_gear_floor_artifact]] [[project_moving_rover_landing_works]].**
> - **Z_REG=0.2 (gear height) BAKED** (`src/gt_feedback.py`): Z_REG=0.01 let the GT-FB computed z fall below the ~0.2 m gear floor → fake unbounded 1/z→100 → κ_eq 107 / σ 3.66 blow-up → the whole "0-SP terminal wall" era. With it (+ W_U_MAX=2.0 + VDS_KF_Q=1 + breach-leak fix + XIR=0.10): **19/25 SP, rel_med 0.023**. Z_REG is a HARNESS fix (no perception-ON analog) — transfers as understanding: when a GT-FB terminal blow-up looks "fundamental," FIRST verify the harness feeds a physically-bounded 1/Z.
> - **⭐ METRIC RULE (supersedes SP-count A/Bs at n≤5):** GT-FB IC1-5 n=5 SP-count noise floor is **±5–7** (same config/binary: 19/25 vs 12/24; the terminal 1/Z amplifies mid-descent SITL noise into entry velocity, weakly coupled to ICs/config). **Judge by breach% + `s_dot_entry`** (clean SP/non-SP separator ~0.02–0.04 vs ≥0.1, no overlap); breach% separates configs more than SP-count at equal n.
> - **XIR=0.10 HIGH-N CONFIRMED** (pooled 56/98=57% vs XIR=0.15 33/75=44%, breach 44% vs 65%, advantage in off-center IC2/IC3) — the f068774 revert stands. **⛔ NEW DEAD-END: `PLASMC_SOFT_BREACH` at every frac** (see Known dead-ends). `W_U_MAX=2.0` now runs 0% duty (inert as a clamp); `P2INF_z` reduction dead (1.5=19 > 0.5=17 > 1.0=12).
> - **ROVER:** infra + root causes closed 07-01/02 — `POSE_IDX_UAV/TARGET` pose landmine (bridge FULL `pose/info`, never `dynamic_pose`); rover fly-away root = NO landing platform (marker +0.5 m visual-only, no collision) → platform fix CONFIRMED → stationary-rover **5/5 clean (0.02–0.05 m)** at ceiling → **first MOVING landings 3/3 ON the platform** (Linear @0.47 m/s, velocity-matched). Yaw "cal" RESOLVED (`cal_s[3]=1.0` correct; the only TURNING gap is the alpha-rate cap → `PLASMC_YAW_ALPHA_FILT=0` under GT-FB or `MAX_RATE≈0.8`). Entry: `docs/MOVING_TARGET_PREP.md`; launchers `run_rover_landing[_retry].sh`; A/B data `test_data/Rover_AB_*/`. NEXT: rover speed sweep, Circular (turning) validation, perception-ON.

> **🟢🟢 2026-06-30 — RESIDUAL TERMINAL LIMIT CYCLE characterized; W_U_MAX=2.0 BAKED. Memory [[project_residual_cycle_wumax_bake]].**
> After Z_REG=0.2 ([[feedback_zreg_gear_floor_artifact]]) killed the unbounded-1/Z ARTIFACT, what remains is a small RESIDUAL ~1Hz lateral LIMIT CYCLE that ignites in the last <0.25 m (1/Z≈4–5 still enough loop gain); validated as a sustained oscillation in `s_e_n` (3–6 crossings) & `v_x`. Soft reps = same loop, damped to nothing.
> - **BAKED W_U_MAX 1.0→2.0** (controller.py:2188+1781). The 1.0 body-rate clamp's DISCONTINUITY *seeds* the cycle (clamp→bang-bang→bigger cmd→more clamp, biting ~24% terminal). At 2.0 the cmd stays below the cap → no discontinuity → cycle not seeded → SP. **The cmd is LOWER at 2.0, not higher** — REFUTES "throttled brake"; the clamp was a cycle DRIVER. → 3 SP incl 1st IC5 SP (IC5r2 rel 0.545→0.0075). Still bounds chatter (unlike the 1000-uncap). Partly stochastic (other driver = the drift).
> - **a_u oscillation DRIVER = the drift `chi_r·ζ̇_r/G`** (amp 25.7 ≫ c-term 11, switching 9.6), entirely via `dr_bar_e = s_dot_meas/p_10` (the measured bearing rate; `g_r`/`1/G` flat = `s_e_n` inside the funnel `S_r≈0.1`, **NOT a barrier blowup**). A velocity-MEASUREMENT feedback (`s=lat/Z`→1/Z²-amplified), on the SINGLE impact axis. Filters REPORT the cycle (s_dot_meas AND flow `h_x` both ring) — don't generate it → smoothing one channel (VDS_KF_Q) can't fix a multi-channel physical oscillation.
> - **P2INF_xy (flow funnel) = the fly-away fix** (env, NOT baked): widening un-saturates `ζ_h` → breaks `v_lat→h_e>p_h→ζ_h pins@3.66→σ out of layer→bang-bang→κ-runaway→FLY`. 2.0/3.0 = 3 SP/0 fly; **1.5 = tightest un-saturated → stronger ζ_h damping** (mean terminal v_lat 0.4→0.23) but a **soft↔precise** tradeoff (χ_r maxed at 1.5 by the 38 ms lag).
> - **IC5 fails** = LARGEST normalized error (`s_e_n0=0.93`, NOT the funnel ceiling — funnel starts `PR0=10`→`S_r≈0.09`; mean 1/Z is FLAT ~3 across ICs) + LEAST runway (3 m) → centers POSITION at high `v_lat` (2.32) → overshoots. Velocity arrest is lag-limited + DECOUPLED from position; descent paces on position.
> - **RULED OUT** (mis-reads corrected): `w×s`/yaw (1–2% of h_e; `h_d` uses `w_i` xy-zeroed; yaw spike is a +0.9–2.3 s POST-onset consequence); CBF cone clamp (real cut 4–15%; the "13–18%" audit metric was DEGENERATE); single-channel filter smoothing.
> - **Descent pacer `PLASMC_DESCENT_GATE` is OFF this whole session** (default 0, kept dormant — the descent ran at full `h_rd·Z`, un-paced). A velocity-aware gate extension was considered then DROPPED by the user.
> - **OPEN:** P2INF_xy value + gate (1.5 damping vs 2.0/3.0 no-fly); VDS_KF_Q sweep (lower-priority by the multi-channel logic); IC5 runway/pacing.

> **⭐⭐⭐ 2026-06-26 — VELOCITY-DAMPING LEVER CONFIRMED + EXTENDED; the residual is SOFT, not precision. Memory [[feedback_flow_funnel_zetah_works]].**
> The lateral fly-away/wall is SOLVED by tightening the lateral optic-flow funnel (engage the dormant velocity barrier `ζ_h` in `σ_xy=ζ_h+χ_r·ζ_r`). `XI2_xy` 0.2→0.5→**0.7** + `P20_xy`=15 → GT-FB IC1-5 n=2 = **10/10 bounded, ZERO fly-aways** (safe-floor drops ~1m→~0.08m). `XI2_xy=0.9`+`P20=10` fixes the high-start IC4 (xy 2.84→1.04). **This SUPERSEDES the 2026-06-24 "untried: fix terminal loom collapse" framing below** — the lever was the flow funnel (arrest `v_lat` early), and the terminal root was re-framed to lateral `ζ_r`/`s_e_n`, not loom collapse ([[feedback_terminal_root_lateral_zeta_r]]).
> - **The reported `xy_err` is NOT a precision failure.** The controller touches down DEAD-CENTERED (5/10 reps lat 0.005–0.09 m at min-Z, incl. all the "flys"); a terminal 1/Z kick then balloons it 1–4 m. Score min-Z, not the post-kick endpoint. The genuine residual = terminal VELOCITY (`vlat` 0.5–1.5, `vz` 0–1.3 m/s) ≈ the 38 ms lag (GT-FB ceiling xy≈0.2 m, rel_vel≈0.37 m/s).
> - **Which barriers to trust:** `s_e_n` (converges, min 0.014–0.05, 86–97% in `p_r`) and the Z-channel `h_e_z` are informative. **Lateral `h_e` is DEGENERATE** in the combined surface (`h_d`=measured ṡ ⇒ `h_e`≈0 by construction) — 100% "inside p_2" means nothing.
> - **Z funnel was silently loosened by the combined auto-align** (`P2INF_z` 0.5→1.5, `XI2_z` 0.6→0.2 vs the old soft-config) → `h_e_z` rides its funnel at a fixed ~0.6 ratio (never converges). Tightening (`XI2_z=0.6`, `P2INF_z=0.5`) scales `|h_e_z|` down 1.3→0.35 + adds mild bounded z-chatter; vz-softening verdict in progress.
> - **`N_xy=0.1` baked** (wakes `κ_xy`); safe ONLY with the velocity damping. **MATLAB gains don't port** (38 ms lag; `chi_r`=2.0 catastrophic, PX4 `chi_r`=0.5). **STALE — do NOT cite as binding:** "full soft-config-Z on combined = 5/5 fly catastrophic" (measured on the pre-velocity-damping, pre-`E_z=0.5` base). h_rd CONSTANT (−0.42) throughout.
> - **⚠ ENV TRAP (still live):** setting any single axis of a VDF-aligned param (`PLASMC_XI2_Z` etc.) bypasses the combined auto-align → X/Y revert to hot defaults; pin all three axes. Per-axis caps use the `PLASMC_` prefix (`PLASMC_KAPPA_MAX_Z`, not `KAPPA_MAX_Z`). `IC_LIST` env now subsets `run_ic_validation.sh` (e.g. `IC_LIST="IC4"`).

> **⭐ 2026-06-24 [SUPERSEDED by 2026-06-26 above] — TERMINAL DESCENT-CONTROL FAILURE is the binding IC1/IC4 issue; h_rd / funnel / κ_z-cap ALL RULED OUT. Memory [[project_descent_terminal_failure]]; records NC165-168.**
> After the cycle fixes below, IC1/IC4 still fail via a TERMINAL DESCENT failure (mechanizes the "terminal-1/Z spike below 0.4m"): the loom-proportional reference `vz=h_rd·Z` tapers vz→0 at the deck → measured loom h_z COLLAPSES toward 0 → loom error grows → **κ_z over-brakes** → vz flips + (BALLOON) → perturbs the (already-converged, lat→0.1 by Z=1) lateral → `s_e_n=lat/Z` 1/Z-amplifies → lateral RUNAWAY → off-target/fly-away. The terminal VERTICAL transient throws it, not lateral convergence.
> - **h_rd NOT the lever (DEAD-END: aggressive constant h_rd):** feasible -0.42 tracks the loom (h_e_z≈0.01 to Z=0.7); -1.0 is INFEASIBLE at altitude (vz=-1·Z=-6@6m → loom overshoots -2.0 → over-brake) → stochastic off-target 4-8m OR hover-deadlock, 0/5. Both extremes hit the same terminal trap.
> - **Funnel tightening = DEAD-END:** tight loom funnel (P2INF_z=0.5,XI2_z=0.8) feeds the Singhal containment GENUINE (non-glitch) loom errors → fabricates a "descending" loom → fools controller into hover → deadlock (hover h_e_z=|h_rd| > floor) or struggle/off (1/5). **Funnel floor must exceed |h_rd|.** Containment is HARMFUL in GT-FB (no glitches to reject). Keep VDF P2INF_z=1.5.
> - **κ_z cap NOT the lever:** capped@3.0 → over-brake→balloon; UNCAPPED → κ_z RUNS AWAY 88-287 on collapse-reps → struggle/off (recovered IC1 0.06 where the cap over-braked). Loom COLLAPSE is the root (κ misbehaves either way). Keep cap 3.0 (0.03 is GT-FB-only).
> - **REAL LEVER (untried): fix the terminal loom collapse directly** — reference governor that doesn't taper vz→0 near deck, OR terminal brake-authority handling. NOT h_rd/funnel/κ-cap.
> - **⚠ ENV TRAPS:** per-axis params use the PLASMC_ prefix via pa() — `PLASMC_KAPPA_MAX_Z` (NOT KAPPA_MAX_Z — ignored, ran 3.0 all session), `PLASMC_GAMMA_Z` etc. Setting `PLASMC_GAMMA_Z`/`XI2_Z` ALONE bypasses the VDF auto-align → X/Y revert to hot defaults (2.0); pin all three. Gate `run_ic_validation.sh` (IC1-5 now): ~50% SITL flake; setsid children survive harness kill → leftover SITL; use run_in_background not `&`. K_R rp=2.5 baked (uncommitted).

> **⭐ 2026-06-24 — GT-FEEDBACK CONTROL-ISOLATION CAMPAIGN (per-axis limit cycles; yaw→z→lateral→terminal-1/Z). Memory [[project_gt_feedback_control_tuning]]; records NC110-128.**
> Feed EXACT V-frame GT s/h (`src/gt_feedback.py`, env `PLASMC_GT_FEEDBACK=1`; control loop + landing_test `feature_fresh` gates made GT-aware so it runs to touchdown) to ISOLATE control from perception — any failure on exact features is a CONTROL-LAW failure, not perception.
> - **YAW cycle = double integrator → BAKED `PLASMC_YAW_OMEGA` 0.5→0.1** (controller.py:300). `u_a` is a yaw-RATE cmd (`psi_d=∫u_a`) so `Ω_a·ie_a` is a 2nd integrator with no phase margin vs PX4 inner lag → growing ±40° cycle that pumps lateral via image-frame coupling. 0.1 → single bounded overshoot (ncross 5→2), validated DES_ALPHA 0/10/30/45 at IC1. RULED OUT: `K_R_YAW`↑ (worse, not inner lag), `E_a`↓ (worse — more switching = more overshoot). GT alpha sign-validated vs perception moment-alpha (≤5° through descent; the ±200° corruption is the terminal marker-fill 180° flip, NOT cal).
> - **LATERAL mid-descent cycle → `K_R` roll/pitch 1.5→2.5** (env-tested, NOT baked; sharp optimum). Combined surface `σ=ζ_h+χ_r·ζ_r` has `ζ_h≈0` (h_d's measured-rate FF cancels ~56% of the flow, corr 0.88) → position-only/no velocity damping + inner lag (eR −22°). K_R=2.5 → eR ±5°, cycle GONE (s_e_n held 0.05-0.14), IC4 36m→0.56m. OPPOSITE of yaw (faster inner helps: lateral outer is under-damped-not-unstable).
> - **ALTITUDE healthy mid-descent** once the GT loom estimator was de-noised (`PLASMC_GT_FB_TAU=0.12` time-windowed LS slope; the z high-freq was estimator noise). Terminal spike below ~0.4m.
> - **TERMINAL 1/Z = the binding residual; PURE CONTROL (features exact in GT mode — NOT perception/observability).** a_u blow-up is the SWITCHING term `θ·sat·κ` (NOT the c-term); `θ=‖Theta‖_F` is a SHARED SCALAR so the lateral/z terminal barrier blow-up detonates EVERY axis. **SHRINKING the flow funnel SELF-DEFEATS**: smaller p_2 → r=h_e/p_2→1 → barrier ζ inflates → σ≫E → `sat` saturates ±1 → boundary layer DEFEATED → bang-bang `sign()` → CHATTER. (Chatter = discontinuous `sign(σ)` × loop lag; the discontinuity is intrinsic to 1st-order SMC — needed for finite-time robust reaching. σ inflated NOT because h_e grew (it shrank) but because the funnel outpaced the control's ability to shrink h_e = the terminal authority limit re-expressed.)
> - **LYAPUNOV PROOF (control_formulation.tex Thm 1) — terminal is unfixable by κ/gain:** GUUB rests on **Assumption 1: β=1/Z BOUNDED (Z≥Z_min>0)**; Z→0 ⇒ β→∞ VIOLATES it, disturbance bound d̃∝1/Z→∞, proven UUB radius ∝d̃ blows up. **κ provably tracks only a BOUNDED d̃ (rate-limited N + capped κ_max) → CANNOT fight unbounded 1/Z.** Soft-touchdown claim (v_rel=h·Z→0) extrapolates the UUB past its own β-bounded assumption = the empirical fly-away. **Fix dictated: NOT more κ/gain, NOT tighter funnel — RESTORE Assumption 1 (bound d̃) via the `h_rd` REFERENCE GOVERNOR** (ease h_rd→0 near deck on scale-free proximity → drone actively settles, h_e bounded, back inside the proof's domain). Loom-clamp worked (IC2 soft 0.25/0.52) but rejected as band-aid (reverted). h_rd governor = UNTESTED next step; super-twisting separately kills chatter but doesn't bound d̃.

> **⭐ 2026-06-22 — VERTICAL CHANNEL: the size-norm corner MOMENT loom is the fix; SINGLE-MARKER is rank-deficiency-bound ([[feedback_single_marker_rank_deficiency]], [[feedback_virtual_plane_loom_ring]]).** The terminal VERTICAL LAUNCH is the nested-ArUco primary-marker SWITCHING → corner-spread jump → pinv loom spike (|Δh_z| up to 4.4/frame). **FIX = `FLOW_LOOM_DECOUPLE=1` (size-normalized corner MOMENT loom `−½·d(lnM)/dt`, `M/sz²`):** a DIRECT area-rate scalar (NO `pinv(L_s)` → no rank-deficiency) that stays CONTINUOUS across switches (|Δh_z| 4.4→0.4) → **vert 5/15→1/15 (n=15), 0/5 (n=5)**. Bakeable; pending the IC2-5 gate. **SINGLE-MARKER (`PLASMC_SINGLE_MARKER`, default-off) is a DEAD-END on the nested board:** lock+GFT-dense+visibility-margin ring-switch all correctly wired, but a single near-axial marker is RANK-DEFICIENT (`cond 15-60`, p90 270-471, vs multi-marker ~11; spread-set `≈s_phys/Z`, NOT point-count). The moment loom fixes only `h_z`; the LATERAL `h_x,h_y` stays pinv-rank-deficient → SCALE BIAS (not jitter — single lat_noise 0.126 < momentsz 0.212) → drift → 1/Z → vz launch. n=5: single 1/5 vert 0/5 sub vs momentsz **0/5 vert 2/5 sub**. The board is nested because **no single marker fills the FoV across 10m→0.4m (~4.5 octaves)**. Noise levers (`loom_noise=input_noise/σ_min²`): GFT (cuts input noise, clean), `FLOW_LSTSQ_RCOND`↑ (caps `1/σ_min²` but BIASES magnitude 0.90→0.66 — band-aid), moment loom (avoids lstsq entirely — the clean win). OPEN: single-LARGE-marker WORLD (Gazebo SDF change) untested.

> **⭐ 2026-06-18 — STRUCTURAL RESOLUTION (combined/blended sliding surface).** The lateral-authority
> wall this skill spends most of its levers chasing (s_e_n won't converge / `G_s⁻¹→0` SEN-funnel demand-
> starvation / a_u outward) is **structurally fixed** by the manuscript's combined surface, now VALIDATED
> in the MATLAB single-run (gated `COMBINED_BARRIER`): the position barrier `ζ_r` enters the sliding
> surface DIRECTLY (`σ_xy = ζ_h + χ_r·ζ_r`) instead of back-mapping to a desired feature rate — so there
> is **no `G_s⁻¹` starvation**. Winning config = corrected `c̃_h` + **`s̈`-drop** + `p_2inf_xy=0.5` +
> `χ_r=0.85` + `p_r_inf=1.0` (proof Standing-Condition-1) → **25/25 SP + 75/75 noisy, no breach**, beats
> the back-mapped form (24/25) and is comparably-to-MORE noise-robust (16/24 vs 13/24 on the hard cells).
> Two design facts that matter for the PX4 port: (1) `h_d = measured-finite-diff ṡ + transport + h_rd·s`
> (the MEASURED centroid rate, NOT a PID `ṡ_d`); (2) the `s̈` term in `c_h=c̃_h−ḣ_d` is `1/z`-inflated and
> over-aggressive at terminal — **DROP it** (κ absorbs it as `d_h`); keeping it (cap / tau-LPF / SG-clean /
> per-axis-tuned) was exhaustively shown irreconcilable (Circ-IC4 needs `χ_r_x≥1.1` for precision but that
> destabilizes other cells). PX4 port = gated `PLASMC_COMBINED_SURFACE` (purely-lateral: `ζ_r=_zeta_s[-1]`,
> `ζ̇_r=smooth4(_dzeta_s_deque)`, drop `ds_d` from `h_d`; descent PI unchanged) — **NOT yet SITL-validated**.
> See [[project_combined_barrier_matlab]], [[feedback_combined_surface_divergence]], [[feedback_hd_uses_measured_sdot]].
>
> **⚠ 2026-06-19 — the lateral terminal velocity is a FORCED LIMIT CYCLE, not a tunable offset.** It's an
> under-damped lateral loop pumped by the `s̈` feedforward (positive feedback through the optical-flow
> measurement). The velocity amplitude is **forcing-set** — exhaustively, NO feedback or optical-flow
> parameter damps it (`χ_r` damps position-amplitude not velocity; `p_2,∞`/cap/`γ_2`/`E`/`N`/`κ_0`/`CB_SDOT_FILT`
> all flat, diverge, or lag-break the fast cell). Scale-free, only **dropping `s̈`** (most damped, the optimum)
> or heavy-`tau` LPF (damped cycle, less margin) reduces it; the no-lag z-taper fix needs altitude → invalid.
> Underlying-cycle root (present even in drop): `σ` rings inside the boundary layer (`|σ|≪E` → switching
> linearized away) AND `eR > commanded tilt` → **inner-loop attitude lag** (cascade-bandwidth mismatch);
> `kR`/`kΩ`/`E` test pending. Relevant for the PX4 port too — same `s̈`/inner-loop structure. See [[project_sddot_limit_cycle]].

> **⭐ 2026-06-20 — CYCLE-DAMPING CAMPAIGN + FLY-AWAY ROOT CAUSE (MATLAB clean blocks; [[project_cycle_damping_campaign]]).** Follow-up to the 06-19 limit-cycle block; the `kR`/`kΩ` test "pending" above is now DONE.
> - **The yaw limit cycle is the hidden MASTER driver.** Per-axis analysis: a YAW cycle (worst Circular-IC3=1.78 swing-pp, concentrated on the `[2,-2]` IC3 offset, rotating-target-worst) PUMPS the lateral cycles through yaw↔image-frame coupling. **`kΩ_z` (SO(3) yaw-rate gain) under-damped at 0.1 → root cause.** `kΩ_z=0.2` kills the yaw cycle −90% AND the lateral cycles (Y 0.98→0.11) at zero gate cost, MONOTONICALLY (robust, unlike a sharp `kR` optimum). **Check the yaw cycle FIRST on any lateral-cycle problem.** Baked: `chi_r=1.15` (full ±40%) + `kR-roll=2.5` (Y-attitude damping, but SHARP/fragile optimum) + `kΩ_z=0.2` (root).
> - **The terminal fly-away is a 1/z geometric POSITIVE-FEEDBACK instability, NOT a noise outlier.** Drone descends cleanly to ~0.33m then LAUNCHES upward (descVz=-22). The loom `V_h(3)=vz/z` has loop gain ∝1/z→∞ near touchdown; it self-excites (V_h(3)↑→thrust↑→ascent→corner-velocity dPdt↑ RAMPS gradually→V_h(3)↑). Noise only SEEDS it; geometry provides the gain. It's a race: descent-to-zf vs the loop igniting. **κ can't fix it** — the spike enters `u_eq`'s c-term `-h_z·h` (quadratic in flow), bypassing `u_sw`/κ; and κ assumes σ is correctly measured (this is a sensor-side amplification).
> - **`h_ez` does NOT converge to 0** (so the loom isn't held at `h_rd` → area rate not constant → dPdt ramps → enables the instability). Root = weak descent integral (`χ_z=0.025`+izeta clamp). Proper fix = strengthen descent integral so `h_ez→0`; band-aid = clamp `V_h(3)` before the SG filter (gated `LOOM_CLAMP`).
> - **`h_rd` paradox:** gentler `h_rd` SHRINKS the descent cycle but is LESS robust (more time in the low-alt 1/z danger zone → more fly-aways). `h_rd=-0.40` looked clean on seeds 1-3, flew away on seed 1002. **`h_rd=-0.42` stands.**
> - **METHODOLOGY (load-bearing): validate gains across a SEED ENSEMBLE, not seed 1.** The whole campaign optimized on seed1+noiseless and shipped a fragile `h_rd`. Even the original config is only 80% over the hard ensemble (IC2/3/5 × seeds 1000-1014). A sharp single-seed optimum is a robustness RED FLAG. Use `sweep_vdf` + a multi-seed fly-away/SP-rate metric.

> **⭐ 2026-06-21 — DESCENT/Z-CYCLE RESOLUTION + AGGRESSIVE BASELINES + THE TWO FUNDAMENTAL LIMITS ([[project_cycle_damping_campaign]], [[project_aggressive_baseline_campaign]]).**
> - **FINAL baked gains** (committed, bit-exact): `chi_r=1.15, kR=diag(2.5,1.5,0.5), kOmega=diag(.3,.3,.2), chi_z=0.1, E_z=diag(1,1,0.5), h_rd=-0.42`.
>   - **`chi_z=0.1`** (was 0.025): the proper fix for the 1/z fly-away the 06-20 block diagnosed — strengthened descent integral drives `h_ez→0` → fly-aways 35→3/300. `h_ez`-convergence and robustness DIVERGE past 0.1 (`chi_z=0.2+Gamma_z=1.0` gives better h_ez but fly 3→13: more loop gain over-drives the cascade lag). 0.1 = optimum. (Superseded the `LOOM_CLAMP` band-aid, reverted.)
>   - **`E_z=0.5`** (was 1.0): the Z limit cycle rang 100% INSIDE the boundary layer (`|σ_z|/E_z≤0.2` → sat linearized → **κ's switching damping locked out**; κ IS the damping). Tightening E_z engages it → Z cycle −8%. `E_z<0.5` hard-switch OVER-DRIVES the aggressive cells (NOT noise chatter — σ_z noise is only 0.05; it's sat-saturation on large-σ_z cells; `P_z`↑ softens the over-drive but trades damping). **X/Y cycles are ALSO inside the layer but tightening E_x/E_y makes them WORSE — they're NOISE-PUMPED (s̈ positive feedback), so κ switching FEEDS them.** Z=under-damped (κ helps), X/Y=forced (κ hurts; s̈-drop is the lever). **✅ RE-TEST DONE — CONFIRMED on the new base (2026-06-30):** I hypothesized that `VDS_KF_Q=1` (which removed the drift forcing) might let tightening E_xy help on the now-small residual. It does NOT. A/B on the q=1 + W_U_MAX=2.0 + Z_REG=0.2 base (IC2/4/5 n=3): E_xy 1.0→0.7→0.5 REGRESSES monotonically (6→5→4 SP; E_xy=0.5 adds a fly-out + IC5 all-3 collapse). So the relay+lag mechanism is FUNDAMENTAL, not forcing-magnitude-dependent — tightening E engages the κ-switching as a relay that the 38 ms lag feeds in-phase, regardless of forcing size. **KEEP E_xy=1.0; reducing it is a confirmed dead-end even at q=1.** (Pin all 3 axes `PLASMC_E_X/Y/Z` — any single bypasses the auto-align → all revert to bare 0.8/0.8/0.5.)
> - **⛔ 2026-06-26 CORRECTION (GT-FB, [[feedback_kappa0_unfreezes_lateral]]):** the "lateral fundamental limit, NOT gain-tunable" claim immediately below is OVERTURNED for the lateral RESIDUAL/wall — `kappa_0_xy` 0.125→0.5 un-freezes κ_xy → s_e_n converges → IC4/IC1 2/2 SP (n=2). The lateral residual IS gain-tunable; the GT-FB "wall" was the frozen-κ gain bug. (The target-visibility CBF accel-cut on a FAST/OFFSET target may still bind under perception-ON — untested.) PROVISIONAL.
> - **THE TWO FUNDAMENTAL LIMITS (both perception/observability, NOT gain-tunable — confirmed by saturation audits; relevant to the PX4 port: same CBF + virtual-compass):**
>   - **Lateral = the target-visibility CBF itself.** On a fast/offset target the CBF cuts the commanded lateral accel to **9%** of demand (100% of steps), lean is **100% cone-clamped**. The flow funnel `S_2` is NOT saturated (0.05) → **the control GAINS have headroom; the bottleneck is DOWNSTREAM**. So `chi_r`/`Gamma` retuning can't help — `chi_r=2` REGRESSES the nominal set (24→18). Visibility-vs-agility tradeoff, hard-capped by FoV. Fix = wider FoV (hardware) or anticipatory guidance, NOT a gain.
>   - **Yaw = ±90° marker-symmetry alias.** Square marker → `e_a` observable only to ±90°; only binds when the lateral collapse drags the drone far behind. Faster `e_a` convergence (`kappa_a0`/`Gamma_a`) pulls `e_a` off the alias but can't rescue a lateral-FoV-bound case.
> - **AGGRESSIVE-TARGET METHODOLOGY:** two SEPARATE tests — **25/25 = 5-IC set at baseline** (offset ICs bind), **±40% = CENTERED-IC speed sweep** (2–3× more headroom). Don't conflate. Trajectory 5-IC ceilings: Linear 1.1× (visibility-CBF-bound = the floor), Sinusoidal ~1.9×, Lissajous 2.5×, Circular 2.0×. **When a fast-target run fails, AUDIT THE LIMITS** (CBF accel-cut %, lean cone-clamp %, funnel `S_2`, `e_a`-vs-90°) before reaching for a gain — the gains usually have headroom; the binding is the CBF/observability.

## When to invoke

- "Tune the controller", "tune gain X", "sweep parameter Y"
- "Why did the landing fail / why is the controller diverging"
- "Should I change this parameter"
- "Run a sensitivity sweep" / "what parameters haven't we tried"
- Any time the user mentions a PLASMC gain by name (K_R, p_2_0, kappa, Omega, Gamma, E, N, etc.)

## Mental model — the seven failure modes

Every PLASMC landing in this SITL setup fails for exactly one of these reasons. Before tuning, identify which is happening.

| # | Failure mode | Diagnostic signal | Fix |
|---|---|---|---|
| **1** | **MAVSDK rate-loop lag** (architectural ceiling) | xy_mean stuck ~0.3-0.5 m across many gain configs; impulse-test t_d > 20 ms; PX4/MATLAB σ_xy ratio > 3× | Loop-lag reduction (see "Levers within MAVSDK" below). Not a tuning fix. |
| **2** | **Marker-detection breakdown** at z<0.3m | `Image Feature Pts` frozen for ≥100ms in final descent; sides stay <100 px through touchdown | Interventions 1+2+3 (committed 8baea2b). KLT fallback (`MARKER_KLT_MAX_STEPS=20`, default ON) further bridges short outages by LK-tracking last good corners — see [[klt-marker-fallback]]. |
| **3** | **IC sensitivity amplified by lag** | vh0 > 0.2 m/s → σ_xy diverges; PX4 σ_trajectory deviates from MATLAB by 4× in σ_xy only | Tightening IC tol alone doesn't help (validated). Need lag reduction (mode 1). |
| **4** | **Kr too aggressive** | Peak body rates >1.7 rad/s → LK loses corners → TARGET_LOST | `PLASMC_KR_SCALE=0.4` (current default) + `PLASMC_W_U_MAX=1.0` |
| **5** | **PID-output saturation** | a_u peaks > 100 m/s² before LPF; tau_ia can't smooth | tau_ia ∈ [0.08, 0.20] is the safe range. Higher = lag, lower = noise. |
| **6** | **Funnel-envelope breach** | rho_fov_log near zero, theta_current near theta_cone | Widen `RHOFOV0` or relax `THETACAP`. Very rare in current config. |
| **7** | **Sensor cal drift** | Optic-flow lstsq returns clipped or non-finite | Don't refresh sensor cal — methodology mismatch in `aggregate_calibration.py` produces 7-10× wrong gains (see `feedback_calibration_lessons.md`). Sensor noise already matches MATLAB per Phase 4. |
| **8** | **Compass yaw drift** under aggressive maneuvers | EKF yaw diverges 30-46° from GT in input-cal-style commands; body-frame analysis shows ~100m phantom position mismatch even though world position is correct within ~10m | Use `BODY_YAW_SOURCE=alpha` (control-side, **now the default**) — SO(3) loop uses the drift-free alpha feature instead of euler[2]. (NOT `V_YAW_SOURCE=alpha` — REMOVED 2026-06-04; it zeroed the yaw feature → open-loop yaw. See [[compass-free-yaw-sign]], [[v-yaw-source-alpha]].) Use `BODY_YAW_SOURCE=gt` in `plotter_input_calibration.ipynb` for analysis. |
| **8b** | **Compass drift at LANDING START → "yaw runaway"** (the real IC2-5 yaw failure) | At descent start GT yaw ≈ 77° while EKF yaw ≈ 0°; the IC rig holds/gates on the **EKF** yaw, so the drone BEGINS the descent physically yawed ~77° → `psi_d`→±180°, xy 2-16m. `alpha_start ≈ GT_yaw_start` every rep (**alpha is correct**). | **Don't touch alpha** — three alpha redesigns all hit 180° because the cause is the bad start, not the feature. Fix the **test rig** (`landing_test.py` IC convergence): servo NED yaw to null the **true (Gazebo)** yaw + gate on truth so the descent starts aligned. Controller untouched (yaw is image-alpha; compass only enters the rotation matrix, absorbed by the alpha outer loop). See [[yaw-compass-drift-ic-start]]. |

If the failure is mode 1 → no controller-side tuning fixes it. Mode 2 → already fixed by defaults. Mode 4-7 → tuneable. Mode 3 → secondary to mode 1. **Mode 8b → fix the IC rig, NOT the controller/alpha.**

## ⭐ 2026-06-25 FRONTIER — the terminal fly-away is an ACTUATOR-BOUNDED SMC wall (read first; supersedes much below)

Established this session via GT-feedback (PLASMC_GT_FEEDBACK=1, exact V-frame s/h → isolates control from perception). Detail in memory `px4/feedback_terminal_smc_actuator_wall`, `feedback_sp_task2_terminal_limit_cycle`, `feedback_kappa_4axis_hexy_param_map`, `feedback_lateral_flow_reduced_solve`.

- **Funnel = performance, SMC = convergence (distinct!).** The funnel `ζ=barrier(e/p)` only BOUNDS `e` within `±p(t)` (transient + precision); the SMC reaching law drives `σ→0` = convergence. Tightening a funnel does NOT "ensure convergence" — it shapes the envelope and *enrolls* the channel into the SMC objective.
- **The terminal `σ`-breach is NOT tunable on the SMC side.** `σ` breaches `E` because the *deliverable* authority is actuator-bounded (`g·tan(θ_cap)=17 m/s²`; a_u_xy commands 703, ~2% deliverable) while the `1/Z` disturbance is unbounded. More κ / Γ / θ_cap all fail or backfire (over-commanding pins tilt → steals vertical thrust → loom breach). **DEAD on the SMC side: κ caps, Γ, θ_cap, output clamps, loom clamp.**
- **Win it BEFORE the terminal:** keep the disturbance small = **arrest `v_lat` early** (verified: clean reps have `v_lat ≤ 0.5 m/s @ 1 m` → breach late/benign; flys carry 1.8–2.5 → breach high). `h_rd` CONSTANT.
- **`κ_xy` is FROZEN** — `N_xy=0.02` gives κ-ODE `τ=1/(N·P)=33 s ≫ 7 s` descent, so it never adapts (z `τ=2s`, yaw `τ=0.5s` do). No adaptive lateral robustness.
- **The flow funnel is too loose** — `p2_xy` 9–35 in descent → `ζ_h` only 7% of `σ_xy` → velocity damping is DORMANT → `v_lat` not arrested.
- **`E_z=3` kills the loom limit cycle** (the baked `E_z=0.5` is MIS-SIZED too tight → `sat(σ_z/E_z)` saturates → bang-bang balloon). `σ_z` residual caps at 3.66 = the barrier ceiling at the S_MARGIN=0.05 clamp → `E_z=4` or `P2INF_Z=2.0` closes it. `E_xy` stays 1.0.
- **`FLOW_LAT_REDUCED=1` baked** — drops the `w_xy` cols so lateral `h_xy` goes σ_min→σ_max (cond 14→2, corr-GT 0.2-0.3→0.5-0.65). ⚠ **needs a paired RECAL** (the cal `h_x/h_y` rows are a `w_xy` recombination that breaks under the reduced solve → ~3× under-read; re-run output-cal with `FLOW_LAT_REDUCED=1` → diagonal rows). `FLOW_TARGET_LEVEL=0` = full-solve fallback for a tilting deck.
- **Fix sequence — DONE & CONFIRMED (the velocity-damping lever WORKS):** recal `h_xy` ✓ → `N_xy=0.1` BAKED (826e655, wakes κ_xy, τ 33s→7s) ✓ → **tighten flow funnel `XI2_xy 0.2→0.5` to engage `ζ_h` ✓**. Result (GT-FB `20260625-202813`): ζ_h share 7%→18%, `v_lat` arrested, **tally 175559 4/2/3 → 6 sub/2 marg/2 fly**, NO new limit cycle (the loom cycle is SUPPRESSED even at E_z=0.5), κ healthy all 4 axes. The "safe floor" (altitude where the terminal 1/Z `v_lat`/`h_xy` loop triggers) drops ~1m→~0.15m → flys become subs. `h_xy` still spikes at `Z→0` (irreducible geometric 1/Z; metric = safe-floor altitude, NOT hxy_deck magnitude). **NEXT: `XI2_xy→0.7`** (+lower p_2_0_xy) to push the floor <0.05m + catch IC4_rep1; **trace IC2_rep1** (flew with v_lat arrested = woken-κ N=0.1 transient?). NOT baked — validate n≥5 + perception-on. Detail [[feedback_flow_funnel_zetah_works]]. ⚠ **MATLAB gains DON'T transfer** (porting χ_r=2.0/N=0.02 → 1 sub/2 marg/7 fly, IC1 centered 18m; the 38ms lag leaves a terminal residual χ_r=2.0 over-reacts to — PX4 χ_r=0.5 is the ceiling, [[feedback_matlab_gains_not_portable]]). All keep `h_rd` constant.
- ⚠ **`N_xy=0.1` REGRESSES alone** (the woken κ amplifies the terminal: κ chases `κ_eq=θ·G·|σ|/P`, `θ∝h_xy∝1/Z`, explodes 0.42→13.5; frozen N=0.02 was a de-facto SAFEGUARD). It's safe ONLY with the velocity damping (which stops `h_xy`/`κ_eq` exploding). `P_xy=1.5`=leakage (raise to bound the woken κ; partial). The κ explosion is FLY-ONLY (needs the v_lat loop to trigger).

## Complete parameter inventory

> **⭐ 2026-06-20 — LATERAL WALL MECHANISM SOLVED + partial fix ([[feedback_lateral_wall_anti_restoring_au]]).** The
> "perception-gated / flow-under-report" conclusion below is OVERTURNED by GT-grounded diagnosis on IC1 fly-aways.
> **ROOT = terminal 1/Z amplification, NOT perception:** the drone descends FINE to ~2 m (offset 0.25 m ±0.3, tight,
> decode 100%, flow accurate) — then `s_e_n = lat/Z` blows up a SMALL residual offset (~0.85 m) near touchdown →
> funnel breach → violent `ds_d` spike (4–6.6, vs ~0.5 controlled) → marker leaves FoV → TARGET_LOST → open-loop
> fly-away (the 7–60 m numbers are POST-loss). Ruled out, GT-verified: perception (decode/flow fine in overshoot),
> delivery/authority (brake commanded AND delivered, GT accel inward 4–14 m/s²), D1 sign (`PLASMC_AU_ROTZ_ONLY` no
> change), and **freeze-`s_e_n` (0/3 — lying to the flow-SMC/CBF/descent destabilizes; 64–110 m)**. **FIX = COMMAND-
> BOUNDING, `PLASMC_COMMIT_DSD_MAX`:** once committed (marker fills FoV, `PLASMC_COMMIT_EXTENT`≈50 px ≈1.3 m, median
> trigger `PLASMC_COMMIT_WIN`), cap `|V_ds_d|` while keeping `s_e_n` LIVE → bounds the reaction, no inconsistency.
> **IC1: variance STD 22.9→0.6 (38×), fly-aways 4/5→0–1/5 = WALL CLOSED (centered).** ⛔ **IC2-5 GATE FAILED
> (centered-specific):** off-center reps converge then still diverge — the position cap can't arrest the RESIDUAL
> LATERAL VELOCITY carried from the 2 m offset. **Next = velocity-aware terminal handling.** All default-off, NOT baked.
> **METHODOLOGY:** the lateral outcome is STOCHASTICITY-DOMINATED (baseline STD swings 2–22 m run-to-run) → n=5 single-
> config A/Bs measure noise; judge by VARIANCE COLLAPSE, not median. The stochasticity is purely TERMINAL (<1 m), not t=0.
> **Other 2026-06-19/20 (all default-off):** combined-barrier ported (parity-clean vs MATLAB except D1/D2/centroid-CBF;
> `PLASMC_COMBINED_BARRIER`); moment-loom (`FLOW_LOOM_DECOUPLE`, `FLOW_LOOM_GAIN=1.0`, MATLAB 92→95/100); CV-KF V_ds
> (`PLASMC_VDS_KF`, corr 0.41→0.52 vs smooth4; `PLASMC_VDS_KF_SCALE=0.79` = cal_hw/cal_s consistency); pyramidal-LK
> ruled INERT (deficit = ArUco decode-availability, [[feedback_pyramidal_lk_inert]]).

> **🟢 2026-06-15/16 — `KP 5→3` BAKED + INNER-LOOP/FLOW thread (read [[feedback_inner_loop_velocity_thread]] [[feedback_lateral_overshoot_root]]).**
> **BAKED: `KP_{X,Y}`=3.0** (6ee0f2c; KP sets the saturated-barrier demand ceiling ∝p_s·KP; 3 cuts arrival vel 3.4→2.8, peak vel 8.1→3.8, xy 22→8.6; IC2-5 gate fixed the IC5 catastrophe 27→2.3 m; KP=2 sluggish). `PS0` now **resolution-derived** (FoV-edge=1.0 in s_e_n + `PLASMC_PS0_MARGIN` default 0.2 = 1.2). `CBF_LPF_BEFORE` default-**off** (reverted; breaches funnel early at altitude). `N_z`=0.1 (≠ MATLAB 0.02; parity doc fixed). `theta_cap` moved OUT of cbf2_filter → controller (CBF now pure-visibility).
> **GAINS PLATEAUED — every signal-side lever ruled out as a SOLE cure (NC80-87):** KP↓ (winner, not curer), PS0-widen (raises demand, dead-end), KD 0.8/1.0 (no TL removal), XI2_xy=0.8 (stochastic), N_xy=0.1 (neutral), `SEN_RECOVERY_K` (outer authority — didn't reduce breach), `FLOW_CENTROID_RATE` (inner velocity — beat the LK ceiling h/true 0.37→0.96 but still 5/5 TL), both-pillars CR0.5+REC (5/5→2/5, not cured), `CH_CLEAN` consistent c_h (smoother+non-regressing, not wall-breaker).
> **ROOT (triangulated):** s_e converges in p_1 then OVERSHOOTS; h_e converges but s_e_n doesn't because the SMC tracks the LK flow `h` which **under-reports the true velocity ~2.4× (saturating sensor, ~1 rad/s ceiling at the lstsq SOURCE)**; cascade fails on BOTH pillars (inner velocity saturates + outer `G_s⁻¹` collapses at the funnel boundary, 4-7×, killing the integral recovery). `a_u` points OUTWARD in the overshoot (h_d dominated by `cross(w_i,s)`). **The one untried lever = the COMBINED SLIDING VARIABLE `σ = ζ_h + λ·ζ_s`** (best `ζ̇_s + λ·ζ_s` with `ζ̇_s` from the accurate centroid-rate) — direct position authority, escapes the ceiling AND the cascade-decoupling. Lyapunov re-derivation; MATLAB first.
> **NEW default-off env knobs (behavior-preserving; A/B before trusting):** `FLOW_CENTROID_RATE`∈[0,1] (blend centroid-rate into h[:2]; 0.5 best), `PLASMC_CH_CLEAN` (consistent c_h: clean c-term + drop w×s from h_d; smoother), `PLASMC_SEN_RECOVERY_K` (escalating funnel-breach recovery outside G_s⁻¹), `PLASMC_DSD_LAT_MAX` (ds_d cap — diagnostic; KP is the real lever), `FLOW_KF_Q`/`FLOW_KF_R` (flow-KF bandwidth, marginal), `FLOW_COND_REJECT` (condition-aware lstsq outlier rejection = SMART EKF-flow smoothing, ceiling-preserving), `PLASMC_PS0_MARGIN`.

**2026-06-03 CLEANUP: all knobs are now DIRECT per-axis parameter values** (e.g. `PLASMC_KP_X=1.4`),
not scale factors. Obsolete mechanisms (K_rp scheduling, DSD clamps, VFRAME_ROT, PX4_RATE_SCALE)
were removed — see parameter_record.ods sheet `Removed_Parameters`. (THETA_FLOOR was RE-ADDED; see FoV cone.)

> **⚠️ 2026-06-06 CAL-REGIME RESET + NEW KNOBS — read [[project_tuning_campaign_newcal_reset]] first.**
> Tuning **restarts** under the honest 8-run cal (`d60973a`). There are **3 cal regimes**: trial 46's
> "28% SP @ 10cm" ran under the *multisine* cal (regime 2, still had the V-frame g-sign bug) — it is now a
> HYPOTHESIS, not a result. Everything in "Known winners / dead-ends / frontier" below is regime-1/2 and
> mostly GAIN-STARVED; the memory has the re-classified prioritized 6-knob sweep order.
> **Corrected current code defaults vs the tables below:** `PLASMC_DH_D_MAX`=**50.0** (default; NOT reduced — intentionally kept to expose real failures),
> `PLASMC_THETA_FLOOR_DEG`=**60** (not 0), `PLASMC_LFOV`=**0.0** (not 0.1, rho_fov held constant).
> **2026-06-10 baked defaults:** `PLASMC_KD_{X,Y}`=**0.0** (K_rd=0; gamma_s=1.0 replaces D-term damping role),
> `PLASMC_XIS_{X,Y}`=**1.0** (gamma_s=1.0 outer funnel), `PLASMC_KAPPA0_Z`=**1.0** (bootstrap fix),
> `PLASMC_KAPPA_MAX_Z`=**3.0** (κ_z cap), `PLASMC_DH_D_MAX`=**50.0** (stays at 50 to expose real failures),
> `PLASMC_DW_MAX`=**30.0** (cleaner image-rate dw clamp, commit 85e1011).
> **θ_norm is CONTAINED downstream, NOT eliminated** (corrected 2026-06-10): the img_data KLT-bounds guard
> removes the *sustained off-screen-KLT* source (s[0]→3.15, w_z→4.54); the cleaner image-rate dw tidies the
> *frame-jump* peak; but θ still spikes (~480) and the κ-cap + P-leakage contain it. **dw source-fix +
> θ-freeze are DEAD-ENDS** (see the 2026-06-10 addendum below). Trial-49 IC1 done; IC2-5 still pending.
> **New knobs not in the tables:** outer PPC funnel `PLASMC_SEN_FUNNEL`(off)+`XIS/PS0/PSINF_{X,Y}`;
> visibility CBF `FUNNEL_MODE`(cone|cone0|cbf1|**cbf2**, **default cbf2**)+`CBF_TAU`+`CBF_DMIN_EMA`+`CONE0_{SWAP,SIGN_X,SIGN_Y}`+`CBF_PHASE2_HYSTERESIS`(3)+`CBF_PHASE2_RAMP_FRAMES`(5); cbf2 now uses two-phase δ: Phase 1 (decode) m2=φ_max (centroid-only, δ=0); Phase 2 (overflow/fail) ramps δ→½ptp with hysteresis; theta_cap applied post-QP (not inside loop);
> `PLASMC_TAU_UA`(0.1 yaw-rate LPF); `PLASMC_YAW_PSID_RATE`(0.7); `W_XY_DEROT`(zero|imu|image, =zero via CTRL_ZERO_WXY=1);
> `YAW_TERMINAL_HOLD`(on)+`YAW_HOLD_ALPHA_RATE`(3.0)+`TERMINAL_STABILIZE`(off); `IMG_FEAT_KF_R`(0.004, centroid-KF noise).
> Sweep harness: `scripts/run_knob_sweep.sh`. IC2-5 gate: `run_ic_validation.sh` — never pass it `LANDING_OUT_BASE`.

> **🔑 2026-06-07 NEW-CAL CONTROLLER RESULTS — read [[feedback_newcal_tuning_results]] + [[feedback_sitl_reliability_fixes]].**
> First controller-engaged flights under correct cal. **The boundary layer `E` is THE knob that bounds the
> κ-runaway at full authority** (κ-ODE grows only when `|σ|>E`): `PLASMC_E=2.5` (CBF relaxed) → κ≈k0,
> a_u 6 (vs 16705 at E=1), lateral held **0.04 m dead-center** — BUT too soft → hover (need `E_Z` lower
> for descent). **cbf2 was MASKING the κ-runaway** (clamps a_xy 51–84%); it's a SAFETY NET, not a
> controller — keep it RELAXED (`THETA_FLOOR_DEG=60`) and bound κ via `E`. If the CBF triggers in normal
> ops, the control law is failing. This UPDATES the "lateral drift = pure lag floor" story — it was largely
> κ-runaway + CBF-masking; lateral CAN converge once κ is bounded.
> **TUNE PER-AXIS, NOT ALL-AXES-TOGETHER** (user directive 2026-06-07, reinforces [[feedback_per_axis_tuning]]):
> e.g. sweep `E_Z` alone (descent) vs `E_X`/`E_Y` (lateral) — `E=2.5` on all axes solved lateral but killed descent.
> **New-cal dead-ends:** `KP≥13` overshoot; `KI≥2` integral-windup → κ-runaway returns; `W_U_MAX>1.7` breaks
> LK (TARGET_LOST); `THETA_FLOOR` low just masks κ. `P_Z=5` (MATLAB parity, κ_eq∝1/P) tames κ_z. **n=1 is
> noise** — KP=12 looked like 1.5 m at n=1, gave mean 6.1 m / 0 SP at n=5. Per-rep root cause:
> `tools/diagnose_failure_cause.py <rep>` (perception vs control via image-`s_e_n` vs GT-lateral at onset).

> **🧭 METHODOLOGY RULES (user, 2026-06-07) — read [[feedback_dont_conclude_lag_floor]].**
> **(1) Don't conclude "lag floor / architectural ceiling" while control levers remain.** If relaxing a
> SAFETY-NET clamp (cbf2 cone via `THETA_FLOOR=60`) makes a failure WORSE, that clamp was **MASKING an
> under-tuned controller** — relaxing it EXPOSES a tuning gap to FIX at the control level (same as κ: cbf2
> masked it → relax exposed it → `E` fixed it). It is NOT proof the clamp is load-bearing or that the
> residual is lag. The 125 Hz control loop is *fast*; actuation lag (38–61 ms roll / 287 ms yaw) is the LAST
> resort, not the first explanation. **(2) One knob, one job.** `E` was doing two conflicting jobs — it
> bounds κ (raise E) AND sets SMC stiffness (raise E → softer tracking). `E=2.5` bounded κ but *softened*
> the lateral hold→drift and the descent→hover. Decompose: **use `P` (κ_eq∝1/P) to bound κ; use `E` only for
> stiffness**, PER-AXIS. Don't let a κ-bounding move silently detune tracking.

> **📊 2026-06-09 DEEP SWEEP RESULTS — 42 rows in PX4_NewCal_Record. [[feedback_newcal_tuning_results]] [[feedback_kp_e_coupling]] [[project_tuning_campaign_newcal_reset]].**
> **BEST CONFIRMED STACK:** `KP=9, E=[2.5,2.5,0.5], P=[5,5,5], KI=1.0` → median **3.80m**, min 1.95m (n=5).
> **KP×E COUPLING (new):** KP and E_XY cannot be tuned independently. KP=12+E=1.5 works (3.20m, SEN_FUNNEL blocks t=0 LK spike). KP=12+E=2.5 is WORSE (4.79m) — wide E keeps σ<E so κ≈κ_0; higher KP drives h_d windup faster → earlier funnel breach. **Rule: KP=9 with E_XY≥2.5; KP=12 only with E_XY≤1.5.**
> **ALL PARAMETERS CONFIRMED AT OPTIMAL DEFAULTS (no further gain headroom):** Gamma_Z=0.75, KR_YAW=2.0, P2INF_Z=1.5, TERMINAL_HOLD_EXTENT=70, YAW_OMEGA=0.5, PSID_RATE=1.0, tau_ua=0.1, E_Z=0.5 (marginal over 1.0).
> **DEAD-ENDS (do not retry):** KI≤0.35, E_Z≥1.5, KP=12+E=2.5, N_Z=0.05 alone, tau_ua=0.3, P_XY=3 (9.75m — P controls κ_eq∝1/P), KP≥13.
> **HOVER-AT-CENTER (NOT SP):** 3 events where xy=0.0/vel=0.0/flight>50s. Boundary-layer-induced (σ<E throughout). Classified FAIL.
> **BINDING FAILURE (2026-06-09):** Stochastic LK/ArUco collapses — 1-2 TL per 5 reps even at perfect IC. LK dynamic range ceiling ~2 m/s. NOT gain-tunable. Gap between xy_min (~2m) and xy_med (~4-6m) is entirely stochastic perception. **[⛔ 2026-06-26: "NOT gain-tunable / LK-dynamic-range" as the LATERAL-WALL verdict is SUPERSEDED — that was the frozen-κ gain bug; `kappa_0_xy`=0.5 fixes the lateral GT-FB ([[feedback_kappa0_unfreezes_lateral]]). The stochastic perception TL may persist under perception-ON — untested.]**
> **DEAD-ENDS (do not retry):** KI≤0.35, E_Z≥1.5, KP=12+E=2.5, N_Z=0.05 alone, tau_ua=0.3, P_XY=3, KP≥13, **N_XY=0.05** (5.87m vs 3.80m — faster κ without σ signal = noisy).
> **GAIN-SIDE LEVERS — `gamma_s>1.0` + `KP=12` SWEPT (NC56-60, 2026-06-10, IC1 n=5):** `gamma_s=1.2` → **0 TL** (vs baseline's 2) but **1/5 hover** (over-centers → σ_z weak → no descent); `gamma_s≥1.4` degrades (1.4 catastrophic 8.7 m) — ceiling is **descent-weakening, NOT a demand-breach** (ds_d stayed small). **`KP=12+E=1.5` → tightest landings of any cell (0.34 best ever, no t=0 LK collapse — SEN_FUNNEL works) but 1/5 the touchdown lateral breach fires harder (κ_xy=0.85, a_u_xy=521) → next experiment = `KP=12 + κ_xy cap`.** Neither cleanly beats baseline at n=5. Binding limit underneath is still the LK dynamic range ~2 m/s; main next lever: code-level perception.
> (1) **Pyramidal LK levels 2→3** in `img_data.py:getLKFlowAngVel` — directly extends dynamic range ceiling ~2 m/s → ~4 m/s; addresses binding stochastic TL failures. (2) `KP_Y < KP_X` — asymmetric KP (low priority). (3) Anti-windup on outer PID: freeze `is_e_n` when `h_e/p > 0.7`.

> **🔧 CLAMPS ARE BAND-AIDS — disable during tuning, re-engage after (user, 2026-06-08). [[feedback_clamps_during_tuning]].**
> A clamp that BITES during tuning is masking poor control + hiding the signal you need. Tune the bare law,
> then re-engage protective clamps. Prefer proper control techniques (conditional integration, gain shaping)
> over fixed clamps. **Clamp audit vs MATLAB:** A=canonical/keep (`sat(σ/E)`, FoV cone, `izeta`); B=PX4-physics/
> keep (`W_U_MAX` LK>1.7, `DH_D_MAX` 1/Z spike, `W_I_MAX`, `YAW_TERMINAL_HOLD`/`STALE`); C=large-yaw band-aids
> MATLAB has NEITHER (psi_d-rate clamp + `_ie_a_clamp`). **DONE: `_ie_a_clamp` REMOVED → conditional integration**
> in `_yawCtrl` (freeze `ie_a` while heading-rate saturated) — fixed the windup→overshoot (102°→−22°). MATLAB
> spawns SQUARE (never perturbs initial yaw); PX4's large-initial-yaw is the (B) extension beyond the reference.

> **🟢 2026-06-10 DESCENT-HOVER THREAD + TL CAUSES — [[feedback_descent_hover_thread]] [[feedback_theta_norm_klt_drift]].**
> **BAKED: cleaner image-rate `dw`** (commit 85e1011) — divides the frame-jump by the REAL inter-frame interval + zero-between-frames + clamp `PLASMC_DW_MAX=30`; tidies the θ_norm *peak*, integrator-safe; no regression at E_z=1.0 (κz=1.00).
> **DESCENT HOVER (1/5 at E_z=1.0):** fixed by `E_z=0.5` (stronger descent drive) BUT it un-bounds κ → touchdown κ-runaway (**one-knob-one-job**: E_z does descent stiffness AND κ-bound). **`E_z=0.5 + P_z=8` TESTED + FAILED (NC55):** 1 TL, mean 7.77 m. P_z=8 didn't even bound κ_z (still hit cap), and the catastrophe is a **LATERAL κ-runaway (κ_xy→7.26, a_u_xy→631)** that P_z (z-only) can't touch — E_z's faster descent amplifies the close-range 1/Z lateral error. The lateral κ-lever is `P_XY↑` (untested, softens tracking); the true driver is close-range 1/Z geometry/perception. E_z=0.5 NOT baked (hover↔κ trade).
> **NEW DEAD-ENDS (do not retry):** (1) **image-rate-HOLD dw** — holding dw between frames turns brief θ spikes into *sustained* moderate θ → poisons the κ-INTEGRATOR → MORE runaways (3/5 vs 1/5). (2) **sustained-high-θ κ-freeze** (2nd trigger) — mis-targeted: κ ratchets at MODERATE θ (median 3-7, freeze OFF) via large G·|σ|, NOT the brief spikes it chased; no θ threshold (50/200) catches it. Both reverted/dropped. **Validate κ-ODE fixes against the time-INTEGRAL/sustained θ, never the peak; judge feedforward changes in closed loop.**
> **TL CAUSE BREAKDOWN — re-categorized by the ACTUAL image centroid/corners (2026-06-10, 12 campaign TLs):** **9/12 = ArUco DECODE failure with the marker FULLY in-FoV** (4/4 corners in at the loss, mostly centered — mode-2 close-range; **NOT a geometric loss**); **3/12 = clip** (≥1 corner out; 2 of those = drone already flew off). The **detection-loss is the TRIGGER** of the κ-runaway (loss precedes runaway 0.80<0.88): freeze → held/extrapolated feature → off-screen VIRTUAL centroid → wrong `h_d` → κ-runaway. The nan in `Img_Data["Quat"]` is the marker-LOST sentinel (img_data:1007-1009), not an attitude failure (FC quat valid). **Lever:** the real ArUco corners ARE stored (`_feature_pts`) → **KLT corner-track the in-FoV corners** through the decode gap + decode robustness + a corners-based CBF; during marker-LOST **use genuine data (FC quat + KLT corners), do NOT nan/extrapolate** the centroid (the extrapolation is what reprojects off-screen)."

> **🔬 2026-06-10 LATERAL κ-RUNAWAY = TOUCHDOWN FUNNEL BREACH — [[feedback_lateral_kappa_runaway]].** Decomposed (P_z=8 rep3, κ_xy=7.26): at alt<0.5 m the 1/Z geometry breaches the **inner** funnel (`|h_e/p_2|→0.99`) → ζ→5.3 → σ→3.6, G→3.1 → κ-ODE growth `θ·N·G·|σ|`=16.1 overwhelms leakage `N·P·κ`=0.10 by **160×** → κ_xy slams up. **UNBOUNDABLE by any gain:** P can't (needs P~800 — this *corrects* the "lateral lever = P_XY↑" note above), θ-freeze can't (θ moderate 37–72), the Singhal freeze MISSES it (fires at |h_e/p|≥1.0, growth is at 0.9–0.99), and **κ_xy is UNCAPPED** (`KAPPA_MAX=[1e6,1e6,3.0]` — only z capped at 3.0; that's why κ_x hit 7.26 vs κ_z's 3.0). Same root as the perception TLs (1/Z touchdown blowup). **At IC1 lateral converges FAST** (early |s_e_n|≈0.04) then **re-grows to 0.74–0.99 at touchdown** — not slow convergence, but the descent reaching the 1/Z zone.
> **FIX STATUS (2026-06-10): clamp the PHANTOM s, NEVER the GENUINE s.** Two img_data guards were implemented. **#2 (`PLASMC_VIRT_GUARD`, global `_getVirtualPts` z_v-floor + ±(p_10+δ) clamp) REGRESSED → DEFAULT-OFF:** it clamps the GENUINE in-FoV centroid when the drone is off-center (the far-drift reps did NOT lose the marker, "lost 0%") → bounded s_e_n + κ-growth → SMC under-corrects (a_u≤8, κ≤0.2) → far drift (29/91 m, E_z=1.0 mean 1.94→10.3, VirtGuard_EZ_IC1). The genuine off-center s IS the error signal + drives the κ-authority. **#1 (`PLASMC_FEAT_FOV_CLIP`, clip the marker-LOST EXTRAPOLATION to ±(p_10+δ)) = DEFAULT-ON:** conditional, bounds only the phantom that caused the original κ-runaway, never touches a genuine detection. + FC quat kept valid through marker-LOST. Re-test #1-ON/#2-OFF = FeatClip_EZ_IC1 (pending). **3rd open-loop-validated/closed-loop-negative fix (after dw-rewrite + θ-freeze): clamping the GENUINE feature is the harm; the real lever is keeping the marker DECODED (KLT corner-track).**
> **FIX = convergence-ordering, not a gain:** (A1) **gate the descent on the image error** `h_rd_eff = h_rd·g(|s_e_n|)` (slow until centered → full; scale-free, continuous — NOT the failed Batch-9 hard handoff [[feedback_convergence_ordering]]); (B1) **κ_xy cap = 3.0** (backstop, symmetric with z); (B2) **freeze κ at |h_e/p|≥0.9** (before the barrier singularity, not Singhal's 1.0). **Funnel mechanics (controller.py:510-517):** `gamma_s`(XIS)=outer `p_s` contraction = lateral-convergence speed (untested >1.0); `gamma`(XI2)=inner `p_2` contraction → raising it accelerates the breach (don't).
> **🔗 2026-06-11 IC=2 LATERAL GAIN CHAIN — [[feedback_ic2_lateral_gain_chain]].** Four stacked gain defects, each data-confirmed at IC=(2,0,5) with FULL-FLIGHT logs (the controller's re-init log-wipe is FIXED — 162b86e — logs now persist 'until the controller fails'): **(1) γ₂(XI2)=0.2 keeps the middle funnel so wide that σ sits at 5–20% of E for ~10 s — the SMC is structurally ASLEEP during the approach and κ LEAKS 0.156→0.06 (un-adapts); MATLAB runs γ₂=1.2/1.5** → `XI2_XY=0.6` closes a 2 m offset in ~3 s. **(2) κ unarmed at the center-crossing** → `KAPPA0_XY=0.5` damps it (useless without XI2 — sat gates it). **(3) γ_s=1.0: the OUTER funnel overtakes a 2 m error at t≈1.2 → ratio clamp → G_s⁻¹∝p_s collapse → DEMAND STARVATION — a 'nicely damped hold' can be a saturated crawl; ALWAYS check r=s_e_n/p_s before believing a hold** → `XIS_X=0.5` keeps the funnel valid (sustained demand, genuine 1.2 m/s closure). **(4) K_ri=1.0 is 10× MATLAB (0.1) — windup ∫ζ_s≈−2.4 pushes ds_d=+1.4 THROUGH center** → `KI_X=0.1` makes the demand reverse correctly at the crossing (the 10× was compensation for the OLD starved-demand regime). **(5) K_rd=0 vs MATLAB 0.5031 — NO PHASE LEAD = structural overshoot.** Counter-proof: `GAMMA_X=1.0` produced a +10 kick AT the crossing (the middle loop is a velocity TRACKER enforcing a lagging demand — no σ-side gain can brake before center). `KD_X=0.5` (MATLAB parity): **crossing overshoot 5× smaller (+0.6 vs 2.4–9.5 m), first damped ring-down (0.47→1.10→0.58)** — the old K_rd dead-end was the CLOSE-RANGE corner-jump spike (DH_D_MAX guards it; approach-altitude centroid is KF-clean). OPEN: the descent reaches the deck before the ring-down settles (descent pacing / funnel-gated h_rd) + the terminal close-range wall. **STACK BAKED 2026-06-11 (c5f9989): KD_XY=0.5, KI_XY=0.1, XIS_XY=0.5, PSINF_XY=0.35, XI2_XY=0.6, KAPPA0_XY=0.5, h_rd=−0.25 — now code defaults; VALIDATION DEBT: n=5 IC2 + IC1 regression + IC2-5 gate.** Post-bake findings: (6) dh_d-spike attribution CORRECTED — ~100% the numerical rate of the GENUINE cross(w_i,s) feedforward (gyro-verified real yaw motion, std 0.8/peaks 7 rad/s), NOT the KD flip; USER RULE: capping dh_d/dw is NOT the solution (DH_D_MAX/DW_MAX stay 50/30) — structural fix = smooth/analytic ḣ_d or gyro-sourced w_i (OPEN). (7) The post-crossing plateau = the SATURATED-BARRIER demand ceiling: outside the funnel ζ clamps at 3.66 and G_s⁻¹∝p_s·e^(−ζmax) → ds_d pinned at ~G_s⁻¹·KP·ζmax≈0.6–0.8 REGARDLESS of error (verified −0.8…−1.0); ceiling ∝ p_s·KP → PSINF_XY=0.6 was a candidate, now a **TESTED DEAD-END (2026-06-12 addendum below).**
> **🔴 2026-06-12 BAKED-STACK n=5 VALIDATION + CASCADE + PSINF DEAD-END + MATLAB-VALS PROBE (NC61-64) — [[feedback_ic2_lateral_gain_chain]] item 9.** New tool `tools/analyze_baked_validation.py` (validity gate: reject flight_s<2 / empty GT / final_alt>0.25 m + per-axis funnel residency r=s_e_n/p_s) + `p_s(t)`/`dp_s(t)` now LOGGED in controller `_buildLogDict`. **Baked stack (c5f9989) n=5: IC2(2,0,5) SP 0/5, 1 TL, xy_med 10.14; IC1(0,0,5) SP 0/5, 1 TL, xy_med 4.01 — regresses pre-bake 1.46.** **VERDICT — the chain WORKS in approach but the TERMINAL WALL blocks it:** every rep converges (IC1 to dead-center |s_e_n| 0.00–0.03; IC2 closes the 2 m offset to 0.07–0.27) then blows up at GT_alt **0.9–2.7 m** (58–93% through descent; a_u_xy to **1087**; 2/5 IC2 catastrophic spurious-flow-h, h_meas_x 17–27 = the ill-conditioned n_flow_corners→4 lstsq). The big final xy is the wall, not the gains → **bake NOT reverted** (approach gain real), **IC2-5 gate NOT run** (no winner to promote). **TERMINAL CASCADE (diagnostic):** `zeta_1` (OUTER position barrier, reconstruct: r=s_e_n/p_s clamped ±0.95 → log((1+r)/(1-r)), ceiling 3.66) **saturates FIRST** as s_e re-grows near the deck → outer demand collapses (G_s⁻¹∝p_s·e^−ζ→0) → `zeta_2`(middle)→ceiling → `sigma`≫E → κ; **severity tracks whether it propagates to the middle loop** (zeta_2 stays sub-ceiling → mild 2 m landing; zeta_2 saturates → catastrophe). **PSINF_XY=0.6 DEAD-END (NC63, n=1 IC2):** hypothesized the funnel FLOOR was binding (since zeta_1 saturates first) → FALSIFIED — terminal s_e re-grows to **~1.2–1.7 rad ≫ any floor** (0.53–0.71 at 0.6) → zeta_1 STILL pins 3.66, xy 2.38 vs baked 2.23 (no change); binding = terminal s_e MAGNITUDE = off-screen virtual centroid = PERCEPTION, not the floor (only effect: calmer z, a_u_z 14.7→4.7). **MATLAB-canonical gain VALUES (NC64, IC=2,2,5 n=1) → TL 6.48 m:** approach converges LESS (|s_e_n| 0.249 vs baked 0.066) — confirms the gain-chain's XI2 0.2→0.6 bump (MATLAB γ₂=0.2 = lazy middle funnel). **STANDING CONCLUSION: NO outer-funnel/SMC gain fixes the terminal wall** — the binding lever is keeping the feature valid through the close-range tilt (decode-robustness / off-screen-virtual-centroid / KLT corner-track), an ARCHITECTURE item.
> **🧭 2026-06-13 TWO-TASK FRAMEWORK ANALYSIS — [[feedback_plasmc_two_task_framework]].** Soft-precise = `s_e<p_1` (Task 1, outer) AND `h_e<p_2` (Task 2, middle). **30-rep measurement: Task 2 MET (100% inside, h_e RIDES the p_2 boundary ~3 rad/s); Task 1 FAILS (62%, breaches 3-8×).** **The outer barrier-PID is feedback-linearizing — provably gives `ż₁=−Kp ζ₁−Ki∫ζ₁−Kd ż₁` → `s_e→0` GUARANTEED — but ONLY if `ṡ_e=ds_d` (inner loop achieves the commanded feature velocity).** It doesn't: `ṡ_e≈ds_d−h_e`, so `ż₁=(PID)+G_s·(−h_e)`; near the edge `G_s` is huge → the unmodeled `h_e` dominates → `s_e` diverges. **Convergence fails from the broken feedback-lin assumption (unmodeled h_e), NOT PID gains** (why every outer-gain sweep was neutral). The two signals are KINEMATICALLY linked: `ṡ_e−ds_d≈h_e`, so s_e leaves p_1 at rate ~h_e; `h_e<p_2`(wide) ≠ `h_e≈0`. **h_e can't reach 0 because the SMC SATURATES** (`|σ|≈3.7≫E=1`) tracking a huge `h_d` that is **85-98% the rotational FF `cross(w_i,s)`** (`w_i` spikes ~5 rad/s at close range from the descent tilt; the calm landed rep had w_i=0.9, h_d=2.9). **PARAM ROLES (code-confirmed): Gamma=reaching-law gain (h_e convergence speed); E=boundary-layer (steady-state smoothing); Omega=surface slope; P=κ-leakage.** GAMMA_xy=1.0+E=1.5 (NC76, n=1) made the middle loop markedly CALMER (|σ| 3.7→1.92, a_u 770→30) but did NOT fix Task 1 (h_e still rides p_2, demand untouched) — Gamma/E help the SMC REACH, they don't reduce h_d. **LEVERS (ranked): (1) descent-gate `h_rd·g(|s_e_n|)` (reference governor — shrinks terminal w_i→h_d→SMC reaches→h_e→0→restores the guarantee); (2) backstepping (+h_e term in ds_d); (3) Gamma↑ then E↑ AFTER h_d reduced. NOT: outer P/I/D, p_2 floor (P2INF DOESN'T BIND — funnel never reaches its floor in the ~5 s descent, NC75; lower P20 or raise XI2 instead), Omega↑, un-clamping the barrier.** The de-rotation `_getVirtualPts` is VERIFIED CORRECT (skew-free unit test) — the earlier "virtual under-reports" was a clock-skew artifact; zeta_1 saturation is genuine lateral drift during descent (control), NOT perception — see [[feedback_ic2_lateral_gain_chain]] item 11.
> **🟢 2026-06-13 SOFT-CONFIG BAKE — first soft+precise of the campaign — [[feedback_descent_softness]].** Two corrections to the bullet above + the descent-softness solve. **(A) `GAMMA` IS the `h_e`-reaching lever** (corrects "Gamma/E help reach"): κ_0/E are SATURATION-GATED (when `|σ|>E`, `sat=±1`, so raising them does nothing during reaching) — only `GAMMA` (the un-saturated `−Γσ` proportional term) speeds reaching. `GAMMA_xy 0.5→2.0` cut h_e reach-time and drove `w_u`→1.0 clamp → tightest lateral of the session (xy 0.80). Ceiling = `W_U_MAX`/LK at ~Γ=2.0. **(B) `P2INF` DOES bind once `XI2` is faster** (corrects "P2INF doesn't bind"): at the old XI2_z=0.2 the funnel never reached its floor; with `XI2_z=0.6` baked, `p_2_z(2s)=e^(−0.6·2)(4−p₂∞)+p₂∞` so `P2INF_z=1.0→p_2_z 1.90` (vs 2.25 at 1.5) → tighter h_e_z → vel 1.06 (softest). **DESCENT SOFTNESS (the hard 3-4 m/s touchdown):** root = `h_rd` commands `v=|h_rd|·Z` (fast at altitude) + **1/Z observability** (loom `h_z=v/Z` blind to velocity at altitude → under-braked → late overshoot at ~1.75s); **`h_e_z` structurally never →0** (under-descend early `h_e_z>0`, overshoot late `<0`). **Softness levers (cut vel 2.84→1.06): `KAPPA0_Z=1.0`** (bootstrap — z braking from t=0; Combo had reverted to 0.25, costing softness; ALSO prevents the E_z=0.1 κ_z-ratchet), **`N_z=0.1`** (κ_z is adaptation-rate-limited in the ~2s descent — N matters here though `κ_eq∝1/P` is N-independent at equilibrium), **`E_z=0.1`** (crisp arrest, un-bounds κ_z ALONE → **only stable PAIRED with κ0_z=1.0**; the earlier "E_z=0.1 dead-end" was a κ0_z bootstrap problem), **`P2INF_z=1.0`**. `Γ_z`/`XI2_z` (tracking) do NOT soften. **BAKED 2026-06-13 (ODS NC77):** `XI2=0.6/0.6/0.6 · P2INF=1.5/1.5/1.5 · OMEGA=0.1/0.1/0.025 · GAMMA=2.0/2.0/1.0 · E=0.8/0.8/0.1 · N=0.02/0.02/0.1 · KAPPA0=0.5/0.5/1.0`. n=5 IC2: 3 clean soft-precise (xy 0.85-1.50, vel 1.66-2.44) + 1 lateral-wall TL + 1 flake. **NOT gate-clean** (IC2-5 pending; the TL is the config-independent lateral perception wall). **UNEXPLORED: `h_rd` reference-governor** (ease near deck — the cause-side fix for the under-braked-at-altitude half).
> **🟢 2026-06-11 TERMINAL-DESCENT FIX: z-axis RING-WEIGHTED LOOM (`RING_LOOM_NCORN=12`) — [[feedback_terminal_descent_loom_overreport]].** Root of "no precise touchdown": the descent law is CORRECT to ~1 m (tracks `h_rd=−0.42` perfectly, v∝Z) but **below ~0.5 m the corner LOOM over-reports ~4×** (noisy/spiky; actual descent ~0.1 m/s near-ideal) → z-SMC over-brakes (`a_u_z`→−9) → the drone **BALLOONS up** instead of touching down → drift → marker loss → open-loop off-center touchdown. NOT control lag, NOT a fast descent, NOT gains. The **ring divergence is the cleaner terminal loom** (4–6× stations, ½ noise, no spikes, survives marker-loss; the loom IS planar-observable — unlike the lateral, where the ring has ~0 info, the OPPOSITE of the failed `FLOW_NCORN_SWITCH`). **Prototype (f194989):** when `n_flow_corners ≤ RING_LOOM_NCORN`, inflate ONLY the corner loom-row R (`R_c[2,2]=1e6`) → ring carries the vertical, corner keeps the lateral. **Single-run: xy 1.86→0.41 m, balloon +0.33→+0.01 m, marker kept**; n=5 `RingLoom_IC1` reps 1–2 = 0.59/0.27, 0 TL (pending; then IC2-5 gate before baking). Also 2026-06-11: all prior session guards REMOVED from img_data (baseline restored); `IMG_RECORD=1` video FIXED (frameSize transpose); marker back on `aruco_board` + 6×6 cal RESTORED (the textured-marker thread closed: planar translation↔tilt ambiguity, not corner count, limits h_x/w_x/w_y — cal un-derivable from the tilt-only sweep).
> **🟢 2026-06-19 LOOM ESTIMATOR ROOT CAUSE + SCALE-FREE AREA-RATE FIX (MATLAB single-run; transfers to the PX4 loom over-report above) — [[project_moment_loom]].** The loom over-report/noise is a flow-ESTIMATOR problem: `V_h(3)=vz/z` is the **flow divergence**, recovered as the 3rd row of `pinv(L_s)*dPdt` where the vz column `[-x;-y]` is the **weakest singular mode** (σ_min≈2 of a cond≈290 matrix). The pseudo-inverse SVD tolerance is a **regularization-vs-resolution dial with no good setting** — truncate the loom mode → under-report/sign-flip → over/under-brake; retain it → amplify finite-diff noise → terminal limit cycle. **The cleaner loom is the SCALE-FREE area-rate divergence: `V_h(3) = −½·d(ln M)/dt`, M = 2nd-moment corner spread** (= ½ the fractional area-expansion rate). It uses only the FRACTIONAL rate (size/depth-invariant, fixed ½, no calibration), is **rotation-immune** (curl/shear are area-preserving; only divergence changes area → no gyro, robust to a moving/rotating deck), and **decoupled** (single scalar, no ill-conditioned inversion). MATLAB: better-calibrated than the pinv loom esp. under noise (corr 0.92 vs 0.87, slope 0.79 vs 0.75 vs truth) → closed-loop 92→95/100, lower breach. **This is the principled generalization of the RING_LOOM idea** (the ring is a hand-picked divergence proxy; the 2nd-moment area rate IS the divergence). KEEP the pinv LATERAL — it's the well-conditioned 4-corner-fused part; centroid (1st-moment) lateral is poorly calibrated (slope 0.71) and regresses; reduced-pinv lateral is well-calibrated but no better. Lag-comp gain on the loom HURTS (controller tuned around the ~0.8 filter attenuation). Worth porting the area-rate loom to `img_data.py` as a cleaner alternative to RING_LOOM.

**The validated IC1 config (28% SP @ 10cm, n=25 — parameter_record trial 46), in direct-value form:**
```
PLASMC_KP_X=1.4 PLASMC_KP_Y=1.4  PLASMC_KI_X=0.35 PLASMC_KI_Y=0.35  PLASMC_KD_X=0.5031 PLASMC_KD_Y=0.5031
PLASMC_YAW_OMEGA=0.2 PLASMC_YAW_GAMMA=0.2
PLASMC_RHOFOV0_V=315 PLASMC_RHOFOVINF_U=220 PLASMC_RHOFOVINF_V=300
PLASMC_GAMMA_Z=0.375
MARKER_KLT_MAX_STEPS=20 LANDING_STALE_COMMIT_EXTENT=100
CTRL_ZERO_WXY=1 BOARD_ALPHA0=1.23 BODY_YAW_SOURCE=alpha
```

### Outer loop (image-error → desired optic-flow rate)

> **SEN_FUNNEL coordinate geometry (2026-06-09):**
> `s_e_n = s_e[:2] / p_10` where `p_10 = center/focal = [240,320]/270 = [0.889, 1.185] rad` (half-FoV per axis).
> `r = s_e_n / p_s(t)`, `zeta_s = log((1+r)/(1-r))` — PID runs on zeta_s. Outer PID output: `ds_d = G_s^{-1}(−K_rp·zeta_s − K_ri·izeta_s − K_rd·dzeta_s) + S_s·dp_s`.
> **D-term spike mechanism:** close-range ArUco corner jump → s_e_n swings Δr≈0.57 in one 42Hz frame → dzeta_s=54 rad/s² → `K_rd × G_s_inv × dzeta_s = 28.7 rad/s` (99.7% of ds_d peak). P-term self-limits via G_s_inv; D-term does NOT self-limit (it tracks the derivative of the nonlinear transform).
> **K_rd=0 n=5 (2026-06-09): median 11.96m, 3 TL — DEAD-END.** D-term is LOAD-BEARING for drift arrest between instability events. Without it PI cannot arrest 4-21% flow underreport before 1/alt amplification causes funnel breach.
> **Outer funnel too slow (primary binding constraint 2026-06-09):** `p_s_0=1.2 ≈ FoV edge (1.185); gamma_s=0.1 → p_s(4s)=0.837`. P-term = K_rp × G_inv × zeta_s ≈ 1.8 rad/s at s_e_n=0.1 — too weak. Fix lever: **gamma_s** (faster contraction → r grows sooner → stronger P-term drive while altitude headroom remains). Env: `PLASMC_XIS_X/Y`.

| Param | Default | Env knob | Notes |
|---|---|---|---|
| `K_rp` (P gain) | `diag(9, 9)` | `PLASMC_KP_{X,Y}` (direct per-axis values; defaults 9.0) | Boost >1.5× → instability cascade |
| `K_ri` (I gain) | `diag(1, 1)` | `PLASMC_KI_{X,Y}` (direct; defaults 1.0) | 10× MATLAB's 0.1; needed for SITL drift correction |
| `K_rd` (D gain) | **`diag(0, 0)` (BAKED 2026-06-10)** | `PLASMC_KD_{X,Y}` (direct; defaults 0.0) | **K_rd=0 + gamma_s=1.0 is the sweep winner** (xy_med=1.32m; the logged "1/5 SP at 0.03m" is UNVERIFIED — frozen-GT artifact, no genuine R3 SP, see memory `feedback_false_sp_frozen_gt`). gamma_s=1.0 replaces D-term's damping role via fast funnel pressure. K_rd=0 alone dead-end (11.96m); K_rd=0+gamma_s=1.0 resolves the root cause. |
| `gamma_s` | `diag(0.1, 0.1)` | `PLASMC_XIS_{X,Y}` | **Primary binding constraint (2026-06-09).** Outer funnel contraction rate. 0.1 too slow → p_s(4s)=0.837 → P-term weak → drift not arrested before 1/alt amplification. Sweep: 0.2/0.3/0.5/1.0 (in progress). |
| `p_s_0` | `[1.2, 1.2]` | `PLASMC_PS0_{X,Y}` | Initial outer funnel half-width (normalized). Barely wider than FoV edge (1.185 on x). |
| `p_s_inf` | `[0.1, 0.1]` | `PLASMC_PSINF_{X,Y}` | Terminal outer funnel half-width. 0.1 → |s_e_n|<0.089–0.119 rad at touchdown. Fine. |
| `DH_D_MAX` | **5.0** m/s³ (baked) | `PLASMC_DH_D_MAX` | Clamp on h_d derivative. **LOAD-BEARING (2026-06-02): the clamp value feeds Θ_norm → κ-runaway; =5.0 eliminates IC1 hard impacts (n=5: rel_vel max 9.5→0.4 m/s, κ bounded, xy unchanged). IC2-5 gate PASSED 2026-06-03 (no regression; note both arms ~5-6m there — see memory multisine-cal-ic25-collapse).** |

### Middle loop (PLASMC funnel + sliding)
| Param | Default | Env knob | Notes |
|---|---|---|---|
| `gamma` (Ξ_2) | `diag(0.2, 0.2, 0.2)` | `PLASMC_XI2_{X,Y,Z}` (direct) | Sliding-variable gain |
| `p_2_0` | `[25, 25, 4]` | `PLASMC_P20_{X,Y,Z}` (direct) | Initial half-width of funnel |
| `p_2_inf` | `[2.5, 2.5, 1.5]` | `PLASMC_P2INF_{X,Y,Z}` (direct) | Terminal half-width — LOAD-BEARING |
| `Omega` (Ω) | `diag(0.05, 0.05, 0.025)` | `PLASMC_OMEGA_{X,Y,Z}` (direct) | **sliding-SURFACE slope** `σ=ζ₂+Ω·∫ζ₂` (NOT κ-leakage — that's `P`; corrected 2026-06-13). On the surface ζ₂ decays `e^(−Ωt)`. Delicate: raising it inflates σ via the integral + couples into κ̇ (Ω=0.1 was an n=5 DEAD-END, NC70-71). |
| `Gamma` (Γ) | `diag(0.4375, 0.5, 0.75)` | `PLASMC_GAMMA_{X,Y,Z}` (direct) | κ-adaptation rate |
| `E` | `diag(1, 1, 1)` | `PLASMC_E_{X,Y,Z}` (direct) | Boundary-layer thickness; sat() arg scaled by 1/E |
| `N` | `diag(0.02, 0.02, 0.02)` | `PLASMC_N_{X,Y,Z}` (direct) | Drives e-modification term in κ-ODE |
| `P` | `diag(1.5, 1.5, 2.5)` | `PLASMC_P_{X,Y,Z}` (direct) | Anti-windup-like gain on κ-ODE (P_z=2.5 baked) |
| `kappa_0` (init κ) | `[0.15625, 0.15625, 1.0]` (X/Y unchanged; Z=**1.0 BAKED 2026-06-10**) | `PLASMC_KAPPA0_{X,Y,Z}` (direct) | Z bootstrap fix: KAPPA0_Z=1.0 → descent starts in 0.4–0.9s (vs 25–43s hover with gamma_s=1.0 fast-centering). KAPPA_MAX_Z=3.0 also baked (κ_z cap). |

### Yaw SMC (low-impact per supplement S3-A)
| Param | Default | Env knob |
|---|---|---|
| `Omega_a` | 0.5 | `PLASMC_YAW_OMEGA` (direct) |
| `Gma_a` | 0.5 | `PLASMC_YAW_GAMMA` (direct) |
| `n_a` | 1.0 | `PLASMC_YAW_N` (direct) |
| `p_a` | 2.0 | `PLASMC_YAW_P` (direct) |
| `kappa_a_0` | 2.0 | `PLASMC_YAW_KAPPA0` (direct) |
| `E_a` | 3.0 | `PLASMC_YAW_E` (direct) |

### FoV-margin cone clamp
| Param | Default | Env knob | Notes |
|---|---|---|---|
| `rho_fov_0` | `[290, 210]` | `PLASMC_RHOFOV0_{U,V}` (direct, px) | Initial pixel envelope (per image axis) |
| `rho_fov_inf` | `[80, 80]` | `PLASMC_RHOFOVINF_{U,V}` (direct, px) | Terminal pixel envelope (per image axis) |
| `l_fov` | **0.0** (baked; rho_fov held constant at rho_fov_0) | `PLASMC_LFOV` (direct) | Envelope decay rate (1/s). Was 0.1 → decay to 80px fired perception-death early; set >0 to restore decay (2026-06-05). |
| `theta_cap` | 60° | `PLASMC_THETACAP_DEG` (direct, deg) | Soft cone ceiling — acceleration saturation |
| `theta_floor` | **60°** (baked default; =θ_cap → d_min collapse OFF) | `PLASMC_THETA_FLOOR_DEG` | **Floor on θ_cone (2026-06-03). The d_min collapse was strangling terminal correction (94-100% of final-2s samples at IC1) — THETACAP is irrelevant when d_min=0. floor=60 → SP #6 (xy 0.060/vel 0.149, first mechanism-driven SP). See memory fov-cone-clamp-deadlock.** |

### Inner loop (SO(3))
| Param | Default | Env knob | Notes |
|---|---|---|---|
| `K_R` | `diag(2, 2, 2)` (0.4×5 baked into base) | `PLASMC_KR_{ROLL,PITCH,YAW}` (direct; defaults 2.0) | 2.0 is the stable sweet spot. Higher → LK breakage |
| `W_U_MAX` (body-rate clamp) | 1.0 rad/s | `PLASMC_W_U_MAX` | Caps body-rate command magnitude |

### Misc / control-loop
| Param | Default | Env knob | Notes |
|---|---|---|---|
| `tau_ia` (LPF on I_a_cd) | 0.08 s | **not env-overridable** | Tested 0.04, 0.20 — both worse |
| `iV_s_e_n_clamp` | 5.0 | **not env-overridable** | Anti-windup on integral of normalized error |
| `izeta_clamp` | 5.0 | **not env-overridable** | Anti-windup on integrated sliding variable |
| `ie_a_clamp` | 2.0 | **not env-overridable** | Anti-windup on integrated yaw error |

### Image pipeline (img_data.py)
| Param | Default | Env knob | Notes |
|---|---|---|---|
| `FILTER_WIN` (savgol window) | 13 → user uses 7 | `IMG_FILTER_WIN` | |
| `FILTER_POLYORDER` | 1 | `IMG_FILTER_POLY` | |
| `STALE_THRESH` (intervention 2) | 3 frames | `IMG_STALE_THRESH` | |
| `adaptiveThreshConstant` | 5.0 (intervention 1) | `ARUCO_ADAPT_THRESH_C` | |
| `errorCorrectionRate` | 0.8 (intervention 1) | `ARUCO_ERR_CORRECT` | |
| `minMarkerPerimeterRate` | 0.02 (intervention 1) | `ARUCO_MIN_PERIM_RATE` | |
| `minOtsuStdDev` | 3.0 (intervention 1) | `ARUCO_MIN_OTSU_STD` | |
| `IMG_EXTRA_PTS` (hybrid Shi-Tomasi corners) | 0 | `IMG_EXTRA_PTS` | Set ≥50 → over-determines 8x6 lstsq with extra corners; defaults to OFF |
| `BODY_YAW_SOURCE` (SO(3) yaw source) | `'alpha'` (default since 2026-06-04) | `BODY_YAW_SOURCE` | `'alpha'` = compass-free SO(3) (uses drift-free s[3], not euler[2]); `'compass'` = legacy. See [[compass-free-yaw-sign]]. (`V_YAW_SOURCE` was removed — marker-aligning V zeroed the yaw feature; see [[v-yaw-source-alpha]].) |
| `MARKER_KLT_MAX_STEPS` (KLT fallback cap) | `20` | `MARKER_KLT_MAX_STEPS` | KLT-tracks last good corners when ArUco fails; bridges short outages. Default ON. Set to `0` to disable. See [[klt-marker-fallback]]. |

### Landing-test (landing_test.py)
| Param | Default | Env knob |
|---|---|---|
| `REF_RAD_OPT_FLOW` (h_rd) | -0.42 (MATLAB), user uses -0.70 | `LANDING_REF_RAD_OPT_FLOW` |
| `MARKER_LOSS_GRACE` | 1.0 s | `LANDING_MARKER_LOSS_GRACE` |
| `IC_POS_TOL`, `IC_VEL_TOL`, `IC_TILT_TOL_DEG`, `IC_STABLE_HITS`, `IC_BUDGET_S` | 0.5 m / 0.5 m/s / 3° / 20 / 30s | `LANDING_IC_*` |
| `PX4_RATE_SCALE` | 1.0 (DEAD — see `feedback_mc_rate_p_dead.md`) | `PLASMC_PX4_RATE_SCALE` |

## Tuning methodology (RULES — violate at your peril)

0. **Per-axis knobs ONLY (user directive 2026-06-03).** All uniform scale multipliers
   were REMOVED from controller.py. Tune `_X/_Y/_Z` (or `_ROLL/_PITCH/_YAW`, `_U/_V`)
   variants. The two image axes run at different effective gains (x is 1.39× hotter —
   see memory per-axis-tuning); uniform scaling can never fix that.

1. **n ≥ 5 reps per cell.** Single runs are noise. The "winners" from n=1 sweeps reproducibly fail to replicate at n=10 (see `feedback_sensitivity_sweep_methodology.md`).
2. **IC2-5 validation is mandatory before defaulting.** IC1 improvements consistently regress off-center starts. `run_ic_validation.sh` exists.
3. **Validate combos at n ≥ 10.** Phase 3 found a "PRECISE singleton at n=5 (YEEz combo)" that gave 0/10 at n=10 — a fluke. Stacked-singleton optimism is the most common false positive.
4. **Trust direction-of-effect, not specific xy values.** Per-rep variance is ±0.2 m. A "winner at xy_mean=0.30 vs baseline 0.48" is only believable across n≥10.
5. **Stop sweeping after diminishing returns.** Memory says ~620 SITL runs were spent gain-sweeping before Phase 1 revealed the controller works in MATLAB. Lag is the architectural ceiling.

## Levers within MAVSDK architecture (sorted by remaining headroom)

The user has ruled out uXRCE-DDS migration unless it's the only option. So the levers within MAVSDK are:

| Lever | Expected lag reduction | Cost | Status |
|---|---|---|---|
| **A. MC_*RATE_P at post-takeoff** | ~10 ms (pitch) | 1-line move of the param-set call | Untried — original attempt failed at startup-time |
| **B. Savgol WIN=7 → WIN=5** | ~12 ms (image-side) | 1 env var | WIN=5 previously caused TARGET_LOST; **interventions might absorb now** |
| **C. Smith predictor in Python outer loop** | 20-30 ms (theoretical) | Few hundred LOC | Software-only model-based compensation |
| **E. Airframe-init MC_*RATE_P** | Same ~10 ms as A | Edit `~/PX4-Autopilot/...4014_*` | Outside-repo; bypasses MAVSDK timing |
| **F. PX4 MAV_*RATE params** | Possibly 5-15 ms | env knob via MAVSDK | Untried |

Lever D (image-based velocity feedforward) is **already implemented** via the optic-flow middle loop — the SMC middle loop uses optic-flow centroid Δ directly as feedforward.

## Diagnostic procedure — "why did this rep fail"

**STANDARD PROCEDURE (2026-06-03, user methodology): failure root-causing + saturation audit.**

For every batch (not just failures):
1. `analyze_saturation_audit.py --glob '<bundle>/rep*'` — duty cycle of all 12 limits (6 code guards +
   6 manuscript limits), SP-vs-non-SP ranking. Active limits = silent performance loss even in good reps.
2. `analyze_saturation_audit.py --events <reps>` — every saturation event with when/why (auto-attributed
   reason: e.g. cone "large ask" vs "small allowance", σ/ℰ funnel-error ratio, accel-floor z-spike).
3. For each frequent event type: reason → the manuscript parameter that owns it → tune that parameter so
   the signal stays in its linear regime. NEVER respond by widening the limit itself (limits are a last
   resort, only when performance cannot improve any other way).

For a single failed rep:
1. `analyze_explosion_chain.py <rep_dir> [...]` — finds WHICH state departs its normal envelope FIRST and the resulting causal chain (ds_d → dh_d clamp → ζ saturation → κ runaway → a_u). Aggregates first-movers across reps. This is the per-term a_u attribution tool.
2. Load with `analyze_timeseries.py <rep_dir>` (per-channel correlation with xy_end)
2. Look at:
   - `|σ|_tail` ρ vs xy_end — if high (>0.7) the controller didn't reach sliding surface
   - `|s_e_n|_tail` — if high, controller didn't drive image error to zero
   - `|w_u|_max` — if > 1.5 rad/s, LK tracking probably broke
   - Frozen `Image Feature Pts` in final 100ms → marker detection breakdown (mode 2)

For a comparison across reps:
- `diagnose_intervention_reps.py <bundle>` — produces per-rep summary table + correlations
- Look at `side_max_d`, `stale_pct_descent`, `switches`, `sigma_xy0` — the strongest correlates of SP outcomes

For lag characterization:
- `analyze_loop_latency.py` — cross-correlation; noisy
- `run_impulse_response.sh` + `analyze_impulse_response.py` — cleaner, gives per-axis τ + deadtime

For sensor-noise budget:
- `analyze_sensor_noise.py` — frame-to-frame Δ methodology; absolute std is motion-contaminated

For SEN-funnel control-authority ("does s_e_n stay bounded + converge?"):
- Load a fail vs land run and compare `V_s_e_n` against `p_s` per axis, and the residency
  `S_s = s_e_n/p_s`. The prescribed-performance guarantee `|s_e_n| < p_s(t)` HOLDS iff `|S_s|` stays
  below the 0.95 `S_MARGIN` clamp and recedes; it FAILS when `S_s` PINS at ±0.95 and `s_e_n` then breaches
  (grows past `p_s`). A pinned-then-breaching axis = the outer loop lacks closing authority: at the bound
  the back-mapped demand saturates (`G_s⁻¹ ∝ p_s → 0`, the §9 demand-starvation) so the funnel gives little
  restoring demand. Distinguishes "funnel guarantee holds" (bounded, converges, 0% breach) from "authority
  deficit" (e.g. corrected `c̃_h` on Static IC5: 46% breach, `s_e_n_y → 6× p_s`). Candidate fixes: SEN hard-
  containment (`SEN_CONTAIN_MODE`/`SEN_RECOVER_ST`), wider `p_s_0`, slower `γ_s` — NOT more reaching gain.

For MATLAB-vs-PX4 comparison:
- `MATLAB/Multi_init_cond/phase1_baseline_sweep.m` (MATLAB controller alone)
- `MATLAB/Multi_init_cond/capture_sigma_trace.m` (matched h_rd σ trace)
- `PX4_Gazebo/analyze_sigma_compare.py` (compares MATLAB vs PX4 σ trajectories)

## Known dead-ends (do NOT retry without new evidence)

- `K_rp ≥ 13.5` or `K_ri ≥ 1.5` boost → instability cascade
- `K_R ≥ 0.5` without W_U_MAX clamp → LK breakage
- `tau_ia` not in [0.08, 0.20] → either lag or noise
- `LANDING_REF_RAD_OPT_FLOW < -0.7` (e.g., -0.30) → 40% TARGET_LOST
- Tight IC (vel_tol=0.05) → 33% TL rate, no xy benefit
- Stacking singletons from sensitivity sweeps → cancels benefits 90% of the time
- Sensor cal refresh via `aggregate_calibration.py` → 7-10× wrong values
- `MC_*RATE_P > 1.0` via MAVSDK runtime → SITL preflight failure (mode-1 lever still untried at post-takeoff)
- Enlarging small marker → violates the at-touchdown FoV match constraint
- `PSINF_XY=0.6` (raise outer-funnel floor) → no change (NC63, 2026-06-12); terminal s_e re-grows to ~1.5 rad ≫ any floor → zeta_1 still saturates. No outer-funnel gain fixes the terminal wall.
- `K_rd ≥ 1.0` outer D-gain (incl. MATLAB's 1.4375) and `XI2=0.2` (MATLAB canonical) at off-center IC → lazy middle funnel + close-range D-spike; MATLAB-canonical gain set TLs at IC=(2,2,5) (NC64). The baked XI2=0.6/KD=0.5 already supersede the MATLAB values for SITL.
- `OMEGA_XY=0.1` (2× κ-leakage) → DEAD-END (NC70-71, 2026-06-12). n=1 looked best-ever (calm terminal theta 11.8/a_u 10.1) but n=5 made IC2 0→1 TL (+24.7 m) and IC5 3→4 TL — faster leakage bleeds κ too low → SMC loses authority → under-correction/drift. Cousin of the `N_XY=0.05` dead-end: perturbing the κ-dynamics rate (N↑ noisy, Ω↑ weak) without σ-signal support degrades. No κ-rate knob fixes the terminal wall.
- **`PLASMC_SOFT_BREACH` at every frac (2026-07-01)** → GT-FB IC4/IC5 n=5: OFF 8/10 vs frac 0.05/0.15/0.30/0.50 = 4/9·7/10·7/10·6/10; κ_xy RISES as frac shrinks (0.62→1.07) — the flow arm shares the if/else gate with the PROTECTIVE κ-freeze, so enabling it un-freezes κ on a persistent GENUINE breach (un-arrested 1/Z residual velocity; position already converged, S_r≈0.04). Symptom-side breach handling can't fix a cause-side velocity. Keep default-off. [[project_soft_breach_idea1]]
- **Corrected manuscript c-term (`C_SIMPLE`, simple `c̃_h = −ψ̇_b(ê₃×h) − (h·ê₃)h`) — IC5 deficit NOT
  gain-recoverable** (2026-06-16). No-regression on the Circular IC1-5 gate (30/30) but ~1 seed worse on
  IC5 `[2,2,-3]` across ALL trajectories (loses Static IC5 that old-c lands). Reaching gain `Γ_xy` {1.5,3.0}
  makes it WORSE on both Static+Circular IC5; `K_rd` doesn't fix it (fails at 1.44 AND 2.5). Root: old-c's
  *incorrect* full-`w` terms create a **productive sustained flow error** (`|V_h_e|`~1.0-1.7) that keeps
  driving lateral closure; the leaner-correct form tracks cleanly (`|V_h_e|`~0.5-0.9) but UNDER-closes →
  `s_e_n` breaches the funnel. Untested recovery: SEN hard-containment (`SEN_CONTAIN_MODE`/`SEN_RECOVER_ST`).
  See memory `project_chtilde_correction_option_b`.

## The precision-softness frontier (2026-05-24, mapped via 5 N=10 sweeps)

The architecture's lag forces a frontier — knobs move along it, not off it. Demonstrated empirically:

```
                          PREC  SOFT  SP    xy_min   vh_end_mean
Interventions only         1     2    1     0.039    1.2     ← still best
RHOFOVINF × 0.5            0     4    0     0.110    0.80
RHOFOVINF × 0.7            1     2    0     0.077    0.99
THETACAP × 1.5             0     1    0     0.099    0.94 (+ TL)
Stack ×0.5 + KP×1.25       1     2    0     0.031    0.88
─────────────────────────────────────────────────────────
Combined SP rate           1 / 50 = 2%
```

**The PRECISE-only failure mode** (across 19 PRECISE-only reps out of 1205): every rep has |vh_end| ≫ |vz_end|. The drone lands precisely on target but is still sliding laterally at 1+ m/s. The architecture can HOLD zero lateral velocity but cannot actively BRAKE significant lateral velocity in the final phase.

When the user proposes tuning the terminal-phase parameters (`RHOFOVINF`, `THETACAP`, `KP/KI/KD` boosts, or their combos), cite this finding. The frontier is mapped; further single-knob terminal-phase sweeps will land on a point already characterized.

## Known winners (current defaults)

```
PLASMC_KR_SCALE=0.4         (baked into controller.py:221)
PLASMC_W_U_MAX=1.0          (default in controller.py:766)
LANDING_REF_RAD_OPT_FLOW=-0.70  (env-only — regresses IC2-5)
IMG_FILTER_WIN=7            (env-only — best of [5, 7, 13] for runtime delay)
ARUCO_*                     (interventions, env-overridable, sane defaults baked in)
IMG_STALE_THRESH=3          (intervention 2)
```

Best-known IC1 N=10 outcome:  PRECISE=1, SOFT=2, **SOFT+PRECISE=1**, TL=0, xy_mean=0.328 m, std=0.157 m.

**2026-06-03 (multisine-cal era): `PLASMC_THETA_FLOOR_DEG=60` + `PLASMC_DH_D_MAX=5` (default) → SP #6
(xy=0.060, vel=0.149) at IC1, 1/5 rate, 0 TL. The current best config. Neither knob is sufficient alone;
PID_SCALE=0.54 stacking adds nothing. Open: raise SP rate (terminal softness) + IC2-5 overshoot prevention.**

## Procedure for new tuning

1. **Identify the failure mode** from the 7-mode table. If mode 1 (lag) → not tuneable; consider lever A/B/C/E/F.
2. **Pick ONE parameter** to vary (no combinations until singletons validated).
3. **Sweep at 3-5 values** spanning [0.5×, 2.0×] of default.
4. **Run n=5 per cell** at IC1.
5. **Save bundle to `~/Soft-Precise-Landing/PX4_Gazebo/test_data/<sweep_name>/<timestamp>/`** for the analyzer.
6. **Reject any "winner" without n=10 follow-up** — first run only narrows the search.
7. **Run IC2-5 validation** before any default change (`run_ic_validation.sh`).
8. **Update memory** if a real winner appears; update this skill's "known winners" if it's baked into defaults.

## Anti-patterns observed in past sessions

- Running n=1 sweeps and treating them as data (Big Sensitivity Sweep, 84 cells × n=1 → mostly noise)
- Stacking "promising" singletons without n=10 (YEEz combo)
- Refreshing sensor cal (`aggregate_calibration.py` is broken)
- Tuning gains to fix a lag problem
- Tightening IC to fix a lag problem
- Suggesting the torque/thrust refactor (rejected per `feedback_thrust_torque.md`)

## Files

- `src/controller.py` — all PLASMC gains, env knobs
- `src/img_data.py` — image pipeline, ArUco params, savgol, stale-feature detection
- `apps/landing_test.py` — IC convergence, marker-loss grace, control loop, autosave
- `src/flight_controller.py` — MAVSDK wrapper, telemetry rates, rate-gain hook (DEAD)
- `tools/analyze_saturation_audit.py` — duty cycle + per-event attribution of all 12 limits (2026-06-03; the standard batch diagnostic)
- `tools/analyze_explosion_chain.py` — first-exploding-state + causal-chain diagnosis (2026-06-02)
- `tools/analyze_baked_validation.py` — per-rep validity gate (flight_s/empty-GT/final-alt) + outer-funnel residency r=s_e_n/p_s, aggregated over valid reps (2026-06-12; pairs with the now-logged `p_s(t)`/`dp_s(t)`)
- `tools/analyze_timeseries.py`, `tools/analyze_loop_latency.py`, `tools/analyze_sensor_noise.py`,
  `tools/analyze_marker_switch.py`, `tools/analyze_sigma_compare.py`,
  `tools/diagnose_intervention_reps.py`, `tools/analyze_impulse_response.py` — diagnostics

## Campaign history (read before re-deriving anything)

- `PX4_Gazebo/test_data/Landing_Test/parameter_record.ods` — the raw quantitative log. 4 sheets:
  `PX4_Gain_Record` (old eras, trials `G1–G60`), `PX4_NewCal_Record` (current honest-cal era, `NC1–NC49`),
  `MATLAB_Test_Record` (delay-robustness sweep), `Removed_Parameters` (every knob tried & removed, with why).
- **Memory `reference-tuning-trajectory`** (`reference_tuning_trajectory.md`) — the *connected* timeline: what each
  trial varied, the hypothesis, and the outcome, grouped by **cal-regime epoch** (R0 broken / R1-2 multisine / R3
  honest). Read it before proposing any sweep — most "new" ideas have a trial number already. It links out to the
  ~50 per-decision `feedback_*`/`project_*` memories. **Cardinal rule it encodes: a result is only valid within its
  cal regime** — the famous "28% SP @10cm" (G46) is a *multisine-cal hypothesis*, not an honest-cal result.
