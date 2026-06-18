---
name: feedback_newcal_tuning_results
description: "New-cal control-tuning results (2026-06-07): the boundary layer E BOUNDS the kappa-runaway at full authority (E=2.5 -> kappa~k0 + lateral held 0.04m, CBF never triggers, but too soft -> hover). cbf2 was MASKING the kappa-runaway, not fixing it. Updates the 'lateral drift = pure lag floor' conclusion: it was largely kappa-runaway + CBF-masking."
metadata: 
  node_type: memory
  type: feedback
  originSessionId: dd8920aa-4635-40a9-ac9d-8409e37243d6
---

First real controller-tuning results under the honest cal + frozen perception (stacked config: FLOW_FUSE_RING=1, SEN_FUNNEL=1, cbf2, P=3/3/5, KP=12, h_rd=-0.42). Required infra to even fly: see [[feedback_sitl_reliability_fixes]]. Logged in `parameter_record.ods` sheet **PX4_NewCal_Record** (trials 1-10; old PX4_Gain_Record 1-59 is the broken-cal regime, kept for history only).

**THE key finding — the boundary layer `E` bounds the kappa-runaway.** The kappa adaptive-gain ODE only grows when `|sigma| > E` (`dk/dt = theta*N*G*|sigma| - N*P*k`). At E=1, any transient lateral/descent correction pushes sigma past the boundary -> kappa winds up -> a_u explodes (a_u_z up to 16705, kappa_z up to 96). **`PLASMC_E_{X,Y,Z}=2.5` (CBF relaxed, floor=60): kappa NEVER ran (kappa_end≈k0 [0.25,0.18,0.34]), zeta never saturated, a_u peaks 6/6/1, and the drone held lateral DEAD-CENTER (GT 0.04m, s_e_n 0.04).** So the lateral/boundedness problem is *solved at the control level*. BUT E=2.5 softens the **vertical** SMC too -> no descent -> hover (xy=0/vel=0, watchdog). **Open: decouple — keep E_X=E_Y high (lateral bounded) but lower E_Z to restore descent without reigniting z-kappa. Next: E_Z sweep (~1.5).**

**cbf2 was MASKING the kappa-runaway, not curing it.** cbf2+THETA_FLOOR=20 clamps a_xy 51-84% of the flight (theta_cone pulled to 0-41deg vs 60 cap) — this CONTAINED the a_u/kappa explosion but also strangled lateral authority. Relaxing it (THETA_FLOOR=60) EXPOSED the runaway (it was always there). **Decision (user): the CBF is a SAFETY NET, not a controller — keep it relaxed and bound kappa via control gains (E). If the CBF is triggering in normal ops, the control law is failing.**

**This UPDATES [[feedback_precision_softness_frontier]] / "lateral drift = pure lag floor":** the lateral drift was largely the kappa-runaway + the CBF clamp removing authority, NOT purely architectural lag. E=2.5 holds lateral perfectly -> the lateral servo CAN converge once kappa is bounded. (Lag may still cap the *terminal* phase, but it's not the whole story.)

**Trial progression (PX4_NewCal_Record):** P_Z=5 (MATLAB parity) tames kappa_z 39->6.5 (kappa_eq~1/P); P_X/Y=3 bounds lateral kappa (a_u 20x lower); KP 9->12 looked best at n=1 (1.5m) but **n=5 DEBUNKED it (mean 6.1m, 0 SP)** — n=1 is noise. Dead-ends: **KP>=13** overshoots; **KI>=2** integral windup -> kappa-runaway returns; **W_U_MAX>1.7** breaks LK corner tracking (TARGET_LOST); **THETA_FLOOR low** just masks kappa.

**Per-rep root-cause tool: `tools/diagnose_failure_cause.py <rep>`.** Discriminates PERCEPTION vs CONTROL by contrasting image `s_e_n` against Gazebo-truth lateral offset at the divergence onset (image-err large + GT-lat small = centroid wrong/perception; GT-lat large = real physical drift/control). Across the campaign: failures are CONTROL-led lateral physical drift (NOT perception, NOT — after E — kappa).

**Standard run prefix (infra, not gains):** renice VSCode BEFORE launch: `renice 15 $(pgrep -x code)`. Then use `run_aruco_landing_retry.sh` (handles ic-timeout/lockstep retries). `taskset -c 6-15` on the outer shell does NOT reliably pin PX4 subprocess — renicing is more effective. SITL is shared with another chat — one at a time, kill only your own PIDs.

**2026-06-08 UPDATES:**
- SEN_FUNNEL=1 now code default (was silently OFF for all runs in this session — fix commit f812895).
- LANDING_AUTOSAVE=1 now in run script (fix commit 2e125a9).
- IC_YAW_TOL widened to 5° (EKF drifts 3-4° from lockstep starvation; servo genuinely converges alpha→0 with DMAX=0.3).
- **NEW BINDING FAILURE:** LK dynamic-range collapse at t=0. KP=12 + 4% centroid offset → ds_d=0.49 rad/s → v_req=2.45m/s at Z=5m → LK collapses → crash/TARGET_LOST. Two confirmed reps. SEN_FUNNEL is ON but marker dies in 3 frames before funnel gates. Fix: revert KP→9 first, validate clean engagement, then re-sweep upward.

**2026-06-09 UPDATES — deep sweep campaign results (42 rows in PX4_NewCal_Record):**

**Best confirmed stack (row 31):** `KP=9, E=[2.5,2.5,0.5], P=[5,5,5], KI=1.0` → median **3.80m**, min 1.95m (n=5, 2 TL). This is the current performance ceiling under gain tuning.

**KP×E coupling (new finding — [[feedback_kp_e_coupling]]):** KP and E_XY must be matched. KP=12 with E=1.5 works (median 3.20m at n=3, row 34, SEN_FUNNEL gating t=0 LK spike). KP=12 with E=2.5 is WORSE (4.79m, row 42) — wide E keeps σ<E → κ near κ_0 → KP=12 drives h_d windup faster → earlier funnel breach. KP=9 with E=2.5 is optimal (3.80m). **Rule: KP=9 with wide E; KP=12 only with stiff E.**

**P_XY=3 DEAD-END confirmed (row 33):** Other-chat baseline with P=[3,3,5] gives median 9.75m (n=6) vs P=[5,5,5] at 3.20m. P controls κ_eq ∝ 1/P — P=3 allows more κ accumulation under E=1.5. P_XY=5 is the correct value.

**Hover-at-center events (NOT SP):** 3 events across campaign where xy=0.0, vel=0.0, flight>50s — drone hovers dead-center but never lands. All correlate with conditions that keep σ<E throughout (wide E or low σ config). These are boundary-layer-induced hover failures, not SP. classified as FAIL by landing_test.

**Dead-ends confirmed this session (rows 30-42):** KI=0.35 (4/5 TL), E_Z=1.5 (7.86m, wrong direction), KP=12+E=2.5 (4.79m worse), N_Z=0.05 alone (6.12m), tau_ua=0.3 (11.06m), P_XY=3 (9.75m).

**Parameters confirmed at optimal defaults:** Gamma_Z=0.75, KR_YAW=2.0, P2INF_Z=1.5, TERMINAL_HOLD_EXTENT=70, YAW_OMEGA=0.5, PSID_RATE=1.0.

**Active binding failure (2026-06-09):** Stochastic LK/ArUco collapses give 1-2 TL per 5 reps even with perfect IC (pos=0.04m, vel=0.009m/s → xy=10.8m TL). This is the LK dynamic-range ceiling (~2 m/s), not gain-tunable. The gap between xy_min (~2m) and xy_med (~4-6m) is entirely stochastic perception failure.

**N_XY=0.05 DEAD-END (row 43):** median 5.87m vs 3.80m baseline. Faster κ adaptation (τ_κ 10s→4s) without enough σ signal (E=2.5 keeps σ<E) = noisy/worse. N_XY=0.02 is optimal.

**ALL GAIN-SIDE LEVERS EXHAUSTED.** Next lever must be code-level: **pyramidal LK levels 2→3** in `img_data.py` — extends LK dynamic range ceiling (~2 m/s → ~4 m/s). This directly addresses the binding stochastic TL failure mode.
