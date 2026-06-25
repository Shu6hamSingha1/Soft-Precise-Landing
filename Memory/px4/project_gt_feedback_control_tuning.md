---
name: project_gt_feedback_control_tuning
description: "⭐ NEW TASK (2026-06-23, fresh chat): GT-FEEDBACK CONTROL TUNING. Feed V-frame GT s (centroid) + h (flow) to the controller as feedback (bypass the noisy/saturated perception), tune the control GAINS to drive s_e_n→0 AND h_e→0 on CLEAN signals — this ISOLATES the control problem from the perception ceiling. THEN re-introduce real perception. Rationale: this session PROVED neither s_e_n nor h_e converges to 0 (s_e_n plateaus ~0.4 = Task-1 fail; h_e bounded ~0.4 + grows terminally because the terminal optical flow SATURATES — LK dynamic-range ceiling, ALL estimators corner/ring/fused under-report the deck loom 5-10x, a SCALE-CHANGE not a displacement-range limit so LK params are inert). With perfect GT s/h we find out if the CONTROL LAW can converge at all. Compute GT s/h via tools/gt_optical_flow.py (V-frame); inject via a new env knob in img_data.py/controller.py; GT target pose comes from gz_subscriber. The fed s/h are TRUE IMAGE-SPACE features (scale-free) so the control law stays scale-free — only the sensor is idealized for tuning."
metadata:
  node_type: memory
  type: project
  originSessionId: eba9fa95-5b93-4294-bbca-81468bb36670
---

**THE TASK (new chat).** Replace the perception-derived feedback (centroid `s` and optical flow `h`)
with **V-frame GROUND-TRUTH `s` and `h`** computed from the Gazebo GT poses, feed them to the PLASMC
controller, and **tune the control gains** so that **`s_e_n → 0` (Task-1 precision) AND `h_e → 0`
(Task-2 softness)** on these CLEAN signals. This decouples the CONTROL problem from the PERCEPTION
problem. Once the control law converges on GT feedback, RE-INTRODUCE real perception and tackle the
perception noise/saturation separately.

**WHY (what this session established — carry forward):**
- **Neither task error converges to 0.** Good IC2 reps: `s_e_n` plateaus ~0.3-0.5 (lands AT the residual
  ~0.4m, never centers) ; `h_e` bounded ~0.3-0.5 and GROWS terminally. Catastrophic reps: `s_e_n`
  BREACHES (1.11 mid → 2.30 deck) = the fly-away.
- **The terminal optical flow SATURATES** (the binding perception limit): at the deck GT loom −11..−15
  but EVERY estimator reads ≤~3 (corner=0 below 1m, ring/fused 5-10x under). It's a **scale-change**
  limit (the marker EXPANDS ~19%/frame at the deck → violates LK brightness-constancy/translation) NOT
  a displacement-range limit → **LK params (winSize/maxLevel) are INERT** ([[feedback_pyramidal_lk_inert]]),
  and so is the centroid-rate observer (decode-jitter noise, [[feedback_single_marker_rank_deficiency]]).
- So the terminal is effectively **open-loop** (flow blind) → stochastic land-vs-launch. GT-feedback
  removes this: GT `h` is exact at all altitudes → tests whether the CONTROL can then converge.

**HOW TO COMPUTE V-FRAME GT s AND h.** The reference impl is `tools/gt_optical_flow.py::compute_gt_flow`
(OFFLINE): returns `V_h_g` (V-frame GT flow [h_x,h_y,h_z]) + `W_x_tu` (target−UAV NED) + the V-frame
machinery. For CLOSED-LOOP you must compute these ONLINE from the runtime GT poses:
- **GT centroid `s`** = V-frame target BEARING: `B_x = Ru.T @ W_x_tu` (NED→body); `V_x = _v_frame(Ru) @ B_x`
  (body→V leveled); `s = [V_x[0]/V_x[2], V_x[1]/V_x[2]]` (+ yaw feature `alpha` = GT relative yaw).
- **GT flow `h`** = V-frame relative velocity / depth: `V_v = _v_frame(Ru)@Ru.T@W_v_tu`; `h = V_v/zB`
  (zB = rel altitude). loom `h_z = V_h_g[2] = vz/Z`.
- The V-frame MUST match `_getVirtualPts` exactly (gravity-leveled `rotz(yaw)`; z→world-down via
  `R.T@[0,0,1]`). Frame conventions: [[reference_frame_conventions]], [[feedback_vframe_rhs_yaw_only]],
  `reference_gt_optical_flow`. NOTE the measured-vs-GT ~−14° V-frame yaw offset (all arms) — with GT
  feedback this disappears (we use GT yaw directly).

**INJECTION POINT.** The controller (`src/controller.py`) consumes `s` from `img_data.getImgFeatureParam()`
and `h` from `img_data.getOptFlowAngVel()`. Add an env knob (e.g. `PLASMC_GT_FEEDBACK=1`) that, when on,
substitutes the GT-computed `s,h` (and `alpha`, loom) for the perception values. The GT TARGET pose (the
ArUco pad) + UAV pose come from `src/gz_subscriber.py` (`/pose` ROS2 sub — the same GT used for eval).
Marker WORLD is then largely irrelevant (perception bypassed) — but keep detection alive for the FoV/
visibility gating if the controller uses it; or gate GT feedback on "target in FoV" to stay realistic.

**SCALE-FREE NOTE.** `s` (bearing) and `h` (flow rate) are IMAGE-SPACE, scale-free quantities. Feeding
GT `s,h` keeps the control law scale-free/depth-free ([[feedback_scale_free_depth_free]]) — only the
SENSOR is idealized. Depth Z enters the COMPUTATION of GT s/h (projection) but NOT the fed values or the
control law. This is a legitimate tuning scaffold, not a depth-in-the-loop violation. The FINAL deployed
controller still runs on perception.

**CURRENT CONFIG / STATE.** Single-marker BAKED-but-PROVISIONAL (PLASMC_SINGLE_MARKER=1 +
FLOW_LOOM_DECOUPLE=1, 1m ArUco world, matched cal in img_data.py; FAILED IC2-5 gate, NOT committed —
[[feedback_single_marker_rank_deficiency]]). For GT-feedback the marker choice barely matters (perception
bypassed) — could even revert to the multi-marker board. Test rig: `scripts/run_aruco_landing*.sh`,
IC2=(2,2,5) via INITIAL_DRONE_ENU, headless taskset 6-15, n=5 (user: n=5 enough), analyze via
`tools/analyze_baked_validation.analyze_rep`. The combined sliding surface (manuscript /
[[project_stacked_barrier_backstepping]]) is the STRUCTURAL fix for s_e_n→0 — likely the gain-tuning
target once GT feedback removes the perception confound.

**KEY FILES/TOOLS:** `tools/gt_optical_flow.py` (GT s/h ref impl), `src/controller.py` (control law +
s/h consumption), `src/img_data.py` (perception s/h — injection point), `src/gz_subscriber.py` (runtime
GT poses), `tools/analyze_baked_validation.py` (eval). Memories: [[reference_frame_conventions]],
[[reference_gt_optical_flow]], [[feedback_vframe_rhs_yaw_only]], [[feedback_plasmc_two_task_framework]]
(s_e<p1 Task1 / h_e<p2 Task2), [[feedback_scale_free_depth_free]], [[feedback_inner_loop_velocity_thread]]
(the saturating-sensor ceiling), [[project_current_state]].

**FIRST STEPS for the new chat:** (1) port the V-frame GT s/h computation into the runtime (from
gt_optical_flow.py + gz_subscriber GT), behind `PLASMC_GT_FEEDBACK`; (2) verify the GT-fed s/h match the
offline compute_gt_flow on a recorded rep (sanity); (3) run IC2 with GT feedback — do s_e_n and h_e now
converge to 0? (4) if not, the CONTROL law is the limit (tune gains / combined surface); if yes, the
perception ceiling was the limit → then re-introduce perception + attack the saturation.

---
**PROGRESS (2026-06-23 session) — PORTED + RUNNING + per-axis tuning underway.**

DONE: `src/gt_feedback.py` (online V-frame GT s/h: `s=[V_s_x,V_s_y,1,alpha]`, `flow=[h;w]`, vel = causal
LS-slope over `PLASMC_GT_FB_WIN` samples, default 7); wired behind `PLASMC_GT_FEEDBACK=1` in controller.py
(override after the getters) + landing_test.py (pass `pose_node`). Gate-bypass: when GT-FB on, the control
loop + `feature_fresh` ignore perception visibility so it runs to touchdown on GT. Tools: `validate_gt_feedback.py`
(offline frame check — IDENTITY map, GT vs perception s corr 0.83/0.88, h 0.89/0.95, slope<1 = the
under-report GT removes), `analyze_gt_tuning.py`, `gt_per_axis.py`, `gt_cycle_probe.py`, `log_param_record.py`
(appends to parameter_record.ods :: PX4_NewCal_Record). GT-FB runs logged NC110-120.

KEY RESULTS (all GT-FB, IC2=(2,2,5) unless noted):
- **Control law DOES converge on clean GT** (perception ceiling WAS the limit): IC2 s_e_n converges the offset
  cleanly alt 5→2m, lateral h_e small. So the task's hypothesis is answered — control works at altitude.
- **Then a per-axis LIMIT CYCLE + 1/Z terminal blow-up.** Lateral: under-damped mode, loop gain ∝1/Z
  (G_xx rises ~10x during descent), σ_xy trapped INSIDE the boundary layer (|σ|/E<0.6) → switching
  linearized away → κ_xy FROZEN at κ0 → no damping → cycle grows → terminal blow. The terminal touchdown
  error is an EXP-growing oscillation sampled at a quasi-random impact instant → n=1 is HEAVY-TAILED/non-
  monotonic ("the race"); judge by per-axis loop behaviour, not touchdown xy.
- **Lateral-only levers FAILED** (consistent with the coupling): N_xy=0.1 (terminal κ explosion, uncapped
  κ_max_xy=1e6), E_xy=0.3+κmax3 (engaged switching but mixed/noisy), E_xy=0.5 (catastrophic), descent-gate
  (2/3 better vel but 1/3 still 26m), chi_r=0.3 (uniformly WORSE). A single-axis gain only reshapes its own
  loop; the cycle is COUPLED.
- **USER METHODOLOGY (load-bearing, 2026-06-23): eliminate the limit cycle PER-AXIS in coupling order
  YAW → ALTITUDE → LATERAL.** Yaw sets the image frame the other two live in; a param tuned for one axis
  only relaxes that axis and can destabilize others via coupling, so fix upstream first. Judge each axis by
  its OWN loop, not the coupled touchdown error.
- **YAW (done): cycle CONFIRMED as the master driver** — isolated test = IC1 centered + `DES_ALPHA_DEG=25`
  (lateral quiet, excite yaw only): yaw overshoots +40° and grows; lateral stays <0.06 until the yaw cycle
  grows then COUPLES through the image frame and blows lateral to 20-40 (xy fly-away). **ROOT = double
  integrator:** `u_a` is a yaw-RATE cmd (`psi_d=∫u_a`) so the rate structure already integrates e_a; the
  `Ω_a·ie_a` term adds a 2nd integrator → no phase margin vs PX4 inner-loop lag (K_R_YAW + rate loop +
  tau_ua LPF, absent in MATLAB) → growing cycle. **FIX = `PLASMC_YAW_OMEGA` 0.5→small (0 cleanest: single
  damped overshoot 14°, no growth; 0.1 keeps minimal integral, 16°).** A PX4-lag divergence class (cf
  chi_r=0.5, N_z=0.1). RULED OUT: `K_R_YAW`↑ (0.5→1.5 WORSE — not inner lag), `E_a`↓ (3→0.5, switching
  engages but no help — sigma_a carries the windup). Residual: slow yaw null (Γ_a=0.5 weak) — candidate Γ_a↑.
- NEXT: lock Ω_a≈0.1 for yaw; move to ALTITUDE (z) axis cycle (note: z high-freq in logs is PARTLY the
  online loom estimator — WIN3 noisier than WIN7; improve GT loom vel estimate before tuning z), THEN lateral.

PER-AXIS CAMPAIGN (yaw→z→lateral), continued:
- **YAW ✅ BAKED: `PLASMC_YAW_OMEGA` 0.5→0.1 (controller.py:300).** Validated across DES_ALPHA 0/10/30/45
  at IC1 (GT-FB): growing cycle eliminated (ncross 5-6→2, single bounded overshoot), all land <1.4m, no
  fly-aways (vs Ω_a=0.5 flew 3.8-13m). Residual = single overshoot scaling w/ yaw + slow convergence
  (Γ_a=0.5 weak); structural under-damping (loop is P+I, no rate term) — tighten later via Γ_a↑ or a w_z
  derivative term (code), NOT E_a (E_a↓ made overshoot WORSE — more switching authority = more aggressive).
- **GT ALPHA VALIDATED + alpha is NOT mis-calibrated.** Time-aligned, perception moment-alpha tracks true
  rel-yaw within ~5° the whole descent (alt 5→0.24m); corruption is TERMINAL marker-fill (180° ambiguity
  flip at alt<0.2m), geometric not cal (cal_s[3]=1.0 fine). Earlier weak corr was from including terminal
  garbage. GT alpha sign confirmed (loop drives e_a→0; slope + in 3/3 fits).
- **Z ✅ HEALTHY (no dedicated z cycle).** Improved the online GT loom estimator: `gt_feedback.py` velocity
  = TIME-windowed LS slope (`PLASMC_GT_FB_TAU`=0.12, was fixed-count WIN). Fed h_z vs clean GT loom corr
  +0.84, ncross 32 (~ideal 24, was 116@WIN3) → the earlier z high-freq was MY estimator noise. Isolated z
  (IC1, yaw-fixed): h_e_z converges, mid-descent monotonic, descent tracks h_rd; only event = terminal 1/Z
  loom spike (alt<0.3, brief). The sub-1.3m z oscillation = terminal 1/Z + coupling from the unfixed lateral
  mode, NOT a z instability. **With yaw+loom fixed, IC1 landed xy 0.296 vel 0.332 (best GT-FB yet, near
  soft-precise) — validates the yaw→z→lateral order.**
- **LATERAL ✅ MID-DESCENT CYCLE FIXED (K_R roll/pitch 1.5→2.5, env-tested NOT baked):** ROOT (per-axis,
  IC2 GT-FB) = the combined surface σ_xy=ζ_h+χ_r·ζ_r has ζ_h≈0 because h_d (measured-rate FF) CANCELS ~56%
  of the measured flow h (corr(h,h_d)=0.88, |h_e|/|h|≈0.44) → surface is position-dominated → a_u∝−position
  with NO velocity damping → undamped oscillator; the inner-loop attitude lag (eR_pitch −22° vs cmd 33°) +
  1/Z gain growth → growing cycle. FIX = K_R rp 1.5→2.5 cuts eR lag −22°→±5° → restores effective damping →
  mid-descent lateral cycle ELIMINATED (s_e_n held 0.05–0.14, osc≤0.1, ncross 4 over alt 4→1.5m). OPPOSITE
  of yaw (K_R_YAW↑ was worse): lateral outer is under-damped-not-unstable, so reducing inner lag helps.
  Matches MATLAB kR-roll=2.5 (SHARP/fragile optimum). Outcomes: IC4 RESCUED 36m→0.556m, IC5 7.1→3.05, IC2
  0.25/8.9 (stochastic). Records NC125-126.
- **PER-AXIS LIMIT-CYCLE CAMPAIGN COMPLETE for the MID-DESCENT (yaw→alt→lateral):** validated alt 4→1.5m,
  K_R=2.5 GT-FB, IC2x3+IC4+IC5 — YAW e_a ≤10° ncross2 (Ω_a=0.1); ALT h_e_z rms 0.13-0.15 (jitter not cycle,
  loom-estimator fix); LAT s_e_n 0.05-0.14 (K_R=2.5). ALL three cycles removed.
- **BINDING RESIDUAL = TERMINAL 1/Z (deferred per user).** Every rep stays clean to ~0.8m then the 1/Z loom +
  lateral amplification breaks it (term_onset 0.80-0.90m; IC5 1.68m since it starts at only 3m). This is what
  still causes the stochastic terminal fly-aways (IC2#2 8.9m) — NOT a mid-descent cycle. The deferred
  terminal-loom work (size-normalized moment loom / terminal handling) is the final piece. Records NC110-126.
- BAKED so far this campaign: Ω_a=0.1 (yaw). PENDING bake: K_R rp=2.5 (lateral — confirm sharp value first),
  loom-estimator TAU=0.12 (GT-FB scaffold). Uncommitted in working tree.

**TERMINAL 1/Z (the binding residual after yaw+z+lateral MID-DESCENT cycles fixed):**
- DEFINITION (in GT mode it is PURE CONTROL, NOT perception/observability — features are EXACT; "observability"
  was a mislabel I corrected): lateral image features are bearing `s_e_n∝lat/Z` and flow `h∝v/Z`. As Z→0 any
  residual metric lateral offset/velocity the control hasn't nulled is amplified WITHOUT BOUND in feature space
  → loop gain (accel per metric offset) ∝1/Z → over-stiff near deck → over-reacts to a metrically-fine residual
  → overshoot → marker leaves FoV → fly-away. Scale-free dynamics view: `ṡ_e_n = h_lat − s_e_n·h_z` carries a
  destabilizing `+|h_rd|·s_e_n` (descent-loom positive feedback on the bearing). NOT a geometric wall — a
  control-limit amplified by 1/Z; fixable IF control nulls the residual through the last ~0.8m WITHOUT using Z.
- a_u EXPLOSION MECHANISM (decomposed — CORRECTS the earlier/MATLAB "c-term" claim for PX4-GT): the terminal a_u
  blow-up is the SWITCHING term `θ·sat(σ/E)·κ`, NOT the c-term (c-term contribution ≤9 vs switching 200-1600).
  `θ=‖Theta‖_F` is a SHARED SCALAR across all 3 axes → the lateral (or z-loom) terminal barrier blow-up inflates
  θ (2→920) → detonates the switching term on EVERY axis incl z (so the z terminal explosion is largely
  COLLATERAL via the shared θ). κ at cap 3.0, sat=±1. (Reconstruct the Theta vector: `v_i = Γ_i·σ_i/G_i +
  θ·sat_i·κ_i − a_u_i`; θ=√(‖v‖²+3). Decompose v per-axis to attribute θ — tooling ready, run pending.)
- FUNNEL SHRINK = SELF-DEFEATING (the "tighten p_2 for boundedness" lever, RULED OUT WITH MECHANISM): shrinking
  p_2_z (2.81→0.53) inflates the barrier `ζ_h=log((1+r)/(1−r))`, r=h_e/p_2 (ζ 0.45→2.15, ~5×) → σ inflates
  (0.44→2.16) → `σ/E` 0.88→4.32 (FAR outside boundary layer) → `sat()` SATURATES ±1 (11%→96% of frames) → the
  boundary layer E (which exists to replace discontinuous sign() with smooth sat() = PREVENT chatter) is
  DEFEATED → SMC reverts to bang-bang sign() → CHATTERING limit cycle (a_u thrashes ±285-500, breaches EARLIER
  alt 1.04 vs 0.73). The funnel tightness and E are COUPLED via σ∝ζ∝~1/p_2 — CANNOT shrink the funnel without
  proportionally widening E. Tested XI2_z=0.6+P2INF_z=0.5: IC2 8.6/IC4 1.6/IC5 7.6 (worse than baseline).
- Ω_z (chi_z) NOT verified optimal in PX4 (I extrapolated from MATLAB chi_z=0.1; UNTESTED in PX4/GT) — open lever.
- LOOM CLAMP (`PLASMC_LOOM_CLAMP`, added then REVERTED per user): clamping the loom h_z fed to the SMC WORKED
  empirically (IC2 soft 0.25/0.52, 0.56/0.52; IC4 0.244) — it removes the 1/Z from the FEEDBACK so the barrier
  never blows up — but rejected as a band-aid masking the altitude-control failure. Reverted (not in tree).
- PROPOSED principled lever (UNTESTED): h_rd REFERENCE GOVERNOR — ease h_rd→0 near touchdown (scale-free
  proximity, e.g. MARKER_EXTENT_PX) via the existing descent-gate hook (`h_ref_eff=h_ref·g`, LPF on g, gate on
  PROXIMITY not |s_e_n|). Commands a feasible gentle settle → real loom stays bounded by construction → no θ
  explosion. Honest, not a measurement clamp. CAVEAT (memory): don't ease too early — gentler h_rd = more time
  in the 1/Z danger zone = LESS robust; full h_rd at altitude, ease only the final approach.

**LYAPUNOV PROOF CHECK (control_formulation.tex Thm 1) — the terminal 1/Z is OUTSIDE the stability guarantee:**
- The VDF-ASMC GUUB proof: V=½σᵀσ+(β_min/2)[κ−d̃·1]ᵀN⁻¹[κ−d̃·1]; V̇≤−φ₁V+(3/2)β_min λmax(P)d̃² → UUB radius ∝ d̃
  (the disturbance bound). κ-adaptation (κ̇=‖θ‖N·G|σ|−N·P·κ) is PROVEN to track a BOUNDED d̃ (the [κ−d̃] term).
- **Assumption 1 REQUIRES β=1/Z bounded (0<β_min≤β≤β_max), i.e. Z≥Z_min>0.** Touchdown Z→0 ⇒ β=1/Z→∞ VIOLATES
  Assumption 1 → the terminal is OUTSIDE the theorem's domain. And d̃∝β=1/Z→∞ ⇒ the guaranteed UUB bound itself
  blows up toward the deck.
- **CAN κ FIGHT 1/Z? NO — provably.** κ tracks a bounded d̃; it is rate-limited (finite N) AND capped (κ_max),
  so with d̃→∞ near the deck κ<d̃, the premise κ≳d̃ fails, V̇≤0 breaks. κ-adaptation is a BOUNDED-disturbance
  rejecter — it cannot chase the unbounded 1/Z. (This is the rigorous form of the empirical κ-at-cap/θ-detonation.)
- **GAP in the soft-touchdown claim:** Thm1 claims ‖v_rel‖→0 as Z→0 because v_rel=h·Z and h is UUB-bounded — but
  that h-boundedness IS the UUB that requires Assumption 1 (β bounded). So the soft-touchdown claim extrapolates
  the UUB PAST the assumption (Z≥Z_min) that produced it. The empirical terminal fly-away = this gap made real.
- **FIX DICTATED BY THE PROOF:** NOT more κ/gain (can't catch unbounded d̃), NOT tighter funnel (inflates σ→
  chatter). Must RESTORE Assumption 1 near the deck (keep d̃/β-effect bounded). The h_rd reference governor does
  this: command the drone to ACTIVELY settle (v_rel→0 by deceleration) rather than rely on h·Z→0 → keeps h_e and
  the effective disturbance bounded as Z→0 → re-enters the proof's domain. Super-twisting separately removes the
  chatter (continuity) but does not by itself bound d̃.

**GEAR / DEPTH-REGULARIZATION CORRECTION (2026-06-24, user) — β IS bounded in practice; the "Z→0 voids Assumption 1"
verdict was wrong:** the quadrotor has LANDING GEAR, so true Z never reaches 0 (bottoms at z_gear≈0.09m) → β=1/Z is
PHYSICALLY BOUNDED (β≤1/z_gear≈10) → **Assumption 1 IS satisfied → the terminal is INSIDE the proof domain** (with a
finite, if large, β_max) → the controller HAS a UUB guarantee at touchdown and **κ CAN adapt to the FINITE d̃** (the
unbounded-1/Z verdict only held for the idealization). IMPLEMENTED (gt_feedback.py + gt_optical_flow.py): compute ALL
depth-normalized features with `1/(max(z,0)+0.01)` — bearing `s=V_x/(max(z,0)+0.01)` AND loom `h=V_v/(max(z,0)+0.01)`,
SAME floored depth, dropped the `abs(z)≥0.1` loom gate. The non-negative clamp prevents a transient/post-touchdown
GT z<0 from sign-flipping the regularized depth. This makes the COMPUTED β honor the physical bound (≤100 numerically,
≈10 at the gear) and kills spurious sub-gear/negative-depth spikes. Z_REG=0.01 is the numerical backstop; the GEAR is
the real bound. RESULT (NC129, IC2/IC4/IC5, K_R=2.5): **LOOM now bounded** (|h_z| max 3.0 vs −6 prior; descent CLEAN to
0.82m) — regularization works — **BUT the binding terminal killer is the LATERAL bearing 1/Z** (s_e_n 0.10→1.88→56 below
~0.8m → shared θ→5386 → a_u→16157 → fly; IC4 9.4m). The +0.01 floor is negligible at alt 0.6m so it can't bound the
lateral there. So: regularization = NECESSARY correctness fix (bounds loom, makes proof apply, kills spurious spikes)
but NOT sufficient; the LATERAL terminal 1/Z (residual lateral offset/velocity amplified by 1/Z below ~0.8m) is the
remaining killer. NEXT = convergence-ordering: descent governor gating h_rd on |s_e_n| (existing PLASMC_DESCENT_GATE),
re-test now that yaw(Ω_a=0.1)+lateral-mid(K_R=2.5)+loom are clean (NC114 was mixed but PRE those fixes). With β now
bounded, κ_max_z/N_z (let κ reach the finite d̃) are ALSO viable levers. CAVEAT: |s_e_n| itself 1/Z-amplifies → the
governor can't perfectly distinguish "metrically fine but close" from "off" (scale-free limit).

OPEN/CAVEATS: yaw residual overshoot (structural, no rate term) — acceptable, cycle gone. n=1 per config for
trend; confirm winners at n≥3 (variance is the metric). Loom-estimator + yaw bake are in the working tree,
NOT committed. Lateral terminal 1/Z is the shared hard part (affects all axes near touchdown).
