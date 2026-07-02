---
name: project_rover_turning_open
description: "TURNING-rover (Circular r=0.8, wz=0.48 rad/s) landing is the OPEN rover-phase problem (2026-07-02): baseline FAILS via yaw-loop ramp WINDUP (e_a -> -152 deg, psi_d races, couples to lateral, 8 m); GT-FB alpha sign-flip FALSIFIED as the fix (regresses stationary yaw-align); clean heading-hold (u_a=0, needs YAW_N=0 too) missed 1.0 m n=1 while the dirty sign-flip landed 0.158 n=1 — inconsistent n=1s, needs n>=3 arms. Candidate mechanisms: yaw double-integrator can't track a ramp (needs w_z feedforward); w_z=relative-yaw-rate is WRONG for target-spin in the centroid-flow FF (target rotation doesn't move the centroid); circular tracking-lag geometry (v*tau~0.36 m rotating offset)."
metadata:
  node_type: memory
  type: project
---

**TURNING-rover landing (Circular r=0.8, wz=0.48 rad/s ≈ 27°/s, 0.38 m/s tangential,
GT-FB) — the OPEN problem after the 2026-07-02 session. Straight-line moving landing is
SOLVED ([[project_rover_speed_sweep]]); rotation is not. All arms n=1 unless noted; data
preserved test_data/Rover_Turning/{baseline_sign1,sign_neg1,yawhold_sign1,
stationary_regression_neg1}.**

| arm | config | outcome |
|-----|--------|---------|
| baseline | SIGN=+1, yaw SMC active, YAW_ALPHA_FILT=0 | **FAIL 8.1 m** — yaw RUNAWAY: drone chases at only 10–27°/s, e_a −20→−152° monotone, u_a winds up to −3.35 (note: opposite sign of the drone's actual + rotation — see cascade note), psi_d races at the ±2 clamp, yaw error couples into lateral divergence |
| alpha sign-flip | PLASMC_GT_ALPHA_SIGN=−1 (new knob, default +1) | **LANDED 0.158 m** on the turning platform — but the yaw loop went quasi-inert (drone held heading, e_a swept ±180 as the rover rotated) |
| stationary regression | SIGN=−1, rover stationary | **YAW REGRESSES** (e_a max 77.7°, ends −32.8° vs +1's ~1° aligned) → **+1 is the CORRECT convention; the sign-flip hypothesis is FALSIFIED** (−1 also dirty: re-flips the validated w_z) |
| heading-hold | SIGN=+1, YAW_{GAMMA,KAPPA0,OMEGA,N}=0 → u_a≡0 exactly | **MISS 1.0 m** (min-alt 0.10, beside platform). ⚠ zeroing needs YAW_N=0 too — the κ_a ODE adapts via n_a, NOT Γ_a (first attempt left u_a→0.74) |

**MECHANISMS on the table (all consistent with parts of the data):**
1. **Yaw-loop ramp windup (baseline fail — solid).** The yaw chain u_a→∫→psi_d with the
   SO(3) inner measuring yaw FROM alpha is a marginal double integrator (the YAW_OMEGA
   0.5→0.1 bake); a persistently rotating target = RAMP demand → integral windup → psi_d
   races → e_R saturates → image-frame coupling → lateral divergence. Candidate fix: feed
   the observable target rate as a FEEDFORWARD (u_a += w_z-based term) so the loop only
   handles the residual; or a slew-matched psi_d.
2. **w_z is kinematically WRONG for target spin (hypothesis).** gt_feedback feeds
   w_z = −d(ry)/dt (relative-yaw rate). For DRONE yaw that equals camera rotation →
   correct centroid-flow FF (the validated 06-25 sign fix). But TARGET spin rotates the
   marker PATTERN, not the centroid → contributes ZERO centroid flow → feeding it into
   cross(w_i, s)/h_d injects a spurious FF ∝ target rate. (Effect →0 when centered since
   s≈0 — may explain why it matters only during the offset transient.)
3. **Circular tracking-lag geometry.** The τ≈0.9–1.0 s lag ([[project_rover_speed_sweep]])
   on a circle = a ROTATING offset ≈ v·τ ≈ 0.36 m plus curvature lead — the drone chases a
   point that keeps turning; predicted sustained miss O(0.3–0.5 m), which the terminal
   phase may or may not close depending on phase → could explain both the 0.158 landing
   and the 1.0 miss as the SAME distribution (n=1 noise).

## ✅ RESOLVED into a 2-mechanism decomposition (2026-07-02, pure-spin isolation)
New knob `PLASMC_GT_SPIN_WZ` (gt_feedback): SYNTHETIC in-place target spin (adds
wz·(t−t0) to target yaw; alpha AND w_z self-consistent through the slope estimator;
unit-verified alpha ramp −0.48, w_z=+0.480). The Ackermann rover physically CANNOT spin
in place (min turn radius 0.56 m); for perception-ON use a revolute platform_joint +
gz JointController instead (and POSE_IDX_TARGET → the landing_platform link).

| arm | config | result |
|-----|--------|--------|
| heading-hold, Circular (n=4 pooled) | translation+rotation, u_a≡0 | **0/4, systematic miss 1.011/1.084/1.214/1.680 m** |
| heading-hold, PURE SPIN (n=2) | rotation only, rover parked | **2/2 ON-PLATFORM 0.034/0.025 m** (e_a sweeps ±145 un-tracked, harmless) |
| yaw ACTIVE, PURE SPIN (n=2) | rotation only | **windup in BOTH** (u_a 3.7/3.3, e_a ±180/156): 1 FAIL 2.44 m, 1 landed 0.203 (lucky) |
| yaw ACTIVE, Circular (n=1) | translation+rotation | FAIL 8.1 m (windup + translation) |

**VERDICTS:**
- **Mechanism 1 (yaw ramp windup) CONFIRMED IN ISOLATION** — pure spin with the yaw SMC
  active winds u_a to 3.3–3.7 and stochastically detonates the lateral, with ZERO
  translation present. The current yaw loop must NOT chase a 0.48 rad/s rotation.
- **Mechanism 2 (spurious target-spin w_z) RULED OUT** — the same w_z=+0.48 flowed
  through the flow FF in the pure-spin heading-hold arm and the landing was dead-center.
- **Mechanism 3 (curved-translation lag) CONFIRMED as the heading-hold Circular miss** —
  0/4 at ~1.0–1.7 m on a 0.38 m/s circle while a FASTER 0.47 m/s straight line is 3/3:
  the ROTATING VELOCITY DIRECTION defeats the τ≈1 s type-1 tracker (rotating offset
  e≈v·τ that the terminal phase can't null in phase), not the speed.

**STATE: pure-rotation landing SOLVED (heading-hold, 2/2 dead-center). Remaining gaps:**
(1) CURVED-translation tracking → needs target velocity/acceleration (curvature) FF —
the same τ-lag lever as [[project_rover_speed_sweep]], but binding at 0.38 m/s when the
velocity direction rotates; (2) heading TRACKING on a rotating platform (only if
touchdown ALIGNMENT is required — note the square marker/platform is π/2-symmetric) →
w_z feedforward into u_a so the loop handles only the residual.
Perception-ON on a turning rover also needs the alpha-rate cap raised
([[feedback_rover_yaw_cal_resolved]]).

## Why MATLAB tracks a rotating deck but PX4 can't (2026-07-02, verified)
MATLAB Circular (deck ψ=wz·t @0.48) is 25/25 SP with the SAME error-driven u_a law.
Three deliberate PX4 divergences broke ramp-tracking, each forced by the **~287 ms PX4
yaw actuation lag** ([[feedback_input_cal_yaw_lag]]; MATLAB inner = lag-free SO(3) sim):
(1) `Ω_a` 0.5→0.1 bake (the integral IS the ramp-tracker; detuned because u_a→∫psi_d is
a double integrator with no phase margin vs the lag); (2) e_a wrap π-fold(±90°, MATLAB —
its ellipse alpha is intrinsically π-periodic) → 2π-disambiguated (PX4; DELIBERATE
upgrade killing the ±90° saddle limit cycle — do NOT revert; under a ramp the error can
now reach ±180 and wind up); (3) psi_d rate clamp + conditional integration + alpha-
sourced measured yaw (stationary-tuned band-aids).

> **⚠ CORRECTED by the 2026-07-02 pull ([[project_yaw_observability_campaign]], MATLAB side):**
> (a) a yaw-rate FF **WAS attempted** in MATLAB — `Omega_d=[0;0;u_a]` into so3_tracker — and
> REVERTED (it recycles the ASMC's LAGGED OUTPUT, "circular"; helped constant yaw, broke Sin
> 5/5→3/5). The measured-`w_z`→u_a DISTURBANCE cancellation below remains untried and is
> architecturally different (measure the disturbance, don't recycle the output).
> (b) MATLAB now ALSO has the 2π alpha (ported from PX4, 25/25) — pre-2π MATLAB was PINNED at
> e_a 88–90° (not gracefully folding); post-2π it tracks 0.48 rad/s with termEa≈17° on baked
> gains → the MATLAB-vs-PX4 gap reduces to LAG + the Ω_a detune.
> (c) MATLAB names the yaw-rate CEILING (~0.5 rad/s ok, 2× fails) as ARCHITECTURAL-LATERAL:
> s_e ORBITS in the yaw-aligned V frame at the yaw rate → CBF cone sat + funnel breach — the
> same yaw→lateral detonation as our PX4 spin_active arm.

## NO yaw-rate feedforward into u_a was attempted in MATLAB (2026-07-02, exhaustive check)
Every `u_a` in the codebase (canonical single-run :932, VDF `+blocks/yaw_asmc.m`,
comparison ctrls, Obsolete/, git -S history) is the pure error law
`u_a = Γ_a·σ_a + sat(σ_a/E_a)·κ_a + Ω_a·e_a`. **By DESIGN, per the manuscript**
(manuscript.tex:213): `α̇ = −ψ̇_b + d_α, d_α = l_α^T ω_t` — target rotation is modeled
as the DISTURBANCE d_α, handled by ADAPTIVE DOMINATION (the MATLAB gain comments are its
implementation: `kappa_a_0=2.0 "pre-seed so sat*kappa_a can provide DC feed-forward"`,
`Omega_a 0.8→0.5 "curb integral windup at high wz"`). Works lag-free; winds up through
PX4's 287 ms. ASYMMETRY: the TRANSLATIONAL channel already measures-and-cancels its
rotation terms (h_d's cross(w_i,s) transport FF) while the yaw channel leaves d_α to
adaptation — even though the measured `w = ω_t − ψ̇_b·ê3` (manuscript def) means the
lstsq `w_z` literally CONTAINS the needed signal. **The `u_a ← u_a + k_ff·w_z` FF is
therefore a NEW design element** (candidate manuscript contribution: measured-w
cancellation of d_α under actuation lag, no phase-margin cost), consistent with the
paper's own kinematics. Pin sign/gain EMPIRICALLY on the spin harness (yaw signs have
burned us twice: [[feedback_gtfb_wz_sign_bug]], PLASMC_GT_ALPHA_SIGN); acceptance = pure
spin 0.48 with yaw SMC active → e_a converges ~0 (vs today's ±180 windup), then Circular.

## ✅ YAW RAMP WINDUP **SOLVED** by Omega_d=[0;0;u_a] inner-loop FF (2026-07-02, user-directed)
Implemented `PLASMC_YAW_OMEGA_D_FF=1` (default OFF; controller.py _attCtrl: `w_u[2] +=`
the rate-limited `_ua_psid` that psi_d actually advances at; zeroed during yaw-hold) —
the same FF MATLAB tried in so3_tracker (helped constant-rate, broke oscillating Sin).
On the CONSTANT-rate rotating target it works:
- **PURE SPIN 0.48 + yaw active + FF: 2/2 ON-PLATFORM (0.030/0.025 m), e_a BOUNDED
  ≤83°/≤76° (2nd-half mean 23–25°), u_a ≤1.40** — vs no-FF: e_a ±180 windup, u_a 3.3–3.7,
  1 FAIL 2.44 m. The drone chases the spin with a bounded constant-rate lag (mirrors
  MATLAB post-2π Circ e_a ~53°).
- **Circular + FF (n=2): yaw HEALTHY in both** (e_a ≤70/100, u_a ~1.0, no windup) but
  landing 1 MISS 1.48 / 1 NEAR 0.621 → the residual Circular failure is PURELY the
  CURVED-TRANSLATION lag (independent, as the u_a≡0 heading-hold arm 0/4 proved).
⚠ Keep OFF for oscillating yaw references (the MATLAB Sin 5/5→3/5 caveat: it feeds back
the ASMC's lagged output — only valid for slow/constant rates). Candidate default-ON for
the rover scenario only. The measured-w_z→u_a variant (true disturbance FF) remains
untried — would handle varying rates too.

> **⛔ RECLASSIFIED (2026-07-02, user catch): the a_t FF below is an ORACLE DIAGNOSTIC,
> NOT a deployable fix.** The manuscript Problem Statement forbids it: "unpredictability
> means that no model of the target dynamics is available and NEITHER THE TARGET POSE NOR
> ITS DERIVATIVES can be measured or estimated in the closed loop"; a_t/z sits inside
> d_h = a_t/z − (F_g+F_d)/(mz) (Assumption 1, unknown bounds → adaptive domination is the
> paper's designed answer; UUB ≠ zero residual). PLASMC_TGT_VEL_FF + PLASMC_GT_TGT_LEAD
> therefore quantify the ORACLE BOUND (value of target-motion knowledge) vs the compliant
> adaptive residual — a publishable ABLATION: compliant curved residual ~1.0–1.7 m
> (rotating d_h at ~loop bandwidth maximizes the realized UUB radius, exactly as theory
> predicts) vs oracle 0.31–0.51 m. Deployable ONLY under a declared COOPERATIVE-platform
> extension (deck broadcasts IMU — realistic for ship decks). Image-only d_h observation
> is blocked by unknown β (why the paper chose domination). The yaw Omega_d=[0;0;u_a] FF
> is UNAFFECTED (feeds back the controller's OWN output — no target knowledge).
> Manuscript-compliant framing: straight/const-v targets ≤1.1 m/s land reliably
> (velocity-matching is free — h is RELATIVE, within the formulation); curved targets
> land within the UUB residual → platform size vs curvature is an operational spec.

## ⭐⭐⭐ CURVED-LAG SOURCE IDENTIFIED (2026-07-02 deep-dive): a SELF-SUSTAINED LATERAL LIMIT CYCLE that the curve keeps ENGAGED at altitude
Data-driven decomposition on the heading-hold Circular reps (u_a≡0, no yaw coupling), with
three falsifications on the way:
1. ✗ NOT a static v·τ servo offset: the error is a constant-amplitude (~0.7 m) EPICYCLE —
   the e-vector rotates at 1.10–1.12 rad/s (rock-solid across reps), NOT the target's
   wz=0.48; |a_u| median 1.3–1.6 m/s² = 7–8× the centripetal need. LINEAR contrast: same-
   size error but DIRECTION-LOCKED (rot 0–0.3) → decays into the terminal → lands.
2. ✗ NOT the transport-term parity bug: hypothesized cross(w_rel,s) vs manuscript ψ̇_b —
   but `PLASMC_CH_CLEAN=1` is ALREADY BAKED (controller.py:314; user caught it); the
   epicycle occurs WITH the correct ψ̇_b transport (≈0 under heading-hold). (My CH_CLEAN
   "falsification arm" was vacuous — identical config to baseline.)
3. ✗ NOT forced resonance: e_x spectrum in the tracking window = fundamental ~1.7–1.8
   rad/s + CLEAN HARMONICS (3.4, 5.1) and ~NO energy at the 0.48 drive. Also the
   commanded a_u projects NET-DAMPING onto ė (−0.13…−0.15) — the command opposes the
   motion, yet the cycle persists (harmonic-rich = relay-like waveform).
→ **SOURCE: the lateral loop carries a self-sustained NONLINEAR (relay-type) LIMIT CYCLE
(fundamental ~1.7 rad/s, amplitude ~0.3–0.7 m) — the same relay(κ·sat/E boundary-layer)
+lag family as the stationary campaign's terminal cycle ([[project_residual_cycle_wumax_bake]],
E_xy-tightening relay+lag dead-end). The CURVE's role is NOT forcing: the persistent
centripetal demand HOLDS THE LOOP AWAY from the quiet σ≈0 equilibrium, keeping the relay
engaged at ALTITUDE — straight lines let the error decay into |σ|≪E where the switching
is linear/quiet (direction-locked residual, closed terminally); stationary targets only
meet the cycle at TERMINAL 1/z. The oracle a_t FF removed the cycle's BIAS (center →
near-zero: steady 0.9–1.6→0.31–0.51) but not the cycle — its ~0.3–0.4 m amplitude is
exactly the platform-edge coin-flip in the FF runs.**
Implications: the curved-landing lever = REDUCE THE CYCLE AMPLITUDE (relay describing-
function levers: boundary layer/κ/lag — note the stationary wins W_U_MAX=2.0 + VDS_KF_Q=1
are already ON; P2INF_xy widening = the known ζ_h-unsaturation lever, untested here), NOT
more tracking gain and NOT (only) target-motion FF. Analysis scripts inline (energy
projection + spectrum); data = yawhold_arm_n3 + Rover_SpeedSweep contrast.

## ⭐ CURVED-TRANSLATION LAG: the a_t FF WORKS (steady lag −60%); terminal edge-margin remains (2026-07-02)
Implemented `PLASMC_TGT_VEL_FF=1` (default OFF): gt_feedback estimates the TARGET's own
acceleration a_t (rate of its velocity vector — the quantity a curve demands; velocity-
MATCHING needs no FF since the flow h is RELATIVE) and the controller adds tgt_acc_V[:2]
to a_u after the caps. RESULTS (Circular wz=0.48 + yaw-FF, GT-FB):
- **Steady tracking lag 0.9–1.6 → 0.31–0.51 m** (n=6 across arms) — the CURVE PENALTY is
  removed (≈ the plain v·τ straight-line lag). Linear side-benefit: 0.40 → **0.14 m**
  (the FF also covers the gate-start acceleration). No fly-aways.
- **Terminal still misses at ~platform-edge margin**: the residual ~0.3–0.4 m lag still
  ROTATES, drone rides the descent at rel_lat 0.25–0.43 vs the 0.3 m half-width →
  coin-flip touchdowns (1 NEAR 0.447; misses 1.1–3.0).
⚠ ESTIMATOR LESSON: the first live version read |a_t|≈0.85 vs true 0.184 — the 125 Hz
control loop reading the 54 Hz /pose stair-steps, and DOUBLE differentiation amplifies
stairs ~5× → terminal detonation (17.7 m). FIX: dedup sampling (only on pose CHANGE) +
1.0 s accel window + |a_ff|≤PLASMC_GT_TGT_FF_MAX=0.5 clamp → unit-exact (0.183) under
stair-stepped input. (The single-diff relative-velocity path tolerates stairs; double
diff does not — cousin of [[feedback_gt_noise_uniform_dt]].)
- Injection order tested: before vs after the commit taper/caps = NO difference (the
  TERMINAL_COMMIT extent gate never fires in GT-FB — extent max 295 < 400).
- **LEAD PURSUIT (`PLASMC_GT_TGT_LEAD`, predict p+v·τ+½a·τ², default 0=off): UNVALIDATED
  and n=1-REGRESSED** (τ=0.8: steady lag 1.11, MISS 2.13; 2 of 3 reps lost to a NEW
  silent startup flake `set_rate_odometry TIMEOUT` → exits 0 → false-SUCCESS, detection
  since added to the launcher). Candidates for the last ~0.3 m: smaller lead (0.4–0.5)
  or phase-advanced v (rotate v_t by wz·τ) instead of quadratic; OR shrink the FF's own
  phase lag (FF_TAU 1.0→0.5). Data test_data/Rover_VelFF/.

Knobs: `PLASMC_GT_ALPHA_SIGN` (default +1; −1 falsified, diagnostic only),
`PLASMC_GT_SPIN_WZ` (synthetic in-place spin), `PLASMC_YAW_OMEGA_D_FF` (the windup fix).
Heading-hold recipe: PLASMC_YAW_GAMMA=0 KAPPA0=0 OMEGA=0 **N=0** (all four). Data
test_data/Rover_Turning/{yawff_spin_n2,yawff_circular_n2,yawhold_arm_n3,spin_hold_n2,
spin_active_n2,...}; harnesses in test_data/Rover_AB_harness/.
Continues [[project_moving_rover_landing_works]].
