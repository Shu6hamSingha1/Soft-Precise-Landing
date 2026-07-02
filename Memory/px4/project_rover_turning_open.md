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

## NO yaw-rate feedforward was ever attempted in MATLAB (2026-07-02, exhaustive check)
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

Knobs: `PLASMC_GT_ALPHA_SIGN` (default +1; −1 falsified, diagnostic only),
`PLASMC_GT_SPIN_WZ` (synthetic in-place spin). Heading-hold recipe: PLASMC_YAW_GAMMA=0
KAPPA0=0 OMEGA=0 **N=0** (all four). Data test_data/Rover_Turning/{yawhold_arm_n3,
spin_hold_n2,spin_active_n2,...}; harnesses in test_data/Rover_AB_harness/.
Continues [[project_moving_rover_landing_works]].
