---
name: project_rover_turning_open
description: "TURNING-rover (Circular r=0.8, wz=0.48) thread: yaw ramp windup SOLVED (Omega_d FF); k_r RESOLVED (HD_KR=0 = wrong reference, rejected; DHD_SRC falsified c3-noise surgically; defaults stand). CYCLE MECHANISM CORRECTED 2026-07-03: W*=1.3-1.7 rep-scattered (locked-1.7+harmonics was an FFT-bin artifact), A*W^2 = 1 m/s^2 const (cone-clamp-set amplitude, duty 26-38%), actuation phase only -25..-55 (NOT -120; method artifact), fuel = anti-position command + ANY lag pumps a rotating error (P_cyc>0 7/9, sign predicted by chi vs W*tau 9/9); damping quadrature destroyed in the barrier chain (drift branch chi=-1.5 instead of +90; switch 0.47 share at +7). Exit sized +25-40deg at 1.3-1.7 rad/s: PLASMC_AU_LEAD approved 07-03, under test."
metadata: 
  node_type: memory
  type: project
  originSessionId: 3c2f4c67-05c1-4e6f-966b-0e62018fc8a7
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

### ⛔ [SUPERSEDED 2026-07-03 by the corrected chain below — the −120° number and the
### "phase not parameter-recoverable / only gain is tunable" conclusions are METHOD
### ARTIFACTS (boxcar-smoothed GT double-diff evaluated at an FFT bin of a 3.7 s window;
### the tracking-band bins sit at exactly 1.71/3.42/5.13 rad/s = the reported "fundamental
### + harmonics"). Kept for the audit trail.]
### ⭐⭐ CYCLE SOURCE PINNED (2026-07-02, cross-spectral): ACTUATION PHASE flips the
### velocity-damping sign at ~1.7 rad/s; the drift/c-term group CARRIES the cycle
- **Phase(delivered accel vs commanded a_x) at 1.75 rad/s = −120°** (cross-spectrum,
  GT-derived delivered vs logged command; ~±25° uncertainty incl. the boxcar smoothing
  ≈−15° — even −60…−90° flips most of the damping). A command intended as damping
  (anti-ė) is DELIVERED rotated past quadrature → its projection on ė turns POSITIVE →
  **negative effective damping**. Phase budget: tau_ia LPF (~8°) + PX4 attitude dynamics
  from rate setpoints (~30–40° at 1.75 with K_R≈2–2.5) + rate loop/motors + 38 ms
  transport + ZOH — physically fixed (MC_*RATE_P dead).
- **Per-term spectral amplitude at the 1.7 fundamental:** resid(c-term + drift
  χ_r·ζ̇_r/G) = 1.68 m/s² ≫ κ·sat 0.16 ≫ ζ_h 0.09 ≫ ζ_r 0.03 — the cycle's command
  content is overwhelmingly the VELOCITY-FEEDBACK group (drift via s_dot_meas + c_h),
  matching the stationary campaign's "drift term = a_u oscillation driver".
- **Mechanism:** velocity feedback (drift/c) + >90° actuation phase at ~1.7 rad/s =
  self-excited oscillator; the nonlinearities (barrier slopes, κ·sat) set the AMPLITUDE.
  Amplitude is OPERATING-POINT dependent: straight/stationary → error decays to small
  σ (low barrier slope) → residual cycle only ~5–7 cm (measured on Linear reps!);
  CURVE holds a standing bias 0.3–1 m → steeper barrier slope → bigger loop gain →
  0.3–0.7 m cycle; stationary TERMINAL → 1/z inflates gain → the known ~1 Hz cycle.
  ONE mechanism, three regimes.

### Limit-cycle presence in the OTHER loops (validated 2026-07-02, cfree stationary reps)
- **YAW: NO cycle** — e_a std 0.57–0.87°, 1–2 sign flips, no coherent fundamental
  (post YAW_OMEGA=0.1 bake). Yaw's failure mode is ramp WINDUP (turning targets), not
  self-oscillation.
- **DESCENT: bounded RIPPLE, not a destructive cycle** — loom h_z ripple std 0.071
  about the −0.30 ref (~24%), σ_z std 0.09, spectral content 0.8–1.7 rad/s (same
  family); consistent with the stationary z-story (E_z=0.5 engages κ damping).

### ✗ P2INF_xy widening TESTED on the curve cycle — WEAK/INEFFECTIVE (2026-07-02, n=3/cell)
Heading-hold Circular, P2INF pinned all-3-axes (Z=1.5): **2.0 → 0/3** (1 NEAR 0.464 with a
genuinely suppressed cycle osc_std 0.026/e-rot 0.69 — but 1 WANDER outlier e_mean 15.8 m =
the memory's damping↔softness tradeoff biting as lost restoring stiffness — and 1 unchanged
epicycle miss); **3.0 → 0/3** (2 NEAR-ish 0.62-0.74, e_rot 0.92-1.21 = cycle largely intact,
osc_std ≈ baseline). Net vs baseline (1.0: 0/4, e 0.70, rot 1.11): touchdown lat slightly
better (0.46-0.99 vs 1.01-1.68), cycle NOT killed, new wander risk. CONSISTENT with the
pinned mechanism: widening lowers the ζ_h barrier slope, but the cycle's carrier is the
DRIFT/c group and the >90° actuation phase is untouched. With the ENTIRE stationary
playbook already baked (K_R=2.5, W_U_MAX=2.0, VDS_KF_Q=1, Γ=0.25, Z_REG=0.2) and P2INF now
tested-weak, the curve-cycle residual is STRUCTURAL at the current actuation phase —
remaining real levers: ~~reduce actuation phase (uXRCE-DDS rate path)~~ — **⛔ RULED OUT
BY USER 2026-07-02 ("we will not go ahead with the uXRCE-DDS low-latency rate path");
do not re-propose** — leaving OPERATIONAL SPEC as the accepted resolution: the curved-
target residual (~0.3–0.7 m cycle amplitude) is a characterized property of the
lag-limited plant; platform size vs path curvature is the deployment-side answer
(a ≥1 m platform accepts it). Straight/stationary performance unaffected.
⛔ "This closes the curved-target thread" was WRONG (user corrected same day) — the
thread REOPENED with the cap-audit + exact decomposition below.
Data test_data/Rover_Turning/p2inf_sweep/.

### ⭐⭐⭐ SINGLE-SOURCE UNIFICATION (2026-07-02, user insight, correlation-verified) + GAIN-LEVER SWEEP
**All a_u components are ONE signal:** correlation matrix on the cycle band — ζ_r↔s_e_n
+0.99, drift↔s_dot_meas +0.92, switching↔s_e_n +0.88, h/loom↔s_dot_meas −0.80, pairwise
cluster 0.5–0.99. The lateral channel is a SISO loop: (e,ė) → 1/z bearing → parallel
branches {Γζ, χζ_r, drift, sat·κ relay, ḣ_d, loom} → Σ=a_u → −120° actuation → (e,ė).
The "carriers" are branches of ONE loop — explains why single-channel smoothing/pruning
never kills the cycle (the describing function re-balances), why ONE impact axis, why
all phases cluster within ~30°. Cap audit: NO hard cap active (w_u/tilt/izeta/w_i ~0%
duty); the one live nonlinearity = sat(σ/E) at 29–36% duty + κ ratchet 0.27→1.3–1.7.
Exact decomposition (recon residual 3%): switching 0.51 > c3=−ḣ_d 0.40 (half of which =
the 06-29 `_hd_src=full h_d` regression differentiating the HD_KR funnel term; DHD-KF
itself honest, gain 0.97–1.01) > loom 0.26 > drift 0.10 > Γζ_h 0.09.
**Phase is NOT parameter-recoverable:** measured filter lags at 1.75 rad/s: VDS-KF −9/−16°
(gain 0.69–0.76), TAU_IA −14°, GT-FB h −5/−12° (a −140° first read was frame-mixing in
the reference — h is honest), DHD-KF −6°. Total ~25° vs the ~120° actuation chain → all
velocity-branch gain is pump; only GAIN attenuation is tunable. (Re-interpretation: the
VDS_KF_Q 10→1 bake win = gain attenuation at cycle frequency, not noise smoothing.)
**Singleton gain-lever sweep (heading-hold Circular n=3 each, data cycle_gain_sweep/):**
- **HD_KR 0.5→0: BEST — 1/3 ON-PLATFORM (0.200, first Circular landing of the campaign)**;
  mechanism clean: c3 0.41→0.32, switching 0.52→0.40, sat duty 36→26%, κ peak 1.33→1.12,
  e-rot 1.11→0.77–0.89 (crossover down with gain — DF-consistent). Removes the 06-29
  regression branch; no robustness trade. CANDIDATE for rover configs (stationary IC gate
  required before any bake — HD_KR=0.5 was baked on stationary evidence).
- **P_xy 1.5→5: FALSIFIED 0/3** — κ ratchet WORSE (0.06→3.04, duty 38%): leakage drains
  the domination the curve's real d_h needs → σ grows → adaptation re-triggers harder
  (classic adaptive-leakage limit cycle). κ level is NOT a free amplitude knob.
- **E_xy 1.0→1.5: FLAT 0/3** — duty 36→24% but κ compensates to 1.95; DF re-balances.
**Frontier statement:** each branch lever shaves ~20–25% loop gain; the cycle re-balances
at slightly lower frequency, similar amplitude. Dropping crossover below the ~90°-phase
frequency (~1.0–1.2 rad/s) needs ~3× TOTAL gain cut = ~3× less tracking bandwidth =
standing curve bias ~1.2–3 m → swaps miss-by-cycle for miss-by-bias. At fixed actuation
phase the gain levers move along a bias↔cycle FRONTIER; they cannot beat both. Remaining
structural exits: phase lead on the single shared path (PLASMC_AU_LEAD proposal, NEW code,
not yet approved) or platform size. K_R spent; DDS ruled out by user.

### Why MATLAB doesn't show this cycle (2026-07-02 analysis)
MATLAB DID have the same cycle FAMILY (06-19/20 campaign: X/Y noise-pumped cycles inside
the boundary layer, z-cycle, yaw cycle, "σ rings + eR > commanded tilt = inner-loop
attitude lag") — but at small amplitude and FIXABLE there, because:
(1) **actuation phase ≈ 0 at 1.7 rad/s** — torque-level inner loop at integration rate,
no MAVSDK transport, no PX4 rate loop, no tau_ia-equivalent → the velocity-damping sign
NEVER flips → no self-excitation; cycles arose only via other pumps (s̈ noise) and were
(2) **damped at the inner loop** (kΩ_z 0.2, kR, E_z bakes) — MATLAB's inner gains are
free; PX4's are not. PX4 sits in a qualitatively different regime: ~100–120° actuation
phase at the cycle frequency makes negative damping STRUCTURAL for any velocity-feedback
path, leaving only amplitude management (operating point/barrier slope/κ) as levers.

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

## ⭐⭐⭐ CYCLE MECHANISM CORRECTED (2026-07-03 deep-dive, user-driven): position-phased
## command + ANY lag = energy pump; damping quadrature destroyed by the barrier chain;
## amplitude set by the cone clamp. The −120° story is OVERTURNED.
Tools (preserved `test_data/Rover_AB_harness/`): `cycle_tone_fit_v2.py` (bin-free rotating-
phasor fit), `cycle_stage_phase.py` (complex-pair stage budget, ENU convention — ⚠ NED
complex conjugates the rotation; fit in ENU), `cycle_branch_audit.py` (exact a_u branch
decomposition, recon resid 0.00%). 9 curve reps (yawhold_arm_n3 + dhd_nokr + hdkr_0).
- **Cycle structure:** ONE rotating phasor, W* = 1.3–1.7 rad/s REP-SCATTERED (the locked
  "1.7 + harmonics 3.4/5.1" was the FFT-bin artifact — bins of the 3.7 s tracking window),
  A = 0.40–0.60 m, same rotation sense as the rover orbit, 80–93% of error variance; plus
  orbit-locked bias 0.02–0.47 m + DC ~0.3 m. **A·W*² = a_osc ≈ 1.0 m/s² CONSTANT across
  reps** → amplitude is authority-set: A ≈ a_osc/W*².
- **Stage budget at W* (medians):** controller ≈ anti-position ±25°, ×2.4–10; tau_ia+cone
  −9° but **gain 0.43 (cone active 26–38% of samples — the DF that caps growth)**; PX4
  attitude −3..−18° ×0.86; tilt→delivered −10..−24° ×1.00; **ACTUATION TOTAL −25…−55°,
  NOT −120°.** Built-in check the old method lacked: kinematic closure arg(a_del/e)=±180°
  (ë=a_del) holds +154…+180° in 9/9 reps.
- **Energy ledger (the fuel):** pump ∝ sin(φ); P_cyc>0 in 7/9, ≈0 in 2. Rep-by-rep the sign
  is predicted by χ vs Wτ (χ = command lead beyond anti-position; damping needs χ>Wτ≈25–40°).
  The ONE Circular rep that ever landed (hdkr_0 21-57-07, 0.200 m) is exactly the χ≈Wτ≈97°
  neutral rep. **Mechanism: anti-position command on a circulating error + ANY lag points
  the delivered vector ahead of the target direction → pumps ∝ sin(Wτ). No phase flip
  needed.** Cone clamp caps delivered oscillation at a_osc≈1 m/s² → steady orbit.
- **Branch audit (why we can't damp):** shares/χ of the a_u tone — switch θ·sat·κ **0.47 /
  +7°**; drift χ_r·ζ̇_r/G **0.25 / −1.5°** (the DESIGNED damping branch arrives with ZERO
  quadrature: ζ̇_r's velocity content is destroyed by (i) scale-free normalization
  d/dt(e/Z)=ė/Z+|h_rd|·e/Z, (ii) funnel-rate mixing, (iii) barrier-slope AM g_r(r) by the
  radial error component); c3 0.24 / −1.4°; **loom 0.08 / +85° = the only correctly-phased
  branch, tiny**; others ≤0.04. ⇒ ~96% of the command tone is centripetal → gain knobs
  re-balance branches that all share χ≈0 and CANNOT move total phase (why P/E/P2INF/HD_KR/
  DHD_SRC all "re-balanced"); K_R=2.5 worked because it cut Wτ (real phase). The coin-flip
  Circular landings = the χ-vs-Wτ margin fluctuating rep to rep.
- **Sized exits:** need +25–40° of quadrature at 1.3–1.7 rad/s (NOT +120): (a)
  **PLASMC_AU_LEAD** — first-order lead on the shared lateral world command (e.g. ω_z≈0.9,
  ω_p≈3.5 → +35° at 1.4, HF ×3.9, bounded downstream by the cone) — **USER-APPROVED
  2026-07-03, implementing for Circular test**; (b) restore drift-branch quadrature at the
  source (feed raw measured rate, bypass barrier AM — control-law redesign, manuscript
  implications); (c) cut Wτ further (spent); (d) platform size (operational).
- Caveats: tone fits span ~1.4–2 cycles (descent is 7.6 s — intrinsic); trusted via the
  identical estimator across stages + 9/9 kinematic closure + 9/9 P_cyc-sign prediction.
  Cone duty + gain-0.43 are raw sample statistics (fit-independent).

## ⭐⭐⭐ PLASMC_AU_LEAD WORKS on the curve (2026-07-03) — the corrected mechanism's PREDICTED fix; scale-free ratio clamp is load-bearing for the terminal
Implemented `PLASMC_AU_LEAD` (default OFF): first-order phase-lead C(s)=(1+s/ω_z)/(1+s/ω_p)
on the LATERAL inertial command `I_a[:2]`, applied right after `I_a_raw` (before cone+tau_ia
so its HF gain lands on the cone). `I_a_raw` stays logged PRE-lead (the lead shows up as the
`ACT(Iar→ad)` stage shift). Defaults ω_z=0.9, ω_p=3.5 → **+35° / gain×1.7 @1.4 rad/s**, HF
gain ω_p/ω_z=3.9. Discrete impl verified vs analytic (+35.2° x1.70 @1.4). Depth-free + amplitude
scale-free (linear, unity DC); fixed time-constants only (same class as tau_ia — plant-time-scale).
- **RAW lead (no clamp) — mechanism CONFIRMED, but detonates terminally.** Mid-descent it
  KILLS the cycle: lat@1m 0.6→**0.21–0.40 m**, delivered oscillation a_osc 1.0→0.31, **P_cyc≈0**
  (−0.01/+0.07 vs baseline +0.1–0.2), cone duty 26–38→16–21%, actuation stage now LEADS. But
  the ×3.9 HF gain multiplies the terminal 1/Z spike: termHFgain 0.8→1.5–1.9, **peak|Ia_xy|
  65–1639 m/s²** (baseline 10–20) → detonation <0.5 m → 0/3 both doses (+35°: 2.2/2.2/5.4;
  +48° HF×7.1: 1.4/1.9/5.8). So the lead is a MID-DESCENT tool wrongly left on into the 1/Z zone.
- **⛔ AU_LEAD_MAX (fixed m/s² clamp) = a SCALE VIOLATION (user caught) + didn't work** (4.0:
  1.58/2.11/7.30, 0/3). REMOVED.
- **✅ AU_LEAD_RATIO (scale-free) — the fix.** Bound the lead's EXTRA term relative to the
  command: `|Δ_lead| ≤ ratio·|I_a_raw_xy|` (dimensionless, no metric/altitude gate; rides the
  signal's own scale — constraint-clean per [[feedback_scale_free_depth_free]]). Default 1.0.
  Terminal spike CRUSHED: r0.5 **peak|Ia_xy|=7.3–7.7** (from 65–1639), termHFgain 1.23–1.28.
- **RESULT (heading-hold Circular, GT-FB, ω_z=0.9/ω_p=3.5):**
  - **r0.5 = 3/3 NEAR, TIGHT: 0.321 / 0.356 / 0.488 m, 0 fly-aways, osc_std 0.031–0.040**
    (half the baseline 0.06–0.07), e_rot 0.81–0.89. FIRST robust Circular result of the campaign
    (baseline 0/4 at 1.0–1.7 m; raw-lead 0/3 at 2–6 m). All 3 sit right at the 0.3 m platform edge.
  - **r1.0 = BIMODAL/unstable: 1 ON-PLATFORM 0.151 + 1 fly-away 60 m + 1 NEAR 0.915** — looser
    cap re-admits the terminal instability.
  - **r0.3 = 1/3 NEAR (0.378) + 2 MISS (1.41/1.81), osc_std bimodal (0.037 clean vs 0.349 cycle-back)**
    — TOO TIGHT: the clamp now bites the MID-BAND lead delta too, intermittently clipping the phase
    correction that damps the cycle. ⇒ **NOT monotone — r0.5 is an INTERIOR SWEET SPOT** (pass the
    mid-band lead to damp, block only the terminal 1/Z spike). r0.3 clips damping, r1.0 admits
    terminal instability; r0.5 tight 3/3.
- **⭐ TERMINAL_COMMIT is architecturally WRONG for the MOVING rover (user insight 2026-07-03,
  data-confirmed).** `PLASMC_TERMINAL_COMMIT` (baked ON) fires via _terminalCommitStep when median
  MARKER_EXTENT_PX > ~400; case(b) COMMIT ZEROES zeta_r → STOPS regulating s_e_n & HOLDS (open-loop,
  stationary-designed — "lands where the target WAS"). Log audit of the AU_LEAD reps: in the GOOD
  r0.5 reps commit NEVER fires (extent maxes 241–296 < 400 → regulates to touchdown → the 0.35 m
  residual is the v·τ curved-translation lag, NOT a regulation stop). But the FAILURE reps drive the
  drone close+tilted → extent 405–416 → commit/abort DOES fire: r1.0 fly-aways = case(a) ABORT
  (diverging); raw-lead miss = case(b) COMMIT "zeta_r zeroed" — freezing tracking on a moving deck.
  So commit isn't the good-residual cause but is plausibly implicated in the variance/fly-away reps.
- **✅ BAKED OFF 2026-07-03 (user): `PLASMC_TERMINAL_COMMIT` default 1→0** (controller.py:482).
  The open-loop hold is wrong for any moving target and neutral-to-better on the stationary
  platform (rarely fires, extent<400 at platform min-alt ~0.5 m). All trajectory montages
  (static/linear/circular/sinusoidal/eightshape/lissajous) now run commit-off by default; set
  `PLASMC_TERMINAL_COMMIT=1` to restore the stationary s_e_n→0 ramp. ⚠ validate: the stationary
  aruco-world (ground marker, no platform) descends to the deck where extent CAN reach 400 —
  commit-off means it regulates to touchdown instead of the open-loop hold; expected fine
  (regulating a static target to touchdown is correct) but not yet gate-checked at n≥5.
- **✅ CONFIRMED (user hypothesis): TERMINAL_COMMIT=0 ELIMINATES the moving-target fly-away.**
  r1.0 (loose clamp, the one that had a fly-away) commit-ON peaks 5.0/**9.2**/5.0 (1 detonation)
  → commit-OFF peaks **5.0/5.0/5.0** (NONE); the 60 m fly became a bounded 1.5 m miss. MECHANISM
  (user, code-confirmed controller.py:1605-1646): on COMMIT the surface drops zeta_r →
  `sigma_xy = zeta_h + Omega*izeta` = FLOW/velocity regulation only, POSITION dropped. Velocity-
  matching HOLDS center for a CONSTANT-velocity (linear) target but NOT a ROTATING-velocity
  (curved) one → the open-loop hold is architecturally WRONG for a moving/curving deck. (Caveat:
  in the GOOD reps commit never fires anyway — platform min-alt ~0.5 m keeps extent <400 — so this
  fixes the FAILURE mode, not the good-rep residual; and the linear-vs-curve landing difference is
  the v·τ lag, NOT commit, which fired in NEITHER good linear nor good curve reps.) The perception-ON
  analog of "never stop regulating" = KLT-track corners to touchdown instead of open-loop commit.
- **⭐ BEST CURVED CONFIG (first on-platform Circular of the campaign): AU_LEAD +35° (ω_z 0.9/ω_p 3.5)
  + AU_LEAD_RATIO 0.5 + TERMINAL_COMMIT=0 → 2/3 ON-PLATFORM (0.122, 0.269) + 1 NEAR (0.450), n=3,
  0 fly.** vs baseline 0/4 (1–1.7 m). Ratio 0.5 is an INTERIOR optimum (0.3 clips the mid-band lead
  → cycle back 1/3; 1.0 admits terminal instability → fly). Residual = the v·τ curved-translation lag.
  NOT baked — AU_LEAD default-OFF; needs the STATIONARY IC gate (+ IC1 regression) + higher n before
  any rover-scenario default-on. Data test_data/Rover_Turning/aulead_{ratio,commitoff,commitoff_r1}/.
- **STATUS: PROMISING, NOT baked.** Residual at r0.5 = a ~0.35 m standing offset (the curved-
  translation v·τ lag, NOT a cycle — cycle is damped). NEXT: r0.3; then the MANDATORY STATIONARY
  IC GATE (AU_LEAD default-OFF so stationary is unaffected unless enabled — but any bake/default-on
  for the rover scenario needs the IC1-5 gate + IC1-regression check, same discipline as HD_KR).
  Data test_data/Rover_Turning/aulead_{sweep,clamp,ratio}/; harnesses + cycle_*.py tools in
  Rover_AB_harness/. Knobs: `PLASMC_AU_LEAD` (0/1), `_WZ` (0.9), `_WP` (3.5), `_RATIO` (1.0).

## ⭐⭐⭐ k_r RESOLVED (2026-07-02 late, user-led): HD_KR=0 REJECTED as a WRONG REFERENCE; the c3-noise hypothesis FALSIFIED surgically; defaults stand
**User catch (algebra):** the funnel-ref back-map is the prescription `ζ̇_r,d = −k_r·ζ_r`.
Setting k_r=0 prescribes `ζ̇_r,d = 0` ⇒ **ζ_r = const** (exactly: S_r held; with ṗ_r≈0 at
the funnel floor it degenerates to ṡ_e,d≈0 = FREEZE-THE-OFFSET). Off-center the velocity
barrier ζ_h then DAMPS the very closure velocity the σ-side χ_r·ζ_r commands — the two
channels CONTRADICT. k_r>0 is what makes them consistent (velocity reference demands the
exponential funnel-state decay the position term wants; s_e_n closes at k_lat=|h_rd|+k_r).
Same indictment applies to HD_PASSIVE (rate≡0, default-off). So k_r's purpose = consistent
closure demand through the velocity channel + the depth-free MOVING-target live recovery +
un-degenerating ζ_h off-center (mid-flight it is ~all of hd_rate; terminal-centered ~0).
- **Stationary cost confirmed before the stop:** HD_KR=0 GT-FB IC1 n=5 = **1/5 SP vs 4/5
  same-binary baseline** (0.104/0.563/0.133/0.196/0.061). Gate stopped (user); partial data
  preserved `ICValidation/20260702-224508`. HD_KR=0 will NOT be baked.
- **The "complete job" test (new env knob `PLASMC_DHD_SRC`, default `full`=06-29 behavior;
  `nokr` drops only the −k_r·ζ_r/g_r branch from dh_d, keeping k_r=0.5 live in h_d/h_e;
  `nos` = pre-06-29 s̈-drop of the whole rate term). Heading-hold Circular n=3 each:**
  - **nokr 0/3** (lat 1.64/2.16/6.78), e_rot ~1.02 — **cycle UNCHANGED although the c3
    band-rms dropped to hdkr_0's level** (offline spectral, 1.4–2.1 rad/s: c3_x 0.243→0.175,
    c3_y 0.189→0.101 vs hdkr_0 0.197/0.110; full baseline e_rot 1.11). ⇒ **the k_r-branch
    c3 noise is NOT the cycle carrier; the DF re-balances through the remaining branches —
    the single-source model confirmed surgically.** The 06-29 `_hd_src=full` "regression"
    framing is hereby CORRECTED: honest differentiation is load-bearing-neutral on the curve.
  - **hdkr_0's e_rot 1.11→0.88 win = DEMAND-LEVEL detuning** (the wrong reference lowers
    inward-velocity demand at a standing offset → loop gain down) — an operating-point move
    along the bias↔cycle frontier, not noise removal.
  - **nos 2/3 NEAR 0.343/0.335 + 1 catastrophic WANDER** (e_mean 78.6 m, lat 13.7) — the
    lost-restoring-stiffness tail class (cf. the P2INF=2.0 outlier). No winner.
- **DEFAULTS STAND: HD_KR=0.5 + PLASMC_DHD_SRC=full.** The curve-cycle residual remains on
  the −120° actuation-phase frontier; the c3/derivative route is now EXHAUSTED (both
  variants tested). Remaining exits unchanged: AU lead compensation (unapproved) or platform
  size. Tools: `Rover_AB_harness/dhd_arms.sh` + the c3 spectral check (scratchpad-born,
  band-rms of logged dh_d + GT e_x/e_y in the 0.8–3.5 m window).

Knobs: `PLASMC_GT_ALPHA_SIGN` (default +1; −1 falsified, diagnostic only),
`PLASMC_GT_SPIN_WZ` (synthetic in-place spin), `PLASMC_YAW_OMEGA_D_FF` (the windup fix),
`PLASMC_DHD_SRC` (full|nokr|nos, default full; both alternates tested-not-better).
Heading-hold recipe: PLASMC_YAW_GAMMA=0 KAPPA0=0 OMEGA=0 **N=0** (all four). Data
test_data/Rover_Turning/{yawff_spin_n2,yawff_circular_n2,yawhold_arm_n3,spin_hold_n2,
spin_active_n2,dhd_src_sweep,...}; harnesses in test_data/Rover_AB_harness/.
Continues [[project_moving_rover_landing_works]].
