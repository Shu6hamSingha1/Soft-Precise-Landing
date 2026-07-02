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

**NEXT (in order):** (a) n≥3 the heading-hold and sign-flip arms — decide whether they
actually differ or are one noisy distribution; (b) if heading-hold tracks poorly, test
w_z=0 for the target-spin component (or CTRL zero-w_z variant) to isolate mechanism 2;
(c) yaw-rate feedforward design for genuine heading TRACKING (needed if the mission
requires touchdown alignment on a rotating platform; position-only landing may be
acceptable otherwise — the platform is square, marker π/2-symmetric for ArUco decode!).
Note: perception-ON on a turning rover ALSO needs the alpha-rate cap raised
([[feedback_rover_yaw_cal_resolved]]).

Knob added: `PLASMC_GT_ALPHA_SIGN` (gt_feedback.py, default +1 = unchanged; −1 = the
falsified flip, kept as a diagnostic). Heading-hold recipe: PLASMC_YAW_GAMMA=0
PLASMC_YAW_KAPPA0=0 PLASMC_YAW_OMEGA=0 **PLASMC_YAW_N=0** (all four).
Continues [[project_moving_rover_landing_works]].
