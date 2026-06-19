---
name: feedback_lateral_wall_anti_restoring_au
description: "⭐ (2026-06-19) The lateral wall is NOT perception in the overshoot regime: on IC1 fly-aways the marker is DECODED 100% during the overshoot ONSET (decode collapses only AFTER the breach, as a consequence) AND the consumed flow tracks GT (ratio 1.0-1.46). So KLT/decode-availability can't fix the overshoot. The control side is a COMMANDED-but-NOT-DELIVERED gap, NOT a clean sign bug: in the WORLD frame the inertial command is weak/mixed (slightly outward in slow reps, INWARD/braking in the faster reps — rep4 I_a -1.46, corr -0.53) yet the drone drifts out. (An image-frame a_u·ŝ_e projection LOOKED 89% anti-restoring but that was a FRAME ARTIFACT — body-accel vs image-feature, camera-down Jacobian sign — corrected by the world-frame I_a-vs-GT check.) Lead candidate = D1 parity gap: a_u→inertial uses the FULL body DCM not rotz(yaw), mis-rotating the leveled command by tilt during the overshoot."
metadata:
  node_type: feedback
  type: feedback
---

**The lateral wall (overshoot regime) is NOT perception; the control gap is delivery, not a sign bug.**
Grounded on the 2026-06-19 IC1 baseline fly-aways (`test_data/Loom_IC1_baseline`, n=5).

**SOLID (verified) — perception is exonerated for the overshoot:**
1. **Decode 100% during the overshoot ONSET.** N-flow-corners aligned to control time via Image
   Stamp: converged-phase + the 1 s before the FoV-edge breach = 100% decode, 16–24 corners.
   Decode drops (40–67%) only AFTER the breach → consequence of the marker leaving FoV, not cause.
   → KLT corner-track / decode-availability ([[project_decode_availability_thread]],
   [[feedback_pyramidal_lk_inert]]) CANNOT fix the overshoot (those still hold for close-range descent).
2. **Consumed flow is ACCURATE while decoded.** meas |h| vs GT |V_h| over the decoded overshoot
   window: ratio 1.0–1.46 (slightly over if anything). NOT a flow under-report. Velocities are
   small (|flow|~0.2) — slow outward drift with good perception.

**CONTROL side — refined, with a CORRECTED method:**
- ⚠️ An image-frame projection `a_u·ŝ_e` read 89% "outward/anti-restoring" — **FRAME ARTIFACT.**
  `a_u` is a body/V-frame ACCELERATION, `ŝ_e` an IMAGE-feature direction; the camera-down image
  Jacobian flips the sign, so the naive dot-product is meaningless. Don't conclude from it.
- **World-frame check (unambiguous):** transform to inertial (logged `I_a`, NED) and compare to the
  GT target-relative direction. Result is WEAK/MIXED: slow reps cmd slightly outward (+0.36/+0.10,
  corr ~0); faster reps cmd INWARD/braking (rep3 I_a −0.24; rep4 I_a −1.46, corr −0.53) while GT vel
  is outward. So the brake IS commanded (at least in fast reps) but the drone drifts out anyway =
  **commanded-but-not-delivered**, not a clean anti-restoring sign bug.

**LEAD CANDIDATE — D1 parity gap (docs/CONTROLLER_PARITY.md row D1):** the V-frame `a_u`→inertial
transform uses the **FULL body DCM** `I_a = R@a_u − g` instead of MATLAB's **`rotz(yaw)`** (yaw-only).
`a_u` is a gravity-LEVELED V-frame vector; rotating it by the full tilted DCM mis-rotates it by the
current tilt (~17% cross-axis at 10°). The overshoot is exactly an aggressive lateral maneuver =
high tilt → the commanded inward brake gets mis-rotated → not delivered → drift grows. Documented
candidate fix: **use rotz(yaw) only** for `a_u`→inertial (`PLASMC_AU_ROTZ_ONLY`, controller.py:1177).

**⛔ D1 RULED OUT (2026-06-19 IC1 A/B n=5, `test_data/Rotz_IC1_{baseline,rotz}`):** rotz(yaw) did
NOT fix the wall — median max_lat 7.21→7.17 m (unchanged), flyaway 4/5→3/5 (n=5 noise), and rotz
added a 78 m blowup + a no-descent hover. The mis-rotation is real but not the binding mechanism.
Knob kept default-off (harmless, parity-correct). So the "commanded-but-not-delivered" gap is NOT
the V→inertial transform.

**OPEN — refined.** IC1 starts CENTERED (min_lat 0.00–0.07) and CANNOT HOLD: it drifts out and
flies away (centered hover is unstable), distinct from the IC2–5 "converge-then-overshoot." Remaining
candidates for the delivery/stability gap (D1 excluded): (a) inner-loop/PX4 body-rate tracking lag on
the brake (compare commanded I_a → commanded body-rate → GT achieved tilt/accel; impulse lag ~38 ms,
[[feedback_impulse_response]]); (b) authority (brake too weak to arrest the drift in time);
(c) outer-loop hold has no stable equilibrium at center. NEXT decisive diagnostic = commanded I_a vs
GT ACTUAL accel (not vel) — if I_a inward but GT accel outward → delivery (inner loop); if both inward
but small → authority. Don't fire more blind fixes; characterize delivery first. Still NOT perception
(solid: decode 100% + flow accurate during overshoot). Refines [[feedback_lateral_overshoot_root]].
