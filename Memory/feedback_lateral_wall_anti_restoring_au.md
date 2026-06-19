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

**✅ ROOT FOUND (2026-06-19): TERMINAL 1/Z amplification of a residual lateral offset — NOT a fly-away.**
Brake is commanded AND delivered (cmd I_a inward −0.8 to −1.5; GT ACTUAL accel inward and LARGER, 4–14
m/s²) → rules out delivery, authority, sign, perception. The drone DESCENDS FINE from 5 m to ~0.6 m
staying reasonably centered (lat 0.5–1.4 m). The funnel breach happens at **alt 0.3–0.9 m with small lat
0.5–1.4 m**: `s_e_n = lat/Z` so a modest ~0.85 m residual offset at Z=0.6 m → `s_e_n≈1.4` (breach). The
1/Z blow-up of a residual offset near touchdown → controller reacts violently to the amplified error →
hard tilt → marker leaves FoV → TARGET_LOST → THEN the open-loop fly-away (the 7–40 m `fin_lat` numbers
are POST-marker-loss, not controlled divergence). So the "lateral wall" = (1) a residual lateral offset
(~0.85 m) never nulled during descent + (2) terminal 1/Z amplification + (3) violent reaction + FoV loss.

**LEVERS (grounded):** (a) null the lateral offset EARLIER / faster lateral bandwidth before terminal
(limited, [[feedback_convergence_ordering]] lateral×0.35); (b) TERMINAL COMMIT — below an altitude/
marker-extent proxy, stop reacting to the 1/Z-corrupted s_e_n and just descend (cf. CommitGate,
`test_data/CommitGate_*`); (c) widen the funnel / FoV margin near touchdown so the amplified s_e_n
doesn't trigger the violent reaction (cf. cbf2, THETA_FLOOR). The 1/Z amplification is intrinsic to
image features — the fix is to TOLERATE the residual offset terminally (commit) or null it before Z→0,
NOT more brake authority. Supersedes the "overshoot/fly-away" framing of
[[feedback_lateral_overshoot_root]]; still NOT perception. Diag on `test_data/Rotz_IC1_baseline`.

**TERMINAL-COMMIT GATE — MECHANISM VALIDATED, action wrong (2026-06-19, IC1 A/B `PLASMC_COMMIT_EXTENT=100`,
`test_data/Commit_IC1_*`):** the gate (latch on MARKER_EXTENT_PX>thr + 3-frame confirm, freeze s_e_n)
DEMONSTRABLY kills the 1/Z blow-up — commit reps `sen_max` 0.7–1.0 vs baseline 2.6–4.4 (post-commit std
0.000, cleanly frozen) → confirms the root diagnosis. BUT `FREEZE-AT-HELD` is unstable: the extent>100
trigger fires LATE (s_e_n already ~1.0 at the funnel edge), and freezing a LARGE value makes the lateral
loop apply a CONSTANT OPEN-LOOP push (no feedback) → unbounded drift (rep2 froze s_e_n=1.01 → 110 m,
climbed to 8.7 m). When it froze a small value (0.50) it landed clean (2.18 m). FIX = **FREEZE-AT-ZERO**
(commit ⇒ zero the lateral feature error, descend level, land at the residual ~0.85 m offset) — removes
the open-loop-push instability; trigger timing becomes non-critical. (Avoid clamp-at-p_s: r→1 collapses
the barrier G_s⁻¹ = the demand-starvation, [[feedback_plasmc_two_task_framework]].) Re-test pending.
`PLASMC_COMMIT_EXTENT` knob in controller.py `_updateImgFeatureParam`.
