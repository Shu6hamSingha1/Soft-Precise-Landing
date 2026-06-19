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
climbed to 8.7 m). When it froze a small value (0.50) it landed clean (2.18 m). FIX tried = **FREEZE-AT-ZERO**
(commit ⇒ zero the lateral feature error, descend level). `PLASMC_COMMIT_EXTENT` knob in controller.py
`_updateImgFeatureParam`; both freeze modes default-off (neither baked).

**⚠️ META-FINDING (2026-06-19): the lateral-wall outcome is STOCHASTICITY-DOMINATED → n=5 A/Bs are
UNDERPOWERED.** freeze-at-zero IC1 A/B: within-run baseline median max_lat 1.98 (1/5 fly) vs commit 3.15
(3/5 fly) — looks worse. BUT the BASELINE median across 4 identical-config runs swung **7.21 / 7.21 / 8.21
/ 1.98 m** — a 4× run-to-run swing with NO config change. This run's baseline got anomalously lucky; the
commit arm hit typical values. The gate still suppresses the blow-up (sen_max 1.0 vs 1.5) but neither
freeze variant CLEANLY lands (residual offset + velocity at commit → coast-out 2–3 m). **Conclusion: the
wall severity is dominated by a STOCHASTIC source (startup transient / initial drift), not the control
config — which is why the project's long history of lateral "fixes" is inconclusive.** To validate ANY
lateral fix needs n≥15–20 per arm (avg out the variance) OR identifying+controlling the stochastic source
(characterize the t=0 drift/transient). The terminal-commit mechanism is real (blow-up suppressed) but
unconfirmed as a net win. Don't bake; don't run more n=5 lateral A/Bs (noise). Root diagnosis stands.

**✅ TRANSIENT CHARACTERIZED (2026-06-19) — the stochasticity is PURELY TERMINAL, not t=0.** Pooled 20
IC1 baseline reps: t=0 state is tiny+consistent (offset 0.13 m, vel 0.04 m/s, s_e_n 0.02) and does NOT
predict the outcome (t=0-vel→max_lat corr +0.09; 60% fly either way). The startup-transient hypothesis is
FALSE. Lateral offset vs altitude on descent (n=19): **rock-solid 0.25 m ±0.3 from 5→2 m**, then grows
0.47 (1.5 m) → 0.61 (1.0 m) → **1.33 ±3.80 (0.5 m) → 2.44 ±10.10 (0.3 m)** — the variance EXPLODES only
below ~1 m = the 1/Z terminal window. So the lateral control works FINE during descent; the wall is 100%
terminal. ⇒ **Commit EARLY (~1.5–2 m, where offset is still ~0.25–0.47 m and tight) with freeze-at-zero
should COLLAPSE the variance** → deterministic precise landing within the small frozen offset. My earlier
commit (extent>100) fired too LATE (~1.0 m, offset already 0.6–1.8 m). FIX = lower `PLASMC_COMMIT_EXTENT`
(~50–70 px) to commit at ~1.5 m before the stochastic blow-up. Tradeoff: too-early = longer uncontrolled
descent (mild, offset stable to 2 m); too-late = 1/Z already grown. Target ~1.5 m. Re-test = early-commit
A/B, judge by VARIANCE COLLAPSE (all reps land tight) not just median. Diag: pooled `*_IC1_baseline`.

**⛔ FREEZE-s_e_n IS A DEAD-END (2026-06-19, 0/3 variants).** Early-commit A/B (`PLASMC_COMMIT_EXTENT=50`,
commits @~1.3 m, freeze-at-zero): CATASTROPHIC — td_lat STD 4.95→**28.75** (variance EXPLODED not
collapsed), one rep flew **64 m**, another path-peaked 53 m. `sen_max` low (0.2–1.2) CONFIRMS the gate
fires + freezes s_e_n — yet it flies away. ROOT WHY: **freezing s_e_n=0 LIES to the controller** — only
the outer lateral error is zeroed while the flow SMC (h_d depends on live s), the CBF, and the descent
coupling still see the REAL off-center features → the inconsistency destabilizes. All 3 freeze variants
fail: held-late (rep 110 m), zero-late (3/5 fly vs lucky baseline), zero-early (64 m). The 1/Z root is
real but FREEZING THE ERROR is the wrong fix. NEXT (if pursued): bound/attenuate the lateral COMMAND
(cap |V_ds_d_xy|, or taper K_rp near terminal) while keeping LIVE consistent s_e_n — never lie about the
error. Or accept the wall is a fundamental scale-free terminal limit. Stop firing freeze variants.

**✅ COMMAND-BOUNDING WORKS — first real lead (2026-06-20).** `PLASMC_COMMIT_DSD_MAX` caps |V_ds_d_xy|
once committed while keeping s_e_n LIVE (direction preserved, magnitude bounded → no controller
inconsistency, unlike freeze). IC1 A/B (`PLASMC_COMMIT_EXTENT=50 PLASMC_COMMIT_DSD_MAX=0.6`, n=5):
td_lat **STD 21.35→3.39 (6× collapse)**, **max fly 55→10 m** (tails tamed), flyaway 2/5 both, median
1.64→2.69 (baseline got a lucky median again). FIRST intervention that REDUCES variance instead of
exploding it (all freeze variants exploded it). Per-rep: 4/5 commit@~1.32 m and the cap BINDS perfectly
(post-commit ds_d max 0.59–0.60 = the cap) → 3 land well (0.49/1.77/2.69 m), 1 borderline (3.16). The
one bad rep (10.23 m) was a SPURIOUS EARLY commit (extent-spike latched @1.89 m, marker-switching noise).
Cap motivated by data: terminal ds_d spikes 4–6.6 vs controlled-descent ~0.5; cap 0.6 clips spikes,
keeps normal correction. NEXT to push to a clean win: (a) robustify the commit trigger vs extent-spikes
(higher/smoothed threshold or require extent monotone) to kill the spurious-early-commit failure; (b)
small cap sweep (0.4 / 0.8); (c) larger n (≥15) + IC2-5 gate before baking. Default-off; not baked.
controller.py `_updateImgFeatureParam` + the DSD cap site. Supersedes the freeze dead-end.
