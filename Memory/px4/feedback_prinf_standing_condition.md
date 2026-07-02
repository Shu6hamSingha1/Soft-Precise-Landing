---
name: feedback_prinf_standing_condition
description: "s_e_n off-center convergence lever = restore the proof Standing-Condition-1 p_r_inf>=1 (the 2026-06-29 bake violated it at PRINF=0.8); k_r=0 passive-h_d is the WRONG direction (references zeta_r GROWTH under funnel contraction)"
metadata:
  node_type: memory
  type: feedback
  originSessionId: a378d3e9-67aa-42fc-ae09-63da27f370a9
---

**2026-06-29. The lever for off-center `s_e_n` convergence is the funnel STANDING CONDITION, not the h_d rate term.**

## The k_r=0 (passive h_d) dead-end — and WHY (user-led analysis)
Tried `PLASMC_HD_PASSIVE=1` (new knob, controller.py: `_hd_rate=0`, h_d = passive rotation/descent
FF only — the literal STACKED_BARRIER_BACKSTEPPING.md "no desired-rate term" design). **WRONG
direction.** h_d's rate term sets the implied desired bearing rate `ṙ̄_e,ref`; via
`ζ̇_r = g_r(ṙ̄_e − S_r·ṗ_r)`:
- back-map `k_r>0` (BAKED HD_KR=0.5): `ṙ̄_e,ref = S_r·ṗ_r − k_r·ζ_r/g_r` ⇒ `ζ̇_r,ref = −k_r·ζ_r` (monotone exp decay ✓, CONSISTENT with the ASMC surface objective ζ_r→0).
- passive `k_r=0`: `ṙ̄_e,ref = 0` ⇒ holds `r̄_e` const, but `p_r` CONTRACTS ⇒ `S_r=r̄_e/p_r` rises ⇒ **`ζ̇_r,ref > 0` (ζ_r GROWS toward the edge)** — OPPOSITE the surface objective. The closing motion then has to be made entirely by feedback (ζ_h rides −χ_r·ζ_r; closing accel unmodeled by c-term −ḣ_d → dumped on adaptive κ, lag-sensitive). It's a RACE: feedback closing vs the contracting funnel. n=2 GT-FB: passive helped the LONG descent (IC4, more runway, rep1 textbook maxSr 0.32 no breach) but HURT the short one (IC2 terminal balloon 10–128). So `HD_KR>0` is vindicated as the CONSISTENT reference.
- Reframe of the old "authority vanishes at S_r=0.648": `−k_r·ζ_r/g_r` is small near the edge but in ζ-SPACE the decay is uniform (−k_r·ζ_r) = correct kinematics (steep barrier → tiny r̄_e rate = large ζ change). It is NOT a vanishing CONVERGENCE drive; it lacks DISTURBANCE-REJECTION margin vs the terminal 1/Z near the edge. The precision "wall" was ICs being FORCED into the funnel edge.

## The actual lever: p_r_inf >= 1 (Standing Condition 1)
The combined-surface proof (Drafts/COMBINED_SURFACE_PROOF_ADDENDUM.md; controller.py:255 comment)
requires **`p_r_inf >= 1`** (funnel bottoms at the FoV, never sub-FoV) so contraction can't shove a
bounded `r̄_e` toward the edge → ζ_r stays bounded → CBF→funnel transfer exact. **The 2026-06-29 bake
(787cf2d) VIOLATED it: `PLASMC_PRINF_X/Y = 0.8`** ("terminal-approach", was 1.0). That sub-FoV
contraction MANUFACTURES the edge-divergence — regardless of k_r.

**A/B GT-FB IC2/IC4 n=3 (`PLASMC_PRINF_X=PLASMC_PRINF_Y=1.0` vs baked 0.8), HD_KR kept 0.5:**
convergence SHAPE (maxSr / breach-onset / terminal s_e_n), the right metric (endpoint xy conflates the
2nd softness wall):
- **IC4 (long): essentially solved.** r1 maxSr 26.2→0.29 (breach 0.86→NEVER, term 6.6→0.0); r2 26.8→3.75; r3 7.5→0.54 (NEVER breach). 2/3 keep s_e_n INSIDE the funnel the whole flight, min≈0. Endpoint xy 0.8-arm 2.16/2.48/0.39 (all-3 mean 1.68; the "~2.4" = mean of the two failing reps only — audit 2026-07-02) → 1.0-arm ~0.35 (0.30/0.44/0.32).
- **IC2 (short): improved but still stochastic.** r2 clean (maxSr 1.33→0.40, no breach); r1/r3 still terminal-breach (maxSr ~22–24, term 10–18) — the genuine terminal 1/Z disturbance on the 5m descent (least runway) = the orthogonal SOFTNESS wall ([[project_bake_and_sp_walls]] wall #2; loom-commit territory).

## ⭐ FUNNEL-SHAPE SWEEP (GT-FB IC2/IC4 n=2) — PR0=10 is the breakthrough
The s_e_n convergence is governed ENTIRELY by whether the funnel forces the error toward its steep
edge (ζ_r into the saturating regime). Keep ζ_r out of the edge → converges to the lag floor:
- **PRINF=1.0 + PR0_X/Y=10.0 (slow default XIR=0.10): the WINNER.** All 4 reps maxSr≈0.05 (s_e_n at
  ~5% of the funnel the WHOLE flight), NEVER breach, terminal s_e_n=0.0, min 0.000. Endpoints
  **IC2 0.136/0.151, IC4 0.138/0.221 m**, rel_vel 0.28–0.37 (= the GT-FB lag floor xy≈0.2/vel≈0.37).
  Flight 17–20s (gentle). Wide initial funnel ⇒ S_r tiny ⇒ ζ_r never steep ⇒ no edge-forcing ⇒ no
  terminal 1/Z balloon. Closes the precision wall on BOTH ICs. (SP flag still 0 only b/c xy just above
  the 0.1m "precise" threshold — architectural floor, not a wall.)
- PRINF=1.0, PR0=1.2 (default): IC4 good (maxSr<1), IC2 stochastic (the n=3 A/B above).
- **PRINF=1.0 + XIR_X/Y=1.0 (10× fast contraction): REGRESSION.** Snaps the funnel tight → S_r rises →
  re-enters the edge → terminal breach (maxSr 5–15, term 2.5–8), descent too fast (IC2 ~4.5s),
  endpoints 0.43–1.36. Confirms the mechanism from the opposite side.
- PRINF=0.8 (sub-FoV): contracts past FoV → edge-forced → breach (the bake bug).
- **PRINF=1.0 + PR0=10.0 + XIR_X/Y=1.0: PR0=10 does NOT rescue fast contraction.** 3/4 breach (maxSr
  0.6–20, term 0.4–8.7), endpoints no-land/0.32–1.79. With XIR=1.0 the funnel contracts e^{−1.0·t} →
  collapses from 10 to the p_r∞=1 floor in ~2–3s WHILE the error is still large → S_r jumps → steep
  edge → breach. The wide PR0 start is irrelevant once snapped shut. (Only IC4r2 converged fast enough.)
- **PRINF=1.0 + PR0=10.0 + XIR_X/Y=0.5: CLEAN (4/4, no breach, endpoints 0.28–0.33).** Refines the
  mechanism: terminal p_r DID collapse to ~1.2–1.3 (as the e^{−0.5·T} math predicts) — yet NO breach,
  because terminal |s_e_n| stayed ~0–0.5 (error converged early). So absorption isn't the whole story.
**TWO protections, EITHER suffices:** (1) WIDE TERMINAL funnel absorbs the 1/Z spike (XIR=0.10:
p_r(T)=4.4); (2) EARLY CONVERGENCE — funnel wide LONG ENOUGH for lat to null before the 1/Z zone → no
residual to amplify (XIR=0.5: |s_e_n|≈0 terminally despite narrow p_r). XIR=1.0 fails BOTH (collapses in
~2s → too tight too soon for clean convergence AND too narrow to absorb). Requirement: **funnel stays
wide long enough RELATIVE TO the convergence time** (ξ_r·T small over the early phase); 0.10 & 0.5 pass,
1.0 fails. NOT more authority. The surface χ_r·ζ_r does gentle non-saturating restoring while flow
control centers. BEST = PR0=10, PRINF=1.0, XIR=0.10. **VALIDATED XIR=0.10 > XIR=0.5** (existing data,
n=2 each / 4 reps, both 0/4 breach): XIR=0.10 dominates on BOTH precision (xy 0.136–0.221 vs 0.276–0.333,
CLEAN non-overlapping separation) AND softness (rel_vel 0.28–0.37 vs 0.61–1.53). Slower XIR keeps the
funnel wide longer → gentler ~3s-longer descent (T 10s vs 7s) → softer touchdown + fully-settled s_e_n
(term 0) + wide terminal p_r (4.4 vs 1.3). XIR=0.5 is SAFE but rushes the descent → harder, looser. So
0.5 is NOT an improvement; keep XIR=0.10 (default).

## PRINF is INERT once PR0 keeps the funnel high; IC5 canary lands clean (GT-FB n=2)
- **PRINF=0.8 + PR0=10 + slow XIR=0.10: CLEAN (0/4 breach, xy 0.09–0.21, 2 precise).** Terminal p_r≈3.8–4.8
  — the funnel NEVER reaches the 0.8 floor (slow XIR decays 10→~4 only). So the sub-FoV floor is INERT:
  the standing-condition violation that breached in the 787cf2d bake only bit because PR0 was SMALL (1.2) →
  funnel sat AT the floor. With PR0=10 the funnel rides high, PRINF is moot (never approached). PRINF
  matters ONLY when the funnel actually contracts down to it (small PR0 or fast XIR).
- **IC5 (3m short-descent canary) lands CLEAN with PR0=10** — 0 breach, maxSr 0.08–0.18, term p_r≈5,
  both floors: PRINF=1.0 xy 0.36/0.37, PRINF=0.8 xy 0.09/0.16 (PRINF=0.8 looks tighter but term p_r equal
  → n=2 noise). Notable: IC5 was a notorious campaign failure; the funnel-shape config extends the win to
  the hardest IC.
- WHY XIR=0.10 beats 0.5 (mechanism, data-grounded): ξ_r·(PR0−p_r∞) = the funnel SQUEEZE VELOCITY (the
  −S_r·ṗ_r inward-velocity demand in the flow ref). PR0=10 AMPLIFIES it: XIR=0.10→−0.9/s, XIR=0.5→−4.5/s
  (5×). The SMC chases the 5× demand with hard lateral accel (a_u_xy 35 vs 4) + tilt (44° vs 18°); tilt
  STEALS vertical thrust (B_T·cos44°=0.72 → under-braked) → terminal |vz| 3.4 vs 1.5 (HARD touchdown) +
  faster fall (T 7 vs 10s, h_rd identical) + lateral OVERSHOOT (s_e_n max 0.78 vs 0.58) → looser xy. So
  with a wide PR0, ξ_r is the LATERAL-AGGRESSION dial and aggression couples into the vertical via tilt →
  softness & precision are the same gentle-squeeze coin. One-knob-one-job: keep XIR slow.

## ⭐ SP MECHANISM (all PR0=10 configs analyzed) — precision SOLVED, SP is now a LATERAL-VELOCITY softness wall
SP = precise (xy≤0.1) AND soft (rel_vel≤0.2, 3D speed). Decomposing terminal GT velocity (vlat vs vz)
across ALL PR0=10 runs:
- **Precision SOLVED** by funnel SHAPE (wide PR0 off the edge → s_e_n→0): PRINF=0.8+XIR=0.10+PR0=10 hits
  xy<0.1 (3 reps 0.086–0.093). PRINF=0.8 = most precise (the lower floor permits a tighter terminal target,
  and is SAFE because PR0=10 keeps the funnel high so it's never reached during descent).
- **Vertical softness SOLVED**: vz≈0.05–0.2 at deck (h_rd+loom brakes vertically).
- **LATERAL velocity is THE SP wall**: vlat≈0.5–1.7 in EVERY rep (>> vz). The drone reaches the target
  POSITION but SLIDES laterally through it. rel_vel ≤0.2 achieved by ZERO reps (best 0.255).
- Mechanism: σ=ζ_h+χ_r·ζ_r drives ζ_r→0 = POSITION-centered, NOT velocity-zero → drone crosses center with
  residual inward velocity (under-damped approach: xy<0.2 but crossing-velocity ~0.6). vlat SCALES with
  squeeze aggression (XIR=0.5→vlat~1.5, XIR=0.10→~0.6) — the gentle-squeeze lever minimizes it but bottoms
  at the 38ms-LAG FLOOR (memory GT-FB ceiling rel_vel≈0.37; best reps 0.255–0.28).
- **To get SP = arrest terminal LATERAL velocity** (position/authority is no longer the problem): (1) XIR
  even gentler (0.05) — cheapest in-framework test, diminishing; (2) strengthen ζ_h velocity-damping half
  (lag-limited); (3) descent reference-governor h_rd→0 near deck (buy time to null vlat while holding
  center — architecture CAN hold zero lateral vel, can't brake a moving one in time); (4) the 38ms lag (DDS)
  = the hard floor ~0.37. #1 immediate, #3 principled cause-side.
- **REDUCED chi_r is NOT a softness lever (tested {1.5,1.0,0.5}, PR0=10 PRINF=0.8 XIR=0.10, IC2/IC4 n=2):**
  1.5→1.0 FLAT (rel_vel 0.387→0.395, maxSr 0.05) — touchdown vel already at the 38ms-lag floor, not
  chi_r-limited; 1.5→0.5 REGRESSES (rel_vel→0.699, 2 reps 0.82/1.17) with the UNDER-DAMPED HUMP returning
  (maxSr 0.05→0.14 on exactly those reps). chi_r's DUAL role (position gain in σ + velocity damping
  χ_r·ζ̇_r in u_eq): below ~1.0 it strips damping faster than it gentles aggression, ζ_h can't hold the
  surface → lateral rings → HIGHER terminal vel. chi_r=1.5 is a PLATEAU optimum (2.0 over-drives, <1.0
  under-damps). Even funnel-ref-un-degenerated ζ_h doesn't carry enough damping to let chi_r drop. So the
  lateral softness floor is the lag, not chi_r.

## ⭐ VELOCITY PROFILE — the softness oscillation is a TERMINAL RE-ACCELERATION, not an approach failure
GT v_lat by altitude band (PR0=10 PRINF=0.8 XIR=0.10, all 6 reps consistent): approach >1.5m v_lat~1.0-1.2
(converging) → BRAKES to a MINIMUM at 0.4-0.8m (v_lat 0.07-0.39, near-stop, centered) → RE-ACCELERATES in
the terminal <0.4m (v_lat 0.24-0.81, max ~2.3) — EVERY rep. The drone DOES brake laterally fine; the
softness gap is the terminal re-excitation.
CAUSE = 3 converging signatures (terminal 4s): (1) 1/Z amplification — latpos ~0.19m / Z<0.4m → s_e_n=lat/Z
blows up → command spike (EA_d commanded tilt 44.8°); (2) inner-loop attitude lag — e_R≈EA_d (ratio 1.01,
both ~45-48°) → attitude can't track the spike → overshoot (38ms cascade mismatch); (3) σ_xy rings INSIDE
the boundary layer (|σ|/E=0.91<1) → sat() linear → robust switching damping LINEARIZED AWAY → overshoot
rings not decays → ~2Hz limit cycle (vx 9 sign-changes/4s, mainly x). Chain: terminal 1/Z command spike →
lag-limited inner loop overshoots → un-damped (BL-linearized) switching rings → re-accelerates the centered
drone → v_lat 0.3→0.8. IMPLICATION: approach-shaping gains (XIR/chi_r/Gamma) change v_lat ABOVE ~0.5m but
CAN'T touch the terminal 1/Z re-excitation (why chi_r flat, Gamma predicted flat). THE fix = descent
reference-governor (h_rd→0 near deck): hold descent at ~0.4m where v_lat already 0.1-0.3 → stops Z→0 blowing
up s_e_n + time to null residual → terminal v_lat 0.8→~0.2. The principled SP-softness fix (targets the
actual mechanism, unlike the gain levers).

## ⭐ GAMMA (reaching gain) is a REAL softness+precision lever in the funnel-shape regime — LOWER is better
Sweep GAMMA_xy {0.25,0.5,1.0} (PR0=10 PRINF=0.8 XIR=0.10, all 3 axes pinned to dodge the auto-align
revert-hot ENV TRAP, Z=0.75; GT-FB IC2/IC4 n=2): terminal v_lat scales MONOTONICALLY with Γ — 0.25→0.23,
0.5→0.31, 1.0→0.63. **GAMMA_xy=0.25 = best config yet: xy mean 0.087 (3/4 precise), rel_vel 0.38, vlat_term
0.23.** MECHANISM: the limit-cycle forcing is the proportional reaching term a_u=−Γσ, and σ IS the
limit-cycle variable (rings in the boundary layer terminally, hits 3.66 ceiling in ALL arms) → lower Γ =
smaller command response to the ringing σ = smaller terminal kick = softer AND more precise. REGIME FLIP:
old back-mapped wall needed HIGH Γ (2.0) for REACHING; in the funnel-shape regime the funnel handles
convergence so aggressive Γ just FEEDS the terminal cycle → optimum flipped high→low. CORRECTS my prediction
(I expected Γ flat/approach-only — wrong; Γ scales the terminal cycle because σ spikes terminally).
CAVEATS: rel_vel 0.38 still > 0.2 soft (lag floor — needs descent governor to close); GT-FB ONLY (lower Γ =
less robustness → perception-ON noise rejection unverified); n=2. Trend monotonic → test lower (0.125/0.0625)
for the floor (where reaching fails → precision degrades). First lever to move softness (chi_r flat,
K_R rate-saturated terminally). **GAMMA=0.25 = the FLOOR/optimum, BAKED (controller.py:277 auto-align
[0.25,0.25,0.75] symmetric; PR0=10 also BAKED controller.py:260).** Gamma-lower probe {0.125,0.0625}
REGRESSES: clean reps stay tight (xy 0.034/0.036 precise) but VARIANCE EXPLODES (g0.125 mean rel 1.02 w/ a
3.21 fly @IC4; g0.0625 mean 0.72 w/ 1.23 blowup) — below 0.25 the reaching gain is too weak → SMC can't
reliably reach the surface → stochastic under-correction/fly. So 0.25 = sweet spot (low enough to minimize
LC forcing, high enough to reach). ⚠ All GT-FB n=2 — perception-ON + n>=5 + IC1-regression validation
PENDING (queued).
- **LIMIT-CYCLE CHARACTERIZATION (gamma sweep, terminal 1.5s):** Γ sets AMPLITUDE, loop sets FREQUENCY.
  a_u_x amp 3.8/5.6/6.6 + vx p2p 1.02/1.71/1.97 rise monotonically with Γ, but σ_x amp ~const (5.4/4.9/6.3)
  → it's the GAIN on σ (a_u=−Γσ), not σ ringing harder. Frequency ~0.7-1.0 Hz, nearly Γ-independent (even
  DECREASES 0.99→0.68 — saturating-LC signature: bigger amplitude→lower describing-fn gain→slower). So
  freq = plant/lag property (double-integrator+38ms+1/Z, only the descent governor moves it via capping
  1/Z); amplitude = forcing property (Γ moves it). NO clamp involvement (w_u 2-6%, EA_d 17-19° vs 60cap,
  κ_z 0.06 — all inert/equal across arms → the Γ effect is clean LINEAR, not a clamp artifact). The VIOLENT
  single kicks (EA_d 44°, w_u 38% clamp) are SEPARATE stochastic 1/Z spikes seen ONLY in the VDF-default
  config (2/24 reps), distinct from this sustained Γ-scaled cycle. ⇒ Γ (amplitude) + descent governor
  (frequency/forcing) are ORTHOGONAL+COMPLEMENTARY; stack both to get vlat<0.2 (neither alone: Γ lag-floored,
  governor leaves residual amplitude).
- **PER-AXIS: cycle is LATERAL (x,y) ONLY; yaw is STABLE — the control experiment isolating 1/Z.** x and y
  limit-cycle INDEPENDENTLY (per-axis θ decouples them; different freqs 0.7-1.35Hz, same band; amplitude
  Γ-scaled on both; symmetric in GT-FB = NO hot axis, the "x 1.39× hotter" is a perception/cal asymmetry
  absent with perfect features). Cycle lives in VELOCITY at low Γ (s_e_n amp ~0, funnel holds position) →
  breaks into position at g1.0 (s_e_n amp 0.1). YAW does NOT cycle: e_a terminal 2°/amp 1.3°, sign-changes=2
  (single overshoot), u_a≈0.02. WHY = yaw alpha is a depth-free BEARING (no 1/Z), so NO terminal gain ramp +
  YAW_OMEGA=0.1 tamed its double-integrator. All 3 channels share double-int+38ms lag; ONLY x/y are
  1/Z-normalized; ONLY x/y cycle → DIRECT corroboration that the 1/Z ramp is THE igniting ingredient (yaw =
  the depth-free control case) → confirms the descent governor (caps 1/Z) is correctly targeted.

## E_xy (boundary layer) — control-framework-audit candidate: removes bang-bang, NOT the loop
Framework audit (full a_u law read): every component is forcing (Γ/κ/c-term/h_d), optimized (funnel/χ_r/θ),
harmful-to-change (p_h tighten, filters add lag), or band-aid (caps) — EXCEPT E_xy (boundary layer), which
controls whether sat(σ/E) is continuous vs BANG-BANG. σ_xy hits 3.66 ≫ E_xy=1.0 → 31% out-of-layer →
bang-bang. Sweep E_xy {2,3,4} (cap OFF, Gamma=0.25/PR0=10 baked, GT-FB IC2/IC4 n=2): **CONFIRMS the audit
but PARTIAL.** Bang-bang frac 30%→12%→14%→**1%@E=4**; GT-terminal vlat 0.38→**0.19@E=4**; κ_x 1.13→0.64
(tames κ-runaway); 0/4 fly at every E (NO robustness loss, unlike low Γ). BUT touchdown **rel_vel stays
~0.5 (flat 0.50-0.56)** — doesn't reach SP-soft 0.2. So E_xy removes the BANG-BANG (ingredient 4) but NOT
the ~1Hz linear loop (ingredients 1-3, the 1/Z gain ramp) — the bang-bang was ENERGIZING the cycle (removing
it halves GT-terminal amp + tames κ) but the loop persists. Analog of the proven E_z=3 loom-cycle fix.
E_xy=4 = clean value (1% bang-bang). VERDICT: E_xy is a useful COMPLEMENT (smoother, perception-friendly,
tames κ, free on robustness) but NOT the SP fix. The 3 orthogonal pieces: E_xy (remove bang-bang) + descent
governor (remove 1/Z loop = the headline) + Γ=0.25 (minimize forcing). ⚠ all GT-FB cap-OFF.

## N-raising REFUTED — second κ-runaway mechanism (adaptation-loop overshoot), confirms κ_eq (not slew) is binding
Hypothesis (user): E_xy=4 bounds σ → suppresses the σ-driven κ-runaway (θNG|σ|) → safe to raise N → faster
κ adaptation closes the 857× slew gap. TESTED N_xy {0.5,1.0,2.0} @ E_xy=4 cap-off (GT-FB IC2/IC4 n=2):
REFUTED. κ_max 0.64→1.83→5.39→**8.97** (runs away) DESPITE bang-bang staying LOW (2-7%, σ IS bounded by
E=4). All outcomes REGRESS: rel 0.50→1.48, xy 0.13→0.65, vlat 0.19→0.50, 1 fly @N=2.0, prec 2/4→0/4.
MECHANISM: κ/κ_eq climbs 1.5→**3.0** — κ OVERSHOOTS its own equilibrium. The fast κ-ODE chasing a κ_eq that
slews at 1260/s OVERSHOOTS and ratchets → the ADAPTATION LOOP becomes its own oscillator (a DIFFERENT
runaway than σ-out-of-layer; E_xy can't touch it). DEEP LESSON: closing the slew gap (N) DOESN'T help
because κ_eq is UNBOUNDED+fast — confirms the binding constraint is the unbounded κ_eq (1/Z disturbance
MAGNITUDE), NOT the adaptation SPEED. Can't out-ADAPT an unbounded disturbance. Reinforces: governor (bound
1/Z → bound κ_eq) is the only lever; P↓ (raises κ_eq) is a dead-end (can't reach ∞ + re-opens runaway).
N=0.1 stays optimal. The 2 κ-runaway mechanisms: (1) σ-out-of-layer θNG|σ| [E_xy fixes], (2) fast-N
adaptation-overshoot of slewing κ_eq [E_xy can't fix].

## Status / next
STRONG directional (n=3, large + mechanistically predicted, not noise) — NOT yet baked. Gate:
n>=5 IC1-5 + IC1 regression (why 0.8 was baked: check center-precision cost of p_r_inf=1) before
re-baking PRINF 0.8→1.0. `PLASMC_HD_PASSIVE` left in code default-OFF as a diagnostic knob (documents
the inconsistency); can be removed. Refines [[feedback_sen_authority_analysis]] (the 0.648 collapse is
edge-forcing, not intrinsic), [[project_stacked_barrier_backstepping]] (the design's "no desired-rate"
is the inconsistent variant; keep HD_KR>0 + p_r_inf>=1).
