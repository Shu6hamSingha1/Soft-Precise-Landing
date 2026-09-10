---
name: feedback_adaptive_law_noise_behavior
description: "How the translational κ-ODE behaves under perception noise (2026-09-11). Growth term uses RAW rectified |σ|, no dead-zone → E[|σ_noise|]≈0.8·σ_n>0 is a standing forcing on κ that leakage only drains → κ_floor≈θ·G·0.8·σ_n/P, not 0. GT-FB σ_n≈0.002→κ decays; live perception σ_n ramps 0.006→0.09 xy (~1.6× w/ AU_LEAD)→terminal κ ratchet. FIX TESTED (PLASMC_KAPPA_DZ_* dead-zone on growth term): mechanically works (halves κ-growth) but does NOT improve landings — δ=0.06 net-negative on plain gate (IC4 cost 19→16/25) and does NOT rescue AU_LEAD (4/25 vs 2/25) because that's an upstream-of-κ HF-noise-amplification problem. Do NOT bake KAPPA_DZ; left in code default 0."
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 878fdadb-dd99-4085-bcf2-e19879f48082
  modified: 2026-09-10T20:15:28.393Z
---

**Why GT-FB validates the control path but is BLIND to the adaptive law's noise failure,
and what the κ-ODE actually does under perception noise. Precedes / motivates the
`PLASMC_KAPPA_DZ_*` dead-zone.** Context: [[feedback_aulead_stationary_regresses]] (AU_LEAD
regresses under perception, not GT-FB → the regression is perception-noise-mediated).

## The law (translational, per-axis; `controller.py` `_kappaSolver`, RK5)
```
dκ_i/dt = θ_i · N_i · G_i · |σ_i|   −   N_i · P_i · κ_i
a_v    ⊃  −θ · [ sat(σ/E) ⊙ G ⊙ κ ]           sat(σ/E) = clip(σ/E, ±1)
```
`N_xy=0.1, P_xy=2.5, E_xy=1.0` → `κ_eq = θ·G·|σ|/P`, `τ_κ = 1/(N·P) ≈ 4 s`. Descent is
~9 s so **κ is adaptation-rate-limited — never reaches κ_eq; it is a lagged leaky
integral of the growth term.** `σ = h − h_d` (optic-flow error).

## The vulnerability: raw rectified |σ|, no dead-zone / no filter
Perception noise `n` on the flow → `σ = σ_true + n`. Rectification gives
`E[|σ|] = E[|σ_true+n|] ≥ 0.8·σ_n` even when `σ_true ≈ 0`. So noise is a **constant
positive forcing** on κ; the only opposition is the leakage `−NP·κ`, a *proportional
drain* not a rejection → κ settles at a **floor, not zero**:
```
κ_floor,noise ≈ θ · G · (0.8 · σ_noise) / P
```

## Measured (2026-09-11, bundles ICValidation/{20260909-191246 perc-base, -193638 perc-AULEAD, -221339 gtfb-base, -224018 gtfb-AULEAD}; IC1-4 pooled, xy, HF std of σ = σ minus its 3 Hz LPF)

| set | window | σ_noise std | κ growth (END win) |
|-----|--------|-------------|--------------------|
| GT-FB (both arms) | all | **0.002** | −0.006 (κ **decays**) |
| perception baseline | MID→TERM→END | 0.006 → 0.037 → **0.057** | ≈ 0 (flat) |
| perception + AU_LEAD | MID→TERM→END | 0.007 → 0.054 → **0.093** | **+0.198 (ratchets)** |

- GT-FB: σ_noise ≈ 0.002 → κ_floor ≈ 0 → κ decays every window. **This is why GT-FB
  cannot see the problem** — no noise to rectify.
- Perception baseline: σ_noise ramps with descent (extent grows → centroid/flow noise
  grows); floor is real but small (~0.02–0.05), leakage contains it, κ ~flat. Benign.
- Perception + AU_LEAD: the ×3.9 HF lead inflates σ_noise **~1.6× at every stage**
  (→0.093), `sat(σ/E)` begins railing (0→1.7 % of END frames), κ flips from decay to
  **+0.198 growth** — plus a positive-feedback path (κ↑ → switching-cmd HF↑ → attitude
  jitter↑ → self-induced optic flow↑ → σ_noise↑). The "ratchet".
- **θ, G are near-inert here** (θ≈1.0–1.55, G≈0.8) — the amplified quantity is `|σ|`
  (rectified noise) itself. θ/G only blow up in a genuine funnel-edge divergence
  (GT-FB IC5: θ→17.8, growth term→13 — different regime, and one AU_LEAD *rescues*).

## Gain levers, ranked for this failure
1. **Dead-zone on the growth term** `(|σ|−δ)₊`, δ_xy ≈ measured σ_noise band (~0.06–0.08):
   zeroes the growth term when true error is inside the noise → κ_floor,noise → 0 like
   GT-FB, **without touching real-disturbance response** (δ ≪ any error that matters).
   Implemented as `PLASMC_KAPPA_DZ_{X,Y,Z}` (default 0 = bit-identical; z kept 0 — κ_z is
   terminal braking, rate-limited, |σ_z| is loom not lateral-flow noise). THE targeted fix.
   ⚠ mid-descent |σ| is also small (~0.06, funnel-contained) so δ≈0.07 gates convergence-
   phase adaptation too — expected OK because mid-descent convergence is Γσ-driven
   (Γ_xy=2.0), not κ-driven; κ's job is disturbance domination. Watch for κ leaking below
   κ_0 on a persistent sub-δ real drift (the [[project_ic1_kappa_leakage_drift_20260721]]
   failure mode) — if seen, switch to freeze-all-inside-δ (Peterson–Narendra dead-zone:
   `dκ/dt = 0` when `|σ|<δ`, leakage included) so κ holds instead of draining.
2. **P_xy** (leakage) 2.5→~4: `κ_floor ∝ 1/P`, ~40 % smaller. Also drains κ for REAL
   errors → the IC1 leakage-drift fly-away; τ_κ → 2.5 s. Bounded, known downside.
3. **N_xy** (adapt bandwidth) lower: slower integral filters HF |σ|; doesn't change the
   floor LEVEL (N cancels in κ_eq). Reopens "frozen κ" (0.02→0.1 was a deliberate bake).
   A σ-gated N ≈ soft version of lever 1.
4. **E_xy** (boundary layer) 1.0→~1.3: keeps noise in the linear `sat` region → no
   bang-bang → attenuates the feedback AMPLIFIER (not the rectification floor). ~1.7 %
   railing today, limited headroom.
5. `κ_max` clamp (30 xy): band-aid, already present, not a tuning lever.

## Validation RESULT (2026-09-11) — dead-zone works mechanically, does NOT help landings

4-arm, IC1-5 n=5, `WORLD=cross_marker`, perception, `CBF_DRIFT_TAU=0.15` (default):
A base (`20260911-003120`) / B `KAPPA_DZ_X=Y=0.06` only (`-005506`) / C `AU_LEAD=1
RATIO=0.5` (substitute: `20260909-193638`, env-flag bug skipped arm C this batch) /
D AU_LEAD+DZ (`-011851`).

| arm | precise | mean xy | END-win κ_grow (IC1-4) | END κ_peak |
|-----|---------|---------|------------------------|------------|
| A base       | **19/25** | 0.068 | +0.110 | 0.217 |
| B dz-only    | 16/25 | 0.072 (max 0.537) | **+0.053** | **0.123** |
| C aulead     | 2/25  | 0.159 | +0.198 | 0.266 |
| D aulead+dz  | 4/25  | 0.144 | +0.269 | 0.346 |

- **The dead-zone ENGAGED and did its job on the κ-ODE:** B vs A halved terminal κ-growth
  (+0.110→+0.053) and cut κ_peak (0.217→0.123). The rectified-noise-floor model + fix are
  mechanically correct.
- **But it does NOT improve landing outcomes.** B (16/25) ≈ A (19/25), slightly WORSE:
  IC1/IC2 tightened (xy ~0.03–0.05, 10/10 precise) but IC3 and especially **IC4 regressed
  (0/5, one 0.54 m / 1.47 m/s hard rep)** — δ=0.06 starves κ's legitimate terminal-braking
  build-up on the longer IC4 descent (the "adaptation-rate-limited, don't starve κ" caveat,
  biting on xy). Net wash-to-negative.
- **It does NOT rescue AU_LEAD:** D (4/25) ≈ C (2/25). Because — as
  [[feedback_aulead_stationary_regresses]] / the GT-FB test already established — AU_LEAD's
  regression is PRIMARILY the ×3.9 HF gain amplifying sensor noise straight into the
  command (upstream of κ) and into the `−θ·sat(σ/E)·G·κ` switching term at modest κ≈0.3.
  κ-growth is a SECONDARY symptom; gating it can't fix an upstream-of-κ problem. Under
  AU_LEAD, |σ| (0.18) ≫ δ (0.06) terminally so the dead-zone barely bites anyway.

**Verdict: the κ noise-rectification floor is REAL and the dead-zone provably suppresses
it, but it is NOT the primary lever for the AU_LEAD failure, and δ=0.06 is net-negative on
the plain stationary gate (IC4 cost). Do NOT bake `KAPPA_DZ` as-is.** If revisited: smaller
δ (0.03–0.04) or terminal-only / freeze-all engagement might keep the IC1/IC2 tightening
without the IC4 cost — but it is polish on a secondary lever with no urgent payoff. The
AU_LEAD perception regression still needs a PERCEPTION-QUALITY gate on the lead, not
adaptive-law tuning. `PLASMC_KAPPA_DZ_*` left in the code, default 0 (inert).

⚠ harness bug (`scratchpad/kappa_dz_ab.sh`): `env NAME=VAL -u NAME2` is invalid — `-u`
must precede all `NAME=VALUE`; arm C ran `-u` as a command and no-op'd. Fixed understanding,
not re-run (C substitute suffices; conclusion unchanged).
