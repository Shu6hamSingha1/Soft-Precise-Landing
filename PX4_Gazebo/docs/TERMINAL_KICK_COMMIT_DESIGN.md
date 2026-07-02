# Terminal-Kick Resolution — Approach & Commit Design

**Status:** finalized design, pre-implementation (2026-06-28, user-led, GT-FB).

> **⚠ STALENESS STAMP (2026-07-02 audit) — partially implemented + partially superseded:**
> (1) "pre-implementation" is stale — the §6a framework WAS built; the Part-1 approach knobs were then
> **BAKED ON** (`HD_FUNNEL_REF=1` + `HD_KR=0.5`, 787cf2d) with `PRINF` corrected 0.8→**1.0**
> (Standing-Cond-1, [[feedback_prinf_standing_condition]]) — the provisional `P2INF_xy≈0.12` floor
> direction was dropped (tight-funnel edge-forcing; baked value is **1.0**, 486f713), and `HD_KR=1.0`
> tested WORSE (NC173; keep 0.5). `PLASMC_TERMINAL_COMMIT` is **baked ON** (user decision, 06-30);
> `LANDING_COMMIT_EXTENT` stays rejected/off. (2) The root-cause was REFINED on 06-30: the violent GT-FB
> kick was **substantially the Z_REG=0.01 harness artifact** (fake 1/z below the gear floor,
> [[feedback_zreg_gear_floor_artifact]] → Z_REG=0.2 → 19/25 SP), and the rover-side kick was the
> **no-platform geometry** ([[feedback_rover_flyaway_no_platform]]). (3) §1's velocity discriminator
> SURVIVES the audit (s_dot_entry separates SP/non-SP with no overlap, [[project_why_sp_achieved]]) —
> the case(b)/case(a) **s_dot commit/abort gate remains the open robust-fix spec** for the residual
> ±5–7-SP stochasticity. Still open from §7: the case-(a) re-ascend module + the perception-ON
> ring-handoff validation (§6b trap).
**Constraints honored:** `h_rd = −0.42` fixed; scale-free / depth-free (every trigger
quantity is image-space; Z is analysis-only, never in control).
**Supersedes** the open-loop loom-commit (`LANDING_COMMIT_LOOM`) as the primary terminal
handler, and the gate-OFF `funnel-ref + k_r` thread as prior art. See
`Memory/px4/feedback_terminal_kick_commit_vs_live.md`, `feedback_kappa0_unfreezes_lateral.md`.

---

## 1. Root cause (settled)

The terminal kick is **not** perception, convergence depth, or a 1/Z singularity:

> An **irreducible residual lateral velocity `v_res`**, normalized by a vanishing `Z` in
> `s_e_n = lat/(Z·p_10)`, forces an `s_e_n` breach → position barrier `ζ_r → ∞` → infeasible
> `a_u_xy` (~1000 vs ~17 deliverable) → max tilt → lateral fling + thrust-steal → fly-away.

`v_res` is **commanded, not passive**. Because `h_d = measured ṡ`, the desired lateral
velocity copies the actual one (`corr(||h_d||,||h||)=0.90`); `h_e ≈ 0`; no restoring command;
even dead-centered the controller asks for `h_d ≈ 0.11` instead of "stop." Proven chain:

```
v_res  →  h_e≈0  →  h_d≈h (copies flow)  →  h_d = measured ṡ (degenerate ζ_h)
```

**Equilibrium argument (why faster gain doesn't save it):**
`ṡ_e_n ∝ (|h_rd|−k_lat)·lat + v_res` settles to `lat* = v_res/(k_lat−|h_rd|) ≠ 0`, so
`s_e_n* = lat*/(Z·p_10) → ∞` as `Z→0`. The binding quantity is **`v_res`** (residual
*velocity*), not residual position. Faster `k_lat` only shrinks `lat*`, never zeroes it.

**Data confirmation** (620 Landing_Test reps, @ corner-exit ~0.5 m):
- position `|s_e_n|` does NOT separate outcomes: precise 0.163 vs failed 0.159.
- **velocity does:** `v_res` precise 0.067 vs failed 0.234 (p90 2.18); `|ds_e_n|` precise 0.061
  vs failed 0.252 (p90 2.17); `||h_xy||` precise 0.132 vs failed 0.390.

---

## 2. Strategy — two complementary parts

| Phase | Altitude | Goal | Mechanism |
|---|---|---|---|
| **Approach** | above corner-exit | minimize `v_res` *before* the deck | restore genuine convergence authority in `h_d` |
| **Commit/handoff** | at corner-exit (~0.5 m, marker leaves FoV) | remove the kick + degrade honestly | zero image-lateral control, hand to ring-flow |

They reinforce: better approach convergence → more reps commit (case b) vs abort (case a).

---

## 3. Approach phase — restore convergence authority

`h_d` must carry a real inward command that **vanishes at center**. The three options:

- `s_dot_meas` (current) → degenerate copy; no convergence, no damping.
- pure funnel-ref (`S_r·dp_r`) → prescribes `ζ̇_r = 0`, i.e. **holds `S_r` constant** (drops the
  `p_r·Ṡ_r` convergence term of the product rule `ṙ = p_r·Ṡ_r + S_r·ṗ_r`). Inert because the
  position funnel is near-static (`p_r: 1.2→1.0`, `ξ_r=0.10`, `dp_r≈0.01` ⇒ `_hd_rate≈0.0016`).
- **`HD_FUNNEL_REF=1` + `HD_KR=k_r` → restores `p_r·Ṡ_r` via `−k_r·ζ_r/g_r`** ⇒ inward command
  `≈ −p_10·k_r·p_r·(1−S_r²)·artanh(S_r)`: drives `S_r→0` **and →0 at center**. ✓

  ⚠ **Code coupling:** `HD_KR` only acts inside `if self._hd_funnel_ref` (controller.py ~1061).
  `HD_KR=1` with `HD_FUNNEL_REF=0` is silently ignored. Must set **both**.

  ⚠ The `k_r` convergence term rides into `dh_d` (controller.py ~1140, full `h_d` differentiated
  under funnel-ref); near the funnel edge its derivative can spike (clamped by `DH_D_MAX`).

**Settings:**
- `PLASMC_HD_FUNNEL_REF=1`, `PLASMC_HD_KR=k_r*` — value from the deferred sweep (k_r=0.3 confirmed
  > 0 on off-center; optimum TBD; old "k_r=1.0 destabilizes" was a *terminal* failure the commit
  now cuts off, so re-evaluate purely on convergence @ 0.5 m).
- `PLASMC_P2INF_X/Y → ~0.12` (sweep down to the lag-limited floor) — bounds `h_e_xy` (see §5).
- Likely **lower `χ_r`**: `χ_r·ζ_r` (surface reaching) and `k_r` are two routes to `ζ_r→0`;
  running both hot risks over-drive. Let `k_r` carry convergence, `χ_r` carry damping.

k_r acts **proportionally to the offset**: negligible when centered, strong when off-center
(IC4 `s_e_n@0.5`: 0.76 at k_r=0 → 0.07 at k_r=0.3).

---

## 4. Commit/handoff phase — the trigger

**Event (start-height-independent, depth-free, image-space):** marker corners reach the image
border — `MARKER_EXTENT_PX ≥ ~400` (empirically Z ≈ 0.50 m, IQR ±0.05 for precise reps; fixed
altitude regardless of start height because it fires when the marker's angular extent = FoV).
This replaces the start-dependent loom-integral threshold.

**Robustness:** gate on **corner pixel position near the border**, NOT "corner count dropped."
A corner vanishing mid-frame = decode failure (KLT-bridge it); only a corner vanishing *at the
border* = geometric exit.

### Case (a)/(b) discriminator — FINALIZED

```
ON corner-exit event (corners at border):
  if  |s_e_n| small            # centered (the "4-directions" radial exit)
      AND h_z < 0              # genuine loom (rejects ring-flip receding glitch)
      AND |ds_e_n| small       # SETTLED — not whipping through center with residual velocity
      held for N consecutive frames:
     → CASE (b): COMMIT   (one-way latch)
  else if  |s_e_n| large  OR  ds_e_n > 0 growing:   # off-center drift / s_e_n diverging
     → CASE (a): ABORT → ascend → re-attempt
```

**Why `|ds_e_n|` (not `||h_xy||`):**
- It is the residual velocity in the `s_e_n` coordinate: when centered `ds_e_n ≈ v_res/(Z·p_10)`.
- Same (image-normalized) coordinate as the `|s_e_n|` gate — self-consistent, scale-free.
- **Centroid-derived** (decode), more robust near the deck than LK-flow-derived `||h_xy||`.
- Catches the **zero-crossing** miss: `|s_e_n|` small can coincide with large rate (feature
  whipping through center). `|s_e_n|` small AND `|ds_e_n|` small = genuinely settled.
- Sign maps onto the split directly: rate race ⇒ `s_e_n` safe iff `ds_e_n ≤ 0`; `ds_e_n > 0`
  growing IS the breach/kick precursor → abort.
- Validated @ corner-exit: `|ds_e_n|` precise 0.061 vs failed 0.252 (≈4× separation; position 0×).

The **`||h_xy||` flow guard is dropped** — `|ds_e_n|` carries the same velocity info in the
better sensor and matching coordinate.

**Implementation cautions:** `ds_e_n` is 1/Z-amplified and `s_e_n` is noisy near the deck
(decode breakdown) → MUST be filtered (controller already has `smooth4`/KF on `ds_e_n` in the
legacy outer path) and held over **N frames**. The threshold is NOT a transferable constant —
set it during commit tuning against the controller's *filtered* `ds_e_n`.

### Case (b) — committed (terminal descent)
- Zero the image-feature lateral command (`ζ_s / s_e_n → 0`) — removes the `ζ_r` blow-up term →
  no kick. Honest: the marker is no longer visible, so `s_e_n` is fiction.
- Keep **vertical (loom) + yaw** control via **ring features**; lateral held by the
  (un-degenerate) flow funnel — valid *iff the flow funnel is not breached* (verify in test).
- **One-way latch:** re-acquiring the marker must NOT re-enable lateral control.
- Residual coast is bounded because Part 1 already minimized `v_res` at commit.

### Case (a) — aborted
- Ascend to re-acquire, re-enter approach. New behavior (today a fly-away becomes a retry).

---

## 5. The velocity bound — exact statement (corrected)

`p_2inf` bounds **`h_e_xy = h − h_d`**, NOT raw `||h_xy||`. The bound on `v_res` holds at the
**centered commit** purely by the algebra `h_d→0 at center`:

- after the switch, at center `_hd_rate→0`, `ff→0`, so `h_d ≈ rot ≈ 0.041` (small);
- ⇒ `h_e_xy|center ≈ h_xy − rot ≈ h_xy` ⇒ `p_2inf_xy < 0.15` bounds `h_xy ≈ v_res`.

⚠ **Corrected (do NOT overclaim):** `h_e_xy` is **NOT** "translational velocity with rotation
compensated out." Validated against data: the `rot=cross(w,s)` feedforward is only **~19% of
`h`** (median 0.041 vs 0.222) — far too small to cancel `h`'s rotational content; and `h`
correlates with body rate (0.75) more than with `v_lat/Z` (0.66), partly a tilt-to-accelerate
confound. So `h_e_xy ≈ raw flow` at center (minus a ~19% nudge), tracking `v_res` only as well
as raw `||h_xy||` does (corr ~0.66). The bound works by `h_d→0`, **not** by rotation removal.

⚠ **Floor caveat:** `p_2inf_xy` cannot go below the lag-limited achievable `h_e_xy`; below it the
funnel breaches (`ζ_h→∞` → chatter — the thing we're removing). `0.12` is a target to sweep
*down to*. If the floor sits above 0.15, the `||h_xy||`/`|ds_e_n|` runtime guard returns as
fallback (but `|ds_e_n|` in §4 already provides it).

**Validation metric flips with config:** pre-switch `h_e≈0` (degenerate) so use raw `||h_xy||`
as the `v_res` proxy; **post-switch** `h_e_xy` is the informative error → read it + `|ds_e_n|`.

---

## 6. Architecture summary

| concern | handled by |
|---|---|
| drive `s_e_n → 0` (converge to center) | `HD_KR` inward command + `χ_r·ζ_r` reaching |
| bound residual velocity `v_res` (≈ `h_e_xy`) | `p_2inf_xy < ~0.12` (continuous, by construction) |
| decide we're over the target | corner-exit + `\|s_e_n\|` small + `h_z<0` + `\|ds_e_n\|` small |
| remove the kick / honest blind descent | commit: `ζ_s→0`, ring-flow handoff, one-way latch |
| recover off-center / diverging | case (a) abort → ascend → re-attempt |

---

## 6a. Implementation status (2026-06-28)

**Controller framework — DONE** (`src/controller.py`, env-gated `PLASMC_TERMINAL_COMMIT=1`,
default-off):
- `_terminalCommitStep()` — called from the **top of `PLASMC()`** (which runs *after*
  `_updateOptFlow`), so `h_z` is the **current** frame's loom (no stale lag), and the
  `_committed` latch is set before the surface assembly in the same `PLASMC()` step. Inputs:
  `s_e_n = self._s_e_n[-1]`, filtered `ds_e_n = self._s_dot_meas[-1]/p_10` (combined-mode only).
- **Corner-exit** = median `MARKER_EXTENT_PX` over `COMMIT_WIN` frames > `TC_EXTENT`.
- **case (b)** = centered (`|s_e_n|<TC_SEN`) + loom (`h_z<0`) + settled (`|ds_e_n|<TC_DSEN`),
  held `TC_FRAMES` → latch `_committed` → **zero `ζ_r` + `dζ_r` in `σ_xy`** (surface block).
- **case (a)** = off-center OR diverging (`ds_e_n·s_e_n>0` at magnitude) → set `_abort_requested`
  (exposed via `ABORT_REQUESTED` property); both events `print`.
- Env knobs: `PLASMC_TC_EXTENT` (400), `PLASMC_TC_SEN` (0.3), `PLASMC_TC_DSEN` (0.2),
  `PLASMC_TC_FRAMES` (3); reuses `PLASMC_COMMIT_WIN` (7) for the extent median.

**No app change needed for case (b)** — the controller stops driving lateral position; the
descent loop keeps vertical+yaw+flow. Run the first test with the app-side proximity commit OFF
(`LANDING_COMMIT_EXTENT=0`) so only the controller commit acts. (The old `LANDING_COMMIT_LOOM`
open-loop loom-commit was removed entirely — wrong approach.)

**⚠ Corner→ring handoff is NOT automatic in GT-FB.** In the perception pipeline, flow fusion
degrades corner→ring on corner-loss for free; but the **first tests are GT-based** (GT-FB), where
the flow source is injected and the corner→ring fallback does not happen on its own. The handoff
must be made explicit and the post-handoff ring signal validated for reliability — see §6b.

**Case (a) re-ascend — DEFERRED** (controller only flags it; app module is the next layer).

**Approach config WIRED** — `scripts/run_terminal_approach.sh` (thin preset over
`run_aruco_landing.sh`) bundles Part 1 + Part 2 into one launch, all values overridable:
- Part 1: `HD_FUNNEL_REF=1`, `HD_KR=0.3` (provisional; sweep `{0.3,0.5,0.7,1.0}`),
  `P2INF_X=P2INF_Y=0.12` (provisional; sweep down to the chatter floor). `χ_r` left at baked 0.5
  (commented knob for the χ_r–k_r balance sweep).
- Part 2: `TERMINAL_COMMIT=1` + `TC_*` knobs; `LANDING_COMMIT_EXTENT=0`.
- `GT_FEEDBACK=1` by default (override `=0` for the perception run).

```bash
bash scripts/run_terminal_approach.sh                       # GT-FB, default config
env PLASMC_HD_KR=0.5 bash scripts/run_terminal_approach.sh  # sweep a single var
env PLASMC_GT_FEEDBACK=0 bash scripts/run_terminal_approach.sh   # perception run
```

## 6b. Handoff — GT-FB vs perception (2026-06-28)

The corner→ring handoff means **different things** in the two modes; conflating them is unsafe.

| | perception approach | **GT-FB (first tests)** |
|---|---|---|
| flow source post-corner-exit | corner-flow **→ ring-flow** (real source switch) | **GT flow throughout** (`gt_feedback` replaces s/h; always available, reliable to deck) |
| flow handoff to validate | yes — ring must be reliable below ~0.5 m | none — GT flow is continuous by construction |
| control handoff (`σ_xy` drops `ζ_r`) | yes | yes |

**(1) Smoothness of the control handoff — testable in GT-FB, IMPLEMENTED.** A hard-zero of `ζ_r`
steps `σ_xy` by `χ_r·ζ_r` (~0.27 at `TC_SEN=0.3`) → `a_u` jolt. Fix: **taper `ζ_r,dζ_r → 0` over
`PLASMC_TC_RAMP_S`** (default 0.3 s) from the commit instant, so `σ_xy → ζ_h` continuously.
Validate in GT-FB: `a_u_xy` and `σ_xy` should have **no step** at the commit timestamp.

**(2) Ring reliability after handoff — NOT testable in GT-FB.** Because GT flow never degrades,
a clean GT-FB landing does **not** prove the perception ring-flow holds lateral below ~0.5 m.
This is a **separate perception-ON validation** (the false-confidence trap): confirm ring-flow
tracks GT-flow at close range (ring cal / `io-calibration`), and that with corners gone the fused
flow stays bounded enough that `σ_xy = ζ_h` doesn't breach the flow funnel. Until that passes, a
GT-FB success is necessary but **not sufficient** for perception deployment.

## 7. Open items (deferred — tuning/build, not design)

- `k_r` value — sweep `{0.3,0.5,0.7,1.0}` on **IC4** (off-center provoker), scored on `s_e_n` and
  `v_res` @ Z=0.5 m, ignoring sub-0.5 (commit cuts it); guard: no `s_e_n` overshoot *before* 0.5 m.
- `P2INF_xy` floor — sweep `0.5→0.35→0.25→0.15→0.10`, watch σ-flip onset (chatter floor).
- `χ_r` / `k_r` balance.
- Commit thresholds: `MARKER_EXTENT_PX`, `|s_e_n|` cut, `|ds_e_n|` cut, N frames.
- Case (a) abort/re-ascend module (new).
- Ring-handoff validation: does the flow funnel hold lateral with `ζ_s=0`?
- First test: commit on `|s_e_n|` + `h_z<0` + `|ds_e_n|` (geometric+centered+settled); read
  post-commit touchdown to confirm Part 1 made the velocity guard sufficient.
