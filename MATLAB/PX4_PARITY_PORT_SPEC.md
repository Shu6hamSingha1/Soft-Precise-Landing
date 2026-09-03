# MATLAB ← PX4 parity port spec

**Decision (user, 2026-09-03): PX4 SITL is now the baseline. MATLAB is to be brought to it —
constants and approaches both.**

Direction of travel: `PX4_Gazebo/src/controller.py` (+ `cbf_visibility.py`) → `MATLAB/VDF_ASMC/vdf_params.m`
(+ `MATLAB/VDF_ASMC/+blocks/*`).

## Provenance of the two columns

- **MATLAB** = `MATLAB/VDF_ASMC/vdf_params.m`, read 2026-09-03. This is the single source of truth
  for PLASMC (`run_simulation.m:41` `P = vdf_params()`; `Comparison/InitGains_Comparison.m:58`
  "vdf_params() (single source of truth)"). **Do NOT use
  `Multi_init_cond/visualControl_IBVS_adaptive.m`** — stale since 2026-06-25, superseded gains,
  superseded `h_d`. See memory `feedback_matlab_source_of_truth_vdf_params`.
- **PX4 effective** = `controller.py` under `PLASMC_COMBINED_BARRIER=1` (the default), i.e. the
  post-`pa()` rebake block at ~lines 479-490 — **not** the `pa(...)` defaults, which that block
  overrides. Cross-checked against `Config.resolved` in a 2026-09-03 rep's `Control_Params.npy`.

⚠ Line numbers are from Ubuntu HEAD on 2026-09-03. Re-verify before editing.

---

## STATUS (2026-09-04): FULL GATE PASSES 50/50

`vdf_params.m` now bakes `Xi_h=0.2` (kept at pre-port MATLAB, PX4's faster value doesn't
transfer) + `Xi_r=0.30` (retuned away from PX4's 0.10, which fails everywhere it's tried) on
top of the rest of the port (joint-QP CBF, HD_KR, kappa_max, T_max, and the other ~14
constants). The full 5-trajectory × 5-IC × 2-config gate (50 cells) — via
`run_simulation()`'s baked defaults, no overrides — passes **50/50** fully soft-precise. See
the `Xi_h`/`Xi_r` section below for the full grid + the methodology lesson from a
first-attempt value that passed a narrow probe but failed the full gate.

**Comparison-study spot-check (2026-09-04):** the 4-controller comparison study has NOT been
regenerated in full, but a same-conditions check found no evidence the `T_max` plant change
(60→28.77N, shared by all 5 controllers) breaks the baselines. Two direct pre/post-port
comparisons on `visualControl_comparison` (Lin 2022, the only baseline both backups happened
to cover):

| cell | pre-port | post-port |
|---|---|---|
| Static, ctrl 2 (Lin 2022) | idx=687, xy=2.4519, rel_vel=0.2590 | idx=687, xy=2.4509, rel_vel=0.2590 |
| Circular, ctrl 2 (Lin 2022) | idx=522, xy=2.5744, rel_vel=0.4086 | idx=522, xy=2.5749, rel_vel=0.4072 |

Both essentially identical (same failure step, xy differs by <1mm) — Lin 2022 already fails
on both trajectories pre-port (expected; the comparison's premise is that baselines
underperform PLASMC), and `T_max` has no material effect on when/how it fails. Live spot
checks of Zhang 2026 / Lin 2023 / Cho 2022 on Static ran without crashing (landed=1/0/0
respectively, consistent with mixed baseline performance) but had no matching same-conditions
backup to diff against (the backed-up `Static_comparison.mat` only had controller 2
populated — an incomplete file, not a live-run problem).

**Not done:** full regeneration of the comparison datasets (needed eventually — `run_comparison_all.m`
+ `run_monte_carlo.m` + `multi_speed_comparison.m`). A MATLAB batch-mode crash
(`std::terminate()`, unrelated to any script content — happened after a script's own output
had already printed) was observed once during this session; noted in case it recurs, not
otherwise investigated.

**Joint-QP convergence diagnostic ADDED 2026-09-04.** `cbf2_filter.m`'s `jqp_on` branch now
tracks `jqp_resid(outer) = ||Ia_lat_new - Ia_lat_prev||` per outer iterate and exposes
`state.jqp_residual` (6-vector) and `state.jqp_converged` (residual on the last iterate <
5% of the first, or < 1e-3 absolute) after the loop. This is purely additive/diagnostic — it
does not feed back into `I_a`/`th`/`theta_cone`, and the loop still runs the same fixed 6×5
budget regardless. Confirmed behavior-neutral: a spot check (IC5 Circular noiseless) gives
`xy=0.0128, rel_vel=0.0767`, an exact match to the pre-diagnostic 50/50-gate value.

This directly answers the gap `project_joint_qp_nonconvergence_kappa_ratchet.md` flagged as
"the natural next step, not yet done" — a caller can now check `cs.cbf_state.jqp_converged`
after any call. **Not yet done:** actually using it — checking whether it ever reports
non-convergence on MATLAB's runs (the PX4 defect may or may not be reachable under MATLAB's
idealized perception/plant), and whether non-convergence correlates with any of the harder
IC×trajectory cells. That's the natural follow-up, not performed this session.

## STATUS (2026-09-03)

**Wave 0 + Wave 1 APPLIED.** Files touched:
- `MATLAB/Common/Constants.m` — `T_max` 60.0 → **28.7725 N** (plant; affects all five controllers)
- `MATLAB/VDF_ASMC/vdf_params.m` — 17 values ported + `P.kappa_max` added
- `MATLAB/VDF_ASMC/+blocks/asmc.m` — per-axis `kappa` clamp added (`isfield`-guarded), so
  `P.kappa_max` is actually enforced rather than inert

Every ported line keeps its pre-port value and rationale as `PRIOR`, and the four that **revert a
prior MATLAB LOCK** are marked ⚠ in-file: `E_xy`, `Pleak`, `kappa0`, `Gamma_a`. Those are the first
places to look if the gate regresses.

**`Omega_a` 0.25 → 0.1 PORTED** (user rule: where MATLAB has no explicit reason to retain its own
value, take PX4's). Noted in-file as the second revert candidate after `Gamma_a` if yaw slows.

**Wave 2 — CBF PORTED.** `Common/cbf2_filter.m` gained an optional 14th arg `jqp`
(`.A_cap/.k_az/.g`): when supplied it runs PX4's joint solve — 6 outer × 5 inner passes over the FULL
`I_a`, with `M_i = Lw_i*Rz(-yaw)/a_z` recomputed each outer iterate, the descent-rate relief, and the
**true-thrust** sphere `|I_a| <= A_cap`. New helper `project_box_Ia`. A 13-arg call or `jqp=[]` keeps
the legacy theta-QP bit-identical. `+blocks/cbf_visibility.m` passes it when `P.jqp_on`.
New params: `P.jqp_on=true`, `P.A_cap = T_max/P.m` (**derived**, tracks `Constants.m`), `P.k_az=5.0`.

- The three legacy relaxations (`DIR_INSET_RELAX`, `DRIFT_GATED`, `CBF_NO_REVERSE`) are guarded
  `&& ~jqp_on` — all default-off, but they operate on `th` assuming fixed `a_z`, which the joint
  solve invalidates.
- **Phase 2 (decode-fail cone fallback) still uses the legacy path.** PX4 behaves the same way, so
  this is parity, not an omission.
- `so3_tracker`'s post-QP `theta_cap` rescale is **kept**: PX4 also still enforces `theta_cap` after
  its joint QP (`Config.resolved` shows it live), and with `theta_cap = arccos(g/A_cap)` it is
  near-redundant against the sphere rather than contradictory.

**Wave 2 — `HD_KR` PORTED.** `+blocks/position_funnel.m` now subtracts
`P.phi_max .* (P.hd_kr .* zeta_r ./ g_rv)` from `s_dot_presc` (`g_r` promoted from loop-local to a
stored per-axis vector). `P.hd_kr = 0.5`, `isfield`-guarded. Confirmed live on PX4 at HEAD
(default 0.5, `HD_FUNNEL_REF=1`, `HD_PASSIVE=0`, on the operative `_h_d` path not the `_h_d_kfree`
diagnostic, and resolved as 0.5 in the three newest reps incl. two from 18:1x on 2026-09-03).

`dh_d` needed **no** change: PX4's `PLASMC_DHD_SRC` defaults to `full`, i.e. it differentiates the
whole `h_d` *including* the k_r term — which is what `flow_surface.m:52` already does. (The
`h_d_noS` naming is misleading: it describes that variable, not what feeds `dh_d`.)

⚠ Unrelated finding: `P.drop_sddot` is **dead config** — declared in `vdf_params.m`, read by no
`+blocks/` file (only the stale `visualControl_IBVS_adaptive.m`). Harmless, but it advertises a
behaviour the live path does not have.

**Wave 2 COMPLETE.** All structural items ported.

## VALIDATION (2026-09-04, headless MATLAB, user-authorized)

Ran via `matlab -batch`, not the full 50-run gate (see below for why). Pre-port
`Datasets/MultiInit/*.mat` backed up first (scratchpad, 36 files) as the comparison baseline.

1. `vdf_params()` loads clean; all ported values read back correctly (`A_cap=13.6105`,
   `k_az=5.00`, `hd_kr=0.50`, `theta_cap=43.94°`, `kappa_max=[30 30 3]`).
2. Static IC1 noiseless: lands clean through the full ported path (joint-QP, HD_KR, kappa_max
   clamp all exercised) — `precise=1 soft=1`, xy=0.5mm.
3. **IC5 × all 5 trajectories × {noiseless, realistic} — 5/10 pass, 5/10 fail.** Static/
   Linear/Sinusoidal IC5 land clean in both configs. **Lissajous and Circular IC5 fly away
   in BOTH configs** (FoV violation, xy 1.3–2.3 m, rel_vel 1.8–2.5 m/s).
4. Confirmed these ARE regressions, not pre-existing weakness: the backed-up pre-port `.mat`
   files show all 4 flagged cells landing clean (`success=1 precise=1 soft=1`, xy 0.9–2.4 cm).
5. Single-lever isolation (revert jqp_on / hd_kr / k_az / theta_cap / the 5 "lock" constants,
   one at a time) gave **bit-identical failures across all six** — none is individually
   responsible, and the T_max clamp inside `run_simulation.m:197` reads the script-local
   `T_max`, not any `P.*` field, so `jqp_on=false` alone did not actually restore thrust
   authority either (a real gap in that first attempt — see the `T_MAX_OVERRIDE` fix below).
6. **Decisive test: full revert (every ported value + all 3 structural flags back to PRIOR,
   T_max=60) reproduces the pre-port baseline essentially bit-exact** (xy 0.0137 vs backup's
   0.0137, rel_vel 0.1393 vs 0.1393). **This proves the port's CODE is structurally correct** —
   `cbf2_filter.m`'s jqp branch, `position_funnel.m`'s HD_KR term, and `asmc.m`'s kappa_max
   clamp all behave as no-ops when disabled. The IC5 Lissajous/Circular failure is a
   **gain-retuning gap**, not a bug, exactly per this spec's standing caveat.
   Diagnostic aid added: `Constants.m` gained `T_MAX_OVERRIDE` (mirrors the existing
   `H_RD_OVERRIDE` pattern) so `T_max` can be swept without editing the file.

**RESOLVED 2026-09-04 (grouped + pairwise sweep on IC5 Circular, confirmed on Lissajous):**
the cause is the interaction of `Xi_h` (optic-flow funnel contraction, ported 0.2→**1.0**, 5×
faster) and `Xi_r` (position funnel contraction, ported 0.3→**0.10**, 3× slower) TOGETHER.
No single constant reproduces the failure or the fix alone:

| reverted | IC5 Circular result |
|---|---|
| `Xi_h` alone | still fails (xy=1.56) |
| `Xi_r` alone | still fails (xy=1.24) |
| `chi_r` alone | still fails (xy=1.80) |
| `Xi_h`+`Xi_r` (chi_r left at ported 1.5) | **clean** (xy=0.0128) |
| `Xi_h`+`chi_r` (Xi_r left at ported 0.10) | marginal fail (xy=0.090, just over precise/soft) |
| `Xi_r`+`chi_r` (Xi_h left at ported 1.0) | still fails (xy=1.06) |
| ALL 6 groups (A–F) reverted | clean (xy=0.0072) — superset, consistent |

**`Xi_h`+`Xi_r` reverted alone, with EVERYTHING else (including `chi_r`, the full joint-QP,
HD_KR, kappa_max) left at PX4's ported value, fixes all 4 originally-failing cells** (IC5 ×
{Lissajous,Circular} × {noiseless,realistic}) to `success=1 precise=1 soft=1`.

Mechanism (plausible, not yet proven): the position funnel (`p_r`) now contracts 3× *slower*
while the optic-flow funnel (`p_h`) contracts 5× *faster* — on a fast-moving target (Lissajous/
Circular only; Static/Linear/Sinusoidal IC5 are unaffected) that timing mismatch between the
two funnels, not either rate alone, is what breaks tracking. Not yet root-caused further.

**RESOLVED via a 16-point `Xi_h`×`Xi_r` grid (scored on the 4 failing cells), then corrected
against the FULL 50-cell gate.**

Grid on the 4 failing cells only (IC5 × {Lissajous,Circular} × {noiseless,realistic}):

| Xi_h \ Xi_r | 0.10 | 0.15 | 0.20 | 0.30 |
|---|---|---|---|---|
| 1.0 (PX4) | 0/4 (xy 2.28) | 0/4 (2.04) | 1/4 (1.95) | 3/4 (1.24) |
| 0.8 | 0/4 (2.28) | 0/4 (1.95) | 1/4 (1.80) | 3/4 (1.13) |
| 0.6 | 0/4 (2.24) | 0/4 (1.95) | 3/4 (1.51) | 3/4 (0.14) |
| 0.4 | 0/4 (2.08) | 2/4 (1.58) | 3/4 (0.09) | **4/4 (0.07)** |

`Xi_r` is the dominant lever — 0.10/0.15 fail regardless of `Xi_h` (PX4's own `Xi_r=0.10` sits
in the worst-performing column throughout). Only `Xi_r=0.30` reaches a clean 4/4, and only
paired with `Xi_h≤0.4`.

**First attempt (WRONG, caught by the full gate, not the 4-cell probe):** baked
`Xi_h=0.4, Xi_r=0.30` — the fastest `Xi_h` reaching 4/4 on the grid, read as "a partial move
toward PX4's faster contraction, still safe." Running the full 50-cell gate (5 traj × 5 IC ×
2 configs) with this value: **47/50** — the original 4 cells were fixed, but 3 NEW failures
appeared (noiseless Circular IC2/IC3/IC4, xy 1.02–1.23m) that the narrow 4-cell probe never
covered. The grid's own 4/4 result was true but insufficient evidence — it was scored on a
test set that happened not to contain the case `Xi_h=0.4` broke.

**Corrected: `Xi_h=0.2` (fully unchanged from pre-port MATLAB, no partial move at all),
`Xi_r=0.30`.** Confirmed this exact value fixes the 3 new IC2-4 failures
(`success=1 precise=1 soft=1` on all three), then **re-ran the full 50-cell gate: 50/50
fully soft-precise.** `PX4's faster Xi_h does not transfer to MATLAB's plant at all — not
even partially. Only `Xi_r` needed retuning (0.10 → 0.30, i.e. reverting PX4's bake rather
than porting it). BAKED in `vdf_params.m` (2026-09-04).

**Methodology lesson:** a passing grid on a narrow probe set is not sufficient evidence for a
gain change — always confirm against the full gate before baking, even when the narrow probe
looks unambiguous (4/4 with good margin still hid a regression elsewhere). The full 50-run gate was deliberately NOT run
given (3) already shows a real, non-trivial regression needing a re-tune first — running the
full sweep now would just enumerate more instances of the same root cause.

**Cross-reference, checked and RULED OUT as the cause here:** `Memory/px4/
project_joint_qp_nonconvergence_kappa_ratchet.md` (2026-09-04, independent PX4 session) found
the joint-QP's fixed 6×5 iteration genuinely fails to converge near touchdown on real PX4 data
— `theta_cone` chatters, `I_a_xy` swings, `kappa` ratchets, `a_u` detonates to 1274 m/s².
This is the Q5 concern ("no convergence test, no residual logged") CONFIRMED as a live bug,
not just a theoretical gap — and `project_box_Ia` in `cbf2_filter.m` was ported with the same
fixed-iteration structure, so it carries the same latent defect. **But it is not what's
failing IC5 Lissajous/Circular here**: `jqp_on=false` (bypassing the joint-QP entirely, back
to the legacy theta-QP) still fails at IC5 Circular (isolate2.m, xy=1.7872). So this is a
second, independent, real defect worth fixing in the joint-QP branch (add a residual/
convergence check) — just not the explanation for the current regression.

## A0. Name map (read this first)

Source: `PX4_Gazebo/docs/CONTROLLER_PARITY.md` §7.1 (2026-06-15 — **names** reliable, **values** stale),
reconciled against live code 2026-09-03. Several apparent "mismatches" are renames.

| Manuscript | MATLAB (`vdf_params.m`) | MATLAB legacy | PX4 env / attr |
|---|---|---|---|
| Ξ_h | `Xi_h` | `gamma_2` | `PLASMC_XI2_{X,Y,Z}` → `_gamma` |
| p_h0 / p_h∞ | `p_h0` / `p_hinf` | `p_20` / `p_2inf` | `PLASMC_P20/P2INF_{X,Y,Z}` |
| Γ, 𝒫, 𝒩, κ(0), ℰ | `Gamma, Pleak, N, kappa0, E` | same | `PLASMC_GAMMA/P/N/KAPPA0/E_{X,Y,Z}` |
| χ_r | `chi_r` | `chi_r` | `PLASMC_CHI_R_{X,Y}` |
| χ_z | `chi_z` | `chi_z` | **`PLASMC_OMEGA_Z`** |
| h_rd | `h_rd` | `h_rd` | **`LANDING_REF_RAD_OPT_FLOW`** — defined in `apps/landing_test.py:31` (default −0.30), passed into `Controller(...)` at :162; **not** in `controller.py` |
| χ_α, γ_α, … | `Omega_a, Gamma_a, n_a, p_a, kappa_a0, E_a` | same | `PLASMC_YAW_OMEGA/GAMMA/N/P/KAPPA0/E` |
| k_R | `kR` | `kR` | `PLASMC_KR_{ROLL,PITCH,YAW}` |
| θ_cap | `theta_cap` | `theta_cap` | `PLASMC_THETACAP_DEG` |

**Three resolutions that correct the earlier draft of this spec:**

1. **`chi_z` ≡ `OMEGA_Z`.** `controller.py:2850` states it verbatim: *"sigma_3 = zeta_h_3 +
   chi_z\*int(zeta_h_3) (PI, chi_z = Omega_z, unchanged)"*. Both sides are **0.1 → already matching.**
   Not a mismatch; not a missing MATLAB parameter.
2. **`OMEGA_X/Y` is inactive, not missing from MATLAB.** Under `combined_barrier=1` (default both
   sides) the lateral surface uses `chi_r·dzeta_r` (`controller.py:2866`), not `Omega_xy·izeta`. The
   only path that reintroduces `Omega_xy` is `_tc_omega_xy`, gated on `_tc_integral and _committed`
   (terminal commit). **Nothing to port.** The parity doc's `𝒳/Omega` row describes the superseded
   legacy surface.
3. **`p_10` ≡ `phi_max`, so `p_r0` really does differ 8×.** PX4 `_p_10 = center/focal`
   (`controller.py:334`); MATLAB `P.phi_max = P.res/(2*P.f)` (`vdf_params.m:23`) — identical formula,
   so `s_e_n` carries the same units on both sides and 1.2 vs 10.0 is genuine tuning, not a
   normalisation artifact.

Also stale in the parity doc: the **accel-conditioning** row (`rho_fov_0/inf, l_fov`). MATLAB has no
`rho_fov` at all — it moved to the `phi_max_cbf` inset + `theta_cap` (cbf2) formulation. PX4 still
carries `RHOFOV0=105.0 / RHOFOVINF=40.0 / LFOV=0.0` alongside its CBF (`LFOV=0` makes it a constant
cone). Decide whether that cone is live in PX4 before treating it as a port item.

## A. Constants

### A1. Image-feature (position/SEN) funnel

| symbol | MATLAB | PX4 effective | note |
|---|---|---|---|
| `chi_r` | `[2.0; 2.0]` | `[1.5, 1.5]` | PX4 ~465 |
| `p_r0` | `[1.2; 1.2]` | `[10.0, 10.0]` | **8×** — check units/normalisation agree before porting |
| `p_rinf` | `[0.85; 0.85]` | `[0.8, 0.8]` | close |
| `Xi_r` | `diag([0.3, 0.3])` | `diag([0.10, 0.10])` | 3× |

### A2. Optic-flow funnel

| symbol | MATLAB | PX4 effective | note |
|---|---|---|---|
| `Xi_h` | `diag([0.2,0.2,0.2])` | `diag([1.0,1.0,1.0])` | **5×**; PX4 ~488, "the line that ACTUALLY governs default runtime gamma" |
| `p_h0` | `[25;25;4]` | `[15,15,10]` | xy down, z up |
| `p_hinf` | `[1.0;1.0;1.5]` | `[2.5,2.5,1.5]` | xy only; z already agrees ("vdf p_hinf z") |

### A3. Optic-flow ASMC

| symbol | MATLAB | PX4 effective | note |
|---|---|---|---|
| `Gamma` | `diag([0.4375,0.5,0.75])` | `diag([0.25,0.25,0.75])` | xy symmetric in PX4; z agrees |
| `E` | `diag([0.5,0.5,0.5])` | `diag([1.0,1.0,0.5])` | xy only |
| `N` | `diag([0.10,0.10,0.10])` | `diag([0.1,0.1,0.1])` | ✅ **already matches** |
| `Pleak` | `diag([0.5,0.5,1.5])` | `diag([2.5,2.5,5.0])` | 5× / 3.3× |
| `kappa0` | `[0.05;0.05;0.05]` | `[0.5,0.5,0.25]` | 10× / 5× |
| `kappa_max` | **absent** | `[30,30,3.0]` | new — see B4 |
| `chi_z` | `0.1` | `0.1` (as `OMEGA_Z`) | ✅ **already matches** — see A0.1 |
| `Omega_xy` | n/a | `0.1` but **inactive** | ❌ **not a port item** — see A0.2 |

### A4. Descent / yaw / CBF

| symbol | MATLAB | PX4 effective |
|---|---|---|
| `h_rd` | `-0.42` | `-0.30` |
| `Omega_a` (χ_α) | `0.25` | `0.1` |
| `Gamma_a` (γ_α) | `0.25` | `0.5` |
| `n_a, p_a, kappa_a0, E_a` | `1.0, 2.0, 2.0, 3.0` | same | ✅ |
| `kR` | `diag([2.5, 1.5, 0.5])` | `diag([2.5, 2.5, 0.5])` | **pitch 1.5 → 2.5** |
| `kOmega` | `diag([0.3,0.3,0.2])` | n/a (PX4 rate loop) | architecture, not a port |
| `theta_cap` | `60°` | `43.94°` (= `arccos(g/A_CAP)`) |
| `theta_floor` | absent | `60°` (`PLASMC_THETA_FLOOR_DEG`) |
| `CBF_TAU` | `0.3` | `0.3` | ✅ |

⚠ **`Omega_a` should arguably NOT be ported.** `controller.py:531-537` states the 0.25→0.1 cut buys
margin against the PX4 inner-loop lag (`K_R_YAW` + rate loop + `tau_ua` LPF) that is **absent in
MATLAB**. Porting it imports a compensation for a lag the plant doesn't have. `Gamma_a` 0.25→0.5 has
no comparable justification on record — worth asking why.

### A5. Plant

| | MATLAB | PX4 | action |
|---|---|---|---|
| `mass` | `2.114` | `2.114` | ✅ |
| `g` | `9.81` | `9.8` | trivial; align |
| thrust ceiling | `T_max = 60.0 N` (`Constants.m:28`) | `A_CAP = 13.610 m/s²` = **28.77 N** | **2.09× mismatch** |

---

## B. Approaches (structural)

### B1. Lower the thrust ceiling to PX4's value

> **RETRACTED (2026-09-03):** an earlier draft of this section claimed PLASMC's thrust was
> **unenforced** while the four baselines were clamped, i.e. a fairness defect in the comparison.
> **That was wrong.** `T_cd` is clamped to `T_max` on the PLASMC path in *every* driver:
> `visualControl_comparison.m:624` (inside `if CTRL_SEL == 1`, alongside identical torque clamps at
> 622-623), `run_simulation.m:197`, `VDF_ASMC/simulate_landing.m:94`,
> `visualControl_IBVS_adaptive.m:1019`, `..._loop.m:376`. PLASMC and the baselines are saturated
> identically. **There is no fairness defect.**
> Cause of the error: a `grep ... | head -12` truncated the result list exactly before the driver
> hits, and the truncated list was treated as exhaustive. `T_max` has 22 hits, not 5.

What survives is a **plant-fidelity** mismatch, in scope for this port but not urgent:

| | MATLAB | PX4 |
|---|---|---|
| ceiling | `T_max = 60.0 N` = **2.89 g** | `A_CAP*mass = 28.77 N` = **1.389 g** |

MATLAB's modelled airframe has 2.09× the thrust authority of the measured SITL x500. Set
`T_max = THRUST_MAX_N*THRUST_MARGIN = 33.85*0.85 = 28.77 N` for parity. This is also the precondition
for B2: with a 2.89 g ceiling the deliverability sphere would almost never bind, so porting it without
this change is close to a no-op.

Side effect: this also settles the manuscript's `theta_cap = 60°` "2× hover-thrust margin"
justification, which is *true* at 2.89 g and *false* at 1.389 g — which is exactly why PX4 derives
43.94°. Lowering `T_max` and porting `theta_cap` must happen together.

### B2. Visibility CBF: theta-QP → joint-`I_a` QP + true-thrust sphere
MATLAB `+blocks/cbf_visibility.m` is a camera-plane theta-QP via `cbf2_filter` with `P.theta_cap`
(a post-QP tilt rescale). PX4 (`CBF_JOINT_QP=1`, `CBF_SPHERE_TRUE_THRUST=1`, both default-on as of
`937db5a9`) solves for the full `I_a` — lateral **and** vertical — interleaved with a projection onto
`|I_a| <= A_CAP`. Depends on B1.

Note `CBF_SPHERE_TRUE_THRUST` fixed a real bug: the sphere previously bounded `|I_a + g*e3|`
(deviation-from-hover, zero at hover), a ~72% over-permit. Port the **fixed** form only.

### B3. `a_z` descent-rate relief (`CBF_AZ_COST_GAIN = 5.0`)
Inside the joint QP: `relief = gain*||th_desired - th_safe||`;
`Ia_z = min(Ia_z, max(I_a[2] - relief, -g))`. One-way: can slow a descent toward hover, never reverse
it into a climb. No MATLAB counterpart.

⚠ **5.0 was picked, not swept** — and it was chosen when the sphere never bound. Its tuning regime no
longer exists. Re-sweep on PX4 before porting.

### B4. `kappa_max` caps `[30, 30, 3.0]`
MATLAB has no cap. PX4's xy cap came from real hardware runaway (kappa_xy 25-29 pinned 10-28 s);
z=3.0 is load-bearing in bad reps, inert in good ones.

### B5. `h_d` back-map convergence term (`HD_KR = 0.5`)
MATLAB `+blocks/flow_surface.m:25` builds `h_d` from the funnel-prescribed rate
(`s_dot_presc = p_10.*S_r.*dp_r`) + transport + descent — **already matching PX4's `HD_FUNNEL_REF=1`.**
The *only* `h_d` divergence is PX4's extra term:

```
_hd_rate = _hd_rate_smooth - p_10 * hd_kr * zeta_r / g_r      # k_r = 0.5
```

i.e. `h_d_xy = phi_max .* (S_r.*dp_r - k_r*G_r^-1*zeta_r)`, which makes
`h_e_xy = phi_max .* G_r^-1 .* (dzeta_r + k_r*zeta_r) + h_e3*s_xy` — so `h_e → 0` gives *exponential*
`zeta_r` convergence at rate `k_r`, not merely `dzeta_r → 0`.

⚠ **Also unswept** (baked 2026-06-29 inside a bundled re-bake; no 0.3/0.7/1.0 A/B on record).
⚠ **Manuscript consequence:** the paper states three times (lines 286, 290, Remark `rem:normalization`)
that the design deliberately avoids a back-mapped rate proportional to the barrier coordinate. Porting
`k_r` makes those statements false and requires eq `h_d final` and eq `h_e identity` to be updated.

### B6. Do **NOT** port (default-OFF in PX4)
`PLASMC_AU_LEAD=0`, `PLASMC_SOFT_BREACH=0`, `PLASMC_YAW_ALPHA_KF=0`, `PLASMC_YAW_OMEGA_D_FF=0`,
`PLASMC_AU_ROTZ_ONLY=0`, `PLASMC_CH_PSIDOT_SIGN=+1` (default correct).
Already agreeing, no action: `theta_per_axis` (on both sides), `CH_CLEAN` c-term form, `CBF_TAU`,
`N`, the yaw `n_a/p_a/kappa_a0/E_a` set, `mass`.

---

## C. Staging

Nothing here is safely done as one big-bang edit — every item changes every number in the paper.

1. **B1** thrust ceiling: `T_max` 60.0 → 28.77 N (+ `theta_cap` 60° → 43.94°, which pairs with it).
   Re-run the IC×trajectory gate. Expect regressions; they are informative — MATLAB has been running
   with ~2× the authority of the real airframe, so some of its tuning may depend on that headroom.
2. **Re-sweep on PX4 first**: `HD_KR` (B5) and `CBF_AZ_COST_GAIN` (B3). Do not port unswept constants
   into the baseline that generates a paper's results.
3. **Log the joint-QP residual** and add a convergence test on PX4's fixed 6×5 iteration before porting
   B2 — the box ↔ relief ↔ (now-binding) sphere interaction is unanalysed, and non-convergence would be
   bad to discover after publication.
4. **B2 + B3 + B4 + B5** port, then **re-tune MATLAB gains for the new structure** (A1-A4 as starting
   points, not as final values — see the caveat below).
5. **Regenerate everything**: the IC×trajectory gate, then the 4-controller comparison in full
   (`run_comparison_all.m`, `run_monte_carlo.m`, `multi_speed_comparison.m`) — any PLASMC change
   invalidates the comparison, which carries the paper's competitive claim.
6. **Manuscript**: update eq `h_d final` / eq `h_e identity` (B5), the Visibility-CBF section and
   appendix (B2/B3), Table `sup:control params` wholesale (A1-A4), `theta_cap`, and the "2×
   hover-thrust margin" justification (true at MATLAB's 2.89 g, false at PX4's 1.389 g).

## Caveat worth re-reading before step 4

PX4's gains are tuned against ~38 ms inner-loop lag, real perception noise, and a 1.389 g ceiling.
MATLAB is an idealised RK5 13-state plant. Several ports are **large** (`Xi_h` 5×, `Pleak` 5×,
`kappa0` 10×, `p_r0` 8×) and at least one (`Omega_a`) is explicitly a compensation for a PX4-only lag.
Expect A1-A4 to need re-tuning after B1-B5 rather than to transfer directly. If MATLAB regresses badly
after the port, the likely reading is that the *plant* needs more PX4 parity (actuator lag, sensor
noise), not that the gains are wrong.
