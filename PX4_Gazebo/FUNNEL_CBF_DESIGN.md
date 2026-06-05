# Target-visibility funnel — cone clamp (current heuristic) → input-aware CBF (the guarantee)

**Status (2026-06-05):** SWITCHING the visibility mechanism from the **cone clamp** to an
**input-aware Control Barrier Function**.

---
## §0. CONVERGED DESIGN (2026-06-06): camera-plane visibility CBF as a QP over body tilt

**Supersedes the optical-flow-dynamics CBF (§2–§4) and the cone clamp (§1) for the LATERAL barrier.**
The optic-flow CBF was abandoned: calibration vs GT showed the flow (esp. divergence) is poorly
observed and carries `β` + `d_h`, so a barrier on the flow *dynamics* is unreliable (corner `h_z`
cross-val −0.98, ring −0.04). Visibility is fundamentally an **attitude** constraint — the tilt shifts
the FoV — so we filter the desired tilt directly, on the *real* camera plane.

**Barrier (REAL camera plane), per binding axis `k`:**
```
h_k = ρ_k − |c_cam,k| − δ_k ,      c_cam = c_V + f·θ
```
- `c_cam` = measured camera-plane feature (the REAL FoV position).
- `c_V = c_cam − f·θ` = de-rotated (tilt-removed) target component (slow; homography centroid).
- `f·θ` = tilt-induced displacement; `∂c_cam/∂θ = f` — exact geometric Jacobian, **no `β`, no flow, no `d_h`**.
- `δ` = marker extent (continuous, §3); `ρ = R/2` (full FoV).

**WHY NOT the V-frame:** a barrier on `c_V` does NOT ensure visibility. With `c_V = 0` (target centred
in the de-rotated frame) a tilt gives `c_cam = f·θ`, which can exceed `ρ` — target out of the *real*
FoV while the V-frame reads "safe." The photons land on the tilted sensor → the barrier must be on
`c_cam`.

**Visibility filter (QP over the body tilt `θ`):**
```
θ* = argmin_θ ‖θ − θ_d‖²   s.t.   |c_V + τ·ċ_V + f·θ| ≤ ρ − δ ,   |θ| ≤ θ_cap
```
- `θ_d` = desired body tilt from the geometric (SO(3)) tracking law.
- Per-axis **box** ⇒ the QP is a trivial clamp:
  `θ_k ∈ [(−(ρ−δ) − c_V − τċ_V)/f , ((ρ−δ) − c_V − τċ_V)/f] ∩ [−θ_cap, θ_cap]`.
- `τ·ċ_V` = drift look-ahead (`ċ_V` from the centroid finite-diff) → forward-invariance under
  velocity-driven drift. `θ_cap` = input-awareness, built in.

**Controller insertion:** `outer loop → θ_d → [visibility clamp] → θ* → SO(3) inner loop`. Replaces the
cone clamp (`controller.py:~825-860`) at the **same level** (the clamp already lives in tilt space).

### §0.1 Comparison: CBF QP (rel. deg. 1) vs directional static cone clamp (rel. deg. 0)
The **directional static cone clamp IS this box with `τ=0`**: `θ ≤ θ_curr + tan⁻¹(d_min/f)` ≡ the box
without the look-ahead. So the *only* difference is the `τ·ċ_V` drift term.

| | static clamp (deg. 0, `τ=0`) | CBF QP (deg. 1, `τ>0`) |
|---|---|---|
| Visibility | instantaneous (now) | **forward-invariant** (anticipates) |
| Velocity drift | **not prevented** — if `θ_d≈0` while sliding, `c_V` walks out | braked — box tightens ahead, forces inward tilt |
| Inputs | `c_cam` + tilt + thrust (minimal noise) | + `ċ_V` (centroid finite-diff) |
| Robustness | bullet-proof | `ċ_V` noise (smooth centroid → manageable) |
| Conservatism | none, but no anticipation | `τ` trades anticipation vs over-braking |
| Tuning | none (geometric) | `τ` (look-ahead) |

**Plan:** ship the **deg.-0 static clamp first** (cone clamp made directional + real-FoV + a clean
`θ_d` projection — robust, tuning-free), which already cures the old clamp's two defects (magnitude
strangle, shrinking-funnel early death) but only guarantees *instantaneous* visibility. **Add the
`τ·ċ_V` look-ahead (deg. 1)** for the forward-invariance guarantee, gated on whether the static version
is observed to lose the target under drift. Both env-gated (`FUNNEL_MODE=cone0|cbf1`). Optical flow is
reserved for the *vertical* loom only.

**IMPLEMENTED 2026-06-06 (`FUNNEL_MODE=cone0`):** the deg.-0 directional clamp is live in
`controller.py` at the cone-clamp application — it limits ONLY the outward (away-from-marker) accel and
frees the inward/tangential, vs the default `"cone"` magnitude clamp. The marker direction is the
V-frame centroid `s[:2]` mapped to NED via `Rz(yaw)`; the image→NED **sign/axis map is uncalibrated** —
env knobs `CONE0_SWAP` (swap u/v), `CONE0_SIGN_X`, `CONE0_SIGN_Y` (default no-swap, +1). **A wrong sign
drives the marker OUT (CBF doc Assumption 1) — calibrate empirically (small lateral accel, watch the
centroid) before trusting it.** Falls back to the magnitude clamp when no marker direction is available.
Default unchanged (`"cone"`).

**IMPLEMENTED + MATCHED TO THE `L_ω` DOC 2026-06-06:** `cone0` and `cbf1` now compute the tilt headroom
from the **rotational interaction matrix `L_ω` at the measured camera centroid** (tangent units,
depth-free, no virtual frame), per `CBF_visibility.pdf`:
`headroom = min_k m_k / ‖L_ω[k]‖`, `m = φ_max − |ᶜr̂ + τ·d| − δ − τ·δ̇`, `θ_cone = θ_curr + headroom`.
`cone0` is `τ=0` (static); **`cbf1`** adds `τ>0` with the exogenous drift `d = ᶜṙ_obs − L_ω·ω` (body-rate
stripped; EMA-filtered) — forward-invariant. Reduces to `atan(d_min/f)` for a centred marker and refines
the off-centre/edge case (the `L_ω` coupling). The directional `a_xy` clamp (cone0's `t̂`) still supplies
the *directionality* (a calibrated-inertial hybrid; a fully camera-frame `θ`-QP would need the
`a_xy↔image-tilt` convention, a second cal gate — deferred). Env: `CBF_TAU`≈0.3, `CBF_DMIN_EMA`≈0.3.

**Usage gate for BOTH cone0 and cbf1:** the `d_min` term is only active with `PLASMC_THETA_FLOOR_DEG < 60`
(the default floor=60 pins `θ_cone=θ_cap` and disables it). Test config: `FUNNEL_MODE=cone0|cbf1` +
`PLASMC_THETA_FLOOR_DEG=15..30` — the directional clamp is what makes that low floor safe (it no longer
strangles recovery, the reason the floor was pinned at 60 under the magnitude clamp). The sign-cal
default map (`SWAP=0,+1,+1`) is validated offline (`tools/calibrate_cone0_sign.py`, cos·I_a=0.84).

---

**Terminology (corrected):** the live `controller.py:757-818` is a **CONE CLAMP** — a heuristic accel
*saturation* (`|a_xy| ≤ |a_z|·tan θ_cone`), **NOT a CBF**: no barrier function, no `ḣ` constraint, no
forward-invariance, **no visibility guarantee** (it caps accel from the present geometry; the existing
velocity can still carry the centroid out). Earlier text loosely called it "centroid CBF" — that was
wrong, fixed throughout. A **CBF** is a formal barrier `h(x) ≥ 0` with `ḣ ≥ −α(h)` (or the HOCBF form)
giving *forward-invariance* of the safe set — see §3, now in its **input-aware** form (never commands
accel the lagged inner loop can't deliver, so the guarantee is one the plant can honor).

Standing decisions: cone clamp stays the env-gated fallback (`FUNNEL_MODE=cone`); `ρ_fov` held CONSTANT
at `ρ_fov_0` (`PLASMC_LFOV=0`); **precision is a SEPARATE** PPC funnel on `s_e_n` (§9, implemented);
the perception-death floor hands off to the ring flow (§7/§8).

## 1. Problem with the current clamp (D1)

`controller.py:757-818`. The FoV cone clamp limits lateral accel via
`a_xy_lim = |I_a_z|·tan(θ_cone)`, with `θ_cone = θ_current + atan(d_min/f)` and

```
d_min = min over the PRIMARY marker's 4 OUTER corners of (ρ_fov − |corner − centre|)
```

D1's invariant — *"keep all 4 corners of the single marker in frame"* — is **wrong for a
multi-marker board**: outer markers are *meant* to exit the frame. Consequences (traced on
floor0/rep2):

- `d_min` collapses on **marker size**, not target drift: the 2.5m board's primary-marker corners
  hit the FoV edge at **Z≈1m** → `θ_cone→4°` → `a_xy_lim` 0.4–2.5 m/s² while the controller wants
  5–28 → the lateral correction is **strangled the entire final descent** → `|s_e_n|` grows
  0.44→3.12 → slide-off.
- It's a **magnitude** clamp: it throttles the *recovery toward centre* exactly as hard as drift
  outward — so a centroid-based `d_min` alone (still magnitude) wouldn't fix the strangling.
- "Primary" = smallest ArUco ID; *which* marker that is changes with altitude → the reference jumps.

MATLAB doesn't hit this only because its target is small/single (corners ≈ centroid).

## 2. The right invariant: keep the **target centre + inner marker** in frame

Define a barrier on the **centroid** (invariant to which markers are visible — concentric board ⇒
every marker centre = board centre) **plus the inner marker's radius** (so "visible" means the
trackable inner marker, not just a centre point):

```
c     = centroid pixel offset from image centre  = s[:2] · f           (2-vector; s[:2] is the
                                                                         normalized centroid feature)
ρ     = the FULL camera FoV (image half-dims [H/2, W/2]), FIXED — NOT a funnel, NOT rho_fov_0. The
        HOCBF's ḣ term (§3) supplies the *dynamic* reaction margin (it brakes the outward drift before
        the edge, sized to the drift rate), so a STATIC margin (ρ < full FoV) is redundant and only
        sacrifices visibility — it would fire the perception-death floor before the marker actually
        fills the frame. (A shrinking funnel is strictly worse — early death; its convergence job is
        the s_e_n funnel's, §9.)
r     = inner (primary) marker half-extent in px = 0.5·max ptp(primary_corners)   (grows ~1/Z)

h(x)  = min( ρ_u − |c_u| − r ,  ρ_v − |c_v| − r )        # margin; h ≥ 0  ⇔  inner marker in frame
```

All terms are **pixels / normalized image quantities → scale-free, depth-free** (no Z, no metric
position). `r` is the *image* size of the marker, not altitude.

**Feasibility floor:** when `r > ρ` (inner marker fills the frame) `h<0` is unavoidable — no
controller keeps it in. That's the true perception-death floor; the **ring (texture-free) optic
flow** must carry the loom past it (see §7/§8). The funnel and the ring flow compose: CBF keeps the
target centred+visible as long as physically possible; the ring divergence carries the *vertical*
flow alive through the fill-frame phase.

**`ρ_fov` held CONSTANT at `ρ_fov_0` `[290,210]px` (IMPLEMENTED 2026-06-05, `PLASMC_LFOV=0` default).**
The decay to `[80,80]px` shrank the visibility envelope far inside the camera FoV (true image edge
≈[320,240]px), firing the perception-death floor `r_inner>ρ` prematurely (marker fills 80px while
still fully visible). Constant `ρ_fov_0` = a fixed near-camera-FoV visibility limit; precision/
convergence is the **position PPC funnel's** job (§9), not the visibility envelope's. `PLASMC_LFOV>0`
restores the decay.

## 3. The constraint: an INPUT-AWARE Control Barrier Function (the guarantee)

`h` has **relative degree 2** w.r.t. lateral accel `a` (`a → v → c`): `ċ = J v`, `c̈ = J a + J̇ v`,
where `J` is the image interaction matrix (downward camera; `J ≈ −(1/Z)·R_img`, sign fixed once
empirically/from `L`). High-Order CBF, per binding axis `j ∈ {u,v}`:

```
ḣ_j  = −sign(c_j)·ċ_j − ṙ                 (ċ from optic flow)
HOCBF:  ḧ_j + k1·ḣ_j + k0·h_j ≥ 0          →   linear in a_xy:   A_j·a_xy ≤ b_j
```

**Input-aware QP — why a clamp can't guarantee and this can:**
```
a_xy* = argmin ‖a_xy − a_des‖²
        s.t.  A_j·a_xy ≤ b_j        (HOCBF, j = u, v)
              ‖a_xy‖ ≤ a_max        (DELIVERABLE accel: a_max = |a_z|·tan θ_cap, or the actuator cap)
```
The `‖a_xy‖ ≤ a_max` box makes the guarantee *honorable*: the CBF can never command accel the lagged
inner loop can't produce. A naive CBF that ignores it gives forward-invariance **on paper** that the
SITL lag silently violates — strictly worse than an honest clamp. (We watched this exact failure mode
in the `s_e_n` funnel, §9: it demanded flow the lagged middle loop couldn't track → the error ran
away past its own bound.)

**Feasibility floor = honest handoff:** if the box ∩ HOCBF set is **empty** (even `a_max` cannot hold
`h ≥ 0`), visibility is *physically impossible* → raise the perception-floor flag → hand off to the
ring flow (§7/§8). The CBF tells you *exactly* when to give up, rather than failing silently.

**Directionality (free bonus):** the HOCBF bounds only the **outward radial** `a_xy` (the part driving
`c` to the edge), bound **→0 as h→0**; the **inward (recovery) and tangential** components are left
free → **no strangling** (the cone clamp's flaw, §1), hard wall at the edge (forward-invariance).

**Practical `ċ` source (first cut):** measured optic-flow centroid rate (`ċ ≈ h_flow[:2]`) instead of
the `J̇v` term — implementable today, robust. The QP is 2-var (`a_xy ∈ R²`, 2 HOCBF rows + 1 norm-box),
solvable **closed-form** (no QP library): project `a_des` onto the HOCBF half-planes, then onto the
`a_max` ball; if the projection set is empty → infeasible → the floor.

## 4. Implementation plan (env-gated, A/B-able)

- New block replaces `:757-818` behind `FUNNEL_MODE = os.environ.get("FUNNEL_MODE","cone")` →
  `"cone"` (current heuristic, default) | `"cbf"` (input-aware CBF).
- **Solve:** 2-var QP (`a_xy ∈ R²`, 2 HOCBF rows + 1 norm-box) — closed-form projection (cheap,
  real-time), no QP library.
- Reuse: `J` from the flow lstsq, `ċ` from optic flow (`h_flow[:2]`), `r` from the primary corners,
  `a_max` from the existing cone limit (`|I_a_z|·tan θ_cap`), centroid `s[:2]·focal`, constant `ρ_fov_0`.
- New params: `FUNNEL_CBF_K0` (1/s²), `FUNNEL_CBF_K1` (1/s, start `2√k0`), `FUNNEL_CBF_RINNER` (use the
  `r` term vs centroid-point only).
- **Feasibility-floor flag** when the QP is infeasible → raise it for the ring-flow handoff (§8).
  Keep the `I_a[2]<0` upright guard.
- Log for A/B: `h(t)`, `a_des` vs `a_xy*`, the active HOCBF row(s), the feasibility flag.

## 5. A/B methodology (`FUNNEL_MODE` cone vs cbf, n=5 each, IC1, video)

Compare on:
1. **Strangling** — `a_out_desired` vs `a_out_allowed` over the descent. cbf should leave the
   *recovery* component unclamped (no whole-descent strangle) while still bounding outward drift.
2. **Visibility** — `min h(t)` and marker-freeze onset (does keeping the centre in extend feature
   survival?).
3. **Landing** — clean **GT** `vz_end`/`vh_end`/xy (NOT `rel_vel` — EKF velocity is unreliable at
   touchdown; see metric note below).
4. **No fly-aways** — `s_e_n_max`, divergence count (the cbf is the *hard* safety net — it must beat
   the cone's 2/5 divergences without the strangle).

## 6. Open questions / risks (resolve before/with the prototype)
- **Sign of `J`** (which world-lateral direction moves the centroid outward) — fix empirically from
  one logged descent (correlate `a_xy` with `ċ`), or from the manuscript `L`.
- **`r` estimate** when the inner marker is stale/lost — hold last, or fall back to centroid-point
  (`FUNNEL_CBF_RINNER=0`).
- **Tuning `k0,k1`** — start critically-damped (`k1=2√k0`); too stiff → reintroduces strangle, too
  soft → the wall is mushy.
- **Relation to the existing optic-flow PPC funnel** — this CBF is on *centroid position*
  (visibility), the PPC funnel is on *optic-flow error* (the manuscript's `ζ`); they're orthogonal
  but should be checked for interaction.

## 7. Blind-phase soft-landing (perception-death floor) — incorporate Singhal 2025

**[UPDATED 2026-06-05: PLASMC IS the 4D Singhal extension → keep the controller, fix only the FLOW
SOURCE. No separate soft-lander.]**

When `r_inner > ρ` the ArUco centroid+yaw die, but the flow survives on the scene texture. PLASMC's
flow is sparse LK on **marker corners** → dies on decode failure (freeze, line 762). Singhal's
`flowstreamer.FLOW_DIVERGENCE` is **texture-free**: LK on a **fixed grid of concentric rings about
the image centre** (radii 56-64px, 60 pts/ring), radial flow averaged over rings → divergence (the
translation cancels in the mean). The rings track whatever texture is at those pixels (ArUco pattern
edges, board, ground) — **no decode needed.**

**VERIFIED offline on a recorded descent:** ring-divergence stays valid with **~60 pts tracked at
t−0.03s** (corners froze ~0.3s earlier) and tracks GT `vz/Z` through the blind phase — the
texture-free flow survives the marker death. Caveat: **noisy** (per-ring outliers) → needs robust
averaging (median/trimmed-mean) + savgol, not the raw mean.

**Integration:** add the ring sampler to `img_data` (env-gate `FLOW_SOURCE=corners|rings|both`),
decompose the ring field → **divergence** (vertical loom) + **translation** (lateral vel) +
**rotation** (yaw rate), level to the V-frame as corners are, → feed the **existing PLASMC**. The
lateral idea below stands (control v_h via translation, don't hold). The rest of this section is the
original separate-soft-lander framing, **superseded** by the above (kept for the lateral-velocity
reasoning):

**Singhal 2025** (`ArduPilot/scripts/Suresh sir/adaptive_controller_with_IC.py`): pure-vertical
adaptive SMC on the flow DIVERGENCE (loom `vz/z`): `az = SMC(flow_div); vz_cmd = ∫az;
send velocity_z`. Drives camera flow-divergence to `REF = −0.30`; self-terminating soft touchdown
(`vz = flow·z → 0`). Lateral explicitly held (`v_x=v_y=0`). Scale-free (divergence is dimensionless).

**Why it fits:** our ~1–1.6 m/s GT impact is the *flow dying*, not a controller gap — PLASMC's
vertical is the same flow law. Keep the flow alive (texture) and the soft-landing law works.

**Blind-phase stack (scale-free, marker-free):**
- **Vertical:** flow-divergence SMC on the *texture's* loom → soft `vz→0`. Option: hand to the
  simpler/validated Singhal law (`−0.30` ref) for the final phase, bypassing PLASMC's terminal
  κ/DH_D_MAX complexity.
- **Lateral (the adaptation):** old = hold position (`v_xy=0`); new = **control lateral VELOCITY via
  the texture's *translational* optic flow** → brake `v_h→0` (soft, non-sliding) instead of holding
  a centroid we can no longer measure (centroid dead, flow alive). Directly kills the slide.
- **Yaw:** hold (the existing gate).

**Handoff:** trigger on `FEATURE_IS_STALE` / `h<0`. The CBF funnel keeps the target centred *into*
the handoff, so we enter the blind phase **low-velocity and centred** — a good IC for the soft-lander
(its `INITIAL_VELOCITY` assumption).

**Open checks:** (1) does camera flow-divergence survive on the coarse texture at <0.5m (LK on the
squares — `make_coarse_textured_marker.py` is sized for exactly this); (2) translational-flow
sign/scale near the ground.

## 8. Marker→ring flow switch: reliability-gated seamless selection (no debounce)

**Validated 2026-06-05.** The control switch from corner flow (`V_v`) to ring flow (`V_v_ring`)
**cannot** use a debounced "K consecutive corner misses" trigger:

- **A perfect trigger is impossible** — you can't tell a long *flicker* (recovers) from the
  *permanent* death without seeing the future. On the recorded reps both discriminators overlap:
  recoverable miss streaks reach **190 frames** while the permanent death is only **24–76**; marker
  extent before the loss overlaps too (flicker 47–203px, permanent 79–245px). And the takeoff phase
  is itself a 938-frame "recoverable" streak → any trigger must be **post-acquisition**
  (`FEATURE_IS_VISIBLE`).

**Resolution — make a false trigger *harmless*, then no debounce is needed:**
```
corner flow valid (N_flow_corners > 0)      -> use V_v       (corner, primary)
else if ring reliable (N_ring >= ~40)       -> use V_v_ring   (ring, safety net)
else (both unreliable)                      -> HOLD last good (open-loop, final mm)
```
- **Benign on false positive:** `V_v ≈ V_v_ring` (switch-continuity check: R² up to 0.93, small
  handoff Δ) → switching during a flicker hands the controller a *similar* velocity → no disruption.
- **Ring-reliability gate (the gate that matters):** `N_ring ≥ ~40` is clean (0% clipped); `< 20` is
  garbage (23% clipped — verified). This prevents switching *into* a bad ring flow — the false
  positive that would actually hurt, and unlike flicker-vs-death it **is** detectable.

**Caveat (ties to the primary goal):** the ring flow **also degrades at the very touchdown**
(`N_ring` drops to 13–38 < 40 in the final frames) → at the last mm *both* fronts can be unreliable
→ the hold. So **texturing the board serves both layers**: more texture → `N_ring` stays ≥40 longer
(ring safety net survives) AND ArUco decodes longer (**fewer perception deaths — the primary goal**).

**Implementation (when wiring control):** env-gate `FLOW_SOURCE=corners|rings|auto`; `auto` = the
selection above. Log the active source + `N_ring`, `N_flow_corners` for A/B. The ring flow is the
*safety net* — gate its per-frame computation on `N_flow_corners==0` for production so it costs ~0
while the marker is alive (speed finding: ~4ms/frame, loop 83→62 Hz if run every frame).

## 9. Precision: PPC funnel on `s_e_n` (IMPLEMENTED, env-gated, needs tuning)

**Decision (2026-06-05):** keep visibility (cone clamp / CBF) + add a SEPARATE precision PPC funnel on
the NORMALIZED position error `s_e_n` — **decoupled**. (The baseline's `_v` funnel did precision AND
visibility in one mechanism, which would double-constrain the cone clamp; we split them: this funnel =
precision only, cone clamp / CBF = visibility only.)

**Implemented** (env-gate `PLASMC_SEN_FUNNEL`, default OFF = the legacy outer PID): the baseline's
back-mapped PPC (`baseline controller.py:256-289`) on `s_e_n` — `S_s=s_e_n/p_s` →
`ζ_s=log((1+S)/(1−S))` → `G_s` → `ζ̇_sd = −K_rp·ζ_s − K_ri·∫ζ_s − K_rd·ζ̇_s` →
`V_ds_d = G_s⁻¹·ζ̇_sd + S_s·ṗ_s` (replaces the PID at `controller.py:477-480`). Drop-in for the PID at
small error; `s[:2]` only (depth = the `h[2]` funnel). Reuses `K_rp/ri/rd`; params
`PLASMC_{PS0,PSINF,XIS}_{X,Y}`.

**A/B (n=3, IC1):** REGRESSED — 2/3 diverged (`|s_e_n|` ran to **6.6**, 5× past its bound `p_s0=1.2`) —
partly confounded by worse IC starts, but the mechanism is real: the back-mapped form goes *gentle* at
the bound, and the `S_MARGIN` clip caps the control once the error exceeds `p_s`, so the SITL lag lets
it run away (the **same input-authority lesson** as the CBF, §3). **Needs tuning** — looser `p_s0` /
slower `γ_s`, and restore the baseline's hard outlier-containment (force `s_e_n` back inside the bound
on breach) instead of the soft clip. User decision: **keep it, tune it.**

## Metric note (carry-over, important)
`SoftPrecise.rel_vel` uses the **PX4 EKF velocity sampled at/after contact** → reads ~0.02 while the
**clean GT impact is ~1–1.6 m/s**. The "soft" flag is a false positive; judge softness by clean GT
`vz` (savgol, pre-contact). This contaminates all historical SP "soft" classifications.
