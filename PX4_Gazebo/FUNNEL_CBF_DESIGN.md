# Funnel redesign: centroid CBF for multi-marker target visibility

**Status (2026-06-05):** CBF/cone-clamp **KEPT** (not superseded). `ρ_fov` now held **CONSTANT at
`ρ_fov_0`** in code (`PLASMC_LFOV=0` default — stops the early perception-death from the 80px shrink).
Precision is being added via a **PPC funnel on the virtual image POSITION `s`** (ported from the
baseline; see §9), NOT by shrinking the visibility envelope.

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
ρ     = FoV envelope (per-axis: ρ_u, ρ_v), the existing rho_fov_curr
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

## 3. The constraint: a directional CBF (allows recovery, walls off drift)

`h` has **relative degree 2** w.r.t. lateral accel `a` (`a → v → c`): `ċ = J v`, `c̈ = J a + J̇ v`,
where `J` is the image interaction matrix (downward camera; `J ≈ −(1/Z)·R_img`, sign fixed once
empirically/from `L`). Use a **High-Order CBF**:

```
ḣ  = ∂h/∂c · ċ            (= −sign(c_axis)·ċ_axis − ṙ   on the binding axis)
HOCBF:   ḧ + k1·ḣ + k0·h ≥ 0
```

`c̈` is linear in `a`, so this is **one linear inequality in `a_xy`**. Enforce by projecting the
SMC's desired `a_xy` onto it (clamp only along the gradient ∇_a(ḧ)). Geometrically this bounds the
**outward radial** component of `a_xy` (the part driving the centroid toward the binding edge) with
the bound **→0 as h→0**; the **inward (recovery) and tangential** components are left free. → No
strangling, hard wall at the FoV edge (CBF forward-invariance).

**Practical 1st-order start (recommended first cut):** use the *measured* centroid velocity from
optic flow (`ċ ≈ h_flow[:2]`) instead of the `J̇v` term:

```
û        = c / |c|                                   # outward image direction (binding axis)
ċ_out    = ċ · û        (measured, from optic flow)  # how fast centre is drifting out
a_out    = (a_xy mapped through J) · û               # outward accel the command would add
# barrier: don't let the centre approach the edge faster than the margin allows
a_out_max = clip( k0·h − k1·ċ_out , 0, ∞ )           # →0 as h→0; allows inward (a_out<0) freely
if a_out > a_out_max:  a_xy -= (a_out − a_out_max)·(J⁻¹ û)   # clamp ONLY the outward radial part
```

This is implementable with what's already logged (`s`, optic flow, marker corners for `r`), avoids
the `J̇` term, and is the minimal change that makes the clamp directional + centroid-based.

## 4. Implementation plan (env-gated, A/B-able)

- New block replaces lines 770–817 **behind a flag**:
  `FUNNEL_MODE = os.environ.get("FUNNEL_MODE","cone")` → `"cone"` (current, default) | `"cbf"`.
- Reuse: `rho_fov_curr` (764-768), `theta_current` for the upright safety (812-813), `_img_node._feature_pts`
  (for `r`, primary corners), the centroid from `self._s[-1][:2]` and focal `self._img_node.focal`.
- New params: `FUNNEL_CBF_K0` (≈ barrier stiffness, 1/s²), `FUNNEL_CBF_K1` (≈ damping, 1/s),
  `FUNNEL_CBF_RINNER` on/off (use `r` term or centroid-point only).
- Keep the `I_a[2]<0` upright guard and the `−50` floor unchanged.
- Log additions for A/B: `h(t)` (centroid margin), `a_out_desired`, `a_out_allowed`, the binding axis.

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

## 9. Precision: PPC funnel on virtual image POSITION `s` (port from baseline) — PLANNED

**Decision (2026-06-05):** keep the cone clamp (constant `ρ_fov_0`, visibility) AND add a PPC funnel
on `s` (precision), per the baseline `~/ws/scripts/soft_precise_landing/controller.py`.

**Mapping finding / ISSUE:** the baseline's outer-loop funnel (`:249-290`, the `_v` funnel) is a
log-barrier **VISIBILITY funnel on `s`** — bounds start at the image edge (`p_0_v ≈ s_d + centre/
focal`), shrink to `p_inf_v=[0.3,0.3]` → it does precision (tight terminal) AND visibility (image-edge
start) in ONE mechanism. The PX4 port replaced it with a **plain PID on `s_e`** + the cone clamp for
visibility. So porting the `_v` funnel **overlaps the cone clamp's visibility** (double-constraint).

**Recommended (decouple):** `s`-funnel = **precision only** — mirror the velocity funnel `p_2_*`
(`p_1_0/p_1_inf/γ_1`, log-barrier `ζ_s`+`G_s`); terminal `p_1_inf` = the xy precision target, NOT the
image edge. Output replaces the PID at `V_ds_d` (controller.py:477-480); `s[:2]` only (depth = the
`h[2]` funnel's job). Cone clamp keeps the tilt/accel visibility. Knobs `PLASMC_{P10,P1INF,XI1}_*`,
env-gated for A/B vs the PID. **Open: confirm decouple-vs-subsume before coding.**

## Metric note (carry-over, important)
`SoftPrecise.rel_vel` uses the **PX4 EKF velocity sampled at/after contact** → reads ~0.02 while the
**clean GT impact is ~1–1.6 m/s**. The "soft" flag is a false positive; judge softness by clean GT
`vz` (savgol, pre-contact). This contaminates all historical SP "soft" classifications.
