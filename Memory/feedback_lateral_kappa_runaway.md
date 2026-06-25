---
name: feedback_lateral_kappa_runaway
description: "Lateral κ-runaway = touchdown funnel breach from a WRONG h_d, root-caused 2026-06-10 (GT+data-verified). The controller uses the VIRTUAL (tilt-leveled) centroid s; the CBF uses the ACTUAL image centroid — code split VERIFIED (s←_virtual_feature_pts via _getVirtualPts; cbf2 cr2←_feature_pts=C_nP). At the breach they DIVERGE: actual centroid frozen at ~(70,38)px = IN-FoV corner, but the VIRTUAL reprojection of that near-edge feature under the large touchdown tilt swings OFF-SCREEN (s_y≈-3=-820px). Off-screen virtual s → cross(w_i,s) fabricates h_d≈-8 (NOT the outer PID: ds_d≈+0.1; NOT a flow spike: measured h≈1 rad/s matches GT v/Z; h_d=-8 would need 2.7 m/s vs real 0.4) → h_e breaches funnel → κ_xy runs away (7.26) → harder tilt → more off-screen virtual-s (positive feedback). CBF can't catch it (guards the in-FoV actual centroid). Fix = clamp the VIRTUAL s in cross(w_i,s) + keep drone centered (small tilt→virtual≈actual) + flag LOST on stale feature; NOT a gain/cap."
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 7415f420-9591-41b1-8349-bb9361a8dc82
---

> ⛔ SUPERSEDED/CORRECTED 2026-06-26: The κ-runaway mechanism is valid old-form history, but the prescribed fix (keep marker DECODED / KLT corner-track) and 'binding limit = LK dynamic range ~2 m/s' are obsolete; cross(w_i,s) is also dropped under the baked CH_CLEAN h_d. The PX4 lateral "wall" was a gain-parity bug + the velocity-damping lever (tighten the lateral flow funnel XI2_xy), NOT a perception/architecture/inner-loop-velocity limit; the combined sliding surface σ=ζ_h+χ_r·ζ_r is baked default-on and gives 10/10 bounded landings. The residual is a terminal SOFT velocity kick (≈38ms lag), not a precision wall. See [[feedback_flow_funnel_zetah_works]]. Content below kept as history.

**The lateral κ-runaway is a touchdown funnel breach driven by a WRONG `h_d`** — specifically the kinematic feedforward `cross(w_i, s)` evaluated on an **off-screen VIRTUAL centroid**. Root-caused on P_z=8 rep3 (κ_xy=7.26, 21 m TL), GT + data verified 2026-06-10.

**TRIGGER = ArUco detection loss, which PRECEDES the runaway** (loss at 0.80 of flight vs runaway at 0.88). The marker is mostly STILL IN the FoV when ArUco loses it (9/12 TLs had 4/4 corners in-FoV — a **DECODE** failure, not a geometric loss; [[feedback_marker_detection_stale]]). On loss, img_data freezes `_feature_pts` + **extrapolates** the centroid + logs the nan-quat sentinel (img_data:1007-1009); that stale/extrapolated feature (and, in the active phase, the tilt-reprojected one) becomes the off-screen virtual `s`. **Fix the trigger too:** during marker-LOST use genuine data (FC quat + KLT in-FoV corners), never nan+extrapolate `s` off-screen (user directive 2026-06-10).

## Two centroids — verified code split (the crux)
- **Controller `s`** (feeds `h_d`, `s_e_n`, the c-term) = the **VIRTUAL** (tilt-leveled) centroid, from `_virtual_feature_pts` = `_getVirtualPts(C_nP, quat)` (img_data:808-809,916).
- **cbf2 `cr2`** (visibility QP) = the **ACTUAL** image centroid, from `_feature_pts` = `C_nP` (img_data:915; controller.py:1060).
- At the breach they **DIVERGE**: actual centroid **frozen at (70,38) px = IN-FoV corner** (100% of the terminal phase, stale/held); VIRTUAL `s` = `_getVirtualPts(that, large tilt)` → **−820 px, OFF-SCREEN**. The actual is frozen yet the virtual *moves* → the virtual excursion is **tilt-driven** (the leveling reprojects a near-corner feature off-screen under the ~60° touchdown/runaway tilt).

## Mechanism
```
off-center at low alt → marker drifts to FoV edge/corner; detection freezes the ACTUAL feature at ~(70,38)px
  → under the large tilt, VIRTUAL s = _getVirtualPts(near-corner actual, tilt) swings OFF-SCREEN (s_y≈-3)
  → h_d = ds_d + cross(w_i,s) + loom·s :  ds_d≈+0.14 (PID negligible), loom≈+0.69,
       cross(w_i,s) = -w_z·s_y = -(-3.0)(-3.03) = -9.0  → h_d ≈ -8.2  (SPURIOUS, from off-screen virtual s)
  → actual flow h is PHYSICAL (measured ≈ -1.6 rad/s; GT v_lat 0.4 m/s @ 0.34 m → flow≈v/Z≈1; h_d=-8 needs 2.7 m/s)
  → h_e = h - h_d ≈ +8 → inner funnel BREACHES |h_e/p_2|→1 → ζ≈5.3 → σ≈3.6, G≈3.1
  → κ-ODE growth θ·N·G·|σ|≈16.1 beats leakage N·P·κ≈0.10 by 160× → κ_x slams up, UNBOUNDED (7.26)
  → a_u_xy 631 → harder tilt → virtual s flies further off-screen → POSITIVE FEEDBACK → drift/TL
```
Discriminator = how off-center: runaways breach off-center (marker at FoV edge → virtual s off-screen under tilt); clean reps stay centered (small tilt → virtual ≈ actual, s bounded) or only breach in the last cm. Verified on KP=9 (P_z=8 rep3, EZ0.5 rep1) AND KP=12 (rep2). Supersedes the earlier "1/Z spikes the flow" and "outer-PID over-demand" claims (both wrong: ds_d negligible, h physical).

## Why the CBF doesn't prevent it
The cbf2 guards the **ACTUAL** centroid, which at the breach is frozen at the **in-FoV corner (70,38)** with ~no drift → it sees "still in view" and barely acts. The controller's poison is the **VIRTUAL** centroid, which the CBF neither sees nor protects. **Actual and virtual diverge under large tilt** — so a visibility CBF on the actual image cannot bound the controller's virtual feedforward. (Also: the actual feature is *stale/frozen* — the marker is effectively lost but held at the corner — so the CBF is doubly blind.) See [[feedback_cbf_theta_cap]], [[project_cbf_visibility_design]]; cbf2 only bites with THETA_FLOOR<60 anyway (baked=60).

## Why NO gain bounds it
- **P can't.** Growth=16.1 needs `P_xy≈800` to balance; `κ_eq∝1/P` only holds for moderate growth, not the barrier singularity. (`E_z=0.5+P_z=8` FAILED — [[feedback_descent_softness]].)
- **θ-freeze can't** (θ moderate 37–72; σ,G from the barrier drive it). **Singhal `_contained` MISSES it** (fires at |h_e/p|≥1.0; growth is at 0.9–0.99). **κ_xy is UNCAPPED** (`KAPPA_MAX=[1e6,1e6,3.0]`, only z capped → κ_x hit 7.26 vs κ_z's 3.0).

## The fix — IMPLEMENTED + TESTED (2026-06-10): bound the PHANTOM s, NEVER the GENUINE s
The off-screen s has **TWO regimes**, and clamping the wrong one regressed:
- ✅ **#1 — clip the marker-LOST EXTRAPOLATION to ±(p_10+δ)** (img_data, env `PLASMC_FEAT_FOV_CLIP`, **DEFAULT-ON**). Conditional (only when the marker is provably lost) → bounds the *phantom* extrapolation that caused the original κ-runaway (P_z=8 rep3, marker lost); never touches a genuine detection. + keep the FC quat valid through marker-LOST. **The keeper.** (Closed-loop re-test FeatClip_EZ_IC1 pending.)
- ⛔ **#2 — global `_getVirtualPts` clamp (z_v≥0.01 + output ±(p_10+δ))** (env `PLASMC_VIRT_GUARD`, **DEFAULT-OFF — REGRESSED**, VirtGuard_EZ_IC1). It clamped the GENUINE in-FoV centroid when the drone was off-center — the far-drift reps did NOT lose the marker ("lost 0%") — → bounded s_e_n + the κ-growth → SMC **UNDER-corrected** (a_u≤8, κ≤0.2) → far drift (29/91 m; E_z=1.0 mean 1.94→10.3). **The genuine off-center s IS the error signal AND drives the κ-growth authority; clamping it kills the correction.** → OFF.
- **A1: keep the drone centered (convergence-ordering, gate descent on |s_e_n|)** → marker stays off the FoV edge → s genuine + bounded. (Untested.)
- **B1/B2 (backstops only):** κ_xy cap=3.0; freeze κ at |h_e/p|≥0.9. Bound the symptom κ, not the cause.
**LESSON:** clamping the **GENUINE** feature suppresses the correction authority (open-loop-validated, closed-loop-negative — 3rd such, after dw-rewrite + θ-freeze); bounding the **PHANTOM** (marker provably lost) is fine. The real upstream lever: keep the marker **DECODED** (KLT corner-tracking) so `s` is never a phantom.

## Lever analysis (corrects "all gain levers exhausted")
- `gamma_s` (XIS) = OUTER funnel `p_s` contraction (controller.py:515-517) = lateral-convergence speed. **SWEPT (NC56-59):** 1.2 → 0 TL but 1/5 hover (over-centers → weak descent); ≥1.4 degrades (1.4 catastrophic 8.7 m). Ceiling = descent-weakening, NOT a demand-breach.
- `gamma` (XI2) = INNER funnel `p_2` contraction → raising it accelerates the breach. Don't.
- **KP=12 (NC60):** tightest landings ever (0.34 m, no t=0 LK collapse) but 1/5 the touchdown breach fires harder → `KP=12 + κ_xy cap` only a backstop.
- **Binding limit = LK dynamic range ~2 m/s** ([[feedback_lk_dynamic_range_limit]]); clamps W_U_MAX=1, theta_cap=60°, DH_D_MAX=50 cap the correction rate to stay within it.

**How to apply:** WRONG `h_d` from an off-screen VIRTUAL centroid, GT-verified — not a gain, not the outer PID, not the actual flow. The off-screen `s` has TWO regimes: clamp the **PHANTOM** (marker-LOST extrapolation, #1, ON) but **NEVER the GENUINE in-FoV `s`** (#2 global clamp REGRESSED → under-correction → far drift). The real upstream fix: keep the marker **decoded** (KLT corner-track) so `s` stays genuine; the CBF can't help (guards the actual centroid).
