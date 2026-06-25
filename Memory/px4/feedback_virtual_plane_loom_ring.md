---
name: feedback_virtual_plane_loom_ring
description: "⭐ (2026-06-22) MATLAB computes ALL flow/feature on DE-ROTATED VIRTUAL POINTS (V_nP_i = V_R_C·rays), NOT a warped image, NO LK, NO rings (synthetic corners). PX4's CORNER flow ports this faithfully (LK→_getVirtualPts→difference) and is the GT-ACCURATE consumed signal (corr 0.87). FIXED: pure_div (ring Singhal loom) was computed in the REAL image plane (radial div about REAL centre) → GT corr ~0; now VIRTUAL plane (radial div of de-rotated V1-V0 about V-origin) → corr 0→0.65, BAKED default-on (but ~INERT: pure_div is the EKF fallback, active ~1% of frames, 0% terminal → no closed-loop re-runs needed). V-FRAME RING (PLASMC_RING_VFRAME, default-off): ring formed in virtual plane → reproject POINTS to real (_getRealPtsFromV, NOT image warp) → LK on real → back to virtual → h,w; makes V_v_ring (consumed ring loom) GT-accurate 0.58→0.84. But closed-loop n=15 NEGATIVE: vertical launches 5/15→5/15 unchanged, terminal vz WORSE. Rings = a down-weighted safety net done correctly, NOT the loom fix. The vertical launch is κ-amplified terminal perception, not loom-accuracy → no estimator swap fixes it."
metadata: 
  node_type: memory
  type: feedback
  originSessionId: eba9fa95-5b93-4294-bbca-81468bb36670
---

**The MATLAB↔PX4 flow architecture (the load-bearing understanding).** MATLAB
`visualControl_IBVS_adaptive.m` computes everything on the **de-rotated virtual points**:
`V_R_C = I_R_V'·I_R_C; rays=[C_nP; f]; vr=V_R_C·rays; V_nP_i = f·vr(1:2)/vr(3)` — rotate the
camera rays into the gravity-leveled frame and re-project. Then feature `image_feature(V_nP_i)`,
flow `dPdt = d(V_nP_i)/dt`, `V_v = pinv(L_s)·dPdt`, moment loom `M=Σ(V_nP_i-mean)², -½d(lnM)/dt` —
**all on `V_nP_i` (virtual points)**. It is a **POINT transform (rotate rays), NOT an image warp**;
there is **no LK** (corners are synthetically projected, always present, constant 4-corner set) and
**no rings**. `_getVirtualPts` in PX4 IS exactly this `V_R_C·rays` re-projection.

**PX4 corner flow is a faithful port + is the accurate consumed signal.** PX4 LK-recovers corners
from real images, then de-rotates: `V0=_getVirtualPts(r0,q0), V1=_getVirtualPts(r1,q1)`, flow=`V1-V0`,
`pinv(L_s)`. So PX4 ALREADY "computes flow on the virtual image" the MATLAB way (point de-rotation).
GT-validated: corner loom **corr 0.84-0.87** vs the true loom — the most accurate signal. Control
consumes the corner loom; the ring is a SAFETY NET.

**FIX 1 — pure_div to the virtual plane (BAKED, default-on).** The ring Singhal loom `pure_div`
(`Ring Divergence`) was the ONE signal computed in the REAL image plane: `rvec=self.center-r0;
radial=(r1-r0)·rvec/|rvec|²` — radial divergence of real-px flow about the REAL centre, never
de-rotated → **GT corr ~0 (uncorrelated garbage that merely looked "bounded")**. Fixed to compute
the radial divergence of the **de-rotated `(V1-V0)` about the V-frame ORIGIN (`rvecV=-V0`)** →
matches the moment loom / V_v_ring / MATLAB. GT corr **0.0→0.65**. Baked (not gated). **BUT nearly
INERT**: pure_div is fed to the fusion EKF ONLY in the `elif ring_loom_ok` branch = when the ring
lstsq V_v_ring is REJECTED (audit: active ~1% of frames at altitude, **0% at terminal**). So the
fix is a correctness fix to a near-dead fallback → **NO closed-loop A/B needed re-running for it**
(the session's E_z/gate/commit/N_z results stand; pure_div was inert in all of them).

**FIX 2 — V-frame ring (PLASMC_RING_VFRAME, DEFAULT-OFF).** The user's pipeline: ring formed on the
VIRTUAL image plane (concentric about the nadir) → **re-projected (POINTS, via `_getRealPtsFromV`,
the verified machine-precision inverse of `_getVirtualPts` — NOT an image warp)** to the real image
each frame → LK on the real image → tracked points projected BACK to virtual (`_getVirtualPts`) →
`h_i,w_i` from `pinv(L_s)`. The fixed real-image ring samples an OFF-NADIR slanted patch under tilt;
the V-frame ring tracks the NADIR patch (uniform depth). Signal win: `V_v_ring` (the *consumed* ring
loom, ~99% of frames) GT corr **0.58→0.84**, matching the corner loom. **Closed-loop n=15 A/B
NEGATIVE** (`RingVF15`): vertical-launch rate **5/15→5/15 (unchanged)**, terminal vz_p90 **6.9→12.9
(worse)**, xy_std 9→20. The n=5 vz hint (5.14→3.54) was noise, flipped at n=15. Default-off.

**WHY the accurate ring loom doesn't fix the vertical launches.** (1) The ring is a DOWN-WEIGHTED
safety net behind the corner loom; it only carries the loom at `n_corn≤3` (RING_LOOM_NCORN), a narrow
terminal window. (2) The signal was validated at alt 0.5-3m, NOT the <0.3m launch zone where both
corners AND ring stations degrade. (3) The launch is **κ/c-term-amplified** ([[project_current_state]]
trace): the quadratic c-term `-h_z·h` + κ adapting to the corrupted σ amplify ANY terminal loom
residual → an accurate-on-AVERAGE loom doesn't prevent the terminal transient. **The vertical launch
is κ-amplified terminal perception corruption, NOT a loom-accuracy problem — no estimator swap fixes
it.** (κ adapts to the MEASURED σ; when the sensor lies terminally, κ amplifies the lie — the cap
KAPPA_MAX_Z is INERT because κ_z stays LOW 0.19-0.90 during the launch, like MATLAB CB46.)

**DEAD-ENDS / DON'T RE-DO.** (a) Image-warp for the virtual flow — MATLAB de-rotates POINTS, not the
image; PX4 already does. (b) Rings as the loom/vertical-channel fix — signal-correct now but
closed-loop negative; they're a safety net, not a lever. (c) Moment loom (corner, FLOW_LOOM_DECOUPLE)
to fix the terminal sign-flip — it uses the SAME corner set → same set-change artifact. (d) Switching
to the ring loom at terminal (RingOT / over-target handoff, REVERTED) — negative even with the
accurate V-frame ring.

**WHAT THE TERMINAL LOOM PROBLEM ACTUALLY IS.** The corner loom is accurate (0.87) except for the
**terminal SET-CHANGE sign-flip**: the loom is d/dt of the corner spread/area, and at the deck the
corner SET loses members (board markers drop, Nfc 16→4→0) → the apparent spread jumps discontinuously
→ the loom flips ±10 in one frame. MATLAB never has this (constant synthetic 4-corner set). The
MATLAB-faithful fix = compute the loom on a CONSISTENT corner subset (primary marker's 4) and/or an
innovation gate on the loom — NOT rings, NOT warping. (Untested; the κ-amplification means even this
may not close the launch — the terminal is fundamentally hard.)

**UPDATE 2026-06-24 — fixed real-image ring REMOVED entirely; V-frame ring is now the ONLY ring** (user:
the real-image ring is geometrically WRONG — samples off-nadir under tilt). `PLASMC_RING_VFRAME` knob +
`_ring_pts0`/`_ring_vframe` deleted. Pipeline for ALL ring axes (divergence, moment, 6-DOF lstsq):
V-plane stations → `_getRealPtsFromV(quats[0])` → LK on real → `_getVirtualPts` back to V → flow on V0/V1.
CORRECTNESS fix, NOT a launch/loom fix — the n=15-negative result above STANDS (rings don't fix the
terminal launch; that's CONTROL-side = the terminal tilt-thrust balloon, see [[feedback_ic_yaw_target_fix]]).
NEW near-deck ring-loom-NOISE root cause (confirmed 2026-06-24): the single ~1m marker OVERFLOWS the FoV at
Z≈0.4m → ring stations land on the marker's UNTEXTURED interior (LK aperture) → garbage flow; AND BOTH corner
AND ring loom collapse to ~0 GT-corr below 0.7m (ring_div +0.33@1.5m→-0.03@0.4m; corner +0.67@2.5m→0.01@0.4m)
→ loom is UNOBSERVABLE near the deck in the single-marker world. The V-frame ring fixes the off-nadir-tilt
half, NOT the untextured-interior half (the nadir near the deck IS the untextured marker). Divergence
ALGORITHM confirmed sound (radial mean about V-nadir, post the 2026-06-22 virtual-plane fix); the noise is
an INPUT/observability failure, not an algorithm bug. **Knobs:** none (V-frame ring + virtual-plane pure_div
both baked, no knobs). The corner flow / `_getVirtualPts` is the MATLAB-faithful accurate consumed path.
