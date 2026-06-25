---
name: fov-cone-clamp-deadlock
description: FoV cone clamp (d_min collapse) strangles terminal correction at IC1 (94-100% of final-2s samples) and creates the IC2-5 overshoot deadlock; PLASMC_THETA_FLOOR_DEG knob added; nobody had ever tested relaxing the d_min term
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 7faf44bf-c5f1-4b57-a701-f6d868abfdc1
---

> ⛔ SUPERSEDED/CORRECTED 2026-06-26: Cone clamp already superseded by cbf2; the 'lag→overshoot→clamp→fly-through, needs overshoot-prevention' framing is superseded — the combined surface + lateral velocity damping arrests v_lat before the deck. The design-flaw lesson (a clamp that blocks recovery) endures. The PX4 lateral "wall" was a gain-parity bug + the velocity-damping lever (tighten the lateral flow funnel XI2_xy), NOT a perception/architecture/inner-loop-velocity limit; the combined sliding surface σ=ζ_h+χ_r·ζ_r is baked default-on and gives 10/10 bounded landings. The residual is a terminal SOFT velocity kick (≈38ms lag), not a precision wall. See [[feedback_flow_funnel_zetah_works]]. Content below kept as history.

**The FoV cone clamp's d_min collapse is the dominant limiter of terminal precision (2026-06-03 diagnosis).**

**Mechanism:** `theta_cone = min(theta_current + atan(d_min_fov/f), theta_cap)` (controller.py `_attCtrl`). When marker corners approach the rho_fov envelope edge, `d_min_fov → 0` → `theta_cone` collapses to the *current tilt* → `a_xy_lim = |I_a_z|·tan(theta_cone) ≈ 0` → lateral authority gone, **regardless of `theta_cap`** (this is why THETACAP sweeps never helped — the cap only matters when d_min is large).

**Where it bites:**
1. **IC1 terminal phase:** near touchdown the 1/Z geometry spreads the marker corners wide (marker fills the image) → d_min→0 → the final precision correction is clamped. Measured on DH_D_MAX=5 reps: SMC asks 10–44 m/s² in the final 2 s, clamp allows 0.02–1.7 m/s², **94–100% of samples clamped**. This is the mechanism behind the historical "IC1 xy floor ~0.4 m" ([[feedback-precision-tuning-lessons]]).
2. **IC2-5 overshoot deadlock:** lag → drone overshoots target carrying ~3 m/s → marker swings to image edge → d_min=0 → braking/recovery clamped to ~0.5 m/s² (SMC asking 9–15) → drone drifts away → lands 5–7 m off (the [[multisine-cal-ic25-collapse]] numbers). Every rep shows err-rotation ≈ ±180° (straight fly-through).

**The design flaw:** the clamp assumes tilt moves the marker OUT of the image. But tilting *toward* the marker re-centers it (down-facing camera: optical axis rotates toward the marker). The clamp blocks exactly the recovery action. The frontier sweeps ([[feedback-precision-softness-frontier]]) never touched this — THETACAP is impotent when d_min=0 and RHOFOVINF was only shrunk.

**Fix:** `PLASMC_THETA_FLOOR_DEG` (controller.py, 2026-06-03; default 0 = legacy). Floors theta_cone: 60 = d_min term disabled (theta_cap-only clamp), 15–30 = softened clamp. Safety layers that remain with floor=60: W_U_MAX=1.0 rad/s body-rate clamp, marker-loss grace 1.0 s, KLT fallback.

**Batch-1 context (PID_SCALE=0.54, n=5 IC1):** even WITH the clamp strangling the final 2 s, PID 0.54 + DH_D_MAX=5 gave the best IC1 cluster ever: xy [0.44, 3.02(TL), 0.77, 0.125, 0.143], 2 SOFT, two reps < 0.15 m.

**Batch-2 results (2026-06-03) — SP #6 ACHIEVED:**
- **floor=60 IC1 n=5: rep 4 = SOFT+PRECISE (xy 0.060, vel 0.149)** — first mechanism-driven SP ever (not IC luck: the luckiest-IC sibling rep landed 0.80 m). Arm xy mean 0.83, 0 TL. Rep data git-tracked: `SPCampaign/b2A_floor60_20260603-033719/rep4/`.
- floor=60 + PID 0.54 stack: 0 SP, no improvement over floor alone (stacking-cancels-benefits again — don't stack these).
- floor=60 IC2-5: xy halved (5.84→2.95, IC5 hit 0.87!) but impacts violent (vel 1.13→3.28). The floor breaks the recovery deadlock, but the lag-overshoot still happens — off-center needs overshoot PREVENTION (approach-speed/descent-time shaping), not just recovery authority.

**New global bottleneck after the fix: touchdown softness under aggressive terminal correction.** Non-SP reps land precise-ish (0.6–1.8 m) but hot (0.9–3.5 m/s). Next-batch candidates (UNTESTED): moderate floor (20–30°), floor + DSD_CLAMP=4.5, floor + slower descent (h_rd −0.21), altitude-scheduled floor.
