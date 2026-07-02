---
name: feedback_backstep_tried_clamps_are_lever
description: "⭐⭐ Backstepping h_d (barrier-inversion desired feature rate) was TRIED then REMOVED: it fixes the centered IC1 stragglers + restores velocity authority, but OVER-DEMANDS on off-center ICs (unbounded in zeta_r) and needs band-aid caps. The 2x2 (clamps x backstep) showed the DOMINANT lever is removing the band-aid clamps (W_U_MAX, theta_cap) — they DISTORT the over-aggressive terminal command into the IC3 fly; remove them and even the BLIND law gives IC3 2/3 SP. BUT those two clamps are load-bearing (W_U_MAX guards LK in perception, theta_cap is thrust-deliverability) so they CANNOT be baked off — the interference is a symptom of OVER-COMMAND (the descent-start seed), fix that not the clamps. No inert band-aid is safe to remove (KAPPA_MAX is load-bearing in drift reps; izeta/iV_s_e_n are canonical anti-windup). cbf2 finalized as the only visibility mechanism."
metadata: 
  node_type: memory
  type: feedback
  originSessionId: f1723146-d87f-40d0-9a34-49233fbc4e72
---

## 2026-06-26 — Backstep h_d tried+removed; the 2x2 says the band-aid CLAMPS are the dominant lever (user-led, GT-FB)

### The velocity-blindness (corrected mechanism — common-mode cancellation, NOT "h = s_dot_meas")
The combined-surface `h_d` carries the MEASURED feature rate `s_dot_meas` → the flow barrier `zeta_h` is blind to un-commanded lateral drift at altitude. **Precise mechanism (an earlier "h ≈ s_dot_meas, same quantity" shorthand was WRONG):** `h[:2]` is the lstsq-decomposed TRANSLATIONAL flow; `s_dot_meas` is the RAW total centroid rate — NOT identical. The blindness is **common-mode cancellation of the drift**: a lateral drift `v_lat` produces a translational flow `v_lat/Z` that appears in BOTH `h[:2]` (measured) AND in `h_d` via the `s_dot_meas` feedforward term → it cancels in `h_e = h - h_d` → `h_e` doesn't see the drift (it's the small rotation/descent feedforward residual, ~0.05, regardless of drift). Data: fly rep drifted at 0.7-1.9 m/s (flow 0.16-0.42) yet `|h_e_xy|` stayed 0.05-0.11. So NOT `h_e≈0`; `h_e ≪ drift flow`.

### Backstep h_d (barrier inversion) — TRIED, then REMOVED (PLASMC_HD_ZETAR, reverted)
Replace `s_dot_meas` in `h_d` with the DESIRED feature rate that drives `zeta_r->0`:
`rdot_e_des = phi_max*(zetadot_r_des/g_r + S_r*p_r_dot)`, `zetadot_r_des = -lambda*zeta_r`, `phi_max=p_10`, `g_r=2/((1-S_r^2)p_r)`. (manuscript symbol `phi_max=R/(2f)`; user-derived, validated against code lines 866-873.) This makes `h_e = h - rdot_e_des` a genuine velocity-tracking error → `zeta_h` regains authority. **Result (GT-FB):**
- **IC1 (centered): 3/3 SOFT+PRECISE** — fixes the straggler at the strict COMMIT_SEN=0.35. `h_e` non-zero at altitude (0.03-0.07), `zeta_h` engages, terminal `a_u` 810->2 (velocity managed throughout, no 1/Z eruption), kappa CALMER than baseline (0.15±0.04 vs 0.19±0.12), NO limit cycle, NO clamps fire. Clean.
- **IC3 (off-center): REGRESSES.** `rdot_e_des` is UNBOUNDED in `zeta_r` and peaks at MID-range `S_r≈0.65` (shaping factor `zeta_r(1-S_r^2)` max ≈0.89) = exactly where off-center ICs start → demands `~0.45 rad/s ≈ 1.8 m/s inward at 4m`, infeasible to reverse the outward seed → `h_e` blows to 0.5, `zeta_h` saturates, sigma_xy boundary-layer chatter 30%, kappa_xy ratchets to 4.7, W_U_MAX/tilt clamps bite. lambda=0.5 tempered (IC3 1 SP + 1 borderline + 1 fly) but didn't fix the worst seed; a feasibility cap (HD_ZETAR_MAX) = band-aid.
- **REMOVED** because the 2x2 below showed the clamps, not the backstep, are the lever; backstep adds aggression that over-demands off-center.

### THE 2x2 (clamps x backstep, IC3, GT-FB, velocity-damping XI2_xy=0.7 + loom gate throughout)
| | clamps ON (default) | clamps OFF (W_U_MAX=100, theta_cap=85) |
|---|---|---|
| **blind h_d** | flies (>=2m) | **2/3 SP (0.048,0.034) / 1x0.27, 0 fly** |
| **backstep h_d** | 1 fly + chatter (2.08) | 0 fly, 0.38-0.60 (no SP) |
- **Removing the band-aid clamps (W_U_MAX, theta_cap) is the SINGLE biggest lever** — eliminates the IC3 fly even for the BLIND law (2/3 SP), BETTER than backstep-no-clamps. The "over-demand -> fly/chatter" was largely the **clamps DISTORTING an over-aggressive terminal command** (W_U_MAX clips body rate -> wrong attitude -> kick), not the law going unstable. Unclamped, the law is bounded (no fly), just plateaus ~0.4-0.5m on the worst seed (the genuine residual = the seed not fully braked from altitude).
- **BUT the two interfering clamps are LOAD-BEARING and CANNOT be baked off:** `theta_cap`(60°)=thrust-deliverability (cos60=0.5 vertical thrust; 85° = drone plummets), `W_U_MAX`(1.0)=guards LK in perception mode (>1.7 rad/s loses the marker). Relaxing them is a GT-FB tuning diagnostic only. **The interference is a SYMPTOM of OVER-COMMAND (the descent-start seed); fix the seed, not the clamps.**

### ⭐ NO-CAPS IC1-5 GATE (the deployable config: backstep/combined surface + W_U_MAX=100 theta_cap=85 + loom gate, GT-FB n=3, bundle 20260626-113326) = 10/15 SP — best yet
Confirms caps were the failure source. Tally **10 SP / 1 missSOFT / 1 missPREC / 1 missBOTH / 2 FLY**:
- **IC1 3/3 SP, IC2 3/3 SP** (centered/moderate — no seed, no problem).
- **2 genuine flies = the worst descent-start SEED**: IC3_rep2 (0.63), IC4_rep1 (0.90/vel2.72, v@minZ 2.99) — concentrate at off-center IC3 + HIGH-START IC4 (7m gives the seed most altitude to grow). Actuator/lag-bounded large seed; not a gain.
- **3 borderline (within 0.01-0.02 of a threshold)**: IC3_rep1 xy 0.020 perfect but vel 0.220 (soft miss by 0.02 = LAG floor); IC5_rep1 xy 0.112 (prec miss by 0.012 = seed residual); IC4_rep3 xy 0.106/vel 0.458.
- **Only TWO boundaries keep it off 15/15**: (A) the LAG floor (terminal vel ~0.2-0.46 on the borderline reps, 38ms) and (B) the worst-seed FLY (2 reps, IC3/IC4). Both are physical, not controller-gain. The caps-ON gate's flies (3.45m) become SP-or-near-SP without caps. Terminology: "backstep" = the baked COMBINED SURFACE (PLASMC_COMBINED_BARRIER=1), NOT the removed HD_ZETAR sub-experiment; "blind h_d" = this same combined surface (its h_d=measured s_dot makes zeta_h degenerate). NEXT = perception-ON (the caps, esp. W_U_MAX, are RE-IMPOSED by LK under real perception >1.7 rad/s).

### Clamp audit — NO inert band-aid is safe to remove
Checked all caps/clamps vs the code/memory: the ones that *interfere* (W_U_MAX, theta_cap) are necessary (above); the ones that *look* inert are load-bearing or canonical:
- `KAPPA_MAX` (κ_z=3.0): code comment "load-bearing in BAD reps, inert in good ones" — holds κ_z from running to 10+ in the IC3/IC4 DRIFT reps (the cases we care about). Keep. (κ_xy already uncapped 1e6.)
- `izeta_clamp`/`iV_s_e_n_clamp` (5.0): canonical anti-windup (category A). Keep.
- `_ie_a_clamp`: already removed 2026-06-08 (->conditional integration).
- yaw `psid_rate` clamp: now `=1.0·W_U_MAX` = the yaw-axis body-rate limit. Keep.
**There is no "unnecessary AND interfering" clamp.** "Inert in good reps" = the backstop that fires in the bad reps. The clamps each guard a real failure mode.

### cbf2 FINALIZED (visibility)
The alternate visibility methods were RETIRED from the code (2026-06-26): the magnitude `cone` clamp, the lean-magnitude `cone0`/`cbf1` forms, the abandoned optic-flow HOCBF, the `FUNNEL_MODE` selector + `CONE0_*` env, and `tools/calibrate_cone0/cbf1_sign.py`. **cbf2 (camera-plane θ-QP, `src/cbf_visibility.py::cbf2_filter`) is the only mode** (was the validated default; the alternates were dead env-gated paths). cbf2 is a PRINCIPLED constraint (barrier function w/ FoV guarantee), NOT a band-aid — keep. Docs scrubbed (FUNNEL_CBF_DESIGN/PARAMETER_ANALYSIS/PLASMC_TUNING_GUIDE). Smoke-test IC1 2/2 precise post-refactor.

### Standing conclusion / next
The KEEPERS from the velocity-damping thread: **XI2_xy=0.7 funnel + the loom commit gate** ([[feedback_loom_commit_gate]], [[feedback_flow_funnel_zetah_works]]) — those gave GT-FB IC1-5 9SP/1P/2fly. Backstep + caps are OFF the table. The remaining residual is the **descent-start seed** (un-nulled outward velocity at the approach->IBVS handoff) — the controller-side fixes can't fully brake a large seed from altitude (actuator-bounded). Real next lever = kill the seed at the source (IC-handoff radial-velocity / yaw-rate gate). h_rd CONSTANT. Corrects/continues [[feedback_loom_commit_gate]] root-cause section.
