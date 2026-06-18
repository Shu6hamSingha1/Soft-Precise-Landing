---
name: feedback_descent_hover_thread
description: "Descent-hover investigation (2026-06-10) + TL-cause analysis. E_z=0.5 fixes the 1/5 hover but un-bounds κ (one-knob-one-job: E_z sets descent stiffness AND κ-bound). BAKED: cleaner image-rate dw (tidies θ-peak, integrator-safe). DEAD-ENDS: image-rate-HOLD dw (poisons κ-integrator) + sustained-high-θ κ-freeze (mis-targeted — κ ratchets at MODERATE θ via G·|σ|, not spikes). E_z=0.5 κ-bound lever = P_z (κ_eq∝1/P), untested. TL causes: 2/6 κ-runaway (control), 4/6 PERCEPTION (3 close-range touchdown marker-loss at 1-3m, 1 far drift)."
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 7415f420-9591-41b1-8349-bb9361a8dc82
---

**Thread:** trial-49 IC1 (E_z=1.0, baked) had a 1/5 descent **hover** — the drone parks at ~5.6 m and never descends because `σ_z` stays inside the `E_z=1.0` boundary layer, so `sat(σ_z/E_z)` and `a_u_z≈1` are too weak to start the fall. Chasing it produced a long investigation; net results below.

**Hover fix = `E_z=0.5`** (lower boundary → stronger descent drive). It works (all reps descend, 0 hover) **but un-bounds κ at touchdown** → κ-runaway TL (a_u up to 4005). Root cause is **one-knob-one-job**: `E_z` sets *both* descent stiffness *and* κ-bounding, so lowering it for descent un-bounds κ.

⛔ **`E_z=0.5 + P_z=8` TESTED + FAILED (NC55, 2026-06-10): 1 TL, mean 7.77 m (worse than P_z=5's 4.63).** P_z=8 did NOT even bound κ_z (still hit the 3.0 cap in 2/5), AND the catastrophe (rep3, 21 m) is a **LATERAL κ-runaway** (`κ_xy→7.26` vs κ_0=0.16, `a_u_xy→631`) — which **P_z (z-only) cannot touch**. **The κ-runaway under E_z=0.5 is on the xy axes**, driven by the faster descent amplifying the close-range **1/Z lateral feature error** → lateral σ grows → lateral κ ratchets → a_u_xy blowup → cbf2 tilt-cap (60°) + lateral rate-sat (1 rad/s) both fire → drift/TL. P_z is the wrong lever; the lateral κ-side lever is `P_XY↑` (untested, softens tracking), but the true driver is close-range **1/Z geometry/perception**, not a gain.

**θ_norm / dw findings (detail in [[feedback_theta_norm_klt_drift]]):**
- ✅ **BAKED: cleaner image-rate `dw`** (`controller.py`, commit 85e1011) — divide the frame-jump by the REAL inter-frame interval (kills the ~3× over-amplification), ZERO between frames (brief, integrator-safe), + a physical clamp `PLASMC_DW_MAX=30`. Tidies the θ *peak*; validated no-regression at the baked E_z=1.0 (κz=1.00).
- ⛔ **DEAD-END: image-rate-HOLD `dw`** — holding dw between frames turned brief θ spikes into *sustained* moderate θ → poisoned the κ-integrator → MORE runaways (3/5 vs 1/5). Reverted.
- ⛔ **DEAD-END: sustained-high-θ κ-freeze** (2nd trigger on top of Singhal `_contained`) — **mis-targeted**: at E_z=0.5, κ ratchets up during the **MODERATE-θ** stretches (median 3–7, freeze OFF) via large `G·|σ|` at close range, NOT the brief high-θ spikes the trigger chased (θ>50 on only ~19% of frames). No θ threshold (50 or 200) catches it. Dropped.

**TL CAUSES — RE-CATEGORIZED by the ACTUAL image centroid/corners (2026-06-10, 12 campaign TLs; supersedes the earlier virtual-based "close-range marker-loss" read):**
- **9/12 = ArUco DECODE failure with the marker FULLY in-FoV** (4/4 corners in at the loss, mostly centered) — the mode-2 close-range decode breakdown, NOT a geometric marker-loss. See [[feedback_marker_detection_stale]].
- **3/12 = clip** (≥1 corner out); 2 of those = the drone had already flown off (geometric loss downstream of a control failure).
- **The detection loss is the TRIGGER of the κ-runaway** (loss at 0.80 of flight PRECEDES the runaway at 0.88): freeze → held/extrapolated feature → off-screen VIRTUAL centroid → wrong `h_d` → breach → κ-runaway → fly-away ([[feedback_lateral_kappa_runaway]]).
→ Dominant + binding TL cause = **ArUco decode failure on an in-frame marker** (NOT geometric loss, NOT gain-tunable). Lever = KLT corner-tracking + decode robustness + corners-based CBF; during marker-LOST use genuine data (don't nan/extrapolate).
→ **4/6 TLs are PERCEPTION-driven** (κ contained), only 2/6 control. The dominant + binding TL cause is perception, esp. the close-range touchdown marker-loss.

**Status:** cleaner-dw baked; `E_z=0.5` NOT baked (hover↔κ trade-off unresolved without P_z). The hover is rare (1/5); the dominant failures are perception.
**Why:** a long thread — don't re-run the dw-rewrite or θ-freeze; they're dead.
**How to apply:** descent → `E_z=0.5 + P_z↑` (not θ-freeze, not dw-rewrite). TL → 4/6 are perception (close-range touchdown-loss); the lever is ArUco robustness / multi-marker / pyramidal LK, not gains.
