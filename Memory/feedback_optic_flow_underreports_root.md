---
name: feedback_optic_flow_underreports_root
description: "BINDING ROOT CAUSE (2026-06-08, GT-verified): the optic FLOW under-reports the real velocity — both the lateral flow h_xy (5-25x too low) AND the vertical loom h_z. The controller is BLIND to the drift/descent it must arrest -> lateral drift grows unchecked + descent runs away. This is PERCEPTION (flow estimation), NOT a controller gain — which is why ~30 control-tuning trials never cracked it. Two sub-failures: the corner+ring FUSION EKF over-suppresses the flow (fused output < BOTH raw inputs), and the raw LK flow collapses for fast (>~2 m/s) lateral motion."
metadata: 
  node_type: memory
  type: feedback
  originSessionId: dd8920aa-4635-40a9-ac9d-8409e37243d6
---

**⚠️ REVISED 2026-06-09 — the "5-25× underreport" was CAL-CONTAMINATED (old broken cal era).**

With the correct all-13-run corner cal, GT-comparison on n=5 (KP=12, E=2.5) reps gives:
- **0.5–2s (stable):** meas/GT ratio = 1.03 — no underreporting
- **2–4s (drift onset):** ratio = 0.73, Pearson r=0.92 — 27% underreport, good correlation
- Across all 5 reps: ratio 0.79–0.96 (4–21% underreport), r=0.48–0.95

**The actual binding constraint (2026-06-09) is h_d runaway, not flow estimation:**
K_rp=12 × accumulated s_e_n (~0.5 over 4s) → h_d grows to 9–22 rad/s at t≈5s. Measured h_xy is only 1–11 rad/s. h_e = h − h_d ≈ 9–14 rad/s fills the funnel → θ_norm spikes (40→3524) → κ explosion → TL. **Fix: K_rp=9** (reverted in dc6ea2c). The EKF fusion over-suppression sub-claim below may still apply at E=1.5 (stiff) regime; needs re-test.

**Why the original analysis was wrong:** the flow measurements that showed h_xy=0.12 vs GT=2.9 were from a run with the broken 2-13× cal — the controller was running at 0.08–0.53× design gains. With correct cal the flow tracks GT at 73–96%.

---

**Original (cal-era) analysis follows for archival purposes:**

**GT-verified root cause of the lateral drift (and the descent runaway), 2026-06-08.** Traced a failed rep (P2INF_Z=1.5, 23 m TL) end to end:

- The drone **starts centered** (GT_lat 0.16 m, IC fine) and develops a real lateral **velocity at ALTITUDE** (2.3 m/s at 4.4 m). At that altitude a 0.44 m offset is `s_e_n`=0.04 (1/Z), so the OUTER loop barely sees it; and the MIDDLE-loop **optic flow `h_xy` under-reports the real lateral velocity 5–25×** (at t=3.0 the drone moves 4.67 m/s, GT flow `v/Z`≈2.9, but measured `h_xy`=0.12). **So the controller is blind to the velocity it must brake** → the drift grows unchecked → near ground `s_e_n` finally spikes (1/Z) → `a_u_xy` runs away (8959) → tumble → 18 m crash.

- **This is the single binding cause for BOTH the lateral drift and the descent runaway** — the *vertical* flow (loom `h_z`) under-reports the same way. It explains why ~30 control-tuning trials (E, GAMMA_Y, KP, P, W_U_MAX, …) never worked: **you cannot gain-tune a signal the controller can't see.** It is NOT yaw (GT yaw stayed +1…+16°), NOT marker loss (Ncorn 20–28, Nring 85–147 healthy), NOT the cal (flow scale ~1.0).

**Two distinct flow-estimation failures (the perception-branch hand-off):**
1. **The corner+ring FUSION EKF over-suppresses the lateral flow.** At t=1.5 (low ecc/velocity): corner-KF=0.96, ring-KF=0.68 both *see* the flow, but the **fused output = 0.07 — BELOW both inputs.** A fusion below both measurements = an over-confident slow prediction (process noise Q too low / measurement noise R too high). The controller consumes the *fused* signal, so the fusion is what's killing the velocity info. **This is the primary failure — it suppresses the SLOW early drift, which is the root of the positive feedback (slow drift → grows → fast → raw collapse).**
2. **The raw LK flow collapses for FAST motion.** At GT flow 8.61 every estimate reads ~0.1 — apparent motion exceeds the LK search window / the fast marker de-correlates between frames.

**Correct method (see body of the response 2026-06-08): perception-side, primary = retune the fusion EKF so the fused flow tracks (not suppresses) the corner/ring measurement; secondary = pyramidal-LK / larger search window for fast-motion dynamic range. The VIO/IMU-velocity alternative is ruled out by the scale-free constraint ([[feedback_scale_free_depth_free]]).** See [[feedback_reject_on_single_failure]] (the rule that surfaced this), [[feedback_newcal_tuning_results]] (the 30 control trials this invalidates as the wrong layer).
