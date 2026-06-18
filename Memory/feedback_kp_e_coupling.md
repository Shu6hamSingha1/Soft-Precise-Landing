---
name: feedback_kp_e_coupling
description: "KP and E_XY are coupled: KP=12 works with E=1.5 but worsens with E=2.5 (drives h_d windup faster). KP=9 is optimal when E_XY=2.5. Confirmed 2026-06-09."
metadata: 
  node_type: memory
  type: feedback
  originSessionId: a377a083-d63b-447a-908e-12017cf609f1
---

KP and E_XY interact non-trivially — they cannot be tuned independently.

**Rule: match KP to E_XY stiffness.**
- **KP=12, E_XY=1.5**: median 3.20m (n=3, row 34) — SEN_FUNNEL blocks the t=0 LK spike; E=1.5 is stiff enough that h_d windup hits boundary in ~4s → κ ramps → controller corrects.
- **KP=12, E_XY=2.5**: median 4.79m (n=5, row 42) — WORSE. With E=2.5, κ stays near κ_0 regardless of σ. Higher KP drives h_d to accumulate faster (larger ds_d) → funnel breach arrives sooner than with KP=9.
- **KP=9, E_XY=2.5**: median 3.80m (n=5, row 31) — BEST. Slower h_d buildup gives more time before funnel breach; lateral still bounded by E=2.5.

**Why:** E_XY=2.5 makes the boundary layer very wide — σ rarely exceeds E, so κ stays near κ_0 and the control law is effectively in linear (non-sliding) mode. In this regime, the outer PID's h_d windup rate (proportional to KP) is the binding timescale. Larger KP = faster windup = earlier funnel breach = worse outcome.

**How to apply:** When testing wide E (E_XY≥2.5), use KP=9. When testing stiff E (E_XY≤1.5), KP=12 is viable (with SEN_FUNNEL=1 to gate t=0 LK spike). Never raise KP and E_XY together — they partially cancel.

**Why:** Established 2026-06-09 by direct comparison (rows 31 vs 42).
