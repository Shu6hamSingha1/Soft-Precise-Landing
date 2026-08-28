---
name: project_20260828_kappa_ratchet_campaign
description: "2026-08-28 kappa-ratchet multi-rep sweep campaign (tune-plasmc methodology, per user directive). BASELINE PASS n=5 GT-FB per IC1-5 (25 reps) in the CURRENT config (P2INF_XY=2.5 + FLOW_FUSE_RING=0 + KAPPA0_XY=0.5 + Z_REG=0.2 + W_U_MAX=2.0 + XIR=0.10 + this session's cross-marker perception stack). RESULT: kappa-ratchet DOES NOT OCCUR. 25/25 SOFT+PRECISE, kappa_xy frozen at its 0.5 init on every rep (never adapts up), 0% sigma_xy>1 breach on every rep (sigma_xy_max 0.24-0.50 vs E=1), a_u_xy_max 1.0-2.1 (vs documented ratchet events 25-40+). No knob sweep needed -- the failure mode is already resolved."
metadata:
  node_type: memory
  type: project
---

## Campaign: kappa-ratchet root cause, tune-plasmc methodology (user directive 2026-08-28)

User: "kappa-ratchet root cause needs a proper multi-rep sweep campaign (per the
tune-plasmc skill's own methodology) to actually fix. First test and confirm for
IC1. Then move to IC2 and so on."

### BASELINE PASS -- n=5 GT-FB per IC, cross_marker, current baked config

| IC | ENU | class | kappa_xy_max | sigma_xy_max | \|sig_xy\|>1 | a_u_xy_max | s_e_n_end |
|----|-----|-------|-------------|--------------|-------------|-----------|-----------|
| IC1 | 0,0,5   | 5/5 SP | 0.50 | 0.42 | 0% | ~0.5 | 0.002-0.017 |
| IC2 | 2,2,5   | 5/5 SP | 0.50 | 0.32 | 0% | 1.4  | ~0.05 |
| IC3 | -2,2,5  | 5/5 SP | 0.50 | 0.50 | 0% | 1.5  | ~0.05 |
| IC4 | 2,2,7   | 5/5 SP | 0.50 | 0.25 | 0% | 1.1  | ~0.08 |
| IC5 | 2,2,3   | 5/5 SP | 0.50 | 0.46 | 0% | 2.1  | ~0.06 |

**25/25 SOFT+PRECISE.** kappa_xy stays PINNED at its KAPPA0_XY=0.5 init on every
single rep -- it never adapts up (sigma never saturates: max 0.50 vs E=1). If
anything kappa_xy leaks DOWN (end/init 0.1-0.2x). The `switch = theta*sat(sigma)*
kappa` term (failure mode 11) stays near zero: a_u_xy_max 1.0-2.1 vs the
documented ratchet regime 25-40+ (catastrophic 162-1e6).

### Conclusion: the kappa-ratchet is ALREADY RESOLVED in the current config

Killed by the accumulated stack, no single new knob:
- `FLOW_FUSE_RING=0` (2026-07-09 bake) -- removed a sigma driver (skill flagged this)
- `KAPPA0_XY=0.5` (2026-06-11 bake) -- kappa armed but stable
- `P2INF_XY=2.5` (2026-08-28 REBAKE, user's d7cd430) -- funnel floor is no longer
  the mechanical breach trigger the 0.5/1.0 value was
- `Z_REG=0.2`, `W_U_MAX=2.0`, `XIR=0.10` (2026-06/07 bakes)

**No knob sweep run.** The methodology campaign (single-knob 3-5 values x n>=5 x
IC1-5 gate) was set up but the baseline pass showed nothing to fix.

### Caveat: GT-FB only

This isolates control from perception. The failure-mode-11 note's OTHER arm --
"degrading perception" (extent collapse near touchdown feeding sigma) -- could
still fire under real perception (GT-FB OFF). But that traces to the detection
problem (IC5 oblique-low, project_20260827 memory 2026-08-28), NOT a control-gain
issue. Under real perception cross-marker also hits the separate off-center kappa-
leakage CONVERGENCE wall (project_20260824_crossmarker_offcenter_convergence_wall)
-- a different phenomenon from this ratchet.

### Data
test_data/KappaRatchet_IC{1,2,3,4,5}_base/<ts>/rep{1..5}/  (n=5 GT-FB each)
