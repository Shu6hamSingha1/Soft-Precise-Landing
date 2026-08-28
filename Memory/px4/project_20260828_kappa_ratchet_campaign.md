---
name: project_20260828_kappa_ratchet_campaign
description: "2026-08-28 kappa-ratchet multi-rep sweep campaign (tune-plasmc methodology, per user directive). BASELINE PASS n=5 GT-FB per IC1-5 (25 reps) in the CURRENT config (P2INF_XY=2.5 + FLOW_FUSE_RING=0 + KAPPA0_XY=0.5 + Z_REG=0.2 + W_U_MAX=2.0 + XIR=0.10 + this session's cross-marker perception stack). RESULT: kappa-ratchet DOES NOT OCCUR. 25/25 SOFT+PRECISE, kappa_xy frozen at its 0.5 init on every rep (never adapts up), 0% sigma_xy>1 breach on every rep (sigma_xy_max 0.24-0.50 vs E=1), a_u_xy_max 1.0-2.1 (vs documented ratchet events 25-40+). No knob sweep needed -- the failure mode is already resolved. REAL-PERCEPTION PASS (n=5 GT-FB OFF per IC1-5) ALSO shows NO ratchet: kappa_xy pinned at 0.50 init and leaks DOWN every rep across all ICs. Real-perception failures are the separate off-center CONVERGENCE WALL (IC2-4) + oblique-low detection collapse (IC3/IC5) + IC1 terminal-descent breakdown -- all distinct from the ratchet."
metadata: 
  node_type: memory
  type: project
  originSessionId: 3014ee64-961f-4124-9409-bdf8a7da5196
  modified: 2026-08-28T11:10:34.635Z
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

### REAL-PERCEPTION PASS -- n=5 GT-FB OFF per IC, cross_marker, same config (2026-08-28)

User: "Now run the same campaign under real perception."

| IC | ENU | landing outcome | kappa_xy_max | kappa_xy_end | sigma_xy_max | \|sig\|>1 | a_u_xy_max | s_e_n_min | verdict |
|----|-----|-----------------|-------------|-------------|--------------|-----------|-----------|-----------|---------|
| IC1 | 0,0,5  | 5/5 FAIL (terminal descent; detect 100%) | 0.50 | 0.17-0.22 | 0.08-0.17 | 0% | 0.3-0.5 | 0.000-0.003 | clean-ish (no ratchet, no wall) |
| IC2 | 2,2,5  | 5/5 FAIL/TL | 0.50 | 0.26-0.27 | 0.88-1.10 | 0-2% | 1.8-12.4 | 0.48-0.49 | CONV-WALL |
| IC3 | -2,2,5 | 3/5 TL + noisy (launch flakes; detect 65% on rep3) | 0.50 | 0.06-0.34 | 0.42-2.37 | 0-5% | 1.4-14.1 | 0.036-0.66 | CONV-WALL (rep2 clean) |
| IC4 | 2,2,7  | 5/5 FAIL | 0.50 | 0.21-0.35 | 0.23-0.71 | 0% | 1.0-1.5 | ~0.35 flat | CONV-WALL |
| IC5 | 2,2,3  | 4/5 TARGET_LOST <5 ctrl samples + 1 FAIL | 0.50 | 0.35 (1 rep) | 0.49 | 0% | 2.3 | 0.72 | oblique-low detection collapse; 1 rep CONV-WALL |

**No kappa-ratchet under real perception at ANY IC.** kappa_xy is PINNED at its
0.50 init on every rep across all 5 ICs and leaks DOWN (end 0.06-0.35) -- the
opposite of a ratchet. The `switch = theta*sat(sigma)*kappa` term never detonates
(a_u_xy_max <= 14 on the worst off-center rep; sigma breaches are brief boundary-
layer excursions under bad measurements, 0-5%, not adaptive runaway). The
degrading-perception arm of failure-mode-11 does NOT produce the ratchet here.

The real-perception failures are TWO pre-existing, separately-tracked phenomena:
1. **Off-center CONVERGENCE WALL** (IC2/IC3/IC4) -- s_e_n pins at 0.35-0.50 and
   stops converging, kappa LEAKS down, sigma sits at/below E=1 without detonating,
   xy_err ~2.1 m, descent stalls ~3.3-3.5 m. Exactly
   project_20260824_crossmarker_offcenter_convergence_wall. Needs a separate
   CONTROL campaign (kappa-leakage / funnel), not this ratchet campaign.
2. **Oblique-low detection collapse** (IC3 partial, IC5 dominant) -- from a low
   off-center start the cross-marker detector never locks (detect-ok 65%, 4/5 reps
   TARGET_LOST before 5 control samples). This is the detector-retune gap in
   project_20260827_framerate_and_h_texture_investigation (oblique retune got
   GT-FB IC5 65->79%, still not landing-grade under real perception).

IC1 real perception is CLEAN internally (no wall, no ratchet, s_e_n -> image-center
0.000-0.003) -- its 5/5 FAIL is the terminal-descent breakdown (min_alt stalls
~2.5-3.4 m, hard 1.2 m/s), a third separate issue.

### Overall campaign conclusion
kappa-ratchet (failure mode 11) is RESOLVED and does not occur in the current
baked config under EITHER GT-FB (25/25 SP) OR real perception (kappa frozen at
init, leaks down, every IC). No knob sweep was needed. The remaining
cross-marker real-perception failures are the off-center convergence wall + the
oblique-low detector gap + the IC1 terminal-descent breakdown -- all distinct
from the ratchet, each with its own memory.

### Data
test_data/KappaRatchet_IC{1,2,3,4,5}_base/<ts>/rep{1..5}/       (n=5 GT-FB each)
test_data/KappaRatchet_IC{1,2,3,4,5}_realperc/<ts>/rep{1..5}/   (n=5 GT-FB OFF each)
