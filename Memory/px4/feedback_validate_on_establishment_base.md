---
name: feedback_validate_on_establishment_base
description: "METHODOLOGY: when validating/gating a finding, run it FIRST on the exact base/conditions where it was established — never switch the base (e.g. h_rd) AND scale up (n) simultaneously; that confounds the result."
metadata:
  node_type: memory
  type: feedback
  originSessionId: a378d3e9-67aa-42fc-ae09-63da27f370a9
---

**2026-06-30 (user correction — load-bearing methodology).** When validating or gating a finding,
RUN THE GATE ON THE EXACT BASE WHERE THE FINDING WAS ESTABLISHED FIRST. Change ONE variable at a time.

**Why:** the XIR=0.15 precision win was established on the **h_rd=-0.30** (all-soft) base. I ran the n=5
gate on the **baked -0.42** base instead — switching h_rd AND scaling n=1→5 at once. The -0.42 gate's IC5
fly is therefore CONFOUNDED: it could be XIR=0.15 edge-forcing OR the faster -0.42 descent (IC5 needs
-0.30's runway). I couldn't attribute it. A wasted ~1.5 h gate with an un-interpretable result.

**The rule:**
- Validate a finding on the SAME conditions it was found (same base config), then extend to other bases
  ONE change at a time. Don't conflate "the baked default is X" with "validate there" — validate where
  the EFFECT was demonstrated.
- When scaling n (n=1 → n=5), hold every other knob fixed at the establishment values.
- If a gate must cover multiple bases (e.g. both -0.30 and -0.42), run them as SEPARATE arms so each is
  attributable, not a single mixed config.

**ALSO RUN THE BASELINE at the same conditions (2026-06-30, same correction).** A gate without a
matched baseline is un-comparable. To attribute the XIR=0.15 effect you need the FULL factorial:
baseline (h_rd=-0.42, XIR=0.10) AND each single-change cell, at the SAME n and IC set. The right design
here was a 2x2: h_rd{-0.42,-0.30} x XIR{0.10,0.15} at n=5 IC1-5 — so the XIR effect, the h_rd effect,
and any interaction are each isolated. I ran only the two XIR=0.15 cells (no XIR=0.10 baselines) → even
those were not comparable to anything at matched n/IC. Don't compare an n=5 gate to an old n=3 different-IC
"6/9 SP" number — run the matched baseline.

**How to apply:** before launching any validation/gate, state explicitly: "this finding was established
under config C; the gate runs under config C, WITH a matched baseline arm (the one-variable-back config)."
If the launch config differs from C, or there's no baseline arm, fix it first.
Related: [[feedback_sensitivity_sweep_methodology]] (n=1 is noise; validate singletons at n>=5 before
stacking) and [[feedback_ic_validation]] (always IC2-5 gate before defaulting).
