---
name: Ship-deck Linear/Circular trajectory profile
description: Linear and Circular cases in MATLAB/Common/traj_Gen.m are ship-deck scenarios with Lin 2022 heave + large roll/pitch; all PLASMC results reported under these.
type: project
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---
As of 2026-04-14 the "Linear" and "Circular" cases in MATLAB/Common/traj_Gen.m are ship-deck landing scenarios (no separate LinearDeck/CircularDeck case). Profile:

- Translation:
  - Linear: north-east surge at 1.0·speed_mult m/s on both axes
  - Circular: r=0.5 m, wz=0.3·speed_mult rad/s (wz ceiling for 50/50 multi-init)
- Heave (both): A_z sin(w_z t), A_z=0.2 m, w_z=0.5 rad/s — from Lin 2022
- Attitude (both): (phi, theta) = (15° sin(0.9 t), 8° sin(0.6 t + π/3)), psi=0 for Linear, psi=wz·t for Circular
- 3-2-1 Tait-Bryan composition via local helpers euler321_to_quat, euler_rates_to_bodyrates at bottom of traj_Gen.m

**Why:** Ship-deck motion is the more demanding test (harsher than Lin 2022 heave-only benchmark) and lets the paper claim deck-landing applicability without a separate case.

**How to apply:** When interpreting Multi_init_cond, Comparison, or multi_speed results on Linear/Circular, these are the deck-motion numbers. Table V/VIII discussion prose in results.tex already references the heave and roll/pitch amplitudes.

**Update 2026-04-18:** Ez=0.9 is no longer needed — after three-knob retune (zd=1.15, E(3,3)=1.0, h_rd=-0.42), Linear IC5 ship-deck heave case passes soft (v_rel=0.154 vs 0.20 threshold). The Ez=0.9 widening that was previously locked for Lissajous Run 5 soft-landing margin is obsolete — Ez=1.0 now holds 25/25 across all five trajectories. See project_multi_init_final_lock.md.
