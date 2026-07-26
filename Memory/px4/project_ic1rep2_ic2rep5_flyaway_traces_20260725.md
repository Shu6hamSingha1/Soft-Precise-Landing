---
name: project_ic1rep2_ic2rep5_flyaway_traces_20260725
description: "2026-07-25: in-depth traces of the 2 real fly-aways from the planar_map-merge validation batch (ICValidation/20260725-193807, IC1_rep2 17.67m + IC2_rep5 8.59m). Both confirmed as instances of already-characterized, out-of-scope residual failure families -- NOT caused by or exposed by that session's confidence-lockup/rescue-gate/RANSAC fixes. Corrects the earlier overclaimed 'zero fly-aways' validation summary."
metadata:
  node_type: memory
  type: project
  originSessionId: 68163648-1a9b-4336-962f-9c4c77471aea
  modified: 2026-07-26T10:28:46.040Z
---

**Context.** Same session as [[feedback_planar_map_confidence_lockup]]. That memory's
validation claim was corrected after the user caught it ("n=18, zero fly-aways" was wrong
-- only 2 of 3 batches were checked). This memory documents the deep trace of the 2 real
fly-aways found in the third batch (`ICValidation/20260725-193807`, the
`PLANAR_MAP_DBG=1` probe, IC1/IC2/IC4 n=5).

**`IC1_rep2` (17.67m xy_err, 7.26m/s, TARGET_LOST, T_flight=10.0s).** A compound,
multi-stage event, traced via `Img_Data.npy`'s `N Flow Corners`/`Image Feature Pts`/`Quat`:
1. t=40.5-41.0s (absolute): `MARKER_EXTENT_PX` explodes 264px->357.6px in ~0.5s (a
   near-touchdown-scale size runaway) while `nfc=0` throughout (raw LK correspondence
   failing) and the corner position swings wildly before FREEZING at a fixed value for
   0.6s straight.
2. t=41.71s: reacquires at a completely different pixel location, extent drops to 90px --
   a discontinuous handover, the same signature as the marker-switch pattern already
   characterized this session (see [[project_ic2_ic5_20260723_investigation]]).
3. t=42.34s onward: right as ANOTHER tracking hiccup hits, `Quat` starts rotating hard
   (q0: 0.75->0.70->0.67->0.66->0.64->0.63->...->0.59 over ~0.6s) -- a GENUINE physical
   attitude tumble begins (not a data artifact), and corners freeze again while the real
   rotation continues.
This is the SAME causal chain already traced for `IC2_rep3` earlier this session
([[project_ic2_ic5_20260723_investigation]]'s t=54s trace): a close-range marker-switch/
handover triggers a bad, momentary control response; the vehicle actually EXECUTES it;
that ignites a genuine physical tumble; the tumble then self-perpetuates through the
`cross(w_i,s)` feedforward term (verified correct via first-principles re-derivation,
see the h_x/h_y validation earlier in this session's transcript). Root cause is the
still-OPEN `FLOW_LAT_REDUCED` model-misspecification gap from `project_ic1_terminal_kick_root_cause_chain`
(2026-07-10/11), explicitly flagged "NOT fixed... flagged not implemented" back then --
STILL not fixed, confirmed recurring here. Control_Data corroborates: `zeta`/`sigma`
breach (>3.0/3.5) starting t=41.77s (absolute), `kappa_y` ratchets to 5.76 (not the
thousands seen in the bugs actually fixed this session), `a_u_y` peaks at only 20.5 --
a real but MODEST control response to a genuine, sustained perception disturbance, not
a runaway control-law bug.

**`IC2_rep5` (8.59m xy_err, 2.00m/s, TARGET_LOST, T_flight=4.7s -- a short flight).**
`diagnose_failure_cause.py`'s "onset t=0.03s" label is misleading taken alone -- that's
just flagging IC2's inherent 2.73m starting lateral offset (`|s_e_n|>0.5` from
essentially t=0), which is EXPECTED for this IC, not itself the failure trigger. The
actual mechanism, from `Img_Data.npy`: `nfc` tracks PERFECTLY (184 corners) from
t=34.8-37.4s (absolute) -- the marker is decoding fine while the controller aggressively
closes the large offset -- then `nfc` goes to SUSTAINED 0 for the final ~2 seconds of the
4.7s flight, a terminal marker loss hitting while the vehicle still carries real,
significant residual velocity from the fast offset-closing maneuver. The subsequent blind
coast (no rescue re-engagement in time) is what balloons the error to 8.59m by touchdown.
Same family as the pre-existing IC2 lateral-closing-speed-vs-terminal-perception
characterization already in project history (short, aggressive flight + terminal
perception ceiling, not a new bug).

**Conclusion (unchanged from the correction in [[feedback_planar_map_confidence_lockup]]):
both fly-aways are instances of already-characterized, OUT-OF-SCOPE residual failure
families.** Neither shows the spurious-decode signature (implausible position/size jump
accepted, addressed by this session's three plausibility gates) or the permanently-
pinned-confidence signature (addressed by the self-heal fix) -- `IC1_rep2`'s kappa/a_u
peaks are modest (5.76/20.5, not thousands) and the mechanism is a genuine, real tumble
from a still-open perception-model gap; `IC2_rep5` is a genuine terminal marker-loss
under real residual velocity, not a corrupted/implausible reading. **This is normal SITL
run-to-run variance surfacing real, pre-existing hard problems, not a regression from the
confidence-lockup/rescue-gate/RANSAC merge** ([[feedback_planar_map_confidence_lockup]]).

**Open, still-unaddressed items surfaced/reconfirmed by this trace (for a future
session, not chased further here):**
1. `FLOW_LAT_REDUCED`'s model-misspecification on a marker-switch-triggered real rotation
   (the `IC1_rep2`/`IC2_rep3` shared mechanism) -- flagged since 07-10/11, still open.
2. IC2's fundamental tension between closing a large offset FAST (needed, since it also
   has less margin before hitting the deck) and surviving a terminal marker-loss with low
   enough residual velocity to coast safely -- a control-tuning problem, not a perception
   bug.
