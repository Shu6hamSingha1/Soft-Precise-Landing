---
name: feedback_textured_marker_falsified
description: "Fine-stipple textured marker (0-small_10-big_textured.png) roughly DOUBLED ring-flow LK survival (15.7%->22.8% steady-state) and let TOUCHDOWN-DETECT fire for the first time -- but n=5 IC1 showed a net REGRESSION (0/5 precise, 3/5 TARGET_LOST, mean xy 1.214m vs 0.184m untextured) traced to ArUco PRIMARY decode interference. FALSIFIED, reverted 2026-07-10. A pre-existing, purpose-built COARSE-square alternative (0-small_10-big_coarse.png) exists and is decode-safety-validated offline but UNTESTED in flight."
metadata:
  node_type: memory
  type: feedback
  originSessionId: a8922284-2fe3-4a78-9355-9949c3be5a10
---

## Motivation

Ring-station LK survival was found texture-starved on the plain nested marker
(`0-small_10-big.png`): steady-state ~15.7% survival even at stable hover (before any tilt/
marker-size effects), collapsing to 0 for 300+ consecutive frames near touchdown. Root cause:
background/ground texture-poverty (Gazebo world), not geometry — confirmed via a
`RING_FILTER_DBG` diagnostic printing per-frame survivor counts through the LK→arm-mask→MAD→
pairing filter chain.

## Experiment: fine-stipple texture (2026-07-10)

Swapped the active Gazebo albedo map (`~/PX4-Autopilot/Tools/simulation/gz/models/arucotag/
model.sdf`, outside this git repo) to `0-small_10-big_textured.png` (a dense sub-pixel dot
pattern). Result: ring-flow LK survival roughly doubled (22.8% steady-state, usable-ring-frame
rate 27.9%→54.8%), and `TOUCHDOWN-DETECT` fired successfully for the first time in this
session's investigation.

## Falsified: net landing-quality REGRESSION

n=5 IC1 batch with the SAME code fixes (sign-guard removal etc.) + textured marker:
**0/5 precise, 1/5 soft, mean xy=1.214m, max=2.96m, 3/5 `TARGET_LOST`.** Compare to the SAME
code fixes with the PLAIN marker: **3/5 precise, 2/5 soft, mean xy=0.184m, 0/5 TARGET_LOST**
(matches/beats the 0.21m historical reference). The isolated variable is the texture — this
cleanly attributes the regression to the marker, not the code changes.

Mechanism (inferred, not directly confirmed via ArUco-internal diagnostics): the fine-stipple
pattern likely interferes with ArUco's own corner/threshold detection at typical operating
ranges, trading better ring reliability for worse (and far more consequential) PRIMARY marker
loss — every rep showed frequent `ArUco lost → KLT fallback` cycling, escalating to full
`TARGET_LOST` in 3/5.

**Reverted** to `0-small_10-big.png` (plain, untextured) in the shared Gazebo model dir.

## ⛔ STAMP 2026-07-17 — the "alternatives" below are DELETED; the falsification is being RE-OPENED

**(a) coarse + multiscale DELETED (user directive 2026-07-17): "generated Claude and are absurd
designs."** Removed `Images/0-small_10-big_coarse.png`, `Images/0-small_10-big_multiscale.png`,
`tools/make_coarse_textured_marker.py`, `tools/make_multiscale_marker.py` + the Gazebo model-dir
copies. **Do NOT propose regenerating them.** The section below is retained only as history — its
"natural next experiment" recommendation is VOID. (`make_fineline_textured_board.py` +
`aruco_board_fineline.png` survive — a BOARD asset, out of scope of that directive, still untested.)

**(b) The FALSIFICATION ITSELF may be STALE — it predates `planar_map_rescue`.** This file's verdict
(fine-stipple -> ArUco decode interference -> 3/5 TARGET_LOST) was measured **2026-07-10**;
`planar_map_rescue` landed **2026-07-16**. Back then a decode loss went straight to TARGET_LOST
because nothing could bridge it. The rescue path now infers the marker's position from the map's
OTHER tracked scene features precisely when raw decode fails — and **TEXTURE IS WHAT GIVES THE MAP
THOSE FEATURES** (the plain marker + texture-poor Gazebo ground starves it). So texture and rescue
are COMPLEMENTARY: the texture causes the decode dropouts and simultaneously supplies what the
rescue needs to bridge them. Re-testing at user direction 2026-07-17 (textured swapped back into
model.sdf; cal + IC validation on the textured marker). **This is a genuinely new experiment, not a
re-run** — judge it on whether rescue bridges the decode gaps (RESCUE_ACTIVE / FEATURE_PTS_FRESH /
map_confidence), NOT on the 07-10 numbers. If it regresses AGAIN with rescue live, the falsification
hardens to "texture is unusable at 640x480/fx=270" and should then be treated as closed.

## A pre-existing, purpose-built alternative already exists — UNTESTED (⛔ VOID — see stamp above; assets DELETED 2026-07-17)

`PX4_Gazebo/tools/make_coarse_textured_marker.py` — its own docstring documents that the
fine-stipple approach was ALREADY known to fail at 640×480/fx=270 (dots sub-pixel at altitude →
LK aliasing) in some EARLIER, unrecorded attempt, and was replaced by a coarse-square design:
larger black squares (≥5px in-image), sparse, masked to the marker's WHITE regions only (never
straddling the coded black cells), with a built-in decode-safety self-test across simulated
altitudes. Regenerated + verified 2026-07-10 (`0-small_10-big_coarse.png`, 14 squares placed,
decoded IDs `[0]`→`[10]` correctly across all tested scales). **Swapped into `model.sdf` briefly
but reverted again before flight-testing** (session redirected to a cal-recalibration priority)
— this variant has NEVER been flight-tested. It is the natural next experiment if ring-flow
reliability is revisited: `PX4_Gazebo/tools/make_fineline_textured_board.py` is a third,
also-untested-on-the-nested-marker alternative (thin cross-hatch lines, built for the
multi-marker BOARD asset, would need adapting to the nested single-marker image + its own
decode-safety run before trusting it).

## Also relevant

[[feedback_ring_fusion_marker_overlap]] (FLOW_FUSE_RING=0 bake, 2026-07-09) — a DIFFERENT ring
issue (fixed-radius overlap with a grown marker near touchdown), not resolved by texture; ring
fusion stays default-OFF regardless of texture experiments.
