---
name: project_terminal_velocity_handover_design
description: "AGREED design (2026-07-04) for the terminal deck fix — retarget PLASMC_TERMINAL_COMMIT to keep marker centroid s + switch optical flow (h_xy AND h_z) to the marker-less RING, gated on d(logM) handover THEN centered"
metadata: 
  node_type: memory
  type: project
  originSessionId: 3c2f4c67-05c1-4e6f-966b-0e62018fc8a7
---

Agreed design to fix the terminal deck fly-away/blow-through ([[feedback_terminal_overflow_deck_flyaway]]).
NOT YET IMPLEMENTED. Reuses the (currently OFF) PLASMC_TERMINAL_COMMIT machinery with a retargeted
action + trigger.

**The design (terminal velocity-source handover):**
- **Centroid s (position): stay MARKER-based to touchdown** — closed-loop, moving-target-OK. (Marker
  s stays accurate until the SMALL marker overflows ~0.08m; the last ~0.08m is effectively inertial.)
- **Velocity (h_xy, h_z): retarget, NOT a blanket "switch to ring".** Refined 2026-07-04 by GT-FB
  ring A/Bs (below). Gated by two conditions IN ORDER: (1) **d(logM) HANDOVER latched** (big->small),
  THEN (2) **|s_e_n| < TC_SEN AND |ds_e_n| < TC_DSEN** (centered AND settled/velocity-matched):
  - **h_xy: centroid-rate OBSERVER while the marker centroid exists (~0.08m, flickering near the end),
    then GATED 0 — NOT the ring.** The ring lateral is GEOMETRICALLY UNOBSERVABLE (reads ~1% of GT,
    corr ~0 at ANY signal magnitude; more stations don't help — the near-nadir translation<->tilt
    confound, same σ_min as corners). So there is NO marker-less lateral to fall back on. The observer
    IS the centroid rate (h_xy≈ṡ, couplings vanish near center) → same signal as the kept marker s →
    most accurate (0.85-0.94) + survives longest. Below marker-centroid loss, 0 is the min-error,
    noise-free choice; the marker-s position loop (zeta_r) keeps steering. VALID for MOVING targets
    too: the SETTLED gate (ds_e_n = marker image-motion = h_xy) confirms velocity-MATCH, not just
    centering, so true h_xy≈0 regardless of target motion (risk only if target maneuvers hard in the
    final <1s).
  - **h_z: RING pure_div (Ring Divergence) down to ~0.2m, RING MOMENT at the very deck (<0.2m).
    NOT the lstsq ring loom.** GT-FB A/B: pure_div tracks GT loom ratio ~0.7-1.15 (corr 0.4-0.57) to
    0.2m — the ACCURATE marker-less loom (my earlier "~½ under-report" was the WRONG field, the lstsq
    loom). At the deck BOTH over-report (moment 1.34x, div 2.08x, corr 0.30-0.48) — this is
    ALTITUDE-DEPENDENT (ratio swings 0.24->1.34 / 0.96->2.08), so NOT a cal error: it's MAD-survivor
    sampling bias (fast large-r edge stations) + noise-inflated scatter (M=Σd² only grows) + terminal
    1/Z. Moment is the SAFER deck loom (milder over-report -> less balloon; over-report = over-brake =
    the SAFE direction vs the current stale-under-report blow-through). Apply ~0.75 deck scale.
    Candidate de-bias: radius-weight the divergence (slope of radial-flow-vs-r, not radial mean).

**Trigger = d(logM) handover, NOT the extent threshold.** One-way latch on the first LARGE NEGATIVE
d(logM) spike (big->small = M drops ~10.63^2=113x -> d(logM)~-4.7 vs normal loom ~+0.04; sign
distinguishes big->small(-) from flicker-back small->big(+)). **IMPLEMENTED on the RAW (pre-
LOOM_SZ_RATIO) apparent-size² M via a SEPARATE tracker `_raw_lnM_prev`** — NOT the LOOM_DLNM_MAX
checkpost (that sees the NORMALIZED M, which LOOM_SZ_RATIO makes continuous across the switch by
design). ID/SIZE-FREE + image-space: NO marker-ID, NO physical size, NO board_layout (user correction
2026-07-04 — an earlier board_layout size-drop draft was replaced). Env: HANDOVER_DLOGM (thresh ~1.0). Chosen over MARKER_EXTENT_PX because the extent measures the CURRENTLY
DETECTED marker -> it drops ~10.63x AT the handover and bounces during the flicker (noisy crossing
exactly in the terminal). NOTE (self-correction 2026-07-04): the extent is NOT scale-coupled — it's
a FoV-fill FRACTION (extent_px/frame_px), depth-free + independent of marker metric size (auto-adjusts
trigger altitude). Handover wins on FLICKER-ROBUSTNESS + being the actual event, not on scale-freeness.
Caveat: handover requires the NESTED marker (a board/single marker has no handover -> needs a different
"entered-terminal" event).

**Why this beats PLASMC_TERMINAL_COMMIT as built:** TERMINAL_COMMIT's action on commit is to ZERO
zeta_r (the 1/Z position funnel) + a post-commit flow integral. That attacks the WRONG signal: it
deletes the POSITION term but leaves the VELOCITY (zeta_h) on the CORRUPTED marker flow -> the
flickering h_xy can still launch, the stale marker loom h_z still blows through the descent. (Almost
certainly why it was baked OFF, 78c5309.) It also drops position -> stationary-only. The retarget:
- KEEP zeta_r (position on marker s) — and because the gate requires CENTERED, zeta_r is small at
  commit anyway (no kick to remove -> no need to zero it).
- REPLACE the corrupted velocity (h_xy, h_z) with the CLEAN ring -> fixes both launch (corrupted
  h_xy) and blow-through (stale h_z), the ACTUAL roots.
- The centered gate is exactly what the FAILED FLOW_NCORN_SWITCH lacked (it routed lateral to the
  ring's ~0 while still OFF-center -> lost steering -> fly-away). Centered => true lateral vel ~0 =>
  ring h_xy ~0 is harmless.

**Salvageable core:** reuse _terminalCommitStep's centered/looming/settled discrimination as the
gate; change the ACTION from "zero zeta_r" to "swap flow source marker->ring".

**SINGLE GATE for BOTH channels (2026-07-04, user correction — supersedes the per-channel-gate idea):**
`gate = handover_latched AND |s_e_n|<TC_SEN AND |ds_e_n|<TC_DSEN` (handover + centered + settled).
On gate: h_z ← ring loom, h_xy ← 0. Before gate: h_z ← marker moment-loom, h_xy ← observer, s ← marker.
**Why h_z ALSO needs centered (NOT handover-alone):** the ring samples the NADIR patch → off-center it
sees whatever is BESIDE the target. For an elevated/moving target (rover platform) that means (a) WRONG
depth (lower ground Z_ground>Z_platform → loom under-reports) and (b) WRONG reference — the ground is
STATIONARY, not moving with the rover, so the ring measures drone-vs-ground not drone-vs-target = a
completely wrong relative-motion field for a moving target. So an off-center ring loom is not noisy,
it's measuring the WRONG THING. Ring loom is only valid when the ring OVERLAPS the target = centered,
and STAYS over it = settled. Consequence: in the handover->centered window h_z is still on the
flickering marker moment-loom (blow-through risk) → make that window safe by SLOWING descent while
off-center via PLASMC_DESCENT_GATE (converge lateral before descending into overflow), THEN gate fires.
Composed sequence: descent-gate holds deck approach → center on marker → gate → ring loom + h_xy=0 →
touchdown.

**DE-RISK — RESOLVED (GT-FB ring A/Bs, 2026-07-04):**
- Ring h_xy: **unobservable, not just noisy** (~1% of GT, corr ~0, station-count-independent) → h_xy=0
  is the ONLY option, confirmed. h_xy=0 valid for moving targets via the SETTLED gate.
- Ring loom: **pure_div is the accurate marker-less loom** (ratio ~0.7-1.15 to 0.2m); at the deck both
  pure_div (2.08x) and moment (1.34x) OVER-report, altitude-dependent (NOT cal — sampling bias +
  noise-inflated scatter + 1/Z). Moment is the safer deck loom (milder, over-brake direction). ⚠ the
  scale (~0.75 deck) is from PRE-nested-marker recordings — re-pin on a current nested-marker GT-FB run.
- ⚠ Open depth-free question: the pure_div->moment split at "~0.2m" must NOT use altitude (scale-free
  constraint). Candidate depth-free trigger = Nring drop (~93->42 at the deck) or just use the scaled
  moment from the handover. [[feedback_vds_kf_q_severity_bandaid]] (q=10 amplifies terminal loom error
  -> the over-brake-safe direction of the ring loom error matters).
