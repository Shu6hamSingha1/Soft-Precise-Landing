---
name: project_cross_marker_hz_regression_bisection_20260810
description: Cross-marker Hz raw-signal regression (0.9->0.8) - CLOSED, user accepted the gap as tolerable
metadata: 
  node_type: memory
  type: project
  originSessionId: 3600b91d-f44b-4754-86bc-066d9ec45b18
  modified: 2026-08-10T17:24:42.765Z
---

**⚠ CLOSED 2026-08-10: user explicitly accepted the 0.9→0.8 z-phase
correlation gap as tolerable.** Do not reopen or re-propose bisection
work on this specific regression unless the user raises it again. All 5
candidates from the 08-04/05 change cluster were tested (none explain it),
the implementation was audited clean across all 4 platforms, and the
hi-res-texture and exclude-frac follow-ups are also closed (see below).
This memory is kept for reference/history, not as an open task.

Cross-marker z-excitation-phase raw Tz-vs-GT correlation dropped from a
tight 0.90-0.96 (pre-08-05 archived data) to a tight 0.78-0.82 (current),
apples-to-apples methodology, confirmed real (not the sign-flip artifact —
see [[feedback_correlation_needs_pooling]]).

**Ruled out as the cause (each via a proper n=3-5 A/B, temporarily reverted
then restored), as of 2026-08-10:**
- Color-gate threshold (100→20 revert): breaks detection almost entirely
  under the current thinner marker line (0% ok-rate) — not a subtle-effect
  candidate, just incompatible.
- Camera Z offset (.15→.20 revert): 0.80/0.87/0.80, matches baseline.
- Tracking-based ROI addition (disabled via temp toggle, since removed):
  0.82/0.84/0.84, matches baseline.

**Tested and inconclusive, not adopted:**
- Line-width halving (re-tested independently of the gate threshold, since
  they turned out NOT to be coupled): a full-width mask at the current
  hi-res texture resolution gave -0.03/+0.60/+0.94 across 3 flights — much
  higher variance than baseline, mean not clearly better, tracked point
  count (`n_kept`) also less stable. Full-width line isn't a clean fix;
  may destabilize which corner subset the periodic resample settles on.

**Mount-yaw+90° rotation: TESTED 2026-08-10 (matched revert), INCONCLUSIVE.**
Did a physically self-consistent isolated test: reverted BOTH the SDF mount
yaw (90°→0°, camera Z held at the current .15) AND the corresponding
code-side ray convention (`_getVirtualPts`'s `[y,-x]`→`[x,y]`, both the
quat-based and quat=None branches) via a temporary matched toggle, so the
physical and code conventions stayed self-consistent (100% detection
ok-rate confirmed the match was valid, not broken). Result across 3
flights: **+0.775, +0.803, -0.193** — two land near the current baseline
(0.78-0.82), one flips sign entirely. Given this session's own lesson
about narrow z-phase-only checks being noisy/misleading (see the
exclude-frac experiment above), this is NOT being treated as a real effect
without more data, and NOT being pursued further given how coupled/risky
this specific revert is to keep testing repeatedly. Reverted cleanly (SDF
+ code both confirmed byte-identical to pre-test state).

**Recommendation:** stop spending flight batches on this without a new,
cheap idea — the gap may be a combined/nonlinear effect of several small
08-04/05 changes together, not attributable to any single one on this
candidate list.

**Implementation audit (2026-08-10): no bug found.** Compared the Hz/Tz
computation across all 4 live platforms — MATLAB (`image_feature.m` +
`visualControl_IBVS_adaptive.m`'s `L_s`), ArUco PX4 (`img_data.py`),
cross-marker PX4 (`cross_marker_perception.py`), real hardware
(`img_geometry.py`). The image-Jacobian formula (`_fill_A`/`L_s`, incl. the
Tz column `[-x,-y]`) is byte-identical (post focal-length normalization)
across all 4. The V-frame gravity-leveling transform (`_getVirtualPts`/
`get_virtual_pts`) is a verified direct port between the two PX4 platforms,
and hardware's version is a structurally-equivalent, correctly-adapted
variant for its different physical camera mount. **No formula or
frame-transform error anywhere** — the regression is not a logic bug in
how Tz is computed. Combined with the 4 ruled-out/inconclusive candidates
above, the leading working theory is that it's about point-SAMPLE quality
(which corners GFT finds and how stably, under the current hi-res/thin-line
marker) rather than any computation error.

**CROSS_FLOW_CENTER_EXCLUDE_FRAC 0.35→0.55: TRIED AND REVERTED (2026-08-10).**
A narrow z-phase-only raw Tz-vs-GT correlation check (per the radial-leverage
mechanism the user identified — Hz's per-point signal is linear in radial
distance, `_fill_A`'s Tz column is `[-x,-y]`) looked promising: mean
0.804→0.835 at n=5, individual runs up to 0.901. **But the actual
whole-flight joint calibration fit (`derive_cross_marker_cal.py`, which
uses ALL phases, not just z) showed this is a NET REGRESSION**: Hz
whole-flight R² 0.48→0.07, and Hx/Hy dropped too (0.69→0.48, 0.76→0.65),
consistent across all 5 runs (not an outlier — R² per run was 0.01-0.22).
Mechanism: the more aggressive exclusion starves the point pool (n_kept
median 10-17→8-10), and that point-count reduction adds noise to EVERY
phase — x/y/yaw dominate sample count and got noisier, outweighing the
narrow z-phase leverage gain the isolated check was measuring. **Lesson:
don't judge a point-sampling change from an isolated single-phase
diagnostic — always re-derive the whole-flight fit before adopting.**
See [[feedback_whole_flight_fit_before_adopting]]. Reverted cleanly:
default back to 0.35, live cal matrix confirmed byte-identical to before.

**Hi-res texture swap (08-09, separate from the 08-04/05 cluster):
RULED OUT, and actually a net POSITIVE, not a regression source.**
Tested 2026-08-10: reverted to the pre-hires texture (current line-width
and gate held fixed) for 3 flights — z-phase correlation was **+0.52,
+0.57, +0.20** (mean ~0.43), WORSE than the current hires baseline
(0.78-0.82). Point-set diagnostics explain why: pre-hires flights had
MORE tracked points (`n_kept` median 24-55) but LOWER radial spread
(mean 0.09-0.13, max 0.19-0.21); current-hires flights have FEWER points
(median 10-17) but a higher max spread (0.19-0.26) — the finer speckle
lets GFT occasionally find a few high-leverage far corners, which matters
more for Tz's linear-in-(x,y) column than raw point count. Point-set
QUALITY (radial leverage), not quantity, is what's driving this channel —
consistent with the earlier 08-06 Hx/Hy fix's same "position-weighted
columns need spread, not count" finding. Don't revert the hi-res texture
for this regression; it's a red herring in the opposite direction.

Full narrative: `PX4_Gazebo/docs/HANDOVER_cross_marker_hz_signflip_20260809.md`,
`.claude/skills/io-calibration/SKILL.md`'s cross-marker section.
