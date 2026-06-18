---
name: ekf-default-breaks-descent
description: "RESOLVED 2026-06-07: the no-descent is NOT the EKF. A CORNER-ONLY landing (FLOW_FUSE_RING=0) hovered with the IDENTICAL signature as the fused one (commanded h_d_z=-0.42, measured loom h_z~0, ~6 min timeout). So FLOW_FUSE_RING=1 is EXONERATED — the hover is a SHARED vertical-descent regressor that appeared 2026-06-07 (Jun-5 corner landings descended to ~0.05 m; Jun-7 fused AND corner both hover at 6 m). Candidates: all-13 cal Hz scale, REF_RAD_OPT_FLOW=-0.42 (vs the user's usual -0.70), or gains. FLOW_FUSE_RING=1 default is fine to keep."
metadata:
  node_type: memory
  type: feedback
  originSessionId: 017661ba-4715-40a3-878a-4e73a4f2c6b9
---

**RESOLVED (2026-06-07): the EKF default is EXONERATED.** The "EKF-default breaks the
descent" title is kept only for back-links — the no-descent is NOT caused by
`FLOW_FUSE_RING=1`, and the two mechanisms I first proposed were both falsified.

**The deciding evidence:** TWO Jun-7 landings hovered with the *identical* signature —
- `00-08-38` — `FLOW_FUSE_RING=1` (fused), held ~6 m, ~4.6 min, never descended.
- `00-42-23` — **`FLOW_FUSE_RING=0` (corner-only, no fused log)** — **ALSO** held ~6 m,
  ~6 min, never descended.
Both: controller **commanded** descent (`h_d_z = -0.42` = `REF_RAD_OPT_FLOW`, the MATLAB
default) but the **measured loom `h_z` stayed ~0**. Corner-only hovering identically ⇒
**the flow fusion / EKF is NOT the cause.**

**REFINED 2026-06-07 (input thrust-staircase) — it is CONTROL-side, NOT actuation.** The
first wording here ("vertical actuation/tracking failure") was wrong: the thrust-map
validation descended **20 m → 1 m on direct thrust commands** and the FC hovers at
thrust_norm **0.741**, executing commanded thrust faithfully (slope → mass 2.04 kg). So
the FC *executes* thrust fine; the bug is that the **loom-SMC commands ~hover thrust
despite `h_d_z=-0.42`** — it isn't translating the descent reference into reduced thrust.

**It's a SHARED descent regressor that appeared on 2026-06-07.** Jun-5 corner landings
descended fine (GT Vz min ≈ 0.02–0.11 m, i.e. touchdown); Jun-7 landings (fused AND
corner) hover at ~6 m. So something that changed 2026-06-07 broke the descent for BOTH
paths. Candidates (for the tuning chat to bisect):
- the **all-13 cal `Hz` row** (re-derived 2026-06-07; the divergence row is the
  least-certain — flagged in img_data.py),
- **`REF_RAD_OPT_FLOW = -0.42`** (the runtime default) vs the user's usual **-0.70**,
- the control **gains** being tuned in the other chat.

**CAVEAT (2026-06-07): none of those cleanly produces a 6-min HOVER by itself** —
`REF_RAD=-0.42` is a valid MATLAB default that gives a *slower* descent (not zero), and a
cal-`Hz`-reads-0 would drive *runaway* descent (constant `h_d_z-h_z` error → keep cutting
thrust), not a balanced hover. A hover ⇒ the SMC settles on ~hover thrust. So the bisect
should **instrument the commanded thrust vs `h_d_z` and the loom-error → thrust path**
directly (sign/clamp/feedforward cancellation in the vertical SMC), not just swap cal/REF_RAD.

**Falsified along the way (don't repeat):** (1) "fused h_z over-reports the loom (ring
gain 1.37)" — FALSE, fused `h_z == corner h_z` ratio 1.00; (2) "re-tune `REF_RAD` to the
fused scale" — no scale mismatch.

**Implications:**
- **`FLOW_FUSE_RING=1` stays default** — it is not the descent problem; the EKF is the
  best estimator by signal and feeds corner-equivalent `h_z`.
- The descent bug is real and **owned by the tuning chat** (it surfaced it via the
  corner-only `FLOW_FUSE_RING=0` isolate step from the hand-off note). Bisect the vertical
  SMC's loom-error → thrust path (the FC executes thrust fine — proven by the thrust-staircase
  descending 20→1 m); cal-Hz / REF_RAD / gains are secondary candidates.
- For plots: a hover landing's GT altitude clusters at ~6 m (never 0); real landings reach
  ~0.05 m (camera standoff + `gt_v_flow z_floor=0.15` + marker loss in the final cm), not
  exactly 0.

Lesson: signal-R² alone doesn't validate a control-path swap — and verify a root-cause
against a CONTROL (here corner-vs-fused) before attributing.

Related: [[feedback_ic_validation]], [[ring-flow-calibration]], [[feedback_convergence_ordering]],
[[project_tuning_campaign_newcal_reset]].
