---
name: project_20260815_crossmarker_gtfb_sp_investigation
description: "CORRECTION (2026-08-15): SP WAS achieved with GT feedback on ArUco (56-76% rate, see project_why_sp_achieved), and the terminal 1/Z issue was a Z_REG=0.01 test-harness artifact, NOT an unsolved control-law limit -- an earlier claim this session got both facts wrong. Cross-marker GT-FB IC2 n=1 test failed (xy=1.28,vel=6.65); n=10 rerun launched to get a real comparison."
metadata: 
  node_type: memory
  type: project
  originSessionId: bc71d54e-36e7-454b-9b2e-33cc103d6bc0
  modified: 2026-08-15T05:41:44.655Z
---

**⛔ Self-correction, 2026-08-15.** Earlier in this session I claimed "SP was never achieved with GT
feedback, even on the fully-tuned ArUco pipeline" and "the terminal 1/Z lateral-bearing amplification
was never resolved" — **both wrong**, corrected directly by the user. I had only searched
`parameter_record.ods`'s `PX4_NewCal_Record` table entries NC110-168 (the June 23-24 GT-FB campaign's
EARLY, unsuccessful phase, where SP=0 for every row) and stopped there without checking later dates or
[[project_why_sp_achieved]].

**What actually happened (see [[project_why_sp_achieved]], [[feedback_zreg_gear_floor_artifact]] for
full detail):** the campaign continued past NC168. On 2026-06-29→06-30, SP flipped from chronic 0 to
**19/25 (76%)**, **7/9 (78%)**, pooled **56/98 (57%)** across IC1-5, GT-feedback. Root cause of the
flip: `PLASMC_GT_Z_REG` (the GT-feedback harness's depth-regularization constant, `1/(z+Z_REG)`) was
0.01 — far below the drone's physical landing-gear floor (~0.1-0.2m) — so the GT-synthesized depth
could fall below the real floor, producing a **fake, unbounded 1/Z singularity that doesn't exist
physically**. This fake singularity, not a real control-law limit, was driving the "terminal wall."
Raising `Z_REG` to 0.2 (matching the gear floor) restored the Lyapunov proof's Assumption 1 (β=1/Z
bounded) and the controller converged to SP. Four other real controller fixes contributed:
`W_U_MAX` 1.0→2.0, `VDS_KF_Q` 10→1 (later re-baked back to 10 for an unrelated perception-mode reason
— see caveat below), a kappa breach-leak fix, `GAMMA_xy`/`h_rd` retuning.

**Caveat that DOES matter (from the source memory, still valid):** `Z_REG` itself is a GT-feedback-
harness artifact fix, not something perception-based control benefits from directly (perception gets Z
from vision, not a synthetic `1/(z+Z_REG)`). What transfers is the UNDERSTANDING (bound 1/Z near the
deck) and the four real controller fixes (W_U_MAX, VDS_KF_Q, breach-leak, GAMMA). The perception-ON
terminal collapse (decode/loom failure at the deck) remains a SEPARATE, still-unclosed problem — this
part of my original framing was directionally okay, just wrongly hung on a "control law can't do it
even with perfect info" claim that isn't true.

**Checked current cross-marker `controller.py` defaults against the winning GT-FB config (2026-08-15):**
`XIR=0.10` ✓, `PR0=10.0` ✓, `W_U_MAX=2.0` ✓, `Z_REG=0.2` ✓ (all match). **Mismatch:** `VDS_KF_Q` is
back at 10.0 (not the 1.0 that helped originally) — re-baked later (2026-07-04) because PR0=10's
funnel-shape fix already absorbs the terminal 1/Z at q=10, and q=1 was found to mask a DIFFERENT root
cause (terminal-deck fly-aways from a perc spike after the drone reaches deck but doesn't disarm) that
needs its own fix (terminal commit/disarm), not present by default AFAICT. So most, not all, of the
winning config is active.

**Today's cross-marker GT-FB test, n=1 (before this correction, DO NOT generalize from it):** IC2,
`PLASMC_GT_FEEDBACK=1`, current defaults → xy_err=1.28m, rel_vel=6.65m/s, FAIL, target_lost=False.
Per [[project_why_sp_achieved]]'s own documented methodology, **the GT-FB SP-count noise floor is
±5-7 out of 25 reps** — a single rep is not informative in either direction. **n=10 rerun launched**
(`test_data/GTFB_CrossMarker_IC2_n10/`, task bc4p2fmbz) to get an actually meaningful comparison
against the perception-based cross-marker baseline (0/25 precise, 1/25 soft from
[[project_20260813_cbf_extent_fix_followup]]).

**How to apply next time:** when checking "was X ever achieved," search past the first negative
streak in a campaign's dated record — momentum/breakthroughs often land later in the same campaign
under a different table entry or date range. `parameter_record.ods` has multiple tables
(`PX4_NewCal_Record`, `All_Test_Runs`, `Genuine_SP_Reps`) — `All_Test_Runs`'s per-config SP counts
are a faster sanity check than manually scanning `PX4_NewCal_Record` rows one campaign-phase at a
time. Also grep `Memory/px4/` for `project_why_*`/outcome-summary files before concluding something
was never solved — this codebase has a strong habit of writing exactly that kind of synthesis memory.
