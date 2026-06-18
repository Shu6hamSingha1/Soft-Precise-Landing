---
name: feedback_reject_on_single_failure
description: "TESTING RULE (user, 2026-06-08): while testing ANY control-param tuning value or test setting, STOP on the FIRST failed landing (TL / crash / no-land) — even before completing all n reps — and deep-analyze it. Reject that value/setting on a SINGLE failure. Soft-precise needs 100% reliability, so one failure = unreliable = out."
metadata: 
  node_type: memory
  type: feedback
  originSessionId: dd8920aa-4635-40a9-ac9d-8409e37243d6
---

**Rule (user, 2026-06-08): a single failed landing rejects the tuning value / test setting. Don't wait for the full n — stop on the first failure and deep-analyze why.**

The goal is **100% reliable** soft-precise landing, so any config that produces even one failed landing (target-lost, crash, or no-land) is unreliable and is rejected. Practically:
- Run a sweep/value; the moment a rep fails, STOP (kill the run), don't finish the remaining reps or the remaining sweep values on the assumption they're fine.
- **Deep-analyze the failure** (root cause: which axis, perception vs control, glitch vs drift, what exploded) before moving on — `tools/diagnose_failure_cause.py`, the explosion-chain trace, per-axis a_u/s_e_n.
- Reject the value; only a value with **zero** failures across n advances.

**Corollary — a single failure reveals the BINDING constraint.** First use of this rule (2026-06-08): a descent-tuning value (`P2INF_Z=1.5`) failed rep1 with a 23 m TL crash, and the deep-analysis showed the failure was a **LATERAL divergence** (x-axis s_e_n 5.28, a_u_xy 8959), NOT the loom glitch we were tuning for (|Δh_z| only 1.68). ⟹ the **lateral drift fails independently**, so NO descent-tuning value can ever pass the gate until the lateral is 100% reliable. The rule re-prioritized the whole effort: fix the *binding* failure (lateral) first, not the one we happened to be deep in (descent). See [[feedback_dont_conclude_lag_floor]], [[feedback_ic_validation]], [[feedback_newcal_tuning_results]].
