---
name: feedback_shared_issue_fix_in_ubuntu
description: "When a control bug exists in BOTH MATLAB and PX4/Python, fix it on Ubuntu where it's SITL-validated, not in MATLAB"
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 5ef5a2d7-329a-4539-b101-9f7e6204c84a
---

When a control-formulation issue exists in BOTH the MATLAB Phase-1 controller AND the PX4/Python
implementation, the user does the **correction on the Ubuntu system** — which has **BOTH MATLAB and
PX4** available (MATLAB runs on Ubuntu, not just Windows).

**Why:** keeping the convergence and SITL validation on one machine avoids double work / divergence.
MATLAB on Ubuntu is the fast, deterministic test bed (fixed dt, repeatable); PX4-SITL is the live
validation. Converge the shared mechanism in MATLAB, then port to PX4 (or vice-versa) and keep both
in sync.

**How to apply:** when MATLAB diagnosis surfaces a clearly-shared problem (e.g. the SEN_FUNNEL
demand-starvation, §9 — both codebases have the back-mapped PPC), hand off the diagnosis + lever
ranking + BOTH MATLAB and PX4 code locations to the Ubuntu session. The user may make the IC5/MATLAB
work the PRIMARY focus there, with PX4 alongside. Established 2026-06-14 (CBF/SEN port; IC5 SEN_FUNNEL
resolution moved to Ubuntu-MATLAB). Related: [[project_cbf_sen_matlab_port]].
