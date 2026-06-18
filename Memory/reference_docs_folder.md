---
name: reference-docs-folder
description: "PX4_Gazebo/docs/ holds in-repo design+analysis notes (cbf2 design, MATLAB↔Python parity, .sh patterns) plus two superseded/falsified briefs — check before re-deriving; per-file status inside."
metadata: 
  node_type: memory
  type: reference
  originSessionId: 7415f420-9591-41b1-8349-bb9361a8dc82
---

# PX4_Gazebo/docs/ — in-repo design & analysis notes

These weren't auto-surfaced before (only `SH_REFERENCE.md` was in CLAUDE.md); now indexed in CLAUDE.md's Project Structure too. They live in the git repo (not my memory dir) because they're project documentation. Check before re-deriving. **Status as of 2026-06-10:**

- **PLASMC_TUNING_GUIDE.md** — ⭐ **the tuning entry point** (auto-injected at session start by a SessionStart hook in `.claude/settings.json`): current state, how-to-gather-data, parameter inventory, dead-ends, methodology, gotchas; indexes all the below + the trajectory memory + the tune-plasmc skill. Read it first for any tuning/diagnosis work.
- **SH_REFERENCE.md** — canonical bash patterns for new sweeps/harnesses. **CURRENT** (mirrors the `sh-script-patterns` skill).
- **FUNNEL_CBF_DESIGN.md** — target-visibility cbf2 (camera-plane tilt-QP) design. **CURRENT**; top addendum covers the Jun-9 two-phase-δ + theta_cap-post-QP refinements. See [[project_cbf_visibility_design]], [[feedback_cbf_theta_cap]].
- **CONTROLLER_PARITY.md** — MATLAB↔Python term-by-term diff. §1 math ports **CURRENT**; top addendum lists the Jun-4–10 *intentional* divergences (CTRL_ZERO_WXY, yaw 2π + conditional integration, K_rd=0, gamma_s=1.0, KAPPA0_Z, cbf2). See [[project_plasmc_port_status]].
- **PARAMETER_ANALYSIS.md** — **CURRENT** (rewritten 2026-06-10 under the honest cal): comprehensive per-parameter analysis + the κ-runaway explosion chain + dead-ends + current best config. The old Jun-2 "SP=0.08% / lag floor" version (cal-contaminated) is in git history. See [[reference_tuning_trajectory]].
- **PERCEPTION_FLOW_FINDINGS.md** — **CURRENT** (rewritten; renamed from `PERCEPTION_FLOW_UNDERREPORT_BRIEF.md`): perception-layer findings — flow is HONEST at altitude; the binding limit is **LK dynamic range ~2 m/s** (next lever = pyramidal LK). The "under-reports 5–25×" hypothesis was falsified. See [[feedback_flow_underreport_brief_falsified]].
