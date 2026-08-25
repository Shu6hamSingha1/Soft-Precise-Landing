---
name: feedback_aruco_perception_scope
description: "Don't run perception-mode (non-GT-feedback) test flights using the ArUco marker — ArUco is comparison-only now, cross-marker + GT-feedback is the active development track"
metadata: 
  node_type: memory
  type: feedback
  originSessionId: db521dfb-a6f1-44ca-b3f6-432e98fc4866
  modified: 2026-08-25T10:00:45.954Z
---

Stop running perception-based (non-GT-feedback) landing tests using the ArUco marker (`MARKER_TYPE=aruco`, default). ArUco is now **comparison-only** — a reference point to benchmark against, not a target for further perception-pipeline debugging/fixing.

**Why:** user directive, 2026-08-23. The active development track is cross-marker (`MARKER_TYPE=cross WORLD=cross_marker`) with GT-feedback (`PLASMC_GT_FEEDBACK=1`). Once GT-feedback for cross-marker is working well, the next step is a **GT-ablation test** (perception-ON cross-marker vs GT-feedback cross-marker, isolating what the ablation costs). After that, ArUco marker work is no longer needed at all — see [[project_marker_roadmap_gt_ablation]].

**How to apply:** don't launch `run_aruco_landing*.sh` / `run_ic_validation.sh` runs without `MARKER_TYPE=cross WORLD=cross_marker` unless the user explicitly asks for an ArUco comparison run. If mid-investigation work surfaces an ArUco-only perception bug (e.g. the 2026-08-23 kappa-ratchet-via-detection-flicker finding, [[project_20260823_kappa_ratchet_detection_flicker]]), it's fine to have found/diagnosed it, but don't keep iterating perception-mode ArUco test flights to fix it — that's now out of scope. Cross-marker + GT-feedback is where further testing effort goes.

## ⛔⛔ REPEAT VIOLATION (2026-08-25) — this rule was NOT applied for an entire session

Despite this memory existing since 2026-08-23, an entire session's IC5 fly-away/CBF-margin investigation (2026-08-24 into 2026-08-25) ran EVERY test on bare ArUco — no `WORLD`/`MARKER_TYPE` override on a single launch. It started as a deliberate one-off check (switching to ArUco specifically to test whether a failure mode was cross-marker-flicker-specific or general) — a reasonable, legitimate use once. But once that check was done and the mechanism was confirmed general (not cross-marker-specific), there was no remaining reason to stay on ArUco, and every subsequent test in the sweep (the whole `CBF_MARGIN_RESERVE` A/B, both the contaminated and the clean re-run, 12+ landing tests total) kept copy-pasting the same ArUco-default command instead of switching back to cross-marker. The user had to call it out explicitly ("why are we moving back to ArUco again and again?").

**Root cause of the repeat**: the rule was correctly worded but buried near the bottom of a long, rarely-re-read index file. Once a command template gets copy-pasted across many launches in a session, nothing naturally re-triggers a check against a rule that isn't already front-of-mind — a session can "know" a rule exists in memory and still not apply it to the next command, because memory isn't re-consulted before every individual action, only loaded once at session start.

**Fix applied**: moved a loud, explicit restatement of this rule to the VERY TOP of `px4/MEMORY.md` (the auto-loaded index), with an explicit per-launch checklist ("before every landing-test launch, verify: does this command include `WORLD=cross_marker MARKER_TYPE=cross`?"), rather than trusting a single mid-file entry to be re-surfaced on its own.

**How to apply, reinforced:** a ONE-TIME legitimate reason to run ArUco (e.g. "is this bug marker-type-specific?") does NOT license staying on ArUco for the rest of the investigation — switch back to `MARKER_TYPE=cross WORLD=cross_marker` for the very next launch once that specific comparison is done. Re-verify the marker-type flags on every single launch command, not just the first one in a session or the first one after a course-correction.
