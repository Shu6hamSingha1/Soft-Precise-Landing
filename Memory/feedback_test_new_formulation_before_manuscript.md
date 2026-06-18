---
name: feedback-test-new-formulation-before-manuscript
description: Do NOT port the new (blended-surface) formulation into manuscript.tex until it is tested/validated; control_formulation.tex + the code are the testbed
metadata: 
  node_type: memory
  type: feedback
  originSessionId: ddcd2aa0-2512-455c-bd65-c0801e83e279
---

The new VDF-ASMC formulation (blended sliding surface with `ζ_r`, measured-`ṡ` `h_d`, etc.) is developed and **tested first** in `control_formulation.tex` (the draft) and the controller code (Ubuntu/SITL). **`manuscript.tex` is NOT updated with the new formulation until it is validated.**

**Why:** the canonical paper must reflect the *validated* design. Until the new formulation passes testing, `manuscript.tex` keeps the old SEN-funnel design (PID `s_e_dot_d` → `ṡ_d` → `h_d`, `ζ_h`-only sliding surface).

**How to apply:** when a formulation change comes up, edit `control_formulation.tex` (and defer code to Ubuntu, never edit Python here — see [[feedback_shared_issue_fix_in_ubuntu]]). Do NOT propose or make matching edits to `manuscript.tex`'s formulation (surface, `h_d`, theorems) unless the user explicitly says the new formulation is validated / ready to port. Exception: cross-cutting edits the user explicitly directs on *both* files (e.g. the 2026-06-16 CBF `θ_cap` cleanup) are fine — those weren't the new formulation.

User correction (2026-06-16): "I never asked you to update manuscript with the new formulation. We will first test the new formulation before updating the manuscript." Related: [[feedback_hd_uses_measured_sdot]].

**UPDATE (2026-06-16): GATE CLEARED.** User: "We have validated the new formulation." The blended-surface + measured-`ṡ` + corrected-kinematics path (`COMBINED_BARRIER=1`, `C_SIMPLE=1`) is validated. Porting the new formulation into `manuscript.tex` is now PERMITTED — but still requires explicit user go-ahead before editing the canonical paper (drafts-before-apply; manuscript edits are deliberate). The principle above still applies to any FUTURE not-yet-validated formulation change.
