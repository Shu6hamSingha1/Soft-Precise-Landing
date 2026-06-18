---
name: feedback-hd-uses-measured-sdot
description: "h_d feedforward uses the MEASURED finite-difference s-dot of the centroid, NOT a desired s_dot_d from a PID; this is what makes the blended surface coherent"
metadata: 
  node_type: memory
  type: feedback
  originSessionId: ddcd2aa0-2512-455c-bd65-c0801e83e279
---

In the blended-surface VDF-ASMC design, the desired optic flow is
`h_d = ṡ + (transport) + h_rd s` where **`ṡ` is the MEASURED finite-difference of the image centroid `s`**, NOT a desired feature rate `ṡ_d` from a PID/funnel.

**Why this matters / why it's coherent:** with the measured `ṡ` in `h_d`, the optic-flow error collapses to the descent-rate error, `h_e = h − h_d = (h·ê₃ − h_rd)s` (verified for the old `w×s` kinematics). Lateral position is then regulated by `ζ_r` in the sliding surface (the blended surface), NOT by `ṡ_d` in `h_d`. So `h_d`-with-measured-`ṡ` and the blended `ζ_r`-in-surface are *compatible* — no PID, no desired feature rate needed.

**Correction event (2026-06-16):** I briefly read the `ṡ` term in `h_d` as a *desired* PID rate (`ṡ_d`), concluded it conflicted with the blended surface, and nearly proposed reverting the blended surface back to the SEN-funnel design. User corrected: "Add ṡ in h_d not ṡ_d, where ṡ is computed using finite-difference of s." No revert needed.

**Read [[project_kinematics_correction_2026_06_11]] accordingly:** its `h_d = ṡ_e + ψ̇_b ê₃×s + h_rd s` has `ṡ_e = ṡ` = the measured finite-difference (since `s_d` is constant), NOT a desired rate.

**Status:** applied to `control_formulation.tex` (blended-surface draft) AND it is what the MATLAB code's `combined_barrier` branch implements (`s_dot_meas = smooth4(diff(V_s_e)/dt)`, L639/665). New formulation **VALIDATED 2026-06-16** (user). `manuscript.tex` still on the OLD SEN-funnel design (PID `s_e_dot_d` → `ṡ_d` → `h_d`); porting it to measured-`ṡ` + blended surface is now UNBLOCKED (gate cleared) but not yet done — awaits user go-ahead. See [[project_kinematics_correction_2026_06_11]] and [[feedback_test_new_formulation_before_manuscript]].
