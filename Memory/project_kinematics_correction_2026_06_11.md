---
name: kinematics-correction-2026-06-11
description: "MAJOR: s-dot and c̃_h/d_h equations corrected in manuscript (transport-theorem form, w×s/Coriolis terms falsified numerically vs locked .mat ground truth); MATLAB/PX4 still implement the OLD model (divergence absorbed in d_h) — alignment + re-run pending"
metadata: 
  node_type: memory
  type: project
  originSessionId: 835634e5-9079-4b74-95af-775a75d60862
---

**Finding (2026-06-11, §II para-by-para review):** the legacy image-position kinematics and plant decomposition were wrong given the paper's own definitions (s=r_t/z_t centroid, h=v_rel/z, w=ω_t−ψ̇ê₃). Falsified two ways: (a) 4-line transport-theorem derivation from the paper's definitions; (b) numerically vs locked multi-init .mat ground truth (analytic AND measured centroid; manuscript-form residual 2.5–4× the signal on deck-motion cases, transport form at noise floor ~2%, NOT growing near touchdown).

**Corrected equations (now in manuscript):**
- ṡ = h − ψ̇_b ê₃×s − (h·ê₃)s  (no w×s; target rotation about its own centroid can't move the centroid; deck roll/pitch unequal-depth effect cancels to FIRST order for the symmetric pattern, 2nd-order residual → d_h)
- c̃_h = −ψ̇_b ê₃×h − (h·ê₃)h  (no ẇ, w, s terms!)
- d_h = a_t/z − (F_g+F_d)/(mz)  (Coriolis ω_b×v_b/z term was spurious; UAV absolute velocity exits the model — strengthens own-state-free)
- h_d = ṡ + ψ̇_b ê₃×s + h_rd s  where **ṡ = the MEASURED finite-difference of the centroid**
  (code `s_dot_meas = smooth4(diff(V_s_e)/dt)`), NOT a desired/setpoint rate ṡ_d. This collapses
  h_e = h − h_d = (h·ê₃ − h_rd)s to the descent-rate error; the lateral position FEEDBACK does NOT vanish —
  it moves into ζ_r in the combined sliding surface σ = ζ_h + χζ_r. ⚠ SUPERSEDES the earlier 2026-06-16
  "V_sd = setpoint feature rate" note (that note was WRONG — it mislabeled the measured ṡ as a setpoint demand).
  User 2026-06-16: "Add ṡ in h_d not ṡ_d, where ṡ is computed using finite-difference of s"; VALIDATED 2026-06-16.
  See [[feedback_hd_uses_measured_sdot]].
- Remark 7 reworded: de-rotation removes roll/pitch couplings; surviving yaw-rate term measured + feedback-linearized.
- d_α clause: "together with the residual perspective coupling induced by platform tilt" (α DOES pick up deck tilt via second moments — ~0.08 rad/s oscillatory residual beyond l_α^Tω_t on Case 5; bounded, Assumption-1 covered).

**Provenance of the error:** w×s form is the camera-frame static-target kinematics (Xie/Fink lineage, w = CAMERA angular velocity) transplanted with w redefined as relative rate; in V the camera rate is ψ̇ê₃ → transport form.

**Verified sound:** pseudo-inverse measurement returns TRUE h (not h−ω_t×s; resid 0.029 vs 0.107); α̇=−ψ̇_b+d_α decomposition (d_α bounded ≤0.38, mean = target yaw rate as modeled).

**DONE — implementation alignment + VALIDATED (2026-06-16):** the corrected model is implemented in `visualControl_IBVS_adaptive.m` behind two opt-in global flags:
- `C_SIMPLE=1` → corrected c̃_h = −ψ̇_b ê₃×h − (h·ê₃)h − ḣ_d with clean-IMU ψ̇_b (ZYX yaw-rate from body rates); h_d transport = ψ̇_b ê₃×s (no ẇ/V_dw dependence). Default-OFF path (`c_simple=0`) still runs the OLD w×s/Coriolis c̃_h.
- `COMBINED_BARRIER=1` → blended surface σ=ζ_h+χζ_r (lateral PD ζ_h+χ_r ζ_r, descent PI ζ_h+χ_z∫ζ_h3) + measured-ṡ h_d. Default-OFF path runs the OLD SEN funnel (V_ds_d back-map) on the ζ_h-only surface.
User **VALIDATED the new formulation 2026-06-16** (COMBINED_BARRIER + C_SIMPLE). So the manuscript may now be updated with the new formulation (gate cleared — see [[feedback_test_new_formulation_before_manuscript]]). PX4 controller.py alignment happens on Ubuntu (never edit code here — [[feedback_shared_issue_fix_in_ubuntu]]). [[feedback_optical_flow_naming_epoch5]] [[reference_optical_flow_vs_image_velocity]]
