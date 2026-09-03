# RETRACTED — 2026-09-03

This file previously proposed rewriting the manuscript's Visibility-CBF section and appendix to
describe the PX4 joint-`I_a` QP and the `CBF_AZ_COST_GAIN` descent-rate relief, on the premise that
the manuscript's formulation had diverged from the implementation.

**The premise was wrong.** It rested on reading `MATLAB/Multi_init_cond/visualControl_IBVS_adaptive.m`,
which is a STALE self-contained file (last commit 2026-06-25) that the manuscript pipeline does not use.

The actual chain is `run_simulation.m` (the manuscript driver) → `vdf_params()` →
`MATLAB/VDF_ASMC/+blocks/*`. Checked against those:

- `+blocks/cbf_visibility.m` is a camera-plane **theta-QP** via `cbf2_filter` with `P.theta_cap` —
  exactly the manuscript's `\eqref{cbf qp: equation}`. No joint QP, no `A_CAP` sphere, no `a_z` relief.
- `+blocks/flow_surface.m:25` builds `h_d` from the **funnel-prescribed** rate
  (`s_dot_presc = p_10.*S_r.*dp_r`) + transport + descent, and its own comment cites
  `[tex eq. h_d final]`. There is no `-k_r*zeta_r/g_r` back-map term, so the manuscript's
  "no back-mapped rate" statements are correct.
- Every row of Table `sup:control params` matches `vdf_params.m` exactly.

All edits made under the false premise were reverted; `manuscript.tex` is unchanged and has no known
defect in these sections.

The PX4 divergences (joint-`I_a` QP, `a_z` relief, `HD_KR=0.5`, `theta_cap` 43.94°) are real, but they
are **PX4-vs-paper**: the paper documents the MATLAB implementation that produced its results. They are
implementation drift to track on the PX4 side, not manuscript errors.

Still genuinely open on the PX4 side (see the Q1–Q10 thread): the `|I_a + g*e3| <= A_CAP` sphere is a
confirmed bug (bounds deviation-from-hover, not thrust; ~72% over-permit); the cross-marker centroid
may break the symmetric-feature-pattern assumption behind `d_s = 0`; and `Control_Params.npy` records
none of the knobs baked since ~June.

See memory `feedback_matlab_source_of_truth_vdf_params`.
