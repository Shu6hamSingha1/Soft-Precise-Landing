---
name: centroid-kf-default
description: "2026-06-06 user decision: the OUTER-loop centroid filter switched from savgol(13,1) to the 2-state KF by default (IMG_FEATURE_FILTER default 'savgol'->'kf' in img_data.py). The whole image path (flow + ring + centroid) is now on KF. savgol is opt-out via IMG_FEATURE_FILTER=savgol."
metadata:
  node_type: memory
  type: feedback
  originSessionId: kf-switch-2026-06-06
---

**Change:** `getImgFeatureParam()` default flipped to the centroid KF
(`os.environ.get('IMG_FEATURE_FILTER','kf') != 'savgol'`). Was default 'savgol'
"until A/B-validated"; user closed the case on the analytical argument: savgol(13)
adds **~110 ms group delay** on the centroid and is **~2x noisier** than the KF,
which the KF cuts on both counts. The centroid 4-state model is the same 2-state
constant-velocity KF as the flow KF (`_kf_feat_x`).

**State of the image filter chain now:**
- optic flow `[h;w]` (middle/SMC loop): KF (`getOptFlowAngVel`, `IMG_FILTER` default kf)
- centroid `s` (outer/PID loop): **KF now** (`getImgFeatureParam`, `IMG_FEATURE_FILTER` default kf)
- ring flow: KF only
→ **KF everywhere.** Both KF and savgol outputs are still logged every frame for A/B
(`Opt Flow KF` / `Opt Flow Savgol`).

**Why:** lower lag on the OUTER loop is exactly where off-center convergence stalled
(KP=9 commanding against a ~110 ms-stale centroid). **How to apply:** revert per-run with
`IMG_FEATURE_FILTER=savgol` if a regression appears.

**CAVEAT — gate not yet run.** Per the project rule ([[feedback_ic_validation]]) a default
change should pass the IC2-5 gate (`run_ic_validation.sh`) before merge; this flip was made on
the analytical case, NOT yet IC2-5-validated. The controller (KP/KI/KD, precision tuning) was
co-tuned to the savgol-lagged centroid — watch for terminal overshoot/chatter on the first KF
landings and A/B against `IMG_FEATURE_FILTER=savgol`. See [[feedback_precision_tuning_lessons]].
