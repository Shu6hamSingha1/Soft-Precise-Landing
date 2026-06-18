---
name: feedback_use_defaults
description: "Always use code defaults for all parameters. Ask user before setting any non-default env var in a run command."
metadata:
  node_type: memory
  type: feedback
  originSessionId: a377a083-d63b-447a-908e-12017cf609f1
---

Always use code defaults for all parameters when launching runs. Do not set env vars like `LANDING_REF_RAD_OPT_FLOW`, `PLASMC_KP_X`, `PLASMC_E_X`, etc. unless the user has explicitly asked for a non-default value in the current conversation.

**Why:** non-default values silently make results incomparable to the baseline and the parameter record. Stale comments in the code (e.g. the REF_RAD=-0.70 guidance from 2026-05-20) may no longer be valid under the current cal regime.

**How to apply:** before every `run_aruco_landing*.sh` or `run_ic_validation.sh` call, strip all PLASMC_* and LANDING_* env overrides unless the user has explicitly requested them in this conversation. If a non-default value seems warranted, state the reason and ask first.
