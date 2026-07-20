---
name: feedback_s2_homogeneous_decay_bug
description: "s[2] (the fixed homogeneous \"1.0\" constant in s=[xc,yc,1.0,alpha]) collapsed to 0.0 during extended coasts because it was multiplied by the same consecutive-miss decay factor as position — caused a single-frame a_u spike to 610,997. Fixed 2026-07-17."
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 7bc77c5e-027e-4e24-82cb-7e2996f36559
---

`img_data.py`'s feature vector `s = [xc, yc, 1.0, alpha]` (`_computeFeatureVec`) has a
structural homogeneous constant at index `[2]` — always `1.0`, never a decaying quantity.
During an extended total coast (no raw decode, no PlanarFeatureMap rescue —
`not FEATURE_DATA_IS_LOGGED and not _pm_rescue`), the extrapolation path computes
`extrapolated_img_feature_param = np.clip(_fit * _decay, -5.0, 5.0)`
(img_data.py, the `s` extrapolation block) where `_decay = max(0, 1 -
consec_misses/H_EXTRAP_DECAY_FRAMES)` (default 10 frames). This multiply applies
uniformly to **all four** components of `_fit`, including `[2]` — so once a coast
exceeds ~10 frames (~170ms @ 60fps, not rare), `_decay` hits exactly `0.0` and `s[2]`
collapses to `0.0` instead of staying `1.0`. `alpha` (`s[3]`) is separately held right
after this block (wrap-safe hold), so it's unaffected — only `s[2]` fell through.

**Why:** Confirmed live via IC5 SITL trace (2026-07-17): a ~780ms (46-frame) total coast
produced exactly `s=[0,0,0,-3.068]` (malformed homogeneous coordinate) for its entire
duration, and feeding that into the SMC's interaction-matrix math produced a **single-frame
`a_u` spike to 610,997** (decayed back to normal within 6 frames — a numerical artifact
from the degenerate input, not a sustained κ-ratchet; `kappa_norm` stayed flat at 14.618
throughout, confirming it wasn't a runaway). This is a genuine, pre-existing bug
independent of the same-day PlanarFeatureMap/CBF work — any sufficiently long total coast
could trigger it, at any point in the project's history since the decay mechanism was
introduced.

**Fix (part 1, immediate)**: force `extrapolated_img_feature_param[2] = 1.0`
unconditionally after the decay computation (mirrors how `alpha`/`[3]` is already held
separately). Validated: same IC5 rep, `a_u` max dropped from 610,997 → 6.74, xy_err back
to IC5's normal chronic-baseline range (~2.9m) instead of the 22.6m the degenerate input
produced.

**⛔ SUPERSEDED same day — Fix (part 2, root cause)**: part 1 only patched the
homogeneous-constant symptom; the decay-to-exactly-`[0,0]` behavior for POSITION itself
was a separate, deeper problem. User correction: *"the decay-to-zero behavior for
position is intentional design. But I never approved it. We have kf_predict."* Confirmed
live on IC1: decaying to a fabricated "centered, zero error" reading desynced from the
adaptive-gain state — `kappa` ratcheted (3.57→4.18) then FROZE reading "converged" during
a coast, and the real-error snap-back when tracking resumed produced a 2,976 `a_u` spike
(smaller than part 1's bug, but still a real anomaly). **The entire polyfit-trend-fit +
decay extrapolation for `s` was REMOVED**, replaced with the already-existing feature KF's
own predict-only step: `self._kf_feat_update(None, t)` called once early (moved up from
its old post-hoc call site to avoid double-stepping), then
`extrapolated_img_feature_param = self._kf_feat_x[:, 0].copy()` — a genuine
constant-velocity-propagated, growing-uncertainty estimate instead of a hand-rolled
fit+decay. `s[2]=1.0` stays as a redundant hard clamp (belt-and-braces, cheap to keep).
Validated: same IC1 rep, `a_u` max dropped 2,976 → 12.3 (kappa/a_u back in normal healthy
range). The `_img_feature_param_real`/`_real_t` buffer (fed the now-removed polyfit) was
also deleted as dead code (2026-07-17 cleanup pass) — do not resurrect that pattern.

**How to apply:** Any future extrapolation/decay logic touching a vector with a
structural constant component (homogeneous coordinates, fixed normalization terms) must
explicitly exclude that component from generic decay/trend-fit treatment. More broadly:
prefer this codebase's existing predict-only-KF-coast pattern (already used for `h` via
`_kf_update`, for corner-flow via the same mechanism) over a hand-rolled polyfit+decay
when a principled forward-propagation already exists — don't reinvent one. The `h`
(optical-flow) extrapolation path still uses the OLD polyfit+decay mechanism
(`self._h_extrap`, unaffected by this fix) — same audit is worth doing there if a similar
spike is ever traced to it.
