---
name: feedback_reject_disable_cast_shadows
description: "User rejected disabling cast_shadows on the drone model (or moving sunUTC) as a fix for the drone-shadow-on-marker perception issue near touchdown — cast shadows are a real-world phenomenon the perception pipeline must handle, not a sim artifact to remove."
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 64366844-45aa-444e-96db-ea10a4e7d714
  modified: 2026-08-25T10:25:25.543Z
---

Disabling `cast_shadows` on the drone model, or repositioning the world's `sunUTC` light, was
proposed (see [[project_20260824_cross_marker_montage_overlay_robustness]]) as a "world-side root
fix" for the drone's own cast shadow corrupting `cross_marker_detector.py`'s alpha/s near
touchdown. **User explicitly rejected this** — a real drone flying in real sunlight casts a real
shadow on the marker near touchdown; removing it from the sim would hide a genuine perception
challenge rather than solve it.

**Why:** the shadow is representative of a real deployment condition, not a simulation artifact.
Making the perception pipeline robust to shadow contamination (as `_robust_fit_line`'s pruning fix
and the corner-join Hough filter already do) is the correct direction; scrubbing the sim
environment to avoid the problem is not.

**How to apply:** don't propose disabling shadows/repositioning the sun as a fix for
shadow-related perception issues (cross-marker or otherwise). Treat shadow robustness as an
ongoing perception-hardening target — same theme as [[feedback_cross_marker_texture_history]]'s
"reject non-marker content" line of work.
