---
name: feedback_zv_obliqueness_finding
description: "z_v (V-frame reprojection denominator) findings (2026-07-09) - z_v is ray-obliqueness (cos angle ray-to-nadir), NOT depth (scale-free-safe); goes low/negative only as a SYMPTOM of already-diverging tilt; clamp tried+REVERTED; diagnostic _z_v_min_log kept; frame is 480x640 post-ROTATE_90_CW."
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 32b3f473-5219-48ce-b070-323ef9cc309a
---

**What z_v is:** in `_getVirtualPts`, `z_v = ray·z_axis` = the projection of a pixel's homogeneous camera ray onto the gravity-nadir direction ≈ cos(ray-to-nadir angle) scaled by ‖ray‖. Dimensionless, attitude+pixel-position only — **no metric depth, no scale** (does not violate [[feedback_scale_free_depth_free]]). z_v→0 = ray ~90° off nadir (looking at horizon in the leveled frame); z_v<0 = ray points UP in the gravity frame — the standard pinhole divide-by-z degeneracy, relocated by the leveling to a gravity-fixed cone. It truthfully reports "this corner is not representable in the leveled plane," it is not noise.

**Empirical (post-hoc reconstruction over 7 failed tests, 10k frames):** z_v stays healthy 0.85–1.03 through all normal descent; goes low/negative ONLY in the last frames of already-diverging tilt events (the one z_v<0 case was a genuine 93.6° tumble confirmed by FC telemetry). Approx `z_v_worst ≈ cos(θ_ray + θ_tilt)`; nadir-in-body is yaw-invariant. Marker-fill obliqueness θ_ray stays ~30–40° at ALL altitudes (nested big/small handover keeps the tracked corner set in a consistent apparent-size band) — the margin is eaten by terminal TILT excursions (budget-used 9%→21–25% in the 1–3m band), i.e. z_v tracks the terminal 1/Z tilt amplification, it is NOT an independent trigger.

**Two of my own wrong turns, corrected (recorded so they're not repeated):** (1) a clamp `z_v→max(z_v,0.01)` was added then REVERTED — it only bounds an already-fabricated phantom point (classic [[feedback_clamps_during_tuning]] band-aid); right design if ever needed = treat low-z_v corners as NOT-DECODED (marker-lost path), never patch the value. (2) my first "off-frame corners cause it" claim used the WRONG frame box: ArUco detection runs on the post-`cv2.ROTATE_90_CW` image = **480 wide × 640 tall, center (240,320)** — pixel checks against 640×480 are wrong (CLAUDE.md now documents this).

**Kept in code:** `_z_v_min_log` diagnostic (min z_v per frame) in `_getVirtualPts`, no behavior change. Offline reconstruction scripts were session-scratch (not committed) — recompute from Quat + Image Feature Pts if needed.
