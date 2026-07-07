---
name: feedback_terminal_overflow_deck_flyaway
description: The deck fly-away/TL root is terminal marker OVERFLOW (not drift-out) as Z→0 → loom+lateral both go stale/blind → descent blows through 0.2m + bounce → armed drone reacts blind → launch
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 3c2f4c67-05c1-4e6f-966b-0e62018fc8a7
---

Root-caused 2026-07-04 on a perception-ON IC2 fly-away (kfq rep 20-30-50), with the observer
fixes ([[feedback_centroid_rate_observer_fixes]]) already in. The recovery + descent WORK now
(observer tracks GT, drone reaches the deck near-centered ~0.05m); the failure is **entirely at
the deck**, and it is a PERCEPTION-OVERFLOW event, not a control or drift-out event.

**OVERFLOW, not drift-out (confirmed, quantitative).** At the decode-death frame the marker
CENTROID |s|=0.41 (dead-centered, far inside the 0.89 short-axis FoV edge) but the max
corner-bearing = 0.95 — *past* the edge. The marker's CORNERS exceed the frame while it's
centered = overflow. Drift-out (centroid→edge, |s|→1.17) happens only AFTER, as a consequence of
going blind. Geometry: small nested marker edge = 270·0.14/Z px → 189px@0.2m, 378px@0.1m,
756px@0.05m (OVERFLOWS 480). Big marker overflows far earlier and its partial border interferes
with the nested detection → Ncorn FLICKERS from ~alt 0.2-0.3m, dies permanently <0.1m. **Any
finite marker overflows as Z→0** — the nested small marker only buys altitude (~0.15m), can't
survive touchdown.

**Two terminal signals both die from the same overflow:**
- LATERAL: observable ONLY from the marker's textured corners. The ring flow (marker-less nadir
  patch) IS alive (Nring 17-105) but is a LOOM signal with **~0 lateral observability** (the
  `Ring h` lateral values are noise) — it CANNOT rescue lateral. So at overflow, lateral goes
  blind (`Opt Flow Ang Vel`=0, gated on the marker; KLT fallback fails — corners exit).
- LOOM: held/stale at its last in-funnel value (−0.39) as Ncorn flickers → it **under-reports
  the true descent ~5×** (true loom ≈ −2.0 at 0.2m/0.4m/s vs −0.42 target). The descent SMC
  believes it's on-target → never applies the terminal brake → drone blows THROUGH the 0.2m
  gear-contact height at ~0.4-0.5 m/s (5× the ~0.08 m/s v∝Z target), penetrates the soft Gazebo
  contact to center-alt≈0, and BOUNCES (loom sign-flips positive — real, GT-confirmed +0.21).

**Why it launches:** at the bounce the drone is armed, tilted ~3.7° (lateral a_u), and blind
(marker overflowed). It reacts to the corrupt/lost signal → climbs + flies away (perc h=0 during
the runaway). The **loom-inversion touchdown detector** (controller.py ~801, needs h_z>0 for 3
consecutive frames + near-centered) is DEFEATED by the overflow: the loom flickers
(held −0.39 / spike −1.35 / flip +0.30) so it never gets 3 consecutive h_z>0 → touchdown never
fires (log shows "Marker lost → TARGET_LOST" then "Impact |a|=271", never TOUCHDOWN-DETECT).

**V_ds KF q only modulates severity** (q=10 → 7-31m fly-aways, q=1 → ~1m TL) — NOT the root
[[feedback_vds_kf_q_severity_bandaid]].

**Why:** lateral AND vertical terminal failures share ONE cause — overflow freezing the
perception at the deck. **How to apply:** the fix is NOT a better flow (ring can't give lateral,
any marker overflows). Candidates: (a) a near-centered proximity/extent commit at ~0.2m that
latches open-loop descent+disarm BEFORE overflow (the extent-commit was removed 06-30 — this
argues to reinstate it, near-centered-gated); (b) a marker that survives closer (3rd nested
level buys altitude but never survives Z→0). Do NOT chase the loom-inversion detector — it's
fundamentally too late (fires at contact, not 0.2m) AND fragile to the overflow flicker.
Overflow confirmed quantitatively (corner-bearing 0.95>0.89 while centroid centered); a raw-frame
IMG_RECORD_RAW visual is still TODO (the retry-wrapper run produced 0 frames — gate didn't trip).
