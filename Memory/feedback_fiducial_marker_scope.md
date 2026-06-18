---
name: Don't frame fiducial markers as a constraint to avoid
description: Locked 2026-05-13. The practical VDF-ASMC implementation assumes image feature points are available (e.g., from fiducial markers like AprilTags). Do NOT frame "marker-free" or "no fiducial markers" as part of the operational constraints the paper handles — it would contradict the actual deployment setup.
type: feedback
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---
## Rule

When motivating the paper or listing operational constraints, **do not claim** that VDF-ASMC works "without fiducial markers" or "on non-cooperative targets without markers".

## Why

The actual implementation tracks $N$ distinguishable image feature points $\,^\mathcal{C}\hat{\boldsymbol{r}}_i$ on the target image plane (per main paper §II-B). In practice these come from fiducial markers (AprilTag, ArUco, etc.) or other cooperative-target features. The paper's scope is *given* image feature points — the upstream feature-detection problem is out of scope.

The user clarified 2026-05-13 ("don't mention anything about fiducial marker since the practical implementation of the same approach will have fiducial marker") when an §I draft proposed "marker-free" as a paper differentiator.

## How to apply

**Acceptable framings**:
- "monocular vision" — true
- "scale-free" / "no metric depth or velocity estimate" — true
- "low-SWaP single-camera platform" — true
- "GPS-denied environments" — true
- "unknown target depth / motion / dimensions" — true (the scale parameters are unknown, even if features are detected)

**Unacceptable framings**:
- "marker-free" or "without fiducial markers"
- "on non-cooperative targets with no markers"
- "without prior knowledge of target appearance"
- "partially observed target" — **WRONG**. The target IS fully observed on the image plane (all corner points detected via fiducial markers). What is unknown is the *metric scale* of the observation (depth, motion, geometry), not the observation itself. Calling the target "partially observed" misrepresents the setup. User correction 2026-05-14.

## Related conventions

- `feedback_validate_critiques_against_cited_works.md` — verify scope claims against actual paper setup.
- `feedback_visibility_phrasing.md` — visibility framings.
