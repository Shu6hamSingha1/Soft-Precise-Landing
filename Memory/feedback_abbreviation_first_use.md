---
name: Define abbreviations on first use, then use the short form
description: Any abbreviation (FoV, PPC, IBVS, PBVS, UUB, GUUB, SMC, ASMC, etc.) must be expanded on first occurrence across the manuscript — "camera field of view (FoV)" — and thereafter the short form alone is used
type: feedback
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---
Any abbreviation used in the manuscript must be defined on first occurrence and then used in short form thereafter.

**Known abbreviation set (locked):** FoV, PPC, IBVS, PBVS, UAV, SMC, ASMC, MDF-ASMC, SO(3), UUB, GUUB, PID, NED, OU (Ornstein–Uhlenbeck), ZOH, CoG, IEEE TAES, PX4.

**Why:** User flagged 2026-04-23 that "field of view" was used repeatedly in the Introduction without ever defining "FoV"; first use was mid-§III where FoV was used inline without expansion. The rule now applies globally: every introduced abbreviation must be expanded on its first appearance in the main paper, then short form elsewhere.

**How to apply:**
- Before inserting any abbreviation, grep the .tex files to verify it has already been defined. If not, the FIRST use must expand it: `camera field of view (FoV)`, `image-based visual servoing (IBVS)`, etc.
- Subsequent uses must use the short form — do NOT re-expand ("field-of-view boundary" → "FoV boundary").
- Abstract and main paper maintain separate expansion tracking: an abbreviation expanded in the abstract must still be expanded on its first use in the main text (IEEE convention).
- Supplement inherits main-paper abbreviations but still re-expand at its first use if the supplement stands alone in any distribution channel.

**Edge cases:**
- Proper nouns like "IEEE", "MATLAB", "LiDAR", "GPS", "3-D" do not need expansion.
- "PPC" is already defined at manuscript.tex:105 ("prescribed-performance control (PPC)"). IBVS/PBVS are defined at L101.
