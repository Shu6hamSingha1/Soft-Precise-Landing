#!/usr/bin/env python3
"""
Validate the KLT-fallback ==4 gate hypothesis for momentary (1-2 frame) decode gaps.

img_data.py's KLT corner-fallback requires ALL 4 corners to track (n_tracked == 4,
img_data.py ~line 1398) before accepting a frame -- rejecting an otherwise-good
3/4-tracked frame outright. This tool reads the new "KLT Diag" log (added 2026-07-09,
img_data.py._klt_diag_log) from a landing-test recording and reports how often that
gate is the reason a fallback attempt was rejected, vs. genuine near-total tracking
failure (0-2/4) that a relaxed gate wouldn't have rescued anyway.

Usage: python3 tools/diagnose_klt_gate4.py <rep_dir>
"""
import sys
import numpy as np
from pathlib import Path


def main(rep_dir):
    rep_dir = Path(rep_dir)
    img = np.load(rep_dir / "Img_Data.npy", allow_pickle=True).item()
    diag = img.get("KLT Diag", [])

    if not diag:
        print(f"No KLT Diag entries in {rep_dir.name} -- either no decode gaps occurred, "
              f"or this recording predates the 2026-07-09 diagnostic logging.")
        return

    n_tracked = np.array([d["n_tracked"] for d in diag])
    gate4 = np.array([d["gate4_passed"] for d in diag])
    in_bounds = np.array([d["in_bounds"] for d in diag], dtype=object)
    accepted = np.array([d["accepted"] for d in diag])

    print("=" * 80)
    print(f"KLT FALLBACK GATE DIAGNOSTIC — {rep_dir.name}")
    print("=" * 80)
    print(f"Total KLT-fallback attempts: {len(diag)}")
    print(f"  n_tracked distribution: " + ", ".join(
        f"{k}/4={int(np.sum(n_tracked == k))}" for k in range(5)))
    print()
    print(f"Gate4 PASSED (n_tracked==4): {int(gate4.sum())} / {len(diag)}")
    print(f"  of those, in-bounds (accepted): {int(accepted.sum())}")
    print(f"  of those, out-of-bounds (rejected despite 4/4): {int(gate4.sum() - accepted.sum())}")
    print()
    n_gate4_fail = int((~gate4).sum())
    n_would_rescue = int(np.sum(n_tracked[~gate4] == 3))
    n_total_fail = int(np.sum(n_tracked[~gate4] <= 2))
    print(f"Gate4 FAILED (rejected outright): {n_gate4_fail} / {len(diag)}")
    print(f"  of those, n_tracked==3 (a relaxed >=3 gate + parallelogram completion WOULD rescue): {n_would_rescue}")
    print(f"  of those, n_tracked<=2 (genuine tracking failure, gate relaxation would NOT help): {n_total_fail}")
    print()

    if n_gate4_fail > 0:
        rescue_frac = n_would_rescue / n_gate4_fail * 100
        print(f"HYPOTHESIS CHECK: {rescue_frac:.1f}% of gate4-rejected frames had exactly 3/4 tracked.")
        if rescue_frac > 50:
            print("  -> SUPPORTS the hypothesis: most rejected frames were 1-corner-away from acceptance.")
            print("     A relaxed >=3 gate + parallelogram completion is likely to eliminate most of these gaps.")
        else:
            print("  -> DOES NOT STRONGLY SUPPORT the hypothesis: most rejected frames had <=2 corners tracked,")
            print("     which the strict-4 gate wasn't the proximate cause of -- the underlying LK tracking itself")
            print("     (or the ArUco decode before it) failed more broadly that frame; a relaxed gate wouldn't help.")
    else:
        print("No gate4 rejections in this recording -- nothing to validate here.")

    print("=" * 80)


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python3 tools/diagnose_klt_gate4.py <rep_dir>")
        sys.exit(1)
    main(sys.argv[1])
