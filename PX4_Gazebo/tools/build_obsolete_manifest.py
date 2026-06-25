#!/usr/bin/env python3
"""Build a REVIEWABLE manifest of obsolete test_data/ bundles for deletion.

Scope (user-agreed 2026-06-26):
  A. Combined-barrier GAIN-PARITY-BUG era: configs dated 2026-06-19/06-20 that ran
     the wrong hot back-mapped gains BEFORE the fix (22cc732/edc28c4, ~06-21).
  B. NC-FALSIFIED: bundles explicitly marked Dead-end/Debunked/FALSIFIED/REVERTED
     in PX4_NewCal_Record.

KEEP-LIST (never propose for deletion): genuine-SP, ongoing gates, the live
single-marker / moment-loom / GT-feedback thread (>=06-21), and the big mixed
Landing_Test autosave dir. Writes test_data/OBSOLETE_MANIFEST.tsv for review.
NOTHING is deleted here.
"""
import json
import subprocess
from pathlib import Path
import os

ROOT = Path(os.path.expanduser("~/Soft-Precise-Landing/PX4_Gazebo"))
TEST_DATA = ROOT / "test_data"

# --- B: bundles named in NC rows with an invalidation verdict (dir stems) ---
NC_FALSIFIED = {
    "RestartProbe", "KP_test", "KI_test", "KP13_test", "trial4_n5", "wumax_n3",
    "thetafloor60_n3", "YawGamma1_n3", "TauUa03_n3", "KrYaw_sweep", "YawOmega1_n6",
    "DescentFix_n6", "EzSweep_n5", "Nz05_n6", "KP12_E2505_n5", "Nxy05_n5",
    "DhdMax50_misc_n12", "TauDs05_n5", "GammaS_sweep_n25", "Bootstrap_k0z0312_n4",
    "RateGate_n6", "Stage2_PS0_KP3", "Stage2b_KD_KP3", "LpfBefore_IC2", "LpfKP_IC2",
}

# --- protect: live/valuable threads, never auto-propose ---
KEEP = {
    "CoordDescent", "SPCampaign", "ICValidation", "Landing_Test", "RingFlow", "SenFunnel",
}
# Fix-validation arms to KEEP (manuscript/fixed gains) even though they are
# 06-20-dated — the reference datapoints that motivated the bake (user 2026-06-26).
KEEP_FIXVAL = {"VdfGains_IC2_manuscript", "VdfBake_IC2_combined"}

BUG_DATES = {"2026-06-19", "2026-06-20"}


def stem(name):
    import re
    return re.split(r"_IC\d", name)[0]


def du(path):
    try:
        out = subprocess.check_output(["du", "-sm", str(path)], text=True)
        return int(out.split()[0])  # MB
    except Exception:
        return 0


def main():
    data = json.loads((TEST_DATA / "test_record_runs.json").read_text())
    existing = {p.name for p in TEST_DATA.iterdir() if p.is_dir()}
    rows = []
    for c in data["configs"]:
        name = c["config"]
        if name not in existing:
            continue
        s = stem(name)
        bucket = reason = ""
        rec = "KEEP"
        if name in KEEP or s in KEEP or name in KEEP_FIXVAL:
            continue
        if s in NC_FALSIFIED or name in NC_FALSIFIED:
            bucket, reason, rec = "B", "NC-falsified dead-end", "DELETE"
        elif c["date"] in BUG_DATES:
            bucket, reason, rec = "A", "combined-barrier gain-parity bug era", "DELETE"
        else:
            continue
        rows.append({**c, "bucket": bucket, "reason": reason, "rec": rec,
                     "size_mb": du(TEST_DATA / name)})

    rows.sort(key=lambda r: (r["rec"], r["date"]), reverse=True)
    out = TEST_DATA / "OBSOLETE_MANIFEST.tsv"
    cols = ["rec", "bucket", "config", "date", "n", "SP", "TL", "xy_med",
            "size_mb", "reason"]
    with out.open("w") as f:
        f.write("\t".join(cols) + "\n")
        for r in rows:
            f.write("\t".join(str(r.get(c, "")) for c in cols) + "\n")

    dele = [r for r in rows if r["rec"] == "DELETE"]
    rev = [r for r in rows if r["rec"] == "REVIEW"]
    print(f"manifest: {out}")
    print(f"DELETE candidates: {len(dele)} dirs, "
          f"{sum(r['size_mb'] for r in dele)/1024:.1f} GB, "
          f"{sum(r['n'] for r in dele)} reps")
    print(f"REVIEW (default keep): {len(rev)} dirs, "
          f"{sum(r['size_mb'] for r in rev)/1024:.1f} GB")
    print(f"  bucket A(bug)={sum(1 for r in dele if r['bucket']=='A')} "
          f"B(falsified)={sum(1 for r in dele if r['bucket']=='B')}")


if __name__ == "__main__":
    main()
