#!/usr/bin/env python3
"""Build the Tier-2 test/calibration-data cleanup manifest (REVIEW ARTIFACT — deletes nothing).

Tier 1 (commit 1f0b6cab) removed only provably-redundant data: Landing_Test reps that were
byte-identical mirrors of a curated copy, spent PNG intermediates, and aborted reps. Tier 2 is
different — after Tier 1 these are SOLE copies, so deletion is irreversible.

A naive date cut ("drop June+July") was investigated and REJECTED: it would have orphaned 114
cited ICValidation runs (6+ cited from live source as the provenance for guards still running,
e.g. img_data.py:3436 "fly-away (ICValidation/20260716-211434). Fix: REJECT (not clip)"), and
removed the raw traces behind 581 of 938 genuine SP reps (62%).

This manifest instead marks a run KEEP if ANY of:
  * it is dated 2026-08 or later (the live cross-marker / rover thread), OR
  * its name is cited anywhere in src/, tools/, apps/, scripts/, docs/, the manuscript, or the
    memory tree (a citation means something reasoned from it), OR
  * it contains at least one genuine SP rep per test_record_runs.json (the success dataset).
Everything else — June/July, uncited, zero successful landings — is DROP.

Calibration dirs follow the same citation rule (no SP concept); the live-cal provenance dirs are
cited from src/ and so are kept automatically.

  ~/ws/scripts/env2025/bin/python3 tools/build_tier2_manifest.py

Writes test_data/TIER2_MANIFEST.tsv + prints a summary. Review it, THEN execute separately.

⚠ SEQUENCING TRAP: test_record_runs.json / test_record.tsv / TEST_RECORD.md are the ONLY surviving
record of the dropped reps (they carry per-rep xy_err/rel_vel). Re-running build_test_record.py
AFTER a Tier-2 deletion would drop those rows and destroy the aggregate history too. Snapshot the
current record before deleting, and do NOT blindly re-scan afterwards.
"""
from __future__ import annotations
import datetime
import glob
import json
import os
import re
import sys
from pathlib import Path

ROOT = Path(os.path.expanduser("~/Soft-Precise-Landing"))
PX4 = ROOT / "PX4_Gazebo"
MEM = Path(os.path.expanduser(
    "~/.claude/projects/-home-shubham-Soft-Precise-Landing/memory"))
KEEP_FROM = "2026-08"          # the live cross-marker / rover thread


def dir_size(p: Path) -> int:
    tot = 0
    for dp, _, fn in os.walk(p):
        for f in fn:
            try:
                tot += os.lstat(os.path.join(dp, f)).st_size
            except OSError:
                pass
    return tot


def load_citation_blob() -> str:
    """Every place a run name could be referenced as evidence."""
    pats = [PX4 / "src" / "*.py", PX4 / "tools" / "*.py", PX4 / "apps" / "*.py",
            PX4 / "scripts" / "*.sh", PX4 / "docs" / "*.md",
            ROOT / "Soft_Precise_Landing" / "*.tex", ROOT / "CLAUDE.md"]
    files = [f for p in pats for f in glob.glob(str(p))]
    files += glob.glob(str(MEM / "**" / "*.md"), recursive=True)
    out = []
    for f in files:
        try:
            out.append(open(f, errors="ignore").read())
        except OSError:
            pass
    return "\n".join(out)


def sp_runs() -> tuple[set[str], set[str]]:
    """(ICValidation run ids, Landing_Test rep names) holding >=1 genuine SP rep."""
    rec = json.load(open(PX4 / "test_data" / "test_record_runs.json"))
    icv, lt = set(), set()
    for r in rec["sp_reps"]:
        p = r["path"]
        if p.startswith("ICValidation/"):
            icv.add(p.split("/")[1])
        elif p.startswith("Landing_Test/"):
            lt.add(p.split("/", 1)[1])
    return icv, lt


def month_of(p: Path) -> str:
    return datetime.date.fromtimestamp(os.lstat(p).st_mtime).strftime("%Y-%m")


def classify(name, path, blob, sp_set):
    """-> (verdict, reason)"""
    m = month_of(path)
    if m >= KEEP_FROM:
        return "KEEP", f"live thread ({m})"
    if name in blob:
        return "KEEP", "cited in code/docs/manuscript/memory"
    if sp_set is not None and name in sp_set:
        return "KEEP", "contains a genuine SP rep"
    return "DROP", f"{m}, uncited, no SP"


def main() -> int:
    blob = load_citation_blob()
    icv_sp, lt_sp = sp_runs()
    rows = []

    for bucket, sub, sp_set in (("ICValidation", "test_data/ICValidation", icv_sp),
                                ("Landing_Test", "test_data/Landing_Test", lt_sp),
                                ("calibration_data", "calibration_data", None)):
        base = PX4 / sub
        if not base.is_dir():
            continue
        for name in sorted(os.listdir(base)):
            p = base / name
            if not p.is_dir():
                continue
            verdict, reason = classify(name, p, blob, sp_set)
            rows.append((bucket, name, month_of(p), dir_size(p), verdict, reason))

    out = PX4 / "test_data" / "TIER2_MANIFEST.tsv"
    with open(out, "w") as fh:
        fh.write("bucket\tname\tmonth\tbytes\tverdict\treason\n")
        for r in rows:
            fh.write("\t".join(str(x) for x in r) + "\n")

    G = 2 ** 30
    print(f"wrote {out}\n")
    print(f"{'bucket':18s} {'KEEP':>6s} {'GiB':>8s}   {'DROP':>6s} {'GiB':>8s}")
    tot_d = 0
    for b in ("ICValidation", "Landing_Test", "calibration_data"):
        k = [r for r in rows if r[0] == b and r[4] == "KEEP"]
        d = [r for r in rows if r[0] == b and r[4] == "DROP"]
        tot_d += sum(r[3] for r in d)
        print(f"{b:18s} {len(k):6d} {sum(r[3] for r in k)/G:8.2f}   "
              f"{len(d):6d} {sum(r[3] for r in d)/G:8.2f}")
    print(f"\nTOTAL RECLAIM IF EXECUTED: {tot_d/G:.2f} GiB")
    print("Nothing was deleted. Review TIER2_MANIFEST.tsv before any execution step.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
