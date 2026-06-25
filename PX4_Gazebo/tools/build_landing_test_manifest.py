#!/usr/bin/env python3
"""Phase 2 of the obsolete-cleanup: rep-level manifest for test_data/Landing_Test/.

Landing_Test is the autosave mega-dir (~1224 timestamped rep dirs, 6.3 GB) that
mixes eras, so it can't be dir-deleted like Phase 1. This walks every rep, parses
its datetime from the dir name (`Day Mon DD HH-MM-SS YYYY`, machine-local = IST),
and flags reps dated BEFORE the combined-barrier gain-parity-bug fix cutoff
`2026-06-20 20:41:48 +0530` (22cc732) as obsolete (bug-era + back-mapped +
cal-contaminated). Emits test_data/Landing_Test/LANDING_TEST_MANIFEST.tsv.

NOTHING is deleted here — build + REVIEW, then a separate execute step.

  ~/ws/scripts/env2025/bin/python3 tools/build_landing_test_manifest.py
"""
import os
import subprocess
from datetime import datetime
from pathlib import Path

ROOT = Path(os.path.expanduser("~/Soft-Precise-Landing/PX4_Gazebo"))
LT = ROOT / "test_data" / "Landing_Test"
MANIFEST = LT / "LANDING_TEST_MANIFEST.tsv"

# combined-barrier gain-parity-bug fix (22cc732, 2026-06-20 20:41:48 +0530).
# dir names are machine-local (IST) so compare naive-to-naive.
CUTOFF = datetime(2026, 6, 20, 20, 41, 48)


def parse_dt(name):
    """`Fri Jun 12 00-13-46 2026` -> datetime, or None if unparseable."""
    try:
        return datetime.strptime(name, "%a %b %d %H-%M-%S %Y")
    except ValueError:
        return None


def dir_size_mb(p):
    return int(subprocess.check_output(["du", "-sm", str(p)], text=True).split()[0])


def main():
    rows = []
    n_unparsed = 0
    for entry in sorted(LT.iterdir()):
        if not entry.is_dir():
            continue
        dt = parse_dt(entry.name)
        if dt is None:
            # fall back to mtime so nothing is silently skipped
            dt = datetime.fromtimestamp(entry.stat().st_mtime)
            src = "mtime"
            n_unparsed += 1
        else:
            src = "name"
        rec = "DELETE" if dt < CUTOFF else "KEEP"
        rows.append((rec, entry.name, dt.isoformat(sep=" "), src,
                     dir_size_mb(entry)))

    with MANIFEST.open("w") as f:
        f.write("rec\trep\tdatetime\tdt_src\tsize_mb\n")
        for r in rows:
            f.write("\t".join(str(x) for x in r) + "\n")

    delete = [r for r in rows if r[0] == "DELETE"]
    keep = [r for r in rows if r[0] == "KEEP"]
    del_mb = sum(r[4] for r in delete)
    keep_mb = sum(r[4] for r in keep)
    print(f"reps total: {len(rows)} | unparsed-name (used mtime): {n_unparsed}")
    print(f"  DELETE (< {CUTOFF}): {len(delete)} reps / {del_mb/1024:.2f} GB")
    print(f"  KEEP   (>= cutoff) : {len(keep)} reps / {keep_mb/1024:.2f} GB")
    if delete:
        dd = [datetime.fromisoformat(r[2]) for r in delete]
        print(f"  DELETE date span: {min(dd)} .. {max(dd)}")
    if keep:
        kk = [datetime.fromisoformat(r[2]) for r in keep]
        print(f"  KEEP   date span: {min(kk)} .. {max(kk)}")
    print(f"\nwrote {MANIFEST}")


if __name__ == "__main__":
    main()
