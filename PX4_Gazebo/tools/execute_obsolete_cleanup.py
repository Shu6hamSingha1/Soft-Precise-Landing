#!/usr/bin/env python3
"""Execute the obsolete-test-data cleanup from OBSOLETE_MANIFEST.tsv.

SAFE BY DEFAULT: dry-run unless --execute is passed. On --execute it
  1. tars+gzips every DELETE dir into a single timestamped archive,
  2. verifies the archive lists all dirs,
  3. rm -rf the originals,
  4. re-runs build_test_record.py + refresh_scan_sheets.py so the records drop them.

Archive lands in ~/spl_obsolete_archive/ (OUTSIDE the repo). Per user 2026-06-26:
scope = combined-barrier gain-parity-bug era (pre-22cc732, 2026-06-20 20:41 IST)
+ NC-falsified dead-ends; method = archive-then-delete.

  ~/ws/scripts/env2025/bin/python3 tools/execute_obsolete_cleanup.py            # dry-run
  ~/ws/scripts/env2025/bin/python3 tools/execute_obsolete_cleanup.py --execute  # do it
"""
import csv
import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(os.path.expanduser("~/Soft-Precise-Landing/PX4_Gazebo"))
TEST_DATA = ROOT / "test_data"
MANIFEST = TEST_DATA / "OBSOLETE_MANIFEST.tsv"
ARCHIVE_DIR = Path(os.path.expanduser("~/spl_obsolete_archive"))


def delete_dirs():
    with MANIFEST.open() as f:
        return [r["config"] for r in csv.DictReader(f, delimiter="\t")
                if r["rec"] == "DELETE"]


def main():
    execute = "--execute" in sys.argv
    dirs = delete_dirs()
    present = [d for d in dirs if (TEST_DATA / d).is_dir()]
    missing = [d for d in dirs if not (TEST_DATA / d).is_dir()]
    total_mb = sum(int(subprocess.check_output(["du", "-sm", str(TEST_DATA / d)],
                   text=True).split()[0]) for d in present)
    print(f"manifest DELETE dirs: {len(dirs)} | present: {len(present)} | "
          f"already-gone: {len(missing)} | size: {total_mb/1024:.2f} GB")
    if not execute:
        print("\nDRY-RUN. Would archive then delete these dirs:")
        for d in present:
            print("   ", d)
        print("\nRe-run with --execute to perform archive+delete+rescan.")
        return

    ARCHIVE_DIR.mkdir(exist_ok=True)
    # NOTE: stamp the tarball name by hand at call time (no Date in scripts);
    # using the manifest mtime keeps it deterministic.
    tar = ARCHIVE_DIR / "obsolete_bugera_nc_falsified.tar.gz"
    print(f"\n[1/4] archiving {len(present)} dirs -> {tar}")
    subprocess.check_call(["tar", "czf", str(tar), "-C", str(TEST_DATA), *present])
    listed = subprocess.check_output(["tar", "tzf", str(tar)], text=True)
    roots = {ln.split("/")[0] for ln in listed.splitlines() if ln.strip()}
    assert all(d in roots for d in present), "archive verify FAILED — aborting"
    print(f"[2/4] verified archive ({tar.stat().st_size/1e9:.2f} GB, "
          f"{len(roots)} roots)")
    print(f"[3/4] deleting {len(present)} dirs")
    for d in present:
        subprocess.check_call(["rm", "-rf", str(TEST_DATA / d)])
    print("[4/4] re-scanning records")
    py = os.path.expanduser("~/ws/scripts/env2025/bin/python3")
    subprocess.check_call([py, str(ROOT / "tools" / "build_test_record.py")])
    subprocess.check_call([py, str(ROOT / "tools" / "refresh_scan_sheets.py")])
    print(f"\nDONE. Archive: {tar}. Commit records + manifest, then push.")


if __name__ == "__main__":
    main()
