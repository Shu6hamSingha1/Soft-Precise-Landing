#!/usr/bin/env python3
"""Phase 2 executor: prune obsolete reps from test_data/Landing_Test/ per
LANDING_TEST_MANIFEST.tsv (build it first with build_landing_test_manifest.py).

SAFE BY DEFAULT: dry-run unless --execute is passed. On --execute it
  1. tars+gzips every DELETE rep into one archive (OUTSIDE the repo),
  2. verifies the archive lists all reps,
  3. rm -rf the originals,
  4. re-runs build_test_record.py + refresh_scan_sheets.py so records drop them.

The lone Landing_Test "SP" is a frozen-GT false-SP (feedback_false_sp_frozen_gt),
so no genuine SP is lost.

  ~/ws/scripts/env2025/bin/python3 tools/execute_landing_test_cleanup.py            # dry-run
  ~/ws/scripts/env2025/bin/python3 tools/execute_landing_test_cleanup.py --execute  # do it
"""
import csv
import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(os.path.expanduser("~/Soft-Precise-Landing/PX4_Gazebo"))
LT = ROOT / "test_data" / "Landing_Test"
MANIFEST = LT / "LANDING_TEST_MANIFEST.tsv"
ARCHIVE_DIR = Path(os.path.expanduser("~/spl_obsolete_archive"))


def delete_reps():
    with MANIFEST.open() as f:
        return [r["rep"] for r in csv.DictReader(f, delimiter="\t")
                if r["rec"] == "DELETE"]


def main():
    execute = "--execute" in sys.argv
    reps = delete_reps()
    present = [d for d in reps if (LT / d).is_dir()]
    missing = [d for d in reps if not (LT / d).is_dir()]
    total_mb = sum(int(subprocess.check_output(["du", "-sm", str(LT / d)],
                   text=True).split()[0]) for d in present)
    print(f"manifest DELETE reps: {len(reps)} | present: {len(present)} | "
          f"already-gone: {len(missing)} | size: {total_mb/1024:.2f} GB")
    if not execute:
        print(f"\nDRY-RUN. Would archive then delete {len(present)} reps "
              f"(Jun 5-16, pre-cutoff). Sample:")
        for d in present[:5]:
            print("   ", d)
        print("    ...")
        for d in present[-3:]:
            print("   ", d)
        print("\nRe-run with --execute to perform archive+delete+rescan.")
        return

    if not present:
        print("\nNothing to do — all manifest DELETE reps are already gone "
              "(cleanup completed 2026-06-26). REFUSING --execute: tarring an "
              "empty set would overwrite the real archive "
              "(~/spl_obsolete_archive/obsolete_landing_test_precutoff.tar.gz).")
        return

    ARCHIVE_DIR.mkdir(exist_ok=True)
    tar = ARCHIVE_DIR / "obsolete_landing_test_precutoff.tar.gz"
    print(f"\n[1/4] archiving {len(present)} reps -> {tar}")
    # -C LT so paths in the tar are rep-dir roots; pass via --files-from to
    # avoid an over-long argv (843 dir names).
    listfile = ARCHIVE_DIR / "_lt_filelist.txt"
    listfile.write_text("\n".join(present) + "\n")
    subprocess.check_call(["tar", "czf", str(tar), "-C", str(LT),
                           "--files-from", str(listfile)])
    listed = subprocess.check_output(["tar", "tzf", str(tar)], text=True)
    roots = {ln.split("/")[0] for ln in listed.splitlines() if ln.strip()}
    assert all(d in roots for d in present), "archive verify FAILED — aborting"
    print(f"[2/4] verified archive ({tar.stat().st_size/1e9:.2f} GB, "
          f"{len(roots)} roots)")
    print(f"[3/4] deleting {len(present)} reps")
    for d in present:
        subprocess.check_call(["rm", "-rf", str(LT / d)])
    listfile.unlink()
    print("[4/4] re-scanning records")
    py = os.path.expanduser("~/ws/scripts/env2025/bin/python3")
    subprocess.check_call([py, str(ROOT / "tools" / "build_test_record.py")])
    subprocess.check_call([py, str(ROOT / "tools" / "refresh_scan_sheets.py")])
    print(f"\nDONE. Archive: {tar}. Commit records + manifest, then push.")


if __name__ == "__main__":
    main()
