#!/usr/bin/env python3
"""Build test_data/INDEX.tsv — the CHEAP, GREP-FIRST entry point to the test record.

WHY THIS EXISTS. The record already had two artifacts and neither is usable as a lookup:
  * docs/TEST_RECORD.md      92 KB / ~23k tokens — too expensive to read routinely, so it
                             gets skipped and conclusions get re-derived from raw .npy.
  * test_data/test_record.tsv 8 KB — cheap, but PURE METRICS. It cannot answer "what question
                             did this campaign ask, what did it conclude, where is the reasoning".
INDEX.tsv adds the missing semantic columns while staying small enough to grep a row out of.

HOW TO USE IT (agents: do this instead of reading TEST_RECORD.md):
    grep -i ic5            test_data/INDEX.tsv     # every campaign touching IC5
    grep -i kappa          test_data/INDEX.tsv     # by topic
    awk -F'\t' '$8=="LIVE"' test_data/INDEX.tsv    # only current-era campaigns
    grep GTFB_HoldLastFix  test_data/INDEX.tsv     # one campaign -> its memory backlinks
Then open ONLY the backlink memory file named in the last column. Read TEST_RECORD.md only
when you need the per-rep drill-down that this index deliberately omits.

COLUMNS
  config      test_data/<config>/ directory name (STABLE — 290 citations across the repo
              reference these paths, so they are never renamed; see the memory
              project_landing_test_is_a_duplicate_mirror)
  date        first rep date
  era         aruco-pre320 | cross-320 | rover   (epoch-derived, see ERAS below)
  n, SP, TL   reps, genuine soft+precise landings, target-lost
  xy_med      median final xy error (m)
  status      LIVE        — current era, conclusions still apply
              SUPERSEDED  — predates the 2026-08-27 camera change / 08-28 cross recal;
                            metrics are not comparable to current-era runs
              UNEXPLAINED — no memory/doc records what this campaign asked or concluded.
                            Metrics survive, the reasoning does not. Treat with suspicion:
                            re-deriving intent from a directory name is guesswork.
  backlinks   memory/doc files that actually discuss this config ("-" if none).
              Auto-derived, EXCLUDING the auto-generated digests (which name every config
              and would otherwise make coverage look like 100%).

Regenerate after any campaign or cleanup:
    ~/ws/scripts/env2025/bin/python3 tools/build_test_record.py   # rescan reps first
    ~/ws/scripts/env2025/bin/python3 tools/build_test_index.py
"""
from __future__ import annotations
import glob
import re
import json
import os
import sys
from pathlib import Path

PX4 = Path(os.path.expanduser("~/Soft-Precise-Landing/PX4_Gazebo"))
MEM = Path(os.path.expanduser("~/Soft-Precise-Landing/Memory"))
OUT = PX4 / "test_data" / "INDEX.tsv"

# Auto-generated digests name EVERY config; counting them as backlinks hides the real gap.
AUTO_DOCS = {"TEST_RECORD.md", "TUNING_HISTORY_ARUCO_ERA.md"}

# Epoch boundaries that make older metrics non-comparable.
CAMERA_320 = "2026-08-27"   # 640x480/fx=270 -> 320x240/fx=135
ROVER_PHASE = "2026-09-01"  # moving rover_cross thread opens


def era_of(date: str) -> str:
    if not date:
        return "unknown"
    if date >= ROVER_PHASE:
        return "rover"
    if date >= CAMERA_320:
        return "cross-320"
    return "aruco-pre320"


def load_corpus() -> dict[str, str]:
    corpus = {}
    for p in glob.glob(str(MEM / "**" / "*.md"), recursive=True):
        corpus[p] = _read(p)
    for p in glob.glob(str(PX4 / "docs" / "*.md")):
        if os.path.basename(p) not in AUTO_DOCS:
            corpus[p] = _read(p)
    return corpus


def topic_keys(name: str) -> list[str]:
    """Progressively looser lookup keys for one config directory name.

    Memory almost never quotes a directory verbatim — it names the TOPIC. `KappaRatchet_IC5_base`
    is discussed as "kappa ratchet", so an exact-substring search reports it as unexplained even
    though a whole memory file covers it. So: try the full name, then the name with the IC/rep/
    arm suffixes stripped, then that split on CamelCase into words.
    """
    # Tightest key first: the QUALIFIED path. Bare short names like "Final" are ordinary English
    # and match half the corpus, so a path-qualified hit is the only trustworthy exact match.
    keys = [f"test_data/{name}"]
    if "_" in name or len(name) >= 8:
        keys.append(name)
    stem = re.sub(r'_(IC\d[a-z]?|n\d+|rep\d+|base|baseline|realperc|gate\d*|sweep|test|check|'
                  r'ab|fix\d*|clean|q\d+|z\d+|td\d+|commit\d+)$', '', name, flags=re.I)
    while stem != name and re.search(r'_(IC\d|n\d+|base|baseline|test|ab)$', stem, flags=re.I):
        stem = re.sub(r'_(IC\d[a-z]?|n\d+|base|baseline|test|ab)$', '', stem, flags=re.I)
    if len(stem) >= 5 and stem != name:
        keys.append(stem)
    words = re.sub(r'(?<!^)(?=[A-Z])', ' ', stem.replace('_', ' ')).strip()
    if len(words) >= 6 and words.lower() != stem.lower():
        keys.append(words)
    return keys


def find_refs(name: str, corpus: dict[str, str]) -> list[str]:
    """Files discussing this config, by the loosest key that still hits something."""
    if len(name) < 5:
        return []
    for key in topic_keys(name):
        k = key.lower()
        hits = [os.path.basename(p) for p, t in corpus.items() if k in t.lower()]
        if hits:
            return hits
    return []


def _read(p: str) -> str:
    try:
        return open(p, errors="ignore").read()
    except OSError:
        return ""


def main() -> int:
    rec = json.load(open(PX4 / "test_data" / "test_record_runs.json"))
    corpus = load_corpus()
    names = [(c["config"] if isinstance(c, dict) else c) for c in rec["configs"]]

    # First pass: raw backlinks. A file that name-drops MANY configs (a cleanup note, a
    # session wrap-up, an index) is not explaining any single campaign — counting it makes
    # every row look documented. Keep only campaign-specific files, unless a config has
    # nothing else at all.
    # Breadth is counted in campaign FAMILIES, not rows: KappaRatchet_IC1..5 x base/realperc is
    # 10 rows but ONE campaign, and the memory covering it is specific, not generic.
    raw = {n: find_refs(n, corpus) for n in names}
    fam = {n: topic_keys(n)[-1].lower() for n in names}
    breadth = {}
    for n, refs in raw.items():
        for r in refs:
            breadth.setdefault(r, set()).add(fam[n])
    breadth = {r: len(f) for r, f in breadth.items()}
    GENERIC = 5   # a file spanning >5 distinct campaigns is an index/wrap-up, not an explanation

    rows = []
    for c in rec["configs"]:
        name = c["config"] if isinstance(c, dict) else c
        d = c if isinstance(c, dict) else {}
        date = (d.get("date") or "")[:10]
        specific = [r for r in raw[name] if breadth[r] <= GENERIC]
        refs = specific or raw[name]
        if date and date < CAMERA_320:
            status = "SUPERSEDED"
        elif not refs:
            status = "UNEXPLAINED"
        else:
            status = "LIVE"
        rows.append((name, date, era_of(date), d.get("n", ""), d.get("SP", ""),
                     d.get("TL", ""), d.get("xy_med", ""), status,
                     ",".join(sorted(refs)[:4]) if refs else "-"))

    rows.sort(key=lambda r: (r[1] or "0000", r[0]), reverse=True)
    with open(OUT, "w") as fh:
        fh.write("# test_data/INDEX.tsv — grep this, don't read it whole. "
                 "Regenerate: tools/build_test_index.py (after build_test_record.py)\n")
        fh.write("# status: LIVE=current era | SUPERSEDED=pre-2026-08-27 camera change, "
                 "metrics not comparable | UNEXPLAINED=no memory records what it concluded\n")
        fh.write("config\tdate\tera\tn\tSP\tTL\txy_med\tstatus\tbacklinks\n")
        for r in rows:
            fh.write("\t".join(str(x) for x in r) + "\n")

    n = len(rows)
    st = {s: sum(1 for r in rows if r[7] == s) for s in ("LIVE", "SUPERSEDED", "UNEXPLAINED")}
    print(f"wrote {OUT}  ({os.path.getsize(OUT)/1024:.1f} KiB, {n} configs)")
    print(f"  LIVE {st['LIVE']} | SUPERSEDED {st['SUPERSEDED']} | UNEXPLAINED {st['UNEXPLAINED']}")
    print(f"  vs docs/TEST_RECORD.md at {os.path.getsize(PX4/'docs'/'TEST_RECORD.md')/1024:.0f} KiB")
    return 0


if __name__ == "__main__":
    sys.exit(main())
