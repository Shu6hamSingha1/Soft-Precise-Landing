"""Static check: labels, refs, citations, brace balance for the 4 .tex files."""
import re, os
from pathlib import Path

MANUSCRIPT_DIR = Path(__file__).resolve().parent.parent / "Soft_Precise_Landing"
FILES = ["manuscript.tex", "control_formulation.tex", "results.tex", "supplemental.tex"]

text = {}
for f in FILES:
    with open(MANUSCRIPT_DIR / f, encoding="utf-8") as fh:
        text[f] = fh.read()
all_text = "\n".join(text.values())

labels = set(re.findall(r"\\label\{([^}]+)\}", all_text))
refs   = set(re.findall(r"\\(?:ref|eqref|autoref|cref)\{([^}]+)\}", all_text))
missing_refs = sorted(refs - labels)

cites  = set()
for m in re.findall(r"\\cite[a-z]*\{([^}]+)\}", all_text):
    for k in m.split(","):
        cites.add(k.strip())

bib_keys = set()
bib_path = MANUSCRIPT_DIR / "bibliography.bib"
if bib_path.exists():
    with open(bib_path, encoding="utf-8") as fh:
        bibtxt = fh.read()
    bib_keys = set(re.findall(r"@\w+\{([^,\s]+),", bibtxt))
missing_cites = sorted(cites - bib_keys) if bib_keys else []

print(f"labels   : {len(labels)}")
print(f"refs     : {len(refs)} (missing: {len(missing_refs)})")
if missing_refs:
    for r in missing_refs:
        print(f"  ! {r}")
print(f"bib keys : {len(bib_keys)}")
print(f"cites    : {len(cites)} (missing: {len(missing_cites)})")
if missing_cites:
    for c in missing_cites:
        print(f"  ! {c}")

for f, t in text.items():
    o = t.count("{")
    c = t.count("}")
    print(f"{f:30s}  {{ }} balance: {o} / {c}  {'OK' if o == c else 'MISMATCH'}")
