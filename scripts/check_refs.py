import re, sys

src = open(r'L:\Claude\Soft Landing\Soft_Precise_Landing\manuscript.tex', encoding='utf-8').read()
# strip comments
src = re.sub(r'(?<!\\)%.*', '', src)
labels = set(re.findall(r'\\label\{([^}]+)\}', src))
refs = set(re.findall(r'\\(?:eq)?ref\{([^}]+)\}', src))
missing = sorted(r for r in refs if r not in labels)
unused = sorted(l for l in labels if l not in refs)
print('missing label targets:', missing if missing else 'NONE')
print('unreferenced labels:', unused if unused else 'NONE')
cites = set(re.findall(r'\\cite\{([^}]+)\}', src))
keys = set(k.strip() for group in cites for k in group.split(','))
bib = open(r'L:\Claude\Soft Landing\Soft_Precise_Landing\bibliography.bib', encoding='utf-8').read()
bibkeys = set(re.findall(r'@\w+\{([^,]+),', bib))
miss_bib = sorted(k for k in keys if k not in bibkeys and not k.startswith('lee2010'))
print('cite keys missing from bib:', miss_bib if miss_bib else 'NONE')
