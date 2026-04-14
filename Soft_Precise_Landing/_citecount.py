import re, glob, os
os.chdir(os.path.dirname(__file__))
keys=set()
pat=re.compile(r"\\cite\{([^}]*)\}")
for f in glob.glob("*.tex"):
    t=open(f).read()
    for m in pat.finditer(t):
        for k in m.group(1).split(","):
            keys.add(k.strip())
print("TOTAL",len(keys))
for k in sorted(keys): print(k)
