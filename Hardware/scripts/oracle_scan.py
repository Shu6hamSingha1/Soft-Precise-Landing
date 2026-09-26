import sys, os, numpy as np, cv2
from multiprocessing import Pool
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from perception_hw_common import *
from cross_oracle import oracle
def work(a):
    rd, v = a
    img, ctl, tel = load_run(rd); ti = np.asarray(img['Time'], float); tc = np.asarray(ctl['t'], float)
    if len(tc) < 30: return []
    cap = cv2.VideoCapture(v); out = []; name = os.path.basename(os.path.normpath(rd))
    for k in range(len(ti)):
        r, f = cap.read()
        if not r: break
        if not (tc[0] <= ti[k] <= tc[-1]): continue
        o = oracle(cv2.cvtColor(f, cv2.COLOR_BGR2GRAY))
        out.append((name, k, o['vis'], o.get('cx', np.nan), o.get('cy', np.nan)))
    return out
if __name__ == '__main__':
    jobs = [j for d in ('2026-09-24', '2026-09-25') for j in pair_runs(d)]
    with Pool(max(1, os.cpu_count() - 2)) as p: res = p.map(work, jobs, chunksize=1)
    np.save(sys.argv[1], np.array([r for x in res for r in x], dtype=object), allow_pickle=True)
