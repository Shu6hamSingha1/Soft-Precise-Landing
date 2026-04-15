"""Quick summary of multi_init sweep results — land/fail count and timing."""
import os
from pathlib import Path
import scipy.io as sio
import numpy as np

ROOT = str(Path(__file__).resolve().parent.parent / 'MATLAB' / 'Multi_init_cond' / 'Datasets')
TRAJS = ["Static", "Linear", "Sinusoidal", "Lissajous", "Circular"]
CONDS = [("realistic", ""), ("noiseless", "_noiseless")]

def n_logged(d):
    Z = d.X_DS
    valid = np.where(np.any(Z != 0, axis=0))[0]
    return int(valid[-1]) + 1 if len(valid) else 0

print(f"{'trajectory':<12} {'cond':<10} {'land/run':<10} {'avg_t':<8} {'avg_xy':<10} {'failed_runs'}")
print("-" * 80)
for traj in TRAJS:
    for label, suf in CONDS:
        path = f"{ROOT}/{traj}_multi_init{suf}.mat"
        if not os.path.exists(path):
            print(f"{traj:<12} {label:<10} MISSING")
            continue
        m = sio.loadmat(path, squeeze_me=True, struct_as_record=False)
        results = m["results"]
        n = len(results)
        n_land = sum(int(r.success) for r in results)
        ts = [float(r.final_t) for r in results if r.success]
        xys = [float(r.final_xy) for r in results if r.success]
        failed = [str(i+1) for i, r in enumerate(results) if not r.success]
        avg_t = f"{np.mean(ts):.2f}s" if ts else "-"
        avg_xy = f"{np.mean(xys)*100:.1f}cm" if xys else "-"
        print(f"{traj:<12} {label:<10} {n_land}/{n:<8} {avg_t:<8} {avg_xy:<10} {','.join(failed) if failed else '-'}")
