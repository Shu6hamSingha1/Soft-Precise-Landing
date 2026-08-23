import glob, os
import numpy as np

BASE = os.path.expanduser("~/ws/scripts/precise_landing/Test_Data/Landing")
runs = sorted(glob.glob(os.path.join(BASE, "*")))

print(f"{'run':<28} {'n':>5} {'ratio(sdot/h_xy) median':>24} {'ratio p25-p75':>18} {'corr':>7}")
ratios_all = []
for run_dir in runs:
    name = os.path.basename(run_dir)
    cd_path = os.path.join(run_dir, "Control_Data.npy")
    if not os.path.isfile(cd_path):
        continue
    if os.path.getsize(cd_path) < 50_000:
        continue   # near-empty aborted runs
    try:
        d = np.load(cd_path, allow_pickle=True).item()
        h = np.asarray(d["h(t)"], float)
        sdot = np.asarray(d["s_dot_meas(t)"], float)
    except Exception as e:
        print(f"{name:<28} SKIP ({e})")
        continue
    n = min(len(h), len(sdot))
    h, sdot = h[:n], sdot[:n]
    h_xy = np.linalg.norm(h[:, :2], axis=1)
    sdot_mag = np.linalg.norm(sdot, axis=1)
    # only where there's real measured motion - avoid near-zero/near-zero ratio blowups
    m = (sdot_mag > 0.02) & np.isfinite(h_xy) & np.isfinite(sdot_mag)
    if m.sum() < 20:
        print(f"{name:<28} {n:>5}   too few active samples ({int(m.sum())})")
        continue
    ratio = sdot_mag[m] / np.maximum(h_xy[m], 1e-6)
    corr = np.corrcoef(h_xy[m], sdot_mag[m])[0, 1] if np.std(h_xy[m]) > 1e-9 else np.nan
    med = np.median(ratio)
    p25, p75 = np.percentile(ratio, [25, 75])
    ratios_all.extend(ratio.tolist())
    print(f"{name:<28} {n:>5} {med:>24.2f} {f'{p25:.2f}-{p75:.2f}':>18} {corr:>7.2f}")

if ratios_all:
    ratios_all = np.array(ratios_all)
    print(f"\n=== AGGREGATE across {len(ratios_all)} active samples, all flights ===")
    print(f"median ratio (true/measured, sdot_meas/h): {np.median(ratios_all):.2f}")
    print(f"p10={np.percentile(ratios_all,10):.2f}  p25={np.percentile(ratios_all,25):.2f}  "
          f"p75={np.percentile(ratios_all,75):.2f}  p90={np.percentile(ratios_all,90):.2f}")
