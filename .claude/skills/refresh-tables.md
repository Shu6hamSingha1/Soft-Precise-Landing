# /refresh-tables

Rebuild the manuscript result tables (multi-init Table V in main; comparison Table IV in main, label `comparison table: table`) from the latest `.mat` datasets after an MDF-ASMC gain relock or trajectory change. Assumes the user has already rerun the MATLAB sweeps.

## Usage

```
/refresh-tables                 -- refresh both multi-init and comparison
/refresh-tables multi_init      -- only multi-init Table V
/refresh-tables comparison      -- only comparison Table IV
```

## Landing criterion (current convention, 2026-04-20)

- **Termination** — first time step where altitude `-X_DS[2]` ≤ `zf = 0.20 m` (`Constants.m`). MATLAB writes one extra step past the crossing, so when reading the trajectory in Python use `Z_LAND_EPS = 0.205` if you want the post-cross step, or read at `pidx = idx - 1` to match MATLAB's `fprintf('Landed at t=…')` printout.
- **Precise** — horizontal error at termination ≤ `0.10 m` (paper threshold; MATLAB log uses 0.08, but the manuscript narrative uses 0.10).
- **Soft** — UAV-target relative speed at termination ≤ `0.20 m/s`.
- **There is no longer a separate "comparison tolerance"** — the 0.30 m loose threshold and the `\ddagger` marker were removed from Table IV in 2026-04-20. Only `\dagger` survives.

## Indexing convention (aligned 2026-04-22)

Both tex and `scripts/_analyze_results_for_tex.py` now use the same convention:
- `t_f = (idx - 1) * dt` (matches MATLAB's `Landed at t=…` printout)
- UAV state at termination: `X_DS[:, idx]` (Python; MATLAB stores `X_DS(:,idx+1)=x_c`)
- Target state at termination: `x_t[:, idx-1]` / `dx_t[:, idx-1]` (Python; MATLAB fills `x_t(:,k)` so MATLAB col idx = Python col idx-1)

**UAV uses col `idx`; target uses col `idx-1`.** Paste aggregator cells directly into tex — no manual alignment needed. Earlier drift (t_f +0.01 s; spurious xy_e in the 10s-of-metres on some runs) came from the aggregator reading `x_t[:, idx]`, which is uninitialized preallocation garbage. Do NOT reintroduce that pattern.

## Trajectory-to-Case mapping (Section IV-A in main)

- Case 1 = Static
- Case 2 = Linear (ship deck)
- Case 3 = Sinusoidal
- **Case 4 = Lissajous**
- **Case 5 = Circular (ship deck)**

Table IV column headers MUST follow this. (Cases 4/5 swap in the live tex was resolved 2026-04-20 — see `project_table4_case_swap.md`.)

## Multi-init Table V

Source: `MATLAB/Multi_init_cond/Datasets/{Static,Linear,Sinusoidal,Lissajous,Circular}_multi_init.mat`
(single key `results` is a struct array of 5 runs).

For each trajectory, aggregate only over `success == 1`:

- Lands = `#success / 5`
- `mean_t`, `max_t` = mean/max of `final_t`
- `mean_xy`, `max_xy` = mean/max of `final_xy` (convert to cm)

Snippet:

```python
import scipy.io as sio, numpy as np
trajs = ['Static','Linear','Sinusoidal','Lissajous','Circular']
for tr in trajs:
    m = sio.loadmat(f'MATLAB/Multi_init_cond/Datasets/{tr}_multi_init.mat',
                    squeeze_me=True, struct_as_record=False)
    res = np.atleast_1d(m['results'])
    ok = [r for r in res if getattr(r, 'success', False)]
    tf = np.array([float(r.final_t)  for r in ok])
    xy = np.array([float(r.final_xy) for r in ok]) * 100  # cm
    print(f'{tr:<12} {len(ok)}/5  mean_t={tf.mean():.2f}  max_t={tf.max():.2f}  '
          f'mean_xy={xy.mean():.2f}  max_xy={xy.max():.2f}')
```

Reference: `scripts/_analyze_results_for_tex.py` already produces exactly these numbers and writes `scripts/_current_numbers.json`.

Update:
1. The five data rows of `multi init: table` in `Soft_Precise_Landing/results.tex` (lines ~92-96).
2. The surrounding discussion paragraph — refresh the "mean landing time X-Y s" and "worst-case xy below Z cm" numbers.
3. The corresponding conclusion bullet in Section S3-G of `supplemental.tex`.
4. The worst-case `|u|`/`|v|` line in `results.tex` (~line 69) — pull from `scripts/_current_numbers.json` (`global_phys_worst_u`, `global_phys_worst_v`).

## Comparison Table IV

Source: `MATLAB/Comparison/Datasets/{Static,Linear,Sinusoidal,Lissajous,Circular}_comparison.mat`
(per-trajectory file holds five controllers in `result_ctrl_{1..5}` keys; or a single per-controller file depending on harness version).

For each `(traj, ctrl)`:

```python
d = result.data         # controller's struct from the .mat
N = int(d.idx) if int(d.idx) > 0 else d.tRange.shape[0]
t   = d.tRange[:N]
X   = d.X_DS[:, :N]
alt = -X[2, :]
tgt = d.x_t[:3, :N]
xy  = np.linalg.norm(X[:2] - tgt[:2], axis=0)

below = np.where(alt <= 0.205)[0]
if len(below):
    midx = below[0]
    landed = True
else:
    midx = N - 1
    landed = False

# Choose ONE indexing convention — default matches the tex (MATLAB log line):
pidx = max(0, midx - 1)
tf, xy_f, zf = t[pidx], xy[pidx], alt[pidx]
```

Reference: `scripts/_comparison_numbers.json` carries the latest values per `(traj, ctrl)` under the `idx` (post-cross) convention. To get tex-aligned values (`idx - 1`), re-extract or apply the empirical 0.01-s offset.

Classification markers in the LaTeX table:
- MDF-ASMC (Proposed): bold, no marker.
- `$\dagger$` on `tf` whenever the simulation terminates without reaching `zf = 0.20 m` — this covers BOTH the visibility-break case (sim broke early, `tf < 40`) AND the `tf = 39.99` time-limit case.
- **No `\ddagger`** — the loose-tolerance marker was removed.

Update both the data rows and the surrounding enumerate items in Section IV-E (any numbers cited in the prose: per-baseline failure ranges in lines ~193-194 and the matching Section S3-E paragraphs in `supplemental.tex`).

## Sync check (pre-flight)

Before running, confirm the gain files agree (variable rename `zp/zi/zd → rp/ri/rd` is already global as of 2026-04-19):
- `MATLAB/Multi_init_cond/run_simulation.m`
- `MATLAB/Multi_init_cond/visualControl_IBVS_adaptive.m`
- `MATLAB/Comparison/InitGains_Comparison.m`

Also confirm `Common/Constants.m` `FILTER_WINDOW = 11` and `zf = 0.20`. If any of these diverge, abort and flag — the dataset is stale relative to at least one source of truth.

## After edits

Report: per-cell delta vs the previous committed numbers, and whether any classification flipped (landed ↔ not-landed, precise ↔ not-precise, soft ↔ not-soft). Do NOT commit — the user commits after eyeballing the diff.
