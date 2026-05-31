"""One-shot analysis of current .mat files to cross-check results.tex + supplemental.tex.

Reads:
  - MATLAB/Multi_init_cond/Datasets/<Traj>_multi_init.mat       (5x5 robustness)
  - MATLAB/Multi_init_cond/Datasets/<Traj>_multi_init_noiseless.mat
  - MATLAB/Multi_init_cond/Datasets/<Traj>_multi_speed.mat      (4x5 speed sweep)
  - MATLAB/Comparison/Datasets/result_ctrl_{1..5}.mat           (5-ctrl comparison)

Prints a full report and writes scripts/_current_numbers.json.
"""
import json
from pathlib import Path
import numpy as np
import scipy.io as sio

ROOT = Path(__file__).resolve().parent.parent
MI_DIR = ROOT / 'MATLAB' / 'Multi_init_cond' / 'Datasets'
CMP_DIR = ROOT / 'MATLAB' / 'Comparison' / 'Datasets'

TRAJS = ['Static', 'Linear', 'Sinusoidal', 'Lissajous', 'Circular']
MOVING = ['Linear', 'Sinusoidal', 'Lissajous', 'Circular']


def load(p):
    return sio.loadmat(str(p), squeeze_me=True, struct_as_record=False)


def per_run_metrics(r):
    """Extract authoritative MATLAB-side metrics + per-run time-series summaries."""
    dat = r.data
    idx = int(dat.idx)
    dt = float(dat.dt)
    X = dat.X_DS
    pos = X[0:3, :idx + 1]
    vel = X[7:10, :idx + 1]

    # Pixel trajectories from P_DS
    P_DS = dat.P_DS  # (2, 12, N+1) MATLAB (2, 12, N+1) — columns 1-4 virtual, 9-12 physical
    # After loadmat with squeeze_me=True, shape is (2,12,N+1); trim to idx+1
    if P_DS.ndim == 3:
        P = P_DS[:, :, :idx + 1]
    else:
        P = P_DS

    # Columns: MATLAB 1-4 = virtual, 9-12 = physical → Python 0-3 and 8-11
    virt = P[:, 0:4, :]          # (2, 4, N+1)
    phys = P[:, 8:12, :]         # (2, 4, N+1)
    # Worst-corner worst-time absolute excursion
    virt_u_max = float(np.nanmax(np.abs(virt[0, :, :])))
    virt_v_max = float(np.nanmax(np.abs(virt[1, :, :])))
    phys_u_max = float(np.nanmax(np.abs(phys[0, :, :])))
    phys_v_max = float(np.nanmax(np.abs(phys[1, :, :])))

    final_alt = float(r.final_alt)
    final_xy = float(r.final_xy)
    final_vel = float(r.final_rel_vel)
    success = bool(r.success)
    precise = bool(getattr(r, 'precise', False)) if hasattr(r, 'precise') else None
    soft = bool(getattr(r, 'soft', False)) if hasattr(r, 'soft') else None
    fov_fail = bool(getattr(r, 'fov_fail', False)) if hasattr(r, 'fov_fail') else None

    return dict(
        t_end=idx * dt,
        success=success,
        precise=precise,
        soft=soft,
        fov_fail=fov_fail,
        final_alt=final_alt,
        final_xy=final_xy,
        final_vel=final_vel,
        virt_u_max=virt_u_max,
        virt_v_max=virt_v_max,
        phys_u_max=phys_u_max,
        phys_v_max=phys_v_max,
    )


def multi_init_report():
    print('\n=============================================================')
    print(' 5x5 MULTI-INIT ROBUSTNESS (under full disturbance model)')
    print('=============================================================')
    summary = {}
    per_case_px = {}
    for traj in TRAJS:
        d = load(MI_DIR / f'{traj}_multi_init.mat')
        results = d['results']
        runs = [per_run_metrics(r) for r in results]
        lands = sum(1 for m in runs if m['success'])
        precise = sum(1 for m in runs if m.get('precise'))
        soft = sum(1 for m in runs if m.get('soft'))
        fov = sum(1 for m in runs if m.get('fov_fail'))
        tf = [m['t_end'] for m in runs if m['success']]
        xy = [m['final_xy'] for m in runs if m['success']]
        v = [m['final_vel'] for m in runs if m['success']]
        vu = max(m['virt_u_max'] for m in runs)
        vv = max(m['virt_v_max'] for m in runs)
        pu = max(m['phys_u_max'] for m in runs)
        pv = max(m['phys_v_max'] for m in runs)
        print(f'\n{traj:12s} lands={lands}/5 precise={precise}/5 soft={soft}/5 fov_fail={fov}/5')
        print(f'  t_f   : mean={np.mean(tf):.3f}s  max={np.max(tf):.3f}s')
        print(f'  xy_e  : mean={100*np.mean(xy):.3f}cm max={100*np.max(xy):.3f}cm')
        print(f'  |v_f| : mean={np.mean(v):.4f}m/s  max={np.max(v):.4f}m/s')
        print(f'  virt-px  max |u|={vu:.2f}  max |v|={vv:.2f}')
        print(f'  phys-px  max |u|={pu:.2f}  max |v|={pv:.2f}')
        # Worst IC for physical-pixel excursion
        pu_by_ic = [m['phys_u_max'] for m in runs]
        pv_by_ic = [m['phys_v_max'] for m in runs]
        worst_ic_u = int(np.argmax(pu_by_ic)) + 1
        worst_ic_v = int(np.argmax(pv_by_ic)) + 1
        print(f'  phys-px  worst-IC u={worst_ic_u} ({pu_by_ic[worst_ic_u-1]:.2f}px)  v={worst_ic_v} ({pv_by_ic[worst_ic_v-1]:.2f}px)')
        summary[traj] = dict(
            lands=lands, precise=precise, soft=soft, fov_fail=fov,
            t_f_mean=float(np.mean(tf)), t_f_max=float(np.max(tf)),
            xy_e_mean_cm=float(100 * np.mean(xy)), xy_e_max_cm=float(100 * np.max(xy)),
            v_f_mean=float(np.mean(v)), v_f_max=float(np.max(v)),
            phys_u_max=pu, phys_v_max=pv,
            phys_u_max_ic=worst_ic_u, phys_v_max_ic=worst_ic_v,
            virt_u_max=vu, virt_v_max=vv,
        )
        per_case_px[traj] = runs
    return summary, per_case_px


def multi_init_noiseless_report():
    print('\n=============================================================')
    print(' 5x5 MULTI-INIT IDEALISED (noiseless)')
    print('=============================================================')
    summary = {}
    for traj in TRAJS:
        p = MI_DIR / f'{traj}_multi_init_noiseless.mat'
        if not p.exists():
            print(f'{traj:12s} -- no noiseless dataset')
            continue
        d = load(p)
        results = d['results']
        runs = [per_run_metrics(r) for r in results]
        lands = sum(1 for m in runs if m['success'])
        precise = sum(1 for m in runs if m.get('precise'))
        soft = sum(1 for m in runs if m.get('soft'))
        fov = sum(1 for m in runs if m.get('fov_fail'))
        print(f'{traj:12s} lands={lands}/5 precise={precise}/5 soft={soft}/5 fov_fail={fov}/5')
        summary[traj] = dict(lands=lands, precise=precise, soft=soft, fov_fail=fov)
    return summary


def multi_speed_report():
    print('\n=============================================================')
    print(' SPEED SWEEP (IC1=[0,0,-5], lambda in {0.6,0.8,1.0,1.2,1.4})')
    print('=============================================================')
    summary = {}
    all_xy, all_v, all_t = [], [], []
    for traj in MOVING:
        p = MI_DIR / f'{traj}_multi_speed.mat'
        if not p.exists():
            print(f'{traj:12s} -- no speed dataset')
            continue
        d = load(p)
        results = d['results']
        runs = [per_run_metrics(r) for r in results]
        lams = d.get('speedScale', d.get('speeds', None))
        if lams is not None:
            lams_arr = np.atleast_1d(lams)
        else:
            lams_arr = np.array([0.6, 0.8, 1.0, 1.2, 1.4])
        lands = sum(1 for m in runs if m['success'])
        precise = sum(1 for m in runs if m.get('precise'))
        soft = sum(1 for m in runs if m.get('soft'))
        fov = sum(1 for m in runs if m.get('fov_fail'))
        print(f'\n{traj:12s} lands={lands}/{len(runs)} precise={precise}/{len(runs)} '
              f'soft={soft}/{len(runs)} fov_fail={fov}/{len(runs)}')
        for i, m in enumerate(runs):
            lam = float(lams_arr[i]) if i < len(lams_arr) else None
            flag = 'OK' if (m['success'] and m.get('precise') and m.get('soft')) else 'FAIL'
            print(f'   lam={lam:.2f}  t_f={m["t_end"]:6.2f}s  xy_e={100*m["final_xy"]:6.2f}cm  '
                  f'|v_f|={m["final_vel"]:6.3f}m/s  phys(|u|,|v|)=({m["phys_u_max"]:.1f},{m["phys_v_max"]:.1f})  [{flag}]')
            if m['success']:
                all_xy.append((traj, lam, m['final_xy']))
                all_v.append((traj, lam, m['final_vel']))
                all_t.append((traj, lam, m['t_end']))
        summary[traj] = dict(lands=lands, precise=precise, soft=soft, fov_fail=fov)
    # Worst-case overall
    if all_xy:
        worst_xy = max(all_xy, key=lambda x: x[2])
        worst_v = max(all_v, key=lambda x: x[2])
        print(f'\n  WORST xy_e : {100*worst_xy[2]:.3f}cm  on {worst_xy[0]}  lam={worst_xy[1]:.2f}')
        print(f'  WORST |v_f|: {worst_v[2]:.4f}m/s on {worst_v[0]}  lam={worst_v[1]:.2f}')
        summary['worst_xy'] = dict(traj=worst_xy[0], lam=worst_xy[1], xy_m=float(worst_xy[2]))
        summary['worst_v'] = dict(traj=worst_v[0], lam=worst_v[1], v=float(worst_v[2]))
    return summary


def comparison_report():
    """Extract Table V from MATLAB/Comparison/Datasets/<Traj>_comparison.mat.

    Each aggregate .mat contains:
      ctrl_list   : array of controller IDs
      ctrl_names  : array of controller names
      all_results : struct array of length 5, each with fields ctrl_id, ctrl_name, data
    For each controller x trajectory, Table V reports t_f, xy_e, z_f, |v_rel|.
    """
    print('\n=============================================================')
    print(' 5-CONTROLLER COMPARISON (IC2=[2,2,-5], shared disturbance)')
    print('=============================================================')
    summary = {}
    ctrl_order = ['PLASMC (Proposed)', 'Lin 2022', 'Zhang 2026', 'Lin 2023', 'Cho 2022']
    grid = {name: {} for name in ctrl_order}

    for traj in TRAJS:
        p = CMP_DIR / f'{traj}_comparison.mat'
        if not p.exists():
            print(f'{traj:12s} -- aggregate .mat missing')
            continue
        d = load(p)
        ar = np.atleast_1d(d['all_results'])
        for entry in ar:
            dat = entry.data
            idx = int(dat.idx)
            dt = float(dat.dt)
            X = dat.X_DS[:, :idx + 1]
            x_t = dat.x_t[:, :idx + 1]
            dx_t = dat.dx_t[:3, :idx + 1]
            t_f = (idx - 1) * dt  # match MATLAB-log convention used in Table V
            # MATLAB harness: X_DS(:,idx+1)=x_c (post-step) so Python X[:,idx]
            # is correct. But x_t/dx_t are filled as x_t(:,k) so MATLAB col idx
            # = Python col idx-1 — the harness reads x_t(1:2, idx), which is
            # x_t[:, idx-1] in Python.
            xy_e = float(np.sqrt((X[0, idx] - x_t[0, idx - 1]) ** 2 + (X[1, idx] - x_t[1, idx - 1]) ** 2))
            z_f = float(abs(X[2, idx] - x_t[2, idx - 1]))  # above-target gap (NED)
            v_rel = float(np.linalg.norm(X[7:10, idx] - dx_t[:3, idx - 1]))
            name = str(entry.ctrl_name).strip()
            grid[name][traj] = dict(t_f=t_f, xy_e=xy_e, z_f=z_f, v_rel=v_rel)

    print(f'\n{"Controller":<22}', end='')
    for traj in TRAJS:
        print(f' {traj[:10]:>24}', end='')
    print()
    print(f'{"":<22}', end='')
    for traj in TRAJS:
        print(f'   t_f   xy_e    z_f   |v|', end='')
    print()
    for name in ctrl_order:
        row = grid.get(name, {})
        print(f'{name:<22}', end='')
        for traj in TRAJS:
            r = row.get(traj)
            if r is None:
                print(' {:>24}'.format('(missing)'), end='')
            else:
                print(f'  {r["t_f"]:5.2f}  {r["xy_e"]:5.3f}  {r["z_f"]:5.3f}  {r["v_rel"]:4.2f}', end='')
        print()
        summary[name] = row
    return summary


def main():
    out = {}
    mi_dist, per_case_px = multi_init_report()
    mi_ideal = multi_init_noiseless_report()
    sweep = multi_speed_report()
    cmp_ = comparison_report()
    out['multi_init_disturbed'] = mi_dist
    out['multi_init_ideal'] = mi_ideal
    out['speed_sweep'] = sweep
    out['comparison'] = cmp_

    # Also aggregate global physical-pixel worst-case (multi-init, disturbed)
    all_pu, all_pv = [], []
    for traj, runs in per_case_px.items():
        for ic, m in enumerate(runs):
            all_pu.append((traj, ic + 1, m['phys_u_max']))
            all_pv.append((traj, ic + 1, m['phys_v_max']))
    worst_u = max(all_pu, key=lambda x: x[2])
    worst_v = max(all_pv, key=lambda x: x[2])
    print('\n=============================================================')
    print(f'GLOBAL worst phys-pixel |u|: {worst_u[2]:.2f}px on {worst_u[0]} IC{worst_u[1]}')
    print(f'GLOBAL worst phys-pixel |v|: {worst_v[2]:.2f}px on {worst_v[0]} IC{worst_v[1]}')
    out['global_phys_worst_u'] = dict(traj=worst_u[0], ic=worst_u[1], px=worst_u[2])
    out['global_phys_worst_v'] = dict(traj=worst_v[0], ic=worst_v[1], px=worst_v[2])

    (ROOT / 'scripts' / '_current_numbers.json').write_text(json.dumps(out, indent=2, default=str))
    print('\nWrote scripts/_current_numbers.json')


if __name__ == '__main__':
    main()
