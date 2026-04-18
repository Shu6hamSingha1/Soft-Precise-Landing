"""
Analyze multi-initial-condition simulation results from multi_Init_Var.m.

Reads Datasets/<trajType>_multi_init.mat (saved by multi_Init_Var.m).
Each .mat contains a struct array `results` with fields:
    success      — bool, did UAV land within thresholds
    final_error  — 3D distance to target at termination
    data         — full simulation workspace (X_DS, U_DS, x_t, idx, dt, etc.)
plus `p0` (initial positions Nx3), `numRuns`, `trajType`.

USAGE:
    python analyze_multi_init.py                 # default: Static
    python analyze_multi_init.py Linear          # any trajectory name
    python analyze_multi_init.py Static --plot   # add matplotlib plots
    python analyze_multi_init.py Static 3        # detailed view of run 3
    python analyze_multi_init.py Static 3 --plot # detailed + plots

Run from MATLAB/Multi_init_cond directory.
"""
import sys
import os
from pathlib import Path
import numpy as np
import scipy.io as sio

DATA_DIR = Path(__file__).resolve().parent.parent / 'MATLAB' / 'Multi_init_cond' / 'Datasets'


# Landing thresholds — must match run_simulation.m termination check
LAND_ALT = 0.20    # m  (altitude above target)
LAND_XY  = 0.30    # m  (horizontal distance)


def quat_to_yaw(q):
    """Quaternion [w,x,y,z] -> yaw angle (rad)."""
    w, x, y, z = q[0], q[1], q[2], q[3]
    return np.arctan2(2*(w*z + x*y), 1 - 2*(y**2 + z**2))


def extract_run(r):
    """Extract metrics from a single run's data struct."""
    dat = r.data
    idx = int(dat.idx)
    dt  = float(dat.dt)
    X   = dat.X_DS                  # 13 x (N+1)
    xt  = dat.x_t.astype(float)     # 7 x N

    # Trim to valid samples [0..idx]
    # MATLAB writes X_DS(:,idx+1) = x_c (Python: X[:, idx]); on break
    # branches that column can be unwritten, so trim to the last populated col.
    pos_all = X[0:3, :idx+1]
    last_nz = idx
    while last_nz > 0 and np.abs(pos_all[:, last_nz]).sum() == 0:
        last_nz -= 1
    end_slice = slice(0, last_nz + 1)
    pos = X[0:3, end_slice]
    vel = X[7:10, end_slice]
    quat = X[3:7, end_slice]
    tgt = xt[0:3, end_slice]

    # If loop broke at last_nz, target column may be zero — use previous
    if last_nz > 0 and np.linalg.norm(tgt[:, -1]) == 0 and np.linalg.norm(tgt[:, -2]) > 0:
        tgt[:, -1] = tgt[:, -2]

    # Relative metrics
    rel = pos - tgt
    alt_err = np.abs(rel[2, :])
    xy_err  = np.linalg.norm(rel[0:2, :], axis=0)
    d3_err  = np.linalg.norm(rel, axis=0)

    # Trust MATLAB-computed final metrics (authoritative; no indexing ambiguity)
    final_alt = float(r.final_alt)
    final_xy  = float(r.final_xy)
    final_3d  = float(np.sqrt(final_alt**2 + final_xy**2))
    final_vel = float(r.final_rel_vel)
    final_vz  = vel[2, -1]

    # MATLAB-authoritative landing flag
    landed = bool(r.success)

    # Trajectory metrics
    altitudes = -pos[2, :]    # NED -> positive altitude
    max_alt   = altitudes.max()
    init_alt  = altitudes[0]
    max_climb = max(0.0, max_alt - init_alt)   # overshoot above start

    # Yaw drift
    yaw0 = quat_to_yaw(quat[:, 0])
    yawf = quat_to_yaw(quat[:, -1])
    yaw_drift_deg = np.degrees(np.unwrap([yaw0, yawf])[1] - yaw0)

    return {
        'idx':        idx,
        't_end':      idx * dt,
        'success':    bool(r.success),
        'landed':     landed,
        'final_alt':  final_alt,
        'final_xy':   final_xy,
        'final_3d':   final_3d,
        'final_vel':  final_vel,
        'final_vz':   final_vz,
        'init_pos':   pos[:, 0].tolist(),
        'final_pos':  pos[:, -1].tolist(),
        'final_tgt':  tgt[:, -1].tolist(),
        'max_alt':    max_alt,
        'init_alt':   init_alt,
        'max_climb':  max_climb,
        'yaw_drift':  yaw_drift_deg,
        'pos':        pos,
        'vel':        vel,
        'tgt':        tgt,
        'd3_err':     d3_err,
        'xy_err':     xy_err,
        'alt_err':    alt_err,
        'altitudes':  altitudes,
        'dt':         dt,
    }


def load_dataset(trajType):
    path = str(DATA_DIR / f'{trajType}_multi_init.mat')
    if not os.path.exists(path):
        raise FileNotFoundError(f'Missing dataset: {path}')
    d = sio.loadmat(path, squeeze_me=True, struct_as_record=False)
    results = d['results']
    p0      = d['p0']
    numRuns = int(d['numRuns'])
    return results, p0, numRuns


def print_summary(trajType, runs, p0):
    print(f'\n=== Multi-Init Analysis: {trajType} target ===')
    print(f'{"Run":<4}{"IC [x,y,z]":<22}{"landed":<10}'
          f'{"t_end":<10}{"final_alt":<12}{"final_xy":<12}'
          f'{"|v_f|":<10}{"v_z":<10}{"climb":<10}{"yaw_drift":<10}')
    print('-' * 110)
    for i, m in enumerate(runs):
        ic = p0[i].tolist()
        tag = 'YES' if m['landed'] else ('--' if m['success'] else 'NO')
        print(f"{i+1:<4}{str(ic):<22}{tag:<10}"
              f"{m['t_end']:<10.2f}{m['final_alt']:<12.4f}{m['final_xy']:<12.4f}"
              f"{m['final_vel']:<10.3f}{m['final_vz']:<10.3f}"
              f"{m['max_climb']:<10.3f}{m['yaw_drift']:<10.2f}")

    landed_count = sum(1 for m in runs if m['landed'])
    print('-' * 110)
    print(f'Landed: {landed_count}/{len(runs)}')
    if landed_count > 0:
        landed_runs = [m for m in runs if m['landed']]
        print(f'  Mean t_landing : {np.mean([m["t_end"] for m in landed_runs]):.2f} s')
        print(f'  Mean final_xy  : {np.mean([m["final_xy"] for m in landed_runs]):.3f} m')
        print(f'  Mean final_alt : {np.mean([m["final_alt"] for m in landed_runs]):.3f} m')
        print(f'  Mean |v_f|     : {np.mean([m["final_vel"] for m in landed_runs]):.3f} m/s')
        print(f'  Max climb      : {max(m["max_climb"] for m in landed_runs):.3f} m')


def print_detail(trajType, idx, runs, p0):
    m = runs[idx]
    print(f'\n=== Detailed view: {trajType} run {idx+1} ===')
    print(f'  IC          : {p0[idx].tolist()}')
    print(f'  Final pos   : {[round(x,4) for x in m["final_pos"]]}')
    print(f'  Final target: {[round(x,4) for x in m["final_tgt"]]}')
    print(f'  Landed      : {m["landed"]}  (success flag={m["success"]})')
    print(f'  Steps       : {m["idx"]}  (t_end = {m["t_end"]:.2f} s)')
    print(f'  Final alt   : {m["final_alt"]:.4f} m  (limit {LAND_ALT})')
    print(f'  Final xy    : {m["final_xy"]:.4f} m  (limit {LAND_XY})')
    print(f'  Final |v|   : {m["final_vel"]:.4f} m/s   (v_z = {m["final_vz"]:.4f})')
    print(f'  Init alt    : {m["init_alt"]:.3f} m')
    print(f'  Max alt     : {m["max_alt"]:.3f} m')
    print(f'  Climb above start: {m["max_climb"]:.3f} m')
    print(f'  Yaw drift   : {m["yaw_drift"]:.2f} deg')


def make_plots(trajType, runs, p0, focus=None):
    import matplotlib.pyplot as plt

    if focus is None:
        # Overview: 3D paths + altitude vs time + xy error vs time
        fig = plt.figure(figsize=(15, 5))
        ax1 = fig.add_subplot(1, 3, 1, projection='3d')
        for i, m in enumerate(runs):
            p = m['pos']
            ax1.plot(p[0], p[1], -p[2], label=f'Run {i+1}')
            ax1.scatter(p[0, 0], p[1, 0], -p[2, 0], marker='o')
            ax1.scatter(p[0, -1], p[1, -1], -p[2, -1], marker='x')
        ax1.set_xlabel('x [m]'); ax1.set_ylabel('y [m]'); ax1.set_zlabel('alt [m]')
        ax1.set_title(f'{trajType}: 3D paths')
        ax1.legend(loc='best', fontsize=8)

        ax2 = fig.add_subplot(1, 3, 2)
        for i, m in enumerate(runs):
            t = np.arange(m['altitudes'].size) * m['dt']
            ax2.plot(t, m['altitudes'], label=f'Run {i+1}')
        ax2.set_xlabel('t [s]'); ax2.set_ylabel('altitude [m]')
        ax2.set_title('Altitude vs time'); ax2.grid(True); ax2.legend(fontsize=8)

        ax3 = fig.add_subplot(1, 3, 3)
        for i, m in enumerate(runs):
            t = np.arange(m['xy_err'].size) * m['dt']
            ax3.plot(t, m['xy_err'], label=f'Run {i+1}')
        ax3.axhline(LAND_XY, color='k', ls='--', alpha=0.5, label='xy limit')
        ax3.set_xlabel('t [s]'); ax3.set_ylabel('xy error [m]')
        ax3.set_title('XY error vs time'); ax3.grid(True); ax3.legend(fontsize=8)

        plt.tight_layout()
    else:
        m = runs[focus]
        fig, axes = plt.subplots(2, 2, figsize=(12, 8))
        t = np.arange(m['altitudes'].size) * m['dt']
        axes[0, 0].plot(t, m['altitudes']); axes[0, 0].set_title('Altitude')
        axes[0, 0].set_xlabel('t [s]'); axes[0, 0].set_ylabel('alt [m]'); axes[0, 0].grid(True)
        axes[0, 1].plot(t, m['xy_err'], label='xy err')
        axes[0, 1].plot(t, m['alt_err'], label='alt err')
        axes[0, 1].axhline(LAND_XY, color='r', ls='--', alpha=0.5)
        axes[0, 1].axhline(LAND_ALT, color='g', ls='--', alpha=0.5)
        axes[0, 1].set_title('Tracking errors'); axes[0, 1].grid(True); axes[0, 1].legend()
        v = m['vel']
        axes[1, 0].plot(t, v[0], label='v_x'); axes[1, 0].plot(t, v[1], label='v_y')
        axes[1, 0].plot(t, v[2], label='v_z')
        axes[1, 0].set_title('Velocity'); axes[1, 0].grid(True); axes[1, 0].legend()
        axes[1, 1].plot(m['pos'][0], m['pos'][1], label='UAV')
        axes[1, 1].plot(m['tgt'][0], m['tgt'][1], 'r--', label='target')
        axes[1, 1].set_title('XY top view'); axes[1, 1].grid(True); axes[1, 1].legend()
        axes[1, 1].set_aspect('equal')
        fig.suptitle(f'{trajType} run {focus+1}: IC={p0[focus].tolist()}')
        plt.tight_layout()

    plt.show()


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    do_plot = '--plot' in sys.argv

    trajType = args[0] if len(args) >= 1 else 'Static'
    focus    = int(args[1]) - 1 if len(args) >= 2 else None

    results, p0, numRuns = load_dataset(trajType)
    runs = [extract_run(r) for r in results]

    print_summary(trajType, runs, p0)
    if focus is not None:
        print_detail(trajType, focus, runs, p0)

    if do_plot:
        make_plots(trajType, runs, p0, focus)


if __name__ == '__main__':
    main()
