"""
Analyze comparison simulation results (result_ctrl_{1-5}.mat).
Run from MATLAB/Comparison directory.

Usage:
  python analyze_results.py           # summary of all 5 controllers
  python analyze_results.py 4         # detailed analysis of controller 4
  python analyze_results.py 4 --plot  # detailed + matplotlib plots

State vector X_DS (13 x N): [pos(3); quat(4); vel(3); omega(3)]  — NED frame
Target x_t (7 x N): position + velocity of target
Controls U_DS (4 x N): [tau_x; tau_y; tau_z; T]
idx: final simulation index (< N_MAX means landed/crashed)
"""
import sys
import os
from pathlib import Path
import scipy.io as sio
import numpy as np

DATA_DIR = Path(__file__).resolve().parent.parent / 'MATLAB' / 'Comparison' / 'Datasets'

NAMES = ['DF-ASMC', 'Lin2022', 'Zhang2026', 'Chen2025', 'Cho2022']
DT = 0.01
N_MAX = 4000
LAND_THRESHOLD = 0.5  # m — alt<=0.4 && xy<=0.5 in MATLAB; use 3D proxy


def load_ctrl(ctrl_id):
    """Load a single controller result and extract common fields."""
    d = sio.loadmat(str(DATA_DIR / f'result_ctrl_{ctrl_id}.mat'), squeeze_me=True)
    X  = d['X_DS']
    xt = d['x_t'].astype(float)
    U  = d['U_DS']
    idx = int(d['idx'])
    tR  = d['tRange']

    pos   = X[0:3,   :idx+1]
    quat  = X[3:7,   :idx+1]
    vel   = X[7:10,  :idx+1]
    omega = X[10:13, :idx+1]

    tgt_pos = xt[0:3, :idx+1]
    tgt_quat = xt[3:7, :idx+1]
    # If loop broke at idx, x_t(:,idx) may be uninitialized (zeros).
    # Use idx-1 target position for the final step if target looks zero.
    if idx > 0 and np.linalg.norm(tgt_pos[:, -1]) == 0 and np.linalg.norm(tgt_pos[:, -2]) > 0:
        tgt_pos[:, -1] = tgt_pos[:, -2]
        tgt_quat[:, -1] = tgt_quat[:, -2]
    rel_pos = pos - tgt_pos

    # Quaternion -> Euler (UAV)
    qw, qx, qy, qz_ = quat[0], quat[1], quat[2], quat[3]
    roll  = np.arctan2(2*(qw*qx + qy*qz_), 1 - 2*(qx**2 + qy**2))
    pitch = np.arcsin(np.clip(2*(qw*qy - qz_*qx), -1, 1))
    yaw   = np.arctan2(2*(qw*qz_ + qx*qy), 1 - 2*(qy**2 + qz_**2))

    # Target yaw
    tqw, tqx, tqy, tqz = tgt_quat[0], tgt_quat[1], tgt_quat[2], tgt_quat[3]
    tgt_yaw = np.arctan2(2*(tqw*tqz + tqx*tqy), 1 - 2*(tqy**2 + tqz**2))

    thrust  = U[3, :idx+1]
    torques = U[0:3, :idx+1]
    t = tR[:idx+1]

    # Determine termination type
    final_rel = rel_pos[:, -1]
    final_alt = float(abs(final_rel[2]))
    final_xy  = float(np.linalg.norm(final_rel[:2]))
    final_dist = float(np.linalg.norm(final_rel))
    if idx >= N_MAX:
        status = 'TIMEOUT'
    elif final_alt <= 0.35 and final_xy <= 0.4:
        # Match MATLAB: alt<=0.4 && xy<=0.5, with margin for off-by-one
        status = 'LANDED'
    else:
        # Broke early but far from target: safety break / crash
        status = 'CRASHED'

    return dict(
        pos=pos, vel=vel, omega=omega, roll=roll, pitch=pitch, yaw=yaw,
        tgt_yaw=tgt_yaw,
        thrust=thrust, torques=torques, t=t, idx=idx, tR=tR,
        rel_pos=rel_pos, tgt_pos=tgt_pos, raw=d, status=status,
    )


def print_summary(ctrl_id, c):
    """One-block summary."""
    rel = c['rel_pos']
    vel = c['vel']
    alt = -rel[2]
    vel_z = vel[2]
    xy_err = np.linalg.norm(rel[0:2], axis=0)
    t = c['t']
    T = c['thrust']
    tau = c['torques']
    roll = c['roll']
    pitch = c['pitch']

    final_rel = rel[:, -1]
    final_v   = vel[:, -1]
    t_land = float(t[-1])

    # Vz oscillations
    signs = np.sign(vel_z[vel_z != 0])
    vz_osc = int(np.sum(np.diff(signs) != 0))
    n30 = max(1, int(0.3 * len(vel_z)))
    late_signs = np.sign(vel_z[-n30:][vel_z[-n30:] != 0])
    late_osc = int(np.sum(np.diff(late_signs) != 0))

    def time_to_alt(threshold):
        below = alt <= threshold
        return float(t[np.argmax(below)]) if np.any(below) else float('nan')

    ctrl_energy = float(np.sum(T**2) * DT)

    print(f'=== {NAMES[ctrl_id-1]} (ctrl {ctrl_id}) ===')
    print(f'  Status: {c["status"]}  |  Duration: {t_land:.2f}s')
    print(f'  Final rel pos: [{final_rel[0]:.4f}, {final_rel[1]:.4f}, {final_rel[2]:.4f}] m  (norm={np.linalg.norm(final_rel):.4f})')
    print(f'  Final velocity: [{final_v[0]:.4f}, {final_v[1]:.4f}, {final_v[2]:.4f}] m/s  (|v|={np.linalg.norm(final_v):.4f})')
    print(f'  Final alt above tgt: {alt[-1]:.4f} m  |  Final XY err: {np.linalg.norm(final_rel[0:2]):.4f} m')
    yaw_d = c['yaw'] * 180 / np.pi
    tgt_yaw_d = c['tgt_yaw'] * 180 / np.pi
    yaw_err = yaw_d - tgt_yaw_d
    print(f'  Max |roll|: {np.max(np.abs(roll))*180/np.pi:.2f} deg  |  Max |pitch|: {np.max(np.abs(pitch))*180/np.pi:.2f} deg')
    print(f'  Yaw: final={yaw_d[-1]:.1f} deg  |  Tgt yaw: {tgt_yaw_d[-1]:.1f} deg  |  Yaw err: mean={np.mean(yaw_err):.2f}, max|err|={np.max(np.abs(yaw_err)):.2f} deg')
    print(f'  Thrust: mean={np.mean(T):.2f}, max={np.max(T):.2f}, min={np.min(T):.2f} N')
    print(f'  Max |tau_xy|: {np.max(np.abs(tau[0:2,:])):.4f} Nm  |  Max |tau_z|: {np.max(np.abs(tau[2,:])):.4f} Nm')
    print(f'  Max speed: {np.max(np.linalg.norm(vel, axis=0)):.3f} m/s  |  Max Vz_down: {np.max(vel_z):.3f} m/s')
    print(f'  Max XY err: {np.max(xy_err):.4f} m')
    print(f'  t(1m): {time_to_alt(1.0):.2f}s  |  t(0.5m): {time_to_alt(0.5):.2f}s')
    print(f'  Vz oscillations: {vz_osc} total, {late_osc} late (last 30%)')
    print(f'  Control energy (sum T^2 dt): {ctrl_energy:.1f}')
    print()


def print_detail(ctrl_id, c):
    """Deep-dive analysis for a single controller."""
    rel = c['rel_pos']
    vel = c['vel']
    alt = -rel[2]
    vel_z = vel[2]
    xy_err = np.linalg.norm(rel[0:2], axis=0)
    t = c['t']
    T = c['thrust']
    roll_d = np.degrees(c['roll'])
    pitch_d = np.degrees(c['pitch'])
    speed = np.linalg.norm(vel, axis=0)
    idx = c['idx']
    d = c['raw']

    print(f'\n--- Detailed analysis: {NAMES[ctrl_id-1]} (ctrl {ctrl_id}) ---\n')

    # ---- Phase analysis ----
    yaw_d_phase = np.degrees(c['yaw'])
    tgt_yaw_phase = np.degrees(c['tgt_yaw'])
    yaw_err_phase = yaw_d_phase - tgt_yaw_phase

    print('Phase analysis (2s windows):')
    print(f'{"t[s]":>6} {"alt[m]":>7} {"vz[m/s]":>8} {"xy[m]":>7} {"spd[m/s]":>8} '
          f'{"roll":>7} {"pitch":>8} {"yaw_err":>8} {"T[N]":>6} {"Tstd":>6}')
    win = 200  # 2s
    for j in range(0, idx+1, win):
        sl = slice(j, min(j+win, idx+1))
        print(f'{t[j]:6.1f} {np.mean(alt[sl]):7.2f} {np.mean(vel_z[sl]):8.3f} '
              f'{np.mean(xy_err[sl]):7.3f} {np.mean(speed[sl]):8.3f} '
              f'{np.mean(roll_d[sl]):7.1f} {np.mean(pitch_d[sl]):8.1f} '
              f'{np.mean(yaw_err_phase[sl]):+8.2f} '
              f'{np.mean(T[sl]):6.1f} {np.std(T[sl]):6.02f}')

    # ---- Stability diagnostics ----
    print('\nStability diagnostics:')
    for lim in [10, 20, 45, 90]:
        roll_exc = np.where(np.abs(roll_d) > lim)[0]
        pitch_exc = np.where(np.abs(pitch_d) > lim)[0]
        r_t = f't={t[roll_exc[0]]:.2f}s' if len(roll_exc) else 'never'
        p_t = f't={t[pitch_exc[0]]:.2f}s' if len(pitch_exc) else 'never'
        r_pct = 100 * len(roll_exc) / (idx+1)
        p_pct = 100 * len(pitch_exc) / (idx+1)
        print(f'  |roll|>{lim:3d}: first {r_t:>12s}  ({r_pct:4.1f}% of sim)  |  '
              f'|pitch|>{lim:3d}: first {p_t:>12s}  ({p_pct:4.1f}% of sim)')

    t_max = np.max(T)
    sat_hi = np.sum(T >= t_max * 0.99) / (idx+1) * 100
    sat_lo = np.sum(T <= np.min(T) * 1.01 + 0.01) / (idx+1) * 100
    print(f'  Thrust saturation: {sat_hi:.1f}% at max ({t_max:.0f}N), {sat_lo:.1f}% at min ({np.min(T):.1f}N)')

    # ---- Altitude convergence ----
    print('\nAltitude convergence:')
    for thr in [4.0, 3.0, 2.0, 1.5, 1.0, 0.5, 0.3]:
        below = alt <= thr
        if np.any(below):
            t_reach = t[np.argmax(below)]
            first_below = np.argmax(below)
            stayed = np.all(alt[first_below:] <= thr + 0.5)
            tag = '' if stayed else ' (did NOT stay)'
            print(f'  alt < {thr:.1f}m at t={t_reach:.2f}s{tag}')
        else:
            print(f'  alt < {thr:.1f}m: never reached')

    # ---- Oscillation analysis ----
    print('\nOscillation analysis (Vz zero-crossings per 5s window):')
    win5 = 500
    for j in range(0, idx+1, win5):
        sl = slice(j, min(j+win5, idx+1))
        vz_w = vel_z[sl]
        signs = np.sign(vz_w[vz_w != 0])
        cross = int(np.sum(np.diff(signs) != 0)) if len(signs) > 1 else 0
        print(f'  t={t[j]:5.1f}-{t[min(j+win5-1,idx)]:5.1f}s: {cross:2d} crossings  '
              f'(alt range {np.min(alt[sl]):.2f}-{np.max(alt[sl]):.2f}m)')

    # ---- Optical flow noise (from V_X_DS) ----
    if 'V_X_DS' in d:
        VX = d['V_X_DS']
        if VX.ndim == 2 and VX.shape[1] >= min(idx, 10):
            print('\nOptical flow noise (h_i std + max step jump, 10-step windows):')
            step = max(1, idx // 8)
            for j in range(10, idx, step):
                sl = slice(max(0, j-5), min(j+5, idx))
                h_i = VX[3:6, sl]
                h_std = np.std(h_i, axis=1)
                h_jump = np.max(np.abs(np.diff(h_i, axis=1))) if h_i.shape[1] > 1 else 0
                print(f'  t={t[j]:5.2f}s  alt={alt[j]:.2f}m  '
                      f'std(h)=[{h_std[0]:.3f},{h_std[1]:.3f},{h_std[2]:.3f}]  '
                      f'max_jump={h_jump:.3f}')

    # ---- DF-ASMC barrier proximity (if p_2 available) ----
    if 'p_2' in d and d['p_2'] is not None:
        p2 = d['p_2']
        if hasattr(p2, 'shape') and p2.ndim == 2 and p2.shape[1] >= idx and 'D_DS' in d:
            D_DS = d['D_DS']
            VX = d.get('V_X_DS')
            if VX is not None and VX.ndim == 2 and D_DS.ndim == 2:
                print('\nBarrier proximity (|h_e / p_2| per axis, 1.0 = crash):')
                step = max(1, idx // 10)
                max_ratio = np.zeros(3)
                max_ratio_t = np.zeros(3)
                for j in range(0, idx, step):
                    h_i = VX[3:6, j]
                    h_d = D_DS[0:3, j]
                    h_e = h_i - h_d
                    p = p2[:, j]
                    p[p == 0] = 1e-10
                    ratio = np.abs(h_e / p)
                    print(f'  t={t[j]:5.2f}s  alt={alt[j]:.2f}m  '
                          f'|h_e/p2|=[{ratio[0]:.4f},{ratio[1]:.4f},{ratio[2]:.4f}]  '
                          f'p2=[{p[0]:.2f},{p[1]:.2f},{p[2]:.2f}]')
                    for k in range(3):
                        if ratio[k] > max_ratio[k]:
                            max_ratio[k] = ratio[k]
                            max_ratio_t[k] = t[j]
                print(f'  Peak ratios: [{max_ratio[0]:.4f} @ t={max_ratio_t[0]:.2f}s, '
                      f'{max_ratio[1]:.4f} @ t={max_ratio_t[1]:.2f}s, '
                      f'{max_ratio[2]:.4f} @ t={max_ratio_t[2]:.2f}s]')

    # ---- Yaw control analysis ----
    yaw_d = np.degrees(c['yaw'])
    tgt_yaw_d = np.degrees(c['tgt_yaw'])
    yaw_err = yaw_d - tgt_yaw_d
    omega_z = np.degrees(c['omega'][2])
    tau_z = c['torques'][2]

    print('\nYaw control:')
    print(f'  UAV yaw: [{np.min(yaw_d):.1f}, {np.max(yaw_d):.1f}] deg  |  '
          f'Tgt yaw: [{np.min(tgt_yaw_d):.1f}, {np.max(tgt_yaw_d):.1f}] deg')
    print(f'  Yaw error: mean={np.mean(yaw_err):+.2f}, std={np.std(yaw_err):.2f}, '
          f'max|err|={np.max(np.abs(yaw_err)):.2f} deg')
    print(f'  Yaw rate: max|wz|={np.max(np.abs(omega_z)):.2f} deg/s  |  '
          f'Yaw torque: max|tau_z|={np.max(np.abs(tau_z)):.4f}, mean|tau_z|={np.mean(np.abs(tau_z)):.4f} Nm')

    print(f'  {"t[s]":>6} {"uav_yaw":>8} {"tgt_yaw":>8} {"yaw_err":>8} '
          f'{"wz":>8} {"|tau_z|":>8}')
    win = 200
    for j in range(0, idx+1, win):
        sl = slice(j, min(j+win, idx+1))
        print(f'  {t[j]:6.1f} {np.mean(yaw_d[sl]):+8.1f} {np.mean(tgt_yaw_d[sl]):+8.1f} '
              f'{np.mean(yaw_err[sl]):+8.2f} '
              f'{np.mean(omega_z[sl]):+8.2f} {np.mean(np.abs(tau_z[sl])):8.4f}')

    # ---- Crash diagnostics ----
    if c['status'] == 'CRASHED':
        print('\nCRASH DIAGNOSTICS:')
        print('Last 5 steps before break:')
        D_DS = d.get('D_DS')
        for i in range(max(0, idx-4), idx+1):
            a_cd = D_DS[3:6, i] if D_DS is not None and D_DS.ndim == 2 else np.zeros(3)
            E_crd = D_DS[6:8, i] if D_DS is not None and D_DS.ndim == 2 else np.zeros(2)
            dE_cd = D_DS[8:11, i] if D_DS is not None and D_DS.ndim == 2 else np.zeros(3)
            print(f'  t={t[i]:.3f}s  alt={alt[i]:.3f}m  '
                  f'|a_cd|={np.linalg.norm(a_cd):.1f}  '
                  f'E_crd=[{np.degrees(E_crd[0]):.1f},{np.degrees(E_crd[1]):.1f}]deg  '
                  f'|dE_cd|={np.linalg.norm(dE_cd):.1f}')
        # Identify likely break cause
        a_norm = np.linalg.norm(D_DS[3:6, idx]) if D_DS is not None else 0
        dE_norm = np.linalg.norm(D_DS[8:11, idx]) if D_DS is not None else 0
        if a_norm > 100:
            print(f'  -> Likely cause: |I_a_cd| = {a_norm:.1f} > 100 (safety limit)')
        elif dE_norm > 100:
            print(f'  -> Likely cause: |dE_cd| = {dE_norm:.1f} > 100 (safety limit)')
        elif a_norm == 0 and dE_norm == 0:
            print(f'  -> Break at idx={idx}: logging incomplete (NaN or pre-logging safety break)')
        else:
            print(f'  -> Break cause unclear (|a_cd|={a_norm:.1f}, |dE_cd|={dE_norm:.1f})')


def plot_detail(ctrl_id, c):
    """Generate diagnostic plots."""
    import matplotlib.pyplot as plt

    rel = c['rel_pos']
    vel = c['vel']
    alt = -rel[2]
    t = c['t']
    T = c['thrust']
    roll_d = np.degrees(c['roll'])
    pitch_d = np.degrees(c['pitch'])
    xy_err = np.linalg.norm(rel[0:2], axis=0)

    yaw_d = np.degrees(c['yaw'])
    tgt_yaw_d = np.degrees(c['tgt_yaw'])
    yaw_err = yaw_d - tgt_yaw_d

    fig, axes = plt.subplots(4, 2, figsize=(14, 13), sharex=True)
    fig.suptitle(f'{NAMES[ctrl_id-1]} (ctrl {ctrl_id}) [{c["status"]}]', fontsize=14)

    axes[0, 0].plot(t, alt); axes[0, 0].set_ylabel('Altitude [m]')
    axes[0, 0].axhline(0.2, color='r', ls='--', lw=0.8, label='threshold')
    axes[0, 0].legend()

    axes[0, 1].plot(t, xy_err); axes[0, 1].set_ylabel('XY error [m]')

    axes[1, 0].plot(t, vel[2], label='vz'); axes[1, 0].set_ylabel('Vz [m/s]')
    axes[1, 0].axhline(0, color='k', ls=':', lw=0.5)

    axes[1, 1].plot(t, roll_d, label='roll')
    axes[1, 1].plot(t, pitch_d, label='pitch')
    axes[1, 1].set_ylabel('Attitude [deg]'); axes[1, 1].legend()

    axes[2, 0].plot(t, T); axes[2, 0].set_ylabel('Thrust [N]')

    axes[2, 1].plot(t, np.linalg.norm(vel, axis=0)); axes[2, 1].set_ylabel('Speed [m/s]')

    axes[3, 0].plot(t, yaw_d, label='UAV')
    axes[3, 0].plot(t, tgt_yaw_d, '--', label='Target')
    axes[3, 0].set_ylabel('Yaw [deg]'); axes[3, 0].legend()
    axes[3, 0].set_xlabel('Time [s]')

    axes[3, 1].plot(t, yaw_err)
    axes[3, 1].set_ylabel('Yaw error [deg]')
    axes[3, 1].axhline(0, color='k', ls=':', lw=0.5)
    axes[3, 1].set_xlabel('Time [s]')

    plt.tight_layout()
    fname = f'diag_ctrl_{ctrl_id}.png'
    plt.savefig(fname, dpi=120)
    print(f'\nPlot saved to {fname}')
    plt.close()


# ---- Main ----
if __name__ == '__main__':
    args = sys.argv[1:]
    do_plot = '--plot' in args
    args = [a for a in args if a != '--plot']

    if args:
        cid = int(args[0])
        c = load_ctrl(cid)
        print_summary(cid, c)
        print_detail(cid, c)
        if do_plot:
            plot_detail(cid, c)
    else:
        for cid in range(1, 6):
            fname = DATA_DIR / f'result_ctrl_{cid}.mat'
            if not fname.exists():
                print(f'=== {NAMES[cid-1]} (ctrl {cid}) === MISSING\n')
                continue
            c = load_ctrl(cid)
            print_summary(cid, c)
