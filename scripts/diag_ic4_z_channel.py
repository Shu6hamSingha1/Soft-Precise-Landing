"""
Diagnostic: inspect the vertical (z) channel on IC4 realistic-mode runs.
Goal: identify which term in the outer-loop PLASMC z-path is capping descent
when the drone converges laterally (xy<=2cm) but hovers at alt>=25cm at t=40s.

Signals inspected (from result.data workspace):
    V_h_e(3,:)   — vertical image-feature tracking error
    p_2(3,:)     — time-varying funnel (z)
    S_2(3,3,:)   — normalized error, saturated in [-0.95,+0.95]
    zeta_2(3,:)  — log-barrier transformed error
    izeta_2(3,:) — integrator (clamped |.|<=5)
    sigma(3,:)   — sliding variable (z)
    kappa(3,:)   — adaptive gain (z)
    G_2(3,3,:)   — outer funnel gain (z)
    I_a_cd(3,:)  — commanded inertial vertical accel
    h_rd         — scalar, desired vertical image rate

For each of Linear/Circular IC4 we compute:
    - time-to-break-point: when |S_2(3)| first pins at 0.95
    - final-window statistics (last 2s)
    - which saturation if any is load-bearing
"""
import sys
from pathlib import Path
import numpy as np
import scipy.io as sio

DATA_DIR = Path(__file__).resolve().parent.parent / 'MATLAB' / 'Multi_init_cond' / 'Datasets'
IC_INDEX = 3  # IC4, 0-based
S2_MARGIN = 0.05
S2_CAP = 1.0 - S2_MARGIN  # 0.95


def load_run(trajType, ic_idx):
    path = str(DATA_DIR / f'{trajType}_multi_init.mat')
    d = sio.loadmat(path, squeeze_me=True, struct_as_record=False)
    r = d['results'][ic_idx]
    return r


def diag_one(trajType, ic_idx):
    r = load_run(trajType, ic_idx)
    dat = r.data
    idx = int(dat.idx)
    dt  = float(dat.dt)

    t = np.arange(idx+1) * dt

    X   = dat.X_DS[:, :idx+1]
    xt  = dat.x_t[:, :idx+1].astype(float)
    pos = X[0:3, :]
    vel = X[7:10, :]

    # Relative state
    rel_xy  = np.linalg.norm((pos - xt[:3])[:2], axis=0)
    alt_err = np.abs((pos - xt[:3])[2])

    # Outer-loop z signals -- trim to [1..idx] since they are N_steps sized
    # (indexing in MATLAB is 1-based; logs populated for idx=1..N_steps)
    V_h_e    = dat.V_h_e[:, :idx+1]
    p_2      = dat.p_2[:, :idx+1]
    S_2_full = dat.S_2[:, :, :idx+1]
    zeta_2   = dat.zeta_2[:, :idx+1]
    izeta_2  = dat.izeta_2[:, :idx+1]
    sigma    = dat.sigma[:, :idx+1]
    kappa    = dat.kappa[:, :idx+2]   # size N+1
    I_a_cd   = dat.I_a_cd[:, :idx+1]

    # S_2(3,3,:) extraction
    S_2z = S_2_full[2, 2, :]
    G_2_full = dat.G_2[:, :, :idx+1]
    G_2z = G_2_full[2, 2, :]

    V_h_e_z    = V_h_e[2]
    p_2z       = p_2[2]
    zeta_2z    = zeta_2[2]
    izeta_2z   = izeta_2[2]
    sigma_z    = sigma[2]
    kappa_z    = kappa[2, :idx+1]
    I_a_cd_z   = I_a_cd[2]

    # h_rd scalar
    h_rd = float(np.atleast_1d(dat.h_rd).ravel()[0])

    # FoV / cone logs (Approach 2)
    theta_cone = np.atleast_1d(dat.theta_cone_log[:idx+1]).ravel()
    d_min      = np.atleast_1d(dat.d_min_log[:idx+1]).ravel()

    # Key statistics
    # Final 2 seconds window
    t_final = 2.0
    n_final = int(round(t_final / dt))
    sl = slice(-n_final, None)

    # --- Saturation detection ---
    s2_pinned = np.abs(np.abs(S_2z) - S2_CAP) < 1e-6  # within numerical tol of cap
    s2_pinned_any_after_5s = s2_pinned[int(5/dt):].any()
    t_first_pin = None
    mask_pin_after_5s = s2_pinned.copy()
    mask_pin_after_5s[:int(5/dt)] = False
    if mask_pin_after_5s.any():
        t_first_pin = t[np.argmax(mask_pin_after_5s)]

    izeta_pinned = np.abs(np.abs(izeta_2z) - 5.0) < 1e-6
    izeta_pinned_after_5s = izeta_pinned[int(5/dt):].any()

    # I_a_cd vertical floor
    iacd_floor_hit = (I_a_cd_z > -3.001) & (I_a_cd_z < -2.999)
    iacd_floor_hit_any_after_5s = iacd_floor_hit[int(5/dt):].any()

    # Print
    print(f'\n{"="*20} {trajType} IC{ic_idx+1} realistic {"="*20}')
    print(f'  landed             : {bool(r.success)}  '
          f'xy={r.final_xy*100:.1f}cm  alt={r.final_alt*100:.1f}cm  '
          f'|v_rel|={r.final_rel_vel*100:.1f}cm/s')
    print(f'  t_end              : {t[-1]:.2f} s')
    print(f'  h_rd (desired rate): {h_rd:+.3f}')
    print(f'')
    print(f'  --- Outer-loop Z-channel (last 2s) ---')
    print(f'    V_h_e(3)   : mean={V_h_e_z[sl].mean():+.4f}  '
          f'range=[{V_h_e_z[sl].min():+.4f}, {V_h_e_z[sl].max():+.4f}]')
    print(f'    p_2(3)     : mean={p_2z[sl].mean():.4f}  '
          f'range=[{p_2z[sl].min():.4f}, {p_2z[sl].max():.4f}]')
    print(f'    S_2(3,3)   : mean={S_2z[sl].mean():+.4f}  '
          f'range=[{S_2z[sl].min():+.4f}, {S_2z[sl].max():+.4f}]')
    print(f'    zeta_2(3)  : mean={zeta_2z[sl].mean():+.3f}  '
          f'range=[{zeta_2z[sl].min():+.3f}, {zeta_2z[sl].max():+.3f}]')
    print(f'    izeta_2(3) : mean={izeta_2z[sl].mean():+.3f}  '
          f'range=[{izeta_2z[sl].min():+.3f}, {izeta_2z[sl].max():+.3f}]')
    print(f'    sigma(3)   : mean={sigma_z[sl].mean():+.3f}  '
          f'range=[{sigma_z[sl].min():+.3f}, {sigma_z[sl].max():+.3f}]')
    print(f'    kappa(3)   : mean={kappa_z[sl].mean():.3f}  '
          f'range=[{kappa_z[sl].min():.3f}, {kappa_z[sl].max():.3f}]')
    print(f'    G_2(3,3)   : mean={G_2z[sl].mean():.3f}  '
          f'range=[{G_2z[sl].min():.3f}, {G_2z[sl].max():.3f}]')
    print(f'    I_a_cd(3)  : mean={I_a_cd_z[sl].mean():+.3f}  '
          f'range=[{I_a_cd_z[sl].min():+.3f}, {I_a_cd_z[sl].max():+.3f}]')
    print(f'    v_z        : mean={vel[2][sl].mean():+.3f}  '
          f'range=[{vel[2][sl].min():+.3f}, {vel[2][sl].max():+.3f}]')
    print(f'')
    print(f'  --- Saturation flags ---')
    print(f'    |S_2(3)| pinned at {S2_CAP}: {s2_pinned.sum()}/{idx+1} steps  '
          f'(after t=5s: {s2_pinned_any_after_5s})')
    if t_first_pin is not None:
        print(f'    First S_2 pin after t=5s : {t_first_pin:.2f} s')
    print(f'    |izeta_2(3)| at clamp 5.0 : {izeta_pinned.sum()}/{idx+1} steps '
          f'(after t=5s: {izeta_pinned_after_5s})')
    print(f'    I_a_cd(3) at floor -3.0   : {iacd_floor_hit.sum()}/{idx+1} steps '
          f'(after t=5s: {iacd_floor_hit_any_after_5s})')
    print(f'')
    print(f'  --- Altitude history ---')
    alt_start = -pos[2, 0]
    alt_end   = -pos[2, -1]
    idx_min = np.argmin(-pos[2])
    print(f'    alt start/end     : {alt_start*100:.1f}cm / {alt_end*100:.1f}cm')
    print(f'    alt min           : {-pos[2, idx_min]*100:.1f}cm  at t={t[idx_min]:.2f}s')
    # Descent rate vs time at ~20s, ~30s, ~40s
    for t_sample in [10, 20, 30, 39]:
        i = int(t_sample / dt)
        if i < idx:
            print(f'    t={t_sample:>2}s: alt={-pos[2,i]*100:6.1f}cm  '
                  f'v_z={vel[2,i]:+.3f}  '
                  f'I_a_cd(3)={I_a_cd_z[i]:+.2f}  '
                  f'S_2(3)={S_2z[i]:+.3f}  '
                  f'V_h_e(3)={V_h_e_z[i]:+.4f}')

    return {
        'trajType': trajType,
        's2_pinned_after_5s': s2_pinned_any_after_5s,
        'izeta_pinned_after_5s': izeta_pinned_after_5s,
        'iacd_floor_after_5s': iacd_floor_hit_any_after_5s,
    }


def main():
    summary = []
    for traj in ['Linear', 'Circular', 'Sinusoidal']:
        summary.append(diag_one(traj, IC_INDEX))

    print('\n' + '='*62)
    print('SUMMARY — which saturation is load-bearing on IC4 late-phase?')
    print('='*62)
    for s in summary:
        flags = []
        if s['s2_pinned_after_5s']:   flags.append('S_2(3) pinned')
        if s['izeta_pinned_after_5s']: flags.append('izeta(3) pinned')
        if s['iacd_floor_after_5s']:   flags.append('I_a_cd(3) at floor')
        print(f"  {s['trajType']:12s} : {', '.join(flags) if flags else 'none'}")


if __name__ == '__main__':
    main()
