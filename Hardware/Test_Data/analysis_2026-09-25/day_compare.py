"""Headline metrics per day (run from Hardware/Test_Data): takeover rate, blow-ups, leak, thrust vs voltage."""
import sys, numpy as np
sys.path.insert(0, 'analysis_2026-09-25')
from common import load_day, ctl_arr
from ahrs import Quaternion
def run(date):
    F = load_day(date); R = dict(take=0, land=0, other=0, blown=0, big1k=0, leak=0, iaz5=0, azclamp=0, sen10=0, izc=0, tilt25=0, n=len(F))
    ratios = []; Vs = []; hov = []; cmd = []; extent_nz = 0; kmax = 0
    for f in F:
        c = f['ctl']; n = min(len(c[k]) for k in ['t', 'a_u(t)', 'I_a_raw(t)', 's_e_n(t)', 'izeta(t)', 'kappa(t)'])
        au = ctl_arr(c, 'a_u(t)', n); Ir = ctl_arr(c, 'I_a_raw(t)', n); sen = np.abs(ctl_arr(c, 's_e_n(t)', n)).max(1); iz = np.abs(ctl_arr(c, 'izeta(t)', n)).max()
        axy = np.hypot(au[:, 0], au[:, 1]).max(); leak = np.abs((Ir[:, 2] + 9.81) - au[:, 2]).max()
        R['take'] += f['nav_after'] == 2; R['land'] += f['nav_after'] == 18; R['other'] += f['nav_after'] not in (2, 18)
        R['blown'] += axy >= 100; R['big1k'] += axy >= 1000; R['leak'] += leak > 5; R['iaz5'] += (Ir[:, 2] > -5).any()
        R['azclamp'] += (au[:, 2] <= -2.99).any() or (au[:, 2] <= -0.19).mean() > 0.4; R['sen10'] += sen[-1] > 10; R['izc'] += iz >= 4.99
        kmax += ctl_arr(c, 'kappa(t)', n)[:, 0].max() >= 29.9
        ex = np.array(c['MARKER_EXTENT_PX(t)'][:n], float); extent_nz += (ex > 0).any()
        st = f['fc']; b = st['battery_status']; tb = b['timestamp'] / 1e6
        V = np.median(b['voltage_v'][(tb >= f['to0']) & (tb <= f['to0'] + 2.5)])
        th = st['vehicle_thrust_setpoint']; tt = th['timestamp'] / 1e6; T = np.median(-th['xyz[2]'][(tt >= f['to0'] + 0.5) & (tt <= f['to0'] + 2.5)])
        he = st['hover_thrust_estimate']; th_e = he['timestamp'] / 1e6; H = np.median(he['hover_thrust'][(th_e <= f['to0']) & (th_e >= f['to0'] - 6)]) if ((th_e <= f['to0']) & (th_e >= f['to0'] - 6)).any() else np.nan
        ratios.append(T / H); Vs.append(V); cmd.append(T); hov.append(H)
    print('== %s: %d flights' % (date, R['n']))
    print(' offboard ended by: pilot takeover %d, script land %d, other %d' % (R['take'], R['land'], R['other']))
    print(' a_u_xy max >=100: %d (>=1000: %d); vertical leak >5 m/s^2: %d; flights with I_a_raw_z > -5: %d; s_e_n(end)>10: %d; izeta at clamp: %d; kappa_x at 30: %d; MARKER_EXTENT>0 in: %d' % (R['blown'], R['big1k'], R['leak'], R['iaz5'], R['sen10'], R['izc'], kmax, extent_nz))
    r = np.array(ratios); V = np.array(Vs); print(' battery V at engage %.1f-%.1f ; hover cmd / FC hover estimate: median %.2f range %.2f-%.2f ; corr(V, cmd/est) %.2f' % (V.min(), V.max(), np.nanmedian(r), np.nanmin(r), np.nanmax(r), np.corrcoef(V[~np.isnan(r)], r[~np.isnan(r)])[0, 1]))
for d in sys.argv[1:] or ['2026-09-24', '2026-09-25']: run(d)
