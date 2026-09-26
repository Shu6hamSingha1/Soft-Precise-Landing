"""Achieved body rate (FC vehicle_angular_velocity) vs controller-intended w_u and vs FC rates_setpoint. Run from Hardware/Test_Data."""
import sys, numpy as np
sys.path.insert(0, 'analysis_2026-09-25')
from common import load_day, ctl_arr
def run(date):
    F = load_day(date); g_int = [[], [], []]; g_set = [[], [], []]; lag_int = []
    for f in F:
        c = f['ctl']; n = min(len(c['t']), len(c['w_u(t)'])); t = np.array(c['t'][:n]) - f['o']; wu = ctl_arr(c, 'w_u(t)', n)
        av = f['fc']['vehicle_angular_velocity']; ta = av['timestamp'] / 1e6; rs = f['fc']['vehicle_rates_setpoint']; tr = rs['timestamp'] / 1e6
        w = (t >= f['to0'] + 1.0) & (t <= f['to1'] - 0.3)
        if w.sum() < 50: continue
        tg = t[w]
        for ax, (k1, k2) in enumerate([('xyz[0]', 'roll'), ('xyz[1]', 'pitch'), ('xyz[2]', 'yaw')]):
            x = wu[w, ax]
            if np.std(x) < 1e-3: continue
            best = None
            for lag in np.arange(0.0, 0.4, 0.01):
                y = np.interp(tg + lag, ta, av[k1]); cc = np.corrcoef(x, y)[0, 1]
                if best is None or cc > best[1]: best = (lag, cc, np.polyfit(x, y, 1)[0])
            g_int[ax].append(best[2])
            y = np.interp(tg, ta, av[k1]) ; s = np.interp(tg, tr, rs[k2])
            best2 = max(((np.corrcoef(s[:len(s)-l] if l else s, np.interp(tg + l*0.01, ta, av[k1])[:len(s)-l] if l else np.interp(tg, ta, av[k1]))[0, 1], l) for l in range(0, 20)), key=lambda z: z[0])
            g_set[ax].append(np.polyfit(np.interp(tg, tr, rs[k2]), np.interp(tg + best2[1] * 0.01, ta, av[k1]), 1)[0])
    print('==', date)
    for ax, nm in enumerate(['roll', 'pitch', 'yaw']):
        print(' %-5s achieved/intended(w_u) gain: median %.2f (p10 %.2f p90 %.2f) | achieved/FC-setpoint gain: median %.2f' % (nm, np.median(g_int[ax]), *np.percentile(g_int[ax], [10, 90]), np.median(g_set[ax])))
for d in sys.argv[1:] or ['2026-09-25', '2026-09-24']: run(d)
