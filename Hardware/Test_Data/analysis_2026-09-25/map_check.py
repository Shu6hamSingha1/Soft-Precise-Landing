"""Command-mapping check: do the body rates and thrust the controller computed arrive at the FC as intended? (run from Hardware/Test_Data)"""
import sys, numpy as np
sys.path.insert(0, 'analysis_2026-09-25')
from common import load_day, ctl_arr
def analyse(date):
    F = load_day(date)
    print('==', date, len(F), 'flights; clock-fit SSE median %.4f max %.3f m^2' % (np.median([f['sse'] for f in F]), max(f['sse'] for f in F)))
    ax_gain = [[], [], []]; ax_lag = [[], [], []]; ax_corr = [[], [], []]; bslope = []; bcorr = []
    for f in F:
        c = f['ctl']; n = min(len(c['t']), len(c['w_u(t)']), len(c['B_T(t)']))
        t = np.array(c['t'][:n]) - f['o']; wu = ctl_arr(c, 'w_u(t)', n); BT = np.array(c['B_T(t)'][:n], float)
        rs = f['fc']['vehicle_rates_setpoint']; tr = rs['timestamp'] / 1e6
        w = (t >= f['to0'] + 1.0) & (t <= f['to1'] - 0.3)
        if w.sum() < 50: continue
        tg = t[w]
        for ax, key in enumerate(['roll', 'pitch', 'yaw']):
            x = wu[w, ax]
            best = None
            for lag in np.arange(0.0, 0.3, 0.01):
                y = np.interp(tg + lag, tr, rs[key])
                if np.std(x) < 1e-6 or np.std(y) < 1e-6: continue
                cc = np.corrcoef(x, y)[0, 1]
                if best is None or cc > best[1]: best = (lag, cc, np.polyfit(x, y, 1)[0])
            if best: ax_lag[ax].append(best[0]); ax_corr[ax].append(best[1]); ax_gain[ax].append(best[2])
        th = f['fc']['vehicle_thrust_setpoint']; tt = th['timestamp'] / 1e6; T = -th['xyz[2]']
        best = None
        for lag in np.arange(0.0, 0.3, 0.01):
            y = np.interp(tg + lag, tt, T); cc = np.corrcoef(BT[w], y)[0, 1]
            if best is None or cc < best[1]: best = (lag, cc, np.polyfit(BT[w], y, 1)[0])
        bslope.append(best[2]); bcorr.append(best[1])
    for ax, nm in enumerate(['roll ', 'pitch', 'yaw  ']):
        print(' w_u %s -> FC rates_setpoint: gain median %.2f (p10 %.2f p90 %.2f) lag %.0f ms corr %.2f' % (nm, np.median(ax_gain[ax]), *np.percentile(ax_gain[ax], [10, 90]), 1e3 * np.median(ax_lag[ax]), np.median(ax_corr[ax])))
    print(' B_T -> FC thrust_setpoint slope %.4f per N (expected -1/31.98 = -0.0313) p10 %.4f p90 %.4f ; corr median %.2f' % (np.median(bslope), *np.percentile(bslope, [10, 90]), np.median(bcorr)))
for d in sys.argv[1:] or ['2026-09-25', '2026-09-24']: analyse(d)
