"""Yaw loop check against the FC heading (independent of the controller's e_a). Run from Hardware/Test_Data."""
import sys, numpy as np
sys.path.insert(0, 'analysis_2026-09-25')
from common import load_day, ctl_arr
def wrap(a): return (a + np.pi) % (2 * np.pi) - np.pi
def run(date):
    F = load_day(date); rows = []
    for f in F:
        c = f['ctl']; n = min(len(c['t']), len(c['e_a(t)']), len(c['u_a(t)']))
        t = np.array(c['t'][:n]) - f['o']; ea = np.array(c['e_a(t)'][:n], float); ua = np.array(c['u_a(t)'][:n], float)
        at = f['fc']['vehicle_attitude']; ta = at['timestamp'] / 1e6; q = np.stack([at['q[%d]' % i] for i in range(4)], 1).astype(float)
        yaw = np.unwrap(np.arctan2(2 * (q[:, 0] * q[:, 3] + q[:, 1] * q[:, 2]), 1 - 2 * (q[:, 2] ** 2 + q[:, 3] ** 2)))
        w = (t >= f['to0'] + 0.5) & (t <= f['to1'])
        if w.sum() < 50: continue
        y = np.interp(t[w], ta, yaw); y0 = np.interp(f['to0'] + 0.5, ta, yaw)   # target heading ~ heading at engage
        e_true = wrap(y - y0); ew = ea[w]
        # sign / scale relation between controller e_a and true heading error
        k = np.polyfit(e_true, ew, 1)[0] if np.std(e_true) > 1e-3 else np.nan
        # does the commanded yaw rate reduce the TRUE error? d|e|/dt vs sign(e)*u_a
        de = np.gradient(np.abs(e_true), t[w]); m = np.abs(e_true) > 0.08
        agree = np.mean(np.sign(de[m]) == -np.sign(np.sign(e_true[m]) * ua[w][m])) if m.sum() > 20 else np.nan
        rows.append((f['run'], k, np.abs(e_true).max(), np.abs(e_true[-1]) / max(np.abs(e_true[:10]).mean(), 1e-2), np.degrees(y[-1] - y[0]), agree, np.abs(ua[w]).max()))
    r = np.array([x[1:] for x in rows], float)
    print('==', date, len(rows), 'flights: true (FC) heading error vs controller e_a')
    print(' slope e_a vs true heading change: median %.2f (p10 %.2f p90 %.2f)  [-1 => e_a = -(yaw-yaw0) (sign per design)]' % (np.nanmedian(r[:, 0]), *np.nanpercentile(r[:, 0], [10, 90])))
    print(' net heading change over controlled phase: median %.1f deg, p90 |.| %.1f deg, max |.| %.1f deg' % (np.median(r[:, 3]), np.percentile(np.abs(r[:, 3]), 90), np.abs(r[:, 3]).max()))
    print(' max |true heading error| median %.1f deg p90 %.1f deg' % (np.degrees(np.median(r[:, 1])), np.degrees(np.percentile(r[:, 1], 90))))
    print(' when |err|>0.08 rad: fraction of time command REDUCES true error: median %.2f (0.5 = no better than chance)' % np.nanmedian(r[:, 4]))
    print(' max |u_a| (yaw rate cmd) median %.2f rad/s' % np.median(r[:, 5]))
for d in sys.argv[1:] or ['2026-09-25', '2026-09-24']: run(d)
