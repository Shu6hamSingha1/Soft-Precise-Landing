"""Shared loader for Pi hardware flight-test analysis (any date). Run from Hardware/Test_Data.

    from common import load_day
    flights = load_day('2026-09-24')      # list of dicts, one per controller flight, Pi run <-> FC log matched BY ORDER

Each flight dict: date, k, run (Pi dir name), ctl/tel (Pi npy dicts), fc (FC arrays), to0/to1 (offboard start/end, FC clock),
nav_after (nav_state after offboard: 2=POSCTL i.e. pilot stick takeover, 18=AUTO_LAND i.e. script land), tarm, tk (kill), o (Pi->FC clock
offset: t_fc = t_pi - o), sse (m^2 height-profile fit of the clock offset; small = trustworthy match).
Caches the parsed ulog topics in $TEMP/sp/cache_<date>.pkl (parsing 47 ulogs takes minutes).
Known gotchas: FC clock drifts vs Pi (match by order); FC logs the rangefinder at ~1 Hz (use EKF height from vehicle_local_position);
Control_Data field names carry '(t)'; p(t) is the funnel width, not position.
"""
import glob, os, pickle
import numpy as np

TOPICS = {
    'vehicle_status': ['timestamp', 'nav_state', 'arming_state'],
    'actuator_armed': ['timestamp', 'armed', 'manual_lockdown'],
    'vehicle_local_position': ['timestamp', 'x', 'y', 'z', 'vx', 'vy', 'vz'],
    'vehicle_attitude': ['timestamp', 'q[0]', 'q[1]', 'q[2]', 'q[3]'],
    'vehicle_thrust_setpoint': ['timestamp', 'xyz[2]'],
    'vehicle_rates_setpoint': ['timestamp', 'roll', 'pitch', 'yaw'],
    'vehicle_angular_velocity': ['timestamp', 'xyz[0]', 'xyz[1]', 'xyz[2]'],
    'sensor_combined': ['timestamp', 'accelerometer_m_s2[0]', 'accelerometer_m_s2[1]', 'accelerometer_m_s2[2]'],
    'battery_status': ['timestamp', 'voltage_v', 'current_a'],
    'hover_thrust_estimate': ['timestamp', 'hover_thrust'],
    'manual_control_setpoint': ['timestamp', 'roll', 'pitch', 'yaw', 'throttle'],
    'actuator_motors': ['timestamp', 'control[0]', 'control[1]', 'control[2]', 'control[3]'],
}


def _npy(f):
    x = np.load(f, allow_pickle=True)
    return x.item() if x.shape == () else x


def _tmp():
    d = os.path.join(os.environ.get('TEMP', '/tmp'), 'sp')
    os.makedirs(d, exist_ok=True)
    return d


def _fc_cache(date):
    p = os.path.join(_tmp(), 'cache_%s.pkl' % date)
    if os.path.exists(p):
        return pickle.load(open(p, 'rb'))
    from pyulog import ULog
    out = []
    for f in sorted(glob.glob('FlightLogs/%s/*.ulg' % date)):
        u = ULog(f)
        D = {}
        for d in u.data_list:
            if d.multi_id == 0 and d.name in TOPICS and d.name not in D:
                D[d.name] = {k: np.asarray(d.data[k]) for k in TOPICS[d.name] if k in d.data}
        st = D.get('vehicle_status')
        if st is None or not (st['nav_state'] == 14).any():
            continue
        D['_msgs'] = [(m.timestamp / 1e6, m.message.strip()) for m in u.logged_messages]
        D['_file'] = os.path.basename(f)
        out.append(D)
    pickle.dump(out, open(p, 'wb'))
    return out


def load_day(date):
    pi = []
    for d in sorted(glob.glob('Landing/%s/[A-Z][a-z][a-z] *' % date)):
        try:
            c = _npy(d + '/Control_Data.npy')
        except Exception:
            continue
        if len(c.get('t', [])) >= 20 and np.ndim(c['p(t)']) == 2:
            pi.append(d)
    fc = _fc_cache(date)
    assert len(pi) == len(fc), 'Pi runs %d vs FC offboard flights %d - match by order impossible' % (len(pi), len(fc))
    flights = []
    for k, (d, D) in enumerate(zip(pi, fc)):
        st = D['vehicle_status']; ts = st['timestamp'] / 1e6; nav = st['nav_state']
        to0 = ts[np.where(nav == 14)[0][0]]
        j = np.where((nav != 14) & (ts > to0))[0]
        to1 = ts[j[0]] if len(j) else ts[-1]
        nav_after = int(nav[j[0]]) if len(j) else -1
        lk = D['actuator_armed']; tl = lk['timestamp'] / 1e6
        tarm = tl[np.where(lk['armed'] == 1)[0][0]]
        kk = np.where((lk['manual_lockdown'] == 1) & (tl >= to0))[0]
        tk = tl[kk[0]] if len(kk) else None
        lp = D['vehicle_local_position']; tp = lp['timestamp'] / 1e6
        zg = lp['z'][np.searchsorted(tp, tarm + 0.3)]
        T = _npy(d + '/Telemetry_Data.npy'); ctl = _npy(d + '/Control_Data.npy')
        tpi = np.array(T['Distance Sensor Timestamp']); api = np.array([a.current_distance_m for a in T['Distance Sensor']])
        tf = tp; af = -(lp['z'] - zg)

        def pk(t, a):
            s = np.convolve(a, np.ones(5) / 5, 'same'); return t[s > 0.95 * s.max()][0]
        o0 = pk(tpi, api) - pk(tf, af); best = None
        m = (tf >= to0 - 2) & (tf <= to1 + 3)
        for o in np.arange(o0 - 8, o0 + 8, 0.02):
            sse = np.mean((np.interp(tf[m] + o, tpi, api) - af[m]) ** 2)
            if best is None or sse < best[1]:
                best = (o, sse)
        flights.append(dict(date=date, k=k, run=d[-13:-5], dir=d, ctl=ctl, tel=T, fc=D, to0=to0, to1=to1, nav_after=nav_after,
                            tarm=tarm, tk=tk, o=best[0], sse=float(best[1]), zg=zg))
    return flights


def ctl_arr(c, key, n=None):
    v = c[key]; n = len(v) if n is None else n
    return np.array([np.asarray(x, float) for x in list(v)[:n]])
