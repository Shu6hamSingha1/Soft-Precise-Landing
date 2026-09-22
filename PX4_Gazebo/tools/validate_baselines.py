"""Sanity-check the src/baselines.py ports: ideal-attitude point-mass closed loop
(a = I_a_cd + g), static + linear-moving target, IC = [2,2,-5] over the marker.
Not a substitute for SITL -- it only proves each outer loop is sign-correct and lands."""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
import numpy as np
from baselines import BASELINES, make_baseline, marker_key_points, project_marker_v_frame, accel_to_rate_thrust

M, F_CAM, ZF, DT = 2.114, 135.0, 0.3, 0.02
KEY = marker_key_points()
PXD = (F_CAM / (2 * 0.2)) * KEY[:2]          # desired points: depth 2*zf(MATLAB=0.2)

def run(name, v_t, T=60.0):
    b = make_baseline(name, M)
    p = np.array([2.0, 2.0, -5.0]); v = np.zeros(3); pt0 = np.zeros(3); R = np.eye(3)
    for k in range(int(T / DT)):
        t = k * DT
        pt = pt0 + v_t * t
        s = dict(t=t, dt=DT, p_c=p, v_c=v, p_t=pt, v_t=v_t, yaw=0.0, f=F_CAM, px_d=PXD)
        if b.needs_features:
            s["px"], s["C_s_tc"] = project_marker_v_frame(p, R, pt, R, 0.0, F_CAM, ZF, KEY)
        a = b.step(s)
        w, BT = accel_to_rate_thrust(a, R, 0.0, np.diag([2.5, 1.5, .5]), M)
        assert np.all(np.isfinite(a)), f"{name}: NaN at t={t:.1f}"
        v = v + DT * (a + np.array([0, 0, 9.81])); p = p + DT * v
        if p[2] > -0.15:                        # touchdown (camera ~0.15 above marker plane)
            return t, np.linalg.norm((p - pt)[:2]), v[2]
    return None, np.linalg.norm((p - pt)[:2]), v[2]

if __name__ == "__main__":
    for tgt, vt in (("static", np.zeros(3)), ("linear 0.5 m/s", np.array([0.5, 0.0, 0.0]))):
        print(f"--- {tgt}")
        for n in BASELINES:
            t, xy, vz = run(n, vt)
            print(f"{n:10s} " + (f"touchdown t={t:5.1f}s  xy_err={xy:.3f} m  vz={vz:+.2f} m/s" if t else f"NO TOUCHDOWN in 60 s (xy_err={xy:.2f} m)"))
