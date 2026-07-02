#!/usr/bin/env python3
"""Deep-dive v2: cycle structure + stage phase budget, single LS estimator throughout.

Per rep:
 1. Complex GT error z=e_x+ i e_y (ENU) over the full descent (0.3<alt<4.8), dedup'd.
 2. 3-phasor fit: z ~ b0 + b1 t + B e^{i wz t} + A e^{i W t}  (wz=0.48 orbit bias; W on grid)
    -> TRUE cycle freq W*, cycle amp |A|, orbit-locked bias |B|, DC |b0|.
 3. Stage tones at W*: real-signal LS  s ~ c0+c1 t+a cos+b sin -> C_s (e^{+iW t} conv).
    Stages (x/NED and y/NED separately): e -> I_a_raw (controller), I_a_raw -> I_a (tau_ia),
    I_a -> tilt-accel (PX4 attitude), tilt -> delivered accel (geom, via iW*C_v), total act.,
    w_u -> gyro (rate loop). Kinematic closure check: arg(C_adel/C_e) ~ 180deg exactly.
"""
import numpy as np, os, sys, glob

WZ = 0.48

def quat_to_R(q):
    w, x, y, z = q
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-w*z),   2*(x*z+w*y)],
        [2*(x*y+w*z),   1-2*(x*x+z*z), 2*(y*z-w*x)],
        [2*(x*z-w*y),   2*(y*z+w*x),   1-2*(x*x+y*y)]])

def dedup(t, *sig):
    keep = np.concatenate([[True], np.diff(t) > 1e-9])
    return (t[keep],) + tuple(s[keep] for s in sig)

def tone_complex(t, z, W):
    t0 = t - t.mean()
    M = np.column_stack([np.ones_like(t0), t0, np.exp(1j*WZ*t0), np.exp(1j*W*t0)])
    coef, *_ = np.linalg.lstsq(M, z, rcond=None)
    r = z - M @ coef
    return coef, float(np.mean(np.abs(r)**2))

def fit_cycle(t, z, grid=np.arange(0.6, 3.2, 0.025)):
    best = None
    for W in grid:
        if abs(W - WZ) < 0.15:
            continue
        coef, rr = tone_complex(t, z, W)
        if best is None or rr < best[1]:
            best = (W, rr, coef)
    return best

def tone_real(t, s, W):
    """Complex amp C (convention s = Re{C e^{iWt}} + trend): C = a - i b."""
    t0 = t - t.mean()
    M = np.column_stack([np.ones_like(t0), t0, np.cos(W*t0), np.sin(W*t0),
                         np.cos(WZ*t0), np.sin(WZ*t0)])
    coef, *_ = np.linalg.lstsq(M, s, rcond=None)
    a, b = coef[2], coef[3]
    return complex(a, -b)

def rep_deepdive(d):
    gt = np.load(os.path.join(d, "Ground_Truth.npy"), allow_pickle=True).item()
    cd = np.load(os.path.join(d, "Control_Data.npy"), allow_pickle=True).item()
    td = np.load(os.path.join(d, "Telemetry_Data.npy"), allow_pickle=True).item()
    Tg = np.asarray(gt["Time"], float)
    up, tp = gt["UAV Pose"], gt["Target Pose"]
    n = min(len(up), len(tp), len(Tg)); Tg = Tg[:n]
    uz = np.array([p.position.z for p in up[:n]])
    ex = np.array([p.position.x for p in up[:n]]) - np.array([p.position.x for p in tp[:n]])
    ey = np.array([p.position.y for p in up[:n]]) - np.array([p.position.y for p in tp[:n]])
    Tg, uz, ex, ey = dedup(Tg, uz, ex, ey)
    m = (uz > 0.3) & (uz < 4.8)
    tE = Tg[m]
    zC = ex[m] + 1j*ey[m]                                   # ENU complex error
    W, rr, coef = fit_cycle(tE, zC)
    b0, b1, B, A = coef
    var0 = float(np.mean(np.abs(zC - zC.mean())**2))
    out = dict(rep=os.path.basename(d.rstrip("/")), W=W, A=abs(A), B=abs(B),
               b0=abs(b0), resid=rr/var0)

    # --- stage signals; NED x = ENU y (north), NED y = ENU x (east)
    e_n = ey[m]; e_e = ex[m]                                # NED components of GT error
    ct = np.asarray(cd["t"], float); tc = ct - ct[0]
    Ia_raw = np.asarray(cd["I_a_raw(t)"], float)
    Ia = np.asarray(cd["I_a(t)"], float)
    wu = np.asarray(cd["w_u(t)"], float)
    mc = (tc >= tE[0]) & (tc <= tE[-1])

    ot = np.asarray(td["Odometry Timestamp"], float) - ct[0]
    quats = td["Quaternion"]
    Rall = np.array([quat_to_R([q.w, q.x, q.y, q.z]) for q in quats])
    vB = np.array([[v.x_m_s, v.y_m_s, v.z_m_s] for v in td["Velocity Body"]])
    vW = np.einsum("nij,nj->ni", Rall, vB)
    mo = (ot >= tE[0]) & (ot <= tE[-1])
    it_ = np.asarray(td["IMU Timestamp"], float) - ct[0]
    gyr = np.array([[w.forward_rad_s, w.right_rad_s, w.down_rad_s]
                    for w in td["Angular Velocity FRD"]])
    mi = (it_ >= tE[0]) & (it_ <= tE[-1])
    g = 9.80665

    stages = {}
    for ax, e_sig, tilt_col, wu_i, gy_i in [("x", e_n, 0, 1, 1), ("y", e_e, 1, 0, 0)]:
        C_e = tone_real(tE, e_sig, W)
        C_Ir = tone_real(tc[mc], Ia_raw[mc, 0 if ax == "x" else 1], W)
        C_Ia = tone_real(tc[mc], Ia[mc, 0 if ax == "x" else 1], W)
        C_tl = tone_real(ot[mo], -g * Rall[mo, tilt_col, 2], W)
        C_v = tone_real(ot[mo], vW[mo, 0 if ax == "x" else 1], W)
        C_ad = 1j * W * C_v
        C_wu = tone_real(tc[mc], wu[mc, wu_i], W)
        C_gy = tone_real(it_[mi], gyr[mi, gy_i], W)
        ph = lambda Cb, Ca: float(np.degrees(np.angle(Cb / Ca))) if abs(Ca) > 1e-9 else np.nan
        gn = lambda Cb, Ca: float(abs(Cb) / abs(Ca)) if abs(Ca) > 1e-9 else np.nan
        stages[ax] = {
            "ctl e->Iar":   (ph(C_Ir, C_e),  gn(C_Ir, C_e)),
            "tau Iar->Ia":  (ph(C_Ia, C_Ir), gn(C_Ia, C_Ir)),
            "att Ia->tilt": (ph(C_tl, C_Ia), gn(C_tl, C_Ia)),
            "geo tilt->ad": (ph(C_ad, C_tl), gn(C_ad, C_tl)),
            "ACT Iar->ad":  (ph(C_ad, C_Ir), gn(C_ad, C_Ir)),
            "rate wu->gy":  (ph(C_gy, C_wu), gn(C_gy, C_wu)),
            "closure e->ad":(ph(C_ad, C_e),  gn(C_ad, C_e)),
            "amps": (abs(C_e), abs(C_Ir), abs(C_ad)),
        }
    out["stages"] = stages
    return out

if __name__ == "__main__":
    pats = sys.argv[1:] or [
        "test_data/Rover_Turning/yawhold_arm_n3/*/",
        "test_data/Rover_Turning/dhd_src_sweep/dhd_nokr/*/",
        "test_data/Rover_Turning/cycle_gain_sweep/hdkr_0/*/",
    ]
    for pat in pats:
        for d in sorted(glob.glob(pat)):
            try:
                r = rep_deepdive(d)
            except Exception as e:
                print(f"{d}: ERR {e}")
                continue
            print(f"\n### {r['rep']}  W*={r['W']:.2f} rad/s | cycle A={r['A']:.2f} m, "
                  f"orbit-bias B={r['B']:.2f} m, DC={r['b0']:.2f} m, resid/var={r['resid']:.2f}")
            for ax in ("x", "y"):
                s = r["stages"][ax]
                amps = s.pop("amps")
                line = "  ".join(f"{k} {v[0]:+6.1f}d/{v[1]:4.2f}g" for k, v in s.items())
                print(f"  [{ax}] |e|={amps[0]:.2f} |Iar|={amps[1]:.2f} |ad|={amps[2]:.2f}")
                print(f"  [{ax}] {line}")
