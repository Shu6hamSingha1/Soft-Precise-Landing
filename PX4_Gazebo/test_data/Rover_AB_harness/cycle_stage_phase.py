#!/usr/bin/env python3
"""Deep-dive v3: complex rotating-phasor stage fits (x+iy as one signal) + authority audit.

Model per stage signal S(t) = s_x + i s_y:   S ~ b0 + b1 t + B e^{i WZ t} + A e^{i W* t}
with W* from the GT-error fit. Stage phase/gain = A ratios (leakage cancels in ratios).
Authority: cone duty (theta_current vs theta_cone), |I_a|/|I_a_raw| crush, delivered
oscillatory accel |A_ad| (predicted ~constant), cycle power P = 0.5 Re[A_ad conj(A_v)].
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

def cfit(t, Z, W, tref):
    """3-column complex LS: b0 + b1 t + A e^{iWt} (trend absorbs the slow WZ bias drift)."""
    t0 = t - tref
    M = np.column_stack([np.ones_like(t0), t0, np.exp(1j*W*t0)])
    coef, *_ = np.linalg.lstsq(M, Z, rcond=None)
    r = Z - M @ coef
    return coef, float(np.mean(np.abs(r)**2))

def fit_W(t, Z, tref, grid=np.arange(0.9, 3.2, 0.025)):
    best = None
    for W in grid:
        coef, rr = cfit(t, Z, W, tref)
        if best is None or rr < best[2]:
            best = (W, coef, rr)
    return best

def bias_fit(t, Z, W, A, tref):
    """Orbit-locked bias from the cycle-removed residual: b0 + b1 t + B e^{iWZ t}."""
    t0 = t - tref
    Zr = Z - A * np.exp(1j*W*t0)
    M = np.column_stack([np.ones_like(t0), t0, np.exp(1j*WZ*t0)])
    coef, *_ = np.linalg.lstsq(M, Zr, rcond=None)
    return coef

def rep(d):
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
    tref = float(tE.mean())
    # ENU complex convention THROUGHOUT (east + i north); v1-proven (+W rotation).
    Ze = ex[m] + 1j * ey[m]
    W, coefE, rrE = fit_W(tE, Ze, tref)
    A_e = coefE[2]
    B_e = bias_fit(tE, Ze, W, A_e, tref)[2]
    var0 = float(np.mean(np.abs(Ze - Ze.mean())**2))

    ct = np.asarray(cd["t"], float); tc = ct - ct[0]
    mc = (tc >= tE[0]) & (tc <= tE[-1])
    Iar = np.asarray(cd["I_a_raw(t)"], float); Ia = np.asarray(cd["I_a(t)"], float)
    # NED (n,e) -> ENU complex: east + i north
    Z_iar = Iar[mc, 1] + 1j * Iar[mc, 0]
    Z_ia = Ia[mc, 1] + 1j * Ia[mc, 0]
    thc = np.asarray(cd["theta_current(t)"], float)[mc]
    thk = np.asarray(cd["theta_cone(t)"], float)[mc]

    ot = np.asarray(td["Odometry Timestamp"], float) - ct[0]
    mo = (ot >= tE[0]) & (ot <= tE[-1])
    quats = td["Quaternion"]
    Rall = np.array([quat_to_R([q.w, q.x, q.y, q.z]) for q in quats])
    vB = np.array([[v.x_m_s, v.y_m_s, v.z_m_s] for v in td["Velocity Body"]])
    vW = np.einsum("nij,nj->ni", Rall, vB)
    g = 9.80665
    # NED components -> ENU complex (east + i north)
    Z_tilt = (-g) * (Rall[mo, 1, 2] + 1j * Rall[mo, 0, 2])
    Z_v = vW[mo, 1] + 1j * vW[mo, 0]

    cIar, _ = cfit(tc[mc], Z_iar, W, tref)
    cIa,  _ = cfit(tc[mc], Z_ia, W, tref)
    cTl,  _ = cfit(ot[mo], Z_tilt, W, tref)
    cV,   _ = cfit(ot[mo], Z_v, W, tref)
    A_iar, A_ia, A_tl, A_v = cIar[2], cIa[2], cTl[2], cV[2]
    A_ad = 1j * W * A_v

    def st(Cb, Ca):
        if abs(Ca) < 1e-9: return (np.nan, np.nan)
        return (float(np.degrees(np.angle(Cb / Ca))), float(abs(Cb / Ca)))

    cone_duty = float(np.mean(thc >= 0.95 * thk))
    crush = float(np.median(np.linalg.norm(Ia[mc, :2], axis=1))
                  / max(np.median(np.linalg.norm(Iar[mc, :2], axis=1)), 1e-9))
    # cycle power delivered into the cycle velocity (per unit mass): P=0.5 Re[A_ad conj(A_vcyc)]
    A_vcyc = 1j * W * A_e
    P_cyc = 0.5 * float(np.real(A_ad * np.conj(A_vcyc)))
    return dict(rep=os.path.basename(d.rstrip("/")), W=W, resid=rrE/var0,
                A=abs(A_e), B=abs(B_e), DC=abs(coefE[0]),
                seed_note="",
                a_osc=abs(A_ad), AW2=abs(A_e)*W*W,
                ctl=st(A_iar, A_e), sat=st(A_ia, A_iar), att=st(A_tl, A_ia),
                geo=st(A_ad, A_tl), act=st(A_ad, A_iar), loop=st(A_ad, A_e),
                cone=cone_duty, crush=crush, P=P_cyc,
                iar_med=float(np.median(np.linalg.norm(Iar[mc, :2], axis=1))),
                ia_med=float(np.median(np.linalg.norm(Ia[mc, :2], axis=1))))

if __name__ == "__main__":
    pats = sys.argv[1:] or [
        "test_data/Rover_Turning/yawhold_arm_n3/*/",
        "test_data/Rover_Turning/dhd_src_sweep/dhd_nokr/*/",
        "test_data/Rover_Turning/cycle_gain_sweep/hdkr_0/*/",
    ]
    rows = []
    for pat in pats:
        for d in sorted(glob.glob(pat)):
            try:
                r = rep(d)
            except Exception as e:
                print(f"{d}: ERR {e}"); continue
            rows.append(r)
            print(f"{r['rep'][-13:]}: W*={r['W']:.2f} A={r['A']:.2f} B={r['B']:.2f} DC={r['DC']:.2f} "
                  f"resid={r['resid']:.2f} | a_osc={r['a_osc']:.2f} AW^2={r['AW2']:.2f} "
                  f"P_cyc={r['P']:+.2f} | cone {100*r['cone']:.0f}% crush {r['crush']:.2f} "
                  f"|Iar|med {r['iar_med']:.1f} -> |Ia|med {r['ia_med']:.1f}")
            print(f"    ctl(e->Iar) {r['ctl'][0]:+6.1f}d x{r['ctl'][1]:.1f} | sat(Iar->Ia) {r['sat'][0]:+6.1f}d x{r['sat'][1]:.2f} | "
                  f"att(Ia->tilt) {r['att'][0]:+6.1f}d x{r['att'][1]:.2f} | geo(tilt->ad) {r['geo'][0]:+6.1f}d x{r['geo'][1]:.2f}")
            print(f"    ACT(Iar->ad) {r['act'][0]:+6.1f}d x{r['act'][1]:.2f} | LOOP(e->ad) {r['loop'][0]:+6.1f}d x{r['loop'][1]:.1f}")
    if rows:
        med = lambda k: np.median([r[k] for r in rows])
        medst = lambda k: (np.median([r[k][0] for r in rows]), np.median([r[k][1] for r in rows]))
        print(f"\nMEDIANS (n={len(rows)}): W*={med('W'):.2f} A={med('A'):.2f} B={med('B'):.2f} "
              f"a_osc={med('a_osc'):.2f} AW^2={med('AW2'):.2f} cone={100*med('cone'):.0f}% crush={med('crush'):.2f}")
        for k in ("ctl", "sat", "att", "geo", "act", "loop"):
            p, gn = medst(k)
            print(f"  {k}: {p:+6.1f} deg  x{gn:.2f}")
