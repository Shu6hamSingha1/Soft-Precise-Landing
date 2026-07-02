#!/usr/bin/env python3
"""Per-branch tone audit: who supplies damping lead (chi_i) and who erodes it.

a_u = -G^-1 a_v with a_v = -Gamma sigma - theta.(sat G kappa) + G(-c + S dp) - chi_zeta_aug
Branches of a_u (per lateral axis, diagonal G):
  reach   = (Gamma sigma)_i / g_i
  switch  = theta_i sat_i kappa_i
  transport = +c_transport_i   (CH_CLEAN: -psi_dot_b (e3 x h); ~0 under heading-hold)
  loom    = +c_loom_i          (-loom_scale (h.e3) h)
  c3      = -dh_d_i            (enters via c)
  Sdp     = -(S dp)_i
  drift   = (chi_zeta_aug)_i / g_i    [backed out exactly from logged a_v]
Sum == logged a_u (validated). Tone-fit each branch (complex over lateral pair),
report gain share + phase relative to the total a_u tone, plus chi_i in the world
anti-position frame (chi = lead beyond anti-position, +90 = pure damping).
"""
import numpy as np, os, sys, glob

def dedup(t, *sig):
    keep = np.concatenate([[True], np.diff(t) > 1e-9])
    return (t[keep],) + tuple(s[keep] for s in sig)

def cfit(t, Z, W, tref):
    t0 = t - tref
    M = np.column_stack([np.ones_like(t0), t0, np.exp(1j*W*t0)])
    coef, *_ = np.linalg.lstsq(M, Z, rcond=None)
    r = Z - M @ coef
    return coef[2], float(np.mean(np.abs(r)**2))

def fit_W(t, Z, tref, grid=np.arange(0.9, 3.2, 0.025)):
    best = None
    for W in grid:
        A, rr = cfit(t, Z, W, tref)
        if best is None or rr < best[2]:
            best = (W, A, rr)
    return best

def rep(d):
    gt = np.load(os.path.join(d, "Ground_Truth.npy"), allow_pickle=True).item()
    cd = np.load(os.path.join(d, "Control_Data.npy"), allow_pickle=True).item()
    cp = np.load(os.path.join(d, "Control_Params.npy"), allow_pickle=True).item()
    Tg = np.asarray(gt["Time"], float)
    up, tp = gt["UAV Pose"], gt["Target Pose"]
    n = min(len(up), len(tp), len(Tg)); Tg = Tg[:n]
    uz = np.array([p.position.z for p in up[:n]])
    ex = np.array([p.position.x for p in up[:n]]) - np.array([p.position.x for p in tp[:n]])
    ey = np.array([p.position.y for p in up[:n]]) - np.array([p.position.y for p in tp[:n]])
    Tg, uz, ex, ey = dedup(Tg, uz, ex, ey)
    m = (uz > 0.3) & (uz < 4.8)
    tE = Tg[m]; tref = float(tE.mean())
    Ze = ex[m] + 1j*ey[m]
    W, A_e, rrE = fit_W(tE, Ze, tref)

    ct = np.asarray(cd["t"], float); tc = ct - ct[0]
    mc = (tc >= tE[0]) & (tc <= tE[-1])
    G = np.asarray(cd["G(t)"], float)          # (n,3,3) diag
    S = np.asarray(cd["S(t)"], float)
    dp = np.asarray(cd["dp(t)"], float)
    sig = np.asarray(cd["sigma(t)"], float)
    kap = np.asarray(cd["kappa(t)"], float)[:len(sig)]
    dhd = np.asarray(cd["dh_d(t)"], float)
    h = np.asarray(cd["h(t)"], float)
    wI = np.asarray(cd["w(t)"], float)         # body IMU rate (for psi_dot_b approx)
    a_u = np.asarray(cd["a_u(t)"], float)
    a_v = np.asarray(cd["a_v(t)"], float)
    E = np.diag(np.asarray(cp["E"], float))
    Gam = np.asarray(cp["Gamma"], float)

    g_diag = np.array([G[:, i, i] for i in range(3)]).T          # (n,3)
    sat = np.clip(sig / E[None, :], -1.0, 1.0)
    Sdp = np.einsum("nij,nj->ni", S, dp)
    # c reconstruction (CH_CLEAN baked): transport + loom + c3
    loom = -1.0 * (h[:, 2:3]) * h                                # -(h.e3) h  (loom_scale=1)
    # transport: -psi_dot_b (e3 x h); heading-hold -> psi_dot_b ~ -w_z, small
    psid = -wI[:, 2]
    e3xh = np.stack([-h[:, 1], h[:, 0], np.zeros(len(h))], axis=1)
    transport = -psid[:, None] * e3xh
    c = transport + loom - dhd
    # theta per-axis: vector = -c + S dp - G^-1 chi_aug ; chi_aug backed out from a_v:
    Gneg_c_Sdp = np.einsum("nij,nj->ni", G, (-c + Sdp))
    theta_sw_term = None
    # chi_aug = -a_v - Gam sig - theta.(sat G kappa) + G(-c+Sdp) ... but theta needs chi_aug.
    # Iterate twice (theta weakly depends on chi_aug via sqrt(vec^2+1)).
    chi_aug = np.zeros_like(c)
    for _ in range(3):
        Ginv_chi = chi_aug / g_diag
        vec = (-c + Sdp - Ginv_chi)
        theta = np.sqrt(vec**2 + 1.0)
        sw_v = theta * sat * g_diag * kap                        # in a_v units
        chi_aug = -a_v - (Gam @ sig.T).T - sw_v + Gneg_c_Sdp
    # branches of a_u:
    reach = (Gam @ sig.T).T / g_diag
    switch = theta * sat * kap
    drift = chi_aug / g_diag
    br = {
        "reach": reach, "switch": switch, "transport": transport,
        "loom": loom, "c3(-dh_d)": -dhd, "-Sdp": -Sdp, "drift": drift,
    }
    a_rec = sum(b for b in br.values())
    resid = float(np.median(np.linalg.norm(a_rec[mc, :2] - a_u[mc, :2], axis=1)
                            / np.maximum(np.linalg.norm(a_u[mc, :2], axis=1), 1e-6)))

    # tones (V-frame complex x+iy, resolve conjugation against world Iar tone)
    Iar = np.asarray(cd["I_a_raw(t)"], float)
    Z_iar = Iar[mc, 1] + 1j*Iar[mc, 0]
    A_iar, _ = cfit(tc[mc], Z_iar, W, tref)
    Zau_p = a_u[mc, 0] + 1j*a_u[mc, 1]
    Zau_c = np.conj(Zau_p)
    Ap, _ = cfit(tc[mc], Zau_p, W, tref)
    Ac, _ = cfit(tc[mc], Zau_c, W, tref)
    conj = abs(Ac) > abs(Ap)
    A_au = Ac if conj else Ap
    chi_total = (np.degrees(np.angle(A_iar / A_e)) - 180.0 + 540) % 360 - 180
    rows = []
    for name, b in br.items():
        Zb = np.conj(b[mc, 0] + 1j*b[mc, 1]) if conj else (b[mc, 0] + 1j*b[mc, 1])
        A_b, _ = cfit(tc[mc], Zb, W, tref)
        rel = (np.degrees(np.angle(A_b / A_au)) + 540) % 360 - 180
        chi_i = ((chi_total + rel) + 540) % 360 - 180
        rows.append((name, abs(A_b)/max(abs(A_au), 1e-9), rel, chi_i))
    return dict(rep=os.path.basename(d.rstrip("/")), W=W, resid=resid, conj=conj,
                chi_total=chi_total, rows=rows, A=abs(A_e))

if __name__ == "__main__":
    pats = sys.argv[1:] or [
        "test_data/Rover_Turning/yawhold_arm_n3/*/",
        "test_data/Rover_Turning/dhd_src_sweep/dhd_nokr/*/",
        "test_data/Rover_Turning/cycle_gain_sweep/hdkr_0/*/",
    ]
    allrows = {}
    for pat in pats:
        for d in sorted(glob.glob(pat)):
            try:
                r = rep(d)
            except Exception as e:
                print(f"{d}: ERR {e}"); continue
            print(f"\n### {r['rep'][-13:]} W={r['W']:.2f} A={r['A']:.2f} recon_resid={r['resid']:.2%} "
                  f"conj={r['conj']} chi_total={r['chi_total']:+.0f}d")
            for name, gshare, rel, chi in sorted(r["rows"], key=lambda x: -x[1]):
                print(f"   {name:12s} |A_i|/|A_au|={gshare:5.2f}  rel={rel:+6.1f}d  chi_i={chi:+6.1f}d")
                allrows.setdefault(name, []).append((gshare, chi))
    print("\n=== MEDIANS across reps ===")
    for name, v in sorted(allrows.items(), key=lambda kv: -np.median([x[0] for x in kv[1]])):
        gs = np.median([x[0] for x in v]); ch = np.median([x[1] for x in v])
        print(f"  {name:12s} share {gs:5.2f}   chi {ch:+6.1f} deg   (n={len(v)})")
