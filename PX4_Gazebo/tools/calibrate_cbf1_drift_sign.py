#!/usr/bin/env python3
"""Sign calibration for cbf1's L_omega DRIFT term  d = cr_dot_obs - L_omega @ omega_rp.

For d to be the EXOGENOUS drift, L_omega @ omega_rp must reproduce the ROTATIONAL part of the
measured centroid velocity cr_dot_obs. The open convention: does the body [roll, pitch] rate
(self._w[:2] = IMU AngVel[:2]) map to L_omega's [omega_x, omega_y] columns with what swap/sign?

Calibrate on the CAL data (multisine excitation = rotation-dominated, marker ~stationary in world,
so cr_dot_obs ~ L_omega @ omega -- no translation to contaminate). Everything is in Img_Data on ONE
clock: raw corners (-> camera centroid cr, tangent), IMU AngVel (body rate), FPS. For each candidate
map M(omega), score the directional match cos(L_omega @ M(omega), cr_dot_obs), weighted by |omega|.
The map ~ +1 is the calibration. Read-only.

Usage: calibrate_cbf1_drift_sign.py [glob ...]   (default: calibration_data/output/*/)
"""
import numpy as np, glob, os, sys

CENTER = np.array([240.0, 320.0]); FOCAL = np.array([270.0, 270.0])   # (sign-cal is focal-independent)
MAPS = [(sw, sx, sy) for sw in (0, 1) for sx in (1, -1) for sy in (1, -1)]


def apply_map(w, sw, sx, sy):
    if sw: w = w[..., ::-1]
    return w * np.array([sx, sy])


def load(d):
    img = np.load(os.path.join(d, 'Img_Data.npy'), allow_pickle=True).item()
    fp = img.get('Image Feature Pts'); ang = img.get('IMU AngVel'); fps = img.get('FPS')
    if fp is None or ang is None or fps is None: return None
    n = min(len(fp), len(ang), len(fps))
    cr, crd, w = [], [], []
    for i in range(n):
        pair = np.asarray(fp[i], float)               # (2,4,2) = [prev, curr]
        if pair.shape != (2, 4, 2): continue
        c1 = (pair[1].mean(0) - CENTER) / FOCAL        # cr (curr), tangent
        c0 = (pair[0].mean(0) - CENTER) / FOCAL        # cr (prev)
        f = float(fps[i])
        if not np.isfinite(f) or f <= 0: continue
        cr.append(c1); crd.append((c1 - c0) * f); w.append(np.asarray(ang[i], float)[:2])
    if len(cr) < 50: return None
    return np.array(cr), np.array(crd), np.array(w)


def main():
    g = sys.argv[1:] or ['calibration_data/output/*/']
    dirs = []
    for gg in g:
        dirs += [d for d in glob.glob(gg) if os.path.exists(os.path.join(d, 'Img_Data.npy'))]
    CR, CRD, W = [], [], []
    used = 0
    for d in sorted(set(dirs)):
        r = load(d)
        if r is None: continue
        CR.append(r[0]); CRD.append(r[1]); W.append(r[2]); used += 1
    if not CR:
        print("no usable cal recordings"); return
    cr, crd, w = np.vstack(CR), np.vstack(CRD), np.vstack(W)
    wn = np.linalg.norm(w, axis=1); cn = np.linalg.norm(crd, axis=1)
    m = (wn > 0.15) & (cn > 1e-3) & np.all(np.isfinite(cr), 1) & np.all(np.isfinite(crd), 1)
    cr, crd, w, wn = cr[m], crd[m], w[m], wn[m]
    cn = np.linalg.norm(crd, axis=1)                    # recompute on masked frames
    x, y = cr[:, 0], cr[:, 1]
    # L_omega @ v, per frame, vectorized:  [xy*vx -(1+x^2)*vy ; (1+y^2)*vx - xy*vy]
    def Lw_apply(v):
        return np.stack([x * y * v[:, 0] - (1 + x * x) * v[:, 1],
                         (1 + y * y) * v[:, 0] - x * y * v[:, 1]], axis=1)
    print(f"{used} cal recordings, {m.sum()} rotation-dominated frames (|omega|>0.15 rad/s)\n")
    print(f"  {'map (SWAP,SIGN_X,SIGN_Y)':30s} {'cos':>7} {'slope':>7}")
    rows = []
    for (sw, sx, sy) in MAPS:
        pred = Lw_apply(apply_map(w, sw, sx, sy))
        pn = np.linalg.norm(pred, axis=1) + 1e-12
        cos = float(np.average(np.sum(pred * crd, 1) / (pn * cn), weights=wn))
        slope = float(np.average(pn / cn, weights=wn))
        rows.append((cos, slope, sw, sx, sy))
    best = max(rows, key=lambda r: r[0])
    for cos, slope, sw, sx, sy in sorted(rows, key=lambda r: -r[0]):
        tag = "  <== BEST" if (cos, slope, sw, sx, sy) == best else ""
        print(f"  SWAP={sw} SIGN_X={sx:+d} SIGN_Y={sy:+d}          {cos:+7.3f} {slope:7.2f}{tag}")
    print(f"\n  => omega->L_omega map: SWAP={best[2]} SIGN_X={best[3]:+d} SIGN_Y={best[4]:+d}"
          f"  (cos={best[0]:+.2f}, slope~{best[1]:.2f})")
    print("  code does `L_w @ self._w[:2]` = no-swap,+1,+1; if BEST differs, add an omega map to the drift.")


if __name__ == '__main__':
    main()
