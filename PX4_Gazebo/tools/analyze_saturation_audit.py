#!/usr/bin/env python3
"""
Saturation audit: for each landing rep, measure the DUTY CYCLE (% of flight
time active) of every saturation/limit in the control code — both the
manuscript's own design limits and the guards we added.

Methodology (user, 2026-06-03): instead of only root-causing failures, find
which limits are active (even in successful reps), trace which control
parameter drives the signal into each limit, and tune that parameter so the
signal stays in its linear regime. Active limits = silent performance loss.

Limits audited (source, value):
  CODE-ADDED GUARDS (we put these in; activity = controller overridden):
    w_u clamp        PLASMC_W_U_MAX = 1.0 rad/s      controller.py
    w_i clamp        W_I_MAX = 5.0 rad/s             controller.py (hardcoded)
    dh_d clamp       PLASMC_DH_D_MAX = 50            controller.py
    PID  ∫ clamp     iV_s_e_n_clamp = 5.0 (norm)     controller.py (hardcoded)
    yaw  ∫ clamp     ie_a_clamp = 2.0                controller.py (hardcoded)
    thrust clip      thrust_norm ∈ [0,1]             landing_test.py
  MANUSCRIPT DESIGN LIMITS (activity = operating outside the designed regime):
    ζ barrier        ε_S = 0.05 → |ζ| ≤ 3.66         funnel (Remark 5)
    ∫ζ clamp         izeta = ±5                      Supplement S2-D
    boundary layer   |σ/ℰ| ≥ 1 (switching mode)      SMC sat() function
    cone clamp       a_xy ≤ |a_z|·tan(θ_cone)        acceleration conditioning
    accel floor      I_a_z ≥ 0 → −3.0                Supplement S1.5
    accel max        I_a_z ≥ −50                     MATLAB l.464

Output: per-rep duty-cycle table + aggregate ranking + SP-vs-non-SP contrast.

Usage:
    python3 tools/analyze_saturation_audit.py <rep_dir> [<rep_dir> ...]
    python3 tools/analyze_saturation_audit.py --glob 'test_data/SPCampaign/b10C*/rep[0-9]'
"""
from __future__ import annotations
import argparse
import glob as globmod
import os
import sys

import numpy as np

W_U_MAX = 1.0
W_I_MAX = 5.0
DH_D_MAX = 50.0
IS_EN_CLAMP = 5.0
IE_A_CLAMP = 2.0
IZETA_CLAMP = 5.0
ZETA_MAX = np.log((2.0 - 0.05) / 0.05)      # 3.66 (eps_S = 0.05)
HOVER_THRUST = 0.738
THRUST_SLOPE = 42.3


def _arr(seq):
    return np.asarray([np.asarray(v, dtype=float) for v in seq])


def audit_rep(rep_dir, sat_thresh=0.98):
    cd = np.load(os.path.join(rep_dir, "Control_Data.npy"), allow_pickle=True).item()
    cp = np.load(os.path.join(rep_dir, "Control_Params.npy"), allow_pickle=True).item()
    gt_path = os.path.join(rep_dir, "Ground_Truth.npy")
    sp = {}
    if os.path.exists(gt_path):
        gt = np.load(gt_path, allow_pickle=True).item()
        sp = gt.get("SoftPrecise", {}) or {}

    t = np.asarray(cd["t"], dtype=float)
    if t.size < 20:
        return None
    t = t - t[0]

    g = lambda k: _arr(cd[k])
    w_u   = g("w_u(t)");   w_i  = g("w_i(t)");  dh_d  = g("dh_d(t)")
    is_en = g("is_e_n(t)"); ie_a = np.asarray(cd["ie_a(t)"], dtype=float)
    zeta  = g("zeta(t)");  izeta = g("izeta(t)"); sigma = g("sigma(t)")
    a_u   = g("a_u(t)");   I_a   = g("I_a(t)");   I_ar  = g("I_a_raw(t)")
    B_T   = np.asarray(cd["B_T(t)"], dtype=float)
    thc   = np.asarray(cd["theta_cone(t)"], dtype=float)
    E     = np.diag(np.asarray(cp.get("E", np.eye(3))))

    m = min(len(t), len(w_u), len(w_i), len(dh_d), len(is_en), len(ie_a),
            len(zeta), len(izeta), len(sigma), len(a_u), len(I_a), len(I_ar),
            len(B_T), len(thc))

    duty = {}
    th = sat_thresh

    # ── code-added guards ──────────────────────────────────────────────
    duty["w_u clamp (1.0)"]     = np.mean(np.abs(w_u[:m]).max(axis=1) >= th * W_U_MAX)
    duty["w_i clamp (5.0)"]     = np.mean(np.abs(w_i[:m]).max(axis=1) >= th * W_I_MAX)
    duty["dh_d clamp (50)"]     = np.mean(np.abs(dh_d[:m]).max(axis=1) >= th * DH_D_MAX)
    duty["PID int clamp (5)"]   = np.mean(np.linalg.norm(is_en[:m], axis=1) >= th * IS_EN_CLAMP)
    duty["yaw int clamp (2)"]   = np.mean(np.abs(ie_a[:m]) >= th * IE_A_CLAMP)
    thrust_norm = HOVER_THRUST - B_T[:m] / THRUST_SLOPE
    duty["thrust clip [0,1]"]   = np.mean((thrust_norm <= 0.005) | (thrust_norm >= 0.995))

    # ── manuscript design limits ───────────────────────────────────────
    duty["zeta barrier (3.66)"] = np.mean(np.abs(zeta[:m]).max(axis=1) >= th * ZETA_MAX)
    duty["int-zeta clamp (5)"]  = np.mean(np.abs(izeta[:m]).max(axis=1) >= th * IZETA_CLAMP)
    duty["sigma outside E"]     = np.mean((np.abs(sigma[:m]) / E[None, :]).max(axis=1) >= 1.0)
    ask  = np.linalg.norm(I_ar[:m][:, :2], axis=1)
    alw  = np.abs(I_ar[:m][:, 2]) * np.tan(thc[:m])
    duty["cone clamp"]          = np.mean((ask > alw * 1.02) & (ask > 0.5))
    duty["accel floor (-3)"]    = np.mean(I_ar[:m][:, 2] >= 0.0)
    duty["accel max (-50)"]     = np.mean(I_a[:m][:, 2] <= -49.0)

    is_sp = bool(sp.get("precise")) and bool(sp.get("soft"))
    return dict(rep=os.path.basename(rep_dir.rstrip("/")),
                bundle=os.path.basename(os.path.dirname(rep_dir.rstrip("/"))),
                duty=duty, T=float(t[m - 1]),
                xy=sp.get("xy_err"), vel=sp.get("rel_vel"), sp=is_sp)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("reps", nargs="*")
    ap.add_argument("--glob", action="append", default=[])
    args = ap.parse_args()

    reps = list(args.reps)
    for pat in args.glob:
        reps += sorted(globmod.glob(pat))
    reps = [r for r in reps if os.path.isdir(r)]
    if not reps:
        print("No rep directories found."); sys.exit(1)

    audits = []
    for rep in reps:
        try:
            a = audit_rep(rep)
            if a: audits.append(a)
        except Exception as e:
            print(f"  [error] {rep}: {type(e).__name__} {e}")

    if not audits:
        sys.exit(1)

    limits = list(audits[0]["duty"].keys())

    # ── per-rep table ───────────────────────────────────────────────────
    print(f"\n{'rep':<28} {'xy':>6} {'vel':>6} {'SP':>3} | " +
          " ".join(f"{l.split('(')[0].strip()[:9]:>9}" for l in limits))
    for a in audits:
        xy = f"{a['xy']:.3f}" if a['xy'] is not None else "  -  "
        vel = f"{a['vel']:.3f}" if a['vel'] is not None else "  -  "
        print(f"{a['bundle'][:12]+'/'+a['rep']:<28} {xy:>6} {vel:>6} {'*' if a['sp'] else '':>3} | " +
              " ".join(f"{100*a['duty'][l]:>8.1f}%" for l in limits))

    # ── aggregate: SP vs non-SP duty cycles ─────────────────────────────
    sp_reps  = [a for a in audits if a["sp"]]
    nsp_reps = [a for a in audits if not a["sp"]]
    print(f"\n{'='*100}")
    print(f"AGGREGATE duty cycle (% of flight time each limit is active)")
    print(f"{'limit':<24} {'SP reps (n=%d)' % len(sp_reps):>16} {'non-SP (n=%d)' % len(nsp_reps):>16} {'ratio':>8}")
    rows = []
    for l in limits:
        sp_d  = np.mean([a["duty"][l] for a in sp_reps]) * 100 if sp_reps else 0.0
        nsp_d = np.mean([a["duty"][l] for a in nsp_reps]) * 100 if nsp_reps else 0.0
        ratio = nsp_d / sp_d if sp_d > 0.05 else (np.inf if nsp_d > 0.05 else 1.0)
        rows.append((l, sp_d, nsp_d, ratio))
    for l, sp_d, nsp_d, ratio in sorted(rows, key=lambda r: -r[2]):
        rs = f"{ratio:.1f}x" if np.isfinite(ratio) else "inf"
        print(f"{l:<24} {sp_d:>15.1f}% {nsp_d:>15.1f}% {rs:>8}")


if __name__ == "__main__":
    main()
