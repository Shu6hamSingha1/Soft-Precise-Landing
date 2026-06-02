#!/usr/bin/env python3
"""Deep dive: why doesn't the controller achieve PRECISE reliably?

Trace the lateral-position trajectory through descent for representative
reps — PRECISE-only (xy<0.08, high vel), near-PRECISE (xy<0.15), SP, and
representative failure modes.  Look for:
  - WHEN in the descent does lateral position settle (or fail to)
  - Is the failure due to: (a) lateral velocity peak too high, (b) tracking
    error during descent, or (c) image-Jacobian singularity at touchdown
  - What's the kinematic budget — how much position can the drone close
    given altitude h(t), descent rate v_z, available lateral accel?
"""
from __future__ import annotations
import glob, os, sys
import numpy as np


def trace_rep(rep_dir: str) -> dict:
    td = np.load(os.path.join(rep_dir, "Telemetry_Data.npy"), allow_pickle=True).item()
    cd = np.load(os.path.join(rep_dir, "Control_Data.npy"), allow_pickle=True).item()
    gt = np.load(os.path.join(rep_dir, "Ground_Truth.npy"), allow_pickle=True).item()

    # Time bases differ — use controller time as reference
    t_ctrl = np.asarray(cd["t"], dtype=float)
    t_ctrl = t_ctrl - t_ctrl[0]
    n = len(t_ctrl)

    # Telemetry: resample onto controller time
    n_td = len(td["Position Body"])
    t_td = np.linspace(t_ctrl[0], t_ctrl[-1], n_td)
    pos = np.array([(p.x_m, p.y_m, p.z_m) for p in td["Position Body"]])
    vel = np.array([(v.x_m_s, v.y_m_s, v.z_m_s) for v in td["Velocity Body"]])
    pos_r = np.array([np.interp(t_ctrl, t_td, pos[:, k]) for k in range(3)]).T
    vel_r = np.array([np.interp(t_ctrl, t_td, vel[:, k]) for k in range(3)]).T

    # Altitude above ground (assume target z=0)
    alt = -pos_r[:, 2]
    # Lateral position offset from target (target at origin in PX4 since IC=0,0,5)
    lateral = np.linalg.norm(pos_r[:, :2], axis=1)
    vh = np.linalg.norm(vel_r[:, :2], axis=1)
    vz = vel_r[:, 2]

    # Control signals
    s_e_n = np.array([np.asarray(x) for x in cd["s_e_n(t)"]])
    sen_mag = np.linalg.norm(s_e_n, axis=1)
    sigma = np.array([np.asarray(x) for x in cd["sigma(t)"]])
    sxy = np.linalg.norm(sigma[:, :2], axis=1)
    Ia = np.array([np.asarray(x) for x in cd["I_a(t)"]])
    Ia_xy = np.linalg.norm(Ia[:, :2], axis=1)
    ds_d = np.array([np.asarray(x) for x in cd["ds_d(t)"]])
    dsd_xy = np.linalg.norm(ds_d[:, :2], axis=1)

    sp = gt["SoftPrecise"]
    return {
        "rep_dir": rep_dir,
        "xy_end": float(sp["xy_err"]),
        "vel_end": float(sp["rel_vel"]),
        "t": t_ctrl, "alt": alt, "lateral": lateral, "vh": vh, "vz": vz,
        "sen_mag": sen_mag, "sxy": sxy, "Ia_xy": Ia_xy, "dsd_xy": dsd_xy,
    }


def kinematic_budget(rep: dict) -> dict:
    """Compute time-to-touchdown vs lateral-distance-to-close vs available accel."""
    t = rep["t"]; alt = rep["alt"]
    # Find time at which altitude crosses thresholds
    targets = [3.0, 2.0, 1.0, 0.5, 0.3, 0.2]
    crossings = {}
    for z_thresh in targets:
        idx = np.where(alt <= z_thresh)[0]
        if len(idx):
            i = idx[0]
            crossings[z_thresh] = {
                "t": float(t[i]),
                "lateral": float(rep["lateral"][i]),
                "vh": float(rep["vh"][i]),
                "vz": float(rep["vz"][i]),
                "Ia_xy": float(rep["Ia_xy"][i]),
                "sen_mag": float(rep["sen_mag"][i]),
            }
    # Total descent time + total lateral closure
    return {
        "t_end": float(t[-1]),
        "lateral_start": float(rep["lateral"][0]),
        "lateral_end": float(rep["lateral"][-1]),
        "vh_max": float(rep["vh"].max()),
        "vh_max_t": float(t[np.argmax(rep["vh"])]),
        "Ia_xy_max": float(rep["Ia_xy"].max()),
        "crossings": crossings,
    }


def main():
    candidates = {
        # SP reps (all 5 ever; this dict is the canonical SP registry):
        "SP #1 (Interventions, K_rp=9, lucky IC)":
            "/home/shubham/Soft-Precise-Landing/PX4_Gazebo/test_data/Interventions/20260523-162914/rep2",
        "SP #2 (NewDefaults K_rp=4 P_z=2.5)":
            "/home/shubham/Soft-Precise-Landing/PX4_Gazebo/test_data/NewDefaults/20260525-015341/rep5",
        "SP #3 (CoordDescent RHOFOV0=241.8, degenerate config: xy=0.052 vel=0.061)":
            "/home/shubham/Soft-Precise-Landing/PX4_Gazebo/test_data/CoordDescent/20260525-201532/pass1/RHOFOV0_241.8_rep2",
        "SP #4 (CoordDescent E_Z=0.107 axis test: xy=0.066 vel=0.165)":
            "/home/shubham/Soft-Precise-Landing/PX4_Gazebo/test_data/CoordDescent/20260526-014855/pass1/E_Z_0.1067_rep1",
        "SP #5 (HybridFlow hybrid arm rep5: xy=0.066 vel=0.136)":
            "/home/shubham/Soft-Precise-Landing/PX4_Gazebo/test_data/HybridFlow_AB/20260529-133034/hybrid_rep5",
        # Closest near-SP (PRECISE-only or near-PRECISE):
        "PRECISE-only stack (xy=0.031, vel=0.97)":
            "/home/shubham/Soft-Precise-Landing/PX4_Gazebo/test_data/RhoFov05_KP125/20260524-183744/rep6",
        "Closest-near-SP at K_rp=4 (rep9: xy=0.128 vel=0.156)":
            "/home/shubham/Soft-Precise-Landing/PX4_Gazebo/test_data/Krp4/20260525-011046/rep9",
        # Failure cases:
        "POOR at K_rp=4 (rep2: xy=0.84)":
            "/home/shubham/Soft-Precise-Landing/PX4_Gazebo/test_data/Krp4/20260525-011046/rep2",
    }

    traces = {}
    for label, d in candidates.items():
        if not os.path.exists(os.path.join(d, "Control_Data.npy")):
            print(f"  SKIP: {label} (no data at {d})")
            continue
        try:
            traces[label] = trace_rep(d)
        except Exception as e:
            print(f"  ERR {label}: {e}")

    print("=" * 110)
    print(" KINEMATIC TIMELINE: WHEN does lateral position settle (or fail to)?")
    print("=" * 110)
    print(f"{'rep':<46} {'t_end':>5} {'lat0':>5} {'lat_end':>7} {'vh_max':>6} (at t) {'Ia_xy_max':>9}")
    summaries = {}
    for label, rep in traces.items():
        s = kinematic_budget(rep)
        summaries[label] = s
        print(f"  {label[:44]:<46} {s['t_end']:>5.2f} {s['lateral_start']:>5.3f} "
              f"{s['lateral_end']:>7.4f} {s['vh_max']:>6.3f} (t={s['vh_max_t']:.2f}) "
              f"{s['Ia_xy_max']:>9.2f}")

    print()
    print("=" * 110)
    print(" PER-REP TIMELINE: lateral_pos / vh / Ia at altitude thresholds")
    print("=" * 110)
    for label, rep in traces.items():
        print(f"\n  {label}  (xy_end={rep['xy_end']:.4f}, vel_end={rep['vel_end']:.4f})")
        s = summaries[label]
        print(f"    {'z':<5} {'t':>5} {'lat':>6} {'vh':>5} {'vz':>6} {'Ia_xy':>6} {'sen':>5}")
        for z, c in sorted(s["crossings"].items(), reverse=True):
            print(f"    {z:>4.1f}m  {c['t']:>4.2f}  {c['lateral']:>5.3f}  {c['vh']:>4.2f}  "
                  f"{c['vz']:>5.2f}  {c['Ia_xy']:>5.2f}  {c['sen_mag']:>4.2f}")

    print()
    print("=" * 110)
    print(" PEAK LATERAL DEMAND vs ACHIEVED — is the controller getting what it asks for?")
    print("=" * 110)
    print(f"{'rep':<46} {'peak dsd_xy':>11} {'peak Ia_xy':>10} {'achieve_ratio':>13}")
    for label, rep in traces.items():
        dsd_max = float(rep["dsd_xy"].max())
        Ia_max = float(rep["Ia_xy"].max())
        ratio = Ia_max / max(dsd_max, 1e-9)
        print(f"  {label[:44]:<46} {dsd_max:>11.2f} {Ia_max:>10.2f} {ratio:>13.2%}")

    # Kinematic feasibility — given peak Ia_xy, could the drone close lateral_start
    # in t_end seconds?
    print()
    print("=" * 110)
    print(" KINEMATIC FEASIBILITY (assuming start vh=0, end vh=0, peak accel = Ia_xy_max)")
    print("=" * 110)
    print(f"{'rep':<46} {'lat0':>5} {'t_end':>5} {'peak_a':>6} {'reqd_a':>6} {'margin':>7}")
    for label, rep in traces.items():
        s = summaries[label]
        lat0 = s["lateral_start"]
        T = s["t_end"]
        # Bang-bang: accelerate for T/2 then decelerate for T/2 covers 2*(0.5*a*(T/2)^2) = a*T^2/4
        # → a = 4*lat0 / T^2
        reqd_a = 4 * lat0 / max(T**2, 1e-6)
        peak_a = s["Ia_xy_max"]
        margin = peak_a / max(reqd_a, 1e-9)
        print(f"  {label[:44]:<46} {lat0:>5.3f} {T:>5.2f} {peak_a:>6.2f} "
              f"{reqd_a:>6.2f} {margin:>6.2f}×")


if __name__ == "__main__":
    main()
