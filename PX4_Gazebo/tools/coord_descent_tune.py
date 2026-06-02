#!/usr/bin/env python3
"""Per-axis coordinate-descent tuner for the PLASMC controller.

For each axis (random order per pass):
  1. Sample a new value log-uniformly in [0.0001, 10000] as scale over default
  2. Run 3 reps at IC1 with this axis perturbed, all others at current best
  3. If composite (xy_min + 0.5*vel_at_min) improves → retain the axis
  4. Else revert
  5. Check for SP each rep — exit on first SP

After a full pass, if any axis was updated, do another pass (retunes the
others to incorporate the change).  Stop after MAX_PASSES (default 1) or
on SP.

Env knobs:
  MAX_PASSES        (default 1)
  REPS_PER_CAND     (default 3)
  IC                (default 0.0,0.0,5.0)
  COORD_SEED        (default 0)
"""
from __future__ import annotations
import json
import math
import os
import random
import subprocess
import sys
import time
from pathlib import Path
import numpy as np


# --- AXIS DEFINITIONS ---------------------------------------------------
# Each axis: (display_name, env_var, scale_kind, default_at_unit_scale)
#   scale_kind:
#     "scale"   — env value is the SCALE factor (multiplies built-in default)
#     "absolute" — env value is the ACTUAL value (overrides built-in default)
#   default_at_unit_scale: value to log/persist as the unit baseline
AXES = [
    # PID per-axis (X, Y separate). Scales over current K_rp_close=4, K_ri=1, K_rd=1.4375
    ("KP_X",    "PLASMC_KP_X_SCALE",   "scale", 1.0),
    ("KP_Y",    "PLASMC_KP_Y_SCALE",   "scale", 1.0),
    ("KI_X",    "PLASMC_KI_X_SCALE",   "scale", 1.0),
    ("KI_Y",    "PLASMC_KI_Y_SCALE",   "scale", 1.0),
    ("KD_X",    "PLASMC_KD_X_SCALE",   "scale", 1.0),
    ("KD_Y",    "PLASMC_KD_Y_SCALE",   "scale", 1.0),

    # Middle SMC per-axis (9 gains × 3 axes = 27)
    ("XI2_X",   "PLASMC_XI2_X_SCALE",   "scale", 1.0),
    ("XI2_Y",   "PLASMC_XI2_Y_SCALE",   "scale", 1.0),
    ("XI2_Z",   "PLASMC_XI2_Z_SCALE",   "scale", 1.0),
    ("P20_X",   "PLASMC_P20_X_SCALE",   "scale", 1.0),
    ("P20_Y",   "PLASMC_P20_Y_SCALE",   "scale", 1.0),
    ("P20_Z",   "PLASMC_P20_Z_SCALE",   "scale", 1.0),
    ("P2INF_X", "PLASMC_P2INF_X_SCALE", "scale", 1.0),
    ("P2INF_Y", "PLASMC_P2INF_Y_SCALE", "scale", 1.0),
    ("P2INF_Z", "PLASMC_P2INF_Z_SCALE", "scale", 1.0),
    ("OMEGA_X", "PLASMC_OMEGA_X_SCALE", "scale", 1.0),
    ("OMEGA_Y", "PLASMC_OMEGA_Y_SCALE", "scale", 1.0),
    ("OMEGA_Z", "PLASMC_OMEGA_Z_SCALE", "scale", 1.0),
    ("GAMMA_X", "PLASMC_GAMMA_X_SCALE", "scale", 1.0),
    ("GAMMA_Y", "PLASMC_GAMMA_Y_SCALE", "scale", 1.0),
    ("GAMMA_Z", "PLASMC_GAMMA_Z_SCALE", "scale", 1.0),
    ("E_X",     "PLASMC_E_X_SCALE",     "scale", 1.0),
    ("E_Y",     "PLASMC_E_Y_SCALE",     "scale", 1.0),
    ("E_Z",     "PLASMC_E_Z_SCALE",     "scale", 1.0),
    ("N_X",     "PLASMC_N_X_SCALE",     "scale", 1.0),
    ("N_Y",     "PLASMC_N_Y_SCALE",     "scale", 1.0),
    ("N_Z",     "PLASMC_N_Z_SCALE",     "scale", 1.0),
    ("P_X",     "PLASMC_P_X_SCALE",     "scale", 1.0),
    ("P_Y",     "PLASMC_P_Y_SCALE",     "scale", 1.0),
    ("P_Z",     "PLASMC_P_Z_SCALE",     "scale", 1.0),
    ("KAPPA0_X","PLASMC_KAPPA0_X_SCALE","scale", 1.0),
    ("KAPPA0_Y","PLASMC_KAPPA0_Y_SCALE","scale", 1.0),
    ("KAPPA0_Z","PLASMC_KAPPA0_Z_SCALE","scale", 1.0),

    # Acceleration conditioning (4 + 2 per-axis on rho_fov)
    ("RHOFOV0",   "PLASMC_RHOFOV0_SCALE",   "scale", 1.0),
    ("RHOFOVINF", "PLASMC_RHOFOVINF_SCALE", "scale", 1.0),
    ("LFOV",      "PLASMC_LFOV_SCALE",      "scale", 1.0),
    ("THETACAP",  "PLASMC_THETACAP_SCALE",  "scale", 1.0),

    # Yaw SMC scalars
    ("YAW_OMEGA",  "PLASMC_YAW_OMEGA_SCALE",  "scale", 1.0),
    ("YAW_GAMMA",  "PLASMC_YAW_GAMMA_SCALE",  "scale", 1.0),
    ("YAW_N",      "PLASMC_YAW_N_SCALE",      "scale", 1.0),
    ("YAW_P",      "PLASMC_YAW_P_SCALE",      "scale", 1.0),
    ("YAW_KAPPA0", "PLASMC_YAW_KAPPA0_SCALE", "scale", 1.0),
    ("YAW_E",      "PLASMC_YAW_E_SCALE",      "scale", 1.0),

    # SO(3) per-axis
    ("KR_ROLL",  "PLASMC_KR_ROLL_SCALE",  "scale", 1.0),
    ("KR_PITCH", "PLASMC_KR_PITCH_SCALE", "scale", 1.0),
    ("KR_YAW",   "PLASMC_KR_YAW_SCALE",   "scale", 1.0),

    # Structural axes (manuscript counts these as 2 of the 33)
    # h_rd: default -0.42; we use -0.70 in user-env. Sample log-uniform
    # over the magnitude, keep sign negative.
    # FILTER_WIN: default 13; sample log over integer range, clip to [3, 51]
    # We handle these as "absolute" in the run-config dict.
]


def log_uniform(rng: random.Random) -> float:
    """Sample uniform in log-space [0.0001, 10000]."""
    log_lo = math.log(1e-4)
    log_hi = math.log(1e4)
    return math.exp(log_lo + (log_hi - log_lo) * rng.random())


def composite(xys: list[float], vels: list[float]) -> tuple[float, float, float]:
    """Pick the rep with smallest xy; return (xy_min, vel_at_min, composite)."""
    valid = [(xy, vel) for xy, vel in zip(xys, vels) if xy < 9.0]
    if not valid:
        return (9.99, 9.99, 999.0)
    i = min(range(len(valid)), key=lambda k: valid[k][0])
    xy, vel = valid[i]
    return (xy, vel, xy + 0.5 * vel)


def run_one_rep(env: dict, ic: str, dst: Path) -> tuple[float, float, bool, bool, bool]:
    """Run one SITL landing. Returns (xy, vel, precise, soft, target_lost)."""
    landing_test_dir = Path.home() / "Soft-Precise-Landing/PX4_Gazebo/test_data/Landing_Test"
    landing_test_dir.mkdir(parents=True, exist_ok=True)
    # FIX: sort by mtime not name — names start with weekday abbrev (Mon/Tue/...)
    # which breaks alphabetic sort across week boundaries.
    def _latest_mtime():
        dirs = [d for d in landing_test_dir.iterdir() if d.is_dir()]
        return max((d.stat().st_mtime for d in dirs), default=0.0)
    before_mtime = _latest_mtime()
    full_env = os.environ.copy()
    full_env.update(env)
    full_env["INITIAL_DRONE_ENU"] = ic
    full_env["LANDING_AUTOSAVE"] = "1"
    full_env["MAX_ATTEMPTS"] = "3"
    full_env["HEADLESS"] = "1"
    script_dir = Path(__file__).parent
    log_file = dst.with_suffix(".log")
    dst.parent.mkdir(parents=True, exist_ok=True)
    try:
        with open(log_file, "w") as lf:
            subprocess.run(
                ["bash", str(script_dir / "run_aruco_landing_retry.sh")],
                env=full_env, stdout=lf, stderr=subprocess.STDOUT, timeout=400,
            )
    except subprocess.TimeoutExpired:
        return (9.99, 9.99, False, False, False)
    dirs = [d for d in landing_test_dir.iterdir() if d.is_dir()]
    new_dirs = [d for d in dirs if d.stat().st_mtime > before_mtime]
    if not new_dirs:
        return (9.99, 9.99, False, False, False)
    src = max(new_dirs, key=lambda d: d.stat().st_mtime)
    # Copy to dst
    import shutil
    if dst.exists():
        shutil.rmtree(dst)
    shutil.copytree(src, dst)
    try:
        gt = np.load(dst / "Ground_Truth.npy", allow_pickle=True).item()
        sp = gt.get("SoftPrecise", {})
        xy = float(sp.get("xy_err", 9.99))
        vel = float(sp.get("rel_vel", 9.99))
        prec = bool(sp.get("precise", False))
        soft = bool(sp.get("soft", False))
        tl = bool(sp.get("target_lost", False))
        return (xy, vel, prec, soft, tl)
    except Exception:
        return (9.99, 9.99, False, False, False)


def env_for_state(state: dict) -> dict:
    """Build env dict from {axis_name → scale_value} state."""
    out = {
        "LANDING_REF_RAD_OPT_FLOW": "-0.70",
        "IMG_FILTER_WIN": "7",
        "PLASMC_KP_SCHED_ENABLE": "0",   # disable scheduler — constant K_rp
    }
    for name, env_var, kind, _default in AXES:
        if name not in state:
            continue
        out[env_var] = f"{state[name]:.6g}"
    return out


def main():
    bundle = Path.home() / "Soft-Precise-Landing/PX4_Gazebo/test_data/CoordDescent" / time.strftime("%Y%m%d-%H%M%S")
    bundle.mkdir(parents=True, exist_ok=True)
    state_path = bundle / "state.json"
    log_path = bundle / "log.jsonl"

    max_passes = int(os.environ.get("MAX_PASSES", "1"))
    reps_per_cand = int(os.environ.get("REPS_PER_CAND", "3"))
    ic = os.environ.get("IC", "0.0,0.0,5.0")
    seed = int(os.environ.get("COORD_SEED", "0"))
    rng = random.Random(seed)

    # State: current best scale per axis (all 1.0 = current defaults)
    state = {name: 1.0 for (name, _, _, _) in AXES}

    # Baseline composite — measure current defaults with 3 reps
    print(f"\n=== Baseline measurement (n={reps_per_cand} reps at IC1, current defaults) ===")
    xys, vels = [], []
    n_sp_baseline = 0
    for r in range(reps_per_cand):
        dst = bundle / "baseline" / f"rep{r+1}"
        xy, vel, prec, soft, tl = run_one_rep(env_for_state(state), ic, dst)
        xys.append(xy); vels.append(vel)
        is_sp = prec and soft and not tl
        if is_sp: n_sp_baseline += 1
        print(f"  baseline rep {r+1}: xy={xy:.4f} vel={vel:.4f} prec={prec} soft={soft} tl={tl}{' ★' if is_sp else ''}")
    print(f"  baseline SP count: {n_sp_baseline}/{reps_per_cand}")
    best_xy_min, best_vel, best_composite = composite(xys, vels)
    print(f"  baseline composite={best_composite:.4f}  xy_min={best_xy_min:.4f}  vel={best_vel:.4f}")
    with open(log_path, "a") as f:
        json.dump({"event": "baseline", "composite": best_composite,
                   "xy_min": best_xy_min, "vel": best_vel}, f); f.write("\n")

    # IC2-5 set (per landing_test convention)
    ICS_OFF_CENTER = {
        "IC2": "2.0,2.0,5.0",
        "IC3": "-2.0,2.0,5.0",
        "IC4": "2.0,2.0,7.0",
        "IC5": "2.0,2.0,3.0",
    }

    def validate_ic2_5(env, bundle_subdir):
        """Run 2 reps at each of IC2-5.  Returns dict of stats per IC."""
        results = {}
        for ic_name, ic_pos in ICS_OFF_CENTER.items():
            n_sp = 0; xys = []; vels = []
            for r in range(2):
                dst = bundle / bundle_subdir / f"{ic_name}_rep{r+1}"
                xy, vel, prec, soft, tl = run_one_rep(env, ic_pos, dst)
                xys.append(xy); vels.append(vel)
                is_sp = prec and soft and not tl
                if is_sp: n_sp += 1
                print(f"      {ic_name} rep{r+1}: xy={xy:.4f} vel={vel:.4f} {'★' if is_sp else ''}")
            results[ic_name] = {"sp": n_sp, "xy_mean": sum(xys)/len(xys),
                                "xy_max": max(xys), "vel_mean": sum(vels)/len(vels)}
        return results

    true_win = False
    for pass_idx in range(1, max_passes + 1):
        if true_win: break
        print(f"\n{'='*72}\n  PASS {pass_idx}/{max_passes} — {len(AXES)} axes, random order\n{'='*72}")
        axis_order = list(AXES)
        rng.shuffle(axis_order)
        n_accepted_this_pass = 0
        for ax_i, (name, env_var, kind, _def) in enumerate(axis_order):
            if true_win:
                break
            new_value = log_uniform(rng)
            old_value = state[name]
            state[name] = new_value
            run_env = env_for_state(state)
            print(f"\n  [pass {pass_idx} axis {ax_i+1}/{len(AXES)}] {name}: try {new_value:.4g} (was {old_value:.4g})")
            xys, vels = [], []
            n_sp = 0
            # Run ALL reps_per_cand reps — no early break on first SP.
            # Need n_sp >= 3 for a "true SP candidate" worth IC2-5 validation.
            for r in range(reps_per_cand):
                dst = bundle / f"pass{pass_idx}" / f"{name}_{new_value:.4g}_rep{r+1}"
                xy, vel, prec, soft, tl = run_one_rep(run_env, ic, dst)
                xys.append(xy); vels.append(vel)
                is_sp = prec and soft and not tl
                if is_sp: n_sp += 1
                print(f"    rep {r+1}: xy={xy:.4f} vel={vel:.4f} prec={prec} soft={soft} tl={tl}{' ★' if is_sp else ''}")
            xy_min, vel_at_min, comp = composite(xys, vels)

            # ★ TRUE WIN gate: 3/3 SP at IC1, then IC2-5 validation.
            if n_sp == reps_per_cand:
                print(f"\n    ★★★ {reps_per_cand}/{reps_per_cand} SP at IC1 — validating IC2-5 ★★★")
                ic25 = validate_ic2_5(run_env, f"pass{pass_idx}_IC25_{name}")
                ic25_sp = sum(v["sp"] for v in ic25.values())
                ic25_n = sum(2 for _ in ic25)
                ic25_pass = ic25_sp >= 4  # at least half of 8 reps soft+precise
                print(f"    IC2-5: {ic25_sp}/{ic25_n} SP — {'PASS' if ic25_pass else 'FAIL'}")
                with open(log_path, "a") as f:
                    json.dump({"event": "true_win_check", "axis": name,
                               "value": new_value, "pass": pass_idx,
                               "ic1_sp": n_sp, "ic25_sp": ic25_sp, "ic25_n": ic25_n,
                               "ic25_pass": ic25_pass, "ic25_detail": ic25,
                               "state": state.copy()}, f); f.write("\n")
                if ic25_pass:
                    true_win = True
                    print(f"\n    ★★★ TRUE WIN: IC1 3/3 + IC2-5 {ic25_sp}/{ic25_n} SP ★★★")
                    state_path.write_text(json.dumps(state, indent=2))
                    break
                else:
                    print(f"    IC2-5 failed — keep searching (note: IC1 is reproducibly SP here though)")

            # Composite-improvement gate (just for tracking — every test logged)
            accepted = comp < best_composite
            if accepted:
                print(f"    ★ ACCEPTED: composite {comp:.4f} < best {best_composite:.4f}  (SP_count={n_sp}/{reps_per_cand})")
                best_xy_min, best_vel, best_composite = xy_min, vel_at_min, comp
                n_accepted_this_pass += 1
            else:
                print(f"    rejected: composite {comp:.4f} >= best {best_composite:.4f}  (SP_count={n_sp}/{reps_per_cand}) — reverting")
                state[name] = old_value
            with open(log_path, "a") as f:
                json.dump({"event": "axis_test", "pass": pass_idx, "axis": name,
                           "new_value": new_value, "old_value": old_value,
                           "xy_min": xy_min, "vel": vel_at_min, "composite": comp,
                           "best_composite": best_composite, "accepted": accepted,
                           "n_sp": n_sp,
                           "current_state": state.copy()}, f); f.write("\n")
            state_path.write_text(json.dumps(state, indent=2))
        if true_win:
            break
        print(f"\n  Pass {pass_idx} complete: {n_accepted_this_pass} of {len(AXES)} axes updated.")
        if n_accepted_this_pass == 0:
            print(f"  No axis improved — converged.")
            break

    print(f"\n{'='*72}")
    print(f"  COORDINATE DESCENT DONE")
    print(f"  TRUE WIN (3/3 IC1 + IC2-5 pass): {true_win}")
    print(f"  Best composite: {best_composite:.4f}  (xy_min={best_xy_min:.4f}, vel={best_vel:.4f})")
    print(f"  State: {state_path}")
    print(f"  Log: {log_path}")
    # Pretty-print non-1.0 axes
    changes = [(name, val) for name, val in state.items() if abs(val - 1.0) > 1e-6]
    if changes:
        print(f"  Updated axes ({len(changes)}):")
        for name, val in sorted(changes, key=lambda x: -abs(math.log(x[1]) if x[1] > 0 else 0)):
            print(f"    {name:<14} = {val:.4g}")


if __name__ == "__main__":
    main()
