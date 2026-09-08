"""Isolated SITL validation of the target-visibility CBF.

Flies the drone under **stock PX4 attitude control** (no PLASMC outer/middle loop)
and tests the CBF as a standalone safety filter:

    theta_unsafe (a programmed lean that would push the marker off the sensor)
        -> cbf2_filter -> theta_safe
        -> commanded as an attitude setpoint (PX4's default PID tracks it).

The marker is parked near a FoV edge (the drone hovers at a lateral offset) so a
*modest* tilt threatens visibility -- keeping the lateral drift small. We log
theta_unsafe, theta_safe, the marker corners and the attitude every cycle for
notebooks/cbf_validation.ipynb (Section 8).

A/B: run with PLASMC_CBF_ISO_FILTER=1 (CBF on, default) and =0 (raw theta_unsafe).

Launch (stock stack, just point the launcher at this app):
    PY_SCRIPT=apps/cbf_isolation_test.py HEADLESS=1 bash scripts/run_aruco_landing.sh
"""
import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src'))
import asyncio
import time
import numpy as np
import rclpy
from ahrs import Quaternion

from flight_controller import FC
from gz_subscriber import GZ_Subscriber, Pose_Node, Clock_Node
from controller import Controller
from cbf_visibility import cbf2_filter

# ---- config (env-overridable) ----
G = 9.81
ALT          = float(os.environ.get("CBF_ISO_ALT", "4.0"))        # hover altitude (m)
CR_TARGET    = float(os.environ.get("CBF_ISO_CR", "0.80"))        # desired marker offset on the sensor (tangent)
AMP          = float(os.environ.get("CBF_ISO_AMP", "0.55"))       # theta_unsafe peak lean (rad)
T_RAMP       = float(os.environ.get("CBF_ISO_TRAMP", "2.0"))      # ramp-up time (s)
T_HOLD       = float(os.environ.get("CBF_ISO_THOLD", "1.5"))      # hold-at-peak time (s)
DT           = float(os.environ.get("CBF_ISO_DT", "0.02"))        # loop period (s)
CBF_ON       = os.environ.get("PLASMC_CBF_ISO_FILTER", "1") == "1"
HOVER_THRUST = float(os.environ.get("CBF_ISO_HOVER_THRUST", "0.738"))
ABORT_DRIFT  = float(os.environ.get("CBF_ISO_ABORT_DRIFT", "4.0"))  # |pos-hover| abort (m)
REF_RAD = float(os.environ.get("LANDING_REF_RAD_OPT_FLOW", "-0.3"))
DES_IMG = np.array([0.0, 0.0, 1.0, 0.0])

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "test_data", "CBF_Isolation")


def attitude_from_quat(q):
    """FC quaternion -> (R body->NED, R33, yaw_rad)."""
    quat = np.array([q.w, q.x, q.y, q.z])
    Qc = Quaternion(quat)
    R = Qc.to_DCM()
    yaw = float(Qc.to_angles()[2])
    return R, float(R[2, 2]), yaw


def get_corners(img_node):
    """Current 4 marker corners (raw px) or None."""
    try:
        fp = img_node._feature_pts
        if len(fp) > 0:
            c = np.asarray(fp[-1][1], float)
            if c.shape == (4, 2) and np.all(np.isfinite(c)):
                return c
    except (IndexError, AttributeError, ValueError, TypeError):
        pass
    return None


async def main():
    log = dict(t=[], theta_unsafe=[], theta_safe=[], corners=[], quat=[], cr=[], pos=[])
    pose_sub = time_sub = FC_node = EC_node = None
    try:
        rclpy.init()
        pose_node = Pose_Node(); pose_sub = GZ_Subscriber(pose_node)
        time_node = Clock_Node(); time_sub = GZ_Subscriber(time_node)
        t0 = time.perf_counter()
        while time_node.perf_counter() is None:
            if time.perf_counter() - t0 > 10: raise Exception("no sim time")

        FC_node = FC(time_node); await FC_node.start()
        t0 = time.perf_counter()
        while not FC_node.has_quat():
            if time.perf_counter() - t0 > 20: raise Exception("no FC data")
            time.sleep(0.05)

        EC_node = Controller(REF_RAD, DES_IMG, time_node, FC_node)   # for its img_node only
        img_node = EC_node._img_node
        center = np.asarray(img_node.center, float); focal = np.asarray(img_node.focal, float)

        await FC_node.arm_and_takeoff(ALT)

        # --- park at a lateral offset so the marker sits near the FoV edge ---
        tgt = pose_node.getPose().target
        off = CR_TARGET * ALT                                   # lateral offset for cr ~ CR_TARGET
        sp_n, sp_e, sp_d = tgt.position.x + off, tgt.position.y, -ALT
        print(f"[cbf_iso] parking at offset {off:.2f} m (cr target {CR_TARGET}); CBF={'ON' if CBF_ON else 'OFF'}")
        t0 = time.perf_counter(); cr0 = None
        while time.perf_counter() - t0 < 12:                    # settle + acquire marker
            await FC_node.send_position_ned(sp_n, sp_e, sp_d, 0.0)
            await asyncio.sleep(0.05)
            c = get_corners(img_node)
            if c is not None and time.perf_counter() - t0 > 6:
                cr0 = (c.mean(0) - center) / focal
        if cr0 is None:
            raise Exception("marker not acquired at offset hover")
        hover_pos = FC_node.getPosBody()
        _, _, yaw0 = attitude_from_quat(FC_node.getQuat())
        yaw0_deg = float(np.degrees(yaw0))
        print(f"[cbf_iso] acquired cr0={np.round(cr0,3)}  yaw0={yaw0_deg:.1f} deg")

        # --- CBF test phase: N cycles of [recenter (position-hold) -> oscillatory tilt burst] ---
        # Recentering between bursts bounds the lateral drift; the oscillatory theta_unsafe
        # (net-zero per cycle) repeatedly probes the FoV edge so the CBF clamp is exercised.
        N_CYCLES   = int(os.environ.get("CBF_ISO_CYCLES", "4"))
        FREQ       = float(os.environ.get("CBF_ISO_FREQ", "0.6"))       # Hz
        T_BURST    = float(os.environ.get("CBF_ISO_TBURST", "3.0"))     # s of oscillation per cycle
        T_RECENTER = float(os.environ.get("CBF_ISO_TRECENTER", "2.5"))  # s of position-hold per cycle
        cbf_state = {}; t_start = time_node.perf_counter()
        for cyc in range(N_CYCLES):
            # recenter over the parking point (position-hold), then re-measure cr -> outward DIR
            t0 = time.perf_counter(); cr_c = None
            while time.perf_counter() - t0 < T_RECENTER:
                await FC_node.send_position_ned(sp_n, sp_e, sp_d, 0.0); await asyncio.sleep(0.05)
                c = get_corners(img_node)
                if c is not None:
                    cr_c = (c.mean(0) - center) / focal
            if cr_c is None:
                cr_c = cr0
            x, y = cr_c
            Lw = np.array([[x*y, -(1+x*x)], [1+y*y, -x*y]]) @ np.array([[0.0, 1.0], [-1.0, 0.0]])
            DIR = np.linalg.solve(Lw, cr_c); DIR = DIR / (np.linalg.norm(DIR) + 1e-9)
            hover_pos = FC_node.getPosBody()
            print(f"[cbf_iso] cycle {cyc+1}/{N_CYCLES} recentered cr={np.round(cr_c,3)}")

            # oscillatory burst: theta_unsafe = AMP*sin(2*pi*f*t)*DIR (out on +half, in on -half)
            t0 = time.perf_counter()
            while time.perf_counter() - t0 < T_BURST:
                theta_unsafe = AMP * np.sin(2*np.pi*FREQ*(time.perf_counter()-t0)) * DIR
                q = FC_node.getQuat(); R, R33, yaw = attitude_from_quat(q)
                corners = get_corners(img_node)
                if CBF_ON and corners is not None:
                    a_z = G; cz, sz = np.cos(yaw), np.sin(yaw)
                    a_xy = a_z * np.array([cz*theta_unsafe[0] - sz*theta_unsafe[1],
                                           sz*theta_unsafe[0] + cz*theta_unsafe[1]])
                    I_a = np.array([a_xy[0], a_xy[1], -a_z])
                    _, _, ok, th_safe, _th_des = cbf2_filter(I_a.copy(), R, R33, yaw, corners,
                                                    center, focal, center/focal,
                                                    np.deg2rad(60.0), 0.0, DT, np.zeros(2), cbf_state)
                    theta_safe = th_safe if th_safe is not None else theta_unsafe
                else:
                    theta_safe = theta_unsafe

                # command theta_safe as an attitude setpoint (pitch=-th_x, roll=th_y)
                tilt = float(np.linalg.norm(theta_safe)); pos = FC_node.getPosBody()
                alt_err = ALT - (-pos.z_m)
                thrust = float(np.clip(HOVER_THRUST/max(np.cos(tilt), 0.5) + 0.04*alt_err, 0.5, 0.95))
                await FC_node.send_attitude(np.degrees(theta_safe[1]), -np.degrees(theta_safe[0]),
                                            yaw0_deg, thrust)

                log["t"].append(time_node.perf_counter() - t_start)
                log["theta_unsafe"].append(theta_unsafe.copy())
                log["theta_safe"].append(np.asarray(theta_safe, float).copy())
                log["corners"].append(corners.copy() if corners is not None else np.full((4, 2), np.nan))
                log["quat"].append(np.array([q.w, q.x, q.y, q.z]))
                log["cr"].append((corners.mean(0)-center)/focal if corners is not None else np.array([np.nan, np.nan]))
                log["pos"].append(np.array([pos.x_m, pos.y_m, pos.z_m]))
                if np.hypot(pos.x_m - hover_pos.x_m, pos.y_m - hover_pos.y_m) > ABORT_DRIFT:
                    print(f"[cbf_iso]   drift abort in burst {cyc+1}"); break
                await asyncio.sleep(DT)

        # --- recover: level + recenter over the hover point, then land ---
        print("[cbf_iso] test phase done -- recentering + landing")
        t0 = time.perf_counter()
        while time.perf_counter() - t0 < 4:
            await FC_node.send_position_ned(sp_n, sp_e, sp_d, 0.0); await asyncio.sleep(0.05)
        try:
            await FC_node.vehicle.action.land()
        except Exception:
            pass
        await asyncio.sleep(3)

    finally:
        # save logs
        try:
            if len(log["t"]) > 0:
                ts = time.strftime("%Y%m%d-%H%M%S")
                tag = "cbf" if CBF_ON else "nocbf"
                d = os.path.join(OUT_DIR, f"{ts}_{tag}")
                os.makedirs(d, exist_ok=True)
                out = {k: np.asarray(v) for k, v in log.items()}
                out["meta"] = dict(cbf_on=CBF_ON, alt=ALT, amp=AMP, cr_target=CR_TARGET,
                                   center=center, focal=focal)
                np.save(os.path.join(d, "CBF_Isolation.npy"), out, allow_pickle=True)
                print(f"[cbf_iso] saved {len(log['t'])} samples -> {d}/CBF_Isolation.npy")
        except Exception as e:
            print("[cbf_iso] save failed:", e)
        # stop threads (the Controller's image thread keeps the process alive otherwise)
        try:
            if EC_node is not None and EC_node.is_alive():
                EC_node.close(); EC_node.join()
        except Exception:
            pass
        for s in (pose_sub, time_sub):
            try:
                if s is not None and s.is_alive():
                    s.close(); s.join()
            except Exception:
                pass
        try:
            if FC_node is not None: await FC_node.close()
        except Exception:
            pass
        try:
            if rclpy.ok(): rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    asyncio.run(main())
