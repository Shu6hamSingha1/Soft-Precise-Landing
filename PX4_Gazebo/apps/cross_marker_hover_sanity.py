"""Hover-only sanity check for MARKER_TYPE=cross wired into img_data.py.

Exercises the REAL perception pipeline (Controller -> IMG_PROCESSOR ->
_imgProcess -> _cross_detect_as_aruco_results, i.e. the synthetic-corner path
added 2026-08-01) but does NOT run the PLASMC control loop or descend --
takes off, holds hover under zero-velocity setpoints, and polls the img_node's
feature vector (h, w, s, alpha) + visibility/failure-cause flags each cycle.
Purpose: confirm the synthetic ArUco-shaped corners produced from the cross
detector feed a sane, stable feature vector before trusting them in closed-loop
control (a real landing_test.py run).

Launch:
    MARKER_TYPE=cross HEADLESS=1 PY_SCRIPT=apps/cross_marker_hover_sanity.py \
        bash scripts/run_cross_marker_altitude_test.sh
"""
import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src'))
import asyncio
import time

import numpy as np
import rclpy

from flight_controller import FC
from gz_subscriber import GZ_Subscriber, Pose_Node, Clock_Node
from controller import Controller

ALT = float(os.environ.get("CROSS_ALT", "5.0"))
T_HOLD = float(os.environ.get("CROSS_THOLD", "10.0"))   # hover duration to poll over (s)
POLL_DT = float(os.environ.get("CROSS_POLL_DT", "0.5"))
REF_RAD = float(os.environ.get("LANDING_REF_RAD_OPT_FLOW", "-0.3"))
DES_IMG = np.array([0.0, 0.0, 1.0, 0.0])


async def main():
    pose_sub = time_sub = FC_node = EC_node = None
    samples = []
    try:
        rclpy.init()
        pose_node = Pose_Node(); pose_sub = GZ_Subscriber(pose_node)
        time_node = Clock_Node(); time_sub = GZ_Subscriber(time_node)
        t0 = time.perf_counter()
        while time_node.perf_counter() is None:
            if time.perf_counter() - t0 > 10:
                raise Exception("no sim time")

        FC_node = FC(time_node); await FC_node.start()
        t0 = time.perf_counter()
        while not FC_node.has_quat():
            if time.perf_counter() - t0 > 20:
                raise Exception("no FC data")
            time.sleep(0.05)

        # Controller() constructs and auto-starts the real IMG_PROCESSOR thread
        # (self._img_node.start() inside img_data.py's __init__) -- we never call
        # EC_node.start() itself, so the PLASMC control loop never runs. Same
        # "borrow the img_node only" pattern as apps/cbf_isolation_test.py.
        print(f"[cross_hover] MARKER_TYPE={os.environ.get('MARKER_TYPE', 'aruco')}")
        EC_node = Controller(REF_RAD, DES_IMG, time_node, FC_node, pose_node=pose_node)
        img_node = EC_node._img_node

        print(f"[cross_hover] taking off to {ALT} m...")
        await FC_node.arm_and_takeoff(ALT)

        print(f"[cross_hover] holding {T_HOLD}s, polling every {POLL_DT}s (no control loop, zero-vel hold)...")
        t0 = time.perf_counter()
        print(f"\n  {'t':>5s} | {'visible':7s} | {'xc':>7s} {'yc':>7s} {'alpha':>7s} | "
              f"{'h1':>7s} {'h2':>7s} {'h3':>7s} {'w1':>7s} {'w2':>7s} {'w3':>7s} | fail_cause")
        print(f"  {'-----':>5s} | {'-------':7s} | {'-------':>7s} {'-------':>7s} {'-------':>7s} | "
              f"{'-------':>7s} {'-------':>7s} {'-------':>7s} {'-------':>7s} {'-------':>7s} {'-------':>7s} | ----------")
        while time.perf_counter() - t0 < T_HOLD:
            await FC_node.send_velocity_body(0.0, 0.0, 0.0, 0.0)
            visible = bool(getattr(img_node, 'FEATURE_IS_VISIBLE', False))
            stale = bool(getattr(img_node, 'FEATURE_IS_STALE', False))
            cause = None
            try:
                cause = img_node.getFailureCause()
            except Exception:
                pass
            if visible:
                try:
                    feat = img_node.getImgFeatureParam()
                    xc, yc, s_h, alpha = [float(v) for v in np.asarray(feat).flatten()[:4]]
                except Exception as e:
                    xc = yc = s_h = alpha = float('nan')
                    cause = f"getImgFeatureParam() raised: {e}"
                try:
                    hw = [float(v) for v in np.asarray(img_node.getOptFlowAngVel()).flatten()[:6]]
                except Exception as e:
                    hw = [float('nan')] * 6
                    cause = f"getOptFlowAngVel() raised: {e}"
            else:
                xc = yc = s_h = alpha = float('nan')
                hw = [float('nan')] * 6
            t_elapsed = time.perf_counter() - t0
            print(f"  {t_elapsed:5.1f} | {str(visible):7s} | "
                  f"{xc:7.3f} {yc:7.3f} {alpha:7.3f} | "
                  f"{hw[0]:7.3f} {hw[1]:7.3f} {hw[2]:7.3f} {hw[3]:7.3f} {hw[4]:7.3f} {hw[5]:7.3f} | {cause}")
            samples.append(dict(t=t_elapsed, visible=visible, xc=xc, yc=yc, alpha=alpha, hw=hw))
            await asyncio.sleep(POLL_DT)

        print("\n[cross_hover] landing...")
        try:
            await FC_node.vehicle.action.land()
        except Exception:
            pass
        await asyncio.sleep(4)

        # --- summary ---
        vis = [s for s in samples if s['visible']]
        print(f"\n[cross_hover] visible in {len(vis)}/{len(samples)} polls")
        if vis:
            for k in ('xc', 'yc', 'alpha'):
                vals = np.array([s[k] for s in vis])
                print(f"[cross_hover] {k}: mean={vals.mean():.4f} std={vals.std():.4f} "
                      f"min={vals.min():.4f} max={vals.max():.4f}")
            hw_arr = np.array([s['hw'] for s in vis])
            for idx, name in enumerate(['h1', 'h2', 'h3', 'w1', 'w2', 'w3']):
                vals = hw_arr[:, idx]
                print(f"[cross_hover] {name}: mean={vals.mean():.4f} std={vals.std():.4f} "
                      f"min={vals.min():.4f} max={vals.max():.4f}")

    finally:
        try:
            if EC_node is not None and EC_node._img_node.is_alive():
                EC_node._img_node.close(); EC_node._img_node.join()
        except Exception:
            pass
        for s in (pose_sub, time_sub):
            try:
                if s is not None and s.is_alive():
                    s.close(); s.join()
            except Exception:
                pass
        try:
            if FC_node is not None:
                await FC_node.close()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    asyncio.run(main())
