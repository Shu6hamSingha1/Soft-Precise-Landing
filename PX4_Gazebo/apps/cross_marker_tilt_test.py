"""Live validation of the cross-marker cluster-pairing fix (2026-08-01) under a
genuinely tilted camera view -- NOT just hover. Takes off, holds level briefly,
then commands a SUSTAINED attitude tilt (stock PX4 attitude control, same
send_attitude pattern as apps/cbf_isolation_test.py) directly above the marker,
polling the real perception pipeline (img_node, i.e. CrossMarkerNode ->
cross_marker_detector.detect()) throughout. Does NOT run the PLASMC controller.

Purpose: the pairing fix specifically targets perspective-skewed (non-exactly-
90deg) arm projections under tilt -- a hover-only test can't exercise that,
since the camera stays roughly fronto-parallel to the marker plane at zero tilt.

Launch:
    MARKER_TYPE=cross HEADLESS=1 PY_SCRIPT=apps/cross_marker_tilt_test.py \
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
T_LEVEL = float(os.environ.get("CROSS_TLEVEL", "3.0"))     # level hold before tilting (s)
T_TILT = float(os.environ.get("CROSS_TTILT", "8.0"))       # sustained-tilt duration (s)
TILT_DEG = float(os.environ.get("CROSS_TILT_DEG", "18.0")) # commanded pitch tilt (deg)
POLL_DT = float(os.environ.get("CROSS_POLL_DT", "0.3"))
HOVER_THRUST = float(os.environ.get("CROSS_HOVER_THRUST", "0.738"))
REF_RAD = float(os.environ.get("LANDING_REF_RAD_OPT_FLOW", "-0.3"))
DES_IMG = np.array([0.0, 0.0, 1.0, 0.0])


def attitude_from_quat(q):
    w, x, y, z = q.w, q.x, q.y, q.z
    roll = np.arctan2(2*(w*x+y*z), 1-2*(x*x+y*y))
    pitch = np.arcsin(np.clip(2*(w*y-z*x), -1.0, 1.0))
    yaw = np.arctan2(2*(w*z+x*y), 1-2*(y*y+z*z))
    return roll, pitch, yaw


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

        print(f"[cross_tilt] MARKER_TYPE={os.environ.get('MARKER_TYPE', 'aruco')}")
        EC_node = Controller(REF_RAD, DES_IMG, time_node, FC_node, pose_node=pose_node)
        img_node = EC_node._img_node

        print(f"[cross_tilt] taking off to {ALT} m...")
        await FC_node.arm_and_takeoff(ALT)
        _, _, yaw0 = attitude_from_quat(FC_node.getQuat())
        yaw0_deg = float(np.degrees(yaw0))

        def poll(tag, t_elapsed):
            visible = bool(getattr(img_node, 'FEATURE_IS_VISIBLE', False))
            xc = yc = alpha = float('nan')
            if visible:
                try:
                    feat = img_node.getImgFeatureParam()
                    xc, yc, _, alpha = [float(v) for v in np.asarray(feat).flatten()[:4]]
                except Exception:
                    pass
            _, pitch, _ = attitude_from_quat(FC_node.getQuat())
            print(f"  [{tag}] t={t_elapsed:5.1f} pitch={np.degrees(pitch):6.1f}deg "
                  f"visible={visible!s:5s} xc={xc:7.3f} yc={yc:7.3f} alpha={alpha:7.3f}")
            samples.append(dict(tag=tag, t=t_elapsed, pitch_deg=float(np.degrees(pitch)),
                                 visible=visible, xc=xc, yc=yc, alpha=alpha))

        # Use send_attitude (NOT send_velocity_body) for level hold too -- arm_and_takeoff
        # already leaves the OFFBOARD stream on velocity-type setpoints; switching setpoint
        # TYPE (velocity -> attitude) between phases was found (2026-08-01, first run of this
        # test) to make PX4 silently fall back to HOLD and ignore all subsequent attitude
        # commands (measured pitch never left ~0deg despite commanding 18deg). Keeping the
        # setpoint type consistent (attitude throughout, matching cbf_isolation_test.py's
        # pattern) avoids the type-switch entirely.
        print(f"[cross_tilt] level hold {T_LEVEL}s...")
        t0 = time.perf_counter()
        while time.perf_counter() - t0 < T_LEVEL:
            await FC_node.send_attitude(0.0, 0.0, yaw0_deg, HOVER_THRUST)
            if (time.perf_counter() - t0) % POLL_DT < 0.05:
                poll("level", time.perf_counter() - t0)
            await asyncio.sleep(0.05)

        # OSCILLATORY, not sustained-constant: a constant commanded lean has no position
        # compensation here (no PLASMC loop running), so it just accelerates the drone away
        # from the marker -- confirmed empirically (first version of this test: marker left
        # frame within ~1.9s of a constant 18deg command and never returned). Oscillating
        # pitch (net near-zero drift per cycle, same idea as cbf_isolation_test.py's burst
        # pattern) keeps the marker roughly in view while still passing through genuinely
        # tilted, non-fronto-parallel camera geometry every cycle.
        print(f"[cross_tilt] commanding oscillatory +-{TILT_DEG} deg pitch for {T_TILT}s...")
        t0 = time.perf_counter()
        last_poll = 0.0
        freq_hz = float(os.environ.get("CROSS_TILT_FREQ", "0.4"))
        while time.perf_counter() - t0 < T_TILT:
            t_elapsed = time.perf_counter() - t0
            pitch_cmd = TILT_DEG * np.sin(2 * np.pi * freq_hz * t_elapsed)
            tilt_rad = np.deg2rad(abs(pitch_cmd))
            thrust = float(np.clip(HOVER_THRUST / max(np.cos(tilt_rad), 0.5), 0.5, 0.95))
            await FC_node.send_attitude(0.0, pitch_cmd, yaw0_deg, thrust)
            if t_elapsed - last_poll >= POLL_DT:
                last_poll = t_elapsed
                poll("tilt", t_elapsed)
            await asyncio.sleep(0.02)

        print("[cross_tilt] recovering to level...")
        t0 = time.perf_counter()
        while time.perf_counter() - t0 < 3.0:
            await FC_node.send_attitude(0.0, 0.0, yaw0_deg, HOVER_THRUST)
            await asyncio.sleep(0.05)

        print("[cross_tilt] landing...")
        try:
            await FC_node.vehicle.action.land()
        except Exception:
            pass
        await asyncio.sleep(4)

        # --- summary ---
        for tag in ("level", "tilt"):
            rows = [s for s in samples if s['tag'] == tag]
            vis = [s for s in rows if s['visible']]
            print(f"\n[cross_tilt] {tag}: visible {len(vis)}/{len(rows)} polls"
                  + (f" (mean pitch {np.mean([r['pitch_deg'] for r in rows]):.1f} deg)" if rows else ""))
            if vis:
                for k in ('xc', 'yc', 'alpha'):
                    vals = np.array([s[k] for s in vis])
                    print(f"  {k}: mean={vals.mean():.4f} std={vals.std():.4f}")

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
