---
name: feedback_lk_dynamic_range_limit
description: LK optical flow collapses at ~2 m/s lateral velocity; KP=12 exceeds this at t=0 even with small centroid offsets — causes immediate TARGET_LOST/crash before SEN_FUNNEL or KLT can intervene
metadata:
  node_type: memory
  type: feedback
  originSessionId: dd8920aa-4635-40a9-ac9d-8409e37243d6
---

Lucas-Kanade optical flow tracking saturates at ~2 m/s lateral velocity in the Gazebo SITL setup. When the commanded body rate exceeds this, LK loses corner tracks → ArUco detection fails → FEATURE_IS_STALE within 3 frames → TARGET_LOST or crash.

**The KP=12 threshold:** `ds_d = KP × s_e_n`. At Z=5m, `v_req = ds_d × Z`. For the LK limit to hold: `KP × s_e_n × Z < 2 m/s` → `KP < 2 / (s_e_n × 5)`. At a modest 4% centroid offset (`s_e_n=0.04`): `KP_max = 2 / (0.04 × 5) = 10`. So **KP=12 exceeds the LK limit even at 4% centroid offset** — which is a typical value at IC1 after the drone drifts during SITL startup.

**Why KP=9 worked in old-cal runs:** broken cal scaled effective KP to ~0.7 (= 9/13× cal error) → `ds_d ≈ 0.03 rad/s → v_req ≈ 0.15 m/s` — safely within LK range. The correct cal restored full gain authority and exposed the limit.

**Why SEN_FUNNEL doesn't help:** the funnel acts on `s_e_n` through the outer PID → ds_d → body rates → optical flow. By the time LK collapses (3 consecutive ArUco misses = ~50ms), the FEATURE_IS_STALE flag fires and the controller enters grace timeout. SEN_FUNNEL has no time to gate the initial spike.

**Why KLT doesn't help:** KLT fallback (MARKER_KLT_MAX_STEPS=20) tracks corners via LK from last good frame — but if LK itself collapses due to high velocity, the KLT fallback also fails.

**Fix:** Keep `KP × s_e_n_max × Z_descent < 2 m/s`. At Z=5m and s_e_n_max≈0.08 (conservative): `KP_max = 2 / (0.08 × 5) = 5`. For KP=9: max safe centroid offset = `2 / (9 × 5) = 4.4%`. 

**Why:** confirmed on 2 reps (2026-06-08): xy=9.91m crash + TARGET_LOST xy=6.59m, both with IC converged fine but LK dying within 100ms of controller engagement.

**How to apply:** When setting KP, verify `KP × expected_s_e_n_at_IC × Z_takeoff < 2 m/s`. Start at KP=9 (the old-regime clean value), get consistent engagement, then sweep upward cautiously. If running KP≥10, ensure the drone's centroid offset at the start of control is <3%.
