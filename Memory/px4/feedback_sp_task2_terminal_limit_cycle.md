---
name: feedback_sp_task2_terminal_limit_cycle
description: "⭐⭐ THE fly-away cause (reframed via controller objectives): Task-1 (s_e_n→0) SUCCEEDS — s_e_n converges+bounded to the deck. Task-2 (h_e→0) FAILS at the deck: h_e=v_rel/Z diverges (v_rel doesn't reach 0, lag-limited), detonating an UNSTABLE GROWING LIMIT CYCLE (the bounce). Every fly-away is this Task-2 limit cycle, not a position/sign problem."
metadata:
  node_type: memory
  type: feedback
  originSessionId: 31fd53ca-48b0-48f4-81a8-2e081955028f
---

**THE fly-away CAUSE, framed by what the controller is MEANT to do (user-led, 2026-06-25).** SP =
s_e<p_1 (Task-1 precise) AND h_e<p_2 (Task-2 soft); both must CONVERGE and stay BOUNDED. Looking at
s_e_n and h_e directly (not the launch symptom):

- **Task-1 (s_e_n) SUCCEEDS.** On both clean lands AND flys, s_e_n converges to ~0.05 by mid-descent
  and stays bounded all the way to the deck. The lateral POSITION control works. Position is NOT the
  problem (this kills the whole "lateral divergence / chi_r / s_e_n authority" thread).
- **Task-2 (h_e) FAILS at the deck.** At alt ~0.10m, |h_e_lat| spikes 0.0->3.27 **while s_e_n is still
  0.18** (h_e blows up FIRST, before s_e_n and before the launch). The barrier zeta_h is built on h_e,
  so unbounded h_e -> sigma -> a_u -> thrust saturation -> launch.
- **The fly-away is an UNSTABLE GROWING LIMIT CYCLE.** Post-launch the drone bounces (alt 0.1<->9.8m,
  period ~8s) with s_e_n/h_e oscillating and GROWING (s_e_n 1.3->2.7->11.7). Clean-vs-fly is a RACE:
  touchdown (alt<z_gear) vs the h_e spike launching first.

**WHY h_e diverges (the root):** h = v_rel/Z. As Z->0 the optic flow blows up UNLESS v_rel->0 (the
soft-touchdown condition, manuscript Cor.1/Thm.1 GUARANTEES it -> h=v/Z stays bounded -> h_e UUB). In
PX4 the **38ms inner-loop lag** prevents v_rel from reaching 0 as fast as Z shrinks -> any residual
velocity, x1/Z, makes the MEASURED flow diverge faster than the controller can track -> h_e (Task-2)
cannot be kept bounded -> limit cycle. So the lag breaks exactly the assumption the soft-touchdown
proof needs (beta=1/Z bounded; v_rel->0). [[feedback_descent_softness]] (vertical analog: hard vz
touchdown = same v_rel-not-zero).

**Why this CORRECTS the whole session's framing:** the fly-away is NOT lateral position divergence
(s_e_n converges), NOT the w_z sign (that's hygiene, doesn't fix it), NOT the c-term FORMULA (clean vs
old just shuffles which ICs launch), NOT a cap target (caps bound the OUTPUT of an unbounded h_e). It
is the **Task-2 optic-flow error going unbounded at the deck because v_rel doesn't reach 0 (lag)**, and
the resulting **terminal limit cycle**. The "altitude lateral divergence (mode-1)" was a binning
artifact (post-launch climb mixed into altitude bins); there is ONE mode = the terminal Task-2 limit
cycle. Supersedes the mode-1/mode-2 split and the c-term-as-cause framing.

**How to apply.** The lever is the SOFT condition itself, h_rd CONSTANT: make v_rel->0 INTO the deck
(so h stays bounded and h_e can converge), or stop the diverging h from driving the loop in the last
cm (gate/freeze the optic-flow feedback on a scale-free proximity proxy, NOT metric Z). Reduce the
38ms inner-loop lag (uXRCE-DDS) = the structural lever. NOT: s_e_n/position levers, c-term sign
shuffling, output caps. Confirm the limit-cycle frequency vs the 38ms lag (stability boundary) and
whether h_e_z (loom) leads h_e_lat into the cycle. Continues [[project_gt_feedback_control_tuning]],
[[feedback_terminal_launch_flow_loop]] (same launch, now correctly = Task-2 limit cycle),
[[feedback_plasmc_two_task_framework]].
