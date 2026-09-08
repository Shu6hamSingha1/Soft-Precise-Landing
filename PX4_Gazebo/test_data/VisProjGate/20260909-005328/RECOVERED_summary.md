# visproj wire-in gate — RECOVERED (2026-09-09)

run_visproj_gate.sh logged every OLD-arm rep as "NO save" because BOTH arms'
landing_test.py autosaved into the MAIN tree's test_data/Landing_Test/ (the
worktree's `cd $SCRIPT_DIR/..` did not redirect the save path; LANDING_OUT_BASE
was not set per-arm). All 40 recordings ARE present in
~/Soft-Precise-Landing/PX4_Gazebo/test_data/Landing_Test/ (00:53-01:44,
alternating new/old by rep). Recovered by timestamp-matching each driver
`=== IC.. arm rep=.. HH:MM:SS ===` line to the recording created just after.
~95% reliable (one new-arm rep unmatched). NEW-arm numbers in the driver's own
summary.tsv (direct copy of $BUNDLE/$ic/new/rep$N) are authoritative for NEW.

## Result (NEW = visibility_projection.py  |  OLD = cbf_visibility.py joint-QP @ d380901c)

| metric              | NEW            | OLD            |
|---------------------|----------------|----------------|
| land / TL           | 20/20 / 0      | 20/20 / 0      |
| pooled mean xy (m)  | 0.14           | 0.24           |
| pooled median xy    | 0.09           | 0.12           |
| pooled max xy       | 0.76           | 0.88           |
| pooled max rel_vel  | 1.47           | 1.95           |
| precise             | 11/20          | 8/20           |
| precise+soft        | 15/20          | 12/20          |

Per-IC: NEW wins IC3 (mean 0.09 vs 0.23) and IC4 (0.18 vs 0.38); IC2 and IC5
are a wash. OLD's tail is worse (IC3r4 0.76/1.32, IC4r3 0.88/1.66/9s, IC4r4
0.68/1.95). NEW's worst is IC5r2 (0.76/1.47).

VERDICT: PASS. The visibility_projection wire-in matches or beats the retired
CBF machinery on IC2-5 — no landing-rate or target-loss regression, tighter
and softer, at ~half the code. Smoke IC2 NEW also landed precise xy=0.040 with
vis_active 0% (pure minimal-intervention passthrough).
