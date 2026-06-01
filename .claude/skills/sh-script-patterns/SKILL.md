---
name: sh-script-patterns
description: Canonical bash patterns for the PX4_Gazebo PLASMC project — process cleanup, loop-until-N-valid, per-rep landing harness, env knobs, IC set, pitfalls. INVOKE before writing or substantially editing any .sh in PX4_Gazebo/ (sweep, harness, cal loop, multi-IC, etc.), or when asked "should I make a new .sh for X". Encodes which patterns are canonical vs which routinely cause bugs (self-pkill, missing setsid, empty-dir pollution).
---

# .sh script patterns for PX4_Gazebo

Before writing or substantively editing a `.sh` in `PX4_Gazebo/`, read the master reference:

**`PX4_Gazebo/SH_REFERENCE.md`** — single source of truth for the canonical patterns.

## When to invoke

- User asks to write a new sweep / harness / cal-loop / multi-IC / impulse-response script
- User asks "can you make a .sh that ..."
- About to substantively edit `run_aruco_landing.sh`, `run_output_calibration.sh`, `run_input_calibration.sh`, `run_ic_validation.sh`, or `run_multi_ic_landing.sh`
- A bash sweep loop in a one-shot Bash tool call would be more than ~15 lines (write a file instead, applying these patterns)

## What to do

1. **Read `PX4_Gazebo/SH_REFERENCE.md` first** — it's ~300 lines, covers the 9 patterns the canonical launchers use.
2. **Check section 0 / section 9 of the reference**: does an existing launcher already do this with an env-var override? If yes, parameterize that instead of forking a new script. Most "I need a sweep for X" requests are answered by `env <X>=<VAL> N_REPS=5 bash run_aruco_landing_retry.sh` in a `for` loop — no new file needed.
3. **Check memory** for whether the tuning regime is exhausted before authoring a parameter sweep:
   - `feedback_strict_coord_descent_dry` — per-axis gain sweeps converged dry
   - `feedback_precision_softness_frontier` — RHO/THETACAP/KP frontier mapped
   - `feedback_phase1_matlab_baseline` — controller works in MATLAB; the gap is SITL-specific
4. **If a new file is warranted**, copy the canonical snippets from the reference. Don't invent variants. Specifically:
   - Cleanup helper + `start_bg` from `run_aruco_landing.sh:27-72` (reference §2) — verbatim
   - Per-rep landing harness with `before`/`latest` diff (reference §4) — verbatim
   - IC table from reference §5
   - Env-knob table from reference §6
5. **Avoid the 8 pitfalls listed in reference §8** — especially self-pkill (loop's argv matches), missing `setsid`, empty-dir pollution, and using system `python3` instead of `env2025/bin/python3`.

## Quick decision tree

```
Need to run something repeatedly?
├── Once, < 15 lines, throwaway       → inline shell `for` loop, no file
├── Once, needs metric extraction     → /tmp/loop.sh (not tracked); delete after
├── Recurring methodology / new harness → PX4_Gazebo/run_<name>.sh (tracked),
│                                          following SH_REFERENCE.md patterns
└── Variation on existing launcher    → env-var override, NOT a fork
```

## Hard rules

- Never `pkill -f PATTERN` from inside a `bash -c '...'` whose command line contains PATTERN. Put the loop in a file.
- Never use `set -e` in a sweep script — one failed rep must not abort the rest.
- Never spawn background children without `setsid` — kill -group needs the PGID.
- Never use system `python3` in heredocs — always `$HOME/ws/scripts/env2025/bin/python3`.
- Never skip the empty-dir cleanup in cal loops — `output_calibration.py` mkdirs eagerly.

## After writing

- Test by running once with `N_REPS=1` (or single iteration of the loop)
- Confirm cleanup actually kills the process tree (`pgrep -fa 'px4|gz sim|MicroXRCE'` should be empty after Ctrl-C)
- If the script is a one-off (deleted after the result is collected), don't `git add` it
- If it's recurring infrastructure, `git add` and commit with the patterns explicitly cited
