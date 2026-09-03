---
name: feedback_check_concurrent_sitl_before_launch
description: "Before launching or force-killing PX4/Gazebo SITL processes, check whether another Claude Code session (or the user directly) is already using SITL -- multiple claude sessions can run concurrently on this machine, and an unguarded kill loop can destroy another session's legitimate work."
metadata: 
  node_type: memory
  type: feedback
  originSessionId: d0f95bdc-0584-4ea0-8594-cce5a448a11c
  modified: 2026-08-25T06:12:46.436Z
---

Always check for other active sessions/processes BEFORE starting a new PX4/Gazebo SITL run
or running any cleanup kill loop against `px4_sitl`/`landing_test.py`/`MicroXRCEAgent`/`gz sim`.

**Why:** caught live (2026-08-25) mid-investigation: multiple `claude` processes were found
running concurrently on this machine (`ps -eo pid,ppid,tty,user,lstart,cmd | grep claude`
showed 2 other sessions on separate ptys, running since the prior day). During a chain of
SITL launch retries (chasing an unrelated hang investigation), the cleanup kill loops used
`kill -9` on `px4_sitl_default/bin/px4`/`landing_test.py`/`mavsdk_server`/`MicroXRCEAgent`
matches WITHOUT the `grep -qa claude /proc/$p/cmdline` guard that the `gz sim` kill loop
already used (that guard exists specifically in the project's own launcher scripts, e.g.
`scripts/run_aruco_landing.sh`'s stray_clean pattern, to avoid killing another Claude-owned
Gazebo process) — meaning those other 3 process types could have force-killed a legitimate
concurrent session's SITL run with no protection at all. User caught this and asked directly
whether concurrent SITL use had been checked; it had not.

**How to apply:**
- Before ANY SITL launch (`run_aruco_landing.sh`, `run_rover_landing.sh`, direct
  `landing_test.py` invocations, etc.), check `ps -eo pid,ppid,tty,user,lstart,cmd | grep
  -E "px4_sitl|gz sim|landing_test.py|MicroXRCEAgent"` for existing processes AND `ps -eo
  pid,ppid,tty,user,lstart,cmd | grep claude` for other active Claude Code sessions that
  might be mid-flight even if no SITL process shows yet (a session could be about to launch
  one, or between attempts).
- If SITL processes ARE found and their ownership/session is unclear, ASK the user before
  killing anything — don't assume they're stale leftovers from your own prior work.
- When writing or reusing a cleanup/kill loop, apply the SAME `grep -qa claude
  /proc/$p/cmdline` (or equivalent ownership check) guard to EVERY process type being
  killed, not just `gz sim` — the existing launcher scripts only guard the `gz sim` kill,
  which is an incomplete pattern to copy uncritically into ad-hoc cleanup commands.
- This is a special case of the general destructive-action caution (measure twice before
  `kill -9`/other irreversible actions), scoped specifically to this project's SITL
  workflow since it's easy to reach for a broad `pkill -f` pattern when debugging launch
  flakiness, exactly when the temptation to skip the check is highest.

**Independently confirmed the same day, different session (see [[project_20260825_cbf_margin_reserve_fix]]) —
and this direction of fault matters, don't blur it:** a completely separate SITL process
tree (`test_data/Rover_AB_harness/ic5_hang_chase_*.out`, own `LANDING_OUT_BASE`) was found
running continuously (11:04→11:33+) squarely inside the window of an n=3 IC5 validation
sweep run in a DIFFERENT session — but it was found ONLY because that OTHER session did
its due diligence (`ps aux` after its own sweep came back unexpectedly noisy) and traced
the contamination back to `ic5_hang_chase_*`. **THIS session (the one running
`ic5_hang_chase_*`) never checked for concurrent SITL activity before launching those
reps at all** — the standing rule above existed in this same memory file already (added
earlier the same session, after the user's first catch) and was still not applied before
the next batch of launches. User called this out directly a second time. Two lessons, not
one: (1) the technical mechanism below (port 8888, CPU contention corrupting perception
timing), and (2) having written a rule once is not the same as applying it on every
subsequent launch in the same session — re-check every time, not just after the first
correction.

This adds a THIRD reason to check, beyond "don't kill someone else's work":
`MicroXRCEAgent` binds a single global port (8888), so a concurrent session's stack
directly explains the repeated "bind error: port 8888, errno 98" / "not the race
condition" launch flakes this project has often chalked up to generic SITL flakiness —
and even when launches succeed, two simultaneous `gz sim` physics+rendering stacks
compete for CPU. Lockstep sim timing stays deterministic, but real-wall-clock-dependent
subsystems (image processing rate, OpenCV decode timing, thread scheduling) are NOT —
exactly the machinery behind perception decode reliability that most SITL sweeps in this
project measure. **Before trusting an n>=3 sweep's result** (not a quick smoke test),
confirm no concurrent SITL session was active during the run window; a sweep that comes
back noisier than the mechanism predicts may be contaminated data, not a real negative
result, and isn't fixable retroactively — it needs a clean, isolated re-run.

## 2026-09-03 — the OTHER half: your own orphans. Cost ~2.5 h and 4 failed runs.

The pre-launch check above was run, returned ZERO, and the run still collided — because the
stale SITL was one **this session had orphaned**, and the check couldn't see it yet.

**What happened.** A wrapper script (`sphere_ab.sh`) drove `run_ic_validation.sh`, which drives
`run_aruco_landing_retry.sh`. Killing the wrapper with `pkill -f "sphere_ab.sh"` matched ONLY the
wrapper — the child `run_ic_validation.sh` has a different command line, survived as an orphan,
and kept launching reps **for 26 minutes**. Every subsequent run then fought it:
`MicroXRCEAgent` is `setsid`'d so it survives a driver kill and holds `udp:8888` → the next run's
rep 1 dies `bind error | port: 8888, errno: 98`; and each launcher's own cleanup SIGKILLs the
other run's processes → `run_aruco_landing.sh: line 77: <pid> Killed`. Four consecutive A/B runs
produced 0 usable reps and every summary read `landed=NO`, which looks exactly like "the change
under test broke everything."

**How to apply — three rules, each learned by breaking it:**
1. **Kill the TREE, not the name.** `kill -9 -<PGID>` (process group), or match the harness
   scripts BY NAME too — `run_ic_validation|run_aruco_landing|<wrapper>` — not just
   `px4_sitl|gz sim|MicroXRCEAgent`. A name-matched `pkill` on the wrapper leaves the whole
   child chain alive.
2. **`grep -av grep` is load-bearing in every such check.** `ps -eo args | grep -acE 'run_ic_validation|...'`
   counts its OWN command line and reports phantom competitors. Same self-match class as
   `until ! pgrep -f "x.py"` waiting on itself forever (that one leaked 3 stuck shells the same
   day). Any process check you write: ask whether it matches itself.
3. **Verify with the SAME pattern you killed with.** Killing with a 7-pattern list and then
   verifying with a 2-pattern list reports "residual: 0" while the agent still holds the port.
   That exact mismatch caused run #2 of 4.

**Also assert it, don't just check it:** put a `gate()` in the harness that ABORTS if `udp:8888`
is bound or any harness/SITL process is alive. A silent collision costs a full run; a loud abort
costs a second. And **always smoke-test ONE rep before launching a long sweep** — the first
failure here burned 2 h / 40 reps before anyone looked at a summary.
