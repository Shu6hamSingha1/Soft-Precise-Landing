---
name: feedback_never_edit_running_bash_script
description: Never edit a .sh while a harness is executing it — bash reads scripts incrementally; the byte shift crashed a live A/B harness at its summary step (2026-09-24). Copy to a new file or wait.
metadata:
  node_type: memory
  type: feedback
  originSessionId: d1fc52b7-905a-496e-9fb1-d06cdf4109aa
  modified: 2026-09-24T06:06:31.453Z
---

Never edit a bash script while any process is executing it. Bash reads the file incrementally as it runs, so inserting/removing bytes shifts what the running shell reads next — it executes a torn line.

**Why:** 2026-09-24, adding a `gtbear` arm to `scripts/run_sperc_gtfb_ab.sh` while the rover baseline was still running it -> `syntax error near unexpected token '('` at the final summary step. The data survived only because the edit landed after the flight loop; mid-loop it could have launched a wrong arm or skipped reps silently.

**How to apply:** before editing any `scripts/*.sh`, `pgrep -af <script name>`. If it's running (including queued waiters that will exec it), either wait, or copy to a new filename and edit that. Queued-launch waiters should point at a file that won't change underneath them.
