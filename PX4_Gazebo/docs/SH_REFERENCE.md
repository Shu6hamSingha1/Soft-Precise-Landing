# PX4_Gazebo .sh master reference

Canonical bash patterns for new sweep / harness scripts in this project. Compiled from the active launchers in `scripts/` (`run_aruco_landing.sh`, `run_output_calibration.sh`, `run_input_calibration.sh`, `run_multi_ic_landing.sh`, `run_ic_validation.sh`) and the lessons from one-off sweeps (most of which were deleted — see git log around 2026-06-01).

Source-of-truth refs below point to the canonical implementation. Don't copy a snippet from a random old script — copy from the `scripts/<file>:line` listed here.

## Layout (post-2026-06-01)

```
PX4_Gazebo/
├── src/        # library .py — imported by apps/, never invoked directly
├── apps/       # entry-point .py — invoked by scripts/run_*.sh
├── tools/      # analyzers / aggregators — invoked manually post-recording
├── scripts/    # .sh launchers — THIS file's subject
├── notebooks/  # plotter notebooks
├── calibration_data/, run_logs/, Images/   # data dirs (not under scripts/)
```

From inside `scripts/`, `$SCRIPT_DIR` points at `scripts/`, so data dirs need the `$SCRIPT_DIR/..` prefix and `python3 apps/X.py` invocations should be preceded by `cd "$SCRIPT_DIR/.."`.

---

## 0. Should this be a .sh at all?

Most "one-off" sweeps end up dead within a week. Before writing a new `.sh`:

- **Will it be re-run?** If you'll execute it once and discard, run an inline `for r in ...; do ...; done` from the shell. Don't add a file.
- **Does an existing launcher already do this?** `run_aruco_landing_retry.sh`, `run_multi_ic_landing.sh`, `run_ic_validation.sh`, `run_output_calibration.sh` cover the common harnesses — parameterize via env vars, don't fork.
- **Is the tuning regime exhausted?** Per memory `feedback_strict_coord_descent_dry`, `feedback_precision_softness_frontier`, `feedback_phase1_matlab_baseline`: per-axis PLASMC gain sweeps under current SITL lag don't reach SP. Don't write a sweep for K_rp / KP / KI / θ_cap / ρ_∞ — that ground is covered.

When a new script IS warranted (new methodology, new IC strategy, new env-knob), use the patterns below.

---

## 1. Script header

```bash
#!/usr/bin/env bash
# <one-line description: what this does and why>
set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TIMESTAMP="$(date +%Y%m%d-%H%M%S)"
BUNDLE_DIR="$HOME/Soft-Precise-Landing/PX4_Gazebo/test_data/<BundleName>/${TIMESTAMP}"; mkdir -p "$BUNDLE_DIR"
```

- `set -u` (not `set -e` — many of these scripts intentionally continue after a failed rep).
- `SCRIPT_DIR` resolves via `BASH_SOURCE[0]` so `bash other.sh` from a different cwd still works.
- Bundle root under `~/Soft-Precise-Landing/PX4_Gazebo/test_data/<Name>/<timestamp>/` matches existing convention; downstream analyzers expect this.

---

## 2. Process cleanup — the canonical pattern

**Source: `scripts/run_aruco_landing.sh:27-72`.** Use exactly this pattern for any script that brings up PX4/Gazebo/bridges; do NOT invent your own.

```bash
PIDS=(); declare -A NAMES

cleanup() {
  echo; echo "[run] Shutting down background processes..."
  for pid in "${PIDS[@]}"; do
    if kill -0 "$pid" 2>/dev/null; then
      name="${NAMES[$pid]:-pid$pid}"
      echo "[run]   killing $name (pid $pid + group)"
      kill -TERM "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null
    fi
  done
  sleep 1
  for pid in "${PIDS[@]}"; do
    if kill -0 "$pid" 2>/dev/null; then
      kill -9 "-$pid" 2>/dev/null || kill -9 "$pid" 2>/dev/null
    fi
  done
  # Stray-grandchild belt-and-suspenders
  pkill -9 -f 'px4_sitl_default/bin/px4'        2>/dev/null || true
  pkill -9 -f 'gz sim'                          2>/dev/null || true
  pkill -9 -f 'parameter_bridge.*world/aruco'   2>/dev/null || true
  pkill -9 -f 'MicroXRCEAgent'                  2>/dev/null || true
  echo "[run] done."
}
trap cleanup EXIT INT TERM

start_bg() {
  local name="$1"; shift
  local logfile="$LOG_DIR/$name.log"
  echo "[run] launching $name (log: $logfile)"
  setsid "$@" > "$logfile" 2>&1 &        # setsid -> own process group; PGID == pid
  local pid=$!
  PIDS+=("$pid"); NAMES[$pid]="$name"
  sleep 0.3
  if ! kill -0 "$pid" 2>/dev/null; then
    echo "[run] $name died immediately; last 20 log lines:"; tail -n 20 "$logfile" || true
    exit 1
  fi
}
```

Key invariants:
- **Every** child goes through `setsid` so `kill -- -$pid` reaches the whole process group. A bare `kill $pid` misses bridge wrapper children — that bug bit earlier iterations of these scripts.
- The `pkill` block at the end of cleanup is a stray-grandchild catch, not the primary mechanism. Don't rely on it as the only kill.
- `trap cleanup EXIT INT TERM` — handles Ctrl-C, normal exit, and `kill TERM`.

### The self-pkill trap

`pkill -f PATTERN` matches against the full command line of every process — **including the bash loop that's running pkill**. If your loop is invoked as `bash -c 'for i in ...; do timeout 220 bash run_output_calibration.sh; ...; pkill -f record_output_calibration.py; ...; done'`, the `pkill -f record_output_calibration.py` kills the parent bash because its argv contains the string `record_output_calibration.py`.

**Fix:** put the loop in a file and invoke it as `bash /tmp/loop.sh`. The parent argv becomes just `bash /tmp/loop.sh`, so `pkill -f record_output_calibration.py` no longer matches it.

---

## 3. Loop-until-N-valid (calibration sweeps)

For cal sweeps where ~50% of attempts fail (PX4 failsafe / hung MAVSDK):

```bash
TARGET_DIR="$SCRIPT_DIR/../calibration_data/output"
for i in 1 2 3 4 5 6 7 8 9 10; do
  echo "=== Sweep $i  $(date +%H:%M:%S) ==="
  timeout 220 bash "$SCRIPT_DIR/run_output_calibration.sh"

  # Stray-process cleanup BEFORE counting (so files settle to disk)
  pkill -9 -x MicroXRCEAgent                  2>/dev/null || true
  pgrep -f 'px4_sitl_default/bin/px4'       | xargs -r kill -9 2>/dev/null || true
  pgrep -f 'record_output_calibration.py'          | xargs -r kill -9 2>/dev/null || true
  pgrep -f 'ros_gz_bridge parameter_bridge' | xargs -r kill -9 2>/dev/null || true
  pgrep -fa 'gz sim --verbose' | awk '/gz sim --verbose/ {print $1}' | xargs -r kill -9 2>/dev/null || true
  sleep 5

  # Drop empty timestamped dirs (failed runs)
  for d in "$TARGET_DIR"/*/; do
    [ -z "$(ls "$d" 2>/dev/null)" ] && rmdir "$d"
  done

  # Early exit when target reached
  [ "$(ls "$TARGET_DIR" | wc -l)" -ge 5 ] && break
done
```

Required components:
1. `timeout 220` outer wrapper — `run_output_calibration.sh` can hang past natural completion.
2. Empty-dir cleanup — `record_output_calibration.py` mkdirs its timestamped folder *before* the sweep runs, so a failed sweep leaves an empty dir that pollutes `ls`-by-mtime.
3. Stray pkill of child processes — `run_output_calibration.sh`'s own trap doesn't always reach grandchildren under timeout-SIGKILL.

---

## 4. Per-rep landing harness

For sweep scripts that drive `run_aruco_landing_retry.sh` and collect metrics. **Source: `run_kr_clamp.sh` (deleted, but the pattern below is preserved verbatim).**

```bash
IC="0.0,0.0,5.0"; N_REPS="${N_REPS:-5}"
SUMMARY="$BUNDLE_DIR/summary.tsv"
printf "rep\tlanded\txy_err\trel_vel\tflight_s\tprecise\tsoft\ttarget_lost\tresult_dir\n" > "$SUMMARY"

run_one() {
  local rep="$1"
  local dst="$BUNDLE_DIR/rep${rep}"
  echo "=== rep=$rep ==="

  # Capture latest dir BEFORE the run so we can detect a no-save fail
  local before     # NOTE: -d + */ -> directories only (pitfall 9: skips parameter_record.ods)
  before=$(ls -td "$HOME/Soft-Precise-Landing/PX4_Gazebo/test_data/Landing_Test/"*/ 2>/dev/null | head -1 || true)

  env <PLASMC_OVERRIDES> \
      INITIAL_DRONE_ENU="$IC" LANDING_AUTOSAVE=1 MAX_ATTEMPTS=5 \
      bash "$SCRIPT_DIR/run_aruco_landing_retry.sh" > "$dst.log" 2>&1

  local latest
  latest=$(ls -td "$HOME/Soft-Precise-Landing/PX4_Gazebo/test_data/Landing_Test/"*/ 2>/dev/null | head -1 || true)
  if [ -z "$latest" ] || [ "$latest" = "$before" ]; then
    printf "%s\tNO\t-\t-\t-\t-\t-\t-\t-\n" "$rep" >> "$SUMMARY"
    return
  fi
  cp -r "$latest" "$dst"

  local m
  m=$("$HOME/ws/scripts/env2025/bin/python3" - "$dst" << 'PY'
import sys, numpy as np, os
d = sys.argv[1]
gt   = np.load(os.path.join(d, "Ground_Truth.npy"), allow_pickle=True).item()
ctrl = np.load(os.path.join(d, "Control_Data.npy"), allow_pickle=True).item()
sp   = gt.get("SoftPrecise", {})
u    = gt["UAV Pose"]; fs = len(u)/60.0
print(f"{sp.get('xy_err',0):.4f}\t{sp.get('rel_vel',0):.4f}\t{fs:.1f}\t"
      f"{int(sp.get('precise',False))}\t{int(sp.get('soft',False))}\t"
      f"{int(sp.get('target_lost',False))}")
PY
)
  printf "%s\tYES\t%s\t%s\n" "$rep" "$m" "$(basename "$dst")" >> "$SUMMARY"
}

for r in $(seq 1 "$N_REPS"); do run_one "$r"; sleep 2; done
column -t -s $'\t' "$SUMMARY"
```

Hard requirements:
- **`before`/`latest` diff**: detects when `landing_test.py` exited without producing a recording (PX4 didn't arm, MAVSDK timeout, etc.). Without this you'd silently double-count the previous rep.
- **`LANDING_AUTOSAVE=1`**: required for landing_test to write the recording dir.
- **`MAX_ATTEMPTS=5`**: `run_aruco_landing_retry.sh` retries SITL flakes internally.
- **`env2025/bin/python3` for the metric extract**: matches the project venv. Don't use `python3` (system) — it lacks numpy/ahrs.
- **TSV summary + `column -t -s $'\t'`**: keeps grep-ability and pretty-printing both.

---

## 5. Standard IC set

The MATLAB phase used 5 nominal ICs (`MATLAB/Multi_init_cond/multi_Init_Var.m:27-33`). PX4/Gazebo uses ENU; conversion is `ENU_x = NED_y, ENU_y = NED_x, ENU_z = -NED_z`.

| ID | NED         | ENU (Gazebo)  | Purpose |
|----|-------------|---------------|---------|
| IC1 | (0, 0, -5)  | (0, 0, 5)     | On-axis, baseline alt — IC most tuning is done on |
| IC2 | (2, 2, -5)  | (2, 2, 5)     | Off-center, same alt |
| IC3 | (2, -2, -5) | (-2, 2, 5)    | Opposite side |
| IC4 | (2, 2, -7)  | (2, 2, 7)     | Higher — longer descent, more drift |
| IC5 | (2, 2, -3)  | (2, 2, 3)     | Lower — REF_RAD=-0.70 canary; short window for IBVS to converge |

**Per memory `feedback_ic_validation`**: any change to defaults MUST be validated across IC2-5 before merge. `run_ic_validation.sh` is the mandatory pre-merge gate. IC1-only improvements consistently regress IC2-5.

Standard bash declaration:
```bash
declare -A IC_ENU
IC_ENU[IC1]="0.0,0.0,5.0"
IC_ENU[IC2]="2.0,2.0,5.0"
IC_ENU[IC3]="-2.0,2.0,5.0"
IC_ENU[IC4]="2.0,2.0,7.0"
IC_ENU[IC5]="2.0,2.0,3.0"
```

---

## 6. Known env knobs

### Always-respected by the landing launchers

| Var | Default | Effect | Set by |
|---|---|---|---|
| `HEADLESS` | `''` | Qt offscreen, no GUI, no QGC window | `run_aruco_landing.sh`, `run_output_calibration.sh` |
| `INITIAL_DRONE_ENU` | `0.0,0.0,5.0` | Spawn pose (ENU) | landing_test.py |
| `LANDING_AUTOSAVE` | `0` | Save recording dir on landing_test exit | landing_test.py |
| `MAX_ATTEMPTS` | `1` | SITL retry budget on flake | run_aruco_landing_retry.sh |
| `N_REPS` | per-script | Number of reps in a sweep | sweep wrappers |
| `VENV` | `~/ws/scripts/env2025` | Python venv path | run_aruco_landing.sh |

### img_data / controller env knobs

| Var | Default | Effect | Memory |
|---|---|---|---|
| `MARKER_KLT_MAX_STEPS` | `20` | KLT-fallback steps when ArUco detection fails | `feedback_klt_marker_fallback` |
| `V_YAW_SOURCE` | `compass` | `alpha` makes IBVS pipeline compass-independent (requires sensor_cal redo) | `feedback_v_yaw_source_alpha` |
| `BODY_YAW_SOURCE` | `tel` | `gt` for analysis when compass drifts | `feedback_compass_yaw_drift` |
| `IMG_EXTRA_PTS` | `0` | Hybrid ArUco + Shi-Tomasi additional flow points | `tune-plasmc` skill |
| `IMG_FILTER_WIN` | `13` | savgol runtime window (not the offline 101) | CLAUDE.md |

### output_calibration knobs

| Var | Default | Effect | Memory |
|---|---|---|---|
| `CALIB_AMP_Z` | `0.6` | Vertical excitation amplitude (m) | `reference_aggregate_calibration` |
| `CALIB_AMP_YAW_DEG` | `10` | Yaw excitation amplitude (deg) | `reference_aggregate_calibration` |

---

## 7. Aggregate stats heredoc (post-sweep)

Standard tail block in a sweep script — reads the TSV, computes summary metrics:

```bash
"$HOME/ws/scripts/env2025/bin/python3" - "$SUMMARY" << 'PY'
import sys, csv, numpy as np
rows = [r for r in csv.DictReader(open(sys.argv[1]), delimiter='\t')
        if r['landed'] == 'YES']
if not rows:
    print("no landings"); sys.exit(0)
xy = np.array([float(r['xy_err'])  for r in rows])
ve = np.array([float(r['rel_vel']) for r in rows])
soft = sum(int(r['soft'])    for r in rows)
prec = sum(int(r['precise']) for r in rows)
spr  = sum(1 for r in rows if int(r['soft']) and int(r['precise']))
print(f"\n  n={len(rows)}  PRECISE={prec}  SOFT={soft}  SOFT+PRECISE={spr}")
print(f"  xy_err:  mean={xy.mean():.3f}  max={xy.max():.3f}  min={xy.min():.3f}")
print(f"  rel_vel: mean={ve.mean():.3f}  max={ve.max():.3f}")
PY
```

---

## 8. Common pitfalls

1. **Self-pkill** (section 2) — `pkill -f X` kills the loop's parent bash if its argv contains `X`. Always invoke loops as `bash /tmp/file.sh`, not as inline `bash -c '...'`.
2. **Empty-dir pollution** — `record_output_calibration.py` mkdirs its dir at start; if the sweep fails, the empty dir remains and inflates `ls`-by-mtime counts. The empty-dir `rmdir` step is mandatory.
3. **Stale `latest` symlink** — `ls -t` on `~/Soft-Precise-Landing/PX4_Gazebo/test_data/Landing_Test/` is the recording detector. Capture `before` BEFORE the run; comparing to `latest` after is the only way to detect "no recording was produced."
4. **`set -e` in sweep scripts** — DON'T. One failed rep should not abort the rest. Use explicit returns and let the summary record `NO`.
5. **`python3` (system) vs `env2025/bin/python3`** — only the venv has numpy/ahrs/scipy/cv2. Always use the full venv path in heredocs.
6. **Forgetting `setsid`** — bare `cmd &` puts the child in the script's process group; `kill -- -$pid` then targets the script itself. Always launch via the `start_bg` helper.
7. **Hardcoded `~`** — works in bash but breaks under `setsid env`. Use `$HOME`.
8. **Missing `timeout`** — `run_output_calibration.sh` and `landing_test.py` both have hang modes. Wrap any sweep in `timeout 220 bash ...`.
9. **Non-rep files in `Landing_Test/`** — `parameter_record.ods` lives inside `test_data/Landing_Test/`, so the §4 `ls -t | head -1` rep-detection picks it up if anything touches it mid-sweep (bit the 2026-06-02 DH_D_MAX sweep: a rep "result" resolved to the .ods). Use `ls -td .../Landing_Test/*/ | head -1` (directories only) and never edit the .ods while a sweep is running.
10. **Concurrent SITL from another session** (found 2026-08-25) — multiple Claude Code sessions (or the user directly) can be using PX4/Gazebo SITL on this machine at the same time. Before launching ANY `.sh` here, check `ps -eo pid,ppid,tty,user,lstart,cmd | grep -E "px4_sitl|gz sim|landing_test.py|MicroXRCEAgent"`. Two concrete failure modes if you skip this: (a) `MicroXRCEAgent` binds a single global port (8888) — a concurrent session directly explains repeated `bind error: port 8888, errno 98` launch flakes that otherwise look like generic SITL flakiness; (b) two simultaneous `gz sim` stacks compete for CPU — sim TIME stays deterministic (lockstep), but real-wall-clock-dependent subsystems (image capture rate, OpenCV decode timing, thread scheduling) do NOT, silently corrupting any perception-timing-sensitive sweep result without an error. **A sweep that comes back noisier than the mechanism predicts may be contaminated data, not a real negative result** — it needs a clean, isolated re-run, not a deeper dive into the (possibly nonexistent) code-level cause. Also apply the SAME `grep -qa claude /proc/$p/cmdline`-style ownership guard already used for `gz sim` kills (§2) to EVERY process type in a cleanup loop, not just `gz sim` — an unguarded `pkill -f landing_test.py`/`px4_sitl`/`MicroXRCEAgent` can kill another session's legitimate run. See `feedback_check_concurrent_sitl_before_launch` memory for the full incident writeup.
11. **`before`/`latest` dir-diff mis-attributes a rep to a CONCURRENT session** (found
    2026-09-04) — a third failure mode of #10, distinct from the port-8888 and CPU-contention
    ones: if another session's `landing_test.py` saves into `Landing_Test/` in the window between
    your `before=$(ls -td .../*/ | head -1)` and your post-run `latest=...`, `latest` silently
    resolves to THEIR rep, not yours — no error, no flake message, just a wrong-config rep copied
    into your output dir. Bit a Q8 yaw A/B: 3 of 6 "reps" turned out to belong to a concurrent
    detector-robustness session (foreign `WORLD`/`CROSS_GATE_MODE` in their `Control_Params.npy`),
    which would have produced a confidently-wrong A/B comparison. **Fix: after copying the
    candidate rep, verify its OWN `Control_Params.npy` → `Config.overrides` actually contains the
    env vars THIS run set** (present-with-value for what you exported, `!KEY`-absent for what you
    deliberately didn't) before accepting it; reject + retry (bounded, `MAXLAUNCH`) otherwise —
    don't just accept whatever `latest` resolves to. Requires the resolved-config dump in
    `Control_Params.npy` (`controller.py`'s `getParams()`/`_resolvedConfig()`, added 2026-09-03) —
    without it there is nothing to verify against. Reference implementation:
    `test_data/Q8_SpinFF/q8_probe_omegaff.sh`'s `verify_attribution()`.

---

## 9. When NOT to write a new .sh — alternative entry points

| Want to | Use | Don't write |
|---|---|---|
| Sweep a PLASMC env-var across N reps at IC1 | `N_REPS=5 env <VAR>=<VAL> bash run_aruco_landing_retry.sh` in a one-line `for` | A new `run_<var>_sweep.sh` |
| Run a single IC2-5 validation | `bash run_ic_validation.sh` | A new IC-specific script |
| Collect N cal recordings | `bash run_output_calibration.sh` in a `for i in $(seq 1 N)` loop with empty-dir cleanup | A new `run_<purpose>_calibration.sh` |
| Test a transient idea | Inline shell `for` loop, no file | A `.sh` file that'll be dead in a week |

If after reading the above the script is genuinely warranted (new methodology, new IC set, new analysis pipeline), use the patterns in sections 1-7.

---

## 10. A/B harness + rover two-instance addendum (2026-07-02)

**A/B harness (the now-dominant sweep shape; ~25 committed `run_*_ab.sh`).** Exemplar: `run_ez_ic2_ab.sh`. Canonical structure:
- `run_arm <name> <env...>` function; arm data to `test_data/<Name>_<arm>/` (top-level dir per arm → each arm becomes its own config bundle in `tools/build_test_record.py`).
- Loop-until-N-valid with a `MAXLAUNCH` launch budget (SITL ~50% flaky); `before`/`latest` dir-count diff to detect "no recording produced".
- If the tested param is auto-aligned per-axis (`PLASMC_E_*`, `PLASMC_XI2_*`, …), the arm MUST pin ALL THREE axes — a single-axis env silently reverts the others to hot defaults (the env trap).
- Summary via `$HOME/ws/scripts/env2025/bin/python3` heredoc over `Ground_Truth.npy['SoftPrecise']`.
- Judge stochastic terminal effects by RATE over n (fly-away %, breach %, `s_dot_entry`) — SP-count at n=5 has a ±5–7 noise floor.

**Rover / two-instance launch (`run_rover_landing.sh` + `run_rover_landing_retry.sh`).** Forked from the aruco launcher, same cleanup/`start_bg`/`setsid` patterns, plus:
- **Pose topic:** bridge the FULL `/world/rover/pose/info` — NEVER `dynamic_pose/info` (variable membership → indices point at random links → the 375 m IC-error landmine). Spawn-order indices: `POSE_IDX_TARGET=1`, `POSE_IDX_UAV=2` (same as aruco; env-overridable in `gz_subscriber.py`).
- **Stale gz server guard:** the `-i 1` instance's gz server outlives a single `pkill` → pre-launch guard + verify-and-rekill in cleanup, else the next run attaches to a frozen world.
- **Headless PX4:** use `px4 -d` (daemon, no pxh prompt) — otherwise GB/min logs.
- **Second MAVSDK client** (`apps/rover_drive.py`) needs a DEDICATED gRPC port (50052); sharing the default 50051 with landing_test's FC causes 180 s hangs.
- **`CHASE_GATE_FILE`:** the controller touches it at descent-start; rover motion (`ROVER_MOTION=1`) and chase-cam recording (`CHASE_CAM=1`) both gate on it, so arm/takeoff/IC happen over a stationary target.
- The chase camera is a SENSOR on the existing `ground_plane` link — adding it as a separate model shifts the pose indices (fly-away via garbage poses).
- Retriable flakes: `is_armable` lockstep race, IC non-settle (exit 42), and "Unable to get simulation time" (exits 0 — detected by pattern, not exit code).

**Data preservation rule:** harnesses + raw bundles behind load-bearing findings go under `test_data/` (e.g. `test_data/Rover_AB_{aruco,rover,rover_platform,harness}/`), NOT the session scratchpad — `/tmp` is wiped on reboot.
