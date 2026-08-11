# HANDOVER — running headless Gazebo flight tests with the cross-marker (2026-08-11)

This doc exists because there was no single place documenting how to actually
launch a flight against the cross+stub marker pipeline (`src/cross_marker_perception.py`)
— all the prior sessions' cross-marker work used dedicated recorder apps, and
the real landing-controller path was untested until today. Read this before
launching any cross-marker flight (calibration, validation, or landing).

## The three kinds of cross-marker flight, and when to use each

1. **Calibration flight** (`apps/record_cross_marker_calibration.py`) — phased
   single-axis excitation (x, y, z, yaw, yawagg, ± lowalt variants), position-
   setpoint-driven (NOT the PLASMC controller). Use to derive/re-derive the
   cross-marker's own `_sensor_cal_hw`/`_sensor_cal_s` via
   `tools/derive_cross_marker_cal.py`. Writes to `calibration_data/output_cross/`
   by default.
2. **Validation flight** (`apps/record_cross_marker_validation.py`) — an
   independent maneuver (`VALIDATION_PROFILE=multisine` or `=landing`), also
   position-setpoint-driven, NEVER written to `calibration_data/` (keeps
   train/test separate — see the `io-calibration` skill). Use to check a raw
   signal's correlation with GT without touching the derive tool's scan.
   Writes to `validation_data/cross_output_{multisine,landing}/`.
3. **Real landing test** (`apps/landing_test.py`, the actual PLASMC
   controller) — **only became possible under the cross-marker world as of
   2026-08-11** (see "The launcher fix" below). This is the only one of the
   three that exercises the real closed-loop controller, CBF, and
   touchdown/terminal-kick logic. Writes to `test_data/Landing_Test/`.

Don't confuse (1)/(2) with (3) — a clean calibration or validation flight
says nothing about closed-loop landing behavior, since neither one runs the
actual controller.

## The launcher fix (2026-08-11)

`scripts/run_aruco_landing.sh` (and its retry wrapper
`run_aruco_landing_retry.sh`, which just calls it in a loop) previously
hardcoded `PX4_GZ_WORLD=aruco` and every `/world/aruco/...` bridge topic —
there was no way to point it at the `cross_marker` Gazebo world at all. Fixed
by adding a `WORLD` env override (default `aruco`, unchanged), mirroring the
pattern `scripts/run_output_calibration.sh` already used for the calibration
apps. Model name (`x500_mono_cam_down`) is unaffected — the world differs,
the drone model doesn't.

**To run a real cross-marker landing test:**
```bash
cd ~/Soft-Precise-Landing/PX4_Gazebo
HEADLESS=1 WORLD=cross_marker MARKER_TYPE=cross \
  bash scripts/run_aruco_landing.sh
```
`MARKER_TYPE=cross` is read by `controller.py` (not the launcher) — it
switches `self._img_node` from `img_data.IMG_PROCESSOR` (ArUco) to
`CrossMarkerNode` (`cross_marker_perception.py`). Both env vars are required
together — `WORLD=cross_marker` alone spawns the right Gazebo scene but the
controller still runs the ArUco decoder against it (which will fail to find
anything); `MARKER_TYPE=cross` alone runs the cross-marker decoder against
the wrong (ArUco) world.

**IC validation (position-offset sweep) with the cross-marker:**
```bash
IC_LIST="IC1 IC2 IC3 IC4 IC5" N_REPS=2 HEADLESS=1 \
  MARKER_TYPE=cross WORLD=cross_marker \
  bash scripts/run_ic_validation.sh
```
(`run_ic_validation.sh` itself needed no changes — it just calls
`run_aruco_landing_retry.sh`, which now forwards `WORLD`/`MARKER_TYPE`
through the environment.)

**Retry wrapper** (recommended for any multi-rep sweep, absorbs SITL flakiness):
```bash
HEADLESS=1 WORLD=cross_marker MARKER_TYPE=cross LANDING_AUTOSAVE=1 \
  bash scripts/run_aruco_landing_retry.sh
```

## Common env vars across all three flight kinds

| Var | Meaning | Default |
|---|---|---|
| `HEADLESS=1` | Offscreen Qt, no GUI/QGC window (still fully functional — camera/sensors work) | GUI mode |
| `WORLD` | Gazebo world (`aruco` or `cross_marker`) | `aruco` |
| `MARKER_TYPE` | Perception pipeline (`aruco` or `cross`), read by `controller.py` | `aruco` |
| `PY_SCRIPT` | Which app the launcher runs in the foreground (landing launcher only) | `apps/landing_test.py` |
| `CALIB_APP` | Which app the calibration launcher runs | `apps/record_output_calibration.py` |
| `CALIB_OUT_BASE` / `CALIB_OUT_DIR` | Where calibration/validation data lands | `calibration_data/output` |
| `VALIDATION_PROFILE` | `multisine` or `landing`, for the validation recorder apps | `multisine` |
| `LANDING_AUTOSAVE=1` | Auto-save `test_data/Landing_Test/<timestamp>/` on the landing launcher | off |

## First real cross-marker landing test result (2026-08-11) — open issue, not a launcher bug

The very first closed-loop landing under `MARKER_TYPE=cross WORLD=cross_marker`
(`test_data/Landing_Test/Tue Aug 11 11-42-42 2026`) completed and landed, but
**hard**: impact 541 m/s² (threshold 50), touchdown rel-vel 4.27 m/s (vs.
~0.02-0.5 m/s on a normal ArUco landing), classified SOFT-only (not precise,
xy_err 0.46m). Perception itself was clean throughout (100% detection,
964/965 frames) — this is a controller/terminal-descent issue, not a
perception failure. Diagnosed from `Control_Data.npy`:

- **Ruled out:** kappa-ratchet (`kappa` stayed flat ~0.28-0.29 the whole
  descent, no runaway), funnel breach (`s_e_n` stayed small, 0.17-0.22, well
  inside), CBF-visibility saturation (`theta_cone`/`rho_fov`/`d_min_fov` all
  small and stable — the barrier never bound), and `PLASMC_TERMINAL_COMMIT`
  (defaults OFF, wasn't the mechanism this run since it's disabled by
  default).
- **Live observation:** `w_u` (commanded velocity magnitude) climbed
  steadily through the terminal ~1s (0.16→0.35 m/s and rising) while `B_T`
  swung sharply negative (+0.2→-2.75) as altitude dropped from 1.6m to
  1.3m — the controller was accelerating the descent right through the
  terminal approach instead of braking, right up to the impact.
- **ROOT-CAUSED (2026-08-11, same-session follow-up):** the perceived
  vertical optical-flow signal `h_z` (`Control_Data.npy`'s `h(t)[:,2]`)
  goes noisy and briefly WRONG-SIGNED right at the failure onset. The
  reference `h_d_z` stays flat at -0.30 throughout, but `h_z` reads -0.33,
  then -1.15 (3.8x over), then **+0.27 (positive — perceived ASCENDING
  while actually accelerating downward)**, then -0.23 (under-read), all
  within the ~0.3s window starting at t=39.06 (altitude ≈1.5-1.6m). That
  corrupted error signal feeds directly into the vertical-velocity control
  law, and the true GT descent trace shows the runaway starts at almost
  exactly this same altitude: a gentle ~0.74 m/s average descent from 5m
  down to 1.6m over ~4.6s, then the remaining 1.1m covered in ~0.5s,
  ending at the reported 4.27 m/s. `w_u` and `B_T`'s behavior (noted above)
  were downstream symptoms of the controller reacting to this bad
  measurement, not the root cause themselves.

  **This is a live, practical instance of the Hz-weak-near-the-ground
  problem this whole session's calibration/bisection work was chasing**
  (see [[project_cross_marker_hz_regression_bisection_20260810]] /
  `HANDOVER_cross_marker_hz_signflip_20260809.md`) — the specific
  "sign-flips-at-2m" STATISTICAL theory from that investigation was
  debunked (pooling showed no stable inversion), but the underlying fact
  that `Hz` is genuinely noisy/weak below ~1.5-2m for this marker was never
  in question, and this flight is the first time it showed up as a REAL
  closed-loop consequence instead of an offline correlation number.

  **Next step:** this reframes the whole Hz investigation's priority — the
  offline whole-flight R²≈0.48 metric already flagged Hz as weak, but this
  flight shows it's weak enough, at exactly the wrong moment, to corrupt a
  real landing. Before any more calibration bisection, consider whether the
  terminal descent needs a SAFETY NET independent of raw Hz near the ground
  (e.g. a velocity-rate limiter that ignores an implausible single-frame
  sign flip in `h_z`, or leaning on the vertical-rate reference / a
  descent-rate governor rather than trusting `h_z` directly below some
  altitude) — the same class of fix as the ArUco pipeline's ring-loom
  fusion safety net, which this marker's docstring explicitly says isn't
  implemented.

**Don't treat this landing's hardness as evidence the cross-marker perception
is bad** — tracking was clean and stable the entire flight. This is a
controller/tuning gap in genuinely new territory (first-ever integration),
not a repeat of any previously-diagnosed perception issue.

## What to try next (not yet done)

- Root-cause the accelerating terminal descent (see above).
- Run 2-3 more IC1 reps to see if the hard landing is a one-off SITL flake or
  reproducible (not yet done — this session paused after diagnosing the
  first flight to write this handover instead).
- Once landings are reliably soft, extend to the full IC1-5 set as the
  cross-marker's own first closed-loop baseline (analogous to
  `project_20260811_aruco_ic1to5_comparison_baseline`, the ArUco reference
  point this same day established for comparison).
