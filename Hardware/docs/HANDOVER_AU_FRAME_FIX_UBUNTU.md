# Handover: a_u -> I_a frame bug (fixed on hardware 2026-09-26) -> apply to PX4_Gazebo

Audience: Claude Code on the Ubuntu SITL machine (`~/Soft-Precise-Landing/PX4_Gazebo/`).
Read first: `Hardware/docs/FLIGHT_ANALYSIS_2026-09-25.md` (evidence), then this file.
Constraints from CLAUDE.md still apply: do NOT launch Gazebo/SITL yourself unless the user explicitly asks in the
current conversation (then only `HEADLESS=1`), never edit `~/ws/scripts/soft_precise_landing/`, check for a running
SITL before launching (see memory `feedback_check_sitl_before_launch`), and do not port gain VALUES between
MATLAB/PX4/hardware (only structure).

## 1. The bug (both code bases have it)
`I_a_raw = R_au @ a_u - g*e3` with `R_au = R` = the FULL body(FRD)->NED DCM by default
(`PLASMC_AU_ROTZ_ONLY=0`). `a_u` is expressed in the gravity-levelled V frame (the frame perception's `s` is built in),
so the tilt of `R` leaks the lateral command into the vertical channel:
`(R a_u)_z = -sin(theta)*a_x + cos(theta)*sin(phi)*a_y + cos(theta)*cos(phi)*a_z`.
With `a_u_xy` uncapped (`PLASMC_AU_MAX_XY=0`) and blowing up in the terminal phase, `I_a_z` gets huge POSITIVE values
(= net downward accel command) -> `B_T` (thrust deficit) -> thrust collapse -> plunge.

Locations:
- Hardware (FIXED): `Hardware/scripts/controller.py`, `_attCtrl`, block "Raw inertial accel (net of gravity)".
- Gazebo (NOT yet fixed): `PX4_Gazebo/src/controller.py` ~lines 3826-3842 (same code, same comment
  "Default-off pending IC1 A/B", `PLASMC_AU_ROTZ_ONLY`). Line numbers drift: grep `R_au`.

## 2. What was changed on hardware
- New module-level helper `_levelled_basis(R)` (before `class Controller`): columns [x_V, y_V, z_V] in the body frame,
  z = gravity in body (`R.T @ [0,0,1]`), x = `[0,1,0] x z` normalised, y = `z x x`; falls back to the ZYX-yaw frame if
  degenerate. Identical construction to `img_geometry._rp_basis` / `hw_pos_feedback._v_frame` (and Gazebo
  `img_data._getVirtualPts`), so `R @ _levelled_basis(R)` = exact V->NED.
- New env `PLASMC_AU_FRAME`: `vframe` (DEFAULT, exact) | `rotz` (Rz(yaw_c); also selected by `PLASMC_AU_ROTZ_ONLY=1`) |
  `body` (legacy bug, for A/B only). `I_a_raw = (R @ _levelled_basis(R)) @ a_u - g*e3`.
- Verified offline: basis orthonormal/det=1/V-z == NED-down to 1e-15; on the 47 hardware flights (31,315 control ticks,
  Pi quaternion + logged `a_u`) max |I_a_z| 2746 -> 12.8 and samples with I_a_z > -5 (thrust < 50 % hover weight)
  1496 -> 0 (33 flights affected -> 0). This is an OPEN-LOOP replay; closed-loop effect is unmeasured.
- NOT yet deployed to the Pi and NOT flown (as of this handover).

## 3. What to do in PX4_Gazebo (suggested)
1. Port the helper + the `PLASMC_AU_FRAME` block into `PX4_Gazebo/src/controller.py` (same structure; keep
   `PLASMC_AU_ROTZ_ONLY` back-compat). Default choice is the user's call: the sim's Final VISTA-GT results have
   `a_u_xy` <= 2 m/s^2, so the leak there is ~0 (max z-leak median 0.05) and results should be unchanged - but
   confirm with an A/B rather than assume. Ask the user before changing the default.
2. A/B harness already exists: `scripts/run_rotz_ic1_ab.sh` (arms `body` vs `rotz`). Extend it with a `vframe`
   arm (use `PLASMC_AU_FRAME=vframe|rotz|body`), read `docs/SH_REFERENCE.md` and the `sh-script-patterns` skill
   BEFORE editing any .sh. Gate: `scripts/run_ic_validation.sh` (IC2-5 pre-merge gate) at n>=5 per arm.
3. Metrics to compare (see `docs/PLASMC_TUNING_GUIDE.md`): sub-metre / SOFT+PRECISE counts, `a_u_xy` max,
   terminal `I_a_z` max, touchdown speed, fly-aways. Also count runs with `I_a_z > -5` and `|I_a_z + g - a_u_z|`
   (the vertical leak) - it should be ~0 with `vframe`/`rotz`.
4. Do NOT re-tune gains as part of this; if results move, report the A/B, do not chase with gains.

## 4. Related findings the Ubuntu session may want to consider (not done anywhere)
- **Two "levelled frames" in use.** Perception / analytic s use the body-y x gravity frame; the controller (`yaw_c`,
  ZYX yaw) and the CBF / `visibility_projection.py` use `Rz(yaw_c)` to go inertial <-> image. They differ by
  0 deg (single-axis tilt), 7 deg (20/20), 16 deg (30/30), 24 deg (37/37 roll/pitch). Check whether
  `visibility_projection.py` (2026-09-09 two-tier rewrite) has the same yaw-frame assumption; if so it should use
  the same V basis. Unverified in Gazebo.
- **Thrust law has no tilt compensation:** `B_T = m*(I_a_z + g)/(cos(roll)*cos(pitch))` in both code bases
  (`_B_T.append(mass * (I_a[-1][2] + g) / cos/cos)`). It ignores the lateral part of I_a, so thrust stays ~hover while
  vertical lift is `T*cos(tilt)`: 1.3 m/s^2 sink at 30 deg, 2 m/s^2 at 37 deg. Correct form: thrust magnitude from
  the full vector `m*|I_a|` (or `B_T = m*g + m*I_a_z/(cos cos)`). Sim tilt stays small so it rarely shows; hardware
  reaches 30-37 deg. Design decision - ask the user.
- **Uncapped lateral command:** `PLASMC_AU_MAX_XY` defaults to 0 in both. Hardware normal flight: |a_u_xy| p99 5.3,
  p99.9 17.8 m/s^2 (>1.5 s before takeover); a cap ~10 clips 0.23 % of samples and would have removed the
  100-9900 m/s^2 terminal spikes. Sim data: Final VISTA-GT max a_u_xy 2 m/s^2 (cap inert there).
- Hardware-only, do not port: IMU jerk touchdown detector (`FC_IMU_TD*`, 800 m/s^3), scale/depth latches made
  opt-in, arm-wait stability fix, TD_DEBUG throttling. Gazebo has its own V2 perception detector + `_impactDetector`
  (|a|>50, tuned to Gazebo's 500-900 m/s^2 contact spikes; useless on hardware where soft contacts read 11-13).

## 5. Prompt to paste into Claude Code on Ubuntu
> Read `Hardware/docs/HANDOVER_AU_FRAME_FIX_UBUNTU.md` and `Hardware/docs/FLIGHT_ANALYSIS_2026-09-25.md`.
> Port the `_levelled_basis` helper and the `PLASMC_AU_FRAME` (vframe|rotz|body) switch from
> `Hardware/scripts/controller.py` into `PX4_Gazebo/src/controller.py` (search `R_au`), keeping the current default
> behaviour (`body`) for now. Then extend `scripts/run_rotz_ic1_ab.sh` (read `docs/SH_REFERENCE.md` first) with a
> `vframe` arm and tell me the command to run the A/B - do not launch SITL yourself. After I report results,
> propose whether to change the default. Also check whether `src/visibility_projection.py` assumes `Rz(yaw_c)`
> for the inertial<->image mapping and report (do not change it). Do not touch gains.
