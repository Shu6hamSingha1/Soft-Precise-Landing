# 2026-09-25 flight-test analysis (47 flights, FC logs + Pi telemetry)

Scripts: `Test_Data/analysis_2026-09-25/` (run from `Hardware/Test_Data`; they write JSON to $TEMP/sp).
Pi runs and `.ulg` files are matched 1:1 **by order** (validated on rangefinder descent profiles; the FC clock drifts vs the Pi, so file names do not match).

## Headline findings
1. **Pilot overrides the controller in 37/47 flights** (stick takeover -> POSCTL); only 9 ended via the script's own AUTO_LAND. The "no takeover needed" criterion FAILED.
2. **Terminal blow-up**: `a_u_xy` max >=100 in 31/47 (>=1000 in 16). The peak is 0.2-0.9 s before handover at 0.3-0.7 m AGL (s = lateral/Z ~ 1/Z amplification; funnel |s_e_n| > 1 for 20-100 % of flights, s_e_n at end >10 in ~16 flights). izeta pins at its clamp (5.0) in 9 flights; kappa_x hits 30 in 3.
3. **Not actuator-limited**: motors never >0.95; max thrust setpoint median 0.49. Huge I_a is clipped upstream (lean cap).
4. **PX4 rate loop is fine** (lag ~48 ms roll / 60 ms pitch, corr 0.95) -> the slow lateral response is in the position/perception path, not the FC.
5. **09-24 fixes**: hover throttle tracks voltage (cmd/est ratio 0.97; both fall 0.023/V) -> OK. Yaw sign: |e_a| ends smaller in 24/47, final median 4 deg, 7 flights grew >0.1 rad (worst -0.57) -> partly fixed. TD detect fired at depth <=0.25 always (as coded); EKF/odometry height agrees with the rangefinder to ~0.03 m.
6. **MARKER_EXTENT_PX == 0 in every run** (cross perception is not running on the Pi, FIX-003; all flights are HW_POS_FEEDBACK) -> a marker-scale touchdown trigger cannot fire until perception is ported.
7. Loop 18 ms (p99 25 ms); camera 29 fps; image latency ~17 ms.

## Config comparison (n small; blown = a_u_xy>=100)
Baseline 7/12 blown. K_R=1: 3/3, K_R=2: 2/2, K_R=3: 1/2 (no benefit). OMEGA/GAMMA revert 2/5 but tilt 28 deg. Ga=1.2: 4/4. P_xy=2.5: 3/4 and worst precision (2.7 m median). P_z=2.5: 2/2. AU_LEAD: 4/4 (mapping by time). **kappa_max=10: 1/3 blown, 2/3 clean, sen_end 2.4, xy 0.36 m. KF q=1/r=0.05: 0/2 blown, sen_end 0.9, xy 0.15 m** -> the two configs worth repeating at n>=5.

## Precision / timing
Lateral offset from arm point: 0.22 m (hover) -> 0.41 m (handover) -> 0.53 m median at kill (p90 1.55, max 5.9). Handover vz median 0.83 m/s (p90 2.45) at 0.40 m AGL; contact ~4.7 s later under PX4/pilot; last-0.15 m vz median 0.23 m/s.
Battery: V at start 21.0-24.1 V, hover thrust 0.37-0.44; 7 flights hit a low-battery failsafe.

## Touchdown detector
Magnitude |a| thresholds unusable (soft contacts read 11-13 m/s^2). Replay of jerk |d a_z/dt| > 800 m/s^3 (3-sample mean) on 47 descents: 33 on time (<0.25 m), 6 missed, 5 early, 3 false. Implemented in flight_controller._imuTouchdownStep (FC_IMU_TD*). Untested in flight.

## Why the pilot took over (37/47 flights) -- added after alignment of Pi + FC clocks
- Sticks: roll/pitch/yaw sticks are exactly 0 in all 37; ONLY the throttle stick moves (28 up, 4 down, 5 small). Pilot is arresting a plunge, not steering.
- State at takeover (median): 0.36 m above ground (p10 0.11, p90 0.99), vz 1.2 m/s down (1.6 for stick-up cases, up to 2.6), tilt 22 deg (27 for stick-up). 4 stick-down cases were the takeoff-overshoot flights (3.5-5.9 m).
- Last second: tilt 9 -> 22 deg (median; 30-37 deg in the worst) while commanded thrust FALLS (0.25-0.29, down to 0.03) vs ~0.45-0.52 needed for level flight => ~2 m/s^2+ net downward accel => vz 2.2-2.6 m/s at 0.1-0.4 m.
- Controller cause (CORRECTED 2026-09-26): NOT a nominal-term problem. a_v is the ASMC virtual control (a_u = -G^-1 a_v), so its explosion (0.05 -> +19.5 m/s^2) and a_u_z pinned at the +-3.0 cap (PLASMC_AU_MAX_Z) are SYMPTOMS of the plunge. The thrust collapse comes from the a_u -> I_a frame: `I_a_raw = R_body @ a_u - g e3` (full body DCM, default PLASMC_AU_ROTZ_ONLY=0) leaks the uncapped lateral a_u_xy (100-9900 m/s^2) into I_a_z, giving +53 m/s^2 median peak I_a_z in takeover flights (non-takeover -9.2) -> B_T deficit +2..+12 N -> thrust 0.03-0.29. See "Rotation-matrix audit" below.
- Thrust law (secondary): B_T = m*(I_a_z+g)/(cos phi cos theta) gives no tilt compensation (1.3 m/s^2 sink at 30 deg, 2 m/s^2 at 37 deg). Same formula in PX4_Gazebo. Not changed.
- Trigger chain: lateral blow-up at 0.3-0.7 m (1/Z) -> a_u_xy spikes -> (bug) leak into I_a_z -> thrust collapse + tilt -> sink -> loom/a_v_z explode -> pilot pulls throttle.
- Correction: FC logs the rangefinder at only ~1 Hz; earlier ulg-rangefinder timings were coarse. With 50 Hz EKF height (agrees with the Pi rangefinder to ~0.03 m) the IMU-jerk (>800 m/s^3) replay is: 38 on time (<0.25 m), 7 missed, 1 early, 1 false; median trigger height -0.03 m (contact).

## Rotation-matrix audit (added after the takeover analysis)
Correction to the section above: `a_v_z` is the ASMC virtual control (a_u = -G^-1 a_v); its explosion is a symptom. The thrust collapse comes from the FRAME used to map a_u -> I_a.
- ahrs `Quaternion.to_DCM()` / `to_angles()`: body(FRD)->NED, ZYX. Verified vs textbook Rz(psi)Ry(theta)Rx(phi) on 500 random attitudes, err 1e-15, identical on the Pi (ahrs 0.4.0). OK.
- Pi telemetry quaternion vs FC vehicle_attitude (aligned): median 0.25 deg, p95 5.7 deg (latency). OK.
- Controller R (Rz Ry Rx from euler), R_VB (= Ry(pitch)Rx(roll), only used with W_XY_DEROT=imu), hw_pos `_yaw_from_R`, cbf Rz(+-yaw), img_geometry R_CAM_TO_BODY (proper rotation, det=+1, validated 07-27): OK.
- **BUG**: `I_a_raw = R_au @ a_u - g e3` with R_au = full body DCM (PLASMC_AU_ROTZ_ONLY=0 default). a_u lives in the gravity-levelled V frame, so the tilt leaks a_u_xy into I_a_z. Replay from logs: full-DCM reproduces logged I_a_raw_z to 0.01 m/s^2 (n=1198); leak = 4.2 m/s^2 per 8 m/s^2 of a_u_x at 37 deg. Logged max |I_a_z| 2610 -> 12.8 with rotz(yaw) or exact V-frame R*Vf^T; samples with I_a_z > -5 (thrust < 50% hover): 94 -> 0. (Open-loop replay of logged a_u; closed-loop effect not yet measured.)
- **Inconsistency**: perception/analytic V frame (`_v_frame` / img_geometry `_rp_basis`, heading from body-y x gravity) differs from the ZYX-yaw frame used by controller rotz(yaw_c) and cbf Rz(yaw) by 0 deg for single-axis tilt, 7 deg at 20/20, 16 deg at 30/30, 24 deg at 37/37 (roll/pitch). Exact fix: R_V->NED = R @ _v_frame(R).T.

## Fix records
All fixes and their post-flight confirmation checks are tracked in `docs/FIX_LOG.md` (FIX-004..FIX-016 from this analysis).

## Second bug-hunt pass (09-24 + 09-25) - scripts in analysis_2026-09-25/ (`common.py` loader for any date, `map_check.py`, `rate_gain.py`, `yaw_check.py`, `day_compare.py`)
New defects found (see docs/FIX_LOG.md): FIX-015 low-battery failsafes (16/86 flights, all engaged at <= 21.4 V) -> guard added; FIX-017 RATE_CORRECTION over-corrects
(achieved/intended 0.8); FIX-013/012/014 clarified (09-24 yaw positive feedback confirmed fixed; izeta windup mostly inert; overshoots coincide with failsafes).
Checked, NO defect (do not repeat):
- Controller -> FC command mapping: FC rates_setpoint = 0.75 x w_u (= RATE_CORRECTION, corr 1.00, lag 50 ms); B_T -> FC thrust_setpoint slope -0.0296 vs -0.0313 expected, corr -0.98.
- Hover throttle by voltage: commanded/FC-estimated hover thrust 0.97 on both days (0.91-1.01), independent of voltage.
- Pi telemetry latency (angular velocity/attitude) ~18 ms behind FC gyro; odometry ~95-100 Hz, IMU ~190 Hz. Attitude quaternion matches FC attitude (median 0.25 deg).
- Analytic s in hw_pos_feedback reproduces from telemetry to rms <= 0.012 (implementation faithful; FIX-002 is an input/lever-arm issue).
- Takeoff: offboard starts at median 3.06 m (09-24) / 3.13 m (09-25); descent rate 1->0.4 m is 0.2 m/s as designed.
- 09-24 vs 09-25: same pilot-takeover / leak signature (31/39 vs 37/47), so the frame bug (FIX-004) predates the 09-24 fixes.
