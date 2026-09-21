function [psi_d, u_a, cs] = yaw_rate_law(alpha, alpha_d, w_z, P, cs)
%YAW_RATE_LAW  Virtual-compass direct-w_z yaw control (tex eq. `yaw control law`).
%   PLASMC_YAW_RATE_LAW (PX4 63aa258): drives the image-orientation error
%   alpha_e = alpha - alpha_d and integrates the desired yaw RATE u_a into
%   the heading psi_d (no magnetometer). alpha is the 2pi-disambiguated
%   principal direction (image_feature.m); alpha_e is wrapped to (-pi,pi]
%   via atan2, its sole discontinuity at +-pi (unreachable in a landing).
%   alpha_d (= V_s_d(4)) is the aligned-pattern principal direction (pi/4
%   for the square marker, 0 for the cross), NOT necessarily zero.
%
%   w_z = V_w(3), the measured image angular-velocity z-component
%   (~= alpha_e_dot, tex eq. `alpha_e_dot`).
%
%   PD on alpha_e using the MEASURED derivative w_z in place of a finite
%   difference, no sliding-mode switching term:
%       d/dt w_rl = yrl_kp*alpha_e + yrl_wz_sign*w_z
%       u_a       = clip(w_rl, +-yaw_rate_max)
%   Validated (yrl_kp=0.3): IC1-5 25/25 SP; |e_a| ~1 deg to 0.7 rad/s spin.
%   (Integral term REMOVED 2026-09-10: k_i was 0.0 in every tested config,
%   untested at k_i>0 -- dead code, not a validated omission. Re-add if a
%   future robustness need is demonstrated.)
%
%   Ported out of yaw_asmc.m 2026-09-19 (this law was previously the
%   P.yaw_rate_law=1 branch there) to keep the direct-rate law and the
%   legacy leakage-type ASMC fallback (still in yaw_asmc.m) in separate
%   files, avoiding confusion between the two designs. Callers must
%   dispatch on P.yaw_rate_law themselves (no dispatcher retained in
%   yaw_asmc.m) -- see run_simulation.m / simulate_landing.m /
%   visualControl_comparison.m.
%
%   CLOSED-LOOP YAW-ERROR DYNAMICS: with dot(alpha_e) = -psi_b_dot + d_alpha and this law's
%   dot(u_a), the error obeys  alpha_e'' + alpha_e' + yrl_kp*alpha_e = disturbance, i.e.
%   s^2 + s + yrl_kp = 0 (w_z coefficient fixed at 1 -- NOTE ICRA.tex writes a free k_w).
%   yrl_kp sets the convergence speed sqrt(yrl_kp) and the damping 1/(2 sqrt(yrl_kp)).
%
%   FINAL CONFIG (2026-09-19): yrl_kp = 0.3 together with P.yaw_omega_d_ff = true (tracker
%   feedforward Omega_d = R'*[0;0;u_a] in so3_tracker.m). History: the earlier yrl_kp = 0.02
%   (2026-09-17/18) hid two implementation issues by suppressing the correction -- (1) the tracker
%   had Omega_d = 0, so psi_b lagged psi_d by kOmega_z*rate/kR_z (~25deg at 1.1 rad/s) and u_a wound
%   up against it (peak 1.13 rad/s vs the 0.672 needed); (2) w_z is zeroed by pinv(L_s, pinv_tol)
%   while the marker spans only ~3-5 px, so the implicit target-rate feedforward is absent for the
%   first ~2 s (an undamped phase, peak e_a = d/sqrt(k_p)). yrl_kp = 0.02 paid for that with a ~49 s
%   slow pole and 100+deg terminal alignment error. Issue (1) is fixed by the tracker feedforward;
%   issue (2) is mitigated by a larger marker (see the global MARKER_SCALE in Multi_init_cond/InitVar.m).
%   See project_ic2_speed_sweep_failure_2026_09_17 memory.

    e_raw = alpha - alpha_d;
    e_a   = atan2(sin(e_raw), cos(e_raw));           % full +-pi (alpha is 2pi-disambiguated)

    if ~isfield(cs, 'yrl_cmd')
        cs.yrl_cmd = 0;                                     % first step: u_a = 0
    else
        rl_new = cs.yrl_cmd + P.dt*( P.yrl_kp*e_a ...
                                     + P.yrl_wz_sign*w_z );  % tex `yaw control law`
        cs.yrl_cmd = max(min(rl_new, P.yaw_rate_max), -P.yaw_rate_max);
    end
    u_a = cs.yrl_cmd;
    cs.sigma_a = e_a;  cs.kappa_a = 0;                      % keep the ASMC log fields populated

    cs.e_a = e_a;  cs.e_a_prev = e_a;                       % expose for logging + trapezoidal state

    cs.psi_d = atan2(sin(cs.psi_d + u_a*P.dt), cos(cs.psi_d + u_a*P.dt));   % wrap [-pi,pi]
    psi_d = cs.psi_d;
end
