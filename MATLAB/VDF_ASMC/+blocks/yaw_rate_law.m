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
%   CLOSED-LOOP YAW-ERROR DYNAMICS (2026-09-18 finding, see
%   project_ic2_speed_sweep_failure_2026_09_17 memory): differentiating
%   the alpha_e kinematics with this law's dot(u_a) gives the second-order
%   ODE  alpha_e'' + alpha_e' + yrl_kp*alpha_e = disturbance, i.e. the
%   characteristic equation s^2 + s + yrl_kp = 0. yrl_kp therefore sets
%   BOTH the natural frequency (sqrt(yrl_kp)) and, jointly with the fixed
%   unit w_z-feedback coefficient, the damping -- a single gain cannot
%   independently set fast final convergence and low peak corrective yaw
%   rate. yrl_kp=0.3 gives a ~2s time constant (near-exact alignment
%   within an ~11s flight); yrl_kp=0.02 (baked 2026-09-17/18 to fix the
%   Circular@IC2 FoV breach by keeping peak omega_z low) gives a slow
%   real pole with a ~49s time constant -- alpha_e does not converge
%   within the flight, terminal error 36-156deg across a yaw-rate sweep,
%   NOT within 1deg as originally validated at yrl_kp=0.3. UNRESOLVED as
%   of 2026-09-19: this trades alignment precision for FoV robustness:
%   the peak omega_z during the initial IC2 acquisition (bearing swings
%   fast because IC2 starts off-center) OVERSHOOTS the target's actual
%   steady yaw rate (e.g. Circular @1.4x: target itself only needs
%   0.672 rad/s, but yrl_kp=0.3 drives a peak of 1.126 rad/s -- a ~68%
%   overshoot, not the necessary tracking rate), and that overshoot is
%   what leaks a rotation-induced disturbance into the translational
%   visual servo. A slew-rate limit on u_a (or a bearing-rate feedforward)
%   is the candidate fix being investigated to decouple "fast final
%   convergence" from "low peak corrective rate" -- NOT YET IMPLEMENTED.

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
