function [psi_d, u_a, cs] = yaw_asmc(alpha, alpha_d, P, cs)
%YAW_ASMC  Virtual-compass leakage-type ASMC yaw control (tex eq. `yaw control law`).
%   Drives the image-orientation error alpha_e = alpha - alpha_d and integrates the
%   desired yaw RATE u_a into the heading psi_d (no magnetometer). alpha is the
%   2pi-disambiguated principal direction (image_feature.m); alpha_e is wrapped to
%   (-pi,pi] via atan2, its sole discontinuity at +-pi (unreachable in a landing).
%   alpha_d (= V_s_d(4)) is the aligned-pattern principal direction (pi/4 for the
%   square marker, 0 for the cross), NOT necessarily zero.
%
%   Documented alternative / fallback to the direct-w_z rate law (now in its own
%   file, +blocks/yaw_rate_law.m -- ported out 2026-09-19 to keep the two designs
%   from sharing one file behind a P.yaw_rate_law flag). This leakage kappa_a ASMC
%   law lags a rotating target by ~12-23 deg (the SO(3) e_R[2]=sin(dpsi) ceiling)
%   and collapses past ~0.9 rad/s. Callers select which file to call based on
%   P.yaw_rate_law themselves; this file no longer branches on that flag.

    e_raw = alpha - alpha_d;
    e_a   = atan2(sin(e_raw), cos(e_raw));           % full +-pi (alpha is 2pi-disambiguated)

    % ---- leakage kappa_a ASMC (tex eq. `yaw control law`) -----------------
    if cs.k == 1, cs.ie_a = P.dt*e_a;
    else,         cs.ie_a = cs.ie_a + P.dt*(cs.e_a_prev + e_a)/2; end
    sigma_a  = e_a + P.Omega_a*cs.ie_a;
    cs.sigma_a = sigma_a;
    cs.kappa_a = RK5(@(t,X) kappa_a_Solver(t, X, sigma_a, [P.n_a; P.p_a]), 0, cs.kappa_a, P.dt);
    u_a = P.Gamma_a*sigma_a + sat(sigma_a/P.E_a)*cs.kappa_a + P.Omega_a*e_a;
    cs.e_a = e_a;  cs.e_a_prev = e_a;                           % expose for logging + trapezoidal state

    cs.psi_d = atan2(sin(cs.psi_d + u_a*P.dt), cos(cs.psi_d + u_a*P.dt));   % wrap [-pi,pi]
    psi_d = cs.psi_d;
end
