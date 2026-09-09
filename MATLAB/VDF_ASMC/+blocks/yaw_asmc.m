function [psi_d, u_a, cs] = yaw_asmc(alpha, alpha_d, w_z, P, cs)
%YAW_ASMC  Virtual-compass yaw control (tex eq. `yaw control law`).
%   Drives the image-orientation error alpha_e = alpha - alpha_d and integrates the
%   desired yaw RATE u_a into the heading psi_d (no magnetometer). alpha is the
%   2pi-disambiguated principal direction (image_feature.m); alpha_e is wrapped to
%   (-pi,pi] via atan2, its sole discontinuity at +-pi (unreachable in a landing).
%   alpha_d (= V_s_d(4)) is the aligned-pattern principal direction (pi/4 for the
%   square marker, 0 for the cross), NOT necessarily zero.
%
%   w_z = V_w(3), the measured image angular-velocity z-component (~= alpha_e_dot,
%   tex eq. `alpha_e_dot`); consumed only by the P.yaw_rate_law=1 branch.
%
%   P.yaw_rate_law selects the law:
%     0 (default) -- leakage kappa_a ASMC of tex eq. `yaw control law`.
%     1           -- PLASMC_YAW_RATE_LAW port (PX4 87cf020 / baked ON 63aa258):
%                    drop the sliding-mode switching term, drive u_a as a PI on
%                    alpha_e that uses the MEASURED derivative w_z in place of a
%                    finite difference --
%                       d/dt w_rl = yrl_kp*alpha_e + yrl_wz_sign*w_z - yrl_ki*int(alpha_e)
%                       u_a       = clip(w_rl, +-yaw_rate_max)
%                    UNVALIDATED in MATLAB: run the IC gate before setting =1.

    e_raw = alpha - alpha_d;
    e_a   = atan2(sin(e_raw), cos(e_raw));           % full +-pi (alpha is 2pi-disambiguated)

    if isfield(P, 'yaw_rate_law') && P.yaw_rate_law
        % ---- PLASMC_YAW_RATE_LAW: direct-w_z PI, no kappa_a ASMC ----------------
        if ~isfield(cs, 'yrl_cmd')
            cs.yrl_cmd = 0;  cs.yrl_ie = 0;                     % first step: u_a = 0
        else
            rl_sat = abs(cs.yrl_cmd) >= P.yaw_rate_max - 1e-9;
            if ~rl_sat                                          % anti-windup: hold int while saturated
                cs.yrl_ie = cs.yrl_ie + P.dt*(cs.e_a_prev + e_a)/2;
            end
            rl_new = cs.yrl_cmd + P.dt*( P.yrl_kp*e_a ...
                                         + P.yrl_wz_sign*w_z ...
                                         - P.yrl_ki*cs.yrl_ie );
            cs.yrl_cmd = max(min(rl_new, P.yaw_rate_max), -P.yaw_rate_max);
        end
        u_a = cs.yrl_cmd;
        cs.sigma_a = e_a;  cs.kappa_a = 0;                      % keep the ASMC log fields populated
    else
        % ---- leakage kappa_a ASMC (tex eq. `yaw control law`) -----------------
        if cs.k == 1, cs.ie_a = P.dt*e_a;
        else,         cs.ie_a = cs.ie_a + P.dt*(cs.e_a_prev + e_a)/2; end
        sigma_a  = e_a + P.Omega_a*cs.ie_a;
        cs.sigma_a = sigma_a;
        cs.kappa_a = RK5(@(t,X) kappa_a_Solver(t, X, sigma_a, [P.n_a; P.p_a]), 0, cs.kappa_a, P.dt);
        u_a = P.Gamma_a*sigma_a + sat(sigma_a/P.E_a)*cs.kappa_a + P.Omega_a*e_a;
    end
    cs.e_a = e_a;  cs.e_a_prev = e_a;                           % expose for logging + trapezoidal state

    cs.psi_d = atan2(sin(cs.psi_d + u_a*P.dt), cos(cs.psi_d + u_a*P.dt));   % wrap [-pi,pi]
    psi_d = cs.psi_d;
end
