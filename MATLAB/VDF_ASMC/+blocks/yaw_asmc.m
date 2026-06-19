function [psi_d, u_a, cs] = yaw_asmc(alpha, alpha_d, P, cs)
%YAW_ASMC  Virtual-compass yaw ASMC (tex eq. yaw control law).
%   Drives the image orientation error alpha_e = alpha - alpha_d to a UUB via a
%   leakage ASMC that outputs the desired yaw RATE psi_dot_d, integrated into the
%   heading psi_d (no magnetometer). alpha_e is unwrapped to (-pi/2,pi/2] for the
%   ellipse half-turn symmetry. alpha_d (= V_s_d(4)) is the aligned-marker reference
%   orientation (pi/4 for the square marker), NOT zero.

    e_raw = alpha - alpha_d;
    e_a   = atan2(sin(2*e_raw), cos(2*e_raw))/2;     % unwrap to (-pi/2, pi/2]
    if cs.k == 1, cs.ie_a = P.dt*e_a;
    else,         cs.ie_a = cs.ie_a + P.dt*(cs.e_a_prev + e_a)/2; end
    cs.e_a_prev = e_a;

    sigma_a  = e_a + P.Omega_a*cs.ie_a;
    cs.kappa_a = RK5(@(t,X) kappa_a_Solver(t, X, sigma_a, [P.n_a; P.p_a]), 0, cs.kappa_a, P.dt);
    u_a = P.Gamma_a*sigma_a + sat(sigma_a/P.E_a)*cs.kappa_a + P.Omega_a*e_a;

    cs.psi_d = atan2(sin(cs.psi_d + u_a*P.dt), cos(cs.psi_d + u_a*P.dt));   % wrap [-pi,pi]
    psi_d = cs.psi_d;
end
