function [I_a_cd, cs] = asmc(o, I_R_V, P, cs)
%ASMC  Leakage adaptive sliding-mode control law (tex eq. adaptive control law).
%   From the combined-surface quantities in o, computes the optic-flow control
%   u_h = G_h^{-1}(u_sw + u_eq) with the online-adapted switching gain kappa
%   (leakage law via RK5/kappa_Solver), and maps it to the inertial commanded
%   acceleration  I_a_cd = I_R_V * V_a_cd - g,  V_a_cd = -G_h^{-1}(u_sw + u_eq).

    % adaptive gain kappa: leakage law  kappa_dot = ||theta|| N G |sigma| - N P kappa
    const_kappa = [P.N; P.Pleak];
    u_kappa     = [o.sigma; o.Theta_norm*ones(3,1)];   % kappa_Solver now takes per-axis theta (3x1); replicated scalar == old law
    cs.kappa    = RK5(@(t,X) kappa_Solver(t, X, u_kappa, const_kappa, o.G_2), 0, cs.kappa, P.dt);

    u_sw = -P.Gamma*o.sigma - o.Theta_norm*diag(sat(P.E\o.sigma))*o.G_2*cs.kappa;
    u_eq = o.G_2*(-o.c + o.S_2*o.dp_h - o.G_2\o.chi_zeta_aug);
    V_a_cd = -o.G_2\(u_sw + u_eq);
    I_a_cd = I_R_V*V_a_cd - P.g;
end
