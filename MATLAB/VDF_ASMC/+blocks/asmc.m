function [I_a_cd, cs] = asmc(o, I_R_V, P, cs)
%ASMC  Leakage adaptive sliding-mode control law (tex eq. adaptive control law).
%   From the combined-surface quantities in o, computes the optic-flow control
%   u_h = G_h^{-1}(u_sw + u_eq) with the online-adapted switching gain kappa
%   (leakage law via RK5/kappa_Solver), and maps it to the inertial commanded
%   acceleration  I_a_cd = I_R_V * V_a_cd - g,  V_a_cd = -G_h^{-1}(u_sw + u_eq).

    % switching gain theta: shared scalar ||Theta||_F (parity) or per-axis row-norm
    % theta_k=sqrt(v_k^2+1) (tight bound; decouples z from lateral zeta_r). Replicated
    % scalar reproduces the published law bit-for-bit. See PER_AXIS_THETA_PROOF.md.
    if isfield(P,'theta_per_axis') && P.theta_per_axis
        theta_ctrl = o.theta_perax;                    % 3x1 per-axis
    else
        theta_ctrl = o.Theta_norm*ones(3,1);           % 3x1 replicated scalar == published law
    end

    % adaptive gain kappa: leakage law  kappa_dot = theta .* (N G |sigma|) - N P kappa
    const_kappa = [P.N; P.Pleak];
    u_kappa     = [o.sigma; theta_ctrl];               % kappa_Solver takes per-axis theta (3x1)
    cs.kappa    = RK5(@(t,X) kappa_Solver(t, X, u_kappa, const_kappa, o.G_2), 0, cs.kappa, P.dt);

    % PORTED FROM PX4 2026-09-03: per-axis kappa cap (PLASMC_KAPPA_MAX_{X,Y,Z} = [30;30;3.0]).
    % MATLAB previously had NO cap. Anti-runaway backstop only -- it must be INERT on clean reps
    % (PX4 clean reps sit at kappa_z ~1, kappa_xy <= ~12) and bite only during a genuine detonation
    % (real hardware saw kappa_xy pinned 25-29 for 10-28 s). Guarded so a params struct without the
    % field behaves exactly as before the port.
    if isfield(P, 'kappa_max')
        cs.kappa = min(cs.kappa, reshape(P.kappa_max, size(cs.kappa)));   % reshape: avoid 3x1/1x3 broadcast
    end

    % diag(theta_ctrl)* (not theta_ctrl.*) folds the scalar into the diagonal FIRST ->
    % off-path BIT-EXACT to old Theta_norm*diag(sat)*G*kappa (FP non-assoc); per-axis ON.
    u_sw = -P.Gamma*o.sigma - diag(theta_ctrl)*diag(sat(P.E\o.sigma))*o.G_2*cs.kappa;
    u_eq = o.G_2*(-o.c + o.S_2*o.dp_h - o.G_2\o.chi_zeta_aug);
    V_a_cd = -o.G_2\(u_sw + u_eq);
    I_a_cd = I_R_V*V_a_cd - P.g;
end
