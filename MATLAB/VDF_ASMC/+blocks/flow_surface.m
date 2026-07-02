function [o, cs] = flow_surface(V_s, V_h, B_w_c, I_R_C, zeta_r, dzeta_r, s_dot_presc, t, P, cs)
%FLOW_SURFACE  Optic-flow funnel + combined sliding surface (tex Sec. III-A.2).
%   Builds the desired flow h_d (funnel-prescribed s_dot + clean-IMU rotation FF + descent),
%   the optic-flow error h_e, its barrier zeta_h / Jacobian G_h, the combined sliding
%   surface sigma = zeta_h + chi*zeta_aug (lateral PD via zeta_r, descent PI), the
%   known c-term (c_simple manuscript form), and the regressor Theta. s_ddot-drop
%   variant: the s_dot term is excluded from d/dt(h_d) (absorbed by kappa as d_h).
%   Returns struct o with fields used by blocks.asmc; cs carries integrator/filters.

    e3 = [0;0;1];
    % s_dot_presc = p_10 .* S_r .* dp_r (funnel-prescribed centroid rate from
    % position_funnel) replaces the measured s_dot in h_d: smooth, and bounded on a
    % funnel breach so h_e grows and demands recovery authority. The measured rate
    % still feeds the position funnel's dr_bar_e (it is not removed there).

    % clean-IMU yaw-rate transport FF (c_simple)            [tex eq. h_d final]
    phi_b   = atan2(I_R_C(3,2), I_R_C(3,3));
    theta_b = -asin(max(min(I_R_C(3,1),1),-1));
    psi_dot_b = (B_w_c(2)*sin(phi_b) + B_w_c(3)*cos(phi_b))/cos(theta_b);
    rot     = psi_dot_b * cross(e3, V_s(1:3));
    h_d_ff  = P.h_rd * V_s(1:3);
    % Prescribed rate folded in: its derivative is smooth (RMS ~1e-2, vs measured
    % s_ddot ~7) so it enters dh_d/the c-term cleanly -- the s_ddot-drop is no longer
    % needed (it only ever suppressed the noisy MEASURED s_ddot).
    h_d_all = [s_dot_presc; 0] + rot + h_d_ff;   % prescribed rate + transport + descent (folded)
    V_h_d   = h_d_all;
    V_h_e   = V_h - V_h_d;

    % optic-flow funnel p_h(t) and barrier zeta_h, Jacobian G_h   [tex eq. PPC]
    p_h  = expm(-P.Xi_h*t)*(P.p_h0 - P.p_hinf) + P.p_hinf;
    dp_h = -P.Xi_h*expm(-P.Xi_h*t)*(P.p_h0 - P.p_hinf);
    S_2 = zeros(3); zeta_2 = zeros(3,1); G_2 = zeros(3);
    for j = 1:3
        s = min(max(V_h_e(j)/p_h(j), -1+P.S_margin), 1-P.S_margin);
        S_2(j,j)  = s;
        zeta_2(j) = log((1+s)/(1-s));
        G_2(j,j)  = (exp(zeta_2(j))+1)^2/(2*exp(zeta_2(j))*p_h(j));
    end
    % trapezoidal integral of zeta_h3 (descent PI), anti-windup
    if cs.k == 1, cs.izeta3 = P.dt*zeta_2(3);
    else,         cs.izeta3 = cs.izeta3 + P.dt*(cs.zeta3_prev + zeta_2(3))/2; end
    cs.izeta3 = max(min(cs.izeta3, P.izeta2_max), -P.izeta2_max);
    cs.zeta3_prev = zeta_2(3);

    % combined sliding surface  sigma = zeta_h + chi*zeta_aug    [tex eq. sliding]
    sigma = [ zeta_2(1) + P.chi_r(1)*zeta_r(1); ...
              zeta_2(2) + P.chi_r(2)*zeta_r(2); ...
              zeta_2(3) + P.chi_z  *cs.izeta3 ];
    chi_zeta_aug = [ P.chi_r(1)*dzeta_r(1); P.chi_r(2)*dzeta_r(2); P.chi_z*zeta_2(3) ];

    % d/dt(h_d) including the smooth prescribed rate (s_ddot-drop removed), smooth4 + hard cap
    if cs.k == 1, rdh = zeros(3,1); else, rdh = (h_d_all - cs.h_d_noS_prev)/P.dt; end
    cs.h_d_noS_prev = h_d_all;
    cs.raw_dh = [cs.raw_dh(:,2:end), rdh];
    V_dh_d = smooth4(cs.raw_dh);
    if P.dhd_cap > 0, V_dh_d = max(min(V_dh_d, P.dhd_cap), -P.dhd_cap); end

    % known c-term (c_simple manuscript form)               [tex eq. c_h]
    c = -psi_dot_b*cross(e3, V_h) - dot(V_h, e3)*V_h - V_dh_d;

    Theta = [-c + S_2*dp_h - G_2\chi_zeta_aug, eye(3)];
    o.sigma = sigma; o.G_2 = G_2; o.S_2 = S_2; o.dp_h = dp_h; o.c = c;
    o.chi_zeta_aug = chi_zeta_aug; o.Theta_norm = norm(Theta,'fro');
    % per-axis row-norm theta_k = ||row_k(Theta)|| = sqrt(v_k^2+1); sqrt(sum_k theta_k^2)==||Theta||_F
    o.theta_perax = sqrt(sum(Theta.^2, 2));   % 3x1; used by asmc when P.theta_per_axis
    o.p_h = p_h;                              % optic-flow funnel envelope (3x1) for internals plot
    o.v   = Theta(:,1);                        % regressor first column (for per-axis (Y d_bar)_k plot)
    o.V_h_d = V_h_d; o.V_h_e = V_h_e;
end
