function [zeta_r, dzeta_r, p_r] = position_funnel(s_e_xy, ds_e_xy, t, P)
%POSITION_FUNNEL  Image-feature funnel barrier (tex eq. position barrier).
%   Maps the normalized image-position error r_bar_e = s_e_xy / phi_max onto the
%   unconstrained barrier coordinate zeta_r through the prescribed-performance
%   funnel p_r(t) = e^{-Xi_r t}(p_r0 - p_rinf) + p_rinf. zeta_r enters the combined
%   sliding surface directly (relative-degree-two; not back-mapped to a rate).
%   ds_e_xy is the MEASURED centroid rate (from tex eq. s dot).

    r_bar_e  = s_e_xy   ./ P.phi_max;            % normalized position error (FoV units)
    dr_bar_e = ds_e_xy  ./ P.phi_max;            % measured rate
    p_r  = expm(-P.Xi_r*t)*(P.p_r0 - P.p_rinf) + P.p_rinf;
    dp_r = -P.Xi_r*expm(-P.Xi_r*t)*(P.p_r0 - P.p_rinf);

    zeta_r = zeros(2,1); dzeta_r = zeros(2,1);
    for j = 1:2
        S_r = min(max(r_bar_e(j)/p_r(j), -1+P.S_margin), 1-P.S_margin);
        zeta_r(j)  = log((1+S_r)/(1-S_r));
        g_rj       = (exp(zeta_r(j))+1)^2 / (2*exp(zeta_r(j))*p_r(j));
        dzeta_r(j) = g_rj * (dr_bar_e(j) - S_r*dp_r(j));
    end
end
