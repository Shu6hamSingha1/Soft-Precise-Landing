function [zeta_r, dzeta_r, p_r, s_dot_presc] = position_funnel(s_e_xy, ds_e_xy, t, P)
%POSITION_FUNNEL  Image-feature funnel barrier (tex eq. position barrier).
%   Maps the normalized image-position error r_bar_e = s_e_xy / phi_max onto the
%   unconstrained barrier coordinate zeta_r through the prescribed-performance
%   funnel p_r(t) = e^{-Xi_r t}(p_r0 - p_rinf) + p_rinf. zeta_r enters the combined
%   sliding surface directly (relative-degree-two).
%   ds_e_xy is the MEASURED centroid rate (from tex eq. s dot).
%
%   ⚠ 2026-09-03 (PX4 parity port): the prescribed rate NOW CARRIES a back-mapped
%   convergence term -k_r*G_r^{-1}*zeta_r (P.hd_kr, PX4 PLASMC_HD_KR=0.5). The old
%   docstring claim "not back-mapped to a rate" is therefore no longer true, and
%   neither are the manuscript's three matching statements (lines 286, 290 and Remark
%   rem:normalization) -- those need updating alongside eq. `h_d final` and eq.
%   `h_e identity`. See MATLAB/PX4_PARITY_PORT_SPEC.md §B5.

    r_bar_e  = s_e_xy   ./ P.phi_max;            % normalized position error (FoV units)
    dr_bar_e = ds_e_xy  ./ P.phi_max;            % measured rate
    p_r  = expm(-P.Xi_r*t)*(P.p_r0 - P.p_rinf) + P.p_rinf;
    dp_r = -P.Xi_r*expm(-P.Xi_r*t)*(P.p_r0 - P.p_rinf);

    zeta_r = zeros(2,1); dzeta_r = zeros(2,1); S_rv = zeros(2,1); g_rv = zeros(2,1);
    for j = 1:2
        S_r = min(max(r_bar_e(j)/p_r(j), -1+P.S_margin), 1-P.S_margin);
        S_rv(j)    = S_r;
        zeta_r(j)  = log((1+S_r)/(1-S_r));
        g_rv(j)    = (exp(zeta_r(j))+1)^2 / (2*exp(zeta_r(j))*p_r(j));
        dzeta_r(j) = g_rv(j) * (dr_bar_e(j) - S_r*dp_r(j));
    end
    % Prescribed (funnel-bounded) centroid rate for h_d, replacing the measured
    % s_dot: p_10 .* S_r .* dp_r. Smooth (no finite-diff s_ddot into dh_d), and on a
    % breach S_r saturates -> h_d stays bounded -> h_e grows -> recovery authority.
    s_dot_presc = P.phi_max .* S_rv .* dp_r;

    % ---- HD_KR back-map convergence term (PX4 parity port 2026-09-03) -------------
    % PX4: _hd_rate = p_10*S_r*dp_r - p_10*hd_kr*zeta_r/g_r  (controller.py ~2692-2696,
    % operative path -- it feeds self._h_d, hence h_e and the sliding surface; the
    % k_r-free copy is kept only as the _h_d_kfree diagnostic).
    % Effect: h_e_xy = phi_max .* G_r^{-1} .* (dzeta_r + k_r*zeta_r) + h_e3*s_xy, so
    % driving h_e -> 0 gives EXPONENTIAL zeta_r convergence at rate k_r rather than
    % merely dzeta_r -> 0. Guarded so a params struct without the field is unchanged.
    % ⚠ k_r = 0.5 was baked on PX4 2026-06-29 inside a bundled re-bake and is UNSWEPT
    % (no 0.3/0.7/1.0 A/B on record). Set P.hd_kr = 0 to recover the pre-port law.
    if isfield(P, 'hd_kr') && P.hd_kr ~= 0
        s_dot_presc = s_dot_presc - P.phi_max .* (P.hd_kr .* zeta_r ./ g_rv);
    end
end
