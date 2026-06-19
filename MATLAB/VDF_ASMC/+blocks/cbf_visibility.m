function [I_a_cd_filt, th_safe, theta_cone, cbf_ok, R33, cs] = cbf_visibility(I_a_cd, I_R_C, yaw, C_nP, B_w_c, P, cs)
%CBF_VISIBILITY  Target-visibility CBF on the commanded tilt (tex eq. cbf qp).
%   First a 0.08 s LPF denoises the pseudo-inverse-amplified command, then the
%   camera-plane theta-QP (cbf2_filter, reused) projects the desired lean so every
%   feature point stays in the FoV, returning the safe lean th_safe (R_d built from
%   it). An inertial-z floor preserves a realizable thrust direction.

    cs.I_a_cd_filt = P.alpha_ia*cs.I_a_cd_filt + (1-P.alpha_ia)*I_a_cd;   % upstream LPF
    R33 = max(min(I_R_C(3,3), 1), -1);                                   % measured tilt cosine
    refresh = (mod(cs.k-1, P.ZOH) == 0);
    dt_img  = P.ZOH*P.dt;
    [cs.I_a_cd_filt, theta_cone, cbf_ok, th_safe, cs.cbf_state] = cbf2_filter( ...
        cs.I_a_cd_filt, I_R_C, R33, yaw, C_nP, P.f, P.phi_max_cbf, ...
        P.theta_cap, P.theta_cap, dt_img, refresh, B_w_c(1:2), cs.cbf_state);
    cs.I_a_cd_filt(3) = max(cs.I_a_cd_filt(3), P.a_floor);
    I_a_cd_filt = cs.I_a_cd_filt;
end
