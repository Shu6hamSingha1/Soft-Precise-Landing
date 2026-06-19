function [B_tau, T_cd, cs] = so3_tracker(I_a_cd_filt, th_safe, R33, yaw, psi_d, I_R_C, B_w_c, P, cs)
%SO3_TRACKER  Geometric SO(3) attitude tracker (tex eq. so3 torque).
%   Builds R_d from the CBF-safe lean th_safe (Fix B: body-z directly from th_safe)
%   and heading a_h = [cos psi_d, sin psi_d, 0] via Gram-Schmidt, then the geometric
%   torque tau = -kR e_R - kOmega e_Omega + w x Jw. Thrust T = m|a_{d,z}|/R33 uses
%   the measured tilt cosine. A thrust-scaled adaptive CoG feedforward cancels the
%   constant r_cog x f body torque (Lee-style; default-on, gamma_cog).

    I_F = P.m * I_a_cd_filt;  f_mag = norm(I_F);  T_cd = f_mag;
    if f_mag < 1e-6
        R_d = eye(3);
    else
        if ~isempty(th_safe)                                   % Fix B: lean from CBF
            a_xy = [cos(yaw)*th_safe(1) - sin(yaw)*th_safe(2); ...
                    sin(yaw)*th_safe(1) + cos(yaw)*th_safe(2)];
            rd3  = [-a_xy; 1]; rd3 = rd3/norm(rd3);
            T_cd = P.m*abs(I_a_cd_filt(3))/max(R33,1e-3);       % measured-tilt thrust
        else
            rd3 = -I_F/f_mag;                                  % force-vector fallback
        end
        a_h = [cos(psi_d); sin(psi_d); 0];
        rd2 = cross(rd3, a_h); n2 = norm(rd2);
        if n2 < 1e-6, rd2 = [0;1;0]; n2 = 1; end
        rd2 = rd2/n2; rd1 = cross(rd2, rd3);
        R_d = [rd1, rd2, rd3];
    end

    eR_mat = 0.5*(R_d'*I_R_C - I_R_C'*R_d);
    e_R    = [eR_mat(3,2); eR_mat(1,3); eR_mat(2,1)];          % vee map
    e_Omega = B_w_c;                                           % Omega_d = 0

    cs.ie_R = max(min(cs.ie_R + e_R*P.dt, P.ie_R_max), -P.ie_R_max);
    if P.gamma_cog > 0                                         % adaptive CoG feedforward
        e_comp = e_Omega(1:2) + P.cog_c2*e_R(1:2);
        cs.thetahat = cs.thetahat + P.dt*(P.gamma_cog*T_cd*e_comp - P.cog_leak*cs.thetahat);
        cs.thetahat = max(min(cs.thetahat, P.cog_max), -P.cog_max);
        tau_cog = [-T_cd*cs.thetahat; 0];
    else
        tau_cog = zeros(3,1);
    end
    B_tau = -P.kR*e_R - P.kI_R*cs.ie_R - P.kOmega*e_Omega + cross(B_w_c, P.J*B_w_c) + tau_cog;
end
