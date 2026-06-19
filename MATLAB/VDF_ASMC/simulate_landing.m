function R = simulate_landing(x0, trajType, opts)
%SIMULATE_LANDING  Clean VDF-ASMC closed-loop landing simulation (single controller).
%   R = simulate_landing(x0, trajType, opts) runs the soft-precise-landing loop with
%   the one VDF-ASMC controller (blocks.*), the moving target traj_Gen(trajType), the
%   monocular image model, ground effect + 1-step actuator delay, and RK5 plant
%   integration. opts.noise (default false) toggles the pixel/wind robustness model.
%   Returns final_xy, final_rel_vel, success (soft+precise), landed, idx.
%
%   Reuses Common/{RK5,image_feature,smooth4,sat,traj_Gen,UAVDyn} and the leaf
%   blocks; gains come solely from vdf_params(). This is the manuscript controller.

    if nargin < 3, opts = struct(); end
    if ~isfield(opts,'noise'), opts.noise = false; end
    Constants; InitVar;                       % T_nP3, V_s_d, limits, r, GE, delay, ZOH
    P = vdf_params();
    NOISE = opts.noise;                       % noiseless reference by default

    % --- state init ---
    x_c = x0; I_p_c = x_c(1:3); q_c = x_c(4:7)/norm(x_c(4:7));
    I_v_c = x_c(8:10); B_w_c = x_c(11:13);
    yaw0 = atan2(2*(q_c(1)*q_c(4)+q_c(2)*q_c(3)), 1-2*(q_c(3)^2+q_c(4)^2));
    N_steps = numel(tRange);  e3=[0;0;1];

    % --- controller state cs ---
    cs = struct();
    cs.kappa = P.kappa0;  cs.kappa_a = P.kappa_a0;  cs.psi_d = yaw0;
    cs.izeta3 = 0; cs.zeta3_prev = 0; cs.ie_a = 0; cs.e_a_prev = 0;
    cs.ie_R = zeros(3,1); cs.thetahat = zeros(2,1);
    cs.V_2nP_i_prev = zeros(8,1); cs.V_w_i_prev = zeros(3,1);
    cs.V_s_prev = zeros(2,1); cs.h_d_noS_prev = zeros(3,1);
    cs.raw_ds = zeros(2,4); cs.raw_dh = zeros(3,4);
    cs.V_s_raw = zeros(4,N_steps); cs.V_h_raw = zeros(3,N_steps);
    cs.V_w_raw = zeros(3,N_steps); cs.V_dw_raw = zeros(3,N_steps);
    cs.I_a_cd_filt = -P.g;
    cs.cbf_state = struct('delta_prev',[],'ddelta_ref',zeros(2,1),'decode_fail_n',0, ...
                          'phase2_alpha',0.0,'cr_prev',[],'d',zeros(2,1),'Lw2_prev',[]);
    C_nP = zeros(2,4); V_s=zeros(4,1); V_h=zeros(3,1); V_w=zeros(3,1); V_nP_i=zeros(2,4);
    u_buf = zeros(4,N_steps);  landed=false; idx=N_steps; t0=0;
    LG = struct('Vs',zeros(4,N_steps),'Vh',zeros(3,N_steps),'sigma',zeros(3,N_steps), ...
                'Iacd',zeros(3,N_steps),'Iaf',zeros(3,N_steps),'Vhe',zeros(3,N_steps), ...
                'psid',zeros(1,N_steps),'Btau',zeros(3,N_steps), ...
                'x',zeros(13,N_steps),'Tcd',zeros(1,N_steps),'ths',zeros(2,N_steps));

    for k = 1:N_steps
        cs.k = k;  t = (k-1)*P.dt;
        I_R_C = quat2rotm(q_c');
        yaw = atan2(2*(q_c(1)*q_c(4)+q_c(2)*q_c(3)), 1-2*(q_c(3)^2+q_c(4)^2));
        I_R_V = rotz(rad2deg(yaw));
        tr = traj_Gen(t, trajType); x_t = tr(:,1); dx_t = tr(1:end-1,2);
        I_R_T = quat2rotm(x_t(4:7)');

        % --- image model (ZOH-gated: refresh corners every P.ZOH steps) ---
        if mod(k-1,P.ZOH) == 0
            I_nP3 = I_R_T*T_nP3 + x_t(1:3);
            C_s_tc = I_R_C'*(x_t(1:3) - I_p_c);
            C_nP = (P.f/(C_s_tc(3)+P.zf))*(I_R_C'*(I_nP3 - I_p_c));  C_nP = C_nP(1:2,:);
            if NOISE
                zd = max(abs(C_s_tc(3)),0.1);
                C_nP = C_nP + ((0.3+0.5/(zd+0.5))/P.f)*randn(size(C_nP));
            end
            if any(abs(C_nP(1,:))>P.res(1)/2) || any(abs(C_nP(2,:))>P.res(2)/2)
                idx = k-1; break;                                  % FoV violation
            end
        end
        [V_s, V_h, V_w, V_nP_i, cs] = blocks.image_features(C_nP, I_R_V, I_R_C, P, cs);

        % --- early landing check ---
        if abs(I_p_c(3)-x_t(3)) <= P.zf, landed=true; idx=k; break; end

        % --- outer loop: dual funnel + ASMC -> I_a_cd ---
        % ONE smooth4 centroid rate s_dot_meas feeds BOTH the position-funnel dr_bar_e
        % and the flow h_d (matches the canonical; using a raw diff for one diverges).
        s_e_xy = V_s(1:2) - V_s_d(1:2);
        if k==1, raw=[0;0]; else, raw=(s_e_xy - cs.s_e_prev)/P.dt; end
        cs.s_e_prev = s_e_xy;
        cs.raw_ds = [cs.raw_ds(:,2:end), raw];
        s_dot_meas = smooth4(cs.raw_ds);
        [zeta_r, dzeta_r] = blocks.position_funnel(s_e_xy, s_dot_meas, t, P);
        [o, cs] = blocks.flow_surface(V_s, V_h, B_w_c, I_R_C, zeta_r, dzeta_r, s_dot_meas, t, P, cs);
        [I_a_cd, cs] = blocks.asmc(o, I_R_V, P, cs);

        % --- visibility CBF + inner loop ---
        [I_a_filt, th_safe, ~, ~, R33, cs] = blocks.cbf_visibility(I_a_cd, I_R_C, yaw, C_nP, B_w_c, P, cs);
        [psi_d, ~, cs] = blocks.yaw_asmc(V_s(4), V_s_d(4), P, cs);
        [B_tau, T_cd, cs] = blocks.so3_tracker(I_a_filt, th_safe, R33, yaw, psi_d, I_R_C, B_w_c, P, cs);
        LG.Vs(:,k)=V_s; LG.Vh(:,k)=V_h; LG.sigma(:,k)=o.sigma; LG.Iacd(:,k)=I_a_cd;
        LG.Iaf(:,k)=I_a_filt; LG.psid(k)=psi_d; LG.Btau(:,k)=B_tau; LG.Vhe(:,k)=o.V_h_e;
        LG.x(:,k)=x_c; LG.Tcd(k)=T_cd; if isempty(th_safe), LG.ths(:,k)=[0;0]; else, LG.ths(:,k)=th_safe; end

        % --- ground effect + saturation + 1-step delay ---
        if GE, z_ge=-max(abs(x_c(3)),r); T_cd = T_cd/(1-(r/(4*z_ge))^2); end
        B_tau(1:2)=min(max(B_tau(1:2),-tau_xy_max),tau_xy_max);
        B_tau(3)=min(max(B_tau(3),-tau_z_max),tau_z_max);
        T_cd=max(min(T_cd,T_max),T_min);
        u_buf(:,k)=[B_tau;T_cd];
        if k>delay, u=u_buf(:,k-delay); else, u=[zeros(3,1); P.m*norm(P.g)]; end
        if any(isnan(u))||norm(u)>1e4, idx=k-1; break; end

        % --- plant integration ---
        x_c = RK5(@(t,x) UAVDyn(t,x,u), t0, x_c, P.dt);
        if any(isnan(x_c)), idx=k-1; break; end
        I_p_c=x_c(1:3); q_c=x_c(4:7)/norm(x_c(4:7)); I_v_c=x_c(8:10); B_w_c=x_c(11:13);

        % --- termination ---
        if abs(I_p_c(3)-x_t(3)) <= P.zf, landed=true; idx=k; break; end
        t0=t0+P.dt;
    end

    tr = traj_Gen((idx-1)*P.dt, trajType); x_t=tr(:,1); dx_t=tr(1:end-1,2);
    R.landed = landed;
    R.final_xy = norm(I_p_c(1:2) - x_t(1:2));
    R.final_rel_vel = norm(I_v_c - dx_t(1:3));
    R.success = landed && R.final_xy<=0.08 && R.final_rel_vel<=0.20;
    R.idx = idx;
    R.LG = LG;
end
