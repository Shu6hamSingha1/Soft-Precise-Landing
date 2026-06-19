function result = run_simulation(x0, trajType, K_override, speed_mult, cfg_override, seed)
%RUN_SIMULATION  Multi-init manuscript driver — wraps the VERIFIED VDF-ASMC blocks.
%   result = run_simulation(x0, trajType, K_override, speed_mult, cfg_override, seed)
%   runs the soft-precise-landing loop using the ONE verified controller
%   (MATLAB/VDF_ASMC/+blocks/*, reproduces the canonical bit-exact 25/25), the moving
%   target, the monocular image model, and the realistic robustness model
%   (UAVDyn_robust + wind + CoG offset + parameter uncertainty when NOISE). It packages
%   the full result + data struct the manuscript plotters read.
%
%   The control LOGIC is NOT here — it lives in the blocks (single source of truth),
%   so this driver cannot drift from the canonical. Everything here is the plant model,
%   the image model, logging (X_DS/P_DS/V_X_DS/x_t), fov_fail, and result packaging.

    mfile_dir = fileparts(mfilename('fullpath'));
    addpath(fullfile(mfile_dir, '..', 'Common'));
    addpath(fullfile(mfile_dir, 'plotters'));
    addpath(fullfile(mfile_dir, '..', 'VDF_ASMC'));   % blocks + vdf_params (the controller)
    if nargin < 3, K_override = []; end
    if nargin < 4 || isempty(speed_mult), speed_mult = 1.0; end
    if nargin < 5, cfg_override = []; end
    if nargin < 6, seed = []; end
    if ~isempty(seed)
        rng(seed);
    elseif isempty(K_override)
        rng('shuffle');
    end

    Constants;
    InitVar;
    init_robustness;          % robustness params (only active when NOISE=1)

    % Optional environment override (NOISE / GE / delay) for the sweep harnesses.
    if ~isempty(cfg_override)
        if isfield(cfg_override, 'NOISE'), NOISE = cfg_override.NOISE; end
        if isfield(cfg_override, 'GE'),    GE    = cfg_override.GE;    end
        if isfield(cfg_override, 'delay'), delay = cfg_override.delay; end
    end

    % Controller gains live in vdf_params (single source of truth). multi_Init_Var
    % passes K_override = []; keep h_rd / FILTER_WINDOW overrides for sweep harnesses.
    P = vdf_params();
    if ~isempty(K_override)
        if isfield(K_override, 'h_rd'),          P.h_rd = K_override.h_rd;          end
        if isfield(K_override, 'FILTER_WINDOW'), P.fw   = K_override.FILTER_WINDOW;  end
    end

    % --- state init ---
    x_c = x0; I_p_c = x_c(1:3); q_c = x_c(4:7)/norm(x_c(4:7));
    I_v_c = x_c(8:10); B_w_c = x_c(11:13);
    yaw0 = atan2(2*(q_c(1)*q_c(4)+q_c(2)*q_c(3)), 1-2*(q_c(3)^2+q_c(4)^2));
    N_steps = numel(tRange); e3 = [0;0;1];

    % --- controller state cs (the blocks read/write this; identical to simulate_landing) ---
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
    cs.V_s_i = zeros(4,1); cs.V_h_i = zeros(3,1); cs.V_w_i = zeros(3,1);
    cs.V_dw_i = zeros(3,1); cs.V_nP_i = zeros(2,4);

    % --- logging arrays the manuscript plotters / analyzers read ---
    U_DS   = zeros(4,  N_steps);
    X_DS   = zeros(13, N_steps + 1);  X_DS(:,1) = x_c;
    V_X_DS = zeros(24, N_steps);
    D_DS   = zeros(15, N_steps);       % [V_h_d(3); I_a_cd(3); e_R(3); tau(3); T; psi_d; u_a]
    P_DS   = zeros(2, 12, N_steps);
    x_t    = zeros(7,  N_steps);
    dx_t   = zeros(6,  N_steps);
    V_h_d  = zeros(3,  N_steps);
    V_h_e  = zeros(3,  N_steps);
    I_a_cd = zeros(3,  N_steps);
    sigma  = zeros(3,  N_steps);
    eR_log       = zeros(3, N_steps);   % e_R not exposed by so3_tracker block -> zeros
    B_tau_cd_log = zeros(3, N_steps);
    B_T_cd       = zeros(1, N_steps);
    psi_d_log    = zeros(1, N_steps);
    u_a_log      = zeros(1, N_steps);
    u_2_buf      = zeros(4, N_steps);
    raw_dw_a     = zeros(3, N_steps + 3);
    V_w_a_prev   = zeros(3,1);
    V_nP_i = zeros(2,4); V_nP_a = zeros(2,4); C_nP = zeros(2,4);
    V_s_a = zeros(4,1); V_h_a = zeros(3,1); V_w_a = zeros(3,1); V_dw_a = zeros(3,1);
    V_s = zeros(4,1); V_h = zeros(3,1); V_w = zeros(3,1);
    precise = false; soft = false;

    landed = false; fov_fail = false; fov_fail_t = NaN; t0 = 0; idx = N_steps;

    fprintf('Running Simulation (VDF-ASMC blocks)\n\n');

    for idx = 1:N_steps
        cs.k = idx;  t = (idx-1)*dt;
        I_R_C = quat2rotm(q_c');
        yaw = atan2(2*(q_c(1)*q_c(4)+q_c(2)*q_c(3)), 1-2*(q_c(3)^2+q_c(4)^2));
        I_R_V = rotz(rad2deg(yaw));

        traj_t = traj_Gen(t, trajType, speed_mult);
        x_t(:,idx)  = traj_t(:,1);
        dx_t(:,idx) = traj_t(1:end-1,2);
        I_R_T = quat2rotm(x_t(4:7,idx)');

        % --- image model (ZOH-gated): physical corners C_nP (+ aux/analytical) ---
        if mod(idx-1, P.ZOH) == 0
            I_nP3  = I_R_T*T_nP3 + x_t(1:3,idx);
            C_nP3  = I_R_C'*(I_nP3 - I_p_c);
            C_s_tc = I_R_C'*(x_t(1:3,idx) - I_p_c);
            C_nP   = (f/(C_s_tc(3)+zf))*C_nP3(1:2,:);
            if NOISE
                z_dep = max(abs(C_s_tc(3)), 0.1);
                sigma_px = px_sigma0 + px_sigma1 / (z_dep + px_depth_offset);
                C_nP = C_nP + (sigma_px/f)*randn(size(C_nP));
                if rand < outlier_prob
                    col = randi(size(C_nP,2));
                    C_nP(:,col) = C_nP(:,col) + (outlier_mag/f)*sign(randn(2,1));
                end
            end
            if any(abs(C_nP(1,:)) > res(1)/2) || any(abs(C_nP(2,:)) > res(2)/2)
                fov_fail = true; fov_fail_t = tRange(idx);
                fprintf('  BREAK: FoV violation at idx=%d (t=%.2f)\n', idx, tRange(idx));
                break;
            end
            % auxiliary virtual corners (P_DS cols 5-8) + analytical features (V_X_DS)
            V_nP3  = I_R_V'*(I_nP3 - I_p_c);
            V_s_tc = I_R_V'*(x_t(1:3,idx) - I_p_c);
            V_nP_a = (f/(V_s_tc(3)+zf))*V_nP3(1:2,:);
            V_s_a  = image_feature(V_nP_a/f);
            V_h_a  = I_R_V'*(dx_t(1:3,idx) - I_v_c)/(V_s_tc(3)+zf);
        end
        I_w_c = I_R_C*B_w_c;
        V_w_a = I_R_V'*(dx_t(4:6,idx) - [0;0;I_w_c(3)]);
        if idx == 1, raw_dw_a(:,idx+3) = zeros(3,1);
        else,        raw_dw_a(:,idx+3) = (V_w_a - V_w_a_prev)/dt; end
        V_dw_a = smooth4(raw_dw_a(:,end-3:end));
        V_w_a_prev = V_w_a;

        % --- VERIFIED controller: image features (single source = the blocks) ---
        [V_s, V_h, V_w, V_nP_i, cs] = blocks.image_features(C_nP, I_R_V, I_R_C, P, cs);

        % --- early landing check ---
        alt_above = abs(I_p_c(3) - x_t(3,idx));
        xy_err    = norm(I_p_c(1:2) - x_t(1:2,idx));
        rel_vel   = norm(I_v_c - dx_t(1:3,idx));
        if alt_above <= zf
            precise = xy_err <= 0.08;  soft = rel_vel <= 0.2;
            fprintf('Landed at t=%.2f s (alt=%.3f, xy=%.3f, v=%.3f, p=%d, s=%d)\n', ...
                    tRange(idx), alt_above, xy_err, rel_vel, precise, soft);
            landed = true; break;
        end

        % --- outer loop: ONE smooth4 s_dot_meas -> dual funnel + ASMC ---
        s_e_xy = V_s(1:2) - V_s_d(1:2);
        if idx==1, raw=[0;0]; else, raw=(s_e_xy - cs.s_e_prev)/dt; end
        cs.s_e_prev = s_e_xy;
        cs.raw_ds = [cs.raw_ds(:,2:end), raw];
        s_dot_meas = smooth4(cs.raw_ds);
        [zeta_r, dzeta_r] = blocks.position_funnel(s_e_xy, s_dot_meas, t, P);
        [o, cs] = blocks.flow_surface(V_s, V_h, B_w_c, I_R_C, zeta_r, dzeta_r, s_dot_meas, t, P, cs);
        [Iacd, cs] = blocks.asmc(o, I_R_V, P, cs);

        % --- visibility CBF + inner loop ---
        [I_a_filt, th_safe, ~, ~, R33, cs] = blocks.cbf_visibility(Iacd, I_R_C, yaw, C_nP, B_w_c, P, cs);
        [psi_d, u_a, cs] = blocks.yaw_asmc(V_s(4), V_s_d(4), P, cs);
        [B_tau_cd, T_cd, cs] = blocks.so3_tracker(I_a_filt, th_safe, R33, yaw, psi_d, I_R_C, B_w_c, P, cs);

        % --- ground effect + saturation + 1-step actuator delay ---
        if GE, z_ge = -max(abs(x_c(3)), r); T_cd = 1/(1-(r/(4*z_ge))^2) * T_cd; end
        B_tau_cd(1:2) = min(max(B_tau_cd(1:2), -tau_xy_max), tau_xy_max);
        B_tau_cd(3)   = min(max(B_tau_cd(3),   -tau_z_max),  tau_z_max);
        T_cd          = max(min(T_cd, T_max), T_min);

        u_2_buf(:,idx) = [B_tau_cd; T_cd];
        if idx > delay, u_2 = u_2_buf(:, idx - delay);
        else,           u_2 = [zeros(3,1); m*norm(g)]; end
        if any(isnan(u_2)) || norm(u_2) > 1e4
            fprintf('  BREAK: u_2 invalid at idx=%d (t=%.2f)\n', idx, tRange(idx)); break;
        end

        % --- robustness plant (wind / CoG / parameter uncertainty when NOISE) ---
        if NOISE
            F_turb  = F_turb + (dt/wind_tau) * (-F_turb + wind_sigma*randn(3,1));
            v_rel_w = wind_mean - x_c(8:10);
            F_wind_I = wind_mean*C_d_wind + F_turb + C_d_wind*v_rel_w;
            x_c = RK5(@(t, x) UAVDyn_robust(t, x, u_2, m_p, J_p, F_wind_I, r_cog), t0, x_c, dt);
        else
            x_c = RK5(@(t, x) UAVDyn(t, x, u_2), t0, x_c, dt);
        end
        if any(isnan(x_c)), fprintf('  BREAK: x_c NaN at idx=%d\n', idx); break; end
        I_p_c=x_c(1:3); q_c=x_c(4:7)/norm(x_c(4:7)); I_v_c=x_c(8:10); B_w_c=x_c(11:13);

        % --- logging (X_DS/P_DS/V_X_DS for the plotters; D_DS e_R slot = 0, unused) ---
        U_DS(:,idx)        = u_2;
        X_DS(:,idx+1)      = x_c;
        V_X_DS(:,idx)      = [cs.V_s_i(1:2); cs.V_s_i(4); cs.V_h_i; cs.V_w_i; cs.V_dw_i; ...
                              V_s_a(1:2); V_s_a(4); V_h_a; V_w_a; V_dw_a];
        V_h_d(:,idx)       = o.V_h_d;
        V_h_e(:,idx)       = o.V_h_e;
        I_a_cd(:,idx)      = Iacd;
        sigma(:,idx)       = o.sigma;
        B_tau_cd_log(:,idx)= B_tau_cd;
        B_T_cd(idx)        = T_cd;
        psi_d_log(idx)     = psi_d;
        u_a_log(idx)       = u_a;
        D_DS(:,idx)        = [o.V_h_d; Iacd; zeros(3,1); B_tau_cd; T_cd; psi_d; u_a];
        P_DS(:,:,idx)      = [V_nP_i, V_nP_a, C_nP];

        % --- termination ---
        alt_above = abs(I_p_c(3) - x_t(3,idx));
        xy_err    = norm(I_p_c(1:2) - x_t(1:2,idx));
        rel_vel   = norm(I_v_c - dx_t(1:3,idx));
        if alt_above <= zf
            precise = xy_err <= 0.08;  soft = rel_vel <= 0.2;
            fprintf('Landed at t=%.2f s (alt=%.3f, xy=%.3f, v=%.3f, p=%d, s=%d)\n', ...
                    tRange(idx), alt_above, xy_err, rel_vel, precise, soft);
            landed = true; break;
        end
        t0 = t0 + dt;
    end

    result.success     = landed && ~fov_fail;
    result.final_error = norm(I_p_c - x_t(1:3,idx));
    result.final_t     = tRange(idx);
    result.final_xy    = norm(I_p_c(1:2) - x_t(1:2,idx));
    result.final_alt   = abs(I_p_c(3) - x_t(3,idx));
    result.final_rel_vel = norm(I_v_c - dx_t(1:3,idx));
    result.precise     = result.success && (result.final_xy <= 0.08);
    result.soft        = result.success && (result.final_rel_vel <= 0.2);
    result.fov_fail    = fov_fail;
    result.fov_fail_t  = fov_fail_t;

    if ~landed, idx = idx - 1; end

    scratch = [tempname, '.mat'];
    save(scratch);
    result.data = load(scratch);
    delete(scratch);
end
