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
    Npts = size(T_nP3, 2);    % feature-point count (4 = legacy quad, 5 = cross marker)

    % Optional environment override (NOISE / GE / delay / lag) for the sweep harnesses.
    if ~isempty(cfg_override)
        if isfield(cfg_override, 'NOISE'), NOISE = cfg_override.NOISE; end
        if isfield(cfg_override, 'GE'),    GE    = cfg_override.GE;    end
        if isfield(cfg_override, 'delay'), delay = cfg_override.delay; end
    end

    % --- PX4-SITL lag model (default OFF -> the pure delay=1 path is bit-exact) ---
    %   Enable with cfg_override.lag = 1 (defaults) or a struct to tune fields:
    %     tau_act    first-order hold on roll/pitch torque + thrust  [s]
    %     tau_yaw    first-order hold on yaw torque                  [s]
    %     meas_delay transport delay on measured corners -> image_features [s]
    %
    %   ⚠ CALIBRATED 2026-09-10 (gate_lag_sweep / _md): the TORQUE holds (tau_act/
    %   tau_yaw) add a pole to the attitude loop, which so3_tracker assumes is
    %   instantaneous -> they destabilise the INNER loop (validated config 14/25 at
    %   tau_act=20 ms, 0/25 at 38 ms), NOT the h_rd/chi_r outer loop. PX4's real
    %   "38 ms rate-loop lag" is cmd-rate -> achieved-rate with a PX4 rate loop
    %   around it; MATLAB integrates torque directly and has no rate loop, so a
    %   torque hold is the WRONG surrogate. Use meas_delay (perception latency,
    %   the lever PX4 docs tie h_rd/chi_r to) and keep it <= ~0.03 s -- MATLAB's
    %   own tune cliffs hard past that. See project_matlab_px4_lag_model_2026_09_10.
    LAG = struct('on',false,'tau_act',0.038,'tau_yaw',0.287,'meas_delay',0.16);
    if ~isempty(cfg_override) && isfield(cfg_override,'lag') && ~isempty(cfg_override.lag)
        lg = cfg_override.lag;
        if isstruct(lg)
            LAG.on = true;
            for fn = fieldnames(lg)', LAG.(fn{1}) = lg.(fn{1}); end
        elseif lg   % scalar truthy -> defaults
            LAG.on = true;
        end
    end
    a_act = LAG.tau_act / (LAG.tau_act + dt);      % first-order hold coeff, roll/pitch/thrust
    a_yaw = LAG.tau_yaw / (LAG.tau_yaw + dt);      % first-order hold coeff, yaw torque
    md_n  = max(0, round(LAG.meas_delay / dt));    % measurement transport delay [steps]

    % Controller gains live in vdf_params (single source of truth). multi_Init_Var
    % passes K_override = []; keep h_rd / FILTER_WINDOW overrides for sweep harnesses.
    P = vdf_params();
    if ~isempty(K_override)
        if isfield(K_override, 'h_rd'),          P.h_rd      = K_override.h_rd;          end
        if isfield(K_override, 'FILTER_WINDOW'), P.fw        = K_override.FILTER_WINDOW;  end
        if isfield(K_override, 'theta_cap'),     P.theta_cap = K_override.theta_cap;     end
        if isfield(K_override, 'yaw_rate_law'),  P.yaw_rate_law = K_override.yaw_rate_law; end
        if isfield(K_override, 'yrl_kp'),        P.yrl_kp    = K_override.yrl_kp;        end
        if isfield(K_override, 'yrl_ki'),        P.yrl_ki    = K_override.yrl_ki;        end
        if isfield(K_override, 'yrl_wz_sign'),   P.yrl_wz_sign = K_override.yrl_wz_sign; end
        if isfield(K_override, 'cbf_two_tier'),  P.cbf_two_tier = K_override.cbf_two_tier; end
        if isfield(K_override, 'cbf_drift_tau'), P.cbf_drift_tau = K_override.cbf_drift_tau; end
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
    cs.V_2nP_i_prev = zeros(2*Npts,1); cs.V_w_i_prev = zeros(3,1);
    cs.V_s_prev = zeros(2,1); cs.h_d_noS_prev = zeros(3,1);
    cs.raw_ds = zeros(2,4); cs.raw_dh = zeros(3,4);
    cs.V_s_raw = zeros(4,N_steps); cs.V_h_raw = zeros(3,N_steps);
    cs.V_w_raw = zeros(3,N_steps); cs.V_dw_raw = zeros(3,N_steps);
    cs.I_a_cd_filt = -P.g;
    cs.cbf_state = struct('delta_prev',[],'ddelta_ref',zeros(2,1),'decode_fail_n',0, ...
                          'phase2_alpha',0.0,'cr_prev',[],'d',zeros(2,1),'Lw2_prev',[]);
    cs.V_s_i = zeros(4,1); cs.V_h_i = zeros(3,1); cs.V_w_i = zeros(3,1);
    cs.V_dw_i = zeros(3,1); cs.V_nP_i = zeros(2,Npts);

    % --- logging arrays the manuscript plotters / analyzers read ---
    U_DS   = zeros(4,  N_steps);
    X_DS   = zeros(13, N_steps + 1);  X_DS(:,1) = x_c;
    V_X_DS = zeros(24, N_steps);
    D_DS   = zeros(15, N_steps);       % [V_h_d(3); I_a_cd(3); e_R(3); tau(3); T; psi_d; u_a]
    P_DS   = zeros(2, 3*Npts, N_steps);   % [V_nP_i | V_nP_a | C_nP], Npts cols each
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
    % --- internals-figure logs (pure logging; do not affect the sim) ---
    kappa_log     = zeros(3, N_steps);  % adaptive switching gain kappa(t)
    kappa_a_log   = zeros(1, N_steps);  % yaw adaptive gain kappa_a(t)
    sigma_a_log   = zeros(1, N_steps);  % yaw sliding surface sigma_a(t) = the yaw disturbance kappa_a rejects
    e_a_log       = zeros(1, N_steps);  % yaw orientation error e_a(t)
    theta_cone_log= zeros(1, N_steps);  % CBF tilt-cone bound for thrust/accel plot
    s_e_log       = zeros(2, N_steps);  % lateral centroid feature error s_e_xy (for r_bar_e)
    p_r_log       = zeros(2, N_steps);  % position (image-feature) funnel envelope p_r(t)
    p_h_log       = zeros(3, N_steps);  % optic-flow funnel envelope p_h(t)
    % lumped disturbance reconstruction (tex eq. h_e dot: d_h = h_e_dot - beta*u_h - c_h,
    % u_h=-a_d=-V_a_cd, beta=1/z). d_bar = beta_min^-1 [beta-1; d_h] formed in the plotter.
    d_h_log       = zeros(3, N_steps);  % reconstructed optic-flow disturbance d_h(t)
    beta_log      = zeros(1, N_steps);  % depth scale beta = 1/z(t)
    v_log         = zeros(3, N_steps);  % regressor first column v(t) (for (Y d_bar)_k per axis)
    V_h_e_prev    = zeros(3, 1);        % for the h_e finite difference
    F_known_log   = zeros(3, N_steps);  % injected known-disturbance force (KNOWN_DIST hook)
    au_comp_log   = zeros(3, N_steps);  % a_u component norms [reach; switch; equiv] (asmc decomposition)
    I_a_filt_log  = zeros(3, N_steps);  % CBF-FILTERED (delivered) accel vs commanded Iacd -> saturation
    hd_comp_log   = zeros(9, N_steps);  % h_d components [sdot(3); rot(3); desc(3)] -> what stops h_e converging
    F_wind_I      = zeros(3, 1);        % init (only updated in the NOISE plant branch)
    u_2_buf      = zeros(4, N_steps);
    raw_dw_a     = zeros(3, N_steps + 3);
    V_w_a_prev   = zeros(3,1);
    u_2_lag      = [zeros(3,1); m*norm(g)];   % first-order actuation-lag state (LAG.on)
    C_nP_buf     = cell(1, N_steps);          % measured-corner history for the transport delay
    V_nP_i = zeros(2,Npts); V_nP_a = zeros(2,Npts); C_nP = zeros(2,Npts);
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
        % Perception/pipeline transport lag: feed the controller the corners as
        % measured md_n control steps ago (physical FoV check above still uses the
        % true current C_nP). No-op when LAG.on is false (md_n unused).
        C_nP_buf{idx} = C_nP;
        if LAG.on && md_n > 0 && idx > md_n
            C_nP_meas = C_nP_buf{idx - md_n};
        else
            C_nP_meas = C_nP;
        end
        [V_s, V_h, V_w, V_nP_i, cs] = blocks.image_features(C_nP_meas, I_R_V, I_R_C, P, cs);

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
        [zeta_r, dzeta_r, p_r, s_dot_presc] = blocks.position_funnel(s_e_xy, s_dot_meas, t, P);
        [o, cs] = blocks.flow_surface(V_s, V_h, B_w_c, I_R_C, zeta_r, dzeta_r, s_dot_presc, t, P, cs);
        [Iacd, cs] = blocks.asmc(o, I_R_V, P, cs);

        % --- visibility CBF + inner loop ---
        cs.cbf_Vhxy = V_h(1:2);   % de-rotated optic flow -> Tier-1 moving-target lead (condition_drift'd inside)
        [I_a_filt, th_safe, theta_cone, ~, R33, cs] = blocks.cbf_visibility(Iacd, I_R_C, yaw, C_nP, B_w_c, P, cs);
        [psi_d, u_a, cs] = blocks.yaw_asmc(V_s(4), V_s_d(4), V_w(3), P, cs);
        [B_tau_cd, T_cd, cs] = blocks.so3_tracker(I_a_filt, th_safe, R33, yaw, psi_d, I_R_C, B_w_c, P, cs);

        % --- ground effect + saturation + 1-step actuator delay ---
        if GE, z_ge = -max(abs(x_c(3)), r); T_cd = 1/(1-(r/(4*z_ge))^2) * T_cd; end
        B_tau_cd(1:2) = min(max(B_tau_cd(1:2), -tau_xy_max), tau_xy_max);
        B_tau_cd(3)   = min(max(B_tau_cd(3),   -tau_z_max),  tau_z_max);
        T_cd          = max(min(T_cd, T_max), T_min);

        u_2_buf(:,idx) = [B_tau_cd; T_cd];
        if idx > delay, u_2 = u_2_buf(:, idx - delay);
        else,           u_2 = [zeros(3,1); m*norm(g)]; end
        % PX4 actuation lag: first-order hold on the delayed command -- ~38 ms on
        % roll/pitch torque + thrust, ~287 ms on yaw torque (measured, memory
        % feedback_impulse_response). Bypassed when LAG.on is false.
        if LAG.on
            aa = [a_act; a_act; a_yaw; a_act];
            u_2_lag = aa.*u_2_lag + (1-aa).*u_2;
            u_2 = u_2_lag;
        end
        if any(isnan(u_2)) || norm(u_2) > 1e4
            fprintf('  BREAK: u_2 invalid at idx=%d (t=%.2f)\n', idx, tRange(idx)); break;
        end

        % --- robustness plant (wind / CoG / parameter uncertainty when NOISE) ---
        if NOISE
            F_turb  = F_turb + (dt/wind_tau) * (-F_turb + wind_sigma*randn(3,1));
            v_rel_w = wind_mean - x_c(8:10);
            F_wind_I = wind_mean*C_d_wind + F_turb + C_d_wind*v_rel_w;
            % KNOWN-DISTURBANCE injection (default OFF): a deterministic, fully-known
            % external force (N), optionally stepped on [t_on,t_off). init_robustness
            % has already zeroed all stochastic disturbance, so F_wind_I == the injection.
            if ~isempty(KNOWN_DIST)
                ton = 0; toff = inf;
                if isfield(KNOWN_DIST,'t_on'),  ton  = KNOWN_DIST.t_on;  end
                if isfield(KNOWN_DIST,'t_off'), toff = KNOWN_DIST.t_off; end
                if t0 >= ton && t0 < toff, F_wind_I = KNOWN_DIST.force(:); else, F_wind_I = zeros(3,1); end
            end
            F_known_log(:,idx) = F_wind_I;     %#ok<AGROW>  log the injected force
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
        if isfield(cs,'au_reach'), au_comp_log(:,idx) = [cs.au_reach; cs.au_sw; cs.au_eq]; end
        I_a_filt_log(:,idx) = I_a_filt;     % delivered (CBF-capped) accel
        if isfield(o,'hd_sdot'), hd_comp_log(:,idx) = [o.hd_sdot; o.hd_rot; o.hd_desc]; end
        B_tau_cd_log(:,idx)= B_tau_cd;
        B_T_cd(idx)        = T_cd;
        psi_d_log(idx)     = psi_d;
        u_a_log(idx)       = u_a;
        D_DS(:,idx)        = [o.V_h_d; Iacd; zeros(3,1); B_tau_cd; T_cd; psi_d; u_a];
        P_DS(:,:,idx)      = [V_nP_i, V_nP_a, C_nP];
        % internals-figure logs
        kappa_log(:,idx)    = cs.kappa;
        kappa_a_log(idx)    = cs.kappa_a;
        if isfield(cs,'sigma_a'), sigma_a_log(idx)=cs.sigma_a; e_a_log(idx)=cs.e_a; end
        theta_cone_log(idx) = theta_cone;
        s_e_log(:,idx)      = s_e_xy;
        p_r_log(:,idx)      = p_r;
        p_h_log(:,idx)      = o.p_h;
        % --- lumped disturbance d_h reconstruction (the disturbance kappa rejects) ---
        z_depth = max(abs(I_p_c(3) - x_t(3,idx)), zf);   % depth above target
        beta_k  = 1/z_depth;                              % depth scale beta = 1/z
        V_a_cd  = I_R_V' * (Iacd + P.g);                  % a_d in V frame (asmc: Iacd=I_R_V*V_a_cd-P.g)
        if idx == 1, he_dot = zeros(3,1); else, he_dot = (o.V_h_e - V_h_e_prev)/dt; end
        V_h_e_prev      = o.V_h_e;
        d_h_log(:,idx)  = he_dot + beta_k*V_a_cd - o.c;   % d_h = h_e_dot - beta*u_h - c_h (u_h=-V_a_cd)
        beta_log(idx)   = beta_k;
        v_log(:,idx)    = o.v;                            % regressor column (Y=[v|I3])

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
    result.lag         = LAG;   % PX4-SITL lag model state (LAG.on false => bit-exact legacy path)

    if ~landed, idx = idx - 1; end

    scratch = [tempname, '.mat'];
    save(scratch);
    result.data = load(scratch);
    delete(scratch);
end
