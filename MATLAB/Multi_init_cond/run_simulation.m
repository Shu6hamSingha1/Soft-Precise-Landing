function result = run_simulation(x0, trajType, K_override, speed_mult, cfg_override, seed)
    if nargin < 3, K_override = []; end
    if nargin < 4 || isempty(speed_mult), speed_mult = 1.0; end
    if nargin < 5, cfg_override = []; end
    if nargin < 6, seed = []; end
    load("bestParam.mat");
    if ~isempty(seed)
        rng(seed);
    elseif isempty(K_override)
        rng('shuffle');
    end

    Constants;
    InitVar;

    % Optional environment override (NOISE / GE / delay) for sweep harnesses
    % that need to disable disturbances without editing InitVar.m.
    if ~isempty(cfg_override)
        if isfield(cfg_override, 'NOISE'), NOISE = cfg_override.NOISE; end
        if isfield(cfg_override, 'GE'),    GE    = cfg_override.GE;    end
        if isfield(cfg_override, 'delay'), delay = cfg_override.delay; end
    end

    % Override initial state — also re-derive component variables
    % so the first loop iteration sees the correct IC.
    x_c   = x0;
    I_p_c = x_c(1:3);
    q_c   = x_c(4:7);   q_c = q_c / norm(q_c);
    I_v_c = x_c(8:10);
    B_w_c = x_c(11:13);
    X_DS  = x_c;

    %% =====================================================================
    %  PLASMC GAINS — Geometric SO(3) inner-loop baseline (2026-04-13)
    % =====================================================================
    K_ctrl = struct();

    K_ctrl.gamma_1  = [0.2, 0.2];
    K_ctrl.p_10     = K.p_10;
    K_ctrl.p_1inf   = [0.08; 0.08];

    K_ctrl.zp = diag([6.0, 6.0]);                 % prior 25/25 baseline
    K_ctrl.zi = diag([0.1, 0.1]);
    K_ctrl.zd = diag([1.3, 1.3]);

    K_ctrl.gamma_2  = [0.2, 0.2, 0.2];            % prior 25/25 baseline
    K_ctrl.p_20     = [25.0; 25.0; 8.0];  % widened for faster traj_Gen targets
    K_ctrl.p_2inf   = [0.8;  0.8;  1.0];          % prior 25/25 baseline

    K_ctrl.Omega   = diag([0.003, 0.003, 0.006]);
    K_ctrl.Gamma   = diag([0.4375, 0.5,   0.75 ]); % lateral symmetry lock (IC=±2)
    K_ctrl.P       = diag([1.5,   1.5,   5.0  ]);
    K_ctrl.N       = diag([0.02,  0.02,  0.05 ]);
    K_ctrl.kappa_0 = [0.125; 0.125; 0.25];
    K_ctrl.E       = diag([1.0,   1.0,   0.5  ]);

    % Geometric SO(3) attitude gains (tuned for X500 Gazebo inertia)
    K_ctrl.kR     = diag([1.5, 1.5, 0.5]);
    K_ctrl.kOmega = diag([0.3, 0.3, 0.1]);

    % Yaw adaptive SMC — generates heading reference psi_d (no compass)
    % e_a = V_s(4) - V_s_d(4) = alpha - alpha_d (image-based)
    % psi_d(t) = psi_d(t-1) + u_a*dt (integrated ASMC rate)
    % R_d uses psi_d as heading vector; geometric controller tracks R_d
    K_ctrl.Omega_a   = 0.5;
    K_ctrl.Gamma_a   = 0.5;
    K_ctrl.n_a       = 1.0;
    K_ctrl.p_a       = 2;
    K_ctrl.kappa_a_0 = 2.0;                       % pre-seed for high-wz rotating targets
    K_ctrl.E_a       = 3.0;                       % wide boundary layer smooths sat*kappa_a

    % Apply optional gain overrides (used by sweep harness)
    if ~isempty(K_override)
        ovr_fields = fieldnames(K_override);
        for ii = 1:numel(ovr_fields)
            K_ctrl.(ovr_fields{ii}) = K_override.(ovr_fields{ii});
        end
    end

    %% =====================================================================
    %  PRE-ALLOCATE ARRAYS
    % =====================================================================
    N_steps = numel(tRange);

    U_DS      = zeros(4,  N_steps);
    X_DS      = zeros(13, N_steps + 1);   X_DS(:,1) = x_c;
    V_X_DS    = zeros(24, N_steps);
    D_DS      = zeros(15, N_steps);       % [V_h_d(3); I_a_cd(3); e_R(3); tau(3); T(1); psi_d(1); u_a(1)]
    P_DS      = zeros(2,  12, N_steps);

    x_t       = zeros(7,  N_steps);
    dx_t      = zeros(6,  N_steps);

    V_h_d     = zeros(3,  N_steps);
    V_h_e     = zeros(3,  N_steps);
    I_a_cd    = zeros(3,  N_steps);

    % Inner-loop logging (geometric SO(3))
    eR_log       = zeros(3, N_steps);
    B_tau_cd_log = zeros(3, N_steps);
    B_T_cd       = zeros(1, N_steps);
    psi_d_log    = zeros(1, N_steps);
    u_a_log      = zeros(1, N_steps);

    % Delay buffer for u_2 = [tau; T]
    u_2_buf      = zeros(4, N_steps);

    % Raw visual signal storage for Savitzky-Golay filter (ACTUAL mode)
    V_s_raw   = zeros(4,  N_steps);
    V_h_raw   = zeros(3,  N_steps);
    V_w_raw   = zeros(3,  N_steps);
    V_dw_raw  = zeros(3,  N_steps);

    % V_w_a buffer for smooth derivative
    raw_dw_a  = zeros(3,  N_steps + 3);

    % _prev variables
    V_2nP_i_prev = zeros(8, 1);
    V_w_i_prev   = zeros(3, 1);
    V_w_a_prev   = zeros(3, 1);

    % Initial desired heading = current UAV yaw
    yaw_init = atan2(2*(q_c(1)*q_c(4) + q_c(2)*q_c(3)), ...
                     1 - 2*(q_c(3)^2 + q_c(4)^2));
    psi_d    = yaw_init;

    % I_a_cd low-pass filter state (matches tau_w=0.08s ~ 2Hz cutoff);
    % mimics the PID cascade's implicit filtering budget so R_d/e_R
    % don't chatter on noisy outer-loop output
    tau_ia      = 0.08;   % LPF tau on I_a_cd (noise absorption dominates phase lag)
    alpha_ia    = tau_ia / (tau_ia + dt);
    I_a_cd_filt = -g;   % hover initial condition

    % Initialise image-block variables so they persist across ZOH steps
    V_nP_i  = zeros(2, 4);
    V_nP_a  = zeros(2, 4);
    C_nP    = zeros(2, 4);
    L_s     = zeros(8, 6);
    V_s_i   = zeros(4, 1);
    V_s_a   = zeros(4, 1);
    V_h_i   = zeros(3, 1);
    V_h_a   = zeros(3, 1);
    V_w_i   = zeros(3, 1);
    V_dw_i  = zeros(3, 1);
    V_s     = zeros(4, 1);
    V_h     = zeros(3, 1);
    V_w     = zeros(3, 1);
    V_dw    = zeros(3, 1);

    % PLASMC-specific arrays
    p_1         = zeros(2, N_steps);
    dp_1        = zeros(2, N_steps);
    p_2         = zeros(3, N_steps);
    dp_2        = zeros(3, N_steps);
    S_1         = zeros(2, 2, N_steps);
    G_1         = zeros(2, 2, N_steps);
    zeta_1      = zeros(2, N_steps);
    izeta_1     = zeros(2, N_steps);
    raw_dzeta_1 = zeros(2, N_steps + 3);
    S_2         = zeros(3, 3, N_steps);
    G_2         = zeros(3, 3, N_steps);
    zeta_2      = zeros(3, N_steps);
    izeta_2     = zeros(3, N_steps);
    sigma       = zeros(3, N_steps);
    raw_dh_d    = zeros(3, N_steps + 3);
    V_s_e       = zeros(2, N_steps);
    kappa       = [K_ctrl.kappa_0, zeros(3, N_steps)];
    kappa_a     = [K_ctrl.kappa_a_0, zeros(1, N_steps)];
    e_a         = zeros(1, N_steps);
    ie_a        = zeros(1, N_steps);

    landed = false;

    fprintf('Running Simulation for Adaptive - PID Flight Controller\n\n' );

    %% Main simulation loop
    for idx=1:N_steps
    % *********************************************************************
    % Compute rotation matrices and yaw
    % *********************************************************************
        I_R_C = quat2rotm(q_c');
        E_cr = quat2eul(q_c', 'XYZ');

        yaw = atan2(2*(q_c(1)*q_c(4) + q_c(2)*q_c(3)), ...
            1 - 2*(q_c(3)^2 + q_c(4)^2));

        I_R_V = rotz(rad2deg(yaw));

    % *********************************************************************
    % Simulating moving target
    % *********************************************************************
        traj_t = traj_Gen((idx-1)*dt, trajType, speed_mult);

        x_t(:,idx) = traj_t(:,1);
        dx_t(:,idx) = traj_t(1:end-1,2);

        I_R_T = quat2rotm(x_t(4:7,idx)');

    if mod(idx-1,ZOH) == 0
    % *********************************************************************
    % Image Feature Points
    % *********************************************************************
        I_nP3 = I_R_T * T_nP3 + x_t(1:3,idx);
        C_nP3 = transpose(I_R_C)*(I_nP3 - I_p_c);
        C_s_tc = transpose(I_R_C)*(x_t(1:3,idx) - I_p_c);
        C_nP = (f/(C_s_tc(3)+zf))*C_nP3(1:2,:);

        if NOISE
            C_nP = awgn(C_nP, 50, 'measured');
        end

    % *********************************************************************
    % Scale-Independent Target Image Parameters
    % *********************************************************************
        V_R_C = I_R_V' * I_R_C;
        rays = [C_nP; f*ones(1,size(C_nP, 2))];
        vr = V_R_C * rays;
        V_nP_i = f*vr(1:2,:)./vr(3,:);
        for j=1:size(C_nP,2)
            L_s(2*j-1:2*j,:) = [f, 0, -V_nP_i(1,j), -V_nP_i(1,j)*V_nP_i(2,j)/f, (f^2+V_nP_i(1,j)^2)/f, -V_nP_i(2,j);
                0, f, -V_nP_i(2,j), -(f^2+V_nP_i(2,j)^2)/f, V_nP_i(1,j)*V_nP_i(2,j)/f, V_nP_i(1,j)];
        end
        V_s_i = image_feature(V_nP_i/f);

        V_2nP_i = reshape(V_nP_i,[],1);

        if idx == 1
            dPdt = zeros(size(V_2nP_i));
        else
            dPdt = (V_2nP_i - V_2nP_i_prev)/dt/ZOH;
        end
        V_2nP_i_prev = V_2nP_i;

        V_v_i = pinv(L_s,4)*dPdt;
        V_h_i = V_v_i(1:3);
        V_w_i = V_v_i(4:6);

        if idx == 1
            V_dw_i = zeros(3,1);
        else
            V_dw_i = (V_w_i - V_w_i_prev)/dt/ZOH;
        end
        V_w_i_prev = V_w_i;

    % *********************************************************************
    % Analytical Image Parameters
    % *********************************************************************
        V_nP3 = transpose(I_R_V)*(I_nP3 - I_p_c);
        V_s_tc = transpose(I_R_V)*(x_t(1:3,idx) - I_p_c);
        V_nP_a = (f/(V_s_tc(3)+zf))*V_nP3(1:2,:);
        V_s_a = image_feature(V_nP_a/f);
        V_h_a = transpose(I_R_V) * (dx_t(1:3,idx) - I_v_c) / (V_s_tc(3)+zf);
    end

        I_w_c = I_R_C * B_w_c;
        V_w_a = transpose(I_R_V) * (dx_t(4:6,idx) - [0;0;I_w_c(3)]);

        if idx == 1
            raw_dw_a(:,idx+3) = zeros(3,1);
        else
            raw_dw_a(:,idx+3) = (V_w_a - V_w_a_prev)/dt;
        end
        V_dw_a = smooth4(raw_dw_a(:,end-3:end));
        V_w_a_prev = V_w_a;

    % *********************************************************************
    % Actual vs Analytical (Savitzky-Golay filter)
    % *********************************************************************
        if ACTUAL
            V_s_raw(:,idx) = V_s_i;
            V_h_raw(:,idx) = V_h_i;
            V_w_raw(:,idx) = V_w_i;
            V_dw_raw(:,idx) = V_dw_i;

            if idx < FILTER_WINDOW
                V_s  = mean(V_s_raw(:,  1:idx), 2);
                V_h  = mean(V_h_raw(:,  1:idx), 2);
                V_w  = mean(V_w_raw(:,  1:idx), 2);
                V_dw = mean(V_dw_raw(:, 1:idx), 2);
            else
                V_s_vec = sgolayfilt(V_s_raw(:,idx-FILTER_WINDOW+1:idx),2,FILTER_WINDOW,[],2);
                V_s = V_s_vec(:, end);
                V_h_vec = sgolayfilt(V_h_raw(:,idx-FILTER_WINDOW+1:idx),2,FILTER_WINDOW,[],2);
                V_h = V_h_vec(:, end);
                V_w_vec = sgolayfilt(V_w_raw(:,idx-FILTER_WINDOW+1:idx),2,FILTER_WINDOW,[],2);
                V_w = V_w_vec(:, end);
                V_dw_vec = sgolayfilt(V_dw_raw(:,idx-FILTER_WINDOW+1:idx),2,FILTER_WINDOW,[],2);
                V_dw = V_dw_vec(:, end);
            end

        else
            V_s = V_s_a;
            V_h = V_h_a;
            V_w = V_w_a;
            V_dw = V_dw_a;
        end

    % *********************************************************************
    % Early landing check
    % *********************************************************************
        alt_above = abs(I_p_c(3) - x_t(3,idx));
        xy_err   = norm(I_p_c(1:2) - x_t(1:2,idx));
        rel_vel  = norm(I_v_c - dx_t(1:3,idx));
        if alt_above <= zf
            precise = xy_err <= 0.05;
            soft    = rel_vel <= 0.2;
            fprintf('Landed at t = %.2f s  (alt=%.3fm, xy=%.3fm, v_rel=%.3fm/s, precise=%d, soft=%d)\n', ...
                    tRange(idx), alt_above, xy_err, rel_vel, precise, soft);
            landed = true;
            break;
        end

    % *********************************************************************
    % Desired Optical Flow with Visibility Constraints
    % *********************************************************************
        V_s_e(:,idx) = V_s(1:2) - V_s_d(1:2);

        p_1(:,idx) = expm(-diag(K_ctrl.gamma_1)*tRange(idx)) * (K_ctrl.p_10 - K_ctrl.p_1inf) + K_ctrl.p_1inf;
        dp_1(:,idx) = -diag(K_ctrl.gamma_1) * expm(-diag(K_ctrl.gamma_1)*tRange(idx)) * (K_ctrl.p_10 - K_ctrl.p_1inf);

        for j=1:2
            S_1(j,j,idx) = V_s_e(j,idx)/p_1(j,idx);
            S_1(j,j,idx) = min(max(S_1(j,j,idx), -1+eps), 1-eps);
            zeta_1(j,idx) = log((1+S_1(j,j,idx))/(1-S_1(j,j,idx)));
            G_1(j, j,idx) = (exp(zeta_1(j,idx)) + 1)^2/(2*exp(zeta_1(j,idx))*p_1(j,idx));
        end

        if idx == 1
            izeta_1(:,idx) = dt*zeta_1(:,idx);
            raw_dzeta_1(:,idx+3) = zeros(2,1);
        else
            izeta_1(:,idx) = izeta_1(:,idx-1) + dt*(zeta_1(:,idx-1) + zeta_1(:,idx))/2;
            raw_dzeta_1(:,idx+3) = (zeta_1(:,idx) - zeta_1(:,idx-1))/dt;
        end
        dzeta_1 = smooth4(raw_dzeta_1(:,idx:idx+3));
        dzeta_1d = -K_ctrl.zp*zeta_1(:,idx) - K_ctrl.zi*izeta_1(:,idx) - K_ctrl.zd*dzeta_1;

        V_ds_d = [G_1(:, :, idx)\dzeta_1d + S_1(:,:,idx)*dp_1(:,idx);0.0];

        V_h_d(:,idx) = V_ds_d + cross(V_w, V_s(1:3)) + (h_rd ...
            - dot(cross(V_w, V_s(1:3)), e3))*V_s(1:3);

        V_h_e(:,idx) = V_h - V_h_d(:,idx);

    % *********************************************************************
    % Outer Loop Control Inputs
    % *********************************************************************
        p_2(:,idx) = expm(-diag(K_ctrl.gamma_2)*tRange(idx)) * (K_ctrl.p_20 - K_ctrl.p_2inf) + K_ctrl.p_2inf;
        dp_2(:,idx) = -diag(K_ctrl.gamma_2) * expm(-diag(K_ctrl.gamma_2)*tRange(idx)) * (K_ctrl.p_20 - K_ctrl.p_2inf);

        S_2_margin = 0.05;   % funnel saturation guard: keeps |zeta_2|<=3.66, G_2 finite
        for j=1:3
            S_2(j,j,idx) = V_h_e(j,idx) / p_2(j,idx);
            S_2(j,j,idx) = min(max(S_2(j,j,idx), -1+S_2_margin), 1-S_2_margin);
            zeta_2(j,idx) = log((1+S_2(j,j,idx))/(1-S_2(j,j,idx)));
            G_2(j, j,idx) = (exp(zeta_2(j,idx)) + 1)^2/(2*exp(zeta_2(j,idx))*p_2(j,idx));
        end

        if idx == 1
            izeta_2(:,idx) = dt*zeta_2(:,idx);
        else
            izeta_2(:,idx) = izeta_2(:,idx-1) + dt*(zeta_2(:,idx-1) + zeta_2(:,idx))/2;
        end
        % Anti-windup: clamp izeta_2
        izeta_2_max = 5.0;
        izeta_2(:,idx) = max(min(izeta_2(:,idx), izeta_2_max), -izeta_2_max);

        sigma(:,idx) = zeta_2(:,idx) + K_ctrl.Omega*izeta_2(:,idx);

    % Known System Dynamics (c)
        if idx == 1
            raw_dh_d(:,idx+3) = zeros(3,1);
        else
            raw_dh_d(:,idx+3) = (V_h_d(:,idx) - V_h_d(:,idx-1))/dt;
        end
        V_dh_d = smooth4(raw_dh_d(:,idx:idx+3));

        c = cross(V_dw, V_s(1:3)) + cross(V_w, cross(V_w, V_s(1:3))) ...
            + 2 * cross(V_w, V_h) - (dot(V_h + cross(V_w, V_s(1:3)), e3))*V_h - V_dh_d;

        Theta = [- c + S_2(:,:,idx)*dp_2(:,idx) ...
            - G_2(:,:,idx)\(K_ctrl.Omega*zeta_2(:,idx)), eye(3)];
        Theta_norm = norm(Theta,'fro');

    % Updating kappa
        const_kappa = [K_ctrl.N; K_ctrl.P];
        u_kappa = [sigma(:,idx); Theta_norm];
        kappa(:,idx+1) = RK5(@(t, X) kappa_Solver(t, X, u_kappa, const_kappa, G_2(:,:,idx)), t0, kappa(:,idx), dt);

        if any(isnan(kappa(:,idx+1)))
            fprintf('  BREAK: kappa NaN at idx=%d (t=%.2f)\n', idx, tRange(idx));
            break
        end

    % Outer Loop Control Output
        u_sw = -K_ctrl.Gamma*sigma(:,idx) - Theta_norm*diag(sat(K_ctrl.E\sigma(:,idx)))*G_2(:,:,idx)*kappa(:,idx+1);
        u_eq = G_2(:,:,idx)*(-c + S_2(:,:,idx)*dp_2(:,idx) ...
            - G_2(:,:,idx)\(K_ctrl.Omega*zeta_2(:,idx)));

        V_a_cd = - G_2(:,:,idx)\(u_sw + u_eq);

        I_a_cd(:,idx) = I_R_V * V_a_cd - g;

        if norm(I_a_cd(:,idx)) > 1e02 || any(isnan(I_a_cd(:,idx)))
           fprintf('  BREAK: I_a_cd norm=%.2f or NaN at idx=%d (t=%.2f)\n', norm(I_a_cd(:,idx)), idx, tRange(idx));
           break;
        end

        % Cone clamp
        att_cone = deg2rad(35);
        if I_a_cd(3,idx) >= 0
            I_a_cd(3,idx) = -3.0;
        end
        a_xy_limit = abs(I_a_cd(3,idx)) * tan(att_cone);
        a_xy_norm  = norm(I_a_cd(1:2,idx));
        if a_xy_norm > a_xy_limit
            I_a_cd(1:2,idx) = a_xy_limit * I_a_cd(1:2,idx) / a_xy_norm;
        end
        I_a_cd(3,idx) = max(I_a_cd(3,idx), -50);

        % Low-pass filter I_a_cd before feeding R_d construction.
        % Matches tau_w=0.08s: absorbs pixel-noise spikes propagated
        % through L_s^-1 at low altitude; raw I_a_cd still logged.
        I_a_cd_filt = alpha_ia * I_a_cd_filt + (1 - alpha_ia) * I_a_cd(:,idx);

    % *********************************************************************
    % Yaw adaptive SMC — drives alpha -> alpha_d, outputs rate u_a
    % *********************************************************************
        % alpha has period pi -> wrap e_a to [-pi/2, pi/2]
        e_raw    = V_s(4) - V_s_d(4);
        e_a(idx) = atan2(sin(2*e_raw), cos(2*e_raw)) / 2;
        if idx == 1
            ie_a(idx) = dt * e_a(idx);
        else
            ie_a(idx) = ie_a(idx-1) + dt*(e_a(idx-1) + e_a(idx))/2;
        end
        sigma_a = e_a(idx) + K_ctrl.Omega_a * ie_a(idx);

        const_kappa_a = [K_ctrl.n_a; K_ctrl.p_a];
        kappa_a(idx+1) = RK5(@(t,X) kappa_a_Solver(t, X, sigma_a, const_kappa_a), ...
                             t0, kappa_a(idx), dt);
        u_a = K_ctrl.Gamma_a*sigma_a + sat(sigma_a/K_ctrl.E_a)*kappa_a(idx+1) ...
              + K_ctrl.Omega_a*e_a(idx);

        if ~isfinite(u_a)
            fprintf('  BREAK: u_a non-finite at idx=%d (t=%.2f)\n', idx, tRange(idx));
            break;
        end

        % Integrate ASMC rate into desired heading (replaces compass)
        psi_d = psi_d + u_a * dt;
        psi_d = atan2(sin(psi_d), cos(psi_d));

    % *********************************************************************
    % Construct R_d from filtered I_a_cd (roll/pitch) + psi_d (heading)
    % *********************************************************************
        I_F   = m * I_a_cd_filt;
        f_mag = norm(I_F);
        T_cd  = f_mag;

        if f_mag < 1e-6
            R_d = eye(3);
        else
            rd3 = -I_F / f_mag;
            a_h = [cos(psi_d); sin(psi_d); 0];
            rd2_raw = cross(rd3, a_h);
            n2 = norm(rd2_raw);
            if n2 < 1e-6, rd2_raw = [0;1;0]; n2 = 1; end
            rd2 = rd2_raw / n2;
            rd1 = cross(rd2, rd3);
            R_d = [rd1, rd2, rd3];
        end

    % *********************************************************************
    % Geometric SO(3) attitude controller
    % *********************************************************************
        eR_mat = 0.5 * (R_d' * I_R_C - I_R_C' * R_d);
        e_R    = [eR_mat(3,2); eR_mat(1,3); eR_mat(2,1)];   % vee map
        e_Omega = B_w_c;                                    % Omega_d = 0

        B_tau_cd = -K_ctrl.kR*e_R - K_ctrl.kOmega*e_Omega + cross(B_w_c, J*B_w_c);

    % Ground effect on thrust
        if GE
            z_ge = -max(abs(x_c(3)), r);
            T_cd = 1/(1-(r/(4*z_ge))^2) * T_cd;
        end

    % Saturate torques and thrust
        B_tau_cd(1:2) = min(max(B_tau_cd(1:2), -tau_xy_max), tau_xy_max);
        B_tau_cd(3)   = min(max(B_tau_cd(3),   -tau_z_max),  tau_z_max);
        T_cd          = max(min(T_cd, T_max), T_min);

        B_T_cd(idx) = T_cd;

    % *********************************************************************
    % Computational Delay
    % *********************************************************************
        u_2_buf(:,idx) = [B_tau_cd; T_cd];
        if idx > delay
            u_2 = u_2_buf(:, idx - delay);
        else
            u_2 = [zeros(3,1); m*norm(g)];
        end

        if any(isnan(u_2)) || norm(u_2) > 1e4
            fprintf('  BREAK: u_2 invalid at idx=%d (t=%.2f)\n', idx, tRange(idx));
            break;
        end

        x_c = RK5(@(t, x) UAVDyn(t, x, u_2), t0, x_c, dt);

        if any(isnan(x_c))
            fprintf('  BREAK: x_c NaN at idx=%d (t=%.2f)\n', idx, tRange(idx));
            break;
        end

    % *********************************************************************
    % Update States
    % *********************************************************************
        I_p_c = x_c(1:3);
        q_c = x_c(4:7);
        I_v_c = x_c(8:10);
        B_w_c = x_c(11:13);

        q_c = q_c / norm(q_c);

    % *********************************************************************
    % Logging
    % *********************************************************************
        U_DS(:,idx) = u_2;
        X_DS(:,idx+1) = x_c;
        V_X_DS(:,idx) = [V_s_i(1:2); V_s_i(4); V_h_i; V_w_i; V_dw_i; V_s_a(1:2); V_s_a(4); V_h_a; V_w_a; V_dw_a];
        eR_log(:,idx) = e_R;
        B_tau_cd_log(:,idx) = B_tau_cd;
        psi_d_log(idx) = psi_d;
        u_a_log(idx)   = u_a;
        D_DS(:,idx) = [V_h_d(:,idx); I_a_cd(:,idx); e_R; B_tau_cd; T_cd; psi_d; u_a];
        P_DS(:,:,idx) = [V_nP_i, V_nP_a, C_nP];

    % *********************************************************************
    % Termination
    % *********************************************************************
        alt_above = abs(I_p_c(3) - x_t(3,idx));
        xy_err   = norm(I_p_c(1:2) - x_t(1:2,idx));
        rel_vel  = norm(I_v_c - dx_t(1:3,idx));
        if alt_above <= zf
            precise = xy_err <= 0.05;
            soft    = rel_vel <= 0.2;
            fprintf('Landed at t = %.2f s  (alt=%.3fm, xy=%.3fm, v_rel=%.3fm/s, precise=%d, soft=%d)\n', ...
                    tRange(idx), alt_above, xy_err, rel_vel, precise, soft);
            landed = true;
            break;
        end

         t0 = t0+dt;
    end

    result.success     = landed;
    result.final_error = norm(I_p_c - x_t(1:3,idx));
    result.final_t     = tRange(idx);
    result.final_xy    = norm(I_p_c(1:2) - x_t(1:2,idx));
    result.final_alt   = abs(I_p_c(3) - x_t(3,idx));
    result.final_rel_vel = norm(I_v_c - dx_t(1:3,idx));
    result.precise     = landed && (result.final_xy <= 0.05);
    result.soft        = landed && (result.final_rel_vel <= 0.2);

    if ~landed
        idx = idx - 1;
    end

    save("temp.mat");
    result.data = load("temp.mat");
    delete("temp.mat")
end
