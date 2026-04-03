%% Started working on 14/01/2026
% Last updated on 14/01/2026
%% Refer to "Adaptive Visual Control Strategies for Autonomous Landing of Quadrotor Platforms with visibility constraaints"
% Adaptive Gain Controller and IBVS
% Inertial Frame --> NED
% Camera/Visual Frame ---> NED
% Outer Loop --> s --> h --> a --> E
% Inner Loop --> dE --> w --> dw --> T
clc;
% close all;
if exist('K', 'var') == 1
    clearvars -except K;
elseif isfile("bestParam.mat") == 1
    clear;
    load("bestParam.mat");
end
rng('shuffle');

% header
fprintf('Adaptive - PID Flight Controller\n\n' );

Constants;

InitVar;
flag = false;
    
%% Solving for Control Law and then the System Dynamics using this Control Law
% Finding System State Evolution from t=t_0 to t=t_end
for idx=1:numel(tRange)
% *************************************************************************
% Compute Initial Linear and Angular Velocities of Camera in Camera 
% Reference Frame
% *************************************************************************
    I_R_C = quat2rotm(q_c');
    E_cr = quat2eul(q_c', 'XYZ');

% *************************************************************************
% Simulating moving target
% *************************************************************************
% Generating pose of the Target wrt Global Origin in Inertial Reference 
% Frame for current iteration
% Implement zero hold for trajectory updation of the target
    % traj_t = traj_Gen((idx-1)*dt, "Static");
    % traj_t = traj_Gen((idx-1)*dt, "Linear");
    traj_t = traj_Gen((idx-1)*dt, "Sinusoidal");
    % traj_t = traj_Gen((idx-1)*dt, "Lissajous");
    % traj_t = traj_Gen((idx-1)*dt, "Circular");

    x_t(:,idx) = traj_t(:,1);
    dx_t(:,idx) = traj_t(1:end-1,2);
    
    I_R_T = quat2rotm(x_t(4:7,idx)');

% *************************************************************************
% Simulating Image Feature Points in Image Plane
% *************************************************************************
% Computing Feature Points wrt Global Origin in Inertial Reference Frame
    I_nP3(:,:,idx) = I_R_T * T_nP3 + x_t(1:3,idx);

% Computing Feature Points wrt Camera Origin in Camera Reference Frame
    C_nP3 = transpose(I_R_C)*(I_nP3(:,:,idx) - I_p_c);

% Computing position of Target wrt Camera Origin in Camera Reference Frame
    C_s_tc(:,idx) = transpose(I_R_C)*(x_t(1:3,idx) - I_p_c);

% Computing Image Feature Points in Image Plane
    if mod(idx-1,ZOH) == 0
        C_nP = (f/C_s_tc(3,idx))*C_nP3(1:2,:);

        % Adding noise to estimated radial optical flow
        if NOISE
            C_nP = C_nP + randi([-1, 1], size(C_nP));
        end
        C_nP = round(C_nP);
    end

% *************************************************************************
% Computing Scale-Independent Target Image Parameters using Image Feature Points
% *************************************************************************
% Extract yaw from current rotation
    yaw = atan2(2*(q_c(1)*q_c(4) + q_c(2)*q_c(3)), ...
        1 - 2*(q_c(3)^2 + q_c(4)^2));
    
% Virtual camera rotation
    I_R_V = rotz(rad2deg(yaw));

% Computing Target Image Parameters in Virtual Image Plane (Output)
    V_R_C = I_R_V' * I_R_C;
    rays = [C_nP; f*ones(1,size(C_nP, 2))];
    vr = V_R_C * rays;
    V_nP_i = f*vr(1:2,:)./vr(3,:);
    for j=1:size(C_nP,2)
        L_s(2*j-1:2*j,:) = [f, 0, -V_nP_i(1,j), -V_nP_i(1,j)*V_nP_i(2,j)/f, (f^2+V_nP_i(1,j)^2)/f, -V_nP_i(2,j);
            0, f, -V_nP_i(2,j), -(f^2+V_nP_i(2,j)^2)/f, V_nP_i(1,j)*V_nP_i(2,j)/f, V_nP_i(1,j)];
    end
    V_s_i = image_feature(V_nP_i/f);  % s = [xhat, yhat, 1, s_phi]

    V_2nP_i(:,idx) = reshape(V_nP_i,[],1);

% Computing smooth difference of 'V_2nP_i'.
    if idx == 1
        raw_dPdt(:,:,idx+3) = zeros(size(V_2nP_i));
    else
        raw_dPdt(:,:,idx+3) = (V_2nP_i(:,idx) - V_2nP_i(:,idx-1))/dt;
    end
    dPdt = smooth4(raw_dPdt(:,end-3:end));
    
    V_v_i = pinv(L_s,4)*dPdt;
    % V_v_i = ridge(dPdt, L_s, 10, 0);

    V_h_i = V_v_i(1:3);  % h = [h_x, h_y, h_r]
    V_w_i(:,idx) = V_v_i(4:6);  % w = [w_x, w_y, w_z] 

% Computing smooth difference of 'V_w_i'.
    if idx == 1
        raw_dw_i(:,idx+3) = zeros(size(V_w_i));
    else
        raw_dw_i(:,idx+3) = (V_w_i(:,idx) - V_w_i(:,idx-1))/dt;
    end
    V_dw_i = smooth4(raw_dw_i(:,end-3:end));

% *************************************************************************
% Calculating Scale-Independent Target Image Parameters Analytically
% *************************************************************************
% Computing Feature Points wrt Camera Origin in Virtual Camera Reference Frame
    V_nP3 = transpose(I_R_V)*(I_nP3(:,:,idx) - I_p_c);

% Computing position of Target wrt Camera Origin in Virtual Camera Reference Frame
    V_s_tc(:,idx) = transpose(I_R_V)*(x_t(1:3,idx) - I_p_c);

% Computing Feature Points in Image Plane    
    V_nP_a = (f/V_s_tc(3,idx))*V_nP3(1:2,:);

% Computing Target Image Parameters
    V_s_a = image_feature(V_nP_a/f);  % s = [xhat, yhat, s_phi]
    V_h_a = transpose(I_R_V) * (dx_t(1:3,idx) - I_v_c) / V_s_tc(3,idx);
    I_w_c = I_R_C * B_w_c;
    V_w_a(:,idx) = transpose(I_R_V) * (dx_t(4:6,idx) - [0;0;I_w_c(3)]); 

% Computing smooth difference of 'V_w_a'.
    if idx == 1
        raw_dw_a(:,idx+3) = zeros(size(V_w_a));
    else
        raw_dw_a(:,idx+3) = (V_w_a(:,idx) - V_w_a(:,idx-1))/dt;
    end
    V_dw_a = smooth4(raw_dw_a(:,end-3:end));

% *************************************************************************
% Scale-Independent Target Image Parameters: Actual vs Analytical
% *************************************************************************
if ACTUAL
    V_s(1:3,idx) = V_s_a(1:3);
    V_s(4,idx) = V_s_a(4);
    V_h(:,idx) = V_h_a;
    V_w = V_w_a(:,idx);
    V_dw = V_dw_a;
else
    V_s(:,idx) = V_s_a;
    V_h(:,idx) = V_h_a;
    V_w = V_w_a(:,idx);
    V_dw = V_dw_a;
end

% *************************************************************************
% Computing Desired Optical Flow with Visibility Constraints
% *************************************************************************
% Calculating Deviation of Features Parameters from desired 's_d'
    V_s_e(:,idx) = V_s(1:3,idx) - V_s_d(1:3);

% Computing Performance Function for Visibility Constraints
    p_1(:,idx) = expm(-diag(K.gamma_1)*tRange(idx)) * (K.p_10 - K.p_1inf) + K.p_1inf;
    dp_1(:,idx) = -diag(K.gamma_1) * expm(-diag(K.gamma_1)*tRange(idx)) * (K.p_10 - K.p_1inf);

    for j=1:2     
% Computing Transformation matrix 'S_1'
        S_1(j,j,idx) = V_s_e(j,idx)/p_1(j,idx);
        if (abs(S_1(j,j,idx))>=1)
            S_1(j,j,idx) = abs(S_1(j,j,idx-1))*sign(V_s_e(j,idx));
            V_s(j,idx) = S_1(j,j,idx) * p_1(j,idx) + V_s_d(j);
            % flag = true; % Set the flag to true
            % break; % Break out of the inner loop
        end

% Computing Unconstrained Error 'zeta_1'
        zeta_1(j,idx) = log((1+S_1(j,j,idx))/(1-S_1(j,j,idx)));
    
% Computing Pseudo Performance Function 'g(t)'
        G_1(j, j,idx) = (exp(zeta_1(j,idx)) + 1)^2/(2*exp(zeta_1(j,idx))*p_1(j,idx));
    end

    if flag
        break; % Break out of the outer loop
    end

% Integrating 'zeta_1' wrt time t using Trapezoidal Method
% Computing smooth difference of 'zeta_1'.
    if idx == 1
        izeta_1(:,idx) = dt*zeta_1(:,idx)/2;
        raw_dzeta_1(:,idx+3) = zeros(size(zeta_1));
    else
        izeta_1(:,idx) = izeta_1(:,idx-1) + dt*(zeta_1(:,idx-1) + zeta_1(:,idx))/2;
        raw_dzeta_1(:,idx+3) = (zeta_1(:,idx) - zeta_1(:,idx-1))/dt;
    end
    dzeta_1 = smooth4(raw_dzeta_1(:,end-3:end));
    dzeta_1d = - K.zp*zeta_1(:,idx) - K.zi*izeta_1(:,idx) - K.zd*raw_dzeta_1(:,idx+3);
    V_ds_d = [G_1(:, :, idx)\dzeta_1d + S_1(:,:,idx)*dp_1(:,idx);0.0];

% Computing Desired System Output (h_d)
    V_h_d(:,idx) = V_ds_d + cross(V_w, V_s(1:3,idx)) + (h_rd ...
        - dot(cross(V_w, V_s(1:3,idx)), e3))*V_s(1:3,idx);
    
 % Calculating Deviation of Optical Flow from desired 'h_d'
    V_h_e(:,idx) = V_h(:,idx) - V_h_d(:,idx);

% *************************************************************************
% Computing Outer Loop Control Inputs
% *************************************************************************
% Computing Performance Function for Outlier Elimination
    p_2(:,idx) = expm(-diag(K.gamma_2)*tRange(idx)) * (K.p_20 - K.p_2inf) + K.p_2inf;
    dp_2(:,idx) = -diag(K.gamma_2) * expm(-diag(K.gamma_2)*tRange(idx)) * (K.p_20 - K.p_2inf);

    for j=1:3     
% Computing Transformation matrix 'S_2'
        S_2(j,j,idx) = V_h_e(j,idx)/p_2(j,idx);
        if (abs(S_2(j,j,idx))>=1)
            S_2(j,j,idx) = abs(S_2(j,j,idx-1))*sign(V_h_e(j,idx));
            V_h(j,idx) = S_2(j,j,idx) * p_2(j,idx) + V_h_d(j);
            flag = true; % Set the flag to true
            break; % Break out of the inner loop
        end

% Computing Unconstrained Error 'zeta'
        zeta_2(j,idx) = log((1+S_2(j,j,idx))/(1-S_2(j,j,idx)));
    
% Computing Pseudo Performance Function 'g(t)'
        G_2(j, j,idx) = (exp(zeta_2(j,idx)) + 1)^2/(2*exp(zeta_2(j,idx))*p_2(j,idx));
    end
    
    if flag
        break; % Break out of the outer loop
    end

% Integrating 'zeta' wrt time t using Trapezoidal Method
    if idx == 1
        izeta_2(:,idx) = dt*zeta_2(:,idx)/2;
    else
        izeta_2(:,idx) = izeta_2(:,idx-1) + dt*(zeta_2(:,idx-1) + zeta_2(:,idx))/2;
    end

    sigma(:,idx) = zeta_2(:,idx) + K.Omega*izeta_2(:,idx);

% Computing Known System Dynamics (c)    
% Computing smooth difference of 'h_d'.
    if idx == 1
        raw_dh_d(:,idx+3) = zeros(size(V_h_d));
    else
        raw_dh_d(:,idx+3) = (V_h_d(:,idx) - V_h_d(:,idx-1))/dt;
    end
    V_dh_d = smooth4(raw_dh_d(:,end-3:end));

    c = cross(V_dw, V_s(1:3,idx)) + cross(V_w, cross(V_w, V_s(1:3,idx))) ...
        + 2 * cross(V_w, V_h(:,idx)) - (dot(V_h(:,idx) + cross(V_w, V_s(1:3,idx)), e3))*V_h(:,idx) - V_dh_d;

% Computing Uncertainty Coefficient Matrix 'Theta(t)'
    Theta(:,:,idx) = [- c + S_2(:,:,idx)*dp_2(:,idx) ...
    - G_2(:,:,idx)\K.Omega*zeta_2(:,idx), eye(3)];
    Theta_norm = norm(Theta(:,:,idx),'fro');

% Updating Control Parameter 'kappa' using RK-5 Method
    const_kappa = [K.N; K.P];
    u_kappa = [sigma(:,idx); Theta_norm];
    kappa(:,idx+1) = 1.0*RK5(@(t, X) kappa_Solver(t, X, u_kappa, const_kappa, G_2(:,:,idx)), t0, kappa(:,idx), dt);

    if any(isnan(kappa(:,idx+1)))
        break
    end

% Computing Outer Loop Control Output
    u_sw = - K.Gamma*sigma(:,idx) - Theta_norm*diag(sat(K.E\sigma(:,idx)))*G_2(:,:,idx)*kappa(:,idx+1);
    u_eq = G_2(:,:,idx)*(- c + S_2(:,:,idx)*dp_2(:,idx) ...
        - G_2(:,:,idx)\K.Omega*zeta_2(:,idx));

    V_a_cd = - G_2(:,:,idx)\(u_sw + u_eq);
    % 
    % if idx == 128
    %     fprintf("Break")
    % end

% % Limiting the Acceptible Acceleration Input
%     for j=1:3
%         if abs(V_a_cd(j)) > a_max(j)
%             fprintf("Acceleration has breached the bounds.\n")
%             V_a_cd(j) = a_max(j)*sign(V_a_cd(j));
%         end
%     end

    I_a_cd(:,idx) = I_R_V * V_a_cd - g;
    
    if norm(I_a_cd(:,idx)) > 1e02 || any(isnan(I_a_cd(:,idx))) 
        break; % Break out of the outer loop
    end

    if abs(cos(yaw)*I_a_cd(1,idx) + sin(yaw)*I_a_cd(2,idx)) < 1e-04
        theta_cd = 0;
    else
        theta_cd = atan2(- cos(yaw)*I_a_cd(1,idx) - sin(yaw)*I_a_cd(2,idx), ...
            - I_a_cd(3,idx));
    end

    if abs(sin(yaw)*I_a_cd(1,idx) - cos(yaw)*I_a_cd(2,idx)) < 1e-04
        phi_cd = 0;
    else
        phi_cd = atan2(- sin(yaw)*I_a_cd(1,idx) + cos(yaw)*I_a_cd(2,idx), ...
            - I_a_cd(3,idx)/cos(E_cr(2)));
    end
    
    E_crd = [phi_cd; theta_cd];

    % tol = 1e-12;
    % if any(abs(abs(E_crd) - pi) < tol)
    %     break;
    %     % true if any element is within tol of pi
    % end

% *************************************************************************
% Attitude and Thrust Control
% *************************************************************************
% Computing position of Camera wrt Target Origin
% Note that d(s(4)) = d(alpha) = -w_z = d(psi_tc)
    E_e(:,idx) = [E_cr(1:2)'; -V_s(4,idx)] - [E_crd; -V_s_d(4)];
    % E_e(:,idx) = E_cr' - [E_crd; 0.0];

% Computing the smooth first difference and integral
    if idx == 1
        iE_e(:,idx) = dt*E_e(:,idx)/2;
        raw_dE_e(:,idx+3) = zeros(size(E_e));
    else
        iE_e(:,idx) = iE_e(:,idx-1) + dt*(E_e(:,idx-1) + E_e(:,idx))/2;
        raw_dE_e(:,idx+3) = (E_e(:,idx) - E_e(:,idx-1))/dt;
    end
    dE_e(:,idx) = smooth4(raw_dE_e(:,end-3:end));    

    dE_cd = - K.ep * E_e(:,idx) - K.ei * iE_e(:,idx) - K.ed * raw_dE_e(:,idx+3);

    if norm(dE_cd) > 1e02
        break; % Break out of the outer loop
    end   

    % W_d = [1 , 0, -sin(E_crd(2)); ...
    %     0, cos(E_crd(1)), sin(E_crd(1))*cos(E_crd(2)); ...
    %     0, -sin(E_crd(1)), cos(E_crd(1))*cos(E_crd(2))];
    W = [1 , 0, -sin(E_cr(2)); ...
        0, cos(E_cr(1)), sin(E_cr(1))*cos(E_cr(2)); ...
        0, -sin(E_cr(1)), cos(E_cr(1))*cos(E_cr(2))];

    B_w_cd(:,idx) = W*dE_cd;
   
% Computing desired Thrust force of Camera
    B_T_cd(idx) = - m*(I_a_cd(3,idx))/(cos(E_cr(1))*cos(E_cr(2)));

% Adding ground effect to Vertical Control input
    if GE
        B_T_cd(idx) = 1/(1-(r/(4*x_c(3)))^2)*B_T_cd(idx);
    end

% *************************************************************************
% Simulating Computational Delay and Input Saturation
% ************************************************************************* 
    % Incorporating Computational Delay
    if idx > delay         
        u_1 = [B_w_cd(:,idx - delay); B_T_cd(idx - delay)];
    else
        u_1 = [zeros(3,1);m*dot(g, I_R_C(:,3));];
    end

    % Incorporating Input Saturation
    % 1. Thrust constraint (highest priority)
    u_1(4) = max(min(u_1(4), T_max), T_min);
    
    % 2. Angular-rate constraints
    u_1(1:3) = max(min(u_1(1:3), w_max), -w_max);

% *************************************************************************
% Attitude Rate Control with Input Saturation
% ************************************************************************* 
    w_e(:,idx) = B_w_c - u_1(1:3); 

    % Computing the smooth first difference of desired Inner Loop Output
    if idx == 1
        iw_e(:,idx) = dt*w_e(:,idx)/2;
        B_w_cf(:,idx) = B_w_c;        % initialize filtered rate
        B_dw_cf = zeros(size(B_w_c));   % filtered derivative
    else
        iw_e(:,idx) = iw_e(:,idx-1) + (w_e(:,idx-1) + w_e(:,idx))*dt/2;

        % LPF on body rate
        B_w_cf(:,idx) = alpha_w * B_w_cf(:,idx-1) + (1 - alpha_w) * B_w_c;

        % LPF on derivative
        B_dw_cf = alpha_dw * B_dw_cf + (1 - alpha_dw) * (B_w_cf(:,idx) - B_w_cf(:,idx-1))/dt;
    end

    B_dw_cd = - K.wp * w_e(:,idx) - K.wi * iw_e(:,idx) - K.wd * B_dw_cf + K.ff * u_1(1:3);

    % Control torque  
    B_tau_cd = J * B_dw_cd + cross(B_w_c, J * B_w_c);

    % 3. Rate-controller saturation
    B_tau_cd(1:2) = min(max(B_tau_cd(1:2), -tau_xy_max), tau_xy_max);
    B_tau_cd(3)   = min(max(B_tau_cd(3),   -tau_z_max), tau_z_max);

    % anti-windup
    anti_windup_factor = abs(B_tau_cd) >= [tau_xy_max; tau_xy_max; tau_z_max];
    iw_e(:,idx) = iw_e(:,idx) .* (~anti_windup_factor);

% *************************************************************************
% Simulating UAV Flight Dynamics with System Delay
% ************************************************************************* 
    u_2 = [B_tau_cd; u_1(4)];

% Solving UAV Fight Dynamics using Control Inputs Using RK-5 Method
    x_c = RK5(@(t, x) UAVDyn(t, x, u_2), t0, x_c, dt);

    if any(isnan(x_c))
        break; % Break out of the outer loop
    end

% *************************************************************************
% Updating System States in Inertial frame
% *************************************************************************
    I_p_c = x_c(1:3);
    q_c = x_c(4:7);
    I_v_c = x_c(8:10);
    B_w_c = x_c(11:13);

    q_c = q_c / norm(q_c);

    % if idx == 1200
    %     fprintf("Reached");
    % end
% *************************************************************************
% Logging states, control inputs and output (Datasets)
% *************************************************************************
    U_DS(:,idx) = u_2;
    X_DS(:,idx+1) = x_c;
    V_X_DS(:,idx) = [V_s_i(1:2); V_s_i(4); V_h_i; V_w_i(:,idx); V_dw_i; V_s_a(1:2); V_s_a(4); V_h_a; V_w_a(:,idx); V_dw_a];
    D_DS(:,idx) = [V_h_d(:,idx); I_a_cd(:,idx); E_crd; dE_cd; B_w_cd(:,idx); B_dw_cd];
    P_DS(:,:,idx) = [V_nP_i, V_nP_a]; 

% *************************************************************************
% Termination Condition
% *************************************************************************
    % if norm(I_p_c(3) - x_t(3,idx)) <= 0.2
    if norm(I_p_c - x_t(1:3,idx)) <= 0.18
        fprintf("Landed\n");
        break;
    end

% *************************************************************************
% Updating to next iteration
% *************************************************************************    
     t0 = t0+dt;
     % if idx == 502
     %     pause
     % end
end

idx=idx-1;
save("temp.mat");
data = load("temp.mat");
% Call your plotting function
plotter_adaptive(data); % Assuming plotter_PID takes loaded data as input
delete("temp.mat")