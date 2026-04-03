function plotter_adaptive(data)
%% Extracting Solution
    I_p_c = data.X_DS(1:3,:);
    
    q_c = data.X_DS(4:7,:);
    
    I_v_c = data.X_DS(8:10,:);
    
    C_w_c = data.X_DS(11:13,:);

    tau = data.U_DS(1:3,:);
    
    T = data.U_DS(4,:);

    s_i = data.V_X_DS(1:3,:);
    h_i = data.V_X_DS(4:6,:);
    w_i = data.V_X_DS(7:9,:);
    dw_i = data.V_X_DS(10:12,:);

    s_a = data.V_X_DS(13:15,:);
    h_a = data.V_X_DS(16:18,:);
    w_a = data.V_X_DS(19:21,:);
    dw_a = data.V_X_DS(22:24,:);
    
    h_d = data.D_DS(1:3,:);
    a_d = data.D_DS(4:6,:);
    E_d = data.D_DS(7:8,:);  
    dE_cd = data.D_DS(9:11,:);  
    C_w_cd = data.D_DS(12:14,:); 
    C_dw_cd = data.D_DS(15:17,:); 

    V_nP_a = data.P_DS(:,5:8,:);
    V_nP_d = data.V_nP_d;

    zeta_1 = data.zeta_1;

    I_p_t = data.x_t(1:3,:);
    
    I_dp_t = data.dx_t(1:3,:);
    
    q_t = data.x_t(4:7,:);
    
    w_t = data.dx_t(4:6,:);

    K = data.K;
    p_1 = data.p_1;
    p_2 = data.p_2;
    s_d = data.V_s_d;

    S_1 = data.S_1;

    idx = data.idx - 1;
    s_e = zeros(2, idx);
    for k = 1:idx
        s_e(:,k) = S_1(:,:,k) * p_1(:,k);
    end

    tRange = data.tRange;

    %% Calculating Euler Angles from Quaternions
    E_c = quat2eul(q_c','XYZ');
    E_c = E_c';
    E_t = quat2eul(q_t','XYZ'); 
    E_t = E_t';

    %% Plotting the results    
    
    
    figure(4);
    clf;
    plot3(I_p_c(1,1:idx), I_p_c(2,1:idx), -I_p_c(3,1:idx));
    hold on;
    plot3(I_p_t(1,1:idx), I_p_t(2,1:idx), I_p_t(3,1:idx), ".r");
    % axis([0 tRange(i) -(1.5*Pnf) (1.5*Pnf)]);
    % axis([0 tend -(P_0 +1) (P_0 +1)]);
    xlabel("$^Ix$ (m)",'Interpreter','latex');
    ylabel("$^Iy$ (m)",'Interpreter','latex');
    zlabel("$^Iz$ (m)",'Interpreter','latex');
    legend("Camera", "Target")
    title("3D Plot in Inertial Frame");
    % subtitle(parameters,'Interpreter','latex');
    grid on;

    % figure(5);
    % clf;
    % 
    % subplot(2, 3, 1);
    % plot(tRange(1:idx), s_a(1, 1:idx));
    % hold on;
    % plot(tRange(1:idx), s_i(1, 1:idx));
    % yline(s_d(1), '--c');
    % yline(0.0);
    % xlabel("$t$ (s)",'Interpreter','latex');
    % ylabel("$\hat{x}(t)$ (rad)",'Interpreter','latex');
    % legend("Analytical", "Actual", "Desired", 'Location', 'best');
    % title("$\hat{x}$ vs $t$ Plot",'Interpreter','latex');
    % % hold off;
    % grid on;
    % 
    % subplot(2, 3, 2);
    % plot(tRange(1:idx), s_a(2, 1:idx));
    % hold on;
    % plot(tRange(1:idx), s_i(2, 1:idx));
    % yline(s_d(2), '--c');
    % yline(0.0);
    % xlabel("$t$ (s)",'Interpreter','latex');
    % ylabel("$\hat{y}(t)$ (rad)",'Interpreter','latex');
    % legend("Analytical", "Actual", "Desired", 'Location', 'best');
    % title("$\hat{y}$ vs $t$ Plot",'Interpreter','latex');
    % % hold off;
    % grid on;
    % 
    % subplot(2, 3, 3);
    % plot(tRange(1:idx), s_a(3, 1:idx));
    % hold on;
    % plot(tRange(1:idx), s_i(3, 1:idx));
    % yline(s_d(4), '--c');
    % % yline(0.0);
    % xlabel("$t$ (s)",'Interpreter','latex');
    % ylabel("$\alpha(t)$ (rad)",'Interpreter','latex');
    % legend("Analytical", "Actual", "Desired", 'Location', 'best');
    % title("$\alpha$ vs $t$ Plot",'Interpreter','latex');
    % % hold off;
    % grid on;
    % 
    % subplot(2, 3, 4);
    % plot(tRange(1:idx), h_a(1, 1:idx));
    % hold on;
    % plot(tRange(1:idx), h_i(1, 1:idx));
    % plot(tRange(1:idx), h_d(1,1:idx), '--c');
    % yline(0.0);
    % xlabel("$t$ (s)",'Interpreter','latex');
    % ylabel("$h_{x}(t)$ (rad/s)",'Interpreter','latex');
    % legend("Analytical", "Actual", "Desired", 'Location', 'best');
    % title("$h_{x}$ vs $t$ Plot",'Interpreter','latex');
    % grid on;
    % 
    % subplot(2, 3, 5);
    % plot(tRange(1:idx), h_a(2, 1:idx));
    % hold on;
    % plot(tRange(1:idx), h_i(2, 1:idx));
    % plot(tRange(1:idx), h_d(2,1:idx), '--c');
    % yline(0.0);
    % xlabel("$t$ (s)",'Interpreter','latex');
    % ylabel("$h_{y}(t)$ (rad/s)",'Interpreter','latex');
    % legend("Analytical", "Actual", "Desired", 'Location', 'best');
    % title("$h_{y}$ vs $t$ Plot",'Interpreter','latex');
    % grid on;
    % 
    % subplot(2, 3, 6);
    % plot(tRange(1:idx), h_a(3, 1:idx));
    % hold on;
    % plot(tRange(1:idx), h_i(3, 1:idx));
    % plot(tRange(1:idx), h_d(3,1:idx), '--c');
    % % yline(0.0);
    % xlabel("$t$ (s)",'Interpreter','latex');
    % ylabel("$h_{r}(t)$ (rad/s)",'Interpreter','latex');
    % legend("Analytical", "Actual", "Desired", 'Location', 'best');
    % title("$h_{r}$ vs $t$ Plot",'Interpreter','latex');
    % % hold off;
    % grid on;
    % 
    % figure(6);
    % clf;
    % 
    % subplot(2, 3, 1);
    % plot(tRange(1:idx), w_a(1, 1:idx));
    % hold on;
    % plot(tRange(1:idx), w_i(1, 1:idx));
    % yline(0.0);
    % xlabel("$t$ (s)",'Interpreter','latex');
    % ylabel("$\omega_{x}(t)$ (rad/s)",'Interpreter','latex');
    % legend("Analytical", "Actual", 'Location', 'best');
    % title("$\omega_{x}$ vs $t$ Plot",'Interpreter','latex');
    % % hold off;
    % grid on;
    % 
    % subplot(2, 3, 2);
    % plot(tRange(1:idx), w_a(2, 1:idx));
    % hold on;
    % plot(tRange(1:idx), w_i(2, 1:idx));
    % yline(0.0);
    % xlabel("$t$ (s)",'Interpreter','latex');
    % ylabel("$\omega_{y}(t)$ (rad/s)",'Interpreter','latex');
    % legend("Analytical", "Actual", 'Location', 'best');
    % title("$\omega_{y}$ vs $t$ Plot",'Interpreter','latex');
    % % hold off;
    % grid on;
    % 
    % subplot(2, 3, 3);
    % plot(tRange(1:idx), w_a(3, 1:idx));
    % hold on;
    % plot(tRange(1:idx), w_i(3, 1:idx));
    % % yline(0.0);
    % xlabel("$t$ (s)",'Interpreter','latex');
    % ylabel("$\omega_{z}(t)$ (rad/s)",'Interpreter','latex');
    % legend("Analytical", "Actual", 'Location', 'best');
    % title("$\omega_{z}$ vs $t$ Plot",'Interpreter','latex');
    % % hold off;
    % grid on;
    % 
    % subplot(2, 3, 4);
    % plot(tRange(1:idx), dw_a(1, 1:idx));
    % hold on;
    % plot(tRange(1:idx), dw_i(1, 1:idx));
    % yline(0.0);
    % xlabel("$t$ (s)",'Interpreter','latex');
    % ylabel("$\dot{\omega}_{x}(t)$ (rad/s)",'Interpreter','latex');
    % legend("Analytical", "Actual", 'Location', 'best');
    % title("$\dot{\omega}_{x}$ vs $t$ Plot",'Interpreter','latex');
    % grid on;
    % 
    % subplot(2, 3, 5);
    % plot(tRange(1:idx), dw_a(2, 1:idx));
    % hold on;
    % plot(tRange(1:idx), dw_i(2, 1:idx));
    % yline(0.0);
    % xlabel("$t$ (s)",'Interpreter','latex');
    % ylabel("$\dot{\omega}_{y}(t)$ (rad/s)",'Interpreter','latex');
    % legend("Analytical", "Actual", 'Location', 'best');
    % title("$\dot{\omega}_{y}$ vs $t$ Plot",'Interpreter','latex');
    % grid on;
    % 
    % subplot(2, 3, 6);
    % plot(tRange(1:idx), dw_a(3, 1:idx));
    % hold on;
    % plot(tRange(1:idx), dw_i(3, 1:idx));
    % % yline(0.0);
    % xlabel("$t$ (s)",'Interpreter','latex');
    % ylabel("$\dot{\omega}_{z}(t)$ (rad/s)",'Interpreter','latex');
    % legend("Analytical", "Actual", 'Location', 'best');
    % title("$\dot{\omega}_{z}$ vs $t$ Plot",'Interpreter','latex');
    % % hold off;
    % grid on;

    parameters_7 = '\textbf{Analysis of Normalized Position Control}';
    figure(7);
    clf;
    sgtitle(parameters_7,'Interpreter','latex','FontSize',10);
    
    subplot(3, 2, 1);
    plot(tRange(1:idx), p_1(1,1:idx), '--r', tRange(1:idx), s_a(1,1:idx) - s_d(1), ...
        '-y');
    hold on;
    plot(tRange(1:idx), s_e(1,1:idx), '--c');
    plot(tRange(1:idx), -p_1(1,1:idx), '--r');
    yline(0);
    xlabel("$t$ (s)",'Interpreter','latex');
    ylabel("$\hat{x}_e$ (rad)",'Interpreter','latex');
    legend("$p_{1x}$","$\hat{x}_e$","$S_{1x}p_{1x}$",'Interpreter','latex');
    title("Performance-constrained $\hat{x}_e$ vs $t$ Plot",'Interpreter','latex');
    % hold off;
    grid on;
     
    subplot(3, 2, 3);
    plot(tRange(1:idx), squeeze(S_1(1,1,1:idx)), '-c');
    yline(0);
    xlabel("$t$ (s)",'Interpreter','latex');
    ylabel("$S_{1x}$",'Interpreter','latex');
    title("$S_{1x}$ vs $t$ Plot",'Interpreter','latex');
    % hold off;
    grid on;
       
    subplot(3, 2, 5);
    plot(tRange(1:idx), zeta_1(1,1:idx), '-c');
    yline(0);
    xlabel("$t$ (s)",'Interpreter','latex');
    ylabel("$\zeta_{1x}$ (rad)",'Interpreter','latex');
    title("$\zeta_{1x}$ vs $t$ Plot",'Interpreter','latex');
    % hold off;
    grid on;
    
    subplot(3, 2, 2);
    plot(tRange(1:idx), p_1(2,1:idx), '--r', tRange(1:idx), s_a(2,1:idx) - s_d(2), ...
        '-y');
    hold on;
    plot(tRange(1:idx), s_e(2,1:idx), '--c');
    plot(tRange(1:idx), -p_1(2,1:idx), '--r');
    yline(0);
    xlabel("$t$ (s)",'Interpreter','latex');
    ylabel("$\hat{y}_e$ (rad)",'Interpreter','latex');
    legend("$$p_{1y}$","$\hat{y}_e$","$S_{1y}p_{1y}$",'Interpreter','latex');
    title("Performance-constrained $\hat{y}_e$ vs $t$ Plot",'Interpreter','latex');
    % hold off;
    grid on;
     
    subplot(3, 2, 4);
    plot(tRange(1:idx), squeeze(S_1(2,2,1:idx)), '-c');
    yline(0);
    xlabel("$t$ (s)",'Interpreter','latex');
    ylabel("$S_{1y}$",'Interpreter','latex');
    title("$S_{1y}$ vs $t$ Plot",'Interpreter','latex');
    % hold off;
    grid on;
    
    subplot(3, 2, 6);
    plot(tRange(1:idx), zeta_1(2,1:idx), '-c');
    yline(0);
    xlabel("$t$ (s)",'Interpreter','latex');
    ylabel("$\zeta_{2x}$ (rad)",'Interpreter','latex');
    title("$\zeta_{2x}$ vs $t$ Plot",'Interpreter','latex');
    % hold off;
    grid on;

    % figure(8);
    % clf;
    % plot(squeeze(V_nP_a(1,1,1:idx)), squeeze(V_nP_a(2,2,1:idx)), '-b', ...
    %     squeeze(V_nP_a(1,2,1:idx)), squeeze(V_nP_a(2,2,1:idx)), '-c', ...
    %     squeeze(V_nP_a(1,3,1:idx)), squeeze(V_nP_a(2,3,1:idx)), '-g', ...
    %     squeeze(V_nP_a(1,4,1:idx)), squeeze(V_nP_a(2,4,1:idx)), '-r');
    % hold on
    % scatter(V_nP_d(1,:), V_nP_d(2,:), "y*");
    % xlim([-160 160])
    % ylim([-120 120])
    % xlabel("$\hat{x}$ (rad)",'Interpreter','latex');
    % ylabel("$\hat{y}$ (rad)",'Interpreter','latex');
    % legend("$\hat{y}$","$\hat{y}$","$S_{1x}p_{1x}$",'Interpreter','latex');
    % title("Virtual Plane Plot",'Interpreter','latex');
    % % hold off;
    % grid on;

    parameters_2 = {'\textbf{Outer-Loop Control Parameters:}' ['$K_{zp' ...
        '}$ = diag([' num2str(K.zp(1,1)) ', ' num2str(K.zp(2,2)) ']), $K_{zi}$ = diag([' num2str(K.zi(1,1)) ...
        ', ' num2str(K.zi(2,2)) ']), $K_{zd}$ ' ...
        '= diag([' num2str(K.zd(1,1)) ', ' num2str(K.zd(2,2)) ']), $\mathbf{\gamma}_1$ = [' ...
        num2str(K.gamma_1(1)) ', ' num2str(K.gamma_1(2)) '], $\mathbf{p}_{10}$ = [' num2str(K.p_10(1)) ...
        ', ' num2str(K.p_10(2)) '], $\mathbf{p}_' ...
        '{1\infty}$ = [' num2str(K.p_1inf(1)) ', ' num2str(K.p_1inf(2)) ']'] ['$\mathbf{\gamma}_2$ = [' ...
        num2str(K.gamma_2(1)) ', ' num2str(K.gamma_2(2)) ', ' ...
        num2str(K.gamma_2(3)) '], $\mathbf{p}_{20}$ = [' num2str(K.p_20(1)) ...
        ', ' num2str(K.p_20(2)) ', ' num2str(K.p_20(3)) '], $\mathbf{p}_' ...
        '{2\infty}$ = [' num2str(K.p_2inf(1)) ', ' num2str(K.p_2inf(2)) ', ' ...
        num2str(K.p_2inf(3)) ']'] ['$\Omega$ = diag([' num2str(K.Omega(1,1))...
        ', ' num2str(K.Omega(2,2)) ', ' num2str(K.Omega(3,3)) ']), ' ...
        '$\Gamma$ = diag([' num2str(K.Gamma(1,1)) ', '...
        num2str(K.Gamma(2,2)) ', ' num2str(K.Gamma(3,3)) [']), $P$ = ' ...
        'diag(['] num2str(K.P(1,1)) ', ' num2str(K.P(2,2)) ', ' ...
        num2str(K.P(3,3)) ']), '] ['$N$ = diag([' num2str(K.N(1,1)) ',' ...
        ' ' num2str(K.N(2,2)) ', ' num2str(K.N(3,3)) ']), $\mathbf{' ...
        '\kappa}$ = [' num2str(K.kappa_0(1)) ', ' num2str(K.kappa_0(2))...
        ', ' num2str(K.kappa_0(3)) ']' '$^\top$, $E$ = diag([' ...
        num2str(K.E(1,1)) ', ' num2str(K.E(2,2)) ', ' num2str(K.E(3,3)) ...
        '])']};
    figure(2);
    clf;
    sgtitle(parameters_2,'Interpreter','latex','FontSize',10);
    
    subplot(4, 4, 1);
    plot(tRange(1:idx), s_a(1, 1:idx));
    hold on;
    plot(tRange(1:idx), s_i(1, 1:idx));
    yline(s_d(1), '--c');
    yline(0.0);
    xlabel("$t$ (s)",'Interpreter','latex');
    ylabel("$\hat{x}(t)$ (rad)",'Interpreter','latex');
    legend("Analytical", "Actual", "Desired", 'Location', 'best');
    title("$\hat{x}$ vs $t$ Plot",'Interpreter','latex');
    % hold off;
    grid on;
    
    subplot(4, 4, 2);
    plot(tRange(1:idx), p_1(1,1:idx), '--r', tRange(1:idx), s_a(1,1:idx) - s_d(1), ...
        '-y', tRange(1:idx), -p_1(1,1:idx), '--r');
    yline(0);
    xlabel("$t$ (s)",'Interpreter','latex');
    ylabel("$\hat{x}_e$ (rad)",'Interpreter','latex');
    legend("$$p_{1x}$","$\hat{x}_e$",'Interpreter','latex');
    title("Performance-constrained $\hat{x}_e$ vs $t$ Plot",'Interpreter','latex');
    % hold off;
    grid on;
    
    subplot(4, 4, 3);
    plot(tRange(1:idx), h_a(1, 1:idx));
    hold on;
    plot(tRange(1:idx), h_i(1, 1:idx));
    plot(tRange(1:idx), h_d(1,1:idx), '--c');
    yline(0.0);
    xlabel("$t$ (s)",'Interpreter','latex');
    ylabel("$h_{x}(t)$ (rad/s)",'Interpreter','latex');
    % legend("Analytical", "Desired", 'Location', 'best');
    legend("Analytical", "Actual", "Desired", 'Location', 'best');
    title("$h_{x}$ vs $t$ Plot",'Interpreter','latex');
    grid on;
    
    subplot(4, 4, 4);
    plot(tRange(1:idx), p_2(1,1:idx), '--r', tRange(1:idx), h_a(1,1:idx) - h_d(1,1:idx), ...
        '-y', tRange(1:idx), -p_2(1,1:idx), '--r');
    yline(0);
    xlabel("$t$ (s)",'Interpreter','latex');
    ylabel("$h_{xe}$ (rad/s)",'Interpreter','latex');
    legend("$p_{2x}$","$h_{xe}$",'Interpreter','latex');
    title("Performance-constrained $h_{xe}$ vs $t$ Plot",'Interpreter','latex');
    % hold off;
    grid on;
    
    subplot(4, 4, 5);
    plot(tRange(1:idx), s_a(2, 1:idx));
    hold on;
    plot(tRange(1:idx), s_i(2, 1:idx));
    yline(s_d(2), '--c');
    yline(0.0);
    xlabel("$t$ (s)",'Interpreter','latex');
    ylabel("$\hat{y}(t)$ (rad)",'Interpreter','latex');
    legend("Analytical", "Actual", "Desired", 'Location', 'best');
    title("$\hat{y}$ vs $t$ Plot",'Interpreter','latex');
    % hold off;
    grid on;
    
    subplot(4, 4, 6);
    plot(tRange(1:idx), p_1(2,1:idx), '--r', tRange(1:idx), s_a(2,1:idx) - s_d(2), ...
        '-y', tRange(1:idx), -p_1(2,1:idx), '--r');
    yline(0);
    xlabel("$t$ (s)",'Interpreter','latex');
    ylabel("$\hat{y}_e$ (rad)",'Interpreter','latex');
    legend("$$p_{1y}$","$\hat{y}_e$",'Interpreter','latex');
    title("Performance-constrained $\hat{y}_e$ vs $t$ Plot",'Interpreter','latex');
    % hold off;
    grid on;
        
    subplot(4, 4, 7);
    plot(tRange(1:idx), h_a(2, 1:idx));
    hold on;
    plot(tRange(1:idx), h_i(2, 1:idx));
    plot(tRange(1:idx), h_d(2,1:idx), '--c');
    yline(0.0);
    xlabel("$t$ (s)",'Interpreter','latex');
    ylabel("$h_{y}(t)$ (rad/s)",'Interpreter','latex');
    % legend("Analytical", "Desired", 'Location', 'best');
    legend("Analytical", "Actual", "Desired", 'Location', 'best');
    title("$h_{y}$ vs $t$ Plot",'Interpreter','latex');
    grid on;
    
    subplot(4, 4, 8);
    plot(tRange(1:idx), p_2(2,1:idx), '--r', tRange(1:idx), h_a(2,1:idx) - h_d(2,1:idx), ...
        '-y', tRange(1:idx), -p_2(2,1:idx), '--r');
    yline(0);
    xlabel("$t$ (s)",'Interpreter','latex');
    ylabel("$h_{ye}$ (rad/s)",'Interpreter','latex');
    legend("$p_{2y}$","$h_{ye}$",'Interpreter','latex');
    title("Performance-constrained $h_{ye}$ vs $t$ Plot",'Interpreter','latex');
    % hold off;
    grid on;

    subplot(4, 4, 11);
    plot(tRange(1:idx), h_a(3, 1:idx));
    hold on;
    plot(tRange(1:idx), h_i(3, 1:idx));
    plot(tRange(1:idx), h_d(3,1:idx), '--c');
    % yline(0.0);
    xlabel("$t$ (s)",'Interpreter','latex');
    ylabel("$h_{r}(t)$ (rad/s)",'Interpreter','latex');
    legend("Analytical", "Actual", "Desired", 'Location', 'best');
    title("$h_{r}$ vs $t$ Plot",'Interpreter','latex');
    % hold off;
    grid on;

    subplot(4, 4, 12);
    plot(tRange(1:idx), p_2(3,1:idx), '--r', tRange(1:idx), h_a(3,1:idx) - h_d(3,1:idx), ...
        '-y', tRange(1:idx), -p_2(3,1:idx), '--r');
    yline(0);
    xlabel("$t$ (s)",'Interpreter','latex');
    ylabel("$h_{re}$ (rad/s)",'Interpreter','latex');
    legend("$$p_{2r}$","$h_{re}$",'Interpreter','latex');
    title("Performance-constrained $h_{re}$ vs $t$ Plot",'Interpreter','latex');
    % hold off;
    grid on;
            
    subplot(4, 4, 13);
    plot(tRange(1:idx), s_a(3, 1:idx));
    hold on;
    plot(tRange(1:idx), s_i(3, 1:idx));
    yline(s_d(4), '--c');
    % yline(0.0);
    xlabel("$t$ (s)",'Interpreter','latex');
    ylabel("$\alpha(t)$ (rad)",'Interpreter','latex');
    legend("Analytical", "Actual", "Desired", 'Location', 'best');
    title("$\alpha$ vs $t$ Plot",'Interpreter','latex');
    % hold off;
    grid on;
    
    subplot(4, 4, 15);
    plot(tRange(1:idx), C_w_c(3,1:idx));
    hold on;
    plot(tRange(1:idx), C_w_cd(3,1:idx), '--c');
    plot(tRange(1:idx), w_t(3,1:idx), '.r');
    xlabel("$t$ (s)",'Interpreter','latex');
    ylabel("$\dot{\psi}(t)$ (rad/s)",'Interpreter','latex');
    legend("Camera - Actual", "Camera - Desired", "Target",'Location', 'best');
    title("$\dot{\psi}$ vs $t$ Plot",'Interpreter','latex');
    % hold off;
    grid on;
    
    parameters_3 = {'\textbf{Inner-Loop Control Parameters:}' ['$K_{wp}$ = ' ...
        'diag([' num2str(K.wp(1,1)) ', ' num2str(K.wp(2,2)) ', ' ...
        num2str(K.wp(3,3)) ']), $K_{wi}$ = diag([' num2str(K.wi(1,1)) ', ' ...
        num2str(K.wi(2,2)) ', ' num2str(K.wi(3,3)) ']), $K_{wd}$ = diag([' ...
        num2str(K.wd(1,1)) ', ' num2str(K.wd(2,2)) ', ' num2str(K.wd(3,3)) '])']};
    figure(3);
    clf;
    sgtitle(parameters_3,'Interpreter','latex','FontSize',10);
    
    subplot(4, 4, 1);
    plot(tRange(1:idx), C_w_c(1,1:idx));
    hold on;
    plot(tRange(1:idx), C_w_cd(1,1:idx), '--c');
    plot(tRange(1:idx), w_t(1,1:idx), '.r');
    xlabel("$t$ (s)",'Interpreter','latex');
    ylabel("$\dot{\phi}(t)$ (rad/s)",'Interpreter','latex');
    legend("Camera - Actual", "Camera - Desired", "Target",'Location', 'best');
    title("$\dot{\phi}$ vs $t$ Plot",'Interpreter','latex');
    % hold off;
    grid on;
    
    subplot(4, 4, 2);
    plot(tRange(1:idx), C_w_c(1,1:idx));
    hold on;
    plot(tRange(1:idx), C_w_cd(1,1:idx), '--c');
    % yline(0);
    xlabel("$t$ (s)",'Interpreter','latex');
    ylabel("$\omega_x(t)$ (rad/s)",'Interpreter','latex');
    legend("Actual", "Desired",'Location', 'best');
    title("$\omega_x$ vs $t$ Plot",'Interpreter','latex');
    % hold off;
    grid on;
    
    % subplot(4, 4, 3);
    % plot(tRange(1:idx), dC_w_cd(1,1:idx));
    % xlabel("$t$ (s)",'Interpreter','latex');
    % ylabel("$\dot{\omega}_{x}(t)$ (rad/s$^2$)",'Interpreter','latex');
    % title("$\dot{\omega}_{x}$ vs $t$ Plot",'Interpreter','latex');
    % grid on;
    
    subplot(4, 4, 4);
    plot(tRange(1:idx), tau(1,1:idx));
    % yline(0);
    % axis([0 tRange(i) -0.1 2*v_yB(1)]);
    xlabel("$t$ (s)",'Interpreter','latex');
    ylabel("$\tau_x(t)$ (N m)",'Interpreter','latex');
    % legend("Camera", "Target",'Location', 'best');
    title("$\tau_x$ vs $t$ Plot",'Interpreter','latex');
    % hold off;
    grid on;
    
    subplot(4, 4, 5);
    plot(tRange(1:idx), C_w_c(2,1:idx));
    hold on;
    plot(tRange(1:idx), C_w_cd(2,1:idx), '--c');
    plot(tRange(1:idx), w_t(2,1:idx), '.r');
    xlabel("$t$ (s)",'Interpreter','latex');
    ylabel("$\dot{\theta}(t)$ (rad/s)",'Interpreter','latex');
    legend("Camera - Actual", "Camera - Desired", "Target",'Location', 'best');
    title("$\dot{\theta}$ vs $t$ Plot",'Interpreter','latex');
    % hold off;
    grid on;
    
    subplot(4, 4, 6);
    plot(tRange(1:idx), C_w_c(2,1:idx));
    hold on;
    plot(tRange(1:idx), C_w_cd(2,1:idx), '--c');
    % yline(0);
    xlabel("$t$ (s)",'Interpreter','latex');
    ylabel("$\omega_y(t)$ (rad/s)",'Interpreter','latex');
    legend("Actual", "Desired",'Location', 'best');
    title("$\omega_y$ vs $t$ Plot",'Interpreter','latex');
    % hold off;
    grid on;
    
    % subplot(4, 4, 7);
    % plot(tRange(1:idx), dC_w_cd(2,1:idx));
    % xlabel("$t$ (s)",'Interpreter','latex');
    % ylabel("$\dot{\omega}_{y}(t)$ (rad/s$^2$)",'Interpreter','latex');
    % title("$\dot{\omega}_{y}$ vs $t$ Plot",'Interpreter','latex');
    % grid on;
    
    subplot(4, 4, 8);
    plot(tRange(1:idx), tau(2,1:idx));
    % yline(0);
    % axis([0 tRange(i) -0.1 2*v_yB(1)]);
    xlabel("$t$ (s)",'Interpreter','latex');
    ylabel("$\tau_y(t)$ (N m)",'Interpreter','latex');
    % legend("Camera", "Target",'Location', 'best');
    title("$\tau_y$ vs $t$ Plot",'Interpreter','latex');
    % hold off;
    grid on;
    
    subplot(4, 4, 9);
    plot(tRange(1:idx), C_w_c(3,1:idx));
    hold on;
    plot(tRange(1:idx), C_w_cd(3,1:idx), '--c');
    plot(tRange(1:idx), w_t(3,1:idx), '.r');
    xlabel("$t$ (s)",'Interpreter','latex');
    ylabel("$\dot{\psi}(t)$ (rad/s)",'Interpreter','latex');
    legend("Camera - Actual", "Camera - Desired", "Target",'Location', 'best');
    title("$\dot{\psi}$ vs $t$ Plot",'Interpreter','latex');
    % hold off;
    grid on;
    
    subplot(4, 4, 10);
    plot(tRange(1:idx), C_w_c(3,1:idx));
    hold on;
    plot(tRange(1:idx), C_w_cd(3,1:idx), '--c');
    % yline(0);
    xlabel("$t$ (s)",'Interpreter','latex');
    ylabel("$\omega_z(t)$ (rad/s)",'Interpreter','latex');
    legend("Actual", "Desired",'Location', 'best');
    title("$\omega_z$ vs $t$ Plot",'Interpreter','latex');
    % hold off;
    grid on;
    
    % subplot(4, 4, 11);
    % plot(tRange(1:idx), dC_w_cd(3,1:idx));
    % xlabel("$t$ (s)",'Interpreter','latex');
    % ylabel("$\dot{\omega}_{z}(t)$ (rad/s$^2$)",'Interpreter','latex');
    % title("$\dot{\omega}_{z}$ vs $t$ Plot",'Interpreter','latex');
    % grid on;
    
    subplot(4, 4, 12);
    plot(tRange(1:idx), tau(3,1:idx));
    % yline(0);
    % axis([0 tRange(i) -0.1 2*v_yB(1)]);
    xlabel("$t$ (s)",'Interpreter','latex');
    ylabel("$\tau_z(t)$ (N m)",'Interpreter','latex');
    % legend("Camera", "Target",'Location', 'best');
    title("$\tau_z$ vs $t$ Plot",'Interpreter','latex');
    % hold off;
    grid on;

    subplot(4, 4, 14);
    plot(tRange(1:idx), h_a(3, 1:idx));
    hold on;
    plot(tRange(1:idx), h_i(3, 1:idx));
    plot(tRange(1:idx), h_d(3,1:idx), '--c');
    % yline(0.0);
    xlabel("$t$ (s)",'Interpreter','latex');
    ylabel("$h_{r}(t)$ (rad/s)",'Interpreter','latex');
    legend("Analytical", "Actual", "Desired", 'Location', 'best');
    title("$h_{r}$ vs $t$ Plot",'Interpreter','latex');
    % hold off;
    grid on;
    
    subplot(4, 4, 15);
    plot(tRange(1:idx), a_d(3,1:idx));
    yline(0.0);
    xlabel("$t$ (s)",'Interpreter','latex');
    ylabel("$a_{zd}(t)$ (m/s$^2$)",'Interpreter','latex');
    title("$a_{zd}$ vs $t$ Plot",'Interpreter','latex');
    grid on;

    subplot(4, 4, 16);
    plot(tRange(1:idx), T(1:idx));
    hold on;
    yline(0);
    xlabel("$t$ (s)",'Interpreter','latex');
    ylabel("$T$ (N)",'Interpreter','latex');
    title("$T$ vs $t$ Plot",'Interpreter','latex');
    % hold off;
    grid on;
    
    parameters_1 = {'\textbf{Outer-Loop Control Parameters:}' ['$K_{ep' ...
        '}$ = diag([' num2str(K.ep(1,1)) ', ' num2str(K.ep(2,2)) ', '...
        num2str(K.ep(3,3)) ']), $K_{ei}$ = diag([' num2str(K.ei(1,1)) ', ' ...
        num2str(K.ei(2,2)) ', ' num2str(K.ei(3,3)) ']), $K_{ed}$ = diag([' ...
        num2str(K.ed(1,1)) ', ' num2str(K.ed(2,2)) ', ' num2str(K.ed(3,3)) ...
        '])']};
    figure(1);
    clf;
    sgtitle(parameters_1,'Interpreter','latex','FontSize',10);
    
    subplot(4, 4, 1);
    plot(tRange(1:idx), I_p_c(1,1:idx));
    hold on;
    plot(tRange(1:idx), I_p_t(1,1:idx), '.r');
    % yline(0);
    % axis([0 tRange(i) -0.1 2*x(1)]);
    xlabel("$t$ (s)",'Interpreter','latex');
    ylabel("$^Ix$ (m)",'Interpreter','latex');
    legend("Camera", "Target",'Location', 'best');
    title("$^Ix$ vs $t$ Plot",'Interpreter','latex');
    % hold off;
    grid on;
    
    subplot(4, 4, 2);
    plot(tRange(1:idx), I_v_c(1, 1:idx));
    hold on;
    plot(tRange(1:idx), I_dp_t(1,1:idx), '.r');
    % yline(0);
    % axis([0 tRange(i) -0.1 2*v_xB(1)]);
    xlabel("$t$ (s)",'Interpreter','latex');
    ylabel("$^Iv_x$ (m/s)",'Interpreter','latex');
    legend("Camera", "Target",'Location', 'best');
    title("$^Iv_x$ vs $t$ Plot",'Interpreter','latex');
    % hold off;
    grid on;
        
    subplot(4, 4, 3);
    plot(tRange(1:idx), a_d(1,1:idx));
    yline(0.0);
    xlabel("$t$ (s)",'Interpreter','latex');
    ylabel("$a_{xd}(t)$ (m/s$^2$)",'Interpreter','latex');
    title("$a_{xd}$ vs $t$ Plot",'Interpreter','latex');
    grid on;
    
    subplot(4, 4, 4);
    plot(tRange(1:idx), E_c(1,1:idx));
    hold on;
    plot(tRange(1:idx), E_d(1,1:idx), '--c');
    plot(tRange(1:idx), E_t(1,1:idx), '.r');
    xlabel("$t$ (s)",'Interpreter','latex');
    ylabel("$\phi(t)$ (rad)",'Interpreter','latex');
    legend("Camera - Actual", "Camera - Desired", "Target",'Location', 'best');
    title("$\phi$ vs $t$ Plot",'Interpreter','latex');
    % hold off;
    grid on;
    
    subplot(4, 4, 5);
    plot(tRange(1:idx), I_p_c(2,1:idx));
    hold on;
    plot(tRange(1:idx), I_p_t(2,1:idx), '.r');
    % yline(0);
    % axis([0 tRange(i) -0.1 2*y(1)]);
    xlabel("$t$ (s)",'Interpreter','latex');
    ylabel("$^Iy$ (m)",'Interpreter','latex');
    legend("Camera", "Target",'Location', 'best');
    title("$^Iy$ vs $t$ Plot",'Interpreter','latex');
    % hold off;
    grid on;
    
    subplot(4, 4, 6);
    plot(tRange(1:idx), I_v_c(2, 1:idx));
    hold on;
    plot(tRange(1:idx), I_dp_t(2,1:idx), '.r');
    % yline(0);
    % axis([0 tRange(i) -0.1 2*v_yB(1)]);
    xlabel("$t$ (s)",'Interpreter','latex');
    ylabel("$^Iv_y$ (m/s)",'Interpreter','latex');
    legend("Camera", "Target",'Location', 'best');
    title("$^Iv_y$ vs $t$ Plot",'Interpreter','latex');
    % hold off;
    grid on;
    
    subplot(4, 4, 7);
    plot(tRange(1:idx), a_d(2,1:idx));
    yline(0.0);
    xlabel("$t$ (s)",'Interpreter','latex');
    ylabel("$a_{yd}(t)$ (m/s$^2$)",'Interpreter','latex');
    title("$a_{yd}$ vs $t$ Plot",'Interpreter','latex');
    grid on;
    
    subplot(4, 4, 8);
    plot(tRange(1:idx), E_c(2,1:idx));
    hold on;
    plot(tRange(1:idx), E_d(2,1:idx), '--c');
    plot(tRange(1:idx), E_t(2,1:idx), '.r');
    xlabel("$t$ (s)",'Interpreter','latex');
    ylabel("$\theta(t)$ (rad)",'Interpreter','latex');
    legend("Camera - Actual", "Camera - Desired", "Target",'Location', 'best');
    title("$\theta$ vs $t$ Plot",'Interpreter','latex');
    % hold off;
    grid on;
    
    subplot(4, 4, 9);
    plot(tRange(1:idx), I_p_c(3,1:idx));
    hold on;
    plot(tRange(1:idx), I_p_t(3,1:idx), '.r');
    % yline(0);
    % axis([0 tRange(i) -0.1 2*z(1)]);
    xlabel("$t$ (s)",'Interpreter','latex');
    ylabel("$^Iz$ (m)",'Interpreter','latex');
    legend("Camera", "Target",'Location', 'best');
    title("$^Iz$ vs $t$ Plot",'Interpreter','latex');
    % hold off;
    grid on;
    
    subplot(4, 4, 10);
    plot(tRange(1:idx), I_v_c(3, 1:idx));
    hold on;
    plot(tRange(1:idx), I_dp_t(3,1:idx), '.r');
    % yline(0);
    % axis([0 tRange(i) -0.1 2*v_zB(1)]);
    xlabel("$t$ (s)",'Interpreter','latex');
    ylabel("$^Iv_z$ (m/s)",'Interpreter','latex');
    legend("Camera", "Target",'Location', 'best');
    title("$^Iv_z$ vs $t$ Plot",'Interpreter','latex');
    % hold off;
    grid on;
    
    subplot(4, 4, 11);
    plot(tRange(1:idx), a_d(3,1:idx));
    yline(0.0);
    xlabel("$t$ (s)",'Interpreter','latex');
    ylabel("$a_{zd}(t)$ (m/s$^2$)",'Interpreter','latex');
    title("$a_{zd}$ vs $t$ Plot",'Interpreter','latex');
    grid on;

    subplot(4, 4, 12);
    plot(tRange(1:idx), T(1:idx));
    hold on;
    yline(0);
    xlabel("$t$ (s)",'Interpreter','latex');
    ylabel("$T$ (N)",'Interpreter','latex');
    title("$T$ vs $t$ Plot",'Interpreter','latex');
    % hold off;
    grid on;
    
    subplot(4, 4, 13);
    plot(tRange(1:idx), E_c(3,1:idx));
    hold on;
    % plot(tRange(1:idx), E_d(3,1:idx), '--c');
    plot(tRange(1:idx), E_t(3,1:idx), '.r');
    xlabel("$t$ (s)",'Interpreter','latex');
    ylabel("$\psi(t)$ (rad)",'Interpreter','latex');
    legend("Camera", "Target",'Location', 'best');
    % legend("Camera - Actual", "Camera - Desired", "Target",'Location', 'best');
    title("$\psi$ vs $t$ Plot",'Interpreter','latex');
    % hold off;
    grid on;
    
    subplot(4, 4, 14);
    plot(tRange(1:idx), C_w_c(3,1:idx));
    hold on;
    plot(tRange(1:idx), C_w_cd(3,1:idx), '--c');
    plot(tRange(1:idx), w_t(3,1:idx), '.r');
    xlabel("$t$ (s)",'Interpreter','latex');
    ylabel("$\dot{\psi}(t)$ (rad/s)",'Interpreter','latex');
    legend("Camera - Actual", "Camera - Desired", "Target",'Location', 'best');
    title("$\dot{\psi}$ vs $t$ Plot",'Interpreter','latex');
    % hold off;
    grid on;
    
    subplot(4, 4, 15);
    plot(tRange(1:idx), C_w_c(3,1:idx));
    hold on;
    plot(tRange(1:idx), C_w_cd(3,1:idx), '--c');
    % yline(0);
    xlabel("$t$ (s)",'Interpreter','latex');
    ylabel("$\omega_z(t)$ (rad/s)",'Interpreter','latex');
    legend("Actual", "Desired",'Location', 'best');
    title("$\omega_z$ vs $t$ Plot",'Interpreter','latex');
    % hold off;
    grid on;
    % 
    % subplot(4, 4, 16);
    % plot(tRange(1:idx), dC_w_cd(3,1:idx));
    % xlabel("$t$ (s)",'Interpreter','latex');
    % ylabel("$\dot{\omega}_{z}(t)$ (rad/s$^2$)",'Interpreter','latex');
    % title("$\dot{\omega}_{z}$ vs $t$ Plot",'Interpreter','latex');
    % grid on;

    end