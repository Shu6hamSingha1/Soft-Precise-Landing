function plotter_adaptive(data)
%% Extracting Solution
    I_p_c = data.X_DS(1:3,:);
    
    q_c = data.X_DS(4:7,:);
    
    I_v_c = data.X_DS(8:10,:);
    
    C_w_c = data.X_DS(11:13,:);

    tau = data.U_DS(1:3,:);
    
    T = data.U_DS(4,:);
    
    a_d = data.D_DS(1:3,:);
    E_d = data.D_DS(4:5,:);
    C_w_cd = data.D_DS(6:8,:);

    I_p_t = data.x_t(1:3,:);
    
    I_dp_t = data.dx_t(1:3,:);
    
    q_t = data.x_t(4:7,:);
    
    w_t = data.dx_t(4:6,:);

    idx = data.idx - 1;

    tRange = data.tRange;

    %% Calculating Euler Angles from Quaternions
    E_c = quat2eul(q_c','XYZ');
    E_c = E_c';
    E_t = quat2eul(q_t','XYZ'); 
    E_t = E_t';

    %% Plotting the results        
    figure(1);
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
    
    figure(1);
    clf;
    
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