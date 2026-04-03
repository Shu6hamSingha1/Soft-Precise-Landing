%% =========================================================================
% plotter_comparison.m
%
% Comparison plotter: loads result_ctrl_N.mat files and overlays all
% controllers on common axes.
%
% V_X_DS row layout (from _temp.m line 452):
%   [V_s_i(1:2); V_s_i(4); V_h_i; V_w_i; V_dw_i;     <- rows 1-12  (image)
%    V_s_a(1:2); V_s_a(4); V_h_a; V_w_a;  V_dw_a]     <- rows 13-24 (analytical)
%   Row 1 = xhat_i,  Row 2 = yhat_i,  Row 3 = alpha_i
%   Row 4 = hx_i,    Row 5 = hy_i,    Row 6 = hr_i
%   Row 7 = wx_i,    Row 8 = wy_i,    Row 9 = wz_i
%   Row 10= dwx_i,   Row 11= dwy_i,   Row 12= dwz_i
%   Row 13= xhat_a,  Row 14= yhat_a,  Row 15= alpha_a
%   Row 16= hx_a,    Row 17= hy_a,    Row 18= hr_a
%   Row 19= wx_a,    Row 20= wy_a,    Row 21= wz_a
%   Row 22= dwx_a,   Row 23= dwy_a,   Row 24= dwz_a
%
% Figures produced:
%   10 – 3-D Trajectories
%   11 – Position vs time  (x, y, z)
%   12 – Velocity vs time  (vx, vy, vz)
%   13 – Image features vs time  (xhat, yhat, alpha) — analytical
%   14 – Optical flow vs time  (hx, hy, hr) — analytical + desired
%   15 – Euler angles vs time  (phi, theta, psi)
%   16 – Control inputs  (T, tau_x, tau_y, tau_z)
%   17 – UAV-to-target distance (key comparison metric)
%
% USAGE:
%   plotter_comparison          % loads controllers 1-5
%   plotter_comparison([1 3])   % loads only 1 and 3
% =========================================================================

function plotter_comparison(ctrl_list)

if nargin < 1
    ctrl_list = 1:5;
end

%% Colour / line style map (colour-blind friendly)
colours = { [0.00, 0.45, 0.70],  ...   % 1 PLASMC    – blue
            [0.85, 0.33, 0.10],  ...   % 2 Lin 2022  – red-orange
            [0.47, 0.67, 0.19],  ...   % 3 Zhang 2026– green
            [0.49, 0.18, 0.56],  ...   % 4 Chen 2025 – purple
            [0.93, 0.69, 0.13]};       % 5 Cho 2022  – gold

styles  = {'-', '--', ':', '-.', '-'};
lw      = [2.0, 1.8, 1.8, 1.8, 1.8];

ctrl_names = {'PLASMC (Proposed)', 'Lin 2022', ...
              'Zhang 2026',        'Chen 2025', 'Cho 2022'};

%% Load data
D = cell(1, 5);
for c = ctrl_list
    fname = sprintf('result_ctrl_%d.mat', c);
    if ~isfile(fname)
        warning('File %s not found – skipping controller %d.', fname, c);
        continue;
    end
    D{c} = load(fname);
    fprintf('Loaded %s  (%s)\n', fname, D{c}.ctrl_name);
end
loaded = ctrl_list(cellfun(@(x) ~isempty(x), D(ctrl_list)));
if isempty(loaded), error('No result files found.'); end

ref = D{loaded(1)};

%% =========================================================================
%  FIGURE 10 – 3-D TRAJECTORIES
% =========================================================================
fig10 = figure(10); clf(fig10); fig10.Color = 'w';
fig10.Position = [50, 50, 720, 560];
ax10 = axes(fig10); hold(ax10, 'on');

n0 = ref.idx;
plot3(ax10, ref.x_t(1,1:n0), ref.x_t(2,1:n0), ref.x_t(3,1:n0), ...
      '.r', 'MarkerSize', 4, 'DisplayName', 'Target');

for c = loaded
    d = D{c};  n = d.idx;
    plot3(ax10, d.X_DS(1,1:n), d.X_DS(2,1:n), d.X_DS(3,1:n), ...
          styles{c}, 'Color', colours{c}, 'LineWidth', lw(c), ...
          'DisplayName', ctrl_names{c});
end
grid(ax10,'on');
xlabel(ax10,"$^Ix$ (m)",'Interpreter','latex');
ylabel(ax10,"$^Iy$ (m)",'Interpreter','latex');
zlabel(ax10,"$^Iz$ (m)",'Interpreter','latex');
title(ax10,'\textbf{3D Trajectories – All Controllers}','Interpreter','latex');
legend(ax10,'Location','best','Interpreter','latex');
view(ax10, 45, 25);  set(ax10,'FontSize',11);

%% =========================================================================
%  FIGURE 11 – POSITION vs TIME
% =========================================================================
pos_labels = {"$^Ix$ (m)", "$^Iy$ (m)", "$^Iz$ (m)"};
pos_titles = {"$^Ix$ vs $t$ Plot","$^Iy$ vs $t$ Plot","$^Iz$ vs $t$ Plot"};

fig11 = figure(11); clf(fig11); fig11.Color = 'w';
fig11.Position = [780, 50, 800, 600];
sgtitle(fig11,'\textbf{UAV Position vs Time}','Interpreter','latex','FontSize',12);

for ax_i = 1:3
    subplot(3,1,ax_i); hold on;
    plot(ref.tRange(1:ref.idx), ref.x_t(ax_i,1:ref.idx), '.r', ...
         'MarkerSize',3,'DisplayName','Target');
    for c = loaded
        d = D{c};  n = d.idx;
        plot(d.tRange(1:n), d.X_DS(ax_i,1:n), styles{c}, ...
             'Color',colours{c},'LineWidth',lw(c),'DisplayName',ctrl_names{c});
    end
    grid on;
    xlabel("$t$ (s)",'Interpreter','latex');
    ylabel(pos_labels{ax_i},'Interpreter','latex');
    title(pos_titles{ax_i},'Interpreter','latex');
    if ax_i==1, legend('Location','best','Interpreter','latex','FontSize',8); end
    set(gca,'FontSize',10);
end

%% =========================================================================
%  FIGURE 12 – VELOCITY vs TIME
% =========================================================================
vel_labels = {"$^Iv_x$ (m/s)","$^Iv_y$ (m/s)","$^Iv_z$ (m/s)"};
vel_titles = {"$^Iv_x$ vs $t$ Plot","$^Iv_y$ vs $t$ Plot","$^Iv_z$ vs $t$ Plot"};

fig12 = figure(12); clf(fig12); fig12.Color = 'w';
fig12.Position = [50, 650, 800, 600];
sgtitle(fig12,'\textbf{UAV Velocity vs Time}','Interpreter','latex','FontSize',12);

for ax_i = 1:3
    subplot(3,1,ax_i); hold on;
    plot(ref.tRange(1:ref.idx), ref.dx_t(ax_i,1:ref.idx), '.r', ...
         'MarkerSize',3,'DisplayName','Target');
    for c = loaded
        d = D{c};  n = d.idx;
        plot(d.tRange(1:n), d.X_DS(7+ax_i,1:n), styles{c}, ...
             'Color',colours{c},'LineWidth',lw(c),'DisplayName',ctrl_names{c});
    end
    grid on;
    yline(0,'--k','LineWidth',0.8,'HandleVisibility','off');
    xlabel("$t$ (s)",'Interpreter','latex');
    ylabel(vel_labels{ax_i},'Interpreter','latex');
    title(vel_titles{ax_i},'Interpreter','latex');
    if ax_i==1, legend('Location','best','Interpreter','latex','FontSize',8); end
    set(gca,'FontSize',10);
end

%% =========================================================================
%  FIGURE 13 – IMAGE FEATURES vs TIME  (analytical: rows 13,14,15 of V_X_DS)
% =========================================================================
% V_X_DS rows: 13=xhat_a, 14=yhat_a, 15=alpha_a
feat_rows   = [13, 14, 15];
feat_labels = {"$\hat{x}(t)$ (rad)","$\hat{y}(t)$ (rad)","$\alpha(t)$ (rad)"};
feat_titles = {"$\hat{x}$ vs $t$ Plot","$\hat{y}$ vs $t$ Plot","$\alpha$ vs $t$ Plot"};
s_d_vals    = [ref.V_s_d(1), ref.V_s_d(2), ref.V_s_d(4)];

fig13 = figure(13); clf(fig13); fig13.Color = 'w';
fig13.Position = [780, 650, 800, 600];
sgtitle(fig13,'\textbf{Image Features vs Time}','Interpreter','latex','FontSize',12);

for ax_i = 1:3
    subplot(3,1,ax_i); hold on;
    yline(s_d_vals(ax_i),'--k','LineWidth',1.2,'DisplayName','Desired');
    for c = loaded
        d = D{c};  n = d.idx;
        plot(d.tRange(1:n), d.V_X_DS(feat_rows(ax_i),1:n), styles{c}, ...
             'Color',colours{c},'LineWidth',lw(c),'DisplayName',ctrl_names{c});
    end
    grid on;
    xlabel("$t$ (s)",'Interpreter','latex');
    ylabel(feat_labels{ax_i},'Interpreter','latex');
    title(feat_titles{ax_i},'Interpreter','latex');
    if ax_i==1, legend('Location','best','Interpreter','latex','FontSize',8); end
    set(gca,'FontSize',10);
end

%% =========================================================================
%  FIGURE 14 – OPTICAL FLOW vs TIME  (analytical: rows 16,17,18 + desired)
% =========================================================================
% V_X_DS rows: 16=hx_a, 17=hy_a, 18=hr_a
h_rows   = [16, 17, 18];
h_labels = {"$h_x(t)$ (rad/s)","$h_y(t)$ (rad/s)","$h_r(t)$ (rad/s)"};
h_titles = {"$h_x$ vs $t$ Plot","$h_y$ vs $t$ Plot","$h_r$ vs $t$ Plot"};

fig14 = figure(14); clf(fig14); fig14.Color = 'w';
fig14.Position = [50, 50, 800, 600];
sgtitle(fig14,'\textbf{Optical Flow vs Time}','Interpreter','latex','FontSize',12);

for ax_i = 1:3
    subplot(3,1,ax_i); hold on;
    for c = loaded
        d = D{c};  n = d.idx;
        % Analytical optical flow
        plot(d.tRange(1:n), d.V_X_DS(h_rows(ax_i),1:n), styles{c}, ...
             'Color',colours{c},'LineWidth',lw(c),'DisplayName',ctrl_names{c});
        % Desired (dashed black, first loaded only to avoid legend clutter)
        if c == loaded(1) && isfield(d,'V_h_d') && ~isempty(d.V_h_d)
            plot(d.tRange(1:n), d.V_h_d(ax_i,1:n), '--k','LineWidth',1.0, ...
                 'DisplayName','Desired');
        end
    end
    grid on;
    yline(0,'-','Color',[0.7 0.7 0.7],'LineWidth',0.6,'HandleVisibility','off');
    xlabel("$t$ (s)",'Interpreter','latex');
    ylabel(h_labels{ax_i},'Interpreter','latex');
    title(h_titles{ax_i},'Interpreter','latex');
    if ax_i==1, legend('Location','best','Interpreter','latex','FontSize',8); end
    set(gca,'FontSize',10);
end

%% =========================================================================
%  FIGURE 15 – EULER ANGLES vs TIME
% =========================================================================
euler_labels = {"$\phi(t)$ (rad)","$\theta(t)$ (rad)","$\psi(t)$ (rad)"};
euler_titles = {"$\phi$ vs $t$ Plot","$\theta$ vs $t$ Plot","$\psi$ vs $t$ Plot"};

fig15 = figure(15); clf(fig15); fig15.Color = 'w';
fig15.Position = [780, 50, 800, 600];
sgtitle(fig15,'\textbf{Euler Angles vs Time}','Interpreter','latex','FontSize',12);

for ax_i = 1:3
    subplot(3,1,ax_i); hold on;
    for c = loaded
        d  = D{c};  n = d.idx;
        E_all = quat2eul(d.X_DS(4:7,1:n)', 'XYZ')';   % [3 x n]
        if ax_i == 3 && c == loaded(1)
            E_t = quat2eul(d.x_t(4:7,1:n)', 'XYZ')';
            plot(d.tRange(1:n), E_t(3,:), '.r','MarkerSize',3,'DisplayName','Target $\psi$');
        end
        plot(d.tRange(1:n), E_all(ax_i,:), styles{c}, ...
             'Color',colours{c},'LineWidth',lw(c),'DisplayName',ctrl_names{c});
    end
    grid on;
    yline(0,'-','Color',[0.7 0.7 0.7],'LineWidth',0.6,'HandleVisibility','off');
    xlabel("$t$ (s)",'Interpreter','latex');
    ylabel(euler_labels{ax_i},'Interpreter','latex');
    title(euler_titles{ax_i},'Interpreter','latex');
    if ax_i==1, legend('Location','best','Interpreter','latex','FontSize',8); end
    set(gca,'FontSize',10);
end

%% =========================================================================
%  FIGURE 16 – CONTROL INPUTS  (T, tau_x, tau_y, tau_z)
% =========================================================================
% U_DS rows: [tau_x; tau_y; tau_z; T]  (from UAVDyn.m u_2 = [B_tau_cd; u_1(4)])
u_rows    = [4, 1, 2, 3];
u_labels  = {"$T(t)$ (N)","$\tau_x(t)$ (N\,m)","$\tau_y(t)$ (N\,m)","$\tau_z(t)$ (N\,m)"};
u_titles  = {"$T$ vs $t$","$\tau_x$ vs $t$","$\tau_y$ vs $t$","$\tau_z$ vs $t$"};

fig16 = figure(16); clf(fig16); fig16.Color = 'w';
fig16.Position = [50, 650, 900, 600];
sgtitle(fig16,'\textbf{Control Inputs vs Time}','Interpreter','latex','FontSize',12);

for ax_i = 1:4
    subplot(2,2,ax_i); hold on;
    for c = loaded
        d = D{c};  n = d.idx;
        plot(d.tRange(1:n), d.U_DS(u_rows(ax_i),1:n), styles{c}, ...
             'Color',colours{c},'LineWidth',lw(c),'DisplayName',ctrl_names{c});
    end
    grid on;
    yline(0,'-','Color',[0.7 0.7 0.7],'LineWidth',0.6,'HandleVisibility','off');
    xlabel("$t$ (s)",'Interpreter','latex');
    ylabel(u_labels{ax_i},'Interpreter','latex');
    title(u_titles{ax_i},'Interpreter','latex');
    if ax_i==1, legend('Location','best','Interpreter','latex','FontSize',8); end
    set(gca,'FontSize',10);
end

%% =========================================================================
%  FIGURE 17 – UAV-TO-TARGET DISTANCE  (key comparison metric)
% =========================================================================
fig17 = figure(17); clf(fig17); fig17.Color = 'w';
fig17.Position = [780, 650, 720, 400];
ax17 = axes(fig17); hold(ax17,'on');

for c = loaded
    d   = D{c};  n = d.idx;
    err = vecnorm(d.X_DS(1:3,1:n) - d.x_t(1:3,1:n));
    plot(ax17, d.tRange(1:n), err, styles{c}, ...
         'Color',colours{c},'LineWidth',lw(c),'DisplayName',ctrl_names{c});
end
grid(ax17,'on');
yline(ax17, 0.1, '--k', 'LineWidth', 1.0, 'DisplayName', 'Landing threshold');
xlabel(ax17,"$t$ (s)",'Interpreter','latex');
ylabel(ax17,"$\|r_e\|$ (m)",'Interpreter','latex');
title(ax17,'\textbf{UAV-to-Target Distance vs Time}','Interpreter','latex');
legend(ax17,'Location','best','Interpreter','latex');
set(ax17,'FontSize',11);

%% =========================================================================
%  SUMMARY TABLE
% =========================================================================
fprintf('\n%s\n', repmat('=',1,62));
fprintf('%-22s  %8s  %12s  %8s\n','Controller','Steps','FinalErr(m)','Time(s)');
fprintf('%s\n', repmat('-',1,62));
for c = loaded
    d   = D{c};  n = d.idx;
    err = norm(d.X_DS(1:3,n) - d.x_t(1:3,n));
    fprintf('%-22s  %8d  %12.4f  %8.2f\n', ctrl_names{c}, n, err, d.tRange(n));
end
fprintf('%s\n\n', repmat('=',1,62));

end
