function plot_multi_image_plane(results)

figure; clf;

N = length(results);

for k = 1:N
    
    subplot(ceil(sqrt(N)), ceil(sqrt(N)), k);
    hold on; grid on;

    P = results(k).data.P_DS;
    nP = P(:,9:12,:);
    
    for i = 1:4
        x = squeeze(nP(1,i,:));
        y = squeeze(nP(2,i,:));

        % Trajectory (ONLY one visible in legend)
        if i == 1
            h_traj = plot(x, y, 'y', 'LineWidth', 1.2, ...
                'DisplayName','Trajectory');
        else
            plot(x, y, 'y', 'LineWidth', 1.2, ...
                'HandleVisibility','off');
        end

        % Start pixel
        h_start = plot(x(1), y(1), 'go', ...
            'MarkerFaceColor','g');

        % End pixel
        h_end = plot(x(end), y(end), 'bs', ...
            'MarkerFaceColor','b');
    end

    %% Desired pixels
    P_d = results(k).data.V_nP_d;
    nP_d = P_d(:,1:4,:);

    h_des = scatter(squeeze(nP_d(1,:,:)), ...
                    squeeze(nP_d(2,:,:)), ...
                    40, 'r', 'filled');

    %% Labels
    title(sprintf('Run %d', k));
    xlabel("$\hat{x}$",'Interpreter','latex');
    ylabel("$\hat{y}$",'Interpreter','latex');

    axis equal;
    xlim([-160 160])
    ylim([-120 120])

end

%% Legend (clean, no duplicates)
legend([h_traj, h_start, h_end, h_des], ...
    {'Trajectory','Start Pixel','End Pixel','Desired Pixel'}, ...
    'Location','best');

sgtitle('Image Plane Trajectories');

end