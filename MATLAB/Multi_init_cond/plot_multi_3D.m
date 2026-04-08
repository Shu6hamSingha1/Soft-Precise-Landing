function plot_multi_3D(results)

figure;
clf;
hold on; grid on;

N = length(results);
cmap = lines(N);

for k = 1:N

    idx_k = results(k).data.idx;
    X = results(k).data.X_DS;
    I_p_c = X(1:3, 1:idx_k+1);

    plot3(I_p_c(1,:), I_p_c(2,:), -I_p_c(3,:), ...
        'Color',cmap(k,:), ...
        'LineWidth',1.2, ...
        'DisplayName',sprintf('Run %d',k));
end

idxValues = arrayfun(@(r) r.data.idx, results);
[~, whichResult] = max(idxValues);
idx_t = idxValues(whichResult);
x_t = results(whichResult).data.x_t;
I_p_t = x_t(1:3, 1:idx_t);

plot3(I_p_t(1,:), I_p_t(2,:), -I_p_t(3,:), ...
    'r.','LineWidth',2,'DisplayName','Target');

xlabel('^Ix (m)');
ylabel('^Iy (m)');
zlabel('^Iz (m)');
title('3D Trajectories');
legend('Location','eastoutside');
view(50,25);
axis equal;
% xlim([-10 10])
% ylim([-10 10])

end