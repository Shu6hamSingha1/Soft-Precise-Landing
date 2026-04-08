function plot_success(results)

successVec = [results.success];

figure;
stem(successVec,'filled');
ylim([-0.2 1.2]);
xlabel('Run Index');
ylabel('Success (1/0)');
title('Monte Carlo Success per Run');
grid on;

end