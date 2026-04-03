function plot_error_hist(results)

finalErrors = [results.final_error];

figure;
histogram(finalErrors,10);
xlabel('Final Landing Error (m)');
ylabel('Frequency');
title('Monte Carlo Landing Error Distribution');
grid on;

end