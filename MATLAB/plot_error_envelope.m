function plot_error_envelope(results)

numRuns = length(results);

% Find shortest simulation length
minLen = inf;
for k = 1:numRuns
    minLen = min(minLen, size(results(k).data.X_DS,2));
end

errors = zeros(numRuns, minLen);

for k = 1:numRuns
    
    X = results(k).data.X_DS;
    I_p_c = X(1:3,1:minLen);
    
    % assuming target at origin
    err = vecnorm(I_p_c,2,1);
    
    errors(k,:) = err;
end

meanErr = mean(errors,1);
stdErr  = std(errors,0,1);

t = 0:(minLen-1);

figure;
hold on; grid on;

plot(t, meanErr, 'b','LineWidth',2);
plot(t, meanErr + stdErr, '--k');
plot(t, meanErr - stdErr, '--k');

xlabel('Time Index');
ylabel('Position Error (m)');
title('Monte Carlo Error Envelope');
legend('Mean','Mean ± Std');

end