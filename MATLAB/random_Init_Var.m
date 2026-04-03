%% MULTI INITIAL CONDITION TEST
clc; clear;

numRuns = 1;
trajType = "Static";   % Change here


results(numRuns) = struct( ...
    'success', false, ...
    'final_error', 0, ...
    'data', [] );

for k = 1:numRuns

    fprintf("\nRun %d/%d\n", k, numRuns);

    % Random initial position
    p0 = [ ...
        2*(rand-0.5);    % x in [-1,1]
        2*(rand-0.5);    % y in [-1,1]
        -1 - 4*rand];       % z in [-2,-3]

    % Random small attitude
    phi0   = deg2rad(5*(rand-0.5));
    theta0 = deg2rad(5*(rand-0.5));
    psi0   = deg2rad(10*(rand-0.5));

    q0 = eul2quat([psi0 theta0 phi0], 'XYZ')';
    q0 = q0 / norm(q0);

    v0 = zeros(3,1);
    w0 = zeros(3,1);

    x0 = [p0; q0; v0; w0];

    tmp = run_simulation(x0, trajType);
    results(k) = tmp;
end

successCount = sum([results.success]);
successRate = successCount/numRuns*100;

fprintf("\nSuccess Rate: %.2f %%\n", successRate);

finalErrors = [results.final_error];
fprintf("Mean Final Error: %.4f m\n", mean(finalErrors));
fprintf("Max Final Error: %.4f m\n", max(finalErrors));

plot_monte_carlo(results);