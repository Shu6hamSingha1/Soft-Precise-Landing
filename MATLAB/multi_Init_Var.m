%% MULTI INITIAL CONDITION TEST
clc; clear;

trajType = "Circular";   % Change here

p0 = [
    0,0,-5;
    0,0,-7;
    0,0,-3;
    2,2,-5;
    -2,-2,-5
    ];

numRuns = size(p0,1);

results(numRuns) = struct( ...
    'success', false, ...
    'final_error', 0, ...
    'data', [] );

for k = 1:numRuns

    fprintf("\nRun %d/%d\n", k, numRuns);

    q0w = 1.0; q0x = 0.0; q0y = 0.0; q0z = 0.0;
    q0 = [q0w; q0x; q0y; q0z];
    q0 = q0 / norm(q0);

    v0 = zeros(3,1);
    w0 = zeros(3,1);

    x0 = [p0(k,:)'; q0; v0; w0];

    tmp = run_simulation(x0, trajType);
    results(k) = tmp;
end

plot_multi_3D(results);
plot_multi_image_plane(results);