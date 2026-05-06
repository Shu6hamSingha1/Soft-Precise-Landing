%% run_comparison_all.m
% Batch driver: runs all 5 controllers on all 5 trajectories with a fixed
% deterministic seed so numbers are reproducible and IC2-aligned with
% Multi_init's 5-IC grid (seed = 1000 + IC_idx).
%
% Default seed 1002 matches Multi_init IC2 = [2,2,-5].

seed = 1002;
trajList = ["Static", "Linear", "Sinusoidal", "Lissajous", "Circular"];

for k = 1:numel(trajList)
    fprintf('\n########## Trajectory: %s (seed=%d) ##########\n\n', trajList(k), seed);
    run_comparison(1:5, trajList(k), seed);
end

fprintf('\n========== All 5x5 runs complete ==========\n');
