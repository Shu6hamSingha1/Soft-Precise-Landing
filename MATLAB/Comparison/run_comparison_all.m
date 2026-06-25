%% run_comparison_all.m
% Batch driver: runs all 5 controllers on all 5 trajectories with a fixed
% deterministic seed. Comparison uses InitVar's default IC = [2,2,-5] (= Multi_init
% IC2), and seed 1 now matches the current Multi_init/Multi_speed convention
% (seed 1, every IC). The retired 1000+IC convention drew unlucky per-cell outliers.

% Proposed controller (case 1) uses per-axis regressor-norm theta (current
% formulation), identical to multi_Init_Var/multi_speed_cond. Global read by
% vdf_params; survives visualControl_comparison's clearvars (like CMP_OVERRIDE/MC_SEED).
global VDF_OVERRIDE       %#ok<GVMIS>
VDF_OVERRIDE.theta_per_axis = true;

seed = 1;
trajList = ["Static", "Linear", "Sinusoidal", "Lissajous", "Circular"];

for k = 1:numel(trajList)
    fprintf('\n########## Trajectory: %s (seed=%d) ##########\n\n', trajList(k), seed);
    run_comparison(1:5, trajList(k), seed);
end

fprintf('\n========== All 5x5 runs complete ==========\n');
