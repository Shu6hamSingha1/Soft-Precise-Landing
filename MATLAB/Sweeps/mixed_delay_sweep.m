% MIXED_DELAY_SWEEP  Per-channel (lateral vs yaw) outer-delay sweep.
%
% Motivation (2026-06-03): the uniform outer-delay sweep (delay_robustness_sweep,
% DELAY_MODE=outer) showed manuscript gains hold 100% SP through 100 ms outer
% delay, while PX4's measured lag is ASYMMETRIC:
%     roll/pitch chain : 52-61 ms   (inside the manuscript envelope)
%     yaw chain        : 287 ms     (past the cliff)
% The current PX4-validated config detunes BOTH channels (lateral x0.35 +
% yaw 0.2) because it was tuned before the channels could be separated.
%
% Hypothesis: only the yaw detuning is necessary; the lateral detuning is
% over-conservative and costs precision. Test with the REAL per-channel lag
% profile applied in the clean simulator:
%
%   Arm A — manuscript gains everywhere       (prediction: yaw channel fails)
%   Arm B — current PX4-validated (lat x0.35 + yaw 0.2)
%   Arm C — manuscript lateral + detuned yaw  (prediction: best of both)
%
% Profiles (lat_ms, yaw_ms): (60, 290) PX4-matched; (100, 290) margin;
%                            (150, 290) stress.
% SP criterion: xy <= 0.10 m AND rel_vel <= 0.2 m/s.
%
% Usage:   cd MATLAB/Sweeps;  mixed_delay_sweep
% Output:  ../Datasets/Sweeps/DelayRobustness/mixed_delay_sweep_results.mat

clear; clc;
this_dir = fileparts(mfilename('fullpath'));
addpath(this_dir);
addpath(fullfile(this_dir, '..', 'Multi_init_cond'));  % run_simulation, InitVar
addpath(fullfile(this_dir, '..', 'Common'));
out_dir = fullfile(this_dir, '..', 'Datasets', 'Sweeps', 'DelayRobustness');
if ~exist(out_dir, 'dir'); mkdir(out_dir); end

% Per-channel lag profiles in steps (dt = 10 ms): [lat_steps, yaw_steps]
PROFILES   = [ 6, 29;      % PX4-matched   (60 ms lat, 290 ms yaw)
              10, 29;      % margin check (100 ms lat)
              15, 29];     % stress       (150 ms lat)
N_REPS     = 5;
TRAJ       = "Static";
SP_XY_TOL  = 0.10;
SP_VEL_TOL = 0.20;

% IC1 in the 13-state form run_simulation expects: [p(3); q(4); v(3); w(3)]
x0 = [0.0; 0.0; -5.0;  1; 0; 0; 0;  0; 0; 0;  0; 0; 0];

% ---- Arm definitions --------------------------------------------------------
% Arm B: current PX4-validated set (trial 46) — lateral x0.35 + yaw 0.2
armB = struct();
armB.rp      = diag([1.4,    1.4   ]);
armB.ri      = diag([0.35,   0.35  ]);
armB.rd      = diag([0.5031, 0.5031]);
armB.Omega_a = 0.2;
armB.Gamma_a = 0.2;

% Arm C: hypothesis — manuscript lateral, detuned yaw ONLY
armC = struct();
armC.Omega_a = 0.2;
armC.Gamma_a = 0.2;

arms = struct('name', {'A_manuscript', 'B_px4validated', 'C_lat_full_yaw_detuned'}, ...
              'K',    {struct(),       armB,              armC});

% ---- Sweep -------------------------------------------------------------------
rows = struct('arm', {}, 'lat_ms', {}, 'yaw_ms', {}, 'rep', {}, 'landed', {}, ...
              'xy', {}, 'vel', {}, 'tf', {}, 'fov_fail', {}, 'sp', {});
for a = 1:numel(arms)
    for p = 1:size(PROFILES, 1)
        d_lat = PROFILES(p, 1);
        d_yaw = PROFILES(p, 2);
        for rep = 1:N_REPS
            seed = 10000*d_lat + 100*d_yaw + rep;      % reproducible noise draw
            cfg  = struct('NOISE', 1, 'GE', 1, 'delay', 1, ...
                          'delay_outer_lat', d_lat, 'delay_outer_yaw', d_yaw);
            res  = run_simulation(x0, TRAJ, arms(a).K, 1.0, cfg, seed);
            sp   = res.success && (res.final_xy <= SP_XY_TOL) ...
                               && (res.final_rel_vel <= SP_VEL_TOL);
            rows(end+1) = struct( ...                  %#ok<SAGROW>
                'arm', arms(a).name, 'lat_ms', d_lat*10, 'yaw_ms', d_yaw*10, ...
                'rep', rep, 'landed', res.success, 'xy', res.final_xy, ...
                'vel', res.final_rel_vel, 'tf', res.final_t, ...
                'fov_fail', res.fov_fail, 'sp', sp);
            fprintf('%-24s lat=%3dms yaw=%3dms rep=%d:  landed=%d  xy=%.3f  vel=%.3f  %s\n', ...
                arms(a).name, d_lat*10, d_yaw*10, rep, res.success, ...
                res.final_xy, res.final_rel_vel, ...
                string(sp).replace("true","SP").replace("false",""));
        end
    end
end

results = rows;
save(fullfile(out_dir, 'mixed_delay_sweep_results.mat'), 'results', ...
     'PROFILES', 'N_REPS', 'SP_XY_TOL', 'SP_VEL_TOL');

% ---- Summary table ------------------------------------------------------------
fprintf('\n===== MIXED-PROFILE DELAY SWEEP — SP rate / median xy (n=%d, xy<=%.2f, vel<=%.2f) =====\n', ...
        N_REPS, SP_XY_TOL, SP_VEL_TOL);
fprintf('%-26s', 'profile (lat/yaw ms):');
for p = 1:size(PROFILES, 1)
    fprintf('%16s', sprintf('%d/%d', PROFILES(p,1)*10, PROFILES(p,2)*10));
end
fprintf('\n');
for a = 1:numel(arms)
    fprintf('%-26s', arms(a).name);
    for p = 1:size(PROFILES, 1)
        mask = strcmp({results.arm}, arms(a).name) ...
             & [results.lat_ms] == PROFILES(p,1)*10 ...
             & [results.yaw_ms] == PROFILES(p,2)*10;
        fprintf('%8.0f%% %6.3f', 100*mean([results(mask).sp]), ...
                median([results(mask).xy]));
    end
    fprintf('\n');
end
fprintf('\nResults saved to %s\n', fullfile(out_dir, 'mixed_delay_sweep_results.mat'));
