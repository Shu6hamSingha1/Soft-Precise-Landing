% DELAY_ROBUSTNESS_SWEEP  Delay-robustness envelope of the VDF-ASMC controller.
%
% Motivation (2026-06-03): the manuscript's realistic config models a 1-step
% (10 ms) computational delay, which the controller handles at 10/10 SP.
% PX4 SITL presents much larger delays (52-61 ms roll/pitch rate chain,
% 287 ms yaw chain, ~50-100 ms perception latency). This sweep maps the
% delay-robustness envelope in the clean simulator for TWO gain sets:
%
%   Arm A — manuscript gains (the locked Table-S set, as in run_simulation.m)
%     Prediction: SP holds at small delays, degrades as delay approaches the
%     PX4 magnitudes (i.e. the PX4 failures are the EXPECTED delay response,
%     not implementation defects).
%
%   Arm B — PX4-validated gains (parameter_record.ods, PX4_Gain_Record trial 46)
%     K_rp=1.4, K_ri=0.35, K_rd=0.5031 (per axis), yaw chi/gamma = 0.2.
%     Prediction: SP holds out to 150-300 ms (bandwidth-matched set).
%
% Delay grid: {1,3,5,10,15,30} steps @ dt=10 ms = {10,30,50,100,150,300} ms.
% Static target (matches the PX4 stationary-ArUco case), IC1 = [0,0,-5] NED.
% SP criterion: xy <= 0.10 m AND rel_vel <= 0.2 m/s (10 cm per user 2026-06-03).
%
% Usage:   cd MATLAB/Multi_init_cond;  delay_robustness_sweep
% Output:  Datasets/DelayRobustness/delay_sweep_results.mat + console table.

clear; clc;
mfile_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(mfile_dir, '..', 'Common'));
out_dir = fullfile(mfile_dir, 'Datasets', 'DelayRobustness');
if ~exist(out_dir, 'dir'); mkdir(out_dir); end

DELAY_STEPS = [1, 3, 5, 10, 15, 30];      % x10 ms each
N_REPS      = 5;                          % reps per cell (seeded -> reproducible)
TRAJ        = "Static";                   % matches PX4 stationary ArUco case
SP_XY_TOL   = 0.10;                       % m  (10 cm criterion, user 2026-06-03)
SP_VEL_TOL  = 0.20;                       % m/s

% IC1 in the 13-state form run_simulation expects: [p(3); q(4); v(3); w(3)]
x0 = [0.0; 0.0; -5.0;  1; 0; 0; 0;  0; 0; 0;  0; 0; 0];

% ---- Arm definitions --------------------------------------------------------
% Arm A: empty K_override -> run_simulation's built-in manuscript K_ctrl.
% Arm B: PX4-validated set (trial 46). Field names match K_ctrl fields;
%        run_simulation splats them via K_ctrl.(field) = value.
armB = struct();
armB.rp      = diag([1.4,    1.4   ]);
armB.ri      = diag([0.35,   0.35  ]);
armB.rd      = diag([0.5031, 0.5031]);
armB.Omega_a = 0.2;                       % chi_alpha  (yaw integrator gain)
armB.Gamma_a = 0.2;                       % gamma_alpha (yaw proportional gain)
% Funnel / ASMC / kappa / k_R identical to manuscript in trial 46.
% (The p1-envelope resize and perception knobs have no MATLAB meaning.)

arms = struct('name', {'A_manuscript', 'B_px4validated'}, ...
              'K',    {struct(),       armB});

% ---- Sweep -------------------------------------------------------------------
rows = struct('arm', {}, 'delay_ms', {}, 'rep', {}, 'landed', {}, ...
              'xy', {}, 'vel', {}, 'tf', {}, 'fov_fail', {}, 'sp', {});
for a = 1:numel(arms)
    for d = DELAY_STEPS
        for rep = 1:N_REPS
            seed = 1000*d + rep;                       % reproducible noise draw
            cfg  = struct('NOISE', 1, 'GE', 1, 'delay', d);
            res  = run_simulation(x0, TRAJ, arms(a).K, 1.0, cfg, seed);
            sp   = res.success && (res.final_xy <= SP_XY_TOL) ...
                               && (res.final_rel_vel <= SP_VEL_TOL);
            rows(end+1) = struct( ...                  %#ok<SAGROW>
                'arm', arms(a).name, 'delay_ms', d*10, 'rep', rep, ...
                'landed', res.success, 'xy', res.final_xy, ...
                'vel', res.final_rel_vel, 'tf', res.final_t, ...
                'fov_fail', res.fov_fail, 'sp', sp);
            fprintf('%s  delay=%3dms  rep=%d:  landed=%d  xy=%.3f  vel=%.3f  %s\n', ...
                arms(a).name, d*10, rep, res.success, res.final_xy, ...
                res.final_rel_vel, string(sp).replace("true","SP").replace("false",""));
        end
    end
end

results = rows;   % (result.data workspaces intentionally not kept — scalars only)
save(fullfile(out_dir, 'delay_sweep_results.mat'), 'results', ...
     'DELAY_STEPS', 'N_REPS', 'SP_XY_TOL', 'SP_VEL_TOL');

% ---- Summary table ------------------------------------------------------------
fprintf('\n===== DELAY-ROBUSTNESS ENVELOPE — SP rate per cell (n=%d, xy<=%.2f, vel<=%.2f) =====\n', ...
        N_REPS, SP_XY_TOL, SP_VEL_TOL);
fprintf('%-18s', 'delay (ms):');
for d = DELAY_STEPS; fprintf('%8d', d*10); end; fprintf('\n');
for a = 1:numel(arms)
    fprintf('%-18s', arms(a).name);
    for d = DELAY_STEPS
        mask = strcmp({results.arm}, arms(a).name) & [results.delay_ms] == d*10;
        fprintf('%7.0f%%', 100*mean([results(mask).sp]));
    end
    fprintf('\n');
end
fprintf('\nResults saved to %s\n', fullfile(out_dir, 'delay_sweep_results.mat'));
