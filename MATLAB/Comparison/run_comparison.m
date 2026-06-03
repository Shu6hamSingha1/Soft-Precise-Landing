%% =========================================================================
% run_comparison.m
%
% Batch runner: executes all 5 controllers sequentially using
% visualControl_comparison.m, saves each result to a per-controller
% .mat file, then calls plotter_comparison.m.
%
% USAGE:
%   run_comparison                              % all 5 controllers, Static, shuffled seed
%   run_comparison([1 3])                       % only controllers 1 and 3, Static
%   run_comparison(1:5, "Circular")             % all 5, Circular target
%   run_comparison([1], "Sinusoidal")           % only PLASMC, Sinusoidal
%   run_comparison(1:5, "Static", 1002)         % deterministic seed (matches Multi_init IC2)
%
% OUTPUTS:
%   result_ctrl_<c>.mat                       — per-controller (cwd, legacy)
%   Datasets/<trajType>_comparison.mat        — combined results for all
%                                                controllers in this run
% =========================================================================

function run_comparison(ctrl_list, trajType, seed)

% Shared helpers live in ../Common (collapsed from per-folder duplicates)
addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'Common'));

if nargin < 1 || isempty(ctrl_list)
    ctrl_list = 1:5;
end
if nargin < 2 || isempty(trajType)
    trajType = "Static";
end
if nargin < 3
    seed = [];
end
trajType = string(trajType);

% Hoist seed into MC_SEED — MC_SEED is preserved across visualControl_comparison's
% internal clearvars (line 42/45) and the inner-loop clearvars below, so it
% survives every iteration. A caller-facing `seed` arg would be wiped on
% iteration 2 because visualControl_comparison strips it from the workspace.
if ~isempty(seed)
    MC_SEED = seed;   %#ok<NASGU>  % consumed inside visualControl_comparison
end

ctrl_names = {'PLASMC (Proposed)', 'Lin 2022', ...
              'Zhang 2026',        'Lin 2023', 'Cho 2022'};

fprintf('=== Comparative Study: %d controllers on %s target ===\n\n', ...
        numel(ctrl_list), trajType);

% Pre-allocate combined results struct array (indexed by controller id 1-5)
all_results(5) = struct('ctrl_id', [], 'ctrl_name', '', 'data', []);

%% =========================================================================
%  RUN EACH CONTROLLER
% =========================================================================
for c = ctrl_list

    fprintf('--- Running Controller %d: %s ---\n', c, ctrl_names{c});

    % Full workspace reset between runs (preserve loop vars + trajType + MC_SEED).
    % MC_SEED is preserved when set; otherwise use shuffle for per-run variety.
    if exist('MC_SEED','var') == 1
        clearvars -except c ctrl_list ctrl_names trajType all_results MC_SEED SPEED_MULT;
    else
        clearvars -except c ctrl_list ctrl_names trajType all_results SPEED_MULT;
        rng('shuffle');
    end

    CTRL_SEL  = c;          %#ok<NASGU>
    TRAJ_TYPE = trajType;   %#ok<NASGU>
    visualControl_comparison;        % runs simulation, leaves workspace populated

    % ------------------------------------------------------------------
    % Pack results
    % ------------------------------------------------------------------
    result = struct();

    % UAV state
    result.X_DS   = X_DS;
    result.U_DS   = U_DS;

    % Target trajectory
    result.x_t    = x_t;
    result.dx_t   = dx_t;

    % Logged visual signal matrices (raw, before filtering)
    result.V_s_raw  = V_s_raw;
    result.V_h_raw  = V_h_raw;
    result.V_w_raw  = V_w_raw;
    result.V_dw_raw = V_dw_raw;

    % Full visual signal log  [24 x N]  (image + analytical both)
    result.V_X_DS = V_X_DS;

    % Desired optical flow and acceleration log
    result.V_h_d  = V_h_d;
    result.D_DS   = D_DS;

    % Desired image features
    result.V_s_d  = V_s_d;
    result.V_nP_d = V_nP_d;

    % Performance functions (PLASMC only; others get empty)
    % p_1/S_1/zeta_1 are legacy (pre-Approach 2) fields retained for backward
    % compatibility with older loaders; no longer populated by ctrl 1.
    if exist('p_1','var'),    result.p_1    = p_1;    else, result.p_1    = []; end
    if exist('p_2','var'),    result.p_2    = p_2;    else, result.p_2    = []; end
    if exist('S_1','var'),    result.S_1    = S_1;    else, result.S_1    = []; end
    if exist('zeta_1','var'), result.zeta_1 = zeta_1; else, result.zeta_1 = []; end

    % Approach 2 PLASMC logs (funnel-margin cone clamp + normalized outer PID)
    if exist('V_s_e_n','var'),        result.V_s_e_n        = V_s_e_n;        else, result.V_s_e_n        = []; end
    if exist('rho_fov_log','var'),    result.rho_fov_log    = rho_fov_log;    else, result.rho_fov_log    = []; end
    if exist('d_min_log','var'),      result.d_min_log      = d_min_log;      else, result.d_min_log      = []; end
    if exist('theta_cone_log','var'), result.theta_cone_log = theta_cone_log; else, result.theta_cone_log = []; end
    if exist('theta_cur_log','var'),  result.theta_cur_log  = theta_cur_log;  else, result.theta_cur_log  = []; end

    % Time bookkeeping
    result.tRange   = tRange;
    result.dt       = dt;
    result.idx      = idx;

    % Controller identity, gains, trajectory
    result.ctrl_id   = c;
    result.ctrl_name = ctrl_names{c};
    result.K         = K_ctrl;
    result.trajType  = trajType;

    % Per-controller .mat (stored in Datasets/ alongside trajectory results)
    fname = fullfile(fileparts(mfilename('fullpath')), '..', 'Datasets', 'Comparison', sprintf('result_ctrl_%d.mat', c));
    save(fname, '-struct', 'result');
    fprintf('    Saved %s  (%d steps)\n\n', fname, idx);

    % Add to combined struct array
    all_results(c).ctrl_id   = c;
    all_results(c).ctrl_name = ctrl_names{c};
    all_results(c).data      = result;

end

%% =========================================================================
%  SAVE COMBINED DATASET
% =========================================================================
datasetDir = fullfile(fileparts(mfilename('fullpath')), '..', 'Datasets', 'Comparison');
if ~exist(datasetDir, 'dir')
    mkdir(datasetDir);
end
saveFile = fullfile(datasetDir, sprintf('%s_comparison.mat', trajType));
save(saveFile, 'all_results', 'ctrl_list', 'ctrl_names', 'trajType');
fprintf('\nSaved combined results to %s\n\n', saveFile);

%% =========================================================================
%  PLOT ALL RESULTS TOGETHER
% =========================================================================
fprintf('=== Generating comparison plots ===\n');
plotter_comparison(ctrl_list);

end
