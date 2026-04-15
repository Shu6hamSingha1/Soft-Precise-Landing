%% =========================================================================
% run_comparison.m
%
% Batch runner: executes all 5 controllers sequentially using
% visualControl_comparison.m, saves each result to a per-controller
% .mat file, then calls plotter_comparison.m.
%
% USAGE:
%   run_comparison                       % all 5 controllers, Static target
%   run_comparison([1 3])                % only controllers 1 and 3, Static
%   run_comparison(1:5, "Circular")      % all 5, Circular target
%   run_comparison([1], "Sinusoidal")    % only PLASMC, Sinusoidal
%
% OUTPUTS:
%   result_ctrl_<c>.mat                       — per-controller (cwd, legacy)
%   Datasets/<trajType>_comparison.mat        — combined results for all
%                                                controllers in this run
% =========================================================================

function run_comparison(ctrl_list, trajType)

% Shared helpers live in ../Common (collapsed from per-folder duplicates)
addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'Common'));

if nargin < 1 || isempty(ctrl_list)
    ctrl_list = 1:5;
end
if nargin < 2 || isempty(trajType)
    trajType = "Static";
end
trajType = string(trajType);

ctrl_names = {'PLASMC (Proposed)', 'Lin 2022', ...
              'Zhang 2026',        'Chen 2025', 'Cho 2022'};

fprintf('=== Comparative Study: %d controllers on %s target ===\n\n', ...
        numel(ctrl_list), trajType);

% Pre-allocate combined results struct array (indexed by controller id 1-5)
all_results(5) = struct('ctrl_id', [], 'ctrl_name', '', 'data', []);

%% =========================================================================
%  RUN EACH CONTROLLER
% =========================================================================
for c = ctrl_list

    fprintf('--- Running Controller %d: %s ---\n', c, ctrl_names{c});

    % Full workspace reset between runs (preserve loop vars + trajType)
    clearvars -except c ctrl_list ctrl_names trajType all_results;
    rng('shuffle');

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
    if exist('p_1','var'),    result.p_1    = p_1;    else, result.p_1    = []; end
    if exist('p_2','var'),    result.p_2    = p_2;    else, result.p_2    = []; end
    if exist('S_1','var'),    result.S_1    = S_1;    else, result.S_1    = []; end
    if exist('zeta_1','var'), result.zeta_1 = zeta_1; else, result.zeta_1 = []; end

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
    fname = fullfile(fileparts(mfilename('fullpath')), 'Datasets', sprintf('result_ctrl_%d.mat', c));
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
datasetDir = fullfile(fileparts(mfilename('fullpath')), 'Datasets');
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
