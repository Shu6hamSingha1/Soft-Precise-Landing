%% =========================================================================
% run_comparison.m
%
% Batch runner: executes all 5 controllers sequentially using
% visualControl_comparison.m, saves each result to a per-controller
% .mat file, then calls plotter_comparison.m.
%
% USAGE:
%   run_comparison          % runs controllers 1-5
%   run_comparison([1 3])   % runs only controllers 1 and 3
% =========================================================================

function run_comparison(ctrl_list)

if nargin < 1
    ctrl_list = 1:5;
end

ctrl_names = {'PLASMC (Proposed)', 'Lin 2022', ...
              'Zhang 2026',        'Chen 2025', 'Cho 2022'};

fprintf('=== Comparative Study: %d controllers ===\n\n', numel(ctrl_list));

%% =========================================================================
%  RUN EACH CONTROLLER
% =========================================================================
for c = ctrl_list

    fprintf('--- Running Controller %d: %s ---\n', c, ctrl_names{c});

    % Full workspace reset between runs
    clearvars -except c ctrl_list ctrl_names;
    rng('shuffle');

    CTRL_SEL = c;                    %#ok<NASGU>
    visualControl_comparison;        % runs simulation, leaves workspace populated

    % ------------------------------------------------------------------
    % Pack results — variable list updated to match _temp.m structure:
    %   V_s / V_h / V_w are column vectors (not 2-D logged arrays)
    %   V_X_DS is [24 x N]
    %   P_DS   is [2 x 12 x N]
    %   V_s_raw / V_h_raw etc. are [rows x N] logged matrices
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
    result.tRange = tRange;
    result.dt     = dt;
    result.idx    = idx;

    % Controller identity and gains
    result.ctrl_id   = c;
    result.ctrl_name = ctrl_names{c};
    result.K         = K_ctrl;

    fname = sprintf('result_ctrl_%d.mat', c);
    save(fname, '-struct', 'result');
    fprintf('    Saved %s  (%d steps)\n\n', fname, idx);

end

%% =========================================================================
%  PLOT ALL RESULTS TOGETHER
% =========================================================================
fprintf('=== Generating comparison plots ===\n');
plotter_comparison(ctrl_list);

end
