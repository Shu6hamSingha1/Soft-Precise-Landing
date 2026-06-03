%% MULTI-SPEED COMPARISON DRIVER
%
% Runs the four baseline controllers (Lin 2022, Zhang 2026, Lin 2023,
% Cho 2022) on every moving trajectory at every speed multiplier, under
% the shared disturbance model.
%
%   trajList = ["Linear", "Sinusoidal", "Lissajous", "Circular"]
%   mults    = [0.6, 0.8, 1.0, 1.2, 1.4]
%   ctrls    = [2 3 4 5]
%
% Output: MATLAB/Datasets/Comparison/<traj>_multi_speed_comparison.mat
%
% IMPLEMENTATION NOTE: visualControl_comparison.m executes a clearvars
% that wipes any workspace variables not in its preservation list.
% Therefore ALL driver state must live in MS_STATE (in the preservation
% list) — local variables read AFTER visualControl_comparison vanish.

clc; clear;

mfile_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(mfile_dir, '..', 'Common'));

MS_STATE.trajList        = ["Linear", "Sinusoidal", "Lissajous", "Circular"];
MS_STATE.mults           = [0.6, 0.8, 1.0, 1.2, 1.4];
MS_STATE.ctrls           = [2 3 4 5];
MS_STATE.ctrl_names_full = {'PLASMC (Proposed)', 'Lin 2022', 'Zhang 2026', ...
                            'Lin 2023',          'Cho 2022'};
MS_STATE.datasetDir      = fullfile(mfile_dir, '..', 'Datasets', 'Comparison');
MS_STATE.varsToCapture   = {'X_DS','P_DS','x_t','dx_t','tRange','idx', ...
                            'A_z','BT_u','I_a_d','sigma','kappa', ...
                            'sigma_a','kappa_a','rho_fov','S_2', ...
                            'p_fov_xy_max','attitude_cone_violations', ...
                            'rho_p','rho_v','xi_p','xi_v', ...
                            'final_xy','final_rel_vel','final_alt'};

if ~exist(MS_STATE.datasetDir, 'dir')
    mkdir(MS_STATE.datasetDir);
end

for tIdx = 1:numel(MS_STATE.trajList)
    MS_STATE.tIdx     = tIdx;
    MS_STATE.trajType = char(MS_STATE.trajList(tIdx));

    fprintf('\n############################################################\n');
    fprintf('### Multi-speed comparison: %s\n', MS_STATE.trajType);
    fprintf('############################################################\n');

    MS_STATE.nRuns = numel(MS_STATE.ctrls) * numel(MS_STATE.mults);
    MS_STATE.results(MS_STATE.nRuns) = struct('ctrl_id', [], 'ctrl_name', '', ...
                                              'mult', [], 'data', []);
    MS_STATE.rIdx = 0;

    for cIdx = 1:numel(MS_STATE.ctrls)
        % cIdx gets wiped by visualControl_comparison mid-iteration; capture
        % it ONCE per outer iter (before the inner loop) so it survives via
        % MS_STATE for the rest of this controller's runs.
        MS_STATE.cIdx = cIdx;
        for mIdx = 1:numel(MS_STATE.mults)
            MS_STATE.mIdx = mIdx;
            MS_STATE.rIdx = MS_STATE.rIdx + 1;

            % Hoist parameters that visualControl_comparison consumes/preserves
            CTRL_SEL    = MS_STATE.ctrls(MS_STATE.cIdx);  %#ok<NASGU>
            TRAJ_TYPE   = MS_STATE.trajType;              %#ok<NASGU>
            MC_SEED     = 1000;                           %#ok<NASGU>
            SPEED_MULT  = MS_STATE.mults(MS_STATE.mIdx);  %#ok<NASGU>
            IC_OVERRIDE = [0; 0; -5];                     %#ok<NASGU>  % IC_1 from proposed-controller speed sweep

            fprintf('\n--- %s | %s | lambda=%.2f (run %d/%d) ---\n', ...
                    MS_STATE.trajType, ...
                    MS_STATE.ctrl_names_full{MS_STATE.ctrls(MS_STATE.cIdx)}, ...
                    MS_STATE.mults(MS_STATE.mIdx), ...
                    MS_STATE.rIdx, MS_STATE.nRuns);

            visualControl_comparison;

            % After visualControl_comparison's clearvars, the only driver
            % state surviving is MS_STATE itself; pull everything from it.
            run_data = struct();
            for k = 1:numel(MS_STATE.varsToCapture)
                vn = MS_STATE.varsToCapture{k};
                if exist(vn, 'var') == 1
                    run_data.(vn) = eval(vn);
                end
            end

            ctrl_id_now  = MS_STATE.ctrls(MS_STATE.cIdx);
            mult_now     = MS_STATE.mults(MS_STATE.mIdx);
            MS_STATE.results(MS_STATE.rIdx).ctrl_id   = ctrl_id_now;
            MS_STATE.results(MS_STATE.rIdx).ctrl_name = MS_STATE.ctrl_names_full{ctrl_id_now};
            MS_STATE.results(MS_STATE.rIdx).mult      = mult_now;
            MS_STATE.results(MS_STATE.rIdx).data      = run_data;
        end
    end

    % Save per-trajectory; pull from MS_STATE
    results  = MS_STATE.results;             %#ok<NASGU>
    mults    = MS_STATE.mults;               %#ok<NASGU>
    ctrls    = MS_STATE.ctrls;               %#ok<NASGU>
    trajType = MS_STATE.trajType;            %#ok<NASGU>
    saveFile = fullfile(MS_STATE.datasetDir, ...
        sprintf('%s_multi_speed_comparison.mat', MS_STATE.trajType));
    save(saveFile, 'results', 'mults', 'ctrls', 'trajType', '-v7');
    fprintf('\nSaved %d runs to %s\n', numel(results), saveFile);

    MS_STATE = rmfield(MS_STATE, 'results');
end

fprintf('\n========== Multi-speed comparison sweep complete ==========\n');
fprintf('To plot, run from repo root:\n');
fprintf('    python scripts/make_comparison_multi_speed_plots.py\n');
