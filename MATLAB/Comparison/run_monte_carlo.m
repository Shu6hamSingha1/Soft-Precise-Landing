%% run_monte_carlo.m
% Monte Carlo wrapper: 5 trajectories x 5 controllers x N seeds
% Produces Datasets/monte_carlo.mat with per-run summary stats so the
% main-paper comparison table can carry std-dev columns.
%
% USAGE:
%   run_monte_carlo            % default: 5 seeds per (traj,ctrl)
%   run_monte_carlo(10)        % custom seed count

function run_monte_carlo(n_seeds)

addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'Common'));

if nargin < 1 || isempty(n_seeds), n_seeds = 5; end

trajList = ["Static", "Linear", "Sinusoidal", "Lissajous", "Circular"];
ctrlList = 1:5;
ctrl_names = {'PLASMC', 'Lin2022', 'Zhang2026', 'Lin2023', 'Cho2022'};

nT = numel(trajList); nC = numel(ctrlList); nS = n_seeds;

summary = struct();
summary.trajList   = trajList;
summary.ctrl_names = ctrl_names;
summary.n_seeds    = nS;
summary.p_xy_end   = nan(nT, nC, nS);
summary.v_rel_end  = nan(nT, nC, nS);
summary.t_land     = nan(nT, nC, nS);
summary.idx_end    = nan(nT, nC, nS);
summary.seeds      = nan(nT, nC, nS);

total = nT * nC * nS; run = 0; t0_all = tic;

for ti = 1:nT
    for ci = 1:nC
        for si = 1:nS
            run = run + 1;
            seed = 1000 * ti + 100 * ci + si;

            fprintf('[%3d/%3d] %-10s ctrl=%d seed=%d ... ', ...
                    run, total, trajList(ti), ctrlList(ci), seed);

            MC_SEED   = seed;         %#ok<NASGU>
            CTRL_SEL  = ctrlList(ci); %#ok<NASGU>
            TRAJ_TYPE = trajList(ti); %#ok<NASGU>

            t0 = tic;
            try
                evalin('base', 'clear');
                assignin('base', 'MC_SEED',   seed);
                assignin('base', 'CTRL_SEL',  ctrlList(ci));
                assignin('base', 'TRAJ_TYPE', trajList(ti));
                evalin('base', 'visualControl_comparison');

                X_DS_b  = evalin('base', 'X_DS');
                x_t_b   = evalin('base', 'x_t');
                idx_b   = evalin('base', 'idx');
                tRange_b= evalin('base', 'tRange');

                k = idx_b;
                p_xy = norm(X_DS_b(1:2, k) - x_t_b(1:2, k));
                v_rel = norm(X_DS_b(4:6, k) - x_t_b(4:6, k));
                t_land = tRange_b(k);

                summary.p_xy_end(ti,ci,si)  = p_xy;
                summary.v_rel_end(ti,ci,si) = v_rel;
                summary.t_land(ti,ci,si)    = t_land;
                summary.idx_end(ti,ci,si)   = idx_b;
                summary.seeds(ti,ci,si)     = seed;

                fprintf('p_xy=%.3f v=%.3f t=%.2f (%.1fs)\n', ...
                        p_xy, v_rel, t_land, toc(t0));
            catch ME
                fprintf('FAIL: %s\n', ME.message);
            end
        end
    end

    % Checkpoint save after each trajectory
    save(fullfile(fileparts(mfilename('fullpath')), '..', 'Datasets', 'Comparison', 'monte_carlo.mat'), ...
         '-struct', 'summary');
end

fprintf('\n=== Monte Carlo complete in %.1f min ===\n', toc(t0_all)/60);

% Print summary table
fprintf('\nMean p_xy (m):\n');
fprintf('  %-12s', 'traj\ctrl'); for ci=1:nC, fprintf('%10s', ctrl_names{ci}); end; fprintf('\n');
for ti = 1:nT
    fprintf('  %-12s', trajList(ti));
    for ci = 1:nC
        fprintf('%10.4f', mean(summary.p_xy_end(ti,ci,:), 'omitnan'));
    end
    fprintf('\n');
end

end
