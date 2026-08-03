%% SEARCH: max (noise, base-speed) holding 25/25 SP across a +-40% speed sweep
%
% Current formulation (per-axis theta ON). Per trajectory, for each operating point
% (base_speed, noise_scale) we run the +-40% speed sweep
%   speeds = base * [0.6 0.8 1.0 1.2 1.4]   (5 points)
% over seeds 1..5  => 25 runs => SP/25. Goal: find the HIGHEST (base, noise) at which
% a trajectory still scores 25/25. Coarse probe; climb in a follow-up if an edge holds.
%
% NO clc/clear (preserves globals). Saves after each trajectory. The NOISE_SCALE knob
% is the SEARCH tool only -- the winning scale gets HARD-CODED into init_robustness.m
% (and the base speed into the trajectory) afterwards; the knob is then removed.
close all;

mfile_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(mfile_dir, '..', 'Common'));
addpath(fullfile(mfile_dir, '..', 'VDF_ASMC'));

global VDF_OVERRIDE NOISE_SCALE       %#ok<GVMIS>
VDF_OVERRIDE = struct('theta_per_axis', true);   % CURRENT config: per-axis theta ON

trajList    = ["Linear", "Sinusoidal", "Lissajous", "Circular"];
baseSpeeds  = [1.0 1.2 1.4];
noiseScales = [1 2];
sweepFrac   = [0.6 0.8 1.0 1.2 1.4];     % +-40% band around the base speed
seeds       = 1:5;
cfg = struct('NOISE', 1, 'GE', 1, 'delay', 1);
x0 = [0;0;-5; 1;0;0;0; zeros(6,1)];

nT = numel(trajList); nB = numel(baseSpeeds); nN = numel(noiseScales);
SP25 = zeros(nT, nB, nN);                 % soft-precise out of 25

resDir = fullfile(mfile_dir, '..', 'Datasets', 'MultiInit');
if ~exist(resDir, 'dir'), mkdir(resDir); end

fprintf('SEARCH 25/25 x +-40%% sweep (per-axis ON): %d traj x %d base x %d noise x 25 = %d runs\n\n', ...
        nT, nB, nN, nT*nB*nN*25);

for ti = 1:nT
    traj = trajList(ti);
    for bi = 1:nB
        base = baseSpeeds(bi);
        for ni = 1:nN
            NOISE_SCALE = noiseScales(ni);
            nsp = 0; perPt = zeros(1, numel(sweepFrac));
            for pj = 1:numel(sweepFrac)
                sm = base * sweepFrac(pj);
                pt = 0;
                for se = seeds
                    r = run_simulation(x0, traj, [], sm, cfg, se);
                    pt = pt + (r.success && r.precise && r.soft);
                end
                perPt(pj) = pt; nsp = nsp + pt;
            end
            SP25(ti,bi,ni) = nsp;
            tag = ''; if nsp == 25, tag = '  <== 25/25'; end
            fprintf('%-11s base %.1fx noise %dx: %2d/25 SP  [per-speed %s of 5]%s\n', ...
                    traj, base, noiseScales(ni), nsp, mat2str(perPt), tag);
        end
    end
    fprintf('\n--- %s: SP/25  (rows = base %s, cols = noise %s) ---\n', ...
            traj, mat2str(baseSpeeds), mat2str(noiseScales));
    disp(squeeze(SP25(ti,:,:)));
    save(fullfile(resDir, 'search_2540_perAxis.mat'), ...
         'SP25','trajList','baseSpeeds','noiseScales','sweepFrac','seeds');
    fprintf('(saved partial through %s)\n\n', traj);
end

%% Per-trajectory max operating point with 25/25
fprintf('\n================ MAX 25/25 OPERATING POINT (per trajectory) ================\n');
for ti = 1:nT
    G = squeeze(SP25(ti,:,:));           % nB x nN
    [bi, ni] = find(G == 25);
    if isempty(bi)
        fprintf('%-11s : no 25/25 cell in the probed grid (best %d/25)\n', trajList(ti), max(G(:)));
    else
        % prefer the highest base, then highest noise
        score = baseSpeeds(bi(:)).' * 10 + noiseScales(ni(:)).';
        [~, k] = max(score);
        fprintf('%-11s : 25/25 at base %.1fx, noise %dx  (sweep %.2f-%.2f x)\n', ...
                trajList(ti), baseSpeeds(bi(k)), noiseScales(ni(k)), ...
                baseSpeeds(bi(k))*0.6, baseSpeeds(bi(k))*1.4);
    end
end

VDF_OVERRIDE = []; NOISE_SCALE = [];     % restore defaults
