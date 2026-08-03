%% STRESS ENVELOPE -- current formulation (per-axis theta ON) vs noise x target speed
%
% Maps where the CURRENT baked VDF-ASMC formulation (per-axis regressor-norm theta
% ON) breaks down as the disturbance magnitude and the target speed are pushed past
% nominal. Single baseline IC [0,0,-5] (the multi_speed convention), realistic config
% (NOISE/GE/delay all on), 4 moving trajectories, 5 seeds per cell.
%
%   noise scale : 1x 2x 3x 4x   (NOISE_SCALE multiplies every disturbance amplitude:
%                                 parametric uncertainty, wind, pixel sigma + outlier mag)
%   speed mult  : 1.0 1.5 2.0 2.5 3.0   (traj_Gen target-speed multiplier)
%   trajectories: Linear, Sinusoidal, Lissajous, Circular   (Static has no speed knob)
%   seeds       : 1..5  (soft-precise RATE per cell; single-seed is unreliable)
%
% NO clc/clear -- that would wipe the globals. Calls run_simulation directly.
% Saves Datasets/MultiInit/stress_envelope_perAxis.mat after EACH trajectory.
close all;

mfile_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(mfile_dir, '..', 'Common'));
addpath(fullfile(mfile_dir, '..', 'VDF_ASMC'));

global VDF_OVERRIDE NOISE_SCALE       %#ok<GVMIS>
VDF_OVERRIDE = struct('theta_per_axis', true);   % CURRENT config: per-axis theta ON

trajList    = ["Linear", "Sinusoidal", "Lissajous", "Circular"];
noiseScales = [1 2 3 4];
speedMults  = [1.0 1.5 2.0 2.5 3.0];
seeds       = 1:5;
cfg = struct('NOISE', 1, 'GE', 1, 'delay', 1);   % realistic

q0 = [1;0;0;0]; v0 = zeros(3,1); w0 = zeros(3,1);
x0 = [0;0;-5; q0; v0; w0];

nT = numel(trajList); nN = numel(noiseScales); nS = numel(speedMults); nSeed = numel(seeds);
SP  = zeros(nT, nN, nS);   % soft-precise count (0..5)
SUC = zeros(nT, nN, nS);   % success count
FOV = zeros(nT, nN, nS);   % FoV-breach count
XY  = zeros(nT, nN, nS);   % mean final_xy over seeds
VV  = zeros(nT, nN, nS);   % mean final_rel_vel over seeds

resDir = fullfile(mfile_dir, '..', 'Datasets', 'MultiInit');
if ~exist(resDir, 'dir'), mkdir(resDir); end

fprintf('STRESS ENVELOPE (per-axis ON): %d traj x %d noise x %d speed x %d seeds = %d runs\n\n', ...
        nT, nN, nS, nSeed, nT*nN*nS*nSeed);

for ti = 1:nT
    traj = trajList(ti);
    for ni = 1:nN
        NOISE_SCALE = noiseScales(ni);
        for si = 1:nS
            sm = speedMults(si);
            nsp = 0; nsuc = 0; nfov = 0; sxy = 0; svv = 0;
            for se = 1:nSeed
                r = run_simulation(x0, traj, [], sm, cfg, seeds(se));
                nsp  = nsp  + (r.success && r.precise && r.soft);
                nsuc = nsuc + r.success;
                nfov = nfov + r.fov_fail;
                sxy  = sxy  + r.final_xy;
                svv  = svv  + r.final_rel_vel;
            end
            SP(ti,ni,si)=nsp; SUC(ti,ni,si)=nsuc; FOV(ti,ni,si)=nfov;
            XY(ti,ni,si)=sxy/nSeed; VV(ti,ni,si)=svv/nSeed;
            fprintf('%-11s noise %dx speed %.1fx: SP %d/5  succ %d/5  fov %d/5  xy %.3f v %.3f\n', ...
                    traj, noiseScales(ni), sm, nsp, nsuc, nfov, sxy/nSeed, svv/nSeed);
        end
    end
    fprintf('\n--- %s : soft-precise/5  (rows = noise %s, cols = speed %s) ---\n', ...
            traj, mat2str(noiseScales), mat2str(speedMults));
    disp(squeeze(SP(ti,:,:)));
    save(fullfile(resDir, 'stress_envelope_perAxis.mat'), ...
         'SP','SUC','FOV','XY','VV','trajList','noiseScales','speedMults','seeds');
    fprintf('(saved partial through %s)\n\n', traj);
end

%% Envelope summary: highest speed sustaining SP>=4/5 at each noise level
fprintf('\n================ ENVELOPE SUMMARY (max speed with SP>=4/5) ================\n');
fprintf('%-11s', 'traj\\noise');
for ni = 1:nN, fprintf('  %dx ', noiseScales(ni)); end
fprintf('\n');
for ti = 1:nT
    fprintf('%-11s', trajList(ti));
    for ni = 1:nN
        ok = find(squeeze(SP(ti,ni,:)) >= 4);
        if isempty(ok), fprintf('  --  '); else, fprintf(' %.1fx', speedMults(max(ok))); end
    end
    fprintf('\n');
end
fprintf('(-- = fails SP>=4/5 even at nominal speed for that noise level)\n');

VDF_OVERRIDE = []; NOISE_SCALE = [];   % restore defaults
