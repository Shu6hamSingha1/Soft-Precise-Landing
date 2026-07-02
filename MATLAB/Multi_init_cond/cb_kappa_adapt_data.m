%% CB_KAPPA_ADAPT_DATA  Generate kappa-vs-disturbance-magnitude data for the manuscript
%   "genuine adaptation" panel. For each disturbance level, aggregate per-axis kappa
%   (RMS over flight + peak) across cells. Monotone rise = kappa adapts to disturbance.
%   LOCKED is baked -> bare run. Saves Datasets/MultiInit/kappa_adapt.mat.
%
% Run:  cd MATLAB/Multi_init_cond; cb_kappa_adapt_data
clc; clear;
mfile_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(mfile_dir, '..', 'Common'));
clear mfile_dir;
global STRESS_SCALE KNOWN_DIST %#ok<GVMIS>
KNOWN_DIST = [];

% disturbance ladder: 0=noiseless, then realistic at scale 1,3,5,7
levels = [0 1 3 5 7];                       % 0 -> NOISE=0; else NOISE=1, STRESS_SCALE=level
p0 = [ 0,0,-5; 2.0,2.0,-5; 2.0,-2.0,-5; 2.0,2.0,-7; 2.0,2.0,-3 ];
cells = {};
for tr = ["Sinusoidal","Circular"]
    for ic = 1:5, cells(end+1,:) = {tr, ic}; end %#ok<SAGROW>
end

krms = zeros(3, numel(levels), size(cells,1));   % [axis x level x cell]
kpk  = zeros(3, numel(levels), size(cells,1));
for li = 1:numel(levels)
    L = levels(li);
    for c = 1:size(cells,1)
        if L==0, NZ=0; STRESS_SCALE=[]; else, NZ=1; STRESS_SCALE=L; end
        r = run_simulation([p0(cells{c,2},:)';1;0;0;0;zeros(6,1)], cells{c,1}, [], 1.0, ...
                           struct('NOISE',NZ,'GE',1,'delay',1), 1);
        d=r.data; ka=d.kappa_log(:,1:d.idx);
        krms(:,li,c)=sqrt(mean(ka.^2,2)); kpk(:,li,c)=max(ka,[],2);
    end
    fprintf('level %d done\n', L);
end
krms_mean=mean(krms,3); krms_std=std(krms,0,3);
kpk_mean=mean(kpk,3);   kpk_std=std(kpk,0,3);
save('../Datasets/MultiInit/kappa_adapt.mat','levels','krms_mean','krms_std','kpk_mean','kpk_std');
fprintf('\n=== kappa RMS (mean over %d cells) vs disturbance level ===\n', size(cells,1));
fprintf('  level | kRMS x / y / z\n');
for li=1:numel(levels)
    fprintf('   %d    | %.3f %.3f %.3f\n', levels(li), krms_mean(:,li));
end
clear global STRESS_SCALE KNOWN_DIST
