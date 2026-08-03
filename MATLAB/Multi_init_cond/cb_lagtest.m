%% CB_LAGTEST  Is the high-Gamma divergence PERCEPTION-LAG-limited?
%   Ablate the image ZOH (30Hz hold = 3-step lag) at Gamma=2.0 (diverges) -> does
%   faster feedback (ZOH=1, 100Hz) let the reaching drive sigma->0 and LAND?
%   Also savgol fw {11,5,3} (odd, >polyorder). Baked-Gamma row as the stable control.
%   Metric: land + xy + mean|sigma| mid (reaching working?) + vlat-flips.
%
% Run:  cd MATLAB/Multi_init_cond; cb_lagtest
clc; clear;
mfile_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(mfile_dir, '..', 'Common'));
clear mfile_dir;
global VDF_OVERRIDE %#ok<GVMIS>

B = struct('theta_per_axis',true,'kappa0',[0.05;0.05;0.05], ...
           'N',diag([0.10,0.10,0.10]),'Pleak',diag([0.5,0.5,1.5]),'E',diag([0.5,0.5,0.5]));
x0=[2;2;-5; 1;0;0;0; zeros(3,1); zeros(3,1)];

% {label, Gamma_xy, ZOH, fw}
cases = { 'Gamma.44 baked (control)', 0.4375, 3, 11; ...
          'Gamma2 baked lag',         2.0,    3, 11; ...
          'Gamma2 ZOH1 (100Hz)',      2.0,    1, 11; ...
          'Gamma2 fw5',               2.0,    3,  5; ...
          'Gamma2 ZOH1 + fw5',        2.0,    1,  5 };

fprintf('\n=== high Gamma: perception-lag (ZOH/savgol) ablation ===\n');
fprintf('  %-26s | land  xy=    | mean|sigma|mid | vlat-flips\n','config');
for k=1:size(cases,1)
    cfg=B; cfg.Gamma=diag([cases{k,2},cases{k,2},0.75]); cfg.ZOH=cases{k,3}; cfg.fw=cases{k,4};
    VDF_OVERRIDE=cfg;
    r=run_simulation(x0,"Sinusoidal",[],1.0,struct('NOISE',1,'GE',1,'delay',1),1);
    d=r.data; idx=d.idx; t=d.tRange(1:idx); t=t(:)';
    sg=vecnorm(d.sigma(1:2,1:idx)); midsg=mean(sg(t>2 & t<6));
    vlat=d.X_DS(8,1:idx)-d.dx_t(1,1:idx); flips=sum(abs(diff(sign(vlat)))>0);
    fprintf('  %-26s |  %d  %6.3f |    %6.3f      | %4d\n', ...
        cases{k,1}, r.success, r.final_xy, midsg, flips);
end
clear global VDF_OVERRIDE
fprintf('\n(ZOH1/fw5 recovers Gamma2 land + sigma->small => convergence is PERCEPTION-LAG-limited)\n');
