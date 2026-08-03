%% CB_E_KAPPA_SIGMA  Tune E and see how kappa performs against sigma.
%   The switching term delivered against the surface is  kappa .* sat(sigma/E).
%   E gates it: |sigma|/E < 1 -> LINEAR (kappa*sigma/E, switching "locked out");
%   |sigma|/E >= 1 -> sat saturates -> full kappa delivered. Sweep E_xy and measure,
%   per axis x: does sigma ESCAPE the boundary layer, does kappa ENGAGE, and how much
%   switching authority (kappa*sat) is actually applied AGAINST sigma.
%   Run nominal AND under a KNOWN 3N lateral disturbance (to excite sigma).
%
% Run:  cd MATLAB/Multi_init_cond; cb_E_kappa_sigma
clc; clear;
mfile_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(mfile_dir, '..', 'Common'));
clear mfile_dir;
global VDF_OVERRIDE KNOWN_DIST %#ok<GVMIS>

B = struct('theta_per_axis',true,'kappa0',[0.05;0.05;0.05], ...
           'N',diag([0.10,0.10,0.10]),'Pleak',diag([0.5,0.5,1.5]));
Exys = [1.0, 0.5, 0.25, 0.1];
x0=[2;2;-5; 1;0;0;0; zeros(3,1); zeros(3,1)];

for scenario = ["nominal", "known-3N-step"]
    fprintf('\n================= %s =================\n', scenario);
    fprintf('  E_xy | pk|s_x| pk|s|/E  %%engaged | kap_x pk  mean | deliv kap*sat | land xy=\n');
    for Exy = Exys
        cfg=B; cfg.E=diag([Exy,Exy,0.5]); VDF_OVERRIDE=cfg;
        if scenario=="known-3N-step"
            KNOWN_DIST = struct('force',[3;0;0],'t_on',2.0,'t_off',inf);
        else
            KNOWN_DIST = [];
        end
        r=run_simulation(x0,"Sinusoidal",[],1.0,struct('NOISE',1,'GE',1,'delay',1),1);
        d=r.data; idx=d.idx;
        sx = abs(d.sigma(1,1:idx));  kx = d.kappa_log(1,1:idx);
        soe = sx/Exy;                                   % |sigma_x|/E_x
        deliv = kx .* min(max(d.sigma(1,1:idx)/Exy,-1),1);  % kappa*sat(sigma/E) (signed)
        eng = mean(soe>=1)*100;                         % %time switching saturated
        fprintf('  %4.2f |  %5.3f   %5.2f    %4.0f%% | %6.3f  %5.3f |   %6.3f      | %d %6.3f\n', ...
            Exy, max(sx), max(soe), eng, max(kx), mean(kx), mean(abs(deliv)), r.success, r.final_xy);
    end
end
clear global VDF_OVERRIDE KNOWN_DIST
fprintf('\n(want: E small enough that |sigma|/E reaches >=1 (escapes layer) so kappa SWITCHES against sigma,\n but not so small it chatters/over-drives. delivered kappa*sat = how hard kappa pushes on sigma.)\n');
