%% CB_PR_REDUCE  How low can p_r go? Sweep p_rinf below the LOCKED 0.85 on the baked
%   LOCKED config. Trade-off: lower p_rinf -> tighter position barrier (precision) BUT
%   engR rises toward 1 (breach margin shrinks) + deeper Standing-Cond-1 violation.
%   Report: gate SP, mean/worst xy, max engR (margin to breach), FoV breaches, stress 7x.
%   (Xi_r stays 0.3 baked; only p_rinf swept. p_r0=1.2 FoV-fixed.)
%
% Run:  cd MATLAB/Multi_init_cond; cb_pr_reduce
clc; clear;
mfile_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(mfile_dir, '..', 'Common'));
clear mfile_dir;
global VDF_OVERRIDE STRESS_SCALE %#ok<GVMIS>

trajList = ["Static","Linear","Sinusoidal","Lissajous","Circular"];
p0 = [ 0,0,-5; 2.0,2.0,-5; 2.0,-2.0,-5; 2.0,2.0,-7; 2.0,2.0,-3 ];
scells = { 'Sinusoidal',2;'Sinusoidal',4;'Sinusoidal',5;'Circular',3;'Circular',5 };

fprintf('\n=== p_r reduction sweep (LOCKED base, p_rinf swept) ===\n');
fprintf('  p_rinf | gate  mXY    worstXY | maxEngR | fov fly | 7x SP\n');
for prinf=[0.85, 0.70, 0.60, 0.50]
    cfg=struct('p_rinf',[prinf;prinf]);   % baked LOCKED + this p_rinf override
    g=0; xy=[]; eR=[]; fov=0; fly=0;
    for t=1:numel(trajList)
        for ic=1:5
            VDF_OVERRIDE=cfg; STRESS_SCALE=[];
            r=run_simulation([p0(ic,:)';1;0;0;0;zeros(6,1)],trajList(t),[],1.0,struct('NOISE',1,'GE',1,'delay',1),1);
            d=r.data; idx=d.idx;
            g=g+(r.precise&&r.soft); xy(end+1)=r.final_xy; %#ok<AGROW>
            eR(end+1)=max(max(abs(d.s_e_log(:,1:idx)./d.P.phi_max(:))./max(d.p_r_log(:,1:idx),eps),[],1)); %#ok<AGROW>
            if r.fov_fail, fov=fov+1; end
            if ~r.success||r.final_alt>0.25, fly=fly+1; end
        end
    end
    sp7=0;
    for c=1:size(scells,1)
        VDF_OVERRIDE=cfg; STRESS_SCALE=7;
        r=run_simulation([p0(scells{c,2},:)';1;0;0;0;zeros(6,1)],scells{c,1},[],1.0,struct('NOISE',1,'GE',1,'delay',1),1);
        sp7=sp7+(r.precise&&r.soft);
    end
    fprintf('  %4.2f   | %2d/25 %.4f %.4f  | %5.2f   | %d   %d  | %d/5\n', ...
        prinf, g, mean(xy), max(xy), max(eR), fov, fly, sp7);
end
clear global VDF_OVERRIDE STRESS_SCALE
fprintf('\n(maxEngR->1 = at breach margin; p_rinf must stay > max r_bar_e ~0.54 or FoV breach)\n');
