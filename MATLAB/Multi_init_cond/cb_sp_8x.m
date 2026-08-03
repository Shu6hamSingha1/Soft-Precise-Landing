%% CB_SP_8X  At 8x (where LOCKED drops to 1/5), is the failure a BREACH (engH>1) or a
%   sustained LIMIT CYCLE (vlat flips >=10 + high satFrac)? Run LOCKED on the 5 stress
%   cells at 8x; report SP + s_e_n/h_e bound + engR/engH + kappa + flips + satFrac.
%
% Run:  cd MATLAB/Multi_init_cond; cb_sp_8x
clc; clear;
mfile_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(mfile_dir, '..', 'Common'));
clear mfile_dir;
global VDF_OVERRIDE STRESS_SCALE %#ok<GVMIS>

LK = struct('theta_per_axis',true,'kappa0',[0.05;0.05;0.05],'N',diag([0.10,0.10,0.10]), ...
            'Pleak',diag([0.5,0.5,1.5]),'E',diag([0.5,0.5,0.5]),'Xi_r',diag([0.3,0.3]),'p_rinf',[0.85;0.85]);
Exy=0.5;
p0 = [ 0,0,-5; 2.0,2.0,-5; 2.0,-2.0,-5; 2.0,2.0,-7; 2.0,2.0,-3 ];
cells = { 'Sinusoidal',2; 'Sinusoidal',4; 'Sinusoidal',5; 'Circular',3; 'Circular',5 };

fprintf('\n=== LOCKED @ 8x: breach vs limit-cycle on each stress cell ===\n');
fprintf('  %-14s SP | s_e_n mx/fin | h_e mx/fin | engR engH | kap x/y/z fin | vflip x/y satFrac xy | DIAGNOSIS\n','cell');
for c=1:size(cells,1)
    tr=cells{c,1}; ic=cells{c,2}; VDF_OVERRIDE=LK; STRESS_SCALE=8;
    r=run_simulation([p0(ic,:)';1;0;0;0;zeros(6,1)],tr,[],1.0,struct('NOISE',1,'GE',1,'delay',1),1);
    d=r.data; idx=d.idx;
    sen=max(abs(d.s_e_log(:,1:idx)./d.P.phi_max(:)),[],1); he=vecnorm(d.V_h_e(1:2,1:idx));
    engR=max(max(abs(d.s_e_log(:,1:idx)./d.P.phi_max(:))./max(d.p_r_log(:,1:idx),eps),[],1));
    engH=max(max(abs(d.V_h_e(1:2,1:idx))./max(d.p_h_log(1:2,1:idx),eps),[],1));
    kap=d.kappa_log(:,idx);
    vx=d.X_DS(8,1:idx)-d.dx_t(1,1:idx); vy=d.X_DS(9,1:idx)-d.dx_t(2,1:idx);
    fx=sum(abs(diff(sign(vx)))>0); fy=sum(abs(diff(sign(vy)))>0);
    satf=mean(abs(d.sigma(1:2,1:idx)/Exy)>=1,2);
    sp=r.precise&&r.soft;
    if engH>=1 && max(fx,fy)<10, dg='BREACH (engH>1)';
    elseif max(fx,fy)>=10 && mean(satf)>0.4, dg='LIMIT CYCLE';
    elseif ~sp && engH<1, dg='soft-fail (v residual)';
    elseif sp, dg='OK'; else, dg='other'; end
    fprintf('  %-14s %d  | %5.2f %5.2f | %5.2f %5.2f | %4.2f %5.2f | %.2f/%.2f/%.2f | %3d/%-3d %.2f/%.2f | %s\n', ...
        sprintf('%s IC%d',tr,ic), sp, max(sen),sen(end), max(he),he(end), engR,engH, ...
        kap(1),kap(2),kap(3), fx,fy, satf(1),satf(2), dg);
end
clear global VDF_OVERRIDE STRESS_SCALE
fprintf('\n(BREACH = engH>1 no cycle; LIMIT CYCLE = flips>=10 + satFrac>0.4; soft-fail = v residual, funnel held)\n');
