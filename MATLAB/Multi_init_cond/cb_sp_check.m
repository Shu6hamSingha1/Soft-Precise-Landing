%% CB_SP_CHECK  Convergence + boundedness of s_e_n (=r_bar_e) and h_e; kappa & sigma
%   behaviour; limit-cycle test. LOCKED config (COMBINED + pr-tight + E.5, baked p_h).
%   SP <=> s_e_n bounded (-> lat->0 precise) AND h_e bounded (-> v_rel->0 soft).
%   Limit cycle (PX4 metric): v_lat sign-flips per axis (cycle = 10+ sustained; clean 1-3)
%   + sat(sigma/E) saturation fraction. Report nominal AND 7x stress.
%
% Run:  cd MATLAB/Multi_init_cond; cb_sp_check
clc; clear;
mfile_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(mfile_dir, '..', 'Common'));
clear mfile_dir;
global VDF_OVERRIDE STRESS_SCALE %#ok<GVMIS>

LK = struct('theta_per_axis',true,'kappa0',[0.05;0.05;0.05],'N',diag([0.10,0.10,0.10]), ...
            'Pleak',diag([0.5,0.5,1.5]),'E',diag([0.5,0.5,0.5]),'Xi_r',diag([0.3,0.3]),'p_rinf',[0.85;0.85]);
Exy=0.5;
p0 = [ 0,0,-5; 2.0,2.0,-5; 2.0,-2.0,-5; 2.0,2.0,-7; 2.0,2.0,-3 ];
runs = { 'Sinusoidal',2,[]; 'Circular',3,[]; 'Sinusoidal',2,7; 'Circular',3,7 };

fprintf('\n=== LOCKED: SP-signal convergence/boundedness + limit-cycle check ===\n');
fprintf('  %-18s SP | s_e_n: mx fin | h_e: mx fin | engR engH(resid<1=bnd) | kap x/y/z fin | sig_xy pk | LC: vflip x/y  satFrac xy\n','traj/IC/stress');
for k=1:size(runs,1)
    tr=runs{k,1}; ic=runs{k,2}; st=runs{k,3};
    VDF_OVERRIDE=LK; STRESS_SCALE=st;
    r=run_simulation([p0(ic,:)';1;0;0;0;zeros(6,1)],tr,[],1.0,struct('NOISE',1,'GE',1,'delay',1),1);
    d=r.data; idx=d.idx;
    sen=max(abs(d.s_e_log(:,1:idx)./d.P.phi_max(:)),[],1);   % s_e_n = r_bar_e
    he =vecnorm(d.V_h_e(1:2,1:idx));
    engR=max(abs(d.s_e_log(:,1:idx)./d.P.phi_max(:))./max(d.p_r_log(:,1:idx),eps),[],1);
    engH=max(abs(d.V_h_e(1:2,1:idx))./max(d.p_h_log(1:2,1:idx),eps),[],1);
    kap=d.kappa_log(:,idx); sig=d.sigma(1:2,1:idx);
    vx=d.X_DS(8,1:idx)-d.dx_t(1,1:idx); vy=d.X_DS(9,1:idx)-d.dx_t(2,1:idx);
    fx=sum(abs(diff(sign(vx)))>0); fy=sum(abs(diff(sign(vy)))>0);
    satf=mean(abs(d.sigma(1:2,1:idx)/Exy)>=1, 2);            % sat saturation fraction x,y
    lab=sprintf('%s IC%d %s',tr,ic, ternary(isempty(st),'nom',[num2str(st) 'x']));
    fprintf('  %-18s %d  | %5.2f %5.2f | %5.2f %5.2f | %4.2f %4.2f          | %.2f/%.2f/%.2f | %5.2f    | %3d/%-3d  %.2f/%.2f\n', ...
        lab, r.precise&&r.soft, max(sen),sen(end), max(he),he(end), max(engR),max(engH), ...
        kap(1),kap(2),kap(3), max(vecnorm(sig)), fx,fy, satf(1),satf(2));
end
clear global VDF_OVERRIDE STRESS_SCALE
fprintf('\n(bounded = engR/engH stay <1 in descent; LC = vflip>=10 sustained + high satFrac)\n');
function o=ternary(c,a,b), if c,o=a; else,o=b; end, end
