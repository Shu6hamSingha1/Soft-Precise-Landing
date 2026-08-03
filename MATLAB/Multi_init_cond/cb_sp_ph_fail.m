%% CB_SP_PH_FAIL  SP-signal convergence/boundedness + kappa/sigma + limit-cycle check
%   for the p_h-TIGHTENED FAILURE cases (locked base + Xi_h.35). Are s_e_n & h_e bounded
%   or diverging? Is the failure a BREACH (engH>1) or a sustained LIMIT CYCLE (flips>=10
%   + satFrac>0.4)? How do kappa & sigma behave (detonate)? Nominal + 7x stress cells.
%
% Run:  cd MATLAB/Multi_init_cond; cb_sp_ph_fail
clc; clear;
mfile_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(mfile_dir, '..', 'Common'));
clear mfile_dir;
global VDF_OVERRIDE STRESS_SCALE %#ok<GVMIS>

LK  = struct('theta_per_axis',true,'kappa0',[0.05;0.05;0.05],'N',diag([0.10,0.10,0.10]), ...
             'Pleak',diag([0.5,0.5,1.5]),'E',diag([0.5,0.5,0.5]),'Xi_r',diag([0.3,0.3]),'p_rinf',[0.85;0.85]);
PHT = setfield(LK,'Xi_h',diag([0.35,0.35,0.35])); %#ok<*SFLD>  % p_h tightened (the failure case)
Exy=0.5;
p0 = [ 0,0,-5; 2.0,2.0,-5; 2.0,-2.0,-5; 2.0,2.0,-7; 2.0,2.0,-3 ];
runs = { 'Sinusoidal',2,[]; 'Circular',3,[]; ...
         'Sinusoidal',2,7; 'Sinusoidal',4,7; 'Sinusoidal',5,7; 'Circular',3,7; 'Circular',5,7 };

fprintf('\n=== p_h-TIGHTENED (Xi_h.35) failure cases: SP-signal + kappa/sigma + limit-cycle ===\n');
fprintf('  %-16s SP | s_e_n mx/fin | h_e mx/fin | engR engH | sig_pk kap x/y/z fin | vflip x/y satF xy | DIAG\n','cell');
for k=1:size(runs,1)
    tr=runs{k,1}; ic=runs{k,2}; st=runs{k,3}; VDF_OVERRIDE=PHT; STRESS_SCALE=st;
    r=run_simulation([p0(ic,:)';1;0;0;0;zeros(6,1)],tr,[],1.0,struct('NOISE',1,'GE',1,'delay',1),1);
    d=r.data; idx=d.idx;
    sen=max(abs(d.s_e_log(:,1:idx)./d.P.phi_max(:)),[],1); he=vecnorm(d.V_h_e(1:2,1:idx));
    engR=max(max(abs(d.s_e_log(:,1:idx)./d.P.phi_max(:))./max(d.p_r_log(:,1:idx),eps),[],1));
    engH=max(max(abs(d.V_h_e(1:2,1:idx))./max(d.p_h_log(1:2,1:idx),eps),[],1));
    kap=d.kappa_log(:,idx); sigpk=max(vecnorm(d.sigma(1:2,1:idx)));
    vx=d.X_DS(8,1:idx)-d.dx_t(1,1:idx); vy=d.X_DS(9,1:idx)-d.dx_t(2,1:idx);
    fx=sum(abs(diff(sign(vx)))>0); fy=sum(abs(diff(sign(vy)))>0);
    satf=mean(abs(d.sigma(1:2,1:idx)/Exy)>=1,2); sp=r.precise&&r.soft;
    if engH>=1 && max(fx,fy)<10, dg='BREACH';
    elseif max(fx,fy)>=10 && mean(satf)>0.4, dg='LIMIT CYCLE';
    elseif sp, dg='OK'; else, dg='other'; end
    lab=sprintf('%s IC%d %s',tr,ic,ternary(isempty(st),'nom',[num2str(st) 'x']));
    fprintf('  %-16s %d  | %5.2f %5.2f | %5.2f %5.2f | %4.2f %5.2f | %5.2f %.2f/%.2f/%.2f | %3d/%-3d %.2f/%.2f | %s\n', ...
        lab, sp, max(sen),sen(end), max(he),he(end), engR,engH, sigpk,kap(1),kap(2),kap(3), fx,fy,satf(1),satf(2), dg);
end
clear global VDF_OVERRIDE STRESS_SCALE
fprintf('\n(BREACH = engH>1 no cycle; LIMIT CYCLE = flips>=10 + satFrac>0.4)\n');
function o=ternary(c,a,b), if c,o=a; else,o=b; end, end
