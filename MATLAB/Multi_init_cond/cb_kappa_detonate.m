%% CB_KAPPA_DETONATE  Why do kappa & sigma DETONATE (not adapt) at a funnel breach?
%   kappa-ODE: kdot = theta*N*G*|sigma| - N*P*kappa  -> equilibrium k* = theta*G*|sigma|/P.
%   "Adapt" = funnel HELD: S<1, G/sigma moderate, theta bounded -> k* small -> k~0.1-0.5.
%   "Detonate" = BREACH: S->clip 0.95 -> G & sigma hit their clip CEILING, AND theta
%   inflates (diverging state) -> k* EXPLODES -> kappa ramps; the undamped switching
%   theta*sat*kappa feeds it back -> positive feedback.
%   Trace Circ IC3 @7x p_h-tightened: engH(breach), |sigma|, theta=sqrt(v^2+1),
%   G(reconstructed from engH), k*_pred, kappa, switch term.
%
% Run:  cd MATLAB/Multi_init_cond; cb_kappa_detonate
clc; clear;
mfile_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(mfile_dir, '..', 'Common'));
clear mfile_dir;
global VDF_OVERRIDE STRESS_SCALE %#ok<GVMIS>

PHT = struct('theta_per_axis',true,'kappa0',[0.05;0.05;0.05],'N',diag([0.10,0.10,0.10]), ...
             'Pleak',diag([0.5,0.5,1.5]),'E',diag([0.5,0.5,0.5]),'Xi_r',diag([0.3,0.3]), ...
             'p_rinf',[0.85;0.85],'Xi_h',diag([0.35,0.35,0.35]));
VDF_OVERRIDE=PHT; STRESS_SCALE=7;
r=run_simulation([2;-2;-5;1;0;0;0;zeros(6,1)],"Circular",[],1.0,struct('NOISE',1,'GE',1,'delay',1),1);
d=r.data; idx=d.idx; t=d.tRange(1:idx); t=t(:)'; P=d.P;
z=1./max(d.beta_log(1:idx),1e-6);
% per-axis x quantities
sg=abs(d.sigma(1,1:idx));                                  % |sigma_x|
th=sqrt(d.v_log(1,1:idx).^2+1);                            % theta_x = sqrt(v_x^2+1)
% reconstruct optic-flow barrier gain G_h from engagement S=h_e/p_h (clipped)
S=min(abs(d.V_h_e(1,1:idx))./max(d.p_h_log(1,1:idx),eps), 1-P.S_margin);
G=2./(max(d.p_h_log(1,1:idx),eps).*(1-S.^2));              % G_h = 2/(p(1-S^2))
engH=abs(d.V_h_e(1,1:idx))./max(d.p_h_log(1,1:idx),eps);
kx=d.kappa_log(1,1:idx); Px=P.Pleak(1,1);
kstar=th.*G.*sg./Px;                                      % predicted equilibrium k*
sw=d.au_comp_log(2,1:idx);                                % delivered switching term

fprintf('\n=== kappa/sigma detonation trace: Circ IC3 @7x p_h-tight (land=%d) ===\n', r.success);
fprintf('    t    z  | engH  |sig_x| theta_x  G_h  | k*=thG|s|/P | kappa_x  switch\n');
for tt=[6 8 10 12 13 13.5 14 14.5 15]
    if tt>t(end), continue; end
    [~,j]=min(abs(t-tt));
    fprintf('  %5.1f %4.2f | %5.2f %6.2f %6.1f %6.1f | %10.1f | %7.2f %6.1f\n', ...
        t(j),z(j), engH(j),sg(j),th(j),G(j), kstar(j), kx(j),sw(j));
end
[mk,jm]=max(kx);
fprintf('\n  @ kappa PEAK (t=%.1f z=%.2f): engH=%.1f |sig|=%.1f theta=%.0f G=%.0f -> k*=%.0f  kappa=%.1f\n', ...
    t(jm),z(jm), engH(jm),sg(jm),th(jm),G(jm),kstar(jm),mk);
fprintf('  -> detonation = breach (engH>1) inflates G & saturates sigma AND theta blows up -> k* explodes\n');
clear global VDF_OVERRIDE STRESS_SCALE
