%% CB_AU_DECOMP  Which control-authority component drives the terminal balloon?
%   Runs the ballooning case (X3 fast Xi, Sinusoidal IC2 -> a_u 1627) and dumps the
%   a_u decomposition over the descent:  V_a_cd = REACH + SWITCH + EQUIV
%     REACH  = G_2^-1 Gamma sigma            (DAMPED by G_2^-1 at breach)
%     SWITCH = theta .* sat(sigma/E) .* kappa (G_2 CANCELS -> UNDAMPED; kappa-pumped)
%     EQUIV  = c - S_2 dp_h + G_2^-1 chi_zeta (c-term feedforward; flow-quadratic)
%   Identifies the term that explodes = the cause of the balloon.
%
% Run:  cd MATLAB/Multi_init_cond; cb_au_decomp
clc; clear;
mfile_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(mfile_dir, '..', 'Common'));
clear mfile_dir;
global VDF_OVERRIDE %#ok<GVMIS>

X3 = struct('theta_per_axis',true,'kappa0',[0.05;0.05;0.05], ...
            'N',diag([0.10,0.10,0.10]),'Pleak',diag([0.5,0.5,1.5]),'E',diag([0.5,0.5,0.5]), ...
            'p_hinf',[1.0;1.0;1.5],'p_rinf',[1.0;1.0],'Xi_h',diag([.8 .8 .8]),'Xi_r',diag([.7 .7]));
VDF_OVERRIDE = X3;
x0 = [2;2;-5; 1;0;0;0; zeros(3,1); zeros(3,1)];
r = run_simulation(x0, "Sinusoidal", [], 1.0, struct('NOISE',1,'GE',1,'delay',1), 1);
d = r.data; idx=d.idx; t=d.tRange(1:idx); t=t(:)';
z = 1./max(d.beta_log(1:idx),1e-6);
rbe = abs(d.s_e_log(:,1:idx)./d.P.phi_max(:));
engR = max(rbe./max(d.p_r_log(:,1:idx),eps),[],1);
kx  = max(d.kappa_log(1:2,1:idx),[],1);
au  = vecnorm(d.I_a_cd(1:2,1:idx));
comp = d.au_comp_log(:,1:idx);     % [reach; switch; equiv] (full-vector norms)

fprintf('\n=== a_u component decomposition: X3 fast-Xi Sinusoidal IC2 (landed=%d xy=%.3f) ===\n', ...
    r.success, r.final_xy);
fprintf('   t    z    engR  k_xy | REACH  SWITCH  EQUIV | total|au_xy|  dominant\n');
for tt = 0:1.0:t(end)
    [~,j]=min(abs(t-tt));
    [~,wi]=max(comp(:,j)); names=["REACH","SWITCH","EQUIV"];
    fprintf('  %4.1f %5.2f %5.2f %6.2f | %6.1f %6.1f %6.1f | %8.1f    %s\n', ...
        t(j),z(j),engR(j),kx(j), comp(1,j),comp(2,j),comp(3,j), au(j), names(wi));
end
% peak instant
[~,ip]=max(comp(2,:)+comp(3,:));
fprintf('\n  @ peak (t=%.2f z=%.2f): REACH=%.1f SWITCH=%.1f EQUIV=%.1f  (engR=%.2f kappa=%.2f)\n', ...
    t(ip),z(ip),comp(1,ip),comp(2,ip),comp(3,ip),engR(ip),kx(ip));
fprintf('  share at peak: REACH %.0f%%  SWITCH %.0f%%  EQUIV %.0f%%\n', ...
    100*comp(:,ip)./sum(comp(:,ip)));
clear global VDF_OVERRIDE
