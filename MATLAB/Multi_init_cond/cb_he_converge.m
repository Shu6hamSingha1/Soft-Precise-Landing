%% CB_HE_CONVERGE  Why does h_e not converge terminally? Compare baked-Xi (lands) vs
%   fast-Xi (balloons): trace the PHYSICAL lateral offset (m), relative lateral velocity
%   (m/s), |h_e| (optic flow), and r_bar_e (normalized) down the descent. h_e=v_rel/z:
%   bounded h_e <=> v_rel->0 (soft landing). Show whether v_rel fails to arrest, and how
%   1/z amplifies the residual into the funnel.
%
% Run:  cd MATLAB/Multi_init_cond; cb_he_converge
clc; clear;
mfile_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(mfile_dir, '..', 'Common'));
clear mfile_dir;
global VDF_OVERRIDE %#ok<GVMIS>

C = struct('theta_per_axis',true,'kappa0',[0.05;0.05;0.05], ...
           'N',diag([0.10,0.10,0.10]),'Pleak',diag([0.5,0.5,1.5]),'E',diag([0.5,0.5,0.5]), ...
           'p_hinf',[1.0;1.0;1.5],'p_rinf',[1.0;1.0]);
mk = @(xih,xir) setfield(setfield(C,'Xi_h',diag([xih xih xih])),'Xi_r',diag([xir xir])); %#ok<SFLD>

for cf = {{'BAKED-Xi .2/.1', mk(0.2,0.1)}, {'FAST-Xi .8/.7', mk(0.8,0.7)}}
    nm=cf{1}{1}; VDF_OVERRIDE=cf{1}{2};
    x0=[2;2;-5; 1;0;0;0; zeros(3,1); zeros(3,1)];
    r=run_simulation(x0,"Sinusoidal",[],1.0,struct('NOISE',1,'GE',1,'delay',1),1);
    d=r.data; idx=d.idx; t=d.tRange(1:idx); t=t(:)';
    z = 1./max(d.beta_log(1:idx),1e-6);
    % physical lateral offset (m) and relative lateral velocity (m/s)
    pxy = vecnorm(d.X_DS(1:2,1:idx) - d.x_t(1:2,1:idx));
    vxy = vecnorm(d.X_DS(8:9,1:idx) - d.dx_t(1:2,1:idx));
    hxy = vecnorm(d.V_h_e(1:2,1:idx));                      % |h_e| lateral
    rbe = max(abs(d.s_e_log(:,1:idx)./d.P.phi_max(:)),[],1);% r_bar_e
    pr  = mean(d.p_r_log(:,1:idx),1);
    fprintf('\n=== %s (landed=%d final_xy=%.3f) ===\n', nm, r.success, r.final_xy);
    fprintf('    t    z   | phys_xy(m) v_rel(m/s) | |h_e|  r_bar_e  p_r  engR\n');
    for tt=[3 4 5 6 6.5 7 7.5 8 8.5 9]
        if tt>t(end), continue; end
        [~,j]=min(abs(t-tt));
        fprintf('  %4.1f %5.2f |   %6.3f    %6.3f   | %5.2f  %5.2f  %5.2f %5.2f\n', ...
            t(j),z(j),pxy(j),vxy(j),hxy(j),rbe(j),pr(j),rbe(j)/max(pr(j),eps));
    end
end
clear global VDF_OVERRIDE
