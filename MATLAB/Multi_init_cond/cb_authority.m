%% CB_AUTHORITY  Why does the ASMC fail to converge h_e terminally? Test the two real
%   candidates (NOT the circular "v_rel doesn't arrest"):
%     (a) DISTURBANCE overrun: |d_h| (incl the dropped s_ddot, 1/z-amplified) explodes.
%     (b) AUTHORITY saturation: commanded |Iacd_xy| >> delivered |I_a_filt_xy| (CBF/tilt cap).
%   Compare baked-Xi (converges) vs fast-Xi (diverges) down the descent.
%
% Run:  cd MATLAB/Multi_init_cond; cb_authority
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
    cmd = vecnorm(d.I_a_cd(1:2,1:idx));       % commanded lateral accel (asmc)
    dlv = vecnorm(d.I_a_filt_log(1:2,1:idx)); % delivered (CBF/tilt-capped)
    dh  = vecnorm(d.d_h_log(1:2,1:idx));      % lateral lumped disturbance
    he  = vecnorm(d.V_h_e(1:2,1:idx));        % |h_e| lateral
    sat = cmd./max(dlv,1e-6);
    fprintf('\n=== %s (landed=%d xy=%.3f) ===\n', nm, r.success, r.final_xy);
    fprintf('    t    z   | |d_h|  |h_e| | cmd_au  deliv_au  sat(cmd/dlv)\n');
    for tt=[3 4 5 6 7 7.5 8 8.5 9]
        if tt>t(end), continue; end
        [~,j]=min(abs(t-tt));
        fprintf('  %4.1f %5.2f | %5.2f %5.2f | %7.1f %7.2f   %6.1fx\n', ...
            t(j),z(j),dh(j),he(j),cmd(j),dlv(j),sat(j));
    end
end
clear global VDF_OVERRIDE
