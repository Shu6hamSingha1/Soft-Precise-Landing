%% CB_HD_DECOMP  What stops h_e from converging? h_e = h - h_d. Decompose h_d (lateral)
%   into its 3 parts: s_dot_meas (centroid rate), rot (transport), desc (h_rd*V_s).
%   Trace breach case (p_h-tight) vs LOCKED (baked p_h), Circ IC3 @7x. Is h_d non-zero
%   (controller COMMANDING flow) or is h failing to track a ~0 h_d? Which h_d part?
%
% Run:  cd MATLAB/Multi_init_cond; cb_hd_decomp
clc; clear;
mfile_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(mfile_dir, '..', 'Common'));
clear mfile_dir;
global VDF_OVERRIDE STRESS_SCALE %#ok<GVMIS>

LK  = struct('theta_per_axis',true,'kappa0',[0.05;0.05;0.05],'N',diag([0.10,0.10,0.10]), ...
             'Pleak',diag([0.5,0.5,1.5]),'E',diag([0.5,0.5,0.5]),'Xi_r',diag([0.3,0.3]),'p_rinf',[0.85;0.85]);
PHT = setfield(LK,'Xi_h',diag([0.35,0.35,0.35])); %#ok<*SFLD>

for cf={{'LOCKED baked p_h',LK},{'BREACH p_h-tight',PHT}}
    nm=cf{1}{1}; VDF_OVERRIDE=cf{1}{2}; STRESS_SCALE=7;
    r=run_simulation([2;-2;-5;1;0;0;0;zeros(6,1)],"Circular",[],1.0,struct('NOISE',1,'GE',1,'delay',1),1);
    d=r.data; idx=d.idx; t=d.tRange(1:idx); t=t(:)'; z=1./max(d.beta_log(1:idx),1e-6);
    hd=d.V_h_d(:,1:idx); he=d.V_h_e(:,1:idx); h=hd+he;       % h = h_d + h_e
    hdx=vecnorm(hd(1:2,:)); hx=vecnorm(h(1:2,:)); hex=vecnorm(he(1:2,:));
    sdot=vecnorm(d.hd_comp_log(1:2,1:idx)); rot=vecnorm(d.hd_comp_log(4:5,1:idx)); desc=vecnorm(d.hd_comp_log(7:8,1:idx));
    fprintf('\n=== %s (land=%d) ===\n', nm, r.success);
    fprintf('    z   | h_meas h_d  h_e | h_d parts: sdot rot desc\n');
    for zt=[2 1.5 1 0.7 0.5 0.35 0.25]
        j=find(z<=zt,1); if isempty(j),continue;end
        fprintf('  %4.2f | %5.2f %5.2f %5.2f | %5.2f %5.2f %5.2f\n', ...
            z(j), hx(j),hdx(j),hex(j), sdot(j),rot(j),desc(j));
    end
end
clear global VDF_OVERRIDE STRESS_SCALE
fprintf('\n(h_d>0 = controller COMMANDING flow; which part? sdot=centroid-rate chase, desc=h_rd*s)\n');
