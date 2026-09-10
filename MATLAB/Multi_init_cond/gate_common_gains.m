%% GATE_COMMON_GAINS  Search for a gain set usable UNCHANGED in both PX4 and MATLAB.
%
%   Already matched MATLAB<->PX4: Gamma, E, N, Pleak, kappa0, p_h0, p_hinf,
%   p_r0, p_rinf, hd_kr, chi_z, yaw (k_p/k_i/w_max), two-tier CBF, cbf_drift_tau.
%   The gap is 4 rows:
%      Xi_r   MATLAB 0.30   PX4 0.10
%      Xi_h   MATLAB 0.20   PX4 1.00
%      chi_r  MATLAB 2.00   PX4 1.50
%      h_rd   MATLAB -0.38  PX4 -0.30
%
%   HANDOFF_xir_xih_moving_target.md (earlier config) found Xi_r=0.20 + Xi_h=0.20
%   clean on BOTH stationary and moving in MATLAB. Re-confirm on the CURRENT stack
%   (5-pt cross + yaw-rate-law + two-tier CBF) and bracket chi_r / h_rd for a
%   common point that (a) passes the MATLAB 25-cell gate and (b) leans toward PX4.
%
% Run: cd MATLAB/Multi_init_cond; gate_common_gains
% Saves: gate_common_gains.mat
clc; clear;
here = fileparts(mfilename('fullpath')); addpath(fullfile(here,'..','Common'));
global VDF_OVERRIDE STRESS_SCALE %#ok<GVMIS>
trajList = ["Static","Linear","Sinusoidal","Lissajous","Circular"];
p0 = [0,0,-5; 2,2,-5; 2,-2,-5; 2,2,-7; 2,2,-3];

base = @(xir,xih,chir,hrd) struct('theta_per_axis',true, ...
        'Xi_r',diag([xir xir]),'Xi_h',diag([xih xih xih]), ...
        'chi_r',[chir;chir],'h_rd',hrd);

C = { 'MATLAB current      Xir.30 Xih.20 chi2.0 h-.38', base(0.30,0.20,2.0,-0.38);
      'PX4 current         Xir.10 Xih1.0 chi1.5 h-.30', base(0.10,1.00,1.5,-0.30);
      'common A (HANDOFF)  Xir.20 Xih.20 chi1.5 h-.30', base(0.20,0.20,1.5,-0.30);
      'common B  +chi2.0   Xir.20 Xih.20 chi2.0 h-.30', base(0.20,0.20,2.0,-0.30);
      'common C  +h-.34    Xir.20 Xih.20 chi2.0 h-.34', base(0.20,0.20,2.0,-0.34);
      'common D  Xir.25     Xir.25 Xih.20 chi2.0 h-.34', base(0.25,0.20,2.0,-0.34);
      'common E  Xih.40    Xir.20 Xih.40 chi1.5 h-.30', base(0.20,0.40,1.5,-0.30) };

% moving-only subset for the s_e_n-convergence read (HANDOFF discriminator)
movList = ["Linear","Sinusoidal","Lissajous","Circular"];

fprintf('\n%-48s | stat 5x IC | move 20 | worstXY | mean t_f | FoV\n','config');
fprintf('%s\n', repmat('-',1,104));
R = struct('name',{},'statSP',{},'moveSP',{},'worstXY',{},'meanT',{},'fov',{});
for i = 1:size(C,1)
    ov = C{i,2};
    sSP=0; mSP=0; xy=[]; tt=[]; fov=0;
    for t=1:numel(trajList)
        for ic=1:5
            VDF_OVERRIDE=ov; STRESS_SCALE=[];
            r=run_simulation([p0(ic,:)';1;0;0;0;zeros(6,1)],trajList(t),[],1.0,struct('NOISE',1,'GE',1,'delay',1),1);
            ok=r.precise&&r.soft; xy(end+1)=r.final_xy; tt(end+1)=r.final_t; fov=fov+r.fov_fail; %#ok<AGROW>
            if trajList(t)=="Static", sSP=sSP+ok; else, mSP=mSP+ok; end
        end
    end
    R(i)=struct('name',C{i,1},'statSP',sSP,'moveSP',mSP,'worstXY',max(xy),'meanT',mean(tt),'fov',fov);
    fprintf('%-48s |   %d/5      |  %2d/20  | %.4f  |  %5.2f   | %2d\n', ...
        C{i,1}, sSP, mSP, max(xy), mean(tt), fov);
end
fprintf('%s\n', repmat('-',1,104));
fprintf('Target: a row with stat 5/5, move 20/20, FoV 0, worstXY < 0.06, that is\n');
fprintf('one edit away from PX4 current (ideally just Xi_r + Xi_h).\n');
save(fullfile(here,'gate_common_gains.mat'),'R');
clear global VDF_OVERRIDE STRESS_SCALE
