%% GATE_LAG_MD  Perception/outer-loop latency ONLY (no torque-command lag).
%   The torque-command hold in gate_lag_sweep broke the SO(3) inner loop (which
%   MATLAB assumes instantaneous) rather than the h_rd/chi_r outer loop. PX4 docs
%   say the lever h_rd/chi_r were detuned against is "outer-loop bandwidth, NOT
%   the 38 ms actuation lag" (MOVING_TARGET_PREP.md). So test the measured-corner
%   transport delay alone.
% Run: cd MATLAB/Multi_init_cond; gate_lag_md
clc; clear;
here = fileparts(mfilename('fullpath')); addpath(fullfile(here,'..','Common'));
global VDF_OVERRIDE STRESS_SCALE %#ok<GVMIS>
trajList = ["Static","Linear","Sinusoidal","Lissajous","Circular"];
p0 = [0,0,-5; 2,2,-5; 2,-2,-5; 2,2,-7; 2,2,-3];
MAT = struct('theta_per_axis',true);
PX4 = struct('theta_per_axis',true,'Xi_r',diag([0.10,0.10]),'Xi_h',diag([1.0,1.0,1.0]),'chi_r',[1.5;1.5],'h_rd',-0.30);
mds = [0.00 0.02 0.03 0.05 0.08 0.12];
fprintf('\n%-10s | %-20s | %-20s\n','meas_delay','MATLAB gains','PX4 gains');
fprintf('%s\n',repmat('-',1,56));
for md = mds
    if md==0, co=struct('NOISE',1,'GE',1,'delay',1);
    else, co=struct('NOISE',1,'GE',1,'delay',1,'lag',struct('tau_act',1e-4,'tau_yaw',1e-4,'meas_delay',md)); end
    m=g(trajList,p0,MAT,co); p=g(trajList,p0,PX4,co);
    fprintf('%-10.2f | SP%2d/25 xy%.3f t%4.1f | SP%2d/25 xy%.3f t%4.1f\n', md, m.sp,m.wxy,m.mt, p.sp,p.wxy,p.mt);
end
clear global VDF_OVERRIDE STRESS_SCALE
function s=g(trajList,p0,ov,co)
    global VDF_OVERRIDE STRESS_SCALE
    sp=0;xy=[];tt=[];
    for t=1:numel(trajList), for ic=1:5
        VDF_OVERRIDE=ov; STRESS_SCALE=[];
        r=run_simulation([p0(ic,:)';1;0;0;0;zeros(6,1)],trajList(t),[],1.0,co,1);
        sp=sp+(r.precise&&r.soft); xy(end+1)=r.final_xy; tt(end+1)=r.final_t; %#ok<AGROW>
    end, end
    s=struct('sp',sp,'wxy',max(xy),'mt',mean(tt));
end
