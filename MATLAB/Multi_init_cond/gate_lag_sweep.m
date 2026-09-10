%% GATE_LAG_SWEEP  Calibrate the run_simulation lag model: find the magnitude at
%   which the VALIDATED MATLAB config still passes the 25-cell realistic gate
%   (PX4 SITL with sane gains DOES land, so a faithful surrogate must too), then
%   re-read the PX4 gain set at that same lag.
%
% Run:  cd MATLAB/Multi_init_cond; gate_lag_sweep
% Saves: gate_lag_sweep.mat
clc; clear;
here = fileparts(mfilename('fullpath'));
addpath(fullfile(here,'..','Common'));
global VDF_OVERRIDE STRESS_SCALE %#ok<GVMIS>

trajList = ["Static","Linear","Sinusoidal","Lissajous","Circular"];
p0 = [ 0,0,-5; 2.0,2.0,-5; 2.0,-2.0,-5; 2.0,2.0,-7; 2.0,2.0,-3 ];

MAT = struct('theta_per_axis',true);
PX4 = struct('theta_per_axis',true,'Xi_r',diag([0.10,0.10]), ...
             'Xi_h',diag([1.0,1.0,1.0]),'chi_r',[1.5;1.5],'h_rd',-0.30);

% lag configs, mildest -> PX4-nominal. act = roll/pitch/thrust hold [s];
% yaw = yaw-torque hold [s]; md = measured-corner transport delay [s].
L = { 'act10                ', struct('tau_act',0.010,'tau_yaw',0.010,'meas_delay',0.00);
      'act20                ', struct('tau_act',0.020,'tau_yaw',0.020,'meas_delay',0.00);
      'act38                ', struct('tau_act',0.038,'tau_yaw',0.038,'meas_delay',0.00);
      'act38+md30           ', struct('tau_act',0.038,'tau_yaw',0.038,'meas_delay',0.03);
      'act38+md60           ', struct('tau_act',0.038,'tau_yaw',0.038,'meas_delay',0.06);
      'act38+yaw150+md30    ', struct('tau_act',0.038,'tau_yaw',0.150,'meas_delay',0.03);
      'act38+yaw287+md60    ', struct('tau_act',0.038,'tau_yaw',0.287,'meas_delay',0.06) };

runarm = @(ov,co) deal_gate(trajList,p0,ov,co);

fprintf('\n%-22s | %-18s | %-18s\n','lag config','MATLAB gains','PX4 gains');
fprintf('%s\n', repmat('-',1,66));
G = struct('lag',{},'mat',{},'px4',{});
for i = 1:size(L,1)
    co = struct('NOISE',1,'GE',1,'delay',1,'lag',L{i,2});
    m = runarm(MAT,co);  p = runarm(PX4,co);
    G(i) = struct('lag',L{i,1},'mat',m,'px4',p);
    fprintf('%-22s | SP%2d/25 xy%.3f t%4.1f | SP%2d/25 xy%.3f t%4.1f\n', ...
        L{i,1}, m.sp, m.worstXY, m.meanT, p.sp, p.worstXY, p.meanT);
end
% baselines (no lag)
co0 = struct('NOISE',1,'GE',1,'delay',1);
m0 = runarm(MAT,co0); p0r = runarm(PX4,co0);
fprintf('%s\n', repmat('-',1,66));
fprintf('%-22s | SP%2d/25 xy%.3f t%4.1f | SP%2d/25 xy%.3f t%4.1f\n', ...
    'NO LAG (baseline)     ', m0.sp, m0.worstXY, m0.meanT, p0r.sp, p0r.worstXY, p0r.meanT);
save(fullfile(here,'gate_lag_sweep.mat'),'G','m0','p0r');
clear global VDF_OVERRIDE STRESS_SCALE

function s = deal_gate(trajList,p0,ov,co)
    global VDF_OVERRIDE STRESS_SCALE
    sp=0; xy=[]; tt=[]; vv=[];
    for t=1:numel(trajList)
        for ic=1:5
            VDF_OVERRIDE=ov; STRESS_SCALE=[];
            r=run_simulation([p0(ic,:)';1;0;0;0;zeros(6,1)],trajList(t),[],1.0,co,1);
            sp=sp+(r.precise&&r.soft); xy(end+1)=r.final_xy; tt(end+1)=r.final_t; vv(end+1)=r.final_rel_vel; %#ok<AGROW>
        end
    end
    s=struct('sp',sp,'worstXY',max(xy),'meanXY',mean(xy),'meanT',mean(tt),'worstV',max(vv));
end
