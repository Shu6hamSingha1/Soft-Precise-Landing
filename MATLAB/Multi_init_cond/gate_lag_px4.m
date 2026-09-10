%% GATE_LAG_PX4  Does a PX4-SITL lag model make PX4's gains the right choice in MATLAB?
%
%   Background: h_rd and chi_r do NOT port MATLAB<->PX4 (memory
%   feedback_gain_values_not_portable_either_direction). PX4's slower descent
%   (h_rd=-0.30) + softer lateral surface (chi_r=1.5) compensate SITL lag that
%   MATLAB's zero-latency RK5/ZOH plant lacks; ported straight into MATLAB they
%   regress the manuscript gate (mean t_f ~16.7 s, worst xy ~5.6 cm).
%
%   run_simulation.m now has an opt-in PX4 lag model (cfg_override.lag):
%     - ~38 ms first-order hold on roll/pitch torque + thrust
%     - ~287 ms first-order hold on yaw torque
%     - ~160 ms transport delay on the measured corners into image_features
%   (CONTROLLER_PARITY.md A4; memory feedback_impulse_response / MOVING_TARGET_PREP).
%
%   This script runs the 25-cell realistic gate (5 traj x 5 IC, NOISE/GE/delay=1,
%   seed=1) under 4 arms and prints SP / FoV / precision / landing-time so we can
%   see whether, WITH lag, the PX4 gain set matches or beats the MATLAB set
%   (i.e. whether the divergence is purely a lag artifact).
%
% Run:  cd MATLAB/Multi_init_cond; gate_lag_px4
% Saves: gate_lag_px4.mat
clc; clear;
here = fileparts(mfilename('fullpath'));
addpath(fullfile(here,'..','Common'));
global VDF_OVERRIDE STRESS_SCALE %#ok<GVMIS>

trajList = ["Static","Linear","Sinusoidal","Lissajous","Circular"];
p0 = [ 0,0,-5; 2.0,2.0,-5; 2.0,-2.0,-5; 2.0,2.0,-7; 2.0,2.0,-3 ];

MAT = struct('theta_per_axis',true);                                  % current vdf_params (Xi_r .30, Xi_h .2, chi_r 2.0, h_rd -.38)
PX4 = struct('theta_per_axis',true,'Xi_r',diag([0.10,0.10]), ...      % PX4 finalized set
             'Xi_h',diag([1.0,1.0,1.0]),'chi_r',[1.5;1.5],'h_rd',-0.30);

arms = { 'MATLAB gains  no-lag', MAT, [] ;
         'MATLAB gains  +lag  ', MAT, 1  ;
         'PX4 gains     no-lag', PX4, [] ;
         'PX4 gains     +lag  ', PX4, 1  };

A = struct('name',{},'sp',{},'fov',{},'meanXY',{},'worstXY',{},'meanV',{},'worstV',{},'meanT',{},'cells',{});
for k = 1:size(arms,1)
    nm = arms{k,1}; ov = arms{k,2}; lag = arms{k,3};
    co = struct('NOISE',1,'GE',1,'delay',1);
    if ~isempty(lag), co.lag = lag; end
    sp=0; fov=0; xy=[]; vv=[]; tt=[]; cells=strings(0);
    fprintf('\n===== %s =====\n', nm);
    for t = 1:numel(trajList)
        for ic = 1:5
            VDF_OVERRIDE = ov; STRESS_SCALE = [];
            r = run_simulation([p0(ic,:)';1;0;0;0;zeros(6,1)], trajList(t), [], 1.0, co, 1);
            ok = r.precise && r.soft;
            sp = sp + ok; fov = fov + r.fov_fail;
            xy(end+1)=r.final_xy; vv(end+1)=r.final_rel_vel; tt(end+1)=r.final_t; %#ok<AGROW>
            if ~ok, cells(end+1) = sprintf('%s-IC%d(xy%.3f v%.3f fov%d)', trajList(t), ic, ...
                    r.final_xy, r.final_rel_vel, r.fov_fail); end %#ok<AGROW>
            fprintf('  %-11s IC%d -> SP%d xy%.4f v%.4f t%.2f fov%d\n', ...
                trajList(t), ic, ok, r.final_xy, r.final_rel_vel, r.final_t, r.fov_fail);
        end
    end
    A(k) = struct('name',nm,'sp',sp,'fov',fov,'meanXY',mean(xy),'worstXY',max(xy), ...
                  'meanV',mean(vv),'worstV',max(vv),'meanT',mean(tt),'cells',{cells});
end

fprintf('\n================ SUMMARY (25-cell realistic gate, seed=1) ================\n');
fprintf('%-22s | SP    | FoV | meanXY  worstXY | meanV   worstV | mean t_f\n','arm');
fprintf('%s\n', repmat('-',1,84));
for k = 1:numel(A)
    fprintf('%-22s | %2d/25 | %2d  | %.4f  %.4f | %.4f  %.4f | %6.2f\n', ...
        A(k).name, A(k).sp, A(k).fov, A(k).meanXY, A(k).worstXY, A(k).meanV, A(k).worstV, A(k).meanT);
end
fprintf('%s\n', repmat('-',1,84));
fprintf('Read: if "PX4 gains +lag" ~matches "MATLAB gains no-lag", the h_rd/chi_r\n');
fprintf('divergence is a pure lag artifact and PX4''s values are portable once lag is modeled.\n');
save(fullfile(here,'gate_lag_px4.mat'),'A');
clear global VDF_OVERRIDE STRESS_SCALE
