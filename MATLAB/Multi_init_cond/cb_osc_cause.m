%% CB_OSC_CAUSE  Why does high Gamma oscillate? Test the UNDER-DAMPED hypothesis:
%   sigma_xy = zeta_h (velocity/DAMPING) + chi_r*zeta_r (position/STIFFNESS). Gamma
%   raises loop stiffness; damping = zeta_h (dormant) + inner-loop kOmega, eroded by
%   lag. If it's under-damping, ADDING DAMPING at fixed high Gamma should suppress the
%   limit cycle:
%     - LOWER chi_r  -> surface more velocity-weighted (more damping share)
%     - HIGHER kOmega -> inner-loop attitude-rate damping
%     - REMOVE actuator delay -> restore phase margin
%   Metric = lateral velocity sign-flips (limit cycle = many; clean = few) + land + |h_e|.
%
% Run:  cd MATLAB/Multi_init_cond; cb_osc_cause
clc; clear;
mfile_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(mfile_dir, '..', 'Common'));
clear mfile_dir;
global VDF_OVERRIDE %#ok<GVMIS>

C = struct('theta_per_axis',true,'kappa0',[0.05;0.05;0.05], ...
           'N',diag([0.10,0.10,0.10]),'Pleak',diag([0.5,0.5,1.5]),'E',diag([0.5,0.5,0.5]), ...
           'Gamma',diag([2.0,2.0,0.75]));   % HIGH Gamma (the oscillating config)
x0=[2;2;-5; 1;0;0;0; zeros(3,1); zeros(3,1)];

tests = { ...
  'baseline chi_r=2 kOm=.3 delay1', C,                                          struct('delay',1);
  'chi_r=1.0',                       setfield(C,'chi_r',[1.0;1.0]),             struct('delay',1); %#ok<*SFLD>
  'chi_r=0.5',                       setfield(C,'chi_r',[0.5;0.5]),             struct('delay',1);
  'kOmega x2 (.6/.6/.4)',            setfield(C,'kOmega',diag([0.6,0.6,0.4])),  struct('delay',1);
  'delay=0 (no actuator lag)',       C,                                         struct('delay',0);
  'chi_r=0.5 + kOmega x2',           setfield(setfield(C,'chi_r',[0.5;0.5]),'kOmega',diag([0.6,0.6,0.4])), struct('delay',1) };

fprintf('\n=== high-Gamma (2.0) oscillation: what adds damping? ===\n');
fprintf('  %-32s | land xy=    | vlat-flips | |h_e|term\n','config');
for k=1:size(tests,1)
    VDF_OVERRIDE = tests{k,2};
    co = struct('NOISE',1,'GE',1,'delay',1); co.delay = tests{k,3}.delay;
    r = run_simulation(x0,"Sinusoidal",[],1.0,co,1);
    d=r.data; idx=d.idx;
    vlat = d.X_DS(8,1:idx) - d.dx_t(1,1:idx);            % relative lateral vel (x)
    flips = sum(abs(diff(sign(vlat)))>0);
    z = 1./max(d.beta_log(1:idx),1e-6);
    he = vecnorm(d.V_h_e(1:2,1:idx)); het = median(he(z<1.5));
    fprintf('  %-32s |  %d  %6.3f | %4d       | %.3f\n', ...
        tests{k,1}, r.success, r.final_xy, flips, het);
end
clear global VDF_OVERRIDE
