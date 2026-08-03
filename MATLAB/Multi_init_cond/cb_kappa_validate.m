%% CB_KAPPA_VALIDATE  Validate kappa adaptation against a KNOWN injected disturbance.
%
% Testbed isolates the adaptation: Static trajectory (no target motion) + ALL
% stochastic disturbance zeroed (KNOWN_DIST hook in init_robustness) + a single
% deterministic lateral force injected into the plant (run_simulation hook). The
% disturbance is then fully known, so kappa's response can be checked vs theory.
%
% kappa-ODE:  dk/dt = diag(theta)*N*G*|sigma| - N*P*kappa
%   -> equilibrium  k* = diag(theta)*G*|sigma| / P ;  decay time const tau = 1/(N*P)
%
% Experiments (lateral x-axis):
%   A STEP      force on [t_on,t_off) -> watch k_x RISE then leak-DECAY (checks 1-3,7,8)
%   B DOSE      constant force in {0,.25,.5,1.0} N -> k_x* monotone in |F| (checks 4,5)
%   C SIGN      +-0.5 N -> k_x grows for both; d_h flips sign (check 6)
%
% Run:  cd MATLAB/Multi_init_cond; cb_kappa_validate
% Saves: ../Datasets/MultiInit/kappa_validate.mat
clc; clear;
mfile_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(mfile_dir, '..', 'Common'));
clear mfile_dir;

% Controller under test (default = config D: primed adaptive law). Swap to compare.
CFG = struct('theta_per_axis',true,'kappa0',[0.05;0.05;0.05],'N',diag([0.10,0.10,0.10]));
AX = 1;   % validate on x

% ---------------- Experiment A: STEP disturbance ----------------
% Fx chosen in the responsive regime (cb_kappa_threshold: 0.6 N is fully
% boundary-layer-gated; 5 N drives kappa_x up 0.038->0.073 and still lands).
Fx = 5.0;  ton = 2.0; toff = 6.0;
dA = onerun(CFG, [Fx;0;0], ton, toff);
[tA, kA, dhA, sgA, FA, P] = unpack(dA, AX);
onMask  = FA ~= 0;  offMask = ~onMask & (tA > toff);
k_pre  = mean(kA(tA<ton & tA>ton-0.5));
k_on   = max(kA(onMask));
k_toff = interp1q(tA(:), kA(:), toff);
% leakage decay: fit tau from the off-phase, compare to 1/(N*P)
Nx = CFG.N(AX,AX); Px = P.Pleak(AX,AX); tau_pred = 1/(Nx*Px);
ioff = find(offMask, 1);
if ~isempty(ioff) && numel(tA)-ioff > 10
    seg = kA(ioff:end); ts = tA(ioff:end)-tA(ioff);
    kinf = P.kappa0(AX);                                   % leaks toward bootstrap? -> 0 floor
    y = log(max(seg-min(seg)+1e-6, 1e-6));
    c = polyfit(ts(:), y(:), 1); tau_meas = -1/c(1);
else, tau_meas = NaN; end
Ex = P.E(AX,AX); soe_on = max(abs(sgA(onMask))/Ex);       % boundary-layer gate
dh_on  = mean(dhA(onMask)); dh_off = mean(dhA(tA<ton));

fprintf('\n========== A: STEP (Fx=%.2f N on [%.1f,%.1f]) ==========\n', Fx, ton, toff);
fprintf('  [1] observer  : d_h pre=%.3f  during=%.3f  (steps with the force? %s)\n', ...
    dh_off, dh_on, tf(abs(dh_on-dh_off)>0.05));
fprintf('  [3] rise      : kappa_x pre=%.3f -> on-peak=%.3f  (adapts up? %s)\n', ...
    k_pre, k_on, tf(k_on>k_pre+0.01));
fprintf('  [7] leak decay: at t_off=%.3f ; tau_meas=%.2fs  tau_pred=1/(N*P)=%.2fs  (%s)\n', ...
    k_toff, tau_meas, tau_pred, tf(isfinite(tau_meas)&&abs(tau_meas-tau_pred)/tau_pred<0.6));
fprintf('  [8] BL gate   : max|sigma_x|/E_x=%.2f  (switching engaged if >=1? %s)\n', ...
    soe_on, tf(soe_on>=1));

% ---------------- Experiment B: DOSE-RESPONSE ----------------
mags = [0, 0.25, 0.5, 1.0];
fprintf('\n========== B: DOSE-RESPONSE (constant Fx, whole flight) ==========\n');
fprintf('  |F| (N) | kappa_x* (steady) | k*_pred=thG|s|/P | max|h_e_x| | landed\n');
kstar = zeros(size(mags));
for i = 1:numel(mags)
    d = onerun(CFG, [mags(i);0;0], 0, inf);
    [t,k,dh,sg,F,Pp] = unpack(d, AX);                              %#ok<ASGLU>
    win = t > 0.6*t(end);                                          % steady window (late flight)
    kstar(i) = mean(k(win));
    % predicted equilibrium from logged regressor/barrier/surface
    th = sqrt(d.v_log(AX,d.idx).^2 + 1);
    eng= abs(d.V_h_e(AX,1:d.idx))./max(d.p_h_log(AX,1:d.idx),eps);
    S  = min(eng,0.999); zt = log((1+S)./(1-S));
    g  = (exp(zt)+1).^2 ./ (2*exp(zt).*d.p_h_log(AX,1:d.idx));
    kpred = th .* mean(g(win(1:d.idx))) .* mean(abs(sg(win))) ./ Pp.Pleak(AX,AX);
    hemax = max(abs(d.V_h_e(AX,1:d.idx)));
    fprintf('   %5.2f  |      %7.4f      |     %8.4f     |   %6.3f   | %d\n', ...
        mags(i), kstar(i), kpred, hemax, d.landed);
end
mono = all(diff(kstar)>=-1e-3);
fprintf('  [4/5] dose monotone (kappa_x* rises with |F|)? %s   [%.4f -> %.4f]\n', ...
    tf(mono), kstar(1), kstar(end));

% ---------------- Experiment C: SIGN ----------------
dP = onerun(CFG, [ 0.5;0;0], 0, inf); [tp,kp,dhp]=unpack(dP,AX);
dM = onerun(CFG, [-0.5;0;0], 0, inf); [tm,km,dhm]=unpack(dM,AX); %#ok<ASGLU>
fprintf('\n========== C: SIGN INDEPENDENCE (+-0.5 N) ==========\n');
fprintf('  +F: kappa_x peak=%.3f  d_h mean=%.3f\n', max(kp), mean(dhp(tp>0.6*tp(end))));
fprintf('  -F: kappa_x peak=%.3f  d_h mean=%.3f\n', max(km), mean(dhm(tm>0.6*tm(end))));
% sign check = symmetry of kappa response + d_h sign flip (NOT growth-above-k0,
% which is the boundary-layer-gate story tested in [8], not a sign property)
kp_pk = max(kp); km_pk = max(km);
dhp_s = sign(mean(dhp(tp>0.6*tp(end)))); dhm_s = sign(mean(dhm(tm>0.6*tm(end))));
fprintf('  [6] symmetric kappa (|+peak - -peak|<10%%) & d_h flips sign? %s\n', ...
    tf(abs(kp_pk-km_pk)/max(kp_pk,eps)<0.10 && dhp_s~=dhm_s));

clear global VDF_OVERRIDE KNOWN_DIST
save(fullfile(fileparts(mfilename('fullpath')),'..','Datasets','MultiInit','kappa_validate.mat'), ...
     'dA','mags','kstar','CFG','Fx','ton','toff');
fprintf('\nSaved -> Datasets/MultiInit/kappa_validate.mat\n');

% ============================= local functions ===============================
function d = onerun(cfgOv, force, ton, toff)
    global VDF_OVERRIDE KNOWN_DIST          %#ok<GVMIS>
    VDF_OVERRIDE = cfgOv;
    KNOWN_DIST = struct('force',force(:),'t_on',ton,'t_off',toff);
    x0 = [0;0;-5; 1;0;0;0; zeros(3,1); zeros(3,1)];     % Static, centered IC
    co = struct('NOISE',1,'GE',1,'delay',1);            % NOISE=1 -> robust plant (force enters here)
    r  = onerun_sim(x0, co);
    d  = r.data; d.landed = r.success; d.idx = r.data.idx;
end
function r = onerun_sim(x0, co)
    r = run_simulation(x0, "Static", [], 1.0, co, 1);
end
function [t,k,dh,sg,F,P] = unpack(d, ax)
    idx = d.idx; t = d.tRange(1:idx); t = t(:);
    k = d.kappa_log(ax,1:idx)';  dh = d.d_h_log(ax,1:idx)';
    sg = d.sigma(ax,1:idx)';      F = d.F_known_log(ax,1:idx)';
    P = d.P;
end
function s = tf(b), if b, s='PASS'; else, s='--fail'; end, end
