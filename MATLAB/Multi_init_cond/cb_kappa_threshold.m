%% CB_KAPPA_THRESHOLD  Map kappa's engagement threshold w.r.t. a KNOWN disturbance.
%   Builds on cb_kappa_validate's testbed (Static + KNOWN_DIST injection). Answers:
%     D  FORCE SWEEP  : how big must the known force be for |sigma|/E>=1 so kappa
%                       SWITCHES (adapts up), not just acts as a linear gain?
%     E  E COMPARISON : does a tighter boundary layer E engage kappa at LOWER force?
%     F  D vs PX4     : same known step -- does the bootstrapped-kappa0 PX4 config
%                       reject the sudden disturbance with less error than D?
%
%   kappa "engaged" := max|sigma_x|/E_x >= 1 (escapes boundary layer) AND
%                      kappa_x on-peak > kappa_x pre + 0.02 (genuinely adapts up).
%
% Run:  cd MATLAB/Multi_init_cond; cb_kappa_threshold
clc; clear;
mfile_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(mfile_dir, '..', 'Common'));
clear mfile_dir;

D    = struct('theta_per_axis',true,'kappa0',[0.05;0.05;0.05],'N',diag([0.10,0.10,0.10]));
PX4  = struct('theta_per_axis',true,'kappa0',[0.5;0.5;1.0],'N',diag([0.10,0.10,0.10]), ...
              'Gamma',diag([2.0,2.0,1.0]),'E',diag([0.8,0.8,0.5]));
AX = 1; ton = 2.0; toff = 6.0;

% ---------------- D: FORCE SWEEP (config D) ----------------
forces = [0.6, 1.5, 3.0, 5.0, 8.0];
fprintf('\n========== D: FORCE SWEEP (config D, step [%.0f,%.0f]) ==========\n', ton, toff);
fprintf('  Fx(N) | max|s|/E | k_pre  k_peak | max|h_e_x| | landed | ENGAGED\n');
for F = forces
    m = probe(D, [F;0;0], ton, toff, AX);
    fprintf('  %4.1f  |  %6.2f  | %.3f  %.3f | %8.3f   |   %d    | %s\n', ...
        F, m.soe, m.kpre, m.kpk, m.hemax, m.landed, eng(m));
end

% ---------------- E: BOUNDARY-LAYER COMPARISON (fixed force) ----------------
Ffix = 1.5;  Es = [1.0, 0.5, 0.25];
fprintf('\n========== E: E_x COMPARISON (config D, Fx=%.1f N) ==========\n', Ffix);
fprintf('  E_x  | max|s|/E | k_pre  k_peak | max|h_e_x| | landed | ENGAGED\n');
for Ex = Es
    cfg = D; cfg.E = diag([Ex,Ex,0.5]);
    m = probe(cfg, [Ffix;0;0], ton, toff, AX);
    fprintf('  %4.2f |  %6.2f  | %.3f  %.3f | %8.3f   |   %d    | %s\n', ...
        Ex, m.soe, m.kpre, m.kpk, m.hemax, m.landed, eng(m));
end

% ---------------- F: D vs PX4 REJECTION (force that engages) ----------------
Frej = 3.0;
fprintf('\n========== F: D vs PX4 rejection of a known %.1f N step ==========\n', Frej);
fprintf('  config | k_pre  k_peak | max|h_e_x| (excursion) | settle|h_e|<.1 after? | landed\n');
for cc = {{'D',D},{'PX4',PX4}}
    nm = cc{1}{1}; cfg = cc{1}{2};
    m = probe(cfg, [Frej;0;0], ton, toff, AX);
    fprintf('  %-5s  | %.3f  %.3f |       %8.3f        |        %s         |   %d\n', ...
        nm, m.kpre, m.kpk, m.hemax, tf(m.settled), m.landed);
end
fprintf('\n(PX4 bet: bootstrapped kappa0 -> smaller error excursion / faster settle on the sudden step)\n');

clear global VDF_OVERRIDE KNOWN_DIST

% ============================= local functions ===============================
function m = probe(cfgOv, force, ton, toff, ax)
    global VDF_OVERRIDE KNOWN_DIST          %#ok<GVMIS>
    VDF_OVERRIDE = cfgOv;
    KNOWN_DIST = struct('force',force(:),'t_on',ton,'t_off',toff);
    x0 = [0;0;-5; 1;0;0;0; zeros(3,1); zeros(3,1)];
    co = struct('NOISE',1,'GE',1,'delay',1);
    r = run_simulation(x0, "Static", [], 1.0, co, 1);
    d = r.data; idx = d.idx; P = d.P;
    t  = d.tRange(1:idx); t = t(:);
    k  = d.kappa_log(ax,1:idx)';   sg = d.sigma(ax,1:idx)';
    he = abs(d.V_h_e(ax,1:idx))';  F  = d.F_known_log(ax,1:idx)';
    on = F~=0;
    Ex = P.E(ax,ax);
    m.soe   = max(abs(sg(on))/Ex);
    m.kpre  = mean(k(t<ton & t>ton-0.5));
    m.kpk   = max(k(on));
    m.hemax = max(he(on));
    % settled = |h_e| back under 0.1 by end of the on-phase
    onIdx = find(on); m.settled = ~isempty(onIdx) && he(onIdx(end)) < 0.1;
    m.landed = r.success;
end
function s = eng(m), if m.soe>=1 && m.kpk>m.kpre+0.02, s='YES'; else, s='no'; end, end
function s = tf(b), if b, s='YES'; else, s='no '; end, end
