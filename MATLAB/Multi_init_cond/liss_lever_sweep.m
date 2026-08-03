%% Lissajous 1.4x lateral-precision lever sweep (per-axis ON, realistic, seed 1)
% Diagnosis: terminal y-axis chase-lag (one-sided, grows through descent).
% Levers: p_hinf(1:2) DOWN (tighter feature-funnel floor = less chase-lag),
%         Gamma_xy UP (faster surface convergence), chi_r UP (harder barrier).
% Goal: push Lissajous 1.4x xy well under 0.08 with margin, no terminal blow-up.
close all;
mfile_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(mfile_dir, '..', 'Common'));
addpath(fullfile(mfile_dir, '..', 'VDF_ASMC'));

global VDF_OVERRIDE P2INF_XY_OVERRIDE GAMMA_XY_OVERRIDE CHI_R_OVERRIDE   %#ok<GVMIS>
VDF_OVERRIDE = struct('theta_per_axis', true);
x0 = [0;0;-5; 1;0;0;0; zeros(6,1)];
cfg = struct('NOISE',1,'GE',1,'delay',1);
runliss = @() run_simulation(x0, 'Lissajous', [], 1.4, cfg, 1);
clr = @() assignin('base','dummy',0);  % no-op

reset = @() deal([],[],[]);
[P2INF_XY_OVERRIDE, GAMMA_XY_OVERRIDE, CHI_R_OVERRIDE] = reset();
r = runliss();
fprintf('BASE (p_hinf .5 / Gamma .44/.5 / chi_r 1.15): xy=%.4f v=%.3f t=%.2f SP=%d\n', ...
        r.final_xy, r.final_rel_vel, r.final_t, (r.success&&r.precise&&r.soft));

fprintf('\n-- p_hinf(1:2) UP (looser feature funnel) --\n');
for ph = [0.60 0.70 0.80 1.00]
    [P2INF_XY_OVERRIDE, GAMMA_XY_OVERRIDE, CHI_R_OVERRIDE] = reset();
    P2INF_XY_OVERRIDE = ph;
    r = runliss();
    fprintf('  p_hinf=%.2f: xy=%.4f v=%.3f t=%.2f SP=%d\n', ph, r.final_xy, r.final_rel_vel, r.final_t, (r.success&&r.precise&&r.soft));
end

fprintf('\n-- Gamma_xy DOWN --\n');
for g = [0.30 0.35 0.40]
    [P2INF_XY_OVERRIDE, GAMMA_XY_OVERRIDE, CHI_R_OVERRIDE] = reset();
    GAMMA_XY_OVERRIDE = g;
    r = runliss();
    fprintf('  Gamma_xy=%.2f: xy=%.4f v=%.3f t=%.2f SP=%d\n', g, r.final_xy, r.final_rel_vel, r.final_t, (r.success&&r.precise&&r.soft));
end

fprintf('\n-- chi_r UP (2-vec; cautious, 1.2 regressed Liss-IC3) --\n');
for c = [1.25 1.35 1.50]
    [P2INF_XY_OVERRIDE, GAMMA_XY_OVERRIDE, CHI_R_OVERRIDE] = reset();
    CHI_R_OVERRIDE = [c; c];
    r = runliss();
    fprintf('  chi_r=%.2f: xy=%.4f v=%.3f t=%.2f SP=%d\n', c, r.final_xy, r.final_rel_vel, r.final_t, (r.success&&r.precise&&r.soft));
end

[P2INF_XY_OVERRIDE, GAMMA_XY_OVERRIDE, CHI_R_OVERRIDE] = reset();
VDF_OVERRIDE = [];
