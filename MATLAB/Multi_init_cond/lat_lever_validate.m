%% Validate a lateral-precision lever change across the full realistic set
%  (multi_init 5 traj x 5 IC + multi_speed 4 traj x 5 mult), per-axis ON, seed 1.
%  Baseline (baked) is known 25/25 + 20/20; this flags ANY cell that drops below SP
%  and reports the worst xy per group, so a lateral change can be accepted only if it
%  holds the canonical sets while improving the Lissajous high-speed cell.
%
%  Edit the override block to test a candidate.
close all;
mfile_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(mfile_dir, '..', 'Common'));
addpath(fullfile(mfile_dir, '..', 'VDF_ASMC'));

global VDF_OVERRIDE CHI_R_OVERRIDE P2INF_XY_OVERRIDE GAMMA_XY_OVERRIDE   %#ok<GVMIS>
VDF_OVERRIDE = struct('theta_per_axis', true);
% ---- CANDIDATE under test (overrides EMPTY = test the BAKED vdf_params gains) ----
CHI_R_OVERRIDE    = [];
P2INF_XY_OVERRIDE = [];
GAMMA_XY_OVERRIDE = [];
SEED = 2;                           % second-seed robustness check
fprintf('CANDIDATE: chi_r=%s p_hinf=%s Gamma_xy=%s\n', ...
        mat2str(CHI_R_OVERRIDE(:)'), mat2str(P2INF_XY_OVERRIDE), mat2str(GAMMA_XY_OVERRIDE));

cfg = struct('NOISE',1,'GE',1,'delay',1);   % realistic
q0 = [1;0;0;0]; z6 = zeros(6,1);

% ---- multi_init: 5 traj x 5 IC ----
trajI = ["Static","Linear","Sinusoidal","Lissajous","Circular"];
p0 = [0,0,-5; 2,2,-5; 2,-2,-5; 2,2,-7; 2,2,-3];
fprintf('\n== multi_init (realistic, seed 1) ==\n');
nSPi = 0; worstI = 0; failI = {};
for t = 1:numel(trajI)
    for k = 1:size(p0,1)
        r = run_simulation([p0(k,:)';q0;z6], trajI(t), [], 1.0, cfg, SEED);
        sp = r.success && r.precise && r.soft; nSPi = nSPi + sp;
        worstI = max(worstI, r.final_xy);
        if ~sp, failI{end+1} = sprintf('%s-IC%d(xy=%.3f,v=%.3f,p=%d,s=%d)', trajI(t),k,r.final_xy,r.final_rel_vel,r.precise,r.soft); end %#ok<SAGROW>
    end
end
fprintf('  multi_init: %d/25 SP, worst xy=%.4f\n', nSPi, worstI);
if ~isempty(failI), fprintf('  FAILS: %s\n', strjoin(failI, ' | ')); end

% ---- multi_speed: 4 moving traj x 5 mult, single IC [0,0,-5] ----
trajS = ["Linear","Sinusoidal","Lissajous","Circular"];
mults = [0.6 0.8 1.0 1.2 1.4];
fprintf('\n== multi_speed (realistic, seed 1) ==\n');
nSPs = 0; worstS = 0; failS = {}; lissWorst = 0;
for t = 1:numel(trajS)
    for m = 1:numel(mults)
        r = run_simulation([0;0;-5;q0;z6], trajS(t), [], mults(m), cfg, SEED);
        sp = r.success && r.precise && r.soft; nSPs = nSPs + sp;
        worstS = max(worstS, r.final_xy);
        if trajS(t)=="Lissajous" && mults(m)==1.4, lissWorst = r.final_xy; end
        if ~sp, failS{end+1} = sprintf('%s-%.1fx(xy=%.3f,v=%.3f,p=%d,s=%d)', trajS(t),mults(m),r.final_xy,r.final_rel_vel,r.precise,r.soft); end %#ok<SAGROW>
    end
end
fprintf('  multi_speed: %d/20 SP, worst xy=%.4f | Lissajous 1.4x xy=%.4f\n', nSPs, worstS, lissWorst);
if ~isempty(failS), fprintf('  FAILS: %s\n', strjoin(failS, ' | ')); end

fprintf('\n==== TOTAL: %d/45 SP (baseline 45/45). Lissajous 1.4x: %.4f (baked 0.0758) ====\n', nSPi+nSPs, lissWorst);
CHI_R_OVERRIDE=[]; P2INF_XY_OVERRIDE=[]; GAMMA_XY_OVERRIDE=[]; VDF_OVERRIDE=[];
