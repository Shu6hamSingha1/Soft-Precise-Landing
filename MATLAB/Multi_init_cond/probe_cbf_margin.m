% When the CBF QP clips the -Y lean, does that -Y lean IMPROVE or HURT the
% binding corner's margin?  cbf2_filter logs per call:
%  [axy_d_y; axy_qp_y; marg_curr; marg_d; marg_qp; bind_val; bind_m2; negY_sens; justified]
% justified=+1: -Y pushes the binding corner toward the edge (clip correct).
% justified=-1: -Y pulls the binding corner toward centre (clip is counterproductive).
%   run probe_cbf_margin   (set global PROBE_SEED)

clear -regexp '^(?!IC_OVERRIDE|NOISE_OVERRIDE|RNG_SEED_OVERRIDE|SEN_.*|THETA_.*|FOV_.*|PROBE_SEED|CBF_DBG_.*).*$';
clc;
global IC_OVERRIDE NOISE_OVERRIDE RNG_SEED_OVERRIDE PROBE_SEED CBF_DBG_ON CBF_DBG_LOG;
if isempty(PROBE_SEED); PROBE_SEED = 6; end
IC_OVERRIDE = [2;2;-3]; NOISE_OVERRIDE = 1; RNG_SEED_OVERRIDE = PROBE_SEED;
CBF_DBG_ON = true; CBF_DBG_LOG = [];

visualControl_IBVS_adaptive;

global PROBE_SEED CBF_DBG_LOG;
L = CBF_DBG_LOG; n = size(L,2);
axy_d_y=L(1,:); axy_qp_y=L(2,:); mc=L(3,:); md=L(4,:); mq=L(5,:);
bind=L(6,:); bm2=L(7,:); sens=L(8,:); just=L(9,:);
dAy = axy_qp_y - axy_d_y;
clip = dAy > 1e-6;                       % steps where QP removed -Y

fprintf('\n==== seed %d : is the clipped -Y improving corner margin? (n=%d) ====\n', PROBE_SEED, n);
fprintf(' step | axy_d_y axy_qp_y dAy | marg_curr marg_d marg_qp | bind/m2  negY_sens just\n');
kmax=min(n,165); step=10;
for k=1:step:kmax
  fprintf(' %4d | %6.2f %7.2f %5.2f | %8.3f %7.3f %7.3f | %+5.2f/%.2f %+7.3f  %+d\n', ...
    k, axy_d_y(k), axy_qp_y(k), dAy(k), mc(k), md(k), mq(k), bind(k), bm2(k), sens(k), just(k));
end
w = 1:kmax;
cw = w(clip(w));                          % clip steps in early window
fprintf('--- early window [1..%d], clip steps=%d ---\n', kmax, numel(cw));
fprintf('  of clip steps: JUSTIFIED(-Y pushes corner OUT) %d  |  HARMFUL(-Y toward centre) %d\n', ...
  sum(just(cw)>0), sum(just(cw)<0));
fprintf('  mean marg_curr=%.3f  marg_d(desired)=%.3f  marg_qp(clipped)=%.3f\n', ...
  mean(mc(cw)), mean(md(cw)), mean(mq(cw)));
fprintf('  steps where desired lean IMPROVES margin vs current (md>mc): %d/%d\n', ...
  sum(md(cw)>mc(cw)), numel(cw));
fprintf('  steps where desired lean is FEASIBLE (md>=0) yet QP still clipped: %d/%d\n', ...
  sum(md(cw)>=0), numel(cw));
