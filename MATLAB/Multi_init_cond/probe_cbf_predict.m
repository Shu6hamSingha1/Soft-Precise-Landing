% Why didn't the CBF stop the corner at inset=0?  The CBF constrains the
% PREDICTED corner (one-step L_w linearization + estimated drift) to the box;
% the REALIZED corner differs by linearization + attitude-slew lag + noise.
% Compare per step: CBF predicted margin (mq>=0 => CBF says corner is inside the
% box) vs the realized corner edge ratio from P_DS (>1 => actually out of FoV).
% cbf2 debug rows: [axy_d_y;axy_qp_y; mc; md; mq; bf; m2; negY_sens; just]
%   run probe_cbf_predict   (set global PROBE_SEED; uses inset=0)

clear -regexp '^(?!IC_OVERRIDE|NOISE_OVERRIDE|RNG_SEED_OVERRIDE|PROBE_SEED|FOV_INSET_OVERRIDE|CBF_DBG_.*).*$';
clc;
global IC_OVERRIDE NOISE_OVERRIDE RNG_SEED_OVERRIDE PROBE_SEED FOV_INSET_OVERRIDE CBF_DBG_ON CBF_DBG_LOG;
if isempty(PROBE_SEED); PROBE_SEED = 2; end
IC_OVERRIDE = [2;2;-3]; NOISE_OVERRIDE = 1; RNG_SEED_OVERRIDE = PROBE_SEED;
FOV_INSET_OVERRIDE = 0; CBF_DBG_ON = true; CBF_DBG_LOG = [];

visualControl_IBVS_adaptive;

global PROBE_SEED CBF_DBG_LOG;
ni = idx; t = tRange(1:ni);
L = CBF_DBG_LOG; nL = size(L,2);
mq = L(5,:);                                  % CBF predicted worst-corner margin (tangent)
m2 = L(7,:);                                  % box half-width (tangent)
pred_ratio = 1 - mq ./ max(m2,1e-9);          % predicted worst-corner /box (<=1 => CBF says inside)
% realized corner edge ratio from P_DS (C_nP cols 9:12), edges [160;120]px
nr = min(ni,nL);
real_ratio = zeros(1,nr);
for k=1:nr
  C = P_DS(:,9:12,k);
  real_ratio(k) = max(max(abs(C(1,:))/160, max(abs(C(2,:))/120)));
end
fprintf('\n==== seed %d inset=0 : CBF predicted vs realized corner (fov=%d) ====\n', PROBE_SEED, fov_fail);
fprintf(' pred_ratio = CBF predicted worst corner / box (<=1 = CBF holds it inside)\n');
fprintf(' real_ratio = realized worst corner / FoV edge (>1 = ACTUALLY out -> fov_fail)\n');
fprintf('  t   | cbf_ok | pred_ratio | real_ratio | gap(real-pred)\n');
for k = 1:nr
  fprintf('%5.2f |   %d    |   %5.3f    |   %5.3f    |  %+.3f\n', ...
    t(k), cbf_ok_log(k), pred_ratio(k), real_ratio(k), real_ratio(k)-pred_ratio(k));
end
fprintf('--- CBF held predicted<=1 on %d/%d steps ; realized crossed 1 on %d/%d ; mean gap=%.3f ---\n', ...
  sum(pred_ratio(1:nr)<=1.0), nr, sum(real_ratio(1:nr)>1.0), nr, mean(real_ratio(1:nr)-pred_ratio(1:nr)));
