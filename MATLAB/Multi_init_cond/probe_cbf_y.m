% Does the CBF QP specifically clip the -Y lean (remove the lateral correction
% the drone needs)?  cbf2_filter logs the desired vs post-QP INERTIAL lateral
% accel: rows [axy_d(1:2); axy_qp(1:2); axy_sf(1:2); a_z].
%   dAy = axy_qp(2) - axy_d(2)  > 0  => QP made -Y accel LESS negative = clipped -Y.
% To close +Y offset we want axy_y < 0; if the QP pushes it toward 0/positive it
% is removing the correction.  Compares seed 6 (fails) vs seed 1 (works).
%   run probe_cbf_y   (set global PROBE_SEED)

clear -regexp '^(?!IC_OVERRIDE|NOISE_OVERRIDE|RNG_SEED_OVERRIDE|SEN_.*|THETA_.*|FOV_.*|PROBE_SEED|CBF_DBG_.*).*$';
clc;
global IC_OVERRIDE NOISE_OVERRIDE RNG_SEED_OVERRIDE PROBE_SEED CBF_DBG_ON CBF_DBG_LOG;
if isempty(PROBE_SEED); PROBE_SEED = 6; end
IC_OVERRIDE = [2;2;-3]; NOISE_OVERRIDE = 1; RNG_SEED_OVERRIDE = PROBE_SEED;
CBF_DBG_ON = true; CBF_DBG_LOG = [];

visualControl_IBVS_adaptive;

global PROBE_SEED CBF_DBG_LOG;
L = CBF_DBG_LOG; n = size(L,2);
axy_d = L(1:2,:); axy_qp = L(3:4,:);
dAx = axy_qp(1,:) - axy_d(1,:);
dAy = axy_qp(2,:) - axy_d(2,:);

fprintf('\n==== seed %d : does CBF QP clip -Y? (n=%d) ====\n', PROBE_SEED, n);
fprintf(' want axy_y<0 (accel -Y).  dAy>0 => QP removed -Y correction.\n');
fprintf(' step | axy_d_y  axy_qp_y |  dAy   |  axy_d_x  axy_qp_x   dAx\n');
kmax = min(n,165); step=10;
for k=1:step:kmax
  fprintf(' %4d | %7.2f  %7.2f  | %6.2f | %7.2f  %7.2f  %6.2f\n', ...
    k, axy_d(2,k), axy_qp(2,k), dAy(k), axy_d(1,k), axy_qp(1,k), dAx(k));
end
fprintf('--- early window [1..%d] means ---\n', kmax);
fprintf('  axy_d_y=%.2f  axy_qp_y=%.2f   mean dAy=%+.3f (>0 = QP removes -Y)\n', ...
  mean(axy_d(2,1:kmax)), mean(axy_qp(2,1:kmax)), mean(dAy(1:kmax)));
fprintf('  axy_d_x=%.2f  axy_qp_x=%.2f   mean dAx=%+.3f\n', ...
  mean(axy_d(1,1:kmax)), mean(axy_qp(1,1:kmax)), mean(dAx(1:kmax)));
fprintf('  steps QP pushed Y toward +(dAy>0): %d/%d ; toward -(dAy<0): %d/%d\n', ...
  sum(dAy(1:kmax)>0), kmax, sum(dAy(1:kmax)<0), kmax);
fprintf('  mean |dAy|=%.3f  mean |dAx|=%.3f  (which axis the QP fights more)\n', ...
  mean(abs(dAy(1:kmax))), mean(abs(dAx(1:kmax))));
