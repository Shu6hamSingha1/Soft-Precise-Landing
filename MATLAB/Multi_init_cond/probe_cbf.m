% Confirm whether the CBF QP (alternating projection onto the FoV box) is
% active early in seed 6, cleanly separated from the theta_cap deliverable cap.
% cbf2_filter logs [theta_d(2); theta_qp(2); theta_safe(2); theta_cap] per call:
%   QP movement  = |theta_qp - theta_d|   (0 => projection identity => QP passive)
%   cap movement = |theta_safe - theta_qp|(>0 => deliverable cap clipped)
%   run probe_cbf   (set global PROBE_SEED)

clear -regexp '^(?!IC_OVERRIDE|NOISE_OVERRIDE|RNG_SEED_OVERRIDE|SEN_.*|THETA_.*|FOV_.*|PROBE_SEED|CBF_DBG_.*).*$';
clc;
global IC_OVERRIDE NOISE_OVERRIDE RNG_SEED_OVERRIDE PROBE_SEED CBF_DBG_ON CBF_DBG_LOG;
if isempty(PROBE_SEED); PROBE_SEED = 6; end
IC_OVERRIDE = [2;2;-3]; NOISE_OVERRIDE = 1; RNG_SEED_OVERRIDE = PROBE_SEED;
CBF_DBG_ON = true; CBF_DBG_LOG = [];

visualControl_IBVS_adaptive;

global PROBE_SEED CBF_DBG_LOG;
L = CBF_DBG_LOG;                     % 7 x Ncalls
n = size(L,2);
th_d  = L(1:2,:); th_qp = L(3:4,:); th_sf = L(5:6,:); tcap = L(7,:);
qp_mov  = vecnorm(th_qp - th_d);    % how much the QP projection moved the lean
cap_mov = vecnorm(th_sf - th_qp);   % how much the deliverability cap moved it
qp_n    = vecnorm(th_qp);           % |lean| (vs tcap)

fprintf('\n==== seed %d : CBF QP vs theta_cap (n=%d calls) ====\n', PROBE_SEED, n);
fprintf(' deg conversions; tcap=%.0f deg.  Early window (first ~1.65s):\n', rad2deg(tcap(1)));
fprintf(' step | |th_d|deg |th_qp|deg | QP_moved(deg) | cap_moved(deg)\n');
kmax = min(n, 165); step = 10;
for k = 1:step:kmax
  fprintf(' %4d |  %6.2f    %6.2f    |   %7.4f     |   %7.4f\n', ...
    k, rad2deg(vecnorm(th_d(:,k))), rad2deg(qp_n(k)), rad2deg(qp_mov(k)), rad2deg(cap_mov(k)));
end
fprintf('--- early window [1..%d] ---\n', kmax);
fprintf('  QP  active (|moved|>0.01 deg): %d / %d steps, max move %.4f deg\n', ...
  sum(qp_mov(1:kmax) > deg2rad(0.01)), kmax, rad2deg(max(qp_mov(1:kmax))));
fprintf('  cap active (|moved|>0.01 deg): %d / %d steps, max move %.4f deg\n', ...
  sum(cap_mov(1:kmax) > deg2rad(0.01)), kmax, rad2deg(max(cap_mov(1:kmax))));
fprintf('  max |th_qp| in window = %.2f deg (cap = %.0f deg)\n', rad2deg(max(qp_n(1:kmax))), rad2deg(tcap(1)));
fprintf('--- whole run [1..%d] ---\n', n);
fprintf('  QP  active: %d / %d steps (first at step %d)\n', sum(qp_mov>deg2rad(0.01)), n, find(qp_mov>deg2rad(0.01),1,'first'));
fprintf('  cap active: %d / %d steps (first at step %d)\n', sum(cap_mov>deg2rad(0.01)), n, find(cap_mov>deg2rad(0.01),1,'first'));
