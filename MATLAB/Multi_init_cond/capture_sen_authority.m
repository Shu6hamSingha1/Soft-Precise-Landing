% Capture the SEN-funnel outer-loop arrays to quantify per-PID-term control
% authority over s_e_n. Saves zeta_s, S_s, izeta_s, G_s, p_s, dp_s, V_s_e_n + gains
% for a BREACHING case (IC5 seed6) and a CLEAN case (IC1 seed1). Authority of each
% term on the demanded feature-rate V_ds_d = G_s^-1 * dzeta_sd + S_s*dp_s is then
% computed post-hoc (analyze_sen_authority.py): G_s^-1 = (p_s_eff/2)(1-S_s^2).
%   run capture_sen_authority
clear -regexp '^(?!WS)$'; close all; clc;
global IC_OVERRIDE NOISE_OVERRIDE RNG_SEED_OVERRIDE WS;
WS = struct();
WS.cases = {{[2;2;-3], 6, 'ic5_s6_breach'}, {[0;0;-5], 1, 'ic1_s1_clean'}};
for k = 1:numel(WS.cases)
  global IC_OVERRIDE NOISE_OVERRIDE RNG_SEED_OVERRIDE WS;
  cse = WS.cases{k}; WS.k = k; WS.cse = cse;
  IC_OVERRIDE = cse{1}; NOISE_OVERRIDE = 1; RNG_SEED_OVERRIDE = cse{2};
  fprintf('\n=== %s ===\n', cse{3});
  visualControl_IBVS_adaptive;
  global WS; cse = WS.cse; k = WS.k;   % restore (controller cleared locals)
  D = struct();
  D.t   = tRange(1:idx);
  D.Ss  = [squeeze(S_s(1,1,1:idx))'; squeeze(S_s(2,2,1:idx))'];   % 2 x idx
  D.zeta = zeta_s(:,1:idx);
  D.izeta = izeta_s(:,1:idx);
  D.Ginv = [squeeze(1./G_s(1,1,1:idx))'; squeeze(1./G_s(2,2,1:idx))'];
  D.p_s = p_s(:,1:idx); D.dp_s = dp_s(:,1:idx);
  D.s_e_n = V_s_e_n(:,1:idx); D.breach = sen_breach(:,1:idx);
  D.rp = diag(K_ctrl.rp)'; D.ri = diag(K_ctrl.ri)'; D.rd = diag(K_ctrl.rd)';
  D.p_10 = K_ctrl.p_10(1:2)'; D.dt = dt;
  D.idx = idx; D.land = double(landed); D.fov = double(fov_fail);
  global WS;
  fn = sprintf('/tmp/sen_auth_%s.mat', cse{3});
  save(fn, '-struct', 'D');
  fprintf('  saved %s | land=%d fov=%d max|Ss|=%.3f\n', fn, D.land, D.fov, max(abs(D.Ss(:))));
end
fprintf('\nDONE\n');
