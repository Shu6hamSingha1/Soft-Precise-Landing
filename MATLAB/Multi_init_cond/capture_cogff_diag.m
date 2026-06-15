% Capture full traces for the 2 non-SP IC5 seeds (4=hard-land, 6=CBF-strip TL)
% at gamma=0 (baseline) and gamma=0.005 (CoG-FF), to diagnose WHY they're non-SP.
%   run capture_cogff_diag
clear -regexp '^(?!WS)$'; close all; clc;
global IC_OVERRIDE NOISE_OVERRIDE RNG_SEED_OVERRIDE GAMMA_COG WS;
WS = struct(); WS.gs = {0, 0.005}; WS.seeds = [4 6];
for gi = 1:numel(WS.gs)
  for si = 1:numel(WS.seeds)
    global IC_OVERRIDE NOISE_OVERRIDE RNG_SEED_OVERRIDE GAMMA_COG WS;
    sd = WS.seeds(si); gg = WS.gs{gi}; WS.gi=gi; WS.si=si; WS.sd=sd; WS.gg=gg;
    IC_OVERRIDE=[2;2;-3]; NOISE_OVERRIDE=1; RNG_SEED_OVERRIDE=sd; GAMMA_COG=gg;
    fprintf('\n=== seed %d gamma %.3f ===\n', sd, gg);
    visualControl_IBVS_adaptive;
    global WS; gi=WS.gi; si=WS.si; sd=WS.sd; gg=WS.gg;   % restore (controller cleared locals)
    D = struct();
    D.t = tRange(1:idx);
    D.pos = X_DS(1:3, 1:idx); D.vel = X_DS(8:10, 1:idx); D.omega = X_DS(11:13, 1:idx);
    D.quat = X_DS(4:7, 1:idx);
    D.s_e_n = V_s_e_n(:,1:idx); D.p_s = p_s(:,1:idx); D.sen_breach = sen_breach(:,1:idx);
    D.h_e = V_h_e(:,1:idx); D.p_2 = p_2(:,1:idx);
    D.kappa = kappa(:,1:idx); D.thetahat = THETAHAT_COG(:,1:idx);
    D.eR = D_DS(7:9,1:idx); D.tau = D_DS(10:12,1:idx); D.T = B_T_cd(1:idx);
    D.x_t = x_t(:,1:idx); D.dx_t = dx_t(:,1:idx);
    D.idx = idx; D.landed = double(landed); D.fov = double(fov_fail);
    D.xy = norm(I_p_c(1:2)-x_t(1:2,idx)); D.vrel = norm(I_v_c-dx_t(1:3,idx));
    fn = sprintf('/tmp/cogff_diag_g%g_s%d.mat', gg, sd);
    save(fn, '-struct', 'D');
    fprintf('  saved %s | land=%d fov=%d xy=%.3f vel=%.3f t=%.2f\n', ...
      fn, D.landed, D.fov, D.xy, D.vrel, D.t(end));
    global WS;   % restore after controller clear
  end
end
fprintf('\nDONE capture\n');
