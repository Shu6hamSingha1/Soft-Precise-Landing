% Why does the Y channel fail to reach early in a given seed?  Dumps the early
% (first ~1.6 s) Y-channel chain at fine resolution so seed 6 (fails) can be
% compared to seed 1 (works):
%   V_s_y            image feature (Y)
%   V_h_d_y          DESIRED optic flow (Y) = V_ds_d + cross(V_w,V_s) + ...
%   V_h_i_y          ACTUAL optic flow (Y)
%   vhe_y            error = V_h - V_h_d  (SMC must drive ->0)
%   V_w=[wx wy wz]   target-rel angular vel from optic flow (cross-coupling src)
%   crossY           wz*sx - wx  (the Y cross-coupling injected into V_h_d)
%   sigma_y, kappa_y SMC sliding var + adaptive gain
% V_X_DS(:,k) = [V_s_i(1:2); V_s_i(4); V_h_i(1:3); V_w_i(1:3); V_dw_i(1:3); ...]
%   run probe_ychain   (set global PROBE_SEED)

clear -regexp '^(?!IC_OVERRIDE|NOISE_OVERRIDE|RNG_SEED_OVERRIDE|SEN_.*|THETA_.*|FOV_.*|PROBE_SEED).*$';
clc;
global IC_OVERRIDE NOISE_OVERRIDE RNG_SEED_OVERRIDE PROBE_SEED;
if isempty(PROBE_SEED); PROBE_SEED = 6; end
IC_OVERRIDE = [2;2;-3]; NOISE_OVERRIDE = 1; RNG_SEED_OVERRIDE = PROBE_SEED;

visualControl_IBVS_adaptive;

global PROBE_SEED;
ni = idx;
t  = tRange(1:ni);
sx = V_X_DS(1,1:ni);  sy = V_X_DS(2,1:ni);
vhiy = V_X_DS(5,1:ni);                      % actual optic flow Y (V_h_i)
wx = V_X_DS(7,1:ni); wy = V_X_DS(8,1:ni); wz = V_X_DS(9,1:ni);
vhdy = V_h_d(2,1:ni); vhey = V_h_e(2,1:ni);
crossY = wz.*sx - wx;                        % Y-component of cross(V_w,V_s(1:3))
sig_y = sigma(2,1:ni);  kap_y = kappa(2,1:ni);
seny = V_s_e_n(2,1:ni); psy = p_s(2,1:ni);

fprintf('\n==== seed %d : Y-channel early chain (fov=%d t_end=%.2f) ====\n', PROBE_SEED, fov_fail, t(end));
fprintf('  t   | sy    Yvel? | V_h_d_y V_h_i_y vhe_y | wx     wy     wz    crossY | sig_y  kap_y | sen_y p_s\n');
kmax = min(ni, round(1.65/(t(2)-t(1))));
step = max(1, round(0.10/(t(2)-t(1))));
Yvel = X_DS(9, 2:ni+1);
for k = 1:step:kmax
  fprintf('%5.2f | %5.2f %5.2f | %6.2f %6.2f %6.2f | %6.2f %6.2f %6.2f %6.2f | %6.2f %5.2f | %5.2f %.2f\n', ...
    t(k), sy(k), Yvel(k), vhdy(k), vhiy(k), vhey(k), wx(k), wy(k), wz(k), crossY(k), sig_y(k), kap_y(k), seny(k), psy(k));
end
% summary stats over the early window
fprintf('--- early window [0,%.2f] stats ---\n', t(kmax));
fprintf('  |V_h_d_y|: mean=%.2f max=%.2f   |vhe_y|: mean=%.2f   |V_w|: mean=%.2f max=%.2f\n', ...
  mean(abs(vhdy(1:kmax))), max(abs(vhdy(1:kmax))), mean(abs(vhey(1:kmax))), ...
  mean(vecnorm(V_X_DS(7:9,1:kmax))), max(vecnorm(V_X_DS(7:9,1:kmax))));
fprintf('  |crossY|: mean=%.2f max=%.2f\n', mean(abs(crossY(1:kmax))), max(abs(crossY(1:kmax))));
