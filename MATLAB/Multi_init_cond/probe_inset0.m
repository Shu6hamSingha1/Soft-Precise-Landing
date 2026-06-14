% inset=0 diagnosis. For a given seed: trace the Y-channel + breach ratio r AND
% the corner-vs-FoV-edge ratios (fov_fail if >1). Explains (1) why seed 6 no
% longer breaches and (2) why the good seeds fov_fail despite no breach.
% res=[320;240] -> edges [160;120]px. CBF box at inset0 = the fov_fail edge.
%   run probe_inset0   (set global PROBE_SEED)

clear -regexp '^(?!IC_OVERRIDE|NOISE_OVERRIDE|RNG_SEED_OVERRIDE|PROBE_SEED|FOV_INSET_OVERRIDE).*$';
clc;
global IC_OVERRIDE NOISE_OVERRIDE RNG_SEED_OVERRIDE PROBE_SEED FOV_INSET_OVERRIDE;
if isempty(PROBE_SEED); PROBE_SEED = 6; end
IC_OVERRIDE = [2;2;-3]; NOISE_OVERRIDE = 1; RNG_SEED_OVERRIDE = PROBE_SEED;
FOV_INSET_OVERRIDE = 0;

visualControl_IBVS_adaptive;

global PROBE_SEED;
ni = idx; t = tRange(1:ni);
sy = V_X_DS(2,1:ni); Yvel = X_DS(9,2:ni+1);
seny = V_s_e_n(2,1:ni); psy = p_s(2,1:ni); r = abs(seny)./max(psy,1e-6);
uer = zeros(1,ni); ver = zeros(1,ni);            % corner edge ratios (>1 = fov_fail)
for k=1:ni
  C = P_DS(:,9:12,k);
  uer(k) = max(abs(C(1,:)))/160; ver(k) = max(abs(C(2,:)))/120;
end
xy_end = norm(X_DS(1:2,ni+1) - x_t(1:2,ni));
vel_end= norm(X_DS(8:10,ni+1) - dx_t(1:3,ni));
fprintf('\n==== seed %d inset=0 : fov=%d landed=%d t_end=%.2f | maxr=%.2f xy=%.3f vel=%.3f ====\n', ...
  PROBE_SEED, fov_fail, landed, t(end), max(r), xy_end, vel_end);
fprintf('  t   |  sy    Yvel | sen_y  p_s   r    | u_edge v_edge (>1=fov_fail)\n');
step = max(1, round(0.25/(t(2)-t(1))));
for k = 1:step:ni
  fprintf('%5.2f | %5.2f %5.2f | %5.2f %5.2f %5.2f | %5.2f  %5.2f\n', ...
    t(k), sy(k), Yvel(k), seny(k), psy(k), r(k), uer(k), ver(k));
end
fprintf('  --- last 4 steps ---\n');
for k = max(1,ni-3):ni
  fprintf('%5.2f | %5.2f %5.2f | %5.2f %5.2f %5.2f | %5.2f  %5.2f\n', ...
    t(k), sy(k), Yvel(k), seny(k), psy(k), r(k), uer(k), ver(k));
end
fprintf('  peak r=%.2f@t%.2f | peak u_edge=%.2f v_edge=%.2f | fov_fail=%d\n', ...
  max(r), t(find(r==max(r),1)), max(uer), max(ver), fov_fail);
