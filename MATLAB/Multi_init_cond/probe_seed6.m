% Trace the IC5 [2,2,-3] noisy failure (default seed 6) Y-channel to localize
% WHERE/HOW it diverges: smooth overshoot (gain) vs sudden jump (outlier) vs
% visibility clip.  Set PROBE_SEED (global) to pick the seed.
%   run probe_seed6   (or set globals: PROBE_SEED, and optionally SEN_R*_OVERRIDE)

clear -regexp '^(?!IC_OVERRIDE|NOISE_OVERRIDE|RNG_SEED_OVERRIDE|SEN_.*|THETA_.*|FOV_.*|PROBE_SEED).*$';
clc;
global IC_OVERRIDE NOISE_OVERRIDE RNG_SEED_OVERRIDE PROBE_SEED;
if isempty(PROBE_SEED); PROBE_SEED = 6; end
IC_OVERRIDE = [2;2;-3]; NOISE_OVERRIDE = 1; RNG_SEED_OVERRIDE = PROBE_SEED;

visualControl_IBVS_adaptive;

global PROBE_SEED;   % re-link after the script's clear
ni = idx;
t  = tRange(1:ni);
% state indices: 1-3 pos, 4-7 quat, 8-10 vel, 11-13 omega  (X_DS is [.,N+1])
Ypos = X_DS(2, 2:ni+1);  Yvel = X_DS(9, 2:ni+1);
Xpos = X_DS(1, 2:ni+1);  Xvel = X_DS(8, 2:ni+1);
seny = V_s_e_n(2,1:ni);  psy = p_s(2,1:ni);
Ssy  = squeeze(S_s(2,2,1:ni))';  izy = izeta_s(2,1:ni);
brc  = sen_breach(2,1:ni);
vhey = V_h_e(2,1:ni);    % middle-loop optic-flow error (Y)

fprintf('\n==== seed %d : fov=%d t_end=%.2f xy_end=%.3f ====\n', PROBE_SEED, fov_fail, t(end), norm([Xpos(end);Ypos(end)]));
kb = find(brc,1,'first');  kv = find(abs(Yvel)>0.5,1,'first');
if ~isempty(kb); fprintf('first Y breach (|s_e_n|>p_s):  t=%.2f  (sen_y=%.2f p_s=%.2f)\n', t(kb), seny(kb), psy(kb)); else; fprintf('no Y breach\n'); end
if ~isempty(kv); fprintf('first |Yvel|>0.5:              t=%.2f  (Yvel=%.2f Ypos=%.2f)\n', t(kv), Yvel(kv), Ypos(kv)); end
[mv,mk] = max(abs(Yvel)); fprintf('peak |Yvel|=%.2f at t=%.2f\n', mv, t(mk));

fprintf('\n  t   | Xpos  Ypos  | Xvel  Yvel  | sen_y  p_s   S_s   izy   brc | vhe_y\n');
step = max(1, round(0.25/(t(2)-t(1))));
for k = 1:step:ni
  fprintf('%5.2f | %5.2f %5.2f | %5.2f %5.2f | %5.2f %5.2f %5.2f %5.2f  %d  | %6.2f\n', ...
    t(k), Xpos(k), Ypos(k), Xvel(k), Yvel(k), seny(k), psy(k), Ssy(k), izy(k), brc(k), vhey(k));
end
% always show the last few steps before failure
fprintf('  --- last 6 steps ---\n');
for k = max(1,ni-5):ni
  fprintf('%5.2f | %5.2f %5.2f | %5.2f %5.2f | %5.2f %5.2f %5.2f %5.2f  %d  | %6.2f\n', ...
    t(k), Xpos(k), Ypos(k), Xvel(k), Yvel(k), seny(k), psy(k), Ssy(k), izy(k), brc(k), vhey(k));
end
