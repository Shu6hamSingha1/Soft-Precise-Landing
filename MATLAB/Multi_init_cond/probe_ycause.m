% Disambiguate the seed-6 Y failure: is the wrong-way Y motion a CONTROL error
% (controller commands +Y accel) or the expected quadrotor TILT transient
% (commands -Y but tilt/underactuation moves the feature first)?  Dumps the
% COMMANDED inertial accel I_a_cd, desired heading psi_d / yaw-rate u_a, and the
% actual roll/pitch/yaw, early window.
% D_DS(:,k) = [V_h_d(3); I_a_cd(3); e_R(3); B_tau_cd(3); T_cd; psi_d; u_a]
%   run probe_ycause   (set global PROBE_SEED)

clear -regexp '^(?!IC_OVERRIDE|NOISE_OVERRIDE|RNG_SEED_OVERRIDE|SEN_.*|THETA_.*|FOV_.*|PROBE_SEED).*$';
clc;
global IC_OVERRIDE NOISE_OVERRIDE RNG_SEED_OVERRIDE PROBE_SEED;
if isempty(PROBE_SEED); PROBE_SEED = 6; end
IC_OVERRIDE = [2;2;-3]; NOISE_OVERRIDE = 1; RNG_SEED_OVERRIDE = PROBE_SEED;

visualControl_IBVS_adaptive;

global PROBE_SEED;
ni = idx; t = tRange(1:ni);
Iax = D_DS(4,1:ni); Iay = D_DS(5,1:ni); Iaz = D_DS(6,1:ni);  % commanded inertial accel
psid = D_DS(14,1:ni); ua = D_DS(15,1:ni);
Xpos = X_DS(1,2:ni+1); Ypos = X_DS(2,2:ni+1);
Xvel = X_DS(8,2:ni+1); Yvel = X_DS(9,2:ni+1);
% actual euler from quaternion X_DS(4:7,k+1) = [qw qx qy qz]
roll=zeros(1,ni); pitch=roll; yaw=roll;
for k=1:ni
  q = X_DS(4:7,k+1);
  roll(k)  = atan2(2*(q(1)*q(2)+q(3)*q(4)), 1-2*(q(2)^2+q(3)^2));
  pitch(k) = asin(max(-1,min(1,2*(q(1)*q(3)-q(4)*q(2)))));
  yaw(k)   = atan2(2*(q(1)*q(4)+q(2)*q(3)), 1-2*(q(3)^2+q(4)^2));
end

fprintf('\n==== seed %d : Y CAUSE (commanded vs actual) fov=%d ====\n', PROBE_SEED, fov_fail);
fprintf('To close +Y offset want I_a_cd_y<0 (accel -Y).  rad: roll/pitch/yaw, psi_d.\n');
fprintf('  t   | Xpos  Ypos | Xvel  Yvel | Iacd_x Iacd_y | roll  pitch  yaw   psi_d  u_a\n');
kmax = min(ni, round(1.65/(t(2)-t(1))));
step = max(1, round(0.10/(t(2)-t(1))));
for k = 1:step:kmax
  fprintf('%5.2f | %5.2f %5.2f | %5.2f %5.2f | %6.2f %6.2f | %5.2f %5.2f %5.2f %5.2f %5.2f\n', ...
    t(k), Xpos(k), Ypos(k), Xvel(k), Yvel(k), Iax(k), Iay(k), ...
    roll(k), pitch(k), yaw(k), psid(k), ua(k));
end
fprintf('--- early-window means [0,%.2f]: I_a_cd=[%.2f %.2f]  yaw=%.3f psi_d=%.3f ---\n', ...
  t(kmax), mean(Iax(1:kmax)), mean(Iay(1:kmax)), mean(yaw(1:kmax)), mean(psid(1:kmax)));
