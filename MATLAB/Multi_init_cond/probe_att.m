% Pin the seed-6 delivery failure: commanded I_a_cd_y is strongly negative
% (wants -Y roll) yet the drone rolls +Y.  Is R_d built wrong, or does the inner
% loop fail to track R_d?  e_R (geometric attitude error, D_DS(7:9)) decides:
%   small e_R + wrong roll  => R_d construction is wrong (yaw/heading confound)
%   large e_R               => inner attitude loop not tracking R_d
%   run probe_att   (set global PROBE_SEED)

clear -regexp '^(?!IC_OVERRIDE|NOISE_OVERRIDE|RNG_SEED_OVERRIDE|SEN_.*|THETA_.*|FOV_.*|PROBE_SEED).*$';
clc;
global IC_OVERRIDE NOISE_OVERRIDE RNG_SEED_OVERRIDE PROBE_SEED;
if isempty(PROBE_SEED); PROBE_SEED = 6; end
IC_OVERRIDE = [2;2;-3]; NOISE_OVERRIDE = 1; RNG_SEED_OVERRIDE = PROBE_SEED;

visualControl_IBVS_adaptive;

global PROBE_SEED;
ni = idx; t = tRange(1:ni);
Iax = D_DS(4,1:ni); Iay = D_DS(5,1:ni); Iaz = D_DS(6,1:ni);
eRx = D_DS(7,1:ni); eRy = D_DS(8,1:ni); eRz = D_DS(9,1:ni);
psid = D_DS(14,1:ni); yawr = zeros(1,ni); roll = zeros(1,ni); pitch = zeros(1,ni);
for k=1:ni
  q = X_DS(4:7,k+1);
  roll(k)  = atan2(2*(q(1)*q(2)+q(3)*q(4)), 1-2*(q(2)^2+q(3)^2));
  pitch(k) = asin(max(-1,min(1,2*(q(1)*q(3)-q(4)*q(2)))));
  yawr(k)  = atan2(2*(q(1)*q(4)+q(2)*q(3)), 1-2*(q(3)^2+q(4)^2));
end
% desired tilt direction from I_a_cd (NED, thrust accel): body z_d = -Ia/|Ia|.
% desired roll ~ asin(Iay/|Ia|) [+Iay => need +Y accel => +roll]; here Iay<0 => want roll<0.
nIa = sqrt(Iax.^2+Iay.^2+Iaz.^2);
des_roll  = asin(max(-1,min(1, Iay./max(nIa,1e-6))));
des_pitch = asin(max(-1,min(1,-Iax./max(nIa,1e-6))));

fprintf('\n==== seed %d : ATTITUDE delivery (fov=%d) ====\n', PROBE_SEED, fov_fail);
fprintf('  t   | Iacd_y | des_roll ach_roll  e_Rx | des_pit ach_pit e_Ry | yaw\n');
kmax = min(ni, round(1.65/(t(2)-t(1)))); step = max(1, round(0.10/(t(2)-t(1))));
for k = 1:step:kmax
  fprintf('%5.2f | %6.2f | %7.3f %7.3f %6.3f | %7.3f %7.3f %6.3f | %6.3f\n', ...
    t(k), Iay(k), des_roll(k), roll(k), eRx(k), des_pitch(k), pitch(k), eRy(k), yawr(k));
end
fprintf('--- means [0,%.2f]: Iacd_y=%.2f | des_roll=%.3f ach_roll=%.3f e_Rx=%.3f | des_pit=%.3f ach_pit=%.3f ---\n', ...
  t(kmax), mean(Iay(1:kmax)), mean(des_roll(1:kmax)), mean(roll(1:kmax)), mean(eRx(1:kmax)), ...
  mean(des_pitch(1:kmax)), mean(pitch(1:kmax)));
