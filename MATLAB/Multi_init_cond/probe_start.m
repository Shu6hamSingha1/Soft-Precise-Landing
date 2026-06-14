% Diagnose the IC5 seed-4 startup corner-ejection (fov_fail ~t=0.4, xy=2.7).
% Which drives the ejecting corner toward the FoV edge: LOOM (descent grows the
% marker spread), TILT/translation (centroid shifts), or a noise spike?
%   cr_v   = centroid v (tangent)        -> shifts with tilt/translation
%   delta_v= half corner spread v        -> grows with LOOM (descent)
%   v_edge = max|C_nP_v|/120 (>1 fov_fail), u_edge = max|C_nP_u|/160
%   run probe_start   (set global PROBE_SEED; default inset)

clear -regexp '^(?!IC_OVERRIDE|NOISE_OVERRIDE|RNG_SEED_OVERRIDE|PROBE_SEED).*$';
clc;
global IC_OVERRIDE NOISE_OVERRIDE RNG_SEED_OVERRIDE PROBE_SEED;
if isempty(PROBE_SEED); PROBE_SEED = 4; end
IC_OVERRIDE = [2;2;-3]; NOISE_OVERRIDE = 1; RNG_SEED_OVERRIDE = PROBE_SEED;

visualControl_IBVS_adaptive;

global PROBE_SEED;
ni = idx; t = tRange(1:ni);
roll=zeros(1,ni); pitch=zeros(1,ni); alt=zeros(1,ni);
ued=zeros(1,ni); ved=zeros(1,ni); crv=zeros(1,ni); dlv=zeros(1,ni); cru=zeros(1,ni); dlu=zeros(1,ni);
for k=1:ni
  q = X_DS(4:7,k+1);
  roll(k)  = atan2(2*(q(1)*q(2)+q(3)*q(4)), 1-2*(q(2)^2+q(3)^2));
  pitch(k) = asin(max(-1,min(1,2*(q(1)*q(3)-q(4)*q(2)))));
  alt(k)   = abs(X_DS(3,k+1) - x_t(3,k));
  C = P_DS(:,9:12,k);                       % actual corner pixels
  ued(k)=max(abs(C(1,:)))/160; ved(k)=max(abs(C(2,:)))/120;
  cru(k)=mean(C(1,:))/135;  dlu(k)=0.5*(max(C(1,:))-min(C(1,:)))/135;
  crv(k)=mean(C(2,:))/135;  dlv(k)=0.5*(max(C(2,:))-min(C(2,:)))/135;
end
fprintf('\n==== IC5 seed %d startup (fov=%d t_end=%.2f) — ejecting axis & cause ====\n', PROBE_SEED, fov_fail, t(end));
fprintf('  t   | alt  | u_edge v_edge | cr_v   dl_v  (loom=dl grows) | roll  pitch\n');
kmax=min(ni, round(0.55/(t(2)-t(1)))); step=max(1,round(0.05/(t(2)-t(1))));
for k=1:step:kmax
  fprintf('%5.3f | %4.2f | %5.3f  %5.3f | %6.3f %5.3f | %5.2f %5.2f\n', ...
    t(k), alt(k), ued(k), ved(k), crv(k), dlv(k), roll(k), pitch(k));
end
fprintf('  --- last 3 steps before end ---\n');
for k=max(1,ni-2):ni
  fprintf('%5.3f | %4.2f | %5.3f  %5.3f | %6.3f %5.3f | %5.2f %5.2f\n', ...
    t(k), alt(k), ued(k), ved(k), crv(k), dlv(k), roll(k), pitch(k));
end
fprintf('  start: cr_v=%.3f dl_v=%.3f ; end: cr_v=%.3f dl_v=%.3f (dcr=%.3f ddl=%.3f) | alt %.2f->%.2f\n', ...
  crv(1),dlv(1),crv(ni),dlv(ni),crv(ni)-crv(1),dlv(ni)-dlv(1),alt(1),alt(ni));
