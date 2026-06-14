% Why does IC5 fail when IC1-4 pass?  Compare the 5 canonical ICs on the
% GEOMETRIC margins (noiseless, so noise is excluded): initial normalized image
% error, how close the marker corners start to the FoV edge, the worst-case FoV
% margin over the descent, the peak normalized error, and the descent time.
%   run probe_ic_geometry

clear -regexp '^(?!IC_OVERRIDE|NOISE_OVERRIDE|RNG_SEED_OVERRIDE|SEN_.*|THETA_.*|FOV_.*|WS).*$';
clc;
global IC_OVERRIDE NOISE_OVERRIDE RNG_SEED_OVERRIDE WS;

WS = struct();
WS.ICs  = {[0;0;-5], [2;2;-5], [2;-2;-5], [2;2;-7], [2;2;-3]};
WS.nm   = {'IC1','IC2','IC3','IC4','IC5'};
WS.R = {};

for ic = 1:numel(WS.ICs)
  IC_OVERRIDE = WS.ICs{ic}; NOISE_OVERRIDE = 0; RNG_SEED_OVERRIDE = 1;
  WS.ic = ic;
  visualControl_IBVS_adaptive;
  global WS;
  ni = idx;
  cic = WS.ic;
  % FoV half-extent (pixels): res = [320;240] -> [160;120]
  half = [160; 120];
  % per-step worst corner margin (px) from P_DS C_nP = cols 9:12
  marg = nan(1,ni);
  for k = 1:ni
    C = P_DS(:, 9:12, k);                 % 2x4 actual marker corner pixels
    marg(k) = min(half - max(abs(C),[],2));
  end
  C0 = P_DS(:, 9:12, 1);
  gi.nm     = WS.nm{cic};
  gi.ic     = WS.ICs{cic}';
  gi.Z0     = abs(WS.ICs{cic}(3));
  gi.sen0   = norm(V_s_e_n(:,1));           % initial normalized image error
  gi.maxsen = max(vecnorm(V_s_e_n(:,1:ni)));% peak normalized error over run
  gi.fov0   = min(half - max(abs(C0),[],2));% initial FoV margin (px; small = near edge)
  gi.fovmin = min(marg);                    % worst FoV margin over descent
  gi.tend   = tRange(ni);
  gi.landed = double(landed);
  WS.R{end+1} = gi;
  ic = cic;
end

global WS; R = WS.R;
fprintf('\n========= IC GEOMETRY (noiseless) — why IC5 is marginal =========\n');
fprintf('%-4s  IC           Z0  | sen0  maxsen | fov_marg0(px) fov_min(px) | t_end land\n','');
for k = 1:numel(R)
  gi = R{k};
  fprintf('%-4s [%2g %2g %2g]   %g  | %.2f  %.2f   |   %6.1f       %6.1f    | %5.2f   %d\n', ...
    gi.nm, gi.ic(1),gi.ic(2),gi.ic(3), gi.Z0, gi.sen0, gi.maxsen, gi.fov0, gi.fovmin, gi.tend, gi.landed);
end
fprintf('sen0   = initial normalized image error (|s_e_n|); funnel starts at p_s_0=1.2\n');
fprintf('fov_marg0 = px from nearest corner to FoV edge at t=0 (small => glitch-prone)\n');
fprintf('fov_min   = worst-case corner margin over the whole descent\n');
fprintf('================================================================\n');
