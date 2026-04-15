%% =========================================================================
%  init_robustness.m  —  shared robustness-model setup
%
%  Script (not function) sourced by run_simulation.m,
%  visualControl_IBVS_adaptive.m, and visualControl_comparison.m so that all
%  three files measure the same controller under identical disturbance.
%
%  Samples per-run parametric uncertainty (mass, inertia, CoG offset, wind
%  mean) using the caller's current RNG state. Caller is responsible for
%  seeding (`rng(seed)` or `rng('shuffle')`) before sourcing.
%
%  Requires in workspace:  m, J   (from Constants.m)
%
%  Exports:
%    delta_m, delta_J, r_cog, m_p, J_p          parametric uncertainty
%    wind_mean, wind_tau, wind_sigma, F_turb,
%    C_d_wind                                    wind disturbance state
%    px_sigma0, px_sigma1, px_depth_offset,
%    outlier_prob, outlier_mag                   pixel noise model
%
%  The NOISE flag is NOT checked here — sampled values are harmless when
%  unused. The caller gates consumption at the pixel-noise block and the
%  dynamics integration site.
% =========================================================================

% Per-run parametric uncertainty (constant for the run)
delta_m = 0.05 * (2*rand - 1);            % +/-5% mass
delta_J = 0.05 * (2*rand - 1);            % +/-5% inertia
r_cog   = 0.005 * (2*rand(3,1) - 1);      % +/-5 mm CoG offset
r_cog(3) = 0;
m_p     = m * (1 + delta_m);
J_p     = J * (1 + delta_J);

% Wind disturbance state (mean horizontal gust + first-order turbulence)
wind_mean  = 0.20 * [2*rand - 1; 2*rand - 1; 0];  % up to 0.2 m/s horizontal mean
wind_tau   = 1.0;                                  % turbulence correlation time [s]
wind_sigma = 0.1;                                  % turbulence force std [N]
F_turb     = zeros(3,1);
C_d_wind   = 0.25;                                  % quadrotor drag coefficient

% Depth-dependent pixel noise: sigma_px(z) = 0.3 + 0.5/(z+0.5)  [pixels]
% z=5m: 0.39 px | z=1m: 0.63 px | z=0.2m: 1.01 px
px_sigma0       = 0.3;
px_sigma1       = 0.5;
px_depth_offset = 0.5;
outlier_prob    = 0.005;   % 0.5% Bernoulli rate
outlier_mag     = 5;       % 5 px outlier magnitude
