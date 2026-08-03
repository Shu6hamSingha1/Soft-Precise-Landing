%% CB_YAW_REJECT  Yaw-axis (kappa_a) rejection on the YAWING-target trajectory.
%   Circular yaws the target continuously (psi = wz*t, dpsi = wz). Earlier aggregate
%   median washed kappa_a out by mixing the 5 Circular cells with 20 zero-yaw cells.
%   Here: kappa_a behavior on Circular ONLY (D_test realistic + noiseless), per IC.
%   Reports kappa_a peak/end/RMS and the yaw control effort -> is kappa_a actually
%   adapting to the sustained yaw, or does the virtual-compass track it without kappa_a?
%
% Run:  cd MATLAB/Multi_init_cond; cb_yaw_reject
clc; clear;
mfile_dir = fileparts(mfilename('fullpath'));
ddir = fullfile(mfile_dir, '..', 'Datasets', 'MultiInit', 'D_test');
clear mfile_dir;

for tag = ["", "_noiseless"]
  f = fullfile(ddir, sprintf('Circular_multi_init_D%s.mat', tag));
  if ~exist(f,'file'), warning('missing %s', f); continue; end
  S = load(f); R = S.results;
  lab = "realistic"; if tag~="", lab = "noiseless"; end
  fprintf('\n===== Circular (yawing target, wz=0.48) | %s =====\n', lab);
  fprintf('  IC | kappa_a: start  peak   end    RMS  | |u_a| RMS  peak | yaw-control sustained?\n');
  for ic = 1:numel(R)
    d = R(ic).data; idx = d.idx;
    ka = d.kappa_a_log(1:idx);
    ua = d.u_a_log(1:idx);
    sust = ka(end) > 0.5*max(ka);   % did kappa_a hold up (not just decay)?
    fprintf('  %d  |        %5.2f  %5.2f  %5.2f  %5.2f | %7.3f  %6.3f | %s\n', ...
      ic, ka(1), max(ka), ka(end), sqrt(mean(ka.^2)), ...
      sqrt(mean(ua.^2)), max(abs(ua)), ternary(sust,'YES - sustained','no - decays'));
  end
end

function out = ternary(c,a,b), if c, out=a; else, out=b; end, end
