function cmp_tune(ctrl, label, cfgexpr, traj, seeds)
%% CMP_TUNE  Score one baseline gain override on the comparison harness (non-destructive; saves nothing).
%   cmp_tune(ctrl, label, cfgexpr, traj, seeds)
%     ctrl    : 2..5  (Lin2022, Zhang2026, Lin2023, Cho2022)
%     cfgexpr : MATLAB expression evaluating to the CMP_OVERRIDE struct (fields of that controller's K), or "" for the baked gains
%     traj    : "Static" | "Linear" | ...   seeds : noise/disturbance seeds (init_robustness draws CoG, wind, mass per seed)
%   Same test setting as the proposed controller: identical plant, disturbances, seeds, IC (2,2,-5), landing criteria.
%   Prints one 'TUNE' line per seed: reached (alt<=0.21), soft, precise, FoV, terminal xy [m], terminal rel. speed [m/s], t_end.
global TN_CTRL TN_LABEL TN_CFG TN_TRAJ TN_SEEDS CMP_OVERRIDE VDF_OVERRIDE
TN_CTRL=ctrl; TN_LABEL=char(label); TN_CFG=char(cfgexpr); TN_TRAJ=string(traj); TN_SEEDS=seeds;
here = fileparts(mfilename('fullpath')); addpath(fullfile(here,'..','Common')); cd(here);
VDF_OVERRIDE = struct('theta_per_axis',true);
for sd = seeds
  global TN_CTRL TN_LABEL TN_CFG TN_TRAJ TN_SEEDS CMP_OVERRIDE
  if isempty(TN_CFG), CMP_OVERRIDE = []; else, CMP_OVERRIDE = eval(TN_CFG); end
  MC_SEED = sd; CTRL_SEL = TN_CTRL; TRAJ_TYPE = TN_TRAJ; SPEED_MULT = 1.0;
  visualControl_comparison;
  global TN_CTRL TN_LABEL TN_TRAJ
  alt_end = -(X_DS(3,idx)-x_t(3,idx)); xy = norm(X_DS(1:2,idx)-x_t(1:2,idx)); v = norm(X_DS(8:10,idx)-dx_t(1:3,idx));
  fprintf('TUNE c=%d %-22s %-10s seed %d | reached=%d soft=%d precise=%d fov=%d | xy=%.3f v=%.3f alt_end=%.2f t_end=%.1f%c', ...
      TN_CTRL, TN_LABEL, TN_TRAJ, MC_SEED, alt_end<=0.21, v<=0.2, xy<=0.08, fov_fail, xy, v, alt_end, idx*0.01, char(10));
end
end
