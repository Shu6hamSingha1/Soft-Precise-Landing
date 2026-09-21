%% CMP_SMOKE  Non-destructive smoke test of the comparison harness (all 5 controllers, one trajectory).
%   Runs visualControl_comparison for controllers 1..5 the way run_comparison does but SAVES NOTHING and
%   makes no plots, so Datasets/Comparison/*.mat are untouched (run_comparison overwrites them; a single-ctrl
%   run_comparison also partially wipes the combined .mat -- see memory).  Prints one line per controller.
%   Usage:  cd MATLAB/Comparison; cmp_smoke("Circular", 1)      % trajectory, seed
%   2026-09-21: used to verify the port of the line-sampled marker / exact image model / new gains + yaw.
function cmp_smoke(traj, seed)
global SM_SEED SM_TRAJ VDF_OVERRIDE CMP_NAMES
SM_SEED=seed; SM_TRAJ=string(traj);
here = fileparts(mfilename('fullpath')); addpath(fullfile(here,'..','Common')); cd(here);
VDF_OVERRIDE=struct('theta_per_axis',true);
CMP_NAMES={'PLASMC','Lin2022','Zhang2026','Lin2023','Cho2022'};
for c=1:5
  global SM_SEED SM_TRAJ CMP_NAMES VDF_OVERRIDE
  MC_SEED=SM_SEED; CTRL_SEL=c; TRAJ_TYPE=SM_TRAJ; SPEED_MULT=1.0;
  try
    visualControl_comparison;
    global CMP_NAMES
    fprintf('RES %-11s %-9s | t_end=%5.2f fov=%d precise=%d soft=%d xy=%.3f v=%.3f\n',TRAJ_TYPE,CMP_NAMES{CTRL_SEL},tRange(idx),fov_fail,precise,soft,xy_err,rel_vel);
  catch ME
    global CMP_NAMES
    fprintf('RES %-11s | ERROR: %s (%s line %d)\n',CMP_NAMES{c},ME.message,ME.stack(1).name,ME.stack(1).line);
  end
end
end
