%% CMP_DIAG  Failure anatomy of the 4 baselines in the comparison harness (non-destructive; saves nothing).
%   cmp_diag(traj, seed, mode)   mode = 'base' | 'noiseless' (NOISE_OVERRIDE=0) | 'nofov' (CMP_FOV_ABORT=false: log the
%   first FoV crossing but keep flying) | 'both'.  Prints, per baseline: end state, first FoV-crossing time and the state at that
%   crossing (altitude, xy error, tilt, |a_cd|, body rate, centre pixel), peak tilt/accel before it.
%   2026-09-21: used to show the baselines' failures are driven by the CoG offset (P-only geometric attitude loop, no CoG
%   compensation) -- see memory project_baseline_failure_analysis_2026_09_21.
function cmp_diag(traj, seed, mode)
% mode: 'base' | 'noiseless' | 'nofov' | 'both'
global DG_TRAJ DG_SEED DG_MODE CMP_FOV_ABORT VDF_OVERRIDE DG_NAMES
DG_TRAJ=string(traj); DG_SEED=seed; DG_MODE=mode;
here = fileparts(mfilename('fullpath')); addpath(fullfile(here,'..','Common')); cd(here);
VDF_OVERRIDE=struct('theta_per_axis',true);
global CMP_OVERRIDE
cfg_ = getenv('DG_CFG'); if ~isempty(cfg_), CMP_OVERRIDE = eval(cfg_); end   % optional: gain override / shared_so3 (env DG_CFG)
DG_NAMES={'PLASMC','Lin2022','Zhang2026','Lin2023','Cho2022'};
for c=2:5
  global DG_TRAJ DG_SEED DG_MODE CMP_FOV_ABORT DG_NAMES
  MC_SEED=DG_SEED; CTRL_SEL=c; TRAJ_TYPE=DG_TRAJ; SPEED_MULT=1.0;
  CMP_FOV_ABORT = ~(strcmp(DG_MODE,'nofov')||strcmp(DG_MODE,'both'));
  if strcmp(DG_MODE,'noiseless')||strcmp(DG_MODE,'both'), NOISE_OVERRIDE=0; end
  visualControl_comparison;
  global DG_NAMES DG_MODE
  n=idx; t=(1:n)*dt; alt=-(X_DS(3,1:n)-x_t(3,1:n)); pxy=vecnorm(X_DS(1:2,1:n)-x_t(1:2,1:n)); q=X_DS(4:7,1:n);
  tilt=acosd(max(min(1-2*(q(2,:).^2+q(3,:).^2),1),-1)); vz=X_DS(10,1:n)-dx_t(3,1:n); a=vecnorm(I_a_cd(:,1:n)); w=vecnorm(X_DS(11:13,1:n));
  ktf=find(vecnorm(cen_px_log(:,1:n))>0 & (abs(cen_px_log(1,1:n))>res(1)/2 | abs(cen_px_log(2,1:n))>res(2)/2),1);
  if isempty(ktf), ktf=n; end
  fprintf('DG %-9s %-9s | t_end %5.2f fov=%d first_fov_t %s | at FoV crossing: alt %.2f xy %.2f tilt %.1f |a| %.1f w %.2f cen [%.0f %.0f] | peak tilt %.1f peak|a| %.1f | END alt %.2f xy %.3f v %.2f\n', ...
      DG_NAMES{c},DG_MODE,t(n),fov_fail,num2str(fov_fail_t,'%.2f'),alt(ktf),pxy(ktf),tilt(ktf),a(ktf),w(ktf),cen_px_log(1,ktf),cen_px_log(2,ktf),max(tilt(1:ktf)),max(a(1:ktf)),alt(n),pxy(n),norm(X_DS(8:10,n)-dx_t(1:3,n)));
end
end
