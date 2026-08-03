%% CB_LEAK_SWEEP  Leakage (Pleak) sweep on the D base -- raise kappa's SUSTAINED
%   rejection gain.  kappa* = diag(theta)*G*|sigma| / P  -> lower P raises steady kappa.
%   Goal: kappa functions well to reject disturbances. Watch the too-low-leakage
%   failure mode: kappa wind-up -> chattering -> higher touchdown vel / SP loss.
%   REALISTIC only (noiseless skipped per user). Full 25 cells (5 traj x 5 IC).
%
%   Base = config D: kappa0=0.05, N=diag(0.10), theta_per_axis=true. Funnel baked.
%
% Run:  cd MATLAB/Multi_init_cond; cb_leak_sweep
% Saves: ../Datasets/MultiInit/leak_sweep.mat
clc; clear;
mfile_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(mfile_dir, '..', 'Common'));
clear mfile_dir;

base = struct('theta_per_axis',true,'kappa0',[0.05;0.05;0.05],'N',diag([0.10,0.10,0.10]));
L = { 'L0_baked_1.5_5.0', [1.5;1.5;5.0];
      'L1_1.0_3.0',       [1.0;1.0;3.0];
      'L2_1.0_2.0',       [1.0;1.0;2.0];
      'L3_0.5_1.5',       [0.5;0.5;1.5] };

trajList = ["Static","Linear","Sinusoidal","Lissajous","Circular"];
p0 = [ 0,0,-5; 2.0,2.0,-5; 2.0,-2.0,-5; 2.0,2.0,-7; 2.0,2.0,-3 ];
co = struct('NOISE',1,'GE',1,'delay',1);
global VDF_OVERRIDE %#ok<GVMIS>

R = struct('name',{},'sp',{},'n',{},'mxy',{},'mv',{},'maxv',{}, ...
           'krms',{},'kpk',{},'fails',{});
for li = 1:size(L,1)
  ov = base; ov.Pleak = diag(L{li,2});
  sp=0; n=0; xy=[]; v=[]; krms=zeros(3,0); kpk=zeros(3,0); fails={};
  fprintf('\n===== %s  (Pleak=[%.1f %.1f %.1f]) =====\n', L{li,1}, L{li,2});
  for t = 1:numel(trajList)
    for ic = 1:5
      VDF_OVERRIDE = ov;
      x0 = [p0(ic,:)'; 1;0;0;0; zeros(3,1); zeros(3,1)];
      r = run_simulation(x0, trajList(t), [], 1.0, co, 1);
      d=r.data; idx=d.idx;
      ka = d.kappa_log(:,1:idx);
      isSP = r.precise&&r.soft;
      sp=sp+isSP; n=n+1; xy(end+1)=r.final_xy; v(end+1)=r.final_rel_vel; %#ok<AGROW>
      krms(:,end+1)=sqrt(mean(ka.^2,2)); kpk(:,end+1)=max(ka,[],2);       %#ok<AGROW>
      if ~isSP, fails{end+1}=sprintf('%s-IC%d(p=%d s=%d fov=%d xy=%.3f v=%.3f)', ...
          trajList(t),ic,r.precise,r.soft,r.fov_fail,r.final_xy,r.final_rel_vel); end %#ok<AGROW>
      fprintf('  %-11s IC%d SP=%d xy=%.3f v=%.3f kRMS=[%.3f %.3f %.3f] kpk=[%.2f %.2f %.2f]\n', ...
        trajList(t),ic,isSP,r.final_xy,r.final_rel_vel, krms(:,end), kpk(:,end));
    end
  end
  R(li)=struct('name',L{li,1},'sp',sp,'n',n,'mxy',mean(xy),'mv',mean(v),'maxv',max(v), ...
               'krms',median(krms,2),'kpk',median(kpk,2),'fails',{fails});
end
clear global VDF_OVERRIDE

fprintf('\n==================== LEAKAGE SWEEP SUMMARY (25 realistic) ====================\n');
fprintf('%-18s SP    mXY    mV    maxV | kRMS x/y/z (sustained)   kPK x/y/z\n','config');
for li=1:numel(R)
  fprintf('%-18s %2d/%-2d %.3f %.3f %.3f | %.3f %.3f %.3f      %.2f %.2f %.2f\n', ...
    R(li).name,R(li).sp,R(li).n,R(li).mxy,R(li).mv,R(li).maxv, ...
    R(li).krms(1),R(li).krms(2),R(li).krms(3), R(li).kpk(1),R(li).kpk(2),R(li).kpk(3));
end
fprintf('\nFailures:\n');
for li=1:numel(R)
  if ~isempty(R(li).fails)
    fprintf('  %s:\n', R(li).name);
    for q=1:numel(R(li).fails), fprintf('     %s\n', R(li).fails{q}); end
  end
end
save(fullfile(fileparts(mfilename('fullpath')),'..','Datasets','MultiInit','leak_sweep.mat'),'R','L','base');
fprintf('\nSaved -> Datasets/MultiInit/leak_sweep.mat\nDONE\n');
