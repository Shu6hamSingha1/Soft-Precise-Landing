%% CB_KAPPA_REJECT  Per-axis disturbance-rejection breakdown for config D.
%   Loads the saved D_test REALISTIC runs and, per axis (x,y,z optic-flow + yaw),
%   reports how the lumped disturbance is rejected:
%     disturbance seen  D_k(t) = v_k(beta-1) + d_h,k        (regressor.d_bar, un-normalized)
%     kappa supplies    kappa_k(t)                          (adaptive switching gain)
%     barrier supplies  engagement engH_k=|h_e,k|/p_h,k -> G_h ~ 1/(1-engH^2) (log-barrier)
%   The error stays bounded (all cells SP) so rejection succeeded; this shows the SPLIT
%   between kappa and the funnel barrier, per axis, and whether kappa tracks D_k.
%   Yaw (4th axis): kappa_a peak/terminal + does yaw stay locked.
%
% Run:  cd MATLAB/Multi_init_cond; cb_kappa_reject
clc; clear;
mfile_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(mfile_dir, '..', 'Common'));
ddir = fullfile(mfile_dir, '..', 'Datasets', 'MultiInit', 'D_test');
clear mfile_dir;

trajList = ["Static","Linear","Sinusoidal","Lissajous","Circular"];
axName = ["x","y","z"];

% aggregate per-axis peaks across all 25 realistic cells
Dpk=zeros(3,0); Kpk=zeros(3,0); Kend=zeros(3,0); Hpk=zeros(3,0); Gpk=zeros(3,0);
DHpk=zeros(3,0); VBpk=zeros(3,0);   % unmodeled residual d_h  vs  structured v(beta-1)
DHrms=zeros(3,0); Krms=zeros(3,0);  % sustained (RMS) residual vs kappa
Ka_pk=[]; Ka_end=[];
cellLab = strings(1,0);

for t = 1:numel(trajList)
  f = fullfile(ddir, sprintf('%s_multi_init_D.mat', trajList(t)));   % realistic tag = ''
  if ~exist(f,'file'), warning('missing %s',f); continue; end
  S = load(f); R = S.results;
  for ic = 1:numel(R)
    d = R(ic).data; idx = d.idx;
    rng2 = 1:idx;
    beta = d.beta_log(rng2);
    vb   = d.v_log(:,rng2).*(beta-1);                       % structured (depth-scaled regressor)
    dh   = d.d_h_log(:,rng2);                               % UNMODELED residual disturbance
    Dk   = vb + dh;                                         % 3xidx lumped disturbance
    kap  = d.kappa_log(:,rng2);
    engH = abs(d.V_h_e(:,rng2))./max(d.p_h_log(:,rng2),eps);
    Gh   = 1./max(1-min(engH,0.999).^2, 1e-3);              % log-barrier gain proxy

    Dpk(:,end+1)  = max(abs(Dk),[],2);     %#ok<AGROW>
    DHpk(:,end+1) = max(abs(dh),[],2);     %#ok<AGROW>
    DHrms(:,end+1)= sqrt(mean(dh.^2,2));   %#ok<AGROW>  sustained residual (noise-spikes averaged out)
    Krms(:,end+1) = sqrt(mean(kap.^2,2));  %#ok<AGROW>
    VBpk(:,end+1) = max(abs(vb),[],2);     %#ok<AGROW>
    Kpk(:,end+1)  = max(kap,[],2);         %#ok<AGROW>
    Kend(:,end+1) = kap(:,end);            %#ok<AGROW>
    Hpk(:,end+1)  = max(engH,[],2);        %#ok<AGROW>
    Gpk(:,end+1)  = max(Gh,[],2);          %#ok<AGROW>
    Ka_pk(end+1)  = max(d.kappa_a_log(rng2));   %#ok<AGROW>
    Ka_end(end+1) = d.kappa_a_log(idx);         %#ok<AGROW>
    cellLab(end+1)= sprintf('%s-IC%d',trajList(t),ic); %#ok<AGROW>
  end
end

fprintf('\n============ PER-AXIS DISTURBANCE REJECTION  (config D, 25 realistic cells) ============\n');
fprintf('%-5s | peak|D_k| (dist seen)        | kappa_k peak / end          | barrier engH / G_h\n','axis');
fprintf('%-5s |  med    max   (cell)         |  med    max    end(med)     |  med-engH  max-G_h\n','');
for k = 1:3
  [mx,iMx] = max(Dpk(k,:));
  fprintf('%-5s | %5.2f  %5.2f  (%s) | %5.3f  %5.3f   %5.3f      | %5.2f      %5.1f\n', ...
    axName(k), median(Dpk(k,:)), mx, cellLab(iMx), ...
    median(Kpk(k,:)), max(Kpk(k,:)), median(Kend(k,:)), ...
    median(Hpk(k,:)), max(Gpk(k,:)));
end
fprintf('%-5s | %29s | %5.3f  %5.3f   %5.3f      | (yaw stays locked: see below)\n', ...
  'yaw', 'target-yaw-rate driven', median(Ka_pk), max(Ka_pk), median(Ka_end));

% split lumped D_k into structured (model-cancelled) vs unmodeled (kappa's job)
fprintf('\n--- disturbance split: structured v(beta-1) vs UNMODELED residual d_h (peak, median over cells) ---\n');
fprintf('%-5s | struct |v(b-1)|   UNMODELED |d_h|   kappa peak   kappa/|d_h|\n','axis');
for k=1:3
  ratio = Kpk(k,:)./max(DHpk(k,:),eps);
  fprintf('  %-3s |   %6.2f          %6.3f          %6.3f       %.2f\n', ...
    axName(k), median(VBpk(k,:)), median(DHpk(k,:)), median(Kpk(k,:)), median(ratio));
end
fprintf('\n--- SUSTAINED (RMS) residual d_h vs kappa (noise spikes averaged out; the fair comparison) ---\n');
fprintf('%-5s |  RMS|d_h|    RMS kappa    kappa/d_h(RMS)\n','axis');
for k=1:3
  ratio = Krms(k,:)./max(DHrms(k,:),eps);
  fprintf('  %-3s |  %6.3f      %6.3f        %.2f\n', ...
    axName(k), median(DHrms(k,:)), median(Krms(k,:)), median(ratio));
end

% most-engaged cell per axis (where the barrier did the most work)
fprintf('\n--- axis peak-engagement cell (barrier hardest at work) ---\n');
for k=1:3
  [me,ie]=max(Hpk(k,:));
  fprintf('  axis %s:  engH=%.2f  G_h=%.1f  kappa=%.3f  |D|=%.2f  @ %s\n', ...
    axName(k), me, Gpk(k,ie), Kpk(k,ie), Dpk(k,ie), cellLab(ie));
end
fprintf('\nyaw: kappa_a median peak %.2f end %.2f (kappa_a0=2.0 -> %s)\n', ...
  median(Ka_pk), median(Ka_end), ternary(median(Ka_end)<median(Ka_pk),'decays (no sustained yaw demand)','holds'));

function out = ternary(c,a,b), if c, out=a; else, out=b; end, end
