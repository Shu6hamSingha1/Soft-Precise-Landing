%% Perception-channel fidelity vs marker size at altitude (independent of landing outcome).
% Env: Z0 (start alt, default 7), SCALES (MARKER_SCALE list, total size = 2x), TRAJS, SEEDS. Measured = logged cs.V_*_i (raw per-frame),
% and its causal SG-filtered version (what the controller sees, window P.fw, order 2). Truth = V_X_DS analytic rows.
clc; clear; addpath('../Common');
global VDF_OVERRIDE MARKER_SCALE PX_NOISE_FIX PX_NOISE_PARAMS
PX_NOISE_FIX=true; PX_NOISE_PARAMS=[0.027 0.175 0.5 0 0];
z0=str2double(getenv('Z0')); if isnan(z0), z0=7; end
scs=str2num(getenv('SCALES')); if isempty(scs), scs=[1 3 6 12]; end %#ok<ST2NM>
tn=strsplit(strtrim(getenv('TRAJS'))); if isempty(tn{1}), tn={'Circular','Lissajous','Sinusoidal'}; end
seeds=str2num(getenv('SEEDS')); if isempty(seeds), seeds=1; end %#ok<ST2NM>
cfg=struct('NOISE',1,'GE',1,'delay',1); Wf=11;
wins=[7 5;5 3;3 1.5;1.5 0.5];
chn={'s_x','s_y','alpha','h_x','h_y','h_z','w_z'};
for sc=scs
  MARKER_SCALE=sc; VDF_OVERRIDE=struct('theta_per_axis',true);
  M=cell(1,4); T=cell(1,4);
  for a=1:numel(tn), for sd=seeds
    r=run_simulation([2;2;-z0;1;0;0;0;zeros(6,1)],string(tn{a}),[],1.4,cfg,sd); d=r.data; n=d.idx; if n<=0,n=numel(d.e_a_log);end
    V=d.V_X_DS(:,1:n); alt=-(d.X_DS(3,1:n)-d.x_t(3,1:n));
    meas=[V(1:2,:);V(3,:);V(4:6,:);V(9,:)]; tru=[V(13:14,:);V(15,:);V(16:18,:);V(21,:)];
    dal=angle(exp(1i*(meas(3,:)-tru(3,:)))); meas(3,:)=tru(3,:)+dal;
    filt=meas; for k=Wf:n, filt(:,k)=meshsg(meas(:,k-Wf+1:k)); end
    for w=1:4, m=alt<=wins(w,1)&alt>wins(w,2);
      M{w}=[M{w} [meas(:,m);filt(:,m)]]; T{w}=[T{w} tru(:,m)]; end
  end, end
  fprintf('\n=== marker %gx (total), start alt %g m ===\n',2*sc,z0);
  for w=1:4
    if size(T{w},2)<20, continue; end
    fprintf(' alt %.1f-%.1f m (n=%d)   channel: corr / nRMSE(rms err/rms truth)  [raw | SG-filtered]\n',wins(w,1),wins(w,2),size(T{w},2));
    for c=1:7
      x=M{w}(c,:); xf=M{w}(7+c,:); y=T{w}(c,:);
      fprintf('   %-6s  raw %5.2f / %5.2f | SG %5.2f / %5.2f   (rms truth %.3g)\n',chn{c},cc(x,y),rms(x-y)/rms(y),cc(xf,y),rms(xf-y)/rms(y),rms(y));
    end
  end
end
function c=cc(x,y), ok=isfinite(x)&isfinite(y); x=x(ok)-mean(x(ok)); y=y(ok)-mean(y(ok)); c=(x*y')/sqrt((x*x')*(y*y')); end
function v=meshsg(B), W=size(B,2); F=sgolayfilt(B,2,W,[],2); v=F(:,end); end
