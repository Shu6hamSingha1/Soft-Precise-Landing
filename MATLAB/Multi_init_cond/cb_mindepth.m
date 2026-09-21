clc; clear; addpath('../Common');
global VDF_OVERRIDE MARKER_SCALE PX_NOISE_FIX PX_NOISE_PARAMS
PX_NOISE_FIX=true; PX_NOISE_PARAMS=[0.027 0.175 0.5 0 0]; MARKER_SCALE=24;
VDF_OVERRIDE=struct('theta_per_axis',true,'pinv_tol',4,'flow_reduced',true);
cfg=struct('NOISE',1,'GE',1,'delay',1);
base=[15/sqrt(2),-15/sqrt(2),-15/sqrt(2),15/sqrt(2),22;15/sqrt(2),-15/sqrt(2),15/sqrt(2),-15/sqrt(2),0;0 0 0 0 0]*2*12/250;
for z0=[5 7]
 for tr=["Circular" "Linear"]
  r=run_simulation([2;2;-z0;1;0;0;0;zeros(6,1)],tr,[],1.4,cfg,3); d=r.data; n=d.idx; if n<=0,n=numel(d.e_a_log);end
  md=nan(1,n); alt=nan(1,n); tilt=nan(1,n); pmax=nan(1,n);
  for k=1:n
    q=d.X_DS(4:7,k)'; Rc=quat2rotm(q); Rt=quat2rotm(d.x_t(4:7,k)');
    C=Rc'*((Rt*base+d.x_t(1:3,k))-d.X_DS(1:3,k));
    md(k)=min(C(3,:)); alt(k)=-(d.X_DS(3,k)-d.x_t(3,k)); tilt(k)=acosd(Rc(3,3));
    pmax(k)=max(abs(135*C(1:2,:)./max(C(3,:),1e-3)),[],'all');
  end
  fprintf('== %s Z0=%d (24x) ended t=%.1f success=%d\n',tr,z0,d.tRange(n),r.success);
  for lo=[6 4 3 2 1.5 1 0.5 0.2]
    s=alt<=lo & alt>lo/1.5; if ~any(s), continue; end
    fprintf('  alt~%4.1f m: min pt depth = %6.2f m (min over window), camera tilt max %.1f deg, max|pixel| %.0f\n',lo,min(md(s)),max(tilt(s)),max(pmax(s)));
  end
  fprintf('  steps with min depth<=0.05 m: %d of %d (min overall %.3f m at alt %.2f)\n',sum(md<=0.05),n,min(md),alt(find(md==min(md),1)));
 end
end
