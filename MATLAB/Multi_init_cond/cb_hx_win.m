clc; clear; addpath('../Common');
global VDF_OVERRIDE MARKER_SCALE PX_NOISE_FIX PX_NOISE_PARAMS
PX_NOISE_FIX=true; PX_NOISE_PARAMS=[0.027 0.175 0.5 0 0]; MARKER_SCALE=1;
VDF_OVERRIDE=struct('theta_per_axis',true,'pinv_tol',4,'flow_reduced',true);
cfg=struct('NOISE',1,'GE',1,'delay',1);
r=run_simulation([2;2;-5;1;0;0;0;zeros(6,1)],"Circular",[],1.4,cfg,1); d=r.data; n=d.idx; if n<=0,n=numel(d.e_a_log);end
t=d.tRange(1:n); V=d.V_X_DS(:,1:n); alt=-(d.X_DS(3,1:n)-d.x_t(3,1:n));
W=[0.3 2;2 4;4 6;6 8;8 12;12 20];
for i=1:size(W,1)
  ss=t>=W(i,1)&t<W(i,2); if sum(ss)<10, continue; end
  a=V(4:5,ss); b=V(16:17,ss);
  fprintf('  t %4.1f-%4.1f alt~%.1f: slope=%.2f corr=%.2f rms meas=%.3f true=%.3f | slope x=%.2f y=%.2f\n',W(i,1),W(i,2),mean(alt(ss)),a(:)'*b(:)/(b(:)'*b(:)),local_c(a(:),b(:)),rms(a(:)),rms(b(:)),a(1,:)*b(1,:)'/(b(1,:)*b(1,:)'),a(2,:)*b(2,:)'/(b(2,:)*b(2,:)'));
end
function c=local_c(x,y)
 xc=x(:)-mean(x(:)); yc=y(:)-mean(y(:)); c=(xc'*yc)/sqrt((xc'*xc)*(yc'*yc));
end
