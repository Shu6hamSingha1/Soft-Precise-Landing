clc; clear; addpath('../Common');
global VDF_OVERRIDE MARKER_SCALE PX_NOISE_FIX PX_NOISE_PARAMS TARGET_YAW_RATE TARGET_TILT_OFF
PX_NOISE_FIX=true; PX_NOISE_PARAMS=[0.027 0.175 0.5 0 0]; MARKER_SCALE=1;
VDF_OVERRIDE=struct('theta_per_axis',true,'pinv_tol',4,'flow_reduced',true);
cfg=struct('NOISE',0,'GE',1,'delay',1);
V4={[],[];0,[]};
names={'baseline (yaw+tilt)','no target yaw','no deck tilt','neither'};
for k=1:2
  TARGET_YAW_RATE=V4{k,1}; TARGET_TILT_OFF=V4{k,2};
  r=run_simulation([2;2;-5;1;0;0;0;zeros(6,1)],"Circular",[],1.4,cfg,1); d=r.data; n=d.idx; if n<=0,n=numel(d.e_a_log);end
  t=d.tRange(1:n); s=t>=0.3&t<=8; V=d.V_X_DS(:,1:n); a=V(4:5,s); b=V(16:17,s);
  fprintf('%-22s slope=%.2f corr=%.2f (x %.2f, y %.2f) rms meas=%.3f true=%.3f\n',names{k},a(:)'*b(:)/(b(:)'*b(:)),local_c(a,b),a(1,:)*b(1,:)'/(b(1,:)*b(1,:)'),a(2,:)*b(2,:)'/(b(2,:)*b(2,:)'),rms(a(:)),rms(b(:)),rms(V(9,s)),rms(V(21,s)),rms(V(1:2,s),'all'));
end
function c=local_c(x,y)
 xc=x(:)-mean(x(:)); yc=y(:)-mean(y(:)); c=(xc'*yc)/sqrt((xc'*xc)*(yc'*yc));
end
