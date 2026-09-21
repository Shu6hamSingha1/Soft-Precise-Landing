clc; clear; addpath('../Common');
global VDF_OVERRIDE MARKER_SCALE PX_NOISE_FIX PX_NOISE_PARAMS
PX_NOISE_FIX=true; PX_NOISE_PARAMS=[0.027 0.175 0.5 0 0]; MARKER_SCALE=2;
VDF_OVERRIDE=struct('theta_per_axis',true,'pinv_tol',4,'flow_reduced',true);
cfg=struct('NOISE',1,'GE',1,'delay',1);
for tr=["Circular" "Linear"]
 for sd=[1 3]
  r=run_simulation([2;2;-5;1;0;0;0;zeros(6,1)],tr,[],1.4,cfg,sd); d=r.data; n=d.idx; if n<=0,n=numel(d.e_a_log);end
  t=d.tRange(1:n); s=t>=0.3&t<=8; V=d.V_X_DS(:,1:n);
  hm=V(4:5,s); ha=V(16:17,s); sa=V(13:14,s); wz=V(21,s);
  rot=[-wz.*sa(2,:); wz.*sa(1,:)];
  y=hm(:); X1=ha(:); X2=rot(:);
  c1=X1\y; c2=[X1 X2]\y;
  fprintf('%-8s seed %d: slope on h_a alone=%.2f | joint fit h_meas = %.2f*h_a + %.2f*(w_z x s)  R2 alone=%.2f joint=%.2f | rms|h_a|=%.3f rms(w_z*s)=%.3f\n',tr,sd,c1,c2(1),c2(2),...
    1-sum((y-X1*c1).^2)/sum(y.^2),1-sum((y-[X1 X2]*c2).^2)/sum(y.^2),rms(X1),rms(X2));
 end
end
