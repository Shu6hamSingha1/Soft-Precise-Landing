clc; clear; addpath('../Common');
global VDF_OVERRIDE MARKER_SCALE PX_NOISE_FIX PX_NOISE_PARAMS
MARKER_SCALE=1; VDF_OVERRIDE=struct('theta_per_axis',true,'pinv_tol',4,'flow_reduced',true);
cfg=struct('NOISE',0,'GE',1,'delay',1);
r=run_simulation([2;2;-5;1;0;0;0;zeros(6,1)],"Circular",[],1.4,cfg,1); d=r.data; n=d.idx; if n<=0,n=numel(d.e_a_log);end
disp(fieldnames(d)')
t=d.tRange(1:n); s=t>=0.3&t<=8; V=d.V_X_DS(:,1:n);
wt=d.dx_t(6,1:n); sa=V(13:14,:);
e=(V(4:5,s)-V(16:17,s)); 
for sg=[1 -1]
  reg=sg*[-sa(2,s); sa(1,s)].*wt(s);
  c=reg(:)\e(:); fprintf('sign %+d: fit e = %.3f * (w_t x s); R2=%.3f\n',sg,c,1-sum((e(:)-c*reg(:)).^2)/sum(e(:).^2));
end
