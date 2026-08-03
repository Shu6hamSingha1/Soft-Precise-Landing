function cb_latfit()
% Empirically find the correct SCALE-FREE lateral form: V_h(1:2) = a*centroidRate +
% c*(P_g .* loom). Run cells noiseless (USE_MOMENT_FLOW=0, no effect on control), pool
% (Vh12raw, Pg.*Vh3mom, Vha12=truth) over the descent, least-squares fit a,c per axis.
% Consistent a,c across cells => scale-free. Also report corr of raw-only vs +coupling.
    cells={{"Static",[2;2;-3]},{"Lissajous",[2;-2;-5]},{"Circular",[2;2;-7]},...
           {"Sinusoidal",[2;2;-5]},{"Static",[2;-2;-5]}};
    RAW=[]; CPL=[]; DRO=[]; TRU=[];
    for c=cells
      cc=c{1}; [vr,pg,m3,va,dr]=run_one(cc{2},cc{1});
      RAW=[RAW; vr']; CPL=[CPL; (pg.*m3)']; DRO=[DRO; dr']; TRU=[TRU; va'];  %#ok<AGROW>
    end
    rawS=[RAW(:,1);RAW(:,2)]; cplS=[CPL(:,1);CPL(:,2)]; droS=[DRO(:,1);DRO(:,2)]; truS=[TRU(:,1);TRU(:,2)];
    A2=[rawS cplS]; co2=A2\truS; c2=corrl(A2*co2,truS);
    A3=[rawS cplS droS]; co3=A3\truS; c3=corrl(A3*co3,truS);
    fprintf('  raw+coupling      : a=%.3f c=%+.3f          corr=%.3f rms=%.4f\n', co2(1),co2(2), c2, sqrt(mean((truS-A2*co2).^2)));
    fprintf('  raw+coupling+derot: a=%.3f c=%+.3f d=%+.3f corr=%.3f rms=%.4f\n', co3(1),co3(2),co3(3), c3, sqrt(mean((truS-A3*co3).^2)));
    fprintf('  (corr jump from derot => image-based alpha-rate de-rotation helps the lateral)\n');
end
function [vr,pg,m3,va,dr]=run_one(ic,traj)
    global IC_OVERRIDE NOISE_OVERRIDE RNG_SEED_OVERRIDE TRAJ_OVERRIDE COMBINED_BARRIER C_SIMPLE CB_DROP_SDDOT USE_MOMENT_FLOW USE_MOMENT_LOOM
    IC_OVERRIDE=ic; NOISE_OVERRIDE=0; RNG_SEED_OVERRIDE=0; TRAJ_OVERRIDE=traj; COMBINED_BARRIER=1; C_SIMPLE=1; CB_DROP_SDDOT=1; USE_MOMENT_FLOW=0; USE_MOMENT_LOOM=0;
    visualControl_IBVS_adaptive; n=idx;
    R=find(Vha12_log(1,1:n)~=0 | Vha12_log(2,1:n)~=0); R=R(R>R(1)+4 & R<R(end)-2);
    vr=Vh12raw_log(:,R); pg=Pg_log(:,R); m3=repmat(Vh3mom_log(R),2,1); va=Vha12_log(:,R);
    % alpha-rate de-rotation regressor: centroid sweep from relative yaw = adot*[-Pg_y; Pg_x]
    al=Va4_log(1:n); adot=zeros(1,n);
    Rall=find(Va4_log(1:n)~=0 | (1:n)==1);
    for ii=2:numel(R)
      k=R(ii); kp=R(ii-1); adot_k=(al(k)-al(kp))/((k-kp)*dt*ZOH);
      dr(:,ii)=adot_k*[-pg(2,ii); pg(1,ii)]; %#ok<AGROW>
    end
    if isempty(R); dr=zeros(2,0); else; dr(:,1)=0; end
end
function c=corrl(a,b); a=a-mean(a); b=b-mean(b); d=sqrt(sum(a.^2)*sum(b.^2)); if d==0;c=0;else;c=sum(a.*b)/d;end; end
