function cb_tol_cycle()
% WHY low pinv tol fails the rest: retaining the weak loom singular mode UN-REGULARIZES
% the velocity recovery -> tiny SV divides the finite-diff/quantization error in dPdt
% -> amplified oscillatory loom h_z -> descent LIMIT CYCLE. Trace a NOISELESS cell that
% is clean at tol=4 but breaks at tol=0.01. Dump vz + pinv h_z vs analytic h_z; report
% terminal vz oscillation (pp + freq) at tol=4 vs tol=0.01.
    cells={{"Sinusoidal",[2;-2;-5],'Sin-IC3'},{"Lissajous",[2;2;-7],'Lis-IC4'},{"Static",[2;2;-7],'Sta-IC4'}};
    for c=cells
      cc=c{1};
      fprintf('=== %s (noiseless) ===\n',cc{3});
      for tol=[4 0.01]
        r=run_one(cc{2},cc{1},tol);
        fprintf('  tol=%-5g: term vz=%+.3f  vz pp(last3s)=%.3f  vz freq=%.2fHz  loomErr=%.3f  land=%d SP=%d\n',...
                tol,r.tv,r.pp,r.fr,r.le,r.land,r.sp);
      end
    end
end
function r=run_one(ic,traj,tol)
    global IC_OVERRIDE NOISE_OVERRIDE RNG_SEED_OVERRIDE TRAJ_OVERRIDE COMBINED_BARRIER C_SIMPLE CB_DROP_SDDOT CB_SDDOT_TAU ...
           P2INF_XY_OVERRIDE P2INF_Z_OVERRIDE CHI_R_OVERRIDE PRINF_OVERRIDE KR_OVERRIDE KOMEGA_OVERRIDE PINV_TOL
    IC_OVERRIDE=ic; NOISE_OVERRIDE=0; RNG_SEED_OVERRIDE=0; TRAJ_OVERRIDE=traj; COMBINED_BARRIER=1; C_SIMPLE=1; CB_DROP_SDDOT=0; CB_SDDOT_TAU=0.7;
    P2INF_XY_OVERRIDE=[0.5;0.5]; P2INF_Z_OVERRIDE=1.0; CHI_R_OVERRIDE=[0.65;0.65]; PRINF_OVERRIDE=[1.0;1.0]; KR_OVERRIDE=[4;4;0.5]; KOMEGA_OVERRIDE=[0.6;0.6;0.1]; PINV_TOL=tol;
    try
        visualControl_IBVS_adaptive; n=idx; w=min(300,n-5); k=(n-w):n;
        vz=X_DS(10,1:n)-dx_t(3,1:n); seg=vz(k)-mean(vz(k)); zc=sum(seg(1:end-1).*seg(2:end)<0);
        R=find(condLs_log(1:n)>0); le=mean(abs(Vhi3_log(R)-Vha3_log(R)));
        vrel=I_v_c-dx_t(1:3,idx);
        r=struct('tv',vrel(3),'pp',max(vz(k))-min(vz(k)),'fr',zc/(2*w*0.01),'le',le,...
                 'land',double(landed),'sp',double(landed&&norm(I_p_c(1:2)-x_t(1:2,idx))<=0.08&&norm(vrel)<=0.20));
    catch ME; r=struct('tv',99,'pp',99,'fr',0,'le',99,'land',0,'sp',0); end
end
