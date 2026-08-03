function cb_pinvtol()
% Tune the pinv(L_s, TOL) singular-value tolerance (default 4). Hypothesis: tol=4
% truncates/attenuates the weak loom (vz) singular mode -> h_z under-report + sign-flip
% -> Static IC5 seed4 descent runaway. Smaller tol should retain loom -> fix it.
% Report, on Static IC5 seed4 (DROP): loom error (mean|pinv h_z - analyt h_z| at refresh),
% sign-flips, peak|vz|, term vel, land. Then full 5x5 noiseless + noisy for promising tol.
    tols=[4 1 0.5 0.1 0.01 1e-6];
    fprintf('=== Static IC5 seed4 (DROP): pinv tol sweep ===\n');
    fprintf('  tol     | loomErr | nFlip | peak|vz| | term|v| | land\n');
    for t=tols
      r=run_s5(t);
      fprintf('  %-7g | %6.3f  |  %3d  |  %6.3f  | %6.3f  |  %d\n',t,r.le,r.nf,r.pk,r.tv,r.land);
    end
    fprintf('\n=== Full 5x5 SP at tol=0.01 vs base tol=4 (keep s_ddot 92-config) ===\n');
    for t=[4 0.01]
      [nl,ns]=suite(t);
      fprintf('  tol=%-5g : noiseless SP=%d/25 | noisy(4sd) SP=%d/100\n',t,nl,ns);
    end
end
function r=run_s5(tol)
    global IC_OVERRIDE NOISE_OVERRIDE RNG_SEED_OVERRIDE TRAJ_OVERRIDE COMBINED_BARRIER C_SIMPLE CB_DROP_SDDOT PINV_TOL
    IC_OVERRIDE=[2;2;-3]; NOISE_OVERRIDE=1; RNG_SEED_OVERRIDE=4; TRAJ_OVERRIDE="Static"; COMBINED_BARRIER=1; C_SIMPLE=1; CB_DROP_SDDOT=1; PINV_TOL=tol;
    try
        visualControl_IBVS_adaptive; n=idx; R=find(condLs_log(1:n)>0);
        hi=Vhi3_log(R); ha=Vha3_log(R); vz=X_DS(10,1:n)-dx_t(3,1:n); vrel=I_v_c-dx_t(1:3,idx);
        r=struct('le',mean(abs(hi-ha)),'nf',sum(sign(hi)~=sign(ha)&abs(ha)>0.05),'pk',max(abs(vz)),'tv',norm(vrel),'land',double(landed));
    catch ME; r=struct('le',NaN,'nf',-1,'pk',99,'tv',99,'land',0); end
end
function [nl,ns]=suite(tol)
    trajs={"Static","Linear","Sinusoidal","Lissajous","Circular"};
    ICs={[0;0;-5],[2;2;-5],[2;-2;-5],[2;2;-7],[2;2;-3]};
    nl=0; for ti=1:5; for ii=1:5; d=one(ICs{ii},trajs{ti},0,0,tol); nl=nl+d; end; end
    ns=0; for sd=1:4; for ti=1:5; for ii=1:5; d=one(ICs{ii},trajs{ti},1,sd,tol); ns=ns+d; end; end; end
end
function sp=one(ic,traj,noise,seed,tol)
    global IC_OVERRIDE NOISE_OVERRIDE RNG_SEED_OVERRIDE TRAJ_OVERRIDE COMBINED_BARRIER C_SIMPLE CB_DROP_SDDOT CB_SDDOT_TAU ...
           P2INF_XY_OVERRIDE P2INF_Z_OVERRIDE CHI_R_OVERRIDE PRINF_OVERRIDE KR_OVERRIDE KOMEGA_OVERRIDE PINV_TOL
    IC_OVERRIDE=ic; NOISE_OVERRIDE=noise; RNG_SEED_OVERRIDE=seed; TRAJ_OVERRIDE=traj; COMBINED_BARRIER=1; C_SIMPLE=1; CB_DROP_SDDOT=0; CB_SDDOT_TAU=0.7;
    P2INF_XY_OVERRIDE=[0.5;0.5]; P2INF_Z_OVERRIDE=1.0; CHI_R_OVERRIDE=[0.65;0.65]; PRINF_OVERRIDE=[1.0;1.0]; KR_OVERRIDE=[4;4;0.5]; KOMEGA_OVERRIDE=[0.6;0.6;0.1]; PINV_TOL=tol;
    try
        visualControl_IBVS_adaptive; vrel=I_v_c-dx_t(1:3,idx);
        sp=double(landed && norm(I_p_c(1:2)-x_t(1:2,idx))<=0.08 && norm(vrel)<=0.20);
    catch ME; sp=0; end
end
