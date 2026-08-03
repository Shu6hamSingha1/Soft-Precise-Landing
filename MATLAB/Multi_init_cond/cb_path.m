function cb_path()
% Understand the pathological seed-4 failures under MOMENT LOOM (lateral now diverges).
% Lateral V_h(1:2) is still pinv. Trace used V_h(1:2) vs analytical truth Vha12, the
% rel lateral vel, altGap, and featRad (FoV overflow) -> find what corrupts the lateral
% and when. Static IC5 s4 and Lissajous IC3 s4.
    cells={{"Static",[2;2;-3],'Sta-IC5'},{"Lissajous",[2;-2;-5],'Lis-IC3'}};
    for c=cells
      cc=c{1}; fprintf('=== %s seed4 (moment loom) ===\n',cc{3});
      r=run_one(cc{2},cc{1});
      n=r.n; gap=r.gap; vx=r.vx; vy=r.vy; hu=r.hu; ha=r.ha; fr=r.fr;
      b=find(hypot(vx,vy)>0.6,1);
      if isempty(b); fprintf('  lateral never exceeds 0.6\n'); b=max(2,n-50); else
        fprintf('  |v_xy|>0.6 FIRST at t=%.2f altGap=%.2f featRad=%.2f\n',(b-1)*0.01,gap(b),fr(b)); end
      fprintf('   t(s)|altGap|featRad| vx_rel | vy_rel | Vhx_use Vhx_tru | Vhy_use Vhy_tru\n');
      for k=max(2,b-30):6:min(n,b+90)
        fprintf('  %5.2f| %4.2f | %5.2f |%+6.2f |%+6.2f | %+6.2f %+6.2f | %+6.2f %+6.2f\n',...
                (k-1)*0.01,gap(k),fr(k),vx(k),vy(k),hu(1,k),ha(1,k),hu(2,k),ha(2,k));
      end
      fprintf('\n');
    end
end
function r=run_one(ic,traj)
    global IC_OVERRIDE NOISE_OVERRIDE RNG_SEED_OVERRIDE TRAJ_OVERRIDE COMBINED_BARRIER C_SIMPLE CB_DROP_SDDOT CB_SDDOT_TAU ...
           P2INF_XY_OVERRIDE P2INF_Z_OVERRIDE CHI_R_OVERRIDE PRINF_OVERRIDE KR_OVERRIDE KOMEGA_OVERRIDE USE_MOMENT_LOOM
    IC_OVERRIDE=ic; NOISE_OVERRIDE=1; RNG_SEED_OVERRIDE=4; TRAJ_OVERRIDE=traj; COMBINED_BARRIER=1; C_SIMPLE=1; CB_DROP_SDDOT=0; CB_SDDOT_TAU=0.7;
    P2INF_XY_OVERRIDE=[0.5;0.5]; P2INF_Z_OVERRIDE=1.0; CHI_R_OVERRIDE=[0.65;0.65]; PRINF_OVERRIDE=[1.0;1.0]; KR_OVERRIDE=[4;4;0.5]; KOMEGA_OVERRIDE=[0.6;0.6;0.1]; USE_MOMENT_LOOM=1;
    visualControl_IBVS_adaptive; n=idx;
    r=struct(); r.n=n; r.gap=abs(X_DS(3,1:n)-x_t(3,1:n)); r.vx=X_DS(8,1:n)-dx_t(1,1:n); r.vy=X_DS(9,1:n)-dx_t(2,1:n);
    % carry-forward V_h logs (only set on refresh) to every step
    hu=Vhused_log(:,1:n); ha=Vha12_log(:,1:n); fr=featrad_log(1:n);
    for k=2:n
      if all(hu(:,k)==0); hu(:,k)=hu(:,k-1); end
      if all(ha(:,k)==0); ha(:,k)=ha(:,k-1); end
      if fr(k)==0; fr(k)=fr(k-1); end
    end
    r.hu=hu; r.ha=ha; r.fr=fr;
end
