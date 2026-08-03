function cb_port_dbg()
% Compare early-iteration lateral signals canon vs rsim on Static IC3 [2,-2,-5] to
% locate the combined-barrier lateral port bug.
    ic=[2;-2;-5]; traj="Static";
    global IC_OVERRIDE NOISE_OVERRIDE TRAJ_OVERRIDE COMBINED_BARRIER C_SIMPLE
    IC_OVERRIDE=ic; NOISE_OVERRIDE=0; TRAJ_OVERRIDE=traj; COMBINED_BARRIER=1; C_SIMPLE=1;
    visualControl_IBVS_adaptive;
    CG2=zeros(2,1); CG2(1)=G_2(1,1,2); CG2(2)=G_2(2,2,2);
    Cz2=zeta_2(1:2,2); Cp2=p_2(1:2,2); Cka=kappa(1:2,2); CS2=[S_2(1,1,2);S_2(2,2,2)]; Cdp2=dp_2(1:2,2);
    Cia=I_a_cd(1:2,2);
    clearvars -except CG2 Cz2 Cp2 Cka CS2 Cdp2 Cia
    out=run_simulation([2;-2;-5;1;0;0;0;zeros(6,1)], "Static", [], 1.0, struct('NOISE',0), 1000);
    d=out.data;
    fprintf('idx2 intermediates (canon vs rsim):\n');
    fprintf('  G_2  C=[%.5f %.5f] R=[%.5f %.5f]\n', CG2(1),CG2(2), d.G_2(1,1,2),d.G_2(2,2,2));
    fprintf('  p_2  C=[%.4f %.4f] R=[%.4f %.4f]\n', Cp2(1),Cp2(2), d.p_2(1,2),d.p_2(2,2));
    fprintf('  zeta2 C=[%+.5f %+.5f] R=[%+.5f %+.5f]\n', Cz2(1),Cz2(2), d.zeta_2(1,2),d.zeta_2(2,2));
    fprintf('  S_2  C=[%+.5f %+.5f] R=[%+.5f %+.5f]\n', CS2(1),CS2(2), d.S_2(1,1,2),d.S_2(2,2,2));
    fprintf('  dp_2 C=[%+.5f %+.5f] R=[%+.5f %+.5f]\n', Cdp2(1),Cdp2(2), d.dp_2(1,2),d.dp_2(2,2));
    fprintf('  kappa C=[%.5f %.5f] R=[%.5f %.5f]\n', Cka(1),Cka(2), d.kappa(1,2),d.kappa(2,2));
    fprintf('  I_a_cd C=[%+.4f %+.4f] R=[%+.4f %+.4f]\n', Cia(1),Cia(2), d.I_a_cd(1,2),d.I_a_cd(2,2));
end
