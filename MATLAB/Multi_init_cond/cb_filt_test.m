function cb_filt_test()
% Filtered-s_ddot sweep: does LPF(s_ddot) keep the target-accel FF (cut lateral lag
% on moving Pr cells) WITHOUT reintroducing the terminal spike/divergence (Static IC5)?
% Compare tau in {0(full), 0.05, 0.1, 0.2, 0.5} + drop, on the 4 problem cells.
    cases={{[2;2;-3],'Static'},{[2;2;-3],'Circular'},{[2;-2;-5],'Sinusoidal'},{[2;-2;-5],'Lissajous'}};
    cn={'Static IC5','Circ IC5','Sin IC3','Liss IC3'};
    modes={{'full',0,0},{'tau.05',0,0.05},{'tau.1',0,0.1},{'tau.2',0,0.2},{'tau.5',0,0.5},{'drop',1,0}};
    for ci=1:numel(cases)
      fprintf('-- %s --\n',cn{ci});
      for mi=1:numel(modes)
        r=run_one(cases{ci}{1},cases{ci}{2},modes{mi}{2},modes{mi}{3});
        fprintf('   %-7s -> land=%d fov=%d xy=%.3f vel=%.3f lat=%.3f | resid=%.2f\n',...
            modes{mi}{1},r.land,r.fov,r.xy,r.vel,r.vlat,r.resid);
      end
    end
end
function r=run_one(ic,traj,drop,tau)
    global IC_OVERRIDE NOISE_OVERRIDE TRAJ_OVERRIDE COMBINED_BARRIER C_SIMPLE CB_DROP_SDDOT CB_SDDOT_TAU
    IC_OVERRIDE=ic; NOISE_OVERRIDE=0; TRAJ_OVERRIDE=traj; COMBINED_BARRIER=1; C_SIMPLE=1; CB_DROP_SDDOT=drop; CB_SDDOT_TAU=tau;
    xy=NaN;vel=NaN;ld=0;fov=0;vlat=NaN;resid=NaN;
    try
        visualControl_IBVS_adaptive;
        vrel=I_v_c-dx_t(1:3,idx); xy=norm(I_p_c(1:2)-x_t(1:2,idx)); vel=norm(vrel); vlat=norm(vrel(1:2));
        ld=double(landed); fov=double(fov_fail);
        rb=r_bar_e(:,1:idx); prr=p_r(:,1:idx); resid=max(max(abs(rb)./prr));
    catch ME; fprintf('   ERR %s\n',ME.message); end
    r=struct('land',ld,'fov',fov,'xy',xy,'vel',vel,'vlat',vlat,'resid',resid);
end
