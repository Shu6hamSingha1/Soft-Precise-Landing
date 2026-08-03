function cb_asym()
% Liss IC3 binds y-velocity, Circ IC4 binds x-velocity -> tune ASYMMETRICALLY:
% lower chi_r_y (Liss IC3, has y-precision room) + lower p_2inf_x (Circ IC4, precision-free).
% Base 25/25: chi_r=[0.65,0.65], p_2inf=[0.5,0.5]. cfg={chi_x,chi_y,p2_x,p2_y}.
    cfgs={ {0.65,0.65,0.5,0.5,'base-sym'}, {0.65,0.50,0.5,0.5,'chiY.5'}, ...
           {0.65,0.65,0.3,0.5,'p2X.3'},   {0.65,0.50,0.3,0.5,'chiY.5+p2X.3'} };
    cells={{[2;-2;-5],'Lissajous','L-IC3'},{[2;2;-7],'Circular','C-IC4'},...
           {[2;2;-7],'Lissajous','L-IC4*'},{[0;0;-5],'Circular','C-IC2*'}};
    for c=1:numel(cfgs)
      fprintf('== %s: chi=[%.2f,%.2f] p2inf=[%.1f,%.1f] ==\n',cfgs{c}{5},cfgs{c}{1},cfgs{c}{2},cfgs{c}{3},cfgs{c}{4});
      for k=1:numel(cells)
        d=run_one(cells{k}{1},cells{k}{2},cfgs{c});
        sp=d.land&&d.xy<=0.08&&d.vel<=0.20; br=''; if d.resid>=1; br='BREACH'; end
        fprintf('   %-7s vel=%.3f [vx%+.3f vy%+.3f] xy=%.3f resid=%.2f %s%s\n',cells{k}{3},d.vel,d.vx,d.vy,d.xy,d.resid,tern(sp),br);
      end
    end
end
function s=tern(c); if c; s='<<SP'; else; s=''; end; end
function d=run_one(ic,traj,cfg)
    global IC_OVERRIDE NOISE_OVERRIDE TRAJ_OVERRIDE COMBINED_BARRIER C_SIMPLE CB_DROP_SDDOT P2INF_XY_OVERRIDE CHI_R_OVERRIDE
    IC_OVERRIDE=ic; NOISE_OVERRIDE=0; TRAJ_OVERRIDE=traj; COMBINED_BARRIER=1; C_SIMPLE=1; CB_DROP_SDDOT=1;
    CHI_R_OVERRIDE=[cfg{1};cfg{2}]; P2INF_XY_OVERRIDE=[cfg{3};cfg{4}];
    d=struct('land',0,'vel',NaN,'vx',NaN,'vy',NaN,'xy',NaN,'resid',NaN);
    try
        visualControl_IBVS_adaptive;
        vrel=I_v_c-dx_t(1:3,idx); rb=r_bar_e(:,1:idx); prr=p_r(:,1:idx);
        d.land=double(landed); d.vel=norm(vrel); d.vx=vrel(1); d.vy=vrel(2); d.xy=norm(I_p_c(1:2)-x_t(1:2,idx)); d.resid=max(max(abs(rb)./prr));
    catch ME; fprintf('   ERR %s\n',ME.message); end
end
