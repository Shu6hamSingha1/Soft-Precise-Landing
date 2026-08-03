function cb_asym_diag()
% WHY does asymmetric kR_y=2.0 (which fixes Liss IC3) break other cells?
% Part 1: run the suspect cells at kR=[3;2;0.5], report PER-AXIS rel-vel (vx,vy)
%   to confirm the collateral is a Y-axis cycle (low kR_y under-damps pitch/Y).
% Part 2: try a Y-axis compensator that does NOT raise kR_y back: kOmega_y up
%   (Y attitude-RATE damping), and chi_r_y / p2inf_y. Keep kR=[3;2;0.5].
    cells={{"Static",[2;-2;-5],'Sta-IC3'},{"Static",[2;2;-3],'Sta-IC5'},...
           {"Linear",[2;-2;-5],'Lin-IC3'},{"Lissajous",[2;-2;-5],'Lis-IC3'},...
           {"Circular",[2;2;-7],'Cir-IC4'},{"Circular",[2;2;-3],'Cir-IC5'}};
    fprintf('=== PART 1: kR=[3;2;0.5], per-axis terminal rel-vel ===\n');
    fprintf('  cell    |  xy   |  |v|  |  vx    |  vy    | dominant\n');
    for c=cells
      cc=c{1}; d=run_one(cc{2},cc{1},[3;2;0.5],0.6,[0.85;0.85],[0.5;0.5]);
      dom='--'; if abs(d.vy)>abs(d.vx)+0.03; dom='Y'; elseif abs(d.vx)>abs(d.vy)+0.03; dom='X'; end
      fprintf('  %-7s | %.3f | %.3f | %+.3f | %+.3f |  %s\n',cc{3},d.xy,d.vel,d.vx,d.vy,dom);
    end
    fprintf('\n=== PART 2: Y-compensators at kR=[3;2;0.5] (keep low kR_y) ===\n');
    comps={{'kOmega_y=0.3',0.3,[0.85;0.85],[0.5;0.5]},...
           {'kOmega_y=0.6',0.6,[0.85;0.85],[0.5;0.5]},...
           {'chi_r_y=1.1 ',0.1,[0.85;1.1],[0.5;0.5]},...
           {'p2inf_y=0.35',0.1,[0.85;0.85],[0.5;0.35]}};
    for cp=comps
      pp=cp{1}; ky=pp{2}; chi=pp{3}; p2=pp{4};
      fprintf('  [%s]\n',pp{1});
      for c=cells
        cc=c{1}; d=run_one(cc{2},cc{1},[3;2;0.5],ky,chi,p2);
        fprintf('    %-7s xy=%.3f |v|=%.3f (vx%+.3f vy%+.3f)\n',cc{3},d.xy,d.vel,d.vx,d.vy);
      end
    end
end
function d=run_one(ic,traj,kr,kOy,chi,p2)
    global IC_OVERRIDE NOISE_OVERRIDE TRAJ_OVERRIDE COMBINED_BARRIER C_SIMPLE CB_DROP_SDDOT CB_SDDOT_TAU P2INF_XY_OVERRIDE CHI_R_OVERRIDE PRINF_OVERRIDE KR_OVERRIDE KOMEGA_OVERRIDE
    IC_OVERRIDE=ic; NOISE_OVERRIDE=0; TRAJ_OVERRIDE=traj; COMBINED_BARRIER=1; C_SIMPLE=1; CB_DROP_SDDOT=0; CB_SDDOT_TAU=0.7; P2INF_XY_OVERRIDE=p2; CHI_R_OVERRIDE=chi; PRINF_OVERRIDE=[1.0;1.0]; KR_OVERRIDE=kr; KOMEGA_OVERRIDE=[0.6;kOy;0.1];
    try
        visualControl_IBVS_adaptive;
        vrel=I_v_c-dx_t(1:3,idx);
        d=struct('land',double(landed),'xy',norm(I_p_c(1:2)-x_t(1:2,idx)),...
                 'vel',norm(vrel),'vx',vrel(1),'vy',vrel(2));
    catch ME
        d=struct('land',0,'xy',NaN,'vel',9.99,'vx',NaN,'vy',NaN);
    end
end
